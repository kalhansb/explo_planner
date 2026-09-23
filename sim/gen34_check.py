#!/usr/bin/env python3
"""Hard-fail checker for gen-34 cells (DESIGN_gen34.md §8.8, Q4, Q61, §9.4).

Reads each cell's <robot>.events.jsonl (schema 13 only), run_manifest.txt and
the emulator's link_states.csv. The events are the robots' own claims; every
claim of contact is checked against the link trace, which no robot writes
(Q61(b)).

Per cell:
  bounds     the stamped bounds equal this checker's own copies (§9.4: the
             detector holds its own copy of every bound and checks the run
             against it, never reads it from the run)
  contact    every exchange done, booking pair_met, chase contact and
             all-connected tick has a live link in link_states.csv inside the
             presence window before it (two windows for a pair the claiming
             robot only hears about)
  waits      no tick past its activity's wait bound; no homing gave_up or
             no-home
  flipflop   at most 6 activity changes in any 60 s, and no more than two short
             stints (A -> B -> A with B under 3 s) in a row (the harness's P4)
  proximity  no pair closer than 1.0 m (link_states distance)
  progress   no robot still for 120 s while it should be moving (Q4, K20)
  horizon    a censored run whose robots have all finished exploring has every
             robot home (Q61(a)); censored while someone explores is not a fail
  giveups    exchange give-ups at most a quarter of the non-trivial exchanges
Per arm, over its cells:
  firing     the arm's required mechanisms fired at least once (§9.4). The
             optional ones are counted and printed, never failed: they fire
             only in some geometries.

Usage:  gen34_check.py ROOT_OR_CELL [ROOT_OR_CELL ...] [--json OUT]
        A directory holding run_manifest.txt is a cell; any other directory is
        searched one level down for cells.
Exit:   0 clean
        1 a hard fail (even when other cells were refused)
        2 usage, or a cell refused (not a gen-34 cell, or a newer schema)
        3 nothing failed, but something could not be checked
"""

import argparse
import bisect
import csv
import json
import math
import os
import sys

SCHEMA = 13

# The checker's own copies of the bounds (§8.7). A run stamped with other
# values is a different experiment, and the checks below are sized on these.
BOUNDS = {
    "presence_window_sec": 10.0,
    "beacon_hz": 1.0,
    "exchange_stall_sec": 120.0,
    "exchange_total_sec": 600.0,
    "chase_limit_sec": 600.0,
    "meeting_interval_sec": 300.0,
    "meeting_patience_sec": 120.0,
    "meeting_backstop_sec": 600.0,
    "mission_return_max_sec": 600.0,
    "tick_event_period_sec": 2.0,
}

# Beacon receipt is stamped at the node's next drain, the link trace at 5 Hz.
LINK_SLACK_SEC = 1.5
# A wait ends on the first node tick at or past its bound.
WAIT_SLACK_SEC = 1.0
FLIP_WINDOW_SEC = 60.0
FLIP_MAX = 6
STINT_SEC = 3.0
STINT_RUN_MAX = 2
PROX_MIN_M = 1.0
STILL_WINDOW_SEC = 120.0
STILL_MAX_M = 0.5
GIVEUP_FRAC = 0.25

# Required per arm: a zero over the arm's cells is a hard fail. Optional: shown.
# exchange_nontrivial is derived here (exchange_done - exchange_done_trivial);
# every other name is a run_end.mechanism_counts key.
_FIRE_ALL = ["exchange_nontrivial", "homing_arrived"]
_FIRE_CHASE = ["chase_start", "chase_first_intercept", "activity_follow",
               "chase_done"]
_FIRE_MEET = ["plan_firmed", "booking_departed", "booking_full_met"]
FIRING_REQUIRED = {
    "off":        _FIRE_ALL,
    "pursuit":    _FIRE_ALL + _FIRE_CHASE,
    "rendezvous": _FIRE_ALL + _FIRE_MEET,
    "hybrid":     _FIRE_ALL + _FIRE_MEET + _FIRE_CHASE,
}
FIRING_OPTIONAL = ["chase_first_goal", "chase_first_last-position",
                   "reconnect_moves", "leg_escapes", "booking_missed",
                   "booking_patience", "booking_backstop", "booking_partial",
                   "plan_renewed", "chase_gate_declined", "exchange_gave_up",
                   "exchange_lost", "homing_gave_up", "team_events_unknown"]


class Refused(Exception):
    pass


# --- reading ------------------------------------------------------------------

def read_manifest(cell):
    out = {}
    path = os.path.join(cell, "run_manifest.txt")
    with open(path) as fh:
        for line in fh:
            line = line.rstrip("\n")
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            # Keys the teardown appends are last-wins; the rest first-wins.
            if k in ("run_end_reason", "run_gates_verdict", "run_end_t_sim"):
                out[k] = v
            else:
                out.setdefault(k, v)
    return out


def read_events(path):
    """(events, problems). A torn last line is a problem, not a crash."""
    evs, problems = [], []
    with open(path) as fh:
        lines = fh.readlines()
    for n, line in enumerate(lines, 1):
        line = line.strip()
        if not line:
            continue
        try:
            evs.append(json.loads(line))
        except ValueError:
            problems.append(f"line {n} is not JSON"
                            + (" (torn last line)" if n == len(lines) else ""))
    return evs, problems


def read_links(cell):
    """{(a, b): (times, up, dist)} with a < b, times sorted; None if absent."""
    path = os.path.join(cell, "link_states.csv")
    if not os.path.exists(path):
        return None
    rows = {}
    with open(path) as fh:
        for r in csv.DictReader(fh):
            try:
                t = float(r["t_sim"])
                i, j = int(float(r["i"])), int(float(r["j"]))
                up = float(r["connected"]) > 0.5
                d = float(r.get("distance_m") or "nan")
                pl = float(r.get("path_loss_db") or "0")
            except (KeyError, TypeError, ValueError):
                continue
            if pl <= 0.0:       # a row the emulator did not compute
                continue
            a, b = (i, j) if i < j else (j, i)
            rows.setdefault((a, b), []).append((t, up, d))
    out = {}
    for pair, rs in rows.items():
        rs.sort()
        out[pair] = ([x[0] for x in rs], [x[1] for x in rs], [x[2] for x in rs])
    return out


def link_up_between(links, a, b, lo, hi):
    """True: a live sample in [lo, hi]. False: samples, none live. None: none."""
    if a == b:
        return True
    key = (a, b) if a < b else (b, a)
    if key not in links:
        return None
    ts, ups, _ = links[key]
    k0 = bisect.bisect_left(ts, lo)
    k1 = bisect.bisect_right(ts, hi)
    if k0 >= k1:
        return None
    return any(ups[k0:k1])


def num(v, default=None):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


# --- one cell -------------------------------------------------------------------

class Cell:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path.rstrip("/"))
        self.fails = []        # (check, message)
        self.unresolved = []   # (check, message)
        self.notes = []
        self.counts = {}       # mechanism -> summed over robots, or None
        self.arm = None

    def fail(self, check, msg):
        self.fails.append((check, msg))

    def unres(self, check, msg):
        self.unresolved.append((check, msg))


def check_cell(path):
    c = Cell(path)
    try:
        man = read_manifest(path)
    except OSError as e:
        raise Refused(f"{c.name}: no readable run_manifest.txt ({e})")
    node = man.get("node")
    if node != "gen34":
        raise Refused(f"{c.name}: manifest node={node or '<absent>'}; this "
                      f"checker reads gen-34 cells only")
    c.arm = man.get("arm")
    if c.arm not in FIRING_REQUIRED:
        raise Refused(f"{c.name}: manifest arm={c.arm!r} is not a gen-34 arm")
    robots = [r for r in man.get("robots", "").split(",") if r]
    if len(robots) < 2:
        raise Refused(f"{c.name}: manifest names {len(robots)} robot(s)")
    rid = {name: k for k, name in enumerate(robots)}
    comms = man.get("comms") == "1"
    end_reason = man.get("run_end_reason")

    # Events per robot.
    events = {}
    for r in robots:
        p = os.path.join(path, f"{r}.events.jsonl")
        if not os.path.exists(p):
            c.unres("events", f"{r}: no {r}.events.jsonl")
            continue
        evs, problems = read_events(p)
        for pr in problems:
            c.unres("events", f"{r}: {pr}")
        start = next((e for e in evs if e.get("event") == "run_start"), None)
        if start is None:
            c.unres("events", f"{r}: no run_start")
            continue
        ver = start.get("schema_version")
        if ver != SCHEMA:
            raise Refused(f"{c.name}/{r}: schema_version {ver!r}; this checker "
                          f"reads exactly {SCHEMA}")
        params = start.get("params") or {}
        if params.get("arm") != c.arm:
            c.fail("bounds", f"{r}: run_start arm={params.get('arm')!r} but "
                             f"the manifest says {c.arm!r}")
        for k, want in BOUNDS.items():
            have = num(params.get(k))
            if have is None:
                c.unres("bounds", f"{r}: {k} not stamped in run_start")
            elif abs(have - want) > 1e-6:
                c.fail("bounds", f"{r}: {k}={have:g}, the checker's bound is "
                                 f"{want:g}")
        events[r] = evs
    if not events:
        c.unres("events", "no robot's events could be read")
        return c

    W = BOUNDS["presence_window_sec"]
    links = read_links(path) if comms else None
    if comms and links is None:
        c.unres("contact", "COMMS=1 but no link_states.csv")
    if not comms:
        c.notes.append("COMMS=0: every link is up by construction, so contact "
                       "is not checked; with no link trace, neither is "
                       "proximity")

    def verify_contact(check, r, t, pairs_self, pairs_other, what):
        if links is None:
            return
        for (a, b), span in ([(p, W) for p in pairs_self] +
                             [(p, 2 * W) for p in pairs_other]):
            ok = link_up_between(links, a, b, t - span - LINK_SLACK_SEC,
                                 t + LINK_SLACK_SEC)
            if ok is False:
                c.fail(check, f"{r} t={t:.1f}: {what}, but link "
                              f"{robots[a]}-{robots[b]} was down for the "
                              f"{span:.0f} s before it")
                return
            if ok is None:
                c.unres(check, f"{r} t={t:.1f}: {what}; no valid link sample "
                               f"for {robots[a]}-{robots[b]} in the window")
                return

    for r, evs in events.items():
        me = rid[r]
        ticks = [e for e in evs if e.get("event") == "tick"]
        # The first team_activity (from "") is the starting activity, not a
        # change; the harness's P4 skips it too.
        acts = [e for e in evs if e.get("event") == "team_activity" and
                e.get("from")]

        # --- contact claims -------------------------------------------------
        for e in evs:
            ev, t = e.get("event"), num(e.get("t_sim_sec"))
            if t is None:
                continue
            act = e.get("action")
            if (ev, act) in (("exchange", "done"), ("booking", "pair_met"),
                             ("chase", "contact")):
                j = e.get("peer")
                if not isinstance(j, int) or not 0 <= j < len(robots):
                    c.fail("contact", f"{r} t={t:.1f}: {ev} {act} names peer "
                                      f"{j!r}, not a robot of this run")
                    continue
                verify_contact("contact", r, t, [(me, j)], [],
                               f"{ev} {act} with {robots[j]}")
        for e in ticks:
            if e.get("all_connected") is True:
                t = num(e.get("t_sim_sec"))
                if t is None:
                    continue
                own = [(me, j) for j in range(len(robots)) if j != me]
                other = [(a, b) for a in range(len(robots))
                         for b in range(a + 1, len(robots))
                         if a != me and b != me]
                verify_contact("contact", r, t, own, other, "all_connected")

        # --- waits ------------------------------------------------------------
        for e in ticks:
            t, bound = num(e.get("t_sim_sec")), num(e.get("wait_bound_sec"))
            if t is not None and bound is not None and t > bound + WAIT_SLACK_SEC:
                c.fail("waits", f"{r} t={t:.1f}: {e.get('activity')} is "
                                f"{t - bound:.1f} s past its bound {bound:.1f}")
                break
        for e in evs:
            if e.get("event") == "homing" and e.get("action") in ("gave_up",
                                                                  "no-home"):
                c.fail("waits", f"{r} t={num(e.get('t_sim_sec'), -1):.1f}: "
                                f"homing {e.get('action')}")

        # --- flip-flop --------------------------------------------------------
        times = [num(e.get("t_sim_sec")) for e in acts]
        tos = [e.get("to") for e in acts]
        lo = 0
        for k in range(len(times)):
            if times[k] is None:
                continue
            while times[lo] is None or times[k] - times[lo] >= FLIP_WINDOW_SEC:
                lo += 1
            if k - lo + 1 > FLIP_MAX:
                c.fail("flipflop", f"{r}: {k - lo + 1} activity changes in "
                                   f"{FLIP_WINDOW_SEC:.0f} s ending "
                                   f"t={times[k]:.1f}")
                break
        run = 0
        for k in range(1, len(acts)):
            e = acts[k]
            dwell = num(e.get("dwell_sec"))
            reversal = (e.get("to") == acts[k - 1].get("from") and
                        dwell is not None and 0.0 <= dwell < STINT_SEC)
            run = run + 1 if reversal else 0
            if run > STINT_RUN_MAX:
                c.fail("flipflop", f"{r} t={times[k]:.1f}: {run} short stints "
                                   f"in a row ({acts[k - 1].get('to')} under "
                                   f"{STINT_SEC:.0f} s, back to {tos[k]})")
                break

        # --- progress ---------------------------------------------------------
        def should_move(e):
            if e.get("proximity_hold") is True:
                return False
            # A tick that lands in INTEGRATE or LOG_STEP still counts: a robot
            # cycling plan, fail, plan must not hide behind them.
            if e.get("state") in ("WAIT_FOR_MAP", "PROXIMITY_HOLD", "DONE",
                                  "WAIT"):
                return False
            if e.get("activity") == "explore":
                return True
            return (e.get("drive") == "leg" and
                    e.get("leg_status") in ("driving", "escaping"))
        seg = []
        for e in ticks:
            t, x, y = (num(e.get("t_sim_sec")), num(e.get("x")),
                       num(e.get("y")))
            if t is None or x is None or y is None or not should_move(e):
                seg = []
                continue
            seg.append((t, x, y))
            while seg and t - seg[0][0] > STILL_WINDOW_SEC:
                seg.pop(0)
            if seg and t - seg[0][0] >= STILL_WINDOW_SEC - 1e-6:
                x0, y0 = seg[0][1], seg[0][2]
                spread = max(math.hypot(px - x0, py - y0) for _, px, py in seg)
                if spread < STILL_MAX_M:
                    c.fail("progress", f"{r}: moved {spread:.2f} m in the "
                                       f"{STILL_WINDOW_SEC:.0f} s to "
                                       f"t={t:.1f} while it should be moving "
                                       f"({e.get('activity')}, "
                                       f"{e.get('state')})")
                    break

        # --- run_end counts ---------------------------------------------------
        end = next((e for e in reversed(evs) if e.get("event") == "run_end"),
                   None)
        mc = end.get("mechanism_counts") if end else None
        if not isinstance(mc, dict):
            c.unres("firing", f"{r}: no run_end.mechanism_counts (the node did "
                              f"not shut down cleanly)")
            c.counts = None
        elif c.counts is not None:
            for k, v in mc.items():
                if isinstance(v, (int, float)):
                    c.counts[k] = c.counts.get(k, 0) + int(v)

    # --- proximity (ground truth) -----------------------------------------------
    if links:
        worst = None
        for (a, b), (ts, _, ds) in links.items():
            for t, d in zip(ts, ds):
                if d == d and (worst is None or d < worst[0]):
                    worst = (d, t, a, b)
        if worst and worst[0] < PROX_MIN_M:
            d, t, a, b = worst
            c.fail("proximity", f"{robots[a]}-{robots[b]} {d:.2f} m apart at "
                                f"t={t:.1f} (floor {PROX_MIN_M:.1f} m)")
    elif comms:
        c.unres("proximity", "no link_states.csv to measure distance with")

    # --- horizon ------------------------------------------------------------------
    finished = {r: any(e.get("event") == "exploration_complete" for e in evs)
                for r, evs in events.items()}

    def home_state(evs):
        last = None
        for e in evs:
            if e.get("event") == "homing":
                last = e.get("action")
        return last
    homes = {r: home_state(evs) for r, evs in events.items()}
    if end_reason is None:
        c.unres("horizon", "manifest has no run_end_reason")
    elif end_reason == "censored_at_T":
        if len(events) == len(robots) and all(finished.values()):
            away = [r for r in robots if homes.get(r) != "arrived"]
            if away:
                c.fail("horizon", "censored with every robot finished "
                                  "exploring, but not home: " +
                                  ", ".join(f"{r} (homing "
                                            f"{homes.get(r) or 'never'})"
                                            for r in away))
        else:
            c.notes.append("censored while exploring: "
                           + ", ".join(r for r, f in finished.items() if not f))
    elif end_reason == "all_done":
        # A robot whose events could not be read is already UNRESOLVED.
        away = [r for r in events if homes.get(r) != "arrived"]
        if away:
            c.fail("horizon", "all DONE, but not home: " +
                              ", ".join(f"{r} (homing {homes.get(r) or 'never'})"
                                        for r in away))

    # --- give-ups -----------------------------------------------------------------
    if c.counts is not None:
        done = c.counts.get("exchange_done", 0)
        trivial = c.counts.get("exchange_done_trivial", 0)
        gave = c.counts.get("exchange_gave_up", 0)
        nontrivial = done - trivial + gave
        c.counts["exchange_nontrivial"] = done - trivial
        if gave > max(1, GIVEUP_FRAC * nontrivial):
            c.fail("giveups", f"{gave} exchange give-ups of {nontrivial} "
                              f"non-trivial exchanges (limit: 1, or "
                              f"{GIVEUP_FRAC:.0%} if that is more)")
    return c


# --- population -----------------------------------------------------------------

def find_cells(roots):
    cells = []
    for root in roots:
        root = root.rstrip("/")
        if os.path.exists(os.path.join(root, "run_manifest.txt")):
            cells.append(root)
            continue
        if not os.path.isdir(root):
            raise Refused(f"{root}: not a directory")
        for d in sorted(os.listdir(root)):
            p = os.path.join(root, d)
            if os.path.exists(os.path.join(p, "run_manifest.txt")):
                cells.append(p)
    return cells


def show(label, items, per_check=5):
    """Print the first few messages of each check, then how many more."""
    seen = {}
    for check, msg in items:
        seen[check] = seen.get(check, 0) + 1
        if seen[check] <= per_check:
            print(f"    {label:10s}  [{check}] {msg}")
    for check, n in seen.items():
        if n > per_check:
            print(f"    {label:10s}  [{check}] ... {n - per_check} more")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("roots", nargs="+")
    ap.add_argument("--json", help="write the verdicts here as JSON")
    a = ap.parse_args(argv)

    try:
        paths = find_cells(a.roots)
    except Refused as e:
        print(f"REFUSED  {e}")
        return 2
    if not paths:
        print("UNRESOLVED  no cells found (a cell holds run_manifest.txt)")
        return 3

    cells, refused = [], []
    for p in paths:
        try:
            cells.append(check_cell(p))
        except Refused as e:
            refused.append(str(e))
        except OSError as e:
            refused.append(f"{p}: {e}")

    n_fail = n_unres = 0
    for c in cells:
        verdict = ("FAIL" if c.fails else "UNRESOLVED" if c.unresolved
                   else "CLEAN")
        print(f"{verdict:10s}  {c.name}  (arm {c.arm})")
        show("FAIL", c.fails)
        show("UNRESOLVED", c.unresolved)
        for msg in c.notes:
            print(f"    note        {msg}")
        n_fail += bool(c.fails)
        n_unres += bool(c.unresolved) and not c.fails
    for r in refused:
        print(f"REFUSED     {r}")

    # Firing counts per arm, over the arm's cells.
    print()
    print("firing counts per arm (required: a zero fails the arm)")
    arm_fail = arm_unres = 0
    arms = sorted({c.arm for c in cells})
    for arm in arms:
        pop = [c for c in cells if c.arm == arm]
        known = [c for c in pop if c.counts is not None]
        total = {}
        for c in known:
            for k, v in c.counts.items():
                total[k] = total.get(k, 0) + v
        line = []
        for k in FIRING_REQUIRED[arm]:
            v = total.get(k, 0)
            line.append(f"{k}={v}")
            if known and v == 0:
                print(f"  FAIL        {arm}: {k} never fired in "
                      f"{len(known)} cell(s)")
                arm_fail += 1
        if len(known) < len(pop):
            print(f"  UNRESOLVED  {arm}: {len(pop) - len(known)} of {len(pop)} "
                  f"cell(s) have no mechanism counts")
            arm_unres += 1
        print(f"  {arm:10s}  cells={len(pop)}  " + " ".join(line))
        opt = " ".join(f"{k}={total.get(k, 0)}" for k in FIRING_OPTIONAL)
        print(f"  {'':10s}  optional: {opt}")
    for arm in FIRING_REQUIRED:
        if arm not in arms:
            print(f"  {arm:10s}  no cells")

    if a.json:
        with open(a.json, "w") as fh:
            json.dump({
                "cells": [{"cell": c.name, "arm": c.arm,
                           "fails": c.fails, "unresolved": c.unresolved,
                           "notes": c.notes, "counts": c.counts}
                          for c in cells],
                "refused": refused,
            }, fh, indent=1)

    print()
    print(f"{len(cells)} cell(s): {n_fail} FAIL, {n_unres} UNRESOLVED, "
          f"{len(refused)} refused; arms: {arm_fail} firing FAIL, "
          f"{arm_unres} UNRESOLVED")
    # A hard fail outranks a refusal: a caller that reads only the code must
    # not lose a failing cell to an unrelated one it could not read.
    if n_fail or arm_fail:
        return 1
    if refused:
        return 2
    if n_unres or arm_unres:
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
