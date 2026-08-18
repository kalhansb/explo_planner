#!/usr/bin/env python3
"""Is reconnecting worth what it costs?

  COST     chase_sec   time in PURSUE -- driving at the peer instead of exploring
  BENEFIT  vox_gain    map voxels learned from the peer when the link returns
  NET      t_complete  exploration completion time for the whole run

Measured from three independent sources so a bug in one does not silently
propagate: chase cost from the planner's own PURSUE state transitions, benefit
from the fused-map cache series in the planner log, and the reconnection instants
from the comms emulator's link_states.csv.

Two pairing bugs a first cut got wrong, both worth stating because the numbers
look plausible either way:

  * Chase cost must come from PURSUE intervals, NOT from dispatch->reconnect_end.
    A DECLINED dispatch (resume_exploring / meeting_point) never enters PURSUE,
    and pairing each dispatch with "the next end event" makes four declines in
    p12_pursuit_seed1 all match one distant end, reporting 1390 s of chasing for
    a robot that never chased.
  * Benefit must be keyed on the RESTORATION, not the dispatch. Several
    dispatches can precede one reconnection; charging the same merge to each
    triple-counted the gain.

Benefit caveat, and it is a large one: a robot keeps mapping on its own during
the settle window, so vox_gain is an UPPER BOUND on what the peer contributed,
not a measurement of it. Measured self-rates run 200-3900 vox/s, so 120 s of
solo driving accounts for 24k-470k voxels -- the same order as the gains
themselves. vox_net subtracts self_rate * SETTLE_SEC as a first-order baseline.
It goes negative where the robot was mapping fast into new ground and the peer
had little to add, which is a real signal, not a defect.

Robot logs carry wall-clock; events carry both clocks and supply the anchors.
"""
import csv
import glob
import json
import os
import re
import sys

ROOT = os.environ.get("HMR_CAMPAIGN_ROOT", "/tmp/hmr_campaign")
SETTLE_SEC = 120.0     # window after link return in which the merge lands
SUSTAIN_SEC = 10.0     # connected must hold this long to count as restored
MIN_OUTAGE_SEC = 60.0  # ignore brief flicker; real outages are minutes
ROBOTS = ("atlas", "bestla")
CACHE_RE = re.compile(r"\[INFO\] \[(\d+\.\d+)\].*local map_cache_: (\d+) voxels")


def manifest(d):
    m = {}
    p = os.path.join(d, "run_manifest.txt")
    if os.path.exists(p):
        for ln in open(p, errors="ignore"):
            if "=" in ln and not ln.startswith("#"):
                k, _, v = ln.strip().partition("=")
                m[k] = v
    return m


def events(d, rob):
    out = []
    p = os.path.join(d, f"{rob}.events.jsonl")
    if os.path.exists(p):
        for ln in open(p, errors="ignore"):
            ln = ln.strip()
            if ln:
                try:
                    out.append(json.loads(ln))
                except Exception:
                    pass          # partial trailing line on a live run
    return out


def w2s_factory(evs):
    a = sorted((float(e["t_wall_sec"]), float(e["t_sim_sec"])) for e in evs
               if e.get("t_wall_sec") and e.get("t_sim_sec") is not None)

    def f(w):
        if not a:
            return None
        if w <= a[0][0]:
            return a[0][1]
        if w >= a[-1][0]:
            return a[-1][1]
        lo, hi = 0, len(a) - 1
        while hi - lo > 1:
            m = (lo + hi) // 2
            if a[m][0] <= w:
                lo = m
            else:
                hi = m
        (w0, s0), (w1, s1) = a[lo], a[hi]
        return s0 if w1 == w0 else s0 + (s1 - s0) * (w - w0) / (w1 - w0)
    return f


def cache_series(d, rob, w2s):
    out = []
    p = os.path.join(d, f"planner_{rob}.log")
    if os.path.exists(p):
        for ln in open(p, errors="ignore"):
            m = CACHE_RE.search(ln)
            if m:
                t = w2s(float(m.group(1)))
                if t is not None:
                    out.append((t, int(m.group(2))))
    out.sort()
    return out


def at(series, t):
    v = None
    for ts, vox in series:
        if ts <= t:
            v = vox
        else:
            break
    return v


def pursue_intervals(evs):
    """Authoritative chase episodes: PURSUE entry -> exit."""
    out, enter = [], None
    for e in evs:
        if e.get("event") != "state_change":
            continue
        t = float(e.get("t_sim_sec") or 0)
        if e.get("to") == "PURSUE":
            enter = t
        elif e.get("from") == "PURSUE" and enter is not None:
            out.append((enter, t, e.get("reason") or ""))
            enter = None
    if enter is not None:
        out.append((enter, None, "unterminated"))   # run ended mid-chase
    return out


def outages(d):
    """Sustained link outages as (t_drop, t_restore). t_restore is the start of
    a connected run lasting >= SUSTAIN_SEC, so a single flicker sample inside an
    outage does not end it -- in hybrid_seed1 a lone connected at t=570 is
    followed by a 0 at 580, and anchoring on it shifts the window ~40 s early."""
    lk = []
    p = os.path.join(d, "link_states.csv")
    if not os.path.exists(p):
        return [], []
    with open(p) as fh:
        for row in csv.DictReader(fh):
            try:
                lk.append((float(row["t_sim"]), float(row["distance_m"]),
                           int(float(row["connected"]))))
            except (ValueError, KeyError, TypeError):
                continue
    lk.sort()
    out, drop, run0 = [], None, None
    for t, _, c in lk:
        if not c:
            run0 = None
            if drop is None:
                drop = t
        else:
            if run0 is None:
                run0 = t
            if drop is not None and t - run0 >= SUSTAIN_SEC:
                if run0 - drop >= MIN_OUTAGE_SEC:
                    out.append((drop, run0))
                drop = None
    if drop is not None:
        out.append((drop, None))          # never restored before run end
    return out, lk


def sep_at(lk, t):
    v = None
    for ts, dist, _ in lk:
        if ts <= t:
            v = dist
        else:
            break
    return v


def analyse(d):
    mf = manifest(d)
    cell = os.path.basename(d)
    outs, lk = outages(d)
    # reconnect_mode_requested, NOT reconnect_mode_param: an `off` cell still
    # records mode_param=hybrid (the mode is simply never consulted), so keying
    # on _param silently files all four off cells under hybrid.
    rec = {"cell": cell, "arm": mf.get("reconnect_mode_requested", "?"),
           "seed": mf.get("seed", "?"),
           "budget": (mf.get("pursuit_budget_max_sec") or "?").replace(".0", ""),
           "t_complete": mf.get("run_end_t_sim"),
           "end_reason": mf.get("run_end_reason"),
           "gates": mf.get("run_gates_verdict"),
           "outages": outs, "robots": {}}

    for rob in ROBOTS:
        evs = events(d, rob)
        ser = cache_series(d, rob, w2s_factory(evs))
        chases = pursue_intervals(evs)
        disp = [e for e in evs if e.get("event") == "reconnect_dispatch"]

        gains = []
        for t_drop, t_res in outs:
            if t_res is None or not ser:
                gains.append((t_drop, t_res, None, None, None, None))
                continue
            pre = at(ser, t_res)
            post = at(ser, t_res + SETTLE_SEC)
            b2 = at(ser, t_res - 60.0)
            rate = None
            if pre is not None and b2 is not None:
                rate = (pre - b2) / 60.0
            gain = (post - pre) if (pre is not None and post is not None) else None
            net = (gain - rate * SETTLE_SEC) if (gain is not None and rate is not None) else None
            gains.append((t_drop, t_res, pre, gain, rate, net))

        rec["robots"][rob] = {
            "chases": chases,
            "chase_sec": sum((e - s) for s, e, _ in chases if e is not None),
            "n_dispatch": len(disp),
            "n_chase_dispatch": sum(1 for e in disp if e.get("action") == "chase"),
            "n_declined": sum(1 for e in disp if e.get("decline_reason")),
            "actions": [e.get("action") for e in disp],
            "gains": gains,
            "_disp": disp,
        }
    rec["lk"] = lk
    return rec



def timing_table(cells):
    """The metric that can actually separate the arms.

    Merge SIZE cannot: the `off` arm, which never deliberately reconnects, has
    the highest median vox_gain of any arm, because robots drift back into range
    by accident in every arm and the merge fires either way. What deliberate
    reconnection should change is how LONG a robot sits disconnected before that
    merge, and how much of the run it spends out of contact.

    disc_frac is the headline: total disconnected seconds / run length. It is
    scale-free, so a run that takes 4332 s and one that takes 1092 s compare
    directly -- which matters because run length is itself an outcome here.

    Outages are split by whether the planner dispatched a reconnection during
    them. A working reconnection strategy should show dispatched outages ending
    sooner than undispatched ones WITHIN the same run, which controls for map
    difficulty in a way cross-arm medians cannot.
    """
    print("\n=== TIMING: how long robots stay out of contact ===")
    print(f"{'cell':<24} {'arm':<11} {'t_done':>7} {'disc_s':>7} {'disc%':>6} "
          f"{'n_out':>5} {'med_out':>8} {'disp_out':>9} {'nodisp_out':>11}")
    for c in cells:
        T = float(c["t_complete"] or 0)
        if not T:
            continue
        # dispatch instants, either robot -- the decision is per-robot but the
        # outage is a property of the pair
        disp_t = sorted(float(e.get("t_sim_sec") or 0)
                        for rob in ROBOTS
                        for e in c["robots"][rob]["_disp"])
        durs, dd, nd = [], [], []
        for t_drop, t_res in c["outages"]:
            end = t_res if t_res is not None else T
            d = end - t_drop
            durs.append(d)
            (dd if any(t_drop <= x <= end for x in disp_t) else nd).append(d)
        tot = sum(durs)
        med = lambda xs: sorted(xs)[len(xs) // 2] if xs else None
        f = lambda v: f"{v:8.0f}" if v is not None else f"{'-':>8}"
        print(f"{c['cell']:<24} {c['arm']:<11} {T:7.0f} {tot:7.0f} "
              f"{100.0 * tot / T:5.1f}% {len(durs):5d} {f(med(durs))} "
              f"{f(med(dd)):>9} {f(med(nd)):>11}")

    print("\n=== TIMING BY ARM (median across cells) ===")
    by = {}
    for c in cells:
        T = float(c["t_complete"] or 0)
        if not T:
            continue
        tot = sum((r if r is not None else T) - d for d, r in c["outages"])
        by.setdefault((c["arm"], c["budget"]), []).append(
            (T, tot, 100.0 * tot / T, len(c["outages"])))
    print(f"{'arm':<11} {'bud':>5} {'n':>3} {'t_done':>8} {'disc_s':>8} "
          f"{'disc%':>7} {'n_out':>6}")
    for k, v in sorted(by.items()):
        med = lambda i: sorted(x[i] for x in v)[len(v) // 2]
        print(f"{k[0]:<11} {k[1]:>5} {len(v):>3} {med(0):8.0f} {med(1):8.0f} "
              f"{med(2):6.1f}% {med(3):6.1f}")


def main(argv):
    tags = argv or ["p13smoke", "p12"]
    cells = []
    for tag in tags:
        for d in sorted(glob.glob(os.path.join(ROOT, tag + "_*"))):
            if os.path.isdir(d) and not d.endswith(".attempts"):
                cells.append(analyse(d))
    if not cells:
        print("no cells matched")
        return

    print("=== PER-OUTAGE: what each reconnection cost and returned ===")
    print(f"{'cell':<24} {'rob':<7} {'outage':>15} {'dur_s':>7} "
          f"{'sepDrop':>8} {'chase_s':>8} {'vox_pre':>9} {'vox_gain':>9} {'self/s':>7}")
    for c in cells:
        for rob in ROBOTS:
            r = c["robots"][rob]
            for (t_drop, t_res, pre, gain, rate, net) in r["gains"]:
                # chase seconds overlapping this outage window
                ov = 0.0
                for s, e, _ in r["chases"]:
                    if e is None or t_res is None:
                        continue
                    lo, hi = max(s, t_drop), min(e, t_res + SETTLE_SEC)
                    if hi > lo:
                        ov += hi - lo
                dur = (t_res - t_drop) if t_res else None
                g = lambda v, w, p=1: (f"{v:{w}.{p}f}" if isinstance(v, float)
                                       else f"{str(v if v is not None else '-'):>{w}}")
                print(f"{c['cell']:<24} {rob:<7} "
                      f"{f'{t_drop:.0f}->' + (f'{t_res:.0f}' if t_res else 'never'):>15} "
                      f"{g(dur,7,0)} {g(sep_at(c['lk'], t_drop),8)} {ov:8.1f} "
                      f"{str(pre if pre is not None else '-'):>9} "
                      f"{str(gain if gain is not None else '-'):>9} {g(rate,7)} "
                      f"{g(net,9,0)}")

    print("\n=== PER-CELL LEDGER ===")
    print(f"{'cell':<24} {'arm':<11} {'bud':>5} {'done':<13} {'t_done':>7} "
          f"{'chase_s':>8} {'vox_gain':>9} {'vox_net':>11} {'disp':>5} "
          f"{'chased':>6} {'outg':>5}")
    for c in cells:
        chase = sum(c["robots"][r]["chase_sec"] for r in ROBOTS)
        gain = sum(g or 0 for r in ROBOTS
                   for (_, _, _, g, _, _) in c["robots"][r]["gains"])
        net = sum(n or 0 for r in ROBOTS
                  for (_, _, _, _, _, n) in c["robots"][r]["gains"])
        nd = sum(c["robots"][r]["n_dispatch"] for r in ROBOTS)
        nc = sum(c["robots"][r]["n_chase_dispatch"] for r in ROBOTS)
        print(f"{c['cell']:<24} {c['arm']:<11} {c['budget']:>5} "
              f"{str(c['end_reason']):<13} {str(c['t_complete'] or '-'):>7} "
              f"{chase:8.1f} {gain:9d} {net:9.0f} {nd:5d} {nc:6d} "
              f"{len(c['outages']):5d}")

    print("\n=== PER-ARM SUMMARY (complete cells only) ===")
    by = {}
    for c in cells:
        if c["end_reason"] not in ("all_done", "censored_at_T"):
            continue
        k = (c["arm"], c["budget"])
        chase = sum(c["robots"][r]["chase_sec"] for r in ROBOTS)
        gain = sum(g or 0 for r in ROBOTS
                   for (_, _, _, g, _, _) in c["robots"][r]["gains"])
        net = sum(n or 0 for r in ROBOTS
                  for (_, _, _, _, _, n) in c["robots"][r]["gains"])
        nc = sum(c["robots"][r]["n_chase_dispatch"] for r in ROBOTS)
        nd = sum(c["robots"][r]["n_dispatch"] for r in ROBOTS)
        by.setdefault(k, []).append((float(c["t_complete"] or 0), chase, gain, nc, nd, net))
    print(f"{'arm':<11} {'bud':>5} {'n':>3} {'t_done med':>11} {'chase_s med':>12} "
          f"{'vox_gain med':>13} {'vox_net med':>12} {'chase/disp':>11}")
    for (arm, bud), v in sorted(by.items()):
        med = lambda xs: sorted(xs)[len(xs) // 2]
        nc = sum(x[3] for x in v)
        nd = sum(x[4] for x in v)
        print(f"{arm:<11} {bud:>5} {len(v):>3} {med([x[0] for x in v]):11.0f} "
              f"{med([x[1] for x in v]):12.1f} {med([x[2] for x in v]):13.0f} "
              f"{med([x[5] for x in v]):12.0f} {nc:>5}/{nd:<5}")
    timing_table(cells)


if __name__ == "__main__":
    main(sys.argv[1:])
