#!/usr/bin/env python3
"""Known-answer calibration for gen34_check.py.

Builds a synthetic gen-34 cell the checker must pass, then plants ONE defect at
a time and asserts the checker names the right check and exit code. A checker
only ever run on data it passes is not known to be able to fail
([[checks-that-stopped-checking]]). Near-miss cases (just inside a bound) must
stay clean, so a check cannot pass the calib by being too strict either.

Also pins the checker's copies against the source: the schema number and bound
defaults the node stamps, and every mechanism name the firing table reads
against the counters TeamCore and the node write.

Runs the real gen34_check.py.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CHECK = os.path.join(HERE, "gen34_check.py")
PKG = os.path.join(HERE, "..", "explo_planner")
NODE_SRC = os.path.join(PKG, "src", "explo_planner_node.cpp")
CORE_SRC = os.path.join(PKG, "src", "team_core.cpp")
LOG_HDR = os.path.join(PKG, "include", "explo_planner", "experiment_log.hpp")

sys.path.insert(0, HERE)
import gen34_check as G  # noqa: E402

ROBOTS = ["atlas", "bestla", "husky"]
T0 = 100.0
T_DONE = 600.0      # exploration_complete, homing starts
T_HOME = 650.0      # homing arrived
T_END = 700.0

# The clean cell's claims, each on a live link. (t, robot index, peer index)
T_EXCHANGE = 200.0
T_PAIR_MET = 300.0
T_CHASE = 400.0
T_ALLCONN = 500.0


def counts_clean():
    c = {k: 0 for k in G.FIRING_OPTIONAL if k != "exchange_nontrivial"}
    for arm in G.FIRING_REQUIRED.values():
        for k in arm:
            if k != "exchange_nontrivial":
                c[k] = 1
    c["exchange_done"] = 3
    c["exchange_done_trivial"] = 1
    c["exchange_gave_up"] = 0
    return c


class Fixture:
    """A clean 3-robot hybrid cell; cases mutate it before write()."""

    def __init__(self):
        self.manifest = {
            "node": "gen34", "arm": "hybrid", "comms": "1",
            "robots": ",".join(ROBOTS), "run_end_reason": "all_done",
        }
        self.params = dict(G.BOUNDS, arm="hybrid")
        self.schema = G.SCHEMA
        # (pair, lo, hi) spans where the link is down; pair is (a, b), a < b.
        self.down = []
        self.gaps = []          # (pair, lo, hi) spans with no link sample
        self.dist = {}          # (pair, t) -> distance override
        self.events = {r: self._robot_events(k) for k, r in enumerate(ROBOTS)}
        self.write_links = True
        self.torn = None        # robot whose file ends in a torn line

    def _ev(self, k, t, event, **kw):
        e = {"event": event, "seq": 0, "robot": ROBOTS[k], "t_sim_sec": t,
             "t_rel_sec": t - T0, "t_wall_sec": t, "state": "PLAN", "step": 0}
        e.update(kw)
        return e

    def _tick(self, k, t, x, **kw):
        base = dict(x=x, y=3.0 * k, activity="explore", drive="explore",
                    purpose="", leg_status="idle", nearest_peer=(k + 1) % 3,
                    nearest_peer_m=3.0, nearest_peer_age_sec=0.5,
                    wait_start_sec=None, wait_bound_sec=None,
                    all_connected=False, contact_mask=0, booking_slot=-1,
                    proximity_hold=False, state="NAVIGATE")
        base.update(kw)
        state = base.pop("state")
        e = self._ev(k, t, "tick", **base)
        e["state"] = state
        return e

    def _robot_events(self, k):
        peer = (k + 1) % 3
        evs = [self._ev(k, T0, "run_start", schema_version=None, t0_sim_sec=T0,
                        params=None),
               self._ev(k, T0, "team_activity", **{"from": ""}, to="explore",
                        dwell_sec=-1.0)]
        t, x = T0, 0.0
        while t < T_DONE:
            evs.append(self._tick(k, t, x))
            t += 2.0
            x += 0.2
        evs += [
            self._ev(k, T_EXCHANGE - 20, "exchange", action="start", peer=peer),
            self._ev(k, T_EXCHANGE, "exchange", action="done", peer=peer,
                     duration_sec=20.0),
            self._ev(k, T_PAIR_MET, "booking", action="pair_met", slot=1,
                     peer=peer),
            self._ev(k, T_CHASE, "chase", action="contact", peer=peer,
                     after_sec=30.0),
            self._ev(k, T_DONE, "exploration_complete", reason="coverage"),
            self._ev(k, T_DONE, "team_activity", **{"from": "explore"},
                     to="home", dwell_sec=T_DONE - T0),
            self._ev(k, T_DONE, "homing", action="start", reason="finished",
                     dist_m=50.0),
            self._ev(k, T_HOME, "homing", action="arrived", duration_sec=50.0),
            self._ev(k, T_HOME, "team_activity", **{"from": "home"}, to="done",
                     dwell_sec=T_HOME - T_DONE),
        ]
        if k == 0:
            evs.append(self._tick(k, T_ALLCONN + 1.0, 0.0, all_connected=True,
                                  contact_mask=0b110))
        t = T_DONE
        while t < T_HOME:
            evs.append(self._tick(k, t, x, activity="home", drive="leg",
                                  purpose="home", leg_status="driving",
                                  wait_start_sec=T_DONE,
                                  wait_bound_sec=T_DONE + 600.0,
                                  state="RETURN_HOME"))
            t += 2.0
            x -= 0.5
        while t < T_END:
            evs.append(self._tick(k, t, x, activity="done", drive="hold",
                                  leg_status="arrived", state="DONE"))
            t += 2.0
        evs.append(self._ev(k, T_END, "run_end", reason="shutdown",
                            mechanism_counts=counts_clean()))
        evs.sort(key=lambda e: (e["t_sim_sec"], e["event"] != "run_start"))
        return evs

    def link_rows(self):
        rows = []
        pairs = [(0, 1), (0, 2), (1, 2)]
        t = T0
        while t <= T_END + 1e-9:
            for p in pairs:
                if any(p == q and lo <= t <= hi for q, lo, hi in self.gaps):
                    continue
                up = not any(p == q and lo <= t <= hi
                             for q, lo, hi in self.down)
                d = self.dist.get((p, round(t, 1)), 3.0 * abs(p[1] - p[0]))
                rows.append(f"{t:.1f},{p[0]},{p[1]},{d:.2f},0,60.0,20.0,0.0,"
                            f"6.0,{1.0 if up else 0.0},1")
            t = round(t + 0.5, 1)
        return rows

    def write(self, root, name="cell"):
        cell = os.path.join(root, name)
        os.makedirs(cell)
        with open(os.path.join(cell, "run_manifest.txt"), "w") as fh:
            for k, v in self.manifest.items():
                if v is not None:
                    fh.write(f"{k}={v}\n")
        for r, evs in self.events.items():
            with open(os.path.join(cell, f"{r}.events.jsonl"), "w") as fh:
                for e in evs:
                    if e["event"] == "run_start":
                        e = dict(e, schema_version=self.schema,
                                 params=self.params)
                    fh.write(json.dumps(e) + "\n")
                if self.torn == r:
                    fh.write('{"event": "tick", "seq": 9')
        if self.write_links:
            with open(os.path.join(cell, "link_states.csv"), "w") as fh:
                fh.write("t_sim,i,j,distance_m,trees_on_link,path_loss_db,"
                         "snr_db,ber,bandwidth_mbps,connected,valid\n")
                fh.write("\n".join(self.link_rows()) + "\n")
        return cell


def run(roots):
    p = subprocess.run([sys.executable, CHECK] + roots, capture_output=True,
                       text=True)
    return p.returncode, p.stdout + p.stderr


fails = 0


def case(label, mutate, expect_rc, expect_pat=None, cells=1):
    """Build fixture(s), mutate, run; expect rc and a pattern in the output."""
    global fails
    tmp = tempfile.mkdtemp(prefix="g34cal_")
    try:
        fx = Fixture()
        if mutate:
            mutate(fx)
        for n in range(cells):
            fx.write(tmp, f"cell{n}")
        rc, out = run([tmp])
        ok = rc == expect_rc and (expect_pat is None or
                                  re.search(expect_pat, out, re.M))
        print(f"  {'PASS' if ok else 'FAIL'}  {label} (rc={rc}, want "
              f"{expect_rc})")
        if not ok:
            fails += 1
            print("        " + out.replace("\n", "\n        "))
    finally:
        shutil.rmtree(tmp)


def robot_evs(fx, k):
    return fx.events[ROBOTS[k]]


def first(fx, k, pred):
    return next(e for e in robot_evs(fx, k) if pred(e))


def drop(fx, k, pred):
    fx.events[ROBOTS[k]] = [e for e in robot_evs(fx, k) if not pred(e)]


def set_counts(fx, **kw):
    for r in ROBOTS:
        end = next(e for e in fx.events[r] if e["event"] == "run_end")
        end["mechanism_counts"] = dict(end["mechanism_counts"])
        end["mechanism_counts"].update(kw)


def insert(fx, k, e):
    fx.events[ROBOTS[k]].append(e)
    fx.events[ROBOTS[k]].sort(key=lambda e: e["t_sim_sec"])


# --- the cases ------------------------------------------------------------------

print("clean")
case("clean cell", None, 0, r"^CLEAN\s+cell0")
case("clean cells pool the firing counts", None, 0,
     r"hybrid\s+cells=2 .*booking_full_met=6", cells=2)

print("contact")
W = G.BOUNDS["presence_window_sec"]
S = G.LINK_SLACK_SEC
case("exchange done on a dead link",
     lambda f: f.down.append(((0, 1), T_EXCHANGE - W - S - 1, T_EXCHANGE + 5)),
     1, r"\[contact\] atlas t=200\.0: exchange done with bestla")
case("exchange done, link up inside the window only (clean)",
     lambda f: f.down.append(((0, 1), T_EXCHANGE - 30, T_EXCHANGE - W + 1)),
     0)
case("pair_met on a dead link",
     lambda f: f.down.append(((0, 1), T_PAIR_MET - W - S - 1, T_PAIR_MET + 5)),
     1, r"\[contact\] atlas t=300\.0: booking pair_met with bestla")
case("chase contact on a dead link",
     lambda f: f.down.append(((1, 2), T_CHASE - W - S - 1, T_CHASE + 5)),
     1, r"\[contact\] bestla t=400\.0: chase contact with husky")
TA = T_ALLCONN + 1.0
case("all_connected with the far pair down for two windows",
     lambda f: f.down.append(((1, 2), TA - 2 * W - S - 1, TA + 5)),
     1, r"\[contact\] atlas t=501\.0: all_connected, but link bestla-husky")
case("all_connected, far pair up inside two windows (clean)",
     lambda f: f.down.append(((1, 2), TA - 40, TA - 2 * W + 1)), 0)
# Last up between one and two windows back: legal only because the claim about
# a far pair rests on a peer's hears_mask, which is itself up to W old.
case("all_connected, far pair last up 15 s before (clean)",
     lambda f: f.down.append(((1, 2), TA - 14.5, TA + 5)), 0)
case("all_connected, own pair last up 15 s before",
     lambda f: f.down.append(((0, 1), TA - 14.5, TA + 5)), 1,
     r"all_connected, but link atlas-bestla")
case("all_connected with its own pair down for one window",
     lambda f: f.down.append(((0, 2), TA - W - S - 1, TA + 5)),
     1, r"all_connected, but link atlas-husky")
case("exchange done, no link sample in the window",
     lambda f: f.gaps.append(((0, 1), T_EXCHANGE - 30, T_EXCHANGE + 5)),
     3, r"UNRESOLVED\s+\[contact\] atlas t=200\.0")
case("exchange done names a robot outside the run",
     lambda f: first(f, 0, lambda e: e.get("action") == "done").update(peer=7),
     1, r"names peer 7")


def no_links(f):
    f.write_links = False
case("COMMS=1 without link_states.csv", no_links, 3,
     r"\[contact\] COMMS=1 but no link_states\.csv")


def comms_off(f):
    f.write_links = False
    f.manifest["comms"] = "0"
case("COMMS=0 (clean, with a note)", comms_off, 0, r"note\s+COMMS=0")

print("waits")


def past_bound(slack):
    def m(f):
        for k in range(3):
            for e in robot_evs(f, k):
                if e["event"] == "tick" and e["activity"] == "home":
                    e["wait_bound_sec"] = T_DONE + 20.0
        # The first home tick past the bound lands at slack past it.
        insert(f, 0, f._tick(0, T_DONE + 20.0 + slack, 0.0, activity="home",
                             drive="leg", leg_status="driving",
                             wait_start_sec=T_DONE,
                             wait_bound_sec=T_DONE + 20.0,
                             state="RETURN_HOME"))
        for k in range(3):
            drop(f, k, lambda e: e["event"] == "tick" and
                 e["activity"] == "home" and e["t_sim_sec"] > T_DONE + 20.0 and
                 e["t_sim_sec"] != T_DONE + 20.0 + slack)
    return m
case("tick past its wait bound", past_bound(1.5), 1,
     r"\[waits\] atlas t=621\.5: home is 1\.5 s past its bound")
case("tick inside the bound's slack (clean)", past_bound(0.8), 0)


def gave_up(f):
    first(f, 1, lambda e: e.get("action") == "arrived").update(action="gave_up")
case("homing gave_up", gave_up, 1, r"\[waits\] bestla t=650\.0: homing gave_up")

print("flipflop")


def flips(n, dwell, gap):
    """n activity changes, alternating explore/meet, starting t=150."""
    def m(f):
        drop(f, 0, lambda e: e["event"] == "team_activity" and
             e["from"] == "explore" and e["to"] == "home")
        t, a, b = 150.0, "explore", "meet"
        for i in range(n):
            insert(f, 0, f._ev(0, t, "team_activity", **{"from": a}, to=b,
                               dwell_sec=(t - T0) if i == 0 else dwell))
            a, b = b, a
            t += gap
        insert(f, 0, f._ev(0, T_DONE, "team_activity", **{"from": a},
                           to="home", dwell_sec=10.0))
    return m
case("7 activity changes in 60 s", flips(7, 9.0, 9.0), 1,
     r"\[flipflop\] atlas: 7 activity changes in 60 s")
case("6 activity changes in 60 s (clean)", flips(6, 9.0, 9.0), 0)
case("3 short stints in a row", flips(4, 1.0, 1.0), 1,
     r"\[flipflop\] atlas t=153\.0: 3 short stints in a row")
case("2 short stints in a row (clean)", flips(3, 1.0, 1.0), 0)
case("the starting activity is not a change (clean)",
     lambda f: [insert(f, 0, f._ev(0, T0 + 0.1 * i, "team_activity",
                                   **{"from": ""}, to="explore",
                                   dwell_sec=-1.0)) for i in range(6)], 0)

print("proximity")
case("pair under 1 m", lambda f: f.dist.update({((0, 1), 250.0): 0.8}), 1,
     r"\[proximity\] atlas-bestla 0\.80 m apart at t=250\.0")
case("pair at 1.05 m (clean)", lambda f: f.dist.update({((0, 1), 250.0): 1.05}),
     0)

print("progress")


def still(sec, **kw):
    def m(f):
        for e in robot_evs(f, 2):
            if e["event"] == "tick" and 300.0 <= e["t_sim_sec"] <= 300.0 + sec:
                e["x"] = 500.0   # off the explore path
                e.update(kw)
    return m
case("explorer still for 130 s", still(130.0), 1,
     r"\[progress\] husky: moved 0\.00 m in the 120 s to t=420\.0")
case("explorer still for 110 s (clean)", still(110.0), 0)
case("still for 130 s in a proximity hold (clean)",
     still(130.0, proximity_hold=True, state="PROXIMITY_HOLD"), 0)
case("still for 130 s at a leg's end (clean)",
     still(130.0, activity="meet", drive="leg", leg_status="arrived",
           state="MEET"), 0)
case("still for 130 s on a leg still driving",
     still(130.0, activity="meet", drive="leg", leg_status="driving",
           state="MEET"), 1, r"\[progress\] husky: moved 0\.00 m .*meet, MEET")

print("horizon")


def interrupted(reason):
    def m(f):
        f.manifest["run_end_reason"] = reason
        insert(f, 2, f._ev(2, T_HOME + 5, "homing", action="interrupted",
                           by="meet"))
    return m
case("censored, all finished, one not home", interrupted("censored_at_T"), 1,
     r"\[horizon\] censored with every robot finished exploring, but not "
     r"home: husky \(homing interrupted\)")
case("all_done, one not home", interrupted("all_done"), 1,
     r"\[horizon\] all DONE, but not home: husky")


def censored_exploring(f):
    f.manifest["run_end_reason"] = "censored_at_T"
    drop(f, 2, lambda e: e["event"] in ("exploration_complete", "homing"))
case("censored while one explores (clean, with a note)", censored_exploring, 0,
     r"note\s+censored while exploring: husky")

print("giveups")


def robot_counts(per_robot):
    """Per-robot mechanism_counts overrides, e.g. {0: {...}, 2: {...}}."""
    def m(f):
        for k, kw in per_robot.items():
            end = next(e for e in robot_evs(f, k) if e["event"] == "run_end")
            end["mechanism_counts"] = dict(end["mechanism_counts"], **kw)
    return m
# Clean counts per robot: exchange_done 3, trivial 1. Summed: 9 done, 3 trivial.
case("3 give-ups of 9 non-trivial",
     robot_counts({k: {"exchange_gave_up": 1} for k in range(3)}), 1,
     r"\[giveups\] 3 exchange give-ups of 9 non-trivial")
case("2 give-ups of 8 non-trivial (clean: 25 %)",
     robot_counts({0: {"exchange_gave_up": 1}, 1: {"exchange_gave_up": 1}}),
     0)
case("2 give-ups of 3 non-trivial",
     robot_counts({k: {"exchange_done": 1, "exchange_done_trivial": 1 - (k == 0),
                       "exchange_gave_up": int(k > 0)} for k in range(3)}), 1,
     r"\[giveups\] 2 exchange give-ups of 3 non-trivial")
case("1 give-up of 2 non-trivial (clean: the floor is 1)",
     robot_counts({k: {"exchange_done": 1, "exchange_done_trivial": int(k > 0),
                       "exchange_gave_up": int(k == 1)} for k in range(3)}), 0)

print("bounds")
case("stamped presence window differs",
     lambda f: f.params.update(presence_window_sec=12.0), 1,
     r"\[bounds\] atlas: presence_window_sec=12, the checker's bound is 10")
case("stamped arm differs from the manifest",
     lambda f: f.params.update(arm="pursuit"), 1,
     r"\[bounds\] atlas: run_start arm='pursuit' but the manifest says "
     r"'hybrid'")
case("a bound not stamped",
     lambda f: f.params.pop("chase_limit_sec"), 3,
     r"\[bounds\] atlas: chase_limit_sec not stamped")

print("refusals")
case("schema 12", lambda f: setattr(f, "schema", 12), 2,
     r"REFUSED\s+cell0/atlas: schema_version 12")
case("gen-33 manifest", lambda f: f.manifest.update(node="gen33"), 2,
     r"REFUSED\s+cell0: manifest node=gen33")
case("no node line", lambda f: f.manifest.update(node=None), 2,
     r"node=<absent>")
case("unknown arm", lambda f: f.manifest.update(arm="exploit"), 2,
     r"arm='exploit' is not a gen-34 arm")


def mixed_refused_and_fail():
    """One refused cell beside one hard-failing cell: the fail sets the code."""
    global fails
    tmp = tempfile.mkdtemp(prefix="g34cal_")
    try:
        bad = Fixture()
        bad.manifest.update(node="gen33")
        bad.write(tmp, "cell0")
        failing = Fixture()
        failing.params.update(presence_window_sec=12.0)
        failing.write(tmp, "cell1")
        rc, out = run([tmp])
        ok = rc == 1 and re.search(r"REFUSED\s+cell0", out) and \
            re.search(r"\[bounds\] atlas: presence_window_sec=12", out)
        print(f"  {'PASS' if ok else 'FAIL'}  a hard fail outranks a refused "
              f"cell (rc={rc}, want 1)")
        if not ok:
            fails += 1
            print("        " + out.replace("\n", "\n        "))
    finally:
        shutil.rmtree(tmp)


mixed_refused_and_fail()

print("firing")
case("booking_full_met never fired",
     lambda f: set_counts(f, booking_full_met=0), 1,
     r"FAIL\s+hybrid: booking_full_met never fired in 1 cell")
case("every exchange trivial",
     lambda f: set_counts(f, exchange_done=1, exchange_done_trivial=1), 1,
     r"FAIL\s+hybrid: exchange_nontrivial never fired")
case("an optional mechanism at zero (clean)",
     lambda f: set_counts(f, chase_first_goal=0), 0)
case("pursuit arm needs no meeting counters (clean)",
     lambda f: [f.manifest.update(arm="pursuit"),
                f.params.update(arm="pursuit"),
                set_counts(f, plan_firmed=0, booking_departed=0,
                           booking_full_met=0)], 0)
case("rendezvous arm needs no chase counters (clean)",
     lambda f: [f.manifest.update(arm="rendezvous"),
                f.params.update(arm="rendezvous"),
                set_counts(f, chase_start=0, chase_first_intercept=0,
                           activity_follow=0, chase_done=0)], 0)
case("rendezvous arm: chase_done is not required, plan_firmed is",
     lambda f: [f.manifest.update(arm="rendezvous"),
                f.params.update(arm="rendezvous"),
                set_counts(f, plan_firmed=0)], 1,
     r"FAIL\s+rendezvous: plan_firmed never fired")


def truncated(f):
    drop(f, 1, lambda e: e["event"] == "run_end")
    f.torn = "bestla"
case("torn file, no run_end", truncated, 3,
     r"UNRESOLVED\s+\[events\] bestla: line \d+ is not JSON \(torn last "
     r"line\)")

# --- pins against the source ----------------------------------------------------

print("pins")


def pin(label, ok, detail=""):
    global fails
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{': ' + detail if detail else ''}")
    if not ok:
        fails += 1


hdr = open(LOG_HDR).read()
m = re.search(r"kGen34SchemaVersion\s*=\s*(\d+)", hdr)
pin("schema pin agrees with experiment_log.hpp", m and int(m.group(1)) == G.SCHEMA,
    f"header {m.group(1) if m else '?'}, checker {G.SCHEMA}")

node = open(NODE_SRC).read()
for k, v in G.BOUNDS.items():
    mm = re.search(r'\bdp\("' + re.escape(k) + r'",\s*([0-9.eE+-]+)\)', node)
    have = float(mm.group(1)) if mm else None
    pin(f"bound {k} equals the node default", have == v,
        f"node {have}, checker {v}")
    stamped = re.search(r'addParamNum\(\s*"' + re.escape(k) + '"', node)
    pin(f"bound {k} is stamped in run_start", bool(stamped))

core = open(CORE_SRC).read()
mm = re.search(r"counterNames\(\)\s*\{.*?=\s*\{(.*?)\};", core, re.S)
names = set(re.findall(r'"([^"]+)"', mm.group(1))) if mm else set()
names |= set(re.findall(r'e\.mechanism_counts\["([^"]+)"\]', node))
pin("counter list parsed", len(names) > 60, f"{len(names)} names")
wanted = set(G.FIRING_OPTIONAL)
for arm in G.FIRING_REQUIRED.values():
    wanted |= set(arm)
wanted -= {"exchange_nontrivial"}
wanted |= {"exchange_done", "exchange_done_trivial", "exchange_gave_up"}
missing = sorted(wanted - names)
pin("every firing name is a counter the run writes", not missing,
    ", ".join(missing))

print(f"\n{'ALL PASS' if fails == 0 else str(fails) + ' FAILURE(S)'}")
sys.exit(1 if fails else 0)
