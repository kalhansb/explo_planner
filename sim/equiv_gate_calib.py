#!/usr/bin/env python3
# Moved comments: docs/sim_notes/equiv_gate_calib_notes.md
"""Known-answer calibration for equiv_gate.py.

The plan (§6) requires the equivalence gate to be "calibrated in both
directions before first use: A/A (two same-binary runs must pass) and a
known-answer injection (a deliberately enabled new event kind must fail)".
This file is that calibration, widened to every check the gate makes, because
this repo has already watched six guards go inert while still printing passes
and the only thing that would have caught it was a case that must FAIL.

Two rules, both learned the hard way:

  * A case asserts on the RETURN CODE and on a message pattern. Matching a
    substring of stdout alone is how "check 3" passed forever against a gate
    that only ever prints "check 3b".
  * The fixture's event shapes are audited against a real banked cell at the
    bottom. A fixture written from the reader's beliefs tests the reader's
    beliefs; that is precisely how gate_g8's fixture once reported 18/18 PASS
    while the gate could neither fail on a stale binary nor pass on real data.

Usage:  equiv_gate_calib.py
Env:    EQUIV_CALIB_REAL_ROOT   campaign root to audit fixture shapes against
                                (default /home/kalhan/hmr_campaign)
Exit:   0  every case behaved as declared
        1  at least one did not
"""

import copy
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
GATE = os.path.join(HERE, "equiv_gate.py")

ROBOTS = ["atlas", "bestla"]
SCHEMA = 4

# The param dump. Names and types copied from the WRITER — explo_planner_node.cpp
# addParam* calls and the dp() block behind them — not from what equiv_gate
# expects to find. Numbers are floats because addParamNum writes them as such,
# which is exactly the sort of detail a from-beliefs fixture gets wrong.
BASE_PARAMS = {
    "git_rev": "3d4306c",
    "build_stamp": "Aug 27 2026 17:19:04",
    "node_name": "explo_planner",
    "robot_name": "atlas",
    "reconnect_mode": "hybrid",
    "arm": "hybrid",
    "use_sim_time": True,
    "output_csv": "/tmp/parent/planner_atlas.csv",
    "max_steps": 500.0,
    "metrics_period_sec": 5.0,
    "coordination_enabled": True,
    "rendezvous_enabled": True,
    "done_unknown_fraction": 0.64,
    "done_criterion": "latch",
    "done_coverage_source": "scovox",
    "proximity_stop_enabled": True,
}

# What P0 actually added, at the values an unconfigured run shows. The gate must
# accept these: that is the whole point of "new params present, all at
# defaults".
P0_NEW_PARAMS = {
    "team_robot_names": "",
    "robot_id": -1.0,
    "team_hash": 0.0,
}

MANIFEST = """# hmr_explo comms/reconnection run manifest
started_utc=2026-08-27T17:28:44Z
host=IST031A-01-L
outdir=/tmp/parent
reconnect_mode_requested=hybrid
reconnect_mode_param=hybrid
rendezvous_enabled=true
comms=1
seed=1
tx_power_dbm=30.0
record=0
scenario=flatforest_dense_2robot_lidar.yaml
git_explo_planner=3d4306c
sha256_explo_planner_node=aaaaaaaaaaaaaaaa
run_end_reason=all_done
run_end_t_sim=812.4
run_gates_verdict=CLEAN
"""

GATES = ("PASS\tleakage\t4 gated topic(s) clean\n"
         "PASS\trelay_set\tall 4 expected relay topic(s) present\n"
         "PASS\tqos\t4 relay topic(s) QoS-compatible\n"
         "PASS\toverflow\t2 link(s), no overflow\n"
         "PASS\todom\t2 pose source(s) live\n")


def base_events(robot, params):
    """One robot-run's events: only the kinds the gate reads structure from.

    Deliberately NOT a full run. The gate looks at run_start and at the SET of
    event kinds, so a fixture that faked 500 steps would add nothing but the
    illusion of realism.
    """
    p = dict(params)
    p["robot_name"] = robot
    return [
        {"event": "run_start", "seq": 0, "robot": robot, "t_sim_sec": 26.2,
         "t_rel_sec": 0.0, "t_wall_sec": 1787851726.237575,
         "state": "WAIT_FOR_MAP", "step": 0, "schema_version": SCHEMA,
         "t0_sim_sec": 26.2, "coverage_milestones": [0.9, 0.85],
         "params": p},
        {"event": "step", "seq": 1, "robot": robot, "t_sim_sec": 30.0,
         "t_rel_sec": 3.8, "t_wall_sec": 1787851730.0, "state": "EXPLORE",
         "step": 1},
        {"event": "state_change", "seq": 2, "robot": robot, "t_sim_sec": 31.0,
         "t_rel_sec": 4.8, "t_wall_sec": 1787851731.0, "state": "EXPLORE",
         "step": 1, "from": "WAIT_FOR_MAP", "to": "EXPLORE"},
        {"event": "exploration_complete", "seq": 3, "robot": robot,
         "t_sim_sec": 800.0, "t_rel_sec": 773.8, "t_wall_sec": 1787852500.0,
         "state": "DONE", "step": 120},
        {"event": "run_end", "seq": 4, "robot": robot, "t_sim_sec": 812.4,
         "t_rel_sec": 786.2, "t_wall_sec": 1787852512.0, "state": "DONE",
         "step": 120, "reason": "all_done"},
    ]


def build(root, cells, events_by_robot, manifest=MANIFEST, gates=GATES):
    """Write a campaign root of `cells` cell directories.

    The root is created even when `cells` is empty. That case is the point of
    the empty-population tests: an ABSENT root and a root with nothing in it
    are different situations, and only the second one exercises "I found no
    cells to compare".
    """
    os.makedirs(root, exist_ok=True)
    for cell in cells:
        d = os.path.join(root, cell)
        os.makedirs(d, exist_ok=True)
        if manifest is not None:
            open(os.path.join(d, "run_manifest.txt"), "w").write(manifest)
        if gates is not None:
            open(os.path.join(d, "comms_gates.txt"), "w").write(gates)
        for robot, evs in events_by_robot.items():
            with open(os.path.join(d, f"{robot}.events.jsonl"), "w") as f:
                for e in evs:
                    f.write(json.dumps(e) + "\n")
    return root


def parent_side():
    return {r: base_events(r, BASE_PARAMS) for r in ROBOTS}


def child_side():
    """The P0 child: same configuration, three new params at their defaults."""
    p = dict(BASE_PARAMS, **P0_NEW_PARAMS)
    return {r: base_events(r, p) for r in ROBOTS}


fails = 0


def run(parent_ev, child_ev, parent_kw=None, child_kw=None, env_extra=None,
        parent_cells=("s1", "s2"), child_cells=("s1", "s2")):
    root = tempfile.mkdtemp(prefix="equivcal_")
    try:
        p = build(os.path.join(root, "parent"), parent_cells, parent_ev,
                  **(parent_kw or {}))
        c = build(os.path.join(root, "child"), child_cells, child_ev,
                  **(child_kw or {}))
        env = dict(os.environ)
        env.pop("EQUIV_ALLOW_NEW_KINDS", None)   # never inherited: see below
        env.pop("EQUIV_ALLOW_NEW_FIELDS", None)  # same rule, same reason
        env.update(env_extra or {})
        r = subprocess.run([sys.executable, GATE, p, c], capture_output=True,
                           text=True, env=env)
        return r.returncode, r.stdout + r.stderr
    finally:
        shutil.rmtree(root, ignore_errors=True)


def case(label, expect_rc, expect_pat=None, **kw):
    """Assert a return code AND, for verdicts, the reason given for it."""
    global fails
    rc, out = run(kw.pop("parent_ev", None) or parent_side(),
                  kw.pop("child_ev", None) or child_side(), **kw)
    ok = rc == expect_rc
    why = "" if ok else f" (rc={rc}, want {expect_rc})"
    if ok and expect_pat:
        ok = re.search(expect_pat, out) is not None
        if not ok:
            why = f" (rc ok, but no message matching {expect_pat!r})"
    print(f"  {'PASS' if ok else 'FAIL'}  {label}{why}")
    if not ok:
        fails += 1
        print("    " + "\n    ".join(out.splitlines()[:16]))


def mutate(side, fn):
    """Deep-copy a side and apply fn to each robot's event list."""
    out = copy.deepcopy(side)
    for robot, evs in out.items():
        fn(evs, robot)
    return out


def set_param(side, **kv):
    def f(evs, robot):
        evs[0]["params"].update(kv)
    return mutate(side, f)


def drop_param(side, name):
    def f(evs, robot):
        evs[0]["params"].pop(name, None)
    return mutate(side, f)


# ---------------------------------------------------------------------------

print("=== the A/A direction: a same-binary pair must PASS ===")

case("identical parent and child", 0, r"EQUIVALENT")
case("the real P0 child: three new params at their compiled defaults", 0,
     r"3 new param\(s\) at defaults")
# Run-to-run variation must not fail the gate, or it will be switched off.
case("a legacy kind the child happened not to reach",
     0, r"kind\(s\) in the parent and not the child",
     child_ev=mutate(child_side(),
                     lambda evs, r: evs.pop(3)))       # exploration_complete
case("a legacy kind the parent happened not to reach", 0, r"EQUIVALENT",
     parent_ev=mutate(parent_side(), lambda evs, r: evs.pop(3)))
case("volatile params differing (git_rev, build_stamp, output_csv)", 0,
     r"EQUIVALENT",
     child_ev=set_param(child_side(), git_rev="feeb74b",
                        build_stamp="Aug 30 2026 09:00:00",
                        output_csv="/tmp/child/planner_atlas.csv"))

print("\n=== the injection the plan names: a new event kind at defaults ===")

for kind in ("cell_census", "allocation", "rendezvous_agreed",
             "rendezvous_outcome", "reconnect_gate"):
    def add(evs, robot, kind=kind):
        evs.insert(2, {"event": kind, "seq": 99, "robot": robot,
                       "t_sim_sec": 40.0, "t_rel_sec": 13.8,
                       "t_wall_sec": 1787851740.0, "state": "EXPLORE",
                       "step": 2})
    case(f"child emits {kind} at defaults", 1,
         rf"opt-in event kind\(s\) \['{kind}'\]",
         child_ev=mutate(child_side(), add))

case("the same kind is accepted when EQUIV_ALLOW_NEW_KINDS declares it", 0,
     r"permitted new kind\(s\).*NOT a defaults run",
     child_ev=mutate(child_side(), lambda evs, r: evs.insert(
         2, {"event": "cell_census", "seq": 99, "robot": r, "t_sim_sec": 40.0,
             "t_rel_sec": 13.8, "t_wall_sec": 1787851740.0,
             "state": "EXPLORE", "step": 2})),
     env_extra={"EQUIV_ALLOW_NEW_KINDS": "cell_census"})

case("an event kind experiment_log.hpp does not declare at all", 1,
     r"does not declare at all",
     child_ev=mutate(child_side(), lambda evs, r: evs.insert(
         2, {"event": "brand_new_thing", "seq": 99, "robot": r,
             "t_sim_sec": 40.0, "t_rel_sec": 13.8,
             "t_wall_sec": 1787851740.0, "state": "EXPLORE", "step": 2})))

print("\n=== check 3b: the kind names are unchanged, the contents are not ===")

# Every case here keeps the event VOCABULARY identical, so check 3 passes each
# one. That is the point: before 3b existed, a child that rewrote what is inside
# its events was indistinguishable from an identical binary.

case("a field added to a kind both sides emit", 1,
     r"child writes field\(s\) \['state_change\.reason'\]",
     child_ev=mutate(child_side(),
                     lambda evs, r: evs[2].update(reason="peer_lost")))

case("...permitted when EQUIV_ALLOW_NEW_FIELDS declares it", 0,
     r"permitted new field\(s\).*NOT a defaults run",
     child_ev=mutate(child_side(),
                     lambda evs, r: evs[2].update(reason="peer_lost")),
     env_extra={"EQUIV_ALLOW_NEW_FIELDS": "state_change.reason"})

case("...and the permission does not license a DIFFERENT field", 1,
     r"child writes field\(s\) \['state_change\.cause'\]",
     child_ev=mutate(child_side(),
                     lambda evs, r: evs[2].update(cause="peer_lost")),
     env_extra={"EQUIV_ALLOW_NEW_FIELDS": "state_change.reason"})

case("a field that changed JSON type", 1,
     r"changed JSON type.*run_end\.reason \(str -> int\)",
     child_ev=mutate(child_side(), lambda evs, r: evs[4].update(reason=3)))

case("a field the child stopped writing is a NOTE, not a failure", 0,
     r"field\(s\) the parent wrote and the child did not: \['run_end\.reason'\]",
     child_ev=mutate(child_side(), lambda evs, r: evs[4].pop("reason", None)))

# The two ways an honest same-binary pair can look asymmetric. Both must pass,
# or the gate fails real pairs and gets switched off — the failure mode this
# file exists to prevent.
case("a field on ONE side's ONE run only, present in the union of both", 0,
     r"EQUIVALENT",
     parent_ev=mutate(parent_side(),
                      lambda evs, r: evs[2].update(note="x") if r == ROBOTS[0]
                      else None),
     child_ev=mutate(child_side(),
                     lambda evs, r: evs[2].update(note="y") if r == ROBOTS[1]
                     else None))
case("a field only ever null on one side is not a type disagreement", 0,
     r"EQUIVALENT",
     parent_ev=mutate(parent_side(), lambda evs, r: evs[4].update(extra=None)),
     child_ev=mutate(child_side(), lambda evs, r: evs[4].update(extra="v")))

print("\n=== the param dump ===")

case("a new param NOT at its compiled default", 1,
     r"new param 'team_robot_names' is NOT at its default",
     child_ev=set_param(child_side(), team_robot_names="atlas,bestla"))
case("a new numeric param off its derived default", 1,
     r"new param 'robot_id' is NOT at its default",
     child_ev=set_param(child_side(), robot_id=0.0))
# P2's own new param, both ways round. The default is read from the live
# dp("team_world_hz", 0.0) in explo_planner_node.cpp, so this pair also pins
# that the switch's shipped value really is off: move the source default to
# anything non-zero and the first of these two starts failing.
case("the new team_world_hz at its compiled default", 0,
     r"new param\(s\) at defaults:.*team_world_hz",
     child_ev=set_param(child_side(), team_world_hz=0.0))
case("team_world_hz dumped with the exchange running", 1,
     r"new param 'team_world_hz' is NOT at its default",
     child_ev=set_param(child_side(), team_world_hz=1.0))
# P6's knob, both ways round. Its std::string default also pins that _literal()
# parses that initialiser: if it regresses, the first case fails on a
# parser-refusal message. (notes: calib-pursuit-predictor-string)
case("the new pursuit_predictor at its compiled default", 0,
     r"new param\(s\) at defaults:.*pursuit_predictor",
     child_ev=set_param(child_side(), pursuit_predictor="trail"))
case("pursuit_predictor dumped as mdp", 1,
     r"new param 'pursuit_predictor' is NOT at its default",
     child_ev=set_param(child_side(), pursuit_predictor="mdp"))
case("a param the parent had and the child dropped", 1,
     r"'done_criterion' present in the parent and gone in the child",
     child_ev=drop_param(child_side(), "done_criterion"))
case("a shared param silently re-valued", 1,
     r"'done_unknown_fraction' re-valued",
     child_ev=set_param(child_side(), done_unknown_fraction=0.5))
case("a shared BOOLEAN re-valued (not just numbers)", 1,
     r"'coordination_enabled' re-valued",
     child_ev=set_param(child_side(), coordination_enabled=False))
case("a shared STRING re-valued", 1, r"'arm' re-valued",
     child_ev=set_param(child_side(), arm="off"))
# A True/1.0 confusion would make every boolean check vacuous.
case("True is not 1.0", 1, r"'coordination_enabled' re-valued",
     child_ev=set_param(child_side(), coordination_enabled=1.0))

print("\n=== a side must be ONE configuration ===")


# These build their two cells separately, because the disagreement being tested
# is BETWEEN cells and run()'s helpers vary things per robot.
def mixed_case(label, expect_rc, expect_pat, cell2_params=None,
               cell2_schema=None):
    global fails
    root = tempfile.mkdtemp(prefix="equivcal_")
    try:
        p = build(os.path.join(root, "parent"), ("s1", "s2"), parent_side())
        c = os.path.join(root, "child")
        build(c, ("s1",), child_side())
        alt = dict(BASE_PARAMS, **P0_NEW_PARAMS)
        alt.update(cell2_params or {})
        ev = {r: base_events(r, alt) for r in ROBOTS}
        if cell2_schema is not None:
            for evs in ev.values():
                evs[0]["schema_version"] = cell2_schema
        build(c, ("s2",), ev)
        env = dict(os.environ)
        env.pop("EQUIV_ALLOW_NEW_KINDS", None)
        r = subprocess.run([sys.executable, GATE, p, c], capture_output=True,
                           text=True, env=env)
        out = r.stdout + r.stderr
        ok = r.returncode == expect_rc and re.search(expect_pat, out)
        print(f"  {'PASS' if ok else 'FAIL'}  {label} (rc={r.returncode}, "
              f"want {expect_rc})")
        if not ok:
            fails += 1
            print("    " + "\n    ".join(out.splitlines()[:16]))
    finally:
        shutil.rmtree(root, ignore_errors=True)


mixed_case("one child cell configured differently from the others", 1,
           r"child: param done_unknown_fraction differs between runs",
           cell2_params={"done_unknown_fraction": 0.5})
mixed_case("one child cell written at a different schema", 1,
           r"schema_version is not constant within the side",
           cell2_schema=3)

print("\n=== schema, gates, and the manifest ===")

case("child schema older than the parent's", 1,
     r"schema_version went backwards: child 3 < parent 4",
     child_ev=mutate(child_side(),
                     lambda evs, r: evs[0].update(schema_version=3)))
case("a schema bump is reported, not failed", 0, r"schema_version parent 4 -> "
     r"child 5",
     child_ev=mutate(child_side(),
                     lambda evs, r: evs[0].update(schema_version=5)))
case("a different set of comms gates armed", 1,
     r"different comms gates armed",
     child_kw={"gates": GATES + "PASS\toutage\tlink outage(s) observed\n"})
case("the arm configuration differs in the manifest", 1,
     r"manifest comms: parent '1' != child '0'",
     child_kw={"manifest": MANIFEST.replace("comms=1", "comms=0")})
case("per-run manifest keys differing is NOT a failure", 0, r"EQUIVALENT",
     child_kw={"manifest": MANIFEST.replace("seed=1", "seed=7")
               .replace("run_end_t_sim=812.4", "run_end_t_sim=413.0")
               .replace("git_explo_planner=3d4306c",
                        "git_explo_planner=feeb74b")})
case("an absent manifest is reported as NOT compared, not as agreement", 0,
     r"run_manifest.txt absent on at least one side; the arm configuration "
     r"was NOT compared", child_kw={"manifest": None})

print("\n=== manifest keys a phase ADDS ===")

# The P1 harness with CELL_WORLD unset writes these seven keys; a P0 parent
# writes none of them. (notes: calib-cell-block-off)
CELL_BLOCK_OFF = ("cell_world=0\n"
                  "cell_size_m=10.0\n"
                  "cell_census_period_s=5.0\n"
                  "cell_covered_max_unknown=0.55\n"
                  "cell_exploring_min_unknown=0.62\n"
                  "cell_covered_max_frontier_frac=0.95\n"
                  'team_robot_names=["atlas","bestla"]\n')

case("the real P1 child: a whole knob block recorded with its switch off", 0,
     r"7 new manifest key\(s\) recording a subsystem that is OFF:.*cell_world",
     child_kw={"manifest": MANIFEST + CELL_BLOCK_OFF})
# The relaxation must not survive the switch being on. This is the injection
# the whole group exists to still catch.
case("the same block with its switch ON", 1,
     r"manifest cell_world: new in the child at '1'.*treatment arm",
     child_kw={"manifest": MANIFEST
               + CELL_BLOCK_OFF.replace("cell_world=0", "cell_world=1")})
# A dependent key with no switch beside it is not shown to be inert by
# anything. This is the shape a half-written manifest block takes.
case("gated keys present but the switch itself missing", 1,
     r"manifest cell_size_m: .*gated by cell_world, but the child's "
     r"cell_world is '<absent>'",
     child_kw={"manifest": MANIFEST
               + CELL_BLOCK_OFF.replace("cell_world=0\n", "")})
case("a new manifest key belonging to no declared group", 1,
     r"manifest brand_new_knob: .*not declared in GATED_MANIFEST_GROUPS",
     child_kw={"manifest": MANIFEST + "brand_new_knob=3\n"})
# Relaxed in one direction only: a key the parent described the arm with and
# the child stopped writing is a hole in the record, not agreement.
case("a manifest key the parent had and the child dropped", 1,
     r"manifest cell_world: the parent recorded '0' and the child does not "
     r"record it at all",
     parent_kw={"manifest": MANIFEST + CELL_BLOCK_OFF})
# And the excuse does not extend to keys BOTH sides write: once the parent has
# the block too, the ordinary comparison is back in force.
case("both sides carry the block and a gated knob is re-valued", 1,
     r"manifest cell_covered_max_unknown: parent '0.55' != child '0.70'",
     parent_kw={"manifest": MANIFEST + CELL_BLOCK_OFF},
     child_kw={"manifest": MANIFEST + CELL_BLOCK_OFF.replace(
         "cell_covered_max_unknown=0.55", "cell_covered_max_unknown=0.70")})

# Cell block on both sides, team block on the child only. team_world_hz reads
# 1.0 with team_world=0 (launcher default, never passed to the node); do not
# zero it in the manifest to fix a failure here. (notes: calib-team-block-off)
TEAM_BLOCK_OFF = ("team_world=0\n"
                  "team_world_hz=1.0\n")

case("the real P2 child: the team block recorded with its switch off", 0,
     r"2 new manifest key\(s\) recording a subsystem that is OFF:.*team_world",
     parent_kw={"manifest": MANIFEST + CELL_BLOCK_OFF},
     child_kw={"manifest": MANIFEST + CELL_BLOCK_OFF + TEAM_BLOCK_OFF})
case("the team block with its switch ON", 1,
     r"manifest team_world: new in the child at '1'.*treatment arm",
     parent_kw={"manifest": MANIFEST + CELL_BLOCK_OFF},
     child_kw={"manifest": MANIFEST + CELL_BLOCK_OFF
               + TEAM_BLOCK_OFF.replace("team_world=0", "team_world=1")})
case("the team rate present but its switch missing", 1,
     r"manifest team_world_hz: .*gated by team_world, but the child's "
     r"team_world is '<absent>'",
     parent_kw={"manifest": MANIFEST + CELL_BLOCK_OFF},
     child_kw={"manifest": MANIFEST + CELL_BLOCK_OFF
               + TEAM_BLOCK_OFF.replace("team_world=0\n", "")})

# Four gate keys with no dependents, so no switch-missing case. Two are off at a
# word, not 0 (reconnect_gate at silence, pursuit_predictor at trail); both
# directions are pinned per key. (notes: calib-p3-p6-stack)
STACK_BLOCK_OFF = ("global_alloc=0\n"
                   "reconnect_gate=silence\n"
                   "rendezvous_schedule=0\n"
                   "pursuit_predictor=trail\n")
BELOW = MANIFEST + CELL_BLOCK_OFF + TEAM_BLOCK_OFF

case("the P3-P6 stack recorded with all four switches off", 0,
     r"4 new manifest key\(s\) recording a subsystem that is OFF:.*"
     r"global_alloc.*pursuit_predictor",
     parent_kw={"manifest": BELOW},
     child_kw={"manifest": BELOW + STACK_BLOCK_OFF})
for key, on, off in (("global_alloc", "1", "0"),
                     ("reconnect_gate", "info", "silence"),
                     ("rendezvous_schedule", "1", "0"),
                     ("pursuit_predictor", "mdp", "trail")):
    case(f"{key} recorded ON is a treatment arm, not a defaults run", 1,
         rf"manifest {key}: new in the child at '{on}'.*treatment arm",
         parent_kw={"manifest": BELOW},
         child_kw={"manifest": BELOW + STACK_BLOCK_OFF.replace(
             f"{key}={off}", f"{key}={on}")})

# P7 writes the roster into the manifest. It is not a switch and has no off
# value; it restates the scenario, which both sides record and which the gate
# already compares. So the relaxation is conditional on the source key agreeing
# — and the two cases that matter are the ones where it does not.
ROSTER = ("robots=atlas,bestla\n"
          "n_robots=2\n")
STACKED = BELOW + STACK_BLOCK_OFF

case("the P7 roster, derived from a scenario both sides record", 0,
     r"2 new manifest key\(s\) restating a key both sides record: "
     r"n_robots, robots",
     parent_kw={"manifest": STACKED},
     child_kw={"manifest": STACKED + ROSTER})
case("the roster with no scenario on either side to derive it from", 1,
     r"manifest n_robots: .*derived from scenario, but scenario is '<absent>' "
     r"on the parent",
     parent_kw={"manifest": STACKED.replace(
         "scenario=flatforest_dense_2robot_lidar.yaml\n", "")},
     child_kw={"manifest": STACKED.replace(
         "scenario=flatforest_dense_2robot_lidar.yaml\n", "") + ROSTER})
# The one that would matter in practice: a three-robot child compared against a
# two-robot parent. The scenario mismatch fails on its own line, but the roster
# must not be waved through beside it — a reader who sees only "scenario
# differs" can talk themselves into "same code, bigger world".
case("a roster whose scenario changed under it", 1,
     r"manifest n_robots: .*derived from scenario, but scenario is "
     r"'flatforest_dense_2robot_lidar.yaml' on the parent and "
     r"'flatforest_3robot_lidar.yaml' on the child",
     parent_kw={"manifest": STACKED},
     child_kw={"manifest": STACKED.replace(
         "scenario=flatforest_dense_2robot_lidar.yaml",
         "scenario=flatforest_3robot_lidar.yaml")
         + "robots=atlas,bestla,husky\nn_robots=3\n"})

# reconnect_enabled restates rendezvous_enabled (the harness writes both from
# one variable). When the sides ran different arms, the alias must fail on its
# own line, not ride on the legacy key's failure.
# (notes: calib-reconnect-rename)
case("the renamed reconnect switch, restating a key both sides record", 0,
     r"1 new manifest key\(s\) restating a key both sides record: "
     r"reconnect_enabled",
     parent_kw={"manifest": STACKED},
     child_kw={"manifest": STACKED + "reconnect_enabled=true\n"})
case("the renamed switch where the two sides ran different arms", 1,
     r"manifest reconnect_enabled: .*derived from rendezvous_enabled, but "
     r"rendezvous_enabled is 'true' on the parent and 'false' on the child",
     parent_kw={"manifest": STACKED},
     child_kw={"manifest": STACKED.replace("rendezvous_enabled=true",
                                           "rendezvous_enabled=false")
               + "reconnect_enabled=false\n"})
# Catches run_explo_sim_rviz.sh no longer writing both spellings from one
# variable; without the gate's intra-manifest alias check this pair would read
# EQUIVALENT. (notes: calib-alias-intra-manifest)
case("a child whose two spellings of the switch disagree", 1,
     r"manifest reconnect_enabled: declared a rename of rendezvous_enabled, "
     r"but the child writes reconnect_enabled='true' and "
     r"rendezvous_enabled='false' in the SAME manifest",
     parent_kw={"manifest": STACKED.replace("rendezvous_enabled=true",
                                            "rendezvous_enabled=false")},
     child_kw={"manifest": STACKED.replace("rendezvous_enabled=true",
                                           "rendezvous_enabled=false")
               + "reconnect_enabled=true\n"})

print("\n=== silence must never read as a pass ===")

rc, out = run(parent_side(), child_side(), child_cells=())
ok = rc == 3 and "UNRESOLVED" in out and "not a pass" in out
print(f"  {'PASS' if ok else 'FAIL'}  an empty child side is UNRESOLVED "
      f"(rc={rc}, want 3)")
if not ok:
    fails += 1
    print("    " + "\n    ".join(out.splitlines()[:16]))

rc, out = run(parent_side(), child_side(), parent_cells=())
ok = rc == 3 and "UNRESOLVED" in out
print(f"  {'PASS' if ok else 'FAIL'}  an empty parent side is UNRESOLVED "
      f"(rc={rc}, want 3)")
if not ok:
    fails += 1
    print("    " + "\n    ".join(out.splitlines()[:16]))

case("a robot-run with no run_start refuses instead of comparing nothing", 2,
     r"has no run_start",
     child_ev=mutate(child_side(), lambda evs, r: evs.pop(0)))

print("\n=== the gate must refuse rather than guess ===")

# Gen 34's manifest says node=gen34. The gate reads gen 33's node source and
# vocabulary, so either side run by another node is refused before anything is
# compared, and the refusal names the checker that does read it.
case("a gen-34 child is refused", 2,
     r"child: s1 was run by node=gen34.*gen34_check\.py",
     child_kw={"manifest": MANIFEST + "node=gen34\n"})
case("a gen-34 parent is refused", 2,
     r"parent: s1 was run by node=gen34.*gen34_check\.py",
     parent_kw={"manifest": MANIFEST + "node=gen34\n"})
case("a manifest that names gen33 is scored", 0, r"EQUIVALENT",
     parent_kw={"manifest": MANIFEST + "node=gen33\n"},
     child_kw={"manifest": MANIFEST + "node=gen33\n"})

case("a new param with no dp() call and no derived entry", 1,
     r"no dp\(\"invented_knob\", \.\.\.\) call was found",
     child_ev=set_param(child_side(), invented_knob=1.0))

# A default the parser cannot read is exit 2 — "I could not tell", which is a
# different answer from "they differ". coverage_milestones is a real param with
# a real std::vector<double>{...} default, so this exercises the actual refusal
# path rather than a synthetic one.
case("a new param whose compiled default the parser will not guess at", 2,
     r"cannot parse the compiled default of new param 'coverage_milestones'",
     parent_ev=parent_side(),
     child_ev=set_param(child_side(), coverage_milestones=[0.9, 0.85]))

root = tempfile.mkdtemp(prefix="equivcal_")
try:
    junk = os.path.join(root, "junk.hpp")
    open(junk, "w").write("// no vocabulary here\n")
    rc, out = run(parent_side(), child_side(),
                  env_extra={"EQUIV_LOG_HPP": junk})
    ok = rc == 2 and "no kEventKinds[] initialiser found" in out
    print(f"  {'PASS' if ok else 'FAIL'}  an unreadable event vocabulary "
          f"refuses to score (rc={rc}, want 2)")
    if not ok:
        fails += 1
        print("    " + "\n    ".join(out.splitlines()[:16]))

    bad = os.path.join(root, "bad.hpp")
    open(bad, "w").write(
        'static constexpr const char* kEventKinds[] = {"run_start", "step"};\n'
        'static constexpr size_t kFirstV4EventKind = 9;\n')
    rc, out = run(parent_side(), child_side(), env_extra={"EQUIV_LOG_HPP": bad})
    ok = rc == 2 and "the boundary and the list disagree" in out
    print(f"  {'PASS' if ok else 'FAIL'}  a boundary index past the end of the "
          f"list refuses (rc={rc}, want 2)")
    if not ok:
        fails += 1
        print("    " + "\n    ".join(out.splitlines()[:16]))

    src = os.path.join(root, "empty.cpp")
    open(src, "w").write("int main() { return 0; }\n")
    rc, out = run(parent_side(), child_side(), env_extra={"EQUIV_NODE_SRC": src})
    ok = rc == 2 and "this gate went blind" in out
    print(f"  {'PASS' if ok else 'FAIL'}  a source with no dp() block refuses "
          f"instead of finding every param new (rc={rc}, want 2)")
    if not ok:
        fails += 1
        print("    " + "\n    ".join(out.splitlines()[:16]))
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== the fixture must match the BINARY, not the gate's beliefs ===")


# ---------------------------------------------------------------------------

print("\n=== D1: derived and auto-sentinel params ===")

# A 360 deg comb: 2*pi over 96 rays, one h_step of slack. The node's own
# formula, so the derived flag must come out true and the pair must be clean.
# fov_hfov is declared through dp_f, which narrows 6.28318 to float, and the
# run logs that float widened back: the value a real dump carries.
OMNI = {"fov_hfov": 6.283180236816406, "fov_h_rays": 96,
        "fov_is_omnidirectional": True}

case("a 360 deg FOV with a flag matching its own geometry is clean",
     0, r"new param\(s\) at defaults",
     child_ev=set_param(child_side(), **OMNI))

# The whole reason the entry is a function of the run's params rather than a
# pinned True: this is a binary whose logged flag contradicts the comb it
# logged beside it, and no constant expectation could catch it.
case("a flag contradicting the same run's FOV geometry is a difference",
     1, r"fov_is_omnidirectional.*NOT at its default",
     child_ev=set_param(child_side(),
                        **dict(OMNI, fov_is_omnidirectional=False)))

# A directional FOV child fails, but on fov_hfov itself, never on the derived
# flag. Asserted by reading the FAIL lines, since what is checked is the absence
# of a line. (notes: calib-directional-fov-child)
rc, out = run(parent_side(),
              set_param(child_side(), fov_hfov=1.047, fov_h_rays=16,
                        fov_is_omnidirectional=False))
_bad = [l for l in out.splitlines()
        if "FAIL" in l and "fov_is_omnidirectional" in l]
_ok = rc == 1 and "fov_hfov" in out and not _bad
print(f"  {'PASS' if _ok else 'FAIL'}  a directional child fails on the FOV "
      f"itself, not on the derived flag (rc={rc})")
if not _ok:
    fails += 1
    print("    " + "\n    ".join(out.splitlines()[:16]))

# Derivable only if the run logged what it derives from. A child that records
# the verdict and not the geometry cannot be checked, and "cannot be checked"
# is not "checked and fine".
case("the flag without the geometry it derives from is UNRESOLVED",
     3, r"did not log the params it derives from",
     child_ev=set_param(child_side(), fov_is_omnidirectional=True))

# The AUTO sentinel. coord_claim_radius_m has a dp() call, so the lookup
# SUCCEEDS and returns 0.0 -- which no run ever logs, because the node resolves
# it in the constructor before the dump is written. Testing against it would
# fail every honest defaults run, so this must be UNRESOLVED, not a failure...
case("an AUTO-sentinel param is UNRESOLVED, not a false difference",
     3, r"coord_claim_radius_m.*AUTO sentinel, not a default",
     child_ev=set_param(child_side(), coord_claim_radius_m=10.0))

# The declined AUTO-sentinel param must not be counted as checked: the
# at-defaults note still prints (P0's three new params) but must not name
# coord_claim_radius_m. (notes: calib-auto-sentinel-not-counted)
rc, out = run(parent_side(),
              set_param(child_side(), coord_claim_radius_m=10.0))
_note = [l for l in out.splitlines() if "new param(s) at defaults" in l]
_ok = (rc == 3 and len(_note) == 1
       and "coord_claim_radius_m" not in _note[0]
       and "robot_id" in _note[0])
print(f"  {'PASS' if _ok else 'FAIL'}  ...and is not counted among the params "
      f"checked (rc={rc})")
if not _ok:
    fails += 1
    print("    " + "\n    ".join(out.splitlines()[:16]))

case("the other two AUTO-sentinel knobs are registered too",
     3, r"cost_grid_radius_cap_m.*AUTO sentinel",
     child_ev=set_param(child_side(), cost_grid_radius_cap_m=500.0,
                        coord_vantage_claim_radius_m=0.75))


print("\n=== the params-file witnesses (previously uncalibrated) ===")

# These three rungs of GATED_MANIFEST_GROUPS had no case at all, which is how
# coord_claim_radius_m_in_params sat pinned at the pre-D1 0.0 without anything
# going red. Each off value restates the SHIPPED yaml, so each is a claim about
# a file that changes -- exactly the kind of pin that needs a test.
def witness_case(label, expect_rc, expect_pat, line):
    case(label, expect_rc, expect_pat,
         child_kw={"manifest": MANIFEST + line + "\n"})

witness_case("the claim radius the shipped yaml now pins is clean", 0,
             r"new manifest key\(s\) recording a subsystem that is OFF",
             "coord_claim_radius_m_in_params=10.0")
# The case that would have caught the stale pin: pre-D1 the yaml said 0.0, and
# after D1 a child still reading 0.0 is a child built against a stale installed
# params file -- the very defect these witnesses exist to expose.
witness_case("a child still reading the pre-D1 0.0 is a difference", 1,
             r"coord_claim_radius_m_in_params.*declared off value is '10\.0'",
             "coord_claim_radius_m_in_params=0.0")
witness_case("a missing params file fails rather than relaxing", 1,
             r"coord_claim_radius_m_in_params.*declared off value",
             "coord_claim_radius_m_in_params=missing")
witness_case("the comms-mask witness at its shipped false is clean", 0,
             r"new manifest key\(s\) recording a subsystem that is OFF",
             "global_alloc_comms_mask_in_params=false")
witness_case("...and reading true is a treatment arm", 1,
             r"global_alloc_comms_mask_in_params.*switch\s+is ON",
             "global_alloc_comms_mask_in_params=true")


def audit_fixture_against_real_cell():
    """Check the fixture's run_start shape against a banked cell.

    Reports UNRESOLVED — never PASS — when no banked cell is reachable. The
    fixture above is what every case is built on; if its shape is wrong, every
    case tests the wrong thing and they all still print PASS.

    AND UNRESOLVED COUNTS AGAINST THE EXIT STATUS, which it did not until
    2026-09-18. Both refusals below printed their line and `return`ed without
    touching `fails`, so a run that compared the fixture against nothing at all
    exited 0 — indistinguishable, to anything reading the status, from a run
    where the audit passed. That is the whole point of the audit inverted: it
    exists because a wrong fixture makes every other case agree with itself,
    and an UNRESOLVED audit is precisely the state in which nobody knows
    whether the fixture is wrong. `checks-that-stopped-checking` again, and the
    same edit was needed in gate_g8_calib.py for the same reason.
    """
    global fails
    root = os.environ.get("EQUIV_CALIB_REAL_ROOT", "/home/kalhan/hmr_campaign")
    if not os.path.isdir(root):
        print(f"  UNRESOLVED  no campaign root at {root}; fixture shapes were "
              f"NOT compared against real data, so nothing here establishes "
              f"that the other cases are built on a real schema")
        fails += 1
        return
    # Scans depth 1 and 2 under the root (the campaign/cell layout) and keeps
    # the run_start with the newest schema; deeper walks only cross bag
    # directories. (notes: calib-audit-scan-depth)
    real_start, real_params, real_schema, real_where = None, None, -1, None
    cand = [root] + [os.path.join(root, c) for c in sorted(os.listdir(root))
                     if os.path.isdir(os.path.join(root, c))]
    for parent in cand:
        try:
            kids = sorted(os.listdir(parent))
        except OSError:
            continue
        for cell in kids:
            d = os.path.join(parent, cell)
            if not os.path.isdir(d):
                continue
            # The gate scores gen 33's cells only, so the fixture is audited
            # against those: a banked gen-34 cell has another run_start.
            node = "gen33"
            try:
                for ln in open(os.path.join(d, "run_manifest.txt"),
                               errors="replace"):
                    if ln.startswith("node="):
                        node = ln.strip().partition("=")[2]
                        break
            except OSError:
                pass
            if node != "gen33":
                continue
            for r in ROBOTS:
                p = os.path.join(d, f"{r}.events.jsonl")
                if not os.path.exists(p):
                    continue
                for ln in open(p, errors="replace"):
                    try:
                        e = json.loads(ln)
                    except ValueError:
                        continue
                    if e.get("event") == "run_start":
                        if e.get("schema_version", 0) > real_schema:
                            real_start = set(e.keys())
                            real_params = set(e.get("params", {}))
                            real_schema = e.get("schema_version")
                            real_where = os.path.relpath(d, root)
                        break
    if not real_start:
        print(f"  UNRESOLVED  no run_start found under {root} for any of "
              f"{'/'.join(ROBOTS)}; fixture shapes were NOT compared against "
              f"real data. Note the roster is a constant here, so a campaign "
              f"whose robots are named otherwise lands on this line too")
        fails += 1
        return
    fixture_start = set(base_events("atlas", BASE_PARAMS)[0].keys())
    fixture_params = set(BASE_PARAMS)
    bad = []
    invented = fixture_start - real_start
    if invented:
        bad.append(f"run_start: fixture invents top-level key(s) "
                   f"{sorted(invented)}")
    # Top-level run_start keys must equal the real cell's: invented or missing
    # keys fail. Params are a deliberate subset: invented params fail, missing
    # ones are only counted. (notes: calib-audit-both-directions)
    missing = real_start - fixture_start
    if missing:
        bad.append(f"run_start: a real schema-{real_schema} cell has top-level "
                   f"key(s) {sorted(missing)} that the fixture does not — the "
                   f"fixture is behind the writer, so every case above is "
                   f"built on a run_start shape that is no longer emitted")
    invented_p = fixture_params - real_params
    if invented_p:
        bad.append(f"run_start.params: fixture invents {sorted(invented_p)}, "
                   f"which no writer emits")
    print(f"  {'PASS' if not bad else 'FAIL'}  fixture run_start matches a real "
          f"schema-{real_schema} cell's top-level shape "
          f"({len(fixture_start)} key(s)); params are a deliberate "
          f"{len(fixture_params)}-of-{len(real_params)} subset")
    print(f"           | audited against {real_where}")
    if real_schema != SCHEMA:
        # Not a failure: SCHEMA is pinned at 4 because several cases below
        # exercise a 4 -> 5 bump, and bumping it here would silently retarget
        # them. Printed so the gap is visible rather than assumed.
        print(f"           | fixture pins SCHEMA={SCHEMA}; the newest banked "
              f"cell is schema {real_schema}. That is intentional — the bump "
              f"cases need a fixture below current — but the shape checked "
              f"above is the schema-{real_schema} one.")
    for b in bad:
        print(f"           | {b}")
    if bad:
        fails += 1

    # The shape check cannot tell a stale cell from a current one, so the
    # reference's schema is compared to kSchemaVersion read from
    # experiment_log.hpp: one behind passes, two or more fails.
    # (notes: calib-audit-reference-age)
    hdr = os.path.join(HERE, os.pardir, "backup", "gen33",
                       "experiment_log.hpp")
    m = (re.search(r"kSchemaVersion\s*=\s*(\d+)", open(hdr, errors="replace")
                   .read()) if os.path.exists(hdr) else None)
    if not m:
        print(f"  UNRESOLVED  kSchemaVersion could not be read from {hdr}; the "
              f"audited reference's age could not be checked, so 'schema "
              f"{real_schema}' above is a number with nothing to compare to")
        fails += 1
        return
    truth = int(m.group(1))
    gap = truth - real_schema
    ok = gap <= 1
    print(f"  {'PASS' if ok else 'FAIL'}  the audited reference is within one "
          f"schema of the header (schema {real_schema} vs kSchemaVersion "
          f"{truth})")
    if not ok:
        print(f"           | {gap} generations behind. Either no campaign since "
              f"schema {real_schema} banked a cell where this scan looks "
              f"(depth 1 or 2 under {root}, robots {'/'.join(ROBOTS)}), or the "
              f"layout moved again. Nothing above failed, because the "
              f"run_start top-level shape is the same in both.")
        fails += 1


audit_fixture_against_real_cell()

print(f"\n{'ALL PASS' if fails == 0 else str(fails) + ' FAILURE(S)'}")
sys.exit(1 if fails else 0)
