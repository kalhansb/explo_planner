#!/usr/bin/env python3
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

# Copied from the P1 pair that first hit this: a run of the P1 harness with
# CELL_WORLD unset writes all seven of these, and the P0 parent writes none of
# them. Before GATED_MANIFEST_GROUPS the gate called that seven configuration
# differences, which is a verdict of "not equivalent" against a subsystem that
# was switched off.
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

# P2 adds a second block on top of P1's, so its equivalence pair has the cell
# block on BOTH sides and the team block on the child only. Note the recorded
# rate: TEAM_WORLD_HZ defaults to 1.0 in the launcher and the manifest records
# the harness variable, so "off" here reads as team_world=0 with a non-zero
# hz — the value that never reached the node, because the -p is passed only
# inside the TEAM_WORLD=1 branch. If this case ever starts failing on the hz,
# the fix is not to zero it in the manifest: block 2 of the gate is what proves
# the node ran at its compiled 0.0, and this block is what proves the harness
# knob was off. They are different facts and they are allowed to differ.
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


def audit_fixture_against_real_cell():
    """Check the fixture's run_start shape against a banked cell.

    Reports UNRESOLVED — never PASS — when no banked cell is reachable. The
    fixture above is what every case is built on; if its shape is wrong, every
    case tests the wrong thing and they all still print PASS.
    """
    global fails
    root = os.environ.get("EQUIV_CALIB_REAL_ROOT", "/home/kalhan/hmr_campaign")
    if not os.path.isdir(root):
        print(f"  UNRESOLVED  no campaign root at {root}; fixture shapes were "
              f"NOT compared against real data")
        return
    # Scan for the NEWEST schema present rather than stopping at the first cell
    # alphabetically. The fixture is written at the current schema, so auditing
    # it against the oldest banked cell in the directory would silently compare
    # it to a run_start that predates half the params it claims to copy.
    real_start, real_params, real_schema = None, None, -1
    for cell in sorted(os.listdir(root)):
        d = os.path.join(root, cell)
        if not os.path.isdir(d):
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
                    break
    if not real_start:
        print(f"  UNRESOLVED  no run_start found under {root}; fixture shapes "
              f"were NOT compared against real data")
        return
    fixture_start = set(base_events("atlas", BASE_PARAMS)[0].keys())
    fixture_params = set(BASE_PARAMS)
    bad = []
    invented = fixture_start - real_start
    if invented:
        bad.append(f"run_start: fixture invents top-level key(s) "
                   f"{sorted(invented)}")
    invented_p = fixture_params - real_params
    if invented_p:
        bad.append(f"run_start.params: fixture invents {sorted(invented_p)}, "
                   f"which no writer emits")
    print(f"  {'PASS' if not bad else 'FAIL'}  fixture run_start is a subset "
          f"of a real schema-{real_schema} cell's "
          f"({len(fixture_start)} key(s), {len(fixture_params)} param(s))")
    for b in bad:
        print(f"           | {b}")
    if bad:
        fails += 1


audit_fixture_against_real_cell()

print(f"\n{'ALL PASS' if fails == 0 else str(fails) + ' FAILURE(S)'}")
sys.exit(1 if fails else 0)
