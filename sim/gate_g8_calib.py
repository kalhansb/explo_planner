#!/usr/bin/python3
"""Known-answer calibration for gate_g8.py.

Builds a synthetic campaign cell that the gate must pass, then plants ONE
defect at a time and asserts the gate names the right check. A gate that has
only ever been run against data it passes is not known to be able to fail
([[checks-that-stopped-checking]]); six guards went inert that way.

Runs the REAL gate_g8.py, with only its FILL_ME identity substituted, so the
logic under test is the logic that will score the campaign.
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
GATE = os.path.join(HERE, "gate_g8.py")

REV = "3d4306c"
SHA = "aaaaaaaaaaaaaaaa"
ROBOTS = ["atlas", "bestla"]

MANIFEST = f"""run_end_reason=all_done
run_gates_verdict=CLEAN
run_end_t_sim=812.4
git_explo_planner={REV}
git_simple_nav_3d=c9f83a7
git_scovox=078d3f7
sha256_explo_planner_node={SHA}
"""

PARAMS = {
    "reconnect_mode_requested": "hybrid",
    "rendezvous_enabled": True,
    "goal_rotate_timeout_sec": 20.0,
    "nav_speed_estimate_mps": 0.5,
    "nav_safety_factor": 2.0,
    "nav_min_timeout_sec": 30.0,
    "nav_max_timeout_sec": 180.0,
    # The fixture had no done_action, so check 18c fired on the CLEAN cell the
    # moment it was added -- the fixture, not the gate, was wrong. Worth
    # recording: a check whose happy path was never represented in the
    # calibration would have looked like a real hard failure on the first live
    # campaign, and the temptation then is to delete the check.
    "done_action": "idle",
}


def base_events():
    """A clean robot's event stream, one of each row the new checks read."""
    return [
        {"event": "run_start", "git_rev": REV, "params": copy.deepcopy(PARAMS)},
        {"event": "exploration_complete", "reason": "coverage-latched",
         "unknown_fraction": 0.638},
        # nav_goal_failed: the inequality holds (elapsed exceeded the budget)
        {"event": "nav_goal_failed", "reason": "budget", "budget_sec": 60.0,
         "pose_stale": False, "test_name": "nav_elapsed_sec",
         "test_value": 61.2, "test_threshold": 60.0},
        # nav_goal_failed: the other direction (movement fell short)
        {"event": "nav_goal_failed", "reason": "no-progress", "budget_sec": 60.0,
         "pose_stale": False, "test_name": "window_progress_m",
         "test_value": 0.02, "test_threshold": 0.50},
        # a frozen fire at exactly zero movement — the canonical case that
        # forbids a numeric sentinel
        {"event": "home_watchdog", "kind": "frozen", "test_delta_m": 0.0,
         "test_threshold_m": 0.5},
        # a receding approach fire — the canonical negative
        {"event": "home_watchdog", "kind": "approach", "test_delta_m": -0.03,
         "test_threshold_m": 1.0},
        # a leg termination, which evaluates no inequality
        {"event": "home_watchdog", "kind": "escape-end"},
        {"event": "reconnect_end", "outcome": "gave_up"},
        {"event": "peer_lost", "team_incomplete_sec": 41.0},
        {"event": "peer_seen", "team_incomplete_sec": -1.0},
        {"event": "mission_complete", "result": "arrived"},
        {"event": "run_end", "metrics_timer_rows": 163},
    ]


PLANNER_LOG = """[INFO] Exploration complete [latch]: ROI unknown fraction 0.638 <= 0.640 (source=scovox) in state PLAN at t_sim=603.9 - 41 steps, 210.00 m traveled.
[INFO] Reconnect (mid-run): team incomplete 45s >= gate 40s (attempt 1/3)
[INFO] Reconnect manoeuvre ended after 28.5 s sim: gave_up (-> RETURN_HOME).
"""

CSV = ("step,selected_score,selected_utility\n"
       "0,1.25,1.25\n"
       "1,0.80,0.80\n")

NAV_LOG = "[INFO] global plan ok: 41 poses\n"


def build(root, tag, events_by_robot, planner_log=PLANNER_LOG, csv_text=CSV,
          manifest=MANIFEST):
    cell = f"{tag}_hybrid_seed1"
    d = os.path.join(root, cell)
    os.makedirs(d, exist_ok=True)
    open(os.path.join(d, "run_manifest.txt"), "w").write(manifest)
    for r in ROBOTS:
        with open(os.path.join(d, f"{r}.events.jsonl"), "w") as fh:
            for e in events_by_robot[r]:
                fh.write(json.dumps(e) + "\n")
        open(os.path.join(d, f"planner_{r}.log"), "w").write(planner_log)
        open(os.path.join(d, f"planner_{r}.csv"), "w").write(csv_text)
        open(os.path.join(d, f"nav_{r}.log"), "w").write(NAV_LOG)
    open(os.path.join(root, cell + ".console.log"), "w").write("clean\n")
    return d


def run_gate(root, tag):
    """Run the real gate, declaring the synthetic campaign's identity.

    This used to rewrite the gate's source to patch the two FILL_ME literals,
    which meant the calibration validated a COPY of the gate rather than the
    gate. Now that the identity is declared out of band it can be injected, so
    what runs here is byte-for-byte the file that will score the campaign.
    """
    env = dict(os.environ, GATE_ROOT=root,
               GATE_EXPECT_git_explo_planner=REV,
               GATE_EXPECT_sha256_explo_planner_node=SHA)
    p = subprocess.run([sys.executable, GATE, tag], capture_output=True,
                       text=True, env=env)
    return p.returncode, p.stdout + p.stderr


fails = 0


def case(label, mutate, expect_pat, expect_clean=False):
    """Plant one defect and assert the gate names the right check."""
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events() for r in ROBOTS}
        kw = mutate(ev) or {}
        build(root, "cal", ev, **kw)
        rc, out = run_gate(root, "cal")
        if expect_clean:
            ok = rc == 0 and "HARD FAILURES: none" in out
            detail = "clean" if ok else "gate objected to a clean cell"
        else:
            ok = rc == 1 and re.search(expect_pat, out) is not None
            detail = "caught" if ok else f"did NOT match /{expect_pat}/"
        print(f"  {'PASS' if ok else 'FAIL'}  {label}: {detail}")
        if not ok:
            fails += 1
            for ln in out.splitlines():
                if "check" in ln or "HARD" in ln:
                    print(f"           | {ln}")
    finally:
        shutil.rmtree(root, ignore_errors=True)


def drop(ev, event, field):
    for r in ROBOTS:
        for e in ev[r]:
            if e.get("event") == event and field in e:
                del e[field]


def find(ev, robot, event, **match):
    for e in ev[robot]:
        if e.get("event") == event and all(e.get(k) == v for k, v in match.items()):
            return e
    raise AssertionError(f"no {event} {match}")


print("=== the gate must PASS a clean cell (a gate that fails everything is "
      "as useless as one that passes everything) ===")
case("clean synthetic cell", lambda ev: None, None, expect_clean=True)

print("\n=== check 15: the selected_score/selected_utility alias ===")
case("alias diverged in the CSV", lambda ev: {"csv_text":
     "step,selected_score,selected_utility\n0,1.25,1.25\n1,0.80,0.79\n"},
     r"check 15 .* selected_score=0\.8 != selected_utility=0\.79")

print("\n=== check 16: the home_watchdog fired inequality ===")
case("fire row missing the tested delta",
     lambda ev: drop(ev, "home_watchdog", "test_delta_m"),
     r"check 16 .* missing test_delta_m")
case("fire row whose inequality does not hold",
     lambda ev: find(ev, "atlas", "home_watchdog", kind="frozen")
                .update(test_delta_m=3.69),
     r"check 16 .* is NOT < test_threshold_m")
case("escape-end row carrying a tested value",
     lambda ev: find(ev, "atlas", "home_watchdog", kind="escape-end")
                .update(test_delta_m=0.0),
     r"check 16 .* escape-end row carries test_delta_m")

print("\n=== check 17: the reconnect outcome, log vs event ===")
case("event outcome disagrees with the log",
     lambda ev: find(ev, "atlas", "reconnect_end").update(outcome="reconnected"),
     r"check 17 .* 'reconnected' != .*'gave_up'")
case("generation-7 line wording (no outcome stated)",
     lambda ev: {"planner_log": PLANNER_LOG.replace(
         "s sim: gave_up (->", "s sim (->")},
     r"check 17 .* generation-7 wording")

print("\n=== check 18: the nav_goal_failed inequality ===")
case("fired but the inequality is false",
     lambda ev: find(ev, "atlas", "nav_goal_failed", reason="budget")
                .update(test_value=12.0),
     r"check 18 .* nav_elapsed_sec=12\.0 is NOT > 60\.0")
case("row missing the tested fields",
     lambda ev: drop(ev, "nav_goal_failed", "test_name"),
     r"check 18 .* missing test_name")
def drop_nav_param(ev):
    # A statement, not a comprehension: `case` treats a mutator's return value
    # as kwargs for build(), so a comprehension's list would be splatted.
    for r in ROBOTS:
        for e in ev[r]:
            if e.get("event") == "run_start":
                e["params"].pop("nav_safety_factor")


case("run_start not echoing a nav timeout param", drop_nav_param,
     r"check 18b .* missing nav_safety_factor")


def shutdown_done_action(ev):
    # The failure this models is NOT someone setting done_action=shutdown on
    # purpose. It is shared_params.yaml failing to install, so the node falls
    # back to its own compiled default -- with every hash in the manifest still
    # matching, because the manifest hashes the source tree's copy.
    for r in ROBOTS:
        for e in ev[r]:
            if e.get("event") == "run_start":
                e["params"]["done_action"] = "shutdown"


case("done_action fell back to the node default", shutdown_done_action,
     r"check 18c .* done_action='shutdown'")

print("\n=== check 19: the schema migration reached the DATA ===")
case("peer_lost still carrying the removed field",
     lambda ev: find(ev, "atlas", "peer_lost").update(last_contact_age_sec=12.0),
     r"check 19 .* removed last_contact_age_sec")
case("run_end carrying the old metrics_rows",
     lambda ev: find(ev, "atlas", "run_end").update(metrics_rows=163),
     r"check 19 .* old metrics_rows")

print("\n=== check 20: an unrecognised mid-run line ===")
case("mid-run wording the parser does not know",
     lambda ev: {"planner_log": PLANNER_LOG.replace(
         "team incomplete 45s >= gate 40s", "partner unheard for 45s")},
     r"check 20 .* unrecognised mid-run line")

print("\n=== check 3: provenance ===")
case("cell built from a different binary",
     lambda ev: {"manifest": MANIFEST.replace(REV, "deadbee")},
     r"git_explo_planner=deadbee expected")

print("\n=== the empty-population property: silence must not read as a pass ===")


def strip_watchdogs(ev):
    for r in ROBOTS:
        ev[r][:] = [e for e in ev[r] if e.get("event") != "home_watchdog"]


root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events() for r in ROBOTS}
    strip_watchdogs(ev)
    build(root, "cal", ev)
    rc, out = run_gate(root, "cal")
    ok = (rc == 0 and "HARD FAILURES: none" in out
          and "EMPTY: check is UNRESOLVED" in out
          and "UNRESOLVED" in out)
    print(f"  {'PASS' if ok else 'FAIL'}  zero home_watchdog rows reported as "
          f"UNRESOLVED, not silently passed")
    if not ok:
        fails += 1
        print(out[-1500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== the identity FILE path, which is what the campaign actually uses ===")
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events() for r in ROBOTS}
    build(root, "cal", ev)
    # No GATE_EXPECT_* env at all: the gate must find the identity in
    # <ROOT>/<TAG>.identity.txt. Calibrated separately from the env path
    # because the env path is the calibration harness's own shortcut, and a
    # file-reading branch that only ever runs in production is a branch nobody
    # has tested.
    open(os.path.join(root, "cal.identity.txt"), "w").write(
        f"# declared at launch\ngit_explo_planner={REV}\n"
        f"sha256_explo_planner_node={SHA}\n")
    env = {k: v for k, v in os.environ.items()
           if not k.startswith("GATE_EXPECT_")}
    env["GATE_ROOT"] = root
    p = subprocess.run([sys.executable, GATE, "cal"], capture_output=True,
                       text=True, env=env)
    out = p.stdout + p.stderr
    ok = "REFUSING TO RUN" not in out and "check 3" not in out
    print(f"  {'PASS' if ok else 'FAIL'}  identity read from "
          f"<TAG>.identity.txt (rc={p.returncode})")
    if not ok:
        fails += 1
        print("    " + "\n    ".join(out.splitlines()[:12]))
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== the gate must refuse to run before its identity is declared ===")
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events() for r in ROBOTS}
    build(root, "cal", ev)
    p = subprocess.run([sys.executable, GATE, "cal"], capture_output=True,
                       text=True, env=dict(os.environ, GATE_ROOT=root))
    ok = p.returncode == 2 and "REFUSING TO RUN" in p.stdout
    print(f"  {'PASS' if ok else 'FAIL'}  unfilled FILL_ME identity refuses "
          f"(rc={p.returncode})")
    if not ok:
        fails += 1
finally:
    shutil.rmtree(root, ignore_errors=True)

print(f"\n{'ALL PASS' if fails == 0 else str(fails) + ' FAILURE(S)'}")
sys.exit(1 if fails else 0)
