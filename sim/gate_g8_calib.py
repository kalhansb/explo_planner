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

SCHEMA_VERSION = 3

# The field names below are taken from the C++ WRITERS, not from what the gate
# expects to find. That distinction is the whole reason this file was rewritten
# in round 4: the fixture used to put git_rev at the top level of run_start and
# team_incomplete_sec on peer_lost/peer_seen, matching two mistaken beliefs in
# gate_g8.py. The two errors cancelled inside the harness, so it printed ALL
# PASS while the gate was simultaneously unable to fail on a stale binary and
# unable to pass on any real cell (34 spurious hard failures on the first real
# one). A fixture that encodes the reader's assumptions tests nothing; see
# audit_fixture_against_real_cell() at the bottom, which now checks this
# mechanically against a banked cell whenever one is present.
#
#   run_start.params.git_rev        explo_planner_node.cpp:2520 (addParamStr)
#   run_start.schema_version        experiment_log.cpp:265, top level
#   peer_lost / peer_seen           experiment_log.cpp:377-398 -- peer,
#                                   silent_sec, peers_live, expected_peers,
#                                   and first_contact on peer_seen only
#   reconnect_dispatch              experiment_log.cpp:401-437 -- the ONLY
#                                   writer of team_incomplete_sec
PARAMS = {
    # The node writes the arm and the baked rev as params, so this is where the
    # gate must read them from.
    "arm": "hybrid",
    "git_rev": REV,
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
    # Generation 9, check 3f. These three are the whole treatment-reachability
    # argument: the clock the mid-run trigger arms on, whether the link veto was
    # wired up at all, and the debounce in front of it. g8r1 satisfied every
    # other field in this dict and still ran a treatment that could not fire, so
    # a fixture without them cannot exercise the check that exists to catch it.
    "reconnect_midrun_silence_sec": 90.0,
    "link_gate_configured": True,
    "reconnect_link_down_confirm_sec": 0.0,
}


# experiment_log.cpp begin() stamps these on EVERY row; run_start additionally
# carries schema_version, t0_sim_sec and coverage_milestones. Copied from a real
# banked line, not invented -- see audit_fixture_against_real_cell().
def _row(event, seq, **fields):
    r = {"event": event, "seq": seq, "robot": "atlas", "t_sim_sec": 100.0 + seq,
         "t_rel_sec": 100.0 + seq, "t_wall_sec": 90.0 + seq,
         "state": "NAVIGATE", "step": 1}
    r.update(fields)
    return r


def base_events(arm="hybrid", robot="atlas"):
    """A clean robot's event stream, one of each row the new checks read.

    `arm` is honoured because the off arm is not a cosmetic variation: check 3e
    asserts rendezvous_enabled tracks the directory's arm, and that branch had
    never been executed by anything before round 4 -- the fixture built a hybrid
    cell only, so the half of the check that guards against an off cell running
    the reconnect logic was as untested as the reconnect logic it guards.
    """
    params = copy.deepcopy(PARAMS)
    params["arm"] = arm
    params["rendezvous_enabled"] = (arm == "hybrid")
    ev = [
        _row("run_start", 0, state="IDLE", step=0, t0_sim_sec=0.0,
             schema_version=SCHEMA_VERSION, coverage_milestones=[],
             params=params),
        _row("exploration_complete", 1, reason="coverage-latched",
             unknown_fraction=0.638),
        # nav_goal_failed: the inequality holds (elapsed exceeded the budget)
        _row("nav_goal_failed", 2, reason="budget", budget_sec=60.0,
             pose_stale=False, test_name="nav_elapsed_sec",
             test_value=61.2, test_threshold=60.0),
        # nav_goal_failed: the other direction (movement fell short)
        _row("nav_goal_failed", 3, reason="no-progress", budget_sec=60.0,
             pose_stale=False, test_name="window_progress_m",
             test_value=0.02, test_threshold=0.50),
        # a frozen fire at exactly zero movement — the canonical case that
        # forbids a numeric sentinel
        _row("home_watchdog", 4, kind="frozen", test_delta_m=0.0,
             test_threshold_m=0.5),
        # a receding approach fire — the canonical negative
        _row("home_watchdog", 5, kind="approach", test_delta_m=-0.03,
             test_threshold_m=1.0),
        # a leg termination, which evaluates no inequality
        _row("home_watchdog", 6, kind="escape-end"),
        # logPeerLost / logPeerSeen, experiment_log.cpp:377-398. They carry
        # peer/silent_sec/peers_live/expected_peers and NOTHING else: the
        # fixture used to give them team_incomplete_sec, which no writer in the
        # tree emits, and gate check 19 was asserting it there.
        _row("peer_lost", 7, peer="bestla", silent_sec=5.09, peers_live=0,
             expected_peers=1),
        _row("peer_seen", 8, peer="bestla", silent_sec=41.3,
             first_contact=False, peers_live=1, expected_peers=1),
        _row("mission_complete", 9, result="arrived"),
        _row("run_end", 10, metrics_timer_rows=163),
    ]
    if arm == "hybrid":
        # logReconnectDispatch, experiment_log.cpp:401-437 -- the only writer of
        # team_incomplete_sec, and therefore the only event on which check 19
        # can legitimately demand it. A real generation-6 dispatch row carries
        # link_down_sec but not team_incomplete_sec, which is what makes this a
        # valid migration witness rather than a field that was always there.
        ev.insert(9, _row(
            "reconnect_dispatch", 90, mode="midrun", terminal=False,
            trigger="midrun", reason="team_incomplete", peer="bestla",
            peer_record_age_sec=48.1, action="dispatch", dest_x=12.5,
            dest_y=-3.0, budget_sec=600.0, decline_reason="", attempt=1,
            gate_sec=40.0, est_unshared_vox=1820.0, link_down_sec=45.0,
            team_incomplete_sec=45.0, peers_live=0, expected_peers=1))
        ev.insert(10, _row("reconnect_end", 91, outcome="gave_up"))
    for e in ev:
        e["robot"] = robot
    return ev


PLANNER_LOG = """[INFO] Exploration complete [latch]: ROI unknown fraction 0.638 <= 0.640 (source=scovox) in state PLAN at t_sim=603.9 - 41 steps, 210.00 m traveled.
[INFO] Reconnect (mid-run): team incomplete 45s >= gate 40s (attempt 1/3)
[INFO] Reconnect manoeuvre ended after 28.5 s sim: gave_up (-> RETURN_HOME).
"""

# The off arm never dispatches, so its planner log has no reconnect lines at
# all. Checks 17 and 20 read those lines; giving the off cell the hybrid log
# would have hidden any check that keys on the arm.
OFF_PLANNER_LOG = PLANNER_LOG.splitlines(True)[0]

CSV = ("step,selected_score,selected_utility\n"
       "0,1.25,1.25\n"
       "1,0.80,0.80\n")

# A PAIRED recovery episode. The fixture used to omit these entirely, so gate E
# reported UNRESOLVED on the clean cell and its pairing arithmetic -- the part
# that decides whether a run is scoreable -- had no known-answer case at all.
NAV_LOG = ("[INFO] global plan ok: 41 poses\n"
           "[WARN] Nav2 -> recovery: spin\n"
           "[INFO] Nav2 recovery EXIT: resumed\n")


# Check 3f's runtime half asserts the PRESENCE of this token, so the clean
# fixture has to carry it. Copied from the RCLCPP_INFO in the link-states
# subscription (explo_planner_node.cpp); if that wording is edited without
# editing this, the clean case fails loudly, which is the right direction.
#
# It goes in the PER-ROBOT PLANNER LOG, which is where the node's stdout
# actually lands (run_explo_sim_rviz.sh's start() redirects each node to
# "$OUTDIR/planner_$r.log"). The first draft of this fixture planted it in
# $ROOT/<cell>.console.log -- and so did the first draft of the check, so the
# harness printed ALL PASS while agreeing with a bug that would have hard-failed
# every treated robot-run of every real campaign. A calibration that shares the
# code's mistake is [[checks-that-stopped-checking]] with extra steps, so the
# planted-defect case below pins the file down and not just the wording.
LIVE_LINE = (
    "[explo_planner_node-3] [INFO] link_gate_live: first usable link sample on "
    "'/comms/link_states' (index ok, link up); the mid-run veto can now run.\n")

CONSOLE_LOG = "clean\n"


def _cell(root, tag, arm, events_by_robot, planner_log, csv_text, manifest,
          console=True, console_text=None, live=True):
    cell = f"{tag}_{arm}_seed1"
    d = os.path.join(root, cell)
    os.makedirs(d, exist_ok=True)
    open(os.path.join(d, "run_manifest.txt"), "w").write(manifest)
    for r in ROBOTS:
        with open(os.path.join(d, f"{r}.events.jsonl"), "w") as fh:
            for e in events_by_robot[r]:
                fh.write(json.dumps(e) + "\n")
        open(os.path.join(d, f"planner_{r}.log"), "w").write(
            planner_log + (LIVE_LINE if live else ""))
        open(os.path.join(d, f"planner_{r}.csv"), "w").write(csv_text)
        open(os.path.join(d, f"nav_{r}.log"), "w").write(NAV_LOG)
    if console:
        open(os.path.join(root, cell + ".console.log"), "w").write(
            CONSOLE_LOG if console_text is None else console_text)
    return d


def build(root, tag, events_by_robot, planner_log=PLANNER_LOG, csv_text=CSV,
          manifest=MANIFEST, off_events=None, off_planner_log=OFF_PLANNER_LOG,
          off_console=True, off_manifest=None, console_text=None, live=True):
    """Write BOTH arms. Defects are planted in the hybrid cell.

    A one-armed fixture cannot exercise check 3e's off branch, and cannot
    exercise check 21 at all -- the campaign-shape check would have had no
    known-answer case whatsoever, which is exactly how the six inert guards in
    [[checks-that-stopped-checking]] got there.
    """
    d = _cell(root, tag, "hybrid", events_by_robot, planner_log, csv_text,
              manifest, console_text=console_text, live=live)
    _cell(root, tag, "off",
          off_events or {r: base_events("off", r) for r in ROBOTS},
          off_planner_log, csv_text, off_manifest or manifest,
          console=off_console)
    return d


def run_gate(root, tag, cells_per_arm="1", env_extra=None):
    """Run the real gate, declaring the synthetic campaign's identity.

    This used to rewrite the gate's source to patch the two FILL_ME literals,
    which meant the calibration validated a COPY of the gate rather than the
    gate. Now that the identity is declared out of band it can be injected, so
    what runs here is byte-for-byte the file that will score the campaign.
    """
    env = dict(os.environ, GATE_ROOT=root,
               GATE_EXPECT_git_explo_planner=REV,
               GATE_EXPECT_sha256_explo_planner_node=SHA,
               GATE_CELLS_PER_ARM=cells_per_arm)
    env.update(env_extra or {})
    p = subprocess.run([sys.executable, GATE, tag], capture_output=True,
                       text=True, env=env)
    return p.returncode, p.stdout + p.stderr


fails = 0


def case(label, mutate, expect_pat, expect_clean=False, cells_per_arm="1"):
    """Plant one defect and assert the gate names the right check."""
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events("hybrid", r) for r in ROBOTS}
        kw = mutate(ev) or {}
        build(root, "cal", ev, **kw)
        rc, out = run_gate(root, "cal", cells_per_arm)
        if expect_clean:
            # rc must be 0, not merely "not 1": rc 3 means a population was
            # empty, and the clean fixture is supposed to populate every one of
            # them. Accepting 3 here would let a fixture rot back into the
            # vacuous state this file exists to detect.
            ok = rc == 0 and "HARD FAILURES: none" in out
            detail = "clean" if ok else (
                "gate objected to a clean cell" if rc == 1 else
                f"rc={rc}: a population the fixture should cover was empty")
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
case("reconnect_dispatch without the migrated team_incomplete_sec",
     lambda ev: drop(ev, "reconnect_dispatch", "team_incomplete_sec"),
     r"check 19 .* missing team_incomplete_sec")

print("\n=== check 20: an unrecognised mid-run line ===")
case("mid-run wording the parser does not know",
     lambda ev: {"planner_log": PLANNER_LOG.replace(
         "team incomplete 45s >= gate 40s", "partner unheard for 45s")},
     r"check 20 .* unrecognised mid-run line")

print("\n=== check 3: provenance ===")
case("cell built from a different binary",
     lambda ev: {"manifest": MANIFEST.replace(REV, "deadbee")},
     r"git_explo_planner=deadbee expected")

print("\n=== checks 3b-3e: the provenance the DATA carries, not the manifest ===")
# Every case below fired zero times before round 4. 3b and 3c were inert
# because they read git_rev from the top level of run_start, where the node has
# never written it, so `rev` was always "" and startswith("") is always True.
# The fixture agreed with the bug, which is why the harness printed ALL PASS.


def _each_runstart(ev, fn):
    for r in ROBOTS:
        for e in ev[r]:
            if e.get("event") == "run_start":
                fn(e)


case("JSONL built by a different binary than the manifest claims",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         git_rev="deadbee")),
     r"check 3b — JSONL git_rev=deadbee")
case("run_start carrying no git_rev at all",
     lambda ev: _each_runstart(ev, lambda e: e["params"].pop("git_rev")),
     r"check 3b — run_start params carry no git_rev")
case("binary built from a dirty tree",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         git_rev=REV + "-dirty")),
     r"check 3c — JSONL git_rev is -dirty")
case("event log written at the previous schema version",
     lambda ev: _each_runstart(ev, lambda e: e.update(schema_version=2)),
     r"check 3d — schema_version=2, expected 3")
case("hybrid directory holding an off configuration",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(arm="off")),
     r"check 3e — directory says arm=hybrid but run_start params say arm='off'")
case("hybrid directory with the reconnect logic disabled",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         rendezvous_enabled=False)),
     r"check 3e — arm=hybrid but rendezvous_enabled=False")


print("\n=== check 3f: the treatment must have been able to happen ===")
# g8r1 is the known-answer case this whole section is calibrated against: it
# passed every other check in this file while 87 % of its treated arm was
# behaviourally the control, because the mid-run clock sat at 240 s against an
# outage distribution whose p90 is 52-111 s. Every case below is a way that can
# recur, and each was confirmed to FAIL the gate before being written down --
# the guard added without a plant-a-failure case is the guard that quietly stops
# checking [[checks-that-stopped-checking]].
case("mid-run clock set where the trigger cannot reach it",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         reconnect_midrun_silence_sec=240.0)),
     r"check 3f — reconnect_midrun_silence_sec=240\.0, expected 90\.0")
case("mid-run clock absent from run_start",
     lambda ev: _each_runstart(
         ev, lambda e: e["params"].pop("reconnect_midrun_silence_sec", None)),
     r"check 3f — run_start params carry no reconnect_midrun_silence_sec")
# A malformed param used to raise float() straight out of the gate: the process
# died with a traceback instead of reporting a hard failure, which is the one
# outcome a gate must never have -- an aborted gate is indistinguishable at a
# glance from a gate that has not been run.
case("mid-run clock present but not a number",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         reconnect_midrun_silence_sec="ninety")),
     r"check 3f — reconnect_midrun_silence_sec='ninety' is not a number")
case("link veto never wired up",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         link_gate_configured=False)),
     r"check 3f — link_gate_configured=false")
case("pre-generation-9 binary, which cannot carry the field",
     lambda ev: _each_runstart(
         ev, lambda e: e["params"].pop("link_gate_configured", None)),
     r"check 3f — run_start params carry no link_gate_configured")
# The case that CONFIGURED-only could never catch, and the reason the runtime
# witness exists: the launcher typed both topic names, so link_gate_configured
# is true, and no sample ever arrived. A dead emulator looks exactly like this.
case("topics named but no link sample ever arrived",
     lambda ev: {"live": False},
     r"check 3f — link_gate_configured=true but no 'link_gate_live:' line")
# The regression pin for the defect this fixture itself once carried: the token
# present, but in $ROOT/<cell>.console.log, which holds only the two harness
# scripts' own log() output and never a line the node emitted. The check and the
# fixture agreed on the wrong file, so the harness printed ALL PASS for a check
# that would have hard-failed every treated robot-run ever recorded.
case("token present but in the harness console log, not the node's",
     lambda ev: {"live": False,
                 "console_text": "clean\n" + LIVE_LINE},
     r"check 3f — link_gate_configured=true but no 'link_gate_live:' line")
case("debounce reintroduced in front of the veto",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         reconnect_link_down_confirm_sec=30.0)),
     r"check 3f — reconnect_link_down_confirm_sec=30\.0, expected 0\.0")

# The planner log being ABSENT is a different verdict from the line being
# absent: one is unanswerable, the other is a failure. Conflating them is how a
# missing artefact reads as a pass.
#
# Deleting the log also blinds checks 17 and 20, which read the same file, so
# this cell hard-fails for those reasons too and rc is 1. That is not what is
# under test here, so the assertion is made directly on 3f's two verdicts: the
# UNRESOLVED line must be present AND the "no link_gate_live:" hard failure must
# NOT be, because a missing artefact must never be scored as a missing veto.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("hybrid", r) for r in ROBOTS}
    build(root, "cal", ev)
    for r in ROBOTS:
        os.remove(os.path.join(root, "cal_hybrid_seed1", f"planner_{r}.log"))
    rc, out = run_gate(root, "cal")
    said_unresolved = re.search(
        r"check 3f: cal_hybrid_seed1/\w+ — no planner log", out)
    said_failure = "no 'link_gate_live:' line" in out
    ok = bool(said_unresolved) and not said_failure
    print(f"  {'PASS' if ok else 'FAIL'}  a treated cell with no planner log is "
          f"3f-UNRESOLVED and not 3f-FAILED: unresolved="
          f"{bool(said_unresolved)} failed={said_failure} (rc={rc})")
    if not ok:
        fails += 1
finally:
    shutil.rmtree(root, ignore_errors=True)

# And the escape hatch has to actually work, or re-gating a banked campaign is
# impossible and the temptation is to edit the expectations in place. An earlier
# draft claimed GATE_MIDRUN_SILENCE alone did this; it does not, because the
# three generation-9 fields have no override of their own.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("hybrid", r) for r in ROBOTS}
    _each_runstart(ev, lambda e: [
        e["params"].update(reconnect_midrun_silence_sec=240.0),
        e["params"].pop("link_gate_configured", None),
        e["params"].pop("reconnect_link_down_confirm_sec", None)])
    # live=False because a generation-8 binary has no such line to emit; the
    # escape hatch has to pass the artefact as it really is, not a hybrid of
    # gen-8 params and a gen-9 console.
    build(root, "cal", ev, live=False)
    rc, out = run_gate(root, "cal")
    strict_caught = rc == 1 and "check 3f" in out
    rc2, out2 = run_gate(root, "cal", env_extra={
        "GATE_MIDRUN_SILENCE": "240", "GATE_GEN9_PARAMS": "0"})
    ok = strict_caught and rc2 == 0 and "HARD FAILURES: none" in out2
    print(f"  {'PASS' if ok else 'FAIL'}  a generation-8 cell fails 3f by "
          f"default (rc={rc}) and passes under GATE_MIDRUN_SILENCE=240 "
          f"GATE_GEN9_PARAMS=0 (rc={rc2})")
    if not ok:
        fails += 1
        for ln in out2.splitlines():
            if "check" in ln or "HARD" in ln:
                print(f"           | {ln}")
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== check 3b/18b/18c: a missing run_start is not a skip ===")


def drop_run_start(ev):
    # A robot whose planner died before it could stamp run_start. This used to
    # be a silent skip -- `if start:` with no else -- so the cell most likely
    # to be broken was the one three checks declined to examine.
    for r in ROBOTS:
        ev[r][:] = [e for e in ev[r] if e.get("event") != "run_start"]


case("no run_start row at all", drop_run_start,
     r"no run_start event — checks 3b, 18b and 18c cannot run")

print("\n=== check 21: the campaign is the shape it was pre-registered as ===")
# Seed-major ordering means an early abort truncates the LAST seeds of both
# arms unevenly, so a short campaign is biased, not merely small. Nothing in
# the gate counted cells before round 4.
case("campaign shorter than pre-registered", lambda ev: None,
     r"check 21 — arm hybrid has 1 cells, pre-registered 30",
     cells_per_arm="30")


root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("hybrid", r) for r in ROBOTS}
    build(root, "cal", ev)
    shutil.rmtree(os.path.join(root, "cal_off_seed1"))
    rc, out = run_gate(root, "cal")
    ok = rc == 1 and re.search(r"check 21 — arm off has 0 cells", out)
    print(f"  {'PASS' if ok else 'FAIL'}  a missing arm is a hard failure, not "
          f"a one-armed campaign scored CLEAN")
    if not ok:
        fails += 1
        print(out[-1200:])
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== the empty-population property: silence must not read as a pass ===")


def strip_watchdogs(ev):
    for r in ROBOTS:
        ev[r][:] = [e for e in ev[r] if e.get("event") != "home_watchdog"]


root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("hybrid", r) for r in ROBOTS}
    off = {r: base_events("off", r) for r in ROBOTS}
    # BOTH arms, or the population is not empty and the case tests nothing --
    # which is precisely the failure mode under test one level up.
    strip_watchdogs(ev)
    strip_watchdogs(off)
    build(root, "cal", ev, off_events=off)
    rc, out = run_gate(root, "cal")
    # rc 3, not 0. UNRESOLVED used to be invisible to the exit code, so a
    # wrapper keying on $? read "nothing was tested" as "everything passed" --
    # the same silence this whole section exists to make audible.
    ok = (rc == 3 and "HARD FAILURES: none" in out
          and "EMPTY: check is UNRESOLVED" in out)
    print(f"  {'PASS' if ok else 'FAIL'}  zero home_watchdog rows reported as "
          f"UNRESOLVED and carried into the exit code (rc={rc}, want 3)")
    if not ok:
        fails += 1
        print(out[-1500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("hybrid", r) for r in ROBOTS}
    build(root, "cal", ev, off_console=False)
    rc, out = run_gate(root, "cal")
    ok = rc == 3 and re.search(r"check 9: cal_off_seed1 — no console log", out)
    print(f"  {'PASS' if ok else 'FAIL'}  a cell with no console log is "
          f"UNRESOLVED, not skipped (rc={rc}, want 3)")
    if not ok:
        fails += 1
        print(out[-1200:])
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== the identity FILE path, which is what the campaign actually uses ===")


def identity_file_case(label, body, expect_rc, expect_pat=None):
    """Run the gate with NO GATE_EXPECT_* env, so it must read the file.

    The assertion used to be `"REFUSING TO RUN" not in out and "check 3" not in
    out`, which the gate cannot fail: it prints "check 3b"/"check 3e" but never
    the bare token "check 3", and the return code was captured and discarded.
    Both halves were inert, so the branch the campaign actually uses to learn
    which binary it is gating had no working test.
    """
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events("hybrid", r) for r in ROBOTS}
        build(root, "cal", ev)
        open(os.path.join(root, "cal.identity.txt"), "w").write(body)
        env = {k: v for k, v in os.environ.items()
               if not k.startswith("GATE_EXPECT_")}
        env["GATE_ROOT"] = root
        env["GATE_CELLS_PER_ARM"] = "1"
        p = subprocess.run([sys.executable, GATE, "cal"], capture_output=True,
                           text=True, env=env)
        out = p.stdout + p.stderr
        ok = p.returncode == expect_rc
        if ok and expect_pat:
            ok = re.search(expect_pat, out) is not None
        print(f"  {'PASS' if ok else 'FAIL'}  {label} (rc={p.returncode}, "
              f"want {expect_rc})")
        if not ok:
            fails += 1
            print("    " + "\n    ".join(out.splitlines()[:14]))
    finally:
        shutil.rmtree(root, ignore_errors=True)


identity_file_case(
    "a correct identity file passes the same cell the env path passes",
    f"# declared at launch\ngit_explo_planner={REV}\n"
    f"sha256_explo_planner_node={SHA}\n", 0)
identity_file_case(
    "a WRONG sha in the identity file fails check 3",
    f"git_explo_planner={REV}\n"
    f"sha256_explo_planner_node=bbbbbbbbbbbbbbbb\n", 1,
    r"sha256_explo_planner_node=" + SHA + r" expected bbbbbbbbbbbbbbbb")
identity_file_case(
    "an empty right-hand side refuses instead of comparing against ''",
    # "" is not the literal FILL_ME, so a truncated identity file used to sail
    # past the refusal and then hard-fail every cell with "expected " and
    # nothing after it -- pointing the operator at the data when the fault is
    # in the declaration.
    f"git_explo_planner={REV}\nsha256_explo_planner_node=\n", 2,
    r"REFUSING TO RUN")

print("\n=== the gate must refuse to run before its identity is declared ===")
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("hybrid", r) for r in ROBOTS}
    build(root, "cal", ev)
    p = subprocess.run([sys.executable, GATE, "cal"], capture_output=True,
                       text=True, env=dict(os.environ, GATE_ROOT=root,
                                           GATE_CELLS_PER_ARM="1"))
    ok = p.returncode == 2 and "REFUSING TO RUN" in p.stdout
    print(f"  {'PASS' if ok else 'FAIL'}  unfilled FILL_ME identity refuses "
          f"(rc={p.returncode})")
    if not ok:
        fails += 1
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== the fixture must match the BINARY, not the gate's beliefs ===")


def audit_fixture_against_real_cell():
    """Compare the fixture's per-event key sets against a banked schema-3 cell.

    This is the durable fix for the defect that motivated the rewrite. Two
    hand-written shapes (git_rev at the top level of run_start,
    team_incomplete_sec on peer_lost/peer_seen) matched two mistaken beliefs in
    the gate, the errors cancelled, and the harness reported 18/18 PASS while
    the gate could neither fail on a stale binary nor pass on a real cell. No
    amount of extra negative cases would have caught that, because every one of
    them was written against the same wrong shape.

    Reports UNRESOLVED -- never PASS -- when no banked cell is reachable, since
    "I could not check" and "I checked and it matched" are different answers.
    """
    global fails
    root = os.environ.get("GATE_CALIB_REAL_ROOT", "/home/kalhan/hmr_campaign")
    if not os.path.isdir(root):
        print(f"  UNRESOLVED  no campaign root at {root}; fixture shapes were "
              f"NOT compared against real data")
        return
    real = {}
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
                if e.get("event") == "run_start" and \
                        e.get("schema_version") != SCHEMA_VERSION:
                    real = {}
                    break                      # wrong generation, skip the file
                real.setdefault(e.get("event"), set()).update(e.keys())
            if real:
                break
        if real:
            break
    if not real:
        print(f"  UNRESOLVED  no schema-{SCHEMA_VERSION} cell found under "
              f"{root}; fixture shapes were NOT compared against real data")
        return
    fixture = {}
    for arm in ("hybrid", "off"):
        for e in base_events(arm):
            fixture.setdefault(e["event"], set()).update(e.keys())
    bad = []
    for event, keys in sorted(fixture.items()):
        if event not in real:
            continue                           # not exercised by that cell
        invented = keys - real[event]
        if invented:
            bad.append(f"{event}: fixture invents {sorted(invented)}, which no "
                       f"writer in the tree emits")
    print(f"  {'PASS' if not bad else 'FAIL'}  fixture keys are a subset of a "
          f"real cell's for {len(set(fixture) & set(real))} event type(s)")
    for b in bad:
        print(f"           | {b}")
    if bad:
        fails += 1


audit_fixture_against_real_cell()

print(f"\n{'ALL PASS' if fails == 0 else str(fails) + ' FAILURE(S)'}")
sys.exit(1 if fails else 0)
