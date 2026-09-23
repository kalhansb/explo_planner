#!/usr/bin/python3
# Moved comments: docs/sim_notes/gate_g8_calib_notes.md
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
# The params file is the OTHER half of what the node ran (the harness says so
# in the manifest writer, three times). Check 3l is the first thing that reads
# it, so the fixture is the first thing that has to write it.
PARAMS_SHA = "dddddddddddddddd"
# The other two identity fields the gate checks, named once so the manifest, the
# identity files and run_gate's declarations agree by construction rather than
# by coincidence. (notes: g8cal-nav-scovox-rev)
NAV_REV = "c9f83a7"
SCOVOX_REV = "078d3f7"
ROBOTS = ["atlas", "bestla"]

# Control arms come from gate_g8.py's GATE_CONTROL_ARMS default, not restated: a
# control fixture gets reconnect_enabled false and no reconnect_dispatch, which
# check 3e compares against the arm name. (notes: g8cal-control-arms-from-gate)
_m = re.search(r'os\.environ\.get\(\s*"GATE_CONTROL_ARMS"\s*,\s*"([^"]*)"\s*\)',
               open(GATE).read())
if not _m:
    sys.exit("FATAL: gate_g8.py's GATE_CONTROL_ARMS default is no longer "
             "readable, so the fixture's notion of a control arm is unpinned. "
             "Fix the pattern above rather than hardcoding the set.")
CONTROL_ARMS = frozenset(s.strip() for s in _m.group(1).split(",") if s.strip())

# The radio-regime fields carry the shipped values check 3j compares against.
# Both roster keys are written from ROBOTS because the gate reads each cell's
# roster from them; without them clean cases score rc=3.
# (notes: g8cal-manifest-radio-roster)
MANIFEST = f"""run_end_reason=all_done
run_gates_verdict=CLEAN
run_end_t_sim=812.4
robots={','.join(ROBOTS)}
team_robot_names={json.dumps(ROBOTS)}
git_explo_planner={REV}
git_simple_nav_3d={NAV_REV}
git_scovox={SCOVOX_REV}
sha256_explo_planner_node={SHA}
sha256_shared_params={PARAMS_SHA}
tx_power_dbm=30.0
tree_attenuation_db=70.0
max_range_m=30.0
"""

# Must equal kSchemaVersion in experiment_log.hpp and gate_g8.py's default.
# Exported to the gate as GATE_SCHEMA_VERSION, so a stale value passes silently;
# audit_schema_pins_agree() checks it. (notes: g8cal-schema-pin)
SCHEMA_VERSION = 11
# The schema the injected regression below downgrades TO. One less than the pin,
# always: the case has to be a version the gate must reject, and hard-coding a 2
# after the pin moved to 4 would have kept passing while testing a two-step
# mismatch instead of the off-by-one a real generation mix produces.
PREV_SCHEMA_VERSION = SCHEMA_VERSION - 1
# Schemas whose event shapes the fixture may be audited against: a set, and the
# scan picks the newest present. Drop a version once a bump changes an existing
# event's fields; additive fields keep older ones valid.
# (notes: g8cal-audit-schemas)
AUDIT_SCHEMAS = (10, 9, 8, 7, 6, 5, 4, 3)

# Field names follow the C++ writers, not the gate's expectations: git_rev sits
# in run_start.params, schema_version at top level, and peer_lost/peer_seen
# carry no team_incomplete_sec. (notes: g8cal-fields-from-writers)
PARAMS = {
    # The node writes the arm and the baked rev as params, so this is where the
    # gate must read them from.
    "arm": "hybrid",
    "git_rev": REV,
    "reconnect_enabled": True,
    "rendezvous_enabled": True,   # deprecated alias; the node stamps both
    "goal_rotate_timeout_sec": 20.0,
    "nav_speed_estimate_mps": 0.5,
    "nav_safety_factor": 2.0,
    "nav_min_timeout_sec": 30.0,
    "nav_max_timeout_sec": 180.0,
    # Present so check 18c's happy path is represented in the clean cell.
    # (notes: g8cal-done-action-present)
    "done_action": "idle",
    # Check 3l: the sensor and candidate configuration, present so 3l's happy
    # path is represented in the clean cell. (notes: g8cal-3l-sensor-params)
    "fov_hfov": 6.28318,
    "fov_vfov": 0.5236,
    "fov_h_rays": 96.0,
    "fov_v_rays": 8.0,
    "fov_min_range": 0.3,
    "fov_max_range": 20.0,
    "fov_is_omnidirectional": True,
    "candidate_n_yaw": 1.0,
    # False, matching the campaign path (FRONTIER_ONLY=1). Logged beside n_yaw
    # because it is what decides whether n_yaw means anything at all.
    "candidate_enable_polar": False,
    "exploitation_enabled": False,
    # The RESOLVED disc, not the 0.0 sentinel: the node rewrites it at
    # construction, and 3l reads the outcome.
    "coord_claim_radius_m": 10.0,
    # Check 3f's treatment-reachability fields: the clock the mid-run trigger
    # arms on, whether the link veto was wired up, and the debounce in front of
    # it. (notes: g8cal-3f-reachability-params)
    "reconnect_midrun_silence_sec": 90.0,
    "link_gate_configured": True,
    "reconnect_link_down_confirm_sec": 0.0,
    # Check 3g stack keys, always present at the off values as the node emits
    # them unconditionally; base_events flips them per m-tare arm. Check 3h
    # dates the binary by rendezvous_schedule_enable (P5) and pursuit_predictor
    # (P6). (notes: g8cal-3g-stack-keys)
    "cell_world_enable": False,
    "team_world_hz": 0.0,
    "global_alloc_enable": False,
    "reconnect_gate": "silence",
    "rendezvous_schedule_enable": False,
    "pursuit_predictor": "trail",
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


def base_events(arm="hybrid", robot="atlas", control_arms=None):
    """A clean robot's event stream, one of each row the new checks read.

    `arm` is honoured because the off arm is not a cosmetic variation: check 3e
    asserts reconnect_enabled tracks the directory's arm, and that branch had
    never been executed by anything before round 4 -- the fixture built a hybrid
    cell only, so the half of the check that guards against an off cell running
    the reconnect logic was as untested as the reconnect logic it guards.
    """
    params = copy.deepcopy(PARAMS)
    params["arm"] = arm
    # Every non-control arm runs the manoeuvre; control_arms overrides the set
    # parsed from gate_g8.py for cases that declare their own. Both spellings
    # are stamped, as the node does; later cases remove each in turn.
    # (notes: g8cal-reconnect-by-control-arms)
    _rdv = arm not in (CONTROL_ARMS if control_arms is None else control_arms)
    params["reconnect_enabled"] = _rdv
    params["rendezvous_enabled"] = _rdv
    # Each mtare_* token gets its own stack, mirroring the launcher's per-token
    # expansion rather than the node's OR that check 3g closes; other arms keep
    # the off values. The _mdp suffix is stripped first.
    # (notes: g8cal-mtare-per-token-stack)
    if arm.startswith("mtare_"):
        _base = arm[:-len("_mdp")] if arm.endswith("_mdp") else arm
        params["cell_world_enable"] = True
        params["team_world_hz"] = 1.0
        params["global_alloc_enable"] = True
        params["reconnect_gate"] = \
            "silence" if _base == "mtare_off" else "info"
        params["rendezvous_schedule_enable"] = \
            _base in ("mtare_rendezvous", "mtare_hybrid")
        # Keyed off the SUFFIX and not off a list of names, because that is how
        # the node derives it: the suffix IS the predictor's value, so a fixture
        # that carried them independently could write a `_mdp` cell running the
        # trail and call it clean.
        params["pursuit_predictor"] = "mdp" if arm.endswith("_mdp") else "trail"
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
        # logPeerLost / logPeerSeen, experiment_log.cpp:407-425. They carry
        # peer/silent_sec/peers_live/expected_peers and NOTHING else: the
        # fixture used to give them team_incomplete_sec, which no writer in the
        # tree emits, and gate check 19 was asserting it there.
        _row("peer_lost", 7, peer="bestla", silent_sec=5.09, peers_live=0,
             expected_peers=1),
        _row("peer_seen", 8, peer="bestla", silent_sec=41.3,
             first_contact=False, peers_live=1, expected_peers=1),
        # ONE mission_complete. The contract check 3m enforces, and the
        # fixture states it by holding exactly one row rather than by saying so
        # in a comment -- the "two rows" case is planted below.
        _row("mission_complete", 9, result="arrived", occurrence=1),
        # The endpoint counters ride run_end at zero rather than omitted: check
        # 3m reads an absent field as a binary that predates them.
        # (notes: g8cal-endpoint-counters-zero)
        _row("run_end", 10, metrics_timer_rows=163,
             mission_return_reentries=0, mission_completes_suppressed=0),
    ]
    # Dispatch rows follow reconnect_enabled, not CONTROL_ARMS: mtare_off
    # dispatches nothing and gets the reconnect-free planner log, so a dispatch
    # pair there would fail check 17. (notes: g8cal-dispatch-follows-reconnect)
    if params["reconnect_enabled"]:
        # Mirrors logReconnectDispatch in experiment_log.cpp, the only writer of
        # team_incomplete_sec, so this is the only event on which check 19 may
        # demand the field. (notes: g8cal-reconnect-dispatch-row)
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

# A paired recovery episode, so gate E's pairing arithmetic has a known-answer
# clean case. (notes: g8cal-nav-log-recovery-pair)
NAV_LOG = ("[INFO] global plan ok: 41 poses\n"
           "[WARN] Nav2 -> recovery: spin\n"
           "[INFO] Nav2 recovery EXIT: resumed\n")


# Check 3f's runtime witness, copied from the link-states RCLCPP_INFO in
# explo_planner_node.cpp; change both together. It goes in the per-robot planner
# log, where node stdout lands, not the console log.
# (notes: g8cal-link-gate-live-line)
LIVE_LINE = (
    "[explo_planner_node-3] [INFO] link_gate_live: first usable link sample on "
    "'/comms/link_states' (index ok, link up); the mid-run veto can now run.\n")

CONSOLE_LOG = "clean\n"


def _cell(root, tag, arm, events_by_robot, planner_log, csv_text, manifest,
          console=True, console_text=None, live=True, robots=None,
          extra_files=None, seed=1):
    # `robots` is a parameter and not the module constant, because the N>=3
    # blackout was three independent two-robot hardcodes and a fixture that can
    # only build pairs cannot tell whether any of them is gone.
    robots = list(robots or ROBOTS)
    cell = f"{tag}_{arm}_seed{seed}"
    d = os.path.join(root, cell)
    os.makedirs(d, exist_ok=True)
    open(os.path.join(d, "run_manifest.txt"), "w").write(manifest)
    for name, text in (extra_files or {}).items():
        open(os.path.join(d, name), "w").write(text)
    for r in robots:
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
          off_console=True, off_manifest=None, console_text=None, live=True,
          treated_arm="hybrid", robots=None, extra_files=None,
          off_extra_files=None):
    """Write BOTH arms. Defects are planted in the hybrid cell.

    A one-armed fixture cannot exercise check 3e's off branch, and cannot
    exercise check 21 at all -- the campaign-shape check would have had no
    known-answer case whatsoever, which is exactly how the six inert guards in
    [[checks-that-stopped-checking]] got there.
    """
    robots = list(robots or ROBOTS)
    d = _cell(root, tag, treated_arm, events_by_robot, planner_log, csv_text,
              manifest, console_text=console_text, live=live, robots=robots,
              extra_files=extra_files)
    _cell(root, tag, "off",
          off_events or {r: base_events("off", r) for r in robots},
          off_planner_log, csv_text, off_manifest or manifest,
          console=off_console, robots=robots, extra_files=off_extra_files)
    return d


def run_gate(root, tag, cells_per_arm="1", env_extra=None):
    """Run the real gate, declaring the synthetic campaign's identity.

    This used to rewrite the gate's source to patch the two FILL_ME literals,
    which meant the calibration validated a COPY of the gate rather than the
    gate. Now that the identity is declared out of band it can be injected, so
    what runs here is byte-for-byte the file that will score the campaign.
    """
    # GATE_SCHEMA_VERSION, GATE_ARMS and GATE_CONTROL_ARMS are pinned, not
    # inherited from the shell, so an operator's exports cannot fail every case
    # for an unrelated reason; env_extra still overrides.
    # (notes: g8cal-run-gate-pinned-env)
    env = dict(os.environ, GATE_ROOT=root,
               GATE_EXPECT_git_explo_planner=REV,
               GATE_EXPECT_git_simple_nav_3d=NAV_REV,
               GATE_EXPECT_git_scovox=SCOVOX_REV,
               GATE_EXPECT_sha256_explo_planner_node=SHA,
               GATE_SCHEMA_VERSION=str(SCHEMA_VERSION),
               GATE_ARMS="hybrid,off",
               GATE_CONTROL_ARMS=",".join(sorted(CONTROL_ARMS)),
               GATE_CELLS_PER_ARM=cells_per_arm)
    env.update(env_extra or {})
    p = subprocess.run([sys.executable, GATE, tag], capture_output=True,
                       text=True, env=env)
    return p.returncode, p.stdout + p.stderr


fails = 0


def case(label, mutate, expect_pat, expect_clean=False, cells_per_arm="1",
         env_extra=None, expect_text=None):
    """Plant one defect and assert the gate names the right check.

    `env_extra` reaches run_gate, for the sub-checks an operator can switch off
    from the invocation -- a relaxation that has never been exercised is a
    relaxation that may relax more than it says.

    `expect_text` is asserted on top of the outcome, and exists because some of
    what this gate must do is PRINT rather than fail: a guard that reports only
    when it fires cannot be told apart from one that was compiled out, so the
    zero case has words and the words are part of the contract.
    """
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events("hybrid", r) for r in ROBOTS}
        kw = mutate(ev) or {}
        build(root, "cal", ev, **kw)
        rc, out = run_gate(root, "cal", cells_per_arm, env_extra=env_extra)
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
        if ok and expect_text is not None \
                and re.search(expect_text, out) is None:
            ok, detail = False, (f"outcome right but did NOT print "
                                 f"/{expect_text}/")
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

print("\n=== check 3m: the endpoint is declared exactly once ===")
# Calibrates check 3m: a doubled mission_complete, the run_end counters from the
# node (mission_return_reentries) and the writer (mission_completes_suppressed),
# and whether those fields are present at all. (notes: g8cal-3m-endpoint-cases)


def two_mission_completes(ev):
    # What a generation-8 log looks like: the second homing leg ran, declared,
    # and restarted homing_duration_sec from zero.
    ev["atlas"].append(_row("mission_complete", 11, result="arrived",
                            occurrence=2, homing_duration_sec=4.1))


case("a robot-run declaring mission_complete TWICE", two_mission_completes,
     r"check 3m — 2 mission_complete rows")
case("run_end with no mission_completes_suppressed field",
     lambda ev: drop(ev, "run_end", "mission_completes_suppressed"),
     r"check 3m — run_end carries no mission_completes_suppressed")
case("run_end with no mission_return_reentries field",
     lambda ev: drop(ev, "run_end", "mission_return_reentries"),
     r"check 3m — run_end carries no mission_return_reentries")
case("the writer having thrown a second row away",
     lambda ev: find(ev, "atlas", "run_end").update(
         mission_completes_suppressed=1),
     r"check 3m — run_end says mission_completes_suppressed=1")
# Not a failure: the node refusing the late coverage latch's second homing
# request is correct and must pass. expect_text asserts the health line is
# printed; rc=0 alone would pass a gate that stopped reading it.
# (notes: g8cal-3m-refused-homing-health)
case("the latch having REFUSED a later homing request (health, not fault)",
     lambda ev: find(ev, "atlas", "run_end").update(
         mission_return_reentries=2),
     None, expect_clean=True,
     expect_text=r"latch REFUSED a later homing request in 1 of 4 robot-runs")
# The zero case says so in words. Without this the clean fixture would prove
# only that the gate does not object, which is also true of a gate that never
# looked.
case("a campaign where the latch refused nothing says so out loud",
     lambda ev: None, None, expect_clean=True,
     expect_text=r"latch refused nothing across 4 robot-runs")
# The population row is the anti-inertness witness: 2 robot-runs carried the
# counters and were read. A future edit that stops reading them would leave this
# at 0 and the gate would print UNRESOLVED rather than a pass.
case("the endpoint-counter population is non-empty and reported",
     lambda ev: None, None, expect_clean=True,
     expect_text=r"3m  endpoint-counter witnesses\s+4")


def _strip(events_by_robot):
    for rows in events_by_robot.values():
        for e in rows:
            if e.get("event") == "run_end":
                e.pop("mission_completes_suppressed", None)
                e.pop("mission_return_reentries", None)
    return events_by_robot


def strip_endpoint_fields(ev):
    # Strip both arms: build() writes the off cell from a fresh base_events, so
    # mutating ev alone reaches half the campaign, which is the mixture case
    # below. (notes: g8cal-strip-both-arms)
    _strip(ev)
    return {"off_events": _strip({r: base_events("off", r) for r in ROBOTS})}


# The documented escape for a banked pre-generation-9 campaign. Exercised
# because an untested relaxation may relax more than it claims: here it must
# drop the EXISTENCE requirement and nothing else, and the population row has to
# say the sub-check was disabled rather than read as a silent pass.
case("GATE_ENDPOINT_FIELDS=0 scores a pre-generation-9 campaign",
     strip_endpoint_fields, None, expect_clean=True,
     env_extra={"GATE_ENDPOINT_FIELDS": "0"},
     expect_text=r"endpoint-counter witnesses\s+0\s+<- sub-check disabled")
# ...and the half that has no escape. Every generation ever written can be asked
# how many mission_complete rows it holds, so the relaxation must NOT buy a pass
# for a doubled endpoint.
def strip_fields_and_double(ev):
    kw = strip_endpoint_fields(ev)
    two_mission_completes(ev)
    return kw


case("...but GATE_ENDPOINT_FIELDS=0 still fails a doubled endpoint",
     strip_fields_and_double, r"check 3m — 2 mission_complete rows",
     env_extra={"GATE_ENDPOINT_FIELDS": "0"})
# Half the campaign carrying the counters is not "a campaign that predates
# them", so the relaxation must NOT buy a pass for it. This is the case the
# flag's own negative control turned up.
case("a campaign that MIXES generations is not relaxable",
     lambda ev: _strip(ev) and None, r"check 3m — 2 of 4 robot-runs carry",
     env_extra={"GATE_ENDPOINT_FIELDS": "0"})

print("\n=== check 3n: run_end is written exactly once ===")
# Check 3n reads the planner log, not the jsonl: the duplicate counter cannot
# ride run_end, which must stay the last line. The in-run WARN and destructor
# ERROR come from different code, so each is tested alone.
# (notes: g8cal-3n-planner-log-witness)
_DUP_WARN = ("[WARN] run_end was already written at t_sim=603.900; this "
             "second attempt at t_sim=930.100 (reason=done-re-entry) is "
             "SUPPRESSED so the file keeps one last line.\n")
_DUP_DTOR = ("[ERROR] ExperimentLog closing '/x/events.jsonl' with 1 "
             "SUPPRESSED duplicate run_end attempt(s).\n")

case("the in-run duplicate-run_end WARN",
     lambda ev: {"planner_log": PLANNER_LOG + _DUP_WARN},
     r"check 3n — the planner log reports a SUPPRESSED duplicate run_end")
case("the destructor's duplicate-run_end ERROR, alone",
     lambda ev: {"planner_log": PLANNER_LOG + _DUP_DTOR},
     r"check 3n — the planner log reports a SUPPRESSED duplicate run_end")
# The zero case has WORDS, and the denominator in them. A check whose healthy
# output is an empty list is indistinguishable from a check that was deleted,
# which is how [[checks-that-stopped-checking]] happened.
case("a clean cell says so in words, with the denominator",
     lambda ev: None, None, expect_clean=True,
     expect_text=r"check 3n — no duplicate run_end in 4 robot-run\(s\)")
case("...and the population row is printed",
     lambda ev: None, None, expect_clean=True,
     expect_text=r"3n  planner logs read\s+4")

# A missing planner log must be 3n-UNRESOLVED, not clean. Asserted directly, not
# via case(): deleting the log also blinds 3f, 17 and 20, so rc is 1 for
# unrelated reasons. (notes: g8cal-3n-missing-log)
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("hybrid", r) for r in ROBOTS}
    build(root, "cal", ev)
    for r in ROBOTS:
        os.remove(os.path.join(root, "cal_hybrid_seed1", f"planner_{r}.log"))
    rc, out = run_gate(root, "cal")
    said_unresolved = re.search(
        r"check 3n: cal_hybrid_seed1/\w+ — no planner_\w+\.log", out)
    # The hybrid cell's two robot-runs must be UNRESOLVED and leave the
    # denominator; the off cell's two remain, so it is pinned to 2 by value.
    # (notes: g8cal-3n-denominator)
    said_partial = re.search(r"no duplicate run_end in 2 robot-run", out)
    ok = bool(said_unresolved) and bool(said_partial)
    print(f"  {'PASS' if ok else 'FAIL'}  a deleted planner log is "
          f"3n-UNRESOLVED and drops OUT of the denominator: unresolved="
          f"{bool(said_unresolved)} denominator_shrank={bool(said_partial)} "
          f"(rc={rc})")
    if not ok:
        fails += 1
finally:
    shutil.rmtree(root, ignore_errors=True)

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
# (notes: g8cal-3b-3c-inert-history)


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
     lambda ev: _each_runstart(ev, lambda e: e.update(
         schema_version=PREV_SCHEMA_VERSION)),
     rf"check 3d — schema_version={PREV_SCHEMA_VERSION}, expected "
     rf"{SCHEMA_VERSION}")
case("hybrid directory holding an off configuration",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(arm="off")),
     r"check 3e — directory says arm=hybrid but run_start params say arm='off'")
case("hybrid directory with the reconnect logic disabled",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         reconnect_enabled=False, rendezvous_enabled=False)),
     r"check 3e — arm=hybrid but reconnect_enabled=False")
# The master switch has two spellings, reconnect_enabled and the legacy
# rendezvous_enabled; check 3e reads whichever is present. Each case deletes one
# and disables the other. (notes: g8cal-3e-two-spellings)
case("reconnect disabled, LEGACY key only (a pre-rename cell)",
     lambda ev: _each_runstart(ev, lambda e: (
         e["params"].pop("reconnect_enabled", None),
         e["params"].update(rendezvous_enabled=False))),
     r"check 3e — arm=hybrid but reconnect_enabled=False")
case("reconnect disabled, CURRENT key only",
     lambda ev: _each_runstart(ev, lambda e: (
         e["params"].pop("rendezvous_enabled", None),
         e["params"].update(reconnect_enabled=False))),
     r"check 3e — arm=hybrid but reconnect_enabled=False")


print("\n=== check 3f: the treatment must have been able to happen ===")
# Each case is a way the treated arm can become unable to fire while every other
# check passes. (notes: g8cal-3f-reachability-cases)
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
# The token planted only in the harness console log, which never holds a node
# line, must still fail 3f. (notes: g8cal-3f-token-wrong-file)
case("token present but in the harness console log, not the node's",
     lambda ev: {"live": False,
                 "console_text": "clean\n" + LIVE_LINE},
     r"check 3f — link_gate_configured=true but no 'link_gate_live:' line")
case("debounce reintroduced in front of the veto",
     lambda ev: _each_runstart(ev, lambda e: e["params"].update(
         reconnect_link_down_confirm_sec=30.0)),
     r"check 3f — reconnect_link_down_confirm_sec=30\.0, expected 0\.0")

# A missing planner log is 3f-UNRESOLVED, never a missing veto. Deleting it also
# blinds 17 and 20 and rc is 1, so the assertion reads 3f's two verdicts
# directly. (notes: g8cal-3f-missing-log)
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
        env["GATE_SCHEMA_VERSION"] = str(SCHEMA_VERSION)   # see run_gate()
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


# All four keys, because all four are now declared rather than hardcoded.
IDENT_OK = (f"# declared at launch\ngit_explo_planner={REV}\n"
            f"git_simple_nav_3d={NAV_REV}\ngit_scovox={SCOVOX_REV}\n"
            f"sha256_explo_planner_node={SHA}\n")
identity_file_case(
    "a correct identity file passes the same cell the env path passes",
    IDENT_OK, 0)
identity_file_case(
    "a WRONG sha in the identity file fails check 3",
    f"git_explo_planner={REV}\ngit_simple_nav_3d={NAV_REV}\n"
    f"git_scovox={SCOVOX_REV}\n"
    f"sha256_explo_planner_node=bbbbbbbbbbbbbbbb\n", 1,
    r"sha256_explo_planner_node=" + SHA + r" expected bbbbbbbbbbbbbbbb")
# The two cases below prove a declared git_simple_nav_3d is compared and an
# undeclared one refuses; a correct file passing proves neither.
# (notes: g8cal-declared-nav-rev-cases)
identity_file_case(
    "a DECLARED git_simple_nav_3d is actually compared, not ignored",
    f"git_explo_planner={REV}\ngit_simple_nav_3d=deadbee\n"
    f"git_scovox={SCOVOX_REV}\n"
    f"sha256_explo_planner_node={SHA}\n", 1,
    r"git_simple_nav_3d=" + NAV_REV + r" expected deadbee")
identity_file_case(
    "an UNDECLARED git_simple_nav_3d refuses rather than silently pinning",
    f"git_explo_planner={REV}\ngit_scovox={SCOVOX_REV}\n"
    f"sha256_explo_planner_node={SHA}\n", 2,
    r"Undeclared: git_simple_nav_3d")
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
                                           GATE_SCHEMA_VERSION=str(
                                               SCHEMA_VERSION),
                                           GATE_CELLS_PER_ARM="1"))
    ok = p.returncode == 2 and "REFUSING TO RUN" in p.stdout
    print(f"  {'PASS' if ok else 'FAIL'}  unfilled FILL_ME identity refuses "
          f"(rc={p.returncode})")
    if not ok:
        fails += 1
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== the m-tare arm: a second treated arm the gate had never seen ===")
# Known-answer cases for arm-agnostic checks: a clean off-vs-mtare_hybrid
# campaign scores CLEAN, a misstamped cell fails 3e, and GATE_ARMS stays a
# pre-registration. (notes: g8cal-mtare-arm-cases)


def _mtare_campaign(root, tag, treated_stamp="mtare_hybrid"):
    """off vs mtare_hybrid. `treated_stamp` is what the NODE recorded, which
    is a different question from what the directory is named."""
    ev = {r: base_events(treated_stamp, r) for r in ROBOTS}
    build(root, tag, ev, treated_arm="mtare_hybrid")


root = tempfile.mkdtemp(prefix="gatecal_")
try:
    _mtare_campaign(root, "cal")
    rc, out = run_gate(root, "cal",
                       env_extra={"GATE_ARMS": "mtare_hybrid,off"})
    ok = rc == 0 and "HARD FAILURES: none" in out
    print(f"  {'PASS' if ok else 'FAIL'}  a clean off-vs-mtare_hybrid campaign "
          f"scores CLEAN (rc={rc}, want 0)")
    if not ok:
        fails += 1
        print(out[-1500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

# The failure the arm token exists to prevent, and the reason run_explo_sim_rviz
# refuses a contradicting environment variable: a cell NAMED for the treatment
# whose node was never handed it. Nothing else in the run says so — the
# manoeuvre still fires, the run still ends all_done, every other check passes.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    _mtare_campaign(root, "cal", treated_stamp="hybrid")
    rc, out = run_gate(root, "cal",
                       env_extra={"GATE_ARMS": "mtare_hybrid,off"})
    ok = rc == 1 and re.search(
        r"check 3e — directory says arm=mtare_hybrid but run_start params "
        r"say arm='hybrid'", out)
    print(f"  {'PASS' if ok else 'FAIL'}  a cell named mtare_hybrid whose node "
          f"ran plain hybrid: caught")
    if not ok:
        fails += 1
        print(out[-1500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

# The pre-registration must still be a pre-registration. If GATE_ARMS could be
# satisfied by whatever happens to be on disk, check 21 would stop being able
# to notice that a campaign ran the wrong arms — which is the same class of
# defect as scoring a truncated campaign CLEAN.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    _mtare_campaign(root, "cal")
    rc, out = run_gate(root, "cal")   # default GATE_ARMS = hybrid,off
    ok = rc == 1 and re.search(r"check 21 — unexpected arm\(s\) \['mtare_hybrid'\]",
                               out)
    print(f"  {'PASS' if ok else 'FAIL'}  an mtare campaign scored against the "
          f"default hybrid,off pre-registration: caught")
    if not ok:
        fails += 1
        print(out[-1500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== check 3g: the m-tare arm must witness its WHOLE stack ===")
# The mtare_ stamp is an OR over features, and cell_world_enable and
# team_world_hz rename nothing, so the arm name certifies little. Check 3g holds
# each arm to its per-arm feature vector, in both directions.
# (notes: g8cal-3g-or-stamp)


def _mtare_campaign_patched(root, tag, treated=None, control=None,
                            treated_arm="mtare_hybrid", control_arms=None):
    """`off` vs `treated_arm`, with run_start params overridden per arm.

    build() always names the second cell `off`, so that is the control arm's
    only spelling here; `control_arms` is the gate's GATE_CONTROL_ARMS
    DECLARATION, which is a different thing and may name more.
    """
    def _patch(rows, overrides):
        if overrides:
            for row in rows:
                if row["event"] == "run_start":
                    row["params"].update(overrides)
        return rows
    ca = None if control_arms is None else frozenset(
        a.strip() for a in control_arms.split(",") if a.strip())
    ev = {r: _patch(base_events(treated_arm, r, ca), treated) for r in ROBOTS}
    off = {r: _patch(base_events("off", r, ca), control) for r in ROBOTS}
    build(root, tag, ev, treated_arm=treated_arm, off_events=off)


def _g3(label, want_re, treated=None, control=None,
        treated_arm="mtare_hybrid", control_arms=None):
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        _mtare_campaign_patched(root, "cal", treated=treated, control=control,
                                treated_arm=treated_arm,
                                control_arms=control_arms)
        env = {"GATE_ARMS": f"{treated_arm},off"}
        if control_arms:
            env["GATE_CONTROL_ARMS"] = control_arms
        rc, out = run_gate(root, "cal", env_extra=env)
        ok = rc == 1 and re.search(want_re, out)
        print(f"  {'PASS' if ok else 'FAIL'}  {label}")
        if not ok:
            fails += 1
            print(f"        rc={rc}, wanted /{want_re}/")
            print(out[-1200:])
    finally:
        shutil.rmtree(root, ignore_errors=True)


# Each of the five, alone. Half a treatment is not a weaker treatment, it is a
# different arm, and pooling it into mtare_hybrid does to that comparison what
# grouping on reconnect_mode did to the control.
_g3("an mtare_hybrid cell that ran no cell census: caught",
    r"check 3g — arm=mtare_hybrid but cell_world_enable=False",
    treated={"cell_world_enable": False})
_g3("an mtare_hybrid cell whose exchange was switched off by rate: caught",
    r"check 3g — arm=mtare_hybrid but team_world_hz>0=False",
    treated={"team_world_hz": 0.0})
_g3("an mtare_hybrid cell with the allocator off: caught",
    r"check 3g — arm=mtare_hybrid but global_alloc_enable=False",
    treated={"global_alloc_enable": False})
# THE ONE THE ARM NAME CANNOT SEE. reconnect_gate=silence with the allocator on
# still stamps `mtare_`, so this cell passes 3e and is P3-only — exactly the
# half-treatment the OR admits.
_g3("an mtare_hybrid cell with the knowledge gate off: caught by 3g alone",
    r"check 3g — arm=mtare_hybrid but reconnect_gate=info=False",
    treated={"reconnect_gate": "silence"})
# Without an appointment the cell is still stamped mtare_hybrid, so only 3g can
# tell hybrid fell back to the last-contact midpoint instead of an agreed cell.
# (notes: g8cal-3g-no-appointment)
_g3("an mtare_hybrid cell with no appointment armed: caught by 3g alone",
    r"check 3g — arm=mtare_hybrid but rendezvous_schedule_enable=False",
    treated={"rendezvous_schedule_enable": False})

# A treated arm carrying a feature its definition excludes: mtare_pursuit is the
# 2x2's appointment-off cell, so an appointment makes it a second mtare_hybrid.
# (notes: g8cal-3g-pursuit-with-appointment)
_g3("an mtare_pursuit cell that armed an appointment: caught",
    r"check 3g — arm=mtare_pursuit but "
    r"rendezvous_schedule_enable=True \(want False\)",
    treated={"rendezvous_schedule_enable": True},
    treated_arm="mtare_pursuit")
# And the same in the factorial's own control. mtare_off is the neither-
# mechanism cell; a value gate in it is a treatment in the control column of
# the design, which is the error no care in the treated arms can offset.
_g3("an mtare_off cell running the value gate: caught",
    r"check 3g — arm=mtare_off but "
    r"reconnect_gate=info=True \(want False\)",
    treated={"reconnect_gate": "info"},
    treated_arm="mtare_off", control_arms="off,mtare_off")

# The control direction, which is the one that decides the comparison: a
# treated cell sitting in the control column cannot be compensated for by any
# amount of care in the treated arm.
_g3("an off cell that ran the allocator: caught",
    r"check 3g — arm=off but global_alloc_enable=True",
    control={"global_alloc_enable": True})
_g3("an off cell that armed an appointment: caught",
    r"check 3g — arm=off but rendezvous_schedule_enable=True",
    control={"rendezvous_schedule_enable": True})

# A cell missing the stack keys came from a pre-stack binary and must be
# refused, not certified. STACK_KEYS strips all of them, dating keys included,
# from both arms so check 3h does not also fire.
# (notes: g8cal-pre-stack-refused)
PRE_P5_KEY = "rendezvous_schedule_enable"
PRE_P6_KEY = "pursuit_predictor"
# The two dating keys, stripped together for a pre-stack binary: dropping only
# PRE_P5_KEY leaves a predictor no binary had, which the gate refuses on its
# own. (notes: g8cal-gen-keys-together)
GEN_KEYS = (PRE_P5_KEY, PRE_P6_KEY)
STACK_KEYS = ("cell_world_enable", "team_world_hz",
              "global_alloc_enable", "reconnect_gate") + GEN_KEYS


def _strip_keys(rows, keys):
    for row in rows:
        if row["event"] == "run_start":
            for k in keys:
                row["params"].pop(k, None)
    return rows


root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: _strip_keys(base_events("mtare_hybrid", r), STACK_KEYS)
          for r in ROBOTS}
    off = {r: _strip_keys(base_events("off", r), STACK_KEYS) for r in ROBOTS}
    build(root, "cal", ev, treated_arm="mtare_hybrid", off_events=off)
    rc, out = run_gate(root, "cal", env_extra={"GATE_ARMS": "mtare_hybrid,off"})
    ok = rc == 1 and re.search(
        r"check 3g — run_start params carry no cell_world_enable, "
        r"global_alloc_enable, reconnect_gate, team_world_hz", out)
    print(f"  {'PASS' if ok else 'FAIL'}  a cell from a pre-M-TARE binary is "
          f"refused, not certified")
    if not ok:
        fails += 1
        print(out[-1200:])
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== check 3h: one binary generation per campaign ===")
# P5 changed what mtare_hybrid means under the same token, so 3h dates each cell
# by the presence of rendezvous_schedule_enable and refuses a mix; neither pure
# generation is refused. (notes: g8cal-3h-p5-generation)

# (a) mixed: pre-P5 control cells, P5 treated cells. Every other check passes.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("mtare_hybrid", r) for r in ROBOTS}
    off = {r: _strip_keys(base_events("off", r), GEN_KEYS)
           for r in ROBOTS}
    build(root, "cal", ev, treated_arm="mtare_hybrid", off_events=off)
    rc, out = run_gate(root, "cal", env_extra={"GATE_ARMS": "mtare_hybrid,off"})
    ok = rc == 1 and re.search(
        r"check 3h — this campaign mixes binary generations", out)
    print(f"  {'PASS' if ok else 'FAIL'}  a campaign pooling a pre-P5 arm with "
          f"a P5 one is refused")
    if not ok:
        fails += 1
        print(f"        rc={rc}")
        print(out[-1200:])
finally:
    shutil.rmtree(root, ignore_errors=True)

# (b) uniformly pre-P5 -- campaign mh1's shape. Must still score CLEAN: a check
# that could only pass on cells banked after today would have retired every
# campaign already on disk, which is the loosening-vs-recalibration trap from
# the other direction.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: _strip_keys(base_events("mtare_hybrid", r), GEN_KEYS)
          for r in ROBOTS}
    off = {r: _strip_keys(base_events("off", r), GEN_KEYS)
           for r in ROBOTS}
    build(root, "cal", ev, treated_arm="mtare_hybrid", off_events=off)
    rc, out = run_gate(root, "cal", env_extra={"GATE_ARMS": "mtare_hybrid,off"})
    ok = (rc == 0 and "HARD FAILURES: none" in out
          and "binary generation: pre-P5" in out)
    print(f"  {'PASS' if ok else 'FAIL'}  a uniformly pre-P5 campaign (mh1's "
          f"shape) still scores CLEAN")
    if not ok:
        fails += 1
        print(f"        rc={rc}")
        print(out[-1500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

# (c) and a pre-P5 binary cannot have stamped a P5-era arm token. The three
# factorial names did not exist and the runner refused every route to
# mtare_off, so the name on the directory is the only thing claiming that arm.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: _strip_keys(base_events("mtare_pursuit", r), GEN_KEYS)
          for r in ROBOTS}
    off = {r: _strip_keys(base_events("off", r), GEN_KEYS)
           for r in ROBOTS}
    build(root, "cal", ev, treated_arm="mtare_pursuit", off_events=off)
    rc, out = run_gate(root, "cal", env_extra={"GATE_ARMS": "mtare_pursuit,off"})
    ok = rc == 1 and re.search(
        r"check 3g — arm=mtare_pursuit is not an arm a pre-P5 binary can stamp",
        out)
    print(f"  {'PASS' if ok else 'FAIL'}  a P5-era arm token on a pre-P5 "
          f"binary is refused")
    if not ok:
        fails += 1
        print(f"        rc={rc}")
        print(out[-1200:])
finally:
    shutil.rmtree(root, ignore_errors=True)

# (d) the whole §3.6.1 factorial, correctly configured, must score CLEAN. The
# checks above are all refusals; without this one they could all be satisfied
# by a gate that rejects the four-arm design outright.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    _CA = frozenset({"off", "mtare_off"})
    for _arm in ("mtare_off", "mtare_pursuit", "mtare_rendezvous",
                 "mtare_hybrid"):
        _cell(root, "cal", _arm,
              {r: base_events(_arm, r, _CA) for r in ROBOTS},
              OFF_PLANNER_LOG if _arm == "mtare_off" else PLANNER_LOG,
              CSV, MANIFEST)
    rc, out = run_gate(root, "cal", env_extra={
        "GATE_ARMS": "mtare_off,mtare_pursuit,mtare_rendezvous,mtare_hybrid",
        "GATE_CONTROL_ARMS": "mtare_off"})
    ok = rc == 0 and "HARD FAILURES: none" in out
    print(f"  {'PASS' if ok else 'FAIL'}  the four-arm §3.6.1 factorial scores "
          f"CLEAN — the checks above are not a blanket no")
    if not ok:
        fails += 1
        print(f"        rc={rc}")
        print(out[-2500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== P6: the predictor is a sixth feature and a second generation ===")
# P6 splits the chasing arms by predictor: trail aims at the peer's last
# declared goal, mdp at a modelled intercept. The node stamps the _mdp suffix
# itself, so 3e sees no mismatch; 3g must. (notes: g8cal-p6-predictor-cases)

# (e) the failure the suffix exists to make visible: the model arm running the
# trail. Nothing outside 3g looks at this param, so without the sixth key this
# cell is bit-identical to a correct one in every field the gate reads.
_g3("an mtare_hybrid_mdp cell that ran the trail: caught",
    r"check 3g — arm=mtare_hybrid_mdp but "
    r"pursuit_predictor=mdp=False \(want True\)",
    treated={"pursuit_predictor": "trail"},
    treated_arm="mtare_hybrid_mdp")
_g3("an mtare_pursuit_mdp cell that ran the trail: caught",
    r"check 3g — arm=mtare_pursuit_mdp but "
    r"pursuit_predictor=mdp=False \(want True\)",
    treated={"pursuit_predictor": "trail"},
    treated_arm="mtare_pursuit_mdp")
# (f) and the other direction, which is the one that decides a trail-vs-model
# contrast: the trail arm silently running the model. A campaign contrasting
# mtare_hybrid against mtare_hybrid_mdp would then be comparing the model
# against itself.
_g3("an mtare_hybrid cell that ran the model: caught",
    r"check 3g — arm=mtare_hybrid but "
    r"pursuit_predictor=mdp=True \(want False\)",
    treated={"pursuit_predictor": "mdp"},
    treated_arm="mtare_hybrid")
_g3("an off cell that ran the model: caught",
    r"check 3g — arm=off but pursuit_predictor=mdp=True",
    control={"pursuit_predictor": "mdp"})
# (g) a spelling the node cannot have produced. It throws on any third value, so
# this is not a defective run — it is a param dump that did not come from one,
# and every feature read out of it is suspect.
_g3("a pursuit_predictor the node would have refused to start on: caught",
    r"check 3g — pursuit_predictor='trial' is neither 'trail' nor 'mdp'",
    treated={"pursuit_predictor": "trial"})


def _p6(label, want_re, mutate_treated=None, mutate_control=None,
        treated_arm="mtare_hybrid", expect_clean=False, want_text=None):
    """Same shape as _g3 but for cases that strip keys rather than set them."""
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: (mutate_treated or (lambda x: x))(
            base_events(treated_arm, r)) for r in ROBOTS}
        off = {r: (mutate_control or (lambda x: x))(
            base_events("off", r)) for r in ROBOTS}
        build(root, "cal", ev, treated_arm=treated_arm, off_events=off)
        rc, out = run_gate(root, "cal",
                           env_extra={"GATE_ARMS": f"{treated_arm},off"})
        if expect_clean:
            ok = rc == 0 and "HARD FAILURES: none" in out and want_text in out
        else:
            ok = rc == 1 and re.search(want_re, out)
        print(f"  {'PASS' if ok else 'FAIL'}  {label}")
        if not ok:
            fails += 1
            wanted = repr(want_text) if expect_clean else f"/{want_re}/"
            print(f"        rc={rc}, wanted {wanted}")
            print(out[-1500:])
    finally:
        shutil.rmtree(root, ignore_errors=True)


# (h) Every cell here emits rendezvous_schedule_enable, so 3h's P5 half is
# satisfied; the P6 boundary needs its own witness. (notes: g8cal-3h-p6-mix)
_p6("a campaign pooling a pre-P6 arm with a P6 one is refused",
    r"check 3h — this campaign mixes binary generations at the P6 boundary",
    mutate_control=lambda rows: _strip_keys(rows, (PRE_P6_KEY,)))
# (i) and the mirror: every campaign banked between P5 and P6 is uniformly
# pre-P6 and must still score CLEAN. A check that could only pass on cells
# banked after today would retire every campaign already on disk.
_p6("a uniformly P5-but-pre-P6 campaign still scores CLEAN",
    None,
    mutate_treated=lambda rows: _strip_keys(rows, (PRE_P6_KEY,)),
    mutate_control=lambda rows: _strip_keys(rows, (PRE_P6_KEY,)),
    expect_clean=True,
    want_text="binary generation (predictor): pre-P6")
# (j) a pre-P6 binary cannot have stamped a P6-era token. The suffix did not
# exist, so the directory name is the only thing claiming that arm.
_p6("a P6-era arm token on a pre-P6 binary is refused",
    r"check 3g — arm=mtare_hybrid_mdp is not an arm a P5-but-pre-P6 binary "
    r"can stamp",
    mutate_treated=lambda rows: _strip_keys(rows, (PRE_P6_KEY,)),
    mutate_control=lambda rows: _strip_keys(rows, (PRE_P6_KEY,)),
    treated_arm="mtare_hybrid_mdp")
# (k) and the impossible order. No binary ever had the predictor without the
# scheduler, so a dump claiming one dates to no single generation and the
# dating both checks rest on is void.
_p6("a dump carrying P6's param but not P5's is refused",
    r"check 3g — run_start carries pursuit_predictor \(P6\) but no "
    r"rendezvous_schedule_enable \(pre-P5\)",
    mutate_treated=lambda rows: _strip_keys(rows, (PRE_P5_KEY,)),
    mutate_control=lambda rows: _strip_keys(rows, (PRE_P5_KEY,)))

# (l) The case the others exist for: the real four-arm design, correctly
# configured, scores CLEAN. (notes: g8cal-ts4-four-arms-clean)
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    _CA = frozenset({"off", "mtare_off"})
    _TS4 = ("mtare_off", "mtare_pursuit_mdp", "mtare_rendezvous",
            "mtare_hybrid_mdp")
    for _arm in _TS4:
        _cell(root, "cal", _arm,
              {r: base_events(_arm, r, _CA) for r in ROBOTS},
              OFF_PLANNER_LOG if _arm == "mtare_off" else PLANNER_LOG,
              CSV, MANIFEST)
    rc, out = run_gate(root, "cal", env_extra={
        "GATE_ARMS": ",".join(_TS4), "GATE_CONTROL_ARMS": "mtare_off"})
    ok = (rc == 0 and "HARD FAILURES: none" in out
          and "binary generation (predictor): P6+" in out)
    print(f"  {'PASS' if ok else 'FAIL'}  ts4's four arms score CLEAN — the "
          f"two `_mdp` columns are not rejected for existing")
    if not ok:
        fails += 1
        print(f"        rc={rc}")
        print(out[-2500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

# And the check must not be a blanket no. A hybrid-vs-off campaign has no
# m-tare arm and must still score CLEAN -- with 3g's ON population reported as
# legitimately empty rather than left UNRESOLVED, which is the distinction the
# whole denominator block exists to draw.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("hybrid", r) for r in ROBOTS}
    build(root, "cal", ev)
    rc, out = run_gate(root, "cal")
    ok = (rc == 0 and "HARD FAILURES: none" in out
          and re.search(r"m-tare features certified ON\s+0\s+<- GATE_ARMS .* "
                        r"declares no m-tare arm", out)
          and re.search(r"m-tare features certified OFF\s+[1-9]", out))
    print(f"  {'PASS' if ok else 'FAIL'}  a hybrid-vs-off campaign still scores "
          f"CLEAN, with 3g's ON population exempt not unresolved")
    if not ok:
        fails += 1
        print(out[-1500:])
finally:
    shutil.rmtree(root, ignore_errors=True)

print("\n=== the fixture must match the BINARY, not the gate's beliefs ===")


def audit_fixture_against_real_cell():
    """Compare the fixture's per-event key sets against a banked real cell.

    This is the durable fix for the defect that motivated the rewrite. Two
    hand-written shapes (git_rev at the top level of run_start,
    team_incomplete_sec on peer_lost/peer_seen) matched two mistaken beliefs in
    the gate, the errors cancelled, and the harness reported 18/18 PASS while
    the gate could neither fail on a stale binary nor pass on a real cell. No
    amount of extra negative cases would have caught that, because every one of
    them was written against the same wrong shape.

    Reports UNRESOLVED -- never PASS -- when no banked cell is reachable, since
    "I could not check" and "I checked and it matched" are different answers.

    AND UNRESOLVED COUNTS AGAINST THE EXIT STATUS, which it did not until
    2026-09-18. Both refusals below printed their line and returned without
    touching `fails`, so a run that compared the fixture against nothing at all
    exited 0 -- identical, to anything reading the status, to a run where the
    audit passed. The sentence above says these are different answers; the code
    made them the same answer. equiv_gate_calib.py had the same defect in the
    same shape and took the same edit.
    """
    global fails
    root = os.environ.get("GATE_CALIB_REAL_ROOT", "/home/kalhan/hmr_campaign")
    if not os.path.isdir(root):
        print(f"  UNRESOLVED  no campaign root at {root}; fixture shapes were "
              f"NOT compared against real data")
        fails += 1
        return
    # Pick the newest AUDIT_SCHEMAS cell, not the first by name. Scan to depth 2
    # (campaign/cell). Pass 1 reads each log only to its run_start; pass 2
    # parses the one winner, so cost does not grow with the bank.
    # (notes: g8cal-audit-scan-newest)
    best_path, best_schema, best_dir = None, None, None
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
            for r in ROBOTS:
                p = os.path.join(d, f"{r}.events.jsonl")
                if not os.path.exists(p):
                    continue
                for ln in open(p, errors="replace"):
                    try:
                        e = json.loads(ln)
                    except ValueError:
                        continue
                    if e.get("event") != "run_start":
                        continue
                    s = e.get("schema_version")
                    if s in AUDIT_SCHEMAS and (best_schema is None
                                               or s > best_schema):
                        best_path, best_schema, best_dir = p, s, d
                    break
    real, real_schema, real_where = {}, None, None
    if best_path:
        real_schema = best_schema
        real_where = os.path.relpath(best_dir, root)
        for ln in open(best_path, errors="replace"):
            try:
                e = json.loads(ln)
            except ValueError:
                continue
            real.setdefault(e.get("event"), set()).update(e.keys())
    if not real:
        print(f"  UNRESOLVED  no cell at schema "
              f"{'/'.join(str(v) for v in AUDIT_SCHEMAS)} found under {root}; "
              f"fixture shapes were NOT compared against real data")
        fails += 1
        return
    fixture = {}
    for arm in ("hybrid", "off"):
        for e in base_events(arm):
            fixture.setdefault(e["event"], set()).update(e.keys())
    # A fixture key missing from the banked cell may just be newer; the
    # experiment_log.cpp writers are the authority. Only a key in neither the
    # cell nor any writer fails. (notes: g8cal-audit-new-vs-invented)
    writers, writer_err = _event_field_writers()
    if writer_err:
        print(f"  UNRESOLVED  {writer_err}; fixture shapes could NOT be "
              f"separated into 'new field' and 'invented field'")
        fails += 1
        return
    bad, novel = [], []
    for event, keys in sorted(fixture.items()):
        if event not in real:
            continue                           # not exercised by that cell
        invented = keys - real[event]
        for k in sorted(invented):
            if k in writers:
                novel.append(f"{event}.{k}")
            else:
                bad.append(f"{event}: fixture invents {k!r}, which is in "
                           f"neither a real cell nor any experiment_log.cpp "
                           f"writer")
    # Name the schema actually compared against. "PASS" alone would let a
    # reader assume the current one was available when it was not.
    unseen = sorted(set(fixture) - set(real))
    print(f"  {'PASS' if not bad else 'FAIL'}  fixture keys are a subset of a "
          f"real schema-{real_schema} cell's for "
          f"{len(set(fixture) & set(real))} event type(s)")
    # Which cell, not just which schema. The schema alone cannot distinguish
    # "the newest thing banked" from "the newest thing this scan can see", and
    # that distinction is exactly what was wrong here for four generations.
    print(f"           | audited against {real_where}, pin is "
          f"schema {SCHEMA_VERSION}")
    # Report the fixture event kinds the reference cell never emitted: they were
    # skipped, and the PASS line reads the same either way.
    # (notes: g8cal-audit-unseen-kinds)
    if unseen:
        print(f"           | {len(unseen)} fixture event kind(s) absent from "
              f"that cell and NOT compared against real data: "
              f"{', '.join(unseen)}")
    for b in bad:
        print(f"           | {b}")
    # Named, not hidden. A field that is new is a field the banked cells cannot
    # vouch for, so the reader is told which parts of the fixture rest on the
    # writer scan alone rather than on real data.
    if novel:
        print(f"           | {len(novel)} field(s) postdate that cell and were "
              f"matched against experiment_log.cpp writers instead: "
              f"{', '.join(novel)}")
    if bad:
        fails += 1

    # A subset test cannot fail for a stale reference, so staleness is its own
    # verdict. One schema behind the pin is normal (no new-generation cell
    # exists yet); two or more fails. (notes: g8cal-audit-staleness-tripwire)
    gap = SCHEMA_VERSION - real_schema
    ok = gap <= 1
    print(f"  {'PASS' if ok else 'FAIL'}  the audited reference is within one "
          f"schema of the pin (schema {real_schema} vs pin {SCHEMA_VERSION})")
    if not ok:
        print(f"           | {gap} generations behind. Either no campaign since "
              f"schema {real_schema} banked a cell where this scan looks "
              f"(depth 1 or 2 under {root}, robots {'/'.join(ROBOTS)}), or the "
              f"layout moved again. Every case above still passed, because a "
              f"subset test passes hardest against the oldest reference.")
        fails += 1


print("\n=== checks 3e/3i: the per-cell _r<N> claim-radius suffix ===")
# run_campaign.sh strips an _r<N> claim-radius suffix before the node stamps the
# arm, so 3e and 3g compare without it and 3i holds it against the manifest.
# Cases pin a clean pass and a still-caught mis-assignment.
# (notes: g8cal-radius-suffix)
R10_MANIFEST = MANIFEST + "coord_claim_radius_override=10.0\n"
R40_MANIFEST = MANIFEST + "coord_claim_radius_override=40.0\n"


def radius_case(label, expect_pat, treated_arm, manifest, stamp_arm=None,
                expect_clean=False, expect_rc=1):
    """One suffixed-campaign fixture. `stamp_arm` is what the NODE recorded.

    `expect_rc` is 1 for a planted defect the gate must hard-fail and 3 for one
    it must report as UNRESOLVED. The 3 path additionally requires
    "HARD FAILURES: none", because the whole claim being tested there is that
    the cell was NOT condemned -- a case that accepted rc=3 alongside a hard
    failure would pass on the behaviour it exists to forbid.
    """
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events(stamp_arm or "mtare_hybrid", r,
                             control_arms={"off"}) for r in ROBOTS}
        # off_manifest is the UNSUFFIXED default on purpose: the control cell is
        # plain `off`, so a manifest carrying a pinned radius would fire 3i on
        # the control and every case here would "pass" on the wrong cell.
        build(root, "cal", ev, treated_arm=treated_arm, manifest=manifest,
              off_manifest=MANIFEST)
        rc, out = run_gate(root, "cal", env_extra={
            "GATE_ARMS": f"{treated_arm},off", "GATE_CONTROL_ARMS": "off"})
        if expect_clean:
            ok = rc == 0 and "HARD FAILURES: none" in out
            detail = "clean" if ok else (
                f"gate objected to a correctly-run suffixed cell (rc={rc})"
                if rc == 1 else f"rc={rc}: a population was empty")
        else:
            ok = rc == expect_rc and re.search(expect_pat, out) is not None
            if ok and expect_rc == 3 and "HARD FAILURES: none" not in out:
                ok = False
            detail = ("caught" if ok else
                      f"did NOT match /{expect_pat}/ at rc={expect_rc} "
                      f"(got rc={rc})")
        print(f"  {'PASS' if ok else 'FAIL'}  {label}: {detail}")
        if not ok:
            fails += 1
            for ln in out.splitlines():
                if "check 3" in ln or "HARD" in ln:
                    print(f"           | {ln}")
    finally:
        shutil.rmtree(root, ignore_errors=True)


radius_case("a correctly-run _r10 cell scores clean", None,
            "mtare_hybrid_r10", R10_MANIFEST, expect_clean=True)
# The anti-vacuity case. Same suffixed directory, but the node stamped the
# CONTROL configuration -- a leaked RECONNECT_MODE, the corruption 3e exists
# for. Stripping the suffix must not have cost the gate its ability to see it.
radius_case("a suffixed directory running the wrong arm is still caught",
            r"check 3e — directory says arm=mtare_hybrid_r10 "
            r"\(policy mtare_hybrid\) but run_start params say arm='mtare_off'",
            "mtare_hybrid_r10", R10_MANIFEST, stamp_arm="mtare_off")
# 3i, three ways. Each one files a cell under a radius it did not run, which no
# other check in this gate looks at: the arm token says nothing about the knob.
radius_case("an _r40 directory that ran 10 m",
            r"check 3i — directory says claim radius 40 m but the manifest "
            r"says coord_claim_radius_override='10\.0'",
            "mtare_hybrid_r40", R10_MANIFEST)
radius_case("a suffixed directory with no radius recorded at all",
            r"check 3i — directory says the claim radius was 10 m, but the "
            r"manifest records no coord_claim_radius_override at all",
            "mtare_hybrid_r10", MANIFEST)
radius_case("an unsuffixed directory that ran a pinned radius",
            r"check 3i — directory names no claim radius but the manifest says "
            r"coord_claim_radius_override='40\.0'",
            "mtare_hybrid", R40_MANIFEST)

print("\n=== checks 3e/3i: the _ttl<N> suffix, and TWO suffixes at once ===")
# Cells often carry both an _r<N> and a _ttl<N> suffix. The two-suffix shape is
# pinned in both orders, since order-independence belongs to the parser loop,
# not the table. (notes: g8cal-two-suffix-orders)
TTL0_MANIFEST = R40_MANIFEST + "alloc_peer_pos_max_age_sec=0\n"
TTL120_MANIFEST = R40_MANIFEST + "alloc_peer_pos_max_age_sec=120\n"
radius_case("a correctly-run _r40_ttl0 cell scores clean (the ts1b shape)",
            None, "mtare_hybrid_r40_ttl0", TTL0_MANIFEST, expect_clean=True)
radius_case("the same two suffixes in the OTHER order still parse (at2's shape)",
            None, "mtare_hybrid_ttl0_r40", TTL0_MANIFEST, expect_clean=True)
# 3i over the TTL dimension, the same three ways it is checked over the radius.
# Written out rather than trusted to generalise, because "the loop covers every
# dimension" is the claim, and a loop that silently skipped the second entry
# would look identical from the radius cases alone.
radius_case("a _ttl0 directory that ran a 120 s TTL",
            r"check 3i — directory says allocator peer-position TTL 0 s but "
            r"the manifest says alloc_peer_pos_max_age_sec='120'",
            "mtare_hybrid_r40_ttl0", TTL120_MANIFEST)
radius_case("a directory naming no TTL whose run pinned one",
            r"check 3i — directory names no allocator peer-position TTL but "
            r"the manifest says alloc_peer_pos_max_age_sec='120'",
            "mtare_hybrid_r40", TTL120_MANIFEST)
radius_case("a _ttl0 directory with no TTL recorded at all",
            r"check 3i — directory says the allocator peer-position TTL was "
            r"0 s, but the manifest records no alloc_peer_pos_max_age_sec at "
            r"all", "mtare_hybrid_r40_ttl0", R40_MANIFEST)

print("\n=== check 3e: a suffix this gate has not been taught is ITS gap ===")
# An unknown trailing token (e.g. _zz9) on a matching stamp is a knob the gate
# lacks, not a bad run: UNRESOLVED, never certified and never a hard failure.
# (notes: g8cal-3e-unknown-suffix)
radius_case("an unrecognised trailing token is UNRESOLVED, not a hard failure",
            r"check 3e — .*the single trailing token _zz9 that this gate does "
            r"not know", "mtare_hybrid_zz9", MANIFEST, expect_rc=3)
# The anti-vacuity twin. Same unknown token, but the node stamped the CONTROL
# configuration -- a leaked RECONNECT_MODE, the corruption 3e exists for. The
# forgiving branch must not have swallowed it.
radius_case("an unknown token does not excuse a genuinely mis-assigned cell",
            r"check 3e — directory says arm=mtare_hybrid_zz9 .*but run_start "
            r"params say arm='mtare_off'",
            "mtare_hybrid_zz9", MANIFEST, stamp_arm="mtare_off")
# And the other direction of the same asymmetry: a stamp LONGER than the
# directory name is the binary claiming an arm the directory does not, which is
# a real disagreement and must stay a hard failure. Without this case,
# unrecognised_suffix() could be made symmetric and nothing would notice.
radius_case("a stamp longer than the directory name stays a hard failure",
            r"check 3e — directory says arm=mtare_hybrid .*but run_start "
            r"params say arm='mtare_hybrid_zz9'",
            "mtare_hybrid", MANIFEST, stamp_arm="mtare_hybrid_zz9")

print("\n=== checks 3a/3k: the roster comes from the CELL, not from a constant ===")
# The gate reads the roster from each cell, not a constant. These cases need
# teams of any size, which is why _cell() and build() take a roster.
# (notes: g8cal-roster-from-cell)
TRIO = list(ROBOTS) + ["husky"]


def _manifest_for(robots, base=MANIFEST):
    """`base` with both roster keys rewritten to name `robots`."""
    out = re.sub(r"^robots=.*$", "robots=" + ",".join(robots), base, flags=re.M)
    return re.sub(r"^team_robot_names=.*$",
                  "team_robot_names=" + json.dumps(robots), out, flags=re.M)


def roster_case(label, expect_pat, robots=None, manifest=None, mutate=None,
                expect_clean=False, expect_rc=1, build_robots=None,
                extra_assert=None):
    """One N-robot fixture. `robots` is DECLARED, `build_robots` is on disk.

    They are separate so a cell can declare a robot whose log is missing, which
    is the shape the old `for r in ROBOTS` loop could not see at all: the
    missing robot simply dropped out of the expectation.
    """
    global fails
    robots = list(robots or TRIO)
    on_disk = list(build_robots or robots)
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events("hybrid", r) for r in on_disk}
        kw = (mutate(ev) if mutate else None) or {}
        man = manifest if manifest is not None else _manifest_for(robots)
        build(root, "cal", ev, manifest=man,
              off_manifest=_manifest_for(robots), robots=on_disk, **kw)
        rc, out = run_gate(root, "cal")
        if expect_clean:
            ok = rc == 0 and "HARD FAILURES: none" in out
            detail = "clean" if ok else (
                f"gate objected to a correct {len(robots)}-robot cell (rc={rc})"
                if rc == 1 else f"rc={rc}: a population was empty")
        else:
            ok = rc == expect_rc and re.search(expect_pat, out) is not None
            if ok and expect_rc == 3 and "HARD FAILURES: none" not in out:
                ok = False
            detail = ("caught" if ok else
                      f"did NOT match /{expect_pat}/ at rc={expect_rc} "
                      f"(got rc={rc})")
        if ok and extra_assert and not re.search(extra_assert, out):
            ok, detail = False, f"did NOT match /{extra_assert}/"
        print(f"  {'PASS' if ok else 'FAIL'}  {label}: {detail}")
        if not ok:
            fails += 1
            for ln in out.splitlines():
                if "check 3" in ln or "HARD" in ln or "run_start rows" in ln:
                    print(f"           | {ln}")
    finally:
        shutil.rmtree(root, ignore_errors=True)


# (a) the happy path, and the population count is the assertion that matters:
# 6 run_start rows is three robots across two cells. Two would mean the third
# robot was never opened, which is exactly what the gate used to do while
# printing CLEAN.
roster_case("a clean three-robot campaign scores CLEAN", None,
            expect_clean=True,
            extra_assert=r"check 3b  run_start rows\s+6")


# (b) THE case. A defect in the third robot ONLY. Under the old loop this cell
# scored CLEAN, because husky's log was never read.
def _stale_rev_on_third(ev):
    for e in ev["husky"]:
        if e.get("event") == "run_start":
            e["params"]["git_rev"] = "deadbee"


roster_case("a stale binary in the THIRD robot is caught",
            r"cal_hybrid_seed1/husky: check 3b — JSONL git_rev=deadbee",
            mutate=lambda ev: _stale_rev_on_third(ev))

# (c) a declared robot with no log at all. The message must name the roster and
# where it came from, because "no events" against a silently-assumed pair is
# how a renamed team looks like corrupt data.
roster_case("a declared robot whose log is missing is a hard failure",
            r"cal_hybrid_seed1/husky: no events \(roster from manifest "
            r"robots=: atlas,bestla,husky\)",
            build_robots=ROBOTS)

# (d) check 3k. The launcher writes `robots=`, the node writes
# team_robot_names; they are independent, so a disagreement means one of them
# describes a team that did not run and every per-robot check is iterating the
# wrong set whichever one is believed.
roster_case("the manifest's two rosters disagreeing is a hard failure",
            r"check 3k — the manifest's two rosters disagree",
            manifest=re.sub(r"^team_robot_names=.*$",
                            "team_robot_names=" + json.dumps(list(ROBOTS)),
                            _manifest_for(TRIO), flags=re.M))

# (e) a manifest naming no team at all. This is UNRESOLVED and not a pass: the
# roster then comes from the logs that exist, and a robot whose log is missing
# entirely drops out of the expectation instead of failing (c).
roster_case("a manifest naming no team is UNRESOLVED, not a silent pass",
            r"manifest names no team, so the roster was read off the "
            r"\*\.events\.jsonl files present",
            manifest=re.sub(r"^(robots|team_robot_names)=.*$\n", "", MANIFEST,
                            flags=re.M),
            expect_rc=3)

print("\n=== check 2: a REPORT-ONLY gate must not be able to condemn a cell ===")
# The launcher marks a cell SUSPECT on any UNRUN line in comms_gates.txt,
# whatever gate wrote it, and check 2 hard-fails non-CLEAN verdicts. These cases
# pin the exception for report-only gates like map_agree.
# (notes: g8cal-check2-report-only)
SUSPECT_MANIFEST = re.sub(r"^run_gates_verdict=.*$",
                          "run_gates_verdict=SUSPECT", MANIFEST, flags=re.M)
INVALID_MANIFEST = re.sub(r"^run_gates_verdict=.*$",
                          "run_gates_verdict=INVALID", MANIFEST, flags=re.M)
GATES_CLEAN = "PASS\tbringup\tall topics live\n"

# Check 2's exception is keyed on a gate name, so REPORT_ONLY_GATES and
# map_agreement.py's gate-name default are read back out of their files, not
# restated; a rename on either side fails here.
# (notes: g8cal-report-only-names-pinned)
_rog = re.search(r"REPORT_ONLY_GATES\s*=\s*frozenset\(\{([^}]*)\}\)",
                 open(GATE).read())
if not _rog:
    sys.exit("FATAL: gate_g8.py's REPORT_ONLY_GATES is no longer readable, so "
             "check 2's report-only exception is unpinned. Fix the pattern "
             "above rather than hardcoding the set.")
REPORT_ONLY_GATES = frozenset(
    t.strip().strip('"\'') for t in _rog.group(1).split(",") if t.strip())
_man_name = re.search(r'add_argument\(\s*"--name",\s*default="([^"]+)"',
                      open(os.path.join(HERE, "map_agreement.py")).read())
if not _man_name:
    sys.exit("FATAL: map_agreement.py's --name default is no longer readable, "
             "so nothing pins the token check 2 forgives to the gate that "
             "writes it.")
if _man_name.group(1) not in REPORT_ONLY_GATES:
    sys.exit(f"FATAL: map_agreement.py writes gate name "
             f"{_man_name.group(1)!r} but gate_g8.py's REPORT_ONLY_GATES is "
             f"{sorted(REPORT_ONLY_GATES)}. The exception check 2 makes for a "
             f"report-only gate would never fire, and every N>=3 cell would be "
             f"hard-failed for an informational line again.")


def gates_case(label, expect_pat, gates_text, manifest=None, expect_rc=1):
    """One fixture whose treated cell carries a planted comms_gates.txt."""
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events("hybrid", r) for r in ROBOTS}
        # off_manifest is the CLEAN default on purpose, as in radius_case: the
        # planted verdict belongs to the treated cell, and a control cell
        # carrying it too would fire check 2 twice and let a case "pass" on the
        # wrong cell's failure.
        build(root, "cal", ev, manifest=manifest or SUSPECT_MANIFEST,
              off_manifest=MANIFEST,
              extra_files={"comms_gates.txt": gates_text},
              off_extra_files={"comms_gates.txt": GATES_CLEAN})
        rc, out = run_gate(root, "cal")
        ok = rc == expect_rc and re.search(expect_pat, out) is not None
        if ok and expect_rc == 3 and "HARD FAILURES: none" not in out:
            ok = False
        detail = ("as designed" if ok else
                  f"did NOT match /{expect_pat}/ at rc={expect_rc} "
                  f"(got rc={rc})")
        print(f"  {'PASS' if ok else 'FAIL'}  {label}: {detail}")
        if not ok:
            fails += 1
            for ln in out.splitlines():
                if "check 2" in ln or "verdict" in ln or "HARD" in ln:
                    print(f"           | {ln}")
    finally:
        shutil.rmtree(root, ignore_errors=True)


gates_case("SUSPECT for a report-only gate alone is UNRESOLVED",
           r"check 2 — run_gates_verdict=SUSPECT, but every UNRUN line in "
           r"comms_gates\.txt names a report-only gate \(map_agree\)",
           GATES_CLEAN + "UNRUN\tmap_agree\texpected 2 planner_*.csv, found 3\n",
           expect_rc=3)
# The anti-vacuity cases. Three shapes that must still be hard failures, or the
# exception above has eaten the check.
gates_case("SUSPECT naming a real gate is still a hard failure",
           r"run_gates_verdict=SUSPECT \(UNRUN: link_outage\)",
           GATES_CLEAN + "UNRUN\tlink_outage\tno link samples in the window\n")
gates_case("SUSPECT mixing a real gate with the report-only one still fails",
           r"run_gates_verdict=SUSPECT \(UNRUN: link_outage, map_agree\)",
           GATES_CLEAN + "UNRUN\tmap_agree\tcannot compute\n"
           + "UNRUN\tlink_outage\tno link samples in the window\n")
# The "watcher never reported" path writes NO UNRUN line -- the missing line is
# the outage gate itself, i.e. the proof that the treatment happened, so absence
# of failures is emphatically not a pass. A fixture reading the file would find
# nothing to forgive, and the forgiving branch must not fire on an empty list.
gates_case("SUSPECT with no UNRUN lines at all is still a hard failure",
           r"run_gates_verdict=SUSPECT \(no UNRUN lines — the run-time watcher "
           r"never reported", GATES_CLEAN)
# And INVALID is never forgiven, whatever the file says.
gates_case("INVALID is a hard failure even if only a report-only gate is UNRUN",
           r"run_gates_verdict=INVALID",
           GATES_CLEAN + "UNRUN\tmap_agree\tcannot compute\n"
           + "FAIL\tlink_outage\tthe link never came back\n",
           manifest=INVALID_MANIFEST)

# ---------------------------------------------------------------------------
# check 3j — one radio regime per campaign, and it must be the declared one.
# ---------------------------------------------------------------------------
# These cases pin both halves of 3j, one regime per campaign and the declared
# one, and that the escape hatch for the declaration half does not relax
# uniformity. (notes: g8cal-3j-radio-regime)
OLD_RADIO = MANIFEST.replace("tree_attenuation_db=70.0\n",
                             "tree_attenuation_db=11.98\n") \
                    .replace("max_range_m=30.0\n", "")
assert "11.98" in OLD_RADIO and "max_range_m" not in OLD_RADIO, \
    "the pre-2026-09-03 fixture no longer differs from the shipped one"
HOT_RADIO = MANIFEST.replace("tx_power_dbm=30.0", "tx_power_dbm=160.0")
NO_HORIZON = MANIFEST.replace("max_range_m=30.0\n", "")


def regime_case(label, expect_pat, manifest, off_manifest=None, env=None,
                expect_clean=False):
    """One campaign whose two arms carry `manifest` / `off_manifest`."""
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events("hybrid", r, control_arms=CONTROL_ARMS)
              for r in ROBOTS}
        build(root, "cal", ev, manifest=manifest,
              off_manifest=off_manifest or manifest)
        rc, out = run_gate(root, "cal", env_extra=env)
        if expect_clean:
            ok = rc == 0 and "HARD FAILURES: none" in out
            detail = "clean" if ok else (
                f"gate objected to a declared regime (rc={rc})"
                if rc == 1 else f"rc={rc}: a population was empty")
        else:
            ok = rc == 1 and re.search(expect_pat, out) is not None
            detail = "caught" if ok else f"did NOT match /{expect_pat}/"
        print(f"  {'PASS' if ok else 'FAIL'}  {label}: {detail}")
        if not ok:
            fails += 1
            for ln in out.splitlines():
                if "check 3j" in ln or "HARD" in ln or "radio regime" in ln:
                    print(f"           | {ln}")
    finally:
        shutil.rmtree(root, ignore_errors=True)


print("\n=== check 3j: one radio regime per campaign ===")
regime_case("a campaign at the shipped radio scores clean", None, MANIFEST,
            expect_clean=True)
# The change itself, in the form a resume produces it: one arm banked before
# 2026-09-03 and one after, under a single tag.
regime_case("11.98 dB and 70 dB trunks in one campaign",
            r"check 3j — this campaign mixes 2 radio regimes",
            MANIFEST, off_manifest=OLD_RADIO)
regime_case("the 30 m horizon present in one arm only",
            r"check 3j — this campaign mixes 2 radio regimes",
            MANIFEST, off_manifest=NO_HORIZON)
# The old ideal-comms control ran tx_power_dbm=160, which is not a radio any
# robot carries. Mixed with a real arm it is the same defect as the trunks.
regime_case("tx_power 30 and 160 dBm in one campaign",
            r"check 3j — this campaign mixes 2 radio regimes",
            MANIFEST, off_manifest=HOT_RADIO)
# The declaration half: uniform, but not the regime the gate was told to score.
regime_case("a uniform pre-2026-09-03 campaign is not scored as current",
            r"check 3j — this campaign ran \[tree_attenuation_db=11\.98, "
            r"max_range_m=<absent>, tx_power_dbm=30\]",
            OLD_RADIO)
regime_case("...and scores clean once the old regime is declared", None,
            OLD_RADIO, env={"GATE_TREE_ATTEN": "11.98",
                            "GATE_MAX_RANGE": "none"},
            expect_clean=True)
# Formatting must not be a finding: the harness writes 30.0, a hand-edited or
# older manifest may say 30, and they are the same radio.
regime_case("30 and 30.0 are the same horizon", None,
            MANIFEST.replace("max_range_m=30.0", "max_range_m=30"),
            expect_clean=True)
# THE ANTI-VACUITY CASE. GATE_COMMS_REGIME=0 exists to stop the gate demanding
# a declaration; it must not also stop it noticing that two radios ran. If this
# ever passes as clean, the escape hatch has swallowed the check that has no
# escape.
regime_case("GATE_COMMS_REGIME=0 must NOT excuse a mixed campaign",
            r"check 3j — this campaign mixes 2 radio regimes",
            MANIFEST, off_manifest=OLD_RADIO,
            env={"GATE_COMMS_REGIME": "0"})
regime_case("GATE_COMMS_REGIME=0 does drop the declaration half", None,
            OLD_RADIO, env={"GATE_COMMS_REGIME": "0"}, expect_clean=True)

# ==================================================================
# The nav-binary witness (check 3, NAV_BIN_KEY)
# ==================================================================
# Nav binary hashes say which simple_nav_3d binary ran; git_simple_nav_3d moves
# without a rebuild. Required only when the campaign carries the key; cases test
# both too lax and too strict. (notes: g8cal-nav-binary-witness)
NAV_SHA = "cccccccccccccccc"
NAV_KEYS = ("sha256_simple_nav_planner_node", "sha256_simple_nav_costmap_node",
            "sha256_simple_nav_controller_node",
            "sha256_simple_nav_navigator_node")
NAV_BLOCK = "".join(f"{k}={NAV_SHA}\n" for k in NAV_KEYS)


def nav_case(label, expect_pat, treated=NAV_BLOCK, off=None, declare=NAV_SHA,
             expect_rc=1, expect_clean=False):
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        ev = {r: base_events("hybrid", r) for r in ROBOTS}
        build(root, "cal", ev, manifest=MANIFEST + treated,
              off_manifest=MANIFEST + (NAV_BLOCK if off is None else off))
        extra = ({"GATE_EXPECT_sha256_simple_nav_planner_node": declare}
                 if declare else None)
        rc, out = run_gate(root, "cal", env_extra=extra)
        ok = rc == (0 if expect_clean else expect_rc)
        if ok and expect_pat:
            ok = re.search(expect_pat, out) is not None
        if ok and expect_clean:
            ok = "simple_nav" not in out.split("HARD FAILURES")[-1]
        print(f"  {'PASS' if ok else 'FAIL'}  {label} (rc={rc})")
        if not ok:
            fails += 1
            print("    " + "\n    ".join(
                [l for l in out.splitlines() if "simple_nav" in l
                 or "REFUSING" in l][:8]))
    finally:
        shutil.rmtree(root, ignore_errors=True)


print("\n--- nav-binary witness (D2 provability) ---")
# Direction 1: too lax. A campaign that CAN answer must be made to.
nav_case("a campaign carrying the key REFUSES when it is undeclared",
         r"Undeclared: sha256_simple_nav_planner_node",
         declare=None, expect_rc=2)
# And the declaration must actually be compared -- the failure mode the four
# EXPECT keys already had once, where a value was pinned in source and the
# matching fixture made the check look alive while nothing was read.
nav_case("a DECLARED nav binary hash is compared, not ignored",
         r"sha256_simple_nav_planner_node=" + NAV_SHA + r" expected deadbeef",
         declare="deadbeef")
nav_case("a matching nav binary hash is clean", None, expect_clean=True)

# Direction 2: too strict. Cells banked before the key existed must still be
# gateable, or the key becomes the reason someone stops running the gate.
nav_case("a campaign that predates the key is NOT failed for lacking it",
         r"INFO: no cell carries sha256_simple_nav_planner_node",
         treated="", off="", declare=None, expect_clean=True)
# ...but "not failed" must not mean "silent". If this INFO line ever stops
# printing, the gate is passing D2 campaigns it never checked, which is the
# state this whole key exists to end.
nav_case("...and says so out loud rather than silently passing",
         r"CANNOT tell whether simple_nav_3d was rebuilt",
         treated="", off="", declare=None, expect_clean=True)

# The split. EXPECT only carries the key when EVERY cell has it, so a
# half-and-half campaign is the case where the check could vanish for exactly
# the cells whose provenance is in doubt.
nav_case("a cell missing the key while its siblings have it is a hard failure",
         r"no sha256_simple_nav_planner_node in the manifest, but 1/2 sibling",
         treated="", off=NAV_BLOCK)
# A partial install: the four hashes disagree. Needs no declaration to detect,
# so it must fire even on the undeclared path -- and must NOT be swallowed by
# the refusal, which would report the wrong fault.
nav_case("a sibling nav executable reading 'missing' is a hard failure",
         r"sha256_simple_nav_navigator_node=missing",
         treated=NAV_BLOCK.replace(
             f"sha256_simple_nav_navigator_node={NAV_SHA}",
             "sha256_simple_nav_navigator_node=missing"))

# ==================================================================
# check 3l -- the sensor and candidate configuration
# ==================================================================
# A params-file change moves no git rev or binary hash, so these cases pin check
# 3l, which compares the run_start sensor and candidate params to catch a stale
# installed yaml. (notes: g8cal-3l-stale-yaml)


def cfg_case(label, expect_pat, treated_over=None, off_over=None,
             expect_rc=1, expect_clean=False, manifest=None,
             off_manifest=None, treated_only_first=False):
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        def _ev(arm, over, only_first=False):
            out = {}
            for r in ROBOTS:
                e = base_events(arm, r)
                if over and not (only_first and r != ROBOTS[0]):
                    for row in e:
                        if row.get("event") == "run_start":
                            for k, v in over.items():
                                if v is None:
                                    row["params"].pop(k, None)
                                else:
                                    row["params"][k] = v
                out[r] = e
            return out
        build(root, "cal", _ev("hybrid", treated_over, treated_only_first),
              off_events=_ev("off", off_over),
              manifest=manifest or MANIFEST,
              off_manifest=off_manifest or manifest or MANIFEST)
        rc, out = run_gate(root, "cal")
        ok = rc == (0 if expect_clean else expect_rc)
        if ok and expect_pat:
            ok = re.search(expect_pat, out) is not None
        print(f"  {'PASS' if ok else 'FAIL'}  {label} (rc={rc})")
        if not ok:
            fails += 1
            print("    " + "\n    ".join(
                [l for l in out.splitlines() if "3l" in l][:6]) or "    (no 3l line)")
    finally:
        shutil.rmtree(root, ignore_errors=True)


print("\n--- check 3l: sensor / candidate configuration ---")
# The happy path FIRST, because a check that has never been seen to pass on a
# correct campaign is a check nobody trusts when it goes red.
cfg_case("a uniform configuration is clean and is PRINTED",
         r"configuration \(check 3l.*\n(?:.*\n)*?  fov_is_omnidirectional=true",
         expect_clean=True)
# Format is not a finding: 96 and 96.0 are the same sensor, so values are
# compared normalised, not as serialised text.
# (notes: g8cal-3l-format-not-finding)
cfg_case("int vs float spelling of the same value is not a finding",
         None, expect_clean=True,
         off_over={"fov_h_rays": 96, "fov_v_rays": 8,
                   "fov_max_range": 20, "candidate_n_yaw": 1})
# The converse, and the reason cfg_val tests bool BEFORE int: in Python bool is
# a subclass of int, so a value-ordered normaliser renders True as 1.0 and a
# flag becomes indistinguishable from a count of one. A number is not a flag.
cfg_case("a numeric 1 is still distinct from the flag true",
         r"check 3l — fov_is_omnidirectional is not constant",
         off_over={"fov_is_omnidirectional": 1})
# The defect D1 actually has: one cell built against a stale installed yaml.
cfg_case("a stale fov_h_rays on one arm is a hard failure",
         r"check 3l — fov_h_rays is not constant",
         off_over={"fov_h_rays": 16.0})
cfg_case("a stale fov_hfov -- the flag that D1 turns on -- is a hard failure",
         r"check 3l — fov_is_omnidirectional is not constant",
         off_over={"fov_hfov": 1.047, "fov_is_omnidirectional": False})
# candidate_enable_polar is in the HARD set, because n_yaw cannot be read
# without it: a campaign where it differs is a campaign where candidate_n_yaw=1
# means two different things.
cfg_case("candidate_enable_polar differing between arms is a hard failure",
         r"check 3l — candidate_enable_polar is not constant",
         off_over={"candidate_enable_polar": True})

# The SOFT half. cr2 varied the claim radius by arm on purpose, so this must
# NOT be a hard failure -- a gate that fails a legitimate design is a gate that
# gets switched off. rc=3 is "unresolved", which is not a pass either.
cfg_case("a claim radius that differs BETWEEN arms is unresolved, not failed",
         r"check 3l — coord_claim_radius_m differs BETWEEN arms",
         off_over={"coord_claim_radius_m": 20.0}, expect_rc=3)
# Within one arm the parameter must still be constant. treated_only_first
# overrides one robot only; overriding both would make this a between-arm split
# and test the soft branch instead. (notes: g8cal-3l-within-arm)
cfg_case("...but WITHIN one arm it is still a hard failure",
         r"It varies WITHIN arm\(s\) \['hybrid'\]",
         treated_over={"coord_claim_radius_m": 20.0},
         treated_only_first=True, expect_rc=1)

# Absence is not a pass. A banked campaign predating the keys must still gate,
# and must say out loud that the configuration was not checked.
cfg_case("a campaign predating the keys is not failed, and says so",
         r"fov_is_omnidirectional=<not recorded>",
         treated_over={k: None for k in
                       ("fov_hfov", "fov_vfov", "fov_h_rays", "fov_v_rays",
                        "fov_min_range", "fov_max_range",
                        "fov_is_omnidirectional", "candidate_n_yaw",
                        "candidate_enable_polar", "coord_claim_radius_m")},
         off_over={k: None for k in
                   ("fov_hfov", "fov_vfov", "fov_h_rays", "fov_v_rays",
                    "fov_min_range", "fov_max_range",
                    "fov_is_omnidirectional", "candidate_n_yaw",
                    "candidate_enable_polar", "coord_claim_radius_m")},
         expect_clean=True)

# The params file itself. Within an arm it is a constant; between arms it may
# differ, because that is how a yaml-only knob is varied per arm inside one
# invocation ([[per-cell-knobs-ride-the-arm-suffix]]).
def params_hash_case(label, hybrid_hashes, off_hashes, expect_pat,
                     expect_clean=False, expect_rc=1):
    """Two cells per arm, so within-arm and between-arm are distinguishable."""
    global fails
    root = tempfile.mkdtemp(prefix="gatecal_")
    try:
        for arm, hashes in (("hybrid", hybrid_hashes), ("off", off_hashes)):
            for i, h in enumerate(hashes, start=1):
                _cell(root, "cal", arm,
                      {r: base_events(arm, r) for r in ROBOTS},
                      PLANNER_LOG if arm == "hybrid" else OFF_PLANNER_LOG,
                      CSV,
                      MANIFEST.replace(f"sha256_shared_params={PARAMS_SHA}",
                                       f"sha256_shared_params={h}"),
                      seed=i)
        rc, out = run_gate(root, "cal", cells_per_arm="2")
        ok = rc == (0 if expect_clean else expect_rc)
        if ok and expect_pat:
            ok = re.search(expect_pat, out) is not None
        print(f"  {'PASS' if ok else 'FAIL'}  {label} (rc={rc})")
        if not ok:
            fails += 1
            print("    " + "\n    ".join(
                [l for l in out.splitlines()
                 if "3l" in l or "shared_params" in l][:6]))
    finally:
        shutil.rmtree(root, ignore_errors=True)


# Allowed: a yaml-only knob varied per arm inside one invocation gives each arm
# its own params file ([[per-cell-knobs-ride-the-arm-suffix]]). Failing this
# would make the gate reject the mechanism the campaigns use.
params_hash_case("a params hash differing BETWEEN arms is allowed",
                 [PARAMS_SHA, PARAMS_SHA], ["eeeeeeeeeeeeeeee"] * 2,
                 None, expect_clean=True)
# Never a design: two cells of ONE arm built from different params files.
params_hash_case("a params hash differing WITHIN an arm is a hard failure",
                 [PARAMS_SHA, "eeeeeeeeeeeeeeee"], [PARAMS_SHA] * 2,
                 r"sha256_shared_params is not constant within arm\(s\) "
                 r"\['hybrid'\]")

def _event_field_writers():
    """Every event field name experiment_log.cpp emits, or an error string.

    The writers are ExperimentLog::str/num/integer/boolean/text, all called with
    a literal name. Returns (set, None) or (set(), reason) -- never a silently
    empty set, because an empty set would make every fixture key look invented
    at once and a rename of the helpers would read as a fixture defect.
    """
    src_path = os.path.join(HERE, os.pardir, "explo_planner", "src",
                            "experiment_log.cpp")
    if not os.path.exists(src_path):
        return set(), f"experiment_log.cpp not found at {src_path}"
    src = open(src_path, errors="replace").read()
    found = set(re.findall(r'\b(?:str|num|integer|boolean|text)\("([^"]+)"',
                           src))
    if len(found) < 100:
        return set(), (f"the field-writer probe found only {len(found)} names "
                       f"in experiment_log.cpp; the probe, not the fixture, is "
                       f"broken")
    return found, None


def audit_params_against_writers():
    """Every fixture run_start param must be a name the node actually writes.

    audit_fixture_against_real_cell() above compares TOP-LEVEL event keys, so
    it cannot see inside params -- which is where the defect that motivated
    this whole file lived (git_rev was placed at the top level of run_start
    because that is where the gate looked for it, and the two errors cancelled
    inside the harness while printing ALL PASS).

    Nor can that audit help for a NEW generation: check 3l's twelve params are
    emitted only by the generation-9 binary, so no banked cell contains them
    and "subset of a real cell" would reject them for being new rather than for
    being wrong. The C++ writers are the authority, and they are readable now.
    """
    global fails
    src_path = os.path.join(HERE, os.pardir, "explo_planner", "src",
                            "explo_planner_node.cpp")
    if not os.path.exists(src_path):
        print(f"  UNRESOLVED  node source not found at {src_path}; fixture "
              f"params could not be checked against any writer")
        fails += 1
        return
    src = open(src_path, errors="replace").read()
    writers = set(re.findall(r'addParam(?:Str|Num|Bool)\("([^"]+)"', src))
    # A probe that finds nothing must not report a pass. If the regex ever
    # stops matching -- a rename, a macro, a reformat -- `invented` becomes
    # every key at once rather than silently becoming empty, but the count is
    # the thing that says the probe is alive, so assert it directly.
    if len(writers) < 50:
        print(f"  FAIL  addParam* probe found only {len(writers)} writers in "
              f"the node; the probe, not the fixture, is broken")
        fails += 1
        return
    invented = sorted(k for k in PARAMS if k not in writers)
    print(f"  {'PASS' if not invented else 'FAIL'}  all {len(PARAMS)} fixture "
          f"run_start params are emitted by the node "
          f"({len(writers)} addParam* writers)")
    if invented:
        fails += 1
        print(f"           | fixture invents {invented}, which no addParam* "
              f"call in explo_planner_node.cpp emits")


def audit_schema_pins_agree():
    """The schema pin, checked against the header instead of against prose.

    THIS IS THE THIRD TIME. The pin drifted 5->6 (caught by hand), 6->7 (caught
    by hand), and 7->8 (NOT caught — found by an adversarial review, after the
    comment at the top of this file had already written down the exact failure
    and asked future readers to prevent it). Two of three is the measured
    reliability of that request, so it is replaced here by something that fails.

    There are THREE copies of one number and they are not symmetric:

      * experiment_log.hpp `kSchemaVersion` is the TRUTH — the binary stamps it
        into every run_start event, and nothing else votes.
      * gate_g8.py's SCHEMA_VERSION default is what production scores with.
        Stale, it hard-fails every cell of a new-generation campaign at check
        3d, which is loud but arrives after the campaign has run.
      * THIS file's SCHEMA_VERSION is the dangerous one, because it is exported
        to the gate as GATE_SCHEMA_VERSION. Stale, it overrides the gate's own
        pin, so the calibration agrees with itself on a schema no binary writes
        and prints ALL PASS — the gate's loud failure is precisely what this
        file suppresses.

    Both derived pins are therefore checked against the header, not against
    each other; agreeing with each other is the state they were already in.
    """
    global fails
    hdr = os.path.join(HERE, os.pardir, "explo_planner", "include",
                       "explo_planner", "experiment_log.hpp")
    if not os.path.exists(hdr):
        print(f"  UNRESOLVED  experiment_log.hpp not found at {hdr}; the "
              f"schema pin could not be checked against the binary's truth")
        fails += 1
        return
    m = re.search(r"kSchemaVersion\s*=\s*(\d+)",
                  open(hdr, errors="replace").read())
    if not m:
        # The probe going quiet is itself a failure, for the same reason the
        # addParam* writer count is asserted above: a regex that matches
        # nothing reports no disagreement, which reads exactly like agreement.
        print("  FAIL  kSchemaVersion not found in experiment_log.hpp; the "
              "probe is broken, not the pin")
        fails += 1
        return
    truth = int(m.group(1))

    gate_src = open(GATE, errors="replace").read()
    gm = re.search(r'SCHEMA_VERSION\s*=\s*int\(os\.environ\.get\(\s*'
                   r'"GATE_SCHEMA_VERSION",\s*"(\d+)"\s*\)\)', gate_src)
    if not gm:
        print("  FAIL  could not read gate_g8.py's SCHEMA_VERSION default; "
              "the probe is broken, not the pin")
        fails += 1
        return
    gate_pin = int(gm.group(1))

    bad = []
    if SCHEMA_VERSION != truth:
        bad.append(f"gate_g8_calib.py pins {SCHEMA_VERSION} (and EXPORTS it, "
                   f"so the whole calibration would run on a dead schema)")
    if gate_pin != truth:
        bad.append(f"gate_g8.py defaults to {gate_pin}, so production would "
                   f"hard-fail every schema-{truth} cell at check 3d")
    if bad:
        print(f"  FAIL  experiment_log.hpp stamps kSchemaVersion={truth}, but "
              f"{'; '.join(bad)}")
        fails += 1
        return
    print(f"  PASS  schema pin {truth} agrees across experiment_log.hpp, "
          f"gate_g8.py and this calibrator")


audit_schema_pins_agree()
audit_fixture_against_real_cell()
audit_params_against_writers()

print(f"\n{'ALL PASS' if fails == 0 else str(fails) + ' FAILURE(S)'}")
sys.exit(1 if fails else 0)
