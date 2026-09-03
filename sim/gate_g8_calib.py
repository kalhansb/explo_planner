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

# WHICH ARMS ARE CONTROLS, read out of the gate rather than restated here.
#
# The fixture writes reconnect_enabled=False and omits the reconnect_dispatch
# row for a control arm, and gate check 3e compares exactly that against the arm
# name. So if the two files ever disagree about what "control" means, every case
# in this file exercises a cell the gate refuses for a reason no case mentions:
# the expect_clean cases go red and the planted-defect cases still "pass",
# having caught the disagreement instead of their defect.
#
# That drift had already happened. This file said ("off", "mtare_off") while the
# gate's default had become "off" alone, so an mtare_off fixture would have been
# built manoeuvre-less and then scored as a treated arm with its dispatch
# missing. Reading the literal back out is the same move campaign_guard_calib.sh
# makes for the launcher's LINK_GATE default, and for the same reason: nothing
# else links the two files.
_m = re.search(r'os\.environ\.get\(\s*"GATE_CONTROL_ARMS"\s*,\s*"([^"]*)"\s*\)',
               open(GATE).read())
if not _m:
    sys.exit("FATAL: gate_g8.py's GATE_CONTROL_ARMS default is no longer "
             "readable, so the fixture's notion of a control arm is unpinned. "
             "Fix the pattern above rather than hardcoding the set.")
CONTROL_ARMS = frozenset(s.strip() for s in _m.group(1).split(",") if s.strip())

MANIFEST = f"""run_end_reason=all_done
run_gates_verdict=CLEAN
run_end_t_sim=812.4
git_explo_planner={REV}
git_simple_nav_3d=c9f83a7
git_scovox=078d3f7
sha256_explo_planner_node={SHA}
"""

SCHEMA_VERSION = 4
# The schema the injected regression below downgrades TO. One less than the pin,
# always: the case has to be a version the gate must reject, and hard-coding a 2
# after the pin moved to 4 would have kept passing while testing a two-step
# mismatch instead of the off-by-one a real generation mix produces.
PREV_SCHEMA_VERSION = SCHEMA_VERSION - 1
# Schemas whose EVENT SHAPES the fixture may be audited against, newest first.
# Not the same question as the pin: v4 adds five default-off event kinds and no
# field to any existing one, so a banked schema-3 cell is still an exact shape
# reference for all sixteen legacy kinds. Keeping 3 here is what stops
# audit_fixture_against_real_cell() from going UNRESOLVED — permanently silent —
# on the very schema bump it most needs to be watching.
#
# Drop a version from this tuple the moment a bump CHANGES an existing event's
# fields, because then its cells stop being a valid reference.
AUDIT_SCHEMAS = (4, 3)

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
    "reconnect_enabled": True,
    "rendezvous_enabled": True,   # deprecated alias; the node stamps both
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
    # Check 3g. The node emits all five UNCONDITIONALLY, which is what lets the
    # gate treat their absence as "this cell predates the stack" rather than as
    # a default — so the fixture has to emit them unconditionally too, at the
    # off values, and let build_events flip them for an m-tare arm. A fixture
    # that carried them only on the treated cells would have made 3g's control
    # direction untestable, and the control direction is the one that decides
    # whether the comparison holds.
    #
    # rendezvous_schedule_enable is the fifth, and it does double duty: check
    # 3h reads its PRESENCE as the binary generation, so a fixture that omitted
    # it would silently date every synthetic cell to before P5 and take the
    # per-arm half of 3g out of service entirely.
    "cell_world_enable": False,
    "team_world_hz": 0.0,
    "global_alloc_enable": False,
    "reconnect_gate": "silence",
    "rendezvous_schedule_enable": False,
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
    # Every arm but the control reaches the manoeuvre. Was `arm == "hybrid"`,
    # which made the fixture unable to express any OTHER treated arm: an
    # mtare_hybrid cell would have been written with reconnect_enabled=False
    # and check 3e would have "caught" a defect the fixture invented.
    # `control_arms` overrides the default parsed out of gate_g8.py, for the
    # cases that declare their own GATE_CONTROL_ARMS. Without it a fixture for
    # a newly-declared control arm — mtare_off, the §3.6.1 factorial's own
    # control — would be written with reconnect_enabled=True while the gate
    # was told to expect False, so check 3e would hard-fail alongside the
    # planted defect and the case would "pass" on a failure it did not plant.
    # Both spellings, exactly as the node stamps them since the 2026-09-03
    # rename. A fixture carrying only one would leave the gate's fallback
    # untested on the shape it will actually meet; the two cases below then
    # take each spelling away in turn, so neither read can rot silently.
    _rdv = arm not in (CONTROL_ARMS if control_arms is None else control_arms)
    params["reconnect_enabled"] = _rdv
    params["rendezvous_enabled"] = _rdv
    # Each m-tare arm ran ITS OWN stack; anything else ran none. This mirrors
    # the launcher's own per-token expansion rather than the node's OR,
    # deliberately: the OR is what check 3g exists to close, so a fixture built
    # from it could never express the half-treated cell.
    #
    # Per token and not a single "all on" branch, because the doc §3.6.1
    # factorial's whole point is that its four arms differ in the two reconnect
    # mechanisms while sharing P1-P3. A fixture that switched everything on for
    # any mtare_* name would build mtare_pursuit cells carrying an appointment
    # and mtare_off cells carrying the value gate — i.e. it would be unable to
    # express three of the four arms it is meant to calibrate the gate against,
    # and the cases below would all be testing mtare_hybrid under four names.
    if arm.startswith("mtare_"):
        params["cell_world_enable"] = True
        params["team_world_hz"] = 1.0
        params["global_alloc_enable"] = True
        params["reconnect_gate"] = \
            "silence" if arm == "mtare_off" else "info"
        params["rendezvous_schedule_enable"] = \
            arm in ("mtare_rendezvous", "mtare_hybrid")
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
    # Whether the manoeuvre ran is `reconnect_enabled`, not the module-level
    # control list: mtare_off is the factorial's own control and dispatches
    # nothing, so keying off CONTROL_ARMS here would have written it a dispatch
    # pair while `build()` gave it the reconnect-free planner log, and check 17
    # would hard-fail on a defect the fixture invented.
    if params["reconnect_enabled"]:
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
          off_console=True, off_manifest=None, console_text=None, live=True,
          treated_arm="hybrid"):
    """Write BOTH arms. Defects are planted in the hybrid cell.

    A one-armed fixture cannot exercise check 3e's off branch, and cannot
    exercise check 21 at all -- the campaign-shape check would have had no
    known-answer case whatsoever, which is exactly how the six inert guards in
    [[checks-that-stopped-checking]] got there.
    """
    d = _cell(root, tag, treated_arm, events_by_robot, planner_log, csv_text,
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
    # GATE_SCHEMA_VERSION is pinned, not inherited. An operator who exported it
    # to re-gate a banked campaign would otherwise run this calibration against
    # a gate expecting a different schema from the one the fixture writes, and
    # every case would fail on check 3d — a red harness that says nothing about
    # the check it claims to be calibrating.
    #
    # GATE_ARMS and GATE_CONTROL_ARMS are pinned for exactly the same reason,
    # and the reason is not hypothetical for them: scoring the live campaign
    # requires `export GATE_ARMS=mtare_hybrid,off`, and running this
    # calibration afterwards in that same shell would hand every one of the
    # hybrid/off fixtures below an expectation naming an arm they do not
    # build. Check 21 would then hard-fail all of them — every expect_clean
    # case reporting FAIL, and every planted-defect case still "passing" while
    # the gate failed for a reason having nothing to do with its defect. The
    # harness would go red across the board and be unable to say why.
    # env_extra still overrides, which is how the mtare cases declare theirs.
    env = dict(os.environ, GATE_ROOT=root,
               GATE_EXPECT_git_explo_planner=REV,
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
# The master switch has two spellings after the 2026-09-03 rename, and check 3e
# reads whichever is present. Each of the next two cases DELETES one spelling
# and disables the other, so a gate that stopped reading either one fails here
# instead of passing a control run in a treated directory. The legacy case is
# not hypothetical: every cell up to and including cr5 carries only that key.
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
# Every arm assertion in this gate used to be spelled `arm == "hybrid"` or the
# literal pair ("hybrid", "off"). That is not a check of the campaign, it is a
# check of one campaign's spelling, and the three cases below are the
# known-answer set for the generalisation.
#
# The first is the one that matters: an off-vs-mtare_hybrid campaign is the
# same experiment with a different treatment, and before this it hard-failed on
# a correct run — check 3e demanded reconnect_enabled=False of every arm that
# was not literally "hybrid", and check 21 called mtare_hybrid unexpected.


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
# The node stamps `mtare_` when `global_alloc_enable_ || reconnect_gate_info_
# || rendezvous_schedule_enable_`. That is an OR, so the arm NAME proves at
# most one of P3, P4 and P5 was live, and check 3e — which compares the name
# against the stamp it was derived from — is satisfied by a cell running a
# third of the treatment. Worse, cell_world_enable and team_world_hz rename
# NOTHING, so a cell that ran no census or no exchange keeps the mtare_hybrid
# name, the mtare_hybrid stamp, and every other check.
#
# Since P5 the expectation is a per-arm VECTOR rather than one boolean, because
# the §3.6.1 factorial's four arms deliberately differ in two of the five
# features. That makes two new ways to be wrong, and both are planted below: a
# treated arm missing a feature it is defined to carry (the old failure), and a
# treated arm CARRYING one it is defined not to — an appointment in the
# chase-only cell, which would confound the very contrast the campaign exists
# to measure and which the old all-on rule could not even express.
#
# Below: every way a treated cell can be less (or more) than its arm, the one
# way a control cell can be treated, the pre-stack binary whose arm cannot be
# certified in either direction, and the two generation cases 3h owns. Each
# planted defect leaves every OTHER check passing, which is what makes 3g the
# only thing standing between the campaign and a contrast that is not the
# contrast it reports.


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
# The P5 analogue, and the same blind spot: with the allocator on, the arm is
# still stamped mtare_hybrid, so nothing but 3g can tell that hybrid fell back
# to the last-contact midpoint instead of to an agreed cell. Post-P5 that is a
# different arm wearing the same name — precisely what check 3h refuses to pool
# ACROSS binaries, refused here WITHIN one.
_g3("an mtare_hybrid cell with no appointment armed: caught by 3g alone",
    r"check 3g — arm=mtare_hybrid but rendezvous_schedule_enable=False",
    treated={"rendezvous_schedule_enable": False})

# THE FAILURE THE OLD ALL-ON RULE COULD NOT EXPRESS: a treated arm carrying a
# feature its own definition excludes. mtare_pursuit IS the appointment-off
# cell of the 2x2; an appointment in it makes it a second mtare_hybrid arm, and
# the factorial then estimates the chase effect from two identical columns.
# Only reachable now that the expectation is a per-arm vector.
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

# Absence is not a default. The node emits all four unconditionally, so a cell
# missing them came from a binary predating the stack and its arm cannot be
# certified either way -- the one honest verdict is a refusal, not a pass.
#
# All FIVE keys are stripped, from both arms, so the campaign is uniformly
# pre-stack. Stripping only the four would additionally trip check 3h (one
# generation per campaign) and the case would then be passing on a defect it
# did not plant.
PRE_P5_KEY = "rendezvous_schedule_enable"
STACK_KEYS = ("cell_world_enable", "team_world_hz",
              "global_alloc_enable", "reconnect_gate", PRE_P5_KEY)


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
# P5 changed what `mtare_hybrid` MEANS: before it, hybrid's fallback
# destination was the last-contact midpoint; after it, the agreed cell. The arm
# token is identical on both sides of that line, so a campaign holding cells
# from both is one arm run twice with two different treatments in it -- and
# every downstream script keys off the token.
#
# The discriminator is the PRESENCE of rendezvous_schedule_enable, which a P5+
# binary emits unconditionally and an earlier one cannot emit at all. Three
# known-answer cases: the mix is refused, and neither pure generation is.

# (a) mixed: pre-P5 control cells, P5 treated cells. Every other check passes.
root = tempfile.mkdtemp(prefix="gatecal_")
try:
    ev = {r: base_events("mtare_hybrid", r) for r in ROBOTS}
    off = {r: _strip_keys(base_events("off", r), (PRE_P5_KEY,))
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
    ev = {r: _strip_keys(base_events("mtare_hybrid", r), (PRE_P5_KEY,))
          for r in ROBOTS}
    off = {r: _strip_keys(base_events("off", r), (PRE_P5_KEY,))
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
    ev = {r: _strip_keys(base_events("mtare_pursuit", r), (PRE_P5_KEY,))
          for r in ROBOTS}
    off = {r: _strip_keys(base_events("off", r), (PRE_P5_KEY,))
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
          and re.search(r"m-tare features certified ON\s+0\s+<- no such arm",
                        out)
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
    """
    global fails
    root = os.environ.get("GATE_CALIB_REAL_ROOT", "/home/kalhan/hmr_campaign")
    if not os.path.isdir(root):
        print(f"  UNRESOLVED  no campaign root at {root}; fixture shapes were "
              f"NOT compared against real data")
        return
    real = {}
    real_schema = None
    for cell in sorted(os.listdir(root)):
        d = os.path.join(root, cell)
        if not os.path.isdir(d):
            continue
        for r in ROBOTS:
            p = os.path.join(d, f"{r}.events.jsonl")
            if not os.path.exists(p):
                continue
            found = {}
            schema = None
            for ln in open(p, errors="replace"):
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                if e.get("event") == "run_start":
                    schema = e.get("schema_version")
                    if schema not in AUDIT_SCHEMAS:
                        found = {}
                        break            # shapes not comparable, skip the file
                found.setdefault(e.get("event"), set()).update(e.keys())
            if found:
                real, real_schema = found, schema
                break
        if real:
            break
    if not real:
        print(f"  UNRESOLVED  no cell at schema "
              f"{'/'.join(str(v) for v in AUDIT_SCHEMAS)} found under {root}; "
              f"fixture shapes were NOT compared against real data")
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
    # Name the schema actually compared against. "PASS" alone would let a
    # reader assume the current one was available when it was not.
    print(f"  {'PASS' if not bad else 'FAIL'}  fixture keys are a subset of a "
          f"real schema-{real_schema} cell's for "
          f"{len(set(fixture) & set(real))} event type(s)")
    for b in bad:
        print(f"           | {b}")
    if bad:
        fails += 1


audit_fixture_against_real_cell()

print(f"\n{'ALL PASS' if fails == 0 else str(fails) + ' FAILURE(S)'}")
sys.exit(1 if fails else 0)
