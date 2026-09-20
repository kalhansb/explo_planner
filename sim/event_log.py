#!/usr/bin/env python3
"""Reader for the per-robot JSONL event stream written by explo_planner.

ONE definition of completion, for every consumer.

Before this existed, three scripts each computed a "completion time" and meant
different things by it, none of them saying which in its output:

  modes_compare.t_team   first crossing of unknown_fraction <= 0.55, absolute
                         sim, team = the later robot
  modes_compare.makespan the last CSV row, absolute sim, never clipped, so the
                         teardown tail is included
  comms_metrics.makespan run_end_t_sim from the manifest -- RUN-RELATIVE, and
                         inflated by the harness's done-grace and poll period

The gap between the first two is the reconnect manoeuvre, which is charged only
to the arms that reconnect: 90-845 s per run, measured. On p7modes it reverses
the headline. Rendezvous is 28.4 s FASTER than control on t_team and 185.2 s
SLOWER on makespan, from the same CSVs. Whichever number a table happened to
quote decided the direction of the result.

So completion is now read from the planner's own event log, where the planner
states it directly rather than having it inferred from the shape of a CSV:

  explore_done_sim_sec  absolute sim seconds of the LAST exploration_complete
                        on that robot -- when it stopped trying. Not the first:
                        a robot pulled into a reconnect manoeuvre and resuming
                        afterwards did not finish at its first declaration.
  team completion       the max over the robots, and None if ANY robot never
                        declared. A censored robot has no completion time, and
                        substituting the horizon for it turns a run that did not
                        finish into the fastest-looking run in its arm.

Mission return (schema v2, mission_return_enabled runs) adds a second pair of
endpoints on top -- ADDS, because the fields above keep their banked meaning
and banked and mission-return numbers are never pooled anyway:

  explore_latched_sim_sec  the last exploration_complete whose reason is
                        "coverage-latched". THE pre-registered secondary
                        endpoint (exploration finish). A step-budget or
                        barrier-gave-up declaration ends the run but is not a
                        finish, so it does not populate this field.
  t_mission             THE pre-registered primary endpoint (mission end).
                        max over robots of the mission_complete stamp,
                        REQUIRING result=="arrived" on every robot. A robot
                        that timed out or gave up homing has an ended run but
                        an unfinished mission: withheld, never imputed, same
                        as censoring above -- and visible in mission_results.

Sim time throughout. The wall clock is in the file too but the real-time factor
drifts 0.89 -> 0.76 within a single run, so wall seconds are not comparable
even against themselves.

Two grouping rules that are easy to get wrong and silent when you do:

  the arm         is run_arm(params), NOT params["reconnect_mode"]. The control
                  arm is not a reconnect_mode value, so a control run carries
                  reconnect_mode "hybrid" and grouping on that column deletes
                  the control by averaging it into hybrid. See run_arm.
  censored runs   are withheld, never imputed -- and withholding is itself a
                  bias, because the arms that manoeuvre most run out of horizon
                  most. arm_summary returns the censoring counts in the same
                  record as the mean so the two cannot be separated.
"""

import glob
import json
import os
import re
# Top-level, not in the __main__ block where the argv import lives: the schema
# warning in summarise_robot() writes to sys.stderr, and that function runs when
# this module is IMPORTED by another script — a path on which __main__ never
# executes. A block-scoped import would have made the newer-file warning raise
# NameError instead of warning.
import sys

# Two constants, not one, because they answer different questions.
#
# READER_SCHEMA is the schema this file was WRITTEN AGAINST — the vintage of the
# code. It is fixed and the CLI must not move it; it is what "newer than this
# reader" is measured against.
#
# MIN_SCHEMA is the lowest schema this INVOCATION will accept — a floor the
# operator can lower with --min-schema. The default must track the current
# generation rather than being permissive: scoring a generation-8 campaign
# should not quietly read a generation-6 cell that happens to sit in the same
# directory. Lowering it is legitimate for historical work; doing so silently is
# not, which is why it is a flag and not a default.
#
# They were the same variable until round 4, which made `--min-schema 2` warn
# that every current file was "newer than this reader's 2" and made
# `--min-schema 4` announce that raising the bar "may pool binary generations".
#
# v4 (the M-TARE evolution) is the first bump that adds ONLY new event kinds and
# changes no existing one, because every v4 mechanism ships default-off. A v4
# cell run at defaults is therefore readable by this file's v3 logic and pools
# legitimately with v3 cells — `--min-schema 3` is sound for such a mix, which
# is not something that could be said of any earlier bump. It stops being sound
# the moment a v4 arm is run with a mechanism ENABLED, and this file cannot tell
# the two apart from the version alone: that is equiv_gate.py's job, and mixing
# without it is the "don't pool across binary generations" mistake wearing a
# newer number. The default stays at the current schema so the pooling decision
# is always something an operator typed.
#
# v5 (2026-09-16) is NOT such a bump and `--min-schema 4` is NOT sound for a
# v4/v5 mix. The rendezvous appointment stopped being derived independently on
# each robot and became a pair exchanged over TeamWorld and committed by
# unanimity, so every rendezvous quantity in a v4 file was produced by a
# different mechanism than the same-named quantity in a v5 file — the numbers
# are comparable in type and in nothing else. `RendezvousAgreedEvent.excluded`
# was also removed outright. Leaving this at 4 would have been the quiet
# failure: `ver > READER_SCHEMA` only WARNS, so the floor is the only thing that
# refuses, and a floor one generation stale accepts the old generation in
# silence while the new one merely prints to stderr.
#
# v6 (2026-09-17) is the same kind of bump and the same reasoning applies, so
# this moved with the binary rather than one generation behind it. `t_meet_sec`
# CHANGED MEANING: it was a single absolute instant, and it is now the FIRST
# occurrence of a meeting that recurs every `interval_sec` for the rest of the
# mission. A v5 reader handed a v6 file gets a number of the right type, in the
# right units, that answers a different question — the failure this floor
# exists to refuse. `agreed_provisional` was added in the same change; a v5 file
# has no such column, so a mixed pool cannot tell a placeholder meeting cell
# from a tour-informed one.
#
# v7 (generation 19) is the THIRD consecutive bump driven by a meaning change
# with no field movement, and the constant had moved to 7 before this block said
# why. Two columns turned over:
#
#   `t_meet_sec` stopped being the first occurrence of a recurring meeting and
#   became this robot's own departure deadline — now plus a shared countdown,
#   computed locally. The v6 equality-across-robots test therefore FAILS BY
#   DESIGN on a healthy v7 cell, and `interval_sec` no longer generates it: the
#   recurrence is gone and nothing reads the interval for timing.
#
#   `rendezvous_outcome.outcome` now names what the barrier acted on
#   (teamComplete over live peers) rather than whole-team direct contact. On v6
#   N>=3 files that label is simply wrong — a relayed reunion was written down
#   as a no-show — and it cannot be repaired by re-reading, because the
#   barrier's own verdict was never in the file. `mutual` was added alongside to
#   carry the strict predicate, so v7 rows hold both facts.
#
# Do not pool v6 and v7 rendezvous rows. See ExperimentLog::kSchemaVersion for
# the authoritative list.
#
# v8 (generation 23, 2026-09-18) carves "unplaceable" out of what v7 wrote as
# "no-show". A robot that could not place the agreed cell on any grid it held
# was handed its OWN position as the meeting point, reached that goal on the
# next tick, and logged `no-show arrived=true` — peers blamed for a departure
# that never happened. On v8 those rows read `unplaceable arrived=false`. No
# column moved and both vintages parse, so a no-show tally pooled across the
# boundary is counting two different populations. Do not pool v7 and v8
# rendezvous outcomes either.
#
# v8 also adds run_end.midrun_attempts_used, and is behaviourally a different
# binary in ways with no schema surface at all (the peer-liveness predicate
# behind nearly every reconnection decision changed). Nothing here can detect
# that; the stamp is the only handle.
#
# v9 (generation 25, 2026-09-18) moves the arming floor behind `t_meet_sec`
# from now + rendezvous_depart_delay_sec to bare now. On v8 a healthy cell
# could write DIFFERENT t_meet_sec values for the same outage — the per-robot
# notice forked one N=3 cell across two occurrences — while on v9 rows for the
# same outage match whenever every robot armed before the agreed instant, and
# `t_meet_sec - t_now_sec` lands in [0, interval_sec) instead of
# [delay, delay + interval_sec). `rendezvous_depart_delay_sec` still appears in
# params but decides nothing. v9 also splits `arrived=false` outcome rows into
# two populations — a walker converted to the release barrier because the team
# settled under it (new state_change reason `return-team-settled`) vs one
# stranded while the team stayed apart — so do not pool v8 and v9 arrival
# rates.
READER_SCHEMA = 9
MIN_SCHEMA = READER_SCHEMA


class EventLogError(Exception):
    pass


def arm_from_dirname(run_dir):
    """Arm token from a cell directory name: g8r1_hybrid_seed101 -> "hybrid".

    A fallback for cells that cannot be PARSED at all. The normal arm comes from
    the run_start params, which requires a readable run_start — precisely what
    an excluded cell may not have. Without this fallback every excluded cell
    files under "?" in arm_summary, and the summary becomes unable to say
    whether exclusions are balanced across arms. That is the one question its
    own closing NOTE exists to answer, so losing it on the exclusion path
    defeats the check exactly when it matters.

    Same shape as modes_compare.RE_CELL: greedy up to the final _seed<N>, so
    multi-token arms ("hybrid_seek") survive intact.
    """
    m = re.search(r"_(.+)_seed\d+$", os.path.basename(run_dir.rstrip("/")))
    return m.group(1) if m else None


def reconnect_enabled(params, default=True):
    """Master switch for the reconnect subsystem, under either spelling.

    Renamed from `rendezvous_enabled` to `reconnect_enabled` on 2026-09-03: the
    old name read off a manifest as "the rendezvous arm is on" in three arms
    out of four, including pure pursuit, which cannot arm an appointment at
    all. The node stamps BOTH names now, but every cell up to and including
    cr5 carries only the old one, so both are read and the current name wins.
    """
    v = params.get("reconnect_enabled")
    if v is None:
        v = params.get("rendezvous_enabled")
    return default if v is None else bool(v)


def run_arm(params):
    """The ARM a run belongs to. NOT params["reconnect_mode"].

    The control arm is "no reconnection", which is not a reconnect_mode value:
    it is reconnect_enabled=false, and the harness still has to pass some mode
    alongside it (it passes "hybrid"). So a control run is stamped
    reconnect_mode "hybrid", and grouping on that column silently pools the
    control into the hybrid cell -- the hybrid mean becomes the average of
    treatment and control, and the control arm ceases to exist.

    The planner now stamps `arm` directly. The fallback reconstructs it from the
    two fields it was always split across, so logs written before that still
    group correctly rather than quietly mislabelling themselves.
    """
    arm = params.get("arm")
    if arm:
        return arm
    mode = params.get("reconnect_mode")
    if mode is None:
        return None
    return mode if reconnect_enabled(params) else "off"


def robot_logs(run_dir):
    """Every *.events.jsonl in a run directory, robot name -> path."""
    out = {}
    for p in sorted(glob.glob(os.path.join(run_dir, "*.events.jsonl"))):
        out[os.path.basename(p)[: -len(".events.jsonl")]] = p
    return out


def read_events(path):
    """Parse one robot's stream. Tolerates a truncated final line.

    A run killed mid-write leaves a partial last line; that is a truncation, not
    a corrupt file, and the events before it are still valid. Anything malformed
    EARLIER than the last line is a real corruption and raises.
    """
    evs, lines = [], open(path, errors="replace").read().splitlines()
    for i, ln in enumerate(lines):
        ln = ln.strip()
        if not ln:
            continue
        try:
            evs.append(json.loads(ln))
        except json.JSONDecodeError:
            if i == len(lines) - 1:
                break  # truncated tail
            raise EventLogError(f"{path}: malformed line {i + 1}")
    return evs


def summarise_robot(path):
    """One robot's run, as the fields the analysis actually asks for."""
    evs = read_events(path)
    if not evs:
        raise EventLogError(f"{path}: empty")
    start = next((e for e in evs if e["event"] == "run_start"), None)
    end = next((e for e in evs if e["event"] == "run_end"), None)
    if start is None:
        raise EventLogError(f"{path}: no run_start")

    # The version stamp, finally read by something.
    #
    # It was written from the start and consumed by nothing, which made the
    # "an analysis script can refuse a file it predates" rationale in
    # experiment_log.hpp aspirational rather than true, and left the argument
    # for voiding the generation-7 cells resting on a check that did not exist.
    #
    # Below MIN is a refusal. State the reason accurately, because the first
    # version of this message did not:
    #
    #   - It said the refused files are "the generation-7 cells". They are not.
    #     v1 spans 25 campaign tags and v2 spans 6 (g5smoke, g6pilot, mr0pilot,
    #     mr0smoke, mr1, mr1smoke) — many generations, and NOT the void g7 cells,
    #     which were moved out of the campaign root entirely.
    #   - It said reading a v2 file anyway "yields plausible wrong numbers"
    #     because of the `metrics_rows` -> `metrics_timer_rows` rename. That is
    #     the dangerous shape in general, but it is not a hazard for THIS
    #     reader: grep says no function here touches either that field or the
    #     removed `last_contact_age_sec`. The rename is checked where it is
    #     actually read — gate_g8.py check 19.
    #
    # So the honest justification is generation hygiene, not a parse hazard: a
    # generation-8 summary must not silently absorb pre-generation-8 cells,
    # because pooling across binary generations is the standing error this
    # project keeps making. That is a real reason to refuse BY DEFAULT, and a
    # bad reason to refuse ABSOLUTELY — hence --min-schema, which makes reading
    # the historical bank a deliberate, visible act rather than an impossible
    # one. Before this flag existed the guard refused 1004 of 1006 banked
    # robot-runs with no way to override, which is not a guard, it is an outage.
    #
    # ABOVE max is deliberately NOT a refusal. A future generation is more
    # likely to add fields than to move them, and a check that hard-fails
    # forward gets deleted the first time it is wrong — which is how a guard
    # stops guarding. Warn, keep going, and let the field-level reads fail if
    # they actually break.
    ver = start.get("schema_version")
    if ver is None:
        raise EventLogError(
            f"{path}: no schema_version in run_start — predates versioning, "
            f"and its field names cannot be trusted to mean what this script "
            f"assumes")
    if ver < MIN_SCHEMA:
        # Do NOT phrase this as "generation {MIN_SCHEMA}". The schema version
        # and the binary generation are different counters that happen to both
        # be small integers -- schema 3 belongs to generation 8 -- and naming
        # the wrong one in the error is how a reader ends up believing the void
        # generation-7 cells are the ones being refused. Say "schema".
        #
        # And say WHICH refusal this is. "Refused by default" is a lie once the
        # operator has raised the floor with --min-schema: they are then being
        # told the tool made a conservative choice on their behalf, when in fact
        # it is obeying an instruction they gave.
        if MIN_SCHEMA == READER_SCHEMA:
            raise EventLogError(
                f"{path}: schema_version {ver} < {MIN_SCHEMA}. Refused by "
                f"default so a schema-{MIN_SCHEMA} summary cannot silently pool "
                f"cells from an older binary generation. Pass --min-schema "
                f"{ver} to read it deliberately.")
        raise EventLogError(
            f"{path}: schema_version {ver} < the --min-schema {MIN_SCHEMA} you "
            f"asked for (this reader understands {READER_SCHEMA}).")
    # Compare against READER_SCHEMA, not MIN_SCHEMA. MIN_SCHEMA is a floor the
    # operator can lower; the reader's own vintage does not move with it. When
    # they were the same variable, `--min-schema 2` -- the documented way to
    # read banked cells -- made every CURRENT schema-3 file print "newer than
    # this reader's 2", which is false: the reader is not older, the floor is.
    if ver > READER_SCHEMA:
        print(f"warning: {path}: schema_version {ver} is newer than this "
              f"reader's {READER_SCHEMA}; unknown fields ignored",
              file=sys.stderr)

    completes = [e for e in evs if e["event"] == "exploration_complete"]
    # The planner states this in run_end; recomputing it from the event list is
    # the cross-check, and a disagreement means one of the two is wrong and the
    # run should not be quietly averaged into an arm.
    done = completes[-1]["t_sim_sec"] if completes else None
    if end is not None and end.get("explore_done_sim_sec") is not None:
        stated = end["explore_done_sim_sec"]
        if done is None or abs(stated - done) > 1e-6:
            raise EventLogError(
                f"{path}: run_end says explore_done_sim_sec={stated} but the "
                f"last exploration_complete is at {done}")

    # Exploration FINISH (pre-registered secondary endpoint): only a
    # coverage-latched declaration is a finish. Under mission return a
    # step-budget robot still declares (and still goes home), so filtering on
    # the reason here -- not on what happened next -- is what keeps the
    # endpoint arm-invariant.
    latched = [e for e in completes if e.get("reason") == "coverage-latched"]
    latched_t = latched[-1]["t_sim_sec"] if latched else None

    # Mission end (pre-registered primary endpoint, schema v2).
    #
    # THE LAST one, not the first, and a duplicate is REPORTED rather than
    # excluded. This used to raise on len(missions) > 1, and that raise cost the
    # analysis more than the defect it was guarding against:
    #
    #   * ts1b shipped 16 robot-runs with two mission_complete rows: a
    #     DONE -> RETURN_HOME re-entry, which generation 8 believed was already
    #     guarded because startReturnHome opened with `if (state_ ==
    #     State::RETURN_HOME)`. That test structurally could not see the
    #     re-entry that was measured, because the second request arrives from
    #     DONE. Generation 9 added the run-scoped `mission_return_done_` latch
    #     ahead of it; the state test is still there and still does its own,
    #     narrower job. The raise turned every one of those cells into an
    #     EXCLUSION, and they were not a random 16 — a re-entry needs a second
    #     homing attempt, so they were the slowest runs of the arm that homes
    #     most. Dropping them shortened that arm's mean by removing its tail.
    #   * The data was never actually missing. A robot that declares twice still
    #     declares; the endpoint question ("when did the mission end") has a
    #     clean answer, namely the LAST declaration, consistent with
    #     explore_done_sim_sec being the last exploration_complete.
    #
    # So the duplicate is now surfaced as mission_occurrences / mission_duplicate
    # on the robot record and the cell still contributes its endpoint. The
    # planner also stamps `occurrence` on each row (see
    # MissionCompleteEvent::occurrence), so the two can be cross-checked: the
    # count here is what the FILE contains, the field is what the NODE believed
    # it was emitting, and a disagreement means events were lost in between.
    #
    # On generation-9 and later logs this pair can no longer see a duplicate at
    # all, and that is deliberate rather than an oversight. The writer latches
    # after the first row (ExperimentLog::logMissionComplete) and keeps the
    # FIRST one, so a run that declared twice lands on disk as one row stamped
    # occurrence 1: count 1, stamp 1, mismatch False. Read that way and nothing
    # else, the detector below would report "checked and clean" on precisely the
    # fault it exists for. The replacement signal is on the run_end row --
    # mission_completes_suppressed and mission_return_reentries, both pulled out
    # below -- and it is strictly better than the two-row version because it
    # survives the refusal of the second homing leg, which no longer produces a
    # second declaration to count. The old pair is kept because it is the only
    # thing that can read a generation-8 log.
    missions = [e for e in evs if e["event"] == "mission_complete"]
    mission = missions[-1] if missions else None
    mission_occ_stamped = missions[-1].get("occurrence") if missions else None
    if end is not None and end.get("mission_home_result"):
        if mission is None or mission.get("result") != end["mission_home_result"]:
            raise EventLogError(
                f"{path}: run_end says mission_home_result="
                f"{end['mission_home_result']!r} but the last mission_complete "
                f"event says {mission.get('result') if mission else None!r}")
    mission_ret = start.get("params", {}).get("mission_return_enabled")

    # Truncation is detectable from the file's own contents: run_end reports how
    # many events it wrote, and seq is contiguous from 0.
    written = end.get("events_written") if end else None
    complete = end is not None and (written is None or written == len(evs))

    return dict(
        robot=start.get("robot"),
        params=start.get("params", {}),
        t0_sim_sec=start.get("t0_sim_sec"),
        explore_done_sim_sec=done,
        explore_done_first_sim_sec=(completes[0]["t_sim_sec"]
                                    if completes else None),
        explore_latched_sim_sec=latched_t,
        mission_return_enabled=mission_ret,
        mission_result=mission.get("result") if mission else None,
        mission_sim_sec=mission.get("t_sim_sec") if mission else None,
        mission_reason=mission.get("reason") if mission else None,
        # How many mission_complete rows this file holds. The contract is one;
        # anything above one means the node reached DONE twice and this robot's
        # homing metrics (duration, distance) describe only the LAST attempt.
        # Reported, not raised -- see the block above for what the raise cost.
        mission_occurrences=len(missions),
        mission_duplicate=len(missions) > 1,
        # `occurrence` as the NODE stamped it on the last row, against which
        # len(missions) is the file's own count of rows. Equal is healthy. The
        # node's number being HIGHER means rows were emitted and did not reach
        # the disk, which is a different and worse fault than a duplicate: a
        # duplicate is visible, a lost row is not.
        #
        # None means the binary that wrote this log predates the field (it was
        # added with the generation-9 endpoint work), NOT that the check passed.
        # Those two have to stay distinguishable, which is why the mismatch flag
        # below is tri-state rather than a bool defaulting to False -- a False
        # there would read as "checked and clean" on a log that was never
        # checkable, which is exactly how a guard goes inert while still
        # printing passes.
        mission_occurrence_stamped=mission_occ_stamped,
        mission_occurrence_mismatch=(
            None if mission_occ_stamped is None
            else mission_occ_stamped != len(missions)),
        # The generation-9 endpoint counters, read off run_end because that is
        # the only row guaranteed to exist after a suppression (a duplicate
        # mission_complete necessarily precedes the run ending, unlike a
        # duplicate run_end, which is why these two could be put there and
        # dup_run_ends could not).
        #
        #   mission_return_reentries    -- startReturnHome calls the
        #       mission_return_done_ latch REFUSED. Non-zero is not a defect on
        #       its own: the late coverage latch legitimately asks for a second
        #       homing leg, and the refusal is the fix working. It is the
        #       manipulation check for that fix, so zero across a whole campaign
        #       is the thing to be suspicious of, not a large number.
        #
        #   mission_completes_suppressed -- second and later mission_complete
        #       rows the writer threw away. On generation 9 this SHOULD be 0
        #       everywhere: logMissionComplete has exactly one call site,
        #       finishMissionReturn, and startReturnHome's latch is what lets
        #       that be reached once. Non-zero therefore means a
        #       finishMissionReturn re-entry that did not pass through
        #       startReturnHome -- a path the node-level guard cannot see -- and
        #       the affected run's homing metrics describe the first attempt
        #       while its behaviour contains two. Treat those runs as suspect.
        #
        # Both tri-state for the same reason as the mismatch flag above: None
        # means the binary predates the field, which is not a pass.
        mission_return_reentries=(end.get("mission_return_reentries")
                                  if end else None),
        mission_completes_suppressed=(end.get("mission_completes_suppressed")
                                      if end else None),
        # v8 CHANGED WHAT homing_duration_sec MEASURES: it is now wall time on
        # the leg, where v7 excluded time parked in PROXIMITY_HOLD. Both parse,
        # neither is wrong, and they are not the same number -- so a mixed-
        # schema pool of this column is comparing two intervals. Subtract
        # homing_held_sec to recover the v7 quantity, which is also the one the
        # mission_return_max_sec budget is charged against.
        #
        # homing_held_sec is None on any v7 file, and that is not zero: it means
        # the binary did not measure it. Same tri-state discipline as the two
        # fields above.
        homing_duration_sec=(mission.get("homing_duration_sec")
                             if mission else None),
        homing_distance_m=(mission.get("homing_distance_m")
                           if mission else None),
        homing_held_sec=(mission.get("homing_held_sec")
                         if mission else None),
        # Non-zero only where a robot resumed after a manoeuvre and exhausted
        # again -- i.e. only in the reconnecting arms.
        resume_delta_sec=((completes[-1]["t_sim_sec"] - completes[0]["t_sim_sec"])
                          if completes else None),
        declarations=len(completes),
        censored=done is None,
        complete=complete,
        logger_healthy=bool(end.get("logger_healthy")) if end else False,
        rtf_mean=end.get("rtf_mean") if end else None,
        log_span_sim_sec=end.get("log_span_sim_sec") if end else None,
        milestones={e["threshold"]: e["t_sim_sec"]
                    for e in evs if e["event"] == "coverage_milestone"},
        dispatches=[e for e in evs if e["event"] == "reconnect_dispatch"],
        events=len(evs),
    )


def expected_robots(logs):
    """How many robots this cell was SUPPOSED to have, from its own manifest.

    Returns (n, source) or (None, reason).

    This replaced a hardcoded default of 2, which silently voided every N>=3
    campaign: a three-robot cell tripped "3 event log(s), expected 2" and was
    excluded, so ts1b_n3 and ts1b_n4 produced an exclusion list and no numbers at
    all. The count is not something a reader should have to pass on the command
    line -- the run itself records it, in run_start's team_robot_names.

    It is deliberately NOT len(logs). Deriving the expectation from what is
    present makes the check vacuous: a cell that lost a robot's log entirely
    would then "expect" exactly what it found and pass. The whole point of the
    comparison is to catch a missing log, so the expectation has to come from
    outside the set being counted.

    When no log names the team, the answer is UNRESOLVED rather than a guess. A
    guess here is the same class of mistake as a check that stopped checking:
    it would report a PASS derived from nothing.
    """
    for path in logs.values():
        try:
            evs = read_events(path)
        except EventLogError:
            continue
        start = next((e for e in evs if e.get("event") == "run_start"), None)
        if not start:
            continue
        names = (start.get("params") or {}).get("team_robot_names")
        if names:
            n = len([t for t in str(names).split(",") if t.strip()])
            if n > 0:
                return n, f"team_robot_names={names!r}"
    return None, ("no readable run_start carries team_robot_names; the expected "
                  "robot count is unresolved (pass --expect-robots to override)")


def arm_from_any_params(logs):
    """The params-derived arm, from whichever log can still be read.

    The exclusion paths used to label the cell from its DIRECTORY NAME, on the
    reasoning that params are unreadable exactly there. That is true of some
    exclusions and false of most: a cell excluded for a truncated log, a wrong
    robot count, or a schema refusal usually still has at least one intact
    run_start, and run_start is the first line written. Reading it costs one
    parse and gives the same arm every non-excluded cell is grouped by, instead
    of a string a regex guessed from a path.

    The directory name stays as the last-resort fallback, and
    summarise_run reports when the two disagree.
    """
    for path in logs.values():
        try:
            evs = read_events(path)
        except EventLogError:
            continue
        start = next((e for e in evs if e.get("event") == "run_start"), None)
        if start:
            arm = run_arm(start.get("params") or {})
            if arm:
                return arm
    return None


def _arm_tokens_agree(p_arm, dir_arm):
    """Is the params arm present, whole, inside the directory-derived arm?

    Both are underscore-token strings. The directory's version carries extras
    the arm does not (team size, radio range, a TTL or seek suffix), so the
    question is containment of a CONTIGUOUS token run, not equality:

        'mtare_hybrid' in 'n3_mtare_hybrid_r40_ttl0'  -> True
        'hybrid'       in 'n3_mtare_off_r40_ttl0'     -> False
        'off'          in 'n2_mtare_off_r40'          -> True

    Token-wise rather than substring-wise on purpose: 'off' is a substring of
    'offset' and of 'standoff', and a containment test that accepted those would
    quietly stop reporting the one case it exists to report.
    """
    if not p_arm or not dir_arm:
        return False
    pt = [t for t in p_arm.split("_") if t]
    dt = [t for t in dir_arm.split("_") if t]
    if not pt or len(pt) > len(dt):
        return False
    return any(dt[i:i + len(pt)] == pt for i in range(len(dt) - len(pt) + 1))


def summarise_run(run_dir, expect_robots=None):
    """A whole cell. `t_team` is None when ANY robot is censored.

    Returns a dict with `excluded` set (and nothing else guaranteed) when the
    cell cannot contribute a completion time. Callers must check it -- a run
    that failed to produce data is not a run that finished quickly.

    expect_robots=None (the default) derives the expected count from the run's
    own team_robot_names; pass an int only to override a cell whose manifest
    cannot say. See expected_robots().
    """
    logs = robot_logs(run_dir)
    # `arm` on the exclusion paths: params FIRST, directory name only as a last
    # resort. An exclusion of unknown arm cannot answer the one question the
    # exclusion count exists to answer -- whether the arms lost the same number
    # of cells -- so every exclusion record carries an arm; but the label has to
    # be the same one the surviving cells are grouped by, or the exclusions file
    # under an arm that does not exist. `arm_source` says which it was, and a
    # disagreement between the two is reported rather than silently resolved.
    cell = os.path.basename(run_dir.rstrip("/"))
    dir_arm = arm_from_dirname(run_dir)
    p_arm = arm_from_any_params(logs)
    ex_arm = p_arm or dir_arm
    arm_src = "params" if p_arm else ("dirname" if dir_arm else None)
    arm_note = None
    if p_arm and dir_arm and not _arm_tokens_agree(p_arm, dir_arm):
        # Inequality alone is NOT the test, and this is the whole reason the
        # note is usable. Every real cell directory disagrees with its params
        # arm by construction — 'ts1b_n3_mtare_hybrid_r40_ttl0_seed1' yields a
        # dirname arm of 'n3_mtare_hybrid_r40_ttl0' against a params arm of
        # 'mtare_hybrid' — so a note on `!=` would fire on 100% of cells and be
        # read as decoration within a day. What is worth reporting is the
        # directory naming a DIFFERENT arm, not the directory naming the same arm
        # plus team size, radio, and knob suffixes. See _arm_tokens_agree.
        arm_note = (f"directory says arm={dir_arm!r}, which does not contain the "
                    f"run_start params arm {p_arm!r} — grouping on params")

    def excluded(reason, **kw):
        # n_robots reads `expect_robots` at CALL time, so every exclusion raised
        # after the count is resolved carries it, and the one raised because it
        # could NOT be resolved carries None. That is the honest split: an
        # exclusion of unknown team size must not be filed into a size stratum,
        # because arm_summary keys on the stratum and a guess there would put a
        # lost cell under a team size it may not have had.
        r = dict(excluded=reason, run=cell, arm=ex_arm, arm_source=arm_src,
                 n_robots=expect_robots)
        if arm_note:
            r["arm_note"] = arm_note
        r.update(kw)
        return r

    if expect_robots is None:
        expect_robots, src = expected_robots(logs)
        if expect_robots is None:
            return excluded(src)
    if len(logs) != expect_robots:
        return excluded(f"{len(logs)} event log(s), expected {expect_robots}")
    try:
        per = {r: summarise_robot(p) for r, p in logs.items()}
    except EventLogError as e:
        return excluded(str(e))

    unhealthy = [r for r, s in per.items() if not s["complete"]]
    if unhealthy:
        return excluded(f"event log truncated for {','.join(unhealthy)}",
                        per_robot=per)

    dones = [s["explore_done_sim_sec"] for s in per.values()]
    censored = any(d is None for d in dones)
    finished = [d for d in dones if d is not None]
    any_p = next(iter(per.values()))["params"]

    # Pre-registered endpoints (mission-return campaigns). Same withholding
    # rule as t_team: if ANY robot lacks the endpoint the team has no value,
    # and the per-robot results stay visible so the censoring is countable.
    latches = [s["explore_latched_sim_sec"] for s in per.values()]
    t_explore = max(latches) if all(t is not None for t in latches) else None
    arrived = all(s["mission_result"] == "arrived" for s in per.values())
    t_mission = (max(s["mission_sim_sec"] for s in per.values())
                 if arrived else None)

    return dict(
        run=cell,
        arm=run_arm(any_p),
        arm_source="params",
        arm_note=arm_note,
        # Team size, carried on every row. arm_summary keys on it: a max or a min
        # over robots is an ORDER STATISTIC, and order statistics move with N
        # under a pure null, so pooling an N=2 and an N=3 cell into one arm mean
        # measures team size and calls it treatment.
        n_robots=len(per),
        git_rev=any_p.get("git_rev"),
        per_robot=per,
        # Robot-runs in this cell that declared mission_complete more than once.
        # Non-empty means those robots' homing metrics describe the LAST attempt
        # only. The cell still contributes -- see summarise_robot.
        mission_duplicate_robots=[r for r, s in per.items()
                                  if s.get("mission_duplicate")],
        # Robots whose node-stamped occurrence disagrees with the number of rows
        # on disk, i.e. an endpoint declaration the node made and the file does
        # not hold.
        #
        # `is True`, not truthiness. mission_occurrence_mismatch is TRI-STATE by
        # construction (summarise_robot says so at length): True = rows lost,
        # False = counted and equal, None = the binary that wrote the log never
        # stamped `occurrence` so nothing was compared. A bare truthiness test
        # maps None and False onto the same empty list, and the emptiness is
        # then read downstream as "0 cells lost rows" -- the exact failure the
        # tri-state was built to prevent, reintroduced one line below the
        # comment forbidding it. Measured on the bank: 792/792 robot-runs across
        # ts1b_n3, ts1b_n4 and cr5 are None, so the old test made this list
        # empty on every cell ever analysed and the NOTE below never printed
        # once.
        mission_lost_row_robots=[r for r, s in per.items()
                                 if s.get("mission_occurrence_mismatch") is True],
        # The other side of the same tri-state, carried as its own list so the
        # denominator survives to the report: robots where the comparison could
        # not be made at all. This is not a fault -- it is the reason the line
        # above is empty on a pre-generation-9 log -- and it exists so that
        # "0 of 80 cells lost rows" and "80 of 80 cells were never checked"
        # cannot print as the same silence.
        mission_occ_uncheckable_robots=[
            r for r, s in per.items()
            if s.get("mission_occurrence_mismatch") is None],
        # Robots whose writer threw away a second mission_complete. This is the
        # generation-9 successor to mission_duplicate_robots: the same fault,
        # counted at the writer instead of by counting rows, because the rows
        # are no longer there to count. A `> 0` test and not a truthiness test
        # so that None (pre-generation-9, field absent) does not silently read
        # as clean.
        mission_suppressed_robots=[
            r for r, s in per.items()
            if (s.get("mission_completes_suppressed") or 0) > 0],
        # Robots where the once-per-run latch refused a later homing request.
        # NOT a fault list -- see the field comment in summarise_robot. It is
        # here so the fix can be shown to have fired, because a guard nobody can
        # count is indistinguishable from a guard that went inert.
        mission_reentry_robots=[
            r for r, s in per.items()
            if (s.get("mission_return_reentries") or 0) > 0],
        # THE completion time, and it is a MAKESPAN: max over robots. Named
        # t_team for compatibility with every existing consumer, but read it as
        # an order statistic -- see t_mean beside it, and n_robots above.
        # None when censored -- withheld, never imputed.
        t_team=max(finished) if not censored else None,
        # The per-robot MEAN of the same quantity, which is not an order
        # statistic and therefore does not drift with N. On ts1b the two
        # disagreed in MAGNITUDE (-20.2% per-robot mean vs -6.1% makespan), so a
        # table that quotes only the makespan understates the per-robot effect by
        # a factor of three. Emitted beside it, never instead of it: makespan is
        # the operationally meaningful one for "when can the team go home".
        t_mean=(sum(finished) / len(finished)) if not censored else None,
        # The leader still finished, and its time is real even in a censored
        # run; it is the laggard that is unknown.
        t_lead=min(finished) if finished else None,
        censored=censored,
        censored_robots=[r for r, s in per.items() if s["censored"]],
        # Mission-return endpoints. None on banked (pre-v2) logs, and None on a
        # v2 run any robot of which never latched / never arrived -- withheld.
        t_explore=t_explore,
        t_mission=t_mission,
        mission_results={r: s["mission_result"] for r, s in per.items()},
        resume_delta_total_sec=sum(s["resume_delta_sec"] or 0.0
                                   for s in per.values()),
        rtf_mean=(sum(s["rtf_mean"] or 0.0 for s in per.values())
                  / max(1, len(per))),
    )


def arm_summary(rows):
    """Per (arm, team size) aggregate: the mean AND what it was computed over.

    A censored run contributes no completion time, so an arm's mean is taken
    over its SURVIVORS. That is only honest if you can see how many there were,
    and censoring is not random with respect to the treatment: the arms that
    manoeuvre most are the ones that run out of horizon, so dropping censored
    runs scores each reconnecting arm on its quietest runs and flatters it
    exactly where the treatment cost the most. The counts are returned in the
    same record as the mean so no caller can print one without the other.

    THE KEY IS (arm, n_robots), NOT arm. t_team is a max over robots, and a max
    over robots grows with the number of robots under a pure null — draw one
    more sample from the same distribution and the largest of them is larger, no
    treatment required. So an arm mean pooled across team sizes is part arm and
    part N, in a proportion nothing on the page reveals. Keying on the pair makes
    the pooling impossible rather than merely discouraged; a caller that wants
    an across-N number now has to build it deliberately, which is the point.

    `mean_t_mean` is the companion that does NOT have this problem: the
    per-robot mean of the same finish times is not an order statistic, so it is
    comparable across N. On ts1b the two disagreed threefold in magnitude
    (-20.2% vs -6.1%), so they are reported side by side and neither is called
    "the" effect.
    """
    by_arm = {}
    for r in rows:
        arm = r.get("arm")
        if arm is None and "per_robot" in r:
            for s in r["per_robot"].values():
                arm = run_arm(s.get("params", {}))
                if arm:
                    break
        key = (arm or "?", r.get("n_robots"))
        rec = by_arm.setdefault(key, dict(
            arm=arm or "?", n_robots=r.get("n_robots"), n=0, finished=0,
            censored=0, excluded=0, times=[], times_mean=[],
            mission_done=0, mission_censored=0, times_mission=[],
            explore_done=0, explore_censored=0, times_explore=[],
            mission_duplicate_cells=0, mission_lost_row_cells=0,
            mission_occ_uncheckable_cells=0,
            mission_suppressed_cells=0, mission_reentry_cells=0))
        rec["n"] += 1
        if r.get("mission_duplicate_robots"):
            rec["mission_duplicate_cells"] += 1
        if r.get("mission_suppressed_robots"):
            rec["mission_suppressed_cells"] += 1
        if r.get("mission_reentry_robots"):
            rec["mission_reentry_cells"] += 1
        if r.get("mission_lost_row_robots"):
            rec["mission_lost_row_cells"] += 1
        # Counted on the same pass and printed beside it. A cell lands here when
        # ANY of its robots could not be checked, which is the conservative
        # direction: it is the denominator of a guard, so a partial check has to
        # read as a partial check.
        if r.get("mission_occ_uncheckable_robots"):
            rec["mission_occ_uncheckable_cells"] += 1
        if "excluded" in r:
            rec["excluded"] += 1
            continue
        if r["t_team"] is None:
            rec["censored"] += 1
        else:
            rec["finished"] += 1
            rec["times"].append(r["t_team"])
            if r.get("t_mean") is not None:
                rec["times_mean"].append(r["t_mean"])
        # Mission endpoints ride alongside, counted with the same honesty rule:
        # a mission mean is only reportable next to its own censoring count.
        if r.get("t_mission") is not None:
            rec["mission_done"] += 1
            rec["times_mission"].append(r["t_mission"])
        else:
            rec["mission_censored"] += 1
        # t_explore gets a denominator for the same reason t_mission does, and
        # it matters MORE, not less. Pre-registration 32.14 rule 4 promotes
        # t_explore to the primary inference precisely when t_mission censoring
        # is arm-unbalanced — so the case where the reader falls back to this
        # endpoint is the case where a silent, differential drop of unlatched
        # runs would bias it, with nothing on screen to show it happened.
        if r.get("t_explore") is not None:
            rec["explore_done"] += 1
            rec["times_explore"].append(r["t_explore"])
        else:
            rec["explore_censored"] += 1
    for rec in by_arm.values():
        rec["mean_t_team"] = (sum(rec["times"]) / len(rec["times"])
                              if rec["times"] else None)
        # Same denominator as mean_t_team by construction (both are appended
        # under the same not-censored branch), so the two are comparable without
        # a second censoring count.
        rec["mean_t_mean"] = (sum(rec["times_mean"]) / len(rec["times_mean"])
                              if rec["times_mean"] else None)
        rec["mean_t_mission"] = (sum(rec["times_mission"])
                                 / len(rec["times_mission"])
                                 if rec["times_mission"] else None)
        rec["mean_t_explore"] = (sum(rec["times_explore"])
                                 / len(rec["times_explore"])
                                 if rec["times_explore"] else None)
    return [by_arm[k] for k in sorted(by_arm, key=lambda k: (k[0], k[1] or 0))]


def main(argv):
    import argparse
    global MIN_SCHEMA
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dirs", nargs="+")
    ap.add_argument("--json", action="store_true", help="machine-readable")
    ap.add_argument("--min-schema", type=int, default=MIN_SCHEMA,
                    help=f"lowest event-log schema_version to read (default "
                         f"{MIN_SCHEMA}, what the current binary writes). Lower "
                         f"it to read banked cells from older binaries. Doing "
                         f"so pools across binary generations, which is only "
                         f"valid if you are looking at one generation at a "
                         f"time; it is a flag so that choice is visible in the "
                         f"command line that produced the numbers.")
    ap.add_argument("--expect-robots", type=int, default=None,
                    help="how many event logs each cell must have. The default "
                         "reads it from the run's own team_robot_names, which "
                         "is right for every campaign the harness wrote; pass "
                         "it only to rescue a cell whose run_start predates "
                         "that field. Setting it WRONG turns every cell of "
                         "another team size into an exclusion, which is how "
                         "the old hardcoded 2 voided the N=3 and N=4 arms.")
    a = ap.parse_args(argv)
    if a.min_schema != MIN_SCHEMA:
        # Announce it on stderr as well as accepting it. The flag's whole value
        # is that lowering the bar leaves a trace, and a trace that lives only
        # in a shell history someone has to go and find is not much of one.
        print(f"warning: reading schema >= {a.min_schema} instead of the "
              f"default {MIN_SCHEMA}; results may pool binary generations",
              file=sys.stderr)
        MIN_SCHEMA = a.min_schema

    rows = [summarise_run(d, expect_robots=a.expect_robots)
            for d in a.run_dirs]
    if a.json:
        print(json.dumps(dict(runs=rows, by_arm=arm_summary(rows)),
                         indent=1, default=str))
        return 0
    # Column widths from the DATA, never a literal. The 34 and 11 hardcoded
    # here were both short of what this project actually produces:
    # `ts1b_n4_mtare_hybrid_r40_ttl0_seed1` is 35 characters and
    # `mtare_rendezvous` is 16, so EVERY row of EVERY two-factor campaign
    # overflowed its own column and shoved the nine columns to its right
    # out of alignment. The numbers were all correct and the table was
    # unreadable, which is the same outcome as being wrong. Same fix, same
    # reasoning, as gate_g8.py's cell column.
    _rw = max([len("run")] + [len(str(r.get("run", "?"))) for r in rows])
    _aw = max([len("arm")] + [len(str(r.get("arm"))) for r in rows])
    # t_team is labelled t_make here and everywhere it is printed. It is a
    # makespan, and calling it "the team time" is what let it be read as a
    # per-team average and compared across team sizes.
    print(f"{'run':{_rw}s} {'arm':{_aw}s} {'N':>2s} {'t_make':>9s} {'t_mean':>9s} "
          f"{'t_expl':>9s} {'t_missn':>9s} {'resume':>7s} {'rtf':>5s}  note")
    for r in rows:
        nr = "-" if r.get("n_robots") is None else f"{r['n_robots']:2d}"
        if "excluded" in r:
            # Print the arm on this row too. A blank here is what made an
            # excluded cell look arm-less on inspection even after the record
            # itself carried the arm.
            print(f"{r.get('run','?'):{_rw}s} {str(r.get('arm')):{_aw}s} {nr:>2s} "
                  f"{'EXCLUDED':>9s} {'':>9s} "
                  f"{'':>9s} {'':>9s} {'':>7s} {'':>5s}  {r['excluded']}")
            continue
        tt = "CENSORED" if r["t_team"] is None else f"{r['t_team']:9.1f}"
        tmn = "-" if r.get("t_mean") is None else f"{r['t_mean']:9.1f}"
        te = "-" if r.get("t_explore") is None else f"{r['t_explore']:9.1f}"
        tm = "-" if r.get("t_mission") is None else f"{r['t_mission']:9.1f}"
        note = ("censored: " + ",".join(r["censored_robots"])) if r["censored"] else ""
        # On a mission-return run a "-" in t_missn deserves its reason.
        if r.get("t_mission") is None and any(r.get("mission_results", {}).values()):
            bad = [f"{rob}:{res or 'none'}"
                   for rob, res in r["mission_results"].items()
                   if res != "arrived"]
            note = (note + " " if note else "") + "mission " + ",".join(bad)
        # A robot that declared mission_complete twice keeps its cell (see
        # summarise_robot), so the only way a reader learns about it is here.
        if r.get("mission_duplicate_robots"):
            note = ((note + " " if note else "") + "DUP mission_complete: "
                    + ",".join(r["mission_duplicate_robots"]))
        if r.get("mission_lost_row_robots"):
            note = ((note + " " if note else "")
                    + "LOST mission_complete row(s): "
                    + ",".join(r["mission_lost_row_robots"]))
        # Generation 9: the duplicate row is suppressed at the writer, so this
        # is the only place a reader can learn the run declared twice. Printed
        # with the same weight as DUP above -- it IS DUP, seen from the writer.
        if r.get("mission_suppressed_robots"):
            note = ((note + " " if note else "")
                    + "SUPPRESSED mission_complete: "
                    + ",".join(r["mission_suppressed_robots"]))
        if r.get("arm_note"):
            note = (note + " " if note else "") + r["arm_note"]
        print(f"{r['run']:{_rw}s} {str(r['arm']):{_aw}s} {nr:>2s} {tt:>9s} "
              f"{tmn:>9s} {te:>9s} {tm:>9s} "
              f"{r['resume_delta_total_sec']:7.1f} "
              f"{r['rtf_mean']:5.3f}  {note}")

    summ = arm_summary(rows)
    print()
    # Every mean carries its own censoring count in the adjacent column. Before
    # round 4 mean t_expl had none, which is the endpoint 32.14 rule 4 falls
    # back to when t_mission censoring is unbalanced -- so the number the
    # fallback rests on was the one number with no denominator on screen.
    # Same rule for the per-arm table, computed from ITS rows rather than
    # reusing the per-run width: an arm can appear in the summary that was
    # filtered out of the listing above, and borrowing a width from the
    # wrong population is how a "computed" width silently becomes a
    # literal again.
    _sw = max([len("arm")] + [len(str(x["arm"])) for x in summ])
    print(f"{'arm':{_sw}s} {'N':>2s} {'n':>3s} {'done':>5s} {'cens':>5s} "
          f"{'excl':>5s} {'mean t_make':>12s} {'mean t_mean':>12s} "
          f"{'mean t_expl':>12s} {'e.cens':>6s} "
          f"{'mean t_missn':>12s} {'m.cens':>6s}")
    for s in summ:
        mt = "-" if s["mean_t_team"] is None else f"{s['mean_t_team']:12.1f}"
        mp = "-" if s["mean_t_mean"] is None else f"{s['mean_t_mean']:12.1f}"
        me = ("-" if s["mean_t_explore"] is None
              else f"{s['mean_t_explore']:12.1f}")
        mm = ("-" if s["mean_t_mission"] is None
              else f"{s['mean_t_mission']:12.1f}")
        sn = "-" if s["n_robots"] is None else f"{s['n_robots']:2d}"
        print(f"{s['arm']:{_sw}s} {sn:>2s} {s['n']:3d} {s['finished']:5d} "
              f"{s['censored']:5d} {s['excluded']:5d} {mt:>12s} {mp:>12s} "
              f"{me:>12s} {s['explore_censored']:6d} "
              f"{mm:>12s} {s['mission_censored']:6d}")
    # t_make is a max over robots, so a row-to-row comparison down the N column
    # is not an arm comparison. Say so on the page rather than trusting the
    # column header to carry it.
    if len({s["n_robots"] for s in summ if s["n_robots"] is not None}) > 1:
        print()
        print("NOTE: more than one team size is present. t_make is a MAX over "
              "robots, and a max grows with")
        print("      N under a pure null, so do not compare t_make between N "
              "rows or average them together.")
        print("      Compare within an N row, or use t_mean, which is not an "
              "order statistic.")
    # The lost-row check, printed WHETHER OR NOT it found anything, because its
    # healthy output is an empty list and an empty list is also what a check
    # that never ran produces. `occurrence` is stamped only by generation-9 and
    # later binaries, so on every cell in the bank the comparison is impossible
    # -- and for as long as this block was conditional, that impossibility
    # printed as silence, which a reader scanning the page takes for "checked,
    # nothing wrong". The denominator is therefore on the page in both
    # directions: how many cells failed, and how many could not be asked.
    lostrow = [s for s in summ if s["mission_lost_row_cells"]]
    unchk = [s for s in summ if s["mission_occ_uncheckable_cells"]]
    n_cells = sum(s["n"] for s in summ)
    n_unchk = sum(s["mission_occ_uncheckable_cells"] for s in summ)
    print()
    if lostrow:
        print("NOTE: a robot stamped a mission_complete occurrence number "
              "HIGHER than the rows present in")
        print("      its log — an endpoint declaration the node made and the "
              "file does not hold. Unlike a")
        print("      duplicate this is silent data loss; check the logger "
              "before trusting these cells:")
        for s in lostrow:
            print(f"        {s['arm']} N={s['n_robots']}: "
                  f"{s['mission_lost_row_cells']} of {s['n']} cells")
    if n_unchk >= n_cells and n_cells:
        print("NOTE: the mission_complete lost-row check DID NOT RUN on this "
              "set. None of the")
        print(f"      {n_cells} cells carries an `occurrence` stamp, so the "
              "node's own count of the")
        print("      declarations it made could not be compared against the "
              "rows on disk. The stamp")
        print("      arrived with the generation-9 endpoint work; a log without "
              "it is older than the")
        print("      check. This is absence of the check, NOT a pass — do not "
              "read the empty")
        print("      lost-row list above as evidence that no rows were lost.")
    elif n_unchk:
        print(f"NOTE: the mission_complete lost-row check ran on "
              f"{n_cells - n_unchk} of {n_cells} cells and")
        print(f"      could not be applied to the other {n_unchk}, which carry "
              "no `occurrence` stamp.")
        print("      That means this set MIXES binary generations, which is a "
              "finding in its own right")
        print("      — results must not be pooled across generations. "
              "Per arm, not checkable:")
        for s in unchk:
            print(f"        {s['arm']} N={s['n_robots']}: "
                  f"{s['mission_occ_uncheckable_cells']} of {s['n']} cells")
    elif not lostrow and n_cells:
        print(f"OK:   mission_complete lost-row check: {n_cells} of {n_cells} "
              "cells checked, none lost a row.")
        print("      (Printed even when clean: an empty finding and an "
              "unrunnable check look the same.)")
    dup = [s for s in summ if s["mission_duplicate_cells"]]
    if dup:
        print()
        print("NOTE: some cells hold a robot that declared mission_complete "
              "TWICE. Those cells are KEPT (the")
        print("      last declaration is the endpoint), but their homing "
              "duration/distance describe the last")
        print("      attempt only. An earlier reader EXCLUDED them, which "
              "removed the slowest runs of the")
        print("      arm that homes most:")
        for s in dup:
            print(f"        {s['arm']} N={s['n_robots']}: "
                  f"{s['mission_duplicate_cells']} of {s['n']} cells")
    sup = [s for s in summ if s["mission_suppressed_cells"]]
    if sup:
        print()
        print("NOTE: the WRITER threw away a second mission_complete in some "
              "cells. On a generation-9")
        print("      binary this should be impossible — logMissionComplete has "
              "one call site, and the")
        print("      mission_return_done_ latch is what lets it be reached "
              "once — so a non-zero count here")
        print("      means finishMissionReturn re-entered WITHOUT going "
              "through startReturnHome, which is")
        print("      a path the node-level guard cannot see. The kept row is "
              "the FIRST attempt while the")
        print("      behaviour contains two; treat these cells' homing metrics "
              "as suspect:")
        for s in sup:
            print(f"        {s['arm']} N={s['n_robots']}: "
                  f"{s['mission_suppressed_cells']} of {s['n']} cells")
    # The manipulation check, and the only one of these four blocks that is
    # printed for a HEALTHY campaign. A guard that never reports firing and a
    # guard that was compiled out look identical from the outside, so the count
    # is put on the page whether it is zero or not -- the zero case says so in
    # words rather than by staying silent, because silence is what "the check
    # stopped checking" looks like.
    ret_cells = sum(s["mission_reentry_cells"] for s in summ)
    # Whether the FIELD exists, which is not the same question as whether it is
    # zero. Read off the robot records, because the arm record counts cells and
    # a count of cells is 0 both when no guard fired and when no guard was
    # compiled. An arm-level `is not None` test would have been vacuously true
    # and the branch below would have printed "clean" on a generation-8 log.
    #
    # Counted rather than any()-ed, because a set holding both generations is
    # the worst case for this check: any() would call it present and the zero
    # branch would report a measured zero over robot-runs that were never
    # checkable. Reporting the denominator makes the mixture visible instead.
    latch_seen = [rr.get("mission_return_reentries") is not None
                  for r in rows for rr in (r.get("per_robot") or {}).values()]
    latch_have, latch_all = sum(latch_seen), len(latch_seen)
    if latch_have and latch_have < latch_all:
        print()
        print(f"NOTE: {latch_have} of {latch_all} robot-runs carry "
              "mission_return_reentries and the rest do not, so this")
        print("      set MIXES binary generations. That is a pooling error in "
              "its own right — results are")
        print("      not comparable across generations — and it means the "
              "latch verdict below covers only")
        print("      part of the set. Split the read by generation.")
    if latch_have:
        print()
        if ret_cells:
            print("NOTE: the once-per-run mission-return latch REFUSED a later "
                  "homing request in some cells.")
            print("      This is the generation-9 fix working, not a fault: a "
                  "late coverage latch legitimately")
            print("      asks to home again after the run already homed, and "
                  "the exploration endpoint it")
            print("      carries is still recorded. Only the redundant second "
                  "homing leg is refused.")
            for s in summ:
                if s["mission_reentry_cells"]:
                    print(f"        {s['arm']} N={s['n_robots']}: "
                          f"{s['mission_reentry_cells']} of {s['n']} cells")
        else:
            print("NOTE: the once-per-run mission-return latch refused nothing "
                  "in this set (0 of "
                  f"{sum(s['n'] for s in summ)} cells, "
                  f"{latch_have}/{latch_all} robot-runs checked).")
            print("      The field IS present, so this is a measured zero and "
                  "not an absent check. Zero is")
            print("      plausible on short runs that never latch coverage "
                  "late; it is worth a second look")
            print("      on a full campaign, where ts1b saw the re-entry in "
                  "16 robot-runs.")
    else:
        print()
        print("NOTE: run_end carries no mission_return_reentries field in "
              "these logs, so the once-per-run")
        print("      mission-return latch was NOT checked here. This is a "
              "pre-generation-9 binary. Absence")
        print("      of the check is not a pass, and these logs may contain "
              "the DONE -> RETURN_HOME")
        print("      re-entry that generation 9 fixed — look for DUP "
              "mission_complete above instead.")
    lost = [s for s in summ if s["censored"] or s["excluded"]]
    if lost:
        print()
        print("NOTE: means are over FINISHED runs only. Censoring is not random "
              "with respect to the arm —")
        print("      an arm that manoeuvres more runs out of horizon more, so "
              "dropping its censored runs")
        print("      scores it on its quietest ones. Report these counts "
              "with any completion time:")
        for s in lost:
            print(f"        {s['arm']}: {s['censored']} censored, "
                  f"{s['excluded']} excluded of {s['n']}")
    return 0


if __name__ == "__main__":
    import sys
    raise SystemExit(main(sys.argv[1:]))
