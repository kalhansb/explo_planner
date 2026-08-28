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

# The lowest event-log schema this reader accepts, as a MODULE-level constant so
# the CLI can lower it deliberately (--min-schema) without editing source.
#
# The default must track the current generation rather than being permissive:
# the whole point is that scoring a generation-8 campaign should not quietly
# read a generation-6 cell that happens to sit in the same directory. Lowering
# it is legitimate for historical work; doing so silently is not, which is why
# it is a flag and not a default.
MIN_SCHEMA = 3


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


def run_arm(params):
    """The ARM a run belongs to. NOT params["reconnect_mode"].

    The control arm is "no reconnection", which is not a reconnect_mode value:
    it is rendezvous_enabled=false, and the harness still has to pass some mode
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
    return mode if params.get("rendezvous_enabled", True) else "off"


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
        raise EventLogError(
            f"{path}: schema_version {ver} < {MIN_SCHEMA}. Refused by default "
            f"so a schema-{MIN_SCHEMA} summary cannot silently pool cells from "
            f"an older binary generation. Pass --min-schema {ver} to read it "
            f"deliberately.")
    if ver > MIN_SCHEMA:
        print(f"warning: {path}: schema_version {ver} is newer than this "
              f"reader's {MIN_SCHEMA}; unknown fields ignored", file=sys.stderr)

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

    # Mission end (pre-registered primary endpoint, schema v2). Emitted at most
    # once; cross-checked against run_end the same way as exploration above.
    missions = [e for e in evs if e["event"] == "mission_complete"]
    if len(missions) > 1:
        raise EventLogError(f"{path}: {len(missions)} mission_complete events")
    mission = missions[0] if missions else None
    if end is not None and end.get("mission_home_result"):
        if mission is None or mission.get("result") != end["mission_home_result"]:
            raise EventLogError(
                f"{path}: run_end says mission_home_result="
                f"{end['mission_home_result']!r} but the mission_complete "
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
        homing_duration_sec=(mission.get("homing_duration_sec")
                             if mission else None),
        homing_distance_m=(mission.get("homing_distance_m")
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


def summarise_run(run_dir, expect_robots=2):
    """A whole cell. `t_team` is None when ANY robot is censored.

    Returns a dict with `excluded` set (and nothing else guaranteed) when the
    cell cannot contribute a completion time. Callers must check it -- a run
    that failed to produce data is not a run that finished quickly.
    """
    logs = robot_logs(run_dir)
    # `arm` on the exclusion paths comes from the DIRECTORY NAME, because those
    # are exactly the paths on which the params are unreadable. arm_summary can
    # otherwise only recover an arm via per_robot, so an unparseable cell files
    # under "?" -- and an exclusion of unknown arm cannot answer the one
    # question the exclusion count exists to answer, namely whether the two arms
    # lost the same number of cells. A directory-name arm is weaker evidence
    # than a params arm and is not used for anything but this bookkeeping.
    dir_arm = arm_from_dirname(run_dir)
    if len(logs) != expect_robots:
        return dict(excluded=f"{len(logs)} event log(s), expected {expect_robots}",
                    run=os.path.basename(run_dir.rstrip("/")), arm=dir_arm)
    try:
        per = {r: summarise_robot(p) for r, p in logs.items()}
    except EventLogError as e:
        return dict(excluded=str(e), run=os.path.basename(run_dir.rstrip("/")),
                    arm=dir_arm)

    unhealthy = [r for r, s in per.items() if not s["complete"]]
    if unhealthy:
        return dict(excluded=f"event log truncated for {','.join(unhealthy)}",
                    run=os.path.basename(run_dir.rstrip("/")), per_robot=per)

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
        run=os.path.basename(run_dir.rstrip("/")),
        arm=run_arm(any_p),
        git_rev=any_p.get("git_rev"),
        per_robot=per,
        # THE completion time. None when censored -- withheld, never imputed.
        t_team=max(finished) if not censored else None,
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
    """Per-arm aggregate: the mean AND what it was computed over.

    A censored run contributes no completion time, so an arm's mean is taken
    over its SURVIVORS. That is only honest if you can see how many there were,
    and censoring is not random with respect to the treatment: the arms that
    manoeuvre most are the ones that run out of horizon, so dropping censored
    runs scores each reconnecting arm on its quietest runs and flatters it
    exactly where the treatment cost the most. The counts are returned in the
    same record as the mean so no caller can print one without the other.
    """
    by_arm = {}
    for r in rows:
        arm = r.get("arm")
        if arm is None and "per_robot" in r:
            for s in r["per_robot"].values():
                arm = run_arm(s.get("params", {}))
                if arm:
                    break
        rec = by_arm.setdefault(arm or "?", dict(
            arm=arm or "?", n=0, finished=0, censored=0, excluded=0, times=[],
            mission_done=0, mission_censored=0, times_mission=[],
            times_explore=[]))
        rec["n"] += 1
        if "excluded" in r:
            rec["excluded"] += 1
            continue
        if r["t_team"] is None:
            rec["censored"] += 1
        else:
            rec["finished"] += 1
            rec["times"].append(r["t_team"])
        # Mission endpoints ride alongside, counted with the same honesty rule:
        # a mission mean is only reportable next to its own censoring count.
        if r.get("t_mission") is not None:
            rec["mission_done"] += 1
            rec["times_mission"].append(r["t_mission"])
        else:
            rec["mission_censored"] += 1
        if r.get("t_explore") is not None:
            rec["times_explore"].append(r["t_explore"])
    for rec in by_arm.values():
        rec["mean_t_team"] = (sum(rec["times"]) / len(rec["times"])
                              if rec["times"] else None)
        rec["mean_t_mission"] = (sum(rec["times_mission"])
                                 / len(rec["times_mission"])
                                 if rec["times_mission"] else None)
        rec["mean_t_explore"] = (sum(rec["times_explore"])
                                 / len(rec["times_explore"])
                                 if rec["times_explore"] else None)
    return [by_arm[k] for k in sorted(by_arm)]


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
    a = ap.parse_args(argv)
    if a.min_schema != MIN_SCHEMA:
        # Announce it on stderr as well as accepting it. The flag's whole value
        # is that lowering the bar leaves a trace, and a trace that lives only
        # in a shell history someone has to go and find is not much of one.
        print(f"warning: reading schema >= {a.min_schema} instead of the "
              f"default {MIN_SCHEMA}; results may pool binary generations",
              file=sys.stderr)
        MIN_SCHEMA = a.min_schema

    rows = [summarise_run(d) for d in a.run_dirs]
    if a.json:
        print(json.dumps(dict(runs=rows, by_arm=arm_summary(rows)),
                         indent=1, default=str))
        return 0
    print(f"{'run':34s} {'arm':11s} {'t_team':>9s} {'t_expl':>9s} "
          f"{'t_missn':>9s} {'resume':>7s} {'rtf':>5s}  note")
    for r in rows:
        if "excluded" in r:
            # Print the arm on this row too. A blank here is what made an
            # excluded cell look arm-less on inspection even after the record
            # itself carried the arm.
            print(f"{r.get('run','?'):34s} {str(r.get('arm')):11s} "
                  f"{'EXCLUDED':>9s} "
                  f"{'':>9s} {'':>9s} {'':>7s} {'':>5s}  {r['excluded']}")
            continue
        tt = "CENSORED" if r["t_team"] is None else f"{r['t_team']:9.1f}"
        te = "-" if r.get("t_explore") is None else f"{r['t_explore']:9.1f}"
        tm = "-" if r.get("t_mission") is None else f"{r['t_mission']:9.1f}"
        note = ("censored: " + ",".join(r["censored_robots"])) if r["censored"] else ""
        # On a mission-return run a "-" in t_missn deserves its reason.
        if r.get("t_mission") is None and any(r.get("mission_results", {}).values()):
            bad = [f"{rob}:{res or 'none'}"
                   for rob, res in r["mission_results"].items()
                   if res != "arrived"]
            note = (note + " " if note else "") + "mission " + ",".join(bad)
        print(f"{r['run']:34s} {str(r['arm']):11s} {tt:>9s} "
              f"{te:>9s} {tm:>9s} {r['resume_delta_total_sec']:7.1f} "
              f"{r['rtf_mean']:5.3f}  {note}")

    summ = arm_summary(rows)
    print()
    print(f"{'arm':11s} {'n':>3s} {'done':>5s} {'cens':>5s} {'excl':>5s} "
          f"{'mean t_team':>12s} {'mean t_expl':>12s} "
          f"{'mean t_missn':>12s} {'m.cens':>6s}")
    for s in summ:
        mt = "-" if s["mean_t_team"] is None else f"{s['mean_t_team']:12.1f}"
        me = ("-" if s["mean_t_explore"] is None
              else f"{s['mean_t_explore']:12.1f}")
        mm = ("-" if s["mean_t_mission"] is None
              else f"{s['mean_t_mission']:12.1f}")
        print(f"{s['arm']:11s} {s['n']:3d} {s['finished']:5d} "
              f"{s['censored']:5d} {s['excluded']:5d} {mt:>12s} "
              f"{me:>12s} {mm:>12s} {s['mission_censored']:6d}")
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
