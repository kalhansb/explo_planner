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


class EventLogError(Exception):
    pass


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
    if len(logs) != expect_robots:
        return dict(excluded=f"{len(logs)} event log(s), expected {expect_robots}",
                    run=os.path.basename(run_dir.rstrip("/")))
    try:
        per = {r: summarise_robot(p) for r, p in logs.items()}
    except EventLogError as e:
        return dict(excluded=str(e), run=os.path.basename(run_dir.rstrip("/")))

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
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dirs", nargs="+")
    ap.add_argument("--json", action="store_true", help="machine-readable")
    a = ap.parse_args(argv)

    rows = [summarise_run(d) for d in a.run_dirs]
    if a.json:
        print(json.dumps(dict(runs=rows, by_arm=arm_summary(rows)),
                         indent=1, default=str))
        return 0
    print(f"{'run':34s} {'arm':11s} {'t_team':>9s} {'t_expl':>9s} "
          f"{'t_missn':>9s} {'resume':>7s} {'rtf':>5s}  note")
    for r in rows:
        if "excluded" in r:
            print(f"{r.get('run','?'):34s} {'':11s} {'EXCLUDED':>9s} "
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
