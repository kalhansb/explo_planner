#!/usr/bin/env python3
"""Per-run metrics for the comms/reconnection campaign (plan §5).

Reads a campaign root of run OUTDIRs and emits one row per run plus the
per-arm summaries the plan asks for. Inputs per run:

  planner_<robot>.csv   the planner metrics stream (timer- and step-sampled)
  link_states.csv       link_logger.py's trace, when the run recorded one
  run_manifest.txt      arm, seed, tx_power, end reason, provenance

Deliberate choices, each of which the plan calls out explicitly:

* Censoring is respected. A run that hit the horizon T without every robot
  reaching the criterion is recorded as censored, never as "finished at T".
  Censored and observed makespans are reported separately and the censored
  ones are never averaged in as if observed (§6).
* Realised severity is REPORTED, never conditioned on (§4). It is
  post-treatment — behaviour determines exposure — so it is a manipulation
  check, not a covariate, and nothing here adjusts for it.
* The startup window is masked out of link_states before anything is derived
  from it (§5.3): rows published before a pair has both poses read
  connected=0 with zeroed physics, and counting them fabricates an outage at
  t=0 plus a spurious reconnection when the first pose lands.
* Team-knowledge-complete is the LAST robot to reach the criterion, not the
  first and not the mean. The endpoint is a team makespan.
"""

import argparse
import csv
import os
import re
import statistics
import sys

# A pair is "in contact" from the emulator's point of view when the link
# carries anything at all; connected is already bandwidth > 0.
PATH_LOSS_FLOOR = 0.0     # rows at or below this were never computed


def read_manifest(run_dir):
    out = {}
    p = os.path.join(run_dir, "run_manifest.txt")
    try:
        with open(p) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                out[k] = v
    except OSError:
        pass
    return out


def read_planner(run_dir):
    """-> {robot: [rows]} with numeric fields already floated."""
    out = {}
    for name in sorted(os.listdir(run_dir)):
        if not (name.startswith("planner_") and name.endswith(".csv")):
            continue
        robot = name[len("planner_"):-len(".csv")]
        rows = []
        try:
            with open(os.path.join(run_dir, name)) as f:
                for r in csv.DictReader(f):
                    try:
                        r["_t"] = float(r["sim_time_sec"])
                        r["_unk"] = float(r["unknown_fraction"])
                        r["_dist"] = float(r["distance_traveled"])
                    except (KeyError, ValueError):
                        continue
                    rows.append(r)
        except OSError:
            continue
        if rows:
            out[robot] = rows
    return out


def read_link_trace(run_dir):
    p = os.path.join(run_dir, "link_states.csv")
    trace = []
    try:
        with open(p) as f:
            for r in csv.DictReader(f):
                try:
                    pl = float(r["path_loss_db"])
                    if pl <= PATH_LOSS_FLOOR:   # startup mask, see module docstring
                        continue
                    trace.append((float(r["t_sim"]),
                                  float(r["connected"]) > 0.5,
                                  float(r["distance_m"])))
                except (KeyError, ValueError):
                    continue
    except OSError:
        return []
    trace.sort(key=lambda x: x[0])
    return trace


def run_t0(run_dir, planners, trace):
    """Absolute sim time at which the run's clock started (the harness's T0).

    Needed because the manifest's duration_s and run_end_t_sim are RELATIVE to
    T0 while every CSV is in ABSOLUTE sim time, and T0 itself is not written to
    the manifest. Preferred source is the harness's own "sim t0=" log line;
    failing that, the earliest sample we hold, which is late by the planner's
    start-up (a few seconds) — small next to the ~60-100 s teardown tail this
    exists to cut off.
    """
    name = os.path.basename(os.path.normpath(run_dir))
    for cand in (os.path.join(run_dir, os.pardir, name + ".console.log"),
                 os.path.join(run_dir, "sim.log")):
        try:
            with open(cand, "r", errors="replace") as fh:
                m = re.search(r"sim t0=(\d+(?:\.\d+)?)", fh.read())
                if m:
                    return float(m.group(1)), "log"
        except OSError:
            continue
    firsts = [rows[0]["_t"] for rows in planners.values() if rows]
    if trace:
        firsts.append(trace[0][0])
    return (min(firsts), "inferred") if firsts else (None, "unknown")


def clip(rows, key, horizon_abs):
    """Drop samples past the run's horizon.

    The planner CSVs and the link trace keep being written through teardown —
    60-100 s of sim time in every run here — and that tail is NOT experimental
    data: the horizon already fired, and in the solo run it contains the only
    samples where bestla is under the criterion, which is enough on its own to
    turn a censored run into a reported completion.
    """
    if horizon_abs is None:
        return rows
    return [r for r in rows if key(r) <= horizon_abs]


def time_to_criterion(rows, thresh):
    """First sim time at which unknown_fraction is at or below thresh and stays
    there for the rest of the record. Requiring it to STICK matters: the
    fraction is re-measured against a merged map that can grow between samples,
    so a single dip is not the robot reaching the criterion."""
    hit = None
    for r in rows:
        if r["_unk"] < 0:          # unmeasurable this tick
            continue
        if r["_unk"] <= thresh:
            if hit is None:
                hit = r["_t"]
        else:
            hit = None
    return hit


def duty_and_outages(trace):
    if len(trace) < 2:
        return float("nan"), [], float("nan")
    down = total = 0.0
    for (t0, up0, _), (t1, _, _) in zip(trace, trace[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        total += dt
        if not up0:
            down += dt
    eps, start = [], None
    for t, up, _ in trace:
        if not up and start is None:
            start = t
        elif up and start is not None:
            eps.append(t - start)
            start = None
    # An outage still open when the trace stops is RIGHT-CENSORED, not absent.
    # Dropping it silently discarded the largest outage in the pursuit run
    # (1225 s, from t=2495 to the end) from a set whose median was 6 s, and
    # rendered a permanently-down link as "0 outages, median nan". Its duration
    # is a lower bound, so it is reported and counted but kept out of the
    # median — same discipline as calib_summary.episodes.
    open_ep = (trace[-1][0] - start) if start is not None else None
    med = statistics.median(eps) if eps else float("nan")
    return (down / total if total else float("nan")), eps, med, open_ep


def contact_events(trace):
    """Rising edges: (t, ) for each transition down -> up."""
    out, prev = [], None
    for t, up, _ in trace:
        if prev is False and up:
            out.append(t)
        prev = up
    return out


def state_at(rows, t):
    """Planner state at sim time t (last row at or before t)."""
    s = None
    for r in rows:
        if r["_t"] > t:
            break
        s = r.get("state")
    return s or "UNKNOWN"


MANOEUVRE = {"RETURN_NAV", "RETURN_SYNC", "PURSUE"}


def classify_contact(states):
    """Merge attribution (§5.3). One class per contact, per the plan's list."""
    if any(s in MANOEUVRE for s in states):
        return "deliberate"
    if any(s == "PROXIMITY_HOLD" for s in states):
        return "proximity"
    if all(s == "DONE" for s in states):
        return "opportunistic_terminal"
    return "opportunistic"


def analyse_run(run_dir, thresh):
    man = read_manifest(run_dir)
    planners = read_planner(run_dir)
    trace = read_link_trace(run_dir)
    if not planners:
        return None

    # A run with no end reason never reached one — it was interrupted. It is not
    # a censored observation (which means "ran to T without finishing") and must
    # not enter a survival analysis as one.
    if not man.get("run_end_reason"):
        return {"run": os.path.basename(os.path.normpath(run_dir)),
                "aborted": True, "gates": man.get("run_gates_verdict", "?")}

    horizon_rel = None
    try:
        horizon_rel = float(man.get("duration_s", "")) or None
    except ValueError:
        pass
    t0, t0_src = run_t0(run_dir, planners, trace)
    horizon_abs = (t0 + horizon_rel) if (t0 is not None and horizon_rel) else None
    planners = {r: clip(rows, lambda x: x["_t"], horizon_abs)
                for r, rows in planners.items()}
    planners = {r: rows for r, rows in planners.items() if rows}
    trace = clip(trace, lambda x: x[0], horizon_abs)
    if not planners:
        return None

    per_robot = {}
    for robot, rows in planners.items():
        per_robot[robot] = {
            "t_criterion": time_to_criterion(rows, thresh),
            "final_unk": rows[-1]["_unk"],
            "min_unk": min((r["_unk"] for r in rows if r["_unk"] >= 0),
                           default=float("nan")),
            "distance": rows[-1]["_dist"],
            "end_state": rows[-1].get("state", "?"),
            "prox_hold_count": rows[-1].get("prox_hold_count", "0"),
            "prox_hold_sec": rows[-1].get("prox_hold_total_sec", "0"),
        }

    hits = [v["t_criterion"] for v in per_robot.values()]
    # The team makespan is the LAST robot to get there; if any robot never
    # did, the run is censored and has no observed makespan at all.
    censored = any(h is None for h in hits)
    makespan = None if censored else max(hits)

    # Manoeuvre accounting straight off the state column.
    # reconnect_elapsed_sec must be read from INSIDE the episode, not from the
    # row that leaves it: the planner's transitionTo clears reconnect_active_
    # before that row is emitted, so the exit row always reads -1 and the old
    # ">= 0" filter discarded every value. Verified: the exit rows carry -1
    # while the last in-manoeuvre rows carry 183.08 / 124.45 / 12.85. Take the
    # largest value seen within the episode — the field counts up, so that is
    # its duration, and it also survives an episode still running at the
    # horizon (which otherwise contributes nothing at all).
    manoeuvres, reconnect_secs, open_manoeuvres = 0, [], 0
    for rows in planners.values():
        in_man, best = False, -1.0
        for r in rows:
            s = r.get("state", "")
            if s in MANOEUVRE:
                if not in_man:
                    in_man, best = True, -1.0
                    manoeuvres += 1
                try:
                    best = max(best, float(r.get("reconnect_elapsed_sec", -1)))
                except ValueError:
                    pass
            elif in_man:
                in_man = False
                if best >= 0:
                    reconnect_secs.append(best)
        if in_man:                      # still manoeuvring when the run ended
            open_manoeuvres += 1
            if best >= 0:
                reconnect_secs.append(best)

    duty, eps, med_out, open_ep = duty_and_outages(trace)
    contacts = contact_events(trace)
    attribution = {}
    for t in contacts:
        cls = classify_contact([state_at(rows, t) for rows in planners.values()])
        attribution[cls] = attribution.get(cls, 0) + 1

    return {
        "run": os.path.basename(run_dir.rstrip("/")),
        "arm": man.get("reconnect_mode_requested", "?"),
        "seed": man.get("seed", "?"),
        "tx_power": man.get("tx_power_dbm", "?"),
        # The gate verdict travels with the run. An INVALID run's maps have
        # holes in them and its unknown_fraction — the endpoint itself — is
        # wrong; summarising it alongside clean runs launders that away.
        "gates": man.get("run_gates_verdict", "?"),
        "threshold": man.get("done_unknown_fraction", "?"),
        "end_reason": man.get("run_end_reason", "?"),
        "end_t_sim": man.get("run_end_t_sim", "?"),
        "horizon_abs": horizon_abs,
        "t0_source": t0_src,
        "open_outage_s": open_ep,
        "n_open_manoeuvres": open_manoeuvres,
        "censored": censored,
        "makespan": makespan,
        "per_robot": per_robot,
        "realised_duty": duty,
        "n_outages": len(eps),
        "median_outage": med_out,
        "n_contacts": len(contacts),
        "attribution": attribution,
        "n_manoeuvres": manoeuvres,
        "reconnect_secs": reconnect_secs,
        "trees": man.get("comms_trees_loaded", "?"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--threshold", type=float, required=True,
                    help="done_unknown_fraction the runs were scored against")
    ap.add_argument("--csv", default="", help="also write a flat CSV here")
    args = ap.parse_args()

    runs, aborted = [], []
    for name in sorted(os.listdir(args.root)):
        d = os.path.join(args.root, name)
        if os.path.isdir(d) and os.path.exists(os.path.join(d, "run_manifest.txt")):
            r = analyse_run(d, args.threshold)
            if not r:
                continue
            (aborted if r.get("aborted") else runs).append(r)
    if aborted:
        # Reported, never analysed. An interrupted run is not a censored one.
        print(f"skipped {len(aborted)} interrupted run(s) with no run_end_reason: "
              + ", ".join(r["run"] for r in aborted))
        print()
    if not runs:
        print(f"no analysable runs under {args.root}", file=sys.stderr)
        return 1

    print(f"threshold (unknown_fraction) = {args.threshold}")
    print(f"{'run':<28} {'arm':<11} {'seed':>4} {'tx':>6} {'makespan':>9} "
          f"{'cens':>5} {'duty':>6} {'nout':>5} {'ncon':>5} {'nman':>5}")
    print("-" * 100)
    for r in runs:
        ms = f"{r['makespan']:9.0f}" if r["makespan"] is not None else f"{'--':>9}"
        duty = f"{r['realised_duty']:6.3f}" if r["realised_duty"] == r["realised_duty"] else f"{'--':>6}"
        # "+1" marks an outage still open at the horizon: counted, but kept out
        # of the median because its duration is only a lower bound. Without it
        # a permanently-down link prints "0 outages" next to duty 1.000.
        nout = f"{r['n_outages']}" + ("+1" if r.get("open_outage_s") else "")
        print(f"{r['run']:<28} {r['arm']:<11} {r['seed']:>4} {r['tx_power']:>6} "
              f"{ms} {str(r['censored']):>5} {duty} {nout:>5} "
              f"{r['n_contacts']:>5} {r['n_manoeuvres']:>5}")

    print()
    # Group by the CELL, not the arm alone. Keying on reconnect_mode_requested
    # by itself pooled three tx=160 controls, a tx=-60 blackout and a tx=-14
    # treatment run into one "arm off n=5" line with a realised-severity range
    # of 0.000-1.000 — three different experiments averaged into a number that
    # describes none of them. The criterion is in the key for the same reason:
    # it DEFINES the endpoint, so two thresholds are two endpoints.
    by_arm = {}
    for r in runs:
        by_arm.setdefault((r["arm"], r["tx_power"], r["threshold"]), []).append(r)
    bad = [r for r in runs if r.get("gates") in ("INVALID",)]
    if bad:
        print(f"WARNING: {len(bad)} run(s) carry run_gates_verdict=INVALID and are "
              f"summarised below anyway — their maps are missing data:")
        for r in bad:
            print(f"    {r['run']}")
        print()
    for (arm, txp, thr), rs in sorted(by_arm.items(), key=lambda kv: str(kv[0])):
        arm = f"{arm}@tx={txp},thr={thr}"
        obs = [r["makespan"] for r in rs if r["makespan"] is not None]
        ncen = sum(1 for r in rs if r["censored"])
        duties = [r["realised_duty"] for r in rs
                  if r["realised_duty"] == r["realised_duty"]]
        att = {}
        for r in rs:
            for k, v in r["attribution"].items():
                att[k] = att.get(k, 0) + v
        print(f"arm {arm:<12} n={len(rs)}  finished={len(obs)}  censored={ncen}")
        if obs:
            # Descriptive only, and only over the runs that finished — this is
            # NOT the paired estimand (§6 says that is RMST at T, computed over
            # the seed-matched pairs, not a mean of the completions).
            print(f"    time-given-success (descriptive): "
                  f"median {statistics.median(obs):.0f} s over {len(obs)} runs")
        if duties:
            print(f"    realised disconnected fraction: "
                  f"{min(duties):.3f}-{max(duties):.3f} "
                  f"(reported, never adjusted for)")
        if att:
            print(f"    contacts by class: "
                  + ", ".join(f"{k}={v}" for k, v in sorted(att.items())))

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["run", "arm", "seed", "tx_power_dbm", "end_reason",
                        "end_t_sim", "censored", "makespan_sim_s",
                        "realised_duty", "n_outages", "median_outage_s",
                        "n_contacts", "n_manoeuvres", "trees_loaded"])
            for r in runs:
                w.writerow([r["run"], r["arm"], r["seed"], r["tx_power"],
                            r["end_reason"], r["end_t_sim"], int(r["censored"]),
                            "" if r["makespan"] is None else f"{r['makespan']:.1f}",
                            f"{r['realised_duty']:.4f}", r["n_outages"],
                            f"{r['median_outage']:.2f}", r["n_contacts"],
                            r["n_manoeuvres"], r["trees"]])
        print(f"\nflat CSV -> {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
