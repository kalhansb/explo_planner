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
    med = statistics.median(eps) if eps else float("nan")
    return (down / total if total else float("nan")), eps, med


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

    horizon = None
    try:
        horizon = float(man.get("duration_s", "")) or None
    except ValueError:
        pass

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
    manoeuvres, reconnect_secs = 0, []
    for rows in planners.values():
        in_man = False
        for r in rows:
            s = r.get("state", "")
            if s in MANOEUVRE and not in_man:
                in_man = True
                manoeuvres += 1
            elif s not in MANOEUVRE and in_man:
                in_man = False
                try:
                    v = float(r.get("reconnect_elapsed_sec", -1))
                    if v >= 0:
                        reconnect_secs.append(v)
                except ValueError:
                    pass

    duty, eps, med_out = duty_and_outages(trace)
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
        "end_reason": man.get("run_end_reason", "?"),
        "end_t_sim": man.get("run_end_t_sim", "?"),
        "horizon": horizon,
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

    runs = []
    for name in sorted(os.listdir(args.root)):
        d = os.path.join(args.root, name)
        if os.path.isdir(d) and os.path.exists(os.path.join(d, "run_manifest.txt")):
            r = analyse_run(d, args.threshold)
            if r:
                runs.append(r)
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
        print(f"{r['run']:<28} {r['arm']:<11} {r['seed']:>4} {r['tx_power']:>6} "
              f"{ms} {str(r['censored']):>5} {duty} {r['n_outages']:>5} "
              f"{r['n_contacts']:>5} {r['n_manoeuvres']:>5}")

    print()
    by_arm = {}
    for r in runs:
        by_arm.setdefault(r["arm"], []).append(r)
    for arm, rs in sorted(by_arm.items()):
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
