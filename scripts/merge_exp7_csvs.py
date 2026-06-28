#!/usr/bin/env python3
"""Merge per-robot Exp 7 CSVs into a joint completeness curve.

Each per-robot CSV (written by MetricsLogger) has one row per planner step,
columns documented in metrics_logger.cpp:
  step, sim_time_sec, total_observed_voxels, frontier_voxels,
  distance_traveled, selected_score, plan_time_ms,
  mean_eig, mean_entropy, mean_variance,
  mean_info_gain, mean_path_cost,
  selected_info_gain, selected_path_cost, selected_utility,
  coord_active_peers, rejected_by_minpos, rejected_by_unreachable

We can't sum total_observed_voxels across robots (overlap is double-counted),
but each robot's per-step value approaches the global voxel count when both
share a per-robot fused DSCovox view (the design's claim C3). For the joint
trace we take the max of the two robots' total_observed_voxels at each
sim-time bucket -- a tight lower bound on the global union when the per-robot
fused views are eventually consistent (which they are under reliable QoS, see
the session-log QoS-bug session of 2026-04-08).

Sum-style columns (distance_traveled, rejected_*) are summed across robots.
Mean-style columns (mean_*, selected_*) are kept per-robot in the joint
output as separate columns so the post-hoc analyses don't lose them.

Usage:
  ./merge_exp7_csvs.py --atlas <a.csv> --bestla <r.csv> --output <joint.csv>
                       [--bucket-sec 1.0]
"""

import argparse
import csv
import math
import os
import sys


def read_rows(path):
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        return list(reader)


def to_float(s, default=0.0):
    try:
        return float(s)
    except (TypeError, ValueError):
        return default


def to_int(s, default=0):
    try:
        return int(float(s))
    except (TypeError, ValueError):
        return default


def bucket_rows(rows, bucket_sec):
    """Group rows by floor(sim_time_sec / bucket_sec).

    Returns dict: bucket_index -> last row in that bucket. We keep the
    last row because the planner is monotonic in observed_voxels and
    distance_traveled, so the latest sample within a bucket is the
    one with the most progress.
    """
    out = {}
    for r in rows:
        t = to_float(r.get("sim_time_sec"))
        b = int(math.floor(t / bucket_sec))
        out[b] = r
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--atlas", required=True, help="Path to atlas per-step CSV")
    p.add_argument("--bestla",  required=True, help="Path to bestla per-step CSV")
    p.add_argument("--output", required=True, help="Joint CSV output path")
    p.add_argument("--bucket-sec", type=float, default=1.0,
                   help="Sim-time bucket width (seconds, default 1.0)")
    args = p.parse_args()

    if not os.path.isfile(args.atlas):
        sys.exit(f"ERROR: atlas csv not found: {args.atlas}")
    if not os.path.isfile(args.bestla):
        sys.exit(f"ERROR: bestla csv not found: {args.bestla}")

    atlas_rows = read_rows(args.atlas)
    bestla_rows = read_rows(args.bestla)
    if not atlas_rows or not bestla_rows:
        sys.exit("ERROR: one or both inputs are empty")

    a_buckets = bucket_rows(atlas_rows, args.bucket_sec)
    r_buckets = bucket_rows(bestla_rows, args.bucket_sec)

    all_buckets = sorted(set(a_buckets) | set(r_buckets))

    # Forward-fill state so a bucket without a sample carries the previous
    # bucket's "best so far". This makes the joint curve monotonic in
    # observed-voxels and travelled-distance even when the two robots'
    # tick rates aren't aligned.
    last_a = None
    last_r = None

    fields = [
        "bucket_sec", "sim_time_sec",
        "atlas_step", "bestla_step",
        "atlas_observed_voxels", "bestla_observed_voxels",
        "joint_observed_voxels",     # max(a, r) -- lower bound on global union
        "atlas_distance_m", "bestla_distance_m",
        "joint_distance_m",          # sum
        "atlas_selected_info_gain", "bestla_selected_info_gain",
        "atlas_selected_path_cost", "bestla_selected_path_cost",
        "atlas_selected_utility", "bestla_selected_utility",
        "atlas_coord_active_peers", "bestla_coord_active_peers",
        "atlas_rejected_by_minpos", "bestla_rejected_by_minpos",
        "atlas_rejected_by_unreachable", "bestla_rejected_by_unreachable",
        "joint_rejected_by_minpos",  # sum
    ]

    with open(args.output, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()

        for b in all_buckets:
            if b in a_buckets:
                last_a = a_buckets[b]
            if b in r_buckets:
                last_r = r_buckets[b]
            if last_a is None or last_r is None:
                # Wait until both robots have produced at least one sample
                # so the joint columns are well-defined.
                continue

            a_obs = to_int(last_a.get("total_observed_voxels"))
            r_obs = to_int(last_r.get("total_observed_voxels"))
            a_dist = to_float(last_a.get("distance_traveled"))
            r_dist = to_float(last_r.get("distance_traveled"))
            a_minpos = to_int(last_a.get("rejected_by_minpos"))
            r_minpos = to_int(last_r.get("rejected_by_minpos"))

            row = {
                "bucket_sec": b * args.bucket_sec,
                "sim_time_sec": max(to_float(last_a.get("sim_time_sec")),
                                    to_float(last_r.get("sim_time_sec"))),
                "atlas_step": to_int(last_a.get("step")),
                "bestla_step": to_int(last_r.get("step")),
                "atlas_observed_voxels": a_obs,
                "bestla_observed_voxels": r_obs,
                "joint_observed_voxels": max(a_obs, r_obs),
                "atlas_distance_m": a_dist,
                "bestla_distance_m": r_dist,
                "joint_distance_m": a_dist + r_dist,
                "atlas_selected_info_gain": to_float(last_a.get("selected_info_gain")),
                "bestla_selected_info_gain":  to_float(last_r.get("selected_info_gain")),
                "atlas_selected_path_cost": to_float(last_a.get("selected_path_cost")),
                "bestla_selected_path_cost":  to_float(last_r.get("selected_path_cost")),
                "atlas_selected_utility":   to_float(last_a.get("selected_utility")),
                "bestla_selected_utility":    to_float(last_r.get("selected_utility")),
                "atlas_coord_active_peers": to_int(last_a.get("coord_active_peers")),
                "bestla_coord_active_peers":  to_int(last_r.get("coord_active_peers")),
                "atlas_rejected_by_minpos": a_minpos,
                "bestla_rejected_by_minpos":  r_minpos,
                "atlas_rejected_by_unreachable":
                    to_int(last_a.get("rejected_by_unreachable")),
                "bestla_rejected_by_unreachable":
                    to_int(last_r.get("rejected_by_unreachable")),
                "joint_rejected_by_minpos": a_minpos + r_minpos,
            }
            w.writerow(row)

    print(f"merged: {len(all_buckets)} buckets -> {args.output}")


if __name__ == "__main__":
    main()
