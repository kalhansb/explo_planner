#!/usr/bin/env python3
"""Evaluate Exp 7 — multi-robot active exploration — against voxelised GT.

For every trial stem `exp7_<planner>_<world>_<config>` produced by
run_exp7_batch.sh, computes:

  - final F-score @ threshold m on each robot's fused (dscovox) map
  - voxel counts (solo and fused)
  - pairwise overlap between the two robots' fused maps (fraction of each
    map that falls within `threshold` m of the peer's map)

Completeness-vs-steps and vs-distance curves are read from per-step CSVs
separately (see plot_exp7_curves.py); this script produces the
end-of-trial summary only.

Frame handling mirrors eval_exp6.py: scovox publishes in <robot>/odom,
which is the robot's spawn pose in world.

Usage:
  eval_exp7.py --results-dir <path> --gt <gt_voxels.npz>
               --configs <exp7_start_configs.csv>
               [--threshold 0.10] [--out results/exp7_fscore.csv]
"""
import argparse
import csv
import glob
import os
import re
import sys

sys.path = [p for p in sys.path if ".local" not in p]

import numpy as np
from scipy.spatial import cKDTree

TRIAL_RE = re.compile(
    r"^(?P<prefix>exp7\w*?)_(?P<planner>\w+?)_(?P<world>cmu_forest|flatforest)_(?P<config>c\d+)$"
)


def load_start_configs(path):
    out = {}
    with open(path) as fh:
        for row in csv.reader(fh):
            if not row or row[0].startswith("#") or row[0] == "config_id":
                continue
            cid, ax, ay, ayaw, bx, by, byaw = row
            out[cid] = {
                "atlas":  (float(ax), float(ay), float(ayaw)),
                "bestla": (float(bx), float(by), float(byaw)),
            }
    return out


def to_world(pts_local, spawn):
    x0, y0, yaw = spawn
    c, s = np.cos(yaw), np.sin(yaw)
    x = c * pts_local[:, 0] - s * pts_local[:, 1] + x0
    y = s * pts_local[:, 0] + c * pts_local[:, 1] + y0
    z = pts_local[:, 2]
    return np.column_stack([x, y, z]).astype(np.float32)


def load_pred_occupied(npz_path, occ_thresh, spawn, z_range):
    data = np.load(npz_path)
    pts = data["points"]
    if pts.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    if "occupancy_prob" in data.files:
        pts = pts[data["occupancy_prob"] >= occ_thresh]
    pts = to_world(pts.astype(np.float32), spawn)
    if z_range is not None:
        zlo, zhi = z_range
        pts = pts[(pts[:, 2] >= zlo) & (pts[:, 2] <= zhi)]
    return pts


def load_gt_occupied(gt_path):
    data = np.load(gt_path)
    pts = data["points"]
    if "gt_binary" in data.files:
        pts = pts[data["gt_binary"] > 0.5]
    return pts.astype(np.float32)


def fscore(pred, gt, thr):
    if len(pred) == 0 or len(gt) == 0:
        return 0.0, 0.0, 0.0
    gt_tree = cKDTree(gt)
    pr_tree = cKDTree(pred)
    d_p_to_g, _ = gt_tree.query(pred, workers=-1)
    d_g_to_p, _ = pr_tree.query(gt, workers=-1)
    precision = float(np.mean(d_p_to_g < thr))
    recall = float(np.mean(d_g_to_p < thr))
    f = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
    return precision, recall, f


def overlap(pts_a, pts_b, thr):
    """Fraction of pts_a within thr of any pt in pts_b, and vice versa."""
    if len(pts_a) == 0 or len(pts_b) == 0:
        return 0.0, 0.0
    tb = cKDTree(pts_b)
    ta = cKDTree(pts_a)
    d_a, _ = tb.query(pts_a, workers=-1)
    d_b, _ = ta.query(pts_b, workers=-1)
    return float(np.mean(d_a < thr)), float(np.mean(d_b < thr))


def find_trials(results_dir):
    trials = {}
    # dscovox trials: paired scovox + dscovox snapshots per robot.
    for path in glob.glob(os.path.join(results_dir, "exp7*_atlas_scovox.npz")):
        stem = path[: -len("_atlas_scovox.npz")]
        trial_id = os.path.basename(stem)
        m = TRIAL_RE.match(trial_id)
        if not m:
            continue
        trials[trial_id] = {
            "stem": stem,
            "planner": m["planner"],
            "world": m["world"],
            "config_id": m["config"],
            "variants": {
                "atlas_solo":   f"{stem}_atlas_scovox.npz",
                "atlas_fused":  f"{stem}_atlas_dscovox.npz",
                "bestla_solo":  f"{stem}_bestla_scovox.npz",
                "bestla_fused": f"{stem}_bestla_dscovox.npz",
            },
        }
    # logodds trials: single logodds map per robot, no solo/fused split.
    for path in glob.glob(os.path.join(results_dir, "exp7*_atlas_logodds.npz")):
        stem = path[: -len("_atlas_logodds.npz")]
        trial_id = os.path.basename(stem)
        if trial_id in trials:
            continue
        m = TRIAL_RE.match(trial_id)
        if not m:
            continue
        trials[trial_id] = {
            "stem": stem,
            "planner": m["planner"],
            "world": m["world"],
            "config_id": m["config"],
            "variants": {
                "atlas_logodds":  f"{stem}_atlas_logodds.npz",
                "bestla_logodds": f"{stem}_bestla_logodds.npz",
            },
        }
    return trials


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--results-dir", required=True)
    p.add_argument("--gt", required=True)
    p.add_argument("--configs", required=True)
    p.add_argument("--threshold", type=float, default=0.10)
    p.add_argument("--occ-thresh", type=float, default=0.5)
    p.add_argument("--z-min", type=float, default=None)
    p.add_argument("--z-max", type=float, default=None)
    p.add_argument("--out", default=None)
    args = p.parse_args()

    gt_pts = load_gt_occupied(args.gt)
    print(f"GT: {len(gt_pts)} occupied voxels from {args.gt}")
    zlo = args.z_min if args.z_min is not None else float(gt_pts[:, 2].min())
    zhi = args.z_max if args.z_max is not None else float(gt_pts[:, 2].max())
    print(f"Predicted z-filter: [{zlo:.3f}, {zhi:.3f}]")

    spawns = load_start_configs(args.configs)
    trials = find_trials(args.results_dir)
    if not trials:
        sys.exit(f"No exp7 snapshots found under {args.results_dir}")
    print(f"Found {len(trials)} trials")

    out_path = args.out or os.path.join(args.results_dir, "exp7_fscore.csv")
    with open(out_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "trial", "planner", "world", "config", "variant",
            "voxels", "precision", "recall", "fscore",
            "overlap_with_peer",
        ])

        for trial_id in sorted(trials):
            t = trials[trial_id]
            cid = t["config_id"]
            if cid not in spawns:
                print(f"  [WARN] no start config for {cid}")
                continue

            pts_cache = {}
            results = {}
            for name, path in t["variants"].items():
                if not os.path.exists(path):
                    print(f"  [WARN] missing {path}")
                    continue
                robot = "atlas" if name.startswith("atlas") else "bestla"
                spawn = spawns[cid][robot]
                pts = load_pred_occupied(path, args.occ_thresh, spawn, (zlo, zhi))
                pre, rec, f = fscore(pts, gt_pts, args.threshold)
                pts_cache[name] = pts
                results[name] = {"voxels": len(pts), "P": pre, "R": rec, "F": f}

            ov_a = ov_b = 0.0
            if "atlas_fused" in pts_cache and "bestla_fused" in pts_cache:
                ov_a, ov_b = overlap(pts_cache["atlas_fused"],
                                     pts_cache["bestla_fused"],
                                     args.threshold)

            print(f"\n=== {trial_id} ===")
            variant_order = ("atlas_solo", "atlas_fused", "bestla_solo", "bestla_fused",
                             "atlas_logodds", "bestla_logodds")
            for name in variant_order:
                r = results.get(name)
                if r is None:
                    continue
                ov = ""
                ov_val = 0.0
                if name == "atlas_fused":
                    ov_val = ov_a
                    ov = f"  overlap(a→b)={ov_a:.3f}"
                elif name == "bestla_fused":
                    ov_val = ov_b
                    ov = f"  overlap(b→a)={ov_b:.3f}"
                print(f"  {name:14s} voxels={r['voxels']:>9d}  "
                      f"P={r['P']:.3f}  R={r['R']:.3f}  F={r['F']:.3f}{ov}")
                writer.writerow([
                    trial_id, t["planner"], t["world"], cid, name,
                    r["voxels"],
                    f"{r['P']:.6f}", f"{r['R']:.6f}", f"{r['F']:.6f}",
                    f"{ov_val:.6f}",
                ])

    print(f"\nSummary written to {out_path}")


if __name__ == "__main__":
    main()
