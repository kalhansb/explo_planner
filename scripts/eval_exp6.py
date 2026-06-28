#!/usr/bin/env python3
"""Evaluate Exp 6 — two-robot fusion — against voxelised ground truth.

For every `exp6_<planner>_<world>_<config>_*.npz` snapshot quartet produced
by run_exp6_batch.sh, compute precision / recall / F-score vs a GT NPZ at
threshold 0.10 m (1 voxel at the map resolution).

Frame handling:
  scovox_node publishes its pointcloud in the `<robot>/odom` frame, which
  is anchored at that robot's spawn pose. The GT is in world frame. We
  transform predicted points: world = R(yaw_spawn) * local + (x_spawn, y_spawn, 0)
  using the spawn pose from the start-configs CSV. For fused maps (dscovox),
  consensus fusion is done in each robot's own odom frame, so we apply
  that robot's spawn transform.

  Z filter: the GT voxelisation is a ground slab (e.g. z in [0, 0.9]); the
  scovox map integrates full height. Predicted points outside the GT
  z-range are excluded before the KDTree match so canopy voxels don't get
  scored as false positives.

Usage:
  eval_exp6.py --results-dir <path> --gt <gt_voxels.npz>
               --configs <exp6_start_configs.csv>
               [--threshold 0.10] [--occ-thresh 0.5]
               [--z-min auto] [--z-max auto]
               [--out results/exp6_fscore.csv]
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
    r"^(?P<prefix>exp6\w*?)_(?P<planner>\w+?)_(?P<world>cmu_forest|flatforest)_(?P<config>c\d+)$"
)


def load_start_configs(path):
    """Return {config_id: {atlas: (x,y,yaw), bestla: (x,y,yaw)}}."""
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
    """Transform Nx3 points from <robot>/odom to world using spawn (x, y, yaw).
    Odom frame origin = spawn pose in world; z is unrotated."""
    x0, y0, yaw = spawn
    c, s = np.cos(yaw), np.sin(yaw)
    x = c * pts_local[:, 0] - s * pts_local[:, 1] + x0
    y = s * pts_local[:, 0] + c * pts_local[:, 1] + y0
    z = pts_local[:, 2]
    return np.column_stack([x, y, z]).astype(np.float32)


def load_pred_occupied(npz_path, occ_thresh, spawn, z_range):
    """Return Nx3 array of occupied voxel centres in WORLD frame, after z-filter."""
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
    """GT npz must have 'points' and 'gt_binary' (1=occ)."""
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


def find_trials(results_dir):
    """Group the four NPZs by trial stem. Matches exp6_* and exp6_smoke_*."""
    trials = {}
    for path in glob.glob(os.path.join(results_dir, "exp6*_atlas_scovox.npz")):
        stem = path[: -len("_atlas_scovox.npz")]
        trial_id = os.path.basename(stem)
        m = TRIAL_RE.match(trial_id)
        if not m:
            continue
        variants = {
            "atlas_solo":   f"{stem}_atlas_scovox.npz",
            "atlas_fused":  f"{stem}_atlas_dscovox.npz",
            "bestla_solo":  f"{stem}_bestla_scovox.npz",
            "bestla_fused": f"{stem}_bestla_dscovox.npz",
        }
        trials[trial_id] = {"variants": variants, "config_id": m["config"]}
    return trials


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--results-dir", required=True)
    p.add_argument("--gt", required=True, help="GT voxel NPZ (points + gt_binary)")
    p.add_argument("--configs", required=True, help="Start-configs CSV for spawn poses")
    p.add_argument("--threshold", type=float, default=0.10,
                   help="F-score distance threshold, m (default: 1 voxel)")
    p.add_argument("--occ-thresh", type=float, default=0.5,
                   help="Minimum occupancy_prob for a predicted voxel to count")
    p.add_argument("--z-min", type=float, default=None,
                   help="Override predicted z-min (default: GT z-min)")
    p.add_argument("--z-max", type=float, default=None,
                   help="Override predicted z-max (default: GT z-max)")
    p.add_argument("--out", default=None, help="Summary CSV (default: <results>/exp6_fscore.csv)")
    args = p.parse_args()

    gt_pts = load_gt_occupied(args.gt)
    print(f"GT: {len(gt_pts)} occupied voxels from {args.gt}")
    zlo = args.z_min if args.z_min is not None else float(gt_pts[:, 2].min())
    zhi = args.z_max if args.z_max is not None else float(gt_pts[:, 2].max())
    print(f"Predicted z-filter: [{zlo:.3f}, {zhi:.3f}]")

    spawns = load_start_configs(args.configs)
    print(f"Start configs loaded: {sorted(spawns.keys())}")

    trials = find_trials(args.results_dir)
    if not trials:
        print(f"No exp6 snapshots found under {args.results_dir}", file=sys.stderr)
        sys.exit(1)
    print(f"Found {len(trials)} trials")

    out_path = args.out or os.path.join(args.results_dir, "exp6_fscore.csv")
    with open(out_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "trial", "variant", "voxels", "precision", "recall", "fscore",
            "fused_beats_solo", "fused_adds_voxels",
        ])

        for trial_id in sorted(trials):
            variants = trials[trial_id]["variants"]
            config_id = trials[trial_id]["config_id"]
            if config_id not in spawns:
                print(f"  [WARN] no start config for {config_id} (trial {trial_id})")
                continue
            results = {}
            for name, path in variants.items():
                if not os.path.exists(path):
                    print(f"  [WARN] missing {path}")
                    continue
                # Which robot's odom frame are these points in?
                robot = "atlas" if name.startswith("atlas") else "bestla"
                spawn = spawns[config_id][robot]
                pts = load_pred_occupied(path, args.occ_thresh, spawn, (zlo, zhi))
                pre, rec, f = fscore(pts, gt_pts, args.threshold)
                results[name] = {"voxels": len(pts), "precision": pre, "recall": rec, "fscore": f}

            solo_best = max(
                results.get("atlas_solo", {}).get("fscore", 0.0),
                results.get("bestla_solo", {}).get("fscore", 0.0),
            )
            solo_best_vox = max(
                results.get("atlas_solo", {}).get("voxels", 0),
                results.get("bestla_solo", {}).get("voxels", 0),
            )

            print(f"\n=== {trial_id} ===")
            for name in ("atlas_solo", "atlas_fused", "bestla_solo", "bestla_fused"):
                r = results.get(name)
                if r is None:
                    continue
                fused = name.endswith("_fused")
                beats = fused and (r["fscore"] >= solo_best - 1e-6)
                more_vox = fused and (r["voxels"] > solo_best_vox)
                print(f"  {name:14s} voxels={r['voxels']:>9d}  "
                      f"P={r['precision']:.3f}  R={r['recall']:.3f}  F={r['fscore']:.3f}"
                      + ("  [beats solo]" if beats else "")
                      + ("  [+coverage]"  if more_vox else ""))
                writer.writerow([
                    trial_id, name, r["voxels"],
                    f"{r['precision']:.6f}", f"{r['recall']:.6f}", f"{r['fscore']:.6f}",
                    int(beats), int(more_vox),
                ])

    print(f"\nSummary written to {out_path}")


if __name__ == "__main__":
    main()
