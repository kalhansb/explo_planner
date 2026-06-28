#!/usr/bin/env python3
"""Exp 7 plots — per-planner completeness curves + final F-score bars.

Reads:
  - results/exp7_<planner>_<world>_<config>_joint.csv   (from merge_exp7_csvs.py)
  - results/exp7_fscore.csv                             (from eval_exp7.py)

Produces:
  - results/plots_exp7/completeness_vs_steps.png
  - results/plots_exp7/completeness_vs_distance.png
  - results/plots_exp7/fscore_bars.png
"""
import csv
import glob
import os
import re
import sys

sys.path = [p for p in sys.path if ".local" not in p]

from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RES = os.path.join(os.path.dirname(__file__), "..", "results")
OUT_DIR = os.path.join(RES, "plots_exp7")
os.makedirs(OUT_DIR, exist_ok=True)

JOINT_RE = re.compile(
    r"^exp7_(?P<planner>\w+?)_(?P<world>cmu_forest|flatforest)_(?P<config>c\d+)_joint\.csv$"
)

PLANNER_COLORS = {
    "eig":      "#1f77b4",
    "entropy":  "#d62728",
    "frontier": "#2ca02c",
    "random":   "#7f7f7f",
}


def load_joint(path):
    steps, dists, voxels = [], [], []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            steps.append(int(r["atlas_step"]) + int(r["bestla_step"]))
            dists.append(float(r["joint_distance_m"]))
            voxels.append(int(r["joint_observed_voxels"]))
    return np.array(steps), np.array(dists), np.array(voxels)


def curves_by_planner(world):
    """Return {planner: list of (steps, dists, voxels)}."""
    out = defaultdict(list)
    for path in sorted(glob.glob(os.path.join(RES, "exp7_*_joint.csv"))):
        m = JOINT_RE.match(os.path.basename(path))
        if not m or m["world"] != world:
            continue
        out[m["planner"]].append(load_joint(path))
    return out


def mean_curve(series, x_key):
    """Given list of (steps, dists, voxels), produce a mean vs x via
    interpolation over a common x grid [0, max(x)]."""
    if not series:
        return None, None
    max_x = max(float(s[x_key][-1]) for s in series if len(s[x_key]))
    if max_x <= 0:
        return None, None
    grid = np.linspace(0, max_x, 200)
    interps = []
    for s in series:
        x, v = s[x_key], s[2]
        if len(x) < 2:
            continue
        interps.append(np.interp(grid, x, v, left=v[0], right=v[-1]))
    if not interps:
        return None, None
    return grid, np.mean(np.vstack(interps), axis=0)


def plot_completeness(world, x_key, xlabel, out_name):
    data = curves_by_planner(world)
    if not data:
        print(f"[plot_completeness] no data for world={world}")
        return
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for planner, series in sorted(data.items()):
        # Wrap tuples so mean_curve can index by x_key (0=steps, 1=dists, 2=voxels).
        grid, mean = mean_curve(series, x_key)
        if grid is None:
            continue
        ax.plot(grid, mean / 1000.0,
                color=PLANNER_COLORS.get(planner, None),
                label=f"{planner} (n={len(series)})", lw=2)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Joint observed voxels (×1000, max over robots)")
    ax.set_title(f"Exp 7 — completeness, world={world}")
    ax.grid(True, alpha=0.3)
    ax.legend()
    out = os.path.join(OUT_DIR, out_name)
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


def plot_fscore_bars():
    path = os.path.join(RES, "exp7_fscore.csv")
    if not os.path.isfile(path):
        print(f"[plot_fscore_bars] no {path}")
        return
    # group by (world, planner) -> list of fused F per trial (take max of the 2 robots' fused).
    by_key = defaultdict(lambda: defaultdict(list))  # world -> planner -> [F]
    by_trial = defaultdict(dict)
    with open(path) as fh:
        for r in csv.DictReader(fh):
            by_trial[(r["world"], r["planner"], r["config"])][r["variant"]] = r
    for (world, planner, cfg), variants in by_trial.items():
        fa = float(variants.get("atlas_fused", {}).get("fscore", 0) or 0)
        fb = float(variants.get("bestla_fused", {}).get("fscore", 0) or 0)
        by_key[world][planner].append(max(fa, fb))

    worlds = sorted(by_key)
    if not worlds:
        print("[plot_fscore_bars] no trial rows")
        return
    planners = sorted({p for w in worlds for p in by_key[w]})
    fig, ax = plt.subplots(figsize=(8, 4.5))
    x = np.arange(len(worlds))
    w = 0.8 / max(len(planners), 1)
    for i, planner in enumerate(planners):
        means = [np.mean(by_key[world].get(planner, [0])) for world in worlds]
        ax.bar(x + (i - (len(planners) - 1) / 2) * w, means, w,
               color=PLANNER_COLORS.get(planner, None), label=planner)
    ax.set_xticks(x)
    ax.set_xticklabels(worlds)
    ax.set_ylabel("Fused F-score @ 0.10 m (mean over configs)")
    ax.set_title("Exp 7 — final fused map quality")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    out = os.path.join(OUT_DIR, "fscore_bars.png")
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    for world in ("flatforest", "cmu_forest"):
        plot_completeness(world, 0, "Total planner steps (atlas + bestla)",
                          f"completeness_vs_steps_{world}.png")
        plot_completeness(world, 1, "Joint distance travelled (m)",
                          f"completeness_vs_distance_{world}.png")
    plot_fscore_bars()
