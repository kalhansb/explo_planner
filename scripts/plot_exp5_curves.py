#!/usr/bin/env python3
"""Plot Exp 5 completeness curves from the run_exp5_batch.sh CSVs.

Produces, per world:
  - voxels vs step (mean ± std across 5 starts)
  - voxels vs distance
  - voxels vs sim_time
"""
import sys
# Drop ~/.local paths so the old numpy there doesn't shadow the system numpy
sys.path = [p for p in sys.path if ".local" not in p]

import csv, glob, os, re, collections
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")
OUT_DIR = os.path.join(RESULTS_DIR, "plots_exp5")
os.makedirs(OUT_DIR, exist_ok=True)

PATTERN = re.compile(r"^exp5_(?P<planner>\w+?)_(?P<maptype>dscovox|logodds)_(?P<world>cmu_forest|flatforest)_(?P<start>s\d+)\.csv$")

COLORS = {
    "eig_dscovox":      "#d62728",
    "frontier_dscovox": "#1f77b4",
    "entropy_dscovox":  "#2ca02c",
    "entropy_logodds":  "#8c564b",
    "ssmi_dscovox":     "#9467bd",
    "random_dscovox":   "#7f7f7f",
}
LABEL = {
    "eig_dscovox": "EIG (SCovox)",
    "frontier_dscovox": "Frontier",
    "entropy_dscovox": "Entropy (SCovox)",
    "entropy_logodds": "Entropy (LogOdds)",
    "ssmi_dscovox": "SSMI-MI",
    "random_dscovox": "Random",
}
PLANNER_ORDER = ["eig_dscovox", "frontier_dscovox", "entropy_dscovox", "ssmi_dscovox", "random_dscovox", "entropy_logodds"]


def load():
    trials = collections.defaultdict(list)  # (world, key) -> list of dicts of np arrays
    for path in sorted(glob.glob(os.path.join(RESULTS_DIR, "exp5_*.csv"))):
        m = PATTERN.match(os.path.basename(path))
        if not m:
            continue
        key = f"{m['planner']}_{m['maptype']}"
        world = m["world"]
        with open(path) as fh:
            reader = csv.DictReader(fh)
            data = {col: [] for col in reader.fieldnames}
            for row in reader:
                for col in reader.fieldnames:
                    data[col].append(float(row[col]))
        arr = {col: np.array(v) for col, v in data.items()}
        trials[(world, key)].append(arr)
    return trials


def resample(trials_list, x_key, y_key, x_grid):
    """Interpolate each trial's y onto x_grid, return (n_trials, len(x_grid)) array.
    Values beyond a trial's max x are held at the trial's final y (NaN-padded before start)."""
    out = np.full((len(trials_list), len(x_grid)), np.nan)
    for i, t in enumerate(trials_list):
        x = t[x_key]; y = t[y_key]
        if len(x) < 2:
            continue
        order = np.argsort(x)
        x = x[order]; y = y[order]
        # Interpolate; for x_grid > x.max() hold final value; for x_grid < x.min() leave NaN
        mask = x_grid <= x[-1]
        out[i, mask] = np.interp(x_grid[mask], x, y)
        out[i, ~mask] = y[-1]  # hold the final value past trial end
        # Before first sample
        before = x_grid < x[0]
        out[i, before] = np.nan
    return out


def plot_axis(ax, trials, world, x_key, x_label, y_key="total_observed_voxels"):
    # Determine x_grid across all planners for this world
    all_x_max = []
    for key in PLANNER_ORDER:
        for t in trials.get((world, key), []):
            if len(t[x_key]) > 0:
                all_x_max.append(t[x_key].max())
    if not all_x_max:
        return
    x_max = max(all_x_max)
    x_grid = np.linspace(0, x_max, 200)

    for key in PLANNER_ORDER:
        tl = trials.get((world, key), [])
        if not tl:
            continue
        # skip logodds voxels on dscovox voxel plots (different representation)
        if key == "entropy_logodds" and y_key == "total_observed_voxels":
            continue
        arr = resample(tl, x_key, y_key, x_grid)
        mean = np.nanmean(arr, axis=0)
        std = np.nanstd(arr, axis=0)
        valid = ~np.isnan(mean)
        ax.plot(x_grid[valid], mean[valid] / 1e6, color=COLORS[key], label=LABEL[key], lw=1.8)
        ax.fill_between(x_grid[valid], (mean[valid] - std[valid]) / 1e6,
                        (mean[valid] + std[valid]) / 1e6, color=COLORS[key], alpha=0.15)

    ax.set_xlabel(x_label)
    ax.set_ylabel("Observed voxels (M, dscovox)")
    ax.set_title(f"{world}")
    ax.grid(True, alpha=0.3)


def main():
    trials = load()
    worlds = sorted({w for (w, _) in trials.keys()})
    for x_key, x_label, fname in [
        ("step", "Planner step", "voxels_vs_step.png"),
        ("distance_traveled", "Distance travelled (m)", "voxels_vs_distance.png"),
        ("sim_time_sec", "Sim time (s)", "voxels_vs_simtime.png"),
    ]:
        fig, axes = plt.subplots(1, len(worlds), figsize=(6 * len(worlds), 4.5), squeeze=False)
        for ax, world in zip(axes[0], worlds):
            plot_axis(ax, trials, world, x_key, x_label)
        axes[0][-1].legend(loc="lower right", fontsize=9)
        fig.suptitle("Exp 5 — observed voxels (dscovox), mean ± 1σ over 5 starts", y=1.02)
        fig.tight_layout()
        out = os.path.join(OUT_DIR, fname)
        fig.savefig(out, dpi=140, bbox_inches="tight")
        plt.close(fig)
        print(f"wrote {out}")

    # Uncertainty evolution: mean_entropy and mean_eig over step
    for y_key, y_label, fname in [
        ("mean_entropy", "Map-wide mean per-voxel entropy (nats; more negative = more certain)", "entropy_vs_step.png"),
        ("mean_eig", "Map-wide mean EIG (expected future gain; lower = less left to learn)", "mean_eig_vs_step.png"),
    ]:
        fig, axes = plt.subplots(1, len(worlds), figsize=(6 * len(worlds), 4.5), squeeze=False)
        for ax, world in zip(axes[0], worlds):
            # x = step, interpolate to common grid
            x_max = max((trials[(world, k)][0]["step"].max() for k in PLANNER_ORDER if trials.get((world, k))), default=200)
            x_grid = np.linspace(0, x_max, 200)
            for key in PLANNER_ORDER:
                if key == "entropy_logodds": continue
                tl = trials.get((world, key), [])
                if not tl: continue
                arr = resample(tl, "step", y_key, x_grid)
                mean = np.nanmean(arr, axis=0); std = np.nanstd(arr, axis=0)
                valid = ~np.isnan(mean)
                ax.plot(x_grid[valid], mean[valid], color=COLORS[key], label=LABEL[key], lw=1.8)
                ax.fill_between(x_grid[valid], mean[valid]-std[valid], mean[valid]+std[valid],
                                color=COLORS[key], alpha=0.15)
            ax.set_xlabel("Planner step"); ax.set_ylabel(y_label); ax.set_title(world)
            ax.grid(True, alpha=0.3)
        axes[0][-1].legend(loc="best", fontsize=9)
        fig.tight_layout()
        out = os.path.join(OUT_DIR, fname)
        fig.savefig(out, dpi=140, bbox_inches="tight")
        plt.close(fig)
        print(f"wrote {out}")

    # Total accumulated information (voxels × |mean_entropy|) — combines coverage and per-voxel certainty
    fig, axes = plt.subplots(1, len(worlds), figsize=(6 * len(worlds), 4.5), squeeze=False)
    for ax, world in zip(axes[0], worlds):
        x_max = max((trials[(world, k)][0]["step"].max() for k in PLANNER_ORDER if trials.get((world, k))), default=200)
        x_grid = np.linspace(0, x_max, 200)
        for key in PLANNER_ORDER:
            if key == "entropy_logodds": continue
            tl = trials.get((world, key), [])
            if not tl: continue
            # total_info(t) = voxels(t) * (-mean_entropy(t))  [nat-voxels]
            augmented = []
            for t in tl:
                ti = dict(t)
                ti["total_info"] = t["total_observed_voxels"] * (-t["mean_entropy"])
                augmented.append(ti)
            arr = resample(augmented, "step", "total_info", x_grid)
            mean = np.nanmean(arr, axis=0); std = np.nanstd(arr, axis=0)
            valid = ~np.isnan(mean)
            ax.plot(x_grid[valid], mean[valid]/1e6, color=COLORS[key], label=LABEL[key], lw=1.8)
            ax.fill_between(x_grid[valid], (mean[valid]-std[valid])/1e6, (mean[valid]+std[valid])/1e6,
                            color=COLORS[key], alpha=0.15)
        ax.set_xlabel("Planner step")
        ax.set_ylabel("Accumulated information (M nat·voxels)")
        ax.set_title(world); ax.grid(True, alpha=0.3)
    axes[0][-1].legend(loc="best", fontsize=9)
    fig.suptitle("Exp 5 — total information = voxels × |mean_entropy|", y=1.02)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "total_info_vs_step.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")

    # Separate logodds plot (different voxel scale)
    fig, axes = plt.subplots(1, len(worlds), figsize=(6 * len(worlds), 4.5), squeeze=False)
    for ax, world in zip(axes[0], worlds):
        # re-use plot_axis logic, but filter to logodds only
        x_max_list = []
        for t in trials.get((world, "entropy_logodds"), []):
            if len(t["step"]):
                x_max_list.append(t["step"].max())
        if not x_max_list:
            continue
        x_grid = np.linspace(0, max(x_max_list), 200)
        arr = resample(trials[(world, "entropy_logodds")], "step", "total_observed_voxels", x_grid)
        mean = np.nanmean(arr, axis=0); std = np.nanstd(arr, axis=0)
        valid = ~np.isnan(mean)
        ax.plot(x_grid[valid], mean[valid] / 1e6, color=COLORS["entropy_logodds"], label=LABEL["entropy_logodds"], lw=1.8)
        ax.fill_between(x_grid[valid], (mean[valid] - std[valid]) / 1e6,
                        (mean[valid] + std[valid]) / 1e6, color=COLORS["entropy_logodds"], alpha=0.15)
        ax.set_xlabel("Planner step"); ax.set_ylabel("Observed voxels (M, logodds)")
        ax.set_title(world); ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
    fig.suptitle("Exp 5 — LogOdds baseline voxel growth (different scale)", y=1.02)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "logodds_voxels_vs_step.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")

    # LogOdds vs SCovox (entropy planner) — side-by-side on same voxel axis
    fig, axes = plt.subplots(1, len(worlds), figsize=(6 * len(worlds), 4.5), squeeze=False)
    for ax, world in zip(axes[0], worlds):
        x_max_list = []
        for key in ("entropy_dscovox", "entropy_logodds"):
            for t in trials.get((world, key), []):
                if len(t["step"]):
                    x_max_list.append(t["step"].max())
        if not x_max_list:
            continue
        x_grid = np.linspace(0, max(x_max_list), 200)
        for key, color, label in [
            ("entropy_dscovox", "#2ca02c", "Entropy on SCovox"),
            ("entropy_logodds", "#8c564b", "Entropy on LogOdds"),
        ]:
            tl = trials.get((world, key), [])
            if not tl:
                continue
            arr = resample(tl, "step", "total_observed_voxels", x_grid)
            mean = np.nanmean(arr, axis=0); std = np.nanstd(arr, axis=0)
            valid = ~np.isnan(mean)
            ax.plot(x_grid[valid], mean[valid] / 1e6, color=color, label=label, lw=1.8)
            ax.fill_between(x_grid[valid], (mean[valid] - std[valid]) / 1e6,
                            (mean[valid] + std[valid]) / 1e6, color=color, alpha=0.15)
        ax.set_xlabel("Planner step")
        ax.set_ylabel("Observed voxels (M)")
        ax.set_title(world); ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
    fig.suptitle("Exp 5 — LogOdds vs SCovox (entropy planner, 0.10 m voxels, 1800 s budget)", y=1.02)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "logodds_vs_scovox_voxels.png")
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
