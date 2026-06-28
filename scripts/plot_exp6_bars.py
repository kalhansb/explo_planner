#!/usr/bin/env python3
"""Exp 6 summary bar plot — F-score per trial for solo vs fused maps."""
import sys
sys.path = [p for p in sys.path if ".local" not in p]
import csv, os
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RES = os.path.join(os.path.dirname(__file__), "..", "results")
OUT_DIR = os.path.join(RES, "plots_exp6")
os.makedirs(OUT_DIR, exist_ok=True)

rows = {}
with open(os.path.join(RES, "exp6_fscore.csv")) as fh:
    for r in csv.DictReader(fh):
        if "smoke" in r["trial"]:
            continue
        rows.setdefault(r["trial"], {})[r["variant"]] = r

trials = sorted(rows)
configs = [t.split("_")[-1] for t in trials]

fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

# F-score bars
ax = axes[0]
x = np.arange(len(trials))
w = 0.2
variants = [("atlas_solo", "#d62728", "atlas (solo SCovox)"),
            ("bestla_solo", "#ff7f0e", "bestla (solo SCovox)"),
            ("atlas_fused", "#1f77b4", "atlas (fused DSCovox)"),
            ("bestla_fused", "#2ca02c", "bestla (fused DSCovox)")]
for i, (v, c, lbl) in enumerate(variants):
    vals = [float(rows[t][v]["fscore"]) for t in trials]
    ax.bar(x + (i - 1.5) * w, vals, w, color=c, label=lbl)
ax.set_xticks(x)
ax.set_xticklabels(configs)
ax.set_ylabel("F-score @ 0.10 m")
ax.set_title("Exp 6 — F-score, solo vs per-robot fused")
ax.set_ylim(0, 0.40)
ax.grid(True, axis="y", alpha=0.3)
ax.legend(loc="upper right", fontsize=8)

# Voxel count bars
ax = axes[1]
for i, (v, c, lbl) in enumerate(variants):
    vals = [int(rows[t][v]["voxels"]) / 1000.0 for t in trials]
    ax.bar(x + (i - 1.5) * w, vals, w, color=c, label=lbl)
ax.set_xticks(x)
ax.set_xticklabels(configs)
ax.set_ylabel("Predicted voxels in GT z-slab (×1000)")
ax.set_title("Exp 6 — coverage (GT z-slab, after TF to world)")
ax.grid(True, axis="y", alpha=0.3)

fig.suptitle("Exp 6 — two-robot fusion, flatforest, 3 start configs", y=1.02)
fig.tight_layout()
out = os.path.join(OUT_DIR, "fscore_bars.png")
fig.savefig(out, dpi=140, bbox_inches="tight")
plt.close(fig)
print(f"wrote {out}")
