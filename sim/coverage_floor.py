#!/usr/bin/env python3
"""What does it cost to remove each successive slice of unknown space?

The runs terminate at unknown_fraction <= 0.55, so they never explore "the last
10%" -- they stop with 55% of the ROI still unknown. Much of that is
permanently unobservable: trunks shadow voxels no sensor can reach, so unknown
fraction decays toward a FLOOR, not toward zero. The scenario file warns the
0.55 criterion was calibrated on 74 stems/ha and "does not transfer" to this
250 stems/ha world, and that "no criterion window exists any more" is a real
possible outcome.

If 0.55 sits just above the dense world's floor, time-to-cross is the time to
approach an asymptote, which is arbitrarily sensitive -- and the completion-time
metric would then be measuring the stopping rule, not the exploration strategy.

Test: metres driven per 0.01 of unknown fraction removed, band by band. Flat =
healthy. Exploding near 0.55 = the criterion is against the floor.
"""
import glob
import json
import os

ROOT = os.environ.get("HMR_CAMPAIGN_ROOT", "/tmp/hmr_campaign")
BANDS = [(0.90, 0.85), (0.85, 0.80), (0.80, 0.75), (0.75, 0.70), (0.70, 0.65),
         (0.65, 0.62), (0.62, 0.60), (0.60, 0.58), (0.58, 0.56), (0.56, 0.55)]

rows, floors = {b: [] for b in BANDS}, []
for tag in ("p12", "p13", "p13smoke"):
    for d in sorted(glob.glob(os.path.join(ROOT, tag + "_*"))):
        if not os.path.isdir(d) or d.endswith(".attempts"):
            continue
        tr = []
        for rob in ("atlas", "bestla"):
            p = os.path.join(d, f"{rob}.events.jsonl")
            if not os.path.exists(p):
                continue
            for ln in open(p, errors="ignore"):
                try:
                    e = json.loads(ln)
                except Exception:
                    continue
                if e.get("event") == "step" and e.get("unknown_fraction") is not None:
                    tr.append((float(e["t_sim_sec"]), float(e["unknown_fraction"]),
                               float(e.get("distance_m") or 0)))
        if not tr:
            continue
        tr.sort()
        floors.append((os.path.basename(d), min(u for _, u, _ in tr),
                       max(x for _, _, x in tr)))
        # first crossing of each band edge -> metres and seconds spent inside it
        def cross(u0):
            for t, u, x in tr:
                if u <= u0:
                    return t, x
            return None
        for hi, lo in BANDS:
            a, b = cross(hi), cross(lo)
            if a and b and b[1] >= a[1]:
                rows[(hi, lo)].append((b[1] - a[1], b[0] - a[0], (hi - lo)))

print("=== cost of each slice of coverage, pooled over 23 runs ===")
print(f"{'unknown band':<16} {'n':>3} {'med metres':>11} {'med seconds':>12} "
      f"{'m per 0.01':>11} {'s per 0.01':>11}")
med = lambda xs: sorted(xs)[len(xs) // 2]
for b in BANDS:
    v = rows[b]
    if not v:
        continue
    dm, ds, du = med([x[0] for x in v]), med([x[1] for x in v]), b[0] - b[1]
    print(f"{b[0]:.2f} -> {b[1]:.2f}    {len(v):>3} {dm:11.0f} {ds:12.0f} "
          f"{dm/(du*100):11.0f} {ds/(du*100):11.0f}")

print("\n=== lowest unknown fraction any run reached (the floor is below these) ===")
floors.sort(key=lambda x: x[1])
for name, u, x in floors[:6]:
    print(f"  {name:<26} min unknown {u:.4f}   after {x:.0f} m")
print(f"  ... {len(floors)} runs, criterion = 0.55")
