#!/usr/bin/env python3
"""Locate each run's single worst patch of coverage progress, and time it.

Replaces an earlier, wrong explanation of why some runs take 2-5x longer than
others. That explanation said the long runs get bogged down mopping up the last
few percent of the map. They never get near it: every run terminates at
DONE_UNKNOWN=0.55, i.e. with 55% of the ROI still unknown, because trunk
shadowing makes most of flatforest_dense permanently unobservable.

What this measures instead. Cut the unknown-fraction trace into bands and find
the band each run spent longest inside. The result separates cleanly:

  short runs (t < 2000 s)   worst stall ~341 s = 24% of the run, usually
                            mid-run around 0.70->0.65
  long runs  (t >= 2000 s)  worst stall ~1404 s = 56% of the run, and in
                            EVERY case inside the last three bands

So a long run is not uniformly slow. It is an ordinary run plus one long stall
immediately before the criterion -- p13_rendezvous_seed1 led its same-seed twin
at every checkpoint down to 0.70, then lost 1404 s on a single band.

t_0.55 vs t_done confirms the criterion is what terminates a run: they sit
30-380 s apart in 22 of 23 cells. The lone exception (p12_pursuit_seed1, 1236 s)
is the run that also censored at the duration cap.

Both robots' step events feed one merged trace, since unknown_fraction is
computed against the shared ROI and either robot crossing an edge counts.
Crossings are first-passage: a band with no clean crossing yields no stall
number for that run rather than a fabricated one.
"""
import glob
import json
import os

ROOT = os.environ.get("HMR_CAMPAIGN_ROOT", "/tmp/hmr_campaign")
TAGS = ("p12", "p13", "p13smoke")
CRITERION = 0.55
LONG_SEC = 2000.0
# Wider early, narrow late: the late bands are where the cost per 0.01 rises and
# splitting them keeps one slow band from being averaged away by a fast one.
BANDS = [(0.90, 0.85), (0.85, 0.80), (0.80, 0.75), (0.75, 0.70), (0.70, 0.65),
         (0.65, 0.62), (0.62, 0.60), (0.60, 0.58), (0.58, 0.56), (0.56, 0.55)]


def run_end(cell):
    try:
        with open(os.path.join(cell, "run_manifest.txt"), errors="ignore") as fh:
            for ln in fh:
                if ln.startswith("run_end_t_sim="):
                    return float(ln.strip().split("=", 1)[1])
    except (OSError, ValueError):
        pass
    return None


def trace(cell):
    pts = []
    for rob in ("atlas", "bestla"):
        p = os.path.join(cell, f"{rob}.events.jsonl")
        if not os.path.exists(p):
            continue
        with open(p, errors="ignore") as fh:
            for ln in fh:
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                if e.get("event") == "step" and e.get("unknown_fraction") is not None:
                    pts.append((float(e["t_sim_sec"]), float(e["unknown_fraction"])))
    pts.sort()
    return pts


def main():
    rows = []
    for tag in TAGS:
        for cell in sorted(glob.glob(os.path.join(ROOT, tag + "_*"))):
            if not os.path.isdir(cell) or cell.endswith(".attempts"):
                continue
            end = run_end(cell)
            pts = trace(cell)
            if end is None or not pts:
                continue

            def cross(level):
                for t, u in pts:
                    if u <= level:
                        return t
                return None

            worst_sec, worst_band = 0.0, None
            for hi, lo in BANDS:
                a, b = cross(hi), cross(lo)
                if a is not None and b is not None and b - a > worst_sec:
                    worst_sec, worst_band = b - a, f"{hi:.2f}->{lo:.2f}"
            rows.append((end, os.path.basename(cell), worst_sec, worst_band,
                         cross(CRITERION)))
    rows.sort()

    print(f"{'cell':<24} {'t_done':>7} {'worst band':>12} {'stall_s':>8} "
          f"{'stall%':>7} {'t_crit':>7} {'tail_s':>7}")
    for end, name, sec, band, tc in rows:
        tail = f"{end - tc:7.0f}" if tc is not None else "      -"
        print(f"{name:<24} {end:7.0f} {str(band):>12} {sec:8.0f} "
              f"{100 * sec / end:6.0f}% {(f'{tc:7.0f}' if tc else '      -')} {tail}")

    def med(v):
        return sorted(v)[len(v) // 2]

    for label, sel in (("short", [r for r in rows if r[0] < LONG_SEC]),
                       ("long ", [r for r in rows if r[0] >= LONG_SEC])):
        if not sel:
            continue
        print(f"\n{label} n={len(sel)}: median worst-band stall "
              f"{med([r[2] for r in sel]):.0f}s "
              f"({100 * med([r[2] for r in sel]) / med([r[0] for r in sel]):.0f}%"
              " of run)")
        bands = sorted({r[3] for r in sel if r[3]})
        print(f"  stall bands: {', '.join(bands)}")


if __name__ == "__main__":
    main()
