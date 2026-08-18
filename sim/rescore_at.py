#!/usr/bin/env python3
"""Rescore already-collected cells at a different DONE_UNKNOWN, without re-running.

Usage: rescore_at.py [threshold]        (default 0.60)

Why this is legitimate. unknown_fraction is recorded in every step event, and a
robot's behaviour before it crosses the criterion does not depend on where the
criterion sits -- the run only consults it to decide whether to stop. So the
first crossing of a HIGHER threshold is exactly the completion time that run
would have had if it had been launched with that threshold. Rescoring downward
would not be valid; rescoring upward is.

What does NOT survive the rescore: anything integrated over the whole run
(disconnected fraction, distance, voxel totals). Those were accumulated over the
longer window the run actually executed. Truncating them to the shorter window
is possible but is not what this script does -- it reports completion only.

The other thing this buys is censored runs. p12_pursuit_seed1 hit the 5400 s
duration cap without ever reaching 0.55, so it has no completion time at all and
had to be dropped or horizon-imputed. It crosses 0.60 at 1961 s, so at the
higher criterion it is an ordinary finished run.

See criterion_choice.py for why 0.60: it cuts run-to-run SD from ~920 s to
~198 s while costing one extra unengaged run out of 17.
"""
import glob
import json
import os
import sys

ROOT = os.environ.get("HMR_CAMPAIGN_ROOT", "/tmp/hmr_campaign")
ARMS = ("off", "rendezvous", "pursuit", "hybrid")
DEFAULT_THRESH = 0.60


def load(cell):
    trace = []
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
                    trace.append((float(e["t_sim_sec"]), float(e["unknown_fraction"])))
    return sorted(trace)


def run_end(cell):
    try:
        with open(os.path.join(cell, "run_manifest.txt"), errors="ignore") as fh:
            for ln in fh:
                if ln.startswith("run_end_t_sim="):
                    return float(ln.strip().split("=", 1)[1])
    except (OSError, ValueError):
        pass
    return None


def main(argv):
    thresh = float(argv[0]) if argv else DEFAULT_THRESH
    rows = []
    for d in sorted(glob.glob(os.path.join(ROOT, "p1*_*seed*"))):
        if not os.path.isdir(d) or d.endswith(".attempts"):
            continue
        name = os.path.basename(d)
        arm = next((a for a in ARMS if f"_{a}_" in name), None)
        trace = load(d)
        if not arm or not trace:
            continue
        t_new = next((t for t, u in trace if u <= thresh), None)
        rows.append((name, arm, run_end(d), t_new, min(u for _, u in trace)))

    print(f"rescored at DONE_UNKNOWN={thresh}\n")
    print(f"{'cell':<24} {'arm':<11} {'as run':>8} {'rescored':>9} {'delta':>8}")
    for name, arm, old, new, _ in rows:
        o = f"{old:8.0f}" if old else "       -"
        n = f"{new:9.0f}" if new else "        -"
        d = f"{new - old:8.0f}" if (old and new) else "        -"
        print(f"{name:<24} {arm:<11} {o} {n} {d}")

    print(f"\n{'arm':<12} {'campaign':<5} {'n':>3} {'median':>8} {'spread':>8}")
    for tag in ("p12", "p13"):
        for arm in ARMS:
            v = sorted(r[3] for r in rows
                       if r[1] == arm and r[0].startswith(tag + "_") and r[3])
            if not v:
                continue
            print(f"{arm:<12} {tag:<5} {len(v):3d} {v[len(v) // 2]:8.0f} "
                  f"{v[-1] - v[0]:8.0f}")

    missed = [r[0] for r in rows if r[3] is None]
    if missed:
        print(f"\nnever reached {thresh}: {', '.join(missed)}")
    rescued = [r[0] for r in rows if r[2] is None and r[3] is not None]
    if rescued:
        print(f"censored as run, complete when rescored: {', '.join(rescued)}")


if __name__ == "__main__":
    main(sys.argv[1:])
