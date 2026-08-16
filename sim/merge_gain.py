#!/usr/bin/env python3
"""Score runs by what the reconnection manoeuvre can actually change.

finishOrRendezvous is reached only at exhaustion, so an arm cannot move any
quantity that is settled while the robots are still exploring -- including
time-to-criterion, the plan's stated outcome. What it can move is how much of
the teammate's map you end up holding once your own frontier is gone.

  saturation  unknown_fraction at the first post-exploration state
              (PURSUE / RETURN_NAV / RETURN_SYNC / DONE)
  final       lowest unknown_fraction the robot reaches
  gain        saturation - final, i.e. map acquired after exploring stopped

Reported per robot and summed per run: a run's team gain is what the pair
recovered between them, which is the quantity the arms compete on.
"""
import csv
import glob
import os
import sys

POST = {"PURSUE", "RETURN_NAV", "RETURN_SYNC", "DONE"}


def robot_scores(csv_path):
    rows = [r for r in csv.DictReader(open(csv_path)) if r.get("unknown_fraction")]
    vals = []
    for r in rows:
        try:
            u = float(r["unknown_fraction"])
        except ValueError:
            continue
        if u > 0:
            vals.append((r.get("state"), u))
    if not vals:
        return None
    sat = next((u for s, u in vals if s in POST), None)
    final = min(u for _, u in vals)
    if sat is None:
        return {"sat": None, "final": final, "gain": None, "reached_end": False}
    return {"sat": sat, "final": final, "gain": sat - final, "reached_end": True}


def run_scores(run_dir):
    out = {}
    for p in sorted(glob.glob(os.path.join(run_dir, "planner_*.csv"))):
        who = os.path.basename(p)[len("planner_"):-len(".csv")]
        s = robot_scores(p)
        if s:
            out[who] = s
    return out


def main(dirs):
    print(f"{'run':<26}{'robot':<9}{'saturation':>11}{'final':>9}{'gain':>9}")
    print("-" * 64)
    for d in dirs:
        sc = run_scores(d)
        if not sc:
            print(f"{os.path.basename(d):<26}(no usable planner CSV)")
            continue
        team = 0.0
        complete = True
        for who, s in sc.items():
            f = lambda v: "--" if v is None else f"{v:.4f}"
            print(f"{os.path.basename(d):<26}{who:<9}{f(s['sat']):>11}"
                  f"{f(s['final']):>9}{f(s['gain']):>9}")
            if s["gain"] is None:
                complete = False
            else:
                team += s["gain"]
        tag = "" if complete else "   (a robot never left exploration)"
        print(f"{'':<26}{'TEAM':<9}{'':>11}{'':>9}{team:>9.4f}{tag}")
    print("\ngain = unknown_fraction recovered after exploration ended; "
          "higher is a better reconnection outcome")


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        args = sorted(glob.glob("/tmp/hmr_campaign/p3b_*"))
    main([d for d in args if os.path.isdir(d)])
