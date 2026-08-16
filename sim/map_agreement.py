#!/usr/bin/env python3
"""Measure how far apart the two robots' copies of the merged map ended.

Prints one TAB-separated line (PASS/UNRUN <name> <detail>) into comms_gates.txt.
It REPORTS; it does not fail runs. Read on for why that changed.

WHAT THIS IS NOT. It was written as a validity gate on the theory that three
dense-world runs ending 1.5-1.8 % apart proved silent map loss -- specifically a
reliable KEEP_LAST reader overflowing on the reconnect burst and discarding the
excess with no error and no counter (`drop_overflow` only ever sees the relay's
own pre-relay queue). That theory predicted the disagreement would vanish once
the two queues in the path were deepened. It did not:

    rx_qos_depth / scovox_bin_qos_depth = 500    1.76 %  1.49 %  1.78 %
    rx_qos_depth / scovox_bin_qos_depth = 4000   3.58 %  0.10 %

3.58 % at depth 4000 is worse than every run at depth 500, so overflow is not
what drives it. Nor is it a measurement artifact of comparing each robot's last
CSV row: recomputing at the latest COMMON sim time moves nothing (3.58 -> 3.60,
1.76 -> 1.76, 1.49 -> 1.49), because the two planners stop within 0.0-7.5 s of
each other.

WHAT IT ACTUALLY MEASURES. Track the gap through a run and it is plainly
transient, not cumulative -- percentages at 100 s steps over the last 600 s:

    dense realistic  27.9  28.8  36.9  38.3   7.0   2.5     <- opens, then drains
    dense realistic   8.0   8.0   6.8   6.7   5.0   1.1
    dense realistic   1.2   0.7   7.8   5.3   1.0   0.1
    dense ideal       0.0   0.0   0.0   0.0   0.0   0.0     <- never opens at all

The gap opens DURING an outage -- which is the experiment working, not failing.
Each robot keeps mapping with its own sensors while the peer's deltas sit in the
emulator's reliable queue undelivered, so the two merged maps are legitimately
different for as long as the radio is down. On reconnect the backlog drains and
the gap collapses. A 38 % mid-run gap healing to 2.5 % is a link recovering, and
under perfect comms the gap is flat zero because there is nothing to drain.

So the end-of-run number is not integrity: it is HOW MUCH BACKLOG WAS STILL
DRAINING at the instant the run hit all_done. It scales with outage severity
(sparse world 0.01-0.05 %, dense world up to 3.6 %, ideal 0.03-0.09 %), which
makes it a treatment-intensity reading -- a legitimate secondary metric for map
completeness -- and makes FAILING a run on it exactly backwards: it would
preferentially discard the runs where the comms treatment bit hardest, keeping a
matrix biased toward the cells where the radio barely mattered.

Worse, as a strict gate it was actively dangerous. run_campaign.sh aborts after
three consecutive failures, and two of the three cells that ran under it FAILed,
so it was on course to kill a 12-cell overnight matrix for a non-defect.

WHAT WOULD STILL BE A REAL DEFECT: a gap that opens and never closes while the
link is up. Deltas carry ABSOLUTE voxel state and the receiver snapshot-replaces,
so any voxel touched again self-heals; permanent loss can only survive in voxels
never revisited. `peak` and `end` below are printed together for that reason --
end << peak is a drained backlog, end ~= peak with the link long restored is
worth investigating.

Queue depth was left at 4000 rather than reverted. It is not the fix this file
once claimed it was, but a deeper queue is still cheap insurance against a burst
this experiment has no independent measurement of.
"""
import argparse
import bisect
import csv
import glob
import os
import sys


def series(path):
    """[(sim_time, total_observed_voxels)] for one planner CSV, time-sorted."""
    out = []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                t = float(row["sim_time_sec"])
                v = float(row["total_observed_voxels"])
            except (ValueError, KeyError, TypeError):
                continue
            if v > 0:
                out.append((t, v))
    out.sort()
    return out


def at(s, times, t):
    """Last value at or before t."""
    i = bisect.bisect_right(times, t) - 1
    return s[i][1] if i >= 0 else None


def gap_pct(x, y):
    hi = max(x, y)
    return abs(x - y) / hi * 100.0 if hi > 0 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("outdir", help="run directory holding planner_*.csv")
    ap.add_argument("--name", default="map_agree", help="gate name")
    ap.add_argument("--step", type=float, default=100.0,
                    help="sampling step for the peak/trace, sim seconds")
    ap.add_argument("--start", type=float, default=200.0,
                    help="ignore before this sim time (both maps still filling)")
    # Kept so existing callers that pass it keep working. Nothing is failed on
    # it any more; see the module docstring.
    ap.add_argument("--max-pct", type=float, default=None,
                    help=argparse.SUPPRESS)
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.outdir, "planner_*.csv")))
    if len(paths) != 2:
        print(f"UNRUN\t{args.name}\texpected 2 planner_*.csv, found {len(paths)}")
        return 2

    names = [os.path.basename(p)[len("planner_"):-len(".csv")] for p in paths]
    a, b = (series(p) for p in paths)
    if not a or not b:
        empty = ",".join(n for n, s in zip(names, (a, b)) if not s)
        print(f"UNRUN\t{args.name}\tno voxel counts logged for: {empty}")
        return 2

    ta = [x[0] for x in a]
    tb = [x[0] for x in b]
    # Latest sim time BOTH robots reported, so neither is credited with world the
    # other had no chance to log. In practice they stop within a few seconds.
    t_end = min(ta[-1], tb[-1])
    va, vb = at(a, ta, t_end), at(b, tb, t_end)
    end = gap_pct(va, vb)

    peak = end
    t_peak = t_end
    t = args.start
    while t <= t_end:
        xa, xb = at(a, ta, t), at(b, tb, t)
        if xa is not None and xb is not None:
            g = gap_pct(xa, xb)
            if g > peak:
                peak, t_peak = g, t
        t += args.step

    drained = "drained" if end <= peak * 0.5 else "still open"
    print(f"PASS\t{args.name}\t{names[0]}={va:.0f} {names[1]}={vb:.0f} at t={t_end:.0f}s: "
          f"end {end:.2f}%, peak {peak:.2f}% at t={t_peak:.0f}s ({drained}) — "
          f"REPORT ONLY, not a validity gate: this tracks undrained backlog at "
          f"the stop instant, which scales with outage severity")
    return 0


if __name__ == "__main__":
    sys.exit(main())
