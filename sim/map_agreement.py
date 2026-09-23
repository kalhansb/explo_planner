#!/usr/bin/env python3
# Moved comments: docs/sim_notes/map_agreement_notes.md
"""Measure how far apart the robots' copies of the merged map ended.

Prints one TAB-separated line (PASS/INFO <name> <detail>) into comms_gates.txt.
It REPORTS; it does not fail runs. Read on for why that changed.

TWO THINGS ABOUT THE TOKEN AND THE TEAM SIZE, both fixed together because they
were the same mistake in two places.

It used to print UNRUN when it could not compute, and UNRUN is not a token a
report-only check is allowed to emit. run_explo_sim_rviz.sh counts `^UNRUN`
lines and scores the whole cell SUSPECT if any exist, and gate_g8.py hard-fails
any cell whose manifest is not CLEAN — so a check that "does not fail runs"
failed runs, through a counter it was never meant to reach. INFO is the token
for "this report has nothing to report": visible in the file, ignored by the
verdict, and — unlike PASS — never mistakable for a check that ran and was
satisfied.

It also used to demand EXACTLY TWO planner CSVs, which is how every N>=3 cell in
ts1b acquired `UNRUN map_agree expected 2 planner_*.csv, found 3` and through it
a SUSPECT verdict, and through THAT a gate_g8 hard failure on 100% of cells of a
campaign whose maps were fine. The two-robot restriction was never inherent: the
quantity is the SPREAD of one number (total_observed_voxels) across the team,
and (max-min)/max is the same arithmetic for any N. At N=2 it is identical to
the old |a-b|/max, so banked two-robot numbers are unchanged — verified by
diffing the emitted line before and after on a banked cell.

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
import json
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


def gap_pct(values):
    """Spread across the team as a percentage of the largest map.

    (max - min) / max. At N=2 this is exactly the |a-b|/max this function used
    to take as two positional arguments, so the number printed for a two-robot
    cell is unchanged.

    Why the extreme pair and not, say, the mean absolute deviation: the question
    is "did any robot's copy fall behind", and one lagging robot in a team of
    four is the whole finding. A mean would divide that robot's gap by four and
    report a quarter of it, which is the direction that hides the defect.

    `values` may hold None for robots with no sample at this instant; they are
    dropped, and fewer than two survivors means there is nothing to compare, for
    which the answer is None rather than 0.0. Zero is the value that means "the
    maps agree perfectly", and it must not double as "there was no comparison".
    """
    vals = [v for v in values if v is not None]
    if len(vals) < 2:
        return None
    hi, lo = max(vals), min(vals)
    return (hi - lo) / hi * 100.0 if hi > 0 else 0.0


def latch_time(outdir):
    """Last exploration_complete over both robots, or None (pre-v2 logs).

    On a mission-return run the CSVs run on through the homing leg, so the
    end-of-run gap is measured AFTER the regroup -- a near-zero there is the
    mission return doing its job, and it says nothing about how far apart the
    maps were when exploring stopped. That instant is reported separately.
    """
    t = None
    for p in glob.glob(os.path.join(outdir, "*.events.jsonl")):
        with open(p, errors="replace") as fh:
            for ln in fh:
                try:
                    e = json.loads(ln)
                except ValueError:
                    continue
                if e.get("event") == "exploration_complete":
                    s = float(e["t_sim_sec"])
                    t = s if t is None else max(t, s)
    return t


def roster(outdir):
    """The team, taken from the EVENT LOGS rather than from the CSVs.

    The spread is read off the extreme pair, and the docstring above defends
    that choice on the grounds that "one lagging robot in a team of four is the
    whole finding". Deriving the roster from the planner_*.csv files that were
    FOUND makes exactly that robot unnameable: a planner that died before
    writing its header leaves no file, the glob returns the survivors, and the
    spread is computed over a subset with nothing on the line to say so.

    Reproduced on ts1b_n4_mtare_hybrid_r40_ttl0_seed1 with two CSVs removed:
    PASS over a 2-robot subset, peak 26.70% -> 5.52%, at-latch 2.77% -> 0.16%,
    no dropped clause and no robot count anywhere in the output. `empty` does
    not cover it -- that is exists-but-no-rows -- and len(paths) < 2 only
    catches the collapse all the way down to one.

    Every robot opens its event log at start-up, before any planning, so the
    log existing is a weaker condition than the CSV existing and the roster is
    the right side to take. Returns [] when there is no event log at all, which
    is a THIRD state (roster unknown, not roster empty) and is reported as such
    by the caller -- absence of the check is not a pass.
    """
    return sorted({os.path.basename(p)[:-len(".events.jsonl")]
                   for p in glob.glob(os.path.join(outdir,
                                                   "*.events.jsonl"))})


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("outdir", help="run directory holding planner_*.csv")
    ap.add_argument("--name", default="map_agree", help="gate name")
    ap.add_argument("--step", type=float, default=100.0,
                    help="sampling step for the peak/trace, sim seconds")
    ap.add_argument("--start", type=float, default=200.0,
                    help="ignore before this sim time (both maps still filling)")
    # No --max-pct option, on purpose: this check is report-only, so a banked
    # command line carrying it must fail as an unrecognized argument rather than
    # look like a live threshold. (notes: map-agree-no-max-pct)
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.outdir, "planner_*.csv")))
    names = [os.path.basename(p)[len("planner_"):-len(".csv")] for p in paths]
    # Roster first, so every message below can say what the team WAS rather
    # than only what it found. See roster().
    team = roster(args.outdir)
    absent = [n for n in team if n not in set(names)]
    if not team:
        roster_note = (" (roster unknown: no *.events.jsonl here, so a robot "
                       "whose planner_*.csv is entirely absent cannot be named "
                       "— this is not a statement that none is)")
    elif absent:
        roster_note = (f" (team of {len(team)} per the event logs; NO "
                       f"planner_*.csv at all for: {','.join(absent)} — the "
                       f"numbers here cover a SUBSET of the team)")
    else:
        roster_note = f" (all {len(team)} robots of the team present)"
    # Two series is the minimum for a spread, not a shape limit. Return 0 on
    # every path, INFO included: this is report-only, and the || true in
    # run_explo_sim_rviz.sh must not be what keeps that.
    # (notes: map-agree-two-series-exit-zero)
    if len(paths) < 2:
        print(f"INFO\t{args.name}\tfound {len(paths)} planner_*.csv; a map "
              f"spread needs at least two robots to compare — nothing to "
              f"report{roster_note}")
        return 0

    ser = [series(p) for p in paths]
    empty = [n for n, x in zip(names, ser) if not x]
    if len(names) - len(empty) < 2:
        print(f"INFO\t{args.name}\tno voxel counts logged for: "
              f"{','.join(empty)} — fewer than two comparable "
              f"series{roster_note}")
        return 0
    # A robot whose CSV has no rows is dropped and named, never silently. Kept
    # apart from a robot with no CSV at all (roster_note): header-only means the
    # planner ran, no file means it did not.
    # (notes: map-agree-empty-vs-absent-csv)
    dropped = f" (dropped, no voxel counts: {','.join(empty)})" if empty else ""
    dropped += roster_note
    keep = [(n, x) for n, x in zip(names, ser) if x]
    names = [n for n, _ in keep]
    ser = [x for _, x in keep]

    times = [[x[0] for x in sxx] for sxx in ser]
    # Latest sim time EVERY robot reported, so none is credited with world the
    # others had no chance to log. In practice they stop within a few seconds.
    t_end = min(tt[-1] for tt in times)
    vals = [at(sxx, tt, t_end) for sxx, tt in zip(ser, times)]
    end = gap_pct(vals)
    if end is None:
        print(f"INFO\t{args.name}\tno common sample time across "
              f"{len(names)} robot(s) — nothing to report{roster_note}")
        return 0

    peak = end
    t_peak = t_end
    t = args.start
    while t <= t_end:
        g = gap_pct([at(sxx, tt, t) for sxx, tt in zip(ser, times)])
        if g is not None and g > peak:
            peak, t_peak = g, t
        t += args.step

    drained = "drained" if end <= peak * 0.5 else "still open"
    # The at-latch reading only exists where the event log does; "-" otherwise.
    t_latch = latch_time(args.outdir)
    at_latch = ""
    if t_latch is not None and t_latch <= t_end:
        g = gap_pct([at(sxx, tt, t_latch) for sxx, tt in zip(ser, times)])
        if g is not None:
            at_latch = (f", at-latch {g:.2f}% at t={t_latch:.0f}s"
                        f" (gap when exploring stopped; `end` is after any"
                        f" mission-return regroup)")
    counts = " ".join(f"{n}={v:.0f}" for n, v in zip(names, vals)
                      if v is not None)
    print(f"PASS\t{args.name}\t{len(names)} series {counts} at t={t_end:.0f}s: "
          f"end {end:.2f}%, peak {peak:.2f}% at t={t_peak:.0f}s ({drained})"
          f"{at_latch}{dropped} — "
          f"REPORT ONLY, not a validity gate: this tracks undrained backlog at "
          f"the stop instant, which scales with outage severity")
    return 0


if __name__ == "__main__":
    sys.exit(main())
