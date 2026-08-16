#!/usr/bin/env python3
"""Inter-robot map divergence -- the readout that separates comms conditions.

    divergence(t) = |unknown_fraction_A(t) - unknown_fraction_B(t)|

reported as the MEDIAN over matched sim times, and as a multiple of the
logger's own noise floor.

Why this and not the metrics tried before it.

  time-to-criterion   settled before the arm is ever reachable (section 3.16),
                      and its curve is flat near 0.55 so the crossing time
                      divides by a number near zero.
  final unknown       every condition stops at a different sim time, so the
                      end state scores the stopping rule as much as the run.
  team coverage       the better-informed robot advances at nearly the same
                      rate however bad the link is -- degraded comms does not
                      slow the team down, it pulls the two robots apart.

Divergence measures that last effect directly, and it is the only quantity here
with an EXACT null: if every scovox delta reaches both robots, both MapCaches
hold the same union and report the same unknown_fraction. That is a property of
delivery, not of the planner, so it survives comparison across builds -- which
matters, because the ideal-comms cells on disk predate the current build.

Measured (median divergence over the run):

    ideal comms      0.00000 .. 0.00027   n=3   below the logger noise floor
    realistic        0.01090 .. 0.05816   n=8
    no comms         0.01724 .. 0.02135   n=2

No overlap; the worst realistic run is 41x the best ideal one.

The link is what causes it, tested inside single runs rather than inferred from
the between-condition gap. Pooled over the 8 p3b cells, sustained outage onsets
raise divergence (n=22, mean +0.0128, 16/22 rose, sign p~0.026) and sustained
reconnects lower it (n=7, mean -0.0089, 5/7 fell -- right sign, too few events
to establish). Note the timescale: interval-by-interval the correlation between
connectivity and change in divergence is -0.043, i.e. nothing. A delta queued
during an outage only moves unknown_fraction once the robot covers ground, so
the response lags by minutes.

    Divergence is an integrator, not a live link monitor. Do not read a single
    sample of it as an instantaneous connectivity signal.

Three cautions the numbers above hide.

Aggregate over the run, do not classify an instant. Under realistic comms
divergence collapses to zero on every reconnect, so single-sample divergence is
not a classifier -- 7 of 8 realistic runs touch values below the ideal runs'
peak at some point. The run median separates cleanly; a spot reading does not.

Divergence is NOT monotone in severity. It is zero when the link is perfect and
near-zero again when the link never comes up at all (both robots stay equally
ignorant, symmetrically). It peaks at PARTIAL connectivity, where one robot
receives a merge burst the other misses. So it answers "is comms intact?" and
"how unequal is the team's knowledge?" -- it does not rank how bad a link is.
Read it beside the laggard's own coverage: low divergence also describes two
robots that are equally uninformed.
"""
import argparse
import csv
import glob
import os
import statistics as st


def _num(row, key):
    try:
        return float(row[key])
    except (ValueError, KeyError, TypeError):
        return None


def _series(path):
    """[(sim_time, unknown_fraction)] for one robot, blanks and zeros dropped."""
    out = []
    for r in csv.DictReader(open(path)):
        t, u = _num(r, "sim_time_sec"), _num(r, "unknown_fraction")
        if t is None or u is None or u <= 0:
            continue
        out.append((t, u))
    return out


def _at(series, t):
    """Last sample at or before t; rows land ~20 s apart so this is a hold."""
    prev = None
    for row in series:
        if row[0] > t:
            break
        prev = row
    return prev


def run_divergence(run_dir, step_s=100.0, start_s=200.0):
    """Divergence stats for one run, or None if it has no usable pair."""
    paths = sorted(glob.glob(os.path.join(run_dir, "planner_*.csv")))
    if len(paths) != 2:
        return None
    a, b = _series(paths[0]), _series(paths[1])
    if not a or not b:
        return None

    # Only compare where BOTH robots have logged; past that one series is held
    # flat at its last value and the "divergence" is just the other robot moving.
    t_max = min(a[-1][0], b[-1][0])
    div = []
    t = start_s
    while t <= t_max:
        ra, rb = _at(a, t), _at(b, t)
        if ra and rb:
            div.append(abs(ra[1] - rb[1]))
        t += step_s
    if not div:
        return None

    # Noise floor: how much one robot's own reading moves between adjacent rows.
    # A divergence below this is not a measurable disagreement.
    steps = [abs(s[i][1] - s[i - 1][1]) for s in (a, b) for i in range(1, len(s))]
    floor = st.mean(steps) if steps else 0.0
    return {
        "name": os.path.basename(run_dir),
        "median": st.median(div),
        "peak": max(div),
        "floor": floor,
        "ratio": (st.median(div) / floor) if floor else float("inf"),
        "n": len(div),
        "t_max": t_max,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dirs", nargs="*", help="run directories (default: /tmp/hmr_campaign/*)")
    ap.add_argument("--step", type=float, default=100.0, help="matched-time step (s)")
    args = ap.parse_args()

    dirs = args.dirs or sorted(glob.glob("/tmp/hmr_campaign/*"))
    rows = [r for r in (run_divergence(d.rstrip("/"), args.step)
                        for d in dirs if os.path.isdir(d)) if r]
    if not rows:
        print("no runs with a usable pair of planner_*.csv")
        return

    print(f"{'run':<26}{'median':>10}{'peak':>10}{'noise':>10}{'x noise':>10}{'pts':>6}")
    print("-" * 72)
    for r in sorted(rows, key=lambda r: r["median"]):
        ratio = "   inf" if r["ratio"] == float("inf") else f"{r['ratio']:7.1f}"
        print(f"{r['name']:<26}{r['median']:>10.5f}{r['peak']:>10.5f}"
              f"{r['floor']:>10.5f}{ratio:>10}{r['n']:>6}")
    print("\nmedian |unknown_A - unknown_B| at matched sim time.")
    print("x noise < 1 means the robots hold the same map (residual is timing jitter).")


if __name__ == "__main__":
    main()
