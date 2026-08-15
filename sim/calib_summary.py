#!/usr/bin/env python3
"""Score a tx_power_dbm sweep against the plan's §4 acceptance criteria.

Reads the traces written by link_logger.py during calibrate_txpower.sh and
reports, per (bag, tx_power):

  duty      fraction of link_states samples with connected == 0
  n_out     number of distinct outage episodes
  med_out   MEDIAN outage duration, seconds
  tsc_exh   time since last contact, at the moment the replayed run's planner
            exhausted exploration

Accepted iff  0.55 <= duty <= 0.70  and  med_out > 5.0  and  tsc_exh < 180.

Three things this deliberately does NOT do:

* It does not average the duty cycle across bags before testing it. Each bag is
  a different trajectory realisation and the spread between them is the honest
  uncertainty on the aiming; collapsing it first would hide a value that only
  passes on one trajectory.
* It does not interpolate between swept points to "solve" for a passing power.
  The relationship runs through a hysteresis band and an AWGN/Rayleigh model
  switch, neither of which is smooth, so an interpolated value is not a
  prediction -- if no swept point passes, the answer is to sweep more points.
* It does not treat a trailing outage as an episode of known length. An outage
  still open when the trace ends is right-censored: its true duration is at
  least what was observed. Including it as if complete drags the median down,
  and these are exactly the long outages the criterion cares about, so it is
  excluded from the median and reported separately.
"""

import argparse
import csv
import os
import statistics
import sys

DUTY_LO, DUTY_HI = 0.55, 0.70
MED_OUT_MIN = 5.0
TSC_MAX = 180.0


def read_trace(path):
    """-> [(t_sim, connected)] sorted by time, for the single robot pair."""
    out = []
    try:
        with open(path) as f:
            for row in csv.DictReader(f):
                out.append((float(row["t_sim"]), float(row["connected"]) > 0.5))
    except (OSError, KeyError, ValueError) as e:
        print(f"  ! unreadable trace {path}: {e}", file=sys.stderr)
        return []
    out.sort(key=lambda r: r[0])
    return out


def episodes(trace):
    """Outage episodes as (start_t, end_t, closed). Open final one -> closed=False."""
    eps = []
    start = None
    for t, up in trace:
        if not up and start is None:
            start = t
        elif up and start is not None:
            eps.append((start, t, True))
            start = None
    if start is not None:
        eps.append((start, trace[-1][0], False))
    return eps


def duty_cycle(trace):
    """Time-weighted, not sample-counted.

    The emulator ticks at a fixed link_rate_hz, so the two usually agree -- but
    under a replay that stalls, or a dropped tick, sample-counting silently
    reweights the trace toward whatever the publisher managed to emit. Weighting
    by the gap to the next sample makes a stalled trace read as what it was.
    """
    if len(trace) < 2:
        return float("nan")
    down = total = 0.0
    for (t0, up0), (t1, _) in zip(trace, trace[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        total += dt
        if not up0:
            down += dt
    return down / total if total > 0 else float("nan")


def exhaustion_time(run_dir):
    """Sim time at which the run's planners exhausted exploration.

    Taken as the LAST robot to first enter DONE -- the team is only exhausted
    when both are, and the §2.3 window is about the state the manoeuvre would
    have armed in. Returns None when a planner never got there (a censored run
    cannot date its own exhaustion).
    """
    times = []
    for name in os.listdir(run_dir):
        if not (name.startswith("planner_") and name.endswith(".csv")):
            continue
        first_done = None
        try:
            with open(os.path.join(run_dir, name)) as f:
                for row in csv.DictReader(f):
                    if row.get("state") == "DONE":
                        first_done = float(row["sim_time_sec"])
                        break
        except (OSError, KeyError, ValueError):
            return None
        if first_done is None:
            return None
        times.append(first_done)
    return max(times) if times else None


def time_since_contact(trace, t_at):
    """Seconds since the link was last up, evaluated at t_at."""
    if t_at is None or not trace:
        return None
    last_up = None
    for t, up in trace:
        if t > t_at:
            break
        if up:
            last_up = t
    if last_up is None:
        # Never connected before t_at: the whole prefix is one outage.
        return t_at - trace[0][0]
    return t_at - last_up


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", required=True)
    ap.add_argument("--runs", default="",
                    help="comma-separated run OUTDIRs, for exhaustion times")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    run_by_name = {}
    for d in [x for x in args.runs.split(",") if x.strip()]:
        run_by_name[os.path.basename(d.rstrip("/"))] = d.rstrip("/")

    rows = []
    with open(args.index) as f:
        for r in csv.DictReader(f):
            rows.append(r)

    lines = []
    lines.append(f"{'bag':<24} {'tx':>6} {'duty':>7} {'n_out':>6} "
                 f"{'med_out':>8} {'tsc_exh':>8}  verdict")
    lines.append("-" * 78)

    by_tx = {}
    for r in rows:
        bag, tx, path = r["bag"], float(r["tx_power_dbm"]), r["trace_csv"]
        trace = read_trace(path)
        if len(trace) < 10:
            lines.append(f"{bag:<24} {tx:>6.1f} {'--':>7} {'--':>6} "
                         f"{'--':>8} {'--':>8}  NO TRACE")
            continue
        duty = duty_cycle(trace)
        eps = episodes(trace)
        closed = [e for e in eps if e[2]]
        durs = [e[1] - e[0] for e in closed]
        med = statistics.median(durs) if durs else float("nan")
        rd = run_by_name.get(bag)
        tsc = time_since_contact(trace, exhaustion_time(rd)) if rd else None

        ok = (DUTY_LO <= duty <= DUTY_HI and med > MED_OUT_MIN
              and tsc is not None and tsc < TSC_MAX)
        why = []
        if not (DUTY_LO <= duty <= DUTY_HI):
            why.append("duty")
        if not med > MED_OUT_MIN:
            why.append("med_out")
        if tsc is None:
            why.append("tsc(n/a)")
        elif not tsc < TSC_MAX:
            why.append("tsc")
        verdict = "PASS" if ok else "fail:" + ",".join(why)
        tscs = f"{tsc:8.1f}" if tsc is not None else f"{'--':>8}"
        lines.append(f"{bag:<24} {tx:>6.1f} {duty:>7.3f} {len(closed):>6} "
                     f"{med:>8.1f} {tscs}  {verdict}")
        if len(eps) != len(closed):
            lines.append(f"{'':<24} {'':>6}   (1 open outage at trace end, "
                         f"right-censored, excluded from the median)")
        by_tx.setdefault(tx, []).append(ok)

    lines.append("")
    passing = sorted(tx for tx, oks in by_tx.items() if all(oks) and oks)
    if passing:
        # Highest passing power = weakest degradation that still meets all three.
        # Prefer it: it is the least extrapolation from the shipped radio, and
        # every criterion is a floor rather than a target to overshoot.
        lines.append(f"ACCEPTED tx_power_dbm: {passing[-1]:.1f}  "
                     f"(all bags pass; also passing: "
                     f"{', '.join(f'{p:.1f}' for p in passing[:-1]) or 'none'})")
    else:
        part = sorted(tx for tx, oks in by_tx.items() if any(oks))
        lines.append("NO tx_power_dbm passed on every bag.")
        if part:
            lines.append(f"  passed on SOME bags: "
                         f"{', '.join(f'{p:.1f}' for p in part)} — the spread "
                         f"between trajectories is the aiming uncertainty; "
                         f"sweep finer around these before picking one.")
        else:
            lines.append("  no value passed anywhere — widen --powers; the "
                         "table above shows which criterion is binding.")

    text = "\n".join(lines)
    print(text)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text + "\n")


if __name__ == "__main__":
    main()
