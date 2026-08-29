#!/usr/bin/env python3
"""Score candidate DONE_UNKNOWN values against noise and treatment engagement.

The problem this answers. At DONE_UNKNOWN=0.55 the run-to-run SD of completion
time is ~920 s, larger than any arm difference observed, so n=10 per arm could
only resolve a ~1200 s effect. coverage_floor.py shows why: 0.55 sits ~0.045
above the lowest unknown fraction ever reached here (0.5053) on a curve whose
marginal cost has already risen ~4x, so completion time is partly measuring how
long a run takes to scrape past a threshold near the floor.

Raising the criterion moves the finish line back onto the linear part of the
curve. But it cannot be raised freely, and the reason is the point of this
script: a reconnect manoeuvre only fires after a fixed span of peer silence
(240 s when this was written; 90 s since generation 9), so a
criterion set high enough ends runs BEFORE the treatment ever engages. Those
cells are noise in an arm comparison -- the robots did the same thing in every
arm. Optimising SD alone drives straight into that, since the cleanest possible
measurement is one where nothing ever happens.

So both columns have to be read together:

  thresh   SD est    treated runs finishing before their first dispatch
    0.55     920 s     3/17  (18%)
    0.58     372 s     4/17  (24%)
    0.60     198 s     4/17  (24%)
    0.62     351 s     4/17  (24%)
    0.65     272 s     4/17  (24%)
    0.70     146 s    12/17  (71%)   <-- treatment stops engaging

0.60 takes the 4.6x noise reduction for one extra unengaged run out of 17.
0.70 looks better still on SD and is worthless: 71% of its runs finish before
any robot reconnects.

Reading the SD column. It is a median|paired difference|/0.954 over the four
p12/p13 repeat pairs (off and rendezvous never enter PURSUE, so the budget
change between those campaigns cannot reach them). Four pairs is an
order-of-magnitude estimate and the 0.58-0.65 wobble is not resolvable. What is
robust is that five consecutive thresholds land at 146-372 s while 0.55 and
0.56 sit at 870-920 -- the low criterion is the outlier, not one lucky pair.

Existing runs can be rescored rather than re-run: unknown_fraction is in every
step event, and behaviour before the crossing is unaffected by where the finish
line is drawn, so t_cross is valid for a criterion the run was not launched
with. Only the metrics integrated over the whole run (disconnected fraction,
distance) need the shorter window applied.

STALE SINCE GENERATION 9 -- KEPT AS THE HISTORICAL RECORD, NOT AS ADVICE.
The whole engagement column above was computed against a 240 s mid-run clock,
and generation 9 moved it to 90. The direction of the error is known: a shorter
clock dispatches earlier, so FEWER runs finish before their first dispatch and
every "treatment stops engaging" figure here is pessimistic. The magnitude is
not known, because the counts come from campaigns that no longer exist as a
comparable population (see [[archive-pre-2026-08-20-binary]] and the four binary
generations in [[binary-generations-and-the-link-gate]]).

What this means in practice: 0.64 is the shipped DONE_UNKNOWN and it was NOT
chosen by re-running this script under the new clock. Do not read the table as
endorsing a different threshold, and do not re-derive one from it -- re-run the
script against a generation-9 campaign first. The reasoning it demonstrates,
that SD and treatment engagement have to be read together because the quietest
measurement is the one where nothing happens, is what survives.
"""
import collections
import glob
import json
import os

ROOT = os.environ.get("HMR_CAMPAIGN_ROOT", "/tmp/hmr_campaign")
CELL_GLOB = "p1*_*seed*"
ARMS = ("rendezvous", "pursuit", "hybrid", "off")
# off has reconnection disabled, so it never dispatches by design; counting it
# as "treatment did not engage" would be a tautology, not a finding.
TREATED = ("rendezvous", "pursuit", "hybrid")
CANDIDATES = (0.55, 0.56, 0.58, 0.60, 0.62, 0.65, 0.70)
# p12 vs p13 same-seed cells in these arms are repeat runs: identical binary,
# and the one parameter that differs (pursuit_budget_max_sec) cannot reach an
# arm that never enters PURSUE.
PAIR_ARMS = ("off", "rendezvous")
PAIR_SEEDS = (1, 2)
# D = X1 - X2 over repeat runs has variance 2*sigma^2, so median|D| = 0.954*sigma.
# 1.128 is the MEAN|D| constant and would understate the spread.
MED_ABS_DIFF_TO_SD = 0.954


def load(cell):
    """(unknown-fraction trace, first dispatch time or None, arm)."""
    trace, dispatches = [], []
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
                elif e.get("event") == "reconnect_dispatch":
                    dispatches.append(float(e["t_sim_sec"]))
    arm = next((a for a in ARMS if f"_{a}_" in os.path.basename(cell)), None)
    return sorted(trace), (min(dispatches) if dispatches else None), arm


def cross(trace, thresh):
    for t, u in trace:
        if u <= thresh:
            return t
    return None


def main():
    cells = {}
    for d in sorted(glob.glob(os.path.join(ROOT, CELL_GLOB))):
        if not os.path.isdir(d) or d.endswith(".attempts"):
            continue
        trace, first, arm = load(d)
        if trace and arm:
            cells[os.path.basename(d)] = (trace, first, arm)

    treated = [c for c in cells if cells[c][2] in TREATED]
    print(f"{len(cells)} cells, {len(treated)} in treated arms\n")
    print(f"{'thresh':>6} {'SD est':>8} {'pair |diffs| (s)':>26} "
          f"{'pre-treatment':>14} {'median t_cross':>15}")

    for th in CANDIDATES:
        diffs = []
        for arm in PAIR_ARMS:
            for seed in PAIR_SEEDS:
                a = cells.get(f"p12_{arm}_seed{seed}")
                b = cells.get(f"p13_{arm}_seed{seed}")
                if not a or not b:
                    continue
                ta, tb = cross(a[0], th), cross(b[0], th)
                if ta is not None and tb is not None:
                    diffs.append(abs(ta - tb))
        sd = (sorted(diffs)[len(diffs) // 2] / MED_ABS_DIFF_TO_SD
              if diffs else float("nan"))

        pre, times = 0, []
        for c in treated:
            trace, first, _ = cells[c]
            t = cross(trace, th)
            if t is None:
                continue
            times.append(t)
            if first is None or t < first:
                pre += 1
        share = f"{pre}/{len(times)} ({100 * pre / len(times):.0f}%)" if times else "-"
        med = sorted(times)[len(times) // 2] if times else float("nan")
        print(f"{th:6.2f} {sd:8.0f} {' '.join(f'{d:.0f}' for d in sorted(diffs)):>26} "
              f"{share:>14} {med:14.0f} s")

    print("\nruns that never dispatched, by arm (off is by design):")
    total, silent = collections.Counter(), collections.Counter()
    for c, (_, first, arm) in cells.items():
        total[arm] += 1
        if first is None:
            silent[arm] += 1
    for a in ARMS:
        print(f"  {a:<11} {silent[a]}/{total[a]}")

    print("\ndetectable difference at n=10 per arm (alpha .05, power .80):")
    for sd in (920, 198):
        print(f"  SD {sd:4d} s -> {sd * (15.7 / 10) ** 0.5:.0f} s")


if __name__ == "__main__":
    main()
