#!/usr/bin/env python3
"""Which metrics actually separate perfect comms from realistic comms.

map_divergence.py answers "did the two maps stay equal". This answers the
prior question: of every endpoint the campaign could report, which ones move
when the only change is whether the radio emulator is in the path?

Usage:
    ./comms_metrics.py --a "p5perfect_*" --b "p5real_*"
    ./comms_metrics.py --a /tmp/hmr_campaign/p6denseperfect_* \
                       --b /tmp/hmr_campaign/p6dense_* --label-a perfect --label-b real

THE PRIMARY ENDPOINT IS `laggard_lag`. Degraded comms does not slow exploration
down -- across every condition measured, from a 97%-connected link to a
27%-connected one, the LEADER reaches the coverage criterion at 1336-1845 s and
the ranges overlap completely. What comms slows is exploration COMPLETION. The
run cannot end until the SECOND robot independently reaches the criterion, and a
robot that never received its partner's deltas has to go and re-learn that ground
by driving over it:

    condition                leader crosses     laggard trails by
    sparse perfect            1336-1701 s              0-10 s
    sparse realistic 30 dBm   1440-1800 s               0-4 s
    dense realistic 30 dBm    1365-1845 s            10-1060 s
    sparse detuned -14 dBm    1365-1385 s            10-2045 s

The laggard is not blocked on the radio waiting for a backlog to drain. Measured
during its lag window it drives at 0.357-0.364 m/s against a 0.320-0.352 m/s
whole-run average -- full speed, the entire time. `lag_dist` prices that in robot
metres: 2-5 m under perfect comms, 378 m in the dense forest, 737 m detuned.

Three design decisions that the numbers depend on.

MATCHED HORIZON, not run end. Every run stops at a different sim time because
the stopping rule is a coverage threshold, so an endpoint read at run end
scores the stopping rule as much as the condition. Every metric below is read
at T = min over ALL runs in the comparison of the last sim time at which BOTH
that run's robots had logged. Runs that stop early therefore SHORTEN the
comparison window for everyone; the window is printed, and a run whose t_max
is far below the others is named, because one short run silently truncating
the whole battery is the easiest way to get a null out of this script.

TWO CONTROLS, read first. Neither is an endpoint.

  peer_visible_frac   MANIPULATION CHECK. Fraction of planner rows where the
                      robot could see a live peer. If this does not separate,
                      the emulator did nothing in this world and no downstream
                      metric can carry a real effect -- any that appear to are
                      noise. Check it before reading anything else.
  plan_ms_p50         NEGATIVE CONTROL. Median planner solve time. It has no
                      causal path from the radio. If it separates, the two arms
                      differed in CPU load, not in comms, and section 3.8's
                      warning applies: executor lag is indistinguishable from a
                      radio outage in this output. Every other difference in
                      the table is then suspect.

n=3 vs n=3 CANNOT PRODUCE A SIGNIFICANT RESULT. The exact permutation null over
C(6,3)=20 splits pairs each split with its complement, so the smallest
attainable two-sided p is 2/20 = 0.10. The p column is printed to show which
metrics are even at that floor; it is not evidence of significance and must not
be reported as such. What n=3 can support is SEPARATION -- whether the two
groups' values overlap at all -- which is why that column exists and is listed
first. A clean split at n=3 is a reason to run more seeds, not a result.
"""
import argparse
import csv
import glob
import itertools
import os
import statistics as st

# (key, label, higher_is, note) -- higher_is describes what a LARGER value means
METRICS = [
    ("peer_visible_frac", "peer visible frac", "more comms", "MANIPULATION CHECK"),
    ("laggard_lag",       "laggard lag s",     "worse",      "PRIMARY"),
    ("lag_dist",          "laggard drove m",   "worse",      "cost in robot-metres"),
    ("t_team_cross",      "t team complete s", "worse",      "censored -> blank"),
    ("divergence_med",    "map divergence",    "worse",      ""),
    ("unknown_lead",      "unknown (leader)",  "worse",      ""),
    ("unknown_lag",       "unknown (laggard)", "worse",      ""),
    ("unknown_auc",       "unknown AUC",       "worse",      ""),
    ("dist_team",         "team distance m",   "more effort", ""),
    ("t_to_level",        "t to level s",      "slower",     "matched on coverage"),
    ("dist_to_level",     "m to level",        "less efficient", "matched on coverage"),
    ("minpos_rej_frac",   "deconflict rej frac", "more conflict", ""),
    ("t_to_thresh",       "t LEADER cross s",  "slower",     "near-invariant"),
    ("makespan",          "makespan s",        "slower",     "confounded by stop rule"),
    ("plan_ms_p50",       "plan ms p50",       "slower CPU", "NEGATIVE CONTROL"),
]


def _num(row, key):
    try:
        return float(row[key])
    except (ValueError, KeyError, TypeError):
        return None


def _rows(path):
    out = []
    for r in csv.DictReader(open(path)):
        t = _num(r, "sim_time_sec")
        u = _num(r, "unknown_fraction")
        if t is None or u is None or u <= 0:
            continue
        out.append((t, r))
    return out


def _at(rows, t):
    """Last row at or before t. Planner rows land ~20 s apart, so this holds."""
    prev = None
    for row in rows:
        if row[0] > t:
            break
        prev = row
    return prev


def load_run(run_dir):
    paths = sorted(glob.glob(os.path.join(run_dir, "planner_*.csv")))
    if len(paths) != 2:
        return None
    a, b = _rows(paths[0]), _rows(paths[1])
    if not a or not b:
        return None
    reason = ""
    man = os.path.join(run_dir, "run_manifest.txt")
    end_t = None
    if os.path.exists(man):
        for line in open(man):
            if line.startswith("run_end_reason="):
                reason = line.split("=", 1)[1].strip()
            elif line.startswith("run_end_t_sim="):
                try:
                    end_t = float(line.split("=", 1)[1].strip())
                except ValueError:
                    pass
    return {
        "name": os.path.basename(run_dir),
        "a": a, "b": b,
        "t_max": min(a[-1][0], b[-1][0]),
        "end_reason": reason,
        "makespan": end_t,
    }


def measure(run, horizon, thresh, level, step=100.0, start=200.0):
    """The battery for one run, every time-indexed quantity read at `horizon`."""
    a, b = run["a"], run["b"]
    ra, rb = _at(a, horizon), _at(b, horizon)
    if ra is None or rb is None:
        return None
    ua, ub = _num(ra[1], "unknown_fraction"), _num(rb[1], "unknown_fraction")

    # Divergence and AUC over the matched grid, both capped at the same horizon
    # so a long run cannot contribute extra samples the short ones lack.
    div, mean_u = [], []
    t = start
    while t <= horizon:
        pa, pb = _at(a, t), _at(b, t)
        if pa and pb:
            va = _num(pa[1], "unknown_fraction")
            vb = _num(pb[1], "unknown_fraction")
            div.append(abs(va - vb))
            mean_u.append((va + vb) / 2.0)
        t += step

    # Row-level fractions, pooled over both robots, up to the horizon.
    peer_seen = tot = rej = 0
    plan_ms = []
    for series in (a, b):
        for t_row, r in series:
            if t_row > horizon:
                break
            tot += 1
            if (_num(r, "coord_active_peers") or 0) >= 1:
                peer_seen += 1
            if (_num(r, "rejected_by_minpos") or 0) >= 1:
                rej += 1
            ms = _num(r, "plan_time_ms") or 0.0
            if ms > 0:
                plan_ms.append(ms)

    dist = (_num(ra[1], "distance_traveled") or 0.0) + (_num(rb[1], "distance_traveled") or 0.0)

    lead = min(ua, ub)

    # Effort MATCHED ON COVERAGE, not on time: when the better-informed robot
    # first reached `level`, how long had the run taken and how far had the team
    # driven? An earlier version divided team distance by the unknown reduction
    # since t=200 and it separated in the WRONG DIRECTION -- because by t=200 the
    # perfect-comms robots have already merged maps, so their denominator (the
    # room left to improve) is smaller before any robot has done extra work. Any
    # ratio anchored to a condition-dependent baseline measures the baseline.
    # Anchoring on a coverage level both conditions pass through removes it.
    t_level = dist_level = None
    for t_row, r in a:
        pb = _at(b, t_row)
        if pb is None:
            continue
        if min(_num(r, "unknown_fraction") or 1.0,
               _num(pb[1], "unknown_fraction") or 1.0) <= level:
            t_level = t_row
            dist_level = (_num(r, "distance_traveled") or 0.0) + \
                         (_num(pb[1], "distance_traveled") or 0.0)
            break

    # THE PRIMARY ENDPOINT: when did each robot reach the coverage criterion?
    #
    # Degraded comms does not slow exploration down -- the LEADER's crossing time
    # barely moves between conditions. What it slows is exploration COMPLETION,
    # because the run cannot end until the second robot independently reaches the
    # criterion, and a robot that never received its partner's deltas has to go
    # and re-learn that ground by driving over it. So the quantity that carries
    # the effect is the gap between the two, not either one alone:
    #
    #   t_lead_cross   first robot to the criterion    (near-invariant)
    #   t_team_cross   LAST robot to the criterion     (the run's real end)
    #   laggard_lag    the difference                  (the cost of the outage)
    #
    # Measured over the whole run, not clipped to the matched horizon. The lag is
    # a within-run difference, so unequal run lengths do not bias it the way they
    # bias a level read at a fixed time -- but they DO censor it, which is
    # handled below.
    crossings = []
    for series in (a, b):
        hit = None
        for t_row, r in series:
            if (_num(r, "unknown_fraction") or 1.0) <= thresh:
                hit = t_row
                break
        crossings.append(hit)

    t_lead = min([c for c in crossings if c is not None], default=None)
    # CENSORING, and it matters more than anything else in this file. A run whose
    # laggard never reached the criterion is the WORST case for the condition
    # under test, not a missing observation. Dropping it would bias the whole
    # comparison towards "no effect" exactly when the effect is largest. So the
    # lag is recorded as a lower bound (last logged time minus the leader's
    # crossing) and flagged, and any group containing one reports a median that
    # is itself a lower bound.
    censored = any(c is None for c in crossings)
    if t_lead is None:
        t_team = lag = None
    elif censored:
        t_team = None
        lag = max(a[-1][0], b[-1][0]) - t_lead
    else:
        t_team = max(crossings)
        lag = t_team - t_lead

    # What the laggard actually DID with that time. It is not idling on the
    # radio: measured at 0.357-0.364 m/s against a 0.320-0.352 m/s whole-run
    # average, it drives at full speed the entire window. This is the cost in
    # robot-metres of knowledge that never arrived.
    lag_dist = None
    if t_lead is not None and lag is not None:
        slow = b if (crossings[0] is not None and
                     (crossings[1] is None or crossings[1] > crossings[0])) else a
        p0, p1 = _at(slow, t_lead), _at(slow, t_lead + lag)
        if p0 and p1:
            lag_dist = (_num(p1[1], "distance_traveled") or 0.0) - \
                       (_num(p0[1], "distance_traveled") or 0.0)

    t_cross = t_lead

    return {
        "peer_visible_frac": (peer_seen / tot) if tot else None,
        "divergence_med": st.median(div) if div else None,
        "unknown_lead": lead,
        "unknown_lag": max(ua, ub),
        "unknown_auc": st.mean(mean_u) if mean_u else None,
        "dist_team": dist,
        "t_to_level": t_level,
        "dist_to_level": dist_level,
        "minpos_rej_frac": (rej / tot) if tot else None,
        "t_to_thresh": t_cross,
        "t_team_cross": t_team,
        "laggard_lag": lag,
        "lag_dist": lag_dist,
        "censored": censored,
        "makespan": run["makespan"],
        "plan_ms_p50": st.median(plan_ms) if plan_ms else None,
    }


def perm_p(xs, ys):
    """Exact two-sided permutation p on |median difference|.

    Floors at 2/C(n,k) because every split is paired with its complement.
    """
    pool = list(xs) + list(ys)
    n = len(xs)
    obs = abs(st.median(xs) - st.median(ys))
    hits = tot = 0
    for combo in itertools.combinations(range(len(pool)), n):
        left = [pool[i] for i in combo]
        right = [pool[i] for i in range(len(pool)) if i not in combo]
        tot += 1
        if abs(st.median(left) - st.median(right)) >= obs - 1e-12:
            hits += 1
    return hits / tot if tot else 1.0


def separation(xs, ys):
    """Do the two groups' value ranges overlap at all?"""
    if max(xs) < min(ys) or max(ys) < min(xs):
        return "CLEAN"
    return "overlap"


def expand(patterns):
    out = []
    for p in patterns:
        hits = sorted(glob.glob(p if os.path.sep in p else os.path.join("/tmp/hmr_campaign", p)))
        out.extend(h.rstrip("/") for h in hits if os.path.isdir(h.rstrip("/")))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--a", nargs="+", required=True, help="group A dirs or globs")
    ap.add_argument("--b", nargs="+", required=True, help="group B dirs or globs")
    ap.add_argument("--label-a", default="A")
    ap.add_argument("--label-b", default="B")
    ap.add_argument("--threshold", type=float, default=0.55)
    ap.add_argument("--level", type=float, default=None,
                    help="coverage level for the matched-on-coverage endpoints "
                         "(default: the deepest level EVERY run reaches)")
    ap.add_argument("--step", type=float, default=100.0)
    args = ap.parse_args()

    groups = {}
    for label, pats in ((args.label_a, args.a), (args.label_b, args.b)):
        runs = [r for r in (load_run(d) for d in expand(pats)) if r]
        if not runs:
            print(f"FATAL: no usable runs for {label}: {pats}")
            return 2
        groups[label] = runs

    all_runs = groups[args.label_a] + groups[args.label_b]
    horizon = min(r["t_max"] for r in all_runs)
    binding = min(all_runs, key=lambda r: r["t_max"])

    print(f"matched horizon T = {horizon:.0f} s sim   "
          f"(set by {binding['name']}, {binding['end_reason'] or '?'})")
    spread = [f"{r['name']}:{r['t_max']:.0f}" for r in sorted(all_runs, key=lambda r: r["t_max"])]
    print("run t_max: " + "  ".join(spread))

    # The matched-on-coverage endpoints need a level EVERY run passes through,
    # otherwise the runs that never reach it drop out and the comparison is made
    # on a biased subset -- the fast runs. Take the shallowest of the runs' own
    # best levels within the window, so the worst run defines it.
    if args.level is None:
        bests = []
        for r in all_runs:
            ra, rb = _at(r["a"], horizon), _at(r["b"], horizon)
            if ra and rb:
                bests.append(min(_num(ra[1], "unknown_fraction"),
                                 _num(rb[1], "unknown_fraction")))
        level = max(bests) if bests else args.threshold
    else:
        level = args.level
    worst = max(all_runs, key=lambda r: min(
        _num(_at(r["a"], horizon)[1], "unknown_fraction"),
        _num(_at(r["b"], horizon)[1], "unknown_fraction")))
    print(f"matched coverage level = {level:.4f} unknown "
          f"(set by {worst['name']}; every run reaches it)")
    print()

    vals = {}
    for label, runs in groups.items():
        for r in runs:
            m = measure(r, horizon, args.threshold, level, args.step)
            if m:
                vals[(label, r["name"])] = m

    # Per-run table
    keys = [k for k, _, _, _ in METRICS]
    print(f"{'run':<26}" + "".join(f"{k[:11]:>13}" for k in keys))
    print("-" * (26 + 13 * len(keys)))
    for label in (args.label_a, args.label_b):
        for r in groups[label]:
            m = vals.get((label, r["name"]))
            if not m:
                continue
            cells = ""
            for k in keys:
                v = m[k]
                cells += f"{'':>13}" if v is None else f"{v:>13.4f}"
            print(f"{r['name']:<26}{cells}")
    print()

    # Comparison
    print(f"{'metric':<22}{'higher =':<16}{args.label_a:>11}{args.label_b:>11}"
          f"{'delta':>11}{'sep':>9}{'perm p':>9}  note")
    print("-" * 106)
    verdict = []
    for key, label, direction, note in METRICS:
        xs = [vals[(args.label_a, r["name"])][key] for r in groups[args.label_a]
              if vals.get((args.label_a, r["name"])) and vals[(args.label_a, r["name"])][key] is not None]
        ys = [vals[(args.label_b, r["name"])][key] for r in groups[args.label_b]
              if vals.get((args.label_b, r["name"])) and vals[(args.label_b, r["name"])][key] is not None]
        if len(xs) < 2 or len(ys) < 2:
            print(f"{label:<22}{direction:<16}{'':>11}{'':>11}{'':>11}"
                  f"{'n<2':>9}{'':>9}  {note}")
            continue
        ma, mb = st.median(xs), st.median(ys)
        sep = separation(xs, ys)
        p = perm_p(xs, ys)
        rel = (mb - ma) / abs(ma) * 100.0 if ma else float("nan")
        # Ranges are printed because "CLEAN" is a statement about ORDER, not
        # about magnitude: two groups at 1.9 m and 4.5 m separate perfectly and
        # mean nothing. Without the spreads beside them, a clean split at a
        # negligible effect size reads as a finding.
        rng = f"[{min(xs):.4g}..{max(xs):.4g}] vs [{min(ys):.4g}..{max(ys):.4g}]"
        print(f"{label:<22}{direction:<16}{ma:>11.4f}{mb:>11.4f}{mb - ma:>11.4f}"
              f"{sep:>9}{p:>9.3f}  {note}{'' if not note else ' '}"
              f"{'' if abs(rel) != abs(rel) else f'({rel:+.0f}%)'}  {rng}")
        if sep == "CLEAN":
            verdict.append((label, ma, mb, note))

    print()
    for label in (args.label_a, args.label_b):
        cens = [r["name"] for r in groups[label]
                if vals.get((label, r["name"])) and vals[(label, r["name"])]["censored"]]
        if cens:
            print(f"CENSORED in {label} ({len(cens)}/{len(groups[label])}): "
                  + ", ".join(cens))
            print(f"  their laggard never reached the criterion, so their lag is a "
                  f"LOWER BOUND and {label}'s median lag is a lower bound too. "
                  f"These are the worst cases for the condition, not missing data.")
    p_floor = 2.0 / len(list(itertools.combinations(
        range(len(groups[args.label_a]) + len(groups[args.label_b])),
        len(groups[args.label_a]))))
    print(f"permutation p floor at this n is {p_floor:.3f} — nothing here can be "
          f"significant; read the sep column.")
    if not verdict:
        print(f"NO metric separates {args.label_a} from {args.label_b} at this n.")
    else:
        print("cleanly separated: " + ", ".join(v[0] for v in verdict))
        for lab, _, _, note in verdict:
            if "CONTROL" in note:
                print(f"  !! {lab} is the {note.lower()} — see this file's docstring "
                      f"before trusting any other row.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
