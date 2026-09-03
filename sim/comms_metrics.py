#!/usr/bin/env python3
"""Which metrics actually separate perfect comms from realistic comms.

map_divergence.py answers "did the two maps stay equal". This answers the
prior question: of every endpoint the campaign could report, which ones move
when the only change is whether the radio emulator is in the path?

Usage:
    ./comms_metrics.py --a "p5perfect_*" --b "p5real_*"
    ./comms_metrics.py --a /tmp/hmr_campaign/p6denseperfect_* \
                       --b /tmp/hmr_campaign/p6dense_* --label-a perfect --label-b real

THE COVERAGE CRITERION REWARDS REDUNDANT COVERAGE. Read this before reading any
crossing time out of this file.

`unknown_fraction` is measured on each robot's OWN map. A robot cut off from its
partner therefore has CHEAP unknown sitting right beside it -- the ground the
partner already covered -- and drives it down fast, while a robot that already
holds the union has only the hard, far-away voxels left. Measured late in a dense
run, t = 1400 -> 2200:

    condition             robot    drove    new voxels   voxels/m
    dense perfect         atlas   253.7 m       31 912        126
    dense perfect         bestla  168.2 m       23 183        138
    dense realistic s1    bestla  174.3 m      142 823        820
    dense realistic s2    bestla   73.5 m      132 099       1796

The ideal-comms robots drive FURTHER for a sixth of the information, and the
first dense ideal cell sat at unknown 0.5504 -> 0.5502 for 1000 s while its
robots drove 254 m. It crossed the criterion at 2700 s against 1365-1845 s for
the REALISTIC dense runs. So crossing times systematically favour the degraded
arm, and a shared-map arm must not be read against a partitioned-map arm off
crossing times alone. `vox_per_m_late` is the tell; `unknown_at_dist` is the
effort-matched endpoint that charges for the driving.

WHAT `laggard_lag` STILL MEASURES. Within a condition it is real and it is the
cost of an outage: the run cannot end until the SECOND robot reaches the
criterion, and under realistic comms in the dense forest that robot trails by
10-1060 s. It is not blocked on the radio waiting for a backlog to drain --
during its lag window it drives at 0.357-0.364 m/s against a 0.320-0.352 m/s
whole-run average, full speed the entire time, and `lag_dist` prices that at up
to 378 m. What it CANNOT do is rank a shared-map arm against a partitioned-map
one, because the two arms' laggards face different amounts of cheap ground.

    condition                leader crosses     laggard trails by
    sparse perfect            1336-1701 s              0-10 s
    sparse realistic 30 dBm   1440-1800 s               0-4 s
    dense realistic 30 dBm    1365-1845 s            10-1060 s
    dense perfect             2700 s (n=1)               0 s
    sparse detuned -14 dBm    1365-1385 s            10-2045 s

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

THE p COLUMN IS modes_compare's. Exact by enumeration at the small n this file
was written for -- the 2/20 floor above is a fact about that enumeration -- and
a seeded 100000-draw sampled estimate above 200000 relabellings, which is any
comparison past about 10 per side. Sampled rows are marked with a trailing `~`
and their smallest reportable p is 1e-5. The copy that used to live here
enumerated unconditionally and would not have returned on a 30-per-arm campaign
at all. See modes_compare_calib.py for the known-answer cases.
"""
import argparse
import csv
import glob
import itertools
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modes_compare as mc          # noqa: E402

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
    ("unknown_at_dist",   "unknown @ matched m", "worse",    "effort-matched; see docstring"),
    ("vox_per_m_late",    "voxels per m late", "cheaper ground", "duplication tell"),
    ("dist_team",         "team distance m",   "more effort", ""),
    ("t_to_level",        "t to level s",      "slower",     "matched on coverage"),
    ("dist_to_level",     "m to level",        "less efficient", "matched on coverage"),
    ("minpos_rej_frac",   "deconflict rej frac", "more conflict", ""),
    ("t_to_thresh",       "t LEADER cross s",  "slower",     "near-invariant"),
    # The planner's own statement of when the team stopped trying. Prefer this
    # over makespan: makespan is the HARNESS's run-end, it is run-relative
    # rather than absolute sim, and it carries the done-grace drain plus one
    # poll period of slop -- windows whose length differs by arm, so ranking on
    # it partly ranks how long each arm idled after finishing. Blank when any
    # robot never declared; a censored run has no completion time and imputing
    # the horizon for it makes an unfinished run the fastest in its arm.
    ("t_done_team",       "t team done s",     "slower",     "PRIMARY; blank = censored"),
    ("makespan",          "makespan s",        "slower",     "harness run-end, NOT completion"),
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
    # Read from the planner's event log, where completion is stated rather than
    # inferred. None both when the log is absent (runs predating it) and when a
    # robot was censored; the metric column is blank either way, which is the
    # honest rendering of "this run has no completion time".
    t_done_team = t_explore = t_mission = None
    # WHY the exclusion reason is kept rather than dropped: a blank metric
    # column is the honest rendering of "censored", but it is a DISHONEST
    # rendering of "the event log would not parse". Those are different facts
    # and a bare `except: pass` made them identical -- a whole campaign's logs
    # could fail to load and every column would just quietly read blank, which
    # looks like censoring and would be reported as censoring.
    event_log_note = None
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import event_log
        s = event_log.summarise_run(run_dir)
        if "excluded" in s:
            event_log_note = s["excluded"]
        else:
            t_done_team = s["t_team"]
            # Mission-return endpoints (None on banked, pre-v2 logs). On those
            # runs `makespan` below includes the homing leg plus harness grace,
            # so these are the numbers a mission-return analysis should quote.
            t_explore = s.get("t_explore")
            t_mission = s.get("t_mission")
    except Exception as e:
        event_log_note = f"{type(e).__name__}: {e}"

    return {
        "name": os.path.basename(run_dir),
        "a": a, "b": b,
        "t_max": min(a[-1][0], b[-1][0]),
        "end_reason": reason,
        "makespan": end_t,
        "t_done_team": t_done_team,
        "t_explore": t_explore,
        "t_mission": t_mission,
        # None when the event log was read successfully. Non-None means the
        # three t_* fields above are blank for a reason that is NOT censoring.
        "event_log_note": event_log_note,
    }


def measure(run, horizon, thresh, level, dist_match=None, step=100.0, start=200.0):
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

    # EFFORT-MATCHED KNOWLEDGE, and the reason it exists. The coverage criterion
    # above is measured on each robot's OWN map, so it REWARDS REDUNDANT
    # COVERAGE: a robot cut off from its partner has cheap unknown right beside
    # it -- the ground the partner already covered -- and drives it down fast,
    # while a robot that already holds the union has only the hard, far-away
    # voxels left. Measured late in a dense run that is 820-1796 voxels per metre
    # against 126-138. So crossing times favour the degraded arm, and this metric
    # exists to charge for the driving: what does the team KNOW once every run
    # has spent the same robot-metres?
    #
    # It is a BOUND, not a measurement, and the bound leans the other way.
    # min(u_A, u_B) is an upper bound on the union's unknown fraction (the union
    # contains each robot's map, so it can only know more). Under perfect comms
    # the two maps are identical and the bound is TIGHT -- it is the union. Under
    # degraded comms it is loose, and loose in the pessimistic direction: the
    # degraded team really knows at least this much and possibly more. So a
    # result where the DEGRADED arm still wins on this metric is conclusive,
    # while one where the perfect arm wins is suggestive and partly the bound.
    #
    # The unbiased version needs the union map itself, which means bags. These
    # --record 0 runs cannot reconstruct it; a union-coverage re-run is the
    # outstanding fix.
    u_at_dist = None
    if dist_match:
        for t_row, r in a:
            pb = _at(b, t_row)
            if pb is None:
                continue
            team = (_num(r, "distance_traveled") or 0.0) + \
                   (_num(pb[1], "distance_traveled") or 0.0)
            if team >= dist_match:
                u_at_dist = min(_num(r, "unknown_fraction") or 1.0,
                                _num(pb[1], "unknown_fraction") or 1.0)
                break

    # The duplication tell itself: voxels observed per metre driven over the last
    # third of the matched window. Cheap ground reads high.
    lo, hi = start + (horizon - start) * 2.0 / 3.0, horizon
    yields = []
    for series in (a, b):
        p0, p1 = _at(series, lo), _at(series, hi)
        if p0 and p1:
            dm = (_num(p1[1], "distance_traveled") or 0.0) - (_num(p0[1], "distance_traveled") or 0.0)
            dv = (_num(p1[1], "total_observed_voxels") or 0.0) - (_num(p0[1], "total_observed_voxels") or 0.0)
            if dm > 5.0:
                yields.append(dv / dm)

    return {
        "peer_visible_frac": (peer_seen / tot) if tot else None,
        "divergence_med": st.median(div) if div else None,
        "unknown_lead": lead,
        "unknown_lag": max(ua, ub),
        "unknown_auc": st.mean(mean_u) if mean_u else None,
        "unknown_at_dist": u_at_dist,
        "vox_per_m_late": st.median(yields) if yields else None,
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
        "t_done_team": run["t_done_team"],
        "plan_ms_p50": st.median(plan_ms) if plan_ms else None,
    }


# The permutation test is modes_compare's, imported rather than reimplemented.
#
# This file used to carry its own copy: a bare enumeration of C(nx+ny, nx) that
# was fine on the 4-cell pilots it was written for and does not return at all on
# a 30-per-arm campaign, where C(60,30) is 1.2e17. modes_compare had the same
# defect and now samples above a threshold, with a calibration
# (modes_compare_calib.py) pinning the sampler against exactly-known answers and
# requiring two planted biases to be caught.
#
# Sharing that implementation rather than porting it is the point. Two copies of
# a statistical procedure in one repository is two procedures: they drift, only
# one of them is under calibration, and the campaign is scored by whichever
# script the operator happened to run. The calibration file names modes_compare
# in its docstring, so a reader who finds this import knows where the tests are.
def perm_p(xs, ys):
    """Two-sided permutation p on |median difference|.

    Exact by enumeration for small groups, a seeded 100000-draw estimate with
    the +1 correction above modes_compare.PERM_MAX_EXACT. See modes_compare for
    both, and modes_compare_calib.py for the known-answer cases.
    """
    return mc.perm_p(xs, ys)


def perm_sampled(xs, ys):
    """True when that p was estimated rather than enumerated.

    Printed beside the number. Which branch ran depends on the group sizes, so
    within one table some rows are exact and some are not, and a reader cannot
    tell from the value.
    """
    return not mc.perm_all(xs, ys)[2]


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

    # Matched team distance for the effort-matched endpoint: the largest budget
    # every run actually spent, so no run is scored past the end of its own data.
    budgets = []
    for r in all_runs:
        ra2, rb2 = _at(r["a"], horizon), _at(r["b"], horizon)
        if ra2 and rb2:
            budgets.append((_num(ra2[1], "distance_traveled") or 0.0) +
                           (_num(rb2[1], "distance_traveled") or 0.0))
    dist_match = min(budgets) if budgets else None
    if dist_match:
        print(f"matched team distance  = {dist_match:.0f} m "
              f"(smallest budget any run spent by T)")
    print()

    vals = {}
    for label, runs in groups.items():
        for r in runs:
            m = measure(r, horizon, args.threshold, level, dist_match, args.step)
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
        # mc.fmt_p, not %.3f: above the enumeration threshold the smallest p
        # this can return is 1e-5, and %.3f prints that as 0.000 -- a claim of
        # certainty in a column the docstring above spends a paragraph warning
        # people not to over-read. A trailing '~' marks the sampled rows.
        pstr = mc.fmt_p(p) + ("~" if perm_sampled(xs, ys) else "")
        print(f"{label:<22}{direction:<16}{ma:>11.4f}{mb:>11.4f}{mb - ma:>11.4f}"
              f"{sep:>9}{pstr:>9}  {note}{'' if not note else ' '}"
              f"{'' if abs(rel) != abs(rel) else f'({rel:+.0f}%)'}  {rng}")
        if sep == "CLEAN":
            verdict.append((label, ma, mb, note))

    print()
    # Printed BEFORE the censoring block on purpose. The t_done_team column is
    # documented as "blank = censored", and that documentation is only true for
    # runs whose event log actually loaded. Any run listed here has a blank for
    # a different reason, and reading it as censoring would overstate exactly
    # the quantity this file exists to compare.
    for label in (args.label_a, args.label_b):
        notes = [(r["name"], r["event_log_note"]) for r in groups[label]
                 if r.get("event_log_note")]
        if notes:
            print(f"EVENT LOG UNREAD in {label} ({len(notes)}/"
                  f"{len(groups[label])}): t_done_team / t_explore / t_mission "
                  f"are blank for these runs because the log could not be "
                  f"read. That is a different fact from censoring, and the "
                  f"column header's 'blank = censored' does not apply to them. "
                  f"A run can be both — the CENSORED block below is computed "
                  f"from the CSV crossing and is unaffected by this.")
            for name, note in notes:
                print(f"  {name}: {note}")
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
