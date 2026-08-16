#!/usr/bin/env python3
"""Rank the reconnection modes on TEAM EXPLORATION COMPLETION TIME.

    modes_compare.py /tmp/hmr_campaign/p7modes_* [--threshold 0.55]

PRIMARY ENDPOINT: t_team, the sim time at which BOTH robots' copies of the merged
map have fallen to `--threshold` unknown. Not the first robot -- the team is not
finished until the laggard is, and every earlier phase of this campaign found the
LEADER's crossing time near-invariant to comms while the laggard's moved by
minutes. t_team is also what the run's own stop rule fires on, so it is the
quantity the experiment actually pays for in wall clock.

CENSORING IS DATA, NOT MISSINGNESS. A run whose laggard never reaches the
threshold inside the duration is the WORST outcome for its arm, not an absent
one. So when ANY run in either arm is censored, the t_team delta is WITHHELD
rather than computed over the survivors. Reporting it was not merely optimistic,
it could reverse the ranking: with off = {1500,1600,1700,1800} (median 1650) and
an arm scoring {1400,1450,>5800,>5800} (true median >=3625, about 2000 s WORSE),
dropping the two censored runs leaves {1400,1450} and the table called that arm
225 s FASTER and stamped it CLEAN -- because the deleted runs are exactly the
ones that would have overlapped. A footnote cannot repair a wrong-signed headline
number, so the number is not printed. Read the `cens` count and `lag` instead.

THREE OUTCOMES, NOT TWO. `all_done` and `censored_at_T` are both data. A run
whose sim process DIED is not: it says nothing about the policy, and scoring it
as censored charges an infrastructure failure to whichever arm was running.
Aborted runs, in-flight runs, and runs missing a planner CSV are excluded and
listed by name under EXCLUDED. Read the run's own manifest, never infer the
outcome from how the CSV happens to end.

SECONDARY, in the order they are worth reading:
  lag        t_team - t_lead. The completion delay itself, isolated from how fast
             the world happened to be explored. This is what the manoeuvres are
             supposed to shrink.
  lag_dist   metres the laggard drove during that window -- the delay priced in
             robot-metres rather than seconds.
  map_end    the two merged-map copies' disagreement when the run stopped, and
  map_peak   the largest it got mid-run. Both are treatment-intensity readings,
             NOT integrity: the gap opens while the radio is down (each robot
             keeps mapping, the peer's deltas sit queued) and collapses when the
             backlog drains. See map_agreement.py. A mode that keeps the pair in
             contact should show a lower peak.
  dist_team  total metres driven by both robots -- the cost side. A mode that
             buys completion time by driving a lot further is a real trade, and
             this column is the only place it shows up.
  unk_floor  final unknown fraction of the laggard. Guards the comparison: an arm
             is not faster if it stopped at less coverage.
  fire       reconnect-manoeuvre episodes, and seconds spent in them, as far as
             the CSV can see them. The mechanism check, and a LOWER BOUND on
             both counts -- see firings(). An arm whose manoeuvre never fired is
             not evidence about that manoeuvre; it is another copy of `off`, and
             the whole reason this campaign moved to the dense forest was that at
             81 stems/ha every arm collapsed into exactly that. But fire == 0
             here does NOT establish that: it is equally consistent with every
             episode being shorter than the sample period. Confirm against
             manoeuvre_events.py before calling an arm inert.

STATISTICS. Exact permutation over the arm-vs-`off` split, two-sided on the
median. Each row also prints its own FLOOR: the smallest p that row could ever
return at its group sizes, obtained from the same enumeration as the p-value.
It is not the closed form 2/C(2n,n), which is only valid for EQUAL groups -- and
groups become unequal precisely when censoring bites, so the formula misfired
exactly when it mattered, once printing `perm p = 0.200` directly beneath
`floor is 0.333`. Where floor >= 0.05 the p-value is arithmetic, not evidence.
With four arms there are three comparisons against control, so read 0.05 as
~0.017 family-wise; separation (do the ranges overlap at all?) is the more honest
small-n summary and is printed alongside.
"""
import argparse
import bisect
import csv
import itertools
import os
import re
import statistics as st
import sys

# The planner's actual manoeuvre states. Verified against the CSVs, not guessed:
# the full state vocabulary is {WAIT_FOR_MAP, NAVIGATE, PLAN, INTEGRATE,
# LOG_STEP, DONE, PURSUE, RETURN_NAV, RETURN_SYNC}.
#
# This replaces a regex `recon|pursu|rendez`, which was wrong in the worst
# possible direction. It matched PURSUE but NOT RETURN_NAV or RETURN_SYNC -- the
# states rendezvous and hybrid spend their manoeuvre in. p3b_rendezvous_seed1's
# bestla has 4 RETURN_NAV rows and 0 PURSUE, so the regex scored the rendezvous
# arm fire=0 while its manoeuvre was demonstrably firing, and this file's own
# closing warning would then have condemned a working arm as a relabelled
# control. A mechanism check that reports "mechanism absent" when the mechanism
# ran is worse than no check.
MANOEUVRE_STATES = {"PURSUE", "RETURN_NAV", "RETURN_SYNC"}


def rows(path):
    with open(path) as fh:
        for r in csv.DictReader(fh):
            yield r


def num(r, k):
    try:
        v = float(r[k])
    except (ValueError, KeyError, TypeError):
        return None
    return v


def load(path):
    """[(t, unknown, dist, voxels, state, reconnect_elapsed)] time-sorted."""
    out = []
    for r in rows(path):
        t = num(r, "sim_time_sec")
        if t is None:
            continue
        out.append((t, num(r, "unknown_fraction"), num(r, "distance_traveled"),
                    num(r, "total_observed_voxels"), (r.get("state") or ""),
                    num(r, "reconnect_elapsed_sec")))
    out.sort(key=lambda x: x[0])
    return out


def cross(series, thresh):
    """First sim time unknown_fraction <= thresh, or None if never."""
    for row in series:
        unk = row[1]
        if unk is not None and unk <= thresh:
            return row[0]
    return None


def val_at(series, idx, t):
    ts = [x[0] for x in series]
    i = bisect.bisect_right(ts, t) - 1
    return series[i][idx] if i >= 0 else None


def gap_trace(a, b, start=200.0, step=100.0):
    """(end, peak) percent disagreement in total_observed_voxels."""
    t_end = min(a[-1][0], b[-1][0])

    def g(t):
        xa, xb = val_at(a, 3, t), val_at(b, 3, t)
        if xa is None or xb is None:
            return None
        hi = max(xa, xb)
        return abs(xa - xb) / hi * 100.0 if hi > 0 else 0.0

    # `or 0.0` here used to turn an UNMEASURABLE gap into a reported 0.00 %, i.e.
    # into perfect agreement -- the most flattering possible reading of missing
    # data. None propagates instead.
    end = g(t_end)
    peak = end if end is not None else 0.0
    t = start
    while t <= t_end:
        v = g(t)
        if v is not None and v > peak:
            peak = v
        t += step
    return end, peak


def firings(series):
    """(episodes, seconds) visible in the CSV -- a LOWER BOUND on both.

    The CSV samples on a ~5-10 s timer while manoeuvre episodes are routinely
    shorter than that (p3b_hybrid_seed1 holds chases of 7.1, 6.4 and 3.7 s, and
    p3b_hybrid_seed2's atlas firing leaves no manoeuvre row at all). So this
    undercounts, and it undercounts precisely the FAST reconnections -- the ones
    a good mode is supposed to produce. manoeuvre_events.py parses the planner
    log, which emits one line per decision, and is the authority on firings; this
    column exists only so the mechanism can be sanity-checked in the same table
    as the outcome.

    Two independent signals are OR'd because either alone can miss an episode:
    the state label, and reconnect_elapsed_sec, which the planner counts up from
    zero at the arm row.
    """
    eps = 0
    secs = 0.0
    prev_in = False
    prev_t = None
    for t, _, _, _, state, elapsed in series:
        cur = (state.strip().upper() in MANOEUVRE_STATES
               or (elapsed is not None and elapsed > 0.0))
        if cur and not prev_in:
            eps += 1
        if cur and prev_t is not None:
            secs += t - prev_t
        prev_in, prev_t = cur, t
    return eps, secs


def outcome(run_dir):
    """How the run ENDED, from its own manifest. Three outcomes, not two.

    A run that hit the horizon with work left (`censored_at_T`) is DATA: it is
    the worst result its arm can produce. A run whose sim process died is NOT --
    it says nothing about the policy, and scoring it as censored charges an
    infrastructure failure to whichever arm happened to be running.
    p4mild_rendezvous_seed2 is exactly that: its console log reads "sim died
    mid-run", it stops at t_sim=160 with unknown still 0.817, and treating it as
    a censored rendezvous run would have made rendezvous look catastrophic on the
    strength of a crashed process.

    Runs still in flight also land here (no end reason written yet), and must be
    excluded rather than counted as censored -- the campaign writes this line
    only at teardown.
    """
    mf = os.path.join(run_dir, "run_manifest.txt")
    if not os.path.exists(mf):
        return "no_manifest"
    with open(mf) as fh:
        for line in fh:
            if line.startswith("run_end_reason="):
                return line.strip().split("=", 1)[1] or "unknown"
    return "aborted_or_running"


def measure(run_dir, thresh):
    import glob
    end = outcome(run_dir)
    if end not in ("all_done", "censored_at_T"):
        return dict(excluded=end)
    paths = sorted(glob.glob(os.path.join(run_dir, "planner_*.csv")))
    if len(paths) != 2:
        # Previously a silent `return None`. A run that produced no CSV is an
        # infrastructure failure exactly like a dead sim, and dropping it without
        # a word while scoring its half-written sibling as censored gave two
        # identical failures opposite treatment.
        return dict(excluded=f"{len(paths)} planner CSV(s), expected 2")
    a, b = (load(p) for p in paths)
    if not a or not b:
        return dict(excluded="planner CSV present but empty")

    cr = [cross(a, thresh), cross(b, thresh)]
    censored = any(c is None for c in cr)
    done = [c for c in cr if c is not None]
    t_lead = min(done) if done else None
    t_team = max(cr) if not censored else None

    if t_lead is not None:
        # The laggard's window. When censored it runs to the end of the run --
        # a lower bound on the true lag, which is the conservative direction.
        t_stop = t_team if t_team is not None else max(a[-1][0], b[-1][0])
        lag = t_stop - t_lead
        lag_series = a if cr[0] != t_lead else b
        d0 = val_at(lag_series, 2, t_lead) or 0.0
        d1 = val_at(lag_series, 2, t_stop) or d0
        lag_dist = d1 - d0
    else:
        lag = lag_dist = None

    end, peak = gap_trace(a, b)
    fa, sa = firings(a)
    fb, sb = firings(b)
    unk = max((x[1] for x in (a[-1], b[-1]) if x[1] is not None), default=None)

    return dict(
        excluded=None, end=end,
        t_team=t_team, t_lead=t_lead, censored=censored, lag=lag,
        lag_dist=lag_dist, map_end=end, map_peak=peak,
        dist_team=(a[-1][2] or 0.0) + (b[-1][2] or 0.0),
        unk_floor=unk, fire=fa + fb, fire_s=sa + sb,
        makespan=max(a[-1][0], b[-1][0]),
    )


def perm_all(xs, ys):
    """Every |median difference| reachable by relabelling, and the observed one.

    Returned together so the p-value and its own attainable FLOOR come from the
    same enumeration. They used to be computed separately -- p by enumerating
    C(nx+ny, nx), the floor by a hardcoded 2/C(2n,n) that assumes EQUAL group
    sizes -- and the two disagreed the moment censoring made the groups unequal.
    The tool printed `perm p = 0.200` directly beneath `floor is 0.333`, a
    p-value below its own stated minimum.
    """
    pool = list(xs) + list(ys)
    nx = len(xs)
    obs = abs(st.median(xs) - st.median(ys))
    diffs = []
    for combo in itertools.combinations(range(len(pool)), nx):
        left = [pool[i] for i in combo]
        right = [pool[i] for i in range(len(pool)) if i not in combo]
        diffs.append(abs(st.median(left) - st.median(right)))
    return obs, diffs


def perm_p(xs, ys):
    """Exact two-sided permutation on the difference of medians."""
    obs, diffs = perm_all(xs, ys)
    if not diffs:
        return 1.0
    return sum(1 for d in diffs if d >= obs - 1e-12) / len(diffs)


def perm_floor(xs, ys):
    """Smallest p this comparison could EVER return, given these group sizes.

    Computed by enumeration rather than from a formula, because the closed form
    2/C(2n,n) is only correct for equal groups: with nx != ny the label-swapped
    arrangement is not itself a valid relabelling, so the attainable minimum is
    whatever share of arrangements ties the most extreme one. If this equals or
    exceeds 0.05, no data in that row can be significant and the p-value is
    reporting arithmetic, not evidence.
    """
    _, diffs = perm_all(xs, ys)
    if not diffs:
        return 1.0
    mx = max(diffs)
    return sum(1 for d in diffs if d >= mx - 1e-12) / len(diffs)


def sep(xs, ys):
    return "CLEAN" if max(xs) < min(ys) or max(ys) < min(xs) else "overlap"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--threshold", type=float, default=0.55,
                    help="unknown_fraction defining 'explored' (planner default 0.55)")
    ap.add_argument("--control", default="off", help="arm to test the others against")
    args = ap.parse_args()

    arms = {}
    dropped = []
    for d in sorted(args.runs):
        if not os.path.isdir(d):
            continue
        # normpath first: a shell glob of `p7modes_*/` hands us a TRAILING
        # SLASH, and basename("/a/b/") is "", so every directory silently failed
        # to match and the tool printed "no runs matched" over a complete
        # campaign. The overnight chain calls it with exactly that glob.
        m = re.search(r"_([A-Za-z]+)_seed(\d+)$", os.path.basename(os.path.normpath(d)))
        if not m:
            continue
        r = measure(d, args.threshold)
        if r.get("excluded"):
            dropped.append((os.path.basename(os.path.normpath(d)), r["excluded"]))
            continue
        r["seed"] = int(m.group(2))
        arms.setdefault(m.group(1), []).append(r)
    if not arms:
        print("no runs matched <tag>_<arm>_seed<n>")
        if dropped:
            print("(runs found but excluded: "
                  + ", ".join(f"{n} [{w}]" for n, w in dropped) + ")")
        return 1

    if dropped:
        # Named, never silent. These are infrastructure failures and runs still
        # in flight -- neither is evidence about a policy, but a reader must be
        # able to see that an arm is short a cell and why.
        print("EXCLUDED (not evidence about any arm — infrastructure or in flight):")
        for n, w in dropped:
            print(f"    {n}: {w}")
        print()

    print(f"threshold: unknown_fraction <= {args.threshold}   "
          f"PRIMARY = t_team (BOTH robots across)\n")
    hdr = (f"{'run':<26}{'t_lead':>9}{'t_team':>9}{'lag':>9}{'lag_m':>8}"
           f"{'map_end':>9}{'map_pk':>8}{'dist_m':>9}{'unk':>7}{'fire':>6}{'fire_s':>8}")
    print(hdr); print("-" * len(hdr))
    for arm in sorted(arms):
        for r in sorted(arms[arm], key=lambda x: x["seed"]):
            tt = "CENSORED" if r["censored"] else f"{r['t_team']:.0f}"
            me = "--" if r["map_end"] is None else f"{r['map_end']:.2f}"
            print(f"{arm+'/seed'+str(r['seed']):<26}{r['t_lead'] or 0:>9.0f}{tt:>9}"
                  f"{r['lag'] or 0:>9.0f}{r['lag_dist'] or 0:>8.0f}"
                  f"{me:>9}{r['map_peak']:>8.2f}{r['dist_team']:>9.0f}"
                  f"{r['unk_floor'] or 0:>7.3f}{r['fire']:>6}{r['fire_s']:>8.0f}")

    print(f"\n{'arm':<12}{'n':>3}{'cens':>6}{'t_team med':>12}{'range':>18}"
          f"{'lag med':>10}{'dist med':>10}{'unk med':>9}{'peak med':>10}{'fire':>7}")
    print("-" * 107)
    summary = {}
    for arm in sorted(arms):
        rs = arms[arm]
        ok = [r for r in rs if not r["censored"]]
        tt = [r["t_team"] for r in ok]
        summary[arm] = dict(
            tt=tt, n=len(rs), cens=len(rs) - len(ok),
            lag=[r["lag"] for r in rs if r["lag"] is not None],
            dist=[r["dist_team"] for r in rs],
            unk=[r["unk_floor"] for r in rs if r["unk_floor"] is not None],
            peak=[r["map_peak"] for r in rs],
            fire=sum(r["fire"] for r in rs),
        )
        s = summary[arm]
        med = f"{st.median(tt):.0f}" if tt else "--"
        rng = f"[{min(tt):.0f}..{max(tt):.0f}]" if tt else "--"
        print(f"{arm:<12}{s['n']:>3}{s['cens']:>6}{med:>12}{rng:>18}"
              f"{st.median(s['lag']):>10.0f}{st.median(s['dist']):>10.0f}"
              f"{st.median(s['unk']):>9.3f}{st.median(s['peak']):>10.2f}{s['fire']:>7}")

    ctl = args.control
    if ctl not in summary:
        print(f"\nno '{ctl}' arm — skipping the control comparison")
        return 0

    print(f"\nvs control '{ctl}'   (negative delta = FASTER completion = better)")
    print(f"{'arm':<12}{'metric':<12}{'delta':>11}{'sep':>9}{'perm p':>9}{'floor':>8}  note")
    print("-" * 96)
    for arm in sorted(a for a in summary if a != ctl):
        ncens = summary[arm]["cens"] + summary[ctl]["cens"]
        for key, label in (("tt", "t_team"), ("lag", "lag"), ("dist", "dist_team")):
            xs, ys = summary[ctl][key], summary[arm][key]
            # A t_team median built from the survivors of censoring is not a
            # conservative estimate, it is a WRONG-SIGNED one. Worked example:
            # off = {1500,1600,1700,1800} (median 1650) versus an arm scoring
            # {1400,1450,>5800,>5800} (true median >=3625, i.e. ~2000 s WORSE).
            # Dropping the two censored runs leaves {1400,1450}, median 1425, and
            # the table reports the arm 225 s FASTER and stamps it CLEAN --
            # because the runs deleted are exactly the ones that would have
            # overlapped. The separation is manufactured by the exclusion. A
            # footnote cannot repair a reversed headline number, so the number is
            # withheld instead.
            if key == "tt" and ncens:
                print(f"{arm:<12}{label:<12}{'WITHHELD':>11}{'--':>9}{'--':>9}{'--':>8}  "
                      f"{summary[arm]['cens']} censored ({arm}) + "
                      f"{summary[ctl]['cens']} ({ctl}); a median over the "
                      f"survivors can invert the true ranking — read 'cens' and "
                      f"'lag' instead")
                continue
            if len(xs) < 2 or len(ys) < 2:
                print(f"{arm:<12}{label:<12}{'--':>11}{'--':>9}{'--':>9}{'--':>8}  "
                      f"too few complete runs")
                continue
            d = st.median(ys) - st.median(xs)
            note = ""
            if key == "lag" and ncens:
                # Censored lags run to the end of the run, so they UNDERSTATE the
                # true lag. Keeping them in is the conservative direction here --
                # unlike t_team, nothing is deleted -- but the delta is then a
                # bound, not a point estimate.
                note = (f"includes {ncens} censored lag(s), each a LOWER bound — "
                        f"delta understates")
            fl = perm_floor(xs, ys)
            if fl > 0.05 and not note:
                note = f"floor {fl:.3f} > 0.05: p cannot be significant at this n"
            print(f"{arm:<12}{label:<12}{d:>11.1f}{sep(xs, ys):>9}"
                  f"{perm_p(xs, ys):>9.3f}{fl:>8.3f}  {note}")

    print(f"\n'floor' is the smallest p THAT row could ever return at its group "
          f"sizes, enumerated not assumed. Where floor >= 0.05 the p-value is "
          f"arithmetic, not evidence — read 'sep'. Three arms vs one control is "
          f"three comparisons, so read 0.05 as ~0.017 family-wise.")
    if any(s["cens"] for s in summary.values()):
        print("Censoring is present. A censored run is the WORST outcome for its "
              "arm, not a missing one: an arm with censored runs cannot be ranked "
              "above an arm without them, whatever the surviving medians say.")
    inert = [a for a in summary if a != ctl and summary[a]["fire"] == 0]
    if inert:
        print(f"\n!! CSV-visible firings are 0 for: {', '.join(sorted(inert))}. "
              f"That is a LOWER BOUND (the CSV samples every ~5-10 s and short "
              f"episodes leave no row), so it does not by itself prove the "
              f"manoeuvre never ran. Settle it with manoeuvre_events.py, which "
              f"parses the planner log. If the log agrees the arm never fired, "
              f"it is a relabelled '{ctl}' and its rows above are not evidence "
              f"about that policy.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
