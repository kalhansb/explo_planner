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
one. Dropping it biases toward "no effect" exactly in the cells where the effect
is largest. Censored runs are counted, listed, and excluded from the median with
that exclusion stated -- never silently dropped. An arm with censored runs cannot
be ranked above one without them on median alone; the censored count column is
part of the result.

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
median. The floor is 2/C(2n,n): at n=3 that is p>=0.10 and NOTHING can reach
0.05, at n=4 it is 0.029, at n=5 it is 0.008. The floor is printed so a large
p is never mistaken for evidence of no effect when it is arithmetic. With four
arms there are three comparisons against control, so read 0.05 as ~0.017 if you
want a family-wise reading; separation (do the ranges overlap at all?) is the
more honest small-n summary and is printed alongside.
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
        if not xa or not xb:
            return None
        hi = max(xa, xb)
        return abs(xa - xb) / hi * 100.0 if hi > 0 else 0.0

    end = g(t_end) or 0.0
    peak = end
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


def measure(run_dir, thresh):
    import glob
    paths = sorted(glob.glob(os.path.join(run_dir, "planner_*.csv")))
    if len(paths) != 2:
        return None
    a, b = (load(p) for p in paths)
    if not a or not b:
        return None

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
        t_team=t_team, t_lead=t_lead, censored=censored, lag=lag,
        lag_dist=lag_dist, map_end=end, map_peak=peak,
        dist_team=(a[-1][2] or 0.0) + (b[-1][2] or 0.0),
        unk_floor=unk, fire=fa + fb, fire_s=sa + sb,
        makespan=max(a[-1][0], b[-1][0]),
    )


def perm_p(xs, ys):
    """Exact two-sided permutation on the difference of medians."""
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
        if r:
            r["seed"] = int(m.group(2))
            arms.setdefault(m.group(1), []).append(r)
    if not arms:
        print("no runs matched <tag>_<arm>_seed<n>")
        return 1

    print(f"threshold: unknown_fraction <= {args.threshold}   "
          f"PRIMARY = t_team (BOTH robots across)\n")
    hdr = (f"{'run':<26}{'t_lead':>9}{'t_team':>9}{'lag':>9}{'lag_m':>8}"
           f"{'map_end':>9}{'map_pk':>8}{'dist_m':>9}{'unk':>7}{'fire':>6}{'fire_s':>8}")
    print(hdr); print("-" * len(hdr))
    for arm in sorted(arms):
        for r in sorted(arms[arm], key=lambda x: x["seed"]):
            tt = "CENSORED" if r["censored"] else f"{r['t_team']:.0f}"
            print(f"{arm+'/seed'+str(r['seed']):<26}{r['t_lead'] or 0:>9.0f}{tt:>9}"
                  f"{r['lag'] or 0:>9.0f}{r['lag_dist'] or 0:>8.0f}"
                  f"{r['map_end']:>9.2f}{r['map_peak']:>8.2f}{r['dist_team']:>9.0f}"
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
    print(f"{'arm':<12}{'metric':<12}{'delta':>11}{'sep':>9}{'perm p':>9}  note")
    print("-" * 78)
    for arm in sorted(a for a in summary if a != ctl):
        for key, label in (("tt", "t_team"), ("lag", "lag"), ("dist", "dist_team")):
            xs, ys = summary[ctl][key], summary[arm][key]
            if len(xs) < 2 or len(ys) < 2:
                print(f"{arm:<12}{label:<12}{'--':>11}{'--':>9}{'--':>9}  too few complete runs")
                continue
            d = st.median(ys) - st.median(xs)
            note = ""
            ncens = summary[arm]["cens"] + summary[ctl]["cens"]
            if key == "tt" and ncens:
                note = (f"EXCLUDES {summary[arm]['cens']} censored ({arm}), "
                        f"{summary[ctl]['cens']} ({ctl}) — worst cases dropped")
            elif key == "lag" and ncens:
                # Censored lags run to the end of the run, so they UNDERSTATE the
                # true lag. Keeping them in is the conservative choice, but the
                # delta is then a lower bound and must not be read as a point
                # estimate.
                note = (f"includes {ncens} censored lag(s), each a LOWER bound — "
                        f"delta understates")
            print(f"{arm:<12}{label:<12}{d:>11.1f}{sep(xs, ys):>9}"
                  f"{perm_p(xs, ys):>9.3f}  {note}")

    ns = sorted({len(s['tt']) for s in summary.values()} | {len(s['lag']) for s in summary.values()})
    n = ns[0] if ns else 0
    if n >= 1:
        import math
        floor = 2.0 / math.comb(2 * n, n)
        print(f"\npermutation p floor at n={n} per arm is {floor:.3f} — "
              f"{'nothing here can reach 0.05' if floor > 0.05 else 'p<0.05 is attainable'}. "
              f"Three arms vs control: three comparisons, so read 0.05 as ~0.017 "
              f"family-wise.")
    else:
        # Never silently omit this line: n hits 0 when an arm is entirely
        # censored, which is exactly when a reader most needs telling that no
        # p-value in the table above means anything.
        print("\nat least one arm has NO uncensored run, so its median rests on "
              "nothing — the deltas above are not interpretable for that arm. "
              "The censored count IS the result there: a manoeuvre whose laggard "
              "never finished is the worst outcome, not a missing one.")
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
