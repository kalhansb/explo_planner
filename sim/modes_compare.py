#!/usr/bin/env python3
"""Rank the reconnection modes on TEAM EXPLORATION COMPLETION TIME.

    modes_compare.py /tmp/hmr_campaign/p7modes_* [--threshold 0.64]

PRIMARY ENDPOINT: t_team, the sim time at which BOTH robots' copies of the merged
map have fallen to `--threshold` unknown. Not the first robot -- the team is not
finished until the laggard is, and every earlier phase of this campaign found the
LEADER's crossing time near-invariant to comms while the laggard's moved by
minutes. t_team is also what the run's own stop rule fires on, so it is the
quantity the experiment actually pays for in wall clock.

BUT READ `lag` FIRST -- IT IS THE ONLY THING THE TREATMENT CAN TOUCH.

    t_team = t_lead + lag

THE MANOEUVRE WAS TERMINAL THROUGH GENERATION 7, AND IS NOT ANY MORE. Read the
next two paragraphs as a statement about the campaigns this tool was written
for, not about generation 8.

Through generation 7: finishOrRendezvous() was the sole entry point to all
three modes -- startPursuit is called from inside it -- and every one of its
call sites meant THIS ROBOT HAS FINISHED EXPLORING. There was no
mid-exploration trigger, so nothing any arm did could alter the leader's own
exploration, and t_lead was identical in expectation across arms by
construction.

Generation 8 adds a MID-RUN trigger (explo_planner_node.cpp, the link-gated
dispatch) that fires DURING exploration. The bank already records 302 mid-run
dispatches across the hybrid cells -- not the 422 an earlier version of this
line claimed, which summed three unrelated line counts. Do not read the "302 +
113 vetoes + 7 give-ups = 422" decomposition that briefly replaced it either:
the link-gate veto line is throttled at 30 s, so 113 counts printed lines and
is a lower bound, and 422 totals nothing. Only the 302 is a count of events,
and only manoeuvres the robot actually performed can move a completion time.
That breaks the argument above in both directions: the treatment
can now move t_lead, and the "it can act inside `lag` and nowhere else" claim
this module prints at runtime is simply false for such a campaign. This tool is
not the pre-registered reader for generation 8 -- event_log.py is -- and its
`lag` framing should not be quoted for one.

The manoeuvre can therefore only act inside the window between the leader
finishing and the laggard finishing, by carrying a map backlog to the laggard.
That window has been 10-62 s against a t_team of ~1100 s -- about 5 %. Measuring
t_team adds t_lead as PURE UNTREATABLE NOISE over the 95 % of the run the
treatment cannot influence, and t_lead is the noisiest part (see the noise-floor
banner). This is the mechanical reason every phase of this campaign has found
nothing: not that the modes are equivalent, but that the endpoint is ~95 %
composed of a quantity they cannot move.

So t_team stays the headline because it is the completion time the team actually
pays, but `lag` is the mechanism-aligned endpoint and any claim about a mode
belongs there.

AND CHECK THE ARM FIRED AT ALL. Because the trigger is terminal, an arm declines
whenever the team happens to be together at the finish -- p7modes_rendezvous_seed1
logged "full team present -> DONE" twice and executed ZERO manoeuvres, making it
a relabelled control despite the peer being invisible 56 % of the run. Its
completion time is not evidence about rendezvous. manoeuvre_events.py is the
authority; see `fire` below.

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
import math
import os
import re
import statistics as st
import sys

# Cell-directory name -> (arm, seed). The arm token is `.+`, not `[A-Za-z]+`,
# because the harness itself mandates arm names the letters-only pattern cannot
# read: run_campaign.sh requires the post-latch coast to be carried in the arm
# suffix (e.g. `hybrid_seek`), and those cells exist on disk (ds1_hybrid_seek_
# seed1..4). Under the old pattern they parsed as arm="seek" — an arm nobody
# ran — while their manifests said reconnect_mode_requested=hybrid, so they were
# split out of "hybrid" under a fabricated label. Two arms sharing a final token
# would have pooled silently, which is the same failure with no visible tell.
#
# `.+` is greedy but anchored by `_seed<digits>$` and starts from the leftmost
# `_`, so `g8r1_hybrid_seed101` still yields ("hybrid", 101) and
# `ds1_hybrid_seek_seed1` yields ("hybrid_seek", 1).
RE_CELL = re.compile(r"_(.+)_seed(\d+)$")

# The planner's actual manoeuvre states. Verified against the CSVs, not guessed:
# the full state vocabulary is {WAIT_FOR_MAP, NAVIGATE, PLAN, INTEGRATE,
# LOG_STEP, DONE, PURSUE, RETURN_NAV, RETURN_SYNC, PROXIMITY_HOLD, and — since
# mission return, 2026-08-27 — RETURN_HOME}. RETURN_HOME is deliberately NOT in
# the set below: it is the arm-invariant drive home, present in the control arm
# too, and its rows carry reconnect_elapsed_sec = -1 (transitionTo closes any
# live manoeuvre on entering it), so neither clause of `cur` may claim it.
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


_LOAD_CACHE = {}


def load(path):
    """[(t, unknown, dist, voxels, state, reconnect_elapsed)] time-sorted.

    Memoised so the threshold ladder can re-measure every run at several
    thresholds without re-parsing the CSVs each time.
    """
    if path in _LOAD_CACHE:
        return _LOAD_CACHE[path]
    out = []
    for r in rows(path):
        t = num(r, "sim_time_sec")
        if t is None:
            continue
        out.append((t, num(r, "unknown_fraction"), num(r, "distance_traveled"),
                    num(r, "total_observed_voxels"), (r.get("state") or ""),
                    num(r, "reconnect_elapsed_sec")))
    out.sort(key=lambda x: x[0])
    _LOAD_CACHE[path] = out
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


def gap_trace(a, b, start=200.0):
    """(end, peak) percent disagreement in total_observed_voxels.

    Sampled at the UNION of both series' own timestamps, not on a fixed lattice.
    A 100 s lattice missed outage spikes shorter than its own step, which is the
    wrong direction: the gap opens on an outage and drains on reconnect, so short
    outages are exactly the events a lattice steps over. Same cost, no misses.
    """
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
    for t in sorted({x[0] for x in a} | {x[0] for x in b}):
        if t < start or t > t_end:
            continue
        v = g(t)
        if v is not None and v > peak:
            peak = v
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


def _manifest_field(run_dir, key, default=""):
    mf = os.path.join(run_dir, "run_manifest.txt")
    if not os.path.exists(mf):
        return default
    with open(mf) as fh:
        for line in fh:
            if line.startswith(key + "="):
                return line.strip().split("=", 1)[1]
    return default


def build_of(run_dir):
    """Which planner commit this cell actually ran.

    Arms compared across different builds are confounded with the build, and it
    has happened: p4mild ran six cells from five different planner commits, and
    p3b's control was built from a different commit than all three of its
    treatments. Nothing in the campaign driver pins or checks this, so the check
    lives here, where the comparison is made.
    """
    return _manifest_field(run_dir, "git_explo_planner", "?")


def verdict_of(run_dir):
    return _manifest_field(run_dir, "run_gates_verdict", "")


def declared_of(run_dir):
    """The planner's OWN completion, from its event log, or None.

    This file reconstructs completion by scanning the CSV for the first crossing
    of unknown_fraction <= thresh. The planner separately STATES when it stopped
    trying, in explore_done_sim_sec. The two are different constructs and only
    coincide when the stop rule is that same threshold:

      the crossing     is sampled at plan cadence, so it lands on the first plan
                       tick at or after the true crossing, and it exists even in
                       a run the planner never called finished.
      the declaration  is the decision the robot ACTED on. It is also the LAST
                       one, not the first: a robot pulled into a reconnect
                       manoeuvre and resuming afterwards declares twice, and the
                       first declaration is not when it finished.

    So the reconnecting arms are exactly where these can separate, which is
    exactly where the result lives. Reported, not reconciled: silently swapping
    one for the other would change every number this file has ever printed
    without saying so, and a disagreement is a finding about the run rather than
    a defect in either measure.

    Returns the summary, or a dict carrying only `excluded` (a reason string)
    when there is no summary to return. It never returns None, and that is the
    point: the previous version collapsed "import failed", "the event log will
    not parse", "there is no event log" and "the planner declared nothing" all
    into None, and the caller could not tell a cross-check that AGREED from one
    that never ran. A silent None here makes the COMPLETION CROSS-CHECK block
    print nothing, which reads as "checked, no disagreement" -- the exact
    failure mode where a guard keeps printing PASS after it stopped checking.
    """
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import event_log
    except ImportError as e:
        return dict(excluded=f"event_log unimportable: {e}")
    try:
        # summarise_run already reports its own exclusions this way (missing
        # logs, truncation, schema refusal), so pass that through unchanged
        # rather than flattening it.
        return event_log.summarise_run(run_dir)
    except Exception as e:
        return dict(excluded=f"{type(e).__name__}: {e}")


def midrun_count(run_dir):
    """Mid-run reconnect dispatches in this cell, both robots.

    Exists so the MECHANISM WINDOW block can tell a terminal-only campaign from
    a generation-8 one instead of asserting the former. The pattern is kept
    identical to manoeuvre_events.RE_MIDRUN_DISPATCH -- if that one drifts, this
    silently reads 0 and the block reverts to printing the false claim, so the
    two are meant to be changed together.
    """
    import glob
    import re as _re
    pat = _re.compile(r"Reconnect \(mid-run\): (?:team incomplete|peer silent) "
                      r"(\d+)s >= (?:gate )?\d+s")
    n = 0
    for p in glob.glob(os.path.join(run_dir, "planner_*.log")):
        try:
            with open(p, errors="replace") as f:
                n += sum(1 for ln in f if pat.search(ln))
        except OSError:
            pass
    return n


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

    # NOT `end` -- that name already holds the run outcome from outcome() above,
    # and rebinding it here silently made the returned dict's `end` key the map
    # gap percentage instead of "all_done"/"censored_at_T".
    gap_end, peak = gap_trace(a, b)
    fa, sa = firings(a)
    fb, sb = firings(b)
    unk = max((x[1] for x in (a[-1], b[-1]) if x[1] is not None), default=None)

    # The planner's own account of the same run, for the cross-check. `arm` is
    # taken from here too: it is the only place the control arm is distinguished
    # from hybrid, since a control run carries reconnect_mode "hybrid".
    dec = declared_of(run_dir)
    # Separate "the cross-check ran and found nothing" from "the cross-check
    # could not run". Both used to look like None downstream.
    decl_excluded = dec.get("excluded")
    if decl_excluded:
        dec = None
    t_team_decl = dec["t_team"] if dec else None
    decl_delta = (t_team_decl - t_team
                  if (t_team_decl is not None and t_team is not None) else None)
    # Censoring can disagree in EITHER direction and each direction means
    # something different: crossing-only means the map hit the threshold but the
    # planner kept going; declaration-only means it stopped without the CSV ever
    # showing the crossing (frontier exhaustion, or a cadence miss).
    decl_censor_split = (None if dec is None
                         else ("crossing-only" if (t_team is not None and
                                                   t_team_decl is None)
                               else "declaration-only" if (t_team is None and
                                                           t_team_decl is not None)
                               else None))

    return dict(
        excluded=None, end=end,
        t_team=t_team, t_lead=t_lead, censored=censored, lag=lag,
        t_team_decl=t_team_decl, decl_delta=decl_delta,
        decl_censor_split=decl_censor_split,
        decl_excluded=decl_excluded,
        arm_stamped=(dec.get("arm") if dec else None),
        lag_dist=lag_dist, map_end=gap_end, map_peak=peak,
        # Read at t_team, not at end-of-run. End-of-run includes the DONE grace
        # drain and the teardown tail, and those windows differ by arm, so the
        # cost column was partly measuring how long each arm idled after
        # finishing. Censored runs have no t_team, so they fall back to
        # end-of-run -- correct there, since the run never completed.
        dist_team=((val_at(a, 2, t_team) or a[-1][2] or 0.0)
                   + (val_at(b, 2, t_team) or b[-1][2] or 0.0)
                   if t_team is not None
                   else (a[-1][2] or 0.0) + (b[-1][2] or 0.0)),
        build=build_of(run_dir), verdict=verdict_of(run_dir),
        unk_floor=unk, fire=fa + fb, fire_s=sa + sb,
        makespan=max(a[-1][0], b[-1][0]),
        midrun=midrun_count(run_dir),
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


def noise_floor(null_dirs, thresh, summary, ctl):
    """The spread of REPLICATES of one identical condition — the yardstick.

    Every delta in the table above is meaningless until compared against how much
    this pipeline moves when NOTHING is changed. That number is measurable here
    because `seed` reaches only the comms emulator (run_explo_sim_rviz.sh:651, in
    the `if COMMS = 1` block), so an ideal-comms campaign run at three different
    seeds is three runs of ONE configuration. Its t_team came out 890 / 1429 /
    2700 -- a 3.03x spread, CV 45 %, from runs that differ in nothing at all.

    That band is wider than any arm difference this campaign has produced. It is
    not physics: the same three runs agree to within 20 % at unknown<=0.65 and
    within 11 % at 0.75. It is the endpoint. By 0.55 the coverage curve has gone
    nearly flat, so time-to-threshold inverts a flat function and turns ROS/Gazebo
    scheduling jitter into minutes of apparent difference. See ladder().

    Consequence for reading the table: an arm delta smaller than this band is not
    a small effect, it is no effect. Report it as "inside the noise floor", never
    as a ranking.
    """
    if not null_dirs:
        print(f"\nNO NOISE FLOOR MEASURED. Pass --null-runs with replicates of a "
              f"single condition. Without it, no delta above can be told apart "
              f"from run-to-run jitter, which on this pipeline has been measured "
              f"at 3.03x for t_team at unknown<=0.55.")
        return
    vals = []
    for d in sorted(null_dirs):
        if not os.path.isdir(d):
            continue
        r = measure(d, thresh)
        if r.get("excluded") or r.get("censored"):
            continue
        vals.append(r["t_team"])
    if len(vals) < 2:
        print(f"\nnoise floor: only {len(vals)} usable replicate(s) — not enough "
              f"to bound run-to-run jitter.")
        return
    vals.sort()
    lo, hi = vals[0], vals[-1]
    band = hi - lo
    cv = st.pstdev(vals) / st.mean(vals) * 100.0 if st.mean(vals) else 0.0
    print(f"\nNOISE FLOOR from {len(vals)} replicate(s) of ONE condition at "
          f"unknown<={thresh}: t_team = {[round(v) for v in vals]}")
    print(f"    band {band:.0f} s  ({hi/lo:.2f}x, CV {cv:.1f} %) — this is how "
          f"much the pipeline moves when NOTHING is changed.")
    if ctl in summary and summary[ctl]["tt"]:
        cm = st.median(summary[ctl]["tt"])
        for arm in sorted(a for a in summary if a != ctl):
            if not summary[arm]["tt"] or summary[arm]["cens"] or summary[ctl]["cens"]:
                continue
            d = st.median(summary[arm]["tt"]) - cm
            verdict = ("INSIDE the noise floor — not an effect"
                       if abs(d) <= band else
                       f"clears the floor by {abs(d) / band:.1f}x")
            print(f"    {arm:<12} delta {d:>8.0f} s   {verdict}")


def ladder(run_dirs, primary, spec):
    """Is the ranking a property of the arms, or of where the threshold landed?

    WHY THIS EXISTS. t_team inverts the coverage curve, and by 0.55 that curve is
    almost flat: measured over the 300 s before each crossing, the sim-seconds
    bought per 0.01 of unknown_fraction run from 15 to 13636, a 906x spread, and
    one run spent 77 minutes of sim time to gain three percentage points. So a
    0.001 difference in merged-map content -- one lucky corridor -- can move
    t_team by minutes, and the amplification varies wildly BETWEEN SEEDS OF THE
    SAME ARM. A median of such a quantity at n<=5 is not automatically a ranking.

    The endpoint stays at 0.55 because that is the planner's own DONE rule and
    therefore the completion time the team actually pays in wall clock. But a
    ranking that only exists at 0.55 is a ranking of where the threshold happened
    to fall on each seed's curve. Re-measuring at 0.70/0.65/0.60 costs nothing --
    the same series, a different crossing -- and the higher thresholds sit where
    the curve is still steep, so they carry far less amplified noise.

    Read it as: agreement across the ladder means the ordering is a property of
    the arms. Disagreement means t_team at this n cannot rank them, and the
    honest output is the effect size with its spread, not a winner.
    """
    if not spec:
        return
    try:
        ths = [float(x) for x in spec.split(",") if x.strip()]
    except ValueError:
        return
    for t in (primary,):
        if t not in ths:
            ths.append(t)
    ths.sort(reverse=True)

    rows_out = []
    unmatched = set()
    for th in ths:
        per = {}
        for d in sorted(run_dirs):
            if not os.path.isdir(d):
                continue
            m = RE_CELL.search(os.path.basename(os.path.normpath(d)))
            if not m:
                # Named, not swallowed — the same fix main() already carries.
                # A bare `continue` here dropped unreadable cells with no
                # message, so the sensitivity ladder could disagree with the
                # main table purely because it silently saw fewer runs, and
                # the rank order it prints is exactly what that would corrupt.
                unmatched.add(os.path.basename(os.path.normpath(d)))
                continue
            r = measure(d, th)
            if r.get("excluded"):
                continue
            per.setdefault(m.group(1), []).append(r)
        stats = {}
        for arm, rs in per.items():
            ok = [x["t_team"] for x in rs if not x["censored"]]
            stats[arm] = (st.median(ok) if ok else None,
                          sum(1 for x in rs if x["censored"]), len(rs))
        rows_out.append((th, stats))

    arms_all = sorted({a for _, s in rows_out for a in s})
    if unmatched:
        print(f"!! ladder: {len(unmatched)} directory(ies) did not parse as "
              f"<tag>_<arm>_seed<n> and are ABSENT from the ladder below: "
              + ", ".join(sorted(unmatched)))
    if not arms_all:
        return
    print(f"\nTHRESHOLD SENSITIVITY — median t_team, and the rank order it implies")
    print(f"{'unknown<=':<11}" + "".join(f"{a:>13}" for a in arms_all) + "   order (fastest first)")
    print("-" * (11 + 13 * len(arms_all) + 28))
    orders = []
    for th, stats in rows_out:
        cells = []
        rankable = []
        for a in arms_all:
            med, cens, n = stats.get(a, (None, 0, 0))
            if med is None:
                cells.append(f"{'all cens':>13}")
            else:
                cells.append(f"{med:>10.0f}{('*' * min(cens, 2)):<3}")
                # An arm with censored runs is NOT rankable on the surviving
                # median -- same reason the delta is withheld above.
                if cens == 0:
                    rankable.append((med, a))
        rankable.sort()
        order = " < ".join(a for _, a in rankable) if rankable else "(none rankable)"
        orders.append(tuple(a for _, a in rankable))
        mark = " <== PRIMARY" if abs(th - primary) < 1e-9 else ""
        print(f"{th:<11.2f}" + "".join(cells) + f"   {order}{mark}")

    print("* = arm has censored run(s) at this threshold; its median is over "
          "survivors only and is NOT rankable.")

    # Two very different things can move the ordering down the ladder, and
    # collapsing them into one "UNSTABLE" verdict throws away the finding.
    #
    #   NEAR the completion criterion, a flip means the endpoint is noise: those
    #   thresholds are separated by a few percent of coverage and should not
    #   reorder the arms.
    #
    #   BETWEEN early and late thresholds, a flip is a RESULT. A reconnect
    #   manoeuvre spends time it does not spend exploring, so an arm can be
    #   behind at 0.70 and ahead at 0.55: the manoeuvre costs time early and
    #   repays it near completion. That is a claim about when the policy earns
    #   its keep, not a defect in the ranking at completion.
    rows_ranked = [(th, o) for (th, _), o in zip(rows_out, orders)
                   if len(o) == len(arms_all)]
    if len(rows_ranked) < 2:
        print("Too few thresholds have all arms rankable to judge stability.")
        return
    near = [o for th, o in rows_ranked if th <= primary + 0.051]
    if len(near) >= 2 and len(set(near)) == 1:
        print(f"STABLE NEAR COMPLETION: the ordering {' < '.join(near[0])} holds "
              f"at every threshold within 0.05 of the primary. The ranking at "
              f"completion is not an artifact of where the threshold fell.")
    elif len(near) >= 2:
        print(f"UNSTABLE NEAR COMPLETION: the ordering changes between thresholds "
              f"only a few percent of coverage apart "
              f"({' | '.join(' < '.join(o) for o in dict.fromkeys(near))}). "
              f"t_team at this n cannot rank these arms — report the effect size "
              f"and its spread, not a winner.")
    else:
        print("Only one threshold near the primary is fully rankable; stability "
              "near completion is untested.")

    early = [o for th, o in rows_ranked if th > primary + 0.051]
    if early and near and early[0] != near[-1]:
        print(f"EARLY-vs-LATE REVERSAL (a result, not a defect): at "
              f"unknown<={rows_ranked[0][0]:.2f} the order is "
              f"{' < '.join(early[0])}, at completion it is "
              f"{' < '.join(near[-1])}. A manoeuvre spends time not exploring, so "
              f"it can trail early and lead at the end — this says WHEN each "
              f"policy earns its keep. It does not weaken the completion ranking, "
              f"which is judged by the line above.")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("runs", nargs="+")
    # 0.64, not the 0.55 this defaulted to through generation 7.
    #
    # The threshold has to match the planner's OWN done rule, because t_team is
    # the CSV crossing of it: generation 8 latches and stops at unknown <= 0.64,
    # so a run measured at 0.55 is asked when it crossed a level it was never
    # driven to. It usually never crosses, the cell scores `censored`, and
    # t_team is withheld -- a silent, near-total loss of the endpoint rather
    # than a wrong number. The ladder brackets 0.64 on both sides for the same
    # reason it always did: an ordering that exists only at the endpoint value
    # is a property of the threshold, not of the arms.
    ap.add_argument("--threshold", type=float, default=0.64,
                    help="unknown_fraction defining 'explored' (gen-8 planner "
                         "done rule is 0.64; pre-gen-8 campaigns used 0.55)")
    ap.add_argument("--control", default="off", help="arm to test the others against")
    ap.add_argument("--ladder", default="0.70,0.67,0.64,0.61,0.58",
                    help="thresholds for the sensitivity ladder; '' disables")
    ap.add_argument("--null-runs", default="", nargs="*",
                    help="run dirs that are REPLICATES of one identical condition; "
                         "their spread is the noise floor every arm delta must clear")
    args = ap.parse_args()

    arms = {}
    dropped = []
    unmatched = []
    for d in sorted(args.runs):
        if not os.path.isdir(d):
            continue
        # normpath first: a shell glob of `p7modes_*/` hands us a TRAILING
        # SLASH, and basename("/a/b/") is "", so every directory silently failed
        # to match and the tool printed "no runs matched" over a complete
        # campaign. The overnight chain calls it with exactly that glob.
        m = RE_CELL.search(os.path.basename(os.path.normpath(d)))
        if not m:
            # Named, not swallowed. A bare `continue` here dropped cells BEFORE
            # the `dropped` bookkeeping below, so a directory the regex could
            # not read vanished from the comparison with no message at all —
            # indistinguishable from a campaign that never ran it.
            unmatched.append(os.path.basename(os.path.normpath(d)))
            continue
        r = measure(d, args.threshold)
        if r.get("excluded"):
            dropped.append((os.path.basename(os.path.normpath(d)), r["excluded"]))
            continue
        r["seed"] = int(m.group(2))
        arms.setdefault(m.group(1), []).append(r)
    if unmatched:
        # Printed whether or not anything else matched: a cell whose name the
        # parser cannot read is not an exclusion, it is a cell the comparison
        # never saw, and that must never look like an arm with fewer runs.
        print(f"!! {len(unmatched)} directory(ies) did not parse as "
              f"<tag>_<arm>_seed<n> and are ABSENT from every table below: "
              + ", ".join(sorted(unmatched)))
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
    hdr = (f"{'run':<26}{'t_lead':>9}{'t_team':>9}{'Δdecl':>8}{'lag':>9}{'lag_m':>8}"
           f"{'map_end':>9}{'map_pk':>8}{'dist_m':>9}{'unk':>7}{'fire':>6}{'fire_s':>8}")
    print(hdr); print("-" * len(hdr))
    for arm in sorted(arms):
        for r in sorted(arms[arm], key=lambda x: x["seed"]):
            tt = "CENSORED" if r["censored"] else f"{r['t_team']:.0f}"
            me = "--" if r["map_end"] is None else f"{r['map_end']:.2f}"
            # Blank when the two definitions agree to under a plan tick, which
            # is the resolution the crossing is sampled at; a number here is a
            # real separation, not rounding.
            dd = r.get("decl_delta")
            if r.get("decl_censor_split"):
                dcol = "SPLIT"
            elif dd is None:
                dcol = "--"
            elif abs(dd) < 5.0:
                dcol = ""
            else:
                dcol = f"{dd:+.0f}"
            print(f"{arm+'/seed'+str(r['seed']):<26}{r['t_lead'] or 0:>9.0f}{tt:>9}"
                  f"{dcol:>8}{r['lag'] or 0:>9.0f}{r['lag_dist'] or 0:>8.0f}"
                  f"{me:>9}{r['map_peak']:>8.2f}{r['dist_team']:>9.0f}"
                  f"{r['unk_floor'] or 0:>7.3f}{r['fire']:>6}{r['fire_s']:>8.0f}")

    # The cross-check, stated loudly rather than left as a column to notice.
    split = [(a, r) for a in sorted(arms) for r in arms[a]
             if r.get("decl_censor_split")]
    drift = [(a, r) for a in sorted(arms) for r in arms[a]
             if r.get("decl_delta") is not None and abs(r["decl_delta"]) >= 5.0]
    mislabel = [(a, r) for a in sorted(arms) for r in arms[a]
                if r.get("arm_stamped") and r["arm_stamped"] != a]
    # Cells the cross-check could not run on at all. Reported even when nothing
    # else fired, because "no disagreements" over zero comparisons is not a
    # clean bill of health and must not be allowed to read as one.
    unchecked = [(a, r) for a in sorted(arms) for r in arms[a]
                 if r.get("decl_excluded")]
    if unchecked:
        n_tot = sum(len(arms[a]) for a in arms)
        print(f"\nCOMPLETION CROSS-CHECK NOT RUN on {len(unchecked)} of "
              f"{n_tot} cells — the checks below cover the rest only:")
        by_reason = {}
        for a, r in unchecked:
            # Group by the reason's shape, not its text: the schema refusal
            # names a different file path every time and would otherwise print
            # one line per cell.
            key = re.sub(r"/\S+", " <path>", str(r["decl_excluded"]))
            key = re.sub(r"\bseed\d+\b", "seed<N>", key)
            by_reason.setdefault(key, []).append(f"{a}/seed{r['seed']}")
        for key, cells in sorted(by_reason.items()):
            shown = ", ".join(cells[:6]) + (" …" if len(cells) > 6 else "")
            print(f"    {len(cells)}x {key}")
            print(f"        {shown}")
    if split or drift or mislabel:
        print("\nCOMPLETION CROSS-CHECK — this table's t_team is the CSV "
              "crossing of unknown_fraction;")
        print("the planner separately states when it stopped trying "
              "(explore_done_sim_sec). Where they")
        print("disagree, the table's number is a reconstruction and the "
              "planner's is what the robot did:")
        for a, r in drift:
            print(f"    {a}/seed{r['seed']}: crossing {r['t_team']:.0f}s vs "
                  f"declaration {r['t_team_decl']:.0f}s "
                  f"({r['decl_delta']:+.0f}s)")
        for a, r in split:
            print(f"    {a}/seed{r['seed']}: {r['decl_censor_split']} — one "
                  f"measure has a completion time and the other does not")
        for a, r in mislabel:
            print(f"    {a}/seed{r['seed']}: event log stamps this run "
                  f"arm={r['arm_stamped']!r}, but it is filed under {a!r} — "
                  f"one of the two is wrong and the arm means nothing until "
                  f"you know which")

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

    # --- guards that must be read BEFORE any delta ---------------------------
    builds = {}
    bad_verdict = []
    for arm, rs in arms.items():
        for r in rs:
            builds.setdefault(r["build"], []).append(f"{arm}/seed{r['seed']}")
            if r["verdict"] == "INVALID":
                bad_verdict.append(f"{arm}/seed{r['seed']}")
    if len(builds) > 1:
        # A differing hash is necessary but NOT sufficient evidence of
        # confounding: this repo carries the analysis scripts alongside the
        # planner, so committing a change to sim/*.py mid-campaign moves the
        # recorded hash without touching the binary that ran. That happened
        # during Phase 7 and would have discarded a perfectly good matrix.
        # Name the check rather than the verdict.
        print(f"\n!! ARMS RECORD DIFFERENT COMMITS. This is confounded with the "
              f"build ONLY IF the difference reaches compiled code:")
        for b, who in sorted(builds.items()):
            print(f"       {b}: {', '.join(sorted(who))}")
        pair = " ".join(sorted(builds))
        print(f"   Settle it, do not assume:  git diff --stat {pair} -- "
              f"'*.cpp' '*.hpp' 'config/'")
        print(f"   Empty diff (plus an install tree older than the first cell) "
              f"means the same binary ran everywhere and the matrix stands.")
    if bad_verdict:
        print(f"\n!! IN THE MEDIANS DESPITE A FAILED GATE VERDICT: "
              f"{', '.join(sorted(bad_verdict))}. Their manipulation check did "
              f"not pass, so it is unverified that the comms treatment applied.")

    pooled = []
    for rs in arms.values():
        tt = [r["t_team"] for r in rs if not r["censored"]]
        if len(tt) >= 2 and st.mean(tt):
            pooled.append(st.pstdev(tt) / st.mean(tt))
    if pooled:
        cv = st.mean(pooled)
        # MDE from the same lognormal permutation model the p-values assume.
        # Calibrated against the measured null: CV 0.56 -> ~70 % needed at n=5.
        nmin = min(len(rs) for rs in arms.values())
        mde = 1.0 - math.exp(-2.49 * cv / max(nmin, 1) ** 0.5)
        print(f"\nPOWER: pooled within-arm CV = {cv:.2f} at n={nmin}/arm. "
              f"80 % power reaches only a ~{mde * 100:.0f} % speed-up; anything "
              f"smaller is UNDETECTABLE HERE BY CONSTRUCTION, and a null result "
              f"excludes nothing. Replicates of one identical config on this "
              f"pipeline span 3.03x at unknown<=0.55 (CV 0.56), so treat every "
              f"delta below as a pilot effect-size estimate, not a ranking.")

    # How much of the endpoint the treatment can physically reach. The manoeuvre
    # only triggers once a robot has FINISHED exploring (finishOrRendezvous, the
    # sole entry point, is reached only on step-budget / coverage-saturated), so
    # it can act inside `lag` and nowhere else. Everything before t_lead is
    # untreatable by construction, and it is the bulk of the run.
    all_rs = [r for rs in arms.values() for r in rs]
    lags = [r["lag"] for r in all_rs if r["lag"] is not None]
    tts = [r["t_team"] for r in all_rs if r["t_team"] is not None]
    if lags and tts:
        frac = st.median(lags) / st.median(tts) * 100.0
        # The claim below holds ONLY where every manoeuvre is terminal. A
        # generation-8 campaign has a mid-run trigger that fires during
        # exploration, so print the caveat rather than the conclusion when the
        # cells show mid-run dispatches — otherwise this block tells the reader
        # the treatment cannot reach ~90 % of the endpoint, which is false.
        n_mid = sum(r.get("midrun", 0) or 0 for r in all_rs)
        print(f"\nMECHANISM WINDOW: median lag {st.median(lags):.0f} s of a median "
              f"t_team {st.median(tts):.0f} s = {frac:.1f} % of the endpoint.")
        if n_mid:
            print(f"    !! {n_mid} MID-RUN dispatch(es) present — the manoeuvre is "
                  f"NOT terminal in these cells, so the `lag` window below does "
                  f"NOT bound where the treatment can act. Do not quote this "
                  f"block; use event_log.py, the pre-registered reader.")
        else:
            print(f"    The manoeuvre is TERMINAL here (0 mid-run dispatches) — it "
                  f"can only fire once a robot has finished exploring, so it acts "
                  f"inside `lag` and nowhere else.\n"
                  f"    The other {100 - frac:.1f} % of t_team is untreatable by "
                  f"construction and enters the comparison as pure noise. "
                  f"Read `lag`.")

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
    noise_floor(args.null_runs, args.threshold, summary, ctl)
    ladder(args.runs, args.threshold, args.ladder)

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
