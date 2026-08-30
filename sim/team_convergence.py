#!/usr/bin/env python3
"""Score the P2 gate: do two cell worlds converge after a dropout heals?

docs/mtare_evolution_plan.md's P2 gate asks for "a 2-robot smoke with COMMS=1
showing both cell worlds converging after a dropout heals". This file is what
reads that out of a cell directory, and it exists because the obvious way to
read it is wrong.

WHY NOT THE COUNTS
------------------
cell_census logs each robot's status histogram, and the tempting check is
"their covered counts came back together". Two robots can hold identical
unseen/exploring/covered totals over completely different ground — which is
the NORMAL state during a dropout, when each has covered about as much as the
other somewhere else. A count-based readout calls that convergence at the
moment of maximum divergence. So the quantity here is shared_hash: FNV-1a over
every cell's WIRE status in id order (CellWorld::sharedHash()), which is equal
exactly when the two robots agree cell for cell about the shared belief. The
counts are still printed, because they say how far along the mission was, but
no verdict reads them.

WHAT "CONVERGED" MEANS, PRECISELY
---------------------------------
Equal shared_hash at a common sample time, having been unequal earlier in the
same run. Both halves are required and the second is the one that matters: two
robots that never diverged would score a perfect agreement fraction and prove
nothing about the exchange, since a pair with the radio unplugged and nothing
to say would do the same. The verdict therefore refuses a run in which the
worlds never came apart, rather than passing it.

Convergence is scored at the SAMPLE grid, not continuously: cell_census fires
on its own period per robot and the two robots are not in lockstep, so samples
are paired by nearest neighbour within --pair-tol seconds and unpaired samples
are dropped and counted. A run whose pairing rate is low is reported as such
instead of being scored on whatever happened to line up.

WHAT THIS GATE DOES **NOT** SHOW
--------------------------------
It does not show that the exchange caused the convergence, and the wording of
the PASS line says so. On 2026-08-30 this was measured rather than argued: the
same cell re-run with TEAM_WORLD=0 -- no TeamWorld publisher, zero
team_exchange events -- converged after 7 of 7 scoreable heals, against the
treatment's 2 of 2. sim/team_convergence_control.py is that comparison and
should be re-run before anyone cites this PASS as evidence about a merge.

The reason is structural, not a tuning accident. The cell world is derived from
the FUSED dscovox map, the fused map already contains the partner's voxels, and
it heals when the radio heals -- so the cell worlds reconverge whether or not a
single TeamWorld message is ever sent. Worse, fusion delivers the peer's
coverage as FIRST-HAND local map data, so the merge is not merely redundant but
pre-empted: across 2838 merged messages in the treatment run, 5 applied any
change and covered_by_others never left 0.

What this gate is therefore FOR: a regression check that the exchange is wired
up and behaving -- messages flow, none are dropped for a configuration fault,
the two robots share a grid, the worlds do not wander permanently apart. The
proof that the MERGE is correct lives in test_cell_world.cpp, which merges wire
messages with no map anywhere in the fixture and so cannot be satisfied by
dscovox doing the work.

Usage:  team_convergence.py CELLDIR [--pair-tol S] [--heal-window S]
                            [--report FILE]
Exit:   0 the gate passes
        1 the gate fails
        2 the run cannot be scored (missing data, wrong configuration)
"""

import argparse
import csv
import json
import os
import sys


def read_events(path):
    """(cell_census samples, team_exchange rows) from one robot's log."""
    census, exch = [], []
    with open(path, errors="replace") as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                e = json.loads(ln)
            except ValueError:
                continue
            kind = e.get("event")
            if kind == "cell_census":
                census.append(e)
            elif kind == "team_exchange":
                exch.append(e)
    return census, exch


def read_outages(path):
    """Link-down intervals in sim seconds, from the emulator's own trace.

    Ground truth for "a dropout happened", taken from link_states.csv rather
    than from the planner's peer_lost events on purpose: peer_lost is a TTL
    expiring on the receiver, which is a consequence of the outage delayed by
    the TTL, and [[comms-link-is-occlusion-gated]] records the last time those
    two were confused. Only the 2-robot case is handled here; the pair is
    whatever (i, j) the file contains.

    Returns (start, end, recovered) triples. The flag is not decoration: an
    outage still open when the trace ends has no heal to score, and the only
    thing distinguishing it from a healed one is that nothing observed the
    link come back. Reporting it as a plain interval and leaving the caller to
    infer the difference from whether any later census sample exists makes the
    verdict depend on sample alignment — a census tick landing on the last
    link sample would score a run for failing to converge across a heal that
    never happened.
    """
    if not os.path.exists(path):
        return None
    samples = []
    with open(path, errors="replace") as f:
        for row in csv.DictReader(f):
            try:
                samples.append((float(row["t_sim"]),
                                float(row["connected"]) > 0.5))
            except (KeyError, TypeError, ValueError):
                continue
    if not samples:
        return None
    samples.sort(key=lambda s: s[0])
    outages, start = [], None
    for t, up in samples:
        if not up and start is None:
            start = t
        elif up and start is not None:
            outages.append((start, t, True))
            start = None
    if start is not None:
        outages.append((start, samples[-1][0], False))
    return outages


def pair_samples(a, b, tol):
    """Nearest-neighbour pairing of two robots' census samples in sim time.

    Returns (pairs, unpaired_a, unpaired_b) where each pair is
    (t, sample_a, sample_b) and t is the mean of the two stamps.
    """
    b_sorted = sorted(b, key=lambda e: e.get("t_sim_sec", 0.0))
    bt = [e.get("t_sim_sec", 0.0) for e in b_sorted]
    pairs, unpaired_a, used = [], 0, set()
    for ea in sorted(a, key=lambda e: e.get("t_sim_sec", 0.0)):
        ta = ea.get("t_sim_sec", 0.0)
        best, best_d = None, tol
        # Linear scan: a smoke cell holds a few hundred samples per robot and
        # a bisect here would be one more thing to get wrong for no gain.
        for i, tb in enumerate(bt):
            if i in used:
                continue
            d = abs(tb - ta)
            if d <= best_d:
                best, best_d = i, d
        if best is None:
            unpaired_a += 1
            continue
        used.add(best)
        pairs.append(((ta + bt[best]) / 2.0, ea, b_sorted[best]))
    return pairs, unpaired_a, len(b_sorted) - len(used)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("celldir")
    ap.add_argument("--pair-tol", type=float, default=3.0,
                    help="max sim-second gap between two robots' census "
                         "samples for them to be compared (default 3)")
    ap.add_argument("--heal-window", type=float, default=120.0,
                    help="sim seconds after a link recovers within which the "
                         "worlds must agree again (default 120)")
    ap.add_argument("--report", default=None,
                    help="append PASS/FAIL lines here, comms_gates.txt style")
    args = ap.parse_args()

    d = args.celldir
    if not os.path.isdir(d):
        print(f"not a directory: {d}", file=sys.stderr)
        return 2

    logs = sorted(f for f in os.listdir(d) if f.endswith(".events.jsonl"))
    if len(logs) != 2:
        print(f"expected exactly 2 robot event logs in {d}, found "
              f"{len(logs)}: {logs}", file=sys.stderr)
        return 2
    robots = [f[:-len(".events.jsonl")] for f in logs]

    census, exch = {}, {}
    for name, f in zip(robots, logs):
        census[name], exch[name] = read_events(os.path.join(d, f))

    fails, notes = [], []

    # --- preconditions: the run must be the one the gate is about -----------
    for r in robots:
        if not census[r]:
            fails.append(f"{r} logged no cell_census events — CELL_WORLD was "
                         f"off, so there is no cell world to converge")
        if not exch[r]:
            fails.append(f"{r} logged no team_exchange events — nothing was "
                         f"ever merged, so a convergence verdict would be "
                         f"about two robots that never spoke")
    if fails:
        for m in fails:
            print(f"FAIL\tprecondition\t{m}")
        return 2

    # A shared_hash comparison is meaningless across different grids, exactly
    # as cell_world.hpp says. Check it rather than assume it: two robots given
    # different ROI params would otherwise score a permanent divergence and be
    # reported as an exchange failure.
    grids = {r: {e.get("grid_hash") for e in census[r]} for r in robots}
    for r in robots:
        if len(grids[r]) != 1:
            fails.append(f"{r} changed grid_hash mid-run ({sorted(grids[r])}) "
                         f"— its own cell ids stopped meaning the same ground")
    if not fails and grids[robots[0]] != grids[robots[1]]:
        fails.append(f"the two robots ran different cell grids "
                     f"({grids[robots[0]]} vs {grids[robots[1]]}) — no "
                     f"cross-robot cell comparison is valid, and the exchange "
                     f"should have dropped every message with "
                     f"grid_hash_mismatch")
    if any("shared_hash" not in e for e in census[robots[0]]):
        fails.append("cell_census rows carry no shared_hash — this cell was "
                     "produced by a binary older than the convergence readout")
    if fails:
        for m in fails:
            print(f"FAIL\tprecondition\t{m}")
        return 2

    # --- was there a dropout at all? ----------------------------------------
    # Read BEFORE anything is scored, because a run in which the link never
    # dropped is not a run this gate has an opinion about. The verdict sentence
    # is "diverged under a dropout and agreed again after it healed"; with no
    # dropout there is no such claim to make.
    #
    # This is a REFUSAL (2), not a failure (1). The exchange may be working
    # perfectly — nothing in a permanently-connected run can tell — and
    # returning 1 would blame the planner for a scenario that never separated
    # the robots.
    #
    # It is here because the first real run scored by this gate had exactly
    # this shape (connected in all 6493 link samples, drop_disconnected == 0 on
    # every poll) and an earlier version of this code printed PASS over it: the
    # per-outage loop simply never executed, `scored` stayed 0, the guard below
    # that catches that was itself written `if outages and scored == 0`, and an
    # empty fails list reads as success. Two robots sampling a SHARED fused map
    # at slightly different sim times disagree on shared_hash now and then from
    # timing skew alone, so even the "must have diverged somewhere" requirement
    # was satisfied — by something that is not a dropout. Every ingredient of
    # the pass was real except the dropout.
    outages = read_outages(os.path.join(d, "link_states.csv"))
    if outages is None:
        fails.append("no link_states.csv in this cell — without the emulator's "
                     "link trace there is no outage to measure a heal against, "
                     "and the planner's own peer_lost events are not a "
                     "substitute (they are downstream of the gate under test)")
    elif not outages:
        fails.append("the link never dropped: connected on every sample in "
                     "link_states.csv, so there is no heal to score and the "
                     "comms arm never differed from the control. Re-run with "
                     "more separation, denser canopy between the robots, or a "
                     "lower tx_power_dbm")
    if fails:
        for m in fails:
            print(f"FAIL\tprecondition\t{m}")
        return 2
    notes.append(f"{len(outages)} link outage(s), longest "
                 f"{max(b - a for a, b, _ in outages):.0f} s, "
                 f"{sum(b - a for a, b, _ in outages):.0f} s down in total")

    # --- the drop reasons, before anything else -----------------------------
    # A run where every message was dropped for a config mismatch would show a
    # flat divergence and read as "the exchange does not work", which is true
    # but not the finding. Name it here instead.
    drops = {}
    for r in robots:
        for e in exch[r]:
            why = e.get("drop_reason") or ""
            if why:
                drops[why] = drops.get(why, 0) + 1
    merged = sum(1 for r in robots for e in exch[r]
                 if not (e.get("drop_reason") or ""))
    if drops:
        fails.append(f"{sum(drops.values())} TeamWorld message(s) dropped: "
                     f"{drops} — a dropped message is a configuration fault, "
                     f"not a radio outage (the radio drops before this point)")
    if merged == 0:
        fails.append("no TeamWorld message was ever merged")
    else:
        notes.append(f"{merged} message(s) merged across both robots, "
                     f"0 dropped")

    # --- pair the samples and score the agreement ---------------------------
    pairs, ua, ub = pair_samples(census[robots[0]], census[robots[1]],
                                 args.pair_tol)
    if not pairs:
        fails.append(f"no census sample pair within {args.pair_tol}s — the "
                     f"two robots' logs do not overlap in sim time")
        for m in fails:
            print(f"FAIL\tconvergence\t{m}")
        return 2
    paired_frac = 2.0 * len(pairs) / (len(census[robots[0]]) +
                                      len(census[robots[1]]))
    notes.append(f"{len(pairs)} paired census sample(s) "
                 f"({paired_frac * 100:.0f}% of all samples; {ua}+{ub} "
                 f"unpaired)")
    if paired_frac < 0.5:
        fails.append(f"only {paired_frac * 100:.0f}% of census samples could "
                     f"be paired within {args.pair_tol}s — the verdict below "
                     f"would be about whichever samples happened to line up")

    agree = [(t, ea["shared_hash"] == eb["shared_hash"], ea, eb)
             for t, ea, eb in pairs]
    n_agree = sum(1 for _, ok, _, _ in agree if ok)
    notes.append(f"shared_hash agreed at {n_agree}/{len(agree)} paired "
                 f"samples ({n_agree / len(agree) * 100:.0f}%)")

    # The run must have DIVERGED at some point, or there is nothing for the
    # exchange to have fixed. See the module docstring: a pair that never came
    # apart scores 100% agreement and is not evidence.
    diverged = [t for t, ok, _, _ in agree if not ok]
    if not diverged:
        fails.append("the two worlds never disagreed at any paired sample, so "
                     "this run cannot show convergence: a pair that never "
                     "came apart looks identical to a pair with nothing to "
                     "say. Run longer, or with a lower tx_power_dbm")
    else:
        notes.append(f"diverged over [{min(diverged):.0f}, "
                     f"{max(diverged):.0f}] sim-s")

    # --- the heal, per outage ----------------------------------------------
    # For each outage: were they apart at any point from its start onward, and
    # did they agree again within --heal-window of the link coming back?
    # How long the pair had an undisturbed link after each heal: from the
    # recovery until the next outage begins, or the census trace ends.
    #
    # Convergence is still LOOKED FOR over the whole rest of the run — evidence
    # the merge worked is evidence wherever it lands — but a FAILURE may only
    # be declared when this window was at least --heal-window long. Converging
    # late, or not at all, is a defect only if the run actually gave the pair
    # an undisturbed link long enough to do it in.
    #
    # Without this, a run that stops during a burst of churn fails on its last
    # few outages every time: each heal is followed by a second or two of link
    # and then the next dropout, no census sample falls in the gap, and "never
    # agreed again for the rest of the run" is true while saying nothing about
    # the exchange. That is right-censoring — the same situation as the
    # still-down-at-teardown case above, truncated by the NEXT outage rather
    # than by the end of the trace.
    #
    # The asymmetry is deliberate and it is the safe direction: a short window
    # can leave a heal unproven, never unrefuted. And a run whose outages are
    # ALL censored does not quietly pass — `scored` stays 0 and the guard below
    # fails it. That guard is what stops this rule becoming an excuse
    # generator, so it must not be weakened to accommodate a churny run.
    last_census_t = agree[-1][0]
    starts = sorted(s for s, _, _ in outages)

    scored = 0
    for start, end, recovered in outages:
        if not recovered:
            notes.append(f"outage [{start:.0f}, {end:.0f}] was still down "
                         f"when the trace ended — there is no heal to score, "
                         f"and calling it a failure to converge would fail "
                         f"every run that stops while the radio is down")
            continue
        after = [(t, ok) for t, ok, _, _ in agree if t >= end]
        during = [ok for t, ok, _, _ in agree if start <= t <= end]
        if not after:
            notes.append(f"outage [{start:.0f}, {end:.0f}] healed after the "
                         f"last census sample — no post-heal sample, not "
                         f"scored")
            continue
        if during and all(during):
            notes.append(f"outage [{start:.0f}, {end:.0f}] passed with the "
                         f"worlds already in agreement — nothing diverged, "
                         f"not scored")
            continue
        nxt = next((s for s in starts if s > end), None)
        clean_end = last_census_t if nxt is None else min(nxt, last_census_t)
        clean_window = clean_end - end
        cut_by = "the trace ended" if nxt is None or nxt > last_census_t \
            else "the next outage began"

        healed = next((t for t, ok in after if ok), None)
        converged = healed is not None and healed - end <= args.heal_window
        if not converged and clean_window < args.heal_window:
            notes.append(f"outage [{start:.0f}, {end:.0f}]: only "
                         f"{clean_window:.0f} s of undisturbed link followed "
                         f"the heal before {cut_by}, short of the "
                         f"{args.heal_window:.0f} s window — censored, not "
                         f"scored (too little observation to call it either "
                         f"way, not evidence of a failure to converge)")
            continue

        scored += 1
        if converged:
            notes.append(f"outage [{start:.0f}, {end:.0f}]: converged "
                         f"{healed - end:.0f} s after the link recovered")
        elif healed is None:
            fails.append(f"outage [{start:.0f}, {end:.0f}]: the worlds never "
                         f"agreed again for the rest of the run, with "
                         f"{clean_window:.0f} s of undisturbed link to do it "
                         f"in")
        else:
            fails.append(f"outage [{start:.0f}, {end:.0f}]: the worlds only "
                         f"agreed again at {healed:.0f} s, "
                         f"{healed - end:.0f} s after the link recovered "
                         f"(window {args.heal_window:.0f} s)")
    # No `outages and` guard here: an empty outage list is now refused above,
    # and that conjunct is exactly what let a zero-outage run reach the PASS.
    if scored == 0 and not fails:
        fails.append(f"{len(outages)} outage(s) happened but none could be "
                     f"scored — every one either ran to the end of the run or "
                     f"passed without the worlds diverging, so nothing here "
                     f"measures a heal")

    # --- the census at the end, for context, never for the verdict ----------
    last = agree[-1]
    for label, e in ((robots[0], last[2]), (robots[1], last[3])):
        notes.append(f"{label} final census: covered={e.get('covered')}"
                     f"+{e.get('covered_by_others')} by others, "
                     f"exploring={e.get('exploring')}"
                     f"+{e.get('exploring_by_others')}, "
                     f"covered_fraction={e.get('covered_fraction')}")

    for m in notes:
        print(f"NOTE\tconvergence\t{m}")
    for m in fails:
        print(f"FAIL\tconvergence\t{m}")
    # The count is IN the verdict, not merely in the notes: "converged after a
    # dropout healed" over zero scored outages is the vacuous pass this gate
    # already emitted once, and a reader skimming for the PASS line would have
    # had no way to see it. A verdict that cannot state how many heals it
    # measured should not be phrased as though it measured one.
    #
    # The attribution disclaimer is in the verdict for the same reason and at
    # the same cost: a TEAM_WORLD=0 control converged after 7 of 7 scoreable
    # heals (see the module docstring), so a PASS line reading only "the worlds
    # converged" would be read as "the exchange works" by every future reader
    # of comms_gates.txt -- including me, which is how this went unnoticed
    # through one full smoke run. The one place a caveat cannot be skipped is
    # the sentence the caveat is about.
    verdict = (f"the cell worlds diverged under a dropout and agreed again "
               f"after it healed ({scored} outage(s) scored) — NOT attributed "
               f"to the exchange: dscovox alone reproduces this (see "
               f"team_convergence_control.py); regression check only")
    if args.report:
        with open(args.report, "a") as f:
            for m in notes:
                f.write(f"NOTE\tconvergence\t{m}\n")
            for m in fails:
                f.write(f"FAIL\tconvergence\t{m}\n")
            if not fails:
                f.write(f"PASS\tconvergence\t{verdict}\n")
    if fails:
        return 1
    print(f"PASS\tconvergence\t{verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
