#!/usr/bin/env python3
"""Does the cell world converge after a heal with the exchange SWITCHED OFF?

This is the control for team_convergence.py, and it is a calibration of that
gate rather than a measurement of the planner. The P2 gate asks "do both cell
worlds converge after a dropout heals" and answers it on a run where TeamWorld
is on -- but the cell world is DERIVED from the fused dscovox map, and the map
is relayed over the same emulated radio. So the map alone may already produce
the convergence the gate credits to the exchange.

If this script finds convergence on a TEAM_WORLD=0 cell, the P2 smoke gate
cannot fail when the exchange is broken, and its PASS means only "dscovox
healed", which is not the claim on the tin.

It deliberately REUSES team_convergence.py's own readers, pairing AND
preconditions rather than reimplementing them: a control that scores its
subject with different code tests the difference between two scripts, not the
difference between two runs. The single precondition dropped is the
team_exchange one -- which is exactly the configuration under test, and which
is dropped by passing require_exchange=False to the shared check, so it stays
one named exemption rather than a copy that drifts.

That sentence was true of the readers and false of the preconditions for as
long as this file had none: it went straight from listdir to scoring. A cell
with one robot's log, with no cell_census rows, or with no link trace raised
IndexError, IndexError and TypeError respectively -- all three confirmed
against the version this replaces. The mismatched-grid case is the one that
mattered, because it does not crash: the gate refuses it because two robots on
different grids "would otherwise score a permanent divergence", and a permanent
divergence is what this script reads as "did NOT converge" on every outage,
which is c_conv == 0, which prints DIAGNOSTIC. The flattering answer, reached
from a broken run.

Usage:  team_convergence_control.py [TREATMENT_CELL CONTROL_CELL]
        Defaults to the banked P2 smoke pair.
Exit:   0  a verdict was reached (DIAGNOSTIC, NON-DIAGNOSTIC or PARTIAL)
        2  it was not: a cell could not be scored, or the pair is not a pair
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import team_convergence as tc          # noqa: E402

HEAL_WINDOW = 120.0
PAIR_TOL = 3.0

DEFAULT_ROOT = os.path.expanduser("~/hmr_campaign/p2_teamworld_smoke")


def refuse(msg):
    print(f"  REFUSED\t{msg}")
    return None


def manifest_team_world(d):
    """The recorded TEAM_WORLD, or None if this cell does not record one."""
    p = os.path.join(d, "run_manifest.txt")
    if not os.path.exists(p):
        return None
    for ln in open(p, errors="replace"):
        k, _, v = ln.strip().partition("=")
        if k == "team_world":
            return v
    return None


def score(d, label, expect_team_world):
    """Score one cell, or refuse it.

    Returns (scored, converged, n_exch), or None when the cell cannot be
    scored. Refusing is the point: this script's two interesting verdicts are
    both statements about a control run, and a control that could not be read
    supports neither of them.
    """
    print(f"=== {label}  ({os.path.basename(os.path.normpath(d))}) ===")
    if not os.path.isdir(d):
        return refuse(f"not a directory: {d}")

    # Which run this is, from the record rather than from the directory name.
    # The whole script rests on one cell having had the exchange on and the
    # other having had it off; a mislabelled pair would compare a treatment
    # against a treatment and report the gate as diagnostic.
    mtw = manifest_team_world(d)
    if mtw is None:
        print(f"  note: no team_world line in run_manifest.txt; the arm is "
              f"taken from the team_exchange count alone")
    elif mtw != expect_team_world:
        return refuse(f"run_manifest.txt records team_world={mtw!r}, but this "
                      f"side of the comparison must be team_world="
                      f"{expect_team_world!r}. The two cells given are not a "
                      f"treatment/control pair.")

    logs = sorted(f for f in os.listdir(d) if f.endswith(".events.jsonl"))
    if len(logs) != 2:
        # Same refusal as the gate, for the same reason: pair_samples compares
        # exactly two robots. An N-robot cell (P7) needs a pairwise scoring
        # this script does not have, and picking the first two alphabetically
        # would answer a question nobody asked.
        return refuse(f"expected exactly 2 robot event logs, found "
                      f"{len(logs)}: {logs}")
    robots = [f[: -len(".events.jsonl")] for f in logs]

    census, exch = {}, {}
    for name, f in zip(robots, logs):
        census[name], exch[name] = tc.read_events(os.path.join(d, f))
    n_exch = sum(len(exch[r]) for r in robots)
    outages = tc.read_outages(os.path.join(d, "link_states.csv"))

    # Every precondition the gate applies except the team_exchange one.
    for stage in tc.precondition_stages(robots, census, exch, outages,
                                        require_exchange=False):
        if stage:
            for m in stage:
                refuse(m)
            return None

    pairs, ua, ub = tc.pair_samples(census[robots[0]], census[robots[1]],
                                    PAIR_TOL)
    if not pairs:
        return refuse(f"no census sample pair within {PAIR_TOL}s — the two "
                      f"robots' logs do not overlap in sim time")
    agree = [(t, ea["shared_hash"] == eb["shared_hash"])
             for t, ea, eb in pairs]
    n_agree = sum(1 for _, ok in agree if ok)

    print(f"  team_exchange events        : {n_exch}")
    print(f"  link outages                : {len(outages)}  "
          f"({sum(b - a for a, b, _ in outages):.0f} s down)")
    print(f"  paired census samples       : {len(pairs)} "
          f"({ua}+{ub} unpaired)")
    print(f"  shared_hash agreed          : {n_agree}/{len(agree)} "
          f"({n_agree / len(agree) * 100:.0f}%)")

    last_census_t = agree[-1][0]
    starts = sorted(s for s, _, _ in outages)
    scored = converged_n = 0
    for start, end, recovered in outages:
        if not recovered:
            continue
        after = [(t, ok) for t, ok in agree if t >= end]
        during = [ok for t, ok in agree if start <= t <= end]
        if not after or (during and all(during)):
            continue
        nxt = next((s for s in starts if s > end), None)
        clean_end = last_census_t if nxt is None else min(nxt, last_census_t)
        healed = next((t for t, ok in after if ok), None)
        conv = healed is not None and healed - end <= HEAL_WINDOW
        if not conv and clean_end - end < HEAL_WINDOW:
            print(f"    outage [{start:.0f},{end:.0f}] censored")
            continue
        scored += 1
        converged_n += 1 if conv else 0
        print(f"    outage [{start:.0f},{end:.0f}] "
              + (f"CONVERGED {healed - end:.0f}s after heal" if conv
                 else "did NOT converge"))
    print(f"  scored outages              : {scored}, "
          f"converged {converged_n}")
    return scored, converged_n, n_exch


def main(argv):
    if len(argv) == 3:
        treatment, control = argv[1], argv[2]
    elif len(argv) == 1:
        treatment = os.path.join(DEFAULT_ROOT, "smoke2")
        control = os.path.join(DEFAULT_ROOT, "smoke2_control")
    else:
        print(__doc__.strip())
        return 2

    t = score(treatment, "TEAM_WORLD=1  (treatment)", "1")
    print()
    c = score(control, "TEAM_WORLD=0  (control)", "0")

    print("\n=== what this decides ===")
    # The treatment side is not the finding, but it is the reference: a control
    # that cannot be compared against a scoreable treatment says nothing about
    # a gate nobody has watched pass.
    if t is None or c is None:
        side = "treatment" if t is None else "control"
        print(f"UNRESOLVED: the {side} cell could not be scored (see above). "
              f"Nothing here is evidence about the gate either way.")
        return 2
    c_scored, c_conv, c_ex = c
    if c_ex != 0:
        print("INVALID: the control logged team_exchange events; it is not a "
              "control.")
        return 2
    if c_scored == 0:
        print("INCONCLUSIVE: the control had no scoreable outage, so it cannot "
              "say whether the gate passes without the exchange. Needs a "
              "re-run, not an interpretation.")
        return 2
    if c_conv == c_scored:
        print("The gate is NON-DIAGNOSTIC: the cell worlds converged after "
              "every scoreable heal with the exchange switched off entirely. "
              "dscovox alone produces the PASS, so the P2 smoke cannot detect "
              "a broken TeamWorld. The unit tests remain the only real proof "
              "of the merge.")
    elif c_conv == 0:
        print("The gate is DIAGNOSTIC: with the exchange off the worlds did "
              "not reconverge after any scoreable heal, so the treatment run's "
              "PASS is attributable to TeamWorld.")
    else:
        print(f"PARTIAL: control converged on {c_conv}/{c_scored} scoreable "
              f"heals. The gate is weakened but not empty; the per-outage "
              f"detail above is the finding, not the ratio.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
