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

It deliberately REUSES team_convergence.py's own readers and pairing rather
than reimplementing them: a control that scores its subject with different code
tests the difference between two scripts, not the difference between two runs.
The single precondition dropped is the team_exchange one -- which is exactly
the configuration under test.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import team_convergence as tc          # noqa: E402

HEAL_WINDOW = 120.0
PAIR_TOL = 3.0


def score(d, label):
    logs = sorted(f for f in os.listdir(d) if f.endswith(".events.jsonl"))
    robots = [f[: -len(".events.jsonl")] for f in logs]
    census = {}
    exch = {}
    for name, f in zip(robots, logs):
        census[name], exch[name] = tc.read_events(os.path.join(d, f))

    n_exch = sum(len(exch[r]) for r in robots)
    outages = tc.read_outages(os.path.join(d, "link_states.csv"))
    pairs, ua, ub = tc.pair_samples(census[robots[0]], census[robots[1]],
                                    PAIR_TOL)
    agree = [(t, ea["shared_hash"] == eb["shared_hash"])
             for t, ea, eb in pairs]
    n_agree = sum(1 for _, ok in agree if ok)

    print(f"=== {label}  ({os.path.basename(d)}) ===")
    print(f"  team_exchange events        : {n_exch}")
    print(f"  link outages                : {len(outages)}  "
          f"({sum(b - a for a, b, _ in outages):.0f} s down)")
    print(f"  paired census samples       : {len(pairs)} "
          f"({ua}+{ub} unpaired)")
    print(f"  shared_hash agreed          : {n_agree}/{len(agree)} "
          f"({n_agree / max(1, len(agree)) * 100:.0f}%)")

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


root = os.path.expanduser("~/hmr_campaign/p2_teamworld_smoke")
t_scored, t_conv, t_ex = score(os.path.join(root, "smoke2"),
                               "TEAM_WORLD=1  (treatment)")
print()
c_scored, c_conv, c_ex = score(os.path.join(root, "smoke2_control"),
                               "TEAM_WORLD=0  (control)")

print("\n=== what this decides ===")
if c_ex != 0:
    print("INVALID: the control logged team_exchange events; it is not a "
          "control.")
elif c_scored == 0:
    print("INCONCLUSIVE: the control had no scoreable outage, so it cannot "
          "say whether the gate passes without the exchange. Needs a re-run, "
          "not an interpretation.")
elif c_conv == c_scored:
    print("The gate is NON-DIAGNOSTIC: the cell worlds converged after every "
          "scoreable heal with the exchange switched off entirely. dscovox "
          "alone produces the PASS, so the P2 smoke cannot detect a broken "
          "TeamWorld. The unit tests remain the only real proof of the merge.")
elif c_conv == 0:
    print("The gate is DIAGNOSTIC: with the exchange off the worlds did not "
          "reconverge after any scoreable heal, so the treatment run's PASS "
          "is attributable to TeamWorld.")
else:
    print(f"PARTIAL: control converged on {c_conv}/{c_scored} scoreable "
          f"heals. The gate is weakened but not empty; the per-outage detail "
          f"above is the finding, not the ratio.")
