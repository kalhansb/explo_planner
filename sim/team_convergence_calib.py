#!/usr/bin/env python3
"""Known-answer calibration for team_convergence.py.

Same discipline as equiv_gate_calib.py and for the same reason: this repo has
watched six guards go inert while still printing passes
([[checks-that-stopped-checking]]), and the only thing that catches that is a
case that must FAIL. Every case here asserts on the RETURN CODE and on a
message pattern, because matching a substring of stdout alone is how a check
passes forever against a gate that stopped saying what it used to say.

The case that matters most is `counts agree, ground does not`: it is the
failure mode the whole readout exists to avoid, and it is the one a
count-based rewrite would silently reintroduce.

Usage:  team_convergence_calib.py
Exit:   0 every case behaved as declared
        1 at least one did not
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
GATE = os.path.join(HERE, "team_convergence.py")

ROBOTS = ["atlas", "bestla"]
GRID = 3735928559          # the grid both robots share unless a case says else

fails = 0


def census(t, shared, covered=2, grid=GRID, drop_shared=False):
    e = {"event": "cell_census", "seq": 0, "robot": "-", "t_sim_sec": t,
         "t_rel_sec": t, "t_wall_sec": 1787851740.0 + t, "state": "EXPLORE",
         "step": 1, "cells_total": 9, "unseen": 9 - covered, "exploring": 0,
         "covered": covered, "exploring_by_others": 0, "covered_by_others": 0,
         "covered_fraction": covered / 9.0, "grid_hash": grid,
         "shared_hash": shared}
    if drop_shared:
        del e["shared_hash"]
    return e


def exchange(t, drop=""):
    return {"event": "team_exchange", "seq": 0, "robot": "-", "t_sim_sec": t,
            "t_rel_sec": t, "t_wall_sec": 1787851740.0 + t, "state": "EXPLORE",
            "step": 1, "peer": "other", "peer_id": 1, "drop_reason": drop,
            "coalesced": 1, "cells_in_msg": 9, "applied": 1, "in_comms": True,
            "direct": True, "covered": 2, "covered_by_others": 0,
            "exploring": 0, "exploring_by_others": 0,
            "covered_fraction": 0.22}


def build(root, series_a, series_b, exch_a=None, exch_b=None, outages=None,
          link_period=10.0):
    """One cell directory. `series_*` are lists of census dicts.

    `link_period` is the spacing of the link_states.csv rows. It defaults to
    10 s, coarser than the 20 s census spacing, so every outage a case
    declares contains at least one census sample. Drop it to write an outage
    SHORTER than one census period — the flicker case, which is the whole
    reason the reader has an empty-observation branch.
    """
    os.makedirs(root, exist_ok=True)
    for name, series, exch in ((ROBOTS[0], series_a,
                                exch_a if exch_a is not None else
                                [exchange(t) for t in (20.0, 200.0)]),
                               (ROBOTS[1], series_b,
                                exch_b if exch_b is not None else
                                [exchange(t) for t in (20.0, 200.0)])):
        with open(os.path.join(root, f"{name}.events.jsonl"), "w") as f:
            for e in series:
                e = dict(e, robot=name)
                f.write(json.dumps(e) + "\n")
            for e in exch:
                f.write(json.dumps(dict(e, robot=name)) + "\n")
    if outages is not None:
        with open(os.path.join(root, "link_states.csv"), "w") as f:
            f.write("t_sim,i,j,distance_m,trees_on_link,path_loss_db,snr_db,"
                    "ber,bandwidth_mbps,connected\n")
            # One sample every `link_period` from 0 to 300, connected unless
            # inside an outage interval. The emulator writes at link_rate_hz;
            # the period does not matter to the reader, only the up/down
            # transitions do — but it bounds the NARROWEST outage a case can
            # express, and a sub-census-period outage is a distinct case.
            for k in range(0, int(300.0 / link_period) + 1):
                t = k * link_period
                up = 0 if any(a <= t < b for a, b in outages) else 1
                f.write(f"{t:.3f},0,1,12,3,70,20,0,54,{up}\n")


def case(label, expect_rc, expect_pat=None, **kw):
    global fails
    root = tempfile.mkdtemp(prefix="teamconv_")
    try:
        build(os.path.join(root, "cell"), **kw)
        r = subprocess.run([sys.executable, GATE, os.path.join(root, "cell")],
                           capture_output=True, text=True)
        out = r.stdout + r.stderr
        ok = r.returncode == expect_rc
        why = "" if ok else f" (rc={r.returncode}, want {expect_rc})"
        if ok and expect_pat:
            ok = re.search(expect_pat, out) is not None
            if not ok:
                why = f" (rc ok, but no message matching {expect_pat!r})"
        print(("  PASS  " if ok else "  FAIL  ") + label + why)
        if not ok:
            fails += 1
            print("    " + "\n    ".join(out.splitlines()[:14]))
    finally:
        shutil.rmtree(root, ignore_errors=True)


# ---------------------------------------------------------------------------
# The shape of a healthy run: agreed, diverged while the link was down at
# [100, 180), agreed again once it healed.
TIMES = [float(t) for t in range(0, 301, 20)]


def healthy(diverge=(100.0, 180.0), heal_at=200.0):
    a, b = [], []
    for t in TIMES:
        a.append(census(t, shared=1000 + int(t)))
        if diverge[0] <= t < heal_at:
            b.append(census(t, shared=9000 + int(t)))   # different ground
        else:
            b.append(census(t, shared=1000 + int(t)))
    return a, b


print("=== the healthy direction ===")
a, b = healthy()
case("diverged under the outage and agreed again after it healed", 0,
     r"PASS\tconvergence", series_a=a, series_b=b, outages=[(100.0, 180.0)])

print("\n=== the injections a convergence claim must not survive ===")
# The one the readout exists for. Both robots cover exactly two cells at every
# sample, so every count in cell_census is identical throughout — and the
# ground is not. A count-based check reports perfect agreement here.
a, b = healthy()
for e in a + b:
    e["covered"] = 2
    e["covered_by_others"] = 0
    e["covered_fraction"] = 2 / 9.0
case("counts agree at every sample while the ground does not", 0,
     r"diverged over \[100, 180\]",
     series_a=a, series_b=b, outages=[(100.0, 180.0)])

# A pair that never came apart is not evidence of anything.
same = [census(t, shared=1000 + int(t)) for t in TIMES]
case("the worlds never disagreed", 1, r"never disagreed at any paired sample",
     series_a=same, series_b=list(same), outages=[(100.0, 180.0)])

# Diverged and stayed diverged: the exchange did not heal it.
a, b = healthy(heal_at=1e9)
case("diverged and never agreed again", 1,
     r"never agreed again for the rest of the run",
     series_a=a, series_b=b, outages=[(100.0, 180.0)])

# Healed, but long after the link came back — the merge is not the thing that
# fixed it, and a generous window would call this a pass. The outage is short
# so the 160 s gap is unambiguously outside the 120 s window: with the default
# [100, 180) outage the last sample lands exactly ON the boundary, and a case
# that straddles the comparison tests nothing but which way `>` rounds.
a, b = healthy(heal_at=300.0)
case("agreed again only long after the link recovered", 1,
     r"s after the link recovered \(window 120",
     series_a=a, series_b=b, outages=[(100.0, 140.0)])

# A message the receiver refused is a configuration fault; it must not be
# absorbed into "the radio was down".
a, b = healthy()
case("every TeamWorld dropped for a config mismatch", 1,
     r"message\(s\) dropped:.*grid_hash_mismatch",
     series_a=a, series_b=b, outages=[(100.0, 180.0)],
     exch_a=[exchange(20.0, "grid_hash_mismatch")],
     exch_b=[exchange(20.0, "grid_hash_mismatch")])

print("\n=== runs that cannot be scored must refuse, not pass ===")
a, b = healthy()
case("the two robots ran different cell grids", 2,
     r"different cell grids",
     series_a=a, series_b=[dict(e, grid_hash=GRID + 1) for e in b],
     outages=[(100.0, 180.0)])
case("a binary older than the readout wrote no shared_hash", 2,
     r"carry no shared_hash",
     series_a=[census(t, 0, drop_shared=True) for t in TIMES],
     series_b=[census(t, 0, drop_shared=True) for t in TIMES],
     outages=[(100.0, 180.0)])
case("CELL_WORLD was off, so there is no cell world", 2,
     r"logged no cell_census events",
     series_a=[], series_b=[], outages=[(100.0, 180.0)])
a, b = healthy()
case("the exchange was off, so nothing was ever merged", 2,
     r"logged no team_exchange events",
     series_a=a, series_b=b, outages=[(100.0, 180.0)],
     exch_a=[], exch_b=[])
# Samples too far apart to compare: silence about the pairing would let the
# verdict be about whichever handful happened to line up.
case("the two robots' census samples do not line up", 2,
     r"do not overlap in sim time",
     series_a=[census(t, 1000 + int(t)) for t in TIMES],
     series_b=[census(t + 50.0, 1000 + int(t)) for t in TIMES],
     outages=[(100.0, 180.0)])

print("\n=== the outage must be real, and it must be scoreable ===")
# THE CASE THIS SUITE WAS MISSING. The first real smoke run scored by this
# gate had the link connected on all 6493 samples — the robots never separated
# enough to lose it — and the gate printed PASS: the per-outage loop never
# executed, and the guard meant to catch "nothing was scored" was itself
# written `if outages and scored == 0`, so an empty outage list skipped it.
#
# Note what makes it nasty: the run does diverge. Two robots sampling a shared
# fused map at slightly different sim times disagree on shared_hash from timing
# skew alone, so the "must have diverged somewhere" requirement — the guard
# specifically there to reject vacuous passes — was satisfied by something that
# is not a dropout. Every ingredient of the pass was real except the dropout.
# Hence `healthy()` here, unmodified: this fixture is a PASSING run in every
# respect but the one that matters.
a, b = healthy()
case("the link never dropped, so there is no heal to score", 2,
     r"link never dropped: connected on every sample",
     series_a=a, series_b=b, outages=[])
case("no link trace at all", 2, r"no link_states\.csv",
     series_a=a, series_b=b, outages=None)
# An outage still down when the trace ends has no heal to score. This case
# found a real defect: the reader used to close such an outage at the last
# link sample and the scorer relied on there being no later census sample to
# notice, so a census tick landing on that same second scored the run for
# failing to converge across a recovery that never happened. The verdict is
# now driven by whether the link was OBSERVED to come back, not by sample
# alignment — hence the deliberately coincident stamps here (the trace and the
# census both end at 300).
#
# rc is 1 rather than 0 because this run also has nothing else to score: the
# only outage is unscoreable, which the gate reports separately below.
a, b = healthy(diverge=(240.0, 1e9), heal_at=1e9)
case("an outage still down at teardown is not scored as a failure to heal", 1,
     r"still down when the trace ended — there is no heal to score",
     series_a=a, series_b=b, outages=[(240.0, 400.0)])

# A heal the run gave no room to observe. The second outage recovers on the
# very last census sample, so there is no undisturbed link after it at all —
# the pair cannot be shown to converge and cannot be shown to fail. The first
# smoke run to reach this code failed two such outages at t=1517 and t=1522 in
# a run that stopped at 1559, which says nothing about the exchange.
#
# Both cases below share a fixture and differ ONLY in the outage list, because
# the pair is the point: censoring must rescue a truncated heal WITHOUT
# rescuing a run that has nothing else to show.
a, b = healthy()
b[-1] = census(300.0, shared=77777)      # still apart at the final sample
case("a heal with no room left to observe is censored, not failed", 0,
     r"censored, not scored",
     series_a=a, series_b=b, outages=[(100.0, 180.0), (290.0, 295.0)])
case("a run whose every outage is censored does not pass", 1,
     r"none could be scored",
     series_a=a, series_b=b, outages=[(290.0, 295.0)])

# An outage shorter than one census period observes NOTHING: no paired sample
# falls inside it. The scoring loop used to fall through such an outage to
# `healed`, which picks the first agreeing sample after the end — for a pair
# that had not diverged in that window, the very next tick — and recorded a
# "converged 1 s after the link recovered" that no merge produced. Census
# samples here are 20 s apart, so the flicker at [245, 247) contains none.
# link_period=1.0 because the default 10 s link grid cannot express it.
print("\n=== an outage too short to observe anything ===")
a, b = healthy()
case("a flicker with no census sample inside it is not scored as a heal", 0,
     r"contains no paired census sample",
     series_a=a, series_b=b, link_period=1.0,
     outages=[(100.0, 180.0), (245.0, 247.0)])
# And the direction that proves the skip is a skip and not an excuse: with the
# flicker as the ONLY outage there is nothing left to score, and the run must
# fail rather than inherit a pass from the outage it declined to judge.
case("a run whose only outage observed nothing does not pass", 1,
     r"none could be scored",
     series_a=a, series_b=b, link_period=1.0, outages=[(245.0, 247.0)])

print()
if fails:
    print(f"{fails} case(s) did not behave as declared")
    sys.exit(1)
print("ALL PASS")
