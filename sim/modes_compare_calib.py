#!/usr/bin/python3
# Moved comments: docs/sim_notes/modes_compare_calib_notes.md
"""Known-answer calibration for the permutation test in modes_compare.py.

modes_compare's p-value used to be exact by enumeration, and an exact test needs
no calibration: it either enumerates C(nx+ny, nx) arrangements or it does not.
Sampling changes that. A sampled p-value is a number produced by a random
number generator, and a generator with a subtle bias produces a number that is
wrong in a way nothing downstream can see -- it prints as an ordinary p, in the
ordinary column, with the ordinary number of decimals. [[lcg-low-bits-bias]] is
the case in this project's own history: a hand-rolled sampler returned a biased
permutation p, and the bias was found by checking it against an answer that was
known independently, not by looking at the output.

So every case here is a KNOWN ANSWER: either arithmetic small enough to do by
hand, or the exact enumeration computed by the same file under a threshold that
forces it. The sampled path is then required to reproduce it.

Two of the cases are calibrations OF THE CALIBRATION -- they swap in a
deliberately biased sampler and require the checks to go red
([[checks-that-stopped-checking]]). A uniformity check that has only ever been
run against a uniform sampler is not known to be able to detect a biased one,
and that is the entire reason this file exists.

Exit: 0 all cases pass, 1 otherwise.
"""
import math
import os
import random
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modes_compare as mc          # noqa: E402

FAILED = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   [{detail}]" if detail
                                                     else ""))
    if not ok:
        FAILED.append(name)
    return ok


def fresh():
    """Drop the memo. Every case changes the thresholds, and a cached answer
    computed under the previous ones would be silently reused -- the case would
    then pass by reporting the result of the case before it."""
    mc._PERM_CACHE.clear()


# --------------------------------------------------------------------------
# 1. the exact path, against arithmetic done by hand
# --------------------------------------------------------------------------
# Pool [1,2,3,4], nx=2: the 6 arrangements give diffs [0,0,1,1,2,2] and obs 2,
# so p = floor = 2/6, because the observed labelling is the most extreme one.
# (notes: calib-exact-2v2-by-hand)
print("=== the exact path, hand-computable ===")
fresh()
obs, diffs, exact = mc.perm_all([1, 2], [3, 4])
check("2v2 enumerates rather than samples", exact is True)
check("2v2 yields all 6 arrangements", len(diffs) == 6, f"got {len(diffs)}")
check("2v2 arrangement multiset is [0,0,1,1,2,2]",
      sorted(diffs) == [0, 0, 1, 1, 2, 2], f"got {sorted(diffs)}")
check("2v2 observed difference is 2", obs == 2, f"got {obs}")
EXACT_2V2 = mc.perm_p([1, 2], [3, 4])
check("2v2 exact p is 2/6", abs(EXACT_2V2 - 1 / 3) < 1e-12, f"got {EXACT_2V2}")
check("2v2 floor is 2/6", abs(mc.perm_floor([1, 2], [3, 4]) - 1 / 3) < 1e-12)

# Unequal groups, where 2/C(2n,n) is wrong: xs=[1] ys=[2,3,4] has 4
# arrangements, diffs [1,1,2,2], obs 2, so p = floor = 0.5, not the closed
# form's 0.333. (notes: calib-unequal-groups-floor)
fresh()
_, d13, _ = mc.perm_all([1], [2, 3, 4])
check("1v3 arrangement multiset is [1,1,2,2]", sorted(d13) == [1, 1, 2, 2],
      f"got {sorted(d13)}")
check("1v3 exact p is 2/4", abs(mc.perm_p([1], [2, 3, 4]) - 0.5) < 1e-12)
check("1v3 floor is 2/4 (not the equal-groups 2/C(4,2)=0.333)",
      abs(mc.perm_floor([1], [2, 3, 4]) - 0.5) < 1e-12)

# --------------------------------------------------------------------------
# 2. the sampled path must reproduce an exactly-known answer
# --------------------------------------------------------------------------
# Same data with the threshold dropped so it takes the sampling branch; the
# sampled p must land on the exact 1/3. Tolerance 0.01 is 3 sigma at M = 20000.
# (notes: calib-sampled-path-vs-exact)
print("\n=== the sampled path, against the exact answer ===")
_old_max, _old_n = mc.PERM_MAX_EXACT, mc.PERM_SAMPLES
mc.PERM_MAX_EXACT, mc.PERM_SAMPLES = 1, 20_000
fresh()
obs, diffs, exact = mc.perm_all([1, 2], [3, 4])
check("threshold forces the sampling branch", exact is False)
check("sampling draws M arrangements", len(diffs) == 20_000, f"{len(diffs)}")
SAMP = mc.perm_p([1, 2], [3, 4])
check("sampled p reproduces the exact 1/3 within 3 sigma",
      abs(SAMP - EXACT_2V2) < 0.01, f"sampled {SAMP:.4f} vs exact {EXACT_2V2:.4f}")

# The sampled null distribution must reproduce the whole exact distribution, not
# just the tail the p-value happens to read. A sampler can get one quantile
# right by luck; matching every arrangement's frequency is the real check.
# Exact shares: 0 -> 2/6, 1 -> 2/6, 2 -> 2/6.
freq = {v: sum(1 for d in diffs if abs(d - v) < 1e-12) / len(diffs)
        for v in (0, 1, 2)}
check("sampled distribution matches the exact one at every value",
      all(abs(freq[v] - 1 / 3) < 0.01 for v in (0, 1, 2)),
      " ".join(f"{v}:{freq[v]:.3f}" for v in (0, 1, 2)))

# --------------------------------------------------------------------------
# 3. the +1 correction
# --------------------------------------------------------------------------
# Two checks, because the interesting input -- a sample containing NO
# arrangement as extreme as the observed one -- is hard to produce on purpose
# and easy to inject.
#
# A median-difference null is full of ties: any swap that leaves both medians in
# place still reaches the observed maximum. The same fact makes perm_floor
# exceed 2/C(2n,n). (notes: calib-median-null-ties)
print("\n=== the +1 correction ===")
LO, HI = list(range(10)), list(range(1000, 1010))
fresh()
obs, diffs, exact = mc.perm_all(LO, HI)
raw = sum(1 for d in diffs if d >= obs - 1e-12)
# What CAN be asserted on real data: p is the corrected ratio and not the raw
# one. With raw > 0 the two differ by more than float noise, so this fails if
# the correction is dropped.
check("p on the sampled path is (1+hits)/(1+M), not hits/M",
      abs(mc.perm_p(LO, HI) - (1 + raw) / (1 + len(diffs))) < 1e-12,
      f"raw hits {raw}/{len(diffs)}")
check("the correction is not a rounding difference",
      abs(raw / len(diffs) - (1 + raw) / (1 + len(diffs))) > 1e-9)

# Zero-hit null, injected: the exact branch must return 0, since an exhaustive
# search found nothing, and the sampled branch 1/(M+1), since a sample that
# missed proves less. (notes: calib-zero-hit-injection)
_real_perm_all = mc.perm_all
mc.perm_all = lambda xs, ys: (1.0, [0.0] * 20_000, False)
check("a sampled null with zero hits gives 1/(M+1), not 0",
      abs(mc.perm_p([1], [2]) - 1 / 20_001) < 1e-12)
check("that p never renders as 0.000", mc.fmt_p(mc.perm_p([1], [2])) != "0.000",
      mc.fmt_p(mc.perm_p([1], [2])))
mc.perm_all = lambda xs, ys: (1.0, [0.0] * 20_000, True)
check("an EXHAUSTIVE null with zero hits still gives 0 (no correction)",
      mc.perm_p([1], [2]) == 0.0)
mc.perm_all = _real_perm_all

fresh()
check("sampled floor is 1/(M+1)",
      abs(mc.perm_floor(LO, HI) - 1 / 20_001) < 1e-12)
# ...and that floor must NOT trip the caller's "cannot be significant" warning,
# which is keyed on floor > 0.05. A sampled row that announced itself as
# floor-limited would be telling the reader the opposite of the truth.
check("sampled floor does not trip the floor>0.05 warning",
      mc.perm_floor(LO, HI) <= 0.05)

mc.PERM_MAX_EXACT, mc.PERM_SAMPLES = _old_max, _old_n
fresh()

# --------------------------------------------------------------------------
# 4. the sampler itself, and mutations that must be caught
# --------------------------------------------------------------------------
# Uniformity is checked per subset dealt to the left group, each at 1/C(n,nx), a
# finer test than the diff distribution. It calls mc.perm_split itself, not a
# copy of its loop. (notes: calib-subset-uniformity)
print("\n=== the sampler deals uniform subsets ===")
N_UNIF = 200_000
EXP = 1 / math.comb(4, 2)
TOL = 0.01                      # ~19 sigma at this M; the mutations move it 8x


def subset_freq(rng_factory, pool=(1, 2, 3, 4), nx=2, m=N_UNIF):
    rng = rng_factory(mc.PERM_SEED)
    seen = {}
    for _ in range(m):
        left, _r = mc.perm_split(list(pool), nx, rng)
        k = tuple(sorted(left))
        seen[k] = seen.get(k, 0) + 1
    return {k: v / m for k, v in sorted(seen.items())}


good = subset_freq(random.Random)
check("all C(4,2)=6 subsets are reachable", len(good) == 6, f"{len(good)}")
check("every subset appears at frequency 1/6",
      all(abs(v - EXP) < TOL for v in good.values()),
      f"maxdev {max(abs(v - EXP) for v in good.values()):.4f}")

# Mutation 1, off-by-one Fisher-Yates (j drawn from 0..n-1, not 0..i). The
# subset check must catch it; the diff-level check cannot. It shows only because
# perm_split copies the pool per draw. (notes: calib-mutation-off-by-one)
print("\n=== the check can fail: off-by-one Fisher-Yates ===")


class BadRandom(random.Random):
    def shuffle(self, x, random=None):
        n = len(x)
        for i in range(n - 1, 0, -1):
            j = self.randint(0, n - 1)          # should be randint(0, i)
            x[i], x[j] = x[j], x[i]


bad = subset_freq(BadRandom)
check("the off-by-one shuffle is caught by the subset check",
      any(abs(v - EXP) >= TOL for v in bad.values()),
      f"maxdev {max(abs(v - EXP) for v in bad.values()):.4f}")

# Mutation 2, drawing nx values independently: the null is built from
# non-relabellings. A wrong-support bug, unlike mutation 1's wrong weights, so
# the check is shown to catch both shapes.
# (notes: calib-mutation-with-replacement)
print("\n=== the check can fail: sampling with replacement ===")


def with_replacement(seed, pool=(1, 2, 3, 4), nx=2, m=N_UNIF):
    rng = random.Random(seed)
    seen = {}
    for _ in range(m):
        k = tuple(sorted(rng.choice(pool) for _ in range(nx)))
        seen[k] = seen.get(k, 0) + 1
    return {k: v / m for k, v in sorted(seen.items())}


wr = with_replacement(mc.PERM_SEED)
check("sampling with replacement is caught by the subset check",
      any(abs(wr.get(k, 0.0) - EXP) >= TOL for k in good),
      f"maxdev {max(abs(wr.get(k, 0.0) - EXP) for k in good):.4f}")

# --------------------------------------------------------------------------
# 5. the defect that prompted all of this
# --------------------------------------------------------------------------
# 30 v 30 is the standard campaign size, and it is the size at which the old
# code did not return: C(60,30) = 118264581564861424. The requirement is not
# that the answer be any particular value, it is that there BE one, from the
# shipped constants, in a time a person will wait.
print("\n=== a full-size campaign returns at all ===")
check("C(60,30) is above the enumeration threshold",
      math.comb(60, 30) > mc.PERM_MAX_EXACT, f"{math.comb(60, 30):.3g}")
A = [500.0 + 7 * i for i in range(30)]
B = [640.0 + 7 * i for i in range(30)]
fresh()
_, big_diffs, big_exact = mc.perm_all(A, B)
check("30v30 takes the sampling branch", big_exact is False)
check("30v30 draws the shipped number of samples",
      len(big_diffs) == mc.PERM_SAMPLES, f"{len(big_diffs)}")
big_p = mc.perm_p(A, B)
check("30v30 returns a p in (0,1]", 0 < big_p <= 1, f"p={mc.fmt_p(big_p)}")
# A shifted-by-140 pair of otherwise identical ladders is a large effect, so the
# sampled p must be at the resolution floor rather than merely small.
check("30v30 on a large effect is at the resolution floor",
      abs(big_p - 1 / (mc.PERM_SAMPLES + 1)) < 1e-12, f"p={mc.fmt_p(big_p)}")
# ...and the null case: two arms drawn from the same ladder must NOT be
# significant. A test that returns the floor for everything is not a test.
fresh()
null_p = mc.perm_p(A, list(A))
check("30v30 on identical data is p = 1.0", abs(null_p - 1.0) < 1e-12,
      f"p={mc.fmt_p(null_p)}")

# --------------------------------------------------------------------------
# 6. reproducibility
# --------------------------------------------------------------------------
# The seed is fixed precisely so that re-running the tool on unchanged cells
# cannot move the p-value. Checked by clearing the memo -- otherwise this would
# be testing the cache, not the seed.
print("\n=== the same data gives the same p ===")
fresh()
p1 = mc.perm_p(A, B)
fresh()
p2 = mc.perm_p(A, B)
check("p is reproducible across invocations", p1 == p2, f"{p1} vs {p2}")

# --------------------------------------------------------------------------
# 7. fmt_p
# --------------------------------------------------------------------------
print("\n=== the printed p-value ===")
check("an ordinary p keeps three decimals", mc.fmt_p(1 / 3) == "0.333")
check("p=1 prints as 1.000", mc.fmt_p(1.0) == "1.000")
check("0.001 stays decimal", mc.fmt_p(0.001) == "0.001")
check("1e-5 does not print as 0.000", mc.fmt_p(1e-5) == "1.0e-05")

print()
if FAILED:
    print(f"FAILED ({len(FAILED)}): " + ", ".join(FAILED))
    sys.exit(1)
print("ALL PASS")
