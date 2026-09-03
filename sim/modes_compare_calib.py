#!/usr/bin/python3
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
# xs=[1,2] ys=[3,4]. Pool [1,2,3,4], nx=2, C(4,2)=6 arrangements, and the
# |median difference| of each is short enough to write out:
#
#   {1,2}|{3,4} -> |1.5-3.5| = 2      {1,3}|{2,4} -> |2.0-3.0| = 1
#   {1,4}|{2,3} -> |2.5-2.5| = 0      {2,3}|{1,4} -> |2.5-2.5| = 0
#   {2,4}|{1,3} -> |3.0-2.0| = 1      {3,4}|{1,2} -> |3.5-1.5| = 2
#
# diffs = [2,1,0,0,1,2], obs = 2, so p = #{d>=2}/6 = 2/6 = 1/3, and the floor is
# the same 2/6 because the observed labelling IS the most extreme one. That the
# two coincide here is not an accident to paper over -- it is the smallest
# honest illustration of the point the floor column makes: at n=2 v 2 the most
# significant result the test can produce is p = 0.333.
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

# Unequal groups, the case the floor formula 2/C(2n,n) got wrong and the reason
# perm_floor enumerates. xs=[1] ys=[2,3,4]: C(4,1)=4 arrangements.
#   {1}|{2,3,4} -> |1-3| = 2     {2}|{1,3,4} -> |2-3| = 1
#   {3}|{1,2,4} -> |3-2| = 1     {4}|{1,2,3} -> |4-2| = 2
# obs = 2, p = 2/4 = 0.5, floor = 2/4 = 0.5. The closed form would have said
# 2/C(4,2) = 0.333, a floor BELOW the smallest p the test can return.
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
# This is the case the whole file is for. Same data as above, but the threshold
# is dropped so the 6-arrangement problem takes the SAMPLING branch. The exact
# answer is 1/3 and is known independently; a sampled estimate of it must land
# on 1/3 within Monte-Carlo error and not, say, on 1/2 because the sampler
# favours some arrangements.
#
# Tolerance: with M draws the estimate's standard error is
# sqrt(p(1-p)/M) = sqrt((1/3)(2/3)/20000) = 0.0033, so 0.01 is 3 sigma. Tight
# enough to catch the kind of bias that matters (the LCG case skewed p by far
# more than that) and loose enough not to go red on a correct sampler.
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
# The first draft tried to produce it, with ten small values against ten large
# ones, on the reasoning that the observed labelling is the uniquely most
# extreme arrangement. It is not, and the case failed with 10 hits in 20000. The
# statistic is a difference of MEDIANS, so an arrangement attains the maximum
# whenever the two medians land where they started, and that survives swapping
# any value that is not near a median: {0..8, 1000} against {9, 1001..1009}
# has medians 4.5 and 1004.5, exactly like the observed split. Ties in a
# median-based null are the rule, not the exception -- which is worth knowing on
# its own, since it is the same fact that makes perm_floor's enumerated answer
# larger than the naive 2/C(2n,n).
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

# And the raw==0 case itself, by injecting the distribution rather than hoping
# for it. perm_p is handed a null in which nothing reaches the observed value;
# the exact branch must answer 0/M = 0 (correct -- an exhaustive enumeration
# that finds nothing that extreme has genuinely found nothing) and the sampled
# branch must answer 1/(M+1) rather than 0, because a sample that missed it has
# not established the same thing.
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
# Checked at the level of WHICH SUBSET lands in the left group, because that is
# what a relabelling is; the diff distribution in section 2 is a projection of
# it and a coarse one (six subsets collapse onto three values), so it is the
# weaker instrument. Every subset must appear with frequency 1/C(n,nx).
#
# This section tests mc.perm_split -- the function perm_all actually draws from
# -- rather than a copy of its loop. A calibration that reimplements its subject
# measures the difference between two pieces of code.
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

# MUTATION 1 -- the textbook off-by-one Fisher-Yates. `randint(0, n-1)` where
# `randint(0, i)` belongs: a bug people really write, not obviously wrong by
# inspection, and biased by a factor small enough that the coarse diff-level
# check of section 2 cannot see it (measured deviation 0.011 against a 0.01
# tolerance -- a coin toss). At subset level it is unmissable.
#
# This mutation is also why perm_split copies the pool per draw. Against the
# in-place version this same generator deviates by 0.0016, i.e. not at all: the
# composition of many biased shuffles is a random walk and random walks
# converge to uniform. The check below would have printed PASS forever.
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

# MUTATION 2 -- draw nx values independently instead of permuting. A repeated
# draw puts one value in the left group twice, so the null is assembled from
# arrangements that are not relabellings of the data at all. Different failure
# mode from mutation 1 (wrong support, not merely wrong weights), included
# because a check that only ever catches one shape of bug is only known to
# catch that shape.
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
