# modes_compare_calib.py — design notes and history

The long comments of `sim/modes_compare_calib.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [1. the exact path, against arithmetic done by hand](#1-the-exact-path-against-arithmetic-done-by-hand) — 2
- [2. the sampled path must reproduce an exactly-known answer](#2-the-sampled-path-must-reproduce-an-exactly-known-answer) — 1
- [3. the +1 correction](#3-the-1-correction) — 2
- [4. the sampler itself, and mutations that must be caught](#4-the-sampler-itself-and-mutations-that-must-be-caught) — 1
- [rather than a copy of its loop. A calibration that reimplements its subject](#rather-than-a-copy-of-its-loop-a-calibration-that-reimplements-its-subject) — 2

## 1. the exact path, against arithmetic done by hand

### calib-exact-2v2-by-hand

**The 2 v 2 case by hand** — attached to `print("=== the exact path, hand-computable ===")` (line 56)

```text
xs=[1,2] ys=[3,4]. Pool [1,2,3,4], nx=2, C(4,2)=6 arrangements, and the
|median difference| of each is short enough to write out:

  {1,2}|{3,4} -> |1.5-3.5| = 2      {1,3}|{2,4} -> |2.0-3.0| = 1
  {1,4}|{2,3} -> |2.5-2.5| = 0      {2,3}|{1,4} -> |2.5-2.5| = 0
  {2,4}|{1,3} -> |3.0-2.0| = 1      {3,4}|{1,2} -> |3.5-1.5| = 2

diffs = [2,1,0,0,1,2], obs = 2, so p = #{d>=2}/6 = 2/6 = 1/3, and the floor is
the same 2/6 because the observed labelling IS the most extreme one. That the
two coincide here is not an accident to paper over -- it is the smallest
honest illustration of the point the floor column makes: at n=2 v 2 the most
significant result the test can produce is p = 0.333.
```

### calib-unequal-groups-floor

**Unequal groups and the floor** — attached to `fresh()` (line 80)

```text
Unequal groups, the case the floor formula 2/C(2n,n) got wrong and the reason
perm_floor enumerates. xs=[1] ys=[2,3,4]: C(4,1)=4 arrangements.
  {1}|{2,3,4} -> |1-3| = 2     {2}|{1,3,4} -> |2-3| = 1
  {3}|{1,2,4} -> |3-2| = 1     {4}|{1,2,3} -> |4-2| = 2
obs = 2, p = 2/4 = 0.5, floor = 2/4 = 0.5. The closed form would have said
2/C(4,2) = 0.333, a floor BELOW the smallest p the test can return.
```

## 2. the sampled path must reproduce an exactly-known answer

### calib-sampled-path-vs-exact

**Sampled path against the exact answer** — attached to `print("\n=== the sampled path, against the exact answer ===")` (line 97)

```text
This is the case the whole file is for. Same data as above, but the threshold
is dropped so the 6-arrangement problem takes the SAMPLING branch. The exact
answer is 1/3 and is known independently; a sampled estimate of it must land
on 1/3 within Monte-Carlo error and not, say, on 1/2 because the sampler
favours some arrangements.

Tolerance: with M draws the estimate's standard error is
sqrt(p(1-p)/M) = sqrt((1/3)(2/3)/20000) = 0.0033, so 0.01 is 3 sigma. Tight
enough to catch the kind of bias that matters (the LCG case skewed p by far
more than that) and loose enough not to go red on a correct sampler.
```

## 3. the +1 correction

### calib-median-null-ties

**Ties in a median-based null** — attached to `print("\n=== the +1 correction ===")` (line 135)

```text
The first draft tried to produce it, with ten small values against ten large
ones, on the reasoning that the observed labelling is the uniquely most
extreme arrangement. It is not, and the case failed with 10 hits in 20000. The
statistic is a difference of MEDIANS, so an arrangement attains the maximum
whenever the two medians land where they started, and that survives swapping
any value that is not near a median: {0..8, 1000} against {9, 1001..1009}
has medians 4.5 and 1004.5, exactly like the observed split. Ties in a
median-based null are the rule, not the exception -- which is worth knowing on
its own, since it is the same fact that makes perm_floor's enumerated answer
larger than the naive 2/C(2n,n).
```

### calib-zero-hit-injection

**Injecting the zero-hit null** — attached to `_real_perm_all = mc.perm_all` (line 159)

```text
And the raw==0 case itself, by injecting the distribution rather than hoping
for it. perm_p is handed a null in which nothing reaches the observed value;
the exact branch must answer 0/M = 0 (correct -- an exhaustive enumeration
that finds nothing that extreme has genuinely found nothing) and the sampled
branch must answer 1/(M+1) rather than 0, because a sample that missed it has
not established the same thing.
```

## 4. the sampler itself, and mutations that must be caught

### calib-subset-uniformity

**Uniformity checked per subset** — attached to `print("\n=== the sampler deals uniform subsets ===")` (line 191)

```text
Checked at the level of WHICH SUBSET lands in the left group, because that is
what a relabelling is; the diff distribution in section 2 is a projection of
it and a coarse one (six subsets collapse onto three values), so it is the
weaker instrument. Every subset must appear with frequency 1/C(n,nx).

This section tests mc.perm_split -- the function perm_all actually draws from
-- rather than a copy of its loop. A calibration that reimplements its subject
measures the difference between two pieces of code.
```

## rather than a copy of its loop. A calibration that reimplements its subject

### calib-mutation-off-by-one

**Mutation 1: off-by-one Fisher-Yates** — attached to `print("\n=== the check can fail: off-by-one Fisher-Yates ===")` (line 221)

```text
MUTATION 1 -- the textbook off-by-one Fisher-Yates. `randint(0, n-1)` where
`randint(0, i)` belongs: a bug people really write, not obviously wrong by
inspection, and biased by a factor small enough that the coarse diff-level
check of section 2 cannot see it (measured deviation 0.011 against a 0.01
tolerance -- a coin toss). At subset level it is unmissable.

This mutation is also why perm_split copies the pool per draw. Against the
in-place version this same generator deviates by 0.0016, i.e. not at all: the
composition of many biased shuffles is a random walk and random walks
converge to uniform. The check below would have printed PASS forever.
```

### calib-mutation-with-replacement

**Mutation 2: sampling with replacement** — attached to `print("\n=== the check can fail: sampling with replacement ===")` (line 247)

```text
MUTATION 2 -- draw nx values independently instead of permuting. A repeated
draw puts one value in the left group twice, so the null is assembled from
arrangements that are not relabellings of the data at all. Different failure
mode from mutation 1 (wrong support, not merely wrong weights), included
because a check that only ever catches one shape of bug is only known to
catch that shape.
```
