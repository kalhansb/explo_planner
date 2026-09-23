# modes_compare.py — design notes and history

The long comments of `sim/modes_compare.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 2
- [gap_trace](#gap_trace) — 1
- [measure](#measure) — 2
- [Module scope (part 2)](#module-scope-part-2) — 3
- [ladder](#ladder) — 2
- [main](#main) — 11

## Module scope

### modes-cell-name-regex

**Parsing arm and seed from cell names** — attached to `RE_CELL = re.compile(r"_(.+)_seed(\d+)$")` (line 151)

```text
Cell-directory name -> (arm, seed). The arm token is `.+`, not `[A-Za-z]+`,
because the harness itself mandates arm names the letters-only pattern cannot
read: run_campaign.sh requires the post-latch coast to be carried in the arm
suffix (e.g. `hybrid_seek`), and those cells exist on disk (ds1_hybrid_seek_
seed1..4). Under the old pattern they parsed as arm="seek" — an arm nobody
ran — while their manifests said reconnect_mode_requested=hybrid, so they were
split out of "hybrid" under a fabricated label. Two arms sharing a final token
would have pooled silently, which is the same failure with no visible tell.

`.+` is greedy but anchored by `_seed<digits>$` and starts from the leftmost
`_`, so `g8r1_hybrid_seed101` still yields ("hybrid", 101) and
`ds1_hybrid_seek_seed1` yields ("hybrid_seek", 1).
```

### modes-manoeuvre-states

**Which planner states count as manoeuvre** — attached to `MANOEUVRE_STATES = {"PURSUE", "RETURN_NAV", "RETURN_SYNC"}` (line 165)

```text
The planner's actual manoeuvre states. Verified against the CSVs, not guessed:
the full state vocabulary is {WAIT_FOR_MAP, NAVIGATE, PLAN, INTEGRATE,
LOG_STEP, DONE, PURSUE, RETURN_NAV, RETURN_SYNC, PROXIMITY_HOLD, and — since
mission return, 2026-08-27 — RETURN_HOME}. RETURN_HOME is deliberately NOT in
the set below: it is the arm-invariant drive home, present in the control arm
too, and its rows carry reconnect_elapsed_sec = -1 (transitionTo closes any
live manoeuvre on entering it), so neither clause of `cur` may claim it.

This replaces a regex `recon|pursu|rendez`, which was wrong in the worst
possible direction. It matched PURSUE but NOT RETURN_NAV or RETURN_SYNC -- the
states rendezvous and hybrid spend their manoeuvre in. p3b_rendezvous_seed1's
bestla has 4 RETURN_NAV rows and 0 PURSUE, so the regex scored the rendezvous
arm fire=0 while its manoeuvre was demonstrably firing, and this file's own
closing warning would then have condemned a working arm as a relabelled
control. A mechanism check that reports "mechanism absent" when the mechanism
ran is worse than no check.
```

## gap_trace

### modes-gap-end-none

**Unmeasurable map gap stays None** — attached to `end = g(t_end)` (line 254)

```text
`or 0.0` here used to turn an UNMEASURABLE gap into a reported 0.00 %, i.e.
into perfect agreement -- the most flattering possible reading of missing
data. None propagates instead.
```

## measure

### modes-pairwise-csv-count

**Why measure() requires exactly two CSVs** — attached to `return dict(excluded=(` (line 506)

```text
Previously a silent `return None`. A run that produced no CSV is an
infrastructure failure exactly like a dead sim, and dropping it without
a word while scoring its half-written sibling as censored gave two
identical failures opposite treatment.

2 is NOT a missing generalisation here, and the message says which of
the two cases it is so a reader of an N>=3 campaign does not go looking
for lost logs. Most of what measure() computes would generalise by
index (t_team is a max, t_lead a min, dist_team a sum), but gap_trace
is genuinely pairwise -- inter-robot map divergence between a and b --
and at N>=3 it has no single value: worst pair, mean pair and
first-vs-rest are three different endpoints with three different
answers. Choosing one silently inside a ranking tool is how an
endpoint gets redefined without anyone deciding to redefine it.
event_log.py reads every team size and is the pre-registered reader;
use it for N>=3 completion times.
```

### modes-dist-team-at-t-team

**Distance read at t_team** — attached to `dist_team=((val_at(a, 2, t_team) or a[-1][2] or 0.0)` (line 591)

```text
Read at t_team, not at end-of-run. End-of-run includes the DONE grace
drain and the teardown tail, and those windows differ by arm, so the
cost column was partly measuring how long each arm idled after
finishing. Censored runs have no t_team, so they fall back to
end-of-run -- correct there, since the run never completed.
```

## Module scope (part 2)

### modes-perm-max-exact

**Where the permutation test starts sampling** — attached to `PERM_MAX_EXACT = 200_000` (line 607)

```text
Above this many relabellings the test SAMPLES instead of enumerating. The
value is not a performance tuning knob, it is the point past which the exact
test is not a thing that can be run: a 30-vs-30 campaign -- the standard size
since the power analysis put the completion-time floor at 30 cells per arm --
has C(60,30) = 118264581564861424 arrangements, and itertools.combinations
will sit in that loop until the box is retired. The old code called it
unconditionally, so the fix this replaces was not "slow", it was "modes_compare
does not return on a full-size campaign".

200000 is chosen so the largest EQUAL groups that still enumerate are 10 v 10
(C(20,10) = 184756); 11 v 11 is 705432 and samples. That keeps every small,
floor-limited comparison -- the ones where the floor line is load-bearing --
on the exact path, which is where it has to be, because a floor is a statement
about the enumeration and a sample cannot make it.
```

### modes-perm-seed-fixed

**Fixed sampling seed for reproducibility** — attached to `PERM_SEED = 20260830` (line 626)

```text
Fixed, and NOT exposed as a flag. A permutation p-value computed from a random
subset is a random variable; leaving the stream unseeded would make the same
campaign print a different p every invocation and there would be no way to
tell that drift from a data change. Seeded, re-running the tool on the same
cells reproduces the same number exactly.
```

### modes-perm-cache

**Memoising the permutation distribution** — attached to `_PERM_CACHE = {}` (line 633)

```text
perm_p and perm_floor are called back-to-back on the same two lists for every
printed row (see the table loop), and under sampling each call is 100k median
pairs. Memoised so the row costs one distribution, not two. Keyed on the
values, so two rows that happen to hold identical data share the answer, which
is correct: the distribution is a function of the data alone.
```

## ladder

### modes-ladder-unparsed-cells

**Ladder names unparseable cells** — attached to `unmatched.add(os.path.basename(os.path.normpath(d)))` (line 899)

```text
Named, not swallowed — the same fix main() already carries.
A bare `continue` here dropped unreadable cells with no
message, so the sensitivity ladder could disagree with the
main table purely because it silently saw fewer runs, and
the rank order it prints is exactly what that would corrupt.
```

### modes-ladder-near-vs-early-flips

**Two kinds of ordering flip** — attached to `rows_ranked = [(th, o) for (th, _), o in zip(rows_out, orders)` (line 953)

```text
Two very different things can move the ordering down the ladder, and
collapsing them into one "UNSTABLE" verdict throws away the finding.

  NEAR the completion criterion, a flip means the endpoint is noise: those
  thresholds are separated by a few percent of coverage and should not
  reorder the arms.

  BETWEEN early and late thresholds, a flip is a RESULT. A reconnect
  manoeuvre spends time it does not spend exploring, so an arm can be
  behind at 0.70 and ahead at 0.55: the manoeuvre costs time early and
  repays it near completion. That is a claim about when the policy earns
  its keep, not a defect in the ranking at completion.
```

## main

### modes-threshold-matches-done-rule

**Threshold must match planner done rule** — attached to `ap.add_argument("--threshold", type=float, default=0.64,` (line 999)

```text
0.64, not the 0.55 this defaulted to through generation 7.

The threshold has to match the planner's OWN done rule, because t_team is
the CSV crossing of it: generation 8 latches and stops at unknown <= 0.64,
so a run measured at 0.55 is asked when it crossed a level it was never
driven to. It usually never crosses, the cell scores `censored`, and
t_team is withheld -- a silent, near-total loss of the endpoint rather
than a wrong number. The ladder brackets 0.64 on both sides for the same
reason it always did: an ordering that exists only at the endpoint value
is a property of the threshold, not of the arms.
```

### modes-main-unparsed-cells

**Main table names unparseable cells** — attached to `unmatched.append(os.path.basename(os.path.normpath(d)))` (line 1032)

```text
Named, not swallowed. A bare `continue` here dropped cells BEFORE
the `dropped` bookkeeping below, so a directory the regex could
not read vanished from the comparison with no message at all —
indistinguishable from a campaign that never ran it.
```

### modes-empty-median-lists

**Empty lag and unk lists** — attached to `def m(xs, w, p):` (line 1163)

```text
`lag` and `unk` are the two lists here that can come out EMPTY -- lag
is None on a censored run and unk_floor is None when neither robot's
last row carried an unknown_fraction -- and st.median([]) raises.
An arm in which every run censored is not an exotic input: it is what
a short smoke campaign looks like, which is precisely the run this
tool gets pointed at first. It used to abort mid-row with a
StatisticsError, taking down the coverage guard, the power block, the
mechanism window and the whole delta table with it -- a crash where
"--" was the answer.
```

### modes-build-hash-check

**Differing build hashes are not proof** — attached to `print(f"\n!! ARMS RECORD DIFFERENT COMMITS. This is confounded with the "` (line 1192)

```text
A differing hash is necessary but NOT sufficient evidence of
confounding: this repo carries the analysis scripts alongside the
planner, so committing a change to sim/*.py mid-campaign moves the
recorded hash without touching the binary that ran. That happened
during Phase 7 and would have discarded a perfectly good matrix.
Name the check rather than the verdict.
```

### modes-coverage-guard

**The coverage guard's two checks** — attached to `unheld = [(arm, r["seed"], r["unk_floor"])` (line 1212)

```text
THE COVERAGE GUARD. The legend has described `unk_floor` as a guard --
"an arm is not faster if it stopped at less coverage" -- since the column
was added, and until 2026-09-18 the entire consumption of that column was
one median printed in the table above. Nothing compared it across arms,
nothing flagged anything, nothing could fail. The sentence described a
check that did not exist, which is the failure mode this whole review
pass is about, and it described it on the one threat that can reverse the
headline of a completion-time campaign.

Two questions, and only the second is a comparison.

  (1) Did every run CREDITED with a t_team still hold that coverage when
      it stopped? t_team is the first crossing of unknown <= thresh and
      the robot keeps mapping afterwards, so a non-censored run has to
      end at or below thresh. One that ends above it did not hold the
      crossing, and it sits in the median as a completion anyway.

  (2) Does an arm's speed advantage come with less of the map? This is
      computed over the runs that actually ENTER the delta -- non-censored
      only -- and NOT over the median printed above. That median pools
      censored runs, whose unknown is above thresh by definition, so
      comparing it across arms would re-report every censoring-heavy arm
      as a coverage problem when censoring already has its own guard two
      screens down. Same column, different denominator, different claim.
```

### modes-coverage-band-from-data

**Coverage band taken from the data** — attached to `spreads = [st.pstdev(v) for v in cov.values() if len(v) >= 2]` (line 1262)

```text
The margin is taken FROM THE DATA rather than picked. A between-arm
difference only means something once it clears the spread the same
measurement shows WITHIN an arm, and that spread is a property of this
campaign's forest and threshold, not a number that can be carried in
from another one. Arms with a single non-censored run contribute no
spread and are still compared -- they just cannot widen the band.
```

### modes-mechanism-window

**What the mechanism window measures** — attached to `all_rs = [r for rs in arms.values() for r in rs]` (line 1323)

```text
How much of the endpoint the treatment can physically reach. The manoeuvre
only triggers once a robot has FINISHED exploring (finishOrRendezvous, the
sole entry point, is reached only on step-budget / coverage-saturated), so
it can act inside `lag` and nowhere else. Everything before t_lead is
untreatable by construction, and it is the bulk of the run.
```

### modes-mechanism-window-midrun

**Mid-run dispatches void the terminal claim** — attached to `n_mid = sum(r.get("midrun", 0) or 0 for r in all_rs)` (line 1333)

```text
The claim below holds ONLY where every manoeuvre is terminal. A
generation-8 campaign has a mid-run trigger that fires during
exploration, so print the caveat rather than the conclusion when the
cells show mid-run dispatches — otherwise this block tells the reader
the treatment cannot reach ~90 % of the endpoint, which is false.
```

### modes-arm-column-width

**Arm column width follows arm names** — attached to `print(f"\nvs control '{ctl}'   (negative delta = FASTER completion = better)")` (line 1359)

```text
Widened to the longest arm token actually present rather than pinned at
12. Arms grew a per-cell suffix (mtare_rendezvous_r40 is 20 characters),
which ran the arm column straight into the metric column and printed
`mtare_hybrid_r40t_team` — two fields with no separator, in the table the
whole tool exists to produce.
```

### modes-withhold-censored-t-team

**Why censored t_team deltas are withheld** — attached to `if key == "tt" and ncens:` (line 1371)

```text
A t_team median built from the survivors of censoring is not a
conservative estimate, it is a WRONG-SIGNED one. Worked example:
off = {1500,1600,1700,1800} (median 1650) versus an arm scoring
{1400,1450,>5800,>5800} (true median >=3625, i.e. ~2000 s WORSE).
Dropping the two censored runs leaves {1400,1450}, median 1425, and
the table reports the arm 225 s FASTER and stamps it CLEAN --
because the runs deleted are exactly the ones that would have
overlapped. The separation is manufactured by the exclusion. A
footnote cannot repair a reversed headline number, so the number is
withheld instead.
```

### modes-mark-sampled-rows

**Marking sampled rows in the table** — attached to `note = (note + "; " if note else "") + \` (line 1406)

```text
Say so IN the row. Which rows were enumerated and which were
sampled depends on the group sizes, so within one table some
are exact and some are not, and the two mean different things
in the floor column -- an unmarked table would invite reading
a sampling artefact as a property of the design.
```
