# team_convergence.py — design notes and history

The long comments of `sim/team_convergence.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [precondition_stages](#precondition_stages) — 1
- [main](#main) — 3

## precondition_stages

### conv-no-dropout-refusal

**No dropout means refuse, not fail** — attached to `stage = []` (line 217)

```text
Read BEFORE anything is scored, because a run in which the link never
dropped is not a run this gate has an opinion about. The verdict sentence
is "diverged under a dropout and agreed again after it healed"; with no
dropout there is no such claim to make.

This is a REFUSAL (2), not a failure (1). The exchange may be working
perfectly — nothing in a permanently-connected run can tell — and
returning 1 would blame the planner for a scenario that never separated
the robots.

It is here because the first real run scored by this gate had exactly
this shape (connected in all 6493 link samples, drop_disconnected == 0 on
every poll) and an earlier version of this code printed PASS over it: the
per-outage loop simply never executed, `scored` stayed 0, the guard below
that catches that was itself written `if outages and scored == 0`, and an
empty fails list reads as success. Two robots sampling a SHARED fused map
at slightly different sim times disagree on shared_hash now and then from
timing skew alone, so even the "must have diverged somewhere" requirement
was satisfied — by something that is not a dropout. Every ingredient of
the pass was real except the dropout.
```

## main

### conv-heal-window-censoring

**Heal window and censored outages** — attached to `last_census_t = agree[-1][0]` (line 358)

```text
For each outage: were they apart at any point from its start onward, and
did they agree again within --heal-window of the link coming back?
How long the pair had an undisturbed link after each heal: from the
recovery until the next outage begins, or the census trace ends.

Convergence is still LOOKED FOR over the whole rest of the run — evidence
the merge worked is evidence wherever it lands — but a FAILURE may only
be declared when this window was at least --heal-window long. Converging
late, or not at all, is a defect only if the run actually gave the pair
an undisturbed link long enough to do it in.

Without this, a run that stops during a burst of churn fails on its last
few outages every time: each heal is followed by a second or two of link
and then the next dropout, no census sample falls in the gap, and "never
agreed again for the rest of the run" is true while saying nothing about
the exchange. That is right-censoring — the same situation as the
still-down-at-teardown case above, truncated by the NEXT outage rather
than by the end of the trace.

The asymmetry is deliberate and it is the safe direction: a short window
can leave a heal unproven, never unrefuted. And a run whose outages are
ALL censored does not quietly pass — `scored` stays 0 and the guard below
fails it. That guard is what stops this rule becoming an excuse
generator, so it must not be weakened to accommodate a churny run.
```

### conv-empty-outage-skip

**Outages with no census sample inside** — attached to `if not during:` (line 400)

```text
`not during` is the empty-population case and it must skip, not
score. Was `if during and all(during)`, which meant an outage
containing NO census sample fell through to be scored: `healed` then
picked the first agreeing sample after `end`, which for a pair that
never diverged is the very next tick, and the outage was recorded as
"converged 0.9 s after the link recovered" — a heal no merge could
have produced, out of a window in which nothing was observed at all.

Not hypothetical, and not rare. cell_census_period_s defaults to 5.0
and read_outages applies no debounce, so every sub-5-second flicker
produces an outage with n_during == 0. In the p2 smoke pair that was
1 of the treatment's 2 scored heals and 3 of the control's 7 — both
of the numbers that comparison rests on were substantially this bug
rather than a measurement.

This is the same empty-reads-as-PASS failure the module docstring
claims to have eliminated; the earlier fix closed only the
zero-OUTAGES case and left the zero-observations case open one level
down. See [[checks-that-stopped-checking]].
```

### conv-verdict-wording

**Why the verdict carries count and caveat** — attached to `verdict = (f"the cell worlds diverged under a dropout and agreed again "` (line 484)

```text
The count is IN the verdict, not merely in the notes: "converged after a
dropout healed" over zero scored outages is the vacuous pass this gate
already emitted once, and a reader skimming for the PASS line would have
had no way to see it. A verdict that cannot state how many heals it
measured should not be phrased as though it measured one.

The attribution disclaimer is in the verdict for the same reason and at
the same cost: a TEAM_WORLD=0 control converged after 7 of 7 scoreable
heals (see the module docstring), so a PASS line reading only "the worlds
converged" would be read as "the exchange works" by every future reader
of comms_gates.txt -- including me, which is how this went unnoticed
through one full smoke run. The one place a caveat cannot be skipped is
the sentence the caveat is about.
```
