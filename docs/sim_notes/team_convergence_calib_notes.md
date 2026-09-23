# team_convergence_calib.py — design notes and history

The long comments of `sim/team_convergence_calib.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [build](#build) — 1
- [Module scope](#module-scope) — 5

## build

### calib-link-trace-grid

**Synthetic link trace sampling** — attached to `for k in range(0, int(300.0 / link_period) + 1):` (line 86)

```text
One sample every `link_period` from 0 to 300, connected unless
inside an outage interval. The emulator writes at link_rate_hz;
the period does not matter to the reader, only the up/down
transitions do — but it bounds the NARROWEST outage a case can
express, and a sub-census-period outage is a distinct case.
```

## Module scope

### calib-late-heal-window

**Late heal case kept off the boundary** — attached to `a, b = healthy(heal_at=300.0)` (line 165)

```text
Healed, but long after the link came back — the merge is not the thing that
fixed it, and a generous window would call this a pass. The outage is short
so the 160 s gap is unambiguously outside the 120 s window: with the default
[100, 180) outage the last sample lands exactly ON the boundary, and a case
that straddles the comparison tests nothing but which way `>` rounds.
```

### calib-link-never-dropped

**The run whose link never dropped** — attached to `a, b = healthy()` (line 212)

```text
THE CASE THIS SUITE WAS MISSING. The first real smoke run scored by this
gate had the link connected on all 6493 samples — the robots never separated
enough to lose it — and the gate printed PASS: the per-outage loop never
executed, and the guard meant to catch "nothing was scored" was itself
written `if outages and scored == 0`, so an empty outage list skipped it.

Note what makes it nasty: the run does diverge. Two robots sampling a shared
fused map at slightly different sim times disagree on shared_hash from timing
skew alone, so the "must have diverged somewhere" requirement — the guard
specifically there to reject vacuous passes — was satisfied by something that
is not a dropout. Every ingredient of the pass was real except the dropout.
Hence `healthy()` here, unmodified: this fixture is a PASSING run in every
respect but the one that matters.
```

### calib-outage-down-at-teardown

**Outage still down at teardown** — attached to `a, b = healthy(diverge=(240.0, 1e9), heal_at=1e9)` (line 231)

```text
An outage still down when the trace ends has no heal to score. This case
found a real defect: the reader used to close such an outage at the last
link sample and the scorer relied on there being no later census sample to
notice, so a census tick landing on that same second scored the run for
failing to converge across a recovery that never happened. The verdict is
now driven by whether the link was OBSERVED to come back, not by sample
alignment — hence the deliberately coincident stamps here (the trace and the
census both end at 300).

rc is 1 rather than 0 because this run also has nothing else to score: the
only outage is unscoreable, which the gate reports separately below.
```

### calib-censored-heal-pair

**Censored heal and all-censored run** — attached to `a, b = healthy()` (line 247)

```text
A heal the run gave no room to observe. The second outage recovers on the
very last census sample, so there is no undisturbed link after it at all —
the pair cannot be shown to converge and cannot be shown to fail. The first
smoke run to reach this code failed two such outages at t=1517 and t=1522 in
a run that stopped at 1559, which says nothing about the exchange.

Both cases below share a fixture and differ ONLY in the outage list, because
the pair is the point: censoring must rescue a truncated heal WITHOUT
rescuing a run that has nothing else to show.
```

### calib-flicker-outage

**Flicker outage shorter than a census period** — attached to `print("\n=== an outage too short to observe anything ===")` (line 265)

```text
An outage shorter than one census period observes NOTHING: no paired sample
falls inside it. The scoring loop used to fall through such an outage to
`healed`, which picks the first agreeing sample after the end — for a pair
that had not diverged in that window, the very next tick — and recorded a
"converged 1 s after the link recovered" that no merge produced. Census
samples here are 20 s apart, so the flicker at [245, 247) contains none.
link_period=1.0 because the default 10 s link grid cannot express it.
```
