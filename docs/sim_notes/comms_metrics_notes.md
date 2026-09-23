# comms_metrics.py — design notes and history

The long comments of `sim/comms_metrics.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 1
- [load_run](#load_run) — 1
- [measure](#measure) — 5
- [perm_p](#perm_p) — 1
- [main](#main) — 5

## Module scope

### metrics-t-done-team

**Prefer t_done_team over makespan** — attached to `("t_done_team",       "t team done s",     "slower",     "PRIMARY; blank = censored"),` (line 137)

```text
The planner's own statement of when the team stopped trying. Prefer this
over makespan: makespan is the HARNESS's run-end, it is run-relative
rather than absolute sim, and it carries the done-grace drain plus one
poll period of slop -- windows whose length differs by arm, so ranking on
it partly ranks how long each arm idled after finishing. Blank when any
robot never declared; a censored run has no completion time and imputing
the horizon for it makes an unfinished run the fastest in its arm.
```

## load_run

### metrics-event-log-note

**Keeping the event log exclusion reason** — attached to `event_log_note = None` (line 228)

```text
WHY the exclusion reason is kept rather than dropped: a blank metric
column is the honest rendering of "censored", but it is a DISHONEST
rendering of "the event log would not parse". Those are different facts
and a bare `except: pass` made them identical -- a whole campaign's logs
could fail to load and every column would just quietly read blank, which
looks like censoring and would be reported as censoring.
```

## measure

### metrics-coverage-matched-effort

**Effort matched on coverage** — attached to `t_level = dist_level = None` (line 307)

```text
Effort MATCHED ON COVERAGE, not on time: when the better-informed robot
first reached `level`, how long had the run taken and how far had the team
driven? An earlier version divided team distance by the unknown reduction
since t=200 and it separated in the WRONG DIRECTION -- because by t=200 the
perfect-comms robots have already merged maps, so their denominator (the
room left to improve) is smaller before any robot has done extra work. Any
ratio anchored to a condition-dependent baseline measures the baseline.
Anchoring on a coverage level both conditions pass through removes it.
```

### metrics-primary-laggard-lag

**The primary endpoint: laggard lag** — attached to `crossings = []` (line 327)

```text
THE PRIMARY ENDPOINT: when did each robot reach the coverage criterion?

Degraded comms does not slow exploration down -- the LEADER's crossing time
barely moves between conditions. What it slows is exploration COMPLETION,
because the run cannot end until the second robot independently reaches the
criterion, and a robot that never received its partner's deltas has to go
and re-learn that ground by driving over it. So the quantity that carries
the effect is the gap between the two, not either one alone:

  t_lead_cross   first robot to the criterion    (near-invariant)
  t_team_cross   LAST robot to the criterion     (the run's real end)
  laggard_lag    the difference                  (the cost of the outage)

Measured over the whole run, not clipped to the matched horizon. The lag is
a within-run difference, so unequal run lengths do not bias it the way they
bias a level read at a fixed time -- but they DO censor it, which is
handled below.
```

### metrics-censored-lag

**Censored runs are the worst case** — attached to `censored = any(c is None for c in crossings)` (line 354)

```text
CENSORING, and it matters more than anything else in this file. A run whose
laggard never reached the criterion is the WORST case for the condition
under test, not a missing observation. Dropping it would bias the whole
comparison towards "no effect" exactly when the effect is largest. So the
lag is recorded as a lower bound (last logged time minus the leader's
crossing) and flagged, and any group containing one reports a median that
is itself a lower bound.
```

### metrics-lag-dist

**What the laggard drives during its lag** — attached to `lag_dist = None` (line 371)

```text
What the laggard actually DID with that time. It is not idling on the
radio: measured at 0.357-0.364 m/s against a 0.320-0.352 m/s whole-run
average, it drives at full speed the entire window. This is the cost in
robot-metres of knowledge that never arrived.
```

### metrics-unknown-at-dist

**Effort-matched knowledge is a bound** — attached to `u_at_dist = None` (line 386)

```text
EFFORT-MATCHED KNOWLEDGE, and the reason it exists. The coverage criterion
above is measured on each robot's OWN map, so it REWARDS REDUNDANT
COVERAGE: a robot cut off from its partner has cheap unknown right beside
it -- the ground the partner already covered -- and drives it down fast,
while a robot that already holds the union has only the hard, far-away
voxels left. Measured late in a dense run that is 820-1796 voxels per metre
against 126-138. So crossing times favour the degraded arm, and this metric
exists to charge for the driving: what does the team KNOW once every run
has spent the same robot-metres?

It is a BOUND, not a measurement, and the bound leans the other way.
min(u_A, u_B) is an upper bound on the union's unknown fraction (the union
contains each robot's map, so it can only know more). Under perfect comms
the two maps are identical and the bound is TIGHT -- it is the union. Under
degraded comms it is loose, and loose in the pessimistic direction: the
degraded team really knows at least this much and possibly more. So a
result where the DEGRADED arm still wins on this metric is conclusive,
while one where the perfect arm wins is suggestive and partly the bound.

The unbiased version needs the union map itself, which means bags. These
--record 0 runs cannot reconstruct it; a union-coverage re-run is the
outstanding fix.
```

## perm_p

### metrics-shared-perm-test

**Why the permutation test is imported** — attached to `def perm_p(xs, ys):` (line 456)

```text
The permutation test is modes_compare's, imported rather than reimplemented.

This file used to carry its own copy: a bare enumeration of C(nx+ny, nx) that
was fine on the 4-cell pilots it was written for and does not return at all on
a 30-per-arm campaign, where C(60,30) is 1.2e17. modes_compare had the same
defect and now samples above a threshold, with a calibration
(modes_compare_calib.py) pinning the sampler against exactly-known answers and
requiring two planted biases to be caught.

Sharing that implementation rather than porting it is the point. Two copies of
a statistical procedure in one repository is two procedures: they drift, only
one of them is under calibration, and the campaign is scored by whichever
script the operator happened to run. The calibration file names modes_compare
in its docstring, so a reader who finds this import knows where the tests are.
```

## main

### metrics-min-schema-flag

**Why the min-schema flag exists** — attached to `ap.add_argument("--min-schema", type=int, default=None,` (line 516)

```text
THE FLAG THE ERROR MESSAGE ALREADY TOLD PEOPLE TO PASS. load_run() calls
event_log.summarise_run(), and when that refuses a cell for being below
the schema floor it relays the reader's own text verbatim: "Pass
--min-schema 4 to read it deliberately." This script did not have a
--min-schema, so the instruction it printed was unactionable -- the
operator did exactly what the output said and got
`unrecognized arguments: --min-schema 4`. On every banked campaign below
the current schema (which today is every campaign but one), that meant
t_done_team / t_explore / t_mission came back blank with no way to
recover them from here.

Exposed rather than removed from the message, because the refusal is
right: pooling cells from an older binary generation is the thing the
floor exists to prevent, and the flag makes reading them a deliberate act
with a warning attached rather than a silent default.
```

### metrics-per-row-floor

**Permutation floors are per row** — attached to `floors = []` (line 642)

```text
Per row, because the groups are not the same size from row to row. A run
whose event log would not load has None for t_done_team and drops out of
those rows only; a censored run drops out of others. The footer used to
compute one floor from len(groups[...]) and print it as though it applied
to the whole table.
```

### metrics-perm-floor-enumerated

**Enumerated floor, not closed form** — attached to `floors.append((label, len(xs), len(ys), mc.perm_floor(xs, ys)))` (line 671)

```text
mc.perm_floor, not a closed form: 2/C(2n,n) is only right for EQUAL
groups, and these are routinely unequal by the time the per-metric
None-filtering above has run. It re-enters mc.perm_all, which is
memoised on (xs, ys), so this is the same enumeration the p came from
rather than a second one.
```

### metrics-event-log-unread-first

**Unread event logs print before censoring** — attached to `for label in (args.label_a, args.label_b):` (line 684)

```text
Printed BEFORE the censoring block on purpose. The t_done_team column is
documented as "blank = censored", and that documentation is only true for
runs whose event log actually loaded. Any run listed here has a blank for
a different reason, and reading it as censoring would overstate exactly
the quantity this file exists to compare.
```

### metrics-floor-footer

**Reporting the floors actually found** — attached to `if not floors:` (line 711)

```text
What the floors actually came out at, rather than the conclusion this
line used to print unconditionally. Three cases, and the old text was only
ever right about the first.
```
