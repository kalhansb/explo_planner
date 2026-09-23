# test_experiment_log.cpp — design notes and history

The long comments of `test/test_experiment_log.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [File scope](#file-scope) — 1
- [TempLogPath — declarations](#templogpath--declarations) — 1
- [readNum](#readnum) — 1
- [emitWatchdogRow](#emitwatchdogrow) — 1
- [ExperimentLogHomeWatchdog.ApproachFirePreservesNegativeDelta](#experimentloghomewatchdogapproachfirepreservesnegativedelta) — 1
- [ExperimentLogHomeWatchdog.FrozenInEscapeIsAFireAndDoesNotAliasEscapeEnd](#experimentloghomewatchdogfrozeninescapeisafireanddoesnotaliasescapeend) — 1
- [ExperimentLogHomeWatchdog.DefaultedCallStillEmitsBothFields](#experimentloghomewatchdogdefaultedcallstillemitsbothfields) — 1
- [ExperimentLogCellCensus.CarriesBothCoverageMeasuresOnOneRow](#experimentlogcellcensuscarriesbothcoveragemeasuresononerow) — 1
- [TEST](#test) — 2
- [ExperimentLogAppointmentLeg.CarriesBothConditionalColumnsOnEveryKind](#experimentlogappointmentlegcarriesbothconditionalcolumnsoneverykind) — 1
- [ExperimentLogSchema.DeclaredKindsMatchTheWriter](#experimentlogschemadeclaredkindsmatchthewriter) — 1

## File scope

### watchdog-tests-header-history

**Why the home_watchdog row is tested** — attached to `#include <gtest/gtest.h>` (line 3)

```text
These exist because of a generation-7 defect that no test could have caught,
since no test covered this writer at all: the fire rows recorded `metric_m`
(an INSTANTANEOUS remaining distance) while the quantity the detector
actually compared — the movement over the window — was recorded nowhere, and
the field comment described metric_m as movement "over window_sec".

The banked evidence, re-measured rather than recalled: 7 non-escape-end
home_watchdog rows exist across the g6pilot cells (6 in hybrid_seed103's
bestla, 1 in off_seed102's atlas), all of kind `approach`; no frozen fire was
ever banked. In 4 of the 7 metric_m and dist_home_m disagree (8.06 vs 4.00,
and so on), which is the only reason the conflation was visible at all. The
tested delta itself appears in NEITHER the jsonl NOR the plaintext of any
banked cell — grep finds zero lines carrying it — so any statement about how
far it sat from metric_m is a RECONSTRUCTION from the CSV pose track, not a
reading. That unrecoverability is the finding: on the documented contract a
scorer would have called every fire spurious, and had no logged quantity to
check that against.

That classification is load-bearing: these rows are the only basis for
deciding whether a `home-gave-up` park is a genuine stall or a detector
artefact, and that decision feeds censoring in the mission-completion
analysis. So the fired inequality is asserted here, at the byte level of the
emitted JSONL, rather than trusted.
```

## TempLogPath — declarations

### explog-temp-log-path

**Per-test per-process log path** — attached to `struct TempLogPath {` (line 46)

```text
Writes to a per-test, per-process path under /tmp and removes it on
destruction. Not a fixture member so each test names its own file — a
shared path across tests would let one test's truncation race another's.
The pid is in the name for the same reason one level up: `colcon test`
runs test executables in parallel, and two concurrent runs of this binary
sharing /tmp/explo_test_frozen.jsonl would interleave their writes and
fail intermittently, which is the worst way for a contract test to fail.
```

## readNum

### explog-readnum

**Reading numbers back from the row** — attached to `bool readNum(const std::string& row, const char* name, double* out) {` (line 61)

```text
Reads one `"name":<number>` field back out of a JSONL row.

Exists so the inequality assertions below are made against what the WRITER
emitted, not against the literals the test passed in. Asserting
`EXPECT_LT(-0.03, 1.0)` on two constants is a tautology that holds for
every possible writer, including one that emits nothing at all.
```

## emitWatchdogRow

### explog-emit-watchdog-row

**Emitting one home_watchdog row** — attached to `std::string emitWatchdogRow(const std::string& path, const char* kind,` (line 74)

```text
Emits one home_watchdog row and returns it verbatim.

Deliberately returns the RAW LINE rather than a parsed structure: the defect
being guarded against is a field that is present-but-wrong or absent, and
both of those survive a lenient parser. Substring assertions on the emitted
bytes are what the analysis scripts actually see.
```

## ExperimentLogHomeWatchdog.ApproachFirePreservesNegativeDelta

### watchdog-negative-delta

**Approach fire keeps a negative delta** — attached to `TEST(ExperimentLogHomeWatchdog, ApproachFirePreservesNegativeDelta) {` (line 128)

```text
The sign case. metric_m 3.69 is a real banked row (g6pilot_hybrid_seed103,
bestla); the -0.03 beside it is reconstructed from that run's pose track, not
logged — the point being that a receding robot and an advancing one produced
indistinguishable rows. The field must therefore be able to disagree with
metric_m in DIRECTION, not just in magnitude. If a future change routes this
through a magnitude or clamps at zero, this fails.
```

## ExperimentLogHomeWatchdog.FrozenInEscapeIsAFireAndDoesNotAliasEscapeEnd

### watchdog-frozen-in-escape

**Frozen-in-escape fire and its scope** — attached to `TEST(ExperimentLogHomeWatchdog, FrozenInEscapeIsAFireAndDoesNotAliasEscapeEnd) {` (line 171)

```text
The abort-an-escape fire. This kind exists because a frozen detector firing
DURING an escape leg used to be recorded only by the escape-end row that
followed it, and escape-end is the writer's omit-case — so the one fire on
that path was written down carrying no inequality at all.

SCOPE, because two of the assertions below are weaker than they look.
emitWatchdogRow() calls logHomeWatchdog() with the test's own literals, so
this file can only pin the WRITER's round-trip: given this kind and this
window, does the row come out carrying them, and does the omit-case leave
the inequality alone. It cannot see what homeWatchdogFire actually passes.

So the `window_sec` and the not-`escape-frozen` assertions would keep
passing if the production call site regressed to the leg duration or to the
aliased token. That is the whole failure they are named after, and it is
covered at scoring time — by gate_g8.py, over the emitted cells — not here.
Do not read a green run of this file as proof the call site is right.

Why the name matters at all: `escape-frozen` is the `response` on the
escape-end row this abort emits immediately afterwards, so if the kind ever
becomes that same token, one abort puts the string in two columns of two
consecutive rows and a reader grepping the token rather than the column
counts one abort as two.
```

## ExperimentLogHomeWatchdog.DefaultedCallStillEmitsBothFields

### watchdog-defaulted-call

**Defaulted watchdog call history** — attached to `TEST(ExperimentLogHomeWatchdog, DefaultedCallStillEmitsBothFields) {` (line 216)

```text
Guards the writer against a silent regression to the generation-7 default
arguments, where a caller that passed nothing still produced a row that
LOOKED complete (0.0 / 0.0) and read as a frozen fire at a zero threshold —
an inequality that is false for every possible delta.
```

## ExperimentLogCellCensus.CarriesBothCoverageMeasuresOnOneRow

### census-coverage-pairing

**Both coverage measures on one row** — attached to `TEST(ExperimentLogCellCensus, CarriesBothCoverageMeasuresOnOneRow) {` (line 247)

```text
P1's gate is read off this row and nothing else: it asks whether the coarse
census converges to COVERED as the ROI's continuous unknown fraction falls.
Both numbers therefore have to be ON THE SAME ROW — the sim is nondetermin-
istic enough run-to-run that joining `cell_census` to `coverage_milestone`
on a timestamp would be comparing two ticks and reporting the difference as
a disagreement between the measures. If a future edit moves either number
off this event, the gate silently degrades into that join, so the pairing is
asserted at the byte level rather than assumed.
```

## TEST

### census-covered-reachability

**Reachability of the COVERED threshold** — attached to `ASSERT_TRUE(readNum(row, "cells_measured", &v)) << row;` (line 329)

```text
Reachability of the COVERED threshold. Without these, a census reporting
zero COVERED cells is indistinguishable from a census that is broken, and
P1's first run produced exactly that row: an entire run to the completion
criterion with not one promotion and nothing to say why. They are checked
by VALUE, not just presence — a distribution wired to the wrong cells (the
whole grid rather than the measured ones) still writes all five fields,
and reads as a world where nothing is mappable.
```

### census-frontier-joint-reading

**Frontier veto joint reading** — attached to `ASSERT_TRUE(readNum(row, "cell_frontier_frac_min", &v)) << row;` (line 347)

```text
The frontier veto's own distribution. `_at_best_unknown` is the joint
reading and the only one that can answer whether the two thresholds are
simultaneously satisfiable, so it is pinned to a value DISTINCT from both
marginals — wiring it to either of them would otherwise pass this test
while silently answering a different question.
```

## ExperimentLogAppointmentLeg.CarriesBothConditionalColumnsOnEveryKind

### apptleg-conditional-columns

**Conditional columns on every kind** — attached to `TEST(ExperimentLogAppointmentLeg, CarriesBothConditionalColumnsOnEveryKind) {` (line 361)

```text
Two of this row's columns belong to one kind each — `leg_sec` to escape-end,
`rolled_to_sec` to unreached — and the writer carries both on every row at
-1.0 rather than omitting them off their own kind. That choice is what the
reader depends on, and it is invisible in the data if it breaks: an omitted
key and a sentinel one look identical to any `.get(k, -1)` reader, so the
give-up that HAD no appointment left to roll and the one whose roll was never
written would fold into the same count. Asserted at the byte level, on the
kind that owns NEITHER column, because that is the row where a writer that
omitted them would still look right on both of the others.
```

## ExperimentLogSchema.DeclaredKindsMatchTheWriter

### schema-declared-event-kinds

**Declared event kinds match the writer** — attached to `TEST(ExperimentLogSchema, DeclaredKindsMatchTheWriter) {` (line 419)

```text
kEventKinds is what sim/equiv_gate.py scores "did a new event kind appear at
defaults?" against, and a declared universe that has drifted from the writer
answers that question wrongly in the silent direction: a kind missing from
the list reads as "new" on every run that emits it (noise, which gets the
gate loosened), and — worse — the gate's notion of which kinds are opt-in
comes from the list's tail, so an unlisted v4 kind is scored as legacy and
its appearance at defaults never fails anything.

Nothing in C++ can enumerate the writer's calls, so the test reads the
writer's SOURCE, exactly as test_failed_goal_blacklist reads the shipped
YAML rather than a copy of its values. The path comes from CMake, so this
cannot silently pass by scanning a stale installed tree.
```
