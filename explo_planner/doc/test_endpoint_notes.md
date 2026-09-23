# test_endpoint.cpp — design notes and history

The long comments of `test/test_endpoint.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [File scope](#file-scope) — 1
- [readLines](#readlines) — 1
- [functionBody](#functionbody) — 1
- [stripComments](#stripcomments) — 1
- [EndpointOrdering.ExplorationThenMissionThenRunEnd](#endpointorderingexplorationthenmissionthenrunend) — 1
- [MissionCompleteRow.ResultAndReasonAreDistinctFields](#missioncompleterowresultandreasonaredistinctfields) — 1
- [MissionCompleteRow.DuplicateIsSuppressedAndCounted](#missioncompleterowduplicateissuppressedandcounted) — 1
- [MissionCompleteRow.OccurrenceIsWrittenNotAssumed](#missioncompleterowoccurrenceiswrittennotassumed) — 1
- [ExplorationCompleteRow.SecondDeclarationAdvancesLastAndLatchesFirst](#explorationcompleterowseconddeclarationadvanceslastandlatchesfirst) — 1
- [GROUP B. THE ENDPOINT ROWS.](#group-b-the-endpoint-rows) — 1
- [CoverageMilestonePostLatch.AStepAfterTheDeclarationClearsTheFlag](#coveragemilestonepostlatchastepafterthedeclarationclearstheflag) — 1
- [CoverageMilestonePostLatch.ASecondDeclarationReArmsFromTheLastStamp](#coveragemilestonepostlatchaseconddeclarationrearmsfromthelaststamp) — 1
- [RunEndRow.CensoredRunWritesNullCompletionNotAStamp](#runendrowcensoredrunwritesnullcompletionnotastamp) — 1
- [EndpointOrdering.EventsBeforeRunStartAreDroppedAndCounted](#endpointorderingeventsbeforerunstartaredroppedandcounted) — 1
- [TEST](#test) — 1
- [RunEndRow.DuplicateRunEndIsSuppressedAndCounted](#runendrowduplicaterunendissuppressedandcounted) — 1
- [RunEndRow.DuplicateEndpointCountersAreWrittenEvenAtZero](#runendrowduplicateendpointcountersarewrittenevenatzero) — 1
- [MissionReturnGuard.TheLatchIsReadBeforeAnyReset](#missionreturnguardthelatchisreadbeforeanyreset) — 2
- [TerminalDispatch.ADeclinedManoeuvreStillEndsTheRun](#terminaldispatchadeclinedmanoeuvrestillendstherun) — 1
- [TerminalDispatch.TheDeferralIsRuledOutByTheTerminalFlag](#terminaldispatchthedeferralisruledoutbytheterminalflag) — 1

## File scope

### endpoint-tests-header-history

**Why the endpoint contract tests exist** — attached to `#include <gtest/gtest.h>` (line 4)

```text
It used to also carry the arithmetic rule that decided when a robot left a
task to keep an appointment. That rule is deleted (generation 19) and so is
the group of tests that pinned it; see the GROUP A marker below for what they
asserted and which of it still matters.

WHY THIS FILE EXISTS, AND WHY IT TESTS WHAT IT DOES. The primary metric of
every campaign in this repo is a stamp on one of these rows. Two separate
defects have already been found in them AFTER a 240-cell campaign had been
analysed:

  * ts1b shipped 16 robot-runs carrying TWO `mission_complete` rows, from a
    DONE -> RETURN_HOME re-entry. Every homing metric in those runs was
    double-counted, and finding it required auditing 240 cells by hand
    precisely because the duplicate rows were indistinguishable from each
    other. R1b added a re-entry guard and T5 added
    `MissionCompleteEvent::occurrence` so the NEXT one would be a grep —
    but R1b's guard tested `state_ == RETURN_HOME`, and the re-entry that
    was measured arrives from DONE, so it could not fire on the very case
    it was written for. Generation 9 (F1) adds the guard that can:
    `mission_return_done_`, latched when the return RESOLVES rather than
    while the leg is in flight, plus an idempotence latch in the writer.

  * A refused appointment re-arm left the node with `appointment_armed_ ==
    true` and `t_meet_ms == -1`, which made the departure test false forever
    AND suppressed the mid-run trigger — reconnection dead for the rest of the
    run, with nothing in the log saying so (see the note at
    explo_planner_node.cpp's appointment re-arm). The refusal is correct; what
    was missing was a test asserting that the -1 refuses even when the
    appointment reads as long overdue, which is the exact state that arose.

    THE HAZARD OUTLIVED ITS TEST. The departure test is gone and the tests
    that pinned it went with it, but appointmentDue() is now a bare
    `now >= t_meet` and a -1 t_meet on an armed slot reads as due FOREVER —
    the same silent death by the same door. What stands in for the deleted
    assertion is armAppointment clearing appointment_armed_ on every refusal
    path, which is a node-level invariant this suite cannot reach.

WHAT THIS FILE CANNOT TEST, STATED SO ITS ABSENCE IS NOT MISREAD AS COVERAGE.
The endpoint's STATE MACHINE lives in explo_planner_node.cpp, which is not in
explo_planner_lib's source list — an explicit list of files in CMakeLists.txt,
not a glob — so there is no node-level test of "DONE is entered once" here
or anywhere. `reconnect_terminal_`, the three departure sites, and the
RETURN_HOME re-entry guard are all verified by reading, not by this suite.
That is the reason `occurrence` exists: the thing the tests cannot reach has
to leave evidence in the artifact instead.

And none of it is a self-reporting PASS. A field that a guard prevents from
ever reaching 2 would be another check that stopped checking, so: the node
increments `mission_complete_count_` unconditionally at the call site; the
writer writes whatever it is handed (`OccurrenceIsWrittenNotAssumed` hands it
a 3 on a single emission, so a writer that hardcoded 1 would fail); the
writer's own idempotence latch COUNTS what it suppresses instead of silently
dropping it; and both counters — the node's refused re-entries and the
writer's suppressed rows — ride out on `run_end` and are written even when
they are zero (`DuplicateEndpointCountersAreWrittenEvenAtZero`), so a zero is
a reading rather than an absent field. The offline gate decides.

Generation 8 made the opposite call here — it let the duplicate ROW through
so the duplicate would be visible — and that is why 16 robot-runs reached
analysis with two endpoint rows and their cells were dropped whole. Keeping
the evidence and keeping the defect are separable, and this is the split.
```

## readLines

### endpoint-readlines-raw

**Reading log lines as raw bytes** — attached to `std::vector<std::string> readLines(const std::string& path) {` (line 101)

```text
Every line of the file, in write order.

Returns RAW LINES rather than a parsed structure on purpose: two of the
defects below are about a field being absent or present-but-wrong, and both
survive a lenient parser. The analysis scripts see bytes, so the assertions
are made on bytes.
```

## functionBody

### endpoint-functionbody-scan

**Function body extraction for source scans** — attached to `std::string functionBody(const std::string& text, const std::string& signature) {` (line 169)

```text
The body of a function definition, by brace matching from its signature.

Used only by the node-source scans below. Returns "" when the signature is
not found, which the callers ASSERT on: a scan that silently matches nothing
passes every assertion made about what it did not find, and that is the
failure mode these tests exist to avoid in the first place.
```

## stripComments

### endpoint-strip-comments

**Stripping comments before source scans** — attached to `std::string stripComments(const std::string& text) {` (line 197)

```text
Drop comments before scanning, so an assertion is about CODE.

The node's comments quote the very identifiers these scans look for —
several of the paragraphs around the endpoint explain the defect by naming
the call that caused it — so an unstripped scan can be satisfied by prose
describing a bug that is still present, and a reverted node would pass. The
quote tracking is not decoration: dropping from `//` unconditionally would
truncate any line holding a string literal containing `//` and silently
remove real code from the scan's view, which is the same failure one layer
down. Copied deliberately rather than shared: these scan helpers are
per-file by convention in this suite, and a header would make one file's
tightening everyone else's surprise.
```

## EndpointOrdering.ExplorationThenMissionThenRunEnd

### endpoint-group-a-sentinel

**The un-armed t_meet sentinel hazard** — attached to `TEST(EndpointOrdering, ExplorationThenMissionThenRunEnd) {` (line 257)

```text
Nine tests stood here pinning the edge arithmetic of
RendezvousScheduler::shouldDepart — the `<=` boundary, zero travel as a real
estimate rather than a refusal, t_meet == 0 as a valid appointment, negative
safety and negative margin clamping instead of inverting, zero safety leaving
the margin as the whole lead time, both -1 sentinels refusing even when long
overdue, and the truncation direction.

Every one of them was a correct statement about a function that no longer
exists. THE IDEA CAME BACK IN GENERATION 29 AND THE FUNCTION DID NOT: a robot
signs up to the first agreed occurrence it can still ARRIVE at, then leaves
when `now + appointmentLeadMs >= t_meet_ms`. That is a live lead against an
instant the team agreed, not a privately computed deadline, so there is still
no `remaining <= need` comparison for these tests to guard.

THE SENTINEL CASE IS THE ONE WORTH RE-READING BEFORE ANY FUTURE EDIT. It
pinned that an un-armed slot (t_meet_ms == -1) must not read as "long
overdue" and send a robot to cell -1. appointmentDue() still inherits exactly
that hazard, and the lead makes it strictly worse rather than better: a -1
t_meet on an armed slot is overdue by the bare comparison AND by the
lead-adjusted one.

WHAT PROTECTS IT IS CO-LOCATION, AND NOTHING ELSE. There is exactly one
`appointment_armed_ = true` in the node, and it sits in the same branch that
assigned `t_meet_ms` unconditionally from
`nextAgreedOccurrence(agreed.t_meet_ms, agreed.interval_ms, <a floor at or
after t_now>)`, under `rendezvous_agreed_.valid()` — so an armed slot cannot
carry the sentinel without someone separating those two writes. Keep them
together. The floor gained a per-robot arrival shortfall in generation 29;
the invariant is indifferent to what the floor is, only to the two writes
staying in one branch.

Two things that LOOK like the guard and are not, because a future edit will
reach for them first:

  * armAppointment clearing the flag on its refusal path. Real, but it fires
    where the flag is already false — its own comment says so — so it
    restores an invariant rather than establishing one. It protects the
    PLAN/FLAG pair against a future edit to a distant guard; it does not
    protect t_meet.
  * the `appointment_.valid()` conjunct in the arming condition.
    RendezvousPlan::valid() is `cell >= 0 && refused.empty()` and does not
    look at t_meet_ms at all.
```

## MissionCompleteRow.ResultAndReasonAreDistinctFields

### endpoint-result-vs-reason

**Mission result and reason stay distinct** — attached to `TEST(MissionCompleteRow, ResultAndReasonAreDistinctFields) {` (line 338)

```text
`result` and `reason` answer different questions and must not collapse into
each other: `reason` is WHY the robot went home (the terminal exploration
ending) and `result` is HOW the homing leg resolved. A run whose homing
timed out after a coverage latch reads result=timeout, reason=coverage-
latched, and conflating them would make every timeout look like a
step-budget ending.
```

## MissionCompleteRow.DuplicateIsSuppressedAndCounted

### endpoint-duplicate-mission-complete

**Suppressing a duplicate mission_complete** — attached to `TEST(MissionCompleteRow, DuplicateIsSuppressedAndCounted) {` (line 365)

```text
THE DUPLICATE, BOTH HALVES: refused in the file, recorded in the run.

This replaces generation 8's `OccurrenceCountsPastOne`, which asserted the
opposite contract — two emissions, two rows — on the reasoning that a latch
here would erase the only offline evidence a duplicate had happened. The
reasoning was sound and the conclusion was not: it left the artifact with
two endpoint rows and every consumer silently choosing one, which is exactly
how 16 ts1b robot-runs were double-counted and how `event_log.py` then
dropped their cells whole — the slow tail of the treatment arm.

The file now holds one mission_complete the way it holds one run_end. The
evidence is not lost, it MOVED: to `dupMissionCompletes()` in-process, to
`mission_completes_suppressed` on the run_end row, and to a WARN on the
first suppression. All three are asserted here, because a latch whose only
witness is a log line a human might read is a latch that can go inert.
```

## MissionCompleteRow.OccurrenceIsWrittenNotAssumed

### endpoint-occurrence-transcribed

**Occurrence is written, not assumed** — attached to `TEST(MissionCompleteRow, OccurrenceIsWrittenNotAssumed) {` (line 432)

```text
The hardcoding check the old duplicate-row test also served, kept alive
without the duplicate row. ONE emission, carrying occurrence 3 — a writer
that emitted a literal 1, or that derived the number from its own row count,
fails here. The node's counter is the authority on this field and the writer
is a transcriber.
```

## ExplorationCompleteRow.SecondDeclarationAdvancesLastAndLatchesFirst

### endpoint-second-exploration-complete

**Second exploration_complete keeps both stamps** — attached to `TEST(ExplorationCompleteRow, SecondDeclarationAdvancesLastAndLatchesFirst) {` (line 455)

```text
The opposite contract on the same-shaped field: a second
`exploration_complete` is LEGITIMATE (a merged map can deliver new frontiers
after a reconnect), so the pair of completion stamps has to keep both ends.
`explore_done_first_sim_sec` latches the first declaration while
`explore_done_sim_sec` advances to the latest — they answer "when did this
robot finish its own map" and "when did it stop trying", and a run is not
entitled to only one of them.
```

## GROUP B. THE ENDPOINT ROWS.

### postlatch-flag-purpose

**What the post_latch flag attributes** — attached to `namespace {` (line 504)

```text
C1 added `post_latch`/`sec_since_explore_done` to `coverage_milestone` to
answer one question: was this rung crossed AFTER the robot stopped
exploring, i.e. is it post-stop map merging rather than exploration? A rung
crossed post-declaration is a selection artifact, because WHETHER a run
reaches such a rung at all correlates with the arm.

Through generation 8 the flag was keyed on `explore_done_first_sec_`, a
one-way latch, and that inverted its purpose. A robot that declares, is
pulled into a reconnect manoeuvre, comes back with a merged map and explores
on was "post-latch" for the rest of the run — so every late rung of genuine
resumed exploration was labelled post-stop merging. Manoeuvres happen only
in the reconnecting arms, so a flag added to detect an arm-correlated
artifact was itself arm-correlated, in the direction that would have thrown
away the treated arms' real coverage.

Generation 9 keys it on the LAST declaration plus a resume flag that
`logStep` clears. These four tests pin both halves, and they are a probe
calibration as a set: the flag is observed BOTH true (2, 4) and false (1, 3)
on the same binary, so neither reading can be the flag being hardcoded,
absent, or wired to the wrong member. The interval is asserted by value, not
merely by sign, because -1 is the "not applicable" sentinel and a measured
interval that happened to be negative would be indistinguishable from it.
```

## CoverageMilestonePostLatch.AStepAfterTheDeclarationClearsTheFlag

### postlatch-step-clears-flag

**A step after declaration clears post_latch** — attached to `TEST(CoverageMilestonePostLatch, AStepAfterTheDeclarationClearsTheFlag) {` (line 609)

```text
THE GENERATION-8 DEFECT, as a test. A `step` after the declaration means
exploration resumed — the step counter is frozen for the whole of a
manoeuvre and the whole homing leg, so a step event can only arrive when
the robot is planning again — and every rung crossed from then on is
exploration, not post-stop merging.

Under generation 8 this row read `post_latch:true` with an interval of 100,
which is exactly the mislabelling that would have discarded the
reconnecting arms' resumed coverage.
```

## CoverageMilestonePostLatch.ASecondDeclarationReArmsFromTheLastStamp

### postlatch-rearm-last-declaration

**Second declaration re-arms from last stamp** — attached to `TEST(CoverageMilestonePostLatch, ASecondDeclarationReArmsFromTheLastStamp) {` (line 644)

```text
Declare, resume, declare again. The second declaration re-arms the flag,
and the interval is measured from the LAST declaration — 50 s, not the 350
s a first-declaration latch would report. Both halves matter: the re-arm is
what keeps the flag from being one-shot, and `explore_done_last_sec_` is
what keeps the interval honest.
```

## RunEndRow.CensoredRunWritesNullCompletionNotAStamp

### runend-censored-null-completion

**Censored run writes null completion** — attached to `TEST(RunEndRow, CensoredRunWritesNullCompletionNotAStamp) {` (line 683)

```text
A CENSORED run — the duration cap killed it before it ever declared
exhaustion — must write `explore_done_sim_sec` as NULL on run_end, not as a
number. This is the single most dangerous field in the schema: a reader that
takes "max sim time in the file" as the completion time converts every
censored run into a fast one, biasing the primary metric in the direction
that flatters whichever arm times out most.
```

## EndpointOrdering.EventsBeforeRunStartAreDroppedAndCounted

### endpoint-dropped-before-start

**Endpoints before run_start are dropped** — attached to `TEST(EndpointOrdering, EventsBeforeRunStartAreDroppedAndCounted) {` (line 717)

```text
Endpoints emitted before `run_start` are DROPPED AND COUNTED, never
buffered. The node calls startRun on its first tick with a live clock, so a
run whose clock never comes up, or that ends inside that first tick, loses
its endpoints — and `events_dropped_before_start` on run_end is the only
evidence that anything was lost. A zero there must mean "nothing was
dropped", so this asserts the counter actually counts.
```

## TEST

### runend-dropped-counts-itself

**The dropped run_end counts itself** — attached to `EXPECT_NE(lines[static_cast<size_t>(i)].find(` (line 745)

```text
3, counting the dropped run_end itself. It is counted here because it used
not to be: logRunEnd's guard was a single collapsed `if` that returned
without touching the counter, so this one event kind escaped the logger's
own accounting — in exactly the scenario where the accounting is the only
surviving evidence.
```

## RunEndRow.DuplicateRunEndIsSuppressedAndCounted

### runend-duplicate-suppressed

**Duplicate run_end suppressed and counted** — attached to `TEST(RunEndRow, DuplicateRunEndIsSuppressedAndCounted) {` (line 756)

```text
A SECOND run_end is suppressed — the file must keep exactly one last line,
because every "read the tail" consumer would otherwise get a coin flip — but
the suppression is COUNTED and warned, not silent. A guard that absorbs a
duplicate endpoint indefinitely with nothing recorded is the same shape as
the 16 ts1b runs whose double mission_complete rows took a 240-cell audit to
find.
```

## RunEndRow.DuplicateEndpointCountersAreWrittenEvenAtZero

### endpoint-dup-counters-calibration

**Probe calibration for duplicate counters** — attached to `TEST(RunEndRow, DuplicateEndpointCountersAreWrittenEvenAtZero) {` (line 795)

```text
PROBE CALIBRATION for the two duplicate-endpoint counters.

Both are meant to read 0 for the rest of this project's life, and a field
that is only ever observed at 0 is indistinguishable from a field that is
absent, hardcoded, or wired to the wrong member. So: one run with nothing
wrong must WRITE both keys at 0, and one run handed non-zero values must
write those values back verbatim. Without the second half, "0 re-entries
across the campaign" would be a claim about the gate rather than about the
runs.

`mission_return_reentries` is the NODE's counter (startReturnHome refusing a
second homing leg) and can only arrive through RunEndEvent, so it is handed
in. `mission_completes_suppressed` is the WRITER's own and is read off the
latch — which is why it is driven here by actually emitting a duplicate
rather than by setting a field.
```

## MissionReturnGuard.TheLatchIsReadBeforeAnyReset

### return-guard-source-scan

**Why the return guard is source-scanned** — attached to `TEST(MissionReturnGuard, TheLatchIsReadBeforeAnyReset) {` (line 862)

```text
Everything above this line tests the WRITER. The guard that actually stops
the ts1b defect is in the node — startReturnHome refusing a second homing leg
— and `explo_planner_node.cpp` is not in explo_planner_lib's source list (an
explicit file list, not a glob), so nothing links it and no test can call it.
The choice is therefore between a source scan and no coverage at all, and the
file header's promise that the node-level guard is "verified by reading, not
by this suite" is precisely how R1b shipped a guard that tested the wrong
state for a whole generation without anything noticing.

So these read the source, exactly as test_experiment_log's
DeclaredKindsMatchTheWriter does, and they are honest about their limits:
they can show the latch is present and ORDERED correctly, and they cannot
show it is reached. What they catch is the realistic regression — a later
edit moving, renaming, or deleting it — and they fail loudly if the scan
itself stops matching, which is the way a check of this kind usually dies.
```

### return-guard-before-resets

**Latch read before any reset** — attached to `TEST(MissionReturnGuard, TheLatchIsReadBeforeAnyReset) {` (line 879)

```text
The latch must be READ before any of startReturnHome's resets run. Every
assignment in that function is a reset: a guard placed after even one of
them refills the escape budget and zeroes the approach window on a request
it then refuses, which is worse than no guard, because the damage happens
and the log says the request was denied.
```

## TerminalDispatch.ADeclinedManoeuvreStillEndsTheRun

### terminal-dispatch-disposes-run

**Terminal dispatch always disposes of the run** — attached to `TEST(TerminalDispatch, ADeclinedManoeuvreStillEndsTheRun) {` (line 961)

```text
finishOrRendezvous returns a bool the callers read as "was the run disposed
of". `false` means DEFERRED — ask again on the next tick — and doLogStep acts
on it by sending the node back to PLAN. That contract is only safe while
every `false` it can return is BOUNDED, and exactly one is: the
reconnect_confirm_sec confirmation window, which resolves either into a
manoeuvre or into DONE because team_last_complete_time_ only advances while
the team is whole.

WHAT THIS GROUP EXISTS TO CATCH (2026-09-18). For two days the function also
returned dispatchReconnect's value straight through. On a terminal dispatch
reconnect_terminal_ is set, so may_defer is false by construction, and the
only `false` dispatchReconnect can then produce is the RENDEZVOUS/HYBRID tail
declining outright — armAppointment refused and the 2026-09-16 removal left
that arm with no unscheduled fallback to drive to. Nothing about that
resolves: the agreed pair is frozen for the duration of the outage, step_ is
only incremented by doLogStep, and PLAN's budget test re-fires on the frozen
count. The node spins in PLAN at the 10 Hz tick rate forever, AFTER
recordExplorationComplete has stamped the endpoint — which is also what
disarms the harness's stall gate, so the runner waits out the full wall
clock with nothing in the log saying the run was over.

It never reached a campaign: every campaign sets mission_return_enabled,
which returns from finishOrRendezvous above the dispatch, and all 196 planner
logs written since the change end on [latch] / [coverage-latched] with zero
[step-budget]. Generation 32 put a SECOND early return above them both —
keepAppointmentOnFinish, which takes any robot that finishes with a standing
appointment and sends it to the meeting — so the terminal dispatch's own
appointment branch is now shadowed twice over. The scans below are unmoved by
that: what they constrain is the shape of the path once it IS reached, and
both shadows are config-and-state accidents rather than deletions. Every one of the 39 appointment refusals in them carries the
mid-run reason `peer-lost`, where a `false` is correct and the caller simply
keeps planning. That is a config accident, not a property of the code, and
`--mission-return 0` is a supported flag.

These are source scans, with the limits every scan in this file has: they
show the shape is present, not that it is reached.
```

## TerminalDispatch.TheDeferralIsRuledOutByTheTerminalFlag

### terminal-dispatch-may-defer

**Terminal flag rules out deferral** — attached to `TEST(TerminalDispatch, TheDeferralIsRuledOutByTheTerminalFlag) {` (line 1033)

```text
The dispatch is where an unbounded `false` can originate, so pin that the
terminal flag really does rule out the deferral branch inside it. If
may_defer stopped consulting reconnect_terminal_, a terminal dispatch could
take the "keep exploring until the deadline" path — which is a deferral with
no caller willing to wait for it, and the test above would keep passing.
```
