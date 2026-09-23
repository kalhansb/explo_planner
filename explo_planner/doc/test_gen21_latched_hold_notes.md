# test_gen21_latched_hold.cpp — design notes and history

The long comments of `test/test_gen21_latched_hold.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [stripComments](#stripcomments) — 1
- [Gen21LatchedHold.TheHoldIsBelowTheEndpointStampAndAboveEveryEnding](#gen21latchedholdtheholdisbelowtheendpointstampandaboveeveryending) — 1
- [TEST](#test) — 1
- [Gen21LatchedHold.TheHoldRequiresArrivalAndTheSyncState](#gen21latchedholdtheholdrequiresarrivalandthesyncstate) — 1
- [Gen21LatchedHold.TheTeardownFlagIsSetBelowTheHoldAndAboveTheEndings](#gen21latchedholdtheteardownflagissetbelowtheholdandabovetheendings) — 1
- [Gen21GuardedExits.TheBarrierReleaseEndsTheRunForALatchedRobot](#gen21guardedexitsthebarrierreleaseendstherunforalatchedrobot) — 1
- [Gen21GuardedExits.TheLatchedHoldIsCappedIndependentlyOfTheWaitCap](#gen21guardedexitsthelatchedholdiscappedindependentlyofthewaitcap) — 1
- [TEST (part 2)](#test-part-2) — 1
- [Gen21GuardedExits.TheMidRunGiveUpIsDisqualifiedByTheLatch](#gen21guardedexitsthemidrungiveupisdisqualifiedbythelatch) — 1
- [Gen21OutcomeClassifier.RunEndedSitsBelowTeamBackAndAboveTheArrivalSplit](#gen21outcomeclassifierrunendedsitsbelowteambackandabovethearrivalsplit) — 1
- [Gen21LatchedHold.TheCapIsADeclaredAndValidatedParameter](#gen21latchedholdthecapisadeclaredandvalidatedparameter) — 1
- [TEST (part 3)](#test-part-3) — 1
- [Gen32KeepAppointment.BothEndingsKeepItAheadOfMissionReturn](#gen32keepappointmentbothendingskeepitaheadofmissionreturn) — 1
- [Gen32KeepAppointment.TheLatchStopsThePlanTickWhenItKeeps](#gen32keepappointmentthelatchstopstheplantickwhenitkeeps) — 1
- [Gen32KeepAppointment.AnUnplaceableCellIsDeclinedBeforeDeparture](#gen32keepappointmentanunplaceablecellisdeclinedbeforedeparture) — 1
- [Gen32KeepAppointment.TheUnreachedLegDoesNotResumeALatchedRobot](#gen32keepappointmenttheunreachedlegdoesnotresumealatchedrobot) — 1
- [Gen33MeetingAttendance.ThePublisherSaysDoneThroughTheAttendanceRule](#gen33meetingattendancethepublishersaysdonethroughtheattendancerule) — 1
- [Gen33MeetingAttendance.TheAppointmentBarrierWaitsForAFinishedPeerStillComing](#gen33meetingattendancetheappointmentbarrierwaitsforafinishedpeerstillcoming) — 1
- [Gen33MeetingAttendance.TheWaitIsStampedFlooredKeptAndCleared](#gen33meetingattendancethewaitisstampedflooredkeptandcleared) — 1

## stripComments

### latched-hold-strip-comments-copy

**Why stripComments is copied per binary** — attached to `std::string stripComments(const std::string& text) {` (line 113)

```text
The source with `//` and `/* */` comments removed, string literals preserved.
Duplicated from test_gen20_rendezvous.cpp rather than shared: these are
standalone source-scan binaries that link nothing of their own, and a shared
header between two test executables whose whole job is to be independently
re-runnable against a mutated node would make one mutation break both.
```

## Gen21LatchedHold.TheHoldIsBelowTheEndpointStampAndAboveEveryEnding

### latched-hold-between-stamp-and-endings

**The hold sits between stamp and endings** — attached to `TEST(Gen21LatchedHold, TheHoldIsBelowTheEndpointStampAndAboveEveryEnding) {` (line 190)

```text
THE HOLD SITS BETWEEN THE ENDPOINT STAMP AND EVERY ENDING.

Both halves of that sentence are the assertion, and they fail differently:

  * ABOVE THE ENDINGS. There are three of them (startReturnHome, the park,
    the done_seek coast) and the first is `mission_return_enabled_ &&
    have_home_`, which is TRUE in every campaign config. A hold placed after
    it is dead code that reads as live — the robot drives home and the test
    suite is green.
  * BELOW recordExplorationComplete. The exploration ENDPOINT must not move
    because of the hold. The robot finished when the map saturated, not when
    the meeting resolved, and stamping it later would make the rendezvous
    and hybrid arms pay their own hold on the primary metric.
```

## TEST

### latched-hold-returns-false

**Why the hold returns false** — attached to `const size_t hold_stmt_end = body.find('}', hold);` (line 239)

```text
The hold must EXIT the function, not fall through into the endings it was
placed above. `return false` specifically: coverage_latched_ is already
true, so the guard at the top of the function makes every re-entry a no-op
and the manifest still reports finished. `return true` would tell the
caller the run ended, which is the thing the hold is refusing to do.
```

## Gen21LatchedHold.TheHoldRequiresArrivalAndTheSyncState

### latched-hold-gated-on-arrival

**The hold is gated on arrival** — attached to `TEST(Gen21LatchedHold, TheHoldRequiresArrivalAndTheSyncState) {` (line 259)

```text
THE HOLD IS GATED ON HAVING ARRIVED, NOT ON MERELY BEING ON AN APPOINTMENT.

`appointment_manoeuvre_` is true for the whole drive, and the return-budget
and no-progress paths also enter RETURN_SYNC on an appointment manoeuvre with
arrived=false. Those robots are not standing on any agreed place, so holding
them is a hold in an arbitrary spot — the pre-gen-21 ending is the right one
for them and this gate is what keeps it.
```

## Gen21LatchedHold.TheTeardownFlagIsSetBelowTheHoldAndAboveTheEndings

### latched-hold-teardown-flag-placement

**Where the teardown flag is set** — attached to `TEST(Gen21LatchedHold, TheTeardownFlagIsSetBelowTheHoldAndAboveTheEndings) {` (line 294)

```text
EVERY ENDING BELOW THE HOLD IS FLAGGED AS A TEARDOWN.

`coverage_latch_teardown_` is what stops the appointment outcome classifier
calling a coverage-latch ending a no-show. It must be set BELOW the hold and
ABOVE the endings: above the hold it would fire on the held robot too, which
is the one case that is NOT a teardown — that robot is still keeping its
appointment and its outcome is still open.
```

## Gen21GuardedExits.TheBarrierReleaseEndsTheRunForALatchedRobot

### latched-hold-barrier-release

**Barrier release ends a latched run** — attached to `TEST(Gen21GuardedExits, TheBarrierReleaseEndsTheRunForALatchedRobot) {` (line 342)

```text
1b. THE BARRIER RELEASE MUST NOT HAND A LATCHED ROBOT BACK TO PLAN.

This release is unconditional in every generation before 21, and it was safe
only because a robot in RETURN_SYNC could never be finished. The hold breaks
that premise. A latched robot sent to PLAN explores a finished map to
max_steps_ and calls finishOrRendezvous("step-budget") ->
recordExplorationComplete a second time; that stamp is keyed on `step_`, not
once per run, so the cell carries TWO exploration_complete rows at very
different t_sim, in the rendezvous and hybrid arms only, and the readers
disagree about which to keep (ts1b_cells.py takes the last, n23.py the
first). An asymmetric corruption of the primary endpoint is strictly worse
than the no-show miscount the hold was added to fix.
```

## Gen21GuardedExits.TheLatchedHoldIsCappedIndependentlyOfTheWaitCap

### latched-hold-cap-before-wait-cap

**The latched-hold cap and its order** — attached to `TEST(Gen21GuardedExits, TheLatchedHoldIsCappedIndependentlyOfTheWaitCap) {` (line 397)

```text
1c. THE HOLD HAS ITS OWN CAP, CHECKED BEFORE THE ORDINARY PATIENCE.

The ordinary appointment patience is `rendezvous_appointment_wait_sec`, which
is 0 = unbounded by default and is MEANT to be — that patience belongs to a
robot with exploring left to trade against it. A finished robot has none, so
an unbounded hold is a cell that runs to the harness wall clock with one
robot standing still. Because the ordinary cap is unbounded, rendezvousWaitExpired
never fires on an appointment manoeuvre, so a cap placed after it is dead.
```

## TEST (part 2)

### latched-hold-cap-scan-anchor

**Anchor the cap scan on its ending** — attached to `const size_t cap_end = body.find("latched-hold-expired");` (line 412)

```text
ANCHORED FROM THE ENDING REASON BACKWARDS, and this direction is the whole
point. Anchoring on the member name alone would pass with the cap deleted —
the release path's log line names it thirty lines earlier, also above
rendezvousWaitExpired. Anchoring FORWARDS on the conditional was the fix for
that and it decayed in generation 32, which added a second
`if (coverage_latched_ && ...)` — the lazy hold stamp — ABOVE the cap: the
forward find then resolved to the stamp, every assertion here passed against
the wrong block, and moving the cap below rendezvousWaitExpired stopped
being caught while the suite still reported green. Search back from
"latched-hold-expired" instead. That string is the cap's own ending and
nothing else emits it, so the conditional immediately above it is the cap by
construction no matter how many siblings are added later.
```

## Gen21GuardedExits.TheMidRunGiveUpIsDisqualifiedByTheLatch

### latched-hold-midrun-giveup

**The mid-run give-up excludes a latched robot** — attached to `TEST(Gen21GuardedExits, TheMidRunGiveUpIsDisqualifiedByTheLatch) {` (line 465)

```text
1d. THE MID-RUN GIVE-UP MUST NOT HAND A LATCHED ROBOT BACK TO PLAN EITHER.

This is the same defect as 1b through a different door, and it is the easier
one to miss: `reconnect_terminal_` reads as an exceptional state but its
value on a mid-run manoeuvre is FALSE by construction — all 16 gen-20
dispatch+end row pairs carry terminal=False — so this branch is the DEFAULT
path out of an expired barrier, not a corner of it.
```

## Gen21OutcomeClassifier.RunEndedSitsBelowTeamBackAndAboveTheArrivalSplit

### latched-hold-run-ended-ordering

**Where run-ended sits in the classifier** — attached to `TEST(Gen21OutcomeClassifier, RunEndedSitsBelowTeamBackAndAboveTheArrivalSplit) {` (line 508)

```text
`run-ended` ANSWERS A PRIOR QUESTION TO arrived/unreachable, AND A LATER ONE
THAN team_back.

The ordering IS the semantics, and both neighbours matter:

  * BELOW team_back. If the team is back, the meeting succeeded — regardless
    of what ended this robot's run. Putting run-ended above it would relabel
    successful reunions.
  * ABOVE the arrived/unreachable split. Whether this robot reached the cell
    says how the appointment was GOING; it says nothing once the run ended
    underneath it. A teardown mid-drive is no more "unreachable" than a
    teardown on the cell is a "no-show".

The defect this fixes is a sign reversal, not a miscount: in the smoke20 N=3
rendezvous cell the arm's headline failure count was made of its successes.
```

## Gen21LatchedHold.TheCapIsADeclaredAndValidatedParameter

### latched-hold-cap-parameter

**The cap is a validated parameter** — attached to `TEST(Gen21LatchedHold, TheCapIsADeclaredAndValidatedParameter) {` (line 581)

```text
THE CAP IS A DECLARED PARAMETER WITH A VALIDATED DEFAULT.

A hard-coded hold would be untunable from the harness, and the harness is
the only place the value is ever chosen (`RDV_LATCHED_HOLD`). The validation
matters for the same reason every other duration in this node validates: a
NaN or a negative from a typo'd override would make `held >= cap` false
forever, which is silently the unbounded hold this cap exists to prevent.
```

## TEST (part 3)

### latched-hold-cap-rejects-zero

**Why the cap validator rejects zero** — attached to `const size_t read = text.find("dp(\"rendezvous_latched_hold_sec\"");` (line 602)

```text
NON-POSITIVE, not merely negative. Every guard on this value reads
`> 0.0`, so zero DISABLES the cap rather than setting it to nothing, and
since generation 32 the cap is the only bound a latched keeper has: it is
non-terminal by construction, so rendezvous_max_wait_sec cannot end it and
rendezvous_appointment_wait_sec is itself 0 = forever. A `< 0.0` validator
admits the one value that reproduces the censored cell, through a
parameter that reads like a disable switch.
```

## Gen32KeepAppointment.BothEndingsKeepItAheadOfMissionReturn

### keep-appointment-both-endings

**Both endings keep the appointment** — attached to `TEST(Gen32KeepAppointment, BothEndingsKeepItAheadOfMissionReturn) {` (line 642)

```text
THE KEEP IS CALLED FROM BOTH ENDINGS, AND ABOVE MISSION RETURN IN EACH.

Both halves fail differently and both are silent:

  * ONE CALLER ONLY. The two endings catch different robots — the coverage
    latch catches one that saturates mid-tick, finishOrRendezvous catches
    the step budget and the peer-announced paths — and a robot that reaches
    the ending this call is missing from abandons its meeting exactly as it
    did in generation 31.
  * BELOW `mission_return_enabled_`. That branch is true in every campaign
    config, so a keep placed under it never runs. This is not hypothetical:
    it is precisely why dispatchReconnect's own "exploration is over, go to
    the appointment now" branch was unreachable for eleven generations
    while reading as live.
```

## Gen32KeepAppointment.TheLatchStopsThePlanTickWhenItKeeps

### keep-appointment-latch-returns-true

**The latch's keep returns true** — attached to `TEST(Gen32KeepAppointment, TheLatchStopsThePlanTickWhenItKeeps) {` (line 686)

```text
THE LATCH'S KEEP RETURNS TRUE, AND THE DISTINCTION IS THE SAFETY ARGUMENT.

maybeLatchCoverageDone's return value means "the caller must stop this
tick", and doPlan is the only caller that acts on it. The gen-21 hold above
can afford `false` because it is gated on State::RETURN_SYNC, which doPlan
is never in; this call is not, because a robot saturates from PLAN. `false`
here lets doPlan run on past a dispatch that has already published the
meeting goal, select a frontier, publish it over that goal and enter
NAVIGATE — at which point the classifier closes the appointment
`unreachable` on a fabricated navigation failure, and with the coverage
ending spent there is nothing left to end the run but max_steps_.
```

## Gen32KeepAppointment.AnUnplaceableCellIsDeclinedBeforeDeparture

### keep-appointment-unplaceable

**Decline an unplaceable cell before departing** — attached to `TEST(Gen32KeepAppointment, AnUnplaceableCellIsDeclinedBeforeDeparture) {` (line 723)

```text
AN UNPLACEABLE CELL IS DECLINED BEFORE ANY STATE MOVES.

appointmentPoint() has a branch that cannot solve the agreed cell on either
grid: it latches `appointment_unplaceable_` and answers the robot's OWN
position. Departing for that "arrives" on the next tick, so the keeper would
hold a barrier at its own feet for the full latched cap over a meeting no
travel of its could reach. The check must be above `appointment_departed_`,
because declining after that flag is set closes the record as a departure
that never happened.
```

## Gen32KeepAppointment.TheUnreachedLegDoesNotResumeALatchedRobot

### keep-appointment-leg-terminal-rung

**The leg's last rung ends a latched run** — attached to `TEST(Gen32KeepAppointment, TheUnreachedLegDoesNotResumeALatchedRobot) {` (line 779)

```text
THE APPOINTMENT LEG'S LAST RUNG MUST NOT HAND A LATCHED ROBOT BACK TO PLAN.

The same defect as 1b and 1d through a third door, and this one opened only
in generation 32: before the keep, no drive to an agreed cell could be
carrying a finished robot, so every robot this rung returned to PLAN still
had its ending in front of it. One that has spent the coverage ending does
not — first-touch latching will not give it another — so the exit leaves
nothing but max_steps_ and a SECOND exploration_complete, in the treated
arms only.
```

## Gen33MeetingAttendance.ThePublisherSaysDoneThroughTheAttendanceRule

### attendance-publisher-says-done

**The publisher says DONE via attendance** — attached to `TEST(Gen33MeetingAttendance, ThePublisherSaysDoneThroughTheAttendanceRule) {` (line 1118)

```text
THE PUBLISHER SAYS DONE THROUGH THE ATTENDANCE RULE, AND ONLY THROUGH IT.

The defect was a keeper publishing DONE at the start of its drive to the
meeting. announcedMode holds the level below HOMING while the robot keeps
its appointment — but only if it is TOLD the robot is keeping it, and only
if nothing else in the publisher writes the level afterwards. And the
homing latch must be current when the call reads it, or the tick the robot
turns for home publishes one level too low.
```

## Gen33MeetingAttendance.TheAppointmentBarrierWaitsForAFinishedPeerStillComing

### attendance-finished-peer-veto

**Waiting for a finished peer still coming** — attached to `TEST(Gen33MeetingAttendance, TheAppointmentBarrierWaitsForAFinishedPeerStillComing) {` (line 1154)

```text
THE APPOINTMENT BARRIER WAITS FOR A FINISHED PEER STILL COMING — BOUNDED,
AND ONLY THERE.

Three things, one per assertion: the veto is in the APPOINTMENT branch of
the release predicate (the other branch serves mid-run and terminal
reconnects, which are this robot's own business); it answers false outside
an appointment manoeuvre (the classifier and doReturnNav also call the
predicate); and it runs out at rendezvous_latched_hold_sec measured from
its stamp, or a partner whose "leaving" the radio never delivered holds
this robot to the run's end.
```

## Gen33MeetingAttendance.TheWaitIsStampedFlooredKeptAndCleared

### attendance-finished-peer-wait-clock

**The finished-peer wait's clock** — attached to `TEST(Gen33MeetingAttendance, TheWaitIsStampedFlooredKeptAndCleared) {` (line 1202)

```text
THE WAIT'S CLOCK: STAMPED AT THE BARRIER, FLOORED AT t_meet, KEPT FOR THE
MANOEUVRE, AND ITS END LOGGED WHERE IT CAN BE REACHED.

The floor is GROUP F's argument again: the cap is sized from t_meet, so an
early arrival must not spend it before the team is due. Kept across legs,
because the settle-lapsed resume re-dispatches from the barrier and a
restart per leg is an unbounded wait by instalments. Cleared when the
manoeuvre ends, or the next appointment inherits a spent bound. And the
expiry line sits ABOVE the release gate: the expiry is what opens the gate,
and the settle behind it returns before anything further down runs.
```
