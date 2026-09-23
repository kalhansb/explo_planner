# test_gen22_start_hold.cpp — design notes and history

The long comments of `test/test_gen22_start_hold.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [stripComments](#stripcomments) — 1
- [Gen22StartHold.TheHoldIsAConjunctOfTheStartCondition](#gen22startholdtheholdisaconjunctofthestartcondition) — 1
- [TEST](#test) — 1
- [Gen22StartHold.TheHoldIsMeasuredFromMissionElapsedWithGreaterEqual](#gen22startholdtheholdismeasuredfrommissionelapsedwithgreaterequal) — 1
- [Gen22StartHold.WaitForMapHasOneExitAndStartGuardsIt](#gen22startholdwaitformaphasoneexitandstartguardsit) — 1
- [Gen22StartHold.TheUpgradeIsNotConfinedToTheHold](#gen22startholdtheupgradeisnotconfinedtothehold) — 2
- [Gen22StartHold.TheDeriveGateHasNoClockTerm](#gen22startholdthederivegatehasnoclockterm) — 1
- [Gen22StartHold.TheHandshakeRunsAboveHeartbeatTicksEarlyReturns](#gen22startholdthehandshakerunsaboveheartbeatticksearlyreturns) — 1
- [TEST (part 2)](#test-part-2) — 1
- [Gen22StartHold.TheHoldIsADeclaredAndValidatedParameter](#gen22startholdtheholdisadeclaredandvalidatedparameter) — 1
- [Gen22StartHold.TheMemberAndParameterDefaultsAgreeAndAreOn](#gen22startholdthememberandparameterdefaultsagreeandareon) — 1
- [Gen22StartHold.TheHarnessPassesTheHoldToEveryArm](#gen22startholdtheharnesspassestheholdtoeveryarm) — 1
- [Gen22StartHold.TheHarnessDefaultsTheHoldOnAndRecordsIt](#gen22startholdtheharnessdefaultstheholdonandrecordsit) — 1
- [Gen22StartHold.TheSuppressionWarnIsGatedOutOfWaitForMap](#gen22startholdthesuppressionwarnisgatedoutofwaitformap) — 1
- [Gen22StartHold.TheSuppressionExemptionIsEpisodeScopedNotInstantScoped](#gen22startholdthesuppressionexemptionisepisodescopednotinstantscoped) — 1
- [TEST (part 3)](#test-part-3) — 1
- [Gen22StartHold.TheWaitingToStartWarnFiresOnlyOnAMissingPrecondition](#gen22startholdthewaitingtostartwarnfiresonlyonamissingprecondition) — 1

## stripComments

### start-hold-strip-comments-copy

**Why stripComments is copied per binary** — attached to `std::string stripComments(const std::string& text) {` (line 190)

```text
The source with `//` and `/* */` comments removed, string literals preserved.
Duplicated from test_gen20_rendezvous.cpp and test_gen21_latched_hold.cpp
rather than shared: these are standalone source-scan binaries that link
nothing of their own, and a shared header between test executables whose
whole job is to be independently re-runnable against a mutated node would
make one mutation break all three.
```

## Gen22StartHold.TheHoldIsAConjunctOfTheStartCondition

### start-hold-conjunct-of-start

**The hold is a conjunct of start** — attached to `TEST(Gen22StartHold, TheHoldIsAConjunctOfTheStartCondition) {` (line 313)

```text
THE HOLD IS A CONJUNCT OF `start`, NOT A STATEMENT BEFORE IT.

The distinction is the whole test. Sequencing the hold ahead of the
preconditions ("wait 60 s, then wait for the map") makes the release time
hold + map_latency; ANDing them makes it max(hold, map_latency), which is
what the design says and what the campaign's wall-clock budget assumes. It
also fails safe in the direction that matters: a robot whose map arrives at
t+55 s must not get a 5 s hold.
```

## TEST

### start-hold-conjunct-literals

**Why the start literals are pinned** — attached to `EXPECT_NE(arm.find("const bool preconds_met = have_map_ && have_pose_ &&"),` (line 332)

```text
THE LITERAL MOVED ON 2026-09-18 AND THIS TEST MOVED WITH IT. Until then
the whole condition was one expression, `const bool start = have_map_ &&
have_pose_ && (!use_planning_map_ || have_plan_map_) && hold_done;`, and
both assertions below read halves of that one line. Generation 23 gave the
preconditions their own name so the "Waiting to start" WARN could be keyed
on them WITHOUT the hold — see
TheWaitingToStartWarnFiresOnlyOnAMissingPrecondition, which owns that half.

The rename is a pure refactor of this expression and the invariant here is
untouched: both terms are still conjuncts of ONE boolean, evaluated
unconditionally, so release is at max(hold, map latency). What this test
must not become is a token search — `hold_done` and `have_map_` appearing
somewhere in the arm would be satisfied by a sequenced wait too. The
literals are asserted precisely so that nothing can sit between the terms.
```

## Gen22StartHold.TheHoldIsMeasuredFromMissionElapsedWithGreaterEqual

### start-hold-mission-elapsed-clock

**The hold's clock is missionElapsed** — attached to `TEST(Gen22StartHold, TheHoldIsMeasuredFromMissionElapsedWithGreaterEqual) {` (line 365)

```text
THE HOLD IS MEASURED FROM missionElapsed(), WITH `>=`.

Two claims, and the first is the load-bearing one. missionElapsed() latches
its baseline on the first tick with a positive clock and returns -1.0 before
then — which is BELOW any non-negative hold, so a robot with no /clock yet
holds, the safe direction. Re-keying this on state entry time or on
this->now() breaks that: under use_sim_time this->now() reads exactly 0
until the first /clock message lands, and a wall-clock delta measures the
launch sequence rather than the mission.

The `>=` is the smaller claim but it is asserted literally because the
complement of this predicate is `within_start_hold` in
maintainRendezvousProposal, and the two must partition the timeline exactly
once — see the test that pairs them.
```

## Gen22StartHold.WaitForMapHasOneExitAndStartGuardsIt

### start-hold-single-exit

**WAIT_FOR_MAP has one guarded exit** — attached to `TEST(Gen22StartHold, WaitForMapHasOneExitAndStartGuardsIt) {` (line 400)

```text
WAIT_FOR_MAP HAS EXACTLY ONE EXIT AND `start` GUARDS IT.

The gate is only a gate if there is nothing to walk around. This asserts
the arm contains a single transitionTo and that it is inside `if (start)`,
so a later edit that adds a second escape — a timeout, a "start anyway"
diagnostic path, a retry that gives up — fails here rather than in a
campaign six hours in.
```

## Gen22StartHold.TheUpgradeIsNotConfinedToTheHold

### start-hold-upgrade-not-confined

**Why the upgrade is not hold-confined** — attached to `TEST(Gen22StartHold, TheUpgradeIsNotConfinedToTheHold) {` (line 435)

```text
This group asserts the ABSENCE of an edit, which is unusual enough to say
why. On 2026-09-17 the hold shipped with a companion conjunct confining the
provisional->final upgrade to the hold window:

    (rendezvous_held_provisional_ && within_start_hold)

It is the obvious fix for the split — author the upgrade only while the team
is co-located, and there is no partition to be on the wrong side of — and it
was withdrawn the same day because it does not schedule the upgrade earlier,
it prevents it entirely. Measured over every banked cell carrying
agreed_provisional (5 campaigns, 10 cells, 28 robot-runs, 58 agreement rows):

  * the step counter at the first NON-provisional commit is never below 2;
    the observed distribution is {2, 3, 6, 7, 8} and no upgrade anywhere in
    the corpus was authored at step 0 or step 1;
  * on EVERY provisional commit the proposer's provenance reads
    candidates=1, rejected_unreachable=0, rejected_excluded=0 — the pool was
    EMPTY, not filtered, so the limit is not reachability (which co-location
    would fix) but the absence of any tour to draw a candidate from;
  * two banked upgrades landed at t=311.1 s and t=353.6 s, more than 250 s
    after the provisional they replaced.

A robot held in WAIT_FOR_MAP completes zero steps and so grows none of the
EXPLORING cells the allocator builds tours over. The upgrade branch is also
the only site that ever clears rendezvous_held_provisional_, so a window that
closes first freezes the pair for the whole run and both scheduled arms
quietly become "meet at the team's initial centroid" — while passing every
unanimity check, because a fleet that unanimously agrees a placeholder is
still unanimous. That is a silent null wearing a treatment's name, which is
the failure mode this file exists to make loud.

So the tests below fail if the conjunct comes back, in either branch.
```

### start-hold-derive-gate-disjunction

**The derive gate is a pure disjunction** — attached to `TEST(Gen22StartHold, TheUpgradeIsNotConfinedToTheHold) {` (line 469)

```text
THE UPGRADE IS **NOT** TIME-CONFINED.

The gate is a PURE DISJUNCTION: derive if there is no pair, or re-derive for
as long as the pair is only a placeholder, whenever the team is mutually
whole. No clock term. See the group banner for the measurement.

PINNED ON THE SHAPE, NOT ON THE TEXT (2026-09-18). Until generation 23 this
read the literal string `(!rendezvous_held_.valid() ||
rendezvous_held_provisional_))`, and generation 23 broke it by adding a
third disjunct, the post-reunion re-decide (since removed: its follower
side was never built, so every re-decided pair was refused fleet-wide).
Widening the gate is legal here. Every failure this test was written to
catch narrows it, by conjoining a term onto the upgrade; a literal pin
cannot tell the two apart and fails on the safe direction while a reader
assumes it caught the unsafe one. So the assertion below extracts the
parenthesised group and demands it contain no `&&` at all: new disjuncts
are free, any conjunct is a failure, wherever inside the group it is
spelled and whatever it is named.

What bounds the split instead is stated in the derive gate's own comment and
is weaker than confinement, deliberately: the proposer keeps re-deriving
while provisional and publishTeamWorld re-broadcasts the held pair every
cycle, so a follower that misses an upgrade adopts it when the link returns.
The disagreement lasts as long as the partition, not as long as the run.
Two Generals says no protocol does better; this one at least converges.
```

## Gen22StartHold.TheDeriveGateHasNoClockTerm

### start-hold-derive-no-hold-read

**Neither derive branch reads the hold** — attached to `TEST(Gen22StartHold, TheDeriveGateHasNoClockTerm) {` (line 539)

```text
NEITHER BRANCH OF THE DERIVE GATE READS THE HOLD.

Enforced on the identifier rather than on the branch shape, so that a
re-added conjunct is caught wherever it is spelled and whichever disjunct it
is attached to. `within_start_hold` was the name it had; a zero count is the
invariant, and any clock term reintroduced under a different name will fail
the gate-shape assertion above instead.

The INITIAL derive must stay unconfined for a second and independent reason:
a run whose map arrives after the hold would otherwise leave it with no pair
at all, every arming would refuse, and the rendezvous arm would be a silent
copy of `off`.
```

## Gen22StartHold.TheHandshakeRunsAboveHeartbeatTicksEarlyReturns

### start-hold-handshake-during-hold

**The handshake runs during the hold** — attached to `TEST(Gen22StartHold, TheHandshakeRunsAboveHeartbeatTicksEarlyReturns) {` (line 573)

```text
THE AGREEMENT HANDSHAKE RUNS DURING THE HOLD.

Not an edit — an invariant the hold silently depends on, which is exactly
the kind that rots. heartbeatTick calls maintainRendezvousProposal ABOVE
both of its early returns (`!have_active_intent_` and the state list), and
WAIT_FOR_MAP is in neither list, so the handshake keeps running while the
robot is held. If a later edit hoists a state guard to the top of
heartbeatTick — a plausible "don't beat before we've started" cleanup — the
hold inverts from a fix into a 60 s cost paid by all four arms for nothing,
and every other test in this file still passes.
```

## TEST (part 2)

### start-hold-wait-for-map-tests-below

**WAIT_FOR_MAP tests sit below the handshake** — attached to `for (size_t at = body.find("State::WAIT_FOR_MAP");` (line 613)

```text
EVERY WAIT_FOR_MAP TEST IN THIS FUNCTION MUST SIT BELOW THE HANDSHAKE.

This assertion was `countOf(body, "State::WAIT_FOR_MAP") == 0` until
2026-09-17, and it fired — correctly — on GROUP E's suppression-WARN gate,
which is a WAIT_FOR_MAP test in this function that is NOT a hazard. So the
blanket ban is relaxed to a positional one, which is what the hazard
actually is: the dangerous edit is a state guard hoisted ABOVE
maintainRendezvousProposal ("don't beat before we've started"), because
that kills the agreement round for the whole hold and every other test in
this file still passes. A WAIT_FOR_MAP test below the handshake cannot do
that, by construction — the call has already returned.

Relaxing an assertion because your own edit tripped it is the move that
turns a test suite into decoration, so the relaxation is deliberately the
SMALLEST one that admits the new gate: not "allow WAIT_FOR_MAP", but
"allow it only where it provably cannot skip the handshake".
```

## Gen22StartHold.TheHoldIsADeclaredAndValidatedParameter

### start-hold-param-validated

**The hold parameter is validated** — attached to `TEST(Gen22StartHold, TheHoldIsADeclaredAndValidatedParameter) {` (line 646)

```text
THE HOLD IS A DECLARED AND VALIDATED PARAMETER.

The validation is not boilerplate. `held_for >= mission_start_hold_sec_` is
false for every finite held_for when the right-hand side is NaN, so a NaN
hold is an infinite hold — the robot never leaves WAIT_FOR_MAP and the cell
burns its whole wall-clock budget looking like a dead mapper. A negative
hold is the opposite and worse: it is true immediately, so the run is
generation 21 wearing a generation 22 manifest.
```

## Gen22StartHold.TheMemberAndParameterDefaultsAgreeAndAreOn

### start-hold-defaults-agree

**Member and parameter defaults agree** — attached to `TEST(Gen22StartHold, TheMemberAndParameterDefaultsAgreeAndAreOn) {` (line 675)

```text
THE MEMBER DEFAULT AND THE PARAMETER DEFAULT AGREE, AND BOTH ARE ON.

Two failure modes, one test:

  * DISAGREEING defaults mean the hold depends on which construction path
    ran. Any node built without the parameter — a bench, a unit harness, a
    future launch file — silently gets the member's value, and "the
    campaign got the fix, the reproduction didn't" is the hardest class of
    result to debug because both runs report the same generation.
  * A default of ZERO is the deliberate opposite of the two mission-return
    knobs beside it, which default OFF so this binary reproduces banked
    behaviour unless a campaign opts in. This one defaults ON because it
    closes a defect that split a fleet: a campaign that forgets the knob
    should get the fix, not the split. That asymmetry is the reason
    generation 22 is a new generation and is not poolable with anything
    earlier.
```

## Gen22StartHold.TheHarnessPassesTheHoldToEveryArm

### start-hold-harness-every-arm

**The harness passes the hold to every arm** — attached to `TEST(Gen22StartHold, TheHarnessPassesTheHoldToEveryArm) {` (line 712)

```text
THE HOLD IS PASSED IN THE UNCONDITIONAL PER-ROBOT ARGV.

run_explo_sim_rviz.sh has two ways to hand a parameter to the node: the one
`ros2 run` argv every robot gets, and the arm-conditional `EXTRA` array that
carries things like `rendezvous_schedule_enable:=true`. The hold must be in
the first. In the second it would apply to the rendezvous and hybrid arms
only, and then the 60 s appears in exactly the two arms under test — a
startup cost reported as a treatment effect, which is the confound the
all-four-arms design was chosen to avoid.

Asserted by membership of the backslash-continued block that carries
`-p reconnect_mode:=$MODE_ARG`, which is the unconditional argv by
definition. Raw text, no comment stripping — see the file banner.
```

## Gen22StartHold.TheHarnessDefaultsTheHoldOnAndRecordsIt

### start-hold-harness-default-manifest

**Harness default and manifest record** — attached to `TEST(Gen22StartHold, TheHarnessDefaultsTheHoldOnAndRecordsIt) {` (line 748)

```text
THE HARNESS DEFAULTS THE HOLD ON AND RECORDS IT IN THE MANIFEST.

`START_HOLD=0` is a legal and documented setting — it restores generation
21 exactly, which is the only honest way to measure what the hold cost. It
must not be the DEFAULT, and whichever value ran must reach the manifest,
because a campaign whose manifest does not state its hold cannot be
compared against one that does.
```

## Gen22StartHold.TheSuppressionWarnIsGatedOutOfWaitForMap

### start-hold-suppression-warn-gate

**The suppression WARN during the hold** — attached to `TEST(Gen22StartHold, TheSuppressionWarnIsGatedOutOfWaitForMap) {` (line 775)

```text
The heartbeat beacon is STATE-GATED: heartbeatTick() publishes an intent
only from NAVIGATE/INTEGRATE/EXPLOIT_*/RETURN_*/PURSUE/PROXIMITY_HOLD/
RETURN_HOME/DONE. Every other state is a silent one, and a silent robot is
read by its peers as MISSING once coord_claim_ttl_sec (5 s) has passed —
indistinguishable, from the receiving side, from a radio outage. The WARN in
the !beaconing branch exists to make that distinguishable from INSIDE the
node, and the analysis uses it to classify each peer-missing window as
suppression rather than outage. It is the only in-node evidence of the
difference.

WAIT_FOR_MAP IS A SILENT STATE, AND GENERATION 22 PARKS EVERY ROBOT IN IT
FOR SIXTY SECONDS. Twelve claim TTLs. Before the gate this test guards, the
WARN therefore fired exactly once on every robot of every cell of every arm
— an unconditional line. That is worse than a missing diagnostic, because a
string that always appears reads as background and gets filtered, and the
filter that hides the sixty startup lines hides the mid-run one that matters.

The gate is `state_ != State::WAIT_FOR_MAP` and it is clock-free on purpose.
WAIT_FOR_MAP is assigned once, at the member initialiser, and there is no
transitionTo(State::WAIT_FOR_MAP) anywhere in the node — GROUP A's
WaitForMapHasOneExitAndStartGuardsIt is the other half of that fact — so the
state IS the predicate "has not started its mission". Writing the gate as
`missionElapsed() < mission_start_hold_sec_` instead would have been a third
site reading the hold's clock, and would go wrong the moment a startup
precondition other than the hold (a late map, a late pose) keeps a robot in
WAIT_FOR_MAP past the hold — which is a case where the WARN is just as
uninformative and would come back.

THE GATE MUST CONSUME THE LATCH, NOT SKIP THE WARN — corrected 2026-09-18
after the first cut shipped and was caught in flight.

The first version tested the state at the WARN site and did nothing else:
    if (!hb_suppress_warned_ && held >= ttl && state_ != WAIT_FOR_MAP)
which is wrong by roughly one second, because the state and the episode do
not end together. The state leaves WAIT_FOR_MAP the moment the hold expires;
the heartbeat stays suppressed until the executor next runs. In between, the
robot is in PLAN with `held` still carrying the full sixty seconds — so a
tick landing in that window saw a passing state guard, an unset latch and
held >= TTL, and printed "Heartbeat suppressed 60.1 s in state PLAN": the
precise line the gate was added to prevent.

IT WAS INTERMITTENT, WHICH IS WORSE THAN ALWAYS. Measured on the gen-22
smoke, it fired in 2 of 6 cells — so checking one clean cell was enough to
conclude the gate worked, and the two that fired looked like genuine
mid-run suppression at t≈60 s rather than a startup artifact.

The correction sets hb_suppress_warned_ = true for the whole eligible branch
and emits the WARN only outside WAIT_FOR_MAP. That works because the latch is
reset when an episode BEGINS, not when one ends, so consuming it exempts this
episode and only this episode; the next suppression clears it and warns
normally. No clock, no second member, and no dependence on the ordering of
two asynchronous transitions — which is what made the first cut fragile.

WHY THE EPISODE ACCOUNTING IS DELIBERATELY LEFT ALONE. hb_suppressed_,
hb_suppress_start_ and the "Heartbeat resumed after %.1f s suppressed" INFO
are untouched, so the hold still leaves exactly one line in the log stating
its measured length. Suppressing the episode as well would have removed the
only direct observation of the hold from the node's own log, which is the
opposite of what this fix is for.
```

## Gen22StartHold.TheSuppressionExemptionIsEpisodeScopedNotInstantScoped

### start-hold-exemption-episode-scoped

**The exemption lasts the whole episode** — attached to `TEST(Gen22StartHold, TheSuppressionExemptionIsEpisodeScopedNotInstantScoped) {` (line 870)

```text
The race this guards is the one that got through the first time. The state
guard alone is satisfiable while still printing the WARN, so a test that
only checks the guard exists — as the one above did on its own — passes on
the broken source. This one pins the SHAPE that makes the exemption last as
long as the episode does.
```

## TEST (part 3)

### start-hold-latch-before-state

**Consume the latch before testing state** — attached to `EXPECT_LT(consume, guard)` (line 896)

```text
THE ASSERTION THAT MATTERS. The latch must be spent BEFORE the state is
tested. If the state test sits in the `if` condition instead, then during
the ~1 s in which the hold has expired but the heartbeat has not yet
resumed, the robot is in PLAN with an unset latch and held >= TTL, and the
WARN fires reporting a 60 s suppression that is really the hold. That is
exactly the regression observed in 2 of 6 gen-22 smoke cells.
```

## Gen22StartHold.TheWaitingToStartWarnFiresOnlyOnAMissingPrecondition

### start-hold-waiting-warn-preconds

**Waiting-to-start WARN only on a fault** — attached to `TEST(Gen22StartHold, TheWaitingToStartWarnFiresOnlyOnAMissingPrecondition) {` (line 916)

```text
The WAIT_FOR_MAP arm has two quite different reasons to stay put: a
precondition has not arrived (map, pose, planning_map), or every precondition
HAS arrived and the pre-mission hold is still running. The first is a fault
and wants a WARN naming the missing input. The second is the design working
and already has two INFO lines of its own — the throttled "Pre-mission hold:
Xs of Ys" and the one-shot "Pre-mission hold complete at t+Xs".

Until 2026-09-18 the WARN was keyed on `!start`, and `start` is the
conjunction of BOTH. So for the whole length of the hold the log carried,
every five seconds, immediately below the INFO stating the hold was
progressing normally:

    Waiting to start: map=1 pose=1 planning_map=1 (0 = not yet received;
    planning_map required).

A line that prints three satisfied preconditions and calls itself waiting for
them. It names no fault, suggests no next step, and directly contradicts the
line above it. At ~11 repeats per robot per cell it was also the single most
frequent WARN in a gen-22 log. The cost of that is not the bytes: it is that
someone debugging a genuinely stuck startup has to first work out that the
loudest warning in the file means nothing.

The fix gates it on !preconds_met, which is the fault condition alone.
```
