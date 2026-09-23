# test_rendezvous_scheduler.cpp — design notes and history

The long comments of `test/test_rendezvous_scheduler.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [world5](#world5) — 1
- [tours2](#tours2) — 1
- [RendezvousObjective.PicksTheZeroPenaltyNearMissAndTimesItByTheSlowerArrival](#rendezvousobjectivepicksthezeropenaltynearmissandtimesitbytheslowerarrival) — 1
- [RendezvousObjective.PenaltyIsNeverNegativeForAnyCandidate](#rendezvousobjectivepenaltyisnevernegativeforanycandidate) — 1
- [RendezvousObjective.RecomputedRouteCostMatchesTheAllocatorsOwn](#rendezvousobjectiverecomputedroutecostmatchestheallocatorsown) — 1
- [RendezvousFloor.WinsWhenNoTourCellIsWorthItsDetour](#rendezvousfloorwinswhennotourcellisworthitsdetour) — 1
- [RendezvousFloor.WithNoFloorAnExactTieBreaksToTheLowerCellId](#rendezvousfloorwithnoflooranexacttiebreakstothelowercellid) — 1
- [RendezvousCrossPerspective.IdenticalAppointmentFromBothSides](#rendezvouscrossperspectiveidenticalappointmentfrombothsides) — 1
- [TEST](#test) — 1
- [RendezvousTravel.IsIntegerAndRefusesANonPositiveSpeed](#rendezvoustravelisintegerandrefusesanonpositivespeed) — 1
- [RendezvousAdmissibility.AnUnreachableCellIsRejectedAndCounted](#rendezvousadmissibilityanunreachablecellisrejectedandcounted) — 1
- [RendezvousAdmissibility.TheFloorSurvivesAnUnreachableVerdict](#rendezvousadmissibilitythefloorsurvivesanunreachableverdict) — 1
- [RendezvousNoShow.AnExcludedCellIsDroppedAndTheArgminMovesOn](#rendezvousnoshowanexcludedcellisdroppedandtheargminmoveson) — 1
- [RendezvousNoShow.ExclusionCanNeverEmptyTheCandidateSet](#rendezvousnoshowexclusioncanneveremptythecandidateset) — 1
- [detourTours](#detourtours) — 2
- [RendezvousCap.PullsTheIntervalInAndAdvertisesThatItDid](#rendezvouscappullstheintervalinandadvertisesthatitdid) — 1
- [TEST (part 2)](#test-part-2) — 1
- [RendezvousCap.CannotCutBelowTheReachabilityFloor](#rendezvouscapcannotcutbelowthereachabilityfloor) — 1
- [RendezvousCap.ACapExactlyOnTheTourTermIsNotACut](#rendezvouscapacapexactlyonthetourtermisnotacut) — 1
- [RendezvousFloorInterval.DirectDriveWithSafetyRaisesThePeriod](#rendezvousfloorintervaldirectdrivewithsafetyraisestheperiod) — 1
- [RendezvousFloorInterval.TheCampaignLatticeHoldsAndTheUncappedBarrierNeverBinds](#rendezvousfloorintervalthecampaignlatticeholdsandtheuncappedbarrierneverbinds) — 1
- [RendezvousCap.ABarrierWaitUnderTheLatticeIsOverruledAndCappedDoesNotSaySo](#rendezvouscapabarrierwaitunderthelatticeisoverruledandcappeddoesnotsayso) — 1
- [RendezvousRefusal.UnconfiguredWorld](#rendezvousrefusalunconfiguredworld) — 1
- [Refusals — every one of them a stated answer, none of them a silent -1](#refusals--every-one-of-them-a-stated-answer-none-of-them-a-silent--1) — 1
- [Triple — declarations](#triple--declarations) — 1
- [TEST (part 3)](#test-part-3) — 6

## world5

### rzv-fixture-world5

**The 5x5 scheduler test world** — attached to `CellWorld world5(int self = 0) {` (line 25)

```text
A 5x5 world of 10 m cells over [-25, 25]^2. Straight out of configure(),
the graph is FULLY connected, so distance() is centroid distance and every
number below is hand-checkable: cell id = row*5 + col, centre =
(-20 + 10*col, -20 + 10*row). Row 0 is therefore cells 0..4 at x =
-20, -10, 0, 10, 20 — ten metres apart, which is why the fixtures live
there.
```

## tours2

### rzv-fixture-hand-built-tours

**Hand-built allocations in scheduler tests** — attached to `Allocation tours2(const CellWorld& w, const std::vector<AllocRobot>& robots,` (line 53)

```text
Hand-build an allocation. Most of these tests are about the OBJECTIVE, not
about which tours the allocator happens to produce, and a hand-built pair of
tours makes every penalty in the comments arithmetic a reader can check.
The one test that must not do this — cross-perspective identity — solves for
real, from both sides, through the wire codec.
```

## RendezvousObjective.PicksTheZeroPenaltyNearMissAndTimesItByTheSlowerArrival

### rzv-objective-zero-penalty-case

**The zero-penalty meeting, hand-computed** — attached to `TEST(RendezvousObjective, PicksTheZeroPenaltyNearMissAndTimesItByTheSlowerArrival) {` (line 73)

```text
P5 gate (b). Row 0, ten metres between neighbours:

  robot 0 at cell 0, tour [1, 2]   base cost 10 + 10 = 20 m
  robot 1 at cell 4, tour [3]      base cost 10 m
  base makespan = 20 m

  c=1: r0 already has it (20 m, reaches at 10 m); r1 best insert is [3,1]
       = 10 + 20 = 30 m, reaching 1 at 30 m.  makespan 30, penalty 10 m.
  c=2: r0 already has it (20 m, reaches at 20 m); r1 best insert is [3,2]
       = 10 + 10 = 20 m, reaching 2 at 20 m.   makespan 20, penalty 0.
  c=3: r1 already has it (10 m, reaches at 10 m); r0 best insert is
       [1,2,3] = 30 m, reaching 3 at 30 m.     makespan 30, penalty 10 m.

So cell 2 wins with penalty ZERO — the meeting costs the team nothing,
which is the whole claim of §3.5 — and t_meet is the slower arrival, 20 m
at 1 m/s = 20 s. Not a formula with constants: the tours determined it.
```

## RendezvousObjective.PenaltyIsNeverNegativeForAnyCandidate

### rzv-penalty-never-negative

**Why the penalty is never negative** — attached to `TEST(RendezvousObjective, PenaltyIsNeverNegativeForAnyCandidate) {` (line 107)

```text
The penalty is a DIFFERENCE of makespans over the same cost function, so it
can never be negative: inserting a cell into one robot's tour can only
lengthen that tour, and the other tours are untouched. A negative penalty
would mean the two sides of the subtraction came from different cost
models, which is the drift routeCostMm was made public to prevent.
```

## RendezvousObjective.RecomputedRouteCostMatchesTheAllocatorsOwn

### rzv-route-cost-matches-allocator

**Recomputed route cost matches the allocator** — attached to `TEST(RendezvousObjective, RecomputedRouteCostMatchesTheAllocatorsOwn) {` (line 124)

```text
The base makespan this file computes and the one the allocator reports must
be the same number. They are produced by different code paths — recomputed
from the tours here, accumulated during insertion there — and if they ever
disagree the penalty is a difference between a polished cost and an
unpolished one, which is invisible in every log.
```

## RendezvousFloor.WinsWhenNoTourCellIsWorthItsDetour

### rzv-floor-wins-on-detour

**The floor wins when detours are costly** — attached to `TEST(RendezvousFloor, WinsWhenNoTourCellIsWorthItsDetour) {` (line 147)

```text
P5 gate (a). Both tours point AWAY from the other robot, so every tour cell
costs a 40 m detour:

  robot 0 at cell 0 (-20,-20), tour [20] (-20, 20)  -> 40 m
  robot 1 at cell 4 ( 20,-20), tour [24] ( 20, 20)  -> 40 m, makespan 40 m

  c=20: r1 must add it. Best is [24,20] = 40 + 40 = 80 m. penalty 40 m.
  c=24: symmetric.                                    penalty 40 m.
  floor=2 (0,-20), the midpoint of the two robots: each best-inserts it
       FIRST, 20 + 44.72 = 64.72 m.                   penalty 24.72 m.

The floor wins, and that is the guarantee that matters: when no tour cell is
worth its detour, hybrid degrades to exactly the destination the shipped
implementation already drives to, so this arm cannot come out worse than the
current one on fallback cost.
```

## RendezvousFloor.WithNoFloorAnExactTieBreaksToTheLowerCellId

### rzv-tie-breaks-lower-cell-id

**Exact ties break to the lower cell id** — attached to `TEST(RendezvousFloor, WithNoFloorAnExactTieBreaksToTheLowerCellId) {` (line 180)

```text
The same fixture with no floor available (the midpoint fell outside the
ROI). Cells 20 and 24 tie at exactly 40 m of penalty, and the tie breaks to
the LOWER CELL ID — a value both robots agree on. "Whichever the scan
reached first" is not a tie-break; it is a dependency on iteration order,
and it is the failure this whole family of tests exists to catch.
```

## RendezvousCrossPerspective.IdenticalAppointmentFromBothSides

### rzv-cross-perspective-scope

**What the cross-perspective test proves** — attached to `TEST(RendezvousCrossPerspective, IdenticalAppointmentFromBothSides) {` (line 221)

```text
P5 gate (c). READ THE SCOPE CHANGE FIRST.

This test used to carry the ENTIRE agreement argument: the v4 design had no
proposal, no echo and no adoption, and agreement was supposed to be a
CONSEQUENCE of both robots running identical arithmetic over a shared world.
Gen 10 falsified that — not the arithmetic, which this test still confirms,
but the premise that the two robots feed it the same inputs. In the field
each merges its own map and 21 of 64 separated pairs picked the same cell.
Agreement is now an explicit propose/echo/commit handshake in the node (see
rendezvous_scheduler.hpp's header and the node's P5 state block), and this
file does not test it — the handshake has no presence in this library.

What the test still proves, and why it is still worth running: that the
SEARCH is a pure function of the world, so a proposal is auditable offline
and a disagreement can be attributed to divergent inputs rather than to the
solver. It must be proved HERE rather than inherited from the allocator's
own determinism test — the scheduler adds an argmin, an insertion search and
a time conversion, any of which could reintroduce a cross-process difference
the allocator does not have.

So: two worlds built from opposite viewpoints, reconciled ONLY by exchanging
wire messages, and the vehicle set handed in reversed on one side to prove
the caller's vector order cannot reach the result. Anything that survives
the round trip as a local-only difference — a *_BY_OTHERS provenance byte, a
known_by mask, a local update_id — gets its chance to move the meeting.
```

## TEST

### rzv-cross-perspective-premise

**Non-empty tours before the conclusion** — attached to `ASSERT_FALSE(aa.tours[0].empty());` (line 270)

```text
The premise, checked before the conclusion. Two empty tours would leave
only the floor on both sides, and this test would then agree with itself
for free while proving nothing about the argmin, the insertion search or
the time conversion — the three things the scheduler adds on top of the
allocator's own determinism.
```

## RendezvousTravel.IsIntegerAndRefusesANonPositiveSpeed

### rzv-departure-rule-removed

**Where the departure rule went** — attached to `TEST(RendezvousTravel, IsIntegerAndRefusesANonPositiveSpeed) {` (line 298)

```text
Four tests stood here pinning RendezvousScheduler::shouldDepart: departures
staggered by each robot's own travel so the ARRIVALS coincided, the safety
factor and margin moving only the departure, an overdue deadline departing
immediately, and -1 inputs refusing rather than reading as overdue. They are
deleted with the function. Generation 29 restored the BEHAVIOUR they
described — staggered departures, coincident arrivals — but not in this
class: it is ExploPlannerNode::appointmentDue, aiming at an instant the team
agreed rather than at a deadline each robot derived.

WHAT TOOK ITS PLACE, and where its tests are. A robot whose reconnect
trigger fires signs up to the first occurrence of the committed
(t_meet, interval) it can still arrive at within rendezvous_max_lateness_sec,
leaves in time to be there, and waits until the team is whole. The properties
that replace these tests — that every robot's t_meet is nextAgreedOccurrence
of the ONE committed pair, that the floor only ever rises above bare now by a
robot's own shortfall, and that the residual spread is absorbed by an
unbounded barrier — are node-level, and the expressions are pinned in
test_gen20_rendezvous.cpp
(Gen23AgreedSchedule.TheDeadlineIsTheAgreedOccurrenceNotACountdown).
```

## RendezvousAdmissibility.AnUnreachableCellIsRejectedAndCounted

### rzv-unreachable-rejected-counted

**Unreachable cells are rejected and counted** — attached to `TEST(RendezvousAdmissibility, AnUnreachableCellIsRejectedAndCounted) {` (line 334)

```text
P5 gate (e). A cell no robot can reach on the roadmap is not a place to
promise to stand at a particular minute, so it leaves the candidate set —
and is COUNTED on the way out, because a silently shrinking candidate set
looks exactly like a world with fewer cells in it.

Note this is the one place that must NOT go through costMm, which
deliberately falls back to centroid distance on an unreachable pair. That
fallback is right for ranking a cell somebody will eventually clear and
wrong for an appointment. mTARE has no check here at all: an unreachable
rendezvous reaches a lookup that calls exit(1) (§3.5.1, edge 4).
```

## RendezvousAdmissibility.TheFloorSurvivesAnUnreachableVerdict

### rzv-floor-survives-unreachable

**The floor survives an unreachable verdict** — attached to `TEST(RendezvousAdmissibility, TheFloorSurvivesAnUnreachableVerdict) {` (line 366)

```text
The floor is admitted even when the graph calls it unreachable. It is the
midpoint of two poses that were in radio contact, the shipped
implementation already drives to it, and an over-aggressive edge probe must
not be able to take away the one destination that is always available.
```

## RendezvousNoShow.AnExcludedCellIsDroppedAndTheArgminMovesOn

### rzv-no-show-not-driven-by-node

**No-show exclusion is library-only** — attached to `TEST(RendezvousNoShow, AnExcludedCellIsDroppedAndTheArgminMovesOn) {` (line 390)

```text
THE NODE NO LONGER DRIVES ANY OF THIS. deriveRendezvousProposal hard-clears
cfg.exclude before every solve, so `rejected_excluded` is 0 on every row a
gen-10 campaign will produce. The write-off list was PER ROBOT, and a
per-robot input to a value the whole team must share is the exact mistake
gen 9 made everywhere else. Anti-deadlock is now the wait cap and the
one-appointment-per-outage rule, neither of which needs anyone's consent.

These stay because the library feature stays and an untested live code path
is worse than a tested unused one. Do not read a green run here as evidence
about a campaign — nothing in a campaign reaches them.
```

## RendezvousNoShow.ExclusionCanNeverEmptyTheCandidateSet

### rzv-exclusion-never-empties-set

**Exclusion can never empty the candidates** — attached to `TEST(RendezvousNoShow, ExclusionCanNeverEmptyTheCandidateSet) {` (line 423)

```text
The rule that makes the no-show handler safe: exclusion can never empty the
candidate set, because it never applies to the floor. Excluding EVERY cell
in the world — including the floor itself — still yields a plan. There is no
sequence of no-shows that produces "nowhere to meet", so there is nothing
for a no-show to deadlock on.
```

## detourTours

### rzv-interval-floor-and-cap

**The interval, its floor and its cap** — attached to `Allocation detourTours(const CellWorld& w, const std::vector<AllocRobot>& r) {` (line 469)

```text
THE INTERVAL IS THE RECURRENCE PERIOD AGAIN (generation 23, after generations
19-22 in which it was not). Every assertion below is unchanged and still
correct — solve()'s arithmetic never moved — but WHAT THE NUMBERS MEAN has
moved twice, and this banner has stated the wrong one before. A robot drives
to the agreed cell for the first occurrence of `t_meet_ms + k*interval_ms`
it can still arrive at (nextAgreedOccurrence; floored at t_now plus that
robot's own shortfall, zero for a robot that can make the nearest one) and
waits there until the team is whole. `interval_ms` is that period, and it is also the agreed,
exchanged, compared integer, read as "how long the furthest robot needs to
get there".

The two bounds survive with it, and both still bite:

  floor: the furthest robot's DIRECT drive, so the integer is a journey
         somebody can actually make. Without it the objective's own winner —
         the cheapest space-time near-miss — reports a grid step, 28.284 s in
         the ts4 smoke, for a meeting that is nothing of the sort.
  cap:   the barrier's own wait: "the last robot to set off can still arrive
         before the ones already waiting give up". With the occurrences back
         and the floor making the interval the furthest robot's direct drive,
         that is also the older reading — "a robot that misses occurrence k
         is under one period behind peers still standing there at k".

The floor outranks the cap. Both are fed by the node, so unlike the
map-divergence cap these replaced, both are live on campaign rows.
```

### rzv-fixture-detour-tours

**The detourTours fixture** — attached to `Allocation detourTours(const CellWorld& w, const std::vector<AllocRobot>& r) {` (line 496)

```text
The fixture for the two cap tests, built so the TOUR TERM IS WELL ABOVE THE
DIRECT DRIVE — which most fixtures are not, because a tour that goes
straight to the meeting has a prefix equal to the direct distance and the
floor then lands exactly on the tour term.

Robot 0 at cell 0 (-20,-20) with tour {20, 2}: it drives 40 m north to cell
20 (-20,20) first and only then 44.72 m down to cell 2 (0,-20), reaching the
meeting at 84.72 m. Robot 1 at cell 4 (20,-20) has no tour, so it inserts
cell 2 at its direct 20 m.

  tour term   = max(84.72, 20)     = 84.72 s at 1 m/s
  direct term = max(0->2, 4->2) = max(20, 20) = 20 s

Cells 20 and 2 both cost zero penalty (robot 0 already holds both and robot
1's makespan stays under its), so the tie breaks to the lower id — cell 2.
```

## RendezvousCap.PullsTheIntervalInAndAdvertisesThatItDid

### rzv-cap-advertises-cut

**The cap and the capped flag** — attached to `TEST(RendezvousCap, PullsTheIntervalInAndAdvertisesThatItDid) {` (line 515)

```text
The cap pulls the interval in to the barrier's wait, and `capped` says it
did. The tour term stays readable in `tour_interval_ms`, which is the only
place the objective's own answer survives — and the difference between the
two is exactly how much of the slowest robot's journey the capped integer no
longer accounts for, recoverable from the plan alone. Read them together
before drawing any distance conclusion from `interval_ms`.
```

## TEST (part 2)

### rzv-t-meet-solve-identity

**t_meet equals derive time plus interval** — attached to `EXPECT_EQ(p.t_meet_ms, 41'000) << "derive time + interval";` (line 538)

```text
solve() writes the two together: t_meet = mission_elapsed + interval, one
assignment site. The NODE does not preserve this identity — arming
recomputes t_meet as the first AGREED OCCURRENCE the robot can reach
(nextAgreedOccurrence), which is t_meet + k*interval for some k >= 0 rather
than mission_elapsed + interval — so the identity is a property of a solve
result only.
```

## RendezvousCap.CannotCutBelowTheReachabilityFloor

### rzv-floor-outranks-cap

**The floor outranks the cap** — attached to `TEST(RendezvousCap, CannotCutBelowTheReachabilityFloor) {` (line 550)

```text
THE FLOOR OUTRANKS THE CAP. A cap below the furthest robot's drive asks the
interval to claim a journey shorter than the one that has to be made; the
number would shrink and the drive would not. It stops at the floor instead.

BOTH FLAGS ARE TRUE HERE, and that pair is the whole point of the test.
`capped` is true because the cap did pull the interval in from the tours'
84.72 s; it just did not get the 5 s it asked for. `floored` is true because
the floor is what refused it. Both true — equivalently `interval_ms` above
the cap — is the shape that says the findability inequality is broken: the
furthest robot cannot reach the cell before the barrier gives up on it, and
the node warns when it sees it rather than leaving it to an analysis.

A derivation that reported NEITHER flag here shipped briefly on 2026-09-18
and this test is what caught it. Do not "fix" a failure of these two
assertions by relaxing them: a plan that says nothing bound, in the one
configuration where two things bound, is a silent null.
```

## RendezvousCap.ACapExactlyOnTheTourTermIsNotACut

### rzv-cap-on-tour-term-not-a-cut

**Cap on the tour term cuts nothing** — attached to `TEST(RendezvousCap, ACapExactlyOnTheTourTermIsNotACut) {` (line 584)

```text
A CAP EXACTLY ON THE TOUR TERM CUT NOTHING, AND MUST NOT SAY IT DID.

The boundary between the two readings of `capped`, and the only mutant the
rest of this group did not kill: `max_interval_ms < tour_interval_ms` versus
`<=`. They differ on exactly one input and it is not a contrived one.

It matters because `capped` is a scored column. A run whose cap happens to
sit on the tour term would report the cap as having chosen the meeting time
when the objective chose it, and the diagnostic that exists to say "the cap,
not the objective, is picking the schedule" would say so falsely. Equality
is reachable rather than measure-zero because the node feeds the cap from a
configured round number — `rendezvous_appointment_wait_sec` since generation
29 — while the tour term is a quantised grid distance at a nominal speed.

The cap is read back off a first solve rather than hardcoded, so the test
stays on the boundary if the fixture's distances ever change.
```

## RendezvousFloorInterval.DirectDriveWithSafetyRaisesThePeriod

### rzv-floor-safety-raises-period

**Safety factor raises the period** — attached to `TEST(RendezvousFloorInterval, DirectDriveWithSafetyRaisesThePeriod) {` (line 641)

```text
THE REACHABILITY FLOOR, on the fixture the objective is happiest with. Both
robots are one cell from the meeting on their own tours, so the tour term is
a single grid step — and the direct drive is that same step, so the floor
binds only once `depart_safety_milli` marks it up. At 1.5x the 20 s drive
becomes a 30 s period.

This is the safety factor doing its stated job at the schedule level rather
than only at the departure test: an occurrence exactly one nominal drive
apart leaves no room for the difference between a straight line on the cell
graph and what a robot actually does through trees.
```

## RendezvousFloorInterval.TheCampaignLatticeHoldsAndTheUncappedBarrierNeverBinds

### rzv-campaign-lattice-uncapped

**The campaign's lattice and uncapped barrier** — attached to `TEST(RendezvousFloorInterval, TheCampaignLatticeHoldsAndTheUncappedBarrierNeverBinds) {` (line 690)

```text
THE CAMPAIGN'S OWN CONFIGURATION, as two numbers rather than as a claim in a
comment: a 300 s lattice (`rendezvous_interval_sec`) against an unbounded
barrier wait (`rendezvous_appointment_wait_sec` = 0, the directive's "be
there until all robots are connected").

Generation 29 exists because of what the spacing does downstream. A robot
signs up to the first rung it can reach inside its lateness budget, and
hybrid chases only while its appointment is not yet due — so the chase
window is roughly `interval_ms` minus that budget. On the banked generation-
28 armings the derived lattice was 30 s and the budget 60 s, which is a
NEGATIVE window on 180 of 211 armings: hybrid armed and never chased once.
The two assertions below are what make the 60 s budget affordable, and they
are about the returned INTERVAL, not about the knob, because it is the
interval the departure test reads.

`capped` false is a contract, not an observation. The node announces it at
startup precisely so a reader finds a constant column and knows it was
configured rather than broken — see the rendezvous_appointment_wait_sec
INFO in ExploPlannerNode's parameter block.
```

## RendezvousCap.ABarrierWaitUnderTheLatticeIsOverruledAndCappedDoesNotSaySo

### rzv-barrier-under-lattice-overruled

**A barrier wait under the lattice** — attached to `TEST(RendezvousCap, ABarrierWaitUnderTheLatticeIsOverruledAndCappedDoesNotSaySo) {` (line 736)

```text
A BARRIER WAIT SHORTER THAN THE LATTICE IS OVERRULED SILENTLY, AND `capped`
IS NOT THE PLACE THAT SAYS SO.

This is not hypothetical arithmetic: it is what the campaign configuration
produces the moment anyone sets RDV_APPT_WAIT to a finite value without also
lowering RDV_INTERVAL below it. The floor outranks the cap, so the answer is
a flat 300 s and the broken findability inequality — the furthest robot
needs longer to arrive than the barrier will wait — holds on every derive.

The trap this pins is which signal notices. `capped` asks a narrower
question than its name suggests: did the cap CUT THE OBJECTIVE'S OWN ASK.
Here the tours asked for 20 s, the cap is 240 s, so the cap cut nothing and
the flag is false — correctly, by the definition the solver's own comment
spends sixty lines establishing, and the alternative reading is the one that
was removed for making `capped` unable to separate "the cap bound" from "the
floor did". So an analyst tallying `capped` over a campaign misconfigured
exactly this way finds a column of zeroes and concludes the barrier was
never overrun.

What DOES notice is the comparison the node's findability WARN makes
directly, `interval_ms > max_interval_ms`, asserted below so that the
predicate keeps meaning what the WARN reads it to mean.
```

## RendezvousRefusal.UnconfiguredWorld

### rzv-occurrence-roll-forward-moved

**Where the occurrence roll-forward went** — attached to `TEST(RendezvousRefusal, UnconfiguredWorld) {` (line 785)

```text
Five tests stood here pinning RendezvousScheduler::occurrenceAtOrAfter — a
future phase returned as-is, at-or-after rather than strictly-after, a phase
long past rolling forward, two robots straddling a boundary landing EXACTLY
one period apart, and a non-positive period returning the phase.

The fourth one is why the whole thing is deleted rather than kept for a rainy
day. It pinned the property the barrier was designed around: "the gap two
robots have to bridge is exactly one period, which the barrier's wait
covers." That is true of the function and was false of the system, because
nothing bounded the arming spread to one period. The N=3 smoke armed at
16.1 / 52.5 / 67.4 s against a 30 s period — 1.7 periods — and the three
robots selected three different occurrences off byte-identical integers.

A green unit test for an arithmetic identity, sitting under a comment
asserting a system property the arithmetic cannot deliver, is worse than no
test: it reads as coverage of the thing that broke.

GENERATION 23 TAKES THE ARITHMETIC BACK AND LEAVES THE CLAIM BEHIND. The
roll-forward lives in planner_util as nextAgreedOccurrence, and the barrier
it feeds is unbounded (rendezvous_appointment_wait_sec = 0), so two robots
on different occurrences now cost each other WAITING at the agreed cell
rather than a missed reunion. Its tests say that under their own names, in
test_planner_util.cpp — including
NextAgreedOccurrence.TwoRobotsOnOneAgreementDifferByWholeIntervals, which
pins the honest version of the property the fourth deleted test overstated.
```

## Refusals — every one of them a stated answer, none of them a silent -1

### rzv-handshake-tests-origin

**Why the handshake is unit-tested** — attached to `namespace {` (line 906)

```text
These pin the two decisions that killed the N>=3 rendezvous arm. Both lived
inside the ROS node until 2026-09-17 and neither could be exercised without
running a 600 s cell, which is how both survived three code reviews.
```

## Triple — declarations

### rzv-latch-test-triple-type

**The Triple stand-in for latch tests** — attached to `struct Triple {` (line 923)

```text
A stand-in for the node's private RendezvousProposal, with the two
operations the latch template needs. Deliberately its own type: if the test
used the node's struct it could not be a unit test, and if the template
silently required more than this it would be depending on something the
caller is not obliged to provide.
```

## TEST (part 3)

### rzv-adopt-reagree-when-due

**Re-agreement lands only when asked** — attached to `EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/false,` (line 985)

```text
GENERATION 29. The team kept the meeting, the maps merged, and the rule's
last clause is that the next place and time are agreed before anyone
resumes exploring. On a follower that arrives as R -> R', which the shape
above refuses — so without the request flag the re-agreement could not land
on anyone but the proposer, the commit gate would never see unanimity, and
both appointment arms would re-meet at the t=0 cell for the whole run while
every follower logged an ERROR.
```

### rzv-adopt-equality-outranks-reagree

**Equality outranks the re-agree request** — attached to `EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/false,` (line 995)

```text
EQUALITY STILL OUTRANKS IT, and this is the case that makes the flag safe
to leave set across ticks. The proposer republishes on every heartbeat, so
between the request and the answer there are many ticks where the numbers
still match. Answering those would spend the request on nothing and leave
the robot holding the old meeting with no outstanding ask.
```

### rzv-adopt-upgrade-outranks-equality

**The upgrade outranks integer equality** — attached to `EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/true,` (line 1011)

```text
The proposer re-derived and landed on the same three integers, this time
with something to choose between. The team does not move, but the run
stops being a placeholder run — and since that is the difference between
"measured the scheduler" and "measured meet-at-the-centroid", the flag has
to clear. kIgnore here would leave this robot reporting the wrong
mechanism forever after.
```

### rzv-adopt-is-total

**Why adopt is tested exhaustively** — attached to `int taken = 0, upgraded = 0, conflicts = 0, ignored = 0, reagreed = 0;` (line 1030)

```text
Exhaustive over all 32 inputs. Not for coverage — for the property that
the function is TOTAL. The version this replaced was an `if` with an
`else if` and no `else`, and the case it silently dropped (hold a
placeholder, peer offers a final one, follower side) was the upgrade
itself: it fell through both branches and did nothing at all.
```

### rzv-latch-false-commit

**A peer moving on clears its latch** — attached to `const Triple held{7, 100};` (line 1087)

```text
THE FALSE-COMMIT BUG, pinned. The peer echoed our placeholder, then
upgraded off it. The old code dropped a record only on pair -> empty, so
this stale record stood, was counted toward fleet-1, and produced a
"Rendezvous AGREED by all" for a triple the peer no longer held — an
appointment it would never attend, followed by a no-show logged against a
robot that was never on that schedule.
```

### rzv-latch-own-upgrade-strands-record

**Our own upgrade leaves peer records** — attached to `const Triple placeholder{9, 240};` (line 1120)

```text
The other direction of the same race, and it is guarded DIFFERENTLY — the
distinction is worth a test because getting it backwards is how the false
commit happened in the first place.

When the PEER moves on, the record is cleared, because nothing downstream
would otherwise notice (test above). When THIS robot moves on, the record
is deliberately left alone: it is still a true statement about the peer,
the peer has not contradicted it, and there is no message to clear it with
— the peers are still happily republishing the placeholder. What stops it
counting is that the commit rule compares each record against what this
robot CURRENTLY holds, so a record naming a superseded triple simply is not
equal to anything the commit is asking about.

This test pins that property, because it is the one the safety of leaving
the record alone rests on. If the commit rule is ever relaxed to counting
valid records instead of matching ones, this is what fails.
```
