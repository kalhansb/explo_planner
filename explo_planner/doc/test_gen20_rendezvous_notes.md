# test_gen20_rendezvous.cpp — design notes and history

The long comments of `test/test_gen20_rendezvous.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [stripComments](#stripcomments) — 1
- [functionBody](#functionbody) — 1
- [assignedFrom](#assignedfrom) — 1
- [assignmentsOf](#assignmentsof) — 1
- [Gen20DeadlineWrite.TheDeadlineIsWrittenOnEveryPathThatArms](#gen20deadlinewritethedeadlineiswrittenoneverypaththatarms) — 1
- [Gen23AgreedSchedule.TheDeadlineIsTheAgreedOccurrenceNotACountdown](#gen23agreedschedulethedeadlineistheagreedoccurrencenotacountdown) — 1
- [TEST](#test) — 1
- [Gen29Punctuality.TheDepartureLeadsTheMeetingAndDecidesNoRung](#gen29punctualitythedepartureleadsthemeetinganddecidesnorung) — 1
- [TEST (part 2)](#test-part-2) — 1
- [Gen29Punctuality.TheLeadIsTheSchedulersOwnMarkupAndBothEndsAreScreened](#gen29punctualitytheleadistheschedulersownmarkupandbothendsarescreened) — 1
- [Gen20SeparationPredicate.EveryDecisionSiteAsksTheSameQuestion](#gen20separationpredicateeverydecisionsiteasksthesamequestion) — 1
- [TEST (part 3)](#test-part-3) — 2
- [Gen20AppointmentManoeuvre.SetOnceClearedOnceAndNotInCloseAppointment](#gen20appointmentmanoeuvresetonceclearedonceandnotincloseappointment) — 1
- [Gen20AppointmentManoeuvre.TheWaitCapAndSettleHoldAreKeyedOnTheManoeuvre](#gen20appointmentmanoeuvrethewaitcapandsettleholdarekeyedonthemanoeuvre) — 1
- [Gen20SpentLatch.IsSetWhereTheAppointmentArms](#gen20spentlatchissetwheretheappointmentarms) — 1
- [Gen23AgreedSchedule.EveryCommitRequiresEveryEcho](#gen23agreedscheduleeverycommitrequireseveryecho) — 2
- [Gen29Timetable.TheLatticeAndTheBarrierAreSeparateKnobs](#gen29timetablethelatticeandthebarrierareseparateknobs) — 2
- [Gen29ReAgreement.TheMeetingAsksForTheNextOne](#gen29reagreementthemeetingasksforthenextone) — 1
- [TEST (part 4)](#test-part-4) — 4
- [Gen29Timetable.AZeroIntervalIsNotAValidPair](#gen29timetableazerointervalisnotavalidpair) — 1
- [Gen29ReAgreement.TheTeamStandsOnTheCellUntilTheNextPairCommits](#gen29reagreementtheteamstandsonthecelluntilthenextpaircommits) — 1
- [TEST (part 5)](#test-part-5) — 2
- [Gen29ReAgreement.TheProposerReopensTheDeriveAndSpendsItOnAdoptionOnly](#gen29reagreementtheproposerreopensthederiveandspendsitonadoptiononly) — 1
- [TEST (part 6)](#test-part-6) — 3

## stripComments

### scan-strip-comments

**Why scans strip comments first** — attached to `std::string stripComments(const std::string& text) {` (line 87)

```text
The source with `//` comments removed, string literals preserved.

EVERY SCAN IN THIS FILE RUNS ON THIS, and the first draft of the file did
not, which cost it two false results in opposite directions. This node's
comments quote the code they are about — one of them says, verbatim,
"Keyed on appointment_manoeuvre_, NOT on appointment_armed_", and another
carries a commented-out `rendezvous_spent_ = false;` as the example of a
clear that broke a generation. A raw-text scan reads the first as the code
doing the thing it is denying and the second as the assignment itself, so a
correct node fails and a reverted one passes. Prose is not evidence about
code; strip it before asserting anything.

The quote tracking is not decoration: dropping from `//` unconditionally
would truncate any line holding a string with a `//` in it and silently
remove code from the scan's view, which is the same failure one layer down.
```

## functionBody

### scan-function-body

**functionBody returns empty on miss** — attached to `std::string functionBody(const std::string& text, const std::string& signature) {` (line 133)

```text
The body of a function definition, by brace matching from its signature.

Returns "" when the signature is not found, which every caller ASSERTs on: a
scan that silently matches nothing passes every assertion made about what it
did not find, and that is the failure mode this file exists to avoid.
```

## assignedFrom

### scan-assigned-from

**assignedFrom tolerates alignment** — attached to `std::string assignedFrom(const std::string& text, const std::string& lhs) {` (line 182)

```text
The right-hand side of the statement assigning `lhs`, or "" if `lhs` does
not appear or its first appearance is not an assignment. Every caller
ASSERTs on the empty string, for the reason functionBody does.

WHITESPACE IS WHY THIS EXISTS. The assertions below are of the form "this
field is fed from that one", and the node column-aligns its assignments and
wraps long right-hand sides — so the literal needle a reader would write
(`cfg.min_interval_ms = rendezvous_interval_sec_`) matches nothing, and the
one that does match today stops matching the first time clang-format moves a
line break. A test that silently stops looking is worse than no test.
```

## assignmentsOf

### scan-assignments-of

**Finding writes, not reads** — attached to `std::vector<size_t> assignmentsOf(const std::string& hay, const std::string& needle) {` (line 204)

```text
Every byte offset at which `needle` is ASSIGNED, as opposed to merely read.

The one-write assertions in this file are usually about a literal, so
countOf is enough for them. Generation 29's flag is read in two places and
written in one, and WHICH of the three is the write is the entire contract —
a clear that migrates from the adoption up to the attempt reads identically
to countOf and silently drops every re-agreement whose derive came back
provisional.
```

## Gen20DeadlineWrite.TheDeadlineIsWrittenOnEveryPathThatArms

### deadline-written-before-arm

**The deadline is written before arming** — attached to `TEST(Gen20DeadlineWrite, TheDeadlineIsWrittenOnEveryPathThatArms) {` (line 236)

```text
THE DEADLINE MUST BE WRITTEN ON EVERY PATH THAT ARMS, and this is the test
test_endpoint.cpp's banner says does not exist.

The hazard, stated exactly: `appointmentDue()` compares the clock against
`appointment_.t_meet_ms` guarded by `appointment_armed_`, and
`RendezvousPlan::t_meet_ms` defaults to -1. An armed slot carrying that
sentinel is therefore DUE FOREVER — the robot departs on every tick, for the
rest of the run, and the log records an appointment being kept. Generation
29's departure lead makes that strictly worse rather than better: the test
became `now + lead >= t_meet`, so a larger lead only makes the sentinel fire
sooner. Nothing below depends on which of the two forms is in the file.

What is supposed to prevent it is that `armAppointment` writes the deadline
before it arms, with nothing in between that can skip the write and still
reach the arm. That is a claim about straight-line source, which is why it is
checked as one. Note what does NOT protect it, because a future edit will
reach for these first: `armed` is `ev.refused.empty() && appointment_.valid()`
and `RendezvousPlan::valid()` is `cell >= 0 && refused.empty()`, which never
looks at `t_meet_ms` at all.
```

## Gen23AgreedSchedule.TheDeadlineIsTheAgreedOccurrenceNotACountdown

### deadline-agreed-occurrence

**The deadline is the agreed occurrence** — attached to `TEST(Gen23AgreedSchedule, TheDeadlineIsTheAgreedOccurrenceNotACountdown) {` (line 296)

```text
THE DEADLINE IS THE AGREED OCCURRENCE, NOT A PRIVATE COUNTDOWN.

This test used to assert the exact opposite, and the reversal is the whole of
generation 23. Generations 19-22 set `t_meet_ms` to
`now + rendezvous_depart_delay_sec`, a countdown from the moment THIS robot
noticed the team was incomplete. Every robot ran its own, so the arm labelled
"rendezvous" was N robots independently visiting one cell — an agreed place
and N different times — which is not what the experiment's rendezvous arm is
defined to be.

What the deadline must be now (generation 29): the first occurrence of the
AGREED recurring schedule this robot can still REACH within
rendezvous_max_lateness_sec — floored at `t_now` plus its own shortfall, and
nothing else. Each part is asserted, because each without the others is a
different bug:

  * no floor at all is generation 18's shape: a robot departing for a
    meeting that had already happened;
  * a floor of `t_now + rendezvous_depart_delay_sec_` is generations
    19-24: the shared notice re-applied per robot, whose per-robot sum
    forked the ts4 N=3 cell across two occurrences (two robots kept the
    agreed instant, the third rolled a whole interval past it and its
    peers stood at the cell 351 and 432 s to the censor);
  * a bare `t_now` floor is generation 25-28: correct about the lattice,
    but every robot then signed up to occurrences it could not reach, and
    54 of 54 kept appointments arrived late by their own drive.

THE THIRD IS NOT A RETURN OF THE SECOND, and the difference is one token.
`arrivalShortfallSec` clamps at zero, so a robot that can make the nearest
occurrence contributes EXACTLY nothing to its floor and indexes the lattice
from the same place as every peer that can also make it. The generation-19
term was unconditional, which is why a team that could all attend still
split. That clamp is pinned executably in test_planner_util.cpp; this scan
can only assert that the node asks for it.

The arithmetic itself is NOT scanned for here. It moved into
explo_planner::nextAgreedOccurrence precisely so it could have executable
tests — see test_planner_util.cpp, which pins the lattice property this scan
can only assert is being called.
```

## TEST

### deadline-floor-sign

**The sign of the shortfall floor** — attached to `EXPECT_NE(expr.find("t_now + shortfall_sec"), std::string::npos)` (line 376)

```text
THE SIGN, SCANNED LITERALLY, because it is the only thing standing under
the "deadline already passed" diagnostic that was deleted at the head of
this function as unreachable-by-construction. arrivalShortfallSec clamps at
zero, so `t_now + shortfall` can only ever be at or after t_now; `t_now -
shortfall` puts the floor BEHIND now and hands back occurrences already
past, and every other assertion in this test passes under that flip.
```

## Gen29Punctuality.TheDepartureLeadsTheMeetingAndDecidesNoRung

### departure-lead-no-rung

**Departure lead decides no rung** — attached to `TEST(Gen29Punctuality, TheDepartureLeadsTheMeetingAndDecidesNoRung) {` (line 422)

```text
THE ROBOT LEAVES EARLY ENOUGH TO ARRIVE, AND DECIDES NOTHING ELSE.

Generation 29 splits one question into two, and both halves have to hold or
the arm is not what Kalhan specified ("robots should only agree to meetings
they can come to with a maximum delay", 2026-09-19):

  * WHICH occurrence this robot keeps — decided ONCE, at arming, by the test
    above;
  * WHEN it leaves for it — recomputed every tick here, so a robot that
    drifts or chases away from the cell pulls its own departure earlier and
    the rung it signed up to stays reachable.

THE RATCHET IS WHY THE SPLIT MATTERS. If this function re-chose the rung
instead of just the departure, a robot exploring away from the cell would
grow its lead, roll to a later occurrence, explore further on the strength
of the extra time, and never attend at all — the meeting receding exactly as
fast as the robot leaves. So the scan below is in two parts: the lead must
be here, and the deadline must NOT be written here.

The -1 fallback is asserted for the same reason it exists. Off the snapshot
grid there is no travel estimate, and the robot must still depart — at
t_meet, late by its drive, which is the pre-29 behaviour and the right
degradation because it is the one that needs no estimate. Dropping the
branch would compare against -1 and make an armed appointment due forever.
```

## TEST (part 2)

### departure-ratchet-guard

**The ratchet guard scan** — attached to `for (const size_t at : allOf(body, "appointment_.t_meet_ms")) {` (line 469)

```text
THE RATCHET GUARD. `appointment_` is a member and fully mutable from here;
nothing but this assertion stops a future edit from re-deciding the rung on
the tick that notices the robot cannot make it. Written as "no occurrence
of the field is followed by an assignment" rather than as a literal search,
because the node's own assignments are column-aligned with runs of spaces
and a fixed-spacing needle would miss every realistic edit.
```

## Gen29Punctuality.TheLeadIsTheSchedulersOwnMarkupAndBothEndsAreScreened

### departure-lead-shared-markup

**One markup for lead and lattice** — attached to `TEST(Gen29Punctuality, TheLeadIsTheSchedulersOwnMarkupAndBothEndsAreScreened) {` (line 494)

```text
ONE MARKUP, PRICED THE SAME IN BOTH PLACES THAT READ IT.

The scheduler sizes the lattice spacing so the furthest robot's drive fits,
marked up by rzv_cfg_.depart_safety_milli (rendezvous_scheduler.cpp,
floor_ms). appointmentLeadMs prices this robot's departure with the SAME
constant. If the two ever diverge upward here, a robot needs more notice
than the spacing the team sized for it — and that is the one shortfall a
rung roll cannot fix, because every rung is equally too close.

Scanned rather than executed for the usual reason: these read
rendezvous_world_ and latest_pos_, and explo_planner_node.cpp defines
main(), so nothing in it links into a gtest target. The pure arithmetic that
COULD be lifted already was — see arrivalShortfallSec.
```

## Gen20SeparationPredicate.EveryDecisionSiteAsksTheSameQuestion

### separation-predicate-family

**One predicate family at every site** — attached to `TEST(Gen20SeparationPredicate, EveryDecisionSiteAsksTheSameQuestion) {` (line 542)

```text
ONE QUESTION, ASKED THROUGH ONE FAMILY OF HELPERS, AT EVERY SITE THAT ACTS
ON IT.

Before generation 20 the arming path and the release path asked different
questions about the same fact, so a robot could arm a manoeuvre for a team
the barrier already considered whole, and hold at the meeting point for a
reunion the arming site had not noticed. Unifying them was the fix; this
pins that the unification survives, function by function, because a single
site drifting out of it is invisible at the call site and catastrophic at
run time.

WHAT GENERATION 23 CHANGED, AND WHY THIS TEST HAD TO MOVE WITH IT. Through
generation 22 the unification was the literal token `teamComplete(` at all
five sites, and this test asserted exactly that. Contagious arming split the
one predicate into a rooted family:

    teamSettled(live)              = teamComplete(live, expected)
                                     && !peerReportsTeamBreak()
    manoeuvreReleaseEligible(live) = appointment_manoeuvre_
                                         ? teamSettled(live) ||
                                           teamComplete(reachable, expected)
                                         : teamComplete(live, expected)

(schematic: the appointment branch also carries generation 23's
!peerInboundToAppointment() term with its generation-27 one-hop
peer-report companion, and `reachable` is generation 27's closure count —
the extensions still root in the same two primitives)

so three of the five sites legitimately stopped naming `teamComplete`
directly. Asserting the old token here did not catch a defect — it just went
red against the new design, which is the worst state a guard can be in.

THE DIVISION OF LABOUR WITH test_gen23_contagion.cpp MATTERS. That file's
Gen23ContagionFiveSites pins WHICH member of the family each individual site
must ask, site by site, and is where a `teamSettled` -> bare `teamComplete`
drift fails. Repeating that here would add nothing. What this test owns is
the weaker but differently-shaped claim the per-site tests cannot make: that
no site escapes the family ALTOGETHER, and that the family is still rooted in
one primitive. A site that hand-rolls `live >= rendezvous_expected_peers_`,
or a `teamSettled` rewritten to stop consulting `teamComplete`, satisfies
every per-site assertion and is precisely the generation-18/19 failure
reappearing one layer down.

It asserts PRESENCE, not exclusivity: `rendezvousTeamMutual()` is still the
right predicate for the two quantities that genuinely need both directions
of every link (the anchor stamp and the derive gate), so a blanket ban would
be wrong.
```

## TEST (part 3)

### separation-heartbeat-site-scan

**Scanning the heartbeat site** — attached to `std::vector<size_t> clears;` (line 650)

```text
The fifth site is the heartbeat, which is not its own function — it is the
block that clears rendezvous_spent_ — so it is scanned by its own landmark.

TWO THINGS MATCH THAT LANDMARK AND ONLY ONE OF THEM IS THIS SITE. The
member's own declaration is `bool rendezvous_spent_ = false;`, so a plain
find() lands on it roughly eleven thousand lines early, reads whatever
happens to precede the declaration, and reports on nothing. Take every
occurrence, drop the declaration, and require the rest — because a SECOND
clear added later is exactly the regression worth failing on, and it would
be invisible to a scan that stopped at the first hit.
```

### separation-alias-resolution

**One level of alias resolution** — attached to `const size_t fn_start = text.rfind("\n\nvoid ExploPlannerNode::", guard);` (line 676)

```text
ONE LEVEL OF ALIAS RESOLUTION (generation 23). The guard may name a local
bool rather than call the helper inline — it does today, because the
predicate is now wrapped in a flicker dwell whose window has to be
stepped OUTSIDE this `if` to avoid freezing, so the call sits on its own
statement a few lines above and the condition reads `!appointment_armed_
&& team_back_dwelt`. Scanning only the condition text would then report
"a guard of its own invention" against code that is asking the family
question correctly.

Resolve, do not relax. Every `identifier` in the condition that this
function also DECLARES (`const bool <name> =`) is replaced by its
initialiser, once. A genuinely invented guard still has no family member
anywhere in its definition and still fails. What this cannot follow is a
chain of two aliases, which is the honest limit of a source scan and is
why it is one level and says so.
```

## Gen20AppointmentManoeuvre.SetOnceClearedOnceAndNotInCloseAppointment

### manoeuvre-latch-set-and-clear

**Where the manoeuvre latch changes** — attached to `TEST(Gen20AppointmentManoeuvre, SetOnceClearedOnceAndNotInCloseAppointment) {` (line 726)

```text
SET ONCE, CLEARED ONCE, AND DELIBERATELY NOT CLEARED IN closeAppointment.

`appointment_manoeuvre_` records that the manoeuvre currently in flight was
started to keep an appointment. It is set in `startReturnTo` from the action
string and cleared at the END of the manoeuvre in `transitionTo`.

It must NOT be cleared in `closeAppointment`, and that is the subtle half.
closeAppointment runs when the APPOINTMENT is resolved, which happens on the
tick the robot arrives — while the manoeuvre it started is still running,
still holding at the meeting point, and still needing every decision keyed on
this latch to read true. A clear there is the bug the latch was introduced to
fix, arriving by the other door.
```

## Gen20AppointmentManoeuvre.TheWaitCapAndSettleHoldAreKeyedOnTheManoeuvre

### manoeuvre-latch-keys-wait-cap

**Wait cap keyed on the manoeuvre** — attached to `TEST(Gen20AppointmentManoeuvre, TheWaitCapAndSettleHoldAreKeyedOnTheManoeuvre) {` (line 774)

```text
THE WAIT CAP AND THE SETTLE HOLD ARE KEYED ON THE MANOEUVRE, NOT THE FLAG.

`appointment_armed_` answers "does an appointment stand?" and goes false the
moment the appointment is closed — which is the tick the robot arrives. A
wait cap keyed on it therefore changes value underneath a robot that is
standing at the meeting point waiting for its peers, and the settle hold that
lets the maps merge disappears at the instant it becomes relevant.
```

## Gen20SpentLatch.IsSetWhereTheAppointmentArms

### spent-latch-at-arming

**Spent latch set at arming** — attached to `TEST(Gen20SpentLatch, IsSetWhereTheAppointmentArms) {` (line 826)

```text
THE SPENT LATCH IS SET WHERE THE APPOINTMENT ARMS.

`rendezvous_spent_` is what stops one agreed pair being kept twice in a run.
It has to be set at the arming site — not at departure, not at arrival —
because every later site that could set it is on a path a robot may never
take, and a latch that is only set on the happy path is not a latch.
```

## Gen23AgreedSchedule.EveryCommitRequiresEveryEcho

### agreement-group-scope

**What the agreement group pins** — attached to `TEST(Gen23AgreedSchedule, EveryCommitRequiresEveryEcho) {` (line 858)

```text
Kalhan's rule for this arm, verbatim: "the robots have a predefined time and
place to meet at the start of the mission. if any of the robot is
disconnected, all the robots go to this place at this time. then they share
maps then after they share the maps they decide on the next place and time to
meet. then they start exploring again."

Two of its clauses are pinned mechanisms here: the time is shared, not
private (GROUP A above), and the commit requires every robot (the scan
below). The third — "after they share the maps they decide on the next place
and time to meet" — is generation 29's, and is GROUP G.

WHY SCANS AND NOT BEHAVIOUR: same wall as the rest of this file — the logic
is in explo_planner_node.cpp, which defines main(). The one piece that could
be lifted out was, and has executable tests (nextAgreedOccurrence, in
test_planner_util.cpp). What remains is wiring, and wiring is what a scan can
legitimately check.
```

### agreement-commit-every-echo

**Every commit needs every echo** — attached to `TEST(Gen23AgreedSchedule, EveryCommitRequiresEveryEcho) {` (line 875)

```text
EVERY COMMIT NEEDS EVERY ECHO, AT EVERY N.

This has been reverted twice and restored twice, so the reason it is safe NOW
is recorded at the code site and must be read before it is weakened a third
time. The short form: both measured splits were between a robot holding the
provisional placeholder and a robot holding the real triple, and both were
harmful only because the placeholder carried no meeting time. It carries the
full agreed schedule now, so a commit that fails to reach everyone degrades
to "the team keeps the agreement it already has" — which is the rule, not a
violation of it.

The regression this catches is the exact shape of the previous two reverts:
re-attaching the unanimity requirement to `rendezvous_held_provisional_`, so
that a FINAL triple commits on a partial echo count.
```

## Gen29Timetable.TheLatticeAndTheBarrierAreSeparateKnobs

### reagree-group-scope

**What the re-agreement group pins** — attached to `TEST(Gen29Timetable, TheLatticeAndTheBarrierAreSeparateKnobs) {` (line 931)

```text
Kalhan's requirement, verbatim: "we should make the meeting times more
separate let's say after 300s but robots should come together no matter what
others wait until all are there, and the time to meet is updated before going
to explore again" (2026-09-19).

Three of those four clauses are wiring, and wiring is what this file checks:
the 300 s spacing has to be a knob of its own rather than the proposal
period's side effect, the update has to be REQUESTED by the meeting, and the
proposer has to be allowed to derive a second time. The fourth — "come
together no matter what" — was already true and is GROUP B's: arming is
unconditional on the separation predicate, including the contagion case where
the robot itself can still hear everyone.

The arithmetic these tests do not do is in test_rendezvous_scheduler.cpp,
which executes the solver against the campaign's own two numbers. Everything
here is about which member feeds which, because that is what cannot be
executed from a translation unit that defines main().
```

### timetable-separate-knobs

**Lattice and barrier are separate knobs** — attached to `TEST(Gen29Timetable, TheLatticeAndTheBarrierAreSeparateKnobs) {` (line 949)

```text
THE LATTICE AND THE BARRIER WAIT ARE THEIR OWN KNOBS, and before generation
29 neither was.

The spacing used to be rendezvous_proposal_period_sec — the same 30 s that
paces the derive attempts and bounds the snapshot's staleness, sharing a
number with the timetable for no reason beyond both being periods. That is
what this pins apart, and it is not cosmetic: the departure test is
`now + lead >= t_meet`, a late robot rolls to the next rung, and hybrid
chases only while its appointment is not yet due — so the chase window is
about `interval` minus the lateness budget. On the banked generation-28
armings the lattice came out at 30 s against a 60 s budget, a NEGATIVE
window on 180 of 211 armings, and hybrid never chased once. Putting the
spacing back on the proposal period restores exactly that arm collapse while
every log line still reads correctly.

The cap is the same shape of mistake one field over. It used to be fed from
reconnect_midrun_max_wait_sec — how long a MID-RUN reconnect attempt waits —
which is not the wait a robot keeping an appointment spends. That is
rendezvous_appointment_wait_sec, the barrier, and it is the only one whose
exhaustion can make a meeting unfindable.
```

## Gen29ReAgreement.TheMeetingAsksForTheNextOne

### reagree-after-settle

**Re-agreement after the settle hold** — attached to `TEST(Gen29ReAgreement, TheMeetingAsksForTheNextOne) {` (line 1006)

```text
THE MEETING ASKS FOR THE NEXT ONE, AND ONLY AFTER THE MAPS HAVE MOVED.

"the time to meet is updated before going to explore again" is a statement
about ORDER, so this test is mostly about order. The release site holds the
gathered team for rendezvous_settle_sec while the peer voxels cross the
emulator, and the request has to be raised downstream of that hold — raised
above it, a team that reached the cell and was still exchanging would ask
for its next meeting on the map it arrived with, which is the map the
standing pair was already derived from.

The flag is a request, not the update, and it is raised on every robot while
only the proposer reads it — see the member's own comment for why that is
inert rather than a bug. What matters here is that there is exactly ONE
place that raises it. A second raiser is a meeting the team never had.

THE MERGE COUNTER IS CHECKED HERE because it is the release's only
peer-exclusive evidence: voxels move when the robot's own sensor sweeps the
room it is standing in, and the census hash moves on any status change, but
`applied` counts cells whose local status a PEER changed. Accumulating it
anywhere other than the single mergeWire call site would double-count, and
the "moved nothing at all" WARN would then stop firing on exactly the
exchanges it exists to catch.
```

## TEST (part 4)

### reagree-two-raisers

**The two re-agreement raisers** — attached to `const std::vector<size_t> raises = assignmentsOf(body, "rendezvous_reagree_due_");` (line 1035)

```text
TWO RAISERS, BOTH IN HERE. One at the end of the settle, where an
appointment asks and then stands there for the answer; one in the release
tail, which is all a barrier with no settle stage (a pursuit reunion, or
rendezvous_settle_sec 0) can do. Counted as assignments rather than as a
literal so the count cannot be defeated by whitespace.
```

### reagree-raisers-write-true

**Both raisers must write true** — attached to `for (const size_t at : raises) {` (line 1047)

```text
AND BOTH OF THEM RAISE IT. assignmentsOf answers "written here", not
"written to what": a `true` flipped to `false` at either site keeps the
count at two and the ordering below satisfied, and silently ends
re-agreement — the team goes back out on the pair it just kept and keeps
it for the rest of the run, which is the missing-raiser failure named
above wearing the one disguise this test could not see.
```

### reagree-five-writes

**Five writes of the request flag** — attached to `EXPECT_EQ(assignmentsOf(text, "rendezvous_reagree_due_").size(), 5u)` (line 1060)

```text
Five over the whole file: the member's own default initialiser, which
assignmentsOf cannot tell from a statement, the two raises above, and ONE
CLEAR PER ROLE — the proposer's, where it adopts its own derive, and the
follower's, where it adopts the proposer's. Both clears are required and
neither is reachable from the other's branch, so this is five and not four.
```

### reagree-release-tail-guard

**Guarding the release-tail raiser** — attached to `const std::string guard = "if (!rendezvous_reagree_waiting_) {";` (line 1086)

```text
AND THE RELEASE-TAIL RAISER IS GUARDED (2026-09-20). An appointment asks at
the end of its settle and stands on the cell until the answer commits, so
by the time it reaches the release tail the request has already been SPENT
by the adoption that answered it. Raising it again buys a second derive,
and deriveRendezvousProposal mints t_meet as now+interval — so the instant
the fleet has just agreed to slides forward by however long the release
took, and the run carries two commits for one meeting. The ts4 gen-29 N=2
rendezvous smoke banked exactly that: cell 23 at t+1055s, then t+1061s.
The guard confines the raise to the barriers that had no settle stage in
which to ask — a pursuit reunion, and rendezvous_settle_sec 0.
```

## Gen29Timetable.AZeroIntervalIsNotAValidPair

### timetable-zero-interval-invalid

**A zero interval is not valid** — attached to `TEST(Gen29Timetable, AZeroIntervalIsNotAValidPair) {` (line 1121)

```text
A ZERO-SPACED LATTICE IS NOT A SCHEDULE. nextAgreedOccurrence has no next
occurrence to roll a late robot to when the interval is zero, so it hands
back the agreed instant unchanged and the appointment is due the moment it
arms. armAppointment's deleted "the deadline already passed" diagnostic is
justified in comments by "while the committed interval is positive" — this
is the line that makes that a fact rather than an expectation.
```

## Gen29ReAgreement.TheTeamStandsOnTheCellUntilTheNextPairCommits

### reagree-stand-until-commit

**Stand on the cell until commit** — attached to `TEST(Gen29ReAgreement, TheTeamStandsOnTheCellUntilTheNextPairCommits) {` (line 1137)

```text
"... AND ONLY THEN RESUME EXPLORING" is the clause this pins. Asking for the
next pair and driving away in the same tick satisfies the letter of
TheMeetingAsksForTheNextOne above and none of the intent: the handshake then
runs while the fleet disperses, and the one thing that can lose it — a link
break — is the thing dispersing causes. The team stays on the cell until the
commit lands.
```

## TEST (part 5)

### reagree-release-on-agreed

**The wait releases on the commit** — attached to `const size_t test = body.find("rendezvous_agreed_ == rendezvous_reagree_from_");` (line 1156)

```text
THE RELEASE TEST IS THE COMMIT RULE'S OWN OUTPUT. rendezvous_agreed_ only
moves when every peer has echoed the same triple, and it reads the same on
a follower as on the proposer — unlike rendezvous_reagree_due_, which only
the proposer ever clears and which would therefore hold every follower on
the cell until the bound expired.
```

### reagree-wait-bounded

**The re-agreement wait is bounded** — attached to `EXPECT_NE(body.find("kRendezvousReagreeWaitSec"), std::string::npos)` (line 1170)

```text
BOUNDED. An unbounded wait turns a handshake that cannot land -- a peer
that goes quiet mid-commit -- into an appointment arm that never explores
again, which is a worse failure than leaving on a staler pair: a schedule
rolls forward rather than expiring, so the team that leaves still has a
place and a time.
```

## Gen29ReAgreement.TheProposerReopensTheDeriveAndSpendsItOnAdoptionOnly

### reagree-reopen-and-spend

**Reopen the derive, spend on adoption** — attached to `TEST(Gen29ReAgreement, TheProposerReopensTheDeriveAndSpendsItOnAdoptionOnly) {` (line 1192)

```text
THE REQUEST REOPENS THE DERIVE, AND IS SPENT BY THE ADOPTION — NOT BY THE
ATTEMPT.

One pair per MEETING, which is the generation-29 rule and reads one word
away from the generation-23 rule it replaces (one pair per RUN). The derive
branch is shut between meetings; the three things that open it are no pair
at all, a provisional pair that can be upgraded, and a meeting the team has
just kept. Dropping the third from the gate is a silent revert: everything
still compiles, the release still raises the flag, the log still says the
team met, and the pair simply never changes again.

THE CLEAR IS THE SUBTLE HALF. A release that lands on a momentarily empty
allocator derives a centroid-fallback pair, which `!now_provisional`
correctly declines — and if the flag were cleared at the attempt rather than
at the adoption, that declined attempt would consume the request and the
team would go back out on the pair it had just kept, with nothing left to
retry. So the clear has to sit past the commit of the new pair, and the
provisional guard has to be in the adopt condition rather than only in the
gate. Both are checked by position, because both regressions are single-line
moves that no other assertion in this file would notice.
```

## TEST (part 6)

### reagree-clear-per-role

**One clear per role** — attached to `const std::vector<size_t> cleared =` (line 1246)

```text
TWO CLEARS, one per role. The proposer spends the request when it adopts
its own derive; the follower spends it when it adopts the proposer's. They
are different robots on different branches and neither runs the other's, so
a single clear would leave whichever role lacks it asking forever — and on
the follower "asking forever" means accepting every later generation
unconditionally, which is the fleet split the handshake exists to refuse.
```

### reagree-clears-write-false

**Both clears must write false** — attached to `for (const size_t at : cleared) {` (line 1258)

```text
AND BOTH OF THEM CLEAR IT, which the position assertions below cannot see:
a clear inverted to a raise sits exactly where a clear belongs, passes
every offset comparison in this test and in the follower's, and leaves the
role asking forever — on the follower, "asking forever" is adopt() taking
any later generation unconditionally. Checked here rather than twice,
because both clears live in this one body.
```

### reagree-follower-passes-flag

**The follower passes the request** — attached to `const size_t call = body.find("RendezvousHandshake::adopt(");` (line 1292)

```text
THE DECISION MUST BE TOLD. RendezvousHandshake::adopt refuses final-over-
final by default, and a re-agreement is exactly that shape, so a follower
whose call omits the request flag cannot ever take the replacement. The
proposer still adopts its own, the commit gate never reaches unanimity, and
both appointment arms silently re-meet at the t=0 cell for the whole run.
```
