# test_gen23_contagion.cpp — design notes and history

The long comments of `test/test_gen23_contagion.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [gossipMsg](#gossipmsg) — 1
- [Gen23ContagionWire.TheBitIsStoredAgainstTheSenderThatMadeIt](#gen23contagionwirethebitisstoredagainstthesenderthatmadeit) — 1
- [Gen23ContagionWire.GossipNeverCarriesTheBit](#gen23contagionwiregossipnevercarriesthebit) — 1
- [Gen23ContagionWire.TheBitHasNoTtlOfItsOwnSoLivenessIsTheConsumersJob](#gen23contagionwirethebithasnottlofitsownsolivenessistheconsumersjob) — 1
- [Gen23ContagionWire.OneWayContactStillCarriesTheBit](#gen23contagionwireonewaycontactstillcarriesthebit) — 1
- [Gen23ContagionWire.TheSenderCanWithdrawItsOwnClaim](#gen23contagionwirethesendercanwithdrawitsownclaim) — 1
- [Gen23ContagionWire.TheHubHearsEveryLeafsBreakFirstHandAtNFour](#gen23contagionwirethehubhearseveryleafsbreakfirsthandatnfour) — 1
- [stripComments](#stripcomments) — 1
- [Gen23ContagionProducer.AnnouncesItsOwnReadAndNeverItsArmedState](#gen23contagionproducerannouncesitsownreadandneveritsarmedstate) — 1
- [Gen23ContagionProducer.AnnouncesNothingWithoutAConfiguredExpectation](#gen23contagionproducerannouncesnothingwithoutaconfiguredexpectation) — 1
- [Gen23ContagionPredicate.BelievesOnlyAPeerItIsCurrentlyReceivingFrom](#gen23contagionpredicatebelievesonlyapeeritiscurrentlyreceivingfrom) — 1
- [Gen23ContagionPredicate.ExemptsAFinishedPeer](#gen23contagionpredicateexemptsafinishedpeer) — 1
- [Gen23ContagionFiveSites.TeamSettledIsTeamCompletePlusTheContagion](#gen23contagionfivesitesteamsettledisteamcompleteplusthecontagion) — 1
- [Gen23ContagionFiveSites.TheArmAndTheSupersedeReadOnePredicate](#gen23contagionfivesitesthearmandthesupersedereadonepredicate) — 1
- [Gen23ContagionFiveSites.TheArmRecordsWhichHalfOfThePredicateFired](#gen23contagionfivesitesthearmrecordswhichhalfofthepredicatefired) — 1
- [Gen23ContagionFiveSites.TheClassifiersReadTheRightHelperForWhatTheyLabel](#gen23contagionfivesitestheclassifiersreadtherighthelperforwhattheylabel) — 1
- [Gen23ContagionFiveSites.BothReturnPathsReleaseOnOneHelper](#gen23contagionfivesitesbothreturnpathsreleaseononehelper) — 1
- [Gen23ContagionFiveSites.TheBarrierNamesAContagionHold](#gen23contagionfivesitesthebarriernamesacontagionhold) — 1
- [Gen23ContagionFiveSites.TheLatchReleaseIsOnTheSamePredicate](#gen23contagionfivesitesthelatchreleaseisonthesamepredicate) — 1
- [Gen23ContagionUnchanged.PursuitStaysOnTheRobotsOwnRead](#gen23contagionunchangedpursuitstaysontherobotsownread) — 1
- [Gen23ContagionUnchanged.ThePresenceClockStaysOnTheRobotsOwnRead](#gen23contagionunchangedthepresenceclockstaysontherobotsownread) — 1
- [Gen23ContagionUnchanged.TheReportedTeamCompleteColumnIsStillTeamComplete](#gen23contagionunchangedthereportedteamcompletecolumnisstillteamcomplete) — 1
- [Gen23ContagionEligibility.OnlyAnAppointmentWaitsForTheWholeTeam](#gen23contagioneligibilityonlyanappointmentwaitsforthewholeteam) — 1
- [Gen23ContagionEligibility.TheReachableDoorOpensOnlyAtTheBarrier](#gen23contagioneligibilitythereachabledooropensonlyatthebarrier) — 1
- [GROUP F. manoeuvreReleaseEligible's asymmetry.](#group-f-manoeuvrereleaseeligibles-asymmetry) — 1
- [Gen23FinishedRelay.LandsOnTheSubjectAndNotOnTheRelay](#gen23finishedrelaylandsonthesubjectandnotontherelay) — 1
- [Gen23FinishedRelay.NeverClearsOnAQuietRelay](#gen23finishedrelayneverclearsonaquietrelay) — 1
- [Gen23FinishedRelay.SurvivesBothTheGossipAgeAndTheDirectTtl](#gen23finishedrelaysurvivesboththegossipageandthedirectttl) — 1
- [Gen23FinishedRelay.TheSubjectsOwnWordOutranksARelayedCopyOfIt](#gen23finishedrelaythesubjectsownwordoutranksarelayedcopyofit) — 1
- [Gen23FinishedRelay.RefusesAnArrayLongerThanTheFleet](#gen23finishedrelayrefusesanarraylongerthanthefleet) — 1
- [Gen23FinishedRelay.ClosesTheNThreeBarrierHangFromTheSeatThatHangs](#gen23finishedrelayclosesthenthreebarrierhangfromtheseatthathangs) — 1
- [Gen23FinishedRelay.ThePublishedBitIsLatched](#gen23finishedrelaythepublishedbitislatched) — 1
- [Gen23FinishedRelay.TheProducerFillsTheArray](#gen23finishedrelaytheproducerfillsthearray) — 1
- [Gen23FinishedRelay.TheConsumerCopiesTheArrayIntoTheObservation](#gen23finishedrelaytheconsumercopiesthearrayintotheobservation) — 1
- [Gen23InboundWire.TheBitIsFirstHandAndNeverRelayed](#gen23inboundwirethebitisfirsthandandneverrelayed) — 2
- [Gen23InboundWire.TheSeenReportLandsUnderItsSenderOnlyAndClears](#gen23inboundwiretheseenreportlandsunderitssenderonlyandclears) — 1
- [TEST](#test) — 1
- [Gen23InboundProducer.TheBitIsKeyedOnDrivingNotOnArrival](#gen23inboundproducerthebitiskeyedondrivingnotonarrival) — 1
- [Gen23InboundProducer.TheSeenReportIsDerivedFromRawBitsOnly](#gen23inboundproducertheseenreportisderivedfromrawbitsonly) — 1
- [Gen23InboundEligibility.TheAppointmentBranchWaitsForAPeerStillDriving](#gen23inboundeligibilitytheappointmentbranchwaitsforapeerstilldriving) — 1
- [Gen23InboundEligibility.ThePredicateHasNoFinishedExemption](#gen23inboundeligibilitythepredicatehasnofinishedexemption) — 1
- [Gen23InboundEligibility.TheDoorVetoCoversTheBridgeHop](#gen23inboundeligibilitythedoorvetocoversthebridgehop) — 1
- [Gen28SettleLapse.OnlyTheConversionSetsTheResumeFlag](#gen28settlelapseonlytheconversionsetstheresumeflag) — 1
- [TEST (part 2)](#test-part-2) — 1

## gossipMsg

### fixture-gossip-msg

**The gossip message fixture** — attached to `TeamModel::Observation gossipMsg(int sender, uint32_t mask,` (line 232)

```text
A message carrying third-party gossip. `heard` is indexed by fleet id and
holds times ON THE SENDER'S CLOCK; the sender's own entry is its publish
time. Duplicated from test_team_model.cpp rather than shared — see the
stripComments note in that file for why these scan binaries stay
independently re-runnable.
```

## Gen23ContagionWire.TheBitIsStoredAgainstTheSenderThatMadeIt

### wire-bit-stored-against-sender

**The bit attaches to its sender** — attached to `TEST(Gen23ContagionWire, TheBitIsStoredAgainstTheSenderThatMadeIt) {` (line 250)

```text
THE BIT ATTACHES TO THE SENDER AND ONLY TO THE SENDER.

The trivial-looking assertion is the one that keeps the design honest: a
claim about the team, stored against the robot that made it, can be
attributed. A claim stored anywhere else is a rumour.
```

## Gen23ContagionWire.GossipNeverCarriesTheBit

### wire-gossip-never-carries-bit

**Gossip never carries the break bit** — attached to `TEST(Gen23ContagionWire, GossipNeverCarriesTheBit) {` (line 271)

```text
GOSSIP NEVER CARRIES THE BIT. THIS IS THE DEADLOCK GUARD.

observe()'s first-hand block copies `team_incomplete` beside `finished`; the
third-party gossip block below it relays positions and last-heard times and
deliberately touches neither. If it relayed this one, C would re-announce
A's break as though it were C's own read, A would see its own bit come
back, and the appointment could never clear — the same deadlock as
announcing the armed state, arriving by a different route.

Both directions are asserted, because they fail differently. A relay whose
OWN view is broken must not smear that onto the robot it is gossiping
about; and a relay whose own view is fine must not clear a bit it was never
told anything about.
```

## Gen23ContagionWire.TheBitHasNoTtlOfItsOwnSoLivenessIsTheConsumersJob

### wire-bit-no-ttl-liveness

**The bit has no TTL of its own** — attached to `TEST(Gen23ContagionWire, TheBitHasNoTtlOfItsOwnSoLivenessIsTheConsumersJob) {` (line 320)

```text
THE BIT HAS NO TTL OF ITS OWN, WHICH IS WHY THE CONSUMER MUST GATE ON
LIVENESS.

This is a property of TeamModel and a requirement on the node in one test.
The stored bit is whatever the peer last said and nothing ages it out — so
a robot that announced a break and then dropped off the air entirely leaves
a `true` sitting in the table forever. Read alone, that is a latch: the team
would stay armed off a message nobody can refresh. What makes it safe is
that `direct` and `heard_one_way` BOTH go false on TTL expiry, and
peerReportsTeamBreak() skips a peer that has neither (GROUP C pins that
side).

Asserting the bit SURVIVES is deliberate, not an accident of the
implementation being tested as-is. Clearing it on expiry would look tidier
and would be wrong: the peer's last known state is still the best available
answer if it comes back inside a moment, and it is the node's liveness gate,
not the storage, that decides whether to believe it.
```

## Gen23ContagionWire.OneWayContactStillCarriesTheBit

### wire-one-way-carries-bit

**One-way contact still carries the bit** — attached to `TEST(Gen23ContagionWire, OneWayContactStillCarriesTheBit) {` (line 355)

```text
ONE-WAY CONTACT STILL CARRIES THE BIT, AND THAT IS THE POINT.

doc/limitations.md §10's failure mode: we receive from the peer, the peer
cannot hear us. Its own read of the team is therefore broken — it is
exactly the robot that needs the meeting — and the announcement is the only
way it has of telling us, because by construction it cannot complete a
handshake. Gating the node's consumer on `direct` ALONE would discard
precisely the case the field exists for.
```

## Gen23ContagionWire.TheSenderCanWithdrawItsOwnClaim

### wire-sender-can-withdraw

**The sender can withdraw its claim** — attached to `TEST(Gen23ContagionWire, TheSenderCanWithdrawItsOwnClaim) {` (line 376)

```text
THE SENDER CAN WITHDRAW IT. ASSIGNMENT, NOT A STICKY OR.

The asymmetric clear only works if the producer's `false` is as
authoritative as its `true`. A consumer that latched on first `true` would
give the fleet a permanent appointment after the first transient dropout
anywhere in it.
```

## Gen23ContagionWire.TheHubHearsEveryLeafsBreakFirstHandAtNFour

### wire-hub-one-hop-n4

**The N=4 hub hears every leaf** — attached to `TEST(Gen23ContagionWire, TheHubHearsEveryLeafsBreakFirstHandAtNFour) {` (line 395)

```text
THE HUB CASE AT N=4, WHICH IS WHERE ONE-HOP SUFFICIENCY EARNS ITS KEEP.

A star: atlas hears bestla, cerd and dvalin and all three name it back, so
atlas's OWN read is complete and generation 22 would have refused to arm.
The three leaves hear only atlas — each sees one peer where it expects
three — so each announces a break, and atlas receives all three FIRST-HAND
because a robot with a complete read is adjacent to everybody. No relay, no
second round, whatever N is.
```

## stripComments

### scan-strip-comments-duplicated

**Why stripComments is duplicated** — attached to `std::string stripComments(const std::string& text) {` (line 440)

```text
The source with `//` and `/* */` comments removed, string literals
preserved. Duplicated from test_gen20/21/22 rather than shared: these are
standalone source-scan binaries, and a shared header between test
executables whose whole job is to be independently re-runnable against a
mutated node would make one mutation break all of them.
```

## Gen23ContagionProducer.AnnouncesItsOwnReadAndNeverItsArmedState

### producer-own-read-not-armed

**The producer announces its own read** — attached to `TEST(Gen23ContagionProducer, AnnouncesItsOwnReadAndNeverItsArmedState) {` (line 540)

```text
THE ANNOUNCED BIT IS `!teamComplete(accountedPeerCount, expected)`,
FIRST-HAND.

`accountedPeerCount`, not `livePeerCount`, and the difference is the whole
reason the announcement terminates. The live count is direct links only, so
a robot that has finished and parked out of radio range would be missing
from it forever and this robot would announce a break for the rest of the
run — with the contagion in place, that is the whole team held at an
uncapped barrier. `accountedPeerCount` counts a peer that is direct, or
finished, or reachable through the two-hop closure, which is the same union
peerAccounted applies everywhere else.

This is the anti-deadlock invariant at its source. Publishing the derived
armed state instead — `appointment_armed_`, or anything downstream of
peerReportsTeamBreak() — closes a cycle: C arms because A announced, C
announces, A holds because C announced. Nothing in a fully reconnected team
could then clear it.

The assertion is therefore as much about what is ABSENT as what is present.
`appointment_armed_` is banned from the function outright: it is derived
from what peers announced, so publishing it on ANY field closes the cycle.
`appointment_manoeuvre_` is banned from THIS field's right-hand side rather
than from the function, because GROUP H's `appointment_inbound` is keyed on
it deliberately and does not close a cycle — a robot can only be held at the
barrier while it is in RETURN_SYNC, where its own inbound bit is false, so
the mutual hold is unreachable and every exit from the drive clears the bit
within nav_budget_sec without waiting on anyone.
```

## Gen23ContagionProducer.AnnouncesNothingWithoutAConfiguredExpectation

### producer-inert-expectation

**An unset expectation announces nothing** — attached to `TEST(Gen23ContagionProducer, AnnouncesNothingWithoutAConfiguredExpectation) {` (line 612)

```text
AN UNCONFIGURED EXPECTATION ANNOUNCES NOTHING.

`teamComplete(x, 0)` is FALSE, so an unguarded `!teamComplete(...)` would
make every single-robot and every rendezvous-inert configuration broadcast
a permanent, unclearable "the team is broken" to a fleet of nobody — and
any listener that did exist would arm on it and never release.
shared_params.yaml documents `rendezvous_expected_peers: 0` as the inert
setting; no expectation means the question has no answer, and the honest
wire value for that is "I am not reporting a break".
```

## Gen23ContagionPredicate.BelievesOnlyAPeerItIsCurrentlyReceivingFrom

### predicate-liveness-gate

**Believe only a peer heard now** — attached to `TEST(Gen23ContagionPredicate, BelievesOnlyAPeerItIsCurrentlyReceivingFrom) {` (line 666)

```text
IT BELIEVES ONLY A PEER IT IS RECEIVING FROM RIGHT NOW — AND BOTH LIVENESS
FLAGS COUNT.

GROUP A's TheBitHasNoTtlOfItsOwn is the other half of this: the stored bit
survives the TTL on purpose, so this gate is the only thing standing
between a silent robot's last message and a permanent appointment.

`heard_one_way` must be accepted alongside `direct`. One-way contact is the
§10 mode, and a robot in it has a genuinely broken read, is announcing so,
and cannot complete a handshake by construction. Requiring `direct` would
discard the case with the strongest claim to a meeting.
```

## Gen23ContagionPredicate.ExemptsAFinishedPeer

### predicate-finished-homing-exempt

**A finished or homing peer never arms** — attached to `TEST(Gen23ContagionPredicate, ExemptsAFinishedPeer) {` (line 716)

```text
A FINISHED PEER DOES NOT ARM THE TEAM.

`finished` already means "no longer someone worth reconnecting to"
(TeamWorld.msg), and the reason bites harder here than anywhere else: the
appointment barrier's wait is UNBOUNDED, so a parked robot that can never
hear one distant peer would hold the whole team at the meeting point until
the run's duration cap and censor the cell. It also buys nothing — map
still flows OUT of a finished robot over any link carrying its scovox_bin,
and it has stopped needing map to flow in.

This is NOT the generation-18/19 split reappearing. That defect was arming
and releasing on DIFFERENT predicates; here the arm and the release both
call this one function, so the exemption weakens both sides identically.
What it does leave — deliberately — is that a finished peer still counts
toward teamComplete, so if it goes SILENT the team arms on the ordinary
rule.

WIDENED IN GENERATION 33 (R4) to the whole off-frontier set, `finished` or
TeamWorld/mode >= MODE_HOMING. Every clause above turns on the peer never
coming to the meeting and no longer needing map to flow in, and both are
true from the moment it TURNS FOR HOME — minutes before `finished`. The
assertions below pin BOTH halves: dropping either one restores the hold
this test exists to prevent, for one of the two ways a robot leaves.
```

## Gen23ContagionFiveSites.TeamSettledIsTeamCompletePlusTheContagion

### five-sites-team-settled-def

**teamSettled is teamComplete plus contagion** — attached to `TEST(Gen23ContagionFiveSites, TeamSettledIsTeamCompletePlusTheContagion) {` (line 772)

```text
teamSettled IS teamComplete PLUS THE CONTAGION, AND NOTHING ELSE.

One definition, so that "change P" is a one-line change and the five call
sites cannot drift apart. The conjunction is the literal reading of the
directive: the team is settled when this robot can hear everyone AND nobody
it can hear says otherwise.
```

## Gen23ContagionFiveSites.TheArmAndTheSupersedeReadOnePredicate

### five-sites-arm-supersede

**The arm and supersede share one predicate** — attached to `TEST(Gen23ContagionFiveSites, TheArmAndTheSupersedeReadOnePredicate) {` (line 795)

```text
1 AND 2. THE ARM AND THE SUPERSEDE, WHICH LIVE IN THE SAME FUNCTION.

`arm on !P, end on P, for one P`. doPlan holds both halves within a few
hundred lines of each other, which is exactly why they are the easiest pair
to break in opposite directions: generation 18 and generation 19 are this
pair disagreeing, each time in a way N=2 could not see.

ONE PREDICATE, TWO EVIDENCE THRESHOLDS (generation 23), and the second half
of that is new. Matching the predicate was only ever half the rule. The arm
fires on the first sample of `!P` — coming apart is shown by a missing
beacon, and a robot that waits for confirmation departs late for a meeting it
has already been told about. The supersede requires `P` to have HELD for
reconnect_release_confirm_sec, because coming back is shown by a single
arriving claim inside a 5 s TTL, which a range-edge flicker produces exactly
as readily as a reunion — and cancelling on it strands a peer already driving
to the cell. Until 2026-09-18 this site had no dwell while the barrier
guarding the same reunion refused the same evidence for 6 s.

dwellHeld, NOT dwellConfirmed, and the test pins that too. The window is
written once per heartbeat; this block sits inside `appointment_armed_ &&
!appointment_departed_`, and doPlan is entered only in State::PLAN — roughly
once per step, not at 10 Hz. A continuity window ticked from here would
measure "two consecutive plan entries agreed", which two samples thirty
seconds apart satisfy trivially, and between appointments it would not decay
at all. See FlickerDwell.AnUnTickedWindowFreezesRatherThanDecaying.
```

## Gen23ContagionFiveSites.TheArmRecordsWhichHalfOfThePredicateFired

### five-sites-arm-reason-string

**The arm's reason string** — attached to `TEST(Gen23ContagionFiveSites, TheArmRecordsWhichHalfOfThePredicateFired) {` (line 855)

```text
AND THE ARM'S SECOND REASON STRING, WHICH IS HOW THE CONTAGION IS COUNTED.

RendezvousAgreedEvent has no `reason` field and adding one would bump a
schema several banked parsers pin, so the measurement rides the plaintext
"Rendezvous schedule [%s]" line instead. "separation" is this robot's own
read failing — the generation-22 string, unchanged, so banked parsers still
find it. "peer-separation" means this robot can hear everyone and armed
SOLELY because a peer announced a break: it is the C of the bridge, and
counting those is how much the contagion actually did.

The selector must be teamComplete, not teamSettled: under !teamSettled,
teamComplete distinguishes the two causes exactly, whereas teamSettled is
false in both and the string would be constant.
```

## Gen23ContagionFiveSites.TheClassifiersReadTheRightHelperForWhatTheyLabel

### five-sites-transition-classifiers

**The two classifiers in transitionTo** — attached to `TEST(Gen23ContagionFiveSites, TheClassifiersReadTheRightHelperForWhatTheyLabel) {` (line 894)

```text
3. THE TWO CLASSIFIERS IN transitionTo, AND WHY THEY READ DIFFERENT
   HELPERS.

They label DIFFERENT OBJECTS and may legitimately disagree — the
reconnect_end classifier labels the MANOEUVRE that ended, the appointment
classifier labels the APPOINTMENT. The manoeuvre is judged by
manoeuvreReleaseEligible, whose answer depends on whether the manoeuvre was
an appointment; the appointment is judged by teamSettled — its arm and
supersede predicate — OR, since generation 27, by the barrier's own
verdict, captured into `appointment_barrier_released` while
`appointment_manoeuvre_` was still true. Without that second half a
reachable-door release (closure whole, mesh not) would bank as a no-show
with arrived=true, which is the smoke20 sign reversal restored at the site
that writes the event stream.

THERE IS ALSO A HARD REASON, and it is the one that changed this design
mid-implementation: `appointment_manoeuvre_` HAS ALREADY BEEN CLEARED by
the time control reaches the appointment classifier. Calling
manoeuvreReleaseEligible() there would read a `false` flag and silently
degrade to bare teamComplete — the generation-22 predicate, restored at one
of the five sites, invisible in review. The capture is the sanctioned
carrier across that clear, so it has an ordering of its own to pin: the
assertions below stop either from being re-"simplified" back.
```

## Gen23ContagionFiveSites.BothReturnPathsReleaseOnOneHelper

### five-sites-barrier-one-helper

**Both return paths use one helper** — attached to `TEST(Gen23ContagionFiveSites, BothReturnPathsReleaseOnOneHelper) {` (line 983)

```text
4. THE BARRIER. BOTH RETURN PATHS RELEASE ON THE SAME HELPER.

doReturnSync is the barrier proper. doReturnNav's use is DELIBERATELY
REDUNDANT — it is guarded by `!appointment_manoeuvre_`, so `&&`
short-circuits and on that path the helper can only ever evaluate its
teamComplete branch. It is written as the shared helper anyway so the five
sites read as one predicate and a later change to the guard cannot leave an
appointment releasing there on the wrong half of it.
```

## Gen23ContagionFiveSites.TheBarrierNamesAContagionHold

### five-sites-barrier-contagion-log

**The barrier names a contagion hold** — attached to `TEST(Gen23ContagionFiveSites, TheBarrierNamesAContagionHold) {` (line 1016)

```text
AND THE BARRIER SAYS SO WHEN THE CONTAGION IS WHAT IS HOLDING IT.

"Everyone is here and we are still waiting" is indistinguishable from a
broken release in a log. This is the change's main risk made visible: an
unbounded wait under a stricter predicate. The throttled line names the
cause so a smoke reader can tell a contagion hold from a defect.
```

## Gen23ContagionFiveSites.TheLatchReleaseIsOnTheSamePredicate

### five-sites-latch-release-dwell

**The latch release and its dwell** — attached to `TEST(Gen23ContagionFiveSites, TheLatchReleaseIsOnTheSamePredicate) {` (line 1037)

```text
5. THE LATCH RELEASE. THE SITE GENERATION 18 GOT WRONG.

`rendezvous_spent_` is what stops one outage arming twice, and the ONLY
path back from spent is a reunion observed on the heartbeat. Gen 18 armed
on a weaker predicate than it cleared on; a weaker predicate HERE is that
same defect in this generation's terms — the latch would release while a
peer is still announcing a break, and the next arm would fire against a
team that never regrouped.

AND THIS IS THE SOLE WRITER OF THE TEAM-BACK DWELL WINDOW (generation 23).
Three things have to hold together and each fails silently on its own:

  * the predicate is teamSettled (the five-site rule, above);
  * it is DWELT, so a flicker cannot clear the latch and hand a second
    arming to the same outage — the generation-19 ratchet;
  * and dwellConfirmed is called OUTSIDE the `!appointment_armed_` test, as
    a standalone statement. This is the load-bearing detail and the one with
    no signature to protect it. A dwell window that is not ticked does not
    decay, it FREEZES: written as `!appointment_armed_ && dwellConfirmed(…)`
    the clock would stop for the whole life of a standing appointment, and
    the first heartbeat after closeAppointment would find `now - since` far
    past the confirm time and release on ONE sample — a guard that reads as
    a guard and filters nothing. See
    FlickerDwell.AnUnTickedWindowFreezesRatherThanDecaying for the same
    failure at the level of the function itself.
```

## Gen23ContagionUnchanged.PursuitStaysOnTheRobotsOwnRead

### unchanged-pursuit-own-read

**Pursuit stays on the robot's own read** — attached to `TEST(Gen23ContagionUnchanged, PursuitStaysOnTheRobotsOwnRead) {` (line 1128)

```text
PURSUIT STAYS PER-ROBOT. teamComplete AND NOT teamSettled, ON PURPOSE.

Pursuit is a robot chasing the peer IT cannot hear. Widening its trigger to
teamSettled would send a robot that can hear everybody off chasing on
somebody else's behalf, which is not pursuit — it is a rendezvous without a
meeting point. The arms are defined by what they do differently and this is
the line between two of them.

The assertion is a ZERO COUNT over STRIPPED text, and doPursue's own comment
block spells out "teamComplete AND NOT teamSettled, DELIBERATELY" in prose.
Over raw text this test fails against correct code.
```

## Gen23ContagionUnchanged.ThePresenceClockStaysOnTheRobotsOwnRead

### unchanged-presence-clock

**The presence clock stays on teamComplete** — attached to `TEST(Gen23ContagionUnchanged, ThePresenceClockStaysOnTheRobotsOwnRead) {` (line 1154)

```text
THE PRESENCE CLOCK STAYS ON teamComplete, AND heartbeatTick HOLDS EXACTLY
ONE teamSettled.

`team_last_complete_time_` is this robot's connectivity history and several
banked analyses read the columns downstream of it. Redefining it to include
peers' claims would change what every one of those numbers means, silently,
across a generation boundary. The count pins the other direction too: the
only teamSettled in this function is the latch release GROUP D asserts.
```

## Gen23ContagionUnchanged.TheReportedTeamCompleteColumnIsStillTeamComplete

### unchanged-team-complete-column

**The team_complete column keeps its meaning** — attached to `TEST(Gen23ContagionUnchanged, TheReportedTeamCompleteColumnIsStillTeamComplete) {` (line 1187)

```text
THE EVENT FIELD AND THE CSV COLUMN STAY ON teamComplete.

`team_complete` is a named column in banked output. Whatever it is computed
from must not change between generations without the column changing name,
or every cross-generation read of it is comparing two different quantities
that happen to share a heading.
```

## Gen23ContagionEligibility.OnlyAnAppointmentWaitsForTheWholeTeam

### eligibility-appointment-only

**Only an appointment waits for the team** — attached to `TEST(Gen23ContagionEligibility, OnlyAnAppointmentWaitsForTheWholeTeam) {` (line 1216)

```text
A NON-APPOINTMENT MANOEUVRE MUST NOT BE HELD BY SOMEBODY ELSE'S BREAK.

The helper is a ternary on `appointment_manoeuvre_`, and the asymmetry is
the design, not a shortcut. An APPOINTMENT is a team event: everyone was
summoned, so it ends when the team is settled — or, since generation 27,
when every expected peer is reachable in the comms closure; the next test
pins where that door may and may not appear. A pursuit or a mid-run
reconnect is one robot's business with one peer; holding it open because a
third robot announced a break would convert every pursuit into an
unscheduled team meeting — and the pursuit arm is defined by not having
one.
```

## Gen23ContagionEligibility.TheReachableDoorOpensOnlyAtTheBarrier

### eligibility-reachable-door

**The reachable door is barrier-only** — attached to `TEST(Gen23ContagionEligibility, TheReachableDoorOpensOnlyAtTheBarrier) {` (line 1248)

```text
THE REACHABLE DOOR OPENS AT THE BARRIER, AND ONLY AT THE BARRIER.

Generation 27. The gen-26 N=3 smoke parked all three robots at the agreed
cell to the duration cap — pairs 0-1 and 1-2 up 100%, pair 0-2 up 1.3%
through one trunk at 8 m — because the release read the same first-hand
mesh the arm reads, and a gathered A-B-C bridge satisfies the closure
while never satisfying the mesh. The fix widens the RELEASE alone: the
appointment branch of manoeuvreReleaseEligible also opens when every
expected peer is reachable in the comms closure (reachablePeerCount).

The confinement is the test. The arm and the supersede must NOT acquire
the door — in an A-B-C bridge the ends are genuinely partitioned on the
maps the meeting exists to exchange (there is no map relay), so a
closure-keyed arm SUPPRESSES a needed rendezvous, the doctrine at the top
of this file. The latch clear in heartbeatTick must stay on the strict
mesh predicate, because that strictness is what makes releasing weaker
than arming loop-safe. And the producer must stay first-hand, or the
claim travels the cycle GossipNeverCarriesTheBit forbids.
```

## GROUP F. manoeuvreReleaseEligible's asymmetry.

### relay-finished-soundness

**Why relaying finished is sound** — attached to `namespace {` (line 1312)

```text
THE DEFECT THIS GROUP EXISTS FOR, because a test that only pins the fix is
half a test. Presence is counted on THREE channels — peerAccounted takes
`direct`, `finished` or a claim-table entry — and the contagion is read on
ONE: peerReportsTeamBreak skips a peer that is neither `direct` nor
`heard_one_way`, and exempts `finished` only for a peer it can already hear.
So the exemption is applied by the LISTENER, to first-hand knowledge, and a
robot that never hears the finisher has no way to apply it:

  N=3. A latches done out of contact with B. They were adjacent at spawn, so
  B holds `finished=false` for A; nothing ages that out, because the field
  has no TTL by design (TheBitHasNoTtlOfItsOwn). B reads 1 of 2 forever and
  announces team_incomplete forever. C hears both, exempts A correctly, and
  is held open by B. Both drive to the agreed cell and wait at a barrier with
  no cap (rendezvous_appointment_wait_sec = 0) that neither can release, to
  the duration cap, in every cell where an ordinary thing happens. It is also
  a REGRESSION: before generation 23 the barrier released on bare
  teamComplete, which A's stale-but-present entry satisfied.

WHY RELAYING THIS ONE FIELD IS SOUND WHEN RELAYING team_incomplete IS NOT.
Two independent properties, and the bit needs both:

  MONOTONIC — the publisher latches it (finished_announced_), so a relayed
  copy can never contradict a first-hand one and the merge is a pure OR that
  needs no arbitration, no freshness and no tie-break.

  ABOUT ITSELF — it is not a claim about the team, so there is no cycle for
  it to travel round. C relaying "A is finished" says nothing about C, so A
  cannot receive its own claim back wearing C's name. That is precisely the
  deadlock GossipNeverCarriesTheBit forbids for team_incomplete, and the
  reason that test stays true while these ones exist.

The bit only ever REMOVES a robot from the set the team waits for, and it is
idempotent, so the worst a spurious relay can do is release a barrier early
— never hold one open, which is the failure with no bound.
```

## Gen23FinishedRelay.LandsOnTheSubjectAndNotOnTheRelay

### relay-lands-on-subject

**A relayed bit lands on its subject** — attached to `TEST(Gen23FinishedRelay, LandsOnTheSubjectAndNotOnTheRelay) {` (line 1363)

```text
A RELAYED BIT IS ATTRIBUTED TO THE ROBOT IT IS ABOUT, NOT TO THE RELAY.

The mirror of TheBitIsStoredAgainstTheSenderThatMadeIt, and it has to come
out the OPPOSITE way: team_incomplete is stored against the sender because
it is a claim the sender is making, `finished` is stored against the subject
because the sender is only carrying it. Getting this backwards would mark
every relay finished and empty the team out of the allocator.
```

## Gen23FinishedRelay.NeverClearsOnAQuietRelay

### relay-pure-or

**The finished relay is a pure OR** — attached to `TEST(Gen23FinishedRelay, NeverClearsOnAQuietRelay) {` (line 1387)

```text
PURE OR. A RELAY THAT STOPS MENTIONING IT DOES NOT CLEAR IT.

The merge skips false entries entirely rather than assigning them, and that
is the whole reason the relay needs no arbitration: two peers relaying
different vintages of the same monotonic fact cannot disagree in a way that
matters. An assignment here would let the LAST message win, so a peer that
had not yet heard the finisher would keep un-finishing it and the team would
oscillate between waiting and not waiting.
```

## Gen23FinishedRelay.SurvivesBothTheGossipAgeAndTheDirectTtl

### relay-not-age-gated

**The finished relay is not age-gated** — attached to `TEST(Gen23FinishedRelay, SurvivesBothTheGossipAgeAndTheDirectTtl) {` (line 1416)

```text
NOT AGE-GATED, UNLIKE EVERY OTHER RELAYED FIELD — AND THAT IS THE POINT.

The position gossip beside it is dropped past gossip_max_age_sec, correctly:
a stale position is a wrong answer. A stale `finished` is still the right
answer, and the age gate would suppress it exactly when it is needed —
a finished robot stops moving, stops planning and goes quiet, so its
last-heard entry ages out at the moment the bit starts to matter.

Both clocks are asserted: the relay's own view of the subject can be
arbitrarily old, and the bit survives our own direct TTL afterwards.
```

## Gen23FinishedRelay.TheSubjectsOwnWordOutranksARelayedCopyOfIt

### relay-first-hand-outranks

**The subject's own word wins** — attached to `TEST(Gen23FinishedRelay, TheSubjectsOwnWordOutranksARelayedCopyOfIt) {` (line 1448)

```text
FIRST-HAND STILL WINS FOR THE SENDER'S OWN ENTRY, INCLUDING A FALSE.

The relay loop skips the sender, so a message from cerd about cerd is an
assignment and can clear. That is deliberate, and its limit is stated here
rather than papered over: the clear is not DURABLE against peers who still
believe the old value, because the next relay re-asserts it. It cannot arise
in a campaign run — the publisher latches, so no process ever publishes
false after true — and the case it is kept for is a node restart, where the
robot itself is the only witness worth believing.
```

## Gen23FinishedRelay.RefusesAnArrayLongerThanTheFleet

### relay-fleet-length-backstop

**Over-long relay arrays are refused** — attached to `TEST(Gen23FinishedRelay, RefusesAnArrayLongerThanTheFleet) {` (line 1480)

```text
THE FLEET-LENGTH BACKSTOP COVERS THE NEW ARRAY TOO.

An over-long array means the sender indexes robots differently than we do,
so entry k is not about the robot we think. For a bit that never clears,
applying one to the wrong robot is permanent: it would park a live robot out
of the allocator for the rest of the run with no way back.
```

## Gen23FinishedRelay.ClosesTheNThreeBarrierHangFromTheSeatThatHangs

### relay-n3-hang-end-to-end

**The N=3 barrier hang end to end** — attached to `TEST(Gen23FinishedRelay, ClosesTheNThreeBarrierHangFromTheSeatThatHangs) {` (line 1497)

```text
THE REGRESSION, END TO END, FROM B'S SEAT. This is the test that would have
failed before the relay existed.

A and B are adjacent at spawn, drift apart, and A latches done while out of
contact. B's entry for A is stale-false and nothing can age it out. C is the
only robot that can carry the news, and one hop is all it has.
```

## Gen23FinishedRelay.ThePublishedBitIsLatched

### relay-published-bit-latched

**The published finished bit is a latch** — attached to `TEST(Gen23FinishedRelay, ThePublishedBitIsLatched) {` (line 1536)

```text
THE PUBLISHED BIT IS A LATCH, NOT A RECOMPUTATION.

This is the precondition for relaying it at all. The pre-generation-23
expression was `coverage_latched_ || state_ == State::DONE`, and only the
first of those is monotonic: State::DONE reverts to EXPLORE when the exploit
sub-loop runs. That path is unreachable for this campaign
(exploitation_enabled: false), and resting a wire invariant on a config
value is how an invariant stops being true without anything looking wrong.
```

## Gen23FinishedRelay.TheProducerFillsTheArray

### relay-producer-fills-array

**The producer fills robot_finished** — attached to `TEST(Gen23FinishedRelay, TheProducerFillsTheArray) {` (line 1572)

```text
THE PRODUCER FILLS THE RELAY ARRAY, AND FILLS IT WITHOUT THE PAIRING RULE.

The position gossip beside it is published only for peers whose freshness
pairs up; `finished` is published for any peer believed finished, first-hand
or relayed, because the bit only ever removes a robot from the set the team
waits for and is idempotent in doing so.
```

## Gen23FinishedRelay.TheConsumerCopiesTheArrayIntoTheObservation

### relay-consumer-copies-array

**The consumer copies robot_finished** — attached to `TEST(Gen23FinishedRelay, TheConsumerCopiesTheArrayIntoTheObservation) {` (line 1595)

```text
AND THE CONSUMER COPIES IT INTO THE OBSERVATION.

The counterpart to TheConsumerCopiesTheBitIntoTheObservation. Without this
line the wire field is written and never read, which is the failure mode
that looks exactly like the generation before it.
```

## Gen23InboundWire.TheBitIsFirstHandAndNeverRelayed

### inbound-barrier-second-term

**Why the barrier waits for inbound peers** — attached to `TEST(Gen23InboundWire, TheBitIsFirstHandAndNeverRelayed) {` (line 1620)

```text
THE DEFECT THIS GROUP EXISTS FOR, measured in the generation-23 smoke rather
than argued from the source. The barrier's release predicate is a COMMS test
and the drive to the agreed cell only ends on ARRIVAL, and the two halves do
not compose. In ts4smoke23_n2_mtare_rendezvous_r20_ttl0_seed1 both robots
committed the same cell at the same t_meet_sec — the timing contract was
exact — and then:

  atlas arrives at t_rel 349.5, settles its 36 s, releases at 385.58.
  bestla, dispatched to the same cell at 249.58, arrives at 412.0 — 26 s
  after atlas left — and stands there alone with waited_sec=271.88.

Neither half misbehaved. atlas released because the radio reaches tens of
metres in the 2026-09 regime, so `teamSettled` went true while most of
bestla's walk was still ahead of it; bestla did not release en route because
doReturnNav's en-route release is gated `!appointment_manoeuvre_` on purpose,
the CELL being the agreed thing. 40% of a robot's run, in the two treatment
arms, on the primary endpoint.

WHY THE BIT SAYS "STILL DRIVING" AND NOT "NOT YET ARRIVED", which is the
whole safety argument and the thing these tests are really guarding. The
appointment wait cap is unbounded (rendezvous_appointment_wait_sec = 0), so
any term added here that can stay true forever hangs the team forever. Every
exit from the drive — arrival, the nav budget, the no-progress watchdog —
leaves RETURN_NAV, so a bit keyed on the STATE always clears within
nav_budget_sec whether or not the cell turned out reachable. A bit keyed on
distance-to-cell would not, and that is the tidier-looking rewrite a later
reader is most likely to reach for.
```

### inbound-first-hand-only

**appointment_inbound is first-hand only** — attached to `TEST(Gen23InboundWire, TheBitIsFirstHandAndNeverRelayed) {` (line 1649)

```text
THE BIT IS FIRST-HAND ONLY, LIKE team_incomplete AND UNLIKE finished.

It is non-monotonic — it goes true when the drive starts and false when the
drive ends — so a relayed copy could hold the barrier off an echo of a claim
its own subject has already withdrawn. The relay that GROUP G adds for
`finished` is sound precisely because that bit is latched; this one is not,
and the two must not be tidied into one mechanism.

Both directions, because they fail differently: a relay must not smear its
own bit onto the robot it is gossiping about, and must not clear a bit it
was never told anything about.
```

## Gen23InboundWire.TheSeenReportLandsUnderItsSenderOnlyAndClears

### inbound-seen-sender-only

**The one-hop report stays with its sender** — attached to `TEST(Gen23InboundWire, TheSeenReportLandsUnderItsSenderOnlyAndClears) {` (line 1698)

```text
THE ONE-HOP REPORT LANDS UNDER ITS SENDER ONLY, AND CLEARS BY ASSIGNMENT.

Generation 27's closure door admits peers heard only through a bridge, so
the bridge's report (appointment_inbound_seen) is the still-driving veto's
only witness for them — but the report is itself non-monotonic twice over:
the drive it reports ends, and the reporter's own horizon changes. It must
therefore land only under its sender, and the sender's next message must
withdraw it by plain assignment. Two merge loops in observe() touch a
gossiped robot's row — relayed positions and finished_gossip — and the
sender-only claim must hold through BOTH, so this message exercises both.
```

## TEST

### inbound-seen-both-loops

**One message drives both merge loops** — attached to `report.finished_gossip = {0, 0, 1};` (line 1714)

```text
The same message ALSO relays cerd's finished bit. finishedMsg() keeps the
two gossip channels apart on purpose; bundling them here is the point —
the finished_gossip merge visits cerd's row too, and a careless merge
there could smear the seen-report onto the gossip subject just as the
position loop could. One message, both loops, one sender-only assertion.
```

## Gen23InboundProducer.TheBitIsKeyedOnDrivingNotOnArrival

### inbound-keyed-on-driving

**The inbound bit is keyed on driving** — attached to `TEST(Gen23InboundProducer, TheBitIsKeyedOnDrivingNotOnArrival) {` (line 1738)

```text
THE PRODUCER KEYS THE BIT ON THE DRIVING STATE, NOT ON AN ARRIVAL TEST.

This is the safety argument, pinned. `appointment_manoeuvre_ && state_ ==
State::RETURN_NAV` clears at every exit from the drive; a distance test
against the agreed cell would stay true forever on a robot whose meeting
point turned out unreachable, and the wait cap it feeds is unbounded. The
nav budget and the no-progress watchdog exist because that case is routine.

The assertion is scoped to the assignment statement rather than to the whole
function so that it says what it means: the publisher may talk about arrival
for other reasons, this field may not.
```

## Gen23InboundProducer.TheSeenReportIsDerivedFromRawBitsOnly

### inbound-seen-raw-bits-only

**The report uses raw bits only** — attached to `TEST(Gen23InboundProducer, TheSeenReportIsDerivedFromRawBitsOnly) {` (line 1782)

```text
THE PRODUCER DERIVES THE ONE-HOP REPORT FROM RAW FIRST-HAND BITS ONLY.

The one statement in generation 27 where the tidy-looking rewrite is the
deadlock: publishing peerReportsInboundToAppointment() here — "surely the
wider predicate is the more complete report" — lets A say seen because B
says seen. Both then hold their barriers off an echo nothing can clear,
which is the armed-state relay trap arriving one field later.
```

## Gen23InboundEligibility.TheAppointmentBranchWaitsForAPeerStillDriving

### inbound-term-not-in-settled

**The inbound term is appointment-only** — attached to `TEST(Gen23InboundEligibility, TheAppointmentBranchWaitsForAPeerStillDriving) {` (line 1831)

```text
THE APPOINTMENT BRANCH CARRIES THE SECOND TERM, AND ONLY THE APPOINTMENT
BRANCH.

GROUP F pins the ternary's shape; this pins what generation 23's smoke added
to it. The term belongs in manoeuvreReleaseEligible and not in teamSettled:
that helper has other consumers — the arm, the supersede, the presence dwell
— asking a different question, and none of them should acquire a term about
who is still driving.
```

## Gen23InboundEligibility.ThePredicateHasNoFinishedExemption

### inbound-no-finished-exemption

**No finished exemption for inbound** — attached to `TEST(Gen23InboundEligibility, ThePredicateHasNoFinishedExemption) {` (line 1861)

```text
THE PREDICATE HAS NO FINISHED EXEMPTION, AND THAT IS NOT AN OVERSIGHT.

peerReportsTeamBreak() must skip a finished peer because its bit never
clears and would hang an unbounded barrier. Copying that skip here for
symmetry — the obvious tidy-up, and the reason this test is written as a
negative — would reproduce the very defect the field was added for, for
exactly the robot most likely to hit it: one that latched coverage on its
way to the meeting and is still driving there.

The liveness gate IS shared, and for the same reason in both: the bit has no
TTL of its own, so a peer that goes silent mid-drive must stop holding the
barrier. Its silence then holds the release through teamComplete instead,
which is the stronger test and the one that can clear.
```

## Gen23InboundEligibility.TheDoorVetoCoversTheBridgeHop

### inbound-door-veto-bridge

**The door's veto sees the bridge hop** — attached to `TEST(Gen23InboundEligibility, TheDoorVetoCoversTheBridgeHop) {` (line 1897)

```text
THE DOOR'S VETO SEES THE BRIDGE HOP, AND ONLY THE RELEASE PREDICATE
CARRIES IT.

The closure door admits peers this robot cannot hear; their own
appointment_inbound never arrives (never relayed), so without the bridge's
report the door releases while a third robot is still walking in behind
the bridge — the gen-23 lopsided release, reinstated through the door for
exactly the peers the door newly admits.
```

## Gen28SettleLapse.OnlyTheConversionSetsTheResumeFlag

### settle-lapse-return-path

**The settle conversion is reversible** — attached to `TEST(Gen28SettleLapse, OnlyTheConversionSetsTheResumeFlag) {` (line 1952)

```text
The gen-25 conversion lets an appointment walker join the barrier from the
road once the team has settled, which is right while the team IS settled.
Through generation 27 it was one-way, and the ts4 gen-27 N=2 rendezvous
smoke cell measured the cost: both robots converted mid-drive, the link
died thirty seconds later and never came back, and both stood 13.0 m and
7.2 m short of the agreed cell for the remaining 330 s with no outcome row
and frozen coverage. These tests pin the return path.
```

## TEST (part 2)

### settle-lapse-flag-read

**The resume reads the conversion flag** — attached to `EXPECT_NE(sq.find("appointment_settle_converted_"), std::string::npos)` (line 2019)

```text
The flag must be READ here, not merely written at the conversion. Without
this conjunct the resume also arms on return-budget and return-no-progress,
which give up on the road on purpose: budget-exit resumes into another
budget-exit, unbounded. A mutant deleting exactly this term survived the
whole suite until the gen-28 review ran it by hand.
```
