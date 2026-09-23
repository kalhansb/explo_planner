# explo_planner_node.cpp — design notes and history

The long comments of `src/explo_planner_node.cpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a one-to-three-line gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or declaration block) they sit in. Each gives the line of code the comment was attached to. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [File scope, before the class](#file-scope-before-the-class) — 5
- [class ExploPlannerNode — declarations](#class-exploplannernode--declarations) — 185
- [ExploPlannerNode::ExploPlannerNode (constructor)](#exploplannernodeexploplannernode-constructor) — 107
- [ExploPlannerNode::onFusionCounters](#exploplannernodeonfusioncounters) — 2
- [ExploPlannerNode::loadLatestMap](#exploplannernodeloadlatestmap) — 1
- [ExploPlannerNode::tick](#exploplannernodetick) — 10
- [ExploPlannerNode::transitionTo](#exploplannernodetransitionto) — 13
- [ExploPlannerNode::allocVehicles](#exploplannernodeallocvehicles) — 3
- [ExploPlannerNode::separationAnchors](#exploplannernodeseparationanchors) — 1
- [ExploPlannerNode::doPlan](#exploplannernodedoplan) — 53
- [ExploPlannerNode::coverageUnknownFraction](#exploplannernodecoverageunknownfraction) — 1
- [ExploPlannerNode::doNavigate](#exploplannernodedonavigate) — 9
- [ExploPlannerNode::failGoal](#exploplannernodefailgoal) — 1
- [ExploPlannerNode::recordExplorationComplete](#exploplannernoderecordexplorationcomplete) — 3
- [ExploPlannerNode::finishNow](#exploplannernodefinishnow) — 1
- [ExploPlannerNode::noteCoverageDecisionSample](#exploplannernodenotecoveragedecisionsample) — 1
- [ExploPlannerNode::maybeLatchCoverageDone](#exploplannernodemaybelatchcoveragedone) — 6
- [ExploPlannerNode::keepAppointmentOnFinish](#exploplannernodekeepappointmentonfinish) — 4
- [ExploPlannerNode::finishOrRendezvous](#exploplannernodefinishorrendezvous) — 4
- [ExploPlannerNode::noteMapSize](#exploplannernodenotemapsize) — 2
- [ExploPlannerNode::midrunGateSec](#exploplannernodemidrungatesec) — 2
- [ExploPlannerNode::rendezvousTeamMutual](#exploplannernoderendezvousteammutual) — 2
- [ExploPlannerNode::reachablePeerCount](#exploplannernodereachablepeercount) — 1
- [ExploPlannerNode::peerReportsTeamBreak](#exploplannernodepeerreportsteambreak) — 1
- [ExploPlannerNode::manoeuvreReleaseEligible](#exploplannernodemanoeuvrereleaseeligible) — 1
- [ExploPlannerNode::holdingForFinishedPeer](#exploplannernodeholdingforfinishedpeer) — 2
- [ExploPlannerNode::refreshRendezvousSnapshot](#exploplannernoderefreshrendezvoussnapshot) — 3
- [ExploPlannerNode::deriveRendezvousProposal](#exploplannernodederiverendezvousproposal) — 6
- [ExploPlannerNode::maintainRendezvousProposal](#exploplannernodemaintainrendezvousproposal) — 24
- [ExploPlannerNode::armAppointment](#exploplannernodearmappointment) — 18
- [ExploPlannerNode::appointmentLeadMs](#exploplannernodeappointmentleadms) — 1
- [ExploPlannerNode::appointmentTravelMs](#exploplannernodeappointmenttravelms) — 2
- [ExploPlannerNode::appointmentDue](#exploplannernodeappointmentdue) — 1
- [ExploPlannerNode::appointmentPoint](#exploplannernodeappointmentpoint) — 2
- [ExploPlannerNode::closeAppointment](#exploplannernodecloseappointment) — 1
- [ExploPlannerNode::dispatchReconnect](#exploplannernodedispatchreconnect) — 6
- [ExploPlannerNode::releaseConfirmed](#exploplannernodereleaseconfirmed) — 1
- [ExploPlannerNode::releaseHeld](#exploplannernodereleaseheld) — 1
- [ExploPlannerNode::standDownExploitation](#exploplannernodestanddownexploitation) — 2
- [ExploPlannerNode::startReturnTo](#exploplannernodestartreturnto) — 4
- [ExploPlannerNode::armManoeuvreLegBudget](#exploplannernodearmmanoeuvrelegbudget) — 1
- [ExploPlannerNode::doReturnNav](#exploplannernodedoreturnnav) — 7
- [ExploPlannerNode::appointmentLegWatchdog](#exploplannernodeappointmentlegwatchdog) — 6
- [ExploPlannerNode::resumeAppointmentDrive](#exploplannernoderesumeappointmentdrive) — 2
- [ExploPlannerNode::logAppointmentLegRow](#exploplannernodelogappointmentlegrow) — 1
- [ExploPlannerNode::doReturnSync](#exploplannernodedoreturnsync) — 28
- [ExploPlannerNode::startReturnHome](#exploplannernodestartreturnhome) — 3
- [ExploPlannerNode::doReturnHome](#exploplannernodedoreturnhome) — 11
- [ExploPlannerNode::pickEscapeTarget](#exploplannernodepickescapetarget) — 1
- [ExploPlannerNode::startEscapeLeg](#exploplannernodestartescapeleg) — 1
- [ExploPlannerNode::resumeRetrace](#exploplannernoderesumeretrace) — 2
- [ExploPlannerNode::republishHomeGoal](#exploplannernoderepublishhomegoal) — 1
- [ExploPlannerNode::homeWatchdogFire](#exploplannernodehomewatchdogfire) — 5
- [ExploPlannerNode::finishMissionReturn](#exploplannernodefinishmissionreturn) — 2
- [ExploPlannerNode::predictIntercept](#exploplannernodepredictintercept) — 1
- [ExploPlannerNode::startPursuit](#exploplannernodestartpursuit) — 8
- [ExploPlannerNode::armPursuitWaypoint](#exploplannernodearmpursuitwaypoint) — 2
- [ExploPlannerNode::doPursue](#exploplannernodedopursue) — 3
- [ExploPlannerNode::pursuitFallback](#exploplannernodepursuitfallback) — 5
- [ExploPlannerNode::resumeExploring](#exploplannernoderesumeexploring) — 2
- [ExploPlannerNode::pursuitExploreFallback](#exploplannernodepursuitexplorefallback) — 1
- [ExploPlannerNode::holdForTeam](#exploplannernodeholdforteam) — 1
- [ExploPlannerNode::publishPresenceIntent](#exploplannernodepublishpresenceintent) — 1
- [ExploPlannerNode::heartbeatTick](#exploplannernodeheartbeattick) — 14
- [ExploPlannerNode::checkProximityHold](#exploplannernodecheckproximityhold) — 1
- [ExploPlannerNode::enterProximityHold](#exploplannernodeenterproximityhold) — 1
- [ExploPlannerNode::abandonNavGoal](#exploplannernodeabandonnavgoal) — 3
- [ExploPlannerNode::doProximityHold](#exploplannernodedoproximityhold) — 4
- [ExploPlannerNode::fillCommonMetrics](#exploplannernodefillcommonmetrics) — 6
- [ExploPlannerNode::metricsTick](#exploplannernodemetricstick) — 6
- [ExploPlannerNode::expCtx](#exploplannernodeexpctx) — 1
- [ExploPlannerNode::startExperimentLog](#exploplannernodestartexperimentlog) — 1
- [ExploPlannerNode::expPeerHeard](#exploplannernodeexppeerheard) — 2
- [ExploPlannerNode::refreshDispatchContext](#exploplannernoderefreshdispatchcontext) — 1
- [ExploPlannerNode::logReconnectDispatch](#exploplannernodelogreconnectdispatch) — 3
- [ExploPlannerNode::logRunEnd](#exploplannernodelogrunend) — 2
- [ExploPlannerNode::~ExploPlannerNode](#exploplannernodeexploplannernode) — 1
- [ExploPlannerNode::doLogStep](#exploplannernodedologstep) — 1
- [ExploPlannerNode::onTreeTarget](#exploplannernodeontreetarget) — 1
- [ExploPlannerNode::removeLiveRefinementRegions](#exploplannernoderemoveliverefinementregions) — 1
- [ExploPlannerNode::computeApproachGoal](#exploplannernodecomputeapproachgoal) — 1
- [ExploPlannerNode::exploitZAt](#exploplannernodeexploitzat) — 1
- [ExploPlannerNode::startExploitNavigate](#exploplannernodestartexploitnavigate) — 1
- [ExploPlannerNode::vantageVetoedByPeers](#exploplannernodevantagevetoedbypeers) — 3
- [ExploPlannerNode::holdDwelledVantage](#exploplannernodeholddwelledvantage) — 4
- [ExploPlannerNode::doExploitPlan](#exploplannernodedoexploitplan) — 13
- [ExploPlannerNode::doExploitDwell](#exploplannernodedoexploitdwell) — 8
- [ExploPlannerNode::updatePoseFromTF](#exploplannernodeupdateposefromtf) — 4
- [ExploPlannerNode::trackDistance](#exploplannernodetrackdistance) — 1
- [ExploPlannerNode::publishGoal](#exploplannernodepublishgoal) — 1
- [ExploPlannerNode::republishGoal](#exploplannernoderepublishgoal) — 1
- [ExploPlannerNode::updateCellWorld](#exploplannernodeupdatecellworld) — 3
- [ExploPlannerNode::missionElapsed](#exploplannernodemissionelapsed) — 1
- [ExploPlannerNode::missionElapsedAt](#exploplannernodemissionelapsedat) — 1
- [ExploPlannerNode::publishTeamWorld](#exploplannernodepublishteamworld) — 15
- [ExploPlannerNode::drainTeamWorld](#exploplannernodedrainteamworld) — 10
- [main](#main) — 1

## File scope, before the class

### state-pursue-reconnect-ladder

**The pursuit reconnection ladder** — attached to `PURSUE,`

```text
Mesh-reconnection pursuit (robot-carried radios). Instead of driving home
to the anchor, chase the missing peer's last declared goal (and its last
heard pose) on a staleness-scaled budget; the mesh re-forms the moment we
come within range, no arrival needed. Budget spent -> explore on the
fallback allowance (pursuitExploreFallback), and only then hold in place
and beacon (holdForTeam). THE SAME LADDER IN BOTH ARMS: hybrid has no
separate "fall back to the meeting point" branch here, and has not since
2026-09-16 — what hybrid does that pursuit cannot is KEEP AN APPOINTMENT,
which pre-empts the chase from doPursue rather than following it. Gated by
reconnect_mode_; see the pursuit_* params and planner_util.hpp's arm
definitions, which are the canonical text.
```

### state-proximity-hold-entry

**The coordinated proximity stop state** — attached to `PROXIMITY_HOLD,`

```text
Coordinated proximity stop (multi-robot). A DRIVING robot that has lost
right-of-way to a nearby moving teammate cancels its nav goal and parks
here until the peer clears off or parks, then resumes the same goal.
Entered only from NAVIGATE / RETURN_NAV / PURSUE / RETURN_HOME; see
checkProximityHold().
```

### state-return-home-mission-return

**Mission return at every terminal ending** — attached to `RETURN_HOME`

```text
Mission return (arm-invariant platform behaviour, mission_return_enabled).
At ANY terminal exploration ending — coverage latch, step budget, or a
barrier give-up — the robot drives back to its recorded start pose, so
every run ends with the team regrouped at spawn regardless of arm. The
exploration endpoint (exploration_complete) is stamped BEFORE entry;
run_end after this leg resolves is the mission endpoint. Resolves into
DONE on arrival, give-up, or the mission_return_max_sec cap — it can not
loop back into exploration or a reconnect manoeuvre.
```

### statename-wire-interface

**State names are an analysis interface** — attached to `inline const char* stateName(State s) {`

```text
Stable, machine-readable state names. These are wire/CSV values consumed by
the offline analysis (merge attribution classifies each contact event by the
planner state at contact time), so treat them as an interface: renaming one
silently re-buckets every past run's events. No default case — adding a
State without a name here is a compile warning, not a mystery at analysis
time.
```

### latched-qos-contract

**Latched QoS and the topics it excludes** — attached to `inline rclcpp::QoS latchedQos() {`

```text
Latched QoS for "current value" topics: depth 1, reliable, transient_local,
so an endpoint that comes up after its counterpart is handed the CURRENT
sample immediately instead of waiting for the next publish. Both ends must
agree, which is why this is shared rather than spelled out per site.

NOT for every transient_local topic in this file. The tree-target pair uses
KeepLast(50) because its contract is REPLAY A BACKLOG (each target id is
published exactly once, with no retry path) rather than "latest wins", and
the comms link-index subscription is written to mirror the emulator's own
profile, which is not ours to change. Those three are a different contract
and must not be folded in here.
```

## class ExploPlannerNode — declarations

### dtor-closes-event-log

**Why the destructor closes the event log** — attached to `~ExploPlannerNode() override;`

```text
Last chance to close the event log. rclcpp::shutdown() (SIGTERM from the
campaign harness, or the DONE-shutdown path) unwinds spin() and destroys
the node, and a run that ends that way would otherwise have no run_end
line at all — indistinguishable, from the file alone, from a truncated
one. Never throws (see the definition).
```

### on-fusion-counters-per-peer

**Why fusion counters are folded per peer** — attached to `void onFusionCounters(`

```text
Fold dscovox's per-source integration counters into peer_fusion_deltas_,
which is the only signal in this node that can say WHICH peer's voxels are
arriving. The fused map's own total cannot: it sums this robot's sensing
and every peer's contribution into one number, so a robot standing at a
meeting reads the same flat total whether its partner has finished sending
or is not being heard at all.
```

### transition-reason-tags

**Transition reasons are stable tags** — attached to `void transitionTo(State s, const char* reason = "");`

```text
`reason` names WHY the transition happened and is recorded verbatim in the
event log's state_change event. Defaulted so a new call site compiles, but
every existing one passes a stable, machine-readable tag: these strings are
an analysis interface exactly as stateName() is (see the event log header
on why grepping prose was the thing this replaces).
```

### csv-row-assembly-emitters

**How the two CSV emitters share columns** — attached to `void fillCommonMetrics(StepMetrics& m);`

```text
CSV row assembly. fillCommonMetrics populates every column that is a
property of the world right now (map stats, odometry, peers, holds, state,
reconnect clock) and is shared by both emitters; the plan-attribution
columns are left at zero for it to stay honest on a timer row, and doLogStep
adds them for genuine end-of-step rows. metricsTick is the periodic
sampler that runs in every state.
```

### exp-log-sim-clock-envelope

**The event log and its single clock** — attached to `ExperimentContext expCtx();`

```text
Experiment event log (experiment_log.hpp). The CSV above is unchanged and
stays the per-step record; this is the authoritative, machine-readable
event stream on the node's own sim clock.

expCtx() is the per-event envelope (sim stamp + state + step) and is the
ONLY place the log's time axis is read, so no event can be stamped from a
clock the planner does not make decisions on.
```

### exp-clock-anchor-sim-schedule

**Clock anchors fire on sim time only** — attached to `void expClockAnchorTick();`

```text
Emit clock_anchor on a pure SIM-TIME schedule. Never consults a wall clock
to decide whether to fire — that is the failure mode next door in
metricsTick, where a wall deadline armed after the tick's own work
suppresses whole sim-time ticks at some real-time factors and none at
others, silently changing the realised rate between campaigns.
```

### exp-peer-seen-lost-bookkeeping

**Peer seen and lost bookkeeping** — attached to `void expPeerHeard(const std::string& peer_id);`

```text
Peer-belief bookkeeping behind peer_seen / peer_lost. expPeerHeard is
called from the intent callback (every teammate broadcast, in every arm —
the subscription is wired even with coordination off); expPeerSweep runs
on the state-machine tick and flips a peer to LOST once its last intent is
older than the claim TTL. Deliberately computed from the intent stream
rather than Coordination's table so the control arm, which runs with
coordination_enabled=false, still records outage timing.
```

### reconnect-dispatch-log-at-commit

**Dispatch is logged where it commits** — attached to `void logReconnectDispatch(const char* action, const Eigen::Vector3f* dest,`

```text
Emit reconnect_dispatch for the manoeuvre just chosen. Called from the
leaf that COMMITS the action (startPursuit / startReturnTo / holdForTeam /
pursuitExploreFallback), never from the branch that contemplates it, so
the event set and the behaviour cannot disagree. `dest` is null for
actions with no destination.
```

### finish-or-rendezvous-routing

**Every termination routes through finishOrRendezvous** — attached to `bool finishOrRendezvous(const char* reason);`

```text
Rendezvous (multi-robot reconnection). finishOrRendezvous decides, at
exploration exhaustion, between DONE and the reconnect_mode_ manoeuvre
(return-to-anchor, pursuit, or pursuit-then-meeting-point);
startReturnTo arms a drive to any barrier destination (anchor or meeting
point); doReturnNav drives there; doReturnSync holds until the whole team
is in comms. `reason` is the termination cause, for logging only — EVERY
termination path must route through here, not just coverage saturation.
Decide the ending at exhaustion. Returns false when the decision was
DEFERRED by the reconnect confirmation gate and no transition happened —
the caller must then leave the node somewhere that re-runs this check.
```

### exploration-complete-event

**The exploration complete event** — attached to `void recordExplorationComplete(const char* reason);`

```text
The `exploration_complete` event — THIS robot declaring its own
exploration exhausted, emitted at the instant of the declaration and
before anything is decided about what happens next. Factored out of
finishOrRendezvous so the latch criterion, which skips the rendezvous
decision entirely, still records the same event at the same instant.
```

### latch-coverage-done-return-contract

**What maybeLatchCoverageDone returns** — attached to `bool maybeLatchCoverageDone(double unk, const char* source);`

```text
Latched, state-blind completion test (done_criterion == "latch"). Takes an
already-measured ROI unknown fraction — every caller has just computed one
and a second ROI walk is the most expensive thing in the tick.

RETURNS "THE CALLER MUST STOP THIS TICK", which is not the same as "the
run ended" and has not been since the gen-21 hold. doPlan is the only
caller that reads the value (`if (maybeLatchCoverageDone(...)) return;`)
and stopping is the only thing it can do with it; the metrics tick
discards it. So every path that leaves this function having already
published a goal or changed state must answer true, whether it ended the
run (the homing traverse), deferred the ending to doReturnSync
(keepAppointmentOnFinish), or merely declined to latch again. The one
path that answers false while finished is the at-the-cell hold, which is
gated on a state doPlan cannot be in.
```

### coverage-decision-sample-invariant

**Stamping the sample a decision tested** — attached to `void noteCoverageDecisionSample(double unk, const char* source);`

```text
Publish the sample a completion decision was actually taken on into the
cache that recordExplorationComplete stamps its event from.

Exists because the obvious place to do this — inside the latch — is the
wrong place. Generation 7 fixed the mis-stamp there, but the fix sat after
`if (done_criterion_ != "latch") return false;`, so the streak criterion
kept the identical defect: it decides on a fresh local `unk` and reaches
recordExplorationComplete through finishOrRendezvous, which stamps the
PREVIOUS metrics sample. Every termination path must therefore call this
with the number it decided on, and no path may depend on which criterion
is configured. The invariant is: the event's unknown_fraction is the
quantity the decision tested, never a neighbouring sample of it.

Three of the four completion paths call this explicitly. The fourth —
doLogStep's step-budget ending, the DOMINANT termination in dense terrain
— satisfies the invariant WITHOUT a call, and deliberately so: doLogStep
opens with fillCommonMetrics, which writes the cache from the same tick's
measurement before step_ is incremented and the budget tested, so the
stamped fraction is already same-tick fresh. Adding a call there would buy
a second full ROI walk and zero accuracy. Stated here because the
invariant is what matters and "every path calls this" is not the same
claim — a future reader who checks the call sites instead of the ordering
will find one missing and be tempted to "fix" it.
```

### dispatch-reconnect-factoring

**Why dispatchReconnect is factored out** — attached to `bool dispatchReconnect(const char* reason);`

```text
The manoeuvre dispatch itself (mode -> pursuit / meeting point / anchor /
hold), factored out of finishOrRendezvous so the mid-run trigger in
doPlan can arm the same manoeuvres without the DONE fallthrough. Always
transitions into a manoeuvre state and returns true (kept boolean for
call-site symmetry with startPursuit).
```

### rendezvous-team-mutual-gate

**The mutual direct contact gate** — attached to `bool rendezvousTeamMutual() const;`

```text
Is every peer in MUTUAL direct contact right now?

The rendezvous gate, and deliberately stricter than
`teamComplete(accountedPeerCount, rendezvous_expected_peers_)`. That one
counts peers heard on EITHER channel — "I can hear them" — with no test
that they can hear
me, and doc/limitations.md §10 records one-way contact as observed rather
than hypothetical. The two robots would then stop stamping their anchors
at different instants, which is the one thing an agreed INTERVAL cannot
absorb.

`TeamModel::Peer::direct` is the handshake: true only when we received
from the peer inside the TTL AND the `in_range_mask` it sent named us
back. Both ends evaluate the same symmetric predicate, so both stop on
the same physical event.

Reads the model rather than re-deriving: `drainTeamWorld` ticks it at the
planning rate, well inside one heartbeat.
```

### peer-reports-team-break-contagion

**Team break contagion from peers** — attached to `bool peerReportsTeamBreak() const;`

```text
Is any peer we are receiving from RIGHT NOW announcing that ITS OWN view
of the team is broken? (TeamWorld/team_incomplete, generation 23.)

The contagion term: this is how a robot that can hear everyone learns
that two of its peers cannot hear each other. See TeamWorld.msg for why
nothing else can tell it — no channel in accountedPeerCount relays
REACHABILITY, and there is no map relay, so an A—B—C bridge is a genuine
information partition that only C can close. (Its third channel,
`finished`, IS relayed. That is a claim about a peer's run ending, not
about who can hear whom, so it closes no partition; see peerAccounted.)

`direct || heard_one_way` and not `direct`: both mean "received first-hand
inside the TTL", but `direct` additionally requires the peer's mask to
name us back, and suppressing the one-way case would drop exactly the
robot that most needs relaying — it cannot hear us, so its own read is
already broken and this announcement is the only channel it has.
```

### peer-inbound-to-appointment

**Holding for a peer still driving in** — attached to `bool peerInboundToAppointment() const;`

```text
Is any peer we are receiving from RIGHT NOW still DRIVING to the agreed
cell? (TeamWorld/appointment_inbound, generation 23.)

The appointment barrier's other half. Its release predicate is a comms
test and the drive ends on arrival, so without this a robot standing at
the cell leaves the instant its partner comes into radio range — tens of
metres out — and the partner, which may not release en route because the
CELL is the agreed thing, walks the rest of the way to an empty cell and
then waits there alone.

`direct || heard_one_way` for the same reason peerReportsTeamBreak needs
it: the bit carries no TTL, so a peer that goes silent mid-drive must stop
holding the barrier. Its silence holds it through teamComplete instead,
which is the stronger test and the one that can clear.

NO FINISHED EXEMPTION, deliberately unlike peerReportsTeamBreak. That one
must exempt a finished peer because its bit never clears and would hang an
unbounded barrier; this one always clears within nav_budget_sec, so there
is no hang to prevent — and a robot that latched coverage on its way to
the meeting is still coming, so exempting it would reproduce this very
defect for exactly that robot.
```

### peer-reports-inbound-one-hop

**The one-hop still-driving witness** — attached to `bool peerReportsInboundToAppointment() const;`

```text
Does any peer we are receiving from RIGHT NOW report a still-driving
robot on ITS first-hand horizon? (TeamWorld/appointment_inbound_seen,
generation 27.)

The one-hop companion the closure door requires. The door admits peers
that are only reachable through a bridge robot, and such a peer's own
appointment_inbound cannot arrive here — the bit is never relayed — so
the bridge's report is the only witness the still-driving veto can have
for it. The closure expands exactly one hop through a direct bridge
(team_model.cpp/tick) and the bridge hears that hop first-hand, so this
report covers every peer the door's CLOSURE admits that
peerInboundToAppointment cannot see; the count's `finished` disjunct is
the one admission neither veto covers (the accepted residual noted in
reachablePeerCount). Same liveness pairing, no-TTL and no-finished-exemption
arguments as above; a reporter's own run ending does not invalidate what
it currently receives.
```

### accounted-peer-count-channels

**The three channels of accountedPeerCount** — attached to `int accountedPeerCount(const rclcpp::Time& now) const;`

```text
HOW MANY PEERS ARE ACCOUNTED FOR RIGHT NOW — the liveness number every
DECISION in this file runs on, with one sanctioned exception since
generation 27: the appointment barrier's release door also consults
reachablePeerCount() below. Three channels, unioned:

 1. the peer's coordination claim is unexpired — Coordination::peerLive,
    the pre-generation-23 answer and the only one that exists before
    TeamModel is configured;
 2. TeamWorld reports the link `direct` — a bidirectionally confirmed
    radio statement, aged out on TeamModel::direct_ttl_sec;
 3. the peer announced `finished`, so its run is over and it will never
    arrive at anything. UNLIKE 1 AND 2 THIS ONE IS NOT FIRST-HAND AND HAS
    NO TTL: TeamWorld/robot_finished relays it, team_model.cpp only ever
    ORs it true, and nothing but a first-hand observation of the peer
    un-finishing clears it. A finished peer therefore counts accounted for
    the rest of the run from a bit that may have arrived through a third
    robot, with no contact of any kind. That is the intended reading —
    "will never arrive" is not a statement about the radio — but it does
    mean this count is only a reachability claim over the UNFINISHED
    peers, and every argument built on it has to say so.

CHANNEL 1 ALONE IS NOT A STATEMENT ABOUT THE RADIO, and that is the
defect this closes. The presence beacon is STATE-GATED — see `beaconing`
on the heartbeat, which excludes PLAN — so a planner that spends longer
than coord_claim_ttl_sec inside a single solve publishes nothing, ages
out of every peer's claim table with the link perfectly healthy, and
those peers then announce team_incomplete and arm a "peer-separation"
manoeuvre against a robot standing next to them. That is a false
positive on the exact mechanism generation 23 exists to measure, and it
fires at N=2 as well, where contagion is supposed to contribute nothing.
TeamWorld rides the same comms emulator but publishes on its own
unconditional timer, so channel 2 cannot be suppressed by planner
latency; when the radio is genuinely down BOTH channels stop, so the
union adds no false negatives.

CHANNEL 3 is what bounds the appointment barrier. Waiting is unbounded
on purpose (rendezvous_appointment_wait_sec = 0: the team meets and does
not leave early), so without it one robot ending its run — every ending
except the coverage latch used to publish finished=false forever — holds
every other robot at the meeting point until the duration cap and
censors the cell. Exempting a finished peer is what peerReportsTeamBreak
and the allocator's vehicle set already do with the same bit.

Telemetry uses this too: `peers_live` and `coord_active_peers` in every
CSV are this number as of schema 8, so an analysis can reconstruct the
decision from the row that recorded it. The raw Coordination::livePeerCount
survives at exactly one site — the not-yet-configured fallback below.

THE BEACON-SUPPRESSION WARN ON THE HEARTBEAT IS NOW A DIAGNOSTIC, NOT A
HAZARD. It still measures how long this robot went unbeaconed, which is
worth knowing, but a suppressed episode no longer changes any decision:
channel 2 carries the robot through it. A smoke run that shows suppression
episodes AND no "peer-separation" arms attributable to them is the
confirmation that this fix landed.
```

### peer-accounted-single-predicate

**One predicate for a single peer** — attached to `bool peerAccounted(const std::string& name, const rclcpp::Time& now) const;`

```text
accountedPeerCount for ONE peer, named the way the claim table names it.
Every site that asks about a single teammate — the chase release, the
quarry-live column, missingPeerRecord's choice of who the barrier is
waiting on — goes through this, so "the team is whole" and "this peer is
back" can never be answered by two different predicates.
```

### reachable-peer-count-closure

**Reachable peers over the comms closure** — attached to `int reachablePeerCount() const;`

```text
HOW MANY PEERS ARE REACHABLE RIGHT NOW — accountedPeerCount's question
asked over the comms CLOSURE instead of this robot's own edges: a peer
counts if TeamModel::inComms says a relay path reaches it this tick, or
if it announced `finished` (same exemption as the count above, same
reason). Zero before TeamModel is configured.

ONE DECISION CONSUMES THIS, ON PURPOSE: the appointment barrier's
release door in manoeuvreReleaseEligible (generation 27). The ARM must
never read it — in an A—B—C bridge A and B are genuinely partitioned on
the maps (no relay carries them through C), so the meeting must still
be called; see the arming site. But once the team has GATHERED, a pair
the meeting point itself cannot close is not resolved by more waiting:
the gen-26 N=3 smoke parked all three robots in RETURN_SYNC from
~350 s to the 660 s cap over one tree on one 8 m chord (pair 0-2 up
1.3% of the window while both other pairs held 100%). team_model.hpp
scopes inComms to "the dispatch decision" — and releasing a barrier is
exactly a dispatch decision; every data consumer stays on first-hand
freshness.
```

### team-settled-predicate

**The team settled predicate** — attached to `bool teamSettled(int live_peers) const;`

```text
THE generation-23 team predicate: this robot hears everyone AND nobody it
can hear says otherwise.

The exact complement of the arming condition, and it must be used at
every one of the five sites listed at the arming site — arm on `!P`, end
on `P`, for ONE `P`. Generation 27 sanctions one asymmetry: the barrier
RELEASE ends on `P` OR the reachable door (argued at
manoeuvreReleaseEligible), which cannot ratchet because the spent latch
still clears on exactly `P`. Passing `live_peers` in rather than reading
the coordinator here keeps it callable from const context and keeps every
site visibly sampling the same count it already had.
```

### manoeuvre-release-eligible-split

**Why the release predicate splits** — attached to `bool manoeuvreReleaseEligible(int live_peers) const;`

```text
The barrier's release predicate: for an appointment manoeuvre,
teamSettled OR every expected peer reachable (the generation-27 door —
see the definition), both gated on nobody still driving in; plain
teamComplete otherwise.

The split exists because the two manoeuvres answer different questions.
An APPOINTMENT is the team-wide meeting the contagion arms: it ends when
the team is whole by the predicate that called it — or, since generation
27, when everyone it could ever gather is already REACHABLE, because at
the meeting point the mesh predicate is a geometry test one occluding
trunk can fail forever. A release weaker than the arm cannot churn here:
rendezvous_spent_ still clears only on the strict predicate, so a
closure-released team goes back to exploring and cannot re-arm until a
genuine mesh reunion (contrast generation 19, where the LATCH cleared
weak and the arm never stopped). Every other return —
a midrun reconnect, a terminal homing — is this robot's own business and
keeps the local "I can hear my peers" test it has always had; widening
those to a team-wide predicate would let one distant robot's break hold a
pair that has already reconnected.

Reads appointment_manoeuvre_, which transitionTo() clears AFTER both of
its classifiers run, so the classifiers see the same branch the release
site saw.
```

### holding-for-finished-peer

**Waiting for a finished peer still coming** — attached to `bool holdingForFinishedPeer() const;`

```text
THE APPOINTMENT BARRIER WAITS FOR A FINISHED PEER THAT IS STILL COMING
(2026-09-23). True while some peer has finished exploring, has not said
it is leaving, and cannot be heard — see meeting_attendance.hpp — and the
wait for it has not yet run rendezvous_latched_hold_sec from
finished_peer_wait_start_sec_. A veto inside manoeuvreReleaseEligible's
appointment branch, so the release and both classifiers read one answer.
```

### rendezvous-snapshot-freeze

**Freezing the rendezvous snapshot** — attached to `void refreshRendezvousSnapshot();`

```text
Freeze the world the next appointment will be derived from.

Called on the heartbeat while the team reads COMPLETE, so what it holds
is always the last bidirectionally confirmed state. That is the whole
mechanism: agreement is by construction only if both robots solve the
same problem, and the live world at DISPATCH is not that problem — the
dispatch happens a gate's worth of silence AFTER contact was lost, by
which time the two worlds have diverged by exactly the backlog the
meeting exists to exchange. No-op unless the schedule is enabled.
```

### rendezvous-proposal-handshake-gates

**Gates of the propose echo commit round** — attached to `void maintainRendezvousProposal(bool team_mutual);`

```text
Run one round of the propose/echo/commit handshake. Called from the same
heartbeat site as refreshRendezvousSnapshot, but UNCONDITIONALLY — the
team-mutual state is passed in as `team_mutual` rather than gating the
call, because the three things this does need three different gates.

DERIVE (proposer only) requires `team_mutual`. It is the step that must
predate the separation: it argmins over a snapshot that is only a shared
problem while the team is confirmed whole.

ECHO does not, and gating it was a deadlock: a follower could only adopt
the proposer's triple while the WHOLE team was mutually in contact, but
the commit it feeds needs every follower to have already adopted. At N=2
the two conditions coincide and it worked; at N>=3 the fleet is almost
never mutually whole for the two-plus TeamWorld periods a
propose->echo->commit round trip costs, so nothing was ever agreed and
every arming refused (measured: 0 agreements and 18/18 refusals over a
600 s 3-robot cell).

COMMIT does not require it either, and that is not a relaxation. It
requires every peer to have been HEARD holding this robot's exact triple,
which is strictly stronger and of a different kind: the mask is a claim
about connectivity at one instant, an echo is first-hand evidence of what
that specific robot decided. Freshness is not tested either. The full
argument, including why a latch cannot quietly go stale now that a peer
may upgrade off the centroid placeholder, is at the commit rule itself.

Echoing early is therefore not a private input and does not weaken the
agreement: a follower adopts the proposer's three integers VERBATIM. All
it buys is that the fleet is already converged when a brief whole-team
window opens, so that window only has to be long enough to commit in —
not long enough to negotiate in. At N=4 that window was measured at about
two seconds.
```

### derive-rendezvous-proposal

**Deriving the rendezvous proposal** — attached to `RendezvousProposal deriveRendezvousProposal();`

```text
PROPOSER ONLY: solve the schedule over the frozen snapshot and return the
(cell, interval, t_meet) triple to publish. Returns an invalid proposal
when the snapshot cannot support a solve. The plan behind it lands in
rendezvous_held_provenance_.plan for the event (there is no
rendezvous_held_plan_ member; the next line names it correctly and this
one did not until 2026-09-17). Whether what came back is a CHOICE or
the centroid placeholder is read off the provenance it fills in —
`rendezvous_held_provenance_.plan.candidates <= 1` — at the one call site,
which is what sets `rendezvous_held_provisional_`.
```

### arm-appointment-agreed-occurrence

**Arming the appointment from the triple** — attached to `bool armAppointment(const char* reason);`

```text
Turn the committed triple into a standing appointment. THE PLACE AND THE
TIME ARE BOTH THE TRIPLE'S: `cell` is adopted verbatim off the wire and
`t_meet_ms` is an occurrence of the agreed recurrence — the first one this
robot can still ARRIVE at within rendezvous_max_lateness_sec
(nextAgreedOccurrence, floored at t_now plus this robot's shortfall, which
is zero unless it genuinely cannot make the nearest rung). Every robot
that can reach the agreed instant attends the SAME instant; a robot that
cannot, and a robot arming after the instant has genuinely passed, slip by
whole intervals. (Until generation 25 the floor was
now + rendezvous_depart_delay_sec — a lead time added UNCONDITIONALLY,
which forked the ts4 N=3 cell across two occurrences with every robot able
to make the first. That is the distinction the shortfall's clamp at zero
keeps; see the block at the assignment. It described a private countdown
here until generation 23 — see nextAgreedOccurrence in planner_util.hpp
for why that was not a rendezvous.) `interval_ms` is the recurrence period
that occurrence is taken from. Emits one
`rendezvous_agreed` per call, refusals included. Returns true when an
appointment now stands. Does NOT solve — see the P5 state block for why
deriving twice was abandoned.
```

### travel-ms-to-cell-optimistic

**Why travelMsToCell is optimistic** — attached to `long long travelMsToCell(int cell) const;`

```text
This robot's drive time to `cell` from where it is right now, or -1 when
there is no usable estimate — no configured snapshot world, or a position
or target the snapshot's grid cannot place.

TAKES A CELL rather than reading appointment_.cell so the rung choice in
armAppointment can ask it BEFORE the appointment is armed. It is the only
place the grid lookup lives; both readers below go through it.

OPTIMISTIC BY CONSTRUCTION. GlobalAllocator::costMm masks the graph's
unreachable sentinel with a centroid straight line, so this is finite even
for a cell there is no route to, and it prices no nav overhead. That is
why the two decisions built on it use the marked-up appointmentLeadMs
rather than this, and why the reachability WARN in armAppointment asks the
graph directly instead.
```

### appointment-travel-covariate

**The logged travel covariate** — attached to `long long appointmentTravelMs() const;`

```text
This robot's own travel time to the standing appointment, recomputed
LIVE from where it is now (a chase moves it). -1 when there is no
appointment or no usable cell for the current position.

THE LOGGED COVARIATE. It is written as `travel_sec` and answers "how far
was this robot from the meeting", which is what separates a late arrival
from a robot that was never close. The departure rule reads the marked-up
appointmentLeadMs, not this.
```

### appointment-due-departure

**The appointment departure test** — attached to `bool appointmentDue();`

```text
Departure test: is it time to leave so as to ARRIVE at the agreed
occurrence? `now + appointmentLeadMs >= t_meet_ms`, guarded by the armed
flag, degrading to a bare `now >= t_meet_ms` when there is no estimate.

EVERY ROBOT AIMS AT THE SAME INSTANT AND LEAVES ON ITS OWN, so the far one
leaves first and the arrivals coincide instead of being staggered by drive
distance. Whatever spread is left lands on doReturnSync's barrier, which
holds until the team reads complete rather than until a cap expires.

THE LEAD IS LIVE, and that is what makes drift safe: a robot that explores
away from the meeting — or a chase that drags it away — grows its own lead
and is pulled out earlier, so the rung it signed up to at arming stays
reachable without anyone re-deciding which rung it is.
```

### close-appointment-outcome

**Closing an appointment and its outcome** — attached to `void closeAppointment(const char* outcome, bool arrived, double waited_sec,`

```text
Close a standing appointment and emit its `rendezvous_outcome`. No-op
when nothing is armed. Note it does NOT release rendezvous_spent_: the
outage gets one appointment, and the release happens when the team is
whole again.
Emit the appointment's single outcome row and disarm it.

`mutual` records whether the reunion was whole-team TWO-WAY contact, as
distinct from `outcome`, which names what the barrier acted on
(teamComplete over live peers: one-way and direct-only, no relay — see
the release site in heartbeatTick()). The two were one field
until 2026-09-18 and disagreed at N>=3 — see the classifier for the cell
that logged a no-show in the same millisecond as its own reconnection.
```

### appointment-leg-watchdog

**The appointment leg watchdog** — attached to `bool appointmentLegWatchdog(const char* what_failed, float dist);`

```text
doReturnNav's two give-up watchdogs, asking what to do about an
APPOINTMENT leg that has stopped too far from the cell to hand the barrier
an honest presence. Returns true when it has taken the leg over — an
escape manoeuvre, or the terminal roll back to PLAN — in which case the
caller must not also transition; false leaves the watchdog's own
RETURN_SYNC hand-off untouched. Shared because the budget and the
no-progress exits differ only in what tripped them, and a rule enforced at
one of two exits is a rule with a door in it.
```

### manoeuvre-leg-budget

**Sizing a manoeuvre leg budget** — attached to `void armManoeuvreLegBudget(float dist);`

```text
Size the nav budget and open the no-progress window for a manoeuvre leg of
`dist` metres. Shared by the departure in startReturnTo and by every escape
resume, which must not inherit the budget that just fired. Reads
state_enter_time_ as the leg's start, so a caller that is not entering a
state has to stamp it first.
```

### appointment-leg-row

**The appointment leg log row** — attached to `void logAppointmentLegRow(const char* kind, const char* cause, float dist,`

```text
Write one `appointment_leg` row for the rung just taken. Exists because
all three rungs report the same seven quantities about the same leg, and
a ladder whose rows disagree about which leg they describe is worse than
no rows. `leg_sec` belongs to escape-end rows and `rolled_to_sec` to
unreached ones; both default to the -1.0 the schema reads as "not this
kind of row". Call BEFORE any transition — the cell id and the manoeuvre
can both be closed on the way out.
```

### keep-appointment-on-finish

**Keeping an appointment at finish** — attached to `bool keepAppointmentOnFinish(const char* reason);`

```text
Keep a standing rendezvous appointment instead of ending the run where it
finished. Returns true when this robot is now keeping one, in which case
the caller must NOT end the run: doReturnSync owns the ending from there.
Called from both endings that can find an appointment open — the coverage
latch and finishOrRendezvous — and ahead of mission return in each.
```

### return-home-deferred-publish

**Mission return publish and resolution** — attached to `bool startReturnHome(const char* reason);`

```text
Mission return (RETURN_HOME). startReturnHome enters the state WITHOUT
publishing a goal — abandonNavGoal's cancel-all is fire-and-forget, so a
goal sent in the same tick can be swallowed by the still-in-flight cancel.
doReturnHome publishes once home_pub_not_before_ passes (a wall-clock
gate: tick-count deferral proved to be no deferral under backlogged
timers — see the member comment) and arms the nav budget; later ticks
run arrival / cap / budget / waypoint-advance / no-progress checks and
resolve into DONE via finishMissionReturn (mission_complete event, then
finishNow). The second no-progress retry switches to retracing the
robot's own outbound breadcrumb trail (home_trail_). startReturnHome
returns true so terminal-ending call sites can
`return startReturnHome(...)` like finishNow.
```

### home-watchdog-helpers

**The homing watchdog helpers** — attached to `enum class HomeMode { DIRECT, RETRACE, ESCAPE };`

```text
Homing watchdog helpers (generation 5, see the HomeMode member block).
homeApproachMetric is the distance the watchdog holds accountable:
straight-line to home in DIRECT, remaining-trail length in RETRACE (so
that following a curved trail is not scored as failing to approach).
homeWatchdogFire owns the whole graduated response — resend, retrace,
escape, park — and is the ONLY place a homing watchdog decision is made.
engageRetrace/pickEscapeTarget/resumeRetrace are its state transitions;
republishHomeGoal is the shared brake-then-deferred-publish primitive
(the nav cancel action is a no-op here, so the brake goal at the current
pose is what actually stops the platform — see abandonNavGoal).
HomeMode is declared here rather than beside its state members because
these signatures need it.
```

### home-watchdog-test-delta

**What test_delta means in homeWatchdogFire** — attached to `void homeWatchdogFire(const char* kind, float metric, float dist_home,`

```text
`test_delta` is the LEFT-HAND SIDE of the inequality the firing detector
evaluated, selected by the caller to match `kind`: window movement for
"frozen", closing distance for "approach". It is not interchangeable with
`metric`, which is an instantaneous remaining distance sampled at the fire
instant — on the 3 banked g6pilot fires that log both, the two differ by
4.1x, 123x and 162x, and once in sign (the robot was receding). The other
4 banked fires are retrace-mode and log no delta at all, which is why this
parameter exists; see logHomeWatchdog in experiment_log.hpp for the full
accounting, including the two ways this range has been miscounted. The
matching threshold is re-derived inside from `kind`.
```

### pursuit-helpers-overview

**The pursuit helper functions** — attached to `struct LastContact;  // defined with the members below`

```text
Mesh-reconnection pursuit (see the PURSUE state). startPursuit arms the
chase along the missing peer's last-contact trail (returns false when the
record is too stale to be worth chasing — pursuitBudgetSec() == 0);
armPursuitWaypoint publishes the current trail waypoint as the nav goal;
doPursue drives the trail under the budget; pursuitFallback routes a
spent/failed chase to the mode's fallback (meeting point or hold-here);
holdForTeam raises the RETURN_SYNC barrier at the CURRENT pose.
standDownExploitation is the shared open-target demotion every barrier
entry performs (factored out of the old startReturnToAnchor).
```

### presence-intent-beacon

**The presence-only intent beacon** — attached to `void publishPresenceIntent();`

```text
Presence-only intent (goal = own pose), kept fresh by the heartbeat.
Published at every barrier hold and at DONE-idle entry: a parked robot
must stay countable or a teammate that finishes later waits forever on a
robot that is metres away and silent. Since generation 23 the beacon is
belt to accountedPeerCount's braces: a finisher also publishes finished=1
on TeamWorld, which accounts for it even under done_action=shutdown.
```

### vantage-peer-veto-rules

**One implementation of vantage veto rules** — attached to `bool vantageVetoedByPeers(uint32_t target_id, const Eigen::Vector3f& v_pos,`

```text
The three peer-claim rules of the vantage filter (same-target exploit
claim = unconditional veto; other claim shapes = distance contest; parked
staged peer = position contest), extracted so doExploitPlan's candidate
walk and its hold branch consult ONE implementation — two hand-copies of
these rules would drift, and a hold decision computed from different rules
than the selection it suppresses could deadlock a ring. Returns true if a
peer claim denies `v_pos`. Caller gates on coord_->enabled().
```

### hold-dwelled-vantage

**Parking on a dwelled vantage** — attached to `bool holdDwelledVantage(Target* tgt,`

```text
Hold branch of doExploitPlan: true when this robot should PARK on the
vantage it already dwelled (held_vantage_*) because every other angle of
the ring is either team-visited or peer-denied. Publishes the staged hold
claim / re-anchor goal as needed; the caller returns from the tick when
this returns true.
```

### exploit-approach-goal

**Approaching an unreachable vantage ring** — attached to `bool computeApproachGoal(const Eigen::Vector3f& center, float radius,`

```text
Nearest reachable, free, in-ROI point on the line from the robot toward
`center` (marched from just outside the trunk outward). Lets the planner
drive *toward* a target whose vantage ring isn't reachable yet, mapping en
route, instead of abandoning it. Returns false if no such point meaningfully
closer than the robot exists (nothing to approach). Requires a flooded
cost_grid_.
```

### proximity-hold-helpers

**Proximity hold check and resume** — attached to `bool checkProximityHold();`

```text
Coordinated proximity stop. checkProximityHold runs each tick while a nav
goal is in flight (NAVIGATE / RETURN_NAV) and enters PROXIMITY_HOLD when
the guard says a higher-priority teammate is moving nearby; returns true
when it transitioned. doProximityHold holds until the guard releases,
then resumes the interrupted drive on the same goal.
```

### abandon-nav-goal-stop

**Stopping on an abandoned drive** — attached to `void abandonNavGoal(const char* why);`

```text
Stop the platform when a state transition ABANDONS an in-flight drive
without immediately replacing it with another goal. Same stop mechanics as
enterProximityHold (cancel-all + brake goal at our own pose), none of the
hold bookkeeping. See the definition for the invariant this enforces and
the collision that motivated it.
```

### team-world-publish-timer-only

**TeamWorld publishes on its own timer** — attached to `void publishTeamWorld();`

```text
Broadcast this robot's whole cell census, comms mask and gossip. Called
from the `team_world_hz_` timer, and from nowhere else — the exchange is
deliberately NOT piggybacked on the planning tick, because a planner that
stops publishing its world while it is busy planning is a planner whose
peers conclude it went silent exactly when it had most to say.
```

### drain-team-world-once-per-tick

**Draining TeamWorld once per tick** — attached to `void drainTeamWorld();`

```text
Drain `team_world_pending_` into the cell world and the comms model, then
advance the comms model's clock once. Called once per tick, from the tick.

Once per TICK and not once per message: closure is a property of the
whole contact graph, so recomputing it per arriving message would produce
as many different answers per second as there are peers publishing, and
which one a consumer saw would depend on when it happened to look.
```

### exploit-z-terrain-mode

**Exploitation height in terrain mode** — attached to `float exploitZAt(float x, float y) const;`

```text
Standing / sightline height for an exploitation point at (x, y). Flat
mode returns the fixed absolute vantage height. Terrain mode snaps to the
local ground + candidate clearance, the same way exploration candidates
are placed — see the definition for why the absolute height is unusable
once the map z-band is robot-relative.
```

### cell-world-off-by-default

**The coarse cell world default** — attached to `bool       cell_world_enable_ = false;`

```text
Coarse cell world (P1). OFF by default: when disabled nothing is
configured, no census runs, no event is emitted and no publisher exists,
so the node is the pre-M-TARE node exactly. Enabling it is still
observation-only — nothing in this phase READS cell_world_ to make a
decision, which is what makes the census safe to sample from the metrics
path. See cell_world.hpp.
```

### team-world-hz-switch

**team_world_hz is the TeamWorld switch** — attached to `double     team_world_hz_ = 0.0;`

```text
TeamWorld exchange (P2). `team_world_hz_` IS the switch: 0 = off, and off
means no publisher, no subscription, no timer, no comms model and no
event — the node is the P1 node exactly. Requires the cell world (the
message carries its census) and a configured fleet identity (every mask in
it is indexed by numeric id); both are checked at startup and are fatal
rather than degrading, because a TeamWorld with no ids would merge every
peer's census under robot bit zero.
```

### separation-anchors-every-arm

**Separation anchors built in every arm** — attached to `std::vector<SeparationTerm::Anchor> separationAnchors(double now_sec) const;`

```text
Peers this robot is currently willing to be repelled by, rebuilt from the
team model once per PLAN tick. `now_sec` is MISSION-ELAPSED seconds
(missionElapsed()), the clock the team model is denominated in — passing
a this->now() epoch here would make every position look billions of
seconds old and the term silently inert.

Deliberately does NOT consult separation_.enabled(): the anchor list is
built, and the peer distance and eligible-peer count logged from it, in
EVERY arm. Those two columns in an untreated cell are the counterfactual
the treated arm is scored against, and gating this on the term being on
would silently delete them from every control run. Empty whenever the
team model is unconfigured (TEAM_WORLD=0 — then permanently so, and the
columns carry no information for that cell) or no peer has a position
fresh enough to steer on. Both are normal.
```

### alloc-vehicles-definition

**The allocation vehicle set** — attached to `std::vector<AllocRobot> allocVehicles(float x, float y, bool* all_in_comms,`

```text
The allocation problem's vehicle set at position (x, y): every robot in
the fleet identity, self first-hand and peers from the team model.

ONE definition, because there are three callers — the allocator, the
§3.6 reconnect gate and the rendezvous snapshot. `all_in_comms` is
optional (nullptr to ignore).

One definition, but no longer one ANSWER: `pos_max_age_sec` lets a caller
bound how stale a latched peer position may be, and only the doPlan solve
passes a bound (see alloc_peer_pos_max_age_sec_). That relaxes what this
comment used to require — that the allocator and the gate agree exactly,
because the gate's output is the difference between two solves over this
set and a set that drifted by one robot would move C_no and C_re against
each other with no field in either event to show it.

They may now differ by design, and the asymmetry is the point rather than
an oversight: the gate exists to decide whether reconnecting is worth
leaving exploration for, and it needs the stale estimate to have anything
to value or to steer toward — expiring the pose there would make the gate
blind to exactly the peer it is being asked about. The allocator is the
opposite case: a stale pose there does not describe a peer, it silently
reserves ground. The gate is still internally consistent, because both of
ITS solves run over one vehicle set built in one call.

`now_sec` is MISSION-ELAPSED seconds (missionElapsed()), the clock the
team model dates positions on. It is a parameter because this method is
const and missionElapsed() latches its baseline, so the clock has to come
from the caller. A negative `now_sec` (clock not live yet) makes every
age read 0, i.e. fresh — the unbounded behaviour, which is the safe
direction. That holds because TeamModel::observe refuses a negative
now_sec outright (team_model.cpp), so no stored position can carry a
negative stamp, so positionAgeSec's max(0, now - stamp) is 0 rather than
the -1 it returns for "no position held". If that guard ever goes, the
TTL starts dropping peers during the pre-clock window instead.
```

### global-alloc-off-by-default

**Global allocator default and requirements** — attached to `bool       global_alloc_enable_ = false;`

```text
Global allocator (P3). OFF by default, and off means doPlan never calls
solve(), never reorders a candidate, and emits no `allocation` event — the
node is the P2 node exactly. This is the FIRST phase in which the cell
world changes a decision, so it is also the first whose default-off
equivalence claim is about more than an absent log line.

Requires the cell world (it is the problem) and, in any fleet larger than
one, the fleet identity (every vehicle is a numeric id). Both are checked
at startup and are fatal rather than degrading: an allocator with no ids
would put every peer at robot bit zero and solve a fleet of one, which
looks like a working allocator in every field of the event it emits.
```

### alloc-peer-pos-max-age

**Peer position TTL in the allocator** — attached to `double     alloc_peer_pos_max_age_sec_ = 0.0;`

```text
How long a latched peer POSITION may keep holding cells in the allocator
solve, in mission-elapsed seconds. `<= 0` = unbounded, which is this
file's convention for "no limit" and reproduces the pre-TTL planner
bit-for-bit — that default is what lets one binary run both behaviours as
arms of one campaign.

Applied at the doPlan solve ONLY (see allocVehicles). Every other
consumer of a peer position in this planner already bounds its staleness
— the separation term at 10 s, pursuit at 900 s, the link state at 3 s —
and the allocator was the one exception: a peer last seen an hour ago
still owned the cells around wherever it was standing then.
```

### reconnect-gate-info-flag

**The info reconnect gate over the clock** — attached to `bool       reconnect_gate_info_ = false;`

```text
§3.6. false = `reconnect_gate: silence`, today's exact behaviour: the
mid-run silence clock alone decides. true = `info`, which layers the
knowledge + value gate ON TOP of that clock. The gate can only ever
SUPPRESS a dispatch the clock already allowed — it never brings one
forward — so the silence floor of §3.6.3 is kept structurally rather than
by a second comparison someone could later drop.
```

### p5-schedule-scope

**What the P5 schedule replaces** — attached to `bool       rendezvous_schedule_enable_ = false;`

```text
---- Scheduled rendezvous (P5, §3.5) --------------------------------

OFF by default, and off means dispatchReconnect is byte-identical to the
pre-P5 node: no snapshot is taken, no appointment is derived, no
rendezvous_* event is emitted, and every mode falls through the branches
it always did. That is the P5 equivalence claim.

ON, the mechanism replaces ONE thing: the destination a RENDEZVOUS or
HYBRID manoeuvre drives to, and the moment it departs. It replaces
nothing under PURSUIT — pure pursuit has no agreed fallback point BY
DESIGN, and that absence is the A/B this campaign measures (see
dispatchReconnect and pursuitFallback, which both say so). So the four
arms are a clean 2x2 of chase x appointment: off / pursuit (chase only) /
rendezvous (appointment only) / hybrid (both).
```

### rendezvous-frozen-snapshot

**The frozen rendezvous snapshot** — attached to `CellWorld               rendezvous_world_;`

```text
The frozen problem. Copied wholesale rather than referenced: the live
cell_world_ keeps merging peer voxels and re-solving, and an appointment
derived from a world that moved under it is an appointment the peer never
computed. Refreshed only while the team reads complete, so its age is
exactly the outage's age — logged as snapshot_age_sec so a disagreement
between the pair can be attributed to staleness rather than to the
scheduler.

WHAT AGREEMENT RESTED ON UNTIL 2026-09-16, AND WHY IT DID NOT HOLD. The
claim used to be that RendezvousScheduler::solve is deterministic, so both
robots derive the same appointment IF their snapshots agree — and that the
snapshots differ by at most one beacon period of motion, invisible at cell
granularity. The arithmetic half is true and was confirmed in the data.
The snapshot half is false, and the margin is not small:

  ts3 n2+n3, every separated pair that armed at both ends
    same (cell, interval):        21 of 64 pairs (33%)
    median t_meet disagreement:   91 s at N=2, 125 s at N=3 (max 496 s)
    N=3 alone:                    0 of 7 pairs agreed
    both ends fell to the floor:  2 of 6 agreed — so even the midpoint
                                  floor is not symmetric
    identical shared_hash:        still 9 of 24 disagreed, and the
                                  candidate COUNT differed in 5 of 19

The last line is the one that closes it: with the same snapshot hash the
two solves still saw different candidate sets, so shared_hash was never a
complete witness of the inputs and no amount of hash-gating could have
made independent derivation safe. A private map is a private input.

So the appointment is now EXCHANGED, not derived twice — see
rendezvous_proposal_ below and TeamWorld.msg. The snapshot survives for two
narrower jobs: the PROPOSER solves over it to derive the pair, and every
robot reads its grid for the cell centre and its edges for its own travel
estimate. Those are per-robot by design (staggered departures), so a
snapshot that has drifted costs accuracy in one robot's deadline and can no
longer cost the team the meeting.
```

### p5-agreed-proposal

**The agreed (cell, interval) proposal** — attached to `struct RendezvousProposal {`

```text
---- The agreed proposal (the wire protocol above) -------------------

A (cell, interval) pair, compared as exact integers. `interval_ms` is
measured FROM THE SEPARATION, not from any robot's clock origin, which is
what makes it echoable: the same two numbers mean the same appointment on
every robot, and each end converts to its own t_meet by adding the instant
its own view of the team went incomplete.
```

### proposal-t-meet-instant

**Why the proposal carries an instant** — attached to `long long t_meet_ms   = -1;`

```text
The meeting instant itself, in MISSION-ELAPSED ms on the proposer's
clock, echoed verbatim like the other two.

WHY AN INSTANT AND NOT JUST THE INTERVAL (2026-09-17). The interval was
origin-free by design, and each robot supplied its own origin: the
instant its view of the team stopped being mutually whole. That is one
physical event at N=2 and the ts4 smoke measured the two ends 0.84 s
apart — but it is NOT one event at N>=3, because "every one of MY links
is up" is a per-robot predicate over a graph that comes apart edge by
edge. In the N=3 rendezvous cell the three anchors were 13.18 / 34.18 /
33.98 s, so three robots holding a byte-identical pair kept it at times
spread over 21 s. The interval was exact and the appointment was not.

An instant removes the per-robot term entirely: the proposer computes it
once and everyone adopts the integer. What remains is the skew between
the robots' own mission-clock baselines, which is bounded by node start
and measured at ~1.2 s across the ts4 cells — against the 21 s spread it
replaces, and against the 30 s settle window it has to fit inside.

NOT against a wait cap: there is no longer one to compare it to. This
used to read "against a 240 s wait cap", which was
reconnect_midrun_max_wait_sec, and that is the PURSUIT cap — the
appointment barrier's patience is unbounded by design
(rendezvous_appointment_wait_sec = 0.0, see appointment_manoeuvre_
below). Residual skew therefore cannot cost a meeting at all in the
early direction; it only decides how much of the settle window an
early arrival spends waiting.

Mission-elapsed rather than an absolute stamp, keeping the reason the
original field had: field clocks drift hours apart (the 2026-07-06
bunker/curt bags were 4531 s apart while recording simultaneously), and
a meeting time that arrives as an absolute stamp is one a drifting clock
silently relocates. The interval is kept alongside because it is what
the scheduler actually computed and what the floor below is expressed
in; it is no longer the thing the appointment is built from.
```

### proposal-zero-interval-invalid

**A zero interval is not valid** — attached to `bool valid() const {`

```text
A ZERO INTERVAL IS NOT A SCHEDULE, so it is not a valid pair. The test
was `interval_ms >= 0` until generation 29, which let one through — and
a zero-spaced lattice has no "next occurrence" to roll to, so
nextAgreedOccurrence hands back an instant already in the past and the
appointment is due the moment it is armed. That path is guarded at the
far end and the scheduler cannot mint a zero (the interval is floored
at the larger of rendezvous_interval_sec and the furthest robot's
drive), but the argument for deleting the already-passed diagnostic in
armAppointment is written as "while the committed interval is
positive", and a precondition a predicate does not enforce is one a
later change can quietly remove. It is enforced here instead.
```

### rendezvous-proposer-constant

**Why the proposer is a constant** — attached to `static constexpr int kRendezvousProposerId = 0;`

```text
WHO DERIVES. A constant, not an election. The handshake runs only inside
the team-complete guard, where every robot in the fleet is live by
definition, so "the lowest LIVE id" and "the lowest id" are the same robot
and the scan that would distinguish them can only introduce disagreement —
two robots running it a heartbeat apart can answer differently. The config
block enforces the one assumption this rests on (team-complete means the
WHOLE fleet) and refuses to start otherwise.
```

### rendezvous-bootstrap-period

**The bootstrap derive period** — attached to `static constexpr double kRendezvousBootstrapPeriodSec = 5.0;`

```text
How often the proposer retries a derive BEFORE the team has ever agreed a
pair. See the bootstrap block in maintainRendezvousProposal: the run's one
free agreement window is the opening seconds, when the fleet is still
co-located, and the steady-state period is far too coarse to land inside
it. Not a parameter: it is a pacing floor for a transient state, and every
knob on this path is one more thing two robots can be configured to
disagree about.
```

### rendezvous-reagree-wait

**Waiting on the cell to re-agree** — attached to `static constexpr double kRendezvousReagreeWaitSec =`

```text
How long a gathered team will stand on the agreed cell waiting for the
NEXT pair to commit before it gives up and leaves on the one it has.
Kalhan's rule is "the time to meet is updated before going to explore
again", so the re-agreement is part of the meeting and not something that
happens to the team while it drives away: without the wait the release and
the request land on the same tick and the fleet disperses through the
handshake, which is the one window where a link break leaves the team on a
pair derived before the outage.

BOUNDED, so the worst case is exactly the old behaviour plus this much
standing still. A commit needs one derive (paced by the bootstrap period)
and one echo round-trip, so the honest case is under ten seconds and this
is headroom, not a budget anyone expects to spend. Sized against the retry
cadence rather than picked round: six bootstrap periods.

Not a parameter for the same reason the period above is not one, and
because a robot only ever observes its OWN commit here — there is nothing
for two robots to agree about, so a knob would buy nothing but a way to
set it wrong.
```

### rendezvous-held-published-pair

**The published pair versus the driven one** — attached to `RendezvousProposal rendezvous_held_;`

```text
What this robot PUBLISHES. On the proposer it is its own derivation; on
everyone else it is the proposer's pair copied verbatim — the echo. It is
deliberately NOT what the robot drives: publishing a pair only says "I have
received this", and driving it before the peers have it back is exactly the
race the commit below exists to remove.
```

### rendezvous-provenance-with-pair

**Why diagnostics travel with the pair** — attached to `struct RendezvousProvenance {`

```text
WHY THE DIAGNOSTICS TRAVEL WITH THE PAIR AND NOT ON THEIR OWN. A
(cell, interval) pair is two integers; everything that explains WHY that
cell won — the penalty it beat, how many candidates it beat, the world it
was searched over — lives outside the wire format, so it has to be carried
alongside the pair in memory. Carried WRONG, it silently describes a
different appointment than the one the row logs, which is worse than
logging nothing: the two failure modes are

  * TWO LIVE GENERATIONS. The proposer re-derives every 30 s, so between a
    new solve and the team's echo of it the robot holds a NEW plan and an
    OLD committed pair. A separation in that window arms the old cell and,
    if the plan were read live, would stamp it with the new solve's
    penalty and candidate count.
  * A REFUSED RE-DERIVE. deriveRendezvousProposal clears the plan before it
    solves and a refusal leaves the standing pair alone, so a live read
    would report an armed, committed, whole-team appointment with
    penalty -1 over 0 candidates — the follower shape, on the proposer.

Hence one struct per pair, copied at the commit site. PROPOSER ONLY: a
follower echoes two integers and never runs the argmin, so its copy stays
at the sentinels and the event says so with -1s rather than with zeros a
real search could also have produced.
```

### provenance-snapshot-hashes

**Provenance snapshot hashes** — attached to `unsigned int shared_hash     = 0;`

```text
The frozen snapshot the argmin actually ran over, and when it ran. NOT
the robot's latest snapshot: refreshRendezvousSnapshot runs every
team-complete heartbeat while the argmin runs every
rendezvous_proposal_period_sec_, so the live hashes are up to a period
newer than the search they would be claimed to describe.
```

### rendezvous-held-provisional-upgrade

**The provisional pair and its one upgrade** — attached to `bool rendezvous_held_provisional_ = false;`

```text
The held pair was won by the CENTROID FALLBACK with nothing to beat — the
allocator had produced no tour yet, so the candidate set was the team's own
centroid and nothing else. It is a real, keepable appointment (that is the
point of the fallback), but it is the appointment of a team that had not
yet decided where it was going, and it is the only pair the proposer may
replace WITHOUT the team having first gathered and kept it.

WHY THE EXCEPTION EXISTS AT ALL, given that one-pair-per-meeting is
deliberate, and given that a provisional pair would be replaced at the
first meeting anyway. Because the first meeting may never come: the pair is
what the team drives to, so a placeholder is not merely a poor first
meeting point, it is a poor first meeting point that has to be kept before
anything can improve it.
The two clocks that have to line up for a tour-informed proposal are the
fleet becoming mutually complete and the allocator producing a tour, and
the ts4 smoke measured them both landing around t+25 s with the team
already dispersing: at N=4 the mutual window was about two seconds wide and
the tours were empty for all of it. Freezing the first thing derivable
makes the meeting place a function of which of those two won a race, which
is not a property anything should depend on.

EXACTLY ONE UPGRADE, and only to a pair that had a choice to make
(`candidates > 1`). A provisional pair is never replaced by another
provisional pair, so the published cell does not follow the centroid
around; and once a real pair is held this flag is false forever, leaving a
kept meeting as the only thing that reopens the derive gate. The window the
upgrade reopens is the one the message header already documents — a
separation between the proposer publishing and the last peer echoing leaves
the fleet split across two generations — and it is reopened ONCE, early,
while the whole team is in mutual contact, rather than every 30 s for the
whole run, which is the form that put one robot on cell 56 and its partner
on cell 57.

"ONCE, EARLY, WHILE MUTUAL" WAS NOT A BOUND ON THE WINDOW (generation 22).
It reads like one and it is not, because `team_mutual` is a claim about the
last few seconds: the proposer may hold it true for a peer that has already
stopped listening. That is not a corner case, it is what happened — the
upgrade in ts4 smoke20's N=3 hybrid cell was authored 0.6 s after its last
peer went silent, and split the fleet across exactly the two generations
this paragraph claims to have narrowed. "Early" was doing the real work
here and it was never enforced.

THE PRE-MISSION-HOLD CONFINEMENT IS NOT WHAT SHIPPED, and this member's doc
said it was for a day. It was tried on 2026-09-17 and withdrawn the same day
on measurement — a held robot completes no exploration steps, the upgrade's
input is completed steps, so the confinement deleted the upgrade instead of
scheduling it. Read the forty lines at the derive gate before re-adding it.

WHAT PROTECTS THE UPGRADE NOW is the commit rule, not a window: every
commit needs every echo (the unanimity return near the end of
maintainRendezvousProposal), so an upgrade authored into a fleet that has
already come apart simply does not commit anywhere, and the team keeps the
triple it has.
```

### rendezvous-agreed-provisional

**Whether the committed pair was provisional** — attached to `bool                 rendezvous_agreed_provisional_ = false;`

```text
Was the COMMITTED triple the centroid placeholder? Distinct from
rendezvous_held_provisional_, which tracks what this robot currently holds
and can change under it: this one is stamped with the commit and answers
"what did the team actually agree to", which is the question the arm is
measuring. A whole campaign of true here ran the rendezvous arm without
ever exercising the scheduler's argmin — a legitimate result, but not the
one the arm name implies, and not visible anywhere else.
```

### rendezvous-agreed-peers-count

**Confirmations counted at commit** — attached to `int                  rendezvous_agreed_peers_  = 0;`

```text
How many peers had CONFIRMED the pair at the moment it was committed.

It used to be structurally fleet-1 on every armed row — an assertion, not a
measurement — because the commit rule returned early unless every peer had
echoed. That stopped being true on 2026-09-17, when a FINAL triple started
committing on first-hand evidence from the proposer instead (see the commit
rule in maintainRendezvousProposal for the N=3 split that forced it). A
final-triple row may now read anywhere in [0, fleet-1] and none of those
values is a defect; a PROVISIONAL row is still fleet-1 by construction.
Pair it with rendezvous_agreed_provisional_ before reading anything into it.

It is NOT a liveness count — it says nothing about how many peers are
reachable now, which is the whole point of an appointment agreed beforehand
and kept through an outage.
```

### rendezvous-reagree-due

**A completed meeting is owed a new pair** — attached to `bool                 rendezvous_reagree_due_   = false;`

```text
A COMPLETED MEETING IS OWED A NEW PLACE AND TIME. Set where the barrier
releases the gathered team back to exploring, cleared when the proposer
adopts the pair it asks for.

THIS IS THE ONE MOMENT RE-AGREEMENT IS FREE, and it is the same argument
that justified agreeing beforehand in the first place: deriving needs the
team mutually whole over one shared snapshot, which is exactly what a
gathered team standing on the agreed cell is. The pair the meeting replaces
was argmin'd over the map the team held BEFORE it exchanged anything, so
holding it for the rest of the run means every later meeting is sited by a
map that is now several outages out of date.

SET ON EVERY ROBOT, READ ONLY BY THE PROPOSER. The release site is shared
code and the proposer is a fleet id fixed for the whole run
(kRendezvousProposerId), so a follower sets this and nothing ever consumes
it — harmless in the same way the rest of the held-pair machinery is inert
on a follower, and cheaper than teaching the barrier who proposes. A
follower can never become the proposer mid-run, so the stale `true` it
carries has nothing to wake up.

A REQUEST, NOT A COMMAND. The derive it opens can come back provisional or
refuse, in which case the flag stays set and the request is retried on the
bootstrap period rather than being lost; the team leaves on the pair it
already keeps until a better one is actually found. See the adopt site for
why it is cleared there and not at the attempt.
```

### rendezvous-reagree-waiting-stage

**The re-agreement stage of the meeting** — attached to `bool                 rendezvous_reagree_waiting_ = false;`

```text
THE SECOND HALF OF THE MEETING: the team has exchanged maps and asked for
the next pair, and is now standing on the cell until that pair commits.
A stage, not a duration — the settle above answers "have the maps moved",
this answers "does the next meeting exist yet", and the two run in series
because the re-agreement is argmin'd over the map the settle just merged.

`from` is the pair the robot arrived on. The release test is that
rendezvous_agreed_ has moved OFF it, which is the commit rule's own output
(every peer echoed the same triple) and reads identically on the proposer
and on a follower — unlike rendezvous_reagree_due_ above, which only the
proposer ever clears.

NO CLOCK OF ITS OWN. The wait is measured off the settle's clock, as
`settled_sec - rendezvous_settle_sec_`, because the two stages are
consecutive halves of one uninterrupted stand on the cell and a second
rclcpp::Time here would be a second thing to keep in step for no extra
information. It inherits the settle clock's property along with its
reading: node time, so an unresolvable mission clock cannot skip the wait.
```

### peer-fusion-deltas-per-peer

**Per-peer fusion deltas** — attached to `std::vector<uint64_t> peer_fusion_deltas_;`

```text
Voxel deltas this robot's dscovox has ingested from each peer, by fleet id,
cumulative over the run. Fed by onFusionCounters, which is also what sizes
it: EMPTY means dscovox has never published counters at all, and that is a
different fact from a vector of zeros, which means it is publishing and
nothing has arrived. The first is unmeasured; only the second is evidence.

WHY PER-PEER AND NOT A TOTAL. A total that has stopped rising is the one
observation a meeting cannot act on: a partner that has sent everything it
has and a partner whose bytes are not crossing the radio at all both
present a flat total, and the second is exactly the case the meeting
exists to sit through. Split by source, the two separate — but only
against a baseline. Neither has a rate that distinguishes it, so the test
is whether this rose ABOVE what it read when the meeting started, not
whether it is rising now.

MONOTONE ONLY WITHIN A dscovox LIFETIME. The counters are node-local and
restart at zero, so onFusionCounters treats a DECREASE as a re-baseline.
```

### rendezvous-peer-provisional-flags

**Peer provisional flags beside the pair** — attached to `std::vector<uint8_t>            rendezvous_peer_provisional_;`

```text
The provisional flag that arrived WITH that pair, parallel-indexed like the
receipt times above. Deliberately NOT a field of RendezvousProposal: that
struct's operator== is the protocol's exact-bytes commit comparison, and
folding a flag into it would make two robots holding the same three integers
compare unequal because one of them is further along in deciding to keep
them. The flag qualifies the pair; it is not part of it.

uint8_t rather than bool because std::vector<bool> is a bit-proxy and this
is read by index next to two vectors that are not.
```

### rendezvous-peer-confirmed-latch

**The peer confirmation latch** — attached to `std::vector<RendezvousProposal> rendezvous_peer_confirmed_;`

```text
THE LATCH: the pair each peer has been OBSERVED holding, by fleet id, kept
until this robot's own held pair changes. This is what the commit rule
counts, and replacing an instantaneous scan of rendezvous_peer_ with it is
the difference between a rendezvous arm that works at N=3 and one that is
silently identical to `off`.

WHY A LATCH IS SOUND HERE, and it does NOT rest on the pair being frozen —
it is not frozen. The proposer derives at most one pair per MEETING: the
derive branch in maintainRendezvousProposal is gated on a held pair that is
absent, provisional, or spent by a meeting the team has just kept, so
between meetings it is shut, and a run that never gathers derives exactly
once. (The old gate was `due && (none_yet || confirmed)`; neither term
exists any more, and the second was unreachable. See the ONE PAIR PER
MEETING block at the derive site for why the condition moved.) A follower
only ever adopts the proposer's pair, so at any instant there is one pair
in play — but it is replaced from time to time, and a latch written before
a replacement is stale by definition.

What makes the latch sound is therefore not that it cannot go stale, but
that neither kind of staleness can be read as agreement. This robot's own
replacement is handled by comparing against `rendezvous_held_` at commit
time, so every latch naming the old pair stops counting the moment the new
one is held, with nothing to clear. A PEER's replacement is handled by the
clear in the latch loop, which drops the latch on the first message showing
that peer somewhere else. Between them, a latch that still counts is a
latch nothing has contradicted, and expiring THAT on a TTL discards a fact
that has not stopped being a fact.

WHAT THE OLD RULE COST. It required every peer to be inside
coord_claim_ttl_sec (5 s) AND holding the pair AND mutually in direct
contact, ALL AT THE SAME INSTANT. That is an N-way coincidence on an
occlusion-gated radio, and its probability collapses with team size: one
peer to line up at N=2, two at N=3, three at N=4. Measured on the gen-12
N=3 smoke — the proposer derived cell 24 at t=63.6 s and both followers
echoed it (t=67.1 and t=74.0), so all three robots genuinely held the same
pair — and the commit never fired because the 6.9 s between the two echoes
outlived the window. Every one of the 18 armings in that run refused with
"no (cell, interval) pair was agreed" while the fleet was, in fact, agreed.

WHAT IS NOT RELAXED. Equality is still exact (same cell, same interval, no
tolerance), the pair is still derived once while the team is whole, and the
anchor is still stamped ONLY under rendezvousTeamMutual() — so the shared
time origin is untouched. The only thing dropped is the requirement that
the confirmations be CONTEMPORANEOUS, which was never what made the
appointment exact.

TWO DIFFERENT STALENESSES, AND ONLY ONE OF THEM IS FREE. Comparison against
rendezvous_held_ scopes a latch to OUR pair: if this robot's held pair
changes, every entry for the old one stops matching on the next tick, with
no explicit clear to forget. That one is free.

The PEER's pair changing is not, and it is a false commit rather than a
missed one. It became reachable the moment the proposer was allowed its one
upgrade (see rendezvous_held_provisional_): the proposer moves off the
placeholder, a follower still on the placeholder counts its stale latch for
the proposer, reaches fleet-1, announces agreement and arms against a triple
the proposer does not hold. So the latch loop DOES clear explicitly, on any
first-hand message showing the peer somewhere else, and the property that
makes the remaining staleness harmless is stated where it is enforced.
```

### rendezvous-proposal-period

**The proposal period** — attached to `double rendezvous_proposal_period_sec_ = 30.0;`

```text
How often the proposer may issue a NEW pair. Every re-issue opens a window
of one TeamWorld period in which the team holds two adjacent generations,
so the cost of a short period is agreement and the cost of a long one is a
stale meeting point. 30 s against team_world_hz=1.0 puts the window at
roughly 3%, and a 30-s-old meeting cell is still on somebody's current
tour.

IT ALSO BOUNDS SNAPSHOT STALENESS, and that is the reason it is not the
knob a campaign reaches for to space the meetings out. refreshRendezvousSnapshot
refreshes on this same period, and the snapshot is what appointmentLeadMs
costs the drive against — so raising this to widen the timetable would make
every punctuality estimate up to that much older, which is the opposite of
what generation 29 is for. The timetable spacing is
rendezvous_interval_sec_; these two used to be the same number for no
better reason than that this one existed first.
```

### rendezvous-interval-timetable

**The timetable spacing** — attached to `double rendezvous_interval_sec_ = 300.0;`

```text
THE SPACING OF THE TIMETABLE: the shortest gap between two legal meeting
instants, feeding RendezvousScheduler::Config::min_interval_ms. The
scheduler may only ever space the occurrences FURTHER apart than this (the
reachability floor), never closer, so this is the floor on the recurrence
and in practice is the recurrence — the tour term it competes with was
28.3 s at N=2 in the ts4 smoke.

WHAT IT BUYS AT 300 s. Two things the 30 s it replaced could not. A robot
that misses an occurrence waits five minutes rather than thirty seconds for
the next one, which is the whole point of the rungs being far enough apart
to be worth arriving at; and it widens hybrid's chase window (see the
terminal-chase gate in dispatchReconnect), which at 30 s was negative on
most armings — the hybrid arm stopped chasing altogether and became a
second rendezvous arm, with the four-arm contrast quietly down to three.

WIDENS IS NOT GUARANTEES, and the difference matters to anyone sizing the
arm off this number. Let `lead` be the robot's marked-up drive, `budget`
rendezvous_max_lateness_sec_, and `g` the phase gap from the floored
arming instant to the next rung of the lattice — uniform-ish on [0,
interval), because the outage that arms the appointment does not know
where the timetable is. armAppointment floors at t_now + max(0, lead -
budget) and appointmentDue fires `lead` early, so the window is

    W = g - min(lead, budget)

which ranges over [-budget, interval): `interval - budget` is its
SUPREMUM, not a floor, and W is negative whenever the arming happens to
land within min(lead, budget) of a rung — roughly budget/interval of them,
about one arming in five here against about five in six at 30 s. What 300 s
buys is that most armings chase, not that every arming does.
```

### rendezvous-anchor-time

**The separation anchor** — attached to `rclcpp::Time rendezvous_anchor_time_;`

```text
THE SEPARATION ANCHOR, and the reason it is not team_last_complete_time_.

Both robots convert the agreed INTERVAL into a t_meet by adding it to
their own reading of when contact was lost, so the whole design rests on
the two readings naming the same physical event. team_last_complete_time_
cannot carry that: it advances on received intents alone, so a link that
heals in one direction advances it on exactly one robot. This one advances
only while every peer is in MUTUAL direct contact (see rendezvousTeamMutual),
which is a symmetric condition and therefore stops on both robots at the
same event.

Kept separate rather than tightening team_last_complete_time_ itself: that
clock feeds the reconnect confirmation gate and the release tests in every
arm, and making it stricter would change the off/pursuit arms too — a
behaviour change smuggled in under a rendezvous fix.
```

### rendezvous-anchor-starved

**The starved anchor diagnostic** — attached to `bool rendezvous_anchor_starved_ = false;`

```text
The anchor above may be WRONG ON THIS ROBOT ONLY, and this says so.

It is a latch of the last observed mutual-contact instant, sampled on the
heartbeat timer — and that timer shares a single-threaded executor with the
state machine and the metrics sampler, both of which are measured to
overrun (hb_late_count_). A gap that swallows the whole mutual ->
not-mutual transition leaves the latch at the last tick before the gap
instead of at the separation, and every other robot, unblocked, holds the
right instant. That is the one failure this subsystem cannot absorb: the
pair is exact, the intervals match, and the two ends keep the identical
appointment at times that differ by the length of the stall.

Set on a starved tick that finds the team already apart; cleared by the
next healthy stamp.

IT IS A DIAGNOSTIC ONLY, and has been since the agreed-triple change
(2026-09-17). It used to say "armAppointment refuses while it is set", and
that was true when t_meet was `my_anchor + interval` — a suspect anchor
moved the meeting instant, on this robot alone, and refusing was the only
way to stop two robots keeping an exact appointment at two different times.
t_meet is now the integer the team committed, adopted verbatim off the
wire, so the anchor's value cannot move it and armAppointment does not read
this flag. See armAppointment for the deliberate removal of those refusals.
Its one remaining reader is the WARN below, which it de-duplicates.
```

### rendezvous-spent-one-per-outage

**One appointment per outage** — attached to `bool rendezvous_spent_ = false;`

```text
One appointment per outage. Without this the robot re-arms the SAME agreed
pair the moment the first one closes — same cell, and a t_meet that is now
in the past, so appointmentDue() is instantly true and it drives straight
back to the cell it just left. The pair cannot change during an outage (the
protocol is frozen), so a second arming has nothing new to say. Cleared
when the team reads complete, next to the agreed-pair refresh.
```

### appointment-armed-suppresses-trigger

**The standing appointment suppresses the trigger** — attached to `RendezvousPlan appointment_;`

```text
The standing appointment. Armed by armAppointment, cleared by
closeAppointment, and while armed it SUPPRESSES the mid-run trigger: the
decision was taken when it armed and what remains is a clock. Without
that suppression every PLAN tick of the outage would re-enter the trigger,
burn an attempt and re-derive the same appointment, exhausting the budget
in seconds while nothing moved.
```

### appointment-unplaceable-cell

**Why an unplaceable agreed cell is flagged** — attached to `bool           appointment_unplaceable_ = false;`

```text
"THE AGREED CELL COULD NOT BE PLACED ON ANY GRID I HOLD."

Set by appointmentPoint() on the branch where neither the snapshot world
nor the live world can turn `appointment_.cell` into a position, which is
the branch that returns `latest_pos_` and therefore makes the robot
"depart" to where it already is.

IT EXISTS BECAUSE THE OUTCOME WAS OTHERWISE A LIE IN THE WRONG DIRECTION.
A goal at the robot's own position passes the arrival test on the next
tick (goal_xy_tol_ is 0.4 m), so `appointment_arrived_` goes true, and the
classifier's arrived/unreachable split then reads "I was at the cell and
nobody else came" — a peer no-show — for a robot that never moved and
never had a cell to move to. That is a navigation-side failure wearing a
coordination-side failure's name, which is the exact inversion the
`run-ended` case was added to stop; this is the same class, one branch
over.

Written only by appointmentPoint(), which the four departure sites call.
Those calls ARE the departure, so the write lands once, at the moment the
robot commits to a cell it cannot place.
```

### member-appointment-manoeuvre

**Why the manoeuvre flag is separate** — attached to `bool           appointment_manoeuvre_ = false;`

```text
"THE MANOEUVRE I AM CURRENTLY IN WAS STARTED TO KEEP AN APPOINTMENT."

Separate from appointment_armed_ on purpose, and the separation is a bug
fix rather than bookkeeping. appointment_armed_ answers "does an
appointment stand?", and closeAppointment clears it — including from
transitionTo(), which runs on EVERY state change, including the one that
STARTS the drive to the meeting cell. So the flag could go false on the
departure tick itself, and two decisions downstream silently changed
meaning underneath a robot already on its way:

  - the barrier's wait_cap fell back to reconnect_midrun_max_wait_sec
    (240 s, the PURSUIT cap) instead of the unbounded appointment
    patience, so "be there until all robots are connected" quietly became
    "be there for four minutes";
  - the arrival stamp below never fired, so appointment_arrived_ stayed
    false and no-show could not be told from unreachable.

This flag tracks the MANOEUVRE, so it survives the appointment record
being closed and is cleared only when the manoeuvre itself ends.
```

### appointment-settle-converted-resume

**Settle conversion and the one-resume rule** — attached to `bool           appointment_settle_converted_ = false;`

```text
"I AM AT THIS BARRIER BECAUSE THE TEAM SETTLED WHILE I WAS STILL WALKING."

True only on the generation-25 conversion in doReturnNav, which joins the
barrier from wherever the walker stands. It distinguishes that exit from
the other three (arrival, nav budget, no progress), and the distinction
decides whether doReturnSync may send the robot back onto the road: the
conversion's whole premise is "the team is together, so standing here is
as good as standing at the cell", and when that premise lapses the robot
is simply stopped in the wrong place. The budget and no-progress exits
carry the opposite evidence — the cell could NOT be reached — so resuming
those would loop a drive that has already failed.

Cleared in startReturnTo, so every leg (including a resumed one) starts
false and one lapse can cost at most one resume; a second resume needs a
second genuine settle to convert on.
```

### appointment-escape-detour-flag

**The appointment escape detour flag** — attached to `bool           appointment_escape_active_ = false;`

```text
"THE DRIVE TO THE CELL IS CURRENTLY STEERING AROUND A STALL."

A watchdog firing on an appointment leg has not shown the cell to be
unreachable — only that THIS approach stopped working, which is the one
thing simple_nav_3d cannot say for itself. So the leg answers the way the
homing ladder does: drive somewhere else briefly, then aim at the same
destination again from somewhere the approach is different. While this is
true current_goal_ holds the escape target; the cell it will go back to is
return_dest_, which the detour never touches.

Bookkeeping, not a safety guard. It answers "is a detour in flight", and
nothing else in doReturnNav depends on the answer being right, because
every distance test there measures against return_dest_. The earlier
design made this flag load-bearing — the arrival test was correct only
because it ran after a check on this — and that is the arrangement
return_dest_ exists to retire.

Read only inside doReturnNav, cleared per leg in startReturnTo (the same
convention as appointment_settle_converted_ above), so leaving it set on
an exit to RETURN_SYNC is inert: the only ways back into RETURN_NAV are
startReturnTo, which clears it, and the proximity-hold resume, which is
resuming this very leg and must preserve it.
```

### hybrid-appointment-chase-tried

**One chase per standing appointment in hybrid** — attached to `bool           appointment_chase_tried_ = false;`

```text
HYBRID only: the mid-run trigger has already been allowed one chase for
the appointment that currently stands. Scoped to the appointment, not to
the outage or the run, so it is cleared wherever appointment_armed_ is
set or cleared and nowhere else.

Exists because the mid-run trigger spends an attempt BEFORE calling
dispatchReconnect and does not refund one that dispatches nothing. Hybrid
is the only arm that can reach that trigger with an appointment already
standing, and a chase that declines there (stale peer record, uncoverable
trail) declines again on the next PLAN tick for the same reason — so
without this the budget drains in seconds, exhaustion reverts the run to
terminal-only reconnection, and hybrid's pursuit half dies for the rest
of the run with nothing in the log but an exhaustion warning.
```

### rdv-no-show-list-removed

**Removal of the no-show list** — attached to `bool pursuit_predictor_mdp_ = false;`

```text
THE NO-SHOW LIST IS GONE (2026-09-16). It held the cells a no-show had
written off so the NEXT solve of this outage would avoid them, and it was
removed because both halves of that sentence stopped being true at once:
there is no next solve during an outage (the pair is agreed while connected
and frozen on separation), and there is no second arming either
(rendezvous_spent_). Worse, it was per-robot — a cell one robot wrote off
was still a candidate on the other, so feeding it back in was one more way
for two robots to solve different problems. The `excluded` log column went
with it; `RendezvousPlan::rejected_excluded` survives as a faithful mirror
of a scheduler field that is now structurally 0.
```

### pursuit-mdp-interception-scope

**What the MDP predictor replaces** — attached to `bool pursuit_predictor_mdp_ = false;`

```text
---- MDP interception (P6, §3.7) ------------------------------------

`pursuit_predictor: trail` is the shipped chase — the peer's declared goal
then its last heard pose — and is the default. `mdp` replaces ONE thing:
which point the chase drives to FIRST. The budget, the staleness gates,
the fallback ladder and every log leaf below the head are untouched, so
what the arm compares is an aiming rule and not a manoeuvre.

The floor is enforced structurally rather than by review: the predictor
returns a refusal for every case it cannot serve (no tour heard, tour too
stale to carry mass, the intercept out of budget), and each refusal falls
through to the code below it, which is today's. So "mdp is never worse
than trail" is a claim about a fall-through, not about a model.
```

### allocator-my-tour-two-writers

**Who writes and clears my_tour_** — attached to `std::vector<int> my_tour_;`

```text
This robot's current global tour, as the last allocator solve left it —
the thing TeamWorld.my_tour broadcasts and the peers' predictors consume.
Empty whenever the allocator is off or refused, which is the honest
encoding: a peer that hears no tour predicts nothing and chases the trail.

TWO writers, both in service of that encoding. doPlan assigns it on every
solve (so a refusal clears it); publishTeamWorld clears it whenever this
robot is not in the exploration loop that produced it. The second exists
because the first only runs INSIDE that loop: on a manoeuvre, a homing
leg, or after the latch, no solve happens and the route would otherwise
sit on the 1 Hz heartbeat forever at a receiver-stamped age of zero. The
wire field carries no stamp, so "stale" and "current" are the same
message and the consumer cannot tell them apart — see the long note at
the broadcast site. Re-populated by the next solve on re-entry to PLAN.
```

### peer-tour-anchor-consistency

**Why peer tours are kept apart** — attached to `struct PeerTour {`

```text
What a peer last told us FIRST-HAND about its route, and where it was
when it said so.

Separate from last_contact_ because it arrives on a different message
(TeamWorld, not RobotIntent) and must stay internally consistent: the
anchor position is the one that came in the SAME message as the tour, so
the chain starts where the peer was when it declared the route. Mixing in
the RobotIntent pose would anchor a 30-second-old route at a
1-second-old position and predict the peer backwards along its own tour.

Keyed by fleet id, stamped with LOCAL receipt time (the same clock
discipline as last_contact_ and claim expiry — peer stamps are untrusted
and have been observed hours apart in the field).
```

### pursuit-dispatch-predict-consumed

**The dispatch prediction is consumed once** — attached to `PursuitTarget dispatch_predict_;`

```text
The prediction the CURRENT chase was aimed by, for the dispatch event.
Consumed by logReconnectDispatch exactly like reconnect_decline_reason_:
it belongs to the leaf that made it, and a hold or a resume two minutes
later re-reporting an intercept it did not use would put a defensible
number in a column that was never consulted.
```

### pursuit-dispatch-tour-age

**Why tour age sits beside the target** — attached to `double dispatch_predict_tour_age_sec_ = -1.0;`

```text
Age of the tour the prediction ran on, seconds. Kept beside the target
rather than inside it because PursuitTarget describes an ANSWER and this
describes the input it was computed from — and it is the input that
explains most refusals, so a log without it cannot tell a model that is
wrong from a model that was handed nothing recent.
```

### teamworld-pending-newest-only

**Coalescing pending TeamWorld messages** — attached to `struct PendingTeamWorld {`

```text
Newest undrained message per sender id, with its local receipt time and
how many earlier ones it superseded.

Newest-only is safe BECAUSE the message is full state: an older census
contains nothing the newer one lacks, so coalescing costs no information
(it does cost the record, which is why the count is kept and logged).
The subscription callback does nothing but write here — a 400-cell merge
inside a callback would run on the single-threaded executor between
planning passes, and the claim-grace episode is the standing evidence for
what that does to the other subscriptions.
```

### mission-t0-float32-baseline

**The mission-elapsed baseline** — attached to `double     mission_t0_sec_ = -1.0;`

```text
Mission-elapsed baseline: sim seconds at the first tick with a live
clock, -1 before that. TeamWorld's two scheduling quantities are defined
as mission-elapsed and not as absolute stamps, and float32 makes that
mandatory rather than stylistic — an absolute epoch stamp in a float32
quantises to ~128 s, which would round every freshness measure this
subsystem makes into uselessness on a real robot.

Kept here rather than read from exp_log_->t0Sec() so the comms model does
not silently change behaviour when the event log is switched off.
```

### mission-elapsed-latches-baseline

**Why missionElapsed latches the baseline** — attached to `double     missionElapsed();`

```text
Sim seconds since mission_t0_sec_, or -1 before the clock is live.
LATCHES the baseline on its first call with a live clock, which is why it
is not const: the publish timer and the tick both need the mission clock
and either can be the first to fire, so a baseline set in only one of
them would be a startup-order dependency nothing would ever notice.
```

### mission-elapsed-at-past-time

**missionElapsedAt for past stamps** — attached to `double     missionElapsedAt(const rclcpp::Time& when) const;`

```text
missionElapsed() evaluated at a PAST rclcpp::Time instead of now. The
rendezvous anchor needs it: team_last_complete_time_ is an rclcpp::Time
and the interval it anchors is mission-elapsed, so the two have to be
brought onto one scale. Returns -1 when the baseline is not latched yet or
`when` predates it, so a caller cannot silently anchor on a negative.
```

### fov-omnidirectional-yaw-gate

**Deriving the omnidirectional sensor flag** — attached to `bool   fov_is_omnidirectional_ = false;`

```text
True when the MODELLED sensor spans a full circle, derived from fov_hfov
at load rather than configured. It decides whether the exploration arrival
gate blocks on yaw: that gate exists so "the sensor actually observes the
region the planner scored", which a 360 deg sensor satisfies at every
heading. Derived, not a knob, because the two must never disagree — a
separate `require_goal_yaw` param is exactly the kind of thing that ends up
set to the wrong value beside a changed fov_hfov.
```

### nav-goal-republish-keepalive

**Why the goal keep-alive re-send stays** — attached to `double goal_republish_sec_;`

```text
Minimum interval between re-sends of an *unchanged* goal pose. A changed
pose always publishes immediately; this only throttles the keep-alive.

The rationale here used to be nav2's: bt_navigator turning every goal_pose
into a fresh NavigateToPose goal, GoalUpdated halting the recovery subtree
while still consuming RecoveryNode's retries, so an unthrottled 10 Hz
re-send provoked an immediate ABORT rather than a recovery. None of that
machinery is in this stack — the navigator is simple_nav_3d, which takes
goal_pose as a plain PoseStamped and serves no action at all — so the harm
that argument described cannot happen here, and its conclusion that 0 is
"strictly the best setting" INVERTS. What is actually true:

  * A re-send of an UNCHANGED goal is a genuine no-op. The navigator's
    intake drops a goal equal to the one it is driving (same_goal_pose —
    position AND yaw), and after arrival a re-send re-arms it for a single
    50 ms tick that issues no velocity command. The controller compares
    position only, so an identical pose cannot disturb a rotation already
    in progress.
  * A goal published while the navigator is DOWN is lost in silence. There
    is no action feedback, no acknowledgement of any kind; the planner would
    sit out the entire nav budget waiting on a goal nobody received.

So the keep-alive is cheap insurance rather than damage control, and 0 gives
up the one case that matters while buying nothing. republishGoal's
sub_appeared fast path covers the common instance (bringup ordering); the
periodic re-send covers a navigator that dies and restarts mid-run, which
nothing else in this system would notice.
```

### tf-pose-max-age-gate

**Age gate on TF pose lookups** — attached to `double pose_max_age_sec_ = 5.0;`

```text
A TF lookup that SUCCEEDS can still be a corpse: with a dead broadcaster
tf2 serves the newest stored transform forever (TimePointZero reads never
prune), so without an age gate the planner keeps planning, logging and
braking against a pose frozen at the moment the localiser died.
Transforms older than this (sec) read as pose lost. 0 = disabled.
```

### done-criterion-latch-vs-streak

**Latch versus streak done criteria** — attached to `std::string done_criterion_{"latch"};`

```text
Which rule decides that THIS robot has finished exploring:
 - "latch" (default): the first tick on which the ROI unknown fraction
   reaches done_unknown_fraction, in ANY state and regardless of who is in
   comms range. Latched: it never un-finishes. No streak, no rendezvous.
 - "streak": the legacy rule — done_min_consecutive_steps consecutive PLAN
   ticks below threshold, then finishOrRendezvous, which may spend a
   reconnect manoeuvre before DONE. Kept so banked campaigns remain
   reproducible; see the param load for why the default changed.
```

### planning-map-master-switch

**The planning_map master switch** — attached to `bool   use_planning_map_{false};`

```text
Master switch for the 2D planning_map. When true it is subscribed, treated
as a hard startup precondition, and drives the candidate free/occupied
filter + cost-grid reachability in BOTH exploration and exploitation. When
false (default) the planner never subscribes to it and never consults it in
either phase: costs fall back to straight-line distances, there is no 2D
obstacle / reachability filtering, and obstacle avoidance is delegated to
the downstream navigator. Off by default so the planner runs on the fused
3D map alone (the dscovox merger publishes no planning_map).
```

### coord-exploit-claim-grace

**Grace retention for exploit claims** — attached to `double coord_claim_grace_sec_ = 10.0;`

```text
Extra retention for EXPLOIT claims past their TTL (Coordination ctor doc
has the full receive-side-starvation incident). 2x the TTL by default:
long enough to ride out the multi-second executor gaps observed in sim,
short enough that a genuinely dead winner frees its vantage well inside
the 300 s per-target budget.
```

### coord-heartbeat-suppression-tracking

**Tracking heartbeat suppression episodes** — attached to `bool hb_suppressed_ = false;`

```text
Heartbeat-suppression episode tracking (see heartbeatTick). Lets the
analysis separate "peer missing because the radio was down" from "peer
missing because its planner was busy in a non-beaconing state" — the two
are identical in coord_active_peers, and only one of them is a comms
result.
```

### coord-intent-topic-split

**Splitting the intent stream topics** — attached to `std::string coord_intent_pub_topic_;`

```text
Intent stream, split. By default the planner pubs and subs the SAME
global topic (coord_intent_topic_), which means no external process can
sit between two robots' intents — and a peer that reads as *missing* is
the trigger for every reconnect manoeuvre, so a comms emulator that
cannot gate this stream cannot exercise them at all. These two params
separate the ends: publish to one topic, subscribe to a list of others
(typically the emulator's relayed copies, /<self>/rx/<peer>/...). Both
default empty -> fall back to coord_intent_topic_, i.e. today's exact
behaviour, self-echo included (filtered downstream as before).
```

### reconnect-mode-arm-table

**The reconnection mode arms** — attached to `ReconnectMode reconnect_mode_ = ReconnectMode::RENDEZVOUS;`

```text
--- Mesh reconnection (robot-carried radios) params ---
With the radios on the robots (peer-to-peer mesh, no base station) the
anchor loses its router-bubble meaning: the link existed because the PAIR
of poses was within range, and both endpoints have moved since. The mode
picks the reconnection manoeuvre at exploration exhaustion:
THE CANONICAL ARM DEFINITIONS LIVE IN planner_util.hpp — this table is a
pointer to them, not a second source of truth. It was a second source of
truth until 2026-09-17 and all three entries had drifted into describing a
binary that no longer existed (an own-anchor barrier, a bare hold-in-place,
and a midpoint fallback that was deleted on 2026-09-16), which is worse
than no table at all: every one of them named a mechanism a reader could
then go looking for.
  rendezvous — ONE meeting cell agreed by the whole team while connected,
    driven to at the first agreed occurrence once the team reads
    incomplete, held until everyone is present, then held
    rendezvous_settle_sec more for the map merge. An agreed place, or
    nothing.
  pursuit   — chase the missing peer's last declared goal (trail head) on
    a staleness-scaled budget; budget spent -> explore on the fallback
    allowance -> hold in place and beacon. No agreed destination ever, and
    that absence is the A/B against hybrid.
  hybrid    — exactly the union of the two and nothing else: chase while
    the next agreed occurrence is not yet due, keep the appointment once
    it is. Same fallback ladder as pursuit; the appointment pre-empts
    the chase rather than following it.
Code default is the legacy mode; the yaml/sim opt into hybrid.
```

### pursuit-measured-speed-gate

**A measured speed for chase feasibility** — attached to `double pursuit_speed_measured_mps_ = 0.40;`

```text
P1 / §3.8. The speed the robot ACTUALLY travels at (m/s), measured, used
for one thing only: deciding whether a chase is feasible at all.

It exists because the nav family's speed is a WATCHDOG estimate and the
pursuit gate was asking it a PREDICTOR's question. nav_speed_estimate_mps
(0.15) with nav_safety_factor (3.0) prices travel at 20 s/m, against a
measured 0.397 m/s — a factor of eight. For the watchdog that inflation is
harmless in the safe direction: budgets come out generous and a slow leg
gets more rope. For a REFUSAL it inverts. A bigger model time means "this
chase cannot finish", so the same conservatism that makes the watchdog
lenient makes the gate harsh: under the campaign cap of 600 s the gate
refused every chase past 600/20 = 30 m, which is exactly the observed
refusal floor (108 refusals, min 30.0 m, p50 42.9 m, max 75.1 m) in a plot
whose diagonal is larger than that. Pursuit was being declined on an
arithmetic artifact, not on an affordability judgement.

The fix is NOT to correct nav_speed_estimate_mps: that number also sizes
the exploration nav budget at three call sites, where raising it by 2.6x
would shorten every goal deadline and start failing goals that currently
succeed — a far larger behavioural change than the one being made. Nor is
it to raise pursuit_budget_max_sec: that cap is the worst-case absence a
WAITING teammate is entitled to assume, so it must stay denominated in
real seconds and must not be inflated to buy back model seconds.

So the two questions get the two speeds they were always described as
wanting. No safety factor is applied here, deliberately: a safety factor
on a feasibility bound is a thumb on the scale toward refusing, and the
cap already carries all the margin this decision is allowed to have.
<= 0 is clamped away at the use site rather than rejected, same convention
as nav_speed_estimate_mps.
```

### pursuit-staleness-window

**Sizing the pursuit staleness window** — attached to `double pursuit_staleness_max_sec_ = 900.0;`

```text
Last-contact record age (s) beyond which the trail head is worthless and
pursuit is skipped outright; freshness scales the budget linearly down to
zero across this window. <= 0 = no staleness gate.

Sized to real outages, not to goal freshness: dense-forest separations of
186-861 s were measured, and at the old 180 s the chase declined every
single time it was asked — pursuit was dead code in the world it was
written for. What makes the wider window safe is pursuit_goal_stale_sec
below: past 180 s the chase stops trusting the peer's declared goal and
drives to its last CONTACT POSE, a target whose value does not decay with
age, and declines outright when the budget cannot cover that trail.
```

### pursuit-goal-stale-split

**Dropping a stale peer goal** — attached to `double pursuit_goal_stale_sec_ = 180.0;`

```text
Record age beyond which the peer's declared GOAL is a dead hypothesis
(the peer has re-planned since) but its CONTACT POSE is still worth
driving to: two chasers that both complete a contact-pose trail end at
the swapped contact pair, which was mutually within link range — the same
geometric argument the anchor return rests on, and one that does not
decay with staleness. Past this age the chase drops the goal waypoint,
targets peer_pose alone, and budgets by distance (no freshness discount);
it declines instead when that distance-true budget would not cover the
trail (an uncoverable trail ends the chase at an arbitrary disconnected
point — worse than the mode's own fallback). <= 0 = never split.
```

### pursuit-explore-fallback

**Explore rather than park after a chase** — attached to `bool pursuit_explore_fallback_ = true;`

```text
Pure pursuit's fallback when the chase cannot start or is spent: keep
EXPLORING rather than park.

Parking is a fixed point. Two robots that both hold cannot reconnect —
neither is moving, so the geometry that broke the link never changes —
and p7modes measured exactly that: 5 of 6 holds never reconnected, the
single recovery came from the PEER still driving, and one mutual hold
cost a mission whose maps were complete but split. A robot that goes
back to exploring is still covering ground, still earning the mission's
objective, and can regain the link by luck; a parked one can only be
found. Strictly dominated, so pursuit stops doing it.

Bounded, because the terminal dispatch is the run's ending: reaching DONE
needs coverage saturation AND a complete team, so an unbounded
explore-fallback would re-saturate, re-dispatch and re-explore until the
duration cap, converting runs that would have finished into censored
ones. After this many fallbacks the robot reverts to holdForTeam and the
barrier (plus hold_escalate) guarantees an ending. The default matches
reconnect_midrun_max_attempts so a mid-run chase that declines can resume
exploring on every one of its attempts.
```

### reconnect-confirm-incomplete-dwell

**Confirming the team is incomplete** — attached to `double reconnect_confirm_sec_ = 3.0;`

```text
How long the team must have been INCOMPLETE before a manoeuvre may arm.

Without this the arm test (peer missing, one read of the claim table) and
the release test (peer live, the same read one tick later) disagree inside
a single cycle, and the manoeuvre fires and dissolves before the robot
moves. Measured across the p3b/p4mild campaigns: 8 of 24 firings ended
within 5 s having travelled under a metre, two of them logging "ended
after 0.0 s sim", and 9 of 24 armed while the emulator had the pair
connected with 5-12 messages/s flowing. The cause is that the decision is
taken on a claim table which has not absorbed already-delivered intents:
the planner has just spent a long tick in PLAN (which is also what
suppresses its OWN beacon, see heartbeatTick), and drains the queue
immediately afterwards.

This is the same guard the coverage criterion already carries
(done_min_consecutive_steps_): do not act on one sample of a noisy test.
The wait is on top of coord_claim_ttl_sec, so the peer must be silent for
ttl + this before a manoeuvre commits. <= 0 restores the old
arm-on-first-read behaviour.
```

### team-last-complete-heartbeat

**Tracking when the team was complete** — attached to `rclcpp::Time team_last_complete_time_;`

```text
Last time the team was observed complete, and whether that ever happened.
Maintained on the heartbeat timer so it advances in every state, including
the long PLAN ticks that cause the race. A run whose team was NEVER
complete (total blackout) must not be made to wait for a confirmation that
can never arrive, hence the flag.
```

### midrun-silence-threshold-90s

**Choosing the mid-run silence threshold** — attached to `double reconnect_midrun_silence_sec_  = 90.0;`

```text
--- Mid-exploration reconnect trigger ---
0 (the field default) keeps the manoeuvre strictly terminal: a robot only
considers reconnection once its own exploration is exhausted. A positive
value arms the same dispatch DURING exploration, whenever the team has
been continuously incomplete for this long — the point being that a
mid-run reconnection delivers the peer's queued map deltas while they can
still prune this robot's remaining frontiers. Must exceed the worst
heartbeat-suppression episode (measured ~180 s in the sim campaigns, see
heartbeatTick): below that, a healthy teammate stuck in a long PLAN loop
reads as missing and the trigger drives a manoeuvre at a robot that is in
range and fine. 0 restores the legacy terminal-only trigger.

Default 90 since generation 9. It was 240 through generation 8, chosen to
clear that 180 s suppression tail by pure waiting because the record-age
clock could not tell a silent teammate from an absent one. The link veto
below now makes that distinction directly from the radio, so the threshold
no longer has to be set by the suppression tail — and 240 was measured to
be far out in the tail of the outage distribution this binary actually
produces. In g8r1's 23 hybrid cells the clock expired in 3, so 87 % of the
treated arm ran behaviourally identical to the control (§32.15).

HOW 90 WAS PICKED, AND HOW THE FIRST TWO ATTEMPTS AT IT WERE WRONG.
First draft: 120, from a sweep that scored each candidate on "episodes that
reach the gate AND are still disconnected when it expires", using the
PRESENCE clock for both halves. Circular -- filter and score were the same
quantity, so the reported "zero waste" restated the filter. The real check
filters on the past and scores on the future, on two DIFFERENT clocks.

Second draft: 90, re-scored on two clocks but on the 20 g8r1 HYBRID cells
that never dispatched. That population is SELECTED ON THE OUTCOME the sweep
varies, so its 240 row read "0/20 arm" by construction -- a definition
printed as a measurement -- and the monotonicity below it was mostly the
selection. The off arm never ran the trigger at all and was sitting there
unused: 23 untreated cells, no contamination, no selection.

Scored on the OFF arm's banked link_states.csv, with the veto as it ships
(reconnect_link_down_confirm_sec = 0, i.e. "not up right now"):
    presence   cells arm   fires   wasted   beats natural recovery
       240        6/23       12      50 %           33 %
       150       12/23       24      42 %           58 %
       120       18/23       36      33 %           72 %
        90       18/23       40      20 %           85 %
        75       18/23       42      29 %           86 %
        60       21/23       60      43 %           73 %
"wasted" = the radio outage in progress ended within the ~14.6 s it takes
to start moving. "beats natural recovery" = the fire happened more than
one chase (~53 s, p13 median) before the radio next came up and STAYED up
for 30 s, i.e. the manoeuvre had something real to buy.

On THIS grid -- unwalked -- 90 is the argmin of wasted and within a point
of the max of beats, and 120, 90 and 75 all arm the same 18/23 cells.

THE MODEL IS OPTIMISTIC AND HERE IS BY HOW MUCH, AND THE GRID DOES NOT
SURVIVE IT. The table evaluates the veto at the instant the presence clock
crosses T. The binary evaluates on PLAN ticks, so coord_claim_ttl_sec
(4.98-4.99 s measured) plus tick granularity (9.02-13.82 s measured) puts a
real fire 14.0-18.8 s later, in which window a link can change state either
way. Scored at T + that overshoot, at each of the three measured offsets,
the waste argmin moves off 90 and onto 75 (+14.0: 20.0 % vs 90's 28.6 %;
+16.4: 19.2 % vs 26.3 %; +18.8: 25.0 % vs 29.4 %) and the plateau breaks
(120 -> 14/23, 15/23, 17/23 against 90's 18, 19, 17). An earlier draft of
this comment said the walk "moves the hybrid-arm count 11/20 -> 12/20" and
left the direction of the table unaffected; BOTH halves were wrong. The
unwalked hybrid value is already 12/20 and walking gives 12, 10, 11 -- an
overshoot cannot buy activation -- and the table does reorder.

90 IS THEREFORE NOT CHOSEN BY THIS GRID. A ranking that flips under a 4.8 s
change in a nuisance offset is one 23 cells cannot resolve, and re-tuning to
75 on the same 23 cells would repeat the error the grid was already
criticised for. What survives the walk is only the coarse verdict -- 240 far
too high (4-6 of 23 at every offset), 60 past the point where waste turns
back up -- and inside 120-75 the choice is made off the grid: 90 clears the
p90 radio outage (52.0-111.0 s, nearest-rank, 13 tags) and sits far below
the 240 s point where the presence clock stops filtering at all.

Out-of-sample check: at generation 8's ACTUAL configuration (T = 240,
confirm = 3) the model arms 4 of 23 off-arm cells, against the 3 of 23
hybrid cells that really dispatched. Close, not exact, and quoted that way.

WHAT THIS DOES NOT CLAIM. The rate is fitted on generation-8 traces and
generation 9 changes behaviour, so 18/23 (~78 %) is an estimate of the
treated fraction, not a prediction, against generation 8's measured 3/23.
The analysis must carry the dilution rather than assume it away: the
between-arm effect is intention-to-treat over a partly-treated hybrid arm.
```

### midrun-barrier-max-wait

**Barrier cap for mid-run manoeuvres** — attached to `double reconnect_midrun_max_wait_sec_ = 240.0;`

```text
Barrier give-up for MID-RUN manoeuvres only. A mid-run attempt that waits
the full rendezvous_max_wait_sec (600 in the sim harness) burns ten minutes
of exploration on one failed rendezvous; a shorter cap keeps the attempt
proportionate to what triggered it. Terminal manoeuvres keep
rendezvous_max_wait_sec, because for them there is no exploration left to
lose. Deliberately NOT expressed as a multiple of the trigger threshold:
it used to be commented as "2.5x", which was 600/240 and silently became
false the moment the threshold moved to 90 in generation 9.
```

### rdv-depart-delay-inert

**The inert departure notice floor** — attached to `double rendezvous_depart_delay_sec_ = 100.0;`

```text
--- THE APPOINTMENT IS A SCHEDULE KEPT FROM A 100 s NOTICE FLOOR --------

GENERATION 19 REMOVED THE RECURRING SCHEDULE AND GENERATION 23 PUT IT
BACK. What follows is why it went, because the measurement that killed it
is real; see nextAgreedOccurrence in planner_util.hpp for why it no longer
decides the question. Up to gen 18 the
team agreed a place, a PHASE and a PERIOD, and each robot walked to
"the next occurrence at or after now". That needs every robot to select
the same occurrence index, and there is no shared quantity to select it
with: the mission clock is shared but the moment each robot decides it is
alone is not. "Are all my links up?" is a per-robot question about a graph
that fails one edge at a time, so at N>=3 the fleet splits its answers.
Both candidates were tried and both failed on measurement, not on theory:

  the separation anchor  gen 17 keyed the occurrence off it; the ts4 N=3
                         cell anchored its three robots 13.2 / 34.2 /
                         34.0 s apart, so gen 18 removed it.
  each robot's own now   gen 18's replacement; the same cell armed at
                         16.1 / 52.5 / 67.4 s — a 51 s spread against a
                         30 s period, so the three robots walked to three
                         DIFFERENT meetings (t+34, t+64, t+94).

The spread is real and cannot be removed; what generation 23 changed is
what it costs. The rule is: WHEN THE TEAM IS INCOMPLETE, WAIT OUT THIS
NOTICE, THEN DRIVE TO THE AGREED CELL FOR THE FIRST AGREED OCCURRENCE AT
OR AFTER IT AND STAY THERE UNTIL EVERYONE IS BACK. The occurrence is
selected off the committed (t_meet, interval) and this floor, so the
origin is shared even though the floor is not, and the 51 s spread becomes
a departure spread quantised to whole intervals — then ARRIVAL spread,
which the barrier below absorbs by construction rather than by inequality.

The place is still agreed in advance by the whole team, which is the half
of the protocol that was never broken — the ts4 N=3 cell committed a
byte-identical triple on all three robots within 21 ms.

INERT SINCE GENERATION 25. Through generation 24 this was a per-robot
notice floor on the arming (attend the first occurrence at or after
now + delay), and that per-robot term is what forked the ts4 N=3 cell:
the 33 s arming spread plus 100 s of notice straddled the agreed instant,
two robots kept it and the third rolled a whole interval past it. The
arming floor is now bare `t_now`. The parameter stays declared, read and
logged ONLY so the manifest schema and the cross-arm param rows stay
comparable with earlier generations; it is not validated (see the
validation block) and it decides nothing. Flap patience lives where it
always really was: the reconnect
silence window gates the arming, and the P5 supersede cancels a standing
appointment when the team returns before departure.
```

### rdv-appointment-wait-unbounded

**Unbounded patience at an appointment** — attached to `double rendezvous_appointment_wait_sec_ = 0.0;`

```text
Barrier give-up for an APPOINTMENT specifically. <= 0 is unbounded: stand
at the agreed cell until every robot is present, however long that takes.

SEPARATE FROM reconnect_midrun_max_wait_sec_ ON PURPOSE. That cap is what
bounds a PURSUIT — a chase to a predicted intercept that may be at the
wrong place entirely, where giving up is the correct response to a bad
prediction. An appointment is the opposite: the place is one every robot
agreed to, and the only reason a peer is not there yet is that it has
further to drive or noticed later. Giving up on it converts a meeting that
was going to happen into a no-show, which is exactly what the ts4 N=3 cell
did — husky burned the full 240 s and walked away from a cell two of three
robots had already stood on together.

Sharing one parameter between the two would also mean the pursuit arm's
patience could not be tuned without moving the rendezvous arm's definition,
and the four arms have to stay independently specifiable.
```

### rdv-max-lateness-rung

**How late a robot may agree to be** — attached to `double rendezvous_max_lateness_sec_ = 60.0;`

```text
How late a robot may agree to BE. The team's schedule is a lattice —
t_meet + k*interval — and this decides which rung THIS robot signs up to:
the first one it can still reach with no more than this much lateness,
measured at arming against its own marked-up drive (appointmentLeadMs).

IT IS A FLOOR ON THE RUNG, NEVER A NEW INSTANT. The rung is always one the
whole team allocated; the only per-robot decision is `k`, and the floor can
only ever push it LATER (clamped at bare t_now, see armAppointment). A
robot that can make the nearest rung signs up to that one, so a team whose
members can all make it does not fork — which is the property the bare
floor bought in generation 25 and the reason the lateness budget is
subtracted from the robot's drive rather than added to the floor.

WHY NOT ZERO. The lattice spacing is floored at the furthest robot's drive,
so a zero budget would roll a robot a whole interval for being a second
short — paying one full period of separation to avoid one second of
waiting, when the barrier at the far end absorbs waiting for free. 60 s is
Kalhan's figure (2026-09-19); against the 300 s lattice it rolls about one
arming in five, and it is what the chase window above is measured against.

IT BOUNDS WHAT THE ROBOT SIGNS UP TO, NOT WHEN IT ARRIVES. appointmentDue()
is polled at plan boundaries, so a robot whose departure instant falls
mid-leg departs when that leg ends: realised lateness is this budget plus
the remainder of whatever the robot was already driving. Deliberate — the
alternative is abandoning a nav goal on a timer, and the barrier at the far
end waits (rendezvous_appointment_wait_sec 0) so the cost is the early
arrivals' standing time and not a missed meeting. Analyse realised lateness
off the arrival rows, never off this number.
```

### rdv-latched-hold-cap

**Capping the at-the-rendezvous hold** — attached to `double rendezvous_latched_hold_sec_ = 300.0;`

```text
The cap on the AT-THE-RENDEZVOUS HOLD only (see coverage_latch_hold_start_sec_).
It is deliberately NOT rendezvous_appointment_wait_sec: that one is the
patience of a robot that still has exploring to do, and "stay until
everyone is here" — unbounded — is the requested behaviour for it. A robot
with nothing left to explore has no exploring to trade against the wait, so
an unbounded hold there is not patience, it is a cell that runs to the
harness wall clock with one robot standing still and a censored completion
time. Bounding the two separately is what lets the arm keep its unbounded
patience without that cost.

IT HAS TO OUTLAST ONE ROLLED RUNG, which makes it a function of the meeting
lattice and not a free choice: a teammate that cannot make this rung
arrives rendezvous_interval_sec later, plus its own permitted lateness, and
a cap shorter than that sum tears the run down as that robot drives onto
the cell. The node cannot check the relation — it never sees the harness's
interval — so the derivation lives with the value, in RDV_LATCHED_HOLD in
run_explo_sim_rviz.sh, which ships 420 against a 300 s lattice.
```

### rdv-settle-hold-purpose

**Why the meeting holds after completion** — attached to `double rendezvous_settle_sec_ = 30.0;`

```text
Hold at the meeting point AFTER the team reads complete, before exploring
again. The point of the meeting is the map exchange, and the exchange is
not instantaneous: peer voxels cross the comms emulator at a bounded rate
once the link is up, so a barrier that releases on the first tick of
connectivity releases before the thing it was waiting for has happened,
and both robots re-plan against maps that have not merged yet. That is a
meeting that costs the full drive and delivers a fraction of its value.
```

### rdv-settle-start-ros-time

**Settle start on ROS time** — attached to `rclcpp::Time rendezvous_settle_start_;`

```text
Instant the settle hold started, and whether one is running.

ROS TIME, NOT MISSION-ELAPSED (2026-09-18). This was a mission-elapsed
double with -1 meaning "not settling", and the hold was gated on
`t_now >= 0.0` — so on any tick where missionElapsed() could not answer,
the settle was SKIPPED and the barrier released immediately. That is the
wrong direction to fail: the hold exists because the team being connected
is not the same event as the maps having merged, so an unmeasurable clock
should hold, not release. The clock is unnecessary anyway — the settle is
a DURATION, and a duration needs no epoch, only a difference — and
rclcpp::Time on the sim clock is available on every tick this state runs.
The bool is the usual guard: rclcpp::Time default-constructs on the SYSTEM
clock and subtracting that from a sim-time now() throws inside a timer
callback.
```

### rdv-drain-release-thresholds

**Releasing on the exchange, not the clock** — attached to `bool   rendezvous_drain_release_        = false;  ///< the treatment`

```text
RELEASE ON THE EXCHANGE, NOT ON THE CLOCK (generation 33, R3). The settle
above is a guess at how long an exchange takes, applied identically to a
meeting that finished in four seconds and one that is still delivering at
thirty. With Part 1's per-peer counters the exchange itself is observable,
so the trigger can be the thing the hold is for. Nothing else moves: the
routing after the release is untouched.

DEFAULT OFF, AND IT CANNOT BE TURNED ON WITHOUT THE THRESHOLDS. R and W
must come from the per-peer arrival distribution Part 1 logs, and a default
for either would be a number nobody measured deciding when robots leave
meetings. -1 is "not set" and configure() refuses to run with the release
enabled and either unset, rather than substituting a guess.
```

### rdv-finished-peer-wait-bound

**Bounding the wait for a finished peer** — attached to `double finished_peer_wait_start_sec_       = -1.0;`

```text
WHEN THIS ROBOT STARTED WAITING AT THE APPOINTMENT BARRIER, on the mission
clock and floored at t_meet, for holdingForFinishedPeer's bound. -1 while
no appointment barrier is being waited at. Stamped on doReturnSync's first
tick of an appointment manoeuvre and kept for the rest of it, across
resumed legs and rolled rungs (startReturnTo says why); cleared where
appointment_manoeuvre_ is, and by any leg that is not an appointment's.

THE BOUND IS rendezvous_latched_hold_sec, the knob that already bounds a
finished robot's own stand at the cell, and measured the same way (from
the later of arrival and t_meet). A finished peer that has not arrived by
then has either given up on the cell or is standing out its own cap
somewhere this robot cannot hear; waiting longer buys nothing. Without a
bound this would be the unbounded appointment vigil for a peer whose
"leaving" the radio never delivered — at N=2 with no relay, a vigil to the
run's duration cap.
```

### rdv-drain-release-monotone

**The drain release is monotone** — attached to `bool rendezvous_drain_released_ = false;`

```text
MONOTONE WITHIN A VISIT. Once the drain fires, the hold does not re-enter
on a late burst. The counters are bursty by construction — one fused
message can carry thousands of voxels — so a re-entrant hold would
oscillate around R and could pin a robot whose exchange was already done.
A burst that lands after the release is a merge arriving while departing,
which Part 1's counter still records; that is a logging concern, not a
reason to stop the robot again. Cleared wherever rendezvous_settling_ is,
so the next visit re-arms.
```

### midrun-attempt-budget-dispatches

**The mid-run budget counts dispatches** — attached to `int    reconnect_midrun_max_attempts_ = 6;`

```text
Per-run cap on mid-run attempts. Every dispatch costs exploration time;
after this many DISPATCHES the policy has had its chance and the robot
reverts to terminal-only behaviour (logged, so the analysis can see it).

DISPATCHES, NOT FAILURES (comment corrected 2026-09-18; the code has always
done this). `++midrun_attempts_` is spent on the dispatch path regardless
of how the manoeuvre ends, so six SUCCESSFUL mid-run reconnections exhaust
the budget exactly as fast as six failed ones, and nothing ever refunds or
resets it — `midrun_attempts_` is a run-lifetime counter with no clear site.

THAT IS A DOSE TERM, AND IT IS NOT UNIFORM ACROSS THE DESIGN, which is why
it is spelled out here rather than left to the reader:
  * By ARM. Exhaustion removes mid-run chasing, so it bites `pursuit` and
    hybrid's chase half. It barely touches `rendezvous`, whose trigger is
    already suppressed whenever an appointment stands, and not at all in
    `off`.
  * By RUNG. More robots means more outages means the budget is reached
    sooner in mission-elapsed terms. At the smoke's measured mesh uptimes
    (N=3 rendezvous 29%, N=4 14.7%) the N=4 rung spends attempts fastest.
So a run that exhausts the budget is running a WEAKER treatment from that
point on, and the point arrives earlier at N=4 than at N=2. Whether the
budget should refund successes is a treatment-design question for Kalhan,
NOT an arithmetic defect, so it is left alone and made observable instead:
`midrun_attempts_used` on run_end says how close each cell came, which is
what an analyst needs to decide whether the cap bound anything at all. In
the banked smoke cells it never exceeded 1 of 6, but those cells are ~300 s
against ts4's 3000 s.
```

### link-gate-legitimate-reads

**What the link gate may read** — attached to `std::string comms_link_states_topic_;`

```text
--- Link-state gating for the mid-run trigger (see §30.11 and §30.24) ---
THE DEFECT THIS FIXES. Everything above measures silence with a RECORD-AGE
clock: team_last_complete_time_ advances only while peer intents arrive,
and the intent beacon is conditional twice over (it needs an active intent
and a state outside PLAN), so a healthy in-range teammate stuck in a long
PLAN loop is indistinguishable from one behind a hill. Measured against the
emulator's own link trace the record clock ran a median +49.1 s ahead of
real link-down, and 41-42 % of all mid-run fires bought nothing — 16-21 %
of them fired while the radio was UP. A chase is real distance debited from
exploration, so those are pure loss.

WHAT IS AND IS NOT LEGITIMATE TO READ. The emulator publishes one row per
robot pair with [i, j, distance_m, trees_on_link, path_loss_db, snr_db,
ber, bandwidth_mbps, connected]. Only two of those may be touched here:
`connected` for pairs involving THIS robot, and `path_loss_db` solely as
the startup mask (a row the emulator has not computed yet reads
path_loss_db <= 0 with every physical field zeroed, and counting it as a
real disconnection manufactures a reconnection event at t=0 — same rule as
link_logger.py). `connected` is the stand-in for what a real mesh radio
genuinely exposes: a per-neighbour link indication kept alive by MAC-level
keepalives that are unconditional and fast, which is exactly what the
app-layer beacon is not. Reading distance_m, trees_on_link, path_loss_db
as a signal, snr_db, or any pair not involving self would be peer position
through the back door, and using link data to PREDICT reconnection or to
steer the chase would be oracle-driven. None of that is done below.
The topic is a global side channel and reaches a robot the emulator
considers disconnected; that discipline is by convention here, not
enforced by the transport.

"" (the default) leaves the feature OFF and the trigger bit-identical to
the campaigns already banked. Nothing about the `off` arm can reach this:
the whole mid-run block requires reconnect_enabled_, which is false there.
```

### link-down-confirm-debounce

**Withdrawing the link-down debounce** — attached to `double reconnect_link_down_confirm_sec_ = 0.0;`

```text
How long the radio must have been CONTINUOUSLY down before the veto lets a
mid-run chase through. Its own parameter since generation 9; until then
both veto sites borrowed reconnect_confirm_sec (3.0), which is the
team-PRESENCE release confirm and answers a different question. Splitting
them is right at any value: sharing meant anyone retuning the radio
debounce silently moved the presence-release path with it.

0 IS THE DEFAULT, AND THE 30 s DEBOUNCE THAT WAS HERE IS WITHDRAWN.
30 was chosen off the outage distribution (above the 6.2-12.2 s median
flicker, below the 52.0-111.0 s p90 -- nearest-rank over the 13 tags with
>= 10 cells; linear interpolation would read 51.4-101.6) and credited in an
earlier draft of this
comment with moving wasted fires "from 50 % to 33 %". That was
misattributed: the table it pointed at held this conjunct FIXED at 30 and
varied the presence clock, so 50->33 is the 150->90 move, not this
parameter's effect. Measured
properly, with the presence clock pinned at 90 and only this varying:
    confirm   cells arm (off arm)   fires   wasted   beats
       0            18/23            40     20 %     85 %
      30            12/23            26     15 %     92 %
The debounce buys 5 points of fire purity for A THIRD of the arm's
activation: 18 arming cells down to 12, i.e. 6 of the 18 that armed. (An
earlier draft said "a QUARTER" -- that is 6/23, the share of the arm; the
denominator the purity is traded against is the 18 that armed, not the 23
that exist. The six-cell loss the paragraph below already costs out is the
same six, so only the fraction was wrong, not the trade.)
Dilution is the defect generation 9 exists to correct -- 87 %
of g8r1's treated arm was behaviourally the control -- so trading
activation for purity spends the fix on a refinement. The four extra
wasted chases it would prevent cost ~53 s each against run times in the
hundreds to thousands of seconds; the six lost treated cells cost power
that no amount of analysis recovers.

At 0 the veto is exactly "do not chase a peer that is on the radio right
now", which the link_connected_ disjunct at both sites supplies. That guard
is logically necessary and is kept; the debounce on top of it is a tuning
knob the data does not support, so it defaults off and stays available.
```

### link-cols-read-from-layout

**Reading the link table stride** — attached to `size_t link_cols_        = 0;`

```text
Columns per pair in the emulator's link_states table, read off the message
layout rather than assumed. 9 (pre-generation-9) and 10 (with `valid`) are
both accepted; 0 means no message has been parsed yet.

This used to be a hardcoded 9. It was correct for as long as the emulator
published 9 columns, and it failed silently the moment it did not: the
stride walks the flat array, so at N>=3 a 10-column payload read with a
stride of 9 puts row 1 onwards one field out of phase, and what the gate
then reads as `i`, `j` and `connected` are other rows' physical fields. At
N=2 there is a single row and the extra field is simply never reached,
which is the worst version of the bug: it would have passed every
two-robot smoke test on its way into a three-robot campaign.
```

### link-connected-all-not-any

**All links up, not any** — attached to `bool link_connected_     = false;`

```text
Newest reading of MY links: true only when every peer in the index has a
usable row and all of them are up.

"All", not "any", and P7 (N=3) is where the two part company. The veto
this feeds exists to stop a mid-run fire aimed at a team that is already
reachable, and the trigger it vetoes is written against team COMPLETENESS
— so at N>=3 "any" would suppress a dispatch the run needs while one peer
is still out of contact. At N=2 there is exactly one peer and the two
readings are the same bit, which is why this generalisation leaves every
pairwise campaign byte-identical.
```

### link-gate-live-log-latch

**The gate-went-live log line** — attached to `bool link_gate_live_logged_ = false;`

```text
One-shot latch for the "gate went live" console line. The run_start param
link_gate_configured can only say the topics were NAMED; this is the only
record that a usable sample ever actually arrived and the veto was really
in force. Positive evidence on purpose: gate_g8.py check 3f asserts the
line is PRESENT, because asserting the absence of the not-usable warning
would pass a run whose logging broke ([[nav-global-planner-never-planned]]
-- absence of a log line needs a same-binary control).
```

### link-up-last-seen-keeplast

**Measuring link down-duration** — attached to `rclcpp::Time link_up_last_seen_;`

```text
Receipt time at which the link was last OBSERVED up. Down-duration is
measured from here rather than from a stored up->down edge, because
Float64MultiArray carries no header: if the executor stalls in a long PLAN
tick, a backlog delivered afterwards is all stamped at receipt, which would
compress the history and read the down-duration as ~0.

That is also why the subscription below is KeepLast(1) and not deep, and
why it is safe: the VETO reads link_connected_, which comes from the newest
sample and is therefore never staler than linkGateReady's freshness bound.
Down-duration is used only to debounce that veto, so the residual error is
confined and one-sided — a stall that hides an up->down edge makes the
duration read too LONG by at most the stall, which can at worst lift the
debounce a few seconds early on a link that is genuinely down. It can never
fire on a link that is up, which is the 16-21 % this change exists to
remove.
```

### midrun-info-gated-trigger

**The information-gated mid-run trigger** — attached to `double reconnect_min_share_voxels_     = 0.0;`

```text
--- Info-gated mid-run trigger (voxels, not seconds) ---
0 (the default) keeps the fixed silence clock above, bit-identical legacy
behaviour. Positive: the trigger time becomes "when the pair's estimated
unshared map crosses this many voxels", dead-reckoned from the last
contact: T = min_share / (my_rate + peer_rate), both rates frozen in the
LastContact snapshot so the two robots derive (approximately) the SAME
trigger time with no in-outage communication — asymmetric firing is the
lone-waiter failure p7modes measured (5 of 6 holds never reconnected).
T is clamped to [min_silence, max_silence]; rates that read 0 (peer
predates the beacon field, or no samples yet) push T to max_silence, i.e.
"nothing known to share -> wait for the backstop", never a divide-by-0.

Calibrated against p14 (36 comms-on cells, dense world): a chance merge
delivers ~16k voxels median, a commanded one ~332k; by the 240 s clock
the pair is ALWAYS 300-850k apart — so as a deferral filter this gate is
inert in that world, and its live use is EARLIER, timeliness-driven
triggering. Sizing follows from the measured pair divergence (~2.3k
vox/s under this estimator): T ~ V*/2300, so 200k ~ 84 s, 300k ~ 126 s,
500k ~ 210 s. Anything below ~140k lands under the 60 s floor and the
clamp — not V* — becomes the trigger, which is a fixed clock wearing the
gate's name; pick V* above that or the arm tests nothing.
```

### midrun-info-trigger-clamps

**Clamp bounds for the info trigger** — attached to `double reconnect_midrun_min_silence_sec_ = 60.0;`

```text
Clamp bounds for the derived trigger time. The floor guards the radio-
flicker regime the release-confirm machinery expects (a healthy teammate
in a long PLAN loop can read missing ~180 s; an info trigger BELOW that
knowingly accepts some chases at busy-not-lost teammates — that is the
eager variant's cost, so the floor is a parameter and not 200 hardcoded).
The ceiling is the insurance policy: the ONE effect p14 proved is that
mid-run reconnection caps the worst outage (515 -> 308 s, p=0.0095), and
no info gate is allowed to trade that away by deferring forever. It
therefore defaults to the LEGACY CLOCK, not above it: the gated arm can
then only ever fire earlier than the control, so the proven cap is a floor
on its behaviour and the comparison carries no "gated runs waited longer"
confound. A ceiling above reconnect_midrun_silence_sec is a deliberate
choice to give that up. Because it is defined as TRACKING that clock, the
initialiser below is only a mirror for readers: the value that actually
takes effect is defaulted to the resolved reconnect_midrun_silence_sec at
the parameter-read site, so the two cannot drift apart the way they did
when generation 9 moved the clock and this line was hand-copied after it.
THE FLOOR NOW BINDS OVER MOST OF THE USABLE RANGE, which it did not when
the ceiling was 240. With the ceiling tracking 90, T = V*/2300 is pinned to
the ceiling for anything above ~207k voxels and to the 60 s floor below
~138k, so the window in which V* -- rather than a clamp -- actually decides
is roughly [138k, 207k]. It was [138k, 552k] at a 240 s ceiling. The
sizing note above ("300k ~ 126 s, 500k ~ 210 s") describes the UNCLAMPED
derivation and both of those now clamp to 90. Anyone enabling the info
gate at generation-9 defaults must either pick V* inside that narrow
window or raise the ceiling deliberately, otherwise the warning that
comment gives -- a fixed clock wearing the gate's name -- is what they get.
Both are inert while reconnect_min_share_voxels is 0, which is the default,
so nothing in the shipping configuration depends on this.
```

### midrun-cooldown-stamped-at-end

**Stamping the mid-run cooldown at the end** — attached to `rclcpp::Time midrun_last_end_;`

```text
Cooldown stamped when a mid-run manoeuvre ENDS (transitionTo, where
reconnect_active_ falls) — never at dispatch: missing_for stays satisfied
for the whole outage, so a dispatch-stamped cooldown expires DURING the
manoeuvre and the "resume" becomes a one-tick interlude in an endless
re-dispatch loop. Bool-guarded: rclcpp::Time default-constructs on the
system clock and subtracting it from a sim-time now() throws.
```

### hold-escalation-deadlock-break

**Escalating an expired terminal hold** — attached to `bool   hold_escalate_          = true;`

```text
--- Hold escalation (mutual-hold deadlock break) ---
When a TERMINAL barrier wait expires, drive once to the last-connected
anchor and wait hold_escalate_wait_sec more before giving up. Both robots
converging on their own last-contact poses restores the pair geometry the
link last worked at, which breaks the pure-hold fixed point (observed: 5
of 6 holds never reconnected; the parked pair can only be rescued by peer
motion). Sticky per-manoeuvre flag, NOT a position test: an unreachable
anchor must not re-escalate on every expiry forever. Inert under the
field default rendezvous_max_wait_sec=0 (wait forever, so no terminal
barrier ever expires): it can only act where an escape hatch is already
configured, and there it converts a give-up into one more attempt.

IT IS NOT INERT IN A CAMPAIGN. run_explo_sim_rviz.sh passes
rendezvous_max_wait_sec=$RDV_MAX_WAIT, default 600 (:429), so terminal
barriers DO expire and this escalation is live on every arm. Read the
paragraph above as the field-deployment case only.
```

### reconnect-arrive-tolerance

**Arrival tolerance for manoeuvre destinations** — attached to `double reconnect_arrive_tol_m_ = 4.0;`

```text
(reconnect_rec_, the pair the current manoeuvre was armed from, is declared
with the other LastContact members below — the struct is only forward
declared here.)

Arrival tolerance for a manoeuvre destination. NOT goal_xy_tolerance (0.4 m,
an exploration figure): a manoeuvre destination's whole value is
CONNECTIVITY, not position, and 0.4 m demands a precision the point does not
deserve. Two robots that each stop within this of the same meeting point are
at most 2x this apart — 8 m at the default, against a link that was still
carrying traffic at 54 m in the run that motivated the meeting point.

This is also what makes an obstructed destination cheap. The original
argument was about the MIDPOINT, which was synthetic and never checked
against the map — deleted 2026-09-16 — but the tolerance is still doing the
same job for the destination that replaced it. The agreed cell IS checked
against the map at derive time, so it is not synthetic; what it is not is
checked against the map AT ARRIVAL, tens of seconds and one merge later,
and the comms emulator kills a link by counting trunks within the Fresnel
radius of the segment between the pair — so exactly the places worth
meeting at are the places with trunks near them. Demanding 0.4 m there
means grinding against an obstacle until the budget expires; 4 m means
arriving beside it and waiting, which is all the manoeuvre ever needed.
```

### rdv-present-tolerance-failed-drive

**Counting a failed drive as present** — attached to `double rendezvous_present_tol_m_ = 10.0;`

```text
How far from the agreed cell a FAILED appointment drive may stop and still
count as standing at the meeting. Looser than reconnect_arrive_tol_m_ on
purpose: that tolerance decides when a drive has SUCCEEDED, this one decides
whether a drive that gave up ended somewhere the barrier still means
something, and a robot wedged three metres off the cell is at the meeting in
every sense that matters.

It exists because the two watchdogs in doReturnNav used to hand the barrier
an arbitrary pose. Their note argues a stopped robot is "at least closer to
comms than where exploration stranded us" — true of the anchor return it was
written for, and false of an appointment, where the OTHER robots are at the
cell and nowhere else. The ts4 gen-30 N=2 rendezvous seed8 cell priced it:
one robot reached the cell (1.3 m), its partner's drive died 40.0 m away and
joined the barrier from there, leaving the pair 41.3 m apart with three
trunks between them — past the 30 m horizon, so the presence each was
waiting on could never arrive. Both stood still for 2609 s and the run was
killed by the harness hang detector.

Sized against the link, not the cell grid, though the two happen to agree:
two robots inside this of one cell are at most 2x it apart, so 10 m keeps a
whole meeting inside a 20 m spread against a 30 m horizon, with the margin
spent on trunks. Wider trades that margin for fewer escape ladders.
```

### rdv-escape-ladder-attempts

**The appointment leg escape ladder** — attached to `int    rendezvous_escape_max_attempts_ = 3;`

```text
Escape manoeuvres one appointment leg may spend before its watchdogs stop
re-aiming at the cell and roll the robot to the next rung of the agreed
recurrence. Separate from return_escape_max_attempts_, which governs the
mission-return ladder, so one knob does not silently retune two subsystems
that fail for different reasons at opposite ends of a run.

0 disables the ladder and restores the straight-to-roll behaviour, which
makes it the A/B control for the ladder itself. The ceiling that actually
bounds the cost is return_escape_leg_sec (30 s), shared with the homing
ladder because it describes the same manoeuvre: three rungs is at most
90 s of driving away from a meeting whose barrier waits without limit.
```

### reconnect-nav-leg-ceiling

**Ceiling on one manoeuvre leg** — attached to `double reconnect_nav_max_sec_ = 600.0;`

```text
Ceiling on ONE manoeuvre drive leg. The distance-true budget in
startReturnTo is deliberately exempt from nav_max_timeout_sec (see there),
but "exempt" was unbounded: at nav_speed_estimate 0.15 x safety 3.0 = 20 s/m
a 40 m leg authorises 800 s, and the meeting point sits about half the pair
separation further out than the own-pose anchor it replaced. The invariant
this restores: no single leg may cost more than the barrier it is driving
toward, so a manoeuvre cannot outspend its own purpose. <= 0 = unbounded.
```

### reconnect-release-confirm-flicker

**Release confirmation against flicker** — attached to `double reconnect_release_confirm_sec_ = 6.0;`

```text
--- Release confirmation (flicker guard) ---
A single live claim releases a manoeuvre and resets the silence clock,
crediting a "reconnection" on a range-edge flicker that drained no map
deltas. A positive value requires the release condition to hold
continuously this long.

MUST EXCEED coord_claim_ttl_sec, and the old default (3 s, matched to
reconnect_confirm_sec) did not. Liveness is `receipt + ttl > now`, so ONE
packet at t holds the peer live until t+5 unaided — and a 3 s window is
satisfied at t+3 by that single packet. The guard let through exactly the
flicker it was written to stop. 6 s needs the claim genuinely refreshed at
least once (the beacon is 1 Hz), which a real reconnection does and a
range-edge blip does not. 0 = release on first read (legacy).
```

### team-back-dwell-windows

**Two dwell windows for four sites** — attached to `double release_ok_since_sec_    = 0.0;`

```text
FOUR SITES ACT ON "THE TEAM CAME BACK" AND ALL FOUR DWELL (generation 23,
fourth site generation 25). The confirm length is the SAME parameter for
all of them, because the flicker they filter is one phenomenon and two
constants in a yaml drift.

  release_ok_*   the manoeuvre barrier (doReturnSync / doReturnNav)
  team_back_ok_* cancelling a standing appointment because the outage ended
                 (doPlan, P5) AND clearing rendezvous_spent_ so a new
                 appointment may arm (heartbeatTick) AND converting an
                 appointment walker to the barrier because the outage
                 ended under it (doReturnNav, generation 25)

TWO WINDOWS, NOT FOUR, and the split is by PREDICATE rather than by call
site. The barrier asks manoeuvreReleaseEligible / teamComplete-or-arrived;
the other three ask teamSettled(accountedPeerCount(now)) — character for
character the same question, on the same clock, with the same threshold.
Giving one question two independent windows is how "the outage is over"
acquired two meanings the last time, and one of the two would answer on
evidence the other was still refusing.

ONE WRITER, AT 1 Hz, IN heartbeatTick. This is the load-bearing detail.
dwellConfirmed measures a CONTINUOUS run, so it is only as continuous as its
sampling: a window ticked from inside `if (appointment_armed_ && ...)` in
doPlan would not decay while the branch was untaken, it would FREEZE, and
the first sample after a gap would find `now - since` already past the
confirm time and fire on one reading. Both hazards are real here — doPlan
runs only in State::PLAN, i.e. roughly once per step rather than at 10 Hz,
and the appointment gate is untaken for most of a run. The heartbeat is
gated on nothing but coord_ and runs at coord_heartbeat_hz, which is also
the beacon rate, so it samples the evidence exactly as fast as the evidence
changes. doPlan's P5 and doReturnNav's walker conversion then READ the
window with dwellHeld, each conjoined on its own live count, and cannot
disturb it.

Both added sites dwell in the SAFE direction. Superseding late leaves an
appointment standing a few seconds longer than strictly needed; superseding
on a flicker cancels a meeting the peer is still driving to. Releasing the
latch late delays the next arming; releasing it on a flicker hands out a
second arming inside one outage, which is the generation-19 ratchet.
Seconds, not rclcpp::Time — see dwellConfirmed.
```

### terrain-relative-z-mode

**Terrain-relative z mode** — attached to `bool  terrain_relative_z_ = false;`

```text
Terrain-relative (3D) mode. When true:
 - roi_min_z_/roi_max_z_ are interpreted RELATIVE to the robot's current
   z: the fused-map ingest clip, frontier extraction and the FOV ray
   z-clip all use [robot_z + roi_min_z_, robot_z + roi_max_z_], re-banded
   as the robot climbs/descends (see loadLatestMap);
 - candidates are snapped to local ground + candidate_z_clearance
   (CandidateGenerator terrain mode), so published goals carry a real 3D
   z. simple_nav_3d's UGV role consumes only (x, y, yaw) — its arrival test
   is planar — so on a UGV the z rides along for 3D consumers/RViz, or is
   zeroed when flatten_goal_z_ is set. Its UAV role does measure z, so there
   the goal z is load-bearing rather than a passenger.
When false, all z handling is the legacy absolute flat-world behaviour.
```

### exploit-target-timeout-budget

**The per-target give-up budget** — attached to `double exploit_target_timeout_sec_ = 300.0;`

```text
Per-target give-up budget (s): wall-clock on one trunk since the last
progress event (activation, approach arrival, completed dwell). When it
expires the target closes PARTIAL and exploration resumes. <=0 disables.
Must stay ABOVE nav_max_timeout_sec: failed hops keep charging it (that is
the termination guarantee for unreachable rings), so a smaller value gives
up after the FIRST failed hop with no retry. 300 = one worst-case failed
hop (180) + 120 to re-select and reach another angle.
```

### exploit-dwell-sync-barrier

**The vantage-ring dwell barrier** — attached to `bool   exploit_dwell_sync_enabled_ = true;`

```text
Vantage-ring rendezvous barrier. When true (the default) a robot standing on
its vantage does NOT start its dwell clock until every peer holding an
exploit claim on the same trunk is standing on the vantage IT claimed: the
team requirement is simultaneous capture of one trunk state, not three
sequential single-robot dwells. Inert with coordination off or with no peer
claim on this trunk (single-robot runs are bit-for-bit unaffected).
```

### exploit-dwell-sync-max-wait

**Give-up for the dwell barrier** — attached to `double exploit_dwell_sync_max_wait_sec_ = 0.0;`

```text
Barrier give-up (s), measured from EXPLOIT_DWELL entry and NOT from the
re-anchored dwell start. <= 0 (the default) waits until the peer actually
arrives; see the release-path argument in doExploitDwell for why that
terminates. Set a positive value only to force a deadline: a wall-clock
bound cannot distinguish a distant teammate from a wedged one, and 60 s
abandoned one that needed 111 s to cross the plot.
```

### exploit-fine-region-relay

**Relaying fine-TSDF refinement regions** — attached to `bool   publish_refinement_regions_ = true;`

```text
Fine-TSDF region relay. When true, every ingested TreeTarget is registered
as a RefinementRegion on this robot's scovox_node the moment it arrives
(target release == exploitation-phase start) and unregistered when
finishActiveTarget closes it — so fine grids exist only while a tree is
under exploitation. Removal keeps already-fused fine voxels (the region
gate is integration-time policy, not storage); a scovox_node running with
fine_ratio_log2 = 0 ignores the messages, so this is safe to leave on.
```

### exploit-target-timer-rules

**Per-target give-up timer rules** — attached to `uint32_t exploit_target_started_id_  = 0;`

```text
Per-target give-up timer: the active target id we started timing and when
(sim seconds). Latched when a target becomes active; re-armed on every
progress event (approach-waypoint arrival, completed dwell); refunded for
proximity-hold time; cleared on rendezvous stand-down so a re-activated
target starts fresh. Failed navigation hops deliberately keep charging it
— that is what makes an unreachable ring terminate PARTIAL.
```

### exploit-dwell-sync-bookkeeping

**Dwell-sync barrier bookkeeping** — attached to `rclcpp::Time dwell_sync_wait_start_;`

```text
Dwell-sync barrier bookkeeping, re-initialised on every EXPLOIT_DWELL entry
(transitionTo). The wait clock CANNOT be state_enter_time_: the barrier
holds by re-anchoring that to now every tick, so a wait measured from it
would read ~0 forever and max_wait would never fire. The latch is what
stops a timed-out barrier from re-entering and re-anchoring the dwell clock
on the next tick, which would leave the dwell never completing.

dwell_sync_started_ is the same guarantee for the SUCCESSFUL release: the
barrier is a start condition, not a per-tick precondition, so once the team
is staged the dwell runs to completion. Re-testing the probe every tick
makes a running dwell hostage to the team's schedule — a peer that finishes
its own capture and drives off to its next angle publishes staged=false
again, and the barrier then re-anchored the dwell clock of a robot that had
been motionless on its vantage for seconds (observed: ~3 s of accumulated
dwell discarded and re-dwelled from zero when the peer hopped v1 -> v0,
i.e. a full extra 8 s capture charged per peer departure).
```

### exploit-held-vantage-pose

**The vantage pose a robot keeps** — attached to `CandidateViewpoint held_vantage_pose_;`

```text
The vantage THIS robot last completed a dwell on — pose (position + the
capture yaw), ring index, and which target it belongs to. This is the
pose the robot parks on when the rest of the ring is covered by the team
(doExploitPlan's hold branch): the requirement, verbatim from the field
operator watching the 2-robot sim, is that the robot which does NOT get
the last vantage "should have kept the first vantage pose". Validity is
the id match — target ids are unique for the life of the queue, so a
stale entry from a finished target can never match the next one and no
explicit invalidation is needed.
```

### exploit-flood-cache-key

**Caching the unbounded exploit flood** — attached to `bool exploit_flood_valid_ = false;`

```text
Cache key for the UNBOUNDED exploitation flood of cost_grid_ (see
doExploitPlan). That flood is O(grid) and EXPLOIT_PLAN re-enters at the
full tick rate whenever no vantage is selectable, so it is rebuilt only
when its inputs change. exploit_flood_map_ is an identity handle for the
latched map object — compared, never dereferenced. Invalidated by doPlan's
radius-bounded flood, which overwrites the same grid.
```

### coverage-latch-one-way

**The one-way coverage latch** — attached to `bool   coverage_latched_        = false;`

```text
Coverage termination latch (done_criterion == "latch"). One-way: set on the
first qualifying sample and never cleared, so a map that wobbles back above
threshold — or a merge that re-frontiers the ROI — cannot un-finish a robot
that has already reported finished. The two stamps are recorded because the
latch instant is the endpoint this criterion defines, and it is NOT the same
as the run's t_sim end (which is set by the SLOWER robot, plus teardown
grace); an analysis needs both to separate the two.
```

### teamworld-finished-announced-latch

**The monotonic finished announcement** — attached to `bool   finished_announced_      = false;`

```text
The MONOTONIC form of "this robot's run is over", and the only thing
TeamWorld/finished is ever published from. Latched once
(coverage_latched_ || state_ == State::DONE) first holds and never cleared.
It exists because that disjunction is not itself monotonic — DONE can be
left through the exploit sub-loop — and the wire bit is RELAYED by peers,
which is only sound for a bit that cannot go back to false. See
publishTeamWorld and TeamWorld.msg/robot_finished.
```

### teamworld-homing-announced-latch

**The monotonic homing announcement** — attached to `bool   homing_announced_        = false;`

```text
The same construction one level lower: the MONOTONIC form of "this robot
has left for home", and the only thing TeamWorld/mode is ever published
from at the HOMING level. Latched once (state_ == State::RETURN_HOME ||
mission_return_done_) first holds and never cleared.

WHY IT IS NOT JUST `state_ == State::RETURN_HOME`. The wire field is
relayed and max-merged by peers, so — exactly as for finished_announced_ —
it is only sound for a level that cannot go back down; a peer holding a
relayed HOMING has nothing that could ever clear it. The return leg reads
as monotone TODAY (doReturnHome contains no transitionTo, and
startReturnHome is guarded run-scoped by mission_return_done_ and
leg-scoped by the state test), and mission_return_done_ carries the level
past the arrival, when state_ has already moved on to DONE. Latching it
here makes the guarantee local to the publisher rather than a property of
the return leg's current shape — so making homing resumable later cannot
silently turn a relayed level false. See TeamWorld.msg/mode.
```

### teamworld-done-announced-latch

**The DONE level latch** — attached to `bool   done_announced_          = false;`

```text
The DONE level's latch, and since 2026-09-23 NOT the same latch as
`finished`. Latched once this robot is finished AND no longer keeping an
appointment (announcedMode, meeting_attendance.hpp). A finished robot on
its way to the meeting, or standing at it, is still taking part, and the
level a partner's barrier reads as "it is leaving" must not say otherwise.
```

### rdv-at-rendezvous-hold

**The at-the-rendezvous hold** — attached to `double coverage_latch_hold_start_sec_ = -1.0;`

```text
--- The at-the-rendezvous hold (2026-09-17, generation 21) ---
Saturating the map while STANDING AT the agreed cell used to end the run on
the spot: maybeLatchCoverageDone routed straight to startReturnHome, so a
robot that had already arrived walked away from the meeting — possibly
seconds before the peer it was waiting for got there. The appointment's
whole value is that somebody is present when the other robot arrives, and
the saturated map this robot is carrying is exactly what the meeting exists
to hand over.

WIDENED TO EVERY FINISHED ROBOT THAT HAS AN APPOINTMENT (2026-09-21,
generation 32). Standing on the cell was never the property that mattered —
holding an agreement the peer is also keeping is — so a robot that
saturates its map anywhere now goes to the agreed cell and homes after the
meeting (keepAppointmentOnFinish). The hold below is what bounds it once it
gets there, and it is therefore also the bound on the case that made the
widening necessary: a peer standing at the cell for the whole run because
the robot it agreed with had quietly gone home.

THE EXPLORATION ENDPOINT IS UNTOUCHED. exploration_complete is stamped
before the hold is taken (same line it was always stamped on), so
explore_done_sim_sec still measures when this robot's map saturated and not
when the barrier let it go. What the hold changes is only what the robot
DOES afterwards, which no exploration metric reads.

`coverage_latch_hold_start_sec_` is the hold's own clock, and it is
mission-elapsed at whichever came LAST of the latch and reaching the
barrier: doReturnSync's `waited` runs from the tick the robot entered
RETURN_SYNC, which is before the latch, and capping against that would
charge the hold for time the robot spent waiting while it still had
exploring left to do. The latch stamps it for a robot already standing
here; doReturnSync stamps it for one that finished on the road, so that
the drive to the cell is not charged to it either.

`coverage_latch_teardown_` is what the appointment outcome classifier reads
to tell "the meeting failed" from "this robot's run ended at the meeting".
Without it a coverage-latched teardown logs `no-show` with `arrived=true` —
observed on all three robots of the smoke20 N=3 rendezvous cell while all
three were standing on cell 45 together, i.e. a no-show recorded at a
meeting that had happened. Sticky once set: the run is over.
```

### done-seek-post-latch-coast

**Coasting after a mid-manoeuvre latch** — attached to `bool   done_seek_enabled_       = false;`

```text
--- Post-latch coast (done_seek_enabled) ---
A latch that lands MID-MANOEUVRE currently cancels the chase: the robot
brakes, discards its frozen contact pair, and the partner keeps waiting at
a barrier for a robot that is no longer coming. That is not an edge case —
it is the modal way a manoeuvre ends, 56 of 88 reconnect_end events across
the banked campaigns and 58-76% within every hybrid arm.

Coasting keeps the navigator goal the robot ALREADY had, so it finishes the
drive toward its partner and delivers its map. Crucially it costs nothing
in the metric: state_ reads DONE from the same tick, so the harness's
completion rule (every planner reads DONE) never sees the difference, and
this robot's own clock stopped at the latch regardless.

OFF by default, and deliberately a RUNTIME switch rather than a build:
every banked campaign ran without it, and the A/B for it has to sit inside
ONE run_campaign.sh invocation or the arm is confounded with the session.
```

### mission-return-home

**Returning to the start pose** — attached to `bool   mission_return_enabled_  = false;`

```text
--- Mission return (mission_return_enabled) ---
Arm-invariant platform behaviour: at ANY terminal exploration ending the
robot drives back to its recorded start pose, so both arms end in the same
connected configuration (spawns are 3 m apart) and "mission end" is a
well-defined endpoint in the off arm too. When enabled it pre-empts BOTH
legacy endings — the park-in-place AND the done_seek coast (the branch in
maybeLatchCoverageDone runs before the coast gate, so done_seek_coasting_
is unreachable under mission return).

have_home_ is a separate ONE-SHOT latch, deliberately not have_pose_:
have_pose_ is a revertible TF-health flag that can drop and come back,
and re-recording "home" mid-run would send the robot to wherever TF last
hiccuped. home is captured exactly once, at the first successful pose.

mission_home_tol_m is its own knob because the two homes are only 3 m
apart — reconnect_arrive_tol_m (4.0) would accept the PARTNER's home.
```

### mission-start-hold-all-arms

**The pre-mission hold in every arm** — attached to `double mission_start_hold_sec_  = 60.0;`

```text
THE PRE-MISSION HOLD: no robot plans or navigates until this much mission
time has passed, IN EVERY ARM (2026-09-17, generation 22).

It exists for the rendezvous agreement and it is applied to all four arms
anyway, and both halves of that are deliberate.

WHY IT EXISTS. The protocol's one free window is the start of the run, when
the team is co-located and every link is up — the derive gate says so in as
many words, and the measurement backs it: the centroid placeholder commits
with a full echo round, on every robot, within about 21 ms. What was NOT
guaranteed is that the window lasts long enough to also carry the ONE
upgrade the proposer is allowed. It did not, in the ts4 smoke20 N=3 hybrid
cell: bestla's last peer went silent at t+65.1 s and atlas authored the
upgraded triple at t+65.7 s, 0.6 s later. bestla kept the 26-s-stale
placeholder, atlas and husky took the upgrade, and the fleet drove to two
different cells and waited out the rest of the run in two places. The team
had dispersed before it had finished agreeing.

So this does not add a rule to the protocol — it makes the protocol's
existing precondition TRUE instead of merely likely, by refusing to start
dispersing until the window it needs has actually elapsed.

WHAT THIS HOLD DOES NOT DO, because it shipped for one day claiming to.
A companion edit confined the provisional->final upgrade to this window, on
the theory that an upgrade authored inside it cannot split the fleet. The
theory is sound and the implementation was measured to be worthless: the
upgrade needs candidate cells, candidate cells come from the allocator's
tours, tours come from completed exploration steps, and a robot held here
completes none. Confining the upgrade to the hold does not schedule it
earlier, it prevents it forever. The full measurement is in the derive gate
in maintainRendezvousProposal; the conjunct is gone and must not come back.
What remains here is only the first half: agree the placeholder while
everyone is still standing together.

WHY ALL FOUR ARMS, including the two that never run the protocol. The hold
is dead time on the primary endpoint, so applying it only where it is
needed would add a fixed handicap to rendezvous and hybrid and to nothing
else — a startup cost masquerading as a treatment effect, in the exact
comparison the campaign exists to make. Applied uniformly it is a constant
shared by every arm: it cancels in every between-arm contrast and is
subtractable from every absolute completion time.

WHY 60 s. The hold has exactly one job — carry the INITIAL agreement, the
centroid placeholder, while the fleet is still co-located — and 60 s is far
more than that job needs. The placeholder commits with a full echo round on
every robot within about 21 ms of the first derive that has a snapshot; the
derive retries every kRendezvousBootstrapPeriodSec (5 s) until one lands,
and publishTeamWorld re-broadcasts the result at team_world_hz (1 Hz). So
60 s buys roughly twelve derive attempts and sixty re-broadcasts under
guaranteed mutual contact, against a handshake that normally completes on
the first one. The margin is there for a late map, not for the protocol.

THIS NUMBER WAS 120 s FOR ONE DAY (2026-09-17) AND THE ARGUMENT FOR 120 IS
WITHDRAWN. It was: the window must also contain the provisional->final
upgrade, whose first occurrence was swept at 41-77 s over the banked cells,
so 60 s sat below the N=2 minimum of 61.6 s. Every word of that is true and
it is irrelevant, because the upgrade is no longer confined to the window —
confining it was measured to delete it rather than schedule it, since the
upgrade's input is completed exploration steps and a held robot completes
none. With the confinement gone the upgrade fires whenever the allocator
first has tours, at t=41 s or t=353 s, hold or no hold. The hold neither
helps nor hinders it, so the sweep no longer constrains this number at all.

DO NOT RE-DERIVE THIS NUMBER FROM THAT SWEEP. If a future change re-couples
the upgrade to the hold, the sweep is still not the right input — the right
input is the step counter, and the answer it gives is that no hold length
works. See the derive gate in maintainRendezvousProposal.

TOO SHORT IS NOT A CORRECTNESS RISK. Whatever the team has committed when
the hold closes is agreed by construction, because every commit before that
point happened with the fleet co-located and mutually whole. The failure
mode of a hold that is too short is that the fleet leaves with no pair at
all and the first arming refuses — which the smoke's Q0/Q1 catch directly,
and which 60 s is twelve retries clear of.
```

### homing-leg-clocks-same-interval

**Homing leg clocks share one interval** — attached to `rclcpp::Time return_home_start_time_{0, 0, RCL_ROS_TIME};`

```text
Wall clock and hold clock for the leg, stamped together with the distance
baseline above so all three measure THE SAME INTERVAL (2026-09-18).

mission_complete used to derive its duration from state_enter_time_ while
taking its distance from return_home_dist_at_start_, and those are not the
same window the moment a proximity hold interrupts the leg: the release at
checkProximityHold BACKDATES state_enter_time_ by the drive time already
spent, deliberately, so the nav budget and the mission_return_max_sec cap
CONTINUE across the hold instead of restarting. That is right for a budget
— a commanded yield to a teammate is not the robot's own time to spend —
and wrong for a report: it made homing_duration_sec a drive-only clock
shipped next to a whole-leg distance, so distance/duration overstated the
homing speed by exactly the held fraction.

NOT A RARE SEAM, BUT NOT "BY CONSTRUCTION" EITHER, and the arithmetic here
read 5.0 m until 2026-09-18. That is the yaml field default
(shared_params.yaml:1107) and no campaign uses it: run_explo_sim_rviz.sh
passes -p proximity_hold_dist_m:=$PROX_HOLD_M with PROX_HOLD_M:-1.5 (:175),
and says why — a 5 m disc has each husky braked by the teammate standing on
the next vantage angle ~3.5 m away, which would spend the run in
PROXIMITY_HOLD. At 5.0 m against homes 3 m apart two robots homing at once
would indeed hold each other unavoidably; at 1.5 m (resume 2.5 m) they park
OUTSIDE each other's disc and the hold is a transient of the approach, not
a certainty. The bias is real and arm-correlated all the same — the arms
that regroup home together more often than the ones that do not — and an
arm-correlated bias in a reported quantity is the kind that survives into a
result, however often it fires. It just does not fire on every pair.

So: this pair gives mission_complete a true wall-clock leg and the held
time inside it, separately. The BUDGET still runs off state_enter_time_
and is untouched — homing_duration_sec can therefore legitimately exceed
mission_return_max_sec without the cap having failed, by up to the held
time, and homing_held_sec is how you tell that apart from an overrun.
Explicitly RCL_ROS_TIME, not default-constructed: a default rclcpp::Time
is on the SYSTEM clock, and subtracting mismatched sources throws rather
than returning a wrong number. Every caller of finishMissionReturn sits
inside RETURN_HOME, which only startReturnHome can enter, so the stamp is
always set in practice — this is so that if that ever stops being true the
failure is a nonsense duration in one row and not an exception unwinding
the run's last event.
```

### homing-goal-publish-gate

**Gating the deferred home goal publish** — attached to `rclcpp::Time home_pub_not_before_{0, 0, RCL_ROS_TIME};`

```text
Publish gate for the deferral above. "Next tick" alone is NOT a gap: a
backlogged executor fires queued tick callbacks back-to-back, and
mr0pilot's logs show the "deferred" publish landing 0.2-0.4 ms after
abandonNavGoal — with the async cancel-all still in flight and able to
swallow the fresh goal. A wall-clock gate cannot be defeated by timer
catch-up. ROS time: compared against this->now() (sim time in runs).
```

### homing-breadcrumb-trail

**The homing breadcrumb trail** — attached to `std::vector<Eigen::Vector3f> home_trail_;`

```text
Breadcrumb trail for the retrace fallback. The nav global planner never
plans (66/66 robot-logs, §28 of the experiment doc), so a long direct
home goal is greedy local navigation and can trap in a local minimum
(mr0pilot_hybrid_seed4 bestla: parked 31 m out). The trail is the
robot's own outbound positions at >=2 m spacing — ground it has already
traversed once — recorded from home capture until homing starts. The
second no-progress retry follows it back crumb by crumb.
```

### home-approach-watchdog-rationale

**Approach-based homing watchdog rationale** — attached to `HomeMode home_mode_ = HomeMode::DIRECT;`

```text
---- Approach-based homing watchdog (binary generation 5) ----
The original no-progress watchdog measures GROSS metres travelled, so a
robot orbiting a local minimum at 0.05 m/s satisfies it forever while
netting zero approach: mr1_hybrid_seed11 atlas hit the 600 s cap 48.75 m
from home after 30.35 m of travel with ZERO watchdog fires. This second,
independent watchdog measures the distance that actually matters —
remaining distance home — and the two are reported separately
(kind=approach vs kind=frozen) so each failure mode stays attributable.

Mode also replaces the old return_home_retrace_ flag: with an escape leg
in the ladder there are three publishable targets, and two booleans for
three states is how they drift apart.
```

### home-escape-geometry-band

**Escape geometry and the near-home band** — attached to `static constexpr float kEscapeBandMinM      = 1.5f;`

```text
Geometry constants, all tied to the 2 m breadcrumb spacing: an escape
target is a crumb 0.75x-3x the spacing away, arrival is one spacing, and
the approach check is suppressed inside return_approach_suppress_m_ because
there the 1.0 m threshold is a large fraction of what is left.

That suppression used to claim the frozen detector and the nav budget still
covered the band. They do not, and the band is wider than the arrival test:
arrival needs <= mission_home_tol_m (1.0 m), the mute radius is 3.0 m, and
between the two a robot that keeps MOVING without closing the gap passes
the frozen test every window (it accumulates distance) while the approach
test is switched off. Only the overall nav budget remained, minutes away.
Measured on ts1b: 7 homings parked in that band for 221-565 s against a
70.9 s median homing, 6 of which set their cell's makespan. The band clock
above closes it -- see doReturnHome.
```

### home-watchdog-no-test-delta

**kNoTestDelta is not a sentinel** — attached to `static constexpr double kNoTestDelta        = 0.0;`

```text
Placeholder passed on home_watchdog rows that record no detector
inequality (kind="escape-end", a leg termination rather than a fire).

It is NOT a sentinel and must never be read as one: the writer omits both
test fields entirely on those rows, keyed on `kind`, so absence — not a
magic value — is what marks "no inequality here". A numeric sentinel
cannot work for this field in either direction: 0.0 is the canonical
frozen fire (a robot that moved exactly nothing) and negatives are the
canonical receding approach fire (-0.03 m in g6pilot_hybrid_seed103), so
every candidate value is also a real measurement.
```

### rendezvous-last-connected-anchor

**The last-connected rendezvous anchor** — attached to `Eigen::Vector3f last_connected_anchor_ = Eigen::Vector3f::Zero();`

```text
Rendezvous anchor: the robot pose the last time it heard a teammate. That
pose sits inside the comms bubble, so it is the cheapest point to return to
for reconnection. Recorded on every peer intent (see the intent callback);
have_anchor_ stays false until the first peer is heard (single-robot runs
never rendezvous).
```

### mesh-last-contact-record

**Per-peer last-contact record** — attached to `struct LastContact {`

```text
Per-peer last-contact record (mesh reconnection). The mobile-radio
generalisation of the anchor: at every received intent, the PAIR of poses
that made the link — mine and the peer's advertised one — plus the peer's
declared goal, which is the freshest hypothesis of where it went (the
pursuit trail head). Keyed by robot_id; stamped with LOCAL receipt time
(the same clock discipline as claim expiry — peer stamps are untrusted).
```

### last-contact-map-size-snapshot

**Map-size half of the contact snapshot** — attached to `double self_voxels = 0.0, peer_voxels = 0.0;`

```text
Map-size half of the snapshot (info-gated mid-run trigger). Both sides
of the pair at the moment of contact: my cached count/rate and the
peer's beaconed ones. The peer's record of ME holds my last-BEACONED
values while mine holds my cached-at-receipt values — up to one beacon
period apart, which is noise against the ~metrics-period sampling both
are quantized to and the 15-60 s PLAN-loop dispatch jitter. rate 0.0
means "unknown" (pre-field peer or no samples yet), and the gate falls
back to the time-only trigger rather than divide by it.
```

### pursuit-trail-and-budget

**Pursuit trail and budget bookkeeping** — attached to `std::vector<Eigen::Vector3f> pursue_waypoints_;`

```text
Pursuit bookkeeping (valid while state_ == PURSUE). The trail is the
waypoint list startPursuit builds from the missing peer's record — goal
first (where it was heading), then its last heard pose (sweeps the leg it
was driving; the mesh lights up the moment any point of it is in range).
The budget clock runs from pursue_start_time_ across ALL waypoints;
proximity-hold time is refunded to it (a hold is not chase progress lost).
```

### pursuit-rec-write-only-snapshot

**Why pursue_rec_ is kept write-only** — attached to `LastContact  pursue_rec_;`

```text
Snapshot of the record the chase was armed from. WRITE-ONLY SINCE
2026-09-16 and kept deliberately. Its one reader was pursuitFallback's
hybrid branch, which derived a meeting point from this pair — the last of
the three midpoint drives, all now deleted. The member survives because
the snapshot DISCIPLINE it documents is still load-bearing next door
(reconnect_rec_, below), and because a chase that re-read last_contact_
mid-drive is a mistake this codebase has made before: keeping the armed
record visible is cheaper than rediscovering why it was taken. If a future
reader wants it gone, delete startPursuit's write with it — do not leave
the write and call the member live.
```

### reconnect-rec-manoeuvre-snapshot

**One contact snapshot per manoeuvre** — attached to `LastContact  reconnect_rec_;`

```text
The pair the CURRENT manoeuvre was armed from — the same discipline as
pursue_rec_, applied to the whole manoeuvre rather than just the chase.
Any target a manoeuvre derives from a per-peer record is derived from THIS
snapshot and never from a re-read of last_contact_. The convergence
argument that made it matter is gone with the midpoint drives (2026-09-16)
— the destination is now the team-wide agreed cell, which no robot derives
from a contact record at all — but the discipline is still what keeps the
hold escalation's fallback stable: a one-way packet heard while we drove or
waited must not move the target this robot is already being waited at.
The hold escalation is the site that needed it: it re-queried live, and a
refresh between dispatch and barrier expiry made it escalate to a point
the peer had no reason to be at. One snapshot per manoeuvre, taken by
dispatchReconnect, cleared in transitionTo when reconnect_active_ falls.
```

### return-dest-label-log-only

**return_dest_label_ is only a log string** — attached to `std::string  return_dest_label_;`

```text
Human label of the current RETURN_NAV destination, set by startReturnTo for
doReturnNav's logs and read NOWHERE ELSE — it is a log string, not state.
The values it actually takes are "appointment" (every dispatch and
supersede path) and the hold escalation's own label, which is
"appointment" or "last-connected anchor". "meeting point" was the third
value until the midpoint drives were deleted on 2026-09-16; nothing writes
it now.
```

### return-dest-vs-current-goal

**Why return_dest_ is separate from current_goal_** — attached to `Eigen::Vector3f return_dest_ = Eigen::Vector3f::Zero();`

```text
WHERE THE CURRENT RETURN_NAV LEG IS ACTUALLY GOING — the agreed appointment
cell, or the last-connected anchor. Set by startReturnTo beside the label
above, and the two always describe the same place.

It exists because current_goal_ answers a DIFFERENT question: "where are
the wheels pointed right now". Those two agree for a plain drive, and they
stop agreeing the moment anything borrows the goal to steer around a
problem — which the escape ladder does. Measuring "have I arrived?" against
current_goal_ therefore meant a robot that aimed 2.5 m behind itself to
break a stall reported reaching the MEETING, from wherever it happened to
be standing, because 2.5 m is inside reconnect_arrive_tol_m_ (4.0). It then
joined a headcount that has no time limit, and waited forever for partners
40 m away. Ordering the tests so the escape was noticed first would hide
that, not remove it: the next borrow of current_goal_ — for a detour, a
yield, anything — would reintroduce it silently. Holding the destination
separately is what makes the false arrival impossible rather than merely
not-currently-reachable, so every distance test in doReturnNav measures
against this and none against current_goal_.
```

### reconnect-manoeuvre-clock

**The reconnect-manoeuvre clock** — attached to `bool         reconnect_active_ = false;`

```text
Reconnect-manoeuvre clock, for the CSV's reconnect_elapsed_sec. Deliberately
NOT state_enter_time_: the manoeuvre spans state changes that must not
restart it — RETURN_NAV -> RETURN_SYNC on arrival, a PROXIMITY_HOLD taken
mid-drive, and in HYBRID the PURSUE -> appointment handoff (the chase
breaking off because the departure deadline came due; there has been no
PURSUE -> meeting-point handoff since 2026-09-16). Armed
once by whichever of startReturnTo/startPursuit fires first (hence the
already-active guard in both) and cleared in transitionTo on leaving the
manoeuvre states, so the column measures one thing: wall seconds this robot
spent trying to re-establish contact rather than exploring.
```

### proximity-hold-bookkeeping

**Proximity-hold bookkeeping and the nav budget** — attached to `State  prox_resume_state_    = State::NAVIGATE;`

```text
Proximity-hold bookkeeping: the driving state to resume into (NAVIGATE or
RETURN_NAV — current_goal_ is left untouched across the hold), cumulative
hold count / held seconds (CSV columns, so post-hoc analysis can correlate
holds with the trajectory), and the drive seconds already consumed on the
current goal when the hold began — the resume backdates state_enter_time_
by it so the nav budget CONTINUES instead of restarting, keeping one
goal's total drive time bounded across repeated holds.
```

### plan-rejection-profile-metrics

**The per-attempt rejection profile columns** — attached to `int   pending_plan_cand_total_    = -1;`

```text
Full rejection profile of the most recent doPlan attempt. Unlike the two
above these are drained by fillCommonMetrics, so they reach EVERY row
including the timer rows a starved planner emits — which are the only rows
it emits, since a tick that selects nothing never completes a step. See
metrics_logger.hpp. -1 = no planning attempt yet, never a measured zero.
```

### separation-manipulation-check-columns

**Separation-term manipulation-check columns** — attached to `int   pending_sep_eligible_peers_   = 0;`

```text
Separation-term diagnostics (separation.hpp), filled by doPlan on every
exploration tick and drained the same way.

These exist because the term's predecessor is a cautionary tale: MinPos is
a real mechanism whose only observable, `rejected_by_minpos`, sits at zero
for a whole campaign, so nothing in the data can tell "the veto never
mattered" from "the veto never ran". Every one of the four columns below
is a MANIPULATION CHECK, and between them they separate the two:

  sep_eligible_peers  did the term have anything to act ON this tick?
  sep_peer_dist_m     how far from the teammate did the pick land?
  sep_discount        by how much was the pick actually discounted?
  sep_reordered       did the discount change where the robot was sent?
                      1 yes, 0 no, -1 not asked on this tick

sep_peer_dist_m and sep_eligible_peers are computed and logged even when
the term is OFF — they are then a free measurement of the separation the
untreated planner produces, which is the counterfactual any treated
campaign has to be read against.
```

### map-size-beacon-rate-window

**Map-size beacon and its rate window** — attached to `static constexpr double kRateWindowSec = 300.0;`

```text
Map-size beacon state (info-gated mid-run trigger). History of
(t_sim_sec, total_observed_voxels) samples fed by fillCommonMetrics —
i.e. at the CSV sampling period, the same series every offline analysis
reads — from which the growth-rate slope is cached. stampMapInfo()
copies count+rate onto every outgoing intent; the on_intent lambda
snapshots both sides into LastContact. A vector trimmed in place, not a
deque: it holds a few dozen samples and is touched at the metrics
period, so contiguity beats pop_front.

The window is 300 s, NOT the contact duration: connected windows have a
median of 24 s and 72% are under 90 s (measured, p14), so a window sized
to a contact contains nothing but that contact's merge inflow. Reaching
back across the PRECEDING OUTAGE is what supplies samples of the robot's
own unaided gathering. Validated against p14's measured pair divergence
(2,134 vox/s): the pair's summed rate under this estimator reads 1.12x
truth, against 1.40x for a 90 s two-point slope, at better coverage.
```

### publish-intent-stamps-beacon

**publishIntent stamps the map-size beacon** — attached to `void   publishIntent();`

```text
The ONLY way this node puts an intent on the wire. Stamps the map-size
beacon onto the cached message first, so a publish can never carry a
stale — or, at the sites that rebuild current_intent_msg_ via
buildIntent(), a ZEROED — count and rate. A peer that snapshots a zeroed
beacon as its last contact reads the pair as gathering nothing and defers
to the ceiling, so this is not cosmetic. Callers must have checked
intent_pub_.
```

### midrun-gate-sec-threshold

**The effective mid-run trigger threshold** — attached to `double midrunGateSec(double missing_for, double* est_unshared_out);`

```text
Effective mid-run trigger threshold (seconds of team-incomplete before
dispatch). The fixed silence clock when the info gate is off or the
snapshot is unusable; otherwise the dead-reckoned crossing time of
reconnect_min_share_voxels, clamped to [min,max] silence. Also computes
the live unshared-backlog estimate for the dispatch event's diagnostics.
```

### dispatch-gate-diagnostics-sentinels

**Dispatch gate diagnostic sentinels** — attached to `double dispatch_gate_sec_     = -1.0;`

```text
Gate diagnostics stashed at the trigger decision, consumed by the next
reconnect_dispatch event (same discipline as dispatch_peer_id_): -1 =
not applicable (terminal dispatch, or gate off); -2 = gate ON but no
usable contact snapshot, so the trigger fell back to the time-only
clock (otherwise that fallback logs byte-identically to a control
dispatch, which also has gate_sec = the fixed silence clock).
```

### dispatch-link-down-diagnostic

**Link-down time as a diagnostic column** — attached to `double dispatch_link_down_sec_ = -1.0;`

```text
How long the RADIO had been down when the mid-run trigger fired, stashed on
the same discipline. -1 = the link gate was not in play (feature off, no
usable robot index, stale samples, or a terminal dispatch). It is a
DIAGNOSTIC, never the trigger clock: the fire is decided on
peer_record_age_sec in every case, and the gate only vetoes. On a gated
dispatch it therefore always reads >= reconnect_confirm_sec.

Logged alongside rather than instead of the record age: the whole point of
§30.11 is that the two clocks disagree, so collapsing them into one column
would destroy the measurement that motivated the change. It is also the
positive control — a gated run whose two columns agree everywhere is a run
in which the gate did nothing.
```

### viz-cell-world-marker-topic

**Why cell-world markers get their own topic** — attached to `rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr`

```text
Cell-world markers get their OWN topic rather than a namespace on
viz_pub_: publishCandidateViz() clears with a DELETEALL, which in RViz
wipes the whole display, so sharing the topic would make the two layers
delete each other on alternating ticks. Created only when the cell world
is enabled AND markers are asked for — a publisher that exists changes the
ROS graph, and default-off means default-invisible.
```

### proximity-guard-cancel-client

**Proximity guard inputs and the inert cancel** — attached to `std::vector<rclcpp::Subscription<`

```text
Proximity guard inputs + actuation. The pose subs are the peers'
localiser outputs (map frame, ~10 Hz) — much fresher than the 1 Hz intent
heartbeat that also feeds the guard. The action client was written to cancel
the in-flight NavigateToPose on hold entry — ceasing to publish goal_pose
does NOT stop the robot, since the navigator latches the last goal it
accepted and drives it to completion — but on this stack it has no server to
talk to and never fires. The brake goal does the stopping. Read the block at
its construction before trusting a "cancel" anywhere.
```

### metrics-realised-sampling

**Realised CSV sampling accounting** — attached to `int    metrics_rows_written_ = 0;`

```text
REALISED sampling accounting for the periodic CSV sampler. The configured
period is not what it achieves: the sim-time timer above is gated by the
steady-clock deadline in metricsTick, so at some real-time factors whole
ticks are suppressed and at others none are — a realised period of 10 s
against a configured 5 s was measured in an earlier campaign, with every
log line still asserting 5 s. Counted here and reported in run_end so the
rate of an archived run is a recorded fact.
```

### exploration-complete-dedup

**exploration_complete de-duplication by step** — attached to `int exp_complete_step_  = -1;`

```text
exploration_complete de-duplication. finishOrRendezvous is re-entered every
tick while the reconnect confirmation gate defers, and again if a robot
re-saturates after a manoeuvre delivered a merged map. Keying on step_
emits exactly one event per exhaustion EPISODE (step_ cannot advance during
a deferral) while still recording a genuine second exhaustion later.
```

### mission-complete-exactly-once

**The mission_complete exactly-once contract** — attached to `int mission_complete_count_ = 0;`

```text
mission_complete emissions this run. The contract is exactly one, and TWO
guards at the top of startReturnHome enforce it: mission_return_done_
(run-scoped) and the state_ == RETURN_HOME test (leg-scoped). Through
generation 8 this comment named only the second, which structurally cannot
see the defect that was actually measured — the second request arrives
from DONE, not from RETURN_HOME, so the state test never fired on it.
Incremented on every emission unconditionally: a counter that a guard
prevents from ever reaching 2 would be a check that stopped checking.
```

### mission-return-once-per-run

**Why the mission-return latch exists** — attached to `bool mission_return_done_ = false;`

```text
The mission return is a ONCE-PER-RUN leg, and this is what makes that
structural instead of a property of whichever state the second request
happens to arrive from. Set the instant finishMissionReturn commits the
ending; read by startReturnHome, which refuses every later request.

Why the state test cannot carry this: finishMissionReturn ends in
finishNow, so the robot is in DONE — not RETURN_HOME — by the time the
second request arrives, and it does arrive. The coverage latch fires from
the metrics tick, which is the deciding hook for done_criterion=latch and
measures the ROI in EVERY state, so a robot that ended on barrier-gave-up
can saturate its map minutes later while parked at home and ask to go
home again. Measured on ts1b: 16 DONE->RETURN_HOME re-entries and 16
robot-runs carrying two mission_complete rows, all of them in the
treatment arm, the count rising with team size.
```

### mission-return-reentry-counter

**Counting refused mission-return re-entries** — attached to `int mission_return_reentries_ = 0;`

```text
Requests the latch above refused. A guard that leaves no trace is a guard
no run can be shown to have needed, and "zero re-entries" and "the guard
is inert" are not the same claim — so this rides out on run_end rather
than living and dying inside the process. The first refusal also WARNs
(once only: the caller can be a tick loop).
```

## ExploPlannerNode::ExploPlannerNode (constructor)

### param-dp-f-float-narrowing

**dp_f and float parameter narrowing** — attached to `auto dp_f = [&](auto n, double d) {`

```text
Sibling of dp for the float-typed members and configs. ROS 2 has no float
parameter type, so every one of these is declared as a double and narrowed
here. The default is typed `double` rather than deduced on purpose: a call
site that writes a float literal then declares a parameter rclcpp cannot
represent, and this converts it instead of failing to compile deep inside
declare_parameter.
```

### param-fleet-identity

**The fleet identity parameter** — attached to `team_robot_names_ =`

```text
Fleet identity (docs/mtare_evolution_plan.md §3.0). The ordered list of
every robot in this fleet, IDENTICAL on every robot: a robot's numeric id
is its position in the array, and that id indexes every knowledge mask,
gossip slot and cross-robot tie-break the shared world model uses.

Defaults to empty, which means "unconfigured" and is not an error: that is
every launch predating this, and the per-phase equivalence gate requires
them to behave identically. A MALFORMED array is a different matter — it
is a config error, and the node refuses to start rather than run on a
fleet definition that would make its own masks lie.
```

### param-experiment-log-path

**Deriving the experiment event log path** — attached to `experiment_log_path_    = dp("experiment_log_path", std::string(""));`

```text
Experiment event log — one newline-delimited JSON file per robot per run.
See experiment_log.hpp for what it exists to fix; in short, the CSV plus
the ROS log could not answer "when did this robot reach coverage X" without
reconstructing sim time from wall-clock log stamps against a drifting RTF.

The path DEFAULTS TO EMPTY, meaning "derive from output_csv": <csv without
its .csv suffix>.events.jsonl. That is deliberate and is the one place this
pair diverges from output_csv's flat default. Every harness already passes
a per-run, per-robot output_csv (run_explo_sim_rviz.sh: -p
output_csv:=$OUTDIR/planner_$r.csv), so deriving puts the event log beside
its own CSV automatically — whereas a fixed default like
/tmp/exploration_events.jsonl would have every robot of every arm truncate
the same file, which is exactly the silent data loss this class is about.
An explicit value is always used verbatim.
```

### param-coverage-milestone-ladder

**Sizing the coverage milestone ladder** — attached to `coverage_milestones_ = dp("coverage_milestones",`

```text
Descending unknown-fraction ladder for the coverage_milestone events, which
are how time-to-coverage is made comparable ACROSS ARMS: each arm stops on
its own criterion, so run duration measures a different thing per arm,
while "sim time this robot first drove unknown fraction below X" means the
same thing everywhere.

The default is sized from the campaign data rather than guessed. Across 98
planner CSVs under /tmp/hmr_campaign the first measured unknown fraction is
0.930-1.000 and the final one is 0.497-0.554 in 97 of them (the exception
is a 21-row run aborted at 0.817). So: 0.95 is degenerate — for the robots
whose first sample is already 0.93 it would fire at t0 by construction and
measure the initial map, not exploration — and nothing below 0.50 is ever
reached. 0.90 down to 0.50 is the informative band (in the p8trigger runs
0.90 lands at ~35-43 s and 0.55 at ~820-1490 s, a spread that separates the
arms), and 0.45 is carried as a guard rung so a future arm that covers more
ground still records the crossing instead of silently having no data point.
Rungs that never fire simply never appear in the file.
Ladder rationale (and why the tail is NOT 0.55/0.50/0.45) in
config/shared_params.yaml — measured, those bottom rungs were dead columns
and the deepest one that fired was done_unknown_fraction itself.

C1 (2026-09-14): the 0.62/0.60/0.58/0.56 tail is gone as well. With
done_unknown_fraction at 0.64 -- between the 0.65 and 0.62 rungs -- those
four could only be crossed after the robot had declared itself done, and
measurement confirmed it (>=95% of crossings post-declaration in both
arms, and the reach RATE itself tracked the treatment). The invariant this
ladder now maintains: EVERY RUNG SITS STRICTLY ABOVE done_unknown_fraction.
Move one and move the other.
```

### param-clock-anchor-cadence

**The clock_anchor cadence** — attached to `experiment_log_anchor_period_sec_ =`

```text
clock_anchor cadence in SIM seconds. Each anchor is a (sim, wall) pair plus
the real-time factor since the previous one, which turns this file into the
conversion table for every other log in the run directory — they carry wall
stamps only, and the current practice of fitting one line through the whole
run is wrong by 10-54 s because the RTF drifts within a run (0.89 -> 0.81
measured). 10 s bounds the interpolation error to well under a second at
any plausible drift rate and costs one short line per anchor. <= 0
disables anchors (every other event still carries its own pair).
```

### param-metrics-period

**Why the CSV is sampled periodically** — attached to `metrics_period_sec_ = dp("metrics_period_sec", 5.0);`

```text
Wall-clock CSV sampling period. The end-of-step row is the only row a
pre-experiment run produced, and steps do not advance during a reconnect
manoeuvre — so a run that spent four minutes chasing a peer recorded that
interval as a single flat segment between two step rows, which is precisely
the interval the comms experiment is measuring. 0 restores the old
step-rows-only behaviour.
```

### param-arrival-gate-vs-navigator

**Arrival gate versus the navigator's tolerance** — attached to `if (goal_yaw_tol_ < 0.3) {`

```text
The arrival gate must be strictly LOOSER than the navigator's own stop
condition, or the navigator declares success and stops just outside the
planner's tolerance, the planner never sees arrival, and failGoal()
blacklists a goal the robot is standing on.

The navigator is simple_nav_3d, not nav2 — this comment used to justify the
0.3 threshold with nav2's general_goal_checker defaults (xy 0.25 /
yaw 0.25), a checker that has never run in this system. simple_nav_3d's UGV
stop condition is ugv.goal_xy_tol_m / ugv.goal_yaw_tol_rad, both 0.2 as
shipped, but the park distribution is set by four stacked roundings rather
than by either tolerance and the measured median arrival over ts1b's 240
cells landed 0.269 m out. 0.3 is therefore the point below which the gate
stops clearing the measured median, which is what these two warnings are
actually about. A node cannot read another node's parameters, so the
tolerance figures quoted below are documentation and have to be checked by
hand against simple_nav_3d/launch/simple_nav_3d.launch.py; the 0.269 m is
the load-bearing number and it is independent of them.
```

### param-candidate-min-goal-dist

**Minimum useful hop and frontier oscillation** — attached to `cand_min_goal_dist_ = dp("candidate_min_goal_dist_m", 0.0);`

```text
Minimum useful hop. Candidates nearer than this are rejected in doPlan
alongside the ones inside goal_xy_tolerance.

Without it the planner deadlocks into a two-point oscillation, and the
mechanism is structural rather than a tuning accident. Utility is the
SSMI-style info_gain / (eps + path_cost). In a mostly-unknown map every
candidate's raycast terminates in unknown space, so info_gain is nearly
constant across the whole candidate set — measured spread was 6% while
path_cost varied ninefold — and argmax(U) degenerates to argmin(cost),
i.e. "drive to the closest frontier". The closest frontier is typically
under a metre away, and here is why that never resolves: a VLP-16 has a
+-15 deg vertical field of view, so at 0.6 m of standoff it sees a band
barely 0.3 m tall. Voxels beside the robot at any other height are
physically unobservable from that range. Driving there reveals nothing,
the frontier survives, and the pair of candidates either side of the
robot regenerate every step forever. Neither guard already in the loop
catches it: goal_xy_tolerance only skips candidates at arm's length, and
the failed-goal blacklist never fires because the robot REACHES each goal.

So this is not a heuristic to break ties; it encodes that a goal closer
than the sensor's useful standoff cannot reduce uncertainty where it
stands. Scale it to the sensor, not the robot: a few metres for a VLP-16.
Note it is the VERTICAL fan that sets this, not fov_max_range — the +-15 deg
band is what makes a 0.6 m standoff blind, and that is unchanged by the
horizontal correction or by the range knob.

Default 0.0 keeps the shipped behaviour bit-identical — every field
config that predates this parameter selects exactly the goals it did
before.
```

### param-nav-distance-budget

**Distance-budgeted navigate timeout** — attached to `nav_speed_est_mps_  = dp("nav_speed_estimate_mps", 0.5);`

```text
Distance-budgeted navigate timeout. The total budget for a NAVIGATE
cycle is computed at entry from straight-line distance to the goal:
  budget = clamp(dist / speed_est * safety, nav_min, nav_max)
so a 2 m hop gets a small budget and an 8 m hop gets a larger one,
instead of every goal sharing the same fixed timeout.
```

### param-no-progress-watchdog

**The no-progress watchdog** — attached to `progress_window_sec_   = dp("progress_window_sec", 6.0);`

```text
No-progress watchdog. Independent of the total budget: if the robot
hasn't accumulated at least progress_min_distance_m of travel within
progress_window_sec, declare the goal failed early. Catches the
wedged-robot case (same pose forever) in seconds instead of waiting
out the full budget. Mirrors the navigator's progress check.
```

### param-failed-goal-blacklist-ttl

**Failed-goal blacklist and its TTL** — attached to `failed_goal_radius_m_ =`

```text
Failed-goal blacklist. When a navigate cycle times out before reaching
the goal, the goal position is parked here for `failed_goal_ttl_sec`
seconds; subsequent picks reject any candidate within
`failed_goal_radius_m` of a non-expired entry. Prevents the planner
from re-picking the same unreachable goal forever when the robot is
physically stuck (same pose -> same scores -> same pick loop).

The TTL must outlast one full worst-case attempt SOMEWHERE ELSE plus the
travel and replanning around it. It did not: the 60 s default was sized to
an old 60 s budget cap, and when nav_max_timeout_sec went to 180 s in the
campaign yaml the TTL was not raised with it. The consequence, measured in
mr1_hybrid_seed18: each of two terrain traps expired from the blacklist
while the robot was busy burning a 180 s budget at the other one. Observed
fail -> re-pick gaps for the same trap were 210, 203, 224 and 225 s — all
longer than 60 and all shorter than 240. Eight full budgets, 1441 s, two
sites, and the cell ended 0.021 of unknown-fraction short of done.
```

### param-failed-goal-retirement

**Retiring permanently failed goal sites** — attached to `failed_goal_retire_after_ = dp("failed_goal_retire_after", 3);`

```text
Retirement: after this many failures at one site (clustered at
failed_goal_radius_m), the site is suppressed for the rest of the run
rather than expiring. 0 disables it and restores pure TTL behaviour.

TTL alone cannot handle a PERMANENT trap, only a transient one — any
finite TTL eventually lets a fixed piece of bad terrain back into the
argmax, and the site's information shadow guarantees it wins again,
because the ground behind the trap stays unobserved precisely because the
robot never gets there. Retirement bounds the waste at
retire_after x nav_max_timeout_sec per site per run.

Retiring is not the same as giving up: arriving inside the radius clears
the record (clearNear below), and the amnesty path in the selection loop
re-offers retired sites when they are the only candidates left.

k=3 from the mr1 evidence: across 72 robot-logs and 49 distinct failure
sites, reattempts at [budget] sites succeeded 0/7, and exactly 1 of 49
sites was later reached within the 2 m radius — a no-progress site that
recovered on its FIRST reattempt, which k=3 would not have blocked.
```

### param-visited-goal-suppression

**Visited-goal suppression against oscillation** — attached to `visited_goal_radius_m_ = dp("visited_goal_radius_m", 0.0);`

```text
Visited-goal suppression. A goal the robot REACHED is parked for
visited_goal_ttl_sec, and EXPLORE candidates within visited_goal_radius_m
of a live entry are skipped. 0 (default) = off, shipped behaviour.

Needed because frontier exploration has no fixed point in a forest. A
frontier is a free voxel beside an unknown one, and every trunk casts a
permanently unknown shadow, so frontier clusters regenerate no matter how
thoroughly an area is observed. Utility is info/(eps+cost) and info is
near-constant while the map is mostly unknown, so selection collapses to
argmin(cost) -- and two neighbouring clusters then trade places as
"nearest" forever. Measured on flatforest: after narrowing the frontier
band the robot advanced in bursts but still alternated between two goals
4.7 m apart for six consecutive steps with no net movement.

The failed-goal blacklist cannot cover this: it only fires when a goal is
NOT reached, and here every goal is reached, on time, successfully.

Size the radius near the frontier cluster radius, so suppressing a visited
goal suppresses the cluster that produced it rather than a point inside
it. The TTL must outlast a there-and-back trip or the entry expires before
the oscillation it prevents can recur; it is a TTL rather than permanent so
a genuinely re-frontiered area can be revisited late in a run.
```

### param-done-unknown-fraction

**Streak termination and its generic default** — attached to `done_unknown_fraction_ =`

```text
Coverage-based termination. Each PLAN tick we measure the unknown
fraction of the ROI (source per done_coverage_source below). When it
stays below `done_unknown_fraction` for `done_min_consecutive_steps`
planning cycles in a row we declare the map saturated and transition to
DONE. EIG scores don't fall sharply as the map saturates (the FOV raycast
always finds *some* unobserved voxels at the cone edge), so unknown
fraction is the reliable signal here. Set done_unknown_fraction <= 0 to
disable.
FIELD NOTE: this library default is deliberately NOT the campaign value.
0.64 is calibrated to flatforest, whose unknown fraction floors near 0.486;
it is a world property, not a planner property, and baking it in here would
silently mis-stop a different world. config/shared_params.yaml carries the
calibrated value (C4), and any new world must recalibrate. 0.05 is the
generic "map actually saturates" assumption, which flatforest violates.
```

### milestones-above-done-threshold

**Milestones must sit above the stop threshold** — attached to `if (done_unknown_fraction_ > 0.0) {`

```text
C1 invariant: every coverage milestone must sit STRICTLY ABOVE the stopping
threshold. A rung at or below it can only be crossed after the robot has
declared itself done, which makes it a measure of post-stop map merging;
worse, whether a run reaches such a rung at all correlates with the arm, so
it is a selection artifact masquerading as an arm-independent endpoint.
This is checked rather than commented because the last time it broke,
nothing said so -- the threshold moved to 0.64 and four rungs quietly
changed meaning. Report-only: the rungs still fire, and the events carry
post_latch, so the data stays interpretable either way.
```

### param-done-criterion-latch

**Why latch is the default done criterion** — attached to `done_criterion_ = dp("done_criterion", std::string("latch"));`

```text
WHICH RULE DECIDES FINISHED. The default is "latch" and the paragraph above
describes "streak", which is now opt-in. The change is deliberate and the
reason is measurable: the streak test runs only at the top of doPlan, so a
robot inside a reconnect manoeuvre cannot declare itself finished however
saturated its map is. Measured on the tr1 campaign, that blind spot plus
re-earning the streak from zero after the manoeuvre put ~60 s of pure
detection latency on the hybrid arm and 0 s on the off arm — an endpoint
that moves with the treatment is not an endpoint, it is part of the
treatment. "latch" measures the same quantity in both arms:
  * the robot's OWN fused-map ROI unknown fraction (nothing team-wide),
  * tested on every metrics tick in EVERY state, manoeuvres included,
  * on first touch — no confirmation streak,
  * with no rendezvous gate: being in comms is not required to be finished.
The run then ends when BOTH robots have latched independently, which the
harness already implements by waiting for every planner's state to read
DONE. Set "streak" to reproduce a pre-2026-08-24 campaign.
```

### param-done-coverage-source

**Coverage measurement sources** — attached to `done_coverage_source_ = dp("done_coverage_source", std::string("auto"));`

```text
Measurement source: "planning_map" (legacy 2D; INACTIVE when none is
published), "scovox" (2.5D column coverage of the fused 3D map — works
without a planning_map), or "auto" (planning_map if present, else
scovox). NB the two sources measure different things — the 2D grid
counts nav-grid cells, the column measure counts ROI-footprint columns
with >= 1 observed voxel in the z band — so re-calibrate
done_unknown_fraction when switching.
```

### param-vertical-roi-band

**The vertical ROI band** — attached to `roi_min_z_ = dp_f("roi_min_z", -0.5);`

```text
Vertical ROI band. In dscovox mode this is the z-slab the fused map is
clipped to on ingest, and it bounds the FOV raycast, frontier
extraction, and map-stats volume. Keep it to the robot-relevant band so
the spurious vertical LiDAR smear above the robot isn't scored as
explorable space; FOV rays leaving the band are clipped (treated as
empty — no info gain, no occlusion).
```

### terrain-band-floor-check

**Startup check of the band-floor invariant** — attached to `if (terrain_relative_z_) {`

```text
The band-floor invariant the yaml documents: ground search must stay inside
the ingested slab from anywhere the robot can sit before loadLatestMap()
re-bands. Violated, groundZAt returns NaN on a downslope and every
terrain-mode consumer degrades — silently, since NaN reads as "no ground
here" and not as "the band is misconfigured". Check it at startup rather
than leaving it to a comment.
```

### param-frontier-z-band

**Narrowing the frontier search band** — attached to `const double f_lo_off = dp("frontier_z_lo_offset_m", 0.0);`

```text
Narrow the FRONTIER search band relative to the ROI band, from the bottom
and from the top. Both 0 = search the whole ROI band (shipped behaviour).

The ROI band is sized for terrain and canopy — the yaml ships a 9.5 m slab,
-5.5 to +4.0 — and searching a slab that tall for frontiers produces a
candidate set dominated by voxels no robot can ever observe. A frontier is
a free voxel with an unknown neighbour, and a lidar's free space is a wedge
bounded by its vertical FOV, so the ENTIRE upper and lower surface of that
wedge qualifies, at every range, forever: a VLP-16 at +-15 deg simply has
no ray that reaches 3 m up at 4 m out. Those frontiers cannot be consumed
by driving to them, which is what makes them poison rather than noise —
they regenerate beside the robot every tick, they are always the nearest
ones, and (utility being info/cost with near-constant info) they are
therefore always chosen. Measured on flatforest: frontier_voxels stayed at
~94% of all observed voxels and GREW monotonically as the map grew, while
both robots ping-ponged between two adjacent goals indefinitely.

Set these so the band covers only the heights the sensor sweeps as it
drives — roughly the navigable slice. Then a frontier means "ground I have
not been past", which driving there does clear, and the candidate set
drains as the ROI is covered, which is also what makes coverage
termination reachable at all.

Only ever NARROWS: the result is intersected with the ROI band, because a
frontier outside the ingested band would reference voxels map_cache_ never
loaded.
```

### fov-default-vlp16-model

**FOV defaults match the VLP-16** — attached to `FovConfig fcfg;`

```text
FOV evaluation
Defaults are the VLP-16 the robots actually carry, matching the SDF
(-3.14159..+3.14159 by -0.261799..+0.261799), NOT the 60x45 deg RGB-D
frustum they used to describe. These are the values a launch path that
forgets to load shared_params.yaml gets, so a stale default here is a
silent second sensor model -- which is what it was.
```

### fov-omnidirectional-tolerance

**Detecting an omnidirectional FOV** — attached to `{`

```text
Full circle within one ray step. Not an exact == on 2*pi: the yaml carries
a rounded 6.28318 and the SDF a rounded 3.14159, so an equality test would
read the very configuration this is meant to recognise as directional.
One h_step of slack is the honest tolerance -- anything inside it is a
circle the ray comb cannot distinguish from a closed one.
```

### param-utility-cost-exponent

**Clamping the utility cost exponent** — attached to `utility_cost_exponent_ = dp("utility_cost_exponent", 1.0);`

```text
Distance discount for the utility denominator. 1.0 = shipped behaviour.
Clamped rather than trusted: a negative exponent inverts the denominator
into a REWARD for distance (the further the better, without bound), which
is not a weaker preference but a different and unbounded objective, and a
typo in a sweep script should not be able to express it. Above 2.0 the
planner is more distance-averse than nearest-frontier, which the candidate
filters already enforce more cheaply.
```

### param-separation-term

**Loading the separation term** — attached to `{`

```text
Soft team-separation discount on the utility (separation.hpp). Loaded
beside utility_cost_exponent because it is the same kind of thing — a
multiplier on U(c) — and a reader who finds one should find the other.

Defaults are OFF (weight 0), so this block changes no shipped behaviour
until a campaign turns it on. The radius default is the 20 m the
rho = 0.677 correlation is stated at, and the max age is two TeamWorld
heartbeats plus margin; both are inert while the weight is 0.

configure() REFUSES rather than clamps (see its doc), and the refusal is
logged at ERROR: a run whose manifest records separation_weight=0.6 and
whose binary silently ran at 0 is a cell that would be pooled into the
treated arm while carrying the control's behaviour.
```

### param-alloc-peer-pos-ttl

**Allocator peer-position TTL** — attached to `alloc_peer_pos_max_age_sec_ = dp("alloc_peer_pos_max_age_sec", 0.0);`

```text
Time-to-live on the peer POSITION the allocator solves over. Loaded here,
beside separation_max_age_sec, because it is the same kind of thing — a
bound on how long a latched pose keeps steering this robot — and the two
should be read together. See alloc_peer_pos_max_age_sec_ for what it does
and why only one of the three allocVehicles callers gets it.

Default 0 = unbounded = today's planner exactly, so this line changes no
shipped behaviour until a campaign sets it. NOT clamped and NOT refused:
unlike separation_weight there is no invalid value here — every finite
number is either a TTL or the off switch — but a NaN would make the
`age <= ttl` test false for every peer and quietly amputate the whole
fleet from the allocation, so that one case is caught and logged.
```

### param-split-intent-topics

**Resolving split intent-stream topics** — attached to `auto resolve_topic = [this](const std::string& t) {`

```text
Split intent stream (see member comments). Empty -> today's shared bus.
A non-absolute value is resolved under /<robot_name>/, matching this
file's convention for every other cross-node topic (see
refinement_region_topic below): the packaged launch files pass robot_name
as a parameter but do NOT namespace this node, so a relative topic would
otherwise land in the global scope and silently never match the
emulator's per-robot relays — which reads as "peer missing forever",
indistinguishable from the outage the run is trying to measure.
```

### param-reconnect-enabled-alias

**reconnect_enabled and its legacy alias** — attached to `{`

```text
Master on/off for the WHOLE reconnect subsystem. ON by default. It is NOT
the appointment factor (that is rendezvous_schedule_enable) and NOT a
selector for the legacy return-to-anchor barrier (that is reconnect_mode):
false means no mid-run manoeuvre of any kind runs, which is the control
arm. The arm string below is literally `reconnect_enabled_ ? mode : "off"`.

Renamed from `rendezvous_enabled` on 2026-09-03. The old name predates
pursuit and hybrid existing, and by cr3-cr5 it read off the manifest as
"the rendezvous arm is on" in three arms out of four — including pure
pursuit, which cannot arm an appointment at all (armAppointment refuses on
mode). It cost a reader a double-take on a finished campaign; that is the
whole reason for the rename.

The old name still works and WINS when explicitly set. Rationale: after the
rename NOTHING in this tree passes the old name, so an override under it can
only have come from a caller outside the tree — a stale script — and the
safe reading of a stale script is that it means what it says. The new name,
by contrast, is passed by the harness on every single run
(run_explo_sim_rviz.sh: `-p reconnect_enabled:=$RECONNECT_ENABLED`), so
"the new name is present" carries no intent at all.

The sharp edge, stated rather than hidden: get_parameter_overrides() merges
the params file and the command line into one map and this rule cannot tell
them apart, so a params file carrying the LEGACY key would beat an explicit
`-p reconnect_enabled:=false` — the reverse of ROS's own precedence. No
in-tree yaml carries the legacy key (shared_params.yaml was renamed with
everything else), so that inversion is unreachable from here; it is a trap
only for someone who reintroduces the old spelling into a params file. The
WARN below fires on every disagreement, which is the mitigation.

Both names are stamped into the experiment log with the RESOLVED value, so
no analysis can be misled about which one was passed.
```

### reconnect-preconditions-warn

**Reconnect subsystem preconditions** — attached to `rendezvous_expected_peers_ = dp("rendezvous_expected_peers", 0);`

```text
Reconnect-subsystem preconditions. The subsystem only *activates* where it
is meaningful — coordination on (every manoeuvre reads the peer claim
table) and a positive expected-peer count.

R4: this is a WARN, not an INFO, and the message no longer says
"rendezvous". What this branch turns off is the WHOLE reconnect subsystem:
rendezvous, pursuit and hybrid alike. The old INFO wording cost a real
diagnosis — a cell launched with reconnect_mode=hybrid whose manifest
faithfully recorded `reconnect_mode_param=hybrid` while the binary had
silently disabled every manoeuvre here, which reads downstream as a
treated cell that produced no treatment. An arm being silently voided is
not an INFO-level event. The only run where this is expected is a
single-robot or explicitly uncoordinated cell, and there the warning is
one line at start-up.
```

### reconnect-needs-done-idle

**Reconnect runs need done_action idle** — attached to `if (reconnect_enabled_ && done_action_ != "idle") {`

```text
The barrier counts peers by their claim beacons, and only a DONE-idle
robot keeps beaconing after it finishes (finishOrRendezvous publishes the
presence intent, the heartbeat refreshes it). A done_action=shutdown
robot exits the process instead: the first finisher becomes permanently
invisible and a teammate finishing later runs the whole reconnect
manoeuvre against a robot that no longer exists.
```

### param-reconnect-mode

**Mesh reconnection mode selection** — attached to `{`

```text
Mesh reconnection mode. Default "rendezvous" = the legacy return-to-anchor
barrier, bit-for-bit; "pursuit"/"hybrid" are the robot-carried-radio
manoeuvres (see the param comments above). Gated by the same
reconnect_enabled_ preconditions — the mode only picks WHICH manoeuvre
runs once shouldRendezvous() says one should.
```

### reconnect-mode-unknown-fatal

**Unknown reconnect_mode is fatal** — attached to `if (!mode_known) {`

```text
FATAL, not a warning (2026-09-16). reconnectModeFromString keeps its
documented fallback — it is a pure function with unit tests pinning
"nonsense" -> RENDEZVOUS — but the NODE refuses to run on it. A typo in
the arm name is not a mode choice, and the fallback silently retargets
the run at a DIFFERENT ARM of the same experiment while every manifest
field, every event-log param and every directory name still says the
name that was asked for. There is no way to detect that downstream: the
cell looks like a perfectly healthy member of the arm it was never in.
One line at start-up costs a cell; a silent re-arming costs a campaign.
```

### reconnect-mode-inert-warning

**Warning when reconnect_mode is inert** — attached to `if (!reconnect_enabled_ && reconnect_mode_ != ReconnectMode::RENDEZVOUS) {`

```text
R4, second half. reconnect_mode_ is still parsed and still stamped into
the log when the subsystem above turned itself off, so a run can carry a
mode it will never execute. Say so at the point the mode is read, naming
the mode, so the console log alone distinguishes "hybrid ran" from
"hybrid was asked for and voided".
```

### midrun-max-wait-zero-guard

**Zero mid-run max wait never expires** — attached to `RCLCPP_WARN(get_logger(),`

```text
ZERO IS NOT "DO NOT WAIT" HERE, which is the whole reason this needs a
guard the others made obvious. rendezvousWaitExpired tests
`max_wait_sec > 0.0`, so a zero, a negative or a NaN all mean NEVER
EXPIRES — and this is the one cap in the family whose expiry is the
robot's only way back to exploring. Unbounded turns a mid-run attempt
into the terminal barrier it is documented never to be: the robot stands
at a failed chase's intercept for the rest of the cell with an
unsaturated map, and the run is scored as though it had explored.
```

### max-lateness-nan-guard

**Guarding the rendezvous lateness budget** — attached to `RCLCPP_WARN(get_logger(),`

```text
NaN is the dangerous one, and not for the usual reason: the rung floor
subtracts this from the robot's drive, so a NaN propagates into the
not_before argument and nextAgreedOccurrence's llround of a NaN is
undefined. A negative budget is merely wrong — it demands the robot
arrive EARLY by that much — but it would silently roll rungs on robots
that could make the nearest one, forking a team that had no reason to.
```

### latched-hold-zero-rejected

**Zero latched hold is rejected** — attached to `RCLCPP_WARN(get_logger(),`

```text
ZERO IS REJECTED, NOT ACCEPTED AS "NO CAP" (2026-09-21). Every guard on
this value reads `> 0.0`, so zero disables the hold rather than
unbounding it — and since the coverage latch started handing finished
robots to the barrier (keepAppointmentOnFinish) this cap is the ONLY
thing that ends a latched keeper's wait: it is non-terminal by
construction, so rendezvous_max_wait_sec cannot end it and
rendezvous_appointment_wait_sec is itself 0 = forever. Zero here would
therefore reproduce the exact censored cell this generation was built to
remove, silently, from a parameter that looks like a disable switch.
```

### rendezvous-nonfinite-param-guard

**Non-finite rendezvous parameters are refused** — attached to `if (!std::isfinite(rendezvous_settle_sec_)) {`

```text
rendezvous_depart_delay_sec is NOT validated: it has been inert since
generation 25 (the arming floor carries no notice term — see the
declaration) and a guard on a value that gates nothing would print
reassurance about behaviour that cannot happen. It is logged exactly as
the yaml passed it.

NON-FINITE IS NOT "OFF", it is silent. The rest of the family is compared
with `>`/`<`, and every comparison against NaN is false — so a NaN settle
skips the hold and a NaN cap reads as unbounded. Each of those is a
DIFFERENT arm from the one the campaign label claims, with nothing in the
log saying so. Refuse to the documented default instead.
```

### drain-release-thresholds-refused

**Drain thresholds are refused, not defaulted** — attached to `if (rendezvous_drain_release_ &&`

```text
REFUSED, NOT DEFAULTED. R and W decide when a robot stops waiting for its
partner's map; picking either here would be this file choosing a number
that only the measured per-peer arrival distribution can choose. A
substituted default would run, log a plausible arm name, and be a
different experiment from the one the label claims — which is the exact
failure mode the non-finite guard above exists for, one level up.

R > 0, NOT R >= 0, and the strictness is load-bearing rather than tidy.
The drain test is `rate >= R -> not drained` and `rate` is clamped at zero
by construction, so R = 0 makes the test true for every peer on every
window: the release can never fire, every meeting ends at the cap, and the
arm logs UNFINISHED EXCHANGE for exchanges that finished. That is the
treatment silently not running under its own name, which is worse than a
refusal and indistinguishable in the tables from a mechanism that does
nothing. It is also the zero-test the design rejects on evidence: 10 of 31
long gen-32 meetings were still gaining voxels at departure.
```

### drain-window-vs-latched-hold

**Drain window must be under the hold cap** — attached to `if (rendezvous_drain_release_ &&`

```text
COUPLING, and it is the one that bounds the treatment. The drain hold is
capped by rendezvous_latched_hold_sec (see the gate in doReturnSync), and a
window longer than the cap would mean the first evaluation never happens:
the hold would always end at the cap and always log an unfinished
exchange, which is the treatment silently not running.
```

### appointment-wait-vs-settle

**Appointment cap must exceed the settle** — attached to `if (rendezvous_appointment_wait_sec_ > 0.0 &&`

```text
COUPLING. A positive appointment cap shorter than the settle means the
barrier gives up before it can ever finish holding for the map exchange:
the two are both satisfiable only if the cap leaves room for the hold. Not
reachable on the default (cap 0 = unbounded); worth saying out loud for the
field escape hatch, which is the only way to reach it.
```

### appointment-wait-pins-capped

**Unbounded wait pins the capped column** — attached to `if (rendezvous_appointment_wait_sec_ <= 0.0) {`

```text
SAID OUT LOUD BECAUSE IT PINS A LOGGED COLUMN. This same wait is the
scheduler's findability cap (deriveRendezvousProposal), so an unbounded
barrier is an uncapped interval and `capped` is false on every row the run
produces. That is the truth — a barrier nobody walks away from cannot be
outrun by any interval — but a column that is constant by configuration and
a column that is constant by accident look identical in the csv, and only
one of them is worth investigating.
```

### done-seek-announce-both-ways

**DONE-SEEK announces both directions** — attached to `RCLCPP_INFO(get_logger(),`

```text
Unconditional, both directions, once per run. This is the anchor for the
liveness check: a one-sided "no coast in the control arm" assertion passes
trivially on a build where the feature is dead everywhere, which is exactly
how guards here have gone quiet before while still printing PASS. Every run
states which side it is on, so treated-with-no-line and control-with-a-line
are both detectable from the console log alone.
```

### mission-start-hold-default-on

**Why the pre-mission hold defaults on** — attached to `mission_start_hold_sec_ = dp("mission_start_hold_sec", 60.0);`

```text
The pre-mission hold (see the member). Defaulted ON, unlike the two knobs
above, and the asymmetry is the point: those change what the robot does and
must be opted into so the binary reproduces banked behaviour, whereas this
one closes a defect that split a fleet. A campaign that forgets the knob
should get the fix, not the split. Generation 22 is a new generation
precisely because of it and is not poolable with anything earlier.
```

### approach-watchdog-defaults

**Sizing the approach watchdog defaults** — attached to `return_approach_window_sec_ = dp("return_approach_window_sec", 40.0);`

```text
Approach watchdog (generation 5). 1.0 m per 40 s = 0.025 m/s net, which is
~9x below the slowest ARRIVED homing in the 71-homing population (mean
approach 0.220 m/s, median 0.388) and infinitely above seed11's -0.0001
m/s, so the separation is not marginal in either direction. The 40 s
window is long enough for a legitimate circumnavigation (worst arrived
detour ratio 1.47) and short enough that two fires still leave 5/6 of the
600 s cap. Escapes are capped so the whole ladder is bounded well inside
the cap: 40 + 40 + 3 x (30 escape + 40 window) = 290 s worst case.
```

### midrun-silence-ceiling-pin

**Pinning a degenerate silence ceiling** — attached to `if (reconnect_midrun_silence_sec_ <= 0.0 &&`

```text
Degenerate shape the derived default introduced and the old hand-copied
240 could not: with the mid-run trigger OFF (clock 0) the ceiling derives
to 0 while the floor stays 60, so min > max. Inert in practice -- the info
gate rides the mid-run path, which a 0 clock disables -- but pinned rather
than left lying around as a min > max the clamp would resolve to MAX.
```

### midrun-ceiling-drift-warning

**Ceiling drift warning is unconditional** — attached to `if (reconnect_midrun_max_silence_sec_ > reconnect_midrun_silence_sec_ &&`

```text
NOT conjoined with reconnect_min_share_voxels_ > 0. It used to be, and that
made the invariant unfireable: the info gate ships at 0 on every path, so
the docs promised "the node WARNs if the ceiling is set above the clock"
about a branch no shipped configuration could reach. The ceiling/clock
relation is worth reporting whenever someone has set it wrong, because it
is exactly the drift that left the ceiling at 240 after the clock moved to
90 -- and that happened with the info gate off.
```

### present-tol-vs-arrive-tol

**Presence radius must cover arrival** — attached to `if (!std::isfinite(rendezvous_present_tol_m_) ||`

```text
A presence radius inside the arrival tolerance inverts the pair: every
give-up short of the cell would roll, including the ones that stopped
close enough to have counted as an ARRIVAL had they stopped one tick
earlier. Clamped rather than warned-and-kept, because the inverted
ordering has no legitimate A/B reading — unlike the windows above, where 0
means "legacy behaviour" — and a robot that rolls off a meeting it was
standing at is the failure this radius was added to prevent, wearing the
opposite sign.
```

### release-confirm-vs-claim-ttl

**Release window must exceed the claim TTL** — attached to `if (reconnect_release_confirm_sec_ > 0.0 &&`

```text
A release window at or below the claim TTL is not a flicker guard: one
packet keeps the peer live for the whole TTL, so any window inside it is
satisfied without the claim ever being refreshed. Warn rather than clamp —
0 is a legitimate "legacy behaviour" setting for an A/B, and silently
moving a configured value would make the manifest a lie.
```

### midrun-silence-without-link-veto

**Low silence clock without a link veto** — attached to `if (reconnect_midrun_silence_sec_ > 0.0 && reconnect_enabled_ &&`

```text
Below the ~180 s heartbeat-suppression tail the RECORD-AGE clock alone
cannot tell a silent teammate from an absent one. The link veto can, so
this warns only when the veto is NOT configured — with the gate wired the
low threshold is the intended generation-9 setting and warning on every
run would be pure noise, which is how a guard stops being read at all
(§32.14's inert-check family). Conversely a low threshold with no gate is
now the genuinely dangerous combination, and it says so.

Three conjuncts, each earning its place. reconnect_enabled: with the
manoeuvre off the trigger cannot fire whatever the threshold is, and the
control arm runs exactly that way — without this the off arm would warn on
every run about a risk it does not carry, and a warning that fires on half
the campaign is one nobody reads. The index topic: it is what turns a pair
row into "my pair", so states-without-index is a gate that stands down on
every tick while looking configured, which is the same silent-veto-removal
this guard exists to catch.
```

### prox-stop-default-on-pose-topics

**Proximity stop defaults and blind spots** — attached to `proximity_stop_enabled_ = dp("proximity_stop_enabled", true);`

```text
Proximity stop (coordinated yield). ON by default and deliberately NOT
tied to coordination_enabled: the guard is inert until it actually tracks
a peer (intents from teammates, or the pose topics below), so single-robot
runs are bit-for-bit unaffected. Caveat: with coordination_enabled=false
the 1 Hz intent heartbeat is also off, so the guard NEEDS the pose topics
to see anything — the wiring below warns when that leaves it blind. See
the yaml section for the field rationale (documented panic line:
robot-robot < 1.5 m closing).
```

### exploit-vantage-ring-params

**Exploitation vantage ring parameters** — attached to `exploitation_enabled_  = dp("exploitation_enabled", true);`

```text
Exploitation. When enabled the planner ingests tree targets off
targets_topic and circles each at n_vantages occlusion-free vantage points
(default 3 => ~120 deg apart), dwelling exploit_dwell_sec at each. A target
is "successfully exploited" once min_vantages_required clear-LoS vantages
are dwelled. Standoff = trunk radius + vantage_standoff_m (clamped to the
FOV range). Disable to get pure exploration.
```

### exploit-dwell-sync-param

**The per-vantage capture barrier** — attached to `exploit_dwell_sync_enabled_ = dp("exploit_dwell_sync_enabled", true);`

```text
Vantage-ring rendezvous barrier. ON by default: the whole point of the ring
is overlapping simultaneous views of one trunk state, and without the
barrier the first robot to arrive burns its dwell alone while the peer is
still driving — on a 3/3 quota the early robot can close the target solo
and the peer arrives to dwell an angle nobody needs. Distinct from the
`rendezvous_*` params above, which are the comms-reconnection barrier at
the anchor pose; these two are the per-vantage capture barrier.
```

### refinement-region-topic-absolute

**Why the refinement topic is absolute** — attached to `publish_refinement_regions_ = dp("publish_refinement_regions", true);`

```text
Fine-TSDF region relay (see the member comments). The default topic is
built ABSOLUTE from robot_name_, like every other cross-node topic here
(goal_pose, dscovox_node/*): the packaged launch files pass robot_name as
a parameter but do NOT namespace this node, so a relative default would
resolve to the global scope and silently never match the namespaced
scovox_node. An explicit param value is used verbatim.
```

### explog-provenance-git-rev

**Run parameters and binary provenance** — attached to `#ifdef EXPLO_PLANNER_GIT_REV`

```text
The independent variables of the experiment, recorded IN the data file
rather than only in the harness manifest — a file that cannot say which
arm produced it has to be trusted to a directory name.
PROVENANCE. No derived artefact in this project currently carries any:
the CSV schema changed silently between campaigns and nothing in the data
recorded which version produced it. schema_version (written by the logger
itself) covers the event format; these identify the binary.

EXPLO_PLANNER_GIT_REV comes from explo_planner_git_rev.h, regenerated on
every build (cmake/StampGitRev.cmake), so it is the revision this binary
was actually compiled from. It was previously a configure-time compile
definition and went stale on any rebuild without a reconfigure, which is
why cr5's binary said 6ce7ad0 while its manifest said 1a097dc. The build
stamp below still disambiguates two binaries built from the same
revision, which a revision alone cannot.
```

### explog-fleet-identity-both-forms

**Fleet identity as configured and resolved** — attached to `exp_log_->addParamStr("team_robot_names", join(team_robot_names_, ","));`

```text
Fleet identity, as CONFIGURED and as RESOLVED. Both, because they answer
different questions and a run whose masks turn out to be nonsense needs
both answered from its own file: the joined array says which fleet the
launcher believed in, and the id/hash pair says what this robot actually
indexed its masks by. robot_id -1 / team_hash 0 is the unconfigured
(legacy) run, which is what a defaults run must show.
```

### explog-sensor-model-from-fcfg

**Logging the sensor model** — attached to `exp_log_->addParamNum("fov_hfov", fcfg.hfov);`

```text
THE SENSOR MODEL. Not previously recorded anywhere in the event log, at
all — which meant a run_start row could not answer "what sensor did this
planner think it had?", and the answer was wrong for every campaign
before this generation (a 60 deg cone for a 360 deg lidar). A binary
generation defined by a change to these numbers has to carry them, or the
only proof of which model a cell ran under is the sha256 of the node plus
an out-of-band memory of what that build contained.

From fcfg, not from the raw parameters, for the same reason the
separation block below gives: this is what the evaluator was constructed
with. And the derived flag alongside the inputs, because it is what
actually decides the arrival gate's behaviour.
```

### explog-polar-beside-n-yaw

**Why enable_polar is logged beside n_yaw** — attached to `exp_log_->addParamBool("candidate_enable_polar", ccfg.enable_polar);`

```text
Recorded BECAUSE candidate_n_yaw is recorded, and immediately beside it.
n_yaw only bites on polar-generated candidates; with enable_polar=false
the candidate set comes from the frontier path, which fixes its own
count and never consults n_yaw. The campaign harness runs polar OFF
(FRONTIER_ONLY=1), so a reader who sees candidate_n_yaw drop 4 -> 1 and
infers a 4x smaller candidate set is wrong -- and nothing in the run
record contradicts them. A parameter whose meaning depends on another
parameter cannot be logged without it; that pairing is the witness.
```

### explog-separation-from-config

**Separation logged from the outcome** — attached to `exp_log_->addParamNum("separation_weight", separation_.config().weight);`

```text
Separation term. Logged from separation_.config(), which is what the
planner will actually use, NOT from the raw parameters — configure()
refuses an out-of-range weight and leaves the term off, and recording
the request rather than the outcome is how a control cell ends up
indexed as a treated one.
```

### explog-auto-knobs-after-resolution

**Auto knobs logged after resolution** — attached to `exp_log_->addParamNum("coord_claim_radius_m", coord_claim_radius_m_);`

```text
The three "0 means auto" knobs, logged AFTER resolution (the auto branch
runs well above this block), so what lands in the record is the value the
planner used and not the sentinel that asked for it.

coord_claim_radius_m is the one that matters most and the one most
easily misread: 0.0 resolves to fov_max_range, which this generation
DOUBLES (10 -> 20 m) for the lidar FOV. The shipped params pin 10.0, so
the auto branch does not run and the disc is unchanged -- but that is a
fact about the current yaml, not about the code, and anyone who sets it
back to 0.0 silently doubles the coordination disc as a side effect of a
sensor change. Only the resolved value can say which happened; the
manifest's coord_claim_radius_m_in_params records the request, and a
request is not an outcome.
```

### explog-rendezvous-arm-timing-params

**Rendezvous timing params in every arm** — attached to `exp_log_->addParamNum("rendezvous_depart_delay_sec",`

```text
GENERATION 19'S ARM DEFINITION, one member since retired: wait and
settle still decide how long a robot stays, but depart_delay has been
inert since generation 25 (the arming floor carries no notice term — see
the declaration) and is logged only so the param rows keep their schema. A
cell that does not carry these is a cell from a binary whose rendezvous
arm means something else. Logged unconditionally, on every arm, so the
control arms record the values they are NOT using — a one-sided param is
how a parameter quietly stops being comparable across arms.
```

### explog-link-veto-topics

**Logging whether the link veto can run** — attached to `exp_log_->addParamNum("reconnect_link_down_confirm_sec",`

```text
Whether the link veto could run at all. run_explo_sim_rviz.sh does record
this (`link_gate=` in run_manifest.txt), but the planner did not: an unset
topic makes linkGateReady() return false on its first line and silently
removes the veto from the trigger, and until now the only trace INSIDE the
planner's own output was a -1 sentinel in a free-text console line that no
reader parses. That split matters because the manifest describes what the
launcher intended and the event log describes what the node actually got —
when those disagree, only the second one explains the run. A parameter
that decides whether a safety check exists belongs in both, especially now
that generation 9's 90 s threshold is only safe while the veto is live.
```

### explog-link-gate-configured-not-active

**link_gate_configured is not liveness** — attached to `exp_log_->addParamBool("link_gate_configured",`

```text
NAMED "configured", NOT "active", and the distinction is the whole point.
This is computed from two strings being non-empty, so it reports what the
launcher asked for. It CANNOT report whether a sample ever arrived: it is
written at startRun, before any subscription has delivered anything. An
earlier draft called it link_gate_active and its comment claimed it was
"true only when both topics arrived", which would have passed a run with a
dead emulator -- gate configured, veto absent for the whole run, 90 s
clock running bare. That is the precise failure this generation exists to
stop, dressed as the check for it.

BOTH topics, not just the states one: the index is what turns a pair row
into "my pair", so states-without-index is a gate that looks configured
and stands down on every tick.

The runtime half of the question is answered by the one-shot
"link_gate_live:" console line emitted from the link-states subscription
on the first usable sample; check 3f requires both.
```

### explog-homing-blacklist-knobs

**Homing watchdog and blacklist knobs** — attached to `exp_log_->addParamNum("return_approach_window_sec",`

```text
Homing watchdog + failed-goal blacklist. Stamped because these are
exactly the knobs whose value the analysis has to know and cannot infer:
the mr1 campaign ran with failed_goal_ttl_sec at 60 while the manifest
recorded only the visited_goal_* pair, which is how a 60-vs-180 s
mismatch between the blacklist TTL and the nav timeout stayed invisible
for a whole generation.
```

### explog-nav-failure-thresholds

**Nav goal failure thresholds in the log** — attached to `exp_log_->addParamNum("goal_rotate_timeout_sec", goal_rotate_timeout_sec_);`

```text
The thresholds every nav_goal_failed row is tested against. Previously
absent from the echo entirely, which meant the number that decided 30 of
31 failures in the g6pilot campaign (goal_rotate_timeout_sec = 15 s)
appeared NOWHERE in the machine-readable output and was recoverable only
by reading the YAML by hand. nav_budget_sec is per-goal — computed from
goal distance by navBudgetSec — so it cannot be echoed as a constant;
echoing its four inputs makes it reconstructible instead.
```

### exploit-los-sightline-z-band

**Vantage sightline height and z-band** — attached to `if (exploitation_enabled_ && terrain_relative_z_) {`

```text
The LoS occlusion ray-march runs at the vantage sightline height
(vcfg.robot_z). map_cache_ is clipped to [roi_min_z_, roi_max_z_], so if
the sightline sits outside that band there are no voxels to hit and the
occlusion check silently passes everything. Warn if misconfigured.
Terrain mode: the band is robot-relative, so this absolute comparison is
meaningless — skip it. The vantage *ring geometry* is still flat-world
(one standoff circle, no slope-aware standoff or pitch), but the
sightline height is now snapped to the local ground by exploitZAt(), so
the LoS ray stays inside the ingested band and the occlusion test is
meaningful. See exploitZAt().
```

### planning-map-param

**The 2D planning map master switch** — attached to `use_planning_map_ = dp("use_planning_map", false);`

```text
Master switch for the 2D planning_map (default OFF). When false the planner
never subscribes to planning_map_topic and never consults a 2D map in
exploration or exploitation — straight-line costs, no obstacle/reachability
filtering. Set true (and point planning_map_topic at a publisher, e.g. the
scovox_node) to restore 2D free-cell + reachability filtering as a hard
startup precondition.
```

### explog-planning-map-config-late

**Logging the 2D map configuration** — attached to `if (exp_log_) {`

```text
C5: record the 2D-map configuration in the manifest. These two decide
whether candidate rejection and reachability filtering happen at all, and
until now a run_start row could not say which of the two regimes produced
it. Written here rather than in the block above because that block runs
before these parameters are read; run_start is still held until the first
live-clock tick, so anything added before then lands in the same row.
```

### cellworld-p1-coarse-grid

**Coarse cell world (P1)** — attached to `cell_world_enable_ = dp("cell_world_enable", false);`

```text
--- Coarse cell world (P1) ---------------------------------------
The M-TARE-style global layer: the ROI diced into cells, each carrying a
status derived from this robot's own map. In this phase it is pure
observation — the census is logged and optionally drawn, and NOTHING
reads it. Off by default, and when off it is not even configured.

The grid is derived from the ROI rather than given its own bounds on
purpose: a cell world covering different ground than the planner's ROI
would report coverage of an area the robot is forbidden to enter, and the
P1 gate compares the census against a measure taken over exactly the ROI.
```

### cellworld-status-thresholds-hysteresis

**Cell status thresholds and hysteresis** — attached to `ccw.covered_max_unknown   = dp("cell_covered_max_unknown", 0.15);`

```text
The status thresholds. PROVISIONAL until the P1 smoke run calibrates
them: `covered_max_unknown` is what "this cell is finished" means, and
the band up to `exploring_min_unknown` is the hysteresis that stops a
cell flapping. The band is load-bearing rather than cosmetic — every
committed status change resets the cell's known_by mask, so a flapping
cell would re-arm the §3.6 knowledge gate forever.
The defaults are the saturating-map values and are WORLD-CALIBRATED
knobs, in the same family as done_unknown_fraction — see CellWorldConfig
for the floor that makes them so, and how to read replacements off the
cell_census quantiles rather than guess them.
```

### teamworld-p2-switch-prereqs

**TeamWorld exchange switch and prerequisites** — attached to `team_world_hz_ = dp("team_world_hz", 0.0);`

```text
--- TeamWorld exchange + comms model (P2, plan §3.2/§3.3) --------------

`team_world_hz` is the switch and 0.0 is off, which is the shipped
default: at 0 nothing below is created and the node cannot be
distinguished from its P1 parent by any observable this branch's
equivalence gate reads.

The two prerequisites are hard failures, not degradations. Without the
cell world there is no census to send; without fleet identity `self_id` is
-1, so every mask this robot published would be empty and every peer's
census would merge under a sender id it does not have. Both of those
produce a fleet that looks like it is sharing and is not, which is the
single most expensive failure mode this whole plan exists to remove.
```

### separation-requires-teamworld

**Why separation requires TeamWorld** — attached to `if (separation_.enabled() && !(team_world_hz_ > 0.0)) {`

```text
The separation term reads peer positions out of the comms model, and the
comms model is fed by exactly one thing: incoming TeamWorld. With the
exchange off there is never an eligible anchor, so the term is on in the
manifest, on in the node's startup line, and inert in every decision — the
failure this project has already paid for more than once. Fatal rather
than a warning: a cell that ran the control's behaviour under the
treatment's name is worse than a cell that refused to start, because only
one of the two is visible in the results.
```

### teamworld-comms-ttl-default

**Why the comms TTL defaults to claims** — attached to `tmc.direct_ttl_sec     = dp("team_comms_ttl_sec", coord_claim_ttl_sec_);`

```text
Defaults to the intent-claim TTL rather than a number of its own: the
two answer the same question ("is this peer still there?") off two
streams published at similar rates, and two independently-tuned
liveness clocks would let peer_lost and LOST_COMMS disagree about the
same outage, in a log where both are written.
```

### teamworld-split-pub-sub

**TeamWorld split pub/sub topics** — attached to `team_world_pub_topic_ =`

```text
Split pub/sub, mirroring the intent stream exactly. The emulator gates a
stream by relaying it per-link, which it can only do if the publisher
and the subscribers are on different topics; a run that silently fell
back to a shared bus would show perfect comms in an arm meant to have
none, and would look like a null result rather than a plumbing fault.
```

### alloc-p3-requires-teamworld

**Why the allocator requires TeamWorld** — attached to `if (team_world_hz_ <= 0.0) {`

```text
Without the exchange there is no shared world, so "solve the same
problem" is vacuous: every robot allocates the whole map to itself and
takes the nearest cell. That is a fleet of one wearing the allocator's
clothes, and every field of the `allocation` event it emits looks
healthy — a refused focus, an agreeing peer_focus and a plausible tour.
Fatal rather than warned for that reason: it is the failure mode that
cannot be caught downstream.
```

### alloc-comms-mask-separate-knob

**The no-comms mask is separate** — attached to `alloc_cfg_.comms_mask     = dp("global_alloc_comms_mask", false);`

```text
The no-comms mask is a SEPARATE knob and defaults off, because it is not
part of the P3 claim: it is the machinery P4's reconnection value is
built from, and turning it on here would fold an untested cost model
into the phase whose gate is about focus agreement. See
GlobalAllocator::Config::comms_mask.
```

### reconnect-gate-p4-silence-vs-info

**Utility-gated reconnection (P4) modes** — attached to `{`

```text
---- Utility-gated reconnection (P4) --------------------------------

§3.6. `silence` is today's exact behaviour: the mid-run silence clock
decides alone. `info` layers the knowledge + value gate on top of that
clock, and can only ever SUPPRESS a dispatch the clock already allowed —
never bring one forward — which is how the §3.6.3 silence floor is kept.

NAMING: this is NOT the "info gate" the existing midrunGateSec() implements.
That one is an adaptive silence clock that lengthens the wait while our own
voxel backlog is small. This one is a value comparison at the moment that
clock expires. Both can be on; they compose in that order.
```

### reconnect-gate-requires-teamworld

**Why the info gate requires TeamWorld** — attached to `if (team_world_hz_ <= 0.0) {`

```text
Without the exchange, known_by is {self} on every cell forever, so the
knowledge gate is vacuously true for the whole run and the gate degrades
into a pure value test — while still logging a perfectly healthy verdict
with a large unshared_cells. Fatal for the same reason P3 refuses it: it
is the failure mode that cannot be caught downstream.
```

### reconnect-gate-alloc-cfg-params

**Allocator config for the info gate** — attached to `if (!global_alloc_enable_) {`

```text
The gate prices its two futures with the allocator's own solver, so it
needs a Config even when the allocator itself is off. Declaring a ROS
parameter twice throws, so these are only read on the path where the P3
block above did not already read them; comms_mask is deliberately not
read at all, since the gate overwrites it per solve (that difference IS
the measurement) — see evaluateReconnectGate.
```

### rzv-schedule-p5-meeting-constraint

**Scheduled rendezvous (P5)** — attached to `rendezvous_schedule_enable_ = dp("rendezvous_schedule_enable", false);`

```text
---- Scheduled rendezvous (P5) --------------------------------------

§3.5. The meeting stops being a landmark and becomes a CONSTRAINT on the
tours already being driven: pick the cell whose forced insertion costs the
team's makespan the least, and meet when the slower robot gets there under
its own tour.

The MIDPOINT floor that used to backstop this is gone (2026-09-16; see the
three removal notes in dispatchReconnect / pursuitFallback). It guaranteed
a destination when no tour cell was worth its detour, but a place with no
agreed time is not a rendezvous, and the floor_won telemetry showed it was
not even symmetric — both ends chose it in only 2 of 6 separated pairs.

THERE IS STILL A FLOOR. Passing none was tried for two days and cost the
ts4 smoke its whole N=4 rung: with empty tours the candidate set was empty
and the solve refused, once, 578 s before the end of a run that then never
held an appointment at all. Since 2026-09-18 `floor_cell` is the centroid
of the team's own vehicle cells — a quantity that, unlike the midpoint,
exists at proposal time and is agreed by construction, because it comes out
of the frozen shared snapshot every robot holds. See the floor block in
deriveRendezvousProposal for the full argument. A refusal therefore still
means something, but it now means the solve declined on cost, not that it
had nothing to look at.

WHY, in one measurement. Campaign mh1 ran the midpoint destination under
the §3.6 value gate: 100 of 110 evaluations declined, every single one on
COST (c_re > c_no), never on knowledge. That is not a gate being cautious,
it is the gate correctly reporting that the destination is worthless — the
midpoint of two poses that were in contact sits BEHIND both robots on
ground they have already covered, so meeting there always costs more than
it returns. Nothing about the gate can fix a bad destination; the
destination had to change.
```

### rzv-schedule-interlock

**The rendezvous scheduler interlock** — attached to `if (reconnect_enabled_ && !rendezvous_schedule_enable_ &&`

```text
---- THE INTERLOCK (2026-09-16) -------------------------------------

The scheduler defaults to FALSE and is absent from shared_params.yaml; it
is turned on out of band by the arm stacks. So the rendezvous and hybrid
arms could be requested, recorded and scored with the mechanism that
DEFINES them switched off, and nothing downstream would say so. After
today's removal of the three midpoint drives there is no longer even a
degraded behaviour left underneath:

  rendezvous + scheduler off -> armAppointment never arms, dispatch falls
      to the RENDEZVOUS tail and returns false. The arm does NOTHING. It
      is the `off` arm wearing the rendezvous label — and `off` is a
      SEPARATE ARM OF THE SAME EXPERIMENT, so this does not merely void
      one cell, it silently moves it into a different condition.
  hybrid + scheduler off -> `may_defer` is false forever, so dispatch is
      pursuit's ladder and nothing else. It is the `pursuit` arm wearing
      the hybrid label, which is worse than a null: it manufactures
      agreement between two arms that the design needs to differ.

Neither is detectable after the fact. Both produce complete, healthy,
well-formed runs. `rendezvous_agreed` events are simply absent, and
absence of a log line is exactly the evidence the nav-global-planner
episode proved we cannot read (it needs a same-binary control to mean
anything). So the check has to be here, at start-up, and it has to be
fatal — a WARN in a 3000-second console log is a check that has stopped
checking.

This cannot fire at defaults: rendezvous_expected_peers defaults to 0,
which forces reconnect_enabled_ false above, so a stock single-robot or
uncoordinated launch never reaches the condition. It bites exactly the
population it must — a coordinated multi-robot cell that asked for a
reconnect arm — and the fix is one line in the arm stack.
```

### rzv-schedule-requires-teamworld

**Why the scheduler requires TeamWorld** — attached to `if (team_world_hz_ <= 0.0) {`

```text
Without the exchange the two robots never converge on a world, so
"agreement by construction" has no construction to rest on: each would
solve its own private allocation, derive its own appointment, and drive
to a different cell — while logging an entirely well-formed
rendezvous_agreed event at each end. Exactly the failure that cannot be
caught downstream, so it is fatal here.
```

### rzv-schedule-speed-not-nav-est

**The schedule speed is not nav's** — attached to `rzv_cfg_.speed_mm_s =`

```text
The speed the SCHEDULE assumes, deliberately not nav_speed_est_mps_.
That one is the nav watchdog's model and is ~3x conservative on purpose
(a leg failing slowly must still time out); reusing it here would push
every t_meet three times too far into the future and the appointment
would always be beaten by the outage ending on its own. This one wants
the honest cruise speed, and it must be identical on both robots — it
is an input to a value both ends derive independently — which is why it
is a shared param and not a per-robot estimate.
```

### rzv-schedule-depart-safety

**The rendezvous depart safety multiplier** — attached to `rzv_cfg_.depart_safety_milli = static_cast<int>(`

```text
The safety multiplier on the drive estimate. It reached this tree as half
of a per-robot departure rule — leave when 1.2x your own drive remains,
so the far robot leaves first and the arrivals coincide — and that rule
is deleted (generation 19: departure is the agreed occurrence, and the
spread is absorbed by the barrier instead of by leaving early). What it
does now is mark up the reachability floor in RendezvousScheduler::solve,
which is fleet-wide rather than per-robot, so it MUST match across the
team or the derived interval would differ between robots.

`rendezvous_depart_margin_sec` went with the departure rule; it had no
other reader.
```

### rzv-schedule-proposer-rule

**The proposer rule and its assumption** — attached to `if (rendezvous_expected_peers_ != fleet_.size() - 1) {`

```text
THE PROPOSER RULE, and the one assumption it rests on. The pair is
derived by the LOWEST-ID robot and echoed by everyone else, and the
handshake runs only inside the team-complete guard — where, by
definition, every peer the fleet has is live. That is what lets the
proposer be a constant (fleet id 0) instead of a "lowest LIVE id"
election: with the whole fleet present there is nothing to elect, and a
constant cannot disagree with itself the way a liveness scan run at two
slightly different instants can.

It is only true if team-complete really means the WHOLE fleet. The
harness passes rendezvous_expected_peers = N_ROBOTS-1 and nothing else
ever should, but if it ever passed fewer, "complete" would fire on a
subset that need not contain robot 0 — every member of that subset would
wait for a proposal nobody was making, and the arm would log a clean
stream of "no agreed pair" refusals that reads exactly like a mechanism
deciding not to fire. There is no safe degradation from that, so it is
fatal here rather than a warning nobody reads.
```

### rzv-schedule-proposal-period

**The rendezvous proposal period** — attached to `rendezvous_proposal_period_sec_ =`

```text
How often the proposer may issue a NEW pair. Every re-issue costs one
TeamWorld period of disagreement (the peers have not echoed yet), so
this trades agreement probability against how stale the meeting cell is
allowed to get. At team_world_hz=1 the default puts the exposed window
at ~1 s in 30.
```

### rzv-schedule-requires-heartbeat

**Why the scheduler requires the heartbeat** — attached to `if (!coord_enabled_ || coord_heartbeat_hz_ <= 0.0) {`

```text
THE PROTOCOL'S ONLY DRIVER. refreshRendezvousSnapshot and
maintainRendezvousProposal are called from exactly one place —
heartbeatTick — and that timer only exists when coord is enabled with a
positive rate. With it off, the handshake never runs a single round,
rendezvous_agreed_ is never valid, and every arming refuses with "no
(cell, interval) pair was agreed": the silent-null failure this block
already refuses to ship for rendezvous_expected_peers, arriving through
a different parameter.
```

### rzv-schedule-teamworld-only-wire

**TeamWorld is the protocol's only wire** — attached to `if (team_world_hz_ <= 0.0) {`

```text
THE PROTOCOL'S ONLY WIRE. The (cell, interval) pair travels in
TeamWorld's rendezvous block and nowhere else, and TeamWorld itself is
switched entirely by this rate: the publisher, the subscription and the
whole drain live inside `if (team_world_hz_ > 0.0)`. It also defaults to
0.0 and is NOT in shared_params.yaml, so this is a live way to configure
a rendezvous campaign in which the protocol never runs at all.

It is worth being blunt about what that would have produced, because it
would not have looked like a fault: every robot refuses every arming,
never manoeuvres, and finishes — a complete, plausible, entirely null
result set with no failed run to investigate. The other half of the same
hole is the anchor: rendezvousTeamMutual() reads TeamModel, which is fed
only by this drain, so with no TeamWorld no anchor is ever stamped
either.
```

### rzv-schedule-mutual-contact-ttl

**The mutual-contact TTL budget** — attached to `const double mutual_ttl = team_model_.config().direct_ttl_sec;`

```text
THE MUTUAL-CONTACT BUDGET. Not the commit rule's — the commit latches
confirmations and consults no TTL at all, deliberately, because a peer's
choice of pair is a decision it cannot revise rather than a liveness
claim that can age out. What DOES run on a TTL is everything that has to
be true NOW: TeamModel::Peer::direct, and through it
rendezvousTeamMutual(), and through that the proposer's derive gate and
rendezvous_anchor_time_ — the shared origin every agreed interval is
measured from. A TeamWorld rate slower than that TTL means peers read
stale more often than fresh, the team never reads mutually whole, and
the protocol stalls one step earlier than it used to: no pair is ever
derived, so there is nothing to confirm.

Warned rather than fatal: the exact threshold depends on jitter and
loss, and a campaign deliberately probing a slow team_world_hz should be
able to run. 3 messages inside the TTL is the same slack the default
(1 Hz against 5 s) leaves.
```

### pursuit-mdp-p6-predictor-string

**MDP interception (P6) predictor choice** — attached to `{`

```text
---- MDP interception (P6, §3.7) ------------------------------------

`trail` is the shipped chase and the default; `mdp` aims the first
waypoint at the intercept instead. Read as a string rather than a bool for
the same reason reconnect_gate is: the alternative to the model is not
"nothing", it is a specific other aiming rule that has a name, and a
`pursuit_mdp_enable=false` column would not say which one ran.
```

### pursuit-mdp-hard-requirements

**Hard requirements of the MDP predictor** — attached to `if (!global_alloc_enable_) {`

```text
Three hard requirements, all fatal for P5's reason: without them the
predictor refuses on every single dispatch and the run logs a healthy
stream of refusals that reads exactly like a model deciding not to fire.

The tours are the allocator's OUTPUT. There is no other producer of one
in this node, so with the allocator off `my_tour` is empty on every
robot, nothing is ever broadcast, and every chase in the arm falls
through to the trail — a treatment arm that is bit-identical to its
control while looking configured.
```

### pursuit-mdp-speed-not-nav-est

**The prediction speed is not nav's** — attached to `pursuit_mdp_cfg_.my_speed_mps = dp("pursuit_mdp_speed_mps", 0.5);`

```text
The speed the PREDICTION assumes, deliberately not nav_speed_est_mps_.
Same distinction the rendezvous scheduler draws and the same numbers:
nav_speed_est_mps_ is the watchdog's model and is conservative on
purpose, and a horizon computed from it would propagate the chain far
past where the peer can be — the model would confidently intercept ahead
of a robot it had merely mis-timed. The budget still uses the
conservative one, because "when will I be there" and "how long am I
allowed before giving up" are different questions with different safe
directions to be wrong in.

Physically the same quantity as rendezvous_speed_mm_s, kept as a
separate knob because the two mechanisms are separately switchable and
P6 has to be runnable with P5 off. The consistency check below is there
because "set one, forget the other" is the obvious way to get this
wrong, and the symptom would be two subsystems disagreeing about how
fast the same robot drives.
```

### explog-arm-stamp

**The arm stamp analyses group by** — attached to `if (exp_log_) {`

```text
THE arm this run belongs to, and the field an analysis must group by.

Stamped HERE, not up with reconnect_mode among the other param lines,
because it is a function of knobs that are not read until this point in
the constructor. Written earlier it would compile, run, and record every
M-TARE cell as plain "hybrid" for the whole campaign — the flags it reads
are still false that far up. Order within the dump is cosmetic; being
downstream of every input is not.

reconnect_mode alone is NOT the arm. The control arm is "no reconnection
at all", which is not a reconnect_mode value — it is expressed as
reconnect_enabled=false, and the harness has to pass SOME mode alongside
it (it passes "hybrid"). So a control run is stamped reconnect_mode
"hybrid", and anything grouping on that column pools the control into the
hybrid cell: the hybrid mean becomes the average of treatment and control,
and the control arm ceases to exist. The information was always in the
file, split across two fields; nothing was reading both. This collapses
them once, here, where the planner knows the answer.

The M-TARE prefix follows the same principle one phase further out. P1 and
P2 are pure observation — their whole claim is that they change no
decision — so they do not rename the arm. P3 and P4 DO change decisions:
the allocator reorders candidates and the §3.6 gate suppresses dispatches.
A run with either engaged is not the hybrid arm, and pooling it into the
hybrid cell would do to that comparison exactly what grouping on
reconnect_mode does to the control.

P5 joins them for the same reason and not by analogy: the scheduler
changes WHERE a manoeuvre goes and WHEN it leaves, which is a decision
change of exactly the kind P3 and P4 make. A scheduled hybrid pooled into
the hybrid cell would average the new destination against the midpoint
the whole phase exists to replace.

P6 likewise: the predictor changes where the chase drives. It cannot
actually reach this predicate alone — it is fatal without the allocator,
which is already in the OR — but it is listed anyway, because a term left
out on the grounds that another term implies it is a term that silently
stops being true the day the implication is relaxed.
The predictor also needs its own SUFFIX, not just membership in the OR
above. Without one, a chase aimed by the MDP and a chase aimed by the trail
both stamp "mtare_hybrid": two different treatments under one name, pooled
by anything that groups on this column — the identical failure the prefix
and the reconnect_mode collapse were each introduced to prevent, one level
further out. With it the stamp reproduces the harness's arm token exactly
for all six (mtare_off, mtare_pursuit, mtare_rendezvous, mtare_hybrid, and
the _mdp counterparts of the two that chase), so the directory name and the
param dump cannot disagree about which arm a cell is.

trail appends nothing, so every pre-P6 run stamps the byte-identical
string it stamped before.
```

### dscovox-fused-map-latched-sub

**Fused map subscription from dscovox** — attached to `std::string dscovox_topic = dp("dscovox_topic", std::string(""));`

```text
The dscovox mapping node fuses every robot's voxels (multi-robot
consensus) and publishes the WHOLE fused map as a ScovoxMap topic. We
subscribe with the matching latched QoS (KeepLast(1) reliable +
transient_local) so the current map is delivered immediately on connect —
this replaces the old blocking GetRegion service call, and with it the
MultiThreadedExecutor + dedicated callback group that call required.
FOV raycasting, scoring and frontier extraction still run locally on the
ROI-clipped copy rebuilt into map_cache_ each PLAN tick.
```

### dscovox-fusion-counters-fleet-gated

**Per-source fusion counters subscription** — attached to `if (fleet_.configured) {`

```text
The same node's per-source counters, which answer "whose voxels are
arriving" — a question the fused map above cannot be asked, because it
carries one merged grid with no attribution left in it. Latched like the
map, so the current totals land on connect rather than on the next tick.

FLEET-GATED. peer_fusion_deltas_ is indexed by fleet id, so with no fleet
list there is no index to write into and the subscription would only cost
bandwidth. The vector is sized on the FIRST sample rather than here, so
that "still empty" keeps meaning "dscovox has never told us anything".
```

### planning-map-sub-only-when-enabled

**Planning map subscription gating** — attached to `if (use_planning_map_) {`

```text
Only subscribe when the planning_map is enabled. Leaving the subscription
uncreated guarantees latest_plan_map_ stays null for the whole run, so
every planning_map use site (all guarded on latest_plan_map_) takes its
map-less path — the planner cannot consult a 2D map even if one is being
published on the topic.
```

### planning-map-frame-check

**The planning map frame check** — attached to `if (!msg->header.frame_id.empty() &&`

```text
Frame check, matching the one the ScovoxMap path already does.
planMapCellAt/isCellFree/unknownFractionInRoi index this grid with
raw world XY and no TF at all, so if the publisher stamps it in a
frame that is not numerically the planner's map frame, every
lookup silently reads the wrong cell — candidates rejected as
occupied on the strength of geometry from somewhere else. The
usual sim publisher is scovox_node, which stamps its integration
frame (<robot>/odom); that is only safe because map->odom is
published as identity. Nothing enforces that, so say so out loud
when it stops being true.
```

### ctor-intent-rendezvous-anchor

**Recording the rendezvous anchor on contact** — attached to `if (reconnect_enabled_ && have_pose_ &&`

```text
Rendezvous: record where we were the last time we heard a
teammate. That pose is inside the comms bubble, so it is the
cheapest point to return to for reconnection. onIntent already
drops our own echo, but we gate on robot_id here too since we read
our live pose. have_pose_ guards the very first ticks before TF.
```

### ctor-intent-last-contact-pair

**Per-peer last-contact snapshot** — attached to `auto& rec = last_contact_[msg->robot_id];`

```text
Mesh reconnection: with robot-carried radios BOTH endpoints of
the lost link have moved, so keep the whole last-contact pair
per peer — my pose (the anchor generalised), its advertised
pose, and its declared goal, the pursuit trail head. Stamped
with local receipt time, same clock discipline as claim expiry.
```

### ctor-teamworld-qos-keeplast-two

**Why TeamWorld uses KeepLast(2)** — attached to `auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();`

```text
KeepLast(2), not (8) like intents. Full state means an older message
carries nothing a newer one lacks, so a deep queue would only buy the
right to merge stale censuses after the fresh one — work that changes
nothing and delays the drain. Two, not one, so a message arriving while
the executor is inside a planning pass is not dropped by the middleware
before the copy-only callback ever runs.
```

### ctor-teamworld-sim-clock-timer

**TeamWorld broadcast on its own sim timer** — attached to `const std::chrono::duration<double> period(1.0 / team_world_hz_);`

```text
On its own timer rather than the planning tick: a robot that stops
publishing its world while it is busy planning goes silent from its
peers' point of view at exactly the moment it has most to say, and the
TTL cannot tell that apart from a radio outage.
SIM clock, like every other timer in this node. A wall timer would pace
the broadcast in real seconds while direct_ttl_sec is measured in sim
seconds off this->now(), so at RTF 0.3 a healthy peer publishing at
"1 Hz" would arrive every 3.3 sim-seconds and a 5 s TTL would be one
dropped message from declaring it lost.
```

### ctor-linkgate-index-team-of-two-plus

**Link gate accepts teams of two or more** — attached to `const std::string joined = join(names, ", ");`

```text
Any team of two or more (P7, §P7). This was restricted to
exactly two while "connected to at least one peer" and "the team
is complete" were the same statement and there was no reading of
the gate that was defined for three; the row scan below now
requires ALL of this robot's links to be up, which IS the
completeness the mid-run trigger is written against, so the
restriction has been replaced by the meaning it was standing in
for. A team of one has no link to read and is still refused.
```

### ctor-linkgate-row-validity-mask

**Link-state row validity mask** — attached to `if (kCols == kLinkColsWithValid && d[k + 9] == 0.0) continue;`

```text
Validity mask, same discriminator and same preference order as
link_logger.py. A row the emulator did not compute -- no pose yet,
or a pose that has gone stale -- must not be read as a
disconnection, because the transition out of it would manufacture
a reconnection. Generation 9 publishes the emulator's own verdict
in column 9; older tables do not, and there the path-loss floor
(~49 dB at one metre, monotone increasing) stands in for it. An
invalid row is published fully zeroed, so on a 10-column table
both tests fire together and the second is redundant, not
conflicting.
```

### ctor-linkgate-unknown-peer-down

**Unknown peers count as not connected** — attached to `bool connected = true;`

```text
A peer with no usable row yet is UNKNOWN, and unknown counts as not
connected. That direction is the safe one: it clears the veto, so
the trigger falls back to the record-age clock it would have used
with no gate at all. The opposite default would let a table that
has not warmed up suppress dispatches.
```

### ctor-linkgate-live-token-site

**Where the link_gate_live token is emitted** — attached to `RCLCPP_INFO(get_logger(),`

```text
The token is matched verbatim by gate_g8.py check 3f. Do not
reword it without updating the gate and its calibration: this
line is the ONLY proof in the record that the veto was ever
actually able to run, as opposed to configured on a topic
nothing published to.

Emitted HERE, from the subscription, and not from
linkGateReady(). linkGateReady() is only ever called from the
mid-run trigger branch and from pursuitFallback(), so a run
whose team simply never went silent long enough to consult the
gate would produce no line — and 3f would score a perfectly
healthy run as a hard failure. Everything the token asserts is
established at this point: the index resolved (checked at the
top of this callback), a row addressed to us parsed, and the
sample is by construction zero seconds old.
```

### ctor-proxstop-peer-pose-topics

**Peer localiser poses for proximity stop** — attached to `const auto entries =`

```text
"<robot_name>:<topic>" entries, e.g. "curt:/curt/pcl_pose". These are
the peers' localiser poses: already map-frame, ~10 Hz, and independent
of the peer's planner state — the intent heartbeat alone is 1 Hz and
goes silent in several states, which at 0.8 m/s closing speeds leaves
metre-scale pose lag. With nothing configured the guard runs on
intents alone (the sim launches, where no localiser runs).
```

### ctor-proxstop-dead-nav-cancel

**The dead nav2 cancel path** — attached to `proximity_nav_cancel_action_ =`

```text
THE CANCEL PATH IS DEAD, AND IT IS KEPT DELIBERATELY. Read this before
trusting the word "cancel" anywhere in this file or in a robot log.

This client was written for nav2, where bt_navigator turns every goal_pose
into a NavigateToPose goal it sends itself and honours cancel-all from any
client. The navigator actually running in these experiments is
simple_nav_3d, which subscribes to goal_pose as a plain PoseStamped and
serves NO action — there is no rclcpp_action server anywhere in that
package. So '/<robot>/navigate_to_pose' has no server,
action_server_is_ready() is false forever, and every call site guards on
it: the cancel is never SENT, as opposed to sent-and-ignored.

What that costs operationally: little, because the brake goal was always
the primary mechanism and the cancel the redundancy. A goal published at
the robot's own pose stops it in ~100-150 ms — the navigator declares
arrival on its next 50 ms tick and stops republishing active_goal, and
simple_nav_planner_node clears its plan and publishes an EMPTY path on its
next 100 ms tick, which makes the controller publish a zero Twist. What it
costs in DIAGNOSIS is real, and it has been paid once: "belt and braces"
reads as two independent stop mechanisms when there is one, so a stop that
fails must be diagnosed as a single point of failure. The log lines were
corrected for this (see abandonNavGoal); the comments are corrected here.

Kept rather than deleted: a client with no server does no work, the
readiness guard is correct as written, and deleting it would be a code
change with no behavioural effect during a window where every behavioural
change costs a binary generation. If this stack ever gains a real
NavigateToPose server, the redundancy returns on its own.

Never used to SEND goals — the goal_pose topic remains the only command
path.
```

### ctor-refinement-region-latched-qos

**Latched QoS for refinement regions** — attached to `region_pub_ = create_publisher<scovox_msgs::msg::RefinementRegion>(`

```text
Latched (transient_local) both ends, mirroring the scovox_node
subscription: an ADD fires exactly once per target id (ingest dedup —
no retry path exists), so it must survive a DDS discovery race at
startup-with-backlog and a restarted scovox subscription. Replay is
safe: adds are keyed-replace and removes are idempotent, so a late
joiner converges to the correct region set.
```

### ctor-startup-log-goal-z-roles

**Goal z under the UGV and UAV navigators** — attached to `flatten_goal_z_ ? "flattened to 0 (strict-2D nav)"`

```text
"the navigator ignores z" is true of simple_nav_3d's UGV role, whose
arrival test is std::hypot in XY. It is NOT true of its UAV role, which
measures a 3D distance to the goal — so on a UAV a wrong goal z is a
wrong goal, not a harmless passenger. Both halves are named because the
old wording ("nav2 ignores z") licensed the reader to stop thinking
about z on every platform.
```

## ExploPlannerNode::onFusionCounters

### fusion-counters-sized-first-receipt

**Sizing fusion counters on first receipt** — attached to `if (peer_fusion_deltas_.size() != static_cast<size_t>(fleet_.size()))`

```text
SIZED ON FIRST RECEIPT, not at construction, and a sample that names no
sources still sizes it. That keeps three states apart that would otherwise
collapse into one: dscovox has never published (vector empty — arrival is
UNMEASURED), dscovox is publishing and has heard nobody (vector of zeros —
arrival is MEASURED and zero), and a peer has sent something (non-zero).
Only the middle one is evidence.
```

### fusion-counters-decrease-rebaseline

**A counter decrease is a dscovox restart** — attached to `if (total < peer_fusion_deltas_[static_cast<size_t>(id)]) {`

```text
A DECREASE IS A RE-BASELINE, NOT ARRIVAL. The counters are node-local
and restart at zero when dscovox does; subtracting across that restart
would read as a huge negative, and latching the old higher value would
make the peer look permanently talkative. Take the new value as-is and
say so once, because any baseline captured before the restart is now
meaningless and a reader of the logs needs to know which side of it the
numbers came from.
```

## ExploPlannerNode::loadLatestMap

### loadmap-roi-clip-zband

**ROI clip and z-band on map ingest** — attached to `if (!map_cache_->updateFromScovoxMap(`

```text
The topic carries the whole fused map; clipping here keeps map_cache_
bounded to the ROI as the old per-region GetRegion service did, so frontier
extraction and the doLogStep map stats (both walk the whole grid) stay
bounded. The [band_lo, band_hi] z-band defines one consistent
observation volume that is also what FovEvaluator clips rays to
(doPlan re-syncs the evaluator from eff_roi_*_z_ each tick).
```

## ExploPlannerNode::tick

### tick-event-log-first

**Event log and peer sweep ordering** — attached to `startExperimentLog();`

```text
Event log first: run_start must be the file's first line, and it can only
be emitted once the clock is live (see startExperimentLog). The peer sweep
rides the same tick rather than the coordination heartbeat, which returns
early when coordination_enabled is false — the control arm needs its
outage timing recorded too.
```

### tick-drain-teamworld-order

**Where drainTeamWorld runs in the tick** — attached to `drainTeamWorld();`

```text
Fold in whatever TeamWorld arrived since the last tick and advance the
comms model's clock. AFTER updatePoseFromTF because the local-priority
rule is anchored on the cell this robot is standing in, and a merge run
against last tick's pose defends the ground the robot has just left.
BEFORE the state dispatch so a planning pass this tick sees the merged
census rather than one that is a tick behind — and unconditionally, ahead
of the proximity-hold return below, because a held robot is exactly the
one whose peers most need it to keep tracking them.
```

### tick-all-rejected-reset-off-plan

**Resetting the all-rejected counter off PLAN** — attached to `if (state_ != State::PLAN) {`

```text
Any tick spent outside PLAN ends an all-rejected episode. The counter must
be cleared here and not only where a goal is finally selected: doPlan has
several other exits (max steps reached, a queued exploit target, a failed
map load, the coverage latch) and the machine can leave PLAN entirely for
a reconnect manoeuvre. Resetting on selection alone would let "planning
recovered after N ticks" span a whole rendezvous and bill minutes of
deliberate off-PLAN behaviour to a planning stall.
```

### tick-proximity-hold-driving-states

**Which states the proximity hold covers** — attached to `if ((state_ == State::NAVIGATE || state_ == State::RETURN_NAV ||`

```text
Coordinated proximity stop: while a nav goal is in flight, yield to a
higher-priority teammate moving nearby. Checked at the full 10 Hz tick
rate, before the state dispatch, so the hold pre-empts everything the
driving states would otherwise do this tick. The stationary states are
deliberately exempt — a dwelling/integrating/planning robot is already
still, and the moving peer's obstacle grid treats it as an ordinary
obstacle.
RETURN_HOME is in the driving set for the strongest version of the reason:
under mission return BOTH robots converge on start poses ~3 m apart, so the
final approach is the one leg of the run where a crossing is guaranteed
rather than incidental.
```

### tick-wait-planning-map-handling

**planning_map handling at startup** — attached to `{`

```text
planning_map handling:
 - use_planning_map_ == true  -> hard precondition; wait for it.
 - use_planning_map_ == false -> not subscribed; start as soon as the
   fused map + pose are ready and run map-less (straight-line costs, no
   2D obstacle/reachability filtering, in both exploration and exploit).
```

### tick-wait-pre-mission-hold

**The pre-mission hold and its clock** — attached to `const double held_for = missionElapsed();`

```text
THE PRE-MISSION HOLD, and it is ANDed with the preconditions rather
than sequenced after them: the two clocks are independent, so the
release is at max(preconditions, hold) and a robot whose map arrives
late does not get a shorter hold than one whose map arrived at once.

Measured from missionElapsed(), which is each robot's OWN baseline —
about a second apart across a fleet in the banked smokes. That is the
right clock even so. The hold is not an appointment and nothing has
to line up on it; what it has to do is guarantee a stretch of
co-located mutual contact, and a second of skew at the far end of a
60 s window does not threaten that. Using a shared wire time here
would import the agreement problem the hold exists to solve.

A clock that is not live yet reads -1, which is BELOW the hold and so
holds — the safe direction. It cannot hold forever: missionElapsed()
latches its baseline on the first tick with a positive clock, and
without a clock there is no map and no pose either, so the
preconditions are false regardless.
```

### tick-wait-start-warn-gating

**Gating the waiting-to-start warning** — attached to `RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,`

```text
Name the missing precondition so a stuck startup (wrong topic /
namespace / QoS, dead mapper, no TF) is diagnosable instead of a
silent indefinite wait.

GATED ON !preconds_met, NOT ON !start (2026-09-18). `start` is the
conjunction of the preconditions AND the pre-mission hold, so
keying the WARN off it meant that during the hold — when every
input has arrived and nothing is wrong — the log carried
"Waiting to start: map=1 pose=1 planning_map=1", a line that
reports all three preconditions satisfied while asserting the robot
is still waiting for them, every 5 s for the length of the hold.
That is a message which can only mislead: it names no fault, offers
no next step, and contradicts the "Pre-mission hold: Xs of Ys" INFO
printed immediately above it. The hold already has its own two log
lines (progress and completion); this WARN is for faults, and being
held is not one.
```

### tick-done-idle-and-latch

**DONE idle behaviour and the latch** — attached to `if (done_action_ == "idle") {`

```text
done_action == "idle": stay alive so targets released after
coverage-done still pull the planner into the exploit sub-loop
(field flow: the scheduler releases targets AT the coverage-done cue,
which would race a shutdown). hasPending() includes a still-ACTIVE
target and activate() resumes it, so an exploitation interrupted by
the step budget also continues here, vantage by vantage, until the
queue drains. When the queue empties the exploit sub-loop reverts to
EXPLORE -> PLAN, whose coverage check immediately lands back in DONE
(streak already at threshold) unless the map regressed.

The latch criterion overrides that: "finished" is a one-way property of
this robot, and leaving DONE would contradict it — the run-completion
rule (every planner reads DONE) is evaluated on the state column, so a
latched robot that flicks back out to EXPLOIT_PLAN would re-open a run
it already ended. Targets are not part of this experiment's completion
question; if a configuration ever needs both, it wants done_criterion=
streak, where DONE is genuinely revocable.
```

### tick-done-seek-coast-watchdog

**The DONE-SEEK coast watchdog** — attached to `if (done_seek_coasting_) {`

```text
Coast watchdog. Nothing else in this branch touches the navigator, so if
done_seek let a goal stand this is the ONLY bound on it — without
it an unreachable goal drives a "finished" robot until teardown.
Two exits, and a coast can never leave the platform rolling —
but only ONE of them brakes, and the asymmetry is deliberate:
  [arrived] fires BECAUSE the robot already stopped (30 s with no
    progress: the navigator declared arrival, or its local planner
    stopped producing a path and the controller fell back to a
    zero Twist). There is nothing left to stop, so it just ends
    the coast and parks.
  [timeout] fires with the robot STILL MOVING, so it is the one
    that calls abandonNavGoal and publishes the brake goal.
```

### tick-done-shutdown-remove-regions

**Removing live regions before shutdown** — attached to `removeLiveRefinementRegions();`

```text
Disarm any fine-band regions whose target never reached DONE:
the step-budget checks and the rendezvous give-up land here
WITHOUT passing finishActiveTarget, and shutting down with their
regions live would leave scovox fine-integrating those trunks for
the rest of the run. The actual shutdown is deferred one tick so
DDS gets a cycle to flush the removes (a reliable publisher's
unsent history dies with the process).
```

## ExploPlannerNode::transitionTo

### transition-reconnect-clock-states

**Which states keep the reconnect clock** — attached to `const bool manoeuvre = (s == State::RETURN_NAV || s == State::RETURN_SYNC ||`

```text
Reconnect clock. The manoeuvre owns RETURN_NAV / RETURN_SYNC / PURSUE, plus
any PROXIMITY_HOLD taken while already inside one. Staying within that set
keeps the clock running, which is what lets a single manoeuvre span the
RETURN_NAV -> RETURN_SYNC arrival and the HYBRID chase -> meeting-point
handoff without restarting; leaving it means the manoeuvre resolved, one
way or the other, so the clock stops and the CSV reverts to its sentinel.
```

### transition-manoeuvre-outcome-classify

**Classifying a reconnect manoeuvre's outcome** — attached to `const bool manoeuvre_released = releaseHeld(manoeuvreReleaseEligible(live));`

```text
Classified mechanically from the two facts that decide it, with the
raw fields alongside so an analysis can re-classify: the barrier having
CONFIRMED the team complete is what "reconnected" means, and landing in
a state where exploration is over — DONE, or RETURN_HOME under mission
return — instead of PLAN is what "gave up" means. RETURN_HOME must be
in that set or every latch-ended manoeuvre in a mission-return run
re-buckets from gave_up to abandoned and the outcome mix stops being
comparable across campaigns.

releaseHeld, NOT bare teamComplete (2026-09-18). This line used to claim
that "every release path in the manoeuvre states tests exactly that",
and no release path did: they all test
releaseConfirmed(teamComplete(...)), which adds the
reconnect_release_confirm_sec dwell. On the bare read a single
range-edge flicker ends a manoeuvre as "reconnected" that the barrier
itself never released. See the longer note on the appointment
classifier below; the two sites share one defect and one fix.

manoeuvreReleaseEligible, NOT bare teamComplete, for the same reason
(2026-09-18, generation 23): it is the expression the barrier that just
ended actually tested, which for an APPOINTMENT manoeuvre is teamSettled
or the generation-27 reachable door, and for every other manoeuvre is
teamComplete. Reading teamComplete here
would label a contagion-held appointment "reconnected" at the instant its
own barrier was still refusing to release, and the rows this classifier
feeds are the ones the arms are compared on.

Safe to read appointment_manoeuvre_ here: it is cleared further down this
function, AFTER both classifiers, precisely so the two of them see the
manoeuvre that is ending rather than the absence of one.

Hoisted above the log line, and above the `if (exp_log_)`, so the line
and the event carry ONE expression's answer. Computing it twice is how
a log and its event drift apart, which is the whole defect class this
generation is closing.
```

### transition-outcome-in-log-line

**Why the outcome is in the log line** — attached to `RCLCPP_INFO(get_logger(),`

```text
The outcome is IN the line because it was not, through generation 7, and
the plaintext log therefore could not say how any manoeuvre resolved.
The destination state is not a proxy for it: under mission return every
manoeuvre ends "-> RETURN_HOME" whether it reconnected or gave up, so a
log-based classifier had nothing to key on. Across the 5 banked g6pilot
firings it labelled 4 "open_at_horizon" — never ended — beside a duration
it had just parsed from this very line, and the 5th "arrived_waiting" off
a stray meeting-point line. 0/5 correct against the jsonl, which records
one genuine reconnection among them.
```

### transition-appt-barrier-verdict

**Carrying the appointment barrier's verdict** — attached to `appointment_barrier_released = appointment_manoeuvre_ && manoeuvre_released;`

```text
The appointment classifier below runs after the clear on the last line
of this block and must still know whether it was THIS appointment's own
barrier that released (generation 27: the release can hold with
teamSettled false — the reachable door — and without this the
closure-released meeting would bank as "no-show arrived=true", the
smoke20 sign reversal in a new dress).
```

### transition-appt-manoeuvre-only-clear

**The only clear site of the manoeuvre flag** — attached to `appointment_manoeuvre_ = false;`

```text
The manoeuvre is over, so it is no longer serving an appointment —
whatever became of the appointment RECORD, which the block below decides
separately and on its own predicate. This is the only clear site: the
latch's entire purpose is to outlive closeAppointment(), so clearing it
there would restore the bug it exists to fix.
```

### transition-appt-bookkeeping-block

**Appointment bookkeeping as its own block** — attached to `if (appointment_armed_) {`

```text
---- P5: appointment bookkeeping -------------------------------------

Its own block, deliberately NOT folded into the manoeuvre-end block
above, because the two do not share a lifetime. A DEFERRED appointment
stands while the robot is still EXPLORING — that deferral is the whole
mechanism — so reconnect_active_ is false and the block above never runs
for it.

Every armed appointment must produce exactly one outcome event, or the
1:1 join the two kinds exist to support silently stops holding. The three
ways one can end are all handled here: the team came back, this robot
kept it (departed, then either met somebody or did not), or the run ended
underneath it.
```

### transition-appt-label-barrier-decision

**The label must name what the barrier did** — attached to `const int live_now = accountedPeerCount(this->now());`

```text
THE LABEL MUST NAME WHAT THE BARRIER DID (2026-09-18).

This read rendezvousTeamMutual() — every pair in DIRECT contact — while
doReturnSync releases the robot on teamComplete(accountedPeerCount):
every peer *I* have heard first-hand, on the claim table or on TeamWorld.
The two differ by ONE-WAY CONTACT, by MY EDGES vs ALL PAIRS, and by
FINISHED PEERS — but NOT by relayed reachability: accountedPeerCount's
two reachability channels are both first-hand, see the correction at the
release site in heartbeatTick(). Its third channel is `finished`, which
does relay and has no TTL, so a peer that ended its run counts toward
teamComplete forever and can never be mutual; on that peer the barrier
releases and rendezvousTeamMutual() never will.
At N=2 the all-pairs quantifier collapses onto the single peer, so there
the gap is mutuality alone; at N>=3 they part company on both counts, and
the ts4 N=3 cell wrote both verdicts in the same millisecond:

  ...799657  Rendezvous: full team connected (2/2) -> re-planning...
  ...799709  Reconnect manoeuvre ended after 81.9 s sim: reconnected
  ...799740  Rendezvous appointment at cell 44 closed: no-show

The robot reconnected and recorded a no-show for the meeting that
reconnected it. `outcome` is this arm's headline metric, so that is not a
cosmetic disagreement — it is the measurement inverting on exactly the
team sizes the campaign is about.

The 2026-09-16 move ONTO the mutual predicate was right for the problem
it solved (two robots labelling one appointment differently after a
one-way heal) and wrong here, because the two robots are no longer the
ones disagreeing: the barrier and the classifier are, on the same robot.
The fix is not to pick the stricter predicate, it is to pick the SAME
one — and it has to be the barrier's, because the barrier is what
actually ended the manoeuvre. A label describing a condition the code
never acted on is a label about nothing.

The stricter fact is not discarded, it is DEMOTED to its own column:
`mutual` on the outcome event records whether the reunion was whole-team
two-way, or only one-way / partial, so the two remain separable in the analysis
instead of one silently overwriting the other. That is what the schema
6 -> 7 bump carries.

AND IT HAS TO BE THE BARRIER'S DECISION, NOT THE BARRIER'S INPUT
(2026-09-18). The paragraph above is right and the first implementation
of it still missed by one layer: it called teamComplete(), which is what
the barrier CONSULTS, while the barrier actually releases on
releaseConfirmed(teamComplete(...)) — a reconnect_release_confirm_sec
(6 s) dwell that exists precisely because a single range-edge flicker
can satisfy teamComplete for one tick while draining no map deltas.
Classifying on the raw read labels such a flicker "reconnected" and
closes the appointment on a manoeuvre the barrier never ended and a
meeting that never happened. Gen 18 under-counted reunions by using a
predicate stricter than the barrier's; that version over-counted them by
using a looser one. Same disagreement, same file, opposite sign.

releaseHeld() is the non-mutating twin of releaseConfirmed() — see its
definition for why the classifier must not call the mutating one.

teamSettled AND NOT manoeuvreReleaseEligible (2026-09-18, generation 23),
even though the manoeuvre classifier above uses the latter. Two reasons,
and they point the same way:

  * appointment_manoeuvre_ has ALREADY BEEN CLEARED by the time control
    reaches here — the clear sits in the manoeuvre-end block above, which
    is why that block's classifier can still read it and this one cannot.
    manoeuvreReleaseEligible() here would silently mean teamComplete.
  * It would be the wrong question anyway. This block labels the
    APPOINTMENT, which arms on !teamSettled and supersedes on teamSettled
    whether or not any manoeuvre ever ran for it — a deferred appointment
    stands while the robot is still EXPLORING and has no barrier at all.
    teamSettled is this site's share of the five-site rule.

PLUS THE BARRIER'S OWN VERDICT (2026-09-19, generation 27). The
reachable door means an appointment barrier can now release with
teamSettled still false — a gathered team whose last pair one trunk
keeps dark — and on teamSettled alone this block would bank that
meeting as "no-show arrived=true": the sign reversal documented below,
returned through the release. So team_back also accepts
appointment_barrier_released, the verdict the manoeuvre-end block
captured from manoeuvreReleaseEligible BEFORE clearing
appointment_manoeuvre_. It is the barrier's DECISION, not its input, so
the flicker argument above is preserved; and it is scoped to the
appointment whose own manoeuvre just ended — a DEFERRED appointment
still closes only on teamSettled or run end, so a bridge topology alone
still closes nothing as reconnected.

So the two classifiers in this function CAN disagree, and that is not the
2026-09-18 defect returning: they label different objects (the manoeuvre
that ended vs the appointment that stood), and they disagree only where
a non-appointment manoeuvre ends underneath a deferred appointment.
```

### transition-appt-departed-resolution

**Departed and undeparted appointment resolution** — attached to `const bool kept_and_done = appointment_departed_ && !manoeuvre;`

```text
A DEPARTED appointment is resolved the moment the manoeuvre keeping it
ends. An UNDEPARTED one must survive leaving the manoeuvre states,
because going back to exploring before the deadline is precisely what
the mechanism does — closing it here would delete the appointment on
the first declined chase.
```

### transition-appt-outcome-derivation

**Deriving the appointment outcome label** — attached to `const char* appt_outcome = team_back                ? "reconnected"`

```text
Derived from what happened, not re-decided. Whether the team is back
IS the outcome; whether this robot reached the cell is the only other
fact the classification needs. The two are kept apart on purpose: a
"no-show" with arrived=false would be a navigation failure wearing a
coordination failure's name, and counting those together is how a
mechanism that never arrives anywhere looks like one whose partner
never turns up.

`run-ended` (2026-09-17) separates "the meeting failed" from "this
robot's run ended at the meeting". Those are different events and the
classifier used to call them both no-show: in the smoke20 N=3
rendezvous cell all three robots logged `no-show arrived=True`
immediately after exploration_complete -> reconnect_end
reason=coverage-latched, while all three were standing on cell 45
together. A no-show recorded at a meeting that happened is not a
miscount of one arm's outcomes, it is a sign reversal: the rendezvous
arm's headline failure count was made of its successes.

It sits ABOVE the arrived/unreachable split because it answers a prior
question. Whether this robot reached the cell says how the appointment
was going; it says nothing once the run ended underneath it, and a
teardown mid-drive is no more "unreachable" than a teardown on the cell
is a "no-show". team_back still wins: if the team is back, the meeting
succeeded regardless of what ended the run.

`unplaceable` (2026-09-18) sits in the same position and for the same
reason as `run-ended`: it answers a prior question. If this robot could
not turn the agreed cell into a position it never departed for the
meeting at all, whatever the arrival test says — appointmentPoint()
returned `latest_pos_`, so arrived=true means "I am where I already
was", not "I reached the cell". Below team_back, because a team that
came back anyway did meet; above the arrived/unreachable split, because
that split is only meaningful once there was somewhere to go. It also
reports arrived=false: the row must not carry an arrival this robot
did not make.
```

### transition-state-change-dwell-event

**The state_change event and its dwell** — attached to `if (exp_log_) {`

```text
state_change, emitted before the new state is installed so `from_dwell_sec`
still measures the state being LEFT. -1 when the dwell is unknowable: the
very first transition of the run, where state_enter_time_ is still the
default-constructed SYSTEM-clock value and subtracting it from a sim-time
now() would throw inside a timer callback.
```

### transition-done-run-end-placement

**Where run_end is written on DONE** — attached to `if (s == State::DONE) {`

```text
DONE is the single funnel for every ending the node reaches while running
(coverage, step budget, barrier give-up), so the ending's REASON is
captured here rather than at each of those sites.

Whether run_end is written here depends on what DONE means for this
configuration, because run_end must be the LAST line of the file — an
analysis reads it to decide whether the file is complete, and lines after
it would make `events_written` a lie:
  done_action=shutdown — DONE is terminal, the node exits within a tick, so
    write it now while the run totals are still meaningful.
  done_action=idle     — the node stays up and a target arriving later
    pulls it back into the exploit sub-loop, which produces more events.
    Defer to the destructor, which the last of them precedes by
    construction, and carry this reason there so the ending is still named
    properly rather than degrading to "node-destroyed".
```

### transition-dwell-sync-per-dwell

**The dwell-sync barrier is per dwell** — attached to `if (s == State::EXPLOIT_DWELL) {`

```text
The dwell-sync barrier is per-DWELL: its wait clock starts at entry (the
moment the robot is physically staged on the vantage) and neither latch may
survive into the next capture — a timed-out barrier would otherwise disable
the barrier for every remaining vantage of the run, and a started one would
skip the wait at the next vantage entirely.
```

### transition-dwell-entry-staged-publish

**Broadcasting staged on dwell entry** — attached to `if (intent_pub_ && have_active_intent_) {`

```text
Declare this robot staged on the claim its teammates hold their barriers
against. This entry is the only point that can honestly say it: the dwell
is reached from NAVIGATE only after BOTH the XY-arrival gate and the
post-arrival yaw settle, so the platform is standing still on the vantage
it claimed — not merely inside a tolerance of it, and not parked on an
approach waypoint. Published on the spot instead of waiting for the next
~1 Hz heartbeat, for the same reason as the post-dwell dwelled_mask
broadcast: a peer already parked on its own angle is re-anchoring its
dwell clock every tick until it hears this, so a second of stale staging
is a second shaved off the team's simultaneous-capture window.
```

## ExploPlannerNode::allocVehicles

### alloc-vehicles-vehicle-set

**The allocator vehicle set** — attached to `std::vector<AllocRobot> ExploPlannerNode::allocVehicles(`

```text
Vehicle set: every live, unfinished robot at its freshest known position,
gossip included (§3.4). Self first-hand at (x, y); peers from the team model,
which is the only place a relayed position is dated.

Callers: the P3 allocator (from doPlan, at the planning pose), the §3.6
reconnect gate (from tick, at the current pose) and the rendezvous snapshot.
Shared on purpose, but see the header comment: `pos_max_age_sec` is now a
deliberate exception to "one vehicle set, and every consumer sees the same
one". Only the allocator passes a bound; the gate and the snapshot pass 0.
```

### alloc-vehicles-position-age-ttl

**Keying the vehicle TTL on position age** — attached to `const double age = team_model_.positionAgeSec(id, now_sec);`

```text
POSITION age, not lastKnownAgeSec: a relayed status update refreshes
"when did we last hear about this robot" while leaving the pose we
hold for it untouched, so keying the TTL on the latter would let a
peer we can hear about but not locate keep its cells forever. That is
team_model.hpp's rule 3, and the separation term keys the same way.

Returns -1 when no position is held, which allocPeerPositionFresh
rejects — but have_position already decides that case, so the -1 never
has to mean anything here.
```

### alloc-vehicles-unlocatable-cell

**Unlocatable and expired peers drop out** — attached to `r.cell = (p.have_position && fresh)`

```text
No position at all is not "at the origin": an unlocatable robot is
dropped from the problem (cell -1) rather than defaulted onto a cell
it is not in, because a fictional position moves the makespan and so
moves THIS robot's tour too.

An EXPIRED position is treated as the same thing, and that is the
whole change: `cell = -1` already means "drop this robot from the
problem and return its cells to the pool", so the TTL reuses the
existing unlocatable-robot semantics and GlobalAllocator needs no
change at all. Note what is NOT dropped — the robot stays in the
vehicle list with its in_comms and finished flags intact, so it still
counts toward all_in_comms and toward the fleet size. Only its claim
on ground goes away.
```

## ExploPlannerNode::separationAnchors

### separation-anchors-skip-finished

**Finished peers do not repel** — attached to `if (p.finished) continue;`

```text
A teammate that has latched exploration done is parked or homing and is
clearing no ground, so backing away from it buys nothing and costs the
information that made the candidate attractive. Excluded rather than
down-weighted: "how much less should a finished robot repel" is a knob
with no evidence behind it, and zero is the one answer that needs none.

Bounded, not exact: `finished` is only ever set from a FIRST-HAND
message (team_model.cpp sets sp.finished = obs.finished for the sender
and nowhere else). The wire's gossip arrays carry positions and
last-heard stamps but no finished bit, so a peer reached only through a
relay keeps whatever this robot last learned first-hand — false, if it
has never been in direct contact. Such a peer therefore still repels
after it parks. The error is one-sided and small: the term repels from
somewhere a robot genuinely is, just for longer than it needs to, and
it costs at most the discount on candidates near a parked robot. Adding
a gossip finished bit means widening the message, which is a change to
every arm, and it is not worth making inside a campaign. Note the same
staleness already applies to the allocator's vehicle set, which reads
p.finished the same way and long predates this term.
```

## ExploPlannerNode::doPlan

### plan-step-budget-guard

**The PLAN step-budget guard** — attached to `if (step_ >= max_steps_) {`

```text
Step budget: never start a new exploration step past max_steps_. Only
reachable with done_action=idle — the exploit sub-loop drains its queue
past the budget (deliberate, see State::DONE) and its empty-queue revert
lands here; without this it would sneak one stray exploration hop per
drain. In shutdown mode LOG_STEP routes to DONE before PLAN ever runs
with a spent budget, so legacy behaviour is untouched.
```

### plan-step-budget-coverage-sample

**Fresh coverage sample at budget end** — attached to `const char* budget_src = "";`

```text
Measure fresh rather than letting the event inherit the last sampler
tick. The budget, not coverage, is what ended this run, so the fraction
is descriptive here rather than the tested quantity — but it is still
read as "how much was left when it stopped", and a stale answer to that
is a wrong answer.

Cost is one extra ROI walk per PLAN tick spent at a spent budget, NOT
"once per run" as this said through generation 7. finishOrRendezvous's
return value is discarded here, so a DEFERRED decision (rendezvous
confirmation window) leaves the node in PLAN and re-walks on the next
tick until the window resolves. It is bounded by reconnect_confirm_sec
(3.0 s) and stamps nothing twice — recordExplorationComplete is keyed on
step_, which cannot advance during a deferral. Unreachable in the g8r1
configuration regardless: finishOrRendezvous returns at the
mission_return_enabled_ && have_home_ branch, above the deferral gate,
and startReturnHome always returns true. Documented because the walk is
a grid traversal and "once per run" is what a reader would budget for.
Two statements, deliberately. Written as a single nested call the two
arguments are only INDETERMINATELY SEQUENCED, and g++ evaluates the
bare `budget_src` first at both -O0 and -O2 — so the out-parameter the
inner call fills arrives at the outer call as the "" it was initialised
to, noteCoverageDecisionSample's `if (source && *source)` guard rejects
it, and last_coverage_source_ silently keeps the PREVIOUS decision's
source. Which is this fix defeating its own purpose: recording the
source this decision was actually taken on is the whole point.
```

### plan-coverage-saturation-check

**Coverage saturation check in PLAN** — attached to `if (done_criterion_ == "latch") {`

```text
Coverage saturation check. Runs before candidate generation so we
can short-circuit out of PLAN entirely once the ROI is fully known.
Requires N consecutive low-unknown ticks to avoid premature DONE
from a momentary measurement gap.

Under done_criterion == "latch" this site is NOT the deciding one — the
metrics tick is, because it runs in every state — but the test is repeated
here anyway so the criterion does not silently depend on the sampler being
enabled (metrics_period_sec <= 0 disables that timer entirely), and so a
robot that saturates between two sampler ticks finishes on the PLAN tick
rather than waiting out the sampler period. maybeLatchCoverageDone is
idempotent, so evaluating it from both hooks latches exactly once.
```

### plan-streak-decision-sample

**Recording the streak decision's fraction** — attached to `noteCoverageDecisionSample(unk, cov_src);`

```text
Record the fraction this decision was taken on, not the last sampler
tick's. Same defect the latch path had; it is only absent from the
pilot data because done_criterion=latch was configured throughout.
```

### plan-coverage-source-unmeasurable

**When the coverage source cannot measure** — attached to `RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,`

```text
-1 = the selected source cannot measure: done_coverage_source
"planning_map" with no planning_map published, or a degenerate ROI
box. ("auto" falls back to the scovox column measure, which always
yields a value once the fused map is loaded, so it only lands here
on a bad ROI.) Surface it instead of silently relying on max_steps.
```

### rzv-appointment-is-a-clock

**A standing appointment is a clock** — attached to `if (appointment_armed_ && !appointment_departed_) {`

```text
P5: a standing appointment is a CLOCK, not a decision.

The decision was taken when it armed, so the mid-run trigger below is
suppressed for as long as one stands — re-entering it would re-derive the
same appointment from the same frozen snapshot and consume one of the
manoeuvre attempts on every PLAN tick of the outage, exhausting the budget
in seconds while the robot had not moved. What remains is this block: keep
exploring, and break off exactly when the departure rule says the robot
can still just make it.
```

### rzv-appointment-supersede-dwell

**Superseding an appointment on reunion** — attached to `const int live_for_supersede = accountedPeerCount(this->now());`

```text
SAME PREDICATE AS THE ARMING TEST BELOW — see the long note there for
why it is teamSettled and not rendezvousTeamMutual. The event that
STARTS an appointment and the event that CANCELS it have to be one
event, or one robot cancels while the other drives.

Under contagion that requirement bites harder, not softer: C armed
because A announced a break, so C must also WAIT for A to stop announcing
it. Cancelling on C's own teamComplete — which was true throughout —
would supersede the appointment on the tick after it armed, every tick,
and the contagion would be inert while looking live in the logs.

This is also the cheap filter on the beacon-suppression false positive
the arming note describes: a peer that went quiet for less than the
countdown is back in the count before the deadline, and the appointment
is cancelled here without anyone leaving.

AND IT DWELLS, SINCE GENERATION 23. The predicate matched the arming
site's; the EVIDENCE THRESHOLD did not. Arming acts on the team coming
apart, which one missed beacon shows; superseding acts on the team coming
BACK, and one claim arriving inside the 5 s TTL makes that true for a
single reading. Without a dwell here, a range-edge flicker that happened
to land on a PLAN tick cancelled the appointment outright — while the
barrier that guards the actual reunion refused the very same evidence for
6 s. The two are now the same threshold as well as the same predicate.
Cancelling a few seconds late costs a few seconds of a standing clock;
cancelling on a flicker strands a peer that is already driving to the cell.

A READ, NOT A WRITE, and that is not a stylistic choice. The window is
stepped once per heartbeat in heartbeatTick (see the member declaration
for why the writer has to be the unconditional site). This block is inside
`appointment_armed_ && !appointment_departed_`, which is untaken for most
of a run, and doPlan is entered only in State::PLAN — roughly once per
step, not at 10 Hz. Ticking a continuity window from here would measure
"two consecutive plan entries agreed", which two samples thirty seconds
apart satisfy trivially, and between appointments it would not decay at
all. dwellHeld asks the question without owning the clock.

CONJOINED ON A FRESH COUNT. dwellHeld takes the live read as its
`eligible` argument and returns false when it is false, so the heartbeat's
1 Hz view can never supersede an appointment on a tick where this robot's
own current count says the team is not settled. The dwell can only ever
subtract from the old condition, never add to it.
```

### rzv-supersede-close-mutual-read

**Closing a superseded appointment** — attached to `closeAppointment("superseded", /*arrived=*/false, /*waited_sec=*/0.0,`

```text
The outage ended on its own before the deadline. The appointment did
not cause that and must not claim it — but it DID stand for the whole
outage, and an appointment that simply vanishes leaves a
rendezvous_agreed with no rendezvous_outcome, which reads offline as
one that is still open at the end of the run. Closed as superseded.

`mutual` is a genuine second read, NOT the branch condition restated.
It used to be `rendezvous_schedule_enable_`-era code that passed
rendezvousTeamMutual() from inside `if (rendezvousTeamMutual())`,
i.e. the literal `true` on every superseded row — which falsified the
schema-7 contract that the field distinguishes a whole-team two-way
reunion from a one-way one. Here the team is complete by the branch; whether it is
also all-pairs-TWO-WAY is the open question the column exists to
answer. ("relayed" was the wrong word for the complement and is
corrected throughout — nothing relays an intent; see heartbeatTick().)
```

### rzv-deadline-departure-not-terminal

**Deadline departure is a mid-run manoeuvre** — attached to `reconnect_terminal_ = false;`

```text
An appointment-due departure is a MID-RUN manoeuvre: exploration is not
over, the deadline simply arrived. Say so explicitly, because
reconnect_terminal_ is ambient state whose RESTING value is `true` (it
is declared true and re-armed true at every manoeuvre end), and this
site reaches startReturnTo without passing either of the two places
that set it for a dispatch -- the mid-run clear below, or
finishOrRendezvous's terminal set. Inherited, the barrier that ends
this manoeuvre takes the run-ENDING wait cap (rendezvous_max_wait_sec)
instead of the mid-run one, so a deadline departure can finish the run.
Measured on ts1b: 40 barrier give-ups, every one flagged terminal, none
of which any dispatch had earned.

Must precede startReturnTo: transitionTo stamps reconnect_end with this
flag, so assigning after it would label the manoeuvre in the log as the
opposite of what it ran as. Same ordering note as resumeExploring.
```

### rzv-appointment-arms-on-separation

**Scheduled appointment arms on the separation** — attached to `if ((reconnect_mode_ == ReconnectMode::RENDEZVOUS ||`

```text
A SCHEDULED APPOINTMENT ARMS ON THE CLOCK, NOT ON THE SILENCE GATE.

Pure rendezvous only. The appointment is a promise made while the team was
whole — a fixed place AND a fixed time — so the thing that starts it
running is the separation itself, not a later decision that reconnecting
has become worthwhile. Arming it here does NOT send the robot anywhere:
it keeps exploring, and the departure rule above breaks it off at
t_meet minus its own travel, which is the mechanism that makes the fleet
arrive together.

Gen 10 armed it from dispatchReconnect instead, behind the mid-run gate,
and the gate needs reconnect_midrun_silence_sec of team-incomplete before
it fires. Whenever the agreed interval was shorter than that gate the
meeting time had already passed before any robot was permitted to look at
it: measured on a 2-robot cell, anchor 240 s, interval 20 s, armed at
378.6 s and 438.1 s, both robots recording 234.1 s of lateness on an
appointment neither could ever have kept. The place was agreed and the
time was decorative.

HYBRID IS INCLUDED HERE AS OF 2026-09-16, and the mid-run trigger below
was changed in the same edit to make that safe.

It used to be excluded, on the reasoning that hybrid's rule is "pursue
when deemed necessary unless the rendezvous is due", so the necessity
judgement must fire first — and that arming on the clock would delete its
pursuit half, because the mid-run trigger is suppressed while an
appointment stands. The second half of that was true and is now fixed at
the trigger. The first half confused two different things: WHEN the
appointment is armed, and WHEN the robot departs for it. Arming does not
move t_meet — that is anchor + interval, and the anchor is the shared
separation event — so arming early costs the pursuit half nothing. The
departure is still owned by the deadline.

What the exclusion DID cost was symmetry, which is the one property this
whole protocol exists to provide. Arming only from dispatchReconnect put
hybrid's appointment behind the mid-run gate, and that gate is per-robot
and genuinely asymmetric: the link-gate veto is computed from each robot's
OWN unshared backlog, and it refuses often (102 of 115 refusals on mt2
were the cost inequality alone). So one hybrid robot's gate says go and
arms, the other's says stay and never arms — and the first drives to the
agreed cell and waits out the full 240 s cap for a partner that was never
coming, logging a no-show against an appointment only one end ever held.
A meeting one participant does not know about is not a meeting, and
"hybrid keeps a rendezvous" has to mean the same thing on both robots or
the arm is not testing what its name says.

Arming at the separation, for both arms, makes the appointment a property
of the SEPARATION rather than of each robot's private judgement about it.
The judgement still governs the pursuit, which is where hybrid's rule
actually puts it.
NOT GATED ON have_rendezvous_anchor_ (2026-09-18). It was, and that was a
fifth anchor-keyed refusal surviving the four armAppointment already
dropped — see the block there explaining why: `t_meet` is the integer the
team committed, not `anchor + interval`, so the anchor cannot move the
meeting and cannot be a reason to refuse one. Keeping the conjunct here
had a cost the others did not, because of WHERE the flag is set: its only
write is inside `if (rzv_mutual)` on the heartbeat, while
maintainRendezvousProposal was deliberately un-gated from mutuality so a
follower can echo and commit without it. At N>=3 in this radio regime
all-pairs mutual contact holds for a few seconds at spawn and then
essentially never, so a robot that joined the agreement late — valid
rendezvous_agreed_, no anchor it ever stamped — could hold a perfectly
good committed appointment and be unable to arm it for the whole run.
The anchor is still stamped, still logged, and still the column that
separates an early agreement from a late commit.
```

### rzv-arm-predicate-team-settled

**One predicate for the team came apart** — attached to `const int live_for_arm = accountedPeerCount(this->now());`

```text
ONE PREDICATE FOR "THE TEAM CAME APART" (2026-09-18), and it is
teamComplete. Read this before changing the test: the node has now had
this defect twice, in both polarities, and both times the cause was two
predicates rather than the wrong one.

THE RULE. Arming here, superseding at the top of this function, the
outcome classifier in transitionTo(), the barrier release in
doReturnSync/doReturnNav and the rendezvous_spent_ release on the
heartbeat are FIVE SITES THAT MUST AGREE. Arm on !P, end on P, for one
P. If they disagree in either direction the arm ratchets:

  gen 18  armed on the weak predicate, cleared the latch on the strict
          one. The latch never cleared at N>=3, the arm went inert for
          70% of a 604 s cell, and that starvation is what the countdown
          rewrite was opened to fix.
  gen 19  armed on the strict predicate (!rendezvousTeamMutual) and
          closed, released and cleared on the weak one. The arm never
          STOPPED: arm -> 100 s -> drive to the cell -> barrier releases
          at once because teamComplete was true the whole time -> settle
          -> close -> latch clears on the next heartbeat -> !mutual is
          still true -> arm again, about every 106 s for the whole run,
          against a team that never separated. ~25 spurious regroups per
          3000 s cell, every rendezvous_outcome row reading
          reconnected/arrived=0/waited=0.0, and N=2 unaffected — so it
          would have reached the analysis disguised as a team-size
          effect on exactly the comparison this campaign exists to make.
  gen 27  weakens ONE side at ONE site, deliberately and without the
          ratchet: the barrier RELEASE (and only it) gains a reachable
          door — closure-or-finished over every expected peer — because
          the gen-26 N=3 smoke gathered all three robots at the cell
          and one trunk on one 8 m chord kept the mesh false to the
          duration cap (zero outcome rows, exploration over for half
          the run). Both ratchets above ran through the LATCH, and the
          latch still clears only on the strict dwelt teamSettled, so a
          closure-released, still-mesh-broken team cannot re-arm until
          a genuine reunion. The arm, the supersede, the classifiers'
          mesh half and the latch stay in lock-step on ONE P.

WHY teamComplete IS THE RIGHT P AND rendezvousTeamMutual IS NOT. Mutual
asks for all N(N-1)/2 links DIRECT and mask-confirmed at one instant. In
this radio regime (70 dB trunks, 30 m horizon) that holds for a few
seconds at spawn and then never again at N>=3 — so as a definition of
"the team is together" it declares a healthy team permanently broken,
and any rule built on it fires forever. teamComplete asks the weaker
question — "can I currently hear each of them" — which at least admits
healthy topologies that mutual declares broken.

P IS teamSettled SINCE GENERATION 23, AND THAT IS teamComplete PLUS ONE
TERM. The history above is why the change is made at all five sites in
the same edit, and why the term is the SAME function on both polarities
rather than a second predicate that happens to agree today.

WHAT THE EXTRA TERM FIXES. teamComplete is NOT relay-inclusive — a claim
to the contrary sat here until 2026-09-18 and was backwards.
accountedPeerCount counts DIRECTLY received traffic for reachability
(proof at the release site in heartbeatTick(); RobotIntent carries no
peer list and hmr_comms_sim_node forwards nothing). Its `finished`
channel IS relayed, but a finished robot is not a robot anyone is trying
to reach, so it cannot supply the missing hop. So in an A-B-C bridge with A-B
down: C hears both and stays put, while A and B each see one peer against
rendezvous_expected_peers=2 and BOTH ARM. That is a PARTIAL arming — two
robots break off for a meeting the third never attends — and it is not
what the directive says. The directive is "if ANY robot is disconnected,
ALL robots go".

Nor is the partition cosmetic. There is no map relay either: dscovox
subscribes to peers' raw scovox_bin and publishes only fused products for
LOCAL consumers, so nothing carries A's voxels to B through C. (An older
version of this comment asserted the map "still flows A->C->B" and
flagged it as unverified; it has since been checked and it is FALSE.
Do not restore it.) A and B are genuinely partitioned and C is the only
robot that can close it, so suppressing their arming would be the wrong
repair — the meeting is needed.

THE REPAIR IS CONTAGION, one hop. Every robot publishes its OWN first-hand
!teamComplete as TeamWorld/team_incomplete, and teamSettled additionally
requires that no peer we are currently receiving from is announcing a
break. One hop suffices at every N and every topology, over the UNFINISHED
robots: a robot whose own read is COMPLETE has heard every unfinished peer
first-hand inside one TTL and therefore receives their bits itself, and a
robot whose own read is BROKEN arms off that read without needing anyone's.
So every unfinished robot arms iff some unfinished robot's own bit is set —
unanimous, with no relay and no second hop. A finished peer counts complete
through the `finished` channel with no contact at all, which costs nothing
because its bit is discarded at both ends on purpose (the publisher forces
team_incomplete=false at DONE; peerReportsTeamBreak skips `finished`).
TeamWorld.msg states the one residual window. (Contrast
in_range_mask, whose closure is exactly two hops: A-B-C is covered,
A-B-C-D is not.) The full argument, including why the announced bit must
be the first-hand read and never the derived armed state, is in
TeamWorld.msg.

WHAT IT COSTS, measured rather than asserted. Complementarity makes the
team-wide predicate equivalent to an all-pairs complete graph, and in the
generation-22 smoke (same radio regime) the full mesh held 29.0% of the
run at N=3 rendezvous, 41.1% at N=3 hybrid and 14.7% at N=4 under the 5 s
liveness TTL. So at N>=3 these arms now stand armed for most of the run.
That is NOT a permanent manoeuvre: per P5 above a standing appointment is
a CLOCK, not a decision — robots keep exploring until the departure
deadline — so what it produces is periodic rendezvous at the agreed
interval, which is the mechanism as described. It IS a large behavioural
change and generation 23 is not comparable with 22 on these arms.

WHY rendezvousTeamMutual IS STILL NOT P, even though teamSettled is also
an all-pairs condition in steady state. Mutual demands every link direct
AND mask-confirmed SIMULTANEOUSLY, at one instant, from one robot's
vantage; teamSettled is a conjunction of N first-hand reads each taken in
its own robot's own time and carried on a 1 Hz topic, so it degrades to
"nobody has recently said otherwise" instead of failing on every
sub-second flicker. The anchor needs the instantaneous version and says
so; the arm does not.

THE COST, stated rather than hidden. teamComplete counts received
intents inside a TTL, so it is one-way-blind and STATE-GATED — a peer
stuck in a long PLAN loop stops beaconing and reads as missing. That
false positive is real and it is why the countdown is 100 s and not 10:
a suppression episode that ends with room to spare inside the countdown
is cancelled by the supersede above before anyone departs. It cannot
ratchet, either, because the spent latch clears on exactly this
predicate's complement — one appointment per episode, not one per tick.
The mid-run pursuit trigger already accepts this same exposure on the
same count.

"WITH ROOM TO SPARE" IS reconnect_release_confirm_sec (generation 23),
and the asymmetry is deliberate. This site arms the instant the predicate
goes false, with no dwell; the supersede and the spent-latch release both
require its complement to hold for 6 s of heartbeats first. The two
directions are not the same claim. Coming APART is shown by a missing
beacon, and a robot that waits for confirmation of that is a robot that
departs late for a meeting it has already been told about. Coming BACK is
shown by a single arriving claim, which a range-edge flicker produces just
as readily as a reunion — and acting on it cancels a meeting a peer may
already be driving to, or hands out a second arming inside one outage. So:
arm on one sample, stand down on a held run. The practical cost is the
6 s at the end of the window, during which a suppression episode that has
genuinely ended has not yet been believed.

Every other precondition armAppointment tests is re-tested above rather
than left to it, because this site is evaluated on every PLAN tick and
armAppointment logs a rendezvous_agreed row on refusal as well as
success — calling it speculatively would bury the real rows under
hundreds of refusals.
```

### rzv-arm-reason-peer-separation

**Separation versus peer-separation arm reason** — attached to `armAppointment(teamComplete(live_for_arm, rendezvous_expected_peers_)`

```text
TWO REASONS, ONE PREDICATE. The arm is unconditional on !teamSettled;
the string only records WHICH half of it fired, so a contagion arm can
be counted without a new column or a schema bump. "separation" is this
robot's own read failing — the pre-generation-23 behaviour, unchanged,
so banked parsers that expect it still find it. "peer-separation" means
this robot can hear everyone and armed SOLELY because a peer announced
a break: it is the C of the A-B-C bridge, and the count of these is how
much the contagion actually did. Nothing in the tree matches on this
string, which is what makes a second value safe here.

WHERE IT LANDS: the plaintext "Rendezvous schedule [%s]" line in
armAppointment, NOT the jsonl — RendezvousAgreedEvent has no reason
field and adding one would bump a schema that several banked parsers
pin. grep the cell's planner logs for "[peer-separation]".
```

### reconnect-midrun-trigger-placement

**Mid-run trigger placement and hybrid chase** — attached to `const bool hybrid_may_chase =`

```text
Mid-exploration reconnect trigger (off unless reconnect_midrun_silence_sec
> 0). Sits AFTER the exploit branch (targets are the mission deliverable
and defer the trigger — remember that when reading firing times) and AFTER
the coverage check (a saturated robot must route through the terminal
path). Only evaluated in PLAN, i.e. between hops: detection latency past
the silence crossing is one residual hop (typically 15-60 s), which is
per-robot jitter the analysis inherits. The peer count is re-read here
because the heartbeat-maintained clock is quantized at 1 Hz and starvable
— without the re-check a peer that reconnected within the last heartbeat
period still reads missing and we brake for a manoeuvre that dissolves on
its first tick.

THE APPOINTMENT SUPPRESSION IS NOT ABSOLUTE FOR HYBRID (2026-09-16).
`!appointment_armed_` is the right gate for RENDEZVOUS — that arm has
nothing to do between the separation and the deadline but explore — and it
was the right gate for hybrid only while hybrid armed from inside this
trigger. Now that hybrid arms at the separation, an absolute suppression
would mean the appointment always exists by the time the gate fires, the
trigger never runs, startPursuit is never called, and hybrid collapses
into rendezvous with extra logging. So hybrid is let through while its
appointment is still PENDING — armed, not yet departed — which is exactly
the interval its own rule assigns to the chase:

    [ contact lost .... departure deadline .... t_meet + wait ]
      ^--- CHASE owns this ---^--- APPOINTMENT owns this ---^

Bounded to ONE chase per appointment by appointment_chase_tried_, and that
bound is load-bearing rather than tidiness. `++midrun_attempts_` below is
spent BEFORE dispatchReconnect and is not refunded when it returns false,
and dispatchReconnect returns false on exactly the path this opens up — a
chase that declines (stale record, uncoverable trail) while an appointment
stands. Without the flag a persistently stale record would spend the
entire attempt budget on consecutive PLAN ticks without the robot moving,
and budget exhaustion reverts the run to terminal-only reconnection, so
the failure mode would be "hybrid's pursuit half dies silently a few
seconds into the first outage". One attempt, then the robot explores until
the deadline, which is the correct behaviour for a chase that cannot start.
```

### reconnect-midrun-gate-threshold

**Mid-run gate threshold and cooldown clock** — attached to `if (!teamComplete(live, rendezvous_expected_peers_) && cooldown_ok) {`

```text
Threshold from the info gate when configured (dead-reckoned unshared
backlog crossing, clamped), else the legacy fixed clock — same value
exactly when reconnect_min_share_voxels=0. The COOLDOWN above stays
on the fixed clock either way: it paces retry pressure after a
failed manoeuvre, which has nothing to do with how much map the pair
holds. Gate maths runs only once the team actually reads incomplete:
midrunGateSec walks the peer table, and a fully-connected gated run
would otherwise pay that walk on every PLAN tick of the whole run.
```

### reconnect-link-gate-veto

**The link gate is a veto** — attached to `bool   link_veto     = false;   // stand down this tick`

```text
THE LINK GATE IS A VETO, NOT A CLOCK (§30.11, §30.24, §30.26).

missing_for is the TEAM-PRESENCE clock, not the peer's record age:
team_last_complete_time_ is stamped on the 1 Hz heartbeat while
accountedPeerCount() reads the team complete, and a peer stays
accounted until BOTH its claim (coord_claim_ttl_sec, 5 s) and its
TeamWorld `direct` flag (TeamModel::direct_ttl_sec, 5 s) age out. So
missing_for ~= peer_record_age_sec - TTL. Measured on g8r1's 5 banked
mid-run dispatches the difference is 4.98-4.99 s (4.99, 4.99, 4.99,
4.99, 4.98) -- the TTL, from below, to two decimals. A previous
revision of this comment quoted 4.98-5.01; there is no 5.01 in the
five, and an upper end ABOVE the TTL is not a value this quantity can
take, so the typo was also self-refuting.
An earlier draft widened this to 3.24-5.51 "because the only
banked copy is a %.0f-rounded plaintext number"; that was wrong.
team_incomplete_sec is a real 2-dp column and generation-8 cells DO
carry it, so no rounding allowance is needed.

TWO OFFSETS, NOT ONE, AND THE EARLIER "T + 5" CONFLATED THEM.
The TTL offset above is the gap between the two CLOCKS. Separately,
the trigger is evaluated on PLAN ticks, so the presence clock
overshoots T before anyone looks: measured 9.02-13.82 s past 240.
The record age at dispatch is therefore T + overshoot + TTL, and the
5 banked dispatches sat at 254.01-258.80 s against a nominal 240,
i.e. T + 14 to T + 18.8 -- not T + 5. Only the TTL half is
threshold-independent; the overshoot is set by tick cadence, which is
why this is quoted as a measured range and not as arithmetic. At
generation 9's T = 90 the same decomposition predicts a dispatch
record age around 104-109 s.
team_incomplete_sec is logged as its own column so the fired
inequality is recoverable offline at full precision.

It shares the defect record age has, which is what the veto below is
for: it ages whenever the peer is not SENDING, which includes a
teammate sitting in a long PLAN tick two metres away. The repair is
only to refuse the fires that cannot possibly help, NOT to re-time the
trigger.

Timing on link-down duration instead was tried and rejected against
the banked data. The two quantities are not variations of each
other: record age accumulates ACROSS outages (the beacon is
conditional, so silence spans up-periods), while continuous outage
resets at every flicker. On tl1's 30 hybrid cells only 1 of 286
outages ever reached the campaign's 240 s gate, so timing on it
would have dropped 17 of 19 mid-run fires — switching mid-run
pursuit off rather than correcting it, under a threshold that was
never tuned for that quantity. The veto drops 4 of 19: exactly the
fires that went out to a peer already on the radio. Those 4 do not
all arrive here — 3 came through this trigger and 1 through the
exhausted-chase escalation in pursuitFallback, which is why the veto
is applied at both sites and why gating only this one left a leak.

What the veto CANNOT fix, by design: the other half of §30.24's
"bought nothing" 42 % fired into a genuine outage that ended within
the ~14.6 s it takes to start moving. Suppressing those needs a
prediction of when the link returns, which is peer state the robot
has no deployable way to know. Left in deliberately.
```

### reconnect-link-down-debounce

**Link-down debounce and the connected disjunct** — attached to `if (link_connected_ ||`

```text
Debounced on reconnect_link_down_confirm_sec, which is the RADIO
debounce and nothing else. It used to borrow reconnect_confirm_sec
(the team-presence release confirm, 3.0): two unrelated questions
on one constant. Also makes link_down_sec on any dispatch
unambiguous: >= the confirm when the gate decided, -1 when the gate
was not in play at all (every g8r1 dispatch reads -1, which is how
we know that campaign ran with no veto).

It ships at 0, so in the default configuration the test below IS
the link_connected_ disjunct and nothing more. A positive debounce
was tried at 30 s and withdrawn: it cost A THIRD of the arm's
activation -- 18 arming cells down to 12, i.e. 6 of the 18 that
armed -- for 5 points of fire purity (see the member comment). An
earlier draft said "a quarter", which is 6/23, the share of the ARM;
the denominator the purity is traded against is the 18, not the 23.

The link_connected_ disjunct is not redundant. Whenever the radio
is up link_down_for is exactly 0.0, so at a confirm of 0 the bare
comparison is `0.0 < 0.0` -- false -- and the veto would pass a
chase at a peer that is on the radio right now, which is the one
case it exists to stop. Written this way "0" means the honest
thing: veto only while the link is actually up.
```

### reconnect-gate-event-hoisted

**Why the gate event is hoisted** — attached to `ReconnectGateEvent ev;`

```text
Hoisted out of the reconnect_gate_info_ block below because the row
is now written after the dispatch, which is outside it. Only ever
written when gate_evaluated says the gate actually ran — with the
gate off there is no verdict, and a default-constructed row would
report a knowledge/cost comparison nothing performed.
```

### reconnect-knowledge-value-gate

**Knowledge and value gate scope** — attached to `if (reconnect_gate_info_) {`

```text
§3.6 knowledge + value gate. Strictly downstream of the silence
clock above, so it can only refuse a dispatch that clock already
allowed — the floor is structural, not a second comparison.

Only the mid-run trigger is gated. The exhausted-chase escalation
in pursuitFallback is a fallback WITHIN a manoeuvre already under
way, not a decision to leave exploration, and §3.6's arbitration
table names exactly one mid-run trigger per mode.
```

### reconnect-gate-unbounded-peer-age

**Gate vehicles use unbounded pose age** — attached to `const std::vector<AllocRobot> vehicles =`

```text
Same vehicle set the allocator would solve over, at the pose we
would actually leave from, and the missing list read straight
off it so the two cannot disagree about who is reachable.

UNBOUNDED (0), even when the allocator's TTL is on. The gate is
being asked "is reconnecting with this peer worth abandoning
exploration for?", and the only handle it has on that peer IS
the stale pose — expiring it here would drop the peer to
cell -1, delete it from the missing list built two lines below,
and make the gate refuse to value a reconnection with the one
robot it exists to reconnect to. Staleness is bounded downstream
where it matters: pursuit caps its own chase estimate at 900 s.
```

### reconnect-gate-row-after-dispatch

**Gate row written after the dispatch** — attached to `gate_evaluated = true;`

```text
Emitted on EVERY evaluation, fired or not: a gate is judged by
what it suppressed, and a suppression that logs nothing is
indistinguishable from a trigger that never armed.

BUILT HERE, WRITTEN AFTER THE DISPATCH (2026-09-16). `dispatched`
used to be assigned the gate VERDICT and the row written before
the dispatch was attempted, so the two disagreed on exactly the
population that matters: a gate that said go, an attempt duly
spent, and dispatchReconnect then returning false because a
standing appointment had already claimed this outage's one
chase. Those rows said dispatched=true with no manoeuvre
anywhere in the run, which is the same shape as the checks that
stopped checking — a column reporting its failure case as its
success case. It now reports what actually happened. The flip is
one-directional (only true -> false, only on that population),
so a row that says dispatched on an older generation still means
what it meant; it is the gen-15 rows that gained a distinction.
```

### reconnect-gate-attempts-pre-increment

**attempts_used is read pre-increment** — attached to `ev.attempts_used       = midrun_attempts_;`

```text
Deliberately still PRE-increment: "attempts already used when
this evaluation ran". The increment below happens after this
row's decision, and moving the read past it would silently
shift every value in the column by one against every banked
generation.
```

### reconnect-gate-refusal-no-attempt

**Gate refusal consumes no attempt** — attached to `gate_refuses = true;`

```text
Fall through to ordinary exploration — no attempt consumed
and no cooldown armed, because nothing was dispatched.
Spending budget here would let a stretch of correctly
suppressed evaluations exhaust the manoeuvre the run may
still need later, when the gate does say go.
```

### reconnect-log-team-incomplete-wording

**Team incomplete, not peer silent** — attached to `RCLCPP_INFO(get_logger(),`

```text
"team incomplete", not "peer silent": missing_for measures the
team-presence clock, which lags the peer's record age by
coord_claim_ttl_sec. Through generation 7 this line said "peer
silent", and read against the JSONL's peer_record_age_sec the
two disagreed by 3.24-5.51 s (point estimates 3.74-5.01,
widened by the +/-0.5 s this %.0f costs) with no way to tell
which was the tested one. Naming it here and logging it beside
gate_sec makes the inequality the code evaluated readable off
the line itself.
```

### reconnect-declined-dispatch-bookkeeping

**Closing bookkeeping on a declined dispatch** — attached to `midrun_last_end_    = this->now();`

```text
The attempt is spent anyway and that is deliberate (see the
comment on the increment): for hybrid this IS the one chase
the standing appointment allows, and refunding it would let
the trigger re-fire on the next PLAN tick for the rest of the
outage. Said out loud because the alternative is a spent
attempt with no trace of where it went.

CLOSE THE MID-RUN BOOKKEEPING HERE TOO (2026-09-18). Spending
the attempt is not enough on its own, because nothing else on
this path runs: `reconnect_terminal_ = false` was set above in
anticipation of a manoeuvre, and the two places that undo it
— transitionTo's manoeuvre-end block (gated on
reconnect_active_, which a declined dispatch never sets) and
resumeExploring (only reached from inside PURSUE) — are both
out of reach from here. So the decline used to fall straight
through to ordinary exploration leaving midrun_end_armed_
exactly as it found it, i.e. `cooldown_ok` true on the very
next PLAN tick with missing_for still past the gate. The
trigger then re-fires at 10 Hz and spends the ENTIRE attempt
budget in well under a second, writing one reconnect_gate row
per tick, and the run reads as though it tried six times when
it made one decision six times.

This is the same defect resumeExploring documents at length,
on the one path that does not go through it — and it bites
hardest exactly where the arm needs the budget: for HYBRID,
"a standing appointment already owns this outage" is the
ordinary, correct decline, so the first trigger after arming
burned every remaining attempt.

reconnect_terminal_ is restored to its resting `true` for the
same reason resumeExploring restores it: left false it makes
may_defer true for a LATER terminal dispatch, so the barrier
that is supposed to be allowed to end the run defers instead.
Safe to set here because no manoeuvre started, so no
reconnect_dispatch/reconnect_end row is stamped from this
path and none can be mislabelled by it.
```

### plan-terrain-roi-z-lockstep

**Terrain mode ROI z lock-step** — attached to `if (terrain_relative_z_) {`

```text
Terrain mode: loadLatestMap() may have re-banded the map z-slab around
the robot; keep the FOV ray z-clip in lock-step so rays leaving the
ingested band are clipped, not scored against absent voxels. The candidate
z clamp rides the same band — a candidate above it would put its own FOV
origin outside the ingested volume, where every ray scores the maximal
Beta(1,1) prior.
```

### plan-candidate-generation

**Candidate generation: polar grid and frontiers** — attached to `const MapCache* terrain_map =`

```text
Generate candidates: a polar grid of viewpoints around the robot
(local EIG hops) PLUS frontier centroids anywhere in the ROI (long-
range targets for escaping local IG maxima). Flat mode passes a nullptr
map (3D occupancy check skipped; the 2D planning_map filter below
handles it); terrain mode passes map_cache_ so candidates snap to the
local ground + clearance (and get the 3D occupancy check at that
height). Frontiers are extracted locally from map_cache_ (the fused
grid pulled via the topic), over the effective (robot-relative in
terrain mode) z band.
```

### plan-trajectory-scoring

**Endpoint versus trajectory EIG scoring** — attached to `{`

```text
Evaluate candidates -> populates per-candidate FOV info gain.

trajectory_scoring (param, default false): when true, info_gain is the
sum of EIG scores at poses sampled along the Dijkstra path, not just the
endpoint. Otherwise endpoint scoring. Both run locally via FovEvaluator
on map_cache_ (the fused ROI grid in dscovox mode).
```

### plan-utility-cost-exponent

**Denominator-normalised utility and gamma** — attached to `constexpr float kCostEpsilon = 0.1f;`

```text
SSMI-style denominator-normalised utility (Asgharivaskasi & Atanasov,
TRO 2023), generalised by an exponent γ on the denominator:

  U(c) = info_gain(c) / (ε + path_cost(c))^γ

γ = 1 is the SSMI form: information per unit distance, which at constant
speed is information per SECOND. That is the correct greedy objective when
the metric is time to completion, so γ = 1 is not an arbitrary reference
point and the burden of proof was on moving it.

IT IS NOT WHAT ANY CAMPAIGN RUNS, and this comment called it "the shipped
form ... also the default here" until 2026-09-18. γ = 1.0 survives at
exactly one site, the dp() fallback below. Everywhere the value is actually
configured it is 0.5: shared_params.yaml:693, and run_explo_sim_rviz.sh
passes -p utility_cost_exponent:=$UTIL_GAMMA with UTIL_GAMMA:-0.5 (:1565),
which wins over the yaml. The burden of proof was discharged on 2026-08-19
— see that script's own note that a run reproducing anything older must set
UTIL_GAMMA=1.0 explicitly — by the measurement written out below.

The γ == 1 branch below skips std::pow, so the SSMI reference path stays
bit-identical rather than merely close. That branch is dead in a campaign.

WHY THE KNOB EXISTS. Measured over 703 logged decisions on flatforest_dense
(campaign p14, off arm, to the 0.60 unknown rung): across the candidate set
at a single decision, path_cost spans roughly 5.8x while info_gain spans
only ~0.35 sd/mean. Cost enters linearly and varies far more, so argmax(U)
collapses to argmin(cost) — the planner chose goals at a median 7.8 m when
the mean candidate was 45.2 m away, i.e. it ran as nearest-frontier. The
same collapse is described from the other direction in shared_params.yaml
at candidate_min_goal_dist_m ("6% spread against a ninefold spread in
path_cost"). Solving for the γ at which a mean+2sd-information candidate at
the field's mean distance overtakes the one actually chosen gives a median
of 0.25 (p10 0.15, p90 0.42).

WHAT IT DOES NOT FIX, stated so γ is not mistaken for a repair. Against the
map actually gained afterwards, info_gain has Spearman ρ ≈ +0.18 — real
(the null, raw distance, is ≈ 0) but weak, and its ~1.7x span cannot
separate outcomes that range over 600x. Lowering γ stops a nearly-flat
information term from being overruled by cost; it does not make that term
discriminate. The repair is the information model, not this exponent.

ε (0.1 m) prevents division-by-zero for candidates at the robot's
feet and matches the SSMI reference implementation. It sits INSIDE the
power so the guard survives any γ: at γ = 0 the denominator is exactly 1
and U reduces to pure info_gain with distance ignored.

Unreachable candidates (inf cost) get U = −∞ and sort to the bottom.
```

### sep-discount-anchors

**Team-separation anchors per tick** — attached to `const std::vector<SeparationTerm::Anchor> sep_anchors =`

```text
---- Team-separation discount (separation.hpp) -----------------------

Built once per tick, BEFORE the utility loop, and used for two different
things that must not be confused: `sep_anchors` drives the discount, and
it also drives the sep_* diagnostic columns, which are written in every
arm — including arms where the weight is 0 and no score moves. The
eligibility rules (fresh position, not finished, not self) live in
separationAnchors(); the shape lives in SeparationTerm.

missionElapsed(), not this->now().seconds(): the team model stamps every
peer position on the mission clock, and comparing a mission-elapsed stamp
against a sim-epoch now() would age every position by the epoch and empty
this list on every tick of every run.
```

### sep-discount-applied-to-utility

**Separation discount scales U, not info** — attached to `if (separation_.enabled()) {`

```text
Applied to U, not to info_gain, and that is the whole design. Scaling
the numerator would make the term compete with the information model
— a discounted candidate would look like one the FOV evaluator had
scored lower, and the two would be indistinguishable in
selected_info_gain. Scaling U leaves both components of the pick
reported honestly and puts the separation preference where the reader
can see it, in a column of its own.

Guarded by enabled() so the disabled path does not even multiply by
1.0f: an unreachable candidate carries -inf, and -inf * 1.0f is -inf
in IEEE arithmetic, but "the off configuration executes no arithmetic
at all" is a stronger and cheaper claim than "the arithmetic it
executes happens to be exact".
```

### alloc-focus-rank

**Allocator focus as a sort rank** — attached to `std::vector<int> alloc_rank(candidates.size(), 0);`

```text
---- Global allocator focus (P3) ------------------------------------

Solve the team-wide allocation over the cell world, then rank every
frontier candidate by WHICH tour cell it serves. That rank becomes the
primary sort key below, which is how §3.4's restriction is implemented:
rank 0 is the focus cell's 9-neighbourhood, rank k the k-th tour cell's,
and anything outside every tour cell sorts last. Expressed as an ordering,
that is exactly the specified behaviour — "if the focus neighbourhood
yields no admissible candidate, advance to the next tour cell; if the tour
is empty, fall back to today's unrestricted behaviour" — without a second
and third walk over the candidate list to say it.

Ordering rather than filtering is not a softening of the restriction. The
walk below takes the FIRST admissible candidate in order, so an
out-of-focus goal is still reached only when nothing in the focus
neighbourhood survives the map, reachability, blacklist and MinPos checks.
What the ordering buys is that the allocator cannot starve the planner:
there is no path on which a focus cell gone bad leaves doPlan with no goal
at all, which is the failure mode most of the guards in this function
exist to prevent, and the one an allocator is most likely to introduce.

The 9-neighbourhood, not the cell: frontier candidates are cluster
CENTROIDS, and a cluster straddling a cell boundary parks its centroid in
the neighbour. Filtering to the focus cell alone would starve exactly the
cells whose clusters span it.
Initialised to 0 = "no restriction in force", which is what every
candidate carries when the allocator is off — that is the pre-P3
comparator, since a single rank value makes the first sort key inert.
When the allocator IS on and produced a tour, the loop below overwrites
every entry: frontiers with their tour rank, the polar ring with the
unrestricted band. Nothing keeps the initialiser in that case.
```

### alloc-ring-unrestricted-band

**Polar ring sorts in the unrestricted band** — attached to `const int self_idx = fleet_.self_id;`

```text
Rank the candidates. Local polar-ring candidates stay unrestricted
(§3.4): only frontier candidates are steered, so the ring sorts in the
UNRESTRICTED band alongside the frontiers no tour cell claimed, where
the two compete on utility exactly as they did before P3.

This band used to be 0, on the reading that "unrestricted" meant
"competing on utility with the focus neighbourhood". Rank 0 is not an
exempt band, it is the WINNING one: the comparator is lexicographic on
rank before utility and the walk takes the first admissible candidate,
so a ring point beat every out-of-focus frontier no matter how much
more that frontier would have revealed. The ring is generated around
the robot and is almost always admissible, so in the p4 smoke the pick
was rank 0 on 29 of 29 allocation rows while `reordered` was true on
27 — the tour was solved, logged, and never actually steered anything.
The demotion below made it self-sustaining: it reads `picked_rank == 0`
as "came from the focus", so a ring pick reset the staleness counter
that exists to write off a focus cell nothing can reach, and the focus
wandered the grid a cell per tick.

Sorting the ring last instead would be the other error: it would demote
the ring below every steered frontier and make the allocator able to
starve the local fallback, which is the failure mode the ordering-not-
filtering design exists to prevent.
Copied, not referenced: alloc_robots is index-aligned with the fleet, so
this is a handful of ints, and a dangling reference into a structure the
rest of this function keeps mutating is not worth saving them.
```

### alloc-broadcast-tour-every-solve

**Broadcast tour assigned on every solve** — attached to `my_tour_ = tour;`

```text
Broadcast copy (P6, §3.7). Assigned on EVERY solve including the ones
that produced nothing, so a refused allocation clears it rather than
leaving the last good route on the air: a peer chasing a tour this robot
stopped driving is the one failure the model has no way to detect, since
a stale route and a current one are the same message.
```

### plan-walk-tally-and-amnesty

**Walk tally and the suppressed list** — attached to `struct WalkTally {`

```text
The rejection counters and the suppressed list, bundled into one object so
that the walk below can be run a SECOND time — over the candidate order the
separation term would have produced had it been off — without that second
run's throwaway rejections landing in the numbers the CSV reports. See the
sep_reordered block further down for why the second run has to exist.

`suppressed` holds the candidates whose ONLY disqualification was the
failed-goal blacklist. Retirement makes suppression permanent within a run,
so there has to be a path back: if every candidate is suppressed, the
planner must re-attempt the least-recently-failed one rather than sit in
the "all candidates rejected, retrying next tick" spin, which is already
starvation. Amnesty makes retirement mean "last resort", not "abandoned
while frontiers remain", which bounds the worst case at exactly today's
behaviour.
```

### plan-admissible-filter-chain

**One filter chain for both walks** — attached to `const auto admissible = [&](size_t idx, WalkTally& t) -> bool {`

```text
The candidate filter chain, lifted out of the walk so that the real pick
and the separation counterfactual are decided by the SAME code rather than
by two copies of it that drift apart over time. A manipulation check that
has quietly stopped agreeing with the mechanism it is checking is worse
than no manipulation check, because it still prints a number.

Every filter below is a READ of node state — the map, the cost grid, the
two blacklists, the peer claims. The only writes are to `t`, which is what
makes running the chain a second time safe. Note that nothing here looks at
a candidate's SCORE: admissibility is score-independent, so the two walks
see the identical admissible set and differ only in what order they reach
it in. That is the whole reason the counterfactual is exact.
```

### plan-filter-too-close

**Filter 0: goals too close** — attached to `{`

```text
0. Skip candidates at the robot's feet — these are "already reached"
   by goal_xy_tolerance so they waste a step without any movement —
   and, when candidate_min_goal_dist_m is set, everything inside the
   sensor's useful standoff as well: a goal too close to observe from
   cannot clear the frontier that generated it, which is the whole
   near-frontier oscillation (see the param load).
```

### plan-filter-planning-map-cell

**Filter 1: planning map cell check** — attached to `if (latest_plan_map_) {`

```text
1. Single-cell free check on the planning_map. Frontier centroid
   candidates are exempt from the unknown-cell rejection because
   they naturally sit at the boundary where the 2D planning_map
   cell is still unknown (-1). They are still rejected if the cell
   is occupied/inflated (>= 50). Skipped entirely when no planning_map
   is available (best-effort mode) — there is nothing to check against.
```

### plan-filter-visited-vs-failed

**Filter 3: visited and failed-goal suppression** — attached to `if (visited_goal_radius_m_ > 0.0 &&`

```text
3. Failed-goal blacklist (existing) + recently-visited suppression.
   Both counted as `blk` in the per-step log: they reject for the same
   reason from the planner's point of view -- do not go back there yet.

   BUT THEY ARE RECORDED SEPARATELY, because "the same reason" stopped
   being true at the amnesty below. Only failed-goal rejections were
   pushed onto a suppressed list, so a tick where EVERY candidate sat
   within visited_goal_radius_m (6.0 m) of somewhere reached in the last
   visited_goal_ttl_sec (180 s in every campaign — run_explo_sim_rviz.sh
   passes VISITED_TTL, whose default is 180.0; shared_params.yaml:313
   still reads 120.0 and no campaign uses it) found `suppressed_only`
   empty, skipped the safety valve entirely, and returned starved — at
   10 Hz, up to 1800 consecutive dead ticks waiting for a TTL to expire,
   logged as `blk` next to the mechanism that DOES have a valve. Worse,
   the visited test runs FIRST, so a candidate that is both visited and
   failed never reached the failed branch either: a robot boxed in by
   its own recent trail could not be rescued by the failed-goal amnesty
   on a candidate set that qualified for it twice over.

   So both lists are kept, and the amnesty tries them in order —
   failed-only first, exactly as before, then visited as a last resort.
   That leaves every tick that has a failed-only candidate behaving
   bit-for-bit as it did; the only ticks that change are the ones that
   used to do nothing at all.
```

### plan-filter-minpos-live-claims

**Filter 4: MinPos on live claims** — attached to `if (coord_ && coord_->enabled()) {`

```text
4. MinPos peer-claim check (only when coordination is enabled).
live_after: exploration contests LIVE claims only. Exploit claims are
retained past expiry by the grace window so the vantage contests stay
closed across delivery gaps, but out here a graced claim would keep a
frontier candidate yielded to a peer we have not heard from in 10+
seconds — exploration keeps the original TTL semantics.
```

### plan-amnesty-single-helper

**One amnesty helper for both tiers** — attached to `const auto tryAmnesty = [&](std::vector<size_t>& pool,`

```text
One amnesty, run against whichever suppression list is in play, so the two
tiers cannot drift apart the way the rejection sites did. `bl` is the
blacklist that suppressed these candidates and therefore the one that
orders them and dates them; `source` is the wire/log word for which tier
granted the reprieve.
```

### plan-amnesty-recheck-minpos

**Amnesty re-runs the peer-claim check** — attached to `if (coord_ && coord_->enabled()) {`

```text
Re-run the peer-claim check. In the loop above the blacklist rejection
`continue`s BEFORE MinPos is consulted, so a suppressed candidate has
never been tested against peer claims. Skipping it here would let the
safety valve hand this robot a goal its partner has already claimed —
turning a coordination experiment into a duplicated-effort one at
exactly the moment the planner is least able to notice.
```

### plan-amnesty-tier-order

**Amnesty tier order is the contract** — attached to `tryAmnesty(suppressed_only, failed_goals_, failed_goal_radius_m_, "failed",`

```text
ORDER IS THE BEHAVIOURAL CONTRACT. The failed tier goes first and is
reached on exactly the ticks it was reached on before this split, with the
same pool, the same ordering and the same pick — so no tick that used to
produce a goal produces a different one. The visited tier only runs when
the failed tier found nothing to offer, i.e. on ticks that previously
returned starved and drove the robot nowhere.

WHAT THE VISITED TIER TRADES AWAY, because it is not free. It hands back a
goal within visited_goal_radius_m of somewhere this robot stood in the
last visited_goal_ttl_sec, so a robot boxed in by its own trail now drives
back over covered ground instead of standing still. That converts a
VISIBLE failure into a quieter one: plan_stall_ticks stops climbing (the
column the harness gates watch) and the metres reappear as redundancy,
which is a headline endpoint here. The exchange is still worth making — a
stalled robot re-covers nothing but also explores nothing, and its partner
inherits the whole map — but it must stay countable, which is the entire
reason the goal_amnesty row carries `source` and the step row carries
plan_rej_visited. Any run whose redundancy looks anomalous should be
checked against its `source="visited"` count BEFORE the number is
attributed to an arm.
```

### alloc-bookkeeping-placement

**Allocator events before the starvation return** — attached to `if (alloc_ran) {`

```text
---- Allocator bookkeeping (P3) -------------------------------------

Placed BEFORE the starvation return below, deliberately: a tick on which
every candidate was rejected is the tick a reader most wants the
allocation for, and an event stream that fell silent exactly when planning
failed would make the allocator look innocent by omission.
```

### alloc-focus-staleness-counter

**What the focus staleness counter counts** — attached to `const int focus = alloc_ev.focus_cell;`

```text
Staleness (§3.4). A focus cell that keeps yielding no admissible
candidate is re-measured from the map and, if it still claims to be
worth exploring, demoted to COVERED — mTARE's not-connected -> COVERED
demotion, and the only thing that stops a phantom EXPLORING cell from
being re-assigned forever and from dragging the rendezvous minimax
toward ground nobody can clear.

WHAT THE COUNTER ACTUALLY COUNTS, because it is not "k consecutive
ticks" and said so until 2026-09-18: `alloc_focus_skips_` is indexed BY
CELL ID and is only touched on a tick where that cell is the focus. So
it counts k consecutive APPEARANCES AS FOCUS that failed, not k
consecutive ticks — the focus cell moves as the allocator re-solves, and
a cell's count survives however many ticks pass with some other cell in
the chair. There is no decay. A cell that misses once, drops out of the
focus for two hundred ticks, and comes back to miss again is at 2.

That is deliberately NOT patched into a strict tick run, because the
demotion's real guard is not the counter at all: `shouldDemoteStaleFocus`
fires only if a census taken THIS TICK still reads the cell unexplored.
Ground that got cleared in the gap cannot be written off no matter what
the counter says, so the counter's job is to decide when the question is
worth asking, and a stricter clock would only make it ask less often.

Two ways a tick declines to answer, and they are different:
  - nothing selected at all (`!found`) is global starvation — the map,
    the blacklist or reachability rejected everything everywhere — and
    charging that to the focus cell would demote a cell for being
    unlucky about somebody else's failure. Neither advances nor resets.
  - some other cell was the focus. Not this cell's tick; untouched.
Only a tick that selected a goal and had THIS cell in the chair moves
the number, up on an out-of-focus pick and back to zero on a rank-0 one.
```

### alloc-focus-demotion-one-way

**Stale focus demotion is one-way** — attached to `cell_world_.commitSelf(focus, CellStatus::COVERED);`

```text
A first-hand COVERED, which the merge guard will not let any peer
lower again — so this is a one-way write-off of ground on behalf
of the whole team, and it is logged on every fire for precisely
that reason. What keeps it honest is the three conditions above:
it must be the focus cell, it must have failed k times in a row as
focus (see the counter's note — that is not k consecutive ticks),
and the map must still read it as unexplored after a fresh
measurement taken this tick. The last of those is the one doing
the work.
```

### rejection-profile-placement

**Rejection profile covers starved ticks** — attached to `pending_plan_cand_total_    = static_cast<int>(candidates.size());`

```text
---- Rejection profile of THIS attempt ------------------------------

Written here, above the starvation return, so it covers both outcomes. The
starved branch below returns without ever reaching doLogStep, so anything
recorded after that point is recorded only on ticks that succeeded — which
is precisely how the old columns came to read zero through a 1094 s stall.
```

### plan-starvation-consecutive-count

**Starvation warning carries its own count** — attached to `++consecutive_all_rejected_;`

```text
Throttled, and carrying its own consecutive count because the throttle
alone would destroy the quantity that matters. doPlan runs at 10 Hz and
this branch re-enters PLAN, so sustained candidate starvation — the exact
silent hang the harness gate exists for — emitted ~600 identical lines a
minute and buried every other line in the planner log. Throttling without
the counter would swap that for the opposite error: an hour of starvation
and a run that never picked another goal would look like a dozen isolated
retries. The count is what separates "one unlucky tick" from "this robot
has not been able to plan since t=900".
```

### plan-starvation-blk-vis-reading

**Reading blk and vis on starvation** — attached to `RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,`

```text
blk is the union and vis is its visited half, printed together because
reaching here at all means BOTH amnesty tiers declined — and the two
halves fail for opposite reasons. blk==vis says every candidate was this
robot's own fresh trail and the visited tier still found nothing to
hand back, which can only be MinPos vetoing the whole pool; blk>vis says
real failed goals are in play.
```

### plan-info-gain-std-two-pass

**Two-pass info gain spread** — attached to `{`

```text
Spread of info_gain across the candidate set. Second pass over the vector
already built above — no extra raycast — and deliberately two-pass rather
than the sum-of-squares shortcut: info_gain runs ~2.5e3 with a spread two
orders of magnitude smaller, so E[x^2] - E[x]^2 in float cancels away most
of the answer. Population (not sample) std, over the same denominator as
the mean above, so the two are directly comparable.
```

### sep-manipulation-checks

**Separation manipulation checks** — attached to `pending_sep_eligible_peers_ = static_cast<int>(sep_anchors.size());`

```text
---- Separation manipulation checks ---------------------------------

The first two are measured whether or not the term is on. In an untreated
arm they are the counterfactual: how close to its teammate did the
unmodified planner send this robot, and did it even have a teammate to be
close to. Without them a null result cannot distinguish "dispersion does
not help" from "the robots were never near each other anyway".
```

### sep-reordered-counterfactual-walk

**Did the discount move the pick** — attached to `pending_sep_reordered_ = -1;`

```text
Did the discount change WHERE THE ROBOT WAS SENT? Three states: 1 yes,
0 no, -1 the question was not asked on this tick.

This compares PICKS, by re-sorting on the saved undiscounted utility and
re-walking the same filter chain. An earlier version compared the two
LEADERS instead — the argmax with and without the discount — on the
reasoning that re-running the walk would disturb the run it was measuring.
That reasoning was wrong twice over. The walk's filters are all reads, so
a second run over a throwaway tally disturbs nothing; and the leader
comparison silently under-reports, in the one direction that matters.

The walk takes the first ADMISSIBLE candidate, not the leader, and the
leader is inadmissible often enough that the node carries a whole
starvation branch for it. Take a blacklisted candidate A leading both
orders, with B (close to the teammate, discounted below C) and C behind
it: the term reorders B and C, the walk rejects A and picks C where it
would have picked B, and a leader comparison reports 0. A manipulation
check that reads near-zero across a campaign in which the term was
steering constantly would retire the mechanism for the wrong reason.

The counterfactual is EXACT, not an estimate, because admissibility does
not depend on score (see the filter chain): both walks see the identical
admissible set, so the only thing that can differ is which member of it
comes first. It follows that the counterfactual walk finds a candidate
whenever the real one did, so a -1 here always means the question was not
asked and never means it could not be answered.

Not asked on three kinds of tick: the term is off (with no discount the
two orders are the same by definition, so 0 would be true but vacuous, and
-1 keeps it from being pooled with a measured 0); the walk selected
nothing; or the pick came from the amnesty fallback, whose ordering is by
failure age rather than by utility and which the term therefore cannot
reach.
```

### sep-console-line-throttled

**Throttled separation console line** — attached to `if (separation_.enabled()) {`

```text
Throttled, and only when the term is on: a pilot needs to be able to see
from the console that the treatment is doing something, without waiting
for the CSV. Throttled rather than per-tick because at 10 Hz an always-on
second line would double the planner log for a diagnostic the columns
already carry losslessly.
```

### nav-budget-driven-distance

**Nav budget uses driven distance** — attached to `float dx = current_goal_.position.x() - robot_pos.x();`

```text
Initialise smart-timeout state for this NAVIGATE cycle. Budget the DRIVEN
distance, not the straight line: the selected candidate's Dijkstra path
cost was already computed a few lines above, and a goal 5 m away in a
straight line that needs a 15 m detour around an obstacle used to get a
5 m budget, time out, and be blacklisted for being "unreachable" when it
was merely far. With no cost grid path_cost IS the straight line, so this
is a no-op in the default use_planning_map=false configuration.
```

## ExploPlannerNode::coverageUnknownFraction

### coverage-scovox-z-band

**Scovox coverage source and the z band** — attached to `*source = "scovox";`

```text
scovox source: 2.5D column coverage of the ROI footprint, measured on the
fused 3D map already ingested (ROI + z-band clipped) into map_cache_ by
loadLatestMap() this tick. NB in flat mode the ingest band is the absolute
[roi_min_z, roi_max_z]: on terrain outside that band the cache stays empty
and this reads 1.0 (never done) — set a per-area z band or
terrain_relative_z when the ground leaves the default band.
```

## ExploPlannerNode::doNavigate

### navigate-target-release-stops-hop

**Stopping the hop released for a target** — attached to `abandonNavGoal("target released mid-hop");`

```text
The released hop must be stopped, not just forgotten: EXPLOIT_PLAN can
sit on "no vantage and no reachable approach yet — retrying" for as long
as the give-up timer allows without ever publishing a goal, and the
navigator would drive out the abandoned exploration hop underneath it
(invariant:
see abandonNavGoal).
```

### navigate-goal-inside-obstacle

**Aborting goals that became occupied** — attached to `if (latest_plan_map_ && isCellOccupied(current_goal_.position)) {`

```text
Abort early if the map has updated and the goal cell is now inside
an obstacle (or its inflation zone). This avoids wasting time
navigating toward goals that were valid at selection but became
occupied as the map grew. Skipped when no planning_map is available
(best-effort mode) — isCellOccupied treats "no map" as occupied, which
would otherwise abort every goal immediately.
```

### navigate-vantage-cross-pick-tiebreak

**In-flight vantage cross-pick tiebreak** — attached to `if (phase_ == Phase::EXPLOIT && !current_is_approach_ &&`

```text
In-flight cross-pick tiebreak, exploit VANTAGE hops only. The claim
heartbeat is 1 Hz, so around a target release both robots can select the
SAME ring angle before either has heard the other's claim — the
selection-time yield in doExploitPlan cannot see a claim that has not
arrived yet, and with that yield in place neither would ever release the
angle afterwards. Settle it with the same total order MinPos uses:
selfWinsAgainst is strict closer-distance with a lexicographic robot_id
tiebreak, so on the same pair of inputs exactly one of the two abandons —
never both (the ring silently loses an angle) and never neither (they
re-converge on one point and the proximity guard brakes the loser).
Approach hops carry no ring angle to contest; exploration hops are
deconflicted at selection time by the exploration-scale MinPos disc.
```

### navigate-yielded-vantage-release

**Releasing claim and drive on yield** — attached to `have_active_intent_ = false;`

```text
Release the claim WITH the hop. The winner does not need it — its
symmetric check computes the same total order whether it sees our claim
or none at all — but its dwell-sync barrier reads it: a kept claim is
this yielded angle with staged=false, republished by the heartbeat for
as long as we sit in EXPLOIT_PLAN with nothing else selectable (the
3-vantage / 2-robot final round), so the robot standing on the angle we
yielded would hold its dwell against US until our per-target give-up
fired, ~minutes for a hop we abandoned in one tick. Our next real claim
is whatever the re-plan picks. The DRIVE is stopped as well, and for
the same final round: the re-plan may find nothing selectable at all
(visited + claimed exhaust the ring between two robots), and a planner
retrying in EXPLOIT_PLAN must not still be rolling toward the angle it
just conceded — that heading is a collision course with the winner
standing on it.
```

### navigate-goal-yaw-requirement

**When goal yaw must be reached** — attached to `const bool yaw_required =`

```text
Goal reached on position, and on orientation only where orientation means
something.

The yaw term's whole justification was "so the sensor actually observes the
region the planner scored". That is a statement about the SENSOR MODEL, and
it is now enforced as one: an omnidirectional model scores a full circle
from the goal position, so every heading observes what was scored and
holding the robot still until it reaches a nominal yaw buys nothing. It is
not free, either — the rotate deadline below fails the goal and failGoal()
blacklists the position the robot is standing on, which is how ts1b
produced 984 `budget-rotate` failures on a sensor that does not have a
front.

EXPLOIT vantages are the exception and keep the term unconditionally: a
vantage capture IS directional (the ring angle exists precisely to frame
the trunk), so arriving at the right point facing the wrong way is a real
failure there. The exception is written as a phase test, not as a second
parameter, so it cannot drift away from the model it describes.
```

### exploit-approach-waypoint-rearm

**Approach waypoint re-arms the give-up timer** — attached to `if (exploit_target_timing_)`

```text
Reached an approach waypoint (not a vantage): re-plan from here so
the now-better-mapped surroundings can yield a selectable vantage.
Keep the claim — we are still working this target. Arriving IS
progress on the target, so re-arm the give-up timer exactly like a
completed dwell does — otherwise the drive toward a distant trunk
(~18 m at 0.15 m/s eats a 120 s budget) closes the target PARTIAL
before the ring is ever tried. Termination is preserved: each
approach must land meaningfully NEARER the trunk than the last
(computeApproachGoal returns false otherwise), so the resets are
finite and the hold-and-retry path still runs down the timer.
```

### navigate-visited-goals-break-cycle

**Visited goals break frontier ping-pong** — attached to `visited_goals_.add(current_goal_.position, this->now().seconds());`

```text
Park it in the visited set so the next PLAN does not immediately pick
it again. In a forest the frontier set never drains -- every trunk
casts a permanently unknown shadow -- so "go to the nearest frontier"
has no natural stopping point and the planner ping-pongs between two
neighbouring clusters indefinitely. Recording where it has just BEEN
is what breaks the cycle; recording only where it failed (below) never
could, because these goals are reached successfully every time.
```

### navigate-rotate-own-deadline

**The rotation deadline armed on arrival** — attached to `auto now = this->now();`

```text
At XY, waiting for the controller to finish rotating. This gets its OWN
deadline, armed on arrival. Reusing nav_budget_sec_ was wrong twice
over: that budget covers the DRIVE and is measured from NAVIGATE entry,
so a robot that arrived at 15.9 s against a 16 s budget was failed on
the very next tick with the goal already underfoot — and failGoal()
blacklists current_goal_.position, so the robot then rejected every
candidate within failed_goal_radius_m of where it was standing for the
whole failed_goal_ttl_sec.
```

### navigate-rotate-republish

**Republishing during in-place rotation** — attached to `republishGoal(current_goal_);`

```text
Keep re-publishing so the navigator keeps servicing the yaw — the old
early return skipped this, so a goal dropped by the controller mid-turn
was never re-sent. The no-progress watchdog is deliberately NOT applied
here: rotating in place accumulates no translation, so it would fire on
every correct rotation.
```

## ExploPlannerNode::failGoal

### failgoal-log-tested-pair

**Logging the test that failed a goal** — attached to `RCLCPP_WARN(get_logger(),`

```text
The comparison that fired is logged with the failure, not left at DEBUG
where the campaign never captures it, and it is printed as the TESTED pair
rather than as (elapsed, nav_budget). The old line printed the latter for
all three reasons, which made every budget-rotate row read "failed at 43-58 s
of a 180 s budget" — a claim about ground the robot could not cross — when
the actual test was a 15 s rotation timeout. A "budget" failure at 31 s is a
different animal from one at 179 s (estimator wrong about a short hop vs
genuinely impassable ground), and that distinction only survives if the
threshold printed is the one that was consulted. `pose_stale` marks an
attempt whose progress metric was measured against a pose that stopped
updating — see the pose-health event.
```

## ExploPlannerNode::recordExplorationComplete

### explore-complete-entry-point

**Exploration exhaustion entry point** — attached to `void ExploPlannerNode::recordExplorationComplete(const char* reason) {`

```text
Called at exploration exhaustion (coverage saturated). If rendezvous is on,
an anchor is known, and a teammate is still out of comms, run the
reconnect_mode_ manoeuvre; otherwise finish. When the whole team is
already present the map is already merged, so exhaustion here means the team
is genuinely done — everyone reaches this together and lands in DONE.
```

### explore-complete-event-step-key

**exploration_complete event keyed on step** — attached to `if (exp_complete_step_ != step_) {`

```text
exploration_complete — THIS ROBOT declaring its own exploration exhausted,
recorded here and not at any of the endings below. That is the point: what
happens next is the independent variable (finish / return / chase / hold),
so an event emitted at the ending would measure a different thing in every
arm, while this instant means the same thing in all of them.

Keyed on step_ rather than a plain once-per-run latch: this function is
re-entered on every tick while the reconnect confirmation gate defers
(step_ cannot advance during a deferral, so that is still ONE event), and
again if a manoeuvre delivers a merged map with new frontiers in it and the
robot explores on and re-saturates (which is a genuine second declaration
and gets its own event, with occurrence > 1).
```

### explore-complete-log-hang-gate

**Completion log line disarms the hang gate** — attached to `RCLCPP_INFO(get_logger(),`

```text
Printed from the funnel, not from the endings. The harness hang gate
disarms on "Exploration complete"/"Exploration finished", and through
generation 8 the STEP-BUDGET ending printed neither: it emits only
"Step budget reached", then hands off to startReturnHome, whose homing
leg publishes goals through republishHomeGoal and so advances no step
counter either. The gate integrates 10 heartbeats x 60 sim-s = 600 s of
frozen steps, which is exactly mission_return_max_sec — so a robot
taking its full homing budget after spending its step budget was killed
as HUNG at the very moment it was behaving as designed. That is
differential dropout on the primary endpoint (longest homing legs die
first), and the ending it hits is the one this file calls "the dominant
termination path in dense terrain".

Keyed here rather than patched into the harness pattern list because
this function is the single funnel every ending routes through
(finishOrRendezvous and the latch path are its only callers): a token
list enumerated per-ending is a list that goes stale the next time an
ending is added, silently and in the disarming direction.

Deliberately redundant with the latch path's own line above. A duplicate
log line costs nothing; a missing one costs a cell.
```

## ExploPlannerNode::finishNow

### done-idle-keeps-presence

**Idle finishers keep announcing presence** — attached to `if (done_action_ == "idle") {`

```text
A DONE-idle robot must keep announcing itself: teammates that finish
LATER count peers via claim TTLs, and a silent finisher ages out of every
table within seconds — its teammate would then run the whole reconnect
manoeuvre against a robot that is parked in range, and wait at the
barrier forever. done_action=shutdown robots genuinely disappear; that
combination gets a startup warning (see the param load).
```

## ExploPlannerNode::noteCoverageDecisionSample

### coverage-decision-sample-cache

**Publishing the deciding coverage sample** — attached to `void ExploPlannerNode::noteCoverageDecisionSample(double unk,`

```text
The deciding sample, published into the cache the event log stamps from.

This is load-bearing on the primary endpoint's own record. The completion
hooks arrive with differently-sourced values: the metrics tick passes
last_unknown_fraction_ itself (already refreshed on that tick, so cache and
decision agree by construction), but the doPlan hooks measure fresh and do
NOT touch the cache. recordExplorationComplete then stamps the event from the
cache — so a completion that fired from doPlan wrote the PREVIOUS metrics
sample into the exploration_complete event, up to one metrics period stale
and, since the fraction is falling, systematically HIGHER than the number
that was tested. g6pilot_off_seed104/atlas latched on 0.638 and recorded
0.660, i.e. an endpoint event that reads as if it fired above its own 0.640
criterion. The decision was right in every case; only the record was wrong,
which is worse than harmless because the record is what any analysis reads.

Safe for the other consumer (logRunEnd, which documents the value as "at most
one metrics period old") — this only ever makes it fresher, and
fillCommonMetrics overwrites it on the next row regardless. `source` is a
string literal from coverageUnknownFraction, the same lifetime class the
cache already holds. A negative `unk` means "could not measure" and must not
be published as if it were a coverage reading.
```

## ExploPlannerNode::maybeLatchCoverageDone

### latch-order-record-then-stop

**Latch ordering: record, stop, end** — attached to `recordExplorationComplete("coverage-latched");`

```text
Order matters. Record the declaration against the state we were actually in
(transitionTo would otherwise have already moved us), then stop the
platform, then end. The abandon is what the streak path never needed: it
finished from PLAN, where the robot is stationary and the navigator holds no
goal. This one can fire mid-drive, and the navigator does not know the run is
over — an
uncancelled goal keeps driving a "finished" robot around.
```

### latch-stay-on-agreed-cell

**Latching while standing on the agreed cell** — attached to `if (state_ == State::RETURN_SYNC && appointment_manoeuvre_ &&`

```text
DO NOT WALK AWAY FROM THE MEETING (2026-09-17). Everything below this
block ends the run — parks, coasts, or drives home — and all three are
wrong for a robot that is standing on the agreed cell waiting for the rest
of the team. It has the saturated map the meeting exists to hand over, and
leaving turns a meeting that was going to happen into a no-show.

Gated on appointment_arrived_, not on appointment_manoeuvre_ alone,
because what this block does that the general rule below cannot is stamp
the hold's clock at the latch: this robot is already standing at the agreed
place, so it has no drive left that would have to sit outside the cap. A
manoeuvre with arrived=false — a settle conversion, or a leg the
return-budget and no-progress paths stopped short of the cell — is kept by
keepAppointmentOnFinish instead, and doReturnSync starts its clock on
whichever tick it comes to a stop.

Returning false does NOT un-finish anything. coverage_latched_ is already
true, so the guard at the top of this function makes every re-entry a
no-op, the manifest still reports finished, and exploration_complete was
stamped one line up. The robot is finished; it is just not leaving yet.
doReturnSync owns it from here: it ends the run when the barrier releases
(and the map has settled), or when the latched hold's cap expires.
```

### latch-hold-clock-from-t-meet

**The hold cap starts at the meeting time** — attached to `const double t_now = missionElapsed();`

```text
THE CAP STARTS WHEN THE MEETING IS DUE, NOT WHEN THIS ROBOT GOT HERE
(2026-09-22). appointmentDue() aims the ARRIVAL at t_meet and prices the
departure at t_meet minus appointmentLeadMs, which is the travel estimate
marked up by depart_safety_milli (1.2x). A robot whose estimate holds
therefore arrives EARLY by a fifth of its drive, and an unclamped stamp
spends that margin waiting for a team that is not yet expected.

WHAT THAT BREAKS IS THE CAP'S OWN SIZING. RDV_LATCHED_HOLD is derived in
run_explo_sim_rviz.sh as interval + max_lateness + 60 s of margin, so one
rolled rung is always covered — and that arithmetic measures from t_meet,
because it is comparing against a teammate who rolled to t_meet plus an
interval. Starting early eats the margin directly: on a deadline
departure the lead is at most the whole time to the rung (the robot
cannot leave before now), so the early margin is at most interval/6 =
50 s of the 60 s the derivation has.

THE KEEPER ARRIVES EARLIER THAN THAT, AND WITHOUT A LEAD AT ALL.
keepAppointmentOnFinish departs the moment the map saturates and asks
appointmentDue() nothing — its own log line says so ("this is the
departure the deadline would have triggered later") — so a robot that
finishes early drives straight there and stands for the whole remaining
countdown. Unclamped it would spend the entire cap before the meeting
was due and tear down the hold at t_meet, which is the one outcome the
sizing exists to forbid. That case is the reason this floor is worth
having, not a case it merely tolerates.

THE CLAMP CANNOT UNBOUND THE HOLD, which is the one thing this cap exists
to guarantee, and the argument holds for both departures because it does
not mention either. t_meet is a rung on a fixed lattice of spacing
`interval`, and nextAgreedOccurrence hands back the first rung at or
after now, so t_meet minus arrival is at most one interval whatever
brought the robot here. The total stand is therefore bounded by
interval + cap; a schedule that ROLLS does not extend it, because the
roll clears this stamp and sets the sticky teardown. The same stamp is
made on the other path into this hold; see doReturnSync.
```

### latch-go-to-unreached-meeting

**Finishing drives to the unreached meeting** — attached to `if (keepAppointmentOnFinish("coverage-latched")) return true;`

```text
AND GO TO THE MEETING WHEN IT HAS NOT BEEN REACHED YET (2026-09-21). The
block above is this same rule for the robot already standing on the agreed
cell; this is its general form — "whenever the robots finish they go to the
rendezvous point". What earns the appointment its precedence over the
homing traverse is that it is the only destination the PEER also knows:
home is this robot's own start pose, so superseding a standing agreement to
drive there converts a meeting that was going to happen into a no-show, and
the peer can only discover it by waiting its barrier out. ts4_31 rendezvous
seed 6 is the measured case — bestla latched 157 s before t_meet, closed
cell 27 as superseded, drove home, and atlas kept the appointment and stood
on it for 2275 s to the duration cap.

TRUE, AND THE BLOCK ABOVE RETURNS FALSE, AND THE DIFFERENCE IS THE WHOLE
SAFETY ARGUMENT. What this function returns is read by exactly one caller
that acts on it — doPlan, as `if (maybeLatchCoverageDone(...)) return;` —
so false there means "not finished, carry on planning this tick". The
block above can afford false because it is gated on state_ ==
RETURN_SYNC, which doPlan is never in; it is only ever reached from the
metrics tick, which discards the value. This block has no such gate: it
fires from PLAN, which is where a robot that saturates between two sampler
ticks latches. Returning false here would let doPlan run on past a
startReturnTo that has already published the meeting goal and entered
RETURN_NAV, select a frontier, publish it over that goal, and transition to
NAVIGATE — at which point the outcome classifier sees a departed
appointment whose manoeuvre just ended and closes it `unreachable`. That is
the abandonment this block exists to prevent, wearing a fabricated
navigation failure's name, and with the coverage ending spent
(first-touch-only) the run would then have nothing left to end it but
max_steps_ and a SECOND exploration_complete.
```

### latch-mission-return-preempts

**Mission return pre-empts legacy endings** — attached to `if (mission_return_enabled_ && have_home_) {`

```text
Mission return pre-empts BOTH legacy endings below (park, and the
done_seek coast): the homing traverse is itself the go-reconnect
behaviour the coast approximated, so the coast is superseded rather than
stacked in front of it — running it first would charge a treatment-only
detour to the mission clock. The exploration endpoint is untouched: it
was stamped one line up, before anything about the ending is decided.
```

### latch-done-seek-coast-gate

**When a latched robot coasts on** — attached to `if (done_seek_enabled_ && reconnect_active_) {`

```text
The abandon above this line was correct for a robot that latches while
exploring: the navigator does not know the run is over and an unreplaced goal
would
drive a finished robot around at random. It is exactly WRONG for a robot
that latches while chasing its partner, because there the uncancelled goal
is aimed at the one place we want it to go. Coasting is therefore gated on
reconnect_active_ — the manoeuvre clock — and not on the state name, so it
covers RETURN_NAV, RETURN_SYNC, PURSUE and a PROXIMITY_HOLD taken inside
one, which is the same set transitionTo uses to decide the manoeuvre ended.

Read this next to transitionTo: it runs a few lines later via finishNow,
sees DONE is outside that set, and closes the manoeuvre out (reconnect_end,
reconnect_active_ = false). So the flag must be sampled HERE; by the time
the DONE branch ticks it is already gone.
```

## ExploPlannerNode::keepAppointmentOnFinish

### keep-appointment-defers-homing

**Keeping the appointment defers homing** — attached to `bool ExploPlannerNode::keepAppointmentOnFinish(const char* reason) {`

```text
MISSION RETURN IS DEFERRED BY THIS, NOT SKIPPED. Every ending doReturnSync
can take from the barrier this hands the robot to — the release once the team
has met and the map has settled, and the latched hold's cap when it has not —
routes through startReturnHome itself. So a robot that keeps its appointment
still drives home; it does so after the meeting rather than instead of it.
```

### keep-appointment-manoeuvre-first

**Checking the manoeuvre before the record** — attached to `if (appointment_manoeuvre_) {`

```text
appointment_manoeuvre_ is asked first and separately, because the
appointment RECORD can be closed on the very tick its own drive departs
(see startReturnTo) — so a robot already on the leg can read armed=false
while it is demonstrably keeping one. It needs no dispatch: it is already
going, and re-issuing the leg here would refill the escape ladder it may
have spent getting this far.
```

### keep-appointment-unplaceable-goal

**Declining an unplaceable appointment cell** — attached to `const Eigen::Vector3f goal = appointmentPoint();`

```text
SOLVE THE GOAL BEFORE COMMITTING TO KEEPING IT, because appointmentPoint
has a branch that cannot: with neither grid able to place the agreed cell
it latches appointment_unplaceable_ and returns this robot's own position.
Driving there "arrives" on the next tick, so the keeper would then hold a
barrier at its own feet for the full latched cap over a meeting no travel
of its could ever reach. Declining hands the ending back to the caller and
restores exactly the pre-2026-09-21 behaviour for this one case — home,
outcome `superseded` — which is the honest row when the cell was adopted
off the wire and this robot never froze a snapshot to solve it against.
```

### keep-appointment-bounded-keeper

**Every keeper's wait is bounded** — attached to `reconnect_terminal_   = !coverage_latched_;`

```text
EVERY KEEPER IS BOUNDED, AND THIS LINE IS WHAT MAKES THAT TRUE BY
CONSTRUCTION. reconnect_terminal_ answers one narrow question — may a
failed barrier wait end the run where it stands? — and the barrier's cap
follows from it: terminal takes rendezvous_max_wait_sec, non-terminal on an
appointment takes rendezvous_appointment_wait_sec, which is 0 = UNBOUNDED
and is meant to be.

A coverage-latched keeper is the case that unbounded patience was written
for and the latched hold below is its bound, so it goes non-terminal and
stays reachable right up to that cap. A robot that finished any OTHER way
has no latched hold — coverage_latched_ is the hold's own gate — so an
unbounded wait there would have nothing left to end it, which is the exact
failure this path exists to remove. It takes the terminal cap instead.

Both flags are assigned BEFORE startReturnTo for the reason the
appointment-due departure states: the outcome classifier and the barrier's
wait cap are read inside the transition this call tails into, so assigning
after it labels the manoeuvre in the log as the opposite of what it ran as
— here, specifically, as the `superseded` this whole block exists to stop.
```

## ExploPlannerNode::finishOrRendezvous

### finish-appointment-outranks-homing

**Appointment outranks the homing traverse** — attached to `if (keepAppointmentOnFinish(reason)) return true;`

```text
A STANDING APPOINTMENT OUTRANKS THE HOMING TRAVERSE (2026-09-21), for the
reasons written at the latch's copy of this call. Here rather than below
because the mission-return branch is precisely what it has to pre-empt:
that branch is why the terminal dispatch's own appointment departure
(dispatchReconnect, "exploration is over, go now") is unreachable whenever
homing is enabled, which is every campaign arm.
```

### finish-mission-return-replaces

**Mission return replaces the terminal manoeuvre** — attached to `if (mission_return_enabled_ && have_home_) {`

```text
Mission return replaces the TERMINAL manoeuvre outright (mid-run
manoeuvres, dispatched from doPlan, are untouched — they ARE the
treatment). Both robots' start poses are within comms range of each
other, so driving home is a reconnection manoeuvre with a guaranteed
fixed point; running a chase or barrier first would just add a detour in
front of the same regroup. This also covers the step-budget ending: a
robot that never latched still goes home, so no arm can strand a robot
wherever its budget ran out (the exploration metric filters on the
exploration_complete reason, not on this routing).
```

### finish-reconnect-confirm-gate

**Reconnect confirmation gate** — attached to `if (reconnect_confirm_sec_ > 0.0 && team_seen_complete_) {`

```text
Confirmation gate. `active` above is one read of a claim table that may
not yet have absorbed intents already delivered to this node, so a
manoeuvre committed on it can be released by the very next tick. Require
the team to have been continuously incomplete for reconnect_confirm_sec
first. Returning here leaves the node in PLAN with the coverage streak
already satisfied, so the next tick re-runs this check: the deferral
resolves either into a manoeuvre (peer still gone) or into DONE (peer
was there all along), and cannot loop, because team_last_complete_time_
only advances while the team IS complete — in which case
shouldRendezvous is false and we never reach this branch.

THIS IS THE ONLY `false` finishOrRendezvous RETURNS, and the bound above
is why the caller contract ("ask again next tick") is safe. The dispatch
below used to return one too, with no bound behind it; see the note there
for what that cost.
```

### finish-declined-dispatch-ends-run

**A declined terminal dispatch ends the run** — attached to `RCLCPP_WARN(get_logger(),`

```text
A TERMINAL DISPATCH THAT DECLINES MUST STILL END THE RUN (2026-09-18).
`false` from this function means "deferred, ask again next tick", and the
only caller that acts on it (doLogStep) sends the node back to PLAN.
That contract was written for the confirmation window above, which is
bounded by reconnect_confirm_sec and resolves either way. It is NOT the
contract dispatchReconnect honours: with reconnect_terminal_ set,
may_defer is false by construction, so every `false` it can return here
is the RENDEZVOUS/HYBRID tail declining outright — armAppointment refused
(no agreed triple, or the pair already spent) and the 2026-09-16 removal
left that arm with no unscheduled fallback to drive to.

Propagating that `false` was a livelock, not a deferral. step_ is only
incremented by doLogStep, PLAN's budget test re-fires on the frozen
count, and nothing in the loop can make armAppointment change its mind
while the team is separated (the agreed pair is frozen for the outage) —
so the node spins in PLAN at 10 Hz, after recordExplorationComplete has
already stamped the endpoint and disarmed the harness's stall gate.

Finishing is the right answer, not a patch over one: "no appointment
stands and this arm has nothing unscheduled to fall back on" is exactly
the state the 2026-09-16 change chose to make terminal. The robot has
finished exploring and has nowhere agreed to be; the honest ending is
DONE, recorded against the same reason. The endpoint is unaffected —
exploration_complete was stamped at the top of this function, before any
of this was decided.
```

## ExploPlannerNode::noteMapSize

### dispatch-mode-fallbacks

**Mode dispatch fallbacks and reconnect_terminal_** — attached to `void ExploPlannerNode::noteMapSize(double t_sim_sec, double voxels) {`

```text
Mode dispatch (mesh radios). Pursuit and hybrid try the chase first;
startPursuit declines when the missing peer's record is too stale for
its trail head to mean anything (pursuitBudgetSec == 0), and each mode
then falls through to its fallback. rec can be null even here: the
missing teammate may never have been heard at all (the anchor came from
a DIFFERENT peer) — then there is nothing to chase, and what the robot
does next does not depend on that record at all: rendezvous and hybrid
keep the APPOINTMENT, which was agreed from the team's own positions while
everyone was connected and needs no per-peer contact record, and pursuit
falls through to the explore allowance and then holds in place (below).
This used to read "no pair to midpoint, so hybrid and rendezvous degrade to
the own-anchor return" — both halves of that died with the midpoint drives
on 2026-09-16.

Callers own reconnect_terminal_: finishOrRendezvous sets true (its barrier
may end in DONE), the mid-run trigger in doPlan sets false (its barrier
must resume exploring).
==================================================================
Info-gated mid-run trigger (map-size beacon)
==================================================================
```

### midrun-winsorised-growth-rate

**Winsorised private map growth rate** — attached to `std::vector<double> deltas;`

```text
Winsorised slope: per-interval deltas, each capped at kRateWinsorK x the
window's median delta, summed over the window's total elapsed time. The
cap is what makes this a PRIVATE gathering rate rather than a total one:
a reconnection merges the peer's map into this same cumulative count, a
step of up to 243k voxels in ONE sample (measured, p14), which a plain
two-point slope reports as growth this robot never sensed. Deltas are
floored at 0 for the mirror artifact (a map reload shrinking the count).
```

## ExploPlannerNode::midrunGateSec

### midrun-gate-no-snapshot-fallback

**Gate fallback without a contact snapshot** — attached to `if (est_unshared_out != nullptr) *est_unshared_out = -2.0;`

```text
No contact snapshot to reckon from (anchor predates the record) —
the time-only clock is the only trigger that remains meaningful.
est_unshared = -2, not -1: a gated dispatch that fell back here would
otherwise log gate_sec = the fixed silence clock and est_unshared = -1,
byte-identical to a control run's dispatch — the one situation these
diagnostics exist to tell apart.
```

### midrun-gate-trigger-rate-sum

**Mid-run trigger time from summed rates** — attached to `const double rate_sum =`

```text
The TRIGGER TIME, by contrast, uses only snapshot-frozen quantities so
both robots derive the same value (see LastContact). rate 0 = unknown
(pre-field peer / no samples): the quotient blows past the ceiling and
the clamp turns it into "wait for the backstop", which is the correct
reading of "nothing known to share".

SUM, not mean: once each side's rate is a PRIVATE gathering rate (the
winsorised estimator in noteMapSize — a total-count rate would already
include the peer's contribution and summing it would double-count), the
backlog is the union of two disjoint gatherings and its growth is their
sum. Checked, not assumed: summed = 1.12x p14's measured pair divergence,
where the mean would read 0.56x and fire roughly twice too late.
```

## ExploPlannerNode::rendezvousTeamMutual

### rzv-mutual-every-graph-edge

**Team mutuality checks every graph edge** — attached to `const uint32_t full = fleetMask(team_model_.size());`

```text
EVERY EDGE OF THE GRAPH, not every edge incident on us. Two conditions, and
the second one is the whole reason this function is not a loop over
`direct`.

At N=2 "my links are up" and "the team is complete" are the same sentence,
because there is one link and TeamModel makes `direct` symmetric on it. At
N>=3 they come apart, and the anchor is what falls through the gap. Take
A-B-C and break A-B while A-C and B-C both hold. A sees B non-direct and
freezes; B sees A non-direct and freezes; C sees BOTH of them directly, so
a self-centred test says "complete" and C KEEPS RE-STAMPING for the entire
duration of the split. When C is finally separated at T_C it computes
t_meet = T_C + interval while A and B are holding T_AB + interval, and the
skew T_C - T_AB is bounded by nothing at all: not the TTL, not the
heartbeat, not the mission clock. Against reconnect_midrun_max_wait_sec_
(240 s) any bridging episode longer than that is a guaranteed no-show, so
the pair being exact buys nothing — the two ends keep an exact appointment
at two different times.

The repair needs no new wire field, because the missing information is
already broadcast: TeamWorld.in_range_mask is each robot's own direct-
contact mask, and TeamModel keeps the last one every peer sent. So C can
SEE that A's mask has stopped naming B and freeze on the same event A and
B froze on. The residual is that C learns it one TeamWorld period after A
does, which is the same one-period cut this protocol already declares as
its accepted residual (TeamWorld.msg, "WHAT THIS CANNOT DO") rather than a
new one. What matters is that the bound no longer grows with team size.
```

### rzv-mutual-direct-not-relay

**Own edge must be direct, not relayed** — attached to `if (!team_model_.peer(id).direct) return false;`

```text
1. Our own edge to the peer. `direct` and not `inComms()`: inComms() is
   the transitive closure, which is true through a relay. A relayed link
   carries the gossip fine, but the two ends of it are NOT synchronised
   on when the chain breaks — the middle robot can lose one side seconds
   before the other notices — and that difference lands in the anchor.
```

## ExploPlannerNode::reachablePeerCount

### reachable-count-closure-finished

**What the reachable peer count counts** — attached to `if (team_model_.inComms(id) || team_model_.peer(id).finished) ++n;`

```text
inComms is the closure verdict — direct, or bridged by a peer's
in_range_mask — and `finished` keeps the exemption peerAccounted gives
it. That exemption is this count's ACCEPTED RESIDUAL, not a robot
nobody waits on: a finished robot can still be driving in (see
appointment_inbound's no-finished-exemption note in TeamWorld.msg), and
`finished` relays with no age gate, so on this one channel the door can
admit a peer neither inbound veto sees. Pre-existing in peerAccounted;
generation 27 copies it, it does not widen it.
Deliberately NO claim-table channel: the release ORs this count's
teamComplete with teamSettled, whose accountedPeerCount already carries
the claim table, and this count must stay a statement about the radio
graph rather than about who beaconed recently.
```

## ExploPlannerNode::peerReportsTeamBreak

### team-break-finished-homing-exempt

**Finished and homing peers do not arm** — attached to `if (p.finished ||`

```text
A FINISHED PEER DOES NOT ARM THE TEAM. `finished` already means "no
longer someone worth reconnecting to" (TeamWorld.msg), and the reason is
sharper here than anywhere else: the appointment barrier for a manoeuvre
is UNBOUNDED (rendezvous_appointment_wait_sec defaults to 0 = no cap), so
a parked robot that can never hear one distant peer would hold the whole
team at the meeting point until the run's duration cap and censor the
cell. It also buys nothing — the map still flows OUT of a finished robot
over any link that carries its scovox_bin, and it has stopped needing map
to flow in.

This does NOT reintroduce the generation-18/19 split, because both the
arm and the release read this same function: the predicate is weakened
identically on both sides, which is the property that matters.

A FINISHED PEER IS EXEMPT FROM THE COUNT TOO, AND PERMANENTLY. This read
"a finished peer still counts toward teamComplete, so if it goes silent
the team arms on the ordinary rule" until 2026-09-18, and that inverts
the code: peerAccounted returns true on `p.finished` BEFORE it looks at
any contact, and `p.finished` has no TTL — the gossip relay only ever ORs
it true (team_model.cpp, "PURE OR, NEVER CLEARS") and only a first-hand
observation of the peer un-finishing can clear it. So a finished peer
goes silent, parks, and keeps counting present for the rest of the run.
The team does NOT arm on it, which is the whole point — an ordinary-rule
arm here would hold everyone at the unbounded barrier for a robot that is
never coming, i.e. exactly the censoring this exemption exists to stop.
The exemption is deliberate at BOTH sites; only the sentence describing
it was wrong.

WIDENED TO THE WHOLE OFF-FRONTIER SET IN GENERATION 33 (R4), which is
where the argument above was always pointing: every clause of it turns
on the peer never coming to the meeting and no longer needing map to
flow in, and both hold from the moment it TURNS FOR HOME, not from the
moment it announces the arrival. That window is minutes wide and it is
precisely when this robot is standing at the barrier waiting. The
widening is safe here for the reason the paragraph above gives: this
function is read by the arm and the release alike (:11129 and the
contagion-hold log), so the predicate moves identically on both sides.

NOT COPIED to peerAccounted or reachablePeerCount, deliberately —
teamComplete() is read by the mid-run reconnect trigger and by pursuit's
quarry-heard test, and exempting a homing peer there would stop the gate
from ever PRICING its map, which is the opposite of what R4 wants.
```

## ExploPlannerNode::manoeuvreReleaseEligible

### release-eligible-reachable-door

**Barrier release predicate and its vetoes** — attached to `return appointment_manoeuvre_`

```text
THE APPOINTMENT'S SECOND TERM (2026-09-18). teamSettled is a COMMS test and
the drive ends on ARRIVAL, so on its own it lets a robot standing at the
cell call the meeting over the moment its partner comes into range with
most of the walk still to go. Added HERE and not inside teamSettled because
this function is the barrier's release predicate and teamSettled has other
consumers — the arm, the supersede, the presence dwell — that are asking a
different question and must not acquire a term about who is still driving.

THE REACHABLE DOOR (2026-09-19, generation 27). teamSettled is full mesh
over this robot's own edges plus the contagion, and at the meeting point
that is a GEOMETRY test, not a patience test: under 70 dB trunks one tree
on one chord keeps one pair dark at 8 m forever. The gen-26 N=3 smoke
measured exactly that — pair 0-2 up 1.3% of the window with both other
pairs at 100%, all three robots gathered at the agreed cell and parked in
RETURN_SYNC from ~350 s to the 660 s cap, zero rendezvous_outcome rows,
the cell banked CLEAN. Waiting cannot close such a pair, so the barrier
also releases when every expected peer is REACHABLE — in TeamModel's
comms closure, or finished — i.e. "until all robots are connected" read
as one connected component rather than all pairs. What the dark pair
could not exchange it still has not (there is no map relay); the meeting
delivered every exchange its radio physically allowed, and parking longer
was buying nothing.

The door is OR'd with teamSettled, not a replacement: reachablePeerCount
has no claim-table channel, so the mesh disjunct still carries the
lost-TeamWorld window accountedPeerCount covers. The contagion is
deliberately NOT consulted behind the door — at a gathered barrier the
announcing peers are the ends of the dark pair themselves, and their
announcements ARE the hang; a peer that genuinely drops mid-settle drops
out of the closure and fails the door's teamComplete directly. And the
inbound veto reaches as far as the door admits: reachable includes a
robot still walking in, and leaving before it stands at the cell is the
gen-23 defect again — but a peer admitted through a bridge is one this
robot cannot hear, so its own appointment_inbound never arrives (the bit
is never relayed), and the raw term alone would go blind on exactly the
peers the door newly admits. peerReportsInboundToAppointment closes that
gap with the bridge's one-hop report: the closure expands exactly one hop
through a direct bridge, the bridge hears that hop first-hand, so veto
coverage equals door admission on the CLOSURE channel at every N. A
three-hop straggler is already excluded by the door's own count. The
count's `finished` disjunct is the one admission neither veto covers —
the accepted residual noted in reachablePeerCount — and since 2026-09-23 a
third veto covers it: a finished peer that has not said it is leaving and
cannot be heard is still on its way here (holdingForFinishedPeer).

RELEASING WEAKER THAN THE ARM IS SAFE HERE AND ONLY HERE. The five-site
rule (see the arming site) exists because gen 18/19 ratcheted when the
sites disagreed — but both ratchets ran through the LATCH: the arm can
only re-fire once rendezvous_spent_ clears, and that clear still requires
the strict dwelt teamSettled on the heartbeat. A closure-released team
that is still mesh-broken therefore CANNOT re-arm until a genuine
reunion; it goes back to exploring, which is what the release is for.
```

## ExploPlannerNode::holdingForFinishedPeer

### finished-peer-still-coming-veto

**Waiting for a finished peer to arrive** — attached to `if (!appointment_manoeuvre_) return false;`

```text
A FINISHED ROBOT STILL COMES TO THE MEETING (2026-09-23), so its partner
waits for it until it says it is leaving. Since generation 32 a robot that
finishes with an appointment standing drives to the agreed cell
(keepAppointmentOnFinish), exchanges maps there, and only then turns for
home — and it announces `finished` at the start of that drive, not the
end. peerAccounted counts a finished peer as "will never arrive", so a
partner that heard `finished` once and then lost the radio released this
barrier and left while the finished robot was still on the road; the
finished robot then reached an empty cell and stood out its cap alone.

A VETO HERE, NOT A CHANGE TO peerAccounted. That leaf feeds ~30
teamComplete call sites, two of which (the mid-run reconnect trigger and
pursuit's quarry-heard test) must not move; see DESIGN_gen33.md, the
correction under the finished-consumer table. Only the appointment
barrier asks "is everybody who is coming here?", so only it waits.
```

### finished-peer-wait-bounded

**Bounding the finished-peer wait** — attached to `if (finished_peer_wait_start_sec_ >= 0.0) {`

```text
BOUNDED, because the radio may never deliver the "leaving": at N=2 there
is no relay, and a finished peer that gave up on the cell and turned for
home out of range would otherwise hold this robot for the rest of the run.
An unstamped wait (a manoeuvre that never reached the barrier, asked by the
classifier) is open: it has not started, so it has not run out.
```

## ExploPlannerNode::refreshRendezvousSnapshot

### rzv-snapshot-refresh-period

**Snapshot refresh once per proposal period** — attached to `const double t_snap = missionElapsed();`

```text
ONE REFRESH PER PROPOSAL PERIOD, not one per heartbeat. The copy below is
a CellWorld — a cell grid AND the mutable all-pairs distance matrix it
memoises — so at coord_heartbeat_hz=1 this was a full matrix copy every
second for the whole run, on the single-threaded executor, to feed an
argmin that only runs every rendezvous_proposal_period_sec_.

The period is shared with the derive deliberately. The two clocks are
phased independently, so the bound is one period, not one heartbeat — the
argmin can read a snapshot up to rendezvous_proposal_period_sec_ old. That
is the same bound the proposal itself already carries, so sharing the
number means there is no second staleness to reason about, and 30 s of
drift in a meeting cell is well inside what the wait absorbs.
Followers refresh on the same clock even though
they never solve, because appointmentTravelMs and appointmentPoint read
this world during the outage and a follower that never refreshed would
have no grid to cost its drive against.
```

### rzv-snapshot-ttl-zero

**Unbounded TTL for snapshot vehicles** — attached to `rendezvous_vehicles_ = allocVehicles(latest_pos_.x(), latest_pos_.y(),`

```text
UNBOUNDED (0), and here the TTL could not bite even if it were passed: the
snapshot is refreshed ONLY while the whole team is in comms (see the
caller's guard and rendezvous_world_ above), so every peer position in it
is seconds old by construction. Passing 0 says that in the signature
instead of relying on the guard staying where it is.
```

### rzv-snapshot-force-flags-off

**Forcing finished and off_frontier off** — attached to `for (AllocRobot& v : rendezvous_vehicles_) {`

```text
`finished` is FORCED OFF for everyone in the snapshot, and that is a
correctness fix rather than a simplification. allocVehicles writes
`finished = false` for self ("we are planning, so we are not done") and
`finished = p.finished` for peers, which is right for the live solve and
fatal here: the scheduler DROPS finished vehicles, so a robot that
saturated coverage while still in contact is present in its own snapshot
and absent from its partner's. The two then solve a two-vehicle and a
one-vehicle allocation over the same world, get different tours, different
makespans and a different argmin — different meeting cells, no meeting.
Forcing the flag off on both sides is the only value both robots can agree
on without exchanging it, and it costs nothing: an appointment is a place
to stand, not a work assignment, so including a robot that has stopped
exploring is exactly what we want.

`off_frontier` IS FORCED OFF WITH IT, and it has to be: it is the wider
test, the scheduler applies the same filter, and it is if anything MORE
asymmetric than `finished` — a robot's own homing latch flips the instant
it turns for home, while its partner learns the level one TeamWorld later
or, out of contact, not at all. Leaving it live would put a homing robot
in one snapshot and not the other and reintroduce the disagreement this
whole block exists to remove.
```

## ExploPlannerNode::deriveRendezvousProposal

### rzv-derive-no-exclusions

**No exclusions in the shared derive** — attached to `cfg.exclude.clear();`

```text
NO EXCLUSIONS. The no-show write-off was a per-robot list, and a per-robot
input to a value the whole team has to share is exactly what gen 9 got
wrong. It cannot come back as a shared input either: the team would have
to agree on the write-offs first, which is the same agreement problem one
level down.
```

### rzv-derive-interval-cap

**The interval cap is the appointment wait** — attached to `cfg.max_interval_ms =`

```text
THE DIVERGENCE CAP IS GONE, and that part is structural rather than a
choice: it was fed by min_share / (my_rate + peer_rate) from the
LAST-CONTACT record, a quantity that by definition does not exist yet at
proposal time, because the proposal is derived while everyone is still
connected. The pairwise rate sum does not survive N>2 either.

THE SAME FIELD NOW CARRIES THE FINDABILITY BOUND (2026-09-17). The
inequality survives generation 19 but its SUBJECT CHANGED, and the change
is easy to miss because the arithmetic is untouched. It used to read
`recurrence period <= barrier wait`: a robot arming too late for occurrence
k drove to k+1 and was at most one period behind peers still standing
there. Under generation 20's countdown there were no occurrences at all,
and the same comparison asked `the furthest robot's drive <= barrier wait`
— because after the reachability floor that is what `interval_ms` IS — i.e.
can the last robot to set off get there before the ones already waiting
give up.

GENERATION 23 PUTS THE OCCURRENCES BACK and the ORIGINAL reading is the
live one again: arming keeps the first agreed occurrence this robot can
still reach (nextAgreedOccurrence), so a robot that arms only after
occurrence k has passed — or too close to it to arrive — really does drive
to k+1, exactly as the pre-19 sentence describes. Both readings want the same
number, which is why the arithmetic never moved. Feeding the barrier's own
parameter in keeps the two sides of it one constant rather than two that
drift apart in a yaml.

A wait cap of 0 means "wait forever" at the barrier and maps to an uncapped
interval here, under either reading: if nobody ever leaves, every arrival
is in time. So the sentinel needs no special case.

WHAT THE CAP DOES: it shapes the agreed integer and sets `capped`. Under
generation 23 the interval is load-bearing again — it is the spacing
between occurrences, so it decides how long a robot that misses one waits
for the next, and it caps hybrid's chase window — which since generation
29 ends not at the next agreed occurrence but at the moment this robot
must LEAVE for it, i.e. one marked-up drive earlier. The window is
therefore `interval` minus the robot's lead plus whatever the rung roll
gives back, and it can be negative: a robot that has to depart the instant
it arms does not chase at all. Sizing this cap is sizing that window.
A capped `interval_ms` still understates the furthest drive, so
read `tour_interval_ms`, which is logged uncapped beside it, before drawing
any distance conclusion from the interval.

IT IS THE APPOINTMENT BARRIER'S WAIT, NOT THE MID-RUN RECONNECT WAIT
(2026-09-19). This read reconnect_midrun_max_wait_sec until generation 29,
and the note here used to warn against merging the two without re-deriving
which one the cap belongs to. Re-derived: the robots this bound is about
are the ones ALREADY STANDING at the agreed cell when a straggler arrives
at the next occurrence, and doReturnSync spends
rendezvous_appointment_wait_sec on them — the mid-run wait is what a
PURSUIT reunion spends, and no pursuit ever consults this schedule. Feeding
the wrong barrier in was survivable while it was the larger of the two; it
stops being survivable at a 300 s timetable, where a 240 s cap can no
longer cut the interval (the reachability floor outranks it) and so does
nothing but raise the broken-inequality WARN below on every single derive,
about a barrier that does not give up.

SO THE CAP IS INERT WHENEVER THE APPOINTMENT WAIT IS UNBOUNDED, `capped`
with it, and that is said out loud at startup rather than left for an
analyst to discover as a column of zeroes — see the parameter's own
validation. Inert is the correct state here: a barrier nobody walks away
from cannot be outrun, so there is no interval long enough to break the
inequality. Configure a finite rendezvous_appointment_wait_sec and both the
cap and the column come back to life.
```

### rzv-derive-centroid-floor

**Team centroid as the floor cell** — attached to `int floor_cell = -1;`

```text
THE FLOOR IS THE TEAM'S OWN CENTROID, and it is what makes "always agree a
place beforehand" true rather than aspirational.

The floor used to be the midpoint of the LAST-CONTACT pose pair, and that
quantity does not exist at proposal time — the proposal is derived while
everyone is still connected, so there is no last contact to take a midpoint
of. Passing -1 instead was the obvious reading, and it cost the ts4 smoke an
entire arm: the only refusal the N=4 rung logged was "the tours are empty
and the last-contact midpoint is outside the ROI". The team was mutually
whole for about two seconds, the allocator had not produced a tour yet, the
candidate set was therefore EMPTY, and the run continued for another 578 s
with no appointment at all.

The centroid of the snapshot's own vehicle cells is the same KIND of
quantity the midpoint was — a place defined by where the team is, not by
where the map says it should go — and unlike the midpoint it exists exactly
when it is needed: at proposal time, while everyone is in contact and their
positions are seconds old. It generalises to any N (the midpoint did not),
it comes out of the frozen shared snapshot rather than anything private,
and a rectangular ROI is convex so a centroid of in-ROI points is in-ROI.

What it buys is that the candidate set is NEVER empty while a locatable
robot exists, so the refusal above cannot recur. What it costs is that the
team may agree to meet somewhere no tour goes — which is precisely the
trade a fixed rendezvous makes, and `floor_won` records every time it did.
```

### rzv-derive-timetable-floor

**The timetable floor on rung spacing** — attached to `cfg.min_interval_ms =`

```text
THE TIMETABLE FLOOR. The scheduler already floors the interval at the
furthest robot's DIRECT drive, which is the physics; this second floor is
the policy — how far apart we want the rungs to be irrespective of how
close together the team happens to be standing.

IT BOUNDS A SCHEDULE AGAIN. Generation 19 removed the recurrence and left
this bounding nothing but a logged integer; generation 23 put the
occurrences back, so the number is once more the gap a robot that misses
one waits for the next, and generation 29 makes it the size of hybrid's
chase window too. See rendezvous_interval_sec_ for what that costs at 30 s.

Read here rather than baked into rzv_cfg_ at construction because it is a
parameter a campaign varies per arm, and every robot reads the same
fleet-wide value — which is only a tidiness argument now, since the
proposer is the only node that solves and the other two integers are
adopted verbatim regardless.
```

### rzv-derive-real-mission-clock

**Solving on the real mission clock** — attached to `rendezvous_held_provenance_.plan = RendezvousScheduler::solve(`

```text
THE REAL MISSION CLOCK, not the 0 this used to pass. The plan's t_meet is
now an absolute mission-elapsed instant that goes on the wire and is
adopted verbatim, so the origin has to be the frame every robot in the
fleet evaluates in. Passing 0 here was what made the appointment
origin-free and forced each robot to supply its own origin at arming time
— the defect this replaces.
```

### rzv-derive-findability-warn

**The findability inequality warning** — attached to `if (cfg.max_interval_ms > 0 &&`

```text
THE FINDABILITY INEQUALITY, CHECKED RATHER THAN ASSUMED. Reaching here
means the agreed interval came out longer than the barrier is willing to
stand there: the early arrivals wait out the cap and leave while the last
robot is still en route, and the log records a no-show for a meeting
nobody was late to.

KEYED ON THE INEQUALITY, NOT ON THE FLAGS, and that is not a stylistic
choice. `capped` asks whether the cap cut the TOUR term, so a cap that
sits above the tours and below the lattice floor overrules the barrier
while reporting nothing — since generation 29 that is the common case,
because the floor is a 300 s policy number rather than a drive. Reading
`capped` here would have made this branch silent exactly where it matters.
The two flags are printed below so the line says WHICH term won.

IT IS THE APPOINTMENT BARRIER (2026-09-19), which inverts the note that
stood here. The cap was derived from `reconnect_midrun_max_wait_sec` and
this warn was therefore a statement about mid-run reconnect attempts rather
than about the scheduled meeting — a warn about the wrong barrier, fired
from the site that shapes the right one. It now reads
`rendezvous_appointment_wait_sec`, so the sentence it prints is about the
robots it names, and the campaign's 0 makes the whole branch unreachable
rather than chronically true: there is no interval long enough to outrun a
barrier that never ends. Configure a finite appointment wait and this comes
back.

It is still the honest outcome, because the alternative is worse — cutting
the interval below the drive would only make the number smaller, not the
robot faster — but it is not one to discover in a post-hoc analysis, so it
is said once, here, where the numbers are.
```

## ExploPlannerNode::maintainRendezvousProposal

### rzv-proposal-snapshot-gate-derive

**Snapshot gate applies only to derive** — attached to `const double t = missionElapsed();`

```text
THE SNAPSHOT GATE MOVED DOWN, ONTO THE DERIVE (2026-09-17). It used to sit
here and stop the whole function, which quietly re-imposed the mutual-
contact gate on the ECHO path that the declaration above says it must not
take: have_rendezvous_snapshot_ is only ever set inside the rzv_mutual guard
on the heartbeat, so a follower whose own heartbeat never landed inside the
team's mutual window could not adopt, could not echo, and therefore starved
EVERY robot's commit — the whole-fleet failure, at the same choke point this
change exists to widen. At N=4 that window was about two seconds against a
1 Hz heartbeat with documented overruns, so one late tick was enough.

AND THAT WINDOW IS NOT RADIO-LIMITED, which is the part worth acting on.
Measured on the banked ts4 N=4 rendezvous cell's link_states.csv, all six
pairs are simultaneously connected from t_sim 5.0 s to 46.4 s — a 41.4 s
full mesh — and then never again for the remaining ~600 s of the run. The
node's own rendezvousTeamMutual() recognised about two seconds of those
41.4. The two numbers are different quantities and do not contradict each
other (one is the emulator's link truth, the other is received-intent
freshness sampled on the heartbeat), but the GAP between them is the whole
diagnostic: at N=4 the derive is starved by how the node reads contact, not
by how long the team has it. Relaxing the gate to spanning connectivity
does NOT recover it — the same file gives only 10.2% spanning against 6.4%
full mesh, and after t=46.4 s just 4% of the run — so the lever is intent
freshness inside the one early window, not a weaker topology test.

Only deriveRendezvousProposal reads the snapshot; adopting, echoing and
committing are arithmetic on three integers that arrived over the wire. So
the gate belongs on the one branch that needs it, where a proposer without a
snapshot simply does not derive this tick and still commits normally.
```

### rzv-proposal-fourth-vector-guard

**Guarding all four parallel vectors** — attached to `if (static_cast<int>(rendezvous_peer_provisional_.size()) != fleet_.size()) return;`

```text
THE FOURTH PARALLEL VECTOR, ADDED 2026-09-18. The three above were checked
and this one was not, while the function indexes it unconditionally
(rendezvous_peer_provisional_[kRendezvousProposerId] below, and the write
in the commit path). It is unreachable today only because all four are
resized together in one block, which is a property of the CALLER — the
guard existed to stop depending on that, and by omitting one vector it was
reporting an invariant it had not checked. A guard that covers three of
four is worse than no guard: it reads as proof.
```

### rzv-proposal-single-robot-guard

**Refusing a one-robot fleet commit** — attached to `if (fleet_.size() < 2) return;`

```text
A one-robot fleet would pass the commit loop VACUOUSLY — no peers to be
stale, no peers to disagree, on_pair == 0 == fleet_.size()-1 — and commit a
pair nobody else is holding, logging peers_on_pair = 0 on an armed row. It
is a nonsense config (there is no one to meet) rather than a reachable one,
but the commit rule is the single thing standing between this robot and a
private appointment, so it does not get to pass by having no work to do.
```

### rzv-derive-gate-one-pair-per-meeting

**Why the proposer gates sit inside the branch** — attached to `if (proposer && team_mutual && have_rendezvous_snapshot_ &&`

```text
WHY BOTH PROPOSER GATES LIVE INSIDE THE BRANCH BELOW, AND NOT OUT HERE.

They used to be two early returns at this point in the function, above the
latch and the commit. The commit rule then described itself as symmetric —
"one rule, so no robot can reach a conclusion the others cannot" — while
being nothing of the kind: on a FOLLOWER the latch and the commit ran every
tick, and on the PROPOSER they were skipped entirely whenever the team was
not mutually whole, and skipped FOREVER once an agreement existed. Two
effects, both bad. The proposer could not latch an echo that arrived a tick
after the team went incomplete — exactly the tick its followers were
echoing on — so the robot that chose the pair was the last to be able to
conclude the team held it. And with the second gate above the latch, the
proposer's latch array froze at the instant of agreement, so any later
recount ran on stale entries.

Moving them in makes the sentence true rather than aspirational: everything
from the latch down is now reached by every robot on every tick, and the
ONLY thing the proposer does that a follower does not is derive.

DERIVING is the step that must predate the separation, so it keeps the
whole-team gate. The snapshot it solves over is only refreshed under the
same condition, so a derive without it would re-solve a frozen world and
return the same pair anyway — but it would also let the proposer issue a
NEW generation mid-outage, which is the one thing that could split the
fleet across two pairs. Refused structurally rather than relied upon.

ONE PAIR PER MEETING (2026-09-19). Once a place and a time exist, the
proposer stops deriving and that pair stands until the team has actually
KEPT it: the barrier gathers everyone, the settle exchanges maps, and the
release raises `rendezvous_reagree_due_`, which is the only other thing
that re-opens this branch. It used to stand for the whole mission, which
meant the meeting place was chosen from the thinnest tours the run would
ever have and was never revisited no matter how much map arrived after it.

Expressed as `!rendezvous_held_.valid()` — the condition the invariant
actually needs — and not as the old `!rendezvous_agreed_.valid()`. The two
differ in the window between holding a pair and the team confirming it,
and the old form let the proposer re-derive inside that window, which is
the one place a second generation could still be born. It also retires a
dead term: the derive used to be gated on `due && (none_yet || confirmed)`
where `confirmed` meant `rendezvous_agreed_ == rendezvous_held_`, and with
the agreed-gate above it that disjunct was unreachable — it could only be
true when rendezvous_agreed_ was valid, and a valid rendezvous_agreed_ had
already returned. A condition that cannot change an outcome reads like a
second chance to derive and is not one.

This single condition is also what the confirmation latch's soundness rests
on. The latch records "peer p was HEARD holding this triple" and is never
aged out, so it is only safe while a peer's published triple cannot walk
somewhere a standing latch would misrepresent.

THE PERMITTED TRANSITIONS ARE empty -> P -> R -> R' -> ..., with P
provisional and each R final for the one meeting it schedules. No triple
here is terminal any more, so the latch cannot rely on being uncontradicted
and does not: it clears explicitly on first-hand evidence of a peer holding
something else, and the comparison that counts agreement is against
`rendezvous_held_` rather than against any remembered generation, so the
instant this robot moves to R' every latch still naming R stops matching —
for free, on the next tick, with nothing to age out.

WHAT THAT COSTS, stated plainly: a follower that misses R' keeps R, and if
it never hears the proposer again the fleet keeps two appointments. That is
the same exposure the upgrade exception already carries, contained the same
way (WHAT ACTUALLY CONTAINS IT, below). The difference is in the evidence
each one is authored on. The upgrade fires on `team_mutual`, a claim about
the last few seconds that has measurably been wrong — see the 0.6 s case
below. A re-agreement fires only where the barrier has just released,
which means every expected peer was present and reachable at that instant:
the strongest evidence of a whole fleet this node ever holds.

It used to re-derive every rendezvous_proposal_period_sec_, and every
re-derive opened a commit race: the proposer publishes generation N+1, the
followers adopt and commit it one TeamWorld period later, and the proposer
cannot commit until it sees those echoes a further period after that. For
the round trip in between, the fleet genuinely holds two different pairs.
That was harmless while arming waited for the silence gate — the race was
long over by the time anyone looked — but arming now happens at the
separation itself, so a separation landing inside that window arms a
SPLIT fleet. Measured on a 2-robot cell: three outages, two agreed exactly
(cells 57/57 and 27/27, anchors 0.5 s apart), and the third had one robot
on cell 56 at t+28 s and the other on cell 57 at t+88 s. It waited at the
agreed place; its partner waited at a different agreed place; the run
recorded a no-show against an appointment both had kept faithfully.

Deriving only at the instants above removes that failure mode rather than
shrinking its window. A new generation can be born in exactly two places:
before the fleet has ever separated, and at a barrier that does not release
until every expected peer is present. Neither is a separation, so there is
nothing for a generation to be split across. What remains is the anchor
skew, which moves t_meet but never the CELL, so the worst case degrades
from "two robots wait in two places" to "two robots wait in the same place
a few seconds apart", and the barrier wait absorbs that.

The FIRST pair is still chosen early, from thin tours, and it is still the
literal reading of a fixed rendezvous: the place and the time are agreed
BEFOREHAND, while the team is together, and are never renegotiated by
robots that can no longer talk to each other. What changed is only that
"together" now also means the team standing at an appointment it kept, and
not just the team standing on the start line.

THE OTHER EXCEPTION (2026-09-17) is `rendezvous_held_provisional_`, and every
word above still applies to it: see that member for why a pair derived
before any tour existed is not the pair this invariant is protecting, and
for the exact bound on how often it can be replaced (once, only by a pair
that had a choice to make, only while the team is mutually whole).

CONFINING THAT EXCEPTION TO THE PRE-MISSION HOLD WAS TRIED ON 2026-09-17
AND WITHDRAWN THE SAME DAY. The conjunct was `&& missionElapsed() <
mission_start_hold_sec_` on the branch below. Read the next forty lines
before re-adding it, because it is the obvious fix and it is wrong.

THE DEFECT IT WAS AIMED AT IS REAL. The bound above — once, and only while
mutually whole — is not enough, because `team_mutual` is a claim about the
last few seconds rather than about the next few. In ts4 smoke20's N=3
hybrid cell the proposer authored the upgrade 0.6 s after its last peer had
gone silent: mutual was still reading true off a tolerance window that had
not expired yet. The peer never received the upgrade, armed on the
placeholder it had committed 26 s earlier, and the fleet kept two exact
appointments in two different cells for the remaining 400 s of the run.

WHY THE HOLD CANNOT BE THE CURE. The upgrade fires when the argmin has more
than one admissible cell to choose between, and the candidate pool is the
union of GlobalAllocator's tours, which are built only over cells in
EXPLORING/EXPLORING_BY_OTHERS. A robot parked in WAIT_FOR_MAP completes
zero steps and therefore grows no such cells. Measured over every banked
cell that carries agreed_provisional (5 campaigns, 10 cells, 28 robot-runs,
58 rendezvous_agreed rows):

  first NON-provisional commit, step counter at that commit
    min 2, and 2 only at N=3; the distribution is {2,3,6,7,8}
    no upgrade anywhere in the corpus was ever authored at step 0 or 1

and on EVERY provisional commit the proposer's own provenance reads
`candidates=1, rejected_unreachable=0, rejected_excluded=0` — the pool was
EMPTY, not filtered. Nothing was rejected because nothing was offered. So
the failure during a hold is not that co-located robots cannot reach the
candidates; it is that there are no candidates to reach. Co-location, which
is the one thing the hold manufactures, is not the binding constraint.

A hold therefore does not DELAY the upgrade, it DELETES it: the branch
below is the only site that ever clears rendezvous_held_provisional_, so a
window that closes before the first non-provisional derive leaves the pair
provisional for the rest of the run. Both scheduled arms would then measure
"meet at the team's initial centroid" while passing every unanimity check,
because a fleet that unanimously agrees a placeholder is still unanimous.
Two banked upgrades that the confinement would have rejected outright:
smoke18_n2 hybrid at t=311.1 s and smoke20_n3 rendezvous at t=353.6 s, both
more than 250 s after the provisional they replaced.

WHY NOT TIGHTEN THE LIVENESS TEST INSTEAD. Because it only shrinks the
window. Any tolerance leaves a last instant at which the proposer believes
the team is whole and it is not, and an upgrade authored in that instant
splits the fleet.

WHY NOT REQUIRE AN ECHO ROUND FOR THE UPGRADE, as the provisional pair
requires. Also tried on 2026-09-17, also withdrawn: at N>=3 each FOLLOWER
then waits on the OTHER FOLLOWERS' echoes, over follower-to-follower links
this radio regime need not provide, and the measured result was a proposer
that committed while its followers did not — the same split, with the
proposer on the far side of it.

WHAT ACTUALLY CONTAINS IT, and it is containment rather than a fix: the
proposer keeps re-deriving for as long as the pair is provisional, and
publishTeamWorld re-broadcasts the held pair every cycle to whoever can
hear it. So a follower that misses the upgrade adopts it the next time the
link comes back, and the disagreement is transient rather than permanent
unless the two never speak again. That is the honest statement of this
mechanism's guarantee. Committing one value atomically across a fleet that
can partition mid-round is Two Generals and has no solution; what is
achievable is eventual convergence under re-broadcast, and that is what
this is. The rendezvous_agreed comms gate still reports any split that
outlives the run.
```

### rzv-derive-attempt-pacing

**Pacing derive attempts at bootstrap** — attached to `const double derive_period =`

```text
THE PERIOD GATES THE ATTEMPT, NOT THE SUCCESS, and that distinction is
the whole content of this block now that the branch condition carries
one-pair-per-meeting. Almost every tick that reaches here has no usable
pair at all, so without pacing a run with nothing derivable yet — the
normal state until the allocator has tours — would run a full
GlobalAllocator::solve on every single heartbeat, for as long as that
lasted, on the executor whose starvation this file spends a hundred lines
accounting for. The first attempt is still immediate, because
rendezvous_held_at_sec_ is negative until one has been made.

BOOTSTRAP FASTER THAN STEADY STATE. The team is co-located and in full
mutual contact for the first seconds of every run, and that is the one
window in which agreement is free — the whole point of agreeing a place
and a time BEFOREHAND. But the first derive attempt almost always
refuses, because GlobalAllocator has no tours to insert a meeting cell
into yet, and a refusal stamps the period clock just as a success does.
At the steady-state period that pushes the first real attempt a full
rendezvous_proposal_period_sec_ out, by which time an N>=3 fleet has
dispersed and may not be mutually whole again for the rest of the run.

So: retry on a short period until a pair actually exists. Bounded on both
ends — it is never per-heartbeat (the executor-starvation bug this pacing
exists to prevent), and the branch condition stops it dead the moment a
pair exists. There is still no steady-state branch: between meetings the
proposer never returns here, so rendezvous_proposal_period_sec_ only
serves as a ceiling on the retry for anyone who configures it below the
default. A re-agreement re-enters on the same short period, which is what
it wants — the team is gathered and mutually whole, and every second
spent retrying is a second the barrier holds everyone standing still.
```

### rzv-derive-saves-prev-provenance

**Saving provenance before the derive clobbers it** — attached to `const RendezvousProvenance prev_prov = rendezvous_held_provenance_;`

```text
SAVED BECAUSE THE DERIVE CLOBBERS IT. deriveRendezvousProposal resets
rendezvous_held_provenance_ before it solves, so an attempt this block
declines to adopt would leave the provenance describing a search whose
answer nobody is holding — the "refused re-derive" failure mode
documented on RendezvousProvenance, reached by the other door.
```

### rzv-proposer-adopt-rule

**When the proposer adopts a derived pair** — attached to `const bool adopt = !rendezvous_held_.valid() ||`

```text
ONE PAIR PER MEETING: adopt when nothing is held, when a provisional
pair can be upgraded to a final one, or when the team has just kept
the standing pair and is owed the next one. All three replacements
carry `!now_provisional` because the centroid fallback is what the
argmin returns when it had nothing to choose between, so adopting it
over a tour-informed pair would trade a decision for a placeholder —
and for a re-agreement that guard is what stops a barrier release
that happens to catch the allocator empty from throwing away a good
pair and sending the team back out aimed at its own centroid.
```

### rzv-reagree-due-consumed-by-adoption

**Re-agree request cleared only on adoption** — attached to `rendezvous_reagree_due_      = false;`

```text
CONSUMED BY THE ADOPTION, NOT BY THE ATTEMPT. A release that lands
on a momentarily empty allocator returns a provisional pair, which
the guard above correctly declines — and if that also cleared the
request, the team would go back out on the pair it had just kept
and the meeting the user asked to be re-sited would not be. Left
set, the 5 s retry above simply asks again, still while everyone is
standing at the agreed cell.

IT CAN THEREFORE OUTLIVE THE GATHERING, if the links never firm up
into rendezvousTeamMutual before the team disperses, and a later
mutual moment would then author a pair on the weaker evidence the
split-fleet note above is about. That is contained where every
other new generation is contained, and by the same argument: what
the robots DRIVE is rendezvous_agreed_, the commit rule needs every
peer's echo before anything becomes agreed, so a pair authored into
a fleet that has already come apart commits nowhere and the team
keeps the pair it has.
```

### rzv-keep-first-centroid-fallback

**Keeping the first centroid fallback** — attached to `rendezvous_held_provenance_ = prev_prov;`

```text
Holding a provisional pair and this attempt is provisional too —
still no tours. KEEP THE PUBLISHED PAIR RATHER THAN RESTAMPING IT:
the centroid drifts as the robots move, so adopting each new one
would republish a slightly different cell every bootstrap period,
and every republication is a generation the followers have to
re-echo and re-commit. The team would never finish agreeing to
anything. The first fallback is as good as any later one — they all
name the place the team was standing when it had no better idea.
```

### rzv-refused-derive-keeps-pair

**A refused derive keeps the standing pair** — attached to `RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,`

```text
A refused solve leaves the standing pair alone. Dropping a pair the
team has already committed would be strictly worse than keeping it.

THIS IS NOW AN ABNORMAL OUTCOME. It used to be the normal early-run
answer — no tours to insert into, no floor, nothing admissible — and
the centroid fallback removed exactly that case: a refusal here now
means the snapshot has no locatable robot in it or the world is not
configured, not that the run is young.
THE COUNTS TRAVEL WITH THE REFUSAL. Without them this line cannot
distinguish "no tours yet" from "every tour cell was filtered out",
and those want opposite remedies. It is also, on the evidence of the
ts4 N=4 rung, frequently the ONLY record that an entire arm ran
without a treatment: the derive is gated on team_mutual, which at
N=4 was true for about two seconds, so this fired once and the run
continued for another 578 s with no appointment and every gate
green. A single throttled WARN is not an adequate witness for that,
which is why the campaign gate now fails the cell outright — but the
WARN is what says WHY, so it has to carry the arithmetic.
```

### rzv-restore-provenance-after-warn

**Restoring provenance after the refusal warning** — attached to `rendezvous_held_provenance_ = prev_prov;`

```text
RESTORED AFTER THE WARN, NOT BEFORE IT. deriveRendezvousProposal
overwrites the provenance before it solves, so the counts the WARN
above needs are the REFUSED search's — but leaving them in place
would describe a search whose answer nobody is holding, which is the
exact failure mode prev_prov was saved for, reached by the other
door. It became reachable when the derive gate was widened to allow
the one upgrade: before that, a refusal here meant no pair was held,
so there was nothing to misdescribe. If a provisional pair IS held,
the commit below can fire on this very tick and copy the provenance
into rendezvous_agreed_provenance_, where it lands on every armed row.
```

### rzv-follower-adopts-verbatim

**Follower adoption of the proposer's pair** — attached to `const RendezvousProposal& p = rendezvous_peer_[kRendezvousProposerId];`

```text
`else if (!proposer)` and not a bare `else`. The branch condition above
is now a conjunction rather than a plain `proposer` test, so a bare
`else` would drop the PROPOSER into the adopt path on every tick it did
not derive on — team incomplete, or a pair already held — and the
proposer would adopt whatever a follower had last echoed at it. That is
circular by construction (the followers are echoing the proposer's own
pair) and would make the pair's origin unprovable the moment anything
else went wrong.

FOLLOWER: adopt the proposer's pair verbatim. No re-derivation, no
nearest-valid-cell, no "repair" of a cell this robot dislikes — every
one of those is a private input, and a private input is what the whole
change exists to remove. A follower that dislikes the proposer's cell
drives to it anyway: that is the agreement. The only thing it may do is
fail to get there, which the outcome event records as a no-show —
quietly meeting somewhere else is not on the menu, because the peer
would still be standing at the agreed cell.

Staleness does not clear the held pair. While the team reads complete
the proposer is live by definition, so a gap here is a dropped message,
and forgetting a pair over one drop would churn the whole fleet's
agreement. Staleness bites where it should — the commit test below.
ADOPT ONCE, on the !held.valid() test — the same write-once rule the
proposer's derive branch takes, and for the same reason. The test used to
be `p != rendezvous_held_`, which adopts any pair that DIFFERS from the
one held, so a proposer that restarted mid-run and derived a second pair
would be adopted over a pair this robot's peers have already latched as
confirmed. The fleet would then hold two pairs with no way to tell which,
and the latches would be vouching for the old one. One pair per run has to
be enforced on every path that can write the pair, not just the deriving
one.

A proposer that does restart is not silently tolerated: it republishes an
empty pair first, the latch-clearing branch below sees it first-hand and
drops the confirmation, and the commit test stops passing. That is the
intended failure — no agreement — rather than a private re-agreement.

THE ONE UPGRADE, MIRRORED (2026-09-17). This branch has to take the same
exception the proposer's derive gate takes, and it has to take it from the
SAME evidence, which is why `provisional` is on the wire and not just in
the proposer's head. Without it this code was not merely incomplete — the
upgrade could not succeed in ANY interleaving. Either every follower
ERRORed and kept the placeholder while the proposer published an orphan
(the upgrade a structural no-op, plus a permanent ERROR), or, if the
proposer had not yet seen every echo when it re-derived, the proposer
committed nothing at all and refused every arming for the rest of the run
while its followers drove the placeholder without it. The second shape is
the ts4 N=4 arm death reproduced by the code written to fix it.

NOT GATED ON team_mutual. The proposer's side is, because deriving is the
step that must predate the separation; adopting is not, and gating the
echo path on the whole team being in contact was already a deadlock once
(see the declaration of maintainRendezvousProposal). A follower that hears
the replacement adopts it whenever it hears it.
```

### rzv-adopt-decision-in-handshake

**The adopt decision lives in RendezvousHandshake** — attached to `using Adopt = RendezvousHandshake::Adopt;`

```text
THE DECISION ITSELF IS NOT HERE. It is five booleans in, one of four
answers out, and it lives in RendezvousHandshake::adopt where a test can
enumerate all thirty-two inputs — which is the only way anyone was ever
going to notice that the upgrade could not succeed in any interleaving.
What stays here is the ROS half: which message to print, and the writes.
```

### rzv-follower-spends-reagree-flag

**Follower spends the re-agree flag** — attached to `rendezvous_reagree_due_ = false;`

```text
SPENT HERE, exactly as the proposer spends its own copy the moment it
adopts. This robot asked for a replacement at the end of its settle
and has just been given one; leaving the flag set would stand the
request open for the rest of the run and turn every later generation
into one this robot accepts unconditionally.
```

### rzv-adopt-conflict-is-a-fault

**An unexplained proposer pair is a fault** — attached to `RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 30000,`

```text
The proposer is publishing a pair that is not the one we adopted, it is
not the one upgrade, and this robot is not owed a replacement. So it is
a real fault (a restarted proposer, a fleet that disagrees about who
robot 0 is, two campaigns sharing a bus) and it is loud rather than
silently resolved either way.

THE FLAGS ARE IN THE MESSAGE because they are what separates this from
the legitimate cases above, and the three shapes that land here say
different things: provisional over final is a proposer walking
BACKWARDS, final over final is a generation nobody at this robot asked
for — which since generation 29 means the two sides disagree about
whether the last meeting was kept, not that a second generation is
illegal per se — and provisional over provisional is the centroid being
republished, which the derive side is supposed to make impossible.
```

### rzv-peer-latch-update

**Latching what peers were heard to hold** — attached to `for (int id = 0; id < fleet_.size(); ++id) {`

```text
--- latch what the peers have been HEARD to hold ----------------------

Runs after the adopt branch above on purpose. A follower learns the pair
and records that the proposer holds it on the SAME tick, rather than
waiting a full heartbeat for the next copy of a message it already has.

rendezvous_peer_[id] is the last pair RECEIVED from that peer and is not
cleared by silence, so this reads "the most recent thing peer id told me",
and the latch turns that into "peer id has at some point told me it holds
exactly the pair I hold".

WHAT MAKES THAT SAFE TO OUTLIVE ITS FRESHNESS WINDOW. The transitions a
peer's published triple may make are empty -> P -> R -> R' -> ..., where P
is provisional and every R is final (TeamWorld.msg,
rendezvous_provisional), and each step after the first two is one the team
earned by keeping a meeting. A latch can therefore be contradicted by a
later message in two ways — it records P and the peer has upgraded to R, or
it records R and the team has re-agreed past it — and THIS LOOP DROPS IT IN
BOTH, from the peer's own message, on the tick it arrives, because
updatePeerLatch clears on any disagreement rather than on a named
transition. A latch that survives is one no later message has contradicted.

IT USED TO DROP ONLY ON pair -> empty, and that was a false-commit bug, not
a missed-commit one: the proposer would upgrade P -> R, a follower still
holding P would count its stale P-latch for the proposer, reach fleet-1,
print "Rendezvous AGREED by all" and arm against a triple the proposer does
not hold and can no longer return to. It would then record a no-show
against a peer that was never on that schedule. Committing something false
is strictly worse than committing nothing, and the freshness the commit
rule deliberately does not test is not what was protecting it — first-hand
contradiction is.

The two-line body is RendezvousHandshake::updatePeerLatch, for the same
reason the adopt decision moved: the clear-then-relatch ORDER is the whole
correctness argument and it is worth a test that does not need a simulator.
```

### rzv-latch-skip-never-heard-peer

**Never-heard peers are not evidence** — attached to `if (rendezvous_peer_at_sec_[id] < 0.0) continue;`

```text
NEVER HEARD FROM is not the same as heard-saying-nothing, and only the
second is evidence. A peer whose slot still holds the default-constructed
proposal because no TeamWorld has ever arrived from it must not be read
as having contradicted anything — there is no latch to clear and no
message to clear it with.
```

### rzv-commit-rule-symmetric

**The symmetric commit rule** — attached to `if (!rendezvous_held_.valid()) return;`

```text
--- the commit rule ---------------------------------------------------

Symmetric on every robot, proposer included: commit when EVERY peer has
been heard holding the same pair this robot holds. On the proposer that
reads "all my followers have echoed"; on a follower it reads "the proposer
said this AND my fellow followers have it too". One rule, so no robot can
reach a conclusion the others cannot.

The confirmations ACCUMULATE rather than having to coincide. Each peer's
echo is latched into rendezvous_peer_confirmed_ when it is heard, and the
commit counts latches. That is only sound if a latch cannot quietly stop
being true, and the argument for THAT is not "the triple is frozen for the
run" — it is not frozen at all: a peer moves empty -> provisional -> final,
and then to a fresh final after every meeting it keeps. The argument is
that EVERY contradicting transition is announced in the peer's own next
message and the loop directly above drops the latch when it arrives, which
does not care how many transitions there are. See it there, and the N=3
measurement that forced accumulation in the first place at the declaration.

FRESHNESS IS DELIBERATELY NOT TESTED HERE. It used to be, bounded by
coord_claim_ttl_sec, on the reasoning that "the team is complete" and "the
team is holding a pair" should not be true of different sets of robots.
But those two statements are about different things: completeness is a
claim about NOW, and holding the pair is a claim about a decision the peer
already took and cannot revise. Ageing out the second one discards a fact
that is still true, and at N>=3 it discarded enough of them that nothing
was ever agreed. Liveness still gates everything it should — the anchor,
the derive, the snapshot — just not this.

MUTUAL CONTACT IS NOT TESTED HERE EITHER, and this is the substantive
half. The old rule took it because a one-way link satisfies freshness, and
committing under one would mean "concluding the team holds a pair my own
echo may never have reached". The latch closes that hole directly instead
of approximating it: a latch is only ever written from a message RECEIVED
from that peer carrying that exact pair, so it is first-hand evidence that
the peer holds it. This robot never infers a peer's state from its own
transmissions, so there is nothing for one-way contact to break. The
symmetric condition still guards the one quantity that genuinely needs
both directions — rendezvous_anchor_time_, which is the shared origin every
t_meet is measured from, and which is still stamped only under
rendezvousTeamMutual().
```

### rzv-latch-scoped-to-held-pair

**Latches count only for the held pair** — attached to `if (rendezvous_peer_confirmed_[id] == rendezvous_held_) ++on_pair;`

```text
Scoped to the CURRENT held pair by exact equality, so a latch left over
from a pair THIS robot has superseded does not count, with no clear to
forget. The other direction — a latch left over from a pair the PEER has
superseded — is not covered by this line and never was; it is cleared
explicitly in the loop above, and the difference between the two is a
false commit rather than a missed one.
```

### rzv-commit-needs-every-echo

**Every commit waits for every echo** — attached to `if (on_pair != fleet_.size() - 1) return;`

```text
Counted and then tested, rather than returning early inside the loop. The
early-return form made this line unreachable-false — the loop could only
fall out with the full count — so the one explicit statement of the rule
("every peer, not a quorum") was dead, and `rendezvous_agreed_peers_` was
an arithmetic identity rather than a count of anything. It is now the real
gate, and the field below is a number that could have come out lower.

A FINAL TRIPLE COMMITS ON FIRST-HAND EVIDENCE AND DOES NOT WAIT FOR ECHOES
(2026-09-17). This is the fix for the N=3 SPLIT FLEET, and the measurement
that forced it is worth stating because the obvious reading of that failure
is the wrong one.

  1789641087.897  atlas   proposal (upgraded from the centroid fallback):
                          cell 73 ... over 25 candidate(s)
  1789641089.539  bestla  robot 0 upgraded off the centroid fallback —
                          adopting cell 73 (was cell 44)
  1789641089.539  husky   robot 0 upgraded off the centroid fallback —
                          adopting cell 73 (was cell 44)
  1789641092.865  atlas   AGREED by all 3 robots: cell 73 (provisional=0)
                          ... and no such line on bestla or husky, ever.

ALL THREE ROBOTS HELD CELL 73, 1.6 s apart. Nobody was partitioned from the
proposer and nobody was stuck on the placeholder. What split the fleet was
this rule: the proposer heard both echoes and committed, while each FOLLOWER
was waiting on the OTHER FOLLOWER's echo — a follower-to-follower link that
at N>=3 in this radio regime need not exist and here did not. Arming reads
rendezvous_agreed_, so atlas would have driven to cell 73 while bestla and
husky drove to cell 44. A real behavioural split, produced entirely by
demanding evidence that could not change anyone's action.

WHY THE ECHO ROUND IS THE RIGHT RULE FOR P AND THE WRONG ONE FOR R. The
round exists so that no robot concludes something the others cannot. For a
FINAL triple that is satisfied by construction, from two properties this
protocol already enforces elsewhere:

  * it is authored by ONE robot (kRendezvousProposerId) and adopted verbatim
    — there are no integers to reconcile, only a value to receive; and
  * final is TERMINAL — RendezvousHandshake::adopt refuses to replace it, so
    a robot holding it will still be holding it at the end of the run.

So every robot that ever receives R converges on R and stays there, and a
robot that does not receive R is partitioned from the proposer — in which
case no echo was reaching it either. The echo tells you WHO ELSE heard it,
which is a fact the arming rule has no use for.

THE PARAGRAPH ABOVE IS WRONG, AND IT COST A CELL (corrected 2026-09-17,
generation 22). It is kept rather than deleted because the rule it argues
for is still the rule in force below, and someone will otherwise make the
same argument again.

What it gets right is that a robot which does not receive R is partitioned
from the proposer. What it slips on is the word "either" — it treats that
robot as harmlessly out of the conversation, when what that robot actually
does is ARM, on the placeholder it committed earlier, and drive to a
different cell. Partition is not silence. The echo does not merely tell you
who else heard it; withholding the commit until every peer has echoed is
what stops the proposer from moving to a generation someone else can never
reach. Under the old rule the proposer could not have committed R at all,
and the fleet would have kept its one agreed placeholder.

Measured, in ts4 smoke20's N=3 hybrid cell: bestla lost its last peer at
t+65.1 s, the proposer authored R at t+65.7 s, and bestla armed on the
placeholder while atlas and husky armed on R. Two exact appointments, two
cells, 400 s of the run spent waiting in the wrong places.

THE RULE STAYS ANYWAY, because reverting it restores a different split —
the N=3 follower-to-follower deadlock measured directly above, which is not
hypothetical either. Neither rule is correct on its own, and no rule is:
committing an upgrade atomically across a partitionable fleet is the Two
Generals problem, so every candidate protocol just chooses which robot gets
left behind. Generation 22 tried to change WHEN this may run — confining
the upgrade to the pre-mission hold, while the fleet is co-located — and
that was withdrawn on measurement the same day: the upgrade's input is
completed exploration steps, a held robot completes none, and the
confinement deleted the upgrade instead of scheduling it. See the derive
gate in maintainRendezvousProposal for the numbers.

SO THIS SPLIT IS NOT CLOSED, it is bounded. The proposer keeps re-deriving
while the pair is provisional and publishTeamWorld re-broadcasts the held
pair every cycle, so a robot that misses an upgrade adopts it when the link
returns; the disagreement lasts as long as the partition does, not as long
as the run. A split that outlives the run is still reported by the
rendezvous_agreed comms gate rather than silently tolerated.

THE ECHO ROUND IS BACK, FOR EVERY COMMIT AND EVERY N (2026-09-18). The two
paragraphs above are kept because both failures they describe are real and
measured, and someone will otherwise re-derive one of them. What changed is
not the argument, it is the thing being argued about: BOTH of those splits
are splits between a robot on R and a robot on P, and both were harmful
only because P was a placeholder rather than a usable agreement.

  * P now carries the same three integers R does — cell, t_meet, interval —
    and armAppointment attends the agreed schedule rather than overwriting
    the time with a private countdown. A robot left on P is not stranded on
    "a cell with no meeting"; it holds a complete place and time that the
    whole fleet unanimously committed while it was co-located.
  * A failed upgrade therefore degrades to "the team keeps the agreement it
    already has", which is the correct behaviour for a fixed rendezvous and
    is what the operator asked for in as many words: a predefined time and
    place, agreed before the mission, not renegotiated by robots that can no
    longer all hear each other.

So the choice the Two Generals framing offers — which robot gets left
behind — is not the choice being made here. Under unanimity NOBODY moves to
a generation the rest of the fleet cannot reach: the upgrade either lands on
everyone or on no one. The deadlock the gen-22 measurement recorded (each
follower waiting on the other follower's echo, over a link this radio regime
need not provide) still happens, and its outcome is now a unanimous fleet on
P instead of a fleet in two places.

WHAT IT COSTS is the case the note at the derive gate warned about: an
upgrade that never lands leaves both scheduled arms measuring "meet at the
team's initial centroid". That is no longer a silent degradation — the
provisional flag is on the AGREED log line and in the banked row, so a cell
that never upgraded is visible as such.
```

### rzv-agreed-at-stamp-on-change

**Agreement age is stamped on change only** — attached to `if (rendezvous_agreed_ != rendezvous_held_) {`

```text
Stamped only when the pair CHANGES, so rendezvous_agreed_at_sec_ is the age
of this agreement and not the age of the last re-confirmation of it; a
stamp that refreshed on every heartbeat would report every appointment as
brand new.

It is REPORTED, never enforced: nothing refuses an old pair, and age is not
a defect to bound. An agreement stamped at t+30 s is exactly as usable at
t+2000 s, because what the team agreed is a PLACE and a RECURRING SCHEDULE
— cell, t_meet, interval — and a schedule does not expire, it rolls forward
(see nextAgreedOccurrence). What this number measures is "how long ago the
team last agreed something", which is a diagnostic, not a validity test. A
threshold here would be a private input deciding not to keep a public
agreement, which is the thing the protocol exists to prevent.

A SECOND AGREEMENT IS NOT AN ANOMALY. The triple is derived once per run,
but a provisional pair may later be replaced by the tour-informed upgrade,
so a cell with two rendezvous_agreed rows at increasing t is the upgrade
landing rather than a handshake that failed to settle.
```

### rzv-agreed-log-line-contract

**The AGREED log line is a contract** — attached to `RCLCPP_INFO(get_logger(),`

```text
THE MESSAGE THE CAMPAIGN GATE GREPS FOR. sim/rendezvous_agreement.py
fails a scheduled cell that never printed this line, so the leading
"Rendezvous AGREED by all" is load-bearing text and not prose, and
sim/rendezvous_agreement_calib.py renders this very format string to
prove the two have not drifted apart.

THE PROVISIONAL FLAG IS ON THE LINE because a cell where the whole team
committed the centroid placeholder and a cell where it committed a
tour-informed meeting point are measuring different mechanisms, and
without this they are indistinguishable in the banked logs. Both are
valid runs; only one of them exercised the scheduler's argmin. A team
that commits provisional and upgrades later prints this line twice.

THE MEETING TIME IS BACK ON THIS LINE (2026-09-18, second revision), as
`from t+Ys` appended to the interval rather than as the gen-19 `every Xs
from t+Ys` phrasing — the two carried the same pair of integers and the
gen-19 form is a separate branch of the gate's regex, so reusing it would
make a gen-23 line parse as a pre-gen-20 binary.

It was dropped the same morning as false, and it was: armAppointment had
been changed to overwrite t_meet with a private countdown, so the line
described a schedule the code did not keep. The countdown is gone and the
schedule is the thing armAppointment attends, so the line is a statement
about behaviour again — and it is the ONLY place the agreed meeting time
reaches the banked text logs, which is what an offline reader needs to
check that two robots armed the same MEETING rather than the same cell.

THE ECHO COUNT IS REPORTED SEPARATELY from the fleet size, and now they
agree by construction: the early return above requires on_pair ==
fleet-1 for every commit, so "all N robots" is an OBSERVATION and not, as
it was for one day, an inference from the protocol. Both are printed
anyway. If they ever differ in a banked log, the commit rule regressed,
and a reader who only has the text log should be able to see that.
```

### rzv-agreed-provenance-copied-together

**Triple and provenance copied together** — attached to `rendezvous_agreed_provenance_   = rendezvous_held_provenance_;`

```text
The triple and the reason it was chosen are copied TOGETHER, in the one
place where the two are known to describe each other. On the proposer the
held plan is the solve that produced this exact triple: the derive runs
earlier in this same call, so either it produced both of these or it
refused and put back the provenance of what is still held. On a follower
it is the empty plan, deliberately.
```

### rzv-agreed-peers-assertion

**Why rendezvous_agreed_peers_ is still recorded** — attached to `rendezvous_agreed_peers_ = on_pair;`

```text
Outside the change test on purpose: it is re-asserted on every tick that
reaches here, rather than being frozen at the agreement's birth.

IT IS AN ASSERTION AGAIN (2026-09-18), because the echo round is back for
every commit: the early return above is the only way out of this function
without full agreement, so reaching here means on_pair == fleet-1 and this
field cannot read anything else.

It was a genuine measurement for one day, while a final triple committed on
first-hand evidence and a row could legitimately carry on_pair < fleet-1.
Anything analysing the banked corpus has to read it that way PER GENERATION
and not pool: the same column means "how many peers had echoed" on gen 22
and "fleet-1, always" on gen 23.

Kept rather than replaced by a constant because it is the one place a
regression in the commit rule would show up in the data rather than only in
the code — a gen-23 row carrying on_pair < fleet-1 means this return was
bypassed.
```

## ExploPlannerNode::armAppointment

### rzv-arm-recurring-schedule

**Which occurrence of the schedule arming attends** — attached to `bool ExploPlannerNode::armAppointment(const char* reason) {`

```text
WHICH OCCURRENCE OF THE AGREED SCHEDULE THIS ARMING ATTENDS.

The agreed triple is a RECURRING schedule, not a single instant: meetings
happen at `t_meet_ms + k * interval_ms` for k = 0, 1, 2, ... That is what
makes one agreement, made once while the team was whole, keepable for the
rest of the run — the property the countdown was reached for and got by
abandoning the agreed time instead.

EVERY INPUT HERE IS AGREED OR SHARED. `t_meet_ms` and `interval_ms` are the
integers the whole team committed, byte-identical on every robot and part of
RendezvousProposal::operator== so a fleet cannot hold two of them and report
agreement. `not_before_sec` is this robot's mission clock, whose baselines
are ~1.2 s apart across the ts4 cells. So two robots disagree about k only
when their `not_before` values straddle a multiple of the interval, and when
they do the disagreement is bounded by ONE interval and resolves at the
barrier rather than sending anyone to a different place.

WHY THIS IS NOT THE occurrenceAtOrAfter DESIGN THAT WAS REVERTED. That one
failed because a no-show was terminal: a robot that picked an earlier
occurrence than its peers logged the meeting as missed and left. The barrier
is unbounded now (rendezvous_appointment_wait_sec defaults to 0 = wait for
the team, not for a clock), so picking k too small costs waiting, which is
what a robot at a meeting point is supposed to be doing.

A NON-POSITIVE INTERVAL degrades to the single agreed instant rather than
dividing by zero. The scheduler floors the interval at
rendezvous_proposal_period_sec (30 s), so this is a guard, not a path.

IT LIVES IN planner_util.hpp, NOT HERE, and that is deliberate. This file
defines main(), every gtest target links gtest_main, and the two cannot go in
one binary — so nothing defined in this translation unit can ever be called by
a test, and node-side coverage is source-scan only. The arithmetic that
decides WHICH INSTANT the fleet meets at is the load-bearing half of the
rendezvous arm; it gets executable tests (test_planner_util.cpp), which means
it gets to be a free function in the library. This translation unit is already
inside namespace explo_planner, so the call below needs no qualification.
```

### rzv-arm-keep-standing-appointment

**Keeping an appointment that already stands** — attached to `if (appointment_armed_) {`

```text
An appointment already stands. KEEP IT — do not re-derive one.

The mid-run trigger has always been gated on !appointment_armed_ for this
reason, but the three finishOrRendezvous paths call dispatchReconnect
unconditionally, and dispatchReconnect calls this. Both outcomes of
re-entering were wrong:

  * a SUCCESSFUL re-arm re-solved the same frozen snapshot with a new
    `mission_elapsed_ms`, so t_meet slid forward by however long had
    passed. A robot whose step budget expired at t+600 moved its own
    t_meet from 800 to 900 while its partner still held 800; the partner
    waited out the cap and left before it arrived. It also overwrote
    appointment_ without calling closeAppointment, dropping the outcome
    event and breaking the 1:1 agreed/outcome join that the offline
    analysis is built on.
  * a REFUSED re-arm fell into the else branch below, which cleared the
    plan but not the flag. That left appointment_armed_ true with
    t_meet_ms == -1, so appointmentDue() was false forever (the robot
    never departed) AND the mid-run trigger stayed suppressed — reconnect
    was dead for the remainder of the run, silently.

Keeping it is also the only answer consistent with the mechanism: the
appointment is a promise the PEER is holding too, and nothing that has
happened on this robot since gives it the right to move a shared deadline
unilaterally. It is released by closeAppointment on the paths that already
own that decision — the team returning, the manoeuvre ending, the run
ending — after which the next dispatch arms a fresh one normally.

No event: a rendezvous_agreed here would be an `agreed` with no `outcome`,
which is the very join this is protecting.
```

### rzv-arm-nothing-solved

**Arming adopts, it does not solve** — attached to `RendezvousAgreedEvent ev;`

```text
NOTHING IS SOLVED HERE ANY MORE. This function used to derive the
appointment from the frozen snapshot, and that is precisely what broke: two
robots running the same deterministic arithmetic over two private maps
agreed on the cell 21 times in 64 (see the P5 state block for the full
measurement). All that remains is bookkeeping — take the pair the team
committed while it was still connected, give it this robot's own origin,
and report what it did.
```

### rzv-arm-one-appointment-per-outage

**One appointment per outage** — attached to `ev.refused = "the agreed pair has already been used in this outage";`

```text
The one appointment this outage gets has already been kept or missed.
Without this the latched pair would re-arm immediately with a t_meet
that is now in the PAST, appointmentDue() would fire on the same tick,
and the robot would drive back to the cell it just left — for as long as
the outage lasted. The pair cannot have changed since (the protocol is
frozen while separated), so a second arming has nothing new to say.
```

### rzv-arm-anchor-refusals-removed

**Why the anchor refusals were removed** — attached to `if (have_rendezvous_anchor_) anchor = missionElapsedAt(`

```text
THREE ANCHOR REFUSALS USED TO STAND HERE and all three are gone, because
every one of them was about the ORIGIN and the appointment no longer has
a local origin to be wrong about. They refused when this robot had never
seen mutual contact, when the executor was starved across the transition
that stamps the anchor, and when the anchor predated the mission
baseline — each because `t_meet = my_anchor + interval` would then be
exact, agreed, and at the wrong time. `t_meet` is now the integer the
team committed, so a suspect anchor cannot move it, and keeping the
refusals would have blocked arming for a reason that had stopped
existing. The anchor itself is still stamped and still logged: it is how
the analysis tells an early agreement from a late commit.
```

### rzv-arm-lateness-refusal-removed

**Why the lateness refusal was removed** — attached to `}`

```text
A FOURTH REFUSAL STOOD HERE — "the agreed meeting time passed more than
the wait cap ago" — and it went the same way as the other three, for the
same kind of reason: the condition it tested cannot arise any more.

It existed because `t_meet` was ONE instant, stamped when the proposal was
derived. Nothing tied that instant to when the team finished committing
it, and nothing tied it to when the team eventually came apart, so a
separation an hour into a run armed an appointment whose time was an hour
gone: appointmentDue() fired instantly, the robot drove to a cell whose
meeting was over, and stood there for the full wait cap — a guaranteed
no-show that consumed the outage's one appointment and read in the log
exactly like a kept appointment the peer failed to attend. The refusal
turned that into an honest "too late", which was the best that could be
done with a time that was fixed before the separation it had to serve.

The agreed time is now a RECURRING schedule and this arming attends the
first occurrence of it not already past (nextAgreedOccurrence, below),
so the meeting this robot arms cannot be behind it. There is no lateness
left here to refuse on — and, unlike the countdown that briefly stood in
its place, that is true without giving up the agreed instant: the roll
is over the committed integers, and the only per-robot input is the
robot's own `t_now`, which generation 25 made the BARE floor after the
gen-24 `t_now + notice` floor forked the ts4 N=3 cell (the block at the
assignment prices that out).
```

### rzv-arm-occurrence-floor

**Choosing the rung: floor and shortfall** — attached to `appointment_             = RendezvousPlan{};`

```text
THE PLACE AND THE TIME ARE BOTH ADOPTED (2026-09-18, third revision).

All three integers come off the wire exactly as the team committed them.
`cell` is the place. `t_meet_ms` and `interval_ms` are a RECURRING
SCHEDULE — meetings at t_meet + k*interval — and this arming attends the
first occurrence this robot can still REACH, floored at t_now plus its
own arrival shortfall (generation 29; zero for any robot inside its
lateness budget, which is what keeps the team on one occurrence). The
operator's rule ("a place and a time which is fixed") is the lattice: a
robot never invents an instant, it only picks which rung of the agreed
one it signs up to.

WHAT THIS REPLACES, AND WHY THE REPLACEMENT WENT BACK. From 02:00 to now
this line read `t_now*1000 + depart_delay*1000` — each robot's own
countdown, minted at its own arming. That is not a rendezvous. It is N
robots independently deciding to visit the same place, and the arm was
therefore not measuring the mechanism it is named after. Two earlier
attempts at an agreed time had failed and the countdown was the reaction
to them; both failures are real and neither is a reason to keep it:

  `anchor*1000 + interval` — anchored on "the instant MY view of the team
  stopped being mutually whole". One event at N=2 (the ts4 N=2 cell put
  the two anchors 0.84 s apart), a per-robot predicate over a graph that
  comes apart edge by edge at N>=3 (13.18 / 34.18 / 33.98 s). The origin
  was private. It is not private here: t_meet is authored once by the
  proposer and adopted verbatim, and it is inside operator==, so a fleet
  cannot hold two of them and still report agreement.

  `occurrenceAtOrAfter(phase, period, now)` — a shared schedule indexed by
  each robot's own arming instant: armings at 16.1 / 52.5 / 67.4 s against
  a 30 s period gave three different meetings, "one robot logging a no-show
  for a meeting the others were still driving to". THE NO-SHOW IS THE
  DEFECT IN THAT SENTENCE, NOT THE SCHEDULE. A robot that picks an earlier
  occurrence than its peers is standing at the agreed cell when they
  arrive; it only fails if something makes it leave. The barrier is
  unbounded (rendezvous_appointment_wait_sec = 0, the campaign default —
  it holds until the team reads complete, not until a clock expires), so
  the disagreement costs waiting rather than a missed meeting, and it is
  bounded by one interval rather than by the run.

THE FLOOR IS BARE t_now SINCE GENERATION 25, AND THE 100 s NOTICE THAT
USED TO BE ADDED HERE IS WHAT FORKED THE ts4 N=3 CELL. With
`t_now + 100`, the tolerance for arming spread was the gap from the
last floor to the agreed instant MINUS the notice: the cell's schedule
led by 149 s, the notice ate 100 of it, and the 33 s arming spread
(an N>=3 graph dies edge by edge; anchors 72.90 / 72.84 / 90.58 s)
crossed the rest. Two robots floored at ~178/179 s and kept the agreed
190.148 s; the third floored at ~211 s and rolled to 346.716 — base plus
exactly one interval — so its peers stood at the cell 351 and 432 s and
the cell censored with the barrier open. With this floor the same
armings tolerate the WHOLE 149 s gap; only a robot that arms after the
instant has genuinely passed, or that cannot reach it within its
lateness budget, rolls forward — the two cases where rolling is the
truth. The per-robot terms are `t_now` itself, the mission-clock
baseline (`mission_t0_sec_`, latched on each node's first live tick),
~1.2 s across these cells — bounded by node start, not by the radio —
and the arrival shortfall, which is EXACTLY ZERO for every robot that
can make the occurrence and so cannot separate robots that can both
attend. A late-run separation can still straddle an
occurrence boundary by bad phase — no rule computed from private
observations can prevent that — but the fork is no longer manufactured
by subtracting a constant from the margin, and the barrier prices a
residual fork at one interval of waiting rather than a censored run.

t_meet IS AN ARRIVAL INSTANT (2026-09-19), so this picks the first rung
of the team's lattice that this robot can actually BE AT. Departing in
time to arrive is appointmentDue()'s job and needs no help here; what
this decides is the case appointmentDue() cannot fix, where the nearest
rung is already closer than this robot's drive and leaving instantly
would still be late.

THE BUDGET IS SUBTRACTED FROM THE DRIVE, NOT ADDED TO THE FLOOR, and the
clamp at zero is the whole safety argument. Generation 19-24 set the
floor to `now + rendezvous_depart_delay_sec`, a lead time every robot
added unconditionally, so robots that could all comfortably make the same
rung still split across two of them — the ts4 N=3 cell, two robots on one
occurrence and the third a whole interval past it. Here the floor is bare
t_now for every robot whose marked-up drive fits inside the margin plus
the budget, which is the common case and cannot fork. It rises above
t_now only for a robot that genuinely cannot arrive in time, and then by
exactly its shortfall. Clamped at zero it can only ever move the rung
LATER, so the "deadline already passed" impossibility below still holds.

A FORK HERE IS PRICED, NOT PREVENTED. A robot that rolls is one interval
out of step with peers that did not, and the barrier's unbounded wait
pays for that in standing time. It is the better trade: the alternative
is that robot arriving arbitrarily late — 563 s on the stopped ts4 cells
— with the team standing for that instead, and the rolled robot spends
the interval exploring rather than driving.

NO ESTIMATE, NO ROLL. travel -1 means the robot cannot price its own
drive, and a robot that cannot price it must not be the one to decide the
team is unreachable; it keeps the nearest rung, as every robot did before
this change.

THE ROBOT KEEPS EXPLORING UNTIL IT DEPARTS. A later occurrence is not
idle time — appointmentDue() gates the departure, not the work — so the
quantisation buys exploration, and what it spends is time spent separated.
That is the trade a fixed schedule makes and it is the thing the arm is
supposed to measure.
```

### rzv-arm-copy-capped-floored

**Copying capped, floored and tour interval** — attached to `appointment_.capped               = p.capped;`

```text
THE THREE THAT WERE MISSING (2026-09-17). `capped`, `floored` and
`tour_interval_ms` are RendezvousPlan fields like the five above, the
solve fills all eight, and the logged row reads them off THIS struct —
so omitting them here made them structurally false/-1 on every row
ever written, including the proposer's. That is worse than an inert
column: `capped`'s own doc says a run where it is always true is one
where the cap rather than the objective chose the meeting times, and
the follower-row doc tells an analyst to take the value from the
proposer's row. Both instructions pointed at a constant.
```

### rzv-arm-follower-row-sentinels

**Follower rows use -1 sentinels** — attached to `appointment_.penalty_mm           = -1;`

```text
EXPLICIT SENTINELS ON A FOLLOWER ROW. A follower echoes three integers
and never runs the argmin, so there is no candidate count and no
verdict on the floor. The struct's own defaults would write 0 for the
counts, which is a value a real search can also produce ("nothing was
admissible"), and the two would be indistinguishable in the log. -1
cannot be a count, so it can only mean "this row did not search".
```

### rzv-arm-own-route-check

**Route check is reported, not enforced** — attached to `if (rendezvous_world_.configured()) {`

```text
CAN THIS ROBOT ACTUALLY GET THERE? Asked of the graph DIRECTLY, not
through appointmentTravelMs below, because that goes via
GlobalAllocator::costMm and costMm masks the unreachable sentinel with a
centroid straight line — so the travel estimate is finite even for a
cell there is no route to, and the one robot that is never going to
arrive looks exactly like the ones that will.

It is REPORTED, never enforced. A follower does not solve, so it does
not get to veto: the pair is the team's and the whole design is that it
is kept exactly. And a negative reading here is as often ignorance as
geometry — the graph blocks edges over unknown ground, so a robot that
has simply not explored the corridor yet reads "unreachable" to a cell
it will reach comfortably. Refusing on that would make followers walk
away from appointments precisely in the far-apart case the arm exists to
test. What it buys is that the resulting no-show is legible instead of
anonymous.
```

### rzv-arm-agreed-age-unclamped

**Agreement age is not clamped** — attached to `ev.agreed_age_sec = anchor >= 0.0 ? anchor - rendezvous_agreed_at_sec_`

```text
NOT CLAMPED AT ZERO, and the negative values are the point. This is
(anchor - when the team committed), so a negative reading means the
commit landed AFTER the separation it is anchored to — the late-commit
case the "already passed" refusal above exists for, and the only field
that can distinguish it from an ordinary early agreement. The clamp that
used to be here folded every one of those onto 0.0, where they were
indistinguishable from "agreed at the instant of separation", which is
the healthiest reading the column has. A diagnostic that reports its
worst case as its best case is worse than an absent one.

GUARDED ON THE ANCHOR EXISTING, which it need not any more: arming no
longer requires one (see the three removed refusals above), so `anchor`
can legitimately be -1 here and the subtraction would then report a
plausible-looking negative age that is really just the sentinel with the
agreement time taken off it — the late-commit reading, manufactured.
```

### rzv-arm-derivation-world-hashes

**Logging the world the argmin ran over** — attached to `ev.shared_hash      = rendezvous_agreed_provenance_.shared_hash;`

```text
THE WORLD THE ARGMIN RAN OVER, on the proposer — captured inside
deriveRendezvousProposal and carried here with the pair. Reading the live
snapshot instead, as this did until the provenance existed, logged
whatever the last team-complete heartbeat copied, which is up to one
proposal period newer than the search it would be claiming to describe.
On a follower they stay 0/-1: it never solved, so there is no derivation
world to name, and its own snapshot is not one.
```

### rzv-arm-capped-column

**The capped column is live** — attached to `ev.capped               = appointment_.capped;`

```text
Read from the appointment rather than hardcoded. This is now a live
column: deriveRendezvousProposal feeds the barrier wait in as
max_interval_ms, so a solve whose tour term runs past the wait cap really
does cap, and `capped` says the interval was pulled in to keep the
meeting findable rather than left at the tours' own arrival. (It read
"the period ... the occurrence" until 2026-09-18; nothing recurs, and the
cap now bounds the furthest robot's drive — see Config::max_interval_ms.)
```

### rzv-arm-floored-column

**Which term set the timetable** — attached to `ev.floored              = appointment_.floored;`

```text
WHICH TERM SET THE TIMETABLE. Copied into appointment_ since generation
17 and read by nothing until now, which left the row unable to say
whether `interval_sec` was the objective's answer or the lattice's —
the distinction generation 29's 300 s floor makes the common one, and
the one `capped` above cannot stand in for (it asks only whether the cap
cut the tour term, and a cap above the tours cuts nothing while still
being overruled by the floor).
```

### rzv-arm-agreed-base-unrolled

**Logging the unrolled agreed base** — attached to `ev.agreed_base_sec      = rendezvous_agreed_.t_meet_ms / 1000.0;`

```text
THE UNROLLED INTEGER, from the committed triple rather than from the
appointment: `appointment_.t_meet_ms` above is what nextAgreedOccurrence
made of it for THIS robot, and the two differ by whole intervals exactly
when the roll did something. Logging the input is what lets a reader tell
one generation's rows from the next one's — see the field's doc.
```

### rzv-arm-deadline-passed-unreachable

**Why there is no deadline-passed check** — attached to `}`

```text
A "THE DEADLINE ALREADY PASSED" DIAGNOSTIC STOOD HERE and is deleted as
unreachable rather than left as a guard that can never fire. It tested
`t_meet_ms < t_now * 1000`, and t_meet_ms is assigned, above, as the
first agreed occurrence at or after a floor that is t_now plus a
non-negative shortfall (nextAgreedOccurrence), which cannot precede
t_now while the committed interval is positive — and the scheduler mints
only positive intervals (floored at the furthest robot's drive). The
clamp at zero on that shortfall is what keeps this true: it is why the
reachability roll can only move the instant later, never earlier. The
condition is false by construction on every reachable path, and a branch
that cannot execute is a claim that the reader has to disprove.
```

### rzv-arm-travel-covariate

**Travel is logged, not a deadline** — attached to `ev.travel_sec = travel / 1000.0;`

```text
TRAVEL, NOT A DEPARTURE DEADLINE. There is still no derived deadline
on this row: the robot AIMS at ev.t_meet_sec and leaves whenever its
live lead says it must, which is a per-tick decision and not a number
that can be stamped once. t_meet_sec is the same on every robot's row
for the same outage unless one of them rolled a rung for
unreachability, or armed after the instant had genuinely passed.
Travel is kept because it is the covariate that separates "arrived
late" from "was never close" — and, now, the one that explains a roll.
```

### rzv-arm-refusal-clears-flag-and-plan

**A refusal writes both flag and plan** — attached to `appointment_armed_       = false;`

```text
Both writes, not just the plan. appointment_armed_ is already false on
every path that reaches here — the early return at the top of this
function makes it so — but that invariant is held REMOTELY, by a guard
added for an unrelated defect, while the flag and the plan are read
together everywhere downstream. Leaving one of the pair unwritten on a
refusal path means a future edit to that far-away guard turns a refused
appointment into an armed one pointing at a blank plan, and the first
symptom is appointmentPoint() on an invalid cell. Cheap here, expensive
to diagnose there.
```

## ExploPlannerNode::appointmentLeadMs

### rzv-lead-markup-matches-scheduler

**Departure lead uses the scheduler's markup** — attached to `return (travel * std::max(0, rzv_cfg_.depart_safety_milli)) / 1000;`

```text
THE SAME MARKUP THE SCHEDULER'S REACHABILITY FLOOR USES
(rendezvous_scheduler.cpp: floor_ms). The floor guarantees the lattice
spacing covers the furthest robot's marked-up drive; pricing the lead at a
different rate here would mean a robot needing more lead than the spacing
the team sized for it, which is the one case a rung roll cannot fix.
```

## ExploPlannerNode::appointmentTravelMs

### rzv-travel-guard-either-condition

**Travel guard uses either condition** — attached to `if (!appointment_armed_ || !appointment_.valid()) return -1;`

```text
EITHER condition, because the two states this guards against are different
and either one alone makes the answer meaningless: `!armed` is "nothing to
travel to", `!valid()` is "the appointment field holds no cell".

THIS WAS `&&` UNTIL 2026-09-18, AND THE COMMENT ABOVE IT CLAIMED THE
CONJUNCTION WAS "the weaker test ... rather than an invariant a future edit
could quietly break". It is the weaker test, which is exactly the problem:
a conjunction refuses only when BOTH hold, so the single state the prose
says it is defending against — armed with a blank plan, which armAppointment
names in as many words as the thing its paired writes exist to prevent
("the first symptom is appointmentPoint() on an invalid cell") — fell
straight through to costMm() on cell -1. The two DO coincide today: the one
arming site sets `appointment_armed_` only under `appointment_.valid()`,
and both clear sites write the flag and the plan together. So this changes
no behaviour on any path that exists now, and that is the point — the
guard is here for the edit that breaks the coincidence, and under `&&` it
would not have caught it.
```

### rzv-travel-from-current-position

**Travel is priced from the current position** — attached to `return travelMsToCell(appointment_.cell);`

```text
From where this robot is NOW, not from where it was when the appointment
armed: under hybrid it has been chasing since, so the arming position can
be a long way from the robot by the time anyone reads this. That freshness
is load-bearing twice over — it is what makes the departure lead track a
robot drifting away from the meeting, and stale it would misreport the
`travel_sec` covariate, which is read precisely to explain a no-show.
```

## ExploPlannerNode::appointmentDue

### rzv-appointment-due-arrive-on-time

**Leaving in time to arrive at t_meet** — attached to `const long long lead_ms = appointmentLeadMs(appointment_.cell);`

```text
LEAVE IN TIME TO ARRIVE AT t_meet (2026-09-19).

t_meet is a MEETING instant, so the thing that has to land on it is the
arrival. Between 2026-09-18 and this change it was a DEPARTURE instant —
a bare `now >= t_meet` — and the difference is not cosmetic: under that
rule every robot was late by its own drive, the barrier absorbed the
stagger by making whoever arrived first stand and wait, and across the
stopped ts4 cells that was the whole of the observed waiting. Kalhan,
2026-09-19: robots should only agree to meetings they can come to within a
bounded delay.

THE -1 CASE IS WHY THIS WAS REMOVED, and it is handled rather than
designed around. Off the snapshot grid there is no estimate; a robot there
cannot aim, so it leaves at t_meet and is late by its drive — exactly the
old rule, which is the right degradation because it is the behaviour that
needs no estimate. What the 2026-09-18 note objected to was that the
degraded and normal paths "differed only in whether the robot left early".
They still do. That difference is now the point rather than the defect:
the mechanism is armed and driving on both paths, and neither one is the
armed-but-inert state that note was really about.

NO RUNG IS RE-DECIDED HERE. Which occurrence this robot is keeping was
settled once, at arming; this only moves the departure earlier within it.
Re-choosing the rung each tick would ratchet: a robot exploring away from
the cell grows its lead, rolls to a later rung, explores further on the
strength of it, and never attends at all.

Staying due once overdue is preserved (>=, not ==): the trigger is polled
on the PLAN tick and a strict equality would be missed by every robot that
was mid-navigation on the tick the deadline passed.
```

## ExploPlannerNode::appointmentPoint

### rzv-appointment-point-grid-fallback

**Placing the appointment cell safely** — attached to `const CellWorld* w = nullptr;`

```text
THE SNAPSHOT WORLD MAY NEVER HAVE BEEN POPULATED (2026-09-17), and until
today that was a SIGFPE rather than a bad answer: CellGrid::col() is
`id % nx`, nx is 0 on a default-constructed grid, and the header says in
as many words that callers are expected to have screened with valid().
appointmentTravelMs() screens; this did not.

WHAT MADE IT REACHABLE IS THE GATE MOVE IN THIS SAME CHANGE SET. The
mutual-contact gate came off the adopt/echo/commit path so an N>=3 fleet
could converge at all, but refreshRendezvousSnapshot() still runs ONLY
under that gate. Before the move, committing implied a refresh had
happened; after it, a follower whose own heartbeat never landed inside the
team's mutual window adopts three integers off the wire, commits them,
arms an appointment — armAppointment's refusals are clock/spent/invalid
only, and its rendezvous_world_.configured() test wraps a diagnostic, not
the arming — and then dies at the first departure. That is precisely the
follower the move exists to serve, so the crash would have concentrated in
the treated arms and the larger rungs: biased attrition, not downtime.

FALL BACK TO THE LIVE WORLD rather than refusing. The adopted cell was
bounds-checked against cell_world_ when it came off the wire, so that grid
can place it; the two grids are the same geometry (rendezvous_world_ is a
copy of cell_world_), which is why this is a fallback and not a different
answer. The snapshot is still preferred, because it is the grid every
other robot solved against.
```

### rzv-appointment-unplaceable-hold

**Unplaceable cell: hold and latch** — attached to `appointment_unplaceable_ = true;`

```text
Neither grid can place the cell. Standing still is the only safe answer
— the alternative here is undefined behaviour, so it is not a close call
— but standing still is NOT a no-show, and until 2026-09-18 it was
recorded as one. See appointment_unplaceable_: a goal at the robot's own
position is reached on the next tick, so the outcome classifier saw
arrived=true and blamed the peers. The latch makes the row say what
happened.
```

## ExploPlannerNode::closeAppointment

### rzv-close-no-cell-writeoff

**A no-show does not write off the cell** — attached to `appointment_armed_       = false;`

```text
A no-show used to write the cell off here so the outage's NEXT solve would
avoid it. There is no next solve — the pair is agreed while connected and
frozen on separation — and no second arming either. The write-off would
also have been per-robot, so it could only ever have pushed the two ends
apart. See the rendezvous_spent_ / no-show-list note in the P5 state block.
```

## ExploPlannerNode::dispatchReconnect

### reconnect-rec-frozen-per-manoeuvre

**Freezing the peer record per manoeuvre** — attached to `have_reconnect_rec_ = (rec != nullptr);`

```text
Freeze the pair THIS manoeuvre is armed from, so nothing downstream can
re-derive a destination off a packet that arrived mid-manoeuvre. Its one
remaining consumer is the hold escalation's own-anchor fallback: the
destination proper is the agreed appointment cell, which is team-wide and
was never derived from this pair. See reconnect_rec_.
```

### dispatch-p5-timeline-partition

**P5: the appointment and its timeline** — attached to `const bool have_appointment = armAppointment(reason);`

```text
---- P5: the appointment, and the timeline it partitions -------------

Arm it FIRST, because it is what the branches below are arbitrated
against. §3.6.1: chase and appointment do not compete for the manoeuvre,
they compose on a partitioned timeline —

    [ contact lost .... departure deadline .... t_meet + wait ]
      ^--- CHASE owns this ---^--- APPOINTMENT owns this ---^

Exactly one is armed at any instant, so there is no arbitration to get
wrong: the chase runs while the deadline is in the future and is
terminated BY the deadline (doPursue), and the appointment owns
everything after it.

A terminal dispatch (finishOrRendezvous) is exempt from the deferral:
"keep exploring until the deadline" is not an option for a robot whose
exploration is already over, so a terminal manoeuvre departs at once and
the appointment only supplies the DESTINATION.

EXEMPT FROM THE DEFERRAL IS NOT EXEMPT FROM THE CHASE (2026-09-16).
Hybrid's sentence is "do pursuit if rendezvous is not due yet when deemed
necessary", and at a terminal dispatch with the deadline still ahead the
rendezvous is precisely NOT due yet — so it is pursuit's turn, exactly as
it would be for the pursuit arm reaching the same point. Departing here
would mean hybrid never chases at a terminal dispatch at all, while
pursuit always does: not a difference in dose but a whole missing half, in
the dispatch that ends most runs. The deferral is still gone (the robot
does not go back to exploring), and if the chase declines the appointment
departure below happens on this same call, so nothing is delayed.
```

### dispatch-hybrid-mirrors-pursuit

**Hybrid falls back exactly as pursuit** — attached to `if (may_defer) return false;`

```text
The chase declined (record too stale, trail uncoverable), or there was
no record to chase from. HYBRID IS PURSUIT PLUS RENDEZVOUS AND NOTHING
ELSE, so from here it does exactly what the pursuit arm does — the two
lines below are a copy of that branch, deliberately, so the arms cannot
drift apart.

WHY THE MIDPOINT DRIVE IS GONE (2026-09-16). This branch used to fall
through to startReturnTo(meetingPoint(self_pose, peer_pose)) — park at
the midpoint of the two poses at last contact. It was removed from the
RENDEZVOUS arm the same day for two reasons that apply here word for
word: it is a place with NO TIME (nothing tells the peer when to be
there or how long to wait), and it is not actually symmetric (the
floor_won telemetry says both ends picked the midpoint in only 2 of 6
separated pairs). Leaving it in hybrid alone would have been worse than
leaving it in both, because it makes hybrid a THIRD behaviour rather
than the composition of the other two: any hybrid-vs-pursuit or
hybrid-vs-rendezvous contrast would then be confounded by a manoeuvre
neither of those arms can perform, and the factorial reading of the
four-arm design — which is the entire point of running four — would not
hold. The previous justification ("its own mechanism's fallback ladder")
described pursuit's ladder, and pursuit's ladder is the two lines below.

`may_defer` still short-circuits: an appointment whose deadline is ahead
means the robot keeps exploring until it is due rather than parking, and
that deferral is the composition working as specified — pursuit while
the rendezvous is not due yet.
```

### dispatch-hybrid-terminal-chase-declined

**Terminal chase declined: go to appointment** — attached to `if (hybrid_terminal_chase) {`

```text
The terminal chase declined, so the appointment takes the manoeuvre back
— the behaviour this dispatch had before the chase was given its turn.
It must come before the explore fallback: exploration is over at a
terminal dispatch, so falling through to it would send a robot with a
standing appointment back out to a map it has already saturated.
```

### dispatch-pursuit-explore-before-park

**Pure pursuit explores before parking** — attached to `if (pursuitExploreFallback(reason)) return true;`

```text
Pure pursuit has no agreed fallback point by design (that is the A/B
against hybrid). A chase that never started therefore goes back to
exploring while its fallback budget lasts, and only parks here once
that is spent — parking early is the mutual-hold fixed point that cost
a p7modes mission (see pursuit_explore_fallback_).
```

### dispatch-rendezvous-no-fallback

**Rendezvous arm has no fallback drive** — attached to `if (!may_defer) {`

```text
RENDEZVOUS. The arm IS the appointment: an agreed place and an agreed time,
or nothing. If `may_defer` is true there is a standing appointment whose
deadline is still ahead, and the robot keeps exploring until it is due —
that deferral is the whole treatment. If it is false, armAppointment
refused, and this arm has nothing left to do.

WHY THERE IS NO FALLBACK HERE ANY MORE (2026-09-16). Until today a refusal
fell through to an immediate drive to the midpoint of the two poses at last
contact, justified on the grounds that each end computes that midpoint from
the mirror image of the same pose pair and so both agree. Two things were
wrong with keeping it:

  * It has a place and NO TIME. Nothing tells the peer when to be there or
    for how long to wait, which is exactly the property this arm exists to
    test. Pre-P5 that showed up as a median wait of 240 s — the cap, to the
    second — and 16 no-shows in 21 n3 appointments.
  * It is not actually symmetric. The `floor_won` telemetry says so: in
    only 2 of 6 separated pairs did BOTH ends pick the midpoint floor, so
    the shared-by-construction argument was already failing in the data.

So a refusal now means the robot carries on exploring and the manoeuvre
simply does not happen. That is the honest reading of "meet at a place and
a time both robots agreed on": if no pair was committed while the team was
whole, there is no appointment to keep, and driving somewhere plausible
instead would put the old asymmetric behaviour back inside the arm that is
supposed to be measuring its replacement.

The refusal is logged (armAppointment always emits a `rendezvous_agreed`
row, refusals included), so "the arm did nothing here" is visible offline
rather than inferred from an absence. HYBRID no longer has a midpoint tail
either: its branch above now ends in pursuit's own fallback ladder, so the
midpoint drive exists in no arm at all.
```

## ExploPlannerNode::releaseConfirmed

### release-confirm-flicker-dwell

**Flicker guard on manoeuvre release** — attached to `bool ExploPlannerNode::releaseConfirmed(bool eligible) {`

```text
Flicker guard: a single live claim (one intent inside the 5 s TTL) is
enough to read the team complete for a tick, release a manoeuvre and reset
the silence clock — crediting a "reconnection" on a range-edge flicker that
drained no map deltas. With a positive confirm window the release condition
must hold continuously that long. The dwell is cheap where it runs: PURSUE
keeps driving (budget still ticking), the barriers keep waiting.

THE DWELL ITSELF LIVES IN planner_util (generation 23). This is the node's
window over it — the pair of state words and the node clock — and nothing
more. It was an inline state machine here until 2026-09-18, which meant the
package's one flicker rule was reachable only by a source scan, and meant the
two OTHER sites that act on the team coming back (appointment supersede,
rendezvous_spent_ release) each shipped without any dwell at all rather than
with a copy of this one. All three now call the same tested function.

THIS PAIR OF STATE WORDS IS THE BARRIER'S ALONE. The other sites — the
appointment supersede, the rendezvous_spent_ release, and since generation
25 the walker conversion in doReturnNav — share a second pair
(team_back_ok_*, written once per heartbeat) because they ask one identical
question, teamSettled; this one asks a different question — manoeuvre
release eligibility — so folding it in would make one caller's answer move
on the other's evidence. That is the distinction the member declaration
draws, and it is by predicate, not by call site.
```

## ExploPlannerNode::releaseHeld

### release-held-read-only-twin

**Read-only twin of releaseConfirmed** — attached to `bool ExploPlannerNode::releaseHeld(bool eligible) const {`

```text
THE READ-ONLY HALF OF THE SAME QUESTION (2026-09-18), for callers that need
to know what the barrier decided without BEING the barrier.

releaseConfirmed is a state machine: it arms the dwell on its first eligible
tick and answers false, and it disarms on the first ineligible one. Both
writes are the guard working. So a second caller — the outcome classifier —
cannot simply call it to find out whether the manoeuvre ended in a real
reunion: asking the question would advance or reset the very window being
asked about, and the barrier would then release a confirm-window late or
early depending on who ticked first.

This returns the same boolean from the same two members without touching
either. Keeping the two answers in step is no longer a discipline a reader
has to maintain by hand: both now delegate to planner_util's dwellConfirmed /
dwellHeld pair, which are adjacent in that file and pinned against each other
by executable tests (test_planner_util). What this wrapper must preserve is
only that it passes the SAME three inputs as releaseConfirmed does.
```

## ExploPlannerNode::standDownExploitation

### barrier-stand-down-exploitation

**Standing exploitation down at barrier entry** — attached to `void ExploPlannerNode::standDownExploitation() {`

```text
Shared barrier-entry stand-down. The rendezvous/pursuit barrier is HARD:
nothing preempts it. doNavigate() interrupts an exploration hop the moment a
target arrives, but the RETURN/PURSUE states deliberately do NOT check
target_queue_ — with rendezvous_max_wait_sec <= 0 (wait forever, the
default) a robot that serviced trees on the way would leave its teammate
blocked at the barrier indefinitely.

Stand the queue down rather than destroying it: the ACTIVE target is
demoted to PENDING and the phase reset to EXPLORE, so the exploit claim
stops being broadcast (the fresh non-exploit intent the caller publishes
overwrites current_intent_msg_, clearing exploit/target_id/dwelled_mask and
staged — peers must not merge dwell credit from a robot that is driving
home, nor hold a vantage barrier open for one) and
doPlan() picks the target back up once the barrier releases. Leaving it
ACTIVE mattered once the step-budget path started routing through here:
that path can fire mid-exploitation, unlike coverage saturation.
```

### barrier-stand-down-stops-exploit-timer

**Stopping the exploit give-up timer** — attached to `exploit_target_timing_ = false;`

```text
Stop the give-up timer along with the queue. The demoted target comes
back with the SAME id after the barrier, so doExploitPlan's re-latch
check ("different id?") would keep the old start time — the whole
return drive plus the barrier wait would count against the target and
it would re-activate already past exploit_target_timeout_sec, closing
PARTIAL without a single new dwell attempt.
```

## ExploPlannerNode::startReturnTo

### start-return-to-barrier-drive

**Arming the drive to a barrier** — attached to `void ExploPlannerNode::startReturnTo(const Eigen::Vector3f& dest,`

```text
Arm the drive to a barrier destination — the agreed appointment cell in
every arm that has one, or this robot's own last-connected anchor as the
hold escalation's fallback when it does not — reusing the NAVIGATE
smart-timeout + arrival test. A presence intent is published (and re-sent by
the heartbeat, which fires in the RETURN states) so teammates arriving
later count us at the barrier — without it two robots waiting at their own
anchors would never see each other and would deadlock.
```

### start-return-appointment-manoeuvre-latch

**Latching whether a leg serves an appointment** — attached to `appointment_manoeuvre_ = (std::strcmp(what, "appointment") == 0);`

```text
IS THIS LEG SERVING AN APPOINTMENT? (2026-09-18)

Every destination this function is ever given comes from one of two
places: appointmentPoint(), or last_connected_anchor_. `what` already
names which at every call site, and it is the only input that survives
the tail-call into RETURN_NAV — so it is what the latch is derived from,
here, once, instead of at each departure site where a future path could
forget it. (No count of the call sites is written down on purpose: a
number in a comment is a fact that goes stale silently, and this one was
already wrong once.)

The latch is NOT appointment_armed_ and must not be folded into it.
transitionTo(RETURN_NAV) below runs the appointment outcome block, which
can legitimately close (and disarm) the appointment on this very tick —
the departure tick. Two things then read the wrong answer for the whole
manoeuvre: the barrier's wait_cap falls back to the 240 s PURSUIT cap
instead of the unbounded appointment patience, so "be there until all
robots are connected" quietly becomes "be there for four minutes"; and
the arrival stamp has nothing left to set, so a no-show cannot be told
from an unreachable peer. This flag answers "the manoeuvre I am in was
started to keep an appointment", which stays true until the manoeuvre
ends regardless of what happens to the appointment record.
```

### start-return-finished-peer-wait

**Finished-peer wait survives a new leg** — attached to `if (!appointment_manoeuvre_) {`

```text
THE WAIT FOR A FINISHED PEER IS NOT RESTARTED BY A NEW LEG, only dropped
by a leg that is not an appointment's. A resumed leg (the settle lapsing
short of the cell) re-dispatches from the barrier itself, so a restart
here would let a robot cycling stop-short/resume wait forever. One stamp
per appointment manoeuvre, like the latched hold's, and the same sizing
argument covers a rolled rung.
```

### reconnect-dispatch-action-label

**Reconnect dispatch action label** — attached to `refreshDispatchContext();`

```text
The action string is the analysis' name for WHERE this leg went, and the
two destinations above are the two names. "meeting_point" is NOT one of
them any more: it was the midpoint construction, whose last caller went
with meetingPoint() in generation 19 (see planner_util.hpp for why), and
deriving it from a strcmp that can no longer match left every appointment
drive logged as "anchor_return" — an agreed cell reported as a retreat to
the robot's own last-connected pose, which is a different manoeuvre with a
different failure mode. Analyses read this field; it has to be true.
```

## ExploPlannerNode::armManoeuvreLegBudget

### manoeuvre-leg-budget-exempt-cap

**Manoeuvre leg budget and its cap** — attached to `void ExploPlannerNode::armManoeuvreLegBudget(float dist) {`

```text
Manoeuvre legs are exempt from nav_max_timeout_sec: the smart-timeout
ceiling was sized for exploration hops, and a cross-world return clipped to
it dies tens of metres short of a destination whose whole value is ARRIVING
(the connected geometry). Distance-true budget, floor kept; the no-progress
window remains the stuck-robot watchdog.

Exempt from THAT ceiling, but not unbounded — reconnect_nav_max_sec stops a
leg outspending the barrier it drives toward. The 20 s/m model rate is 3x
conservative, so this binds only on a leg failing SLOWLY; one failing fast
still exits on the 15 s no-progress window, the primary guard, unchanged.
```

## ExploPlannerNode::doReturnNav

### return-nav-appointment-settle-join

**Why appointments join the barrier en route** — attached to `void ExploPlannerNode::doReturnNav() {`

```text
Drive toward the destination — the agreed appointment cell, or the robot's
own last-connected anchor. On arrival (or if the destination turns out
unreachable) hand off to RETURN_SYNC to wait for the team from wherever we
ended up.

AN APPOINTMENT IS NOT RELEASED EN ROUTE (2026-09-18), BUT SINCE GENERATION
25 IT CAN JOIN THE BARRIER EN ROUTE. For an anchor return the drive is a
means and reconnecting is the end, so reconnecting en route makes the rest
of the drive pointless and this function re-plans on the spot. For an
appointment, releasing straight to PLAN from here broke the directive in
two places at once —

  "be there until all robots are connected THEN WAIT FOR A WHILE MORE so
   that map is updated then begin explore"

— because the settle hold that serves that sentence lives in RETURN_SYNC,
which an en-route RELEASE skips entirely. In a three-robot outage the first
robot to see the team complete would leave immediately while the others were
still driving, so the one hold whose whole purpose is to let the merged map
propagate was paid by nobody. The generation-25 answer is neither of the
gen-24 options (release here, or drive to the ring no matter what): when the
team has SETTLED for the shared confirm window, the walker stops driving and
enters RETURN_SYNC from where it stands — the settle hold is still paid,
the release still happens at the one site that owns it, and the last metres
to the cell are not driven because the thing they were for has already
happened. Gen 24 held that finishing the drive was "the literal reading of
an exact place"; the ts4 24b N=2 hybrid cell priced that literalism: a robot
2.4 m outside a 1.5 m ring, crawling at 0.06 m/s against meeting-point
clutter for 350 s with the link up 90.5% and the maps already merged, its
appointment_inbound bit holding two peers' barriers open to the censor.

THE WATCHDOGS BELOW DO NOT CLOSE THAT CASE, which is why the conversion is
not redundant with them: the proximity-hold release refunds held time to the
nav budget and restarts the progress window (see doProximityHold), each
individually correct, so a hold/resume cycle around a parked partner defeats
both indefinitely — the same crawl cycled them for the whole 350 s. For the
genuinely-unreachable-cell case with the team still apart they remain the
exits, and they still hand off to RETURN_SYNC rather than PLAN.
```

### return-nav-release-guard-order

**Guarding releaseConfirmed on the en-route path** — attached to `if (!appointment_manoeuvre_ && releaseConfirmed(manoeuvreReleaseEligible(active))) {`

```text
Guard BEFORE the call, not after: releaseConfirmed is a state machine that
arms and disarms its dwell as a side effect, and running it on a path whose
answer is discarded would advance a window nobody is reading.

manoeuvreReleaseEligible is DELIBERATELY REDUNDANT with the guard beside
it: && short-circuits, so on this path it can only ever evaluate its
teamComplete branch. Written as the shared helper anyway so that the five
sites read as one predicate and a future change to the guard cannot leave
an appointment releasing here on the wrong half of it.
```

### return-nav-measure-return-dest

**Measuring against the manoeuvre destination** — attached to `const auto robot_pos = latest_pos_;`

```text
return_dest_, NOT current_goal_.position: every test below asks how far
this robot is from the place the MANOEUVRE is for, and current_goal_ only
answers that while nothing has borrowed the goal to steer around something
— see the member for what measuring against the borrowed goal cost. The two
are equal for the whole of an ordinary drive, so this changes no behaviour
outside an escape leg; what it changes is that the borrowed case can no
longer produce a wrong answer at all.
```

### return-nav-arrival-stop-and-stamp

**Stopping and stamping appointment arrival** — attached to `if (appointment_manoeuvre_ && !appointment_arrived_) {`

```text
RETURN_SYNC assumes a stationary robot, but arrival-by-tolerance lands
BEFORE the navigator finishes its own goal: without an explicit stop the
controller keeps driving to the exact goal pose underneath the barrier
— and underneath whatever state the release transitions into next.
Same rationale as the release path above.
P5: stamp the arrival so the outcome event can say whether a failed
meeting was a coordination failure (both arrived, nobody was there) or
a navigation one (this robot never got there). Without the distinction
a no-show count is uninterpretable — see RendezvousOutcomeEvent.

Keyed on appointment_manoeuvre_, not on (armed && departed): the
appointment can be closed — and disarmed — on the very tick the drive
starts, which left the arrival of a drive that did reach the cell
unrecordable. What is being asserted here is a fact about THIS LEG.
```

### return-nav-settle-conversion

**The settle conversion en route** — attached to `if (appointment_manoeuvre_ &&`

```text
THE GENERATION-25 CONVERSION (see the function header): an appointment
walker whose team has settled joins the barrier from where it stands.

team_back_ok_*, NOT release_ok_*: this site asks the supersede's question
— "is the outage over?" — not the barrier's. The barrier's own predicate
(manoeuvreReleaseEligible) is unsatisfiable from here by construction:
this robot IS the inbound peer its !peerInboundToAppointment() term is
waiting out. And the release_ok_ window cannot serve either, because its
RETURN_NAV writer above is guarded off for appointments, so that window
is FROZEN on this path — reading it would fire on one stale sample, the
exact failure the dwell exists to prevent.

dwellHeld, NOT dwellConfirmed: heartbeatTick owns this window's clock
(one writer per window; it ticks unconditionally at 1 Hz). This site only
reads the answer, conjoined on its own fresh presence count — the same
pattern as the P5 supersede.

appointment_arrived_ stays FALSE on this path, deliberately: the ring
above is now the diagnostic that says whether the last metres were
actually driven, so the outcome row can tell "met without reaching the
cell" from "never got there while the team stayed apart".
```

### return-nav-escape-leg-termination

**Escape leg termination in RETURN_NAV** — attached to `if (appointment_escape_active_) {`

```text
ESCAPE-LEG TERMINATION. Bookkeeping, not a guard: the tests above measure
against return_dest_ and are already true during a leg, so this only
decides when the DETOUR is over. Placed after them deliberately — a robot
that reaches the cell, or whose team settles, while steering around a stall
should take that exit now rather than finish a detour it no longer needs.

The leg ends on arrival at its own target or on its own cap, and
resumeAppointmentDrive puts current_goal_ back on the cell. Note the two
radii are different quantities: kEscapeArriveM is "the detour is done",
reconnect_arrive_tol_m_ above is "I am at the meeting".
```

### return-nav-watchdog-exits

**RETURN_NAV budget and no-progress exits** — attached to `if (elapsed > nav_budget_sec_) {`

```text
Distance-budgeted timeout / no-progress watchdog: if the destination
can't be reached, wait for the team from here rather than looping on the
drive (we're at least closer to comms than where exploration stranded
us). RETURN_SYNC assumes a stationary robot, so the failed drive must be
stopped explicitly — the meeting point in particular is a synthetic
coordinate that can be unreachable, making these exits routine in hybrid.

"CLOSER TO COMMS" IS AN ANCHOR-RETURN ARGUMENT (2026-09-20), and it does not
survive being applied to an appointment. An anchor return drives at a pose
where the link once worked, so any progress toward it is progress toward the
radio; an appointment drives at the one place the OTHER robots will be, and
stopping short of it is not a shorter version of arriving — it is being
somewhere nobody is going. appointmentLegWatchdog is where that distinction
is made, and it is asked before both exits.
```

## ExploPlannerNode::appointmentLegWatchdog

### appointment-leg-watchdog-ladder

**Failed appointment drive escalation ladder** — attached to `bool ExploPlannerNode::appointmentLegWatchdog(const char* what_failed,`

```text
A FAILED APPOINTMENT DRIVE IS NOT AN ARRIVAL (2026-09-20).

The watchdogs above end a leg that is not getting anywhere. For an anchor
return, ending it at the barrier is right: the destination was this robot's
own last-connected pose, nobody else was going there, and standing still is
what the manoeuvre wanted. For an APPOINTMENT it is a false claim with
teeth, because the barrier is a presence count and the wait on it is
unbounded by design (rendezvous_appointment_wait_sec = 0.0). A robot that
joins it from outside the cell contributes a presence nobody can observe —
it is not where the others are — and then waits forever for a presence that
is, in turn, waiting for it. Two stationary robots cannot heal a link, so
the pair is wedged until the harness kills the run. That is the ts4 gen-30
seed8 cell in one sentence.

A WATCHDOG FIRE IS NOT A REACHABILITY VERDICT, which is what makes giving up
on the first one wrong. Neither watchdog can see WHY the drive stopped:
simple_nav_3d has no failure-reporting channel at all, so "the budget ran
out" and "no progress in 15 s" are the only two things the planner is ever
told, and both are equally consistent with a cell that is unreachable and
with a platform wedged against one trunk on an otherwise open approach. The
seed8 robot was the second case — it closed 3.8 m and then stopped dead, 40 m
from a cell its partner was standing in.

SO THE LEG ESCALATES INSTEAD OF SURRENDERING, and the ladder is the one the
mission return already uses (homeWatchdogFire): drive briefly to a pose the
approach is different from, then aim at the SAME destination again.
pickEscapeTarget is reused unchanged — a breadcrumb 1.5-6.0 m back, else a
pose 2.5 m behind rotated 0/+-60 deg on successive tries — because its whole
design is about the two front-arc freeze modes, and it is goal-agnostic. The
homing ladder's resend rung is deliberately SKIPPED: republishing the same
pose is the one rung whose own header says it cannot unstick a stuck
platform, and RETURN_NAV has been republishing that goal every 5 s since the
leg started, so it is a rung already spent. Retrace does not transfer either
— it is a path back along a trail this robot walked, and no such trail leads
to a cell it has never been to.

THE ROLL IS THE TERMINAL RUNG, NOT THE FIRST ANSWER. It cannot simply be
deleted: an appointment whose t_meet_ms is already past is due on the very
next PLAN tick, so a robot released to PLAN on a rung it has missed is
re-dispatched onto the drive that just failed, forever. Once the ladder is
spent the honest reading IS that this rung is lost, and the lattice already
has the exit — armAppointment rolls a robot whose PREDICTED drive cannot make
a rung onto a later one and lets it explore in the meantime ("a fork here is
priced, not prevented", paid for by the barrier's patience). A drive that was
affordable when priced and failed anyway is the same robot missing the same
rung for the same reason, discovered late, so it takes the same exit.

SO IT TAKES armAppointment's FLOOR, NOT A BARE `now`. Rolling past the
present instant alone is only half the rule, and the missing half is the same
defect one lattice step later: the next rung can be a millisecond away, this
robot's own drive to the cell prices in the hundreds of seconds, and
appointmentDue() is therefore true on the very next PLAN tick. That is a
re-dispatch onto the drive that just failed, a fresh ladder, another roll,
for the rest of the run — no exploration between the cycles, and
appointment_inbound chattering peers' barriers open throughout, which is
worse than the wedge it replaced, because the wedge at least stopped. The
appointment_leg rows this function writes are what make such a cycle
countable rather than merely loud. arrivalShortfallSec is what
makes "and exploring until then" true: the same helper, the same lateness
budget and the same clamp at zero as the arming path, so the rung this lands
on is one the robot can still BE AT, and a robot that can make the nearest
rung still takes it.

MANOEUVRE, NOT RECORD, IS WHAT THIS GATES ON. The barrier a fall-through
hands the robot to waits without limit on appointment_manoeuvre_ alone, and
that is deliberate — see the wait_cap note in doReturnSync. The appointment
RECORD, meanwhile, can be closed and disarmed underneath a live manoeuvre:
transitionTo's appointment classifier is gated on `appointment_armed_` and
carries no `!manoeuvre` term, so any transition taken while the team reads
settled closes it while the drive continues — the departure tick itself, or a
proximity hold taken and released mid-leg. Gating this function on the record
therefore left exactly one door open into the unbounded barrier from 40 m
out: the seed8 wedge, through the one state the 2026-09-18 wait_cap fix
exists to name. The distance question is asked of every appointment leg; only
the ROLL needs a record to roll, and a leg that has none still exits to PLAN,
which clears appointment_manoeuvre_ and with it the patience that made the
wedge possible.

UNDEPARTED, NOT UNARMED, and the ordering matters. transitionTo's P5 block
closes an appointment that is `departed && !manoeuvre`, so leaving the flag
set would bank this as "unreachable" and delete the meeting on the way out —
the robot would then have no appointment at all and nothing to come back for.
Clearing it first puts the appointment back in the DEFERRED state the block
explicitly preserves, which is also the state doPlan's departure gate reads.
The re-agreement path is untouched: this moves the occurrence this robot
aims at along the agreed recurrence, it does not derive a new pair. An escape
rung needs none of this — it stays inside RETURN_NAV, which is a `manoeuvre`
state, so the block cannot fire on it at all.
```

### appointment-escape-log-uses-dest

**Why the escape log uses the destination** — attached to `RCLCPP_WARN(get_logger(),`

```text
return_dest_, not appointment_.cell: the record can be closed underneath a
live leg (see the header), and a WARN that prints "cell -1" on the one
path the reader most needs to follow is worse than no cell id at all. The
destination is true on every path and it is what startReturnTo's own
dispatch line names.
```

### appointment-escape-no-brake

**No brake before the escape goal** — attached to `publishGoal(current_goal_);`

```text
No brake first, unlike the homing ladder: that one publishes the escape
goal a tick LATER, so it needs abandonNavGoal's own-pose goal to stop the
platform in between. Here the replacement goes out in the same tick, and
a new goal_pose is what supersedes the old one — a brake published
alongside it would only be overwritten before the controller saw it.
```

### appointment-roll-strict-floor

**Rolling strictly forward past now** — attached to `const long long now_ms =`

```text
Strictly forward, and past NOW as well as past the rung being abandoned: a
leg that failed slowly can outlive its own meeting, and a rung already in
the past is due the instant it is set. The +1 ms is what makes it strict;
nextAgreedOccurrence keeps an instant that is merely >= its floor. The
shortfall raises the floor again by however much this robot's own drive
is short of the lateness budget, which is the half of the rule that stops
the roll landing on a rung it cannot make either — see the header. A cell
it cannot price answers -1, which arrivalShortfallSec reads as "no
estimate" and returns 0.0 for: the nearest rung, as before.
```

### appointment-roll-lands-later

**Why the roll always lands later** — attached to `appointment_.t_meet_ms =`

```text
Strictly later than the rung being abandoned, by construction: the floor
is above it and interval_ms is positive by valid(), so this takes
nextAgreedOccurrence's ceiling-division path and lands at least one whole
interval on. A guard against the other case stood here. It could not fire
— valid() had already excluded the non-recurrence it named — and what it
did on the way was `return false`, i.e. the fall-through into the
unbounded barrier that this function exists to close.
```

### appointment-unreached-latched-ends

**Latched robot ends run when unreached** — attached to `if (coverage_latched_) {`

```text
A LATCHED ROBOT DOES NOT GO BACK TO EXPLORING (2026-09-21) — the same rule
as the barrier release below, reached by a different road. That road only
opened this generation: before keepAppointmentOnFinish a finished robot
drove home rather than to the meeting, so no leg to the agreed cell could
be carrying one and every robot this rung handed to PLAN still had its
ending in front of it. One that has already spent the coverage ending does
not, and first-touch latching will not give it another, so the exit below
would leave nothing but max_steps_ to stop it — the second
exploration_complete, in the treated arms only, that the release's comment
sets out in full.

The rolled rung above is written and logged before this, deliberately: the
roll is what the REST of the team is still keeping, and a rung this robot
abandoned is not a rung the appointment loses. What ends here is this
robot's run, not the meeting.
```

## ExploPlannerNode::resumeAppointmentDrive

### appointment-resume-same-cell

**Resuming the drive to the same cell** — attached to `void ExploPlannerNode::resumeAppointmentDrive(const char* why) {`

```text
Back onto the road to the SAME cell. The destination is return_dest_, which
the detour left alone, and never a fresh appointmentPoint(): the manoeuvre
exists to change the approach, not the meeting, and the recompute answers
latest_pos_ for a cell it cannot place — which would put the goal under the
robot's own feet. No state change — RETURN_NAV is already the state, and the
leg it is driving is the one that was interrupted.
```

### appointment-resume-rearm-budget

**Re-arming budgets on a resumed leg** — attached to `state_enter_time_ = this->now();`

```text
A resumed leg is a NEW leg and must be judged as one. The budget and the
window still in force are the ones that just fired, so without this the
next tick spends another rung immediately and the whole ladder burns in
three ticks without the robot driving anywhere. armManoeuvreLegBudget reads
state_enter_time_ as the leg's start and there is no transition here to
stamp it, so it is stamped directly — the same thing the proximity-hold
resume does, for the same reason.
```

## ExploPlannerNode::logAppointmentLegRow

### appointment-leg-row-instrument

**The appointment leg event row** — attached to `void ExploPlannerNode::logAppointmentLegRow(const char* kind,`

```text
The ladder's only instrument. Until this existed the rungs were RCLCPP_WARN
text, which is the same as unmeasured: nothing counted escapes per arm, and a
robot cycling give-up -> roll -> re-depart looked in the tables exactly like
one making ordinary dispatches — quieter, in fact, since it never reached a
rendezvous_outcome. The cell id comes from the RECORD and the position from
return_dest_ on purpose: the record can be closed underneath a live leg while
the destination stays true, and a row that carried only the cell would print
-1 on precisely the give-up path a reader is there to follow.
```

## ExploPlannerNode::doReturnSync

### return-sync-overview

**RETURN_SYNC barrier overview** — attached to `void ExploPlannerNode::doReturnSync() {`

```text
Hold at the anchor until the whole team is back in comms, then re-plan. With
rendezvous_max_wait_sec <= 0 the wait is unbounded (STAY until all connected,
the requested default); a positive value is the field escape hatch. The
heartbeat keeps broadcasting our presence throughout so arriving peers count
us and release their own barriers.
```

### return-sync-finished-peer-log

**Logging the finished-peer hold** — attached to `if (appointment_manoeuvre_ && finished_peer_wait_start_sec_ >= 0.0) {`

```text
THE FINISHED-PEER HOLD, NAMED IN THE LOG, for the contagion line's reason
(below): without it a barrier holding for a peer it cannot hear reads as a
barrier that will not release. Its END is named too, once, because a bound
that expired is a meeting that did not happen, and the analysis must be
able to tell it from one that did. HERE, ABOVE THE RELEASE GATE, because
the expiry is what lets the gate open, and the settle that follows returns
before any line further down is reached.
```

### return-sync-five-site-barrier

**The five-site release barrier** — attached to `if (releaseConfirmed(manoeuvreReleaseEligible(active))) {`

```text
THE BARRIER OF THE FIVE-SITE RULE (generation 23). For an APPOINTMENT this
is (teamSettled OR every expected peer reachable in the comms closure —
the generation-27 door) AND nobody still driving here. The mesh half is
the condition the appointment armed on; the door is for the gather that
one occluded chord keeps mesh-false forever, and it cannot hand the robot
back for an instant re-arm because rendezvous_spent_ still clears only on
the strict dwelt predicate. For a midrun or terminal reconnect
it stays teamComplete: those are this robot's own business and a distant
robot's break must not hold a pair that has already reconnected. See
manoeuvreReleaseEligible().

WHY THE SECOND TERM IS THERE, because the first one alone read as though it
were enough. This block used to argue that "at the meeting point the robots
are within a couple of metres of each other, so every pair should be up by
construction — that is what meeting at one place is FOR". True of robots AT
the meeting point, and the release never required them to be there: it is a
comms test, the radio reaches tens of metres, and the drive it is paired
with only ends on arrival. So the first robot to arrive released as soon as
its partner came into range and left with most of that partner's walk still
to go, and the partner — which may not release en route, the cell being the
agreed thing — finished the walk to an empty cell and waited there alone.
The gen-23 smoke measured 36 s of settle against 272 s of standing.

THE HANG THIS ADMITTED WAS THEN MEASURED (gen-26 N=3 smoke: three robots
gathered at the agreed cell, one trunk on one 8 m chord, all parked in
RETURN_SYNC from ~350 s to the 660 s cap, zero outcome rows) and
generation 27 narrows it. The appointment wait cap is
rendezvous_appointment_wait_sec, which the campaign leaves at 0 =
UNBOUNDED, and through generation 26 "gathers but cannot close every
pair" waited here until the run's duration cap; the reachable door now
releases that shape after the settle. What still waits unbounded is a
team missing a peer from the CLOSURE itself — genuinely absent, still
driving in, or dark to every robot present — which is the vigil the
unbounded cap is FOR. The
finished-peer exemption in peerReportsTeamBreak() removes the one case that
could hold it open indefinitely. The claim that stood here — that the
inbound term ends at the nav budget whether or not the cell was reachable —
was FALSIFIED by the ts4 24b N=2 hybrid cell: the proximity-hold release
refunds held time and restarts the progress window (see doProximityHold),
so a walker crawling against meeting-point clutter held the bit, and this
barrier, for 350 s to the censor with the team connected the whole time.
Generation 25 closes that path at the source: a walker whose team has
settled for the confirm window converts to RETURN_SYNC (see doReturnNav),
which clears appointment_inbound on the next heartbeat — so the inbound
term can now outlive the confirm window only while the team is genuinely
still apart, which is the case it was written for.
```

### return-sync-settle-hold

**The settle hold after release** — attached to `const auto settle_now = this->now();`

```text
THE SETTLE HOLD (2026-09-18). The team being connected is not the same
event as the maps having merged, and releasing on the first is releasing
before the thing the meeting was for has happened. dscovox fusion is
radio-gated: peer voxels cross the comms emulator at a bounded rate once
the link is up, so "full team connected" marks the START of the exchange,
not its end. A barrier that released on that tick paid the entire drive
to the meeting point and then re-planned against a map that had received
almost none of the partner's coverage — the cost of the manoeuvre with a
fraction of its benefit, which would read in the analysis as the
rendezvous arm simply being expensive.

Held in RETURN_SYNC rather than as a sleep so the robot keeps
heartbeating and keeps counting peers throughout: if a peer drops during
the settle the completeness test below fails on the next tick and the
barrier correctly goes back to waiting instead of releasing into a team
that has already come apart again.

ONLY AN APPOINTMENT SETTLES (2026-09-18). The hold is the last clause of
the rendezvous rule — meet at the agreed cell, wait for everyone, "then
wait for a while more so that map is updated" — so it belongs to the
appointment and to nothing else. Charging it to every barrier put it on
PURSUIT's reunions and on the terminal anchor hold too, which costs those
arms rendezvous_settle_sec per manoeuvre for a mechanism they are the
control for: the four-arm design reads as a factorial, and a treatment
leaking into the control arm is exactly the leak that makes it stop
reading as one. Every non-appointment barrier is now treated identically
(no settle) in all four arms, so what is left is a difference between
arms rather than a difference between barriers.
NO CLOCK CONDITION IN THE GATE (2026-09-18). It used to carry
`&& t_now >= 0.0` against missionElapsed(), which meant an unresolvable
mission clock SKIPPED the settle and released the barrier on the spot.
See the member: the settle is a duration, it is measured on the sim
clock this state already reads for everything else, and there is no
longer a state in which it can be silently not applied.
```

### return-sync-rejoin-log-count

**Rejoin log count and grep prefix** — attached to `const int present = std::max(active, reachablePeerCount());`

```text
The mesh count for a mesh release, the closure count for a door release
(generation 27): a correct door release must not log "1/2".
sim/manoeuvre_events.py's RE_REJOIN greps this line — it matches both
this text and its pre-27 "full team connected" form — so the prefix
"Rendezvous: team reachable" is load-bearing; the jsonl rows carry the
raw mesh count.
```

### return-sync-hold-trigger

**What ends the settle hold** — attached to `if (!rendezvous_drain_released_) {`

```text
THE TRIGGER, AND ONLY THE TRIGGER. Everything below this block — the
re-agreement wait, the exchange report, the routing — is unchanged;
what moves is the question that ends the hold. On the clock path it is
"has 30 s passed", which is a guess about a duration nobody measured.
On the drain path it is "has every peer this robot believes is here
delivered what it had", which is the thing the hold exists for.
```

### return-sync-drain-predicate

**Where the drain predicate lives** — attached to `const DrainReading drain = stepDrainRelease(`

```text
The predicate itself is stepDrainRelease (exchange_drain.cpp):
a level against the hold-start baseline, then a rate over a
tumbling window, over every peer believed present. It lives in
the library so test-plan 7 and 8 can run it; this block keeps the
state, the log lines and the latch.
```

### return-sync-unfinished-exchange

**Naming an unfinished exchange** — attached to `RCLCPP_WARN(get_logger(),`

```text
A DIFFERENT OUTCOME FROM A RELEASE, AND NAMED DIFFERENTLY. The
team leaves either way, but "the exchange finished" and "the
exchange never happened and we gave up waiting" are not the same
event, and a log that calls both a release puts the ambiguity
this whole generation removes back in as a reporting bug.
```

### return-sync-reagree-on-cell

**Re-agreeing while still on the cell** — attached to `if (!rendezvous_reagree_waiting_) {`

```text
AND THEN IT WAITS FOR THE NEXT APPOINTMENT (generation 29). The maps
have merged; the rule's last clause is that the team agrees the next
place and time and only THEN resumes exploring. Requested here rather
than at the release below so the request is made while the fleet is
still standing on the cell: the derive is the proposer's and the commit
needs every peer's echo, and both of those are cheapest — and least
likely to be lost to a link break — before anyone drives away.
```

### return-sync-reagree-commit-test

**Re-agreement commit test and bound** — attached to `if (settled_sec - rendezvous_settle_sec_ < kRendezvousReagreeWaitSec) {`

```text
THE COMMIT RULE'S OWN OUTPUT is the release test, not the request
flag: rendezvous_agreed_ only moves when every peer has echoed the
same triple, and it reads the same on the proposer and on a follower.

BOUNDED, AND THE EXPIRY IS NOT A FAILURE TO RECOVER FROM. A schedule
rolls forward rather than expiring (see nextAgreedOccurrence), so a
team that leaves on the pair it already has still has a place and a
time — an older place, sited by a staler map. Standing here longer
than this trades that for a worse thing: an appointment arm that
cannot explore because its handshake did not land.
```

### return-sync-exchange-report

**What the exchange report measures** — attached to `if (rendezvous_exchange_.taken) {`

```text
WHAT THE EXCHANGE ACTUALLY MOVED, not that it moved something. The line
this replaces said "re-planning against merged map" on every release
whether or not one byte had crossed, which is the shape of claim that
survives a whole campaign unfalsified because nothing ever measures it.
Three quantities because they fail differently: the cell delta is the
only one a peer alone can move (this robot's own driving cannot change a
status a peer's census set), the census hash answers "did the shared
belief move at all" for a merge whose applied count is zero because the
peer agreed, and the voxel delta is the dense map the planner actually
costs tours over — which includes this robot's own sensing while it
stood there, so it is the loosest of the three and is reported as such.

A BASELINE IS NOT ALWAYS THERE: with the settle off there is no interval
to difference, and printing a zero for that would report a silent
exchange where there was only an unmeasured one.

THE INTERVAL IS THE WHOLE MEETING, both stages of it: the baseline is
stamped when the team becomes reachable and differenced here, after the
re-agreement wait above, so peer cells that land while the team is
standing there waiting for the next pair count as what the meeting
bought. `settled_sec` is therefore the total time on the cell and is
reported as "held", not as the settle parameter.
```

### return-sync-per-peer-deltas

**Per-peer voxel delta report** — attached to `if (rendezvous_exchange_.peer_deltas.size() == peer_fusion_deltas_.size()`

```text
WHICH PEER, not how much in total. The three numbers above are all
team-wide, so a meeting where one partner delivered everything and the
other delivered nothing reads exactly like a meeting where both
delivered half — and at N>=3 that is the difference between a hold
that is over and a hold that is waiting on a robot it cannot hear.
This line is what fixes the drain-release thresholds offline: it is
the per-peer arrival distribution the design says must be measured
before anything is allowed to release on it.

THREE OUTCOMES, KEPT APART. No counters at all is unmeasured and says
so; counters present with a zero delta is a peer that did not speak
during the whole meeting, which is a finding, not a missing reading.
```

### return-sync-empty-exchange-not-gate

**An empty exchange is not a gate** — attached to `if (merged_cells == 0 && !census_moved && gained_voxels <= 0.0) {`

```text
NOT A GATE, AND DELIBERATELY SO. An exchange that moved nothing is a
result — two robots that covered no ground the other needed — and
refusing to re-site the meeting on it would leave the team on a pair
derived before the FIRST outage for the rest of the run, which is
strictly the worse of the two. The numbers are here to be read; the
meeting earns its new place and time either way.
```

### return-sync-reagree-single-raise

**Raising re-agreement only once** — attached to `if (!rendezvous_reagree_waiting_) {`

```text
ASK FOR THE NEXT MEETING. Requested rather than done here because
deriving is the proposer's and needs the frozen snapshot; see
rendezvous_reagree_due_ and the derive gate it opens.

ONLY WHERE THE SETTLE STAGE DID NOT ALREADY ASK, which is what
rendezvous_reagree_waiting_ records. This raise was unconditional and
described as idempotent, and it is not: the request is SPENT by the
adoption that answers it, so raising it again after an appointment's
wait authorises a SECOND derive, and deriveRendezvousProposal mints
t_meet as "now plus the interval" — the instant the team has just
committed to slides forward by however long the release took. The ts4
gen-29 N=2 rendezvous smoke measured it: cell 23 committed twice, at
t+1055s and then 6 s later at t+1061s, off one meeting.

Where it IS needed is the barriers that reach the release without a
settle stage — a pursuit reunion, and an appointment run with
rendezvous_settle_sec 0. Those never entered the wait, so they leave
immediately and re-agree while they drive, which is the older behaviour
and all that is available without a hold to do it in. An appointment
whose wait EXPIRED needs nothing raised either: nothing adopted, so the
request it made at the end of its settle is still standing.
```

### return-sync-latched-release-ends

**Latched robot ends run at release** — attached to `if (coverage_latched_) {`

```text
A LATCHED ROBOT DOES NOT GO BACK TO EXPLORING (2026-09-17). This release
is unconditional in every earlier generation, and once
maybeLatchCoverageDone can hold a FINISHED robot here it stops being
safe: the robot would be handed back to PLAN with coverage_latched_ true,
explore until max_steps_, and call finishOrRendezvous("step-budget") ->
recordExplorationComplete a second time. That stamp is keyed on step_,
not once per run, so the cell would carry TWO exploration_complete rows
at very different t_sim — in the rendezvous and hybrid arms only — and
the readers disagree about which to keep (ts1b_cells.py takes the last,
n23.py the first). The primary endpoint would be corrupted asymmetrically
across arms, which is worse than anything the hold was added to fix.

This robot got what it was waiting for: the team is whole and the settle
has run, so the merge it stayed for has happened. Ending here is the
ending it would have taken at the latch, now taken at the right moment.
```

### return-sync-latched-elapsed-hold

**Reporting the elapsed latched hold** — attached to `const double t_release_now = missionElapsed();`

```text
THE ELAPSED HOLD, read before the start stamp is cleared on the next
line. This printed rendezvous_latched_hold_sec_ — the CAP — until
2026-09-18, so every release-by-reconnection reported the same 300s
whether it had stood there four seconds or two hundred and ninety, and
the one question this line exists to answer (how long does a finished
robot wait at the agreed cell?) could not be read out of the plaintext
log at all. The expiry path 100 lines below always computed this
correctly, which is why the two disagreed. -1 where there is no stamp
to subtract from: the hold only starts if the latch fired with
rendezvous_latched_hold_sec_ > 0, and printing 0.0 for "never started"
would be the same lie pointing the other way.
```

### return-sync-incomplete-voids-settle

**Incomplete team voids the settle** — attached to `rendezvous_settling_ = false;`

```text
NOT COMPLETE. Any partial settle is void — the hold exists to let a WHOLE
team's maps merge, and restarting it is what makes a peer that drops
mid-settle cost the settle rather than shorten it. The re-agreement wait
is void with it: it is the same hold's second stage, and a commit needs
every peer's echo, so a team that has come apart cannot finish one.
```

### return-sync-conversion-reversible

**The settle conversion is reversible** — attached to `if (appointment_manoeuvre_ && appointment_settle_converted_ &&`

```text
THE CONVERSION IS REVERSIBLE (2026-09-19, generation 28). A walker that
joined the barrier from the road did so on one premise — the team had
settled, so this spot was as good as the cell. When that premise lapses
the premise is all that is gone: the robot is left stopped in open forest,
metres of unfinished walk from the one place the team agreed to be, and
through generation 27 nothing ever sent it the rest of the way. The
ts4 gen-27 N=2 rendezvous smoke cell measured it — both robots converted
mid-drive at 367/369 s, the link died at 396 s and never returned, and
both stood 13.0 m and 7.2 m short of the agreed cell until the 660 s cap:
330 s each, no outcome row, coverage frozen, and the one action that
would have closed the pair (finishing the walk, which ends with them
co-located) was the action the conversion cancelled. Waiting longer could
not fix it, because the wait is deliberately unbounded and the robots were
not where the waiting was supposed to happen.

THE TERMS ARE manoeuvreReleaseEligible's FIRST HALF, NEGATED, and written
out rather than called: that function also answers false while the team is
together and a peer is still walking in — the case where standing still is
exactly right — so resuming on its bare negation would put this robot back
on the road to meet a peer that is already coming, each setting the other's
inbound bit. A change to the release's "team is together" disjunction is a
change to this test.

The tolerance guard and the local copy are the hold-escalation's, for its
two reasons: a robot already at the cell has nowhere to resume TO, and
startReturnTo overwrites current_goal_ before it reads its own argument,
so passing current_goal_.position directly would hand it a zeroed vector.
```

### return-sync-wait-cap-by-manoeuvre

**Which patience applies at the barrier** — attached to `const double wait_cap =`

```text
WHICH PATIENCE APPLIES. Three cases, and generation 19 splits the first
one in two:

  appointment  STAY UNTIL EVERYONE IS HERE. The place was agreed by the
               whole team in advance, so the only reason a peer is absent
               is that it has further to drive or noticed the separation
               later — both of which resolve by waiting. (Since
               generation 27 a peer merely MESH-dark at the gathered cell
               no longer spends this patience — the reachable door
               releases that shape — so the peer waited on here is one
               outside the closure itself.) Giving up here
               converts a meeting that was going to happen into a no-show,
               which is what the gen-18 ts4 N=3 cell measured: husky spent
               the full 240 s and walked away from a cell two of the three
               robots had already stood on together. Unbounded by default
               (rendezvous_appointment_wait_sec <= 0).
  pursuit      the mid-run cap. A chase drives to a PREDICTED intercept,
               which can simply be the wrong place, and there is no
               agreement behind it to be kept — so giving up is the right
               response to a bad prediction rather than an abandonment.
  terminal     the field cap, shortened after an escalation (the second
               wait is a confirmation of failure, not a second full vigil).

Keyed on appointment_manoeuvre_, NOT on appointment_armed_ (2026-09-18).
The appointment record can be closed on the tick the drive departs — the
supersede and the outcome block both run inside transitionTo(RETURN_NAV) —
which silently dropped a robot standing at the agreed cell from the
unbounded appointment patience to the 240 s PURSUIT cap, i.e. turned "stay
until everyone is here" into "stay four minutes" without any line of code
saying so. What decides the patience is what the robot is DOING, and it is
keeping an appointment for as long as the manoeuvre lasts.
```

### return-sync-contagion-hold-log

**Logging the contagion hold** — attached to `if (appointment_manoeuvre_ &&`

```text
THE CONTAGION HOLD, NAMED IN THE LOG (generation 23). Reaching here with
the team complete means the ONLY thing still holding the barrier is a peer
announcing that ITS view is broken — the case teamSettled added. Without
this line that reads in the plaintext log as a gathered team and a barrier
that refuses to release, which is indistinguishable from the release being
broken. Since generation 27 the reachable door bounds this hold whenever
the closure is whole, so a LONG run of these lines now also implies a peer
outside the closure. Throttled to 30 s because the interesting quantity is
"was it held, and roughly how long", not a per-tick trace.
```

### return-sync-latched-hold-stamp

**Stamping the latched hold start** — attached to `if (coverage_latched_ && appointment_manoeuvre_ &&`

```text
START THE CAP WHEN THE FINISHED ROBOT STOPS HERE, OR WHEN THE MEETING FALLS
DUE, WHICHEVER IS LATER. The latch stamps it itself when it fires on a robot
already standing at the cell, but one that finished on the road
(keepAppointmentOnFinish) only reaches the barrier later — by arriving, or
by a nav watchdog handing the leg off short of the cell — and an unstamped
cap on an appointment barrier leaves the unbounded appointment wait with
nothing left to end it, which is the failure that path exists to remove.

The t_meet floor is the twin of the latch's own stamp and is derived there:
the departure rule aims the arrival at t_meet, so an early arrival would
otherwise spend the cap's sizing margin before the team is due.

TRAVEL IS DELIBERATELY OUTSIDE THE CAP. The cap's size is derived from the
meeting schedule — it has to outlast one rolled rung — and not from how far
the robot had to come, so charging the drive to it would shorten the hold
by a distance. The drive carries its own bounds: the manoeuvre leg budget
and the escape ladder.

coverage_latch_teardown_ is what stops a hold that has already ended from
being re-armed on a later tick. Both endings below clear the stamp and set
that flag before they leave, and it is sticky for the rest of the run.
```

### return-sync-latched-hold-cap

**The latched hold's own cap** — attached to `if (coverage_latched_ && coverage_latch_hold_start_sec_ >= 0.0 &&`

```text
THE LATCHED HOLD'S OWN CAP (2026-09-17). Checked before, and independently
of, rendezvousWaitExpired, because the cap it has to survive is the one
above: on an appointment manoeuvre wait_cap is
rendezvous_appointment_wait_sec, which is 0 = unbounded by default and is
meant to be. That patience belongs to a robot with exploring left to
trade against it. This robot has none, so an unbounded hold here is a cell
that runs to the harness wall clock with one robot standing still.

Measured from the latch — or from the stop tick stamped just above, for a
robot that finished on the road — and not from state entry: `waited`
starts when the robot entered RETURN_SYNC, which is before it finished, and
charging the hold for that time would cut it short by however long the
robot waited while it was still exploring.
```

### return-sync-no-waited-rewind

**Why waited is not rewound** — attached to `if (rendezvousWaitExpired(waited, wait_cap)) {`

```text
A BLOCK THAT REWOUND `waited` TO max(arrival, t_meet) STOOD HERE. It
existed because a punctual robot began spending its patience while the
meeting was still in the future and could time out ahead of its own
appointment (measured: atlas gave up at lateness_sec = -11.184).

THE NOTE THAT STOOD HERE CALLED IT A NO-OP AND WAS STALE (2026-09-22). It
argued that "under the countdown rule t_meet is the DEPARTURE time, so
arrival is necessarily at or after it" — true only of the bare
`now >= t_meet` test that lived between 2026-09-18 and 2026-09-19.
appointmentDue() now departs at t_meet minus appointmentLeadMs so that the
ARRIVAL lands on t_meet, which makes early arrival the designed case, not
an impossible one: `since_meet < waited` again.

THE REWIND STILL STAYS OUT, for a reason that does not depend on that.
`wait_cap` on an appointment manoeuvre is rendezvous_appointment_wait_sec,
which is 0 = unbounded by default and deliberately so, and
rendezvousWaitExpired cannot fire on an unbounded cap however early
`waited` started. The bounded clock an early arrival DOES reach is the
latched hold above, and that one is floored at t_meet where it is stamped
rather than rewound here — a second clock in this barrier is exactly what
the deleted block made hard to read.
```

### return-sync-midrun-give-up

**Mid-run give-up returns to PLAN** — attached to `RCLCPP_INFO(get_logger(),`

```text
A mid-run attempt must never end the run: the map is not saturated
(the trigger only fires from an unsaturated PLAN tick), so give the
manoeuvre back its time and go explore. The cooldown stamps in
transitionTo when reconnect_active_ falls.

...which is why coverage_latched_ disqualifies this branch (2026-09-17).
The premise stated above — "the map is not saturated" — is exactly what
the at-the-rendezvous hold falsifies: it keeps a SATURATED robot inside
a mid-run manoeuvre on purpose. Sending that robot back to PLAN would
re-explore a finished map and emit a second exploration_complete at the
step budget (see the release path above for why that corrupts the
endpoint). A latched robot falls through to the escalate/give-up ladder
instead, which ends the run the way every other ending does.
```

### hold-escalate-same-target

**Escalating to the dispatch's own target** — attached to `Eigen::Vector3f esc_target = Eigen::Vector3f::Zero();`

```text
Escalate to the SAME point the dispatch picked, never to a freshly
derived one: a packet heard while we waited must not move the target
off the one the peer is driving to. (The point the dispatch picked
WAS the midpoint of the pair this manoeuvre was armed from, held in
reconnect_rec_, until 2026-09-16; it is now the appointment cell, and
the branch below reads it off the leg's own goal for exactly the same
no-re-derivation reason.) Escalating to this robot's own anchor sent a
waiting robot to a point one comms range from where its waiting peer
would go, which is the non-convergence documented in dispatchReconnect;
it has to be fixed in both places or a hold reintroduces it after the
dispatch avoided it.

THE MIDPOINT ESCALATION IS GONE FOR EVERY ARM (2026-09-16), which
leaves the two branches below: the agreed point if one was agreed, and
this robot's own last-connected anchor otherwise.

It used to run for RENDEZVOUS and HYBRID but not PURSUIT, on the
grounds that "pursuit has no agreed fallback point by design — that
absence is the A/B against hybrid". That reasoning survived the
removal of the midpoint from the dispatch paths and should not have:
with the dispatch midpoint gone from both arms, this was the last
place a midpoint could still be driven to, and leaving it here would
have preserved the exact defect in a slower form. A robot reaches this
line ~300 s into a hold, and a place with no time is no better agreed
then than it was at the separation.

It also broke the composition the four-arm design rests on. Hybrid is
meant to be pursuit plus rendezvous and nothing else; an escalation
target hybrid could drive to and pursuit could not is a third
behaviour, so any hybrid-vs-pursuit contrast would have carried it.
Now all three arms escalate identically, and the ONLY thing hybrid can
do that pursuit cannot is keep an appointment — which is the contrast
the arm exists to measure.

The own-anchor escalation has a known weakness (two waiting robots go
to two points up to a comms range apart, the non-convergence noted in
dispatchReconnect). That weakness is unchanged for pursuit, which has
always had it, and for the other two arms it is now the fallback of a
fallback: the appointment branch below covers every case where the
team actually agreed on somewhere to be.
```

### hold-escalate-appointment-cell

**Escalating an appointment to its cell** — attached to `if (appointment_manoeuvre_) {`

```text
P5: an appointment that was departed already HAS an agreed point, and
it is not the midpoint — escalating to the midpoint would walk away
from the one place the peer has a reason to be. A robot that reached
the cell is already there, so the tolerance test below falls through
and it gives up on the spot, which is correct: there is nowhere
better to go. One that never reached it re-attempts the same drive.
```

### hold-escalate-leg-destination

**Why escalation reads the leg's goal** — attached to `esc_target = current_goal_.position;`

```text
The leg's own destination, not a re-derivation: appointmentPoint()
reads appointment_, which closeAppointment() blanks — and closing on
the departure tick is routine (see appointment_manoeuvre_). Asking
again would answer "nowhere" for the one case this branch exists to
serve. current_goal_ is where this leg was sent and RETURN_SYNC does
not overwrite it.
```

### barrier-give-up-reroute-home

**Defensive reroute home on give-up** — attached to `if (mission_return_enabled_ && have_home_) {`

```text
Under mission return this site should be unreachable — terminal
manoeuvres are never dispatched (finishOrRendezvous routes home
instead) — but if a config drift ever re-opens the path, a give-up
must not strand the robot at a dead barrier: reroute it home, which is
what every other ending does. Defensive, and loud in the event stream
(reason survives into reconnect_end and mission_complete).
```

## ExploPlannerNode::startReturnHome

### return-home-reentry-guards

**Mission return re-entry guards** — attached to `if (mission_return_done_) {`

```text
Two re-entry guards, and they answer different questions. Both matter
because every assignment below is a RESET and transitionTo re-stamps
state_enter_time_: a re-entry puts a robot that had escalated to ESCAPE
back to DIRECT with its escape budget refilled and its approach/frozen
windows zeroed — it forgets that it is stuck and the graduated response
starts over from the bottom — grants a second full mission_return_max_sec,
and emits a second mission_complete whose homing_duration_sec and
homing_distance_m restart from zero, so both read short.

GUARD 1, run-scoped: the mission return already RESOLVED. Measured on
ts1b: 16 DONE->RETURN_HOME re-entries and 16 robot-runs carrying two
mission_complete rows, all in the treatment arm. The request itself is
legitimate — the coverage latch runs off the metrics tick, which measures
in every state, so a robot that ended on barrier-gave-up at t=1189 can
genuinely saturate its map at t=1900 while parked at home. Only the
response is wrong. What that latch stamps is kept: recordExplorationComplete
runs in the caller BEFORE this call and nothing here gates it, so the 16
genuine late exploration endpoints survive; it is the redundant homing leg
— asked of a robot that is already home — that is refused.

Returning true, not false: the caller's question is "is this robot heading
home?", and it went home and arrived.
```

### return-home-inflight-guard

**The in-flight homing guard** — attached to `if (state_ == State::RETURN_HOME) {`

```text
GUARD 2, leg-scoped: a homing leg is still IN FLIGHT. The one already
running is strictly further along than the one this call would start.
DEBUG rather than WARN because this fires on ordinary tick-loop re-entry,
where nothing has gone wrong — and it is not the guard the ts1b defect
needed, which is why guard 1 exists above it.
```

### return-home-no-publish-at-start

**No goal publish at return start** — attached to `transitionTo(State::RETURN_HOME, reason);`

```text
No goal publish here — see the declaration comment. The caller's
abandonNavGoal cancel-all is still in flight, so a goal sent now can be
swallowed by it; the first doReturnHome tick publishes instead.
transitionTo also closes any live reconnect manoeuvre (RETURN_HOME is
outside the manoeuvre set), emitting reconnect_end with this reason.
```

## ExploPlannerNode::doReturnHome

### return-home-budget-distance

**Homing budget distance in retrace** — attached to `const double budget_dist =`

```text
Budget distance is the straight line to the goal — except in retrace
mode, where the goal is only the nearest crumb: there it must cover the
whole remaining trail down to home, or the budget would fire moments
after the switch. Deliberately the SAME function the approach watchdog
scores with: one of them says how far there is to go and the other says
how long that may take, so if the two definitions drift apart the ladder
either fires on a robot making fine progress or stops firing at all.
```

### return-home-budget-from-now

**Homing budget measured from now** — attached to `double leg_budget = std::max(`

```text
Distance-true budget, same model as the manoeuvre legs (see
startReturnTo). Measured from NOW (elapsed added) because a retry
re-arms this mid-leg: a budget recomputed from the shrinking remaining
distance alone would fall below the already-elapsed time and fire on
the next tick. No min() against the mission cap — the cap check below
runs first every tick, so it bounds the leg regardless.
```

### return-home-escape-budget-margin

**Escape leg budget versus leg cap** — attached to `constexpr double kEscapeBudgetMarginSec = 10.0;`

```text
An escape leg must expire as an escape, not as a dead mission.

The two deadlines race. The leg cap (return_escape_leg_sec) ends the
leg with resumeRetrace and the run continues; the nav budget ends the
WHOLE return with finishMissionReturn("budget", "home-gave-up"), which
censors the cell. The budget check runs first in the tick, so whichever
deadline is earlier decides which of those two very different outcomes
a cell gets — and the escape target is always short enough that the
distance term loses to nav_min_timeout_sec, leaving the budget at a
flat 30.0 s against a leg cap of a flat 30.0 s. The only thing that has
been keeping the leg cap in front is the 0.3 s publish deferral: three
ticks, held by a coincidence between two independently-tunable params
that nothing in the code relates or checks.

So state the relationship instead of inheriting it. The leg cap is the
authority on how long an escape may take; the budget's job here is only
to catch a leg whose termination logic never runs at all, so it sits a
clear margin behind and is not derived from distance.
```

### return-home-retrace-advance

**Retrace waypoint advance** — attached to `if (home_mode_ == HomeMode::RETRACE && home_trail_idx_ > 0) {`

```text
Retrace waypoint advance: within 3 m of the current crumb, walk the
index toward home past any crumbs already inside that circle and aim at
the next one. Plain goal replacement — no cancel, so nothing to race.
The arrival check above still measures against home itself, and the
watchdogs below keep running as usual. An advance shrinks the approach
metric (triangle inequality), so it can never manufacture a stall.
```

### home-watchdogs-three

**Three independent homing watchdogs** — attached to `const float metric = homeApproachMetric();`

```text
---- Three independent watchdogs ----
FROZEN: gross metres travelled, the historical test, measurement UNCHANGED
(15 s / 0.2 m, shared with the NAVIGATE paths). It answers "is the
platform physically moving at all?".
APPROACH: remaining distance home. It answers the question the frozen test
structurally cannot — "is any of that movement getting the robot home?" —
and is the whole point of this generation: seed11 orbited at 0.05 m/s for
the full 600 s cap, passing the frozen test in every window, netting
-0.08 m of approach.
They are evaluated together, both windows are advanced whichever fires,
NEAR-HOME STALL: time spent inside the radius where APPROACH is muted. It
answers the third question, "is the robot close and simply not finishing?",
which neither of the others can: a robot circling 2 m from home passes
frozen every window (it accumulates metres) and approach never even runs.
They are evaluated together, both windows are advanced whichever fires,
and `kind` in the log distinguishes them, so each defect stays attributable
to exactly one detector.
```

### return-home-frozen-delta-capture

**Capturing the frozen detector delta** — attached to `float frozen_delta = 0.f;`

```text
The frozen detector's own left-hand side, captured because the next two
lines destroy it: progress_check_dist_ is re-baselined immediately after
the test, so by the time homeWatchdogFire runs the tested delta cannot be
recomputed from any member. Through generation 7 it was never passed on at
all, and a frozen fire reached the log holding only metric_m — an
INSTANTANEOUS remaining distance, not the window movement that was tested.
See the note on the log call for what that cost.
```

### return-home-approach-suppression

**Approach watchdog suppression zones** — attached to `if (home_mode_ != HomeMode::ESCAPE &&`

```text
Suppressed in ESCAPE (an escape leg moves AWAY from home on purpose) and
inside return_approach_suppress_m_ of the target, where the 1.0 m the
detector demands per window is a large fraction of what remains. Inside
that radius the near-home stall detector below takes over -- the band is
not left uncovered, which is what the previous generation assumed.
```

### return-home-jump-disarms-band

**Pose jumps disarm the near-home band** — attached to `near_home_armed_ = false;`

```text
Same argument for the band clock: a jump can place the robot inside the
band without it having driven there, and time accumulated across one is
not time spent failing to arrive. Forfeit the window rather than fire on
it -- conservative is the right default for a detector whose response is
to drive AWAY from a goal the robot is already close to.
```

### home-near-stall-band-clock

**Near-home stall band clock** — attached to `bool near_home_stalled = false;`

```text
Band clock for the near-home stall detector, kept up to date on EVERY tick
-- including the ticks where one of the other two detectors is about to
fire and return below. Time inside the band is a property of where the
robot is, not of which detector acted, and skipping the bookkeeping on a
fire tick would silently restart the clock every time the ladder ran.

Time-in-band is the right test here rather than an approach RATE: a healthy
homing crosses this band once and arrives within seconds, whereas the
failure is a robot that sits inside it, still moving, for minutes. Rate is
the wrong question so close to the goal -- the robot is decelerating, and
demanding 1.0 m of approach per window with 1.5 m left would fire on a
perfectly healthy arrival. That is exactly why the approach detector is
muted in here, and why re-using it was not an option.
```

### home-watchdog-frozen-wins-label

**Frozen wins the watchdog label** — attached to `homeWatchdogFire(frozen ? "frozen" : "approach", metric, dist,`

```text
Frozen wins the label when both fire: "not moving" is the more specific
diagnosis and picks the response that assumes the platform is stuck.

The tested delta is selected by the SAME condition that selects the
label, so the logged number is always the one belonging to the detector
named in `kind`. Passing approach_delta unconditionally (as generation 7
did) would attribute the approach detector's value to a frozen fire on
exactly the ticks where both fired.
```

### home-near-stall-fires-last

**Why the near-home stall fires last** — attached to `if (near_home_stalled) {`

```text
Last of the three, deliberately: frozen and approach are the more specific
diagnoses and each picks a response tuned to its own failure, so a tick on
which either is also due belongs to them. In practice they rarely collide
-- frozen fires after 15 s of standing still and would already have run
long before a 60 s band dwell matures -- so reaching here means the robot
really is moving, really is close, and still is not arriving.

Response is homeWatchdogFire's existing graduated ladder (resend, retrace,
escape), which is what an unreachable home needs: ESCAPE routes away and
back, and is capped at return_escape_max_attempts_. Disarm before firing,
not after: the ladder can leave the robot inside the band (a resend does),
and re-arming from the next tick is what stops one stall from firing on
every tick thereafter.
```

## ExploPlannerNode::pickEscapeTarget

### home-escape-target-choice

**Choosing the escape leg target** — attached to `bool ExploPlannerNode::pickEscapeTarget() {`

```text
Choose where an escape leg drives. Preference order:
 1. A breadcrumb 1.5-6.0 m away (0.75x-3x the crumb spacing) — ground this
    robot physically drove over earlier in the run, which is the strongest
    reachability prior available. Nearest wins; ties break toward the HIGHER
    index, i.e. the most recently visited crumb, which is behind the robot.
 2. A pose 2.5 m behind the robot, rotated by 0/+60/-60 deg on successive
    attempts so a repeat never aims at the same blocked spot.
Behind is the point: it forces a ~180 deg rotate-in-place (the controller
zeroes linear and rotates whenever heading error exceeds 45 deg, and angular
is never clearance-scaled), after which the front arc faces ground with
known clearance. Both candidate freeze mechanisms — slow-down-band creep and
heading oscillation — are FRONT-ARC phenomena.
```

## ExploPlannerNode::startEscapeLeg

### home-escape-no-rebaseline

**No frozen-window re-baseline on escape start** — attached to `RCLCPP_WARN(get_logger(),`

```text
No frozen-window re-baseline here on purpose: republishHomeGoal clears
return_home_goal_sent_, so the publish block runs ~0.3 s from now and
resets progress_check_time_/_dist_ itself. Resetting here as well would be
dead code overwritten within three ticks — the leg is already judged from
its own publish, not from the remains of the window that fired.
```

## ExploPlannerNode::resumeRetrace

### home-escape-end-metric-snapshot

**Escape-end metric snapshot before mode flip** — attached to `const float escape_metric = homeApproachMetric();`

```text
Snapshot the approach metric BEFORE engageRetrace() flips home_mode_.
homeApproachMetric() is mode-dependent by design (straight line in
DIRECT/ESCAPE, remaining trail length in RETRACE), and this event is about
the escape leg that just ENDED, whose in-force metric was the straight
line. Measuring after the flip reported the newly-resumed retrace's trail
distance on a row labelled "escape" — the same convention error the `mode`
argument below was already fixed for, left behind in the metric argument.
```

### home-escape-end-log-convention

**Escape-end row mode and placeholders** — attached to `exp_log_->logHomeWatchdog(expCtx(), "escape-end", "escape",`

```text
mode is the mode this event is ABOUT — always "escape" here, because
resumeRetrace is only ever reached from an escape leg. It used to log
homeModeName(home_mode_), which by this point is the mode just resumed
INTO, i.e. the opposite convention to every detector-fire row, so a
reader grouping by mode counted escape-ends as retrace events. The
resumed mode is still recorded, in next_mode, where it does not collide.
No tested pair: an escape-end is not a detector fire, so it has no
inequality to record. The two kNoTestDelta arguments are NOT sentinels
and nothing downstream reads them as such — logHomeWatchdog omits
test_delta_m and test_threshold_m from escape-end rows entirely, gated on
`kind`, precisely because every numeric sentinel collides with a real
reading (0.0 is the canonical frozen fire, negatives are a receding
approach fire). ABSENCE is what carries the meaning. These are inert
placeholders for arguments that will not be written; an earlier version
of this comment credited them with the distinction that the omission
actually makes.
```

## ExploPlannerNode::republishHomeGoal

### home-republish-brake-goal

**Braking via the home goal republish** — attached to `void ExploPlannerNode::republishHomeGoal(const char* why) {`

```text
Brake at the current pose, then let the next tick publish the mode's goal.
The nav cancel action is a no-op here (simple_nav_3d does not serve it), so
the brake goal at the robot's own pose is what actually stops the platform —
and it also clears the navigator's active goal, which is what makes a
positional repeat acceptable again afterwards.
```

## ExploPlannerNode::homeWatchdogFire

### home-watchdog-fire-ladder

**The homing watchdog fire ladder** — attached to `void ExploPlannerNode::homeWatchdogFire(const char* kind, float metric,`

```text
The single decision point for a homing watchdog fire. Graduated, and never
instantly fatal on a first fire: a false positive costs a resend, or at
worst a demotion to the retrace, which is slower but convergent. Only an
exhausted escape budget parks the robot.
  approach in DIRECT, fire 1 -> resend the home goal
  approach in DIRECT, fire 2 -> engage the retrace
  approach in RETRACE       -> escape leg
  frozen, any mode          -> escape leg (a stuck platform will not be
                               unstuck by resending the goal it is already
                               failing to follow — seed16's resend at
                               +106 s changed nothing)
  frozen during an escape   -> abort the leg, back to the retrace
  escape budget exhausted   -> park, with the unchanged result vocabulary
Termination: DIRECT gives at most 2 fires before it becomes RETRACE, every
RETRACE fire spends one of a bounded number of escapes, and an escape leg
can only end in RETRACE.

WHAT ACTUALLY BOUNDS A WEDGED ROBOT — read this before quoting the 600 s
cap. mission_return_max_sec is the outer bound only for a robot that keeps
MOVING without arriving (the seed11 orbit case: it passes every frozen test
and rides the cap to 600 s). A robot that cannot move at all is parked far
sooner, by this ladder rather than by the cap, because `frozen` alone walks
the whole thing:
    15 s  frozen in DIRECT              -> escape 1   (want_escape is true
                                           for frozen in ANY mode)
    30 s  frozen during that escape     -> abort to RETRACE (no escape spent)
    45 s  frozen in RETRACE             -> escape 2
    60 s  abort;  75 s escape 3;  90 s abort
   105 s  frozen in RETRACE, budget spent -> park, "no-progress"
i.e. (2 * return_escape_max_attempts + 1) * progress_window_sec = 105 s at
the campaign's 3 escapes and 15 s window. That is the intended outcome —
a robot that has not moved 0.2 m in any of seven consecutive windows is not
going to, and censoring it at 105 s is both honest and cheaper than 600 s of
sim. It is written down here because the recorded mission-end time of a
censored cell differs by ~8 minutes depending on which bound fired, and
exposure to this ladder is not arm-symmetric (see the nav-failure counts),
so any analysis must treat 105 s and 600 s parks as the same event class and
must not read the difference as a treatment effect.
```

### home-watchdog-kind-exhaustive

**Watchdog kind as a three-valued enum** — attached to `if (!is_frozen && !is_stall && std::strcmp(kind, "approach") != 0) {`

```text
`kind` is a three-valued enum spelled as a string, and the selections below
are exhaustive over it rather than "frozen vs everything else". The third
kind is the one this comment used to warn about: it would have inherited
approach's window AND threshold silently, and the row would then log a
test_threshold_m the detector never compared against. The whole point of
the field is that the fired inequality is re-derivable from the row alone.
```

### home-watchdog-test-threshold

**Per-detector test threshold in watchdog rows** — attached to `const double test_threshold = is_frozen ? progress_min_distance_m_`

```text
Selected by `kind` for the same reason `window` is: one event type carries
three detectors, and the threshold that was compared against differs
between them. Logged beside the delta so the fired inequality
(test_delta_m < test_threshold_m) is re-derivable from the row alone,
without joining to run_start params and without knowing which detector
owns which param.

The stall detector fits that convention rather than breaking it: the
inequality it fired on is `metric < return_approach_suppress_m_`, held
continuously for `window` seconds. Band membership, in metres, in the same
direction as the other two — so a reader who knows nothing about the third
detector still reads the row correctly. The duration is carried by
`window`, which is exactly what that column means on every other row.
```

### home-frozen-in-escape-row

**Logging a frozen fire during an escape** — attached to `if (exp_log_) {`

```text
Record the FIRE before aborting the leg. Without this the only row this
path produced was resumeRetrace's `escape-end`, which is a leg
termination: the writer omits the tested pair on that kind, so a genuine
frozen fire — with its delta and threshold both live right here — was
written down carrying no inequality at all. That is the exact defect
this generation exists to close, surviving on the one path where the
detector fires and something else does the logging.

A distinct kind, not "frozen": the row has to stay separable from a fire
that provoked an escape, because this one ABORTS an escape already in
progress and the two mean opposite things about the leg. It is not
"escape-end", so the writer's omit-case does not swallow it.

And NOT named "escape-frozen", which is what this said first: that token
is already the `response` on the escape-end row that resumeRetrace emits
three lines below. One abort therefore wrote the same string into two
different columns of two consecutive rows, and any reader grepping the
token without also keying on the column counts one abort as two.

`window` is the DETECTOR's window (line 6040), not the escape leg's
duration. This is a detector-fire row, so window_sec has to carry the
fire convention its siblings do — the leg duration is already on the
escape-end row that follows immediately, so nothing is lost.
```

### home-stall-resend-before-escape

**Stall takes the resend ladder in DIRECT** — attached to `const bool want_escape = is_frozen || home_mode_ == HomeMode::RETRACE;`

```text
A stall deliberately takes the `else` ladder, not this one, when the robot
is still heading straight at home: the first thing an unfinished approach
deserves is a fresh goal, and only after that the heavier moves. In RETRACE
it escapes immediately for the same reason approach does — a retrace that
is not converging has already had its resend.
```

## ExploPlannerNode::finishMissionReturn

### home-return-done-latch

**Spending the mission return leg** — attached to `mission_return_done_  = true;`

```text
Spend the leg BEFORE anything can re-open it. Set here and not in
finishNow because it is the mission RETURN specifically that is spent:
finishNow also ends runs that never homed at all, and latching there would
silently forbid a homing leg those runs are still entitled to. Set
unconditionally, ahead of the `exp_log_` block, because the damage a
re-entry does — a second homing budget, a reset escape ladder — is
behavioural and happens in a build with no event log at all.
```

### pursuit-missing-peer-record

**Choosing the missing peer record** — attached to `const ExploPlannerNode::LastContact*`

```text
The teammate the barrier is actually waiting on: among the peers we have
EVER heard (last_contact_), the one not currently accounted for, freshest
record first. Liveness is peerAccounted — THE predicate, so the peer this
nominates is the peer the barrier is actually blocked on, and deliberately
not the exploit grace window.
```

## ExploPlannerNode::predictIntercept

### pursuit-predict-intercept-doc

**predictIntercept arming notes and tour age** — attached to `PursuitTarget ExploPlannerNode::predictIntercept(const std::string& peer_id,`

```text
Arm the chase. The trail is the missing peer's last declared goal (the
trail head — the freshest hypothesis of where it went) and then its last
heard pose (sweeping the leg it was driving; on a mesh the link re-forms
the moment ANY point of the sweep comes within range of it, so partial
coverage of the leg is already useful). Returns false without touching any
state when the budget comes back 0 — record too stale, or pursuit disabled
by pursuit_budget_max_sec <= 0 — and the caller falls through to the
mode's fallback.

`tour_age_sec` is set to the age of the record the prediction ran on, or left
at -1 when there was no record to run on — it is an out-param rather than a
field of the return value because it describes the input, not the answer.
```

## ExploPlannerNode::startPursuit

### pursuit-intercept-snapshot

**Intercept prediction snapshot per chase** — attached to `have_dispatch_predict_ = false;`

```text
--- the intercept, when the model is on (P6, §3.7) --------------------

Computed here, once per chase, and never refreshed while it runs: the same
snapshot discipline as pursue_rec_. A prediction re-derived mid-chase off a
packet heard one-way would move this robot's aim without moving anything
the peer knows about, which is how a chase turns into a wander.

What it replaces is exactly the FIRST waypoint, and nothing else. The
legacy trail is kept behind it, so a missed intercept still sweeps the leg
the peer was last known to be driving — the floor is a real fall-through,
not a promise.

IT DOES RESIZE THE WATCHDOG, and this comment denied it ("does NOT get to
touch the budget ... the mdp and trail arms differ in where the chase drives
and in nothing else") until 2026-09-18. When the swap fires, `budget` is
recomputed from the intercept route at the swap site below. That is not the
model buying itself chase time — the CEILING is untouched, the same
pursuit_budget_max_sec_ the trail is priced against, and the affordability
test the swap must pass is asked at the same measured rate as the trail's
own refusal. What it prevents is a watchdog sized for one route killing a
drive down a longer one, which would have scored as the model losing on an
endpoint it never reached. The confound the old sentence guarded against is
still guarded against; it is the cap that does it, not the budget.

have_dispatch_predict_ is armed at the COMMIT point far below, not here.
Everything between this line and there can still decline the chase, and a
prediction left armed by a declined chase would be consumed by whatever
leaf the mode falls through to — reporting an intercept for a manoeuvre
that never chased anything.
```

### pursuit-model-sec-nav-speed

**Model seconds at the NAV speed** — attached to `auto trailMetres = [&](const std::vector<Eigen::Vector3f>& t) {`

```text
Path length of a waypoint list from here, and the model seconds that buys
at the NAV speed estimate — the conservative one, deliberately: modelSec
answers the watchdog's question ("how long before I call this leg dead"),
not the predictor's ("when will I actually be there"). The two want to be
wrong in opposite directions, which is why travelSec below exists and uses
a different speed; until P1 there was only this one and the refusal gate
was reading it as if it were the other. Factored out because the budget
rule and the intercept's affordability test below must ask it identically;
a second copy of this arithmetic could drift, and the symptom would be a
chase dispatched with a budget sized for a different route than the one it
drives.
```

### pursuit-travel-sec-measured

**Travel seconds at the measured speed** — attached to `auto travelSec = [&](double metres) {`

```text
P1 / §3.8. The OTHER question: not "when do I give up on this leg" but
"can this chase be done at all". Measured speed, no safety factor — see
pursuit_speed_measured_mps_ for why the watchdog's inflated rate is the
wrong yardstick for a refusal, and why neither correcting that rate nor
raising the cap is the fix.
```

### pursuit-stale-trail-budget

**Distance-true budget for a stale trail** — attached to `const double trail_m = trailMetres(trail);`

```text
Distance-true budget: the contact-pose endpoint's value is geometric
(the swapped contact pair was a connected configuration), so it earns
the full model time — but ONLY if the cap covers the whole trail. A
partial chase ends at an arbitrary disconnected point, which is worse
than the mode's fallback (meeting point / hold-and-beacon).

"The cap covers the trail" is now asked in real seconds. The two
quantities below are deliberately different and neither substitutes for
the other: trail_travel_sec decides WHETHER to chase, trail_model_sec
sizes the watchdog that will end the chase if it goes wrong.
```

### pursuit-stale-budget-upper-clamp

**The load-bearing upper budget clamp** — attached to `budget = std::clamp(trail_model_sec,`

```text
The upper clamp is load-bearing now, where before it was implied by the
refusal above: the model rate can price a feasible trail well past the
cap, and an unclamped budget would let one chase outspend the absence
bound a waiting teammate is relying on. This matches what the non-stale
branch already does — pursuitBudgetSec() clamps to max_sec — so the two
branches hand out budgets under the same ceiling. lo <= hi holds by
construction (lo is a min() against the same hi), so the clamp is not UB.
```

### pursuit-intercept-downgrade-rule

**Downgrading an unaffordable intercept** — attached to `if (mdp.valid()) {`

```text
--- swap the intercept in front, if it fits the budget just granted ------

The affordability test is the whole "never worse" rule made mechanical. An
intercept the budget cannot reach is not a better aim, it is a chase that
times out somewhere between here and a cell the peer has by then left; the
legacy trail, sized by the rule that granted the budget, at least ends on a
place the peer provably was. So an unaffordable prediction is DOWNGRADED,
not declined — declining would make the model able to suppress chases the
control arm performs, which is the one failure mode §3.7 rules out.
```

### pursuit-intercept-priced-travel

**Pricing the intercept in travel seconds** — attached to `if (mdp_travel <= pursuit_budget_max_sec_) {`

```text
PRICED IN TRAVEL SECONDS AGAINST THE CAP, NOT IN MODEL SECONDS AGAINST
THE WATCHDOG (2026-09-18). The test used to be `modelSec(mdp_m) <=
budget`, and with the campaign's constants that comparison was dead
arithmetic — the swap could not fire, so `_mdp` and the legacy trail were
the same arm wearing two names.

The arithmetic, at nav_speed_estimate 0.15 / safety 3.0 / measured 0.40:
modelSec is 20 s per metre and travelSec is 2.5. In the stale branch
`trail` is exactly [peer_pose], so trail_m is the straight line to the
contact pose, while mdp_trail is [intercept, peer_pose] — so mdp_m >=
trail_m by the triangle inequality, with equality only for an intercept
sitting on that very line. budget is clamp(20*trail_m, 30, 600), which
makes the old test `mdp_m <= trail_m` in the whole mid-range
(trail_m in [1.5, 30] m), `mdp_m <= 1.5 m` below it, and `mdp_m <= 30 m`
above it — against an mdp_m that is >= trail_m > 30 in that last case.
Every branch is unsatisfiable except the degenerate collinear one. The
non-stale branch is the same story with a staleness discount making it
harder still.

It was also the wrong question twice over. `budget` is the WATCHDOG — how
long before this leg is called dead — and P1/§3.8 established that a
refusal must be asked at the measured rate against pursuit_budget_max_sec,
which is exactly what the stale branch's own trail refusal above does.
Asking the intercept the same question on the same yardstick is what
makes the two aims comparable; asking it a harsher one is how the model
was silently disabled.
```

### pursuit-intercept-resize-watchdog

**Resizing the watchdog to the intercept route** — attached to `budget = std::clamp(mdp_model,`

```text
AND RESIZE THE WATCHDOG TO THE ROUTE ACTUALLY DRIVEN. Leaving `budget`
sized for the trail while driving the (longer) intercept route is how a
swapped-in intercept dies mid-drive at a place the peer never was —
worse than both aims, and it would have shown up as the model losing on
an endpoint it never reached. modelSec is the right rate here for the
same reason it sizes every other watchdog in this file: it is the
conservative estimate, and a watchdog wants to be wrong long.

The non-stale branch's staleness discount is deliberately NOT carried
over. That discount prices how little a stale trail HEAD is worth
chasing — a question already answered `yes` by the budget > 0 test
above — and the intercept is the model's replacement for that head, so
discounting it for the staleness it was computed to absorb would size
the watchdog below the drive it is watching.
```

## ExploPlannerNode::armPursuitWaypoint

### pursuit-arm-waypoint

**Arming a pursuit waypoint** — attached to `void ExploPlannerNode::armPursuitWaypoint() {`

```text
Publish the current trail waypoint as the nav goal and (re)enter PURSUE.
Called for the first waypoint by startPursuit and again on each advance —
re-entering resets state_enter_time_, so the per-waypoint nav budget and
progress window restart while the pursuit budget keeps running from
pursue_start_time_. The presence intent keeps the claim/beacon fresh for
the same reason as the RETURN states: the pursued robot must be able to
count us the moment the mesh re-forms. Known wart: the claim disc this
broadcasts sits on the CHASED peer's own goal (RobotIntent has no flag to
mark a chase beacon), so a returning peer can briefly MinPos-yield its own
goal to us — self-limiting, since contact releases the chase within a tick
and the stray claim ages out with the TTL.
```

### pursuit-leg-nav-budget

**Nav budget for manoeuvre legs** — attached to `nav_budget_sec_ = std::max(`

```text
Manoeuvre legs are exempt from nav_max_timeout_sec: the smart-timeout
ceiling was sized for exploration hops, and a cross-world return clipped
to it dies tens of metres short of a destination whose whole value is
ARRIVING (the connected geometry). Distance-true budget, floor kept; the
no-progress window remains the stuck-robot watchdog.

Same reconnect_nav_max_sec ceiling as startReturnTo. Usually slack here —
pursue_budget_sec_ caps the whole chase and normally binds first — but a
single waypoint leg must not be able to exceed it either.
```

## ExploPlannerNode::doPursue

### pursuit-dopursue-release-conditions

**doPursue release conditions** — attached to `void ExploPlannerNode::doPursue() {`

```text
Drive the trail. Release conditions, in priority order: the team is back
in comms (the whole point — re-plan against the merged map, no arrival
needed); the pursuit budget is spent (the chase hypothesis is dead — hand
off to the mode's fallback); the current waypoint is reached or judged
unreachable (advance to the next, or fall back when the trail is
exhausted). Per-waypoint nav failures advance rather than abort: waypoint
2 can be reachable when waypoint 1 is not.
```

### pursuit-release-quarry-or-team

**Pursuit release on team or quarry** — attached to `const bool quarry_heard = !pursue_peer_id_.empty() &&`

```text
Release when the whole team is back — or when the CHASED peer alone is:
the chase has done its job either way, and on a 3+ team burning the rest
of the budget driving at a teammate that is already in comms only delays
dispatching on whoever is still missing (PLAN re-runs finishOrRendezvous
once saturation re-confirms, and missingPeerRecord picks the next one).

teamComplete AND NOT teamSettled, DELIBERATELY (generation 23). The
contagion belongs to the appointment and only to the appointment: pursuit
is defined as "an individual robot goes looking for another robot when IT
deems it necessary", so a third robot's report that IT cannot hear someone
is not this chase's business, and folding it in would make one robot's
outage hold every other robot's chase open. The trigger site keeps the same
predicate for the same reason, so pursuit's arm and release still agree
with each other — which is what the five-site rule actually demands. Note
the `|| quarry_heard` below already makes this release looser than its
trigger; that asymmetry predates generation 23 and is unchanged here.
```

### pursuit-departure-deadline-first

**Departure deadline ends the chase first** — attached to `if (appointment_armed_ && !appointment_departed_ && appointmentDue()) {`

```text
P5: the departure deadline terminates the chase, and it is tested BEFORE
the budget and before the trail.

Ordering is the whole content of this block. §3.6.1 partitions the outage
into a chase span and an appointment span, and the deadline is the
boundary; if the budget or an exhausted trail could fire first, the chase
would hand off to pursuitFallback and the appointment would be kept — or
missed — as a side effect of whichever timer happened to expire, which is
exactly the arbitration this design removed.

THE MEETING IS A SHARED INSTANT; THE BREAK-OFF IS NOT (2026-09-19).
appointment_.t_meet_ms only ever moves by whole intervals of the agreed
recurrence — at arming, and since 2026-09-20 when a failed drive has spent
its escape ladder without reaching the cell and rolls the robot onto a
later rung (appointmentLegWatchdog; the escapes themselves move nothing,
which is the point of preferring them) — so the ARRIVAL every robot aims at
is always an instant on the agreed lattice. The
moment each robot has to stop chasing to make it is its own, because
appointmentDue() now leaves early enough to arrive: a chase that has driven
this robot AWAY from the meeting cell grows its live lead and cuts the
chase off sooner.

THIS COSTS HYBRID'S CHASE WINDOW ITS INDEPENDENCE, knowingly. A shared
instant made the window a fixed dose; a lead-adjusted one makes a longer
chase buy itself less time to chase. That is the correct direction once
t_meet is an arrival target — a chase that would make the robot miss the
meeting is a chase it cannot afford — and it is the trade Kalhan's
punctuality rule asks for (2026-09-19). What it means for analysis is that
hybrid's chase duration is no longer exogenous to chase distance, so
`travel_sec` below is a covariate on the break-off and not just on the
no-show.
```

## ExploPlannerNode::pursuitFallback

### pursuit-fallback-routing

**Routing a spent chase to its fallback** — attached to `void ExploPlannerNode::pursuitFallback(const char* why) {`

```text
Route a spent or exhausted chase to the mode's fallback. HYBRID drives to
the deterministic meeting point and waits there — the pursued robot's own
hybrid dispatch computes (approximately) the same point from its own
record, so the worst case degenerates to the rendezvous guarantee. Pure
PURSUIT waits wherever the chase ended: no agreed point is part of that
method, which is exactly the A/B against hybrid.
```

### pursuit-fallback-link-veto

**Link veto at the fallback dispatch site** — attached to `const rclcpp::Time fb_now = this->now();`

```text
THE SECOND DISPATCH SITE, and the reason the veto at the mid-run trigger
was not enough on its own. gt1's first 11 cells put the trigger-site leak
at 0 fires with the radio up (against 3 in the ungated tl1 arm), and left
exactly one leak standing — here. Measured over both arms, escalation from
an exhausted chase fired with the radio UP 2 times out of 2, and during a
genuine outage 0 times out of 2. That lopsidedness is not bad luck, it is
the mechanism: the trail runs out BECAUSE the peer moved on, and a peer
that moved on has usually come back into contact. So the site that most
reliably needs the veto was the one running without it.

It is also the more expensive of the two actions. The trigger site arms a
chase along a trail; this one falls back to exploring on the fallback
allowance and then parks at a barrier — in BOTH arms, identically, since
the hybrid meeting-point branch was deleted on 2026-09-16 — so a mistake
here costs the fallback allowance plus an open-ended wait in place.

Same freshness bound and same debounce as the trigger-site veto, on
purpose: two different notions of "the radio is up" in one node would make
any future disagreement between the sites unattributable.
```

### pursuit-fallback-resume-not-transition

**Why resumeExploring, not a bare transition** — attached to `resumeExploring(why);`

```text
resumeExploring, not a bare transition: it stamps the mid-run cooldown
and sets reconnect_terminal_, without which every following PLAN tick
would see the same stale record age and re-dispatch until the attempt
budget burned out. The trigger-site veto would then stand down on each
of those, so the run would look quiet in the logs while having spent
its attempts.
```

### pursuit-fallback-appointment-standing

**Early chase end with an appointment standing** — attached to `if (appointment_armed_ && !appointment_departed_) {`

```text
P5: with an appointment standing, a chase that died EARLY has not reached
the boundary between the two spans — the deadline is still ahead, and
doPursue tests it first, so arriving here means the budget or the trail
ran out before it. Going back to exploring is then strictly better than
parking at the meeting point for the remainder: the appointment is kept
either way, and the ground covered in between is mission progress the
parked robot never earns. This is the same deferral dispatchReconnect
applies when the chase declines at the outset.

The undeparted case is the only one that can reach here (a departed
appointment is in RETURN_NAV, not PURSUE), so no ordering with the
deadline is needed — resumeExploring hands the outage back to doPlan's
departure block, which owns it from here.
```

### pursuit-fallback-no-hybrid-branch

**Removal of the hybrid fallback branch** — attached to `if (pursuitExploreFallback(why)) return;`

```text
NO HYBRID BRANCH HERE ANY MORE (2026-09-16). An exhausted chase used to
park hybrid at meetingPoint(pursue_rec_.self_pose, pursue_rec_.peer_pose)
— the midpoint of the pose pair the chase was armed from — while pursuit
fell through to the two lines below. That was the third and last of the
midpoint drives, and it goes for the same reason as the other two: a place
with no agreed time, in an arm whose only licensed behaviours are pursuit
and the rendezvous.

What reaches here is a hybrid chase that ran out of budget or trail with
NO appointment standing (the block above returns for the standing case).
With no appointment there is no agreed place either, so hybrid is doing
pure pursuit at that moment and takes pursuit's ladder unchanged.
```

## ExploPlannerNode::resumeExploring

### resume-exploring-exit-hygiene

**resumeExploring exit hygiene** — attached to `void ExploPlannerNode::resumeExploring(const char* why) {`

```text
Abandon the manoeuvre and go back to exploring. Mirrors holdForTeam's exit
hygiene (a driving state must be stopped explicitly; an open vantage claim
must be demoted) but lands in PLAN instead of at a barrier. The coverage
streak is cleared because the robot is genuinely resuming, not finishing:
leaving it satisfied would re-trigger the same dispatch on the next tick.
```

### resume-exploring-midrun-bookkeeping

**Closing mid-run bookkeeping in resumeExploring** — attached to `if (!reconnect_terminal_) {`

```text
Close the mid-run bookkeeping HERE and not only in transitionTo. That
clock block is gated on reconnect_active_, and a fallback whose chase was
DECLINED never armed a manoeuvre, so reconnect_active_ is false and the
block does not run. Without this the cooldown would never stamp — every
PLAN tick still sees missing_for past the threshold, so the robot would
re-dispatch and re-decline until it had burned all its attempts within
seconds — and reconnect_terminal_ would stay false, letting a LATER
terminal barrier resume exploring instead of ending the run.

AFTER the transition, not before: transitionTo stamps reconnect_end with
reconnect_terminal_, so setting it here first would label a mid-run
manoeuvre that actually ran (chase armed, budget spent, then fell back to
exploring) as terminal — contradicting its own dispatch event and moving
its duration into the wrong bucket. Idempotent by construction: when the
chase DID arm, transitionTo's block has already stamped and the guard
below is false.
```

## ExploPlannerNode::pursuitExploreFallback

### pursuit-explore-fallback-refresh

**Explore fallback dispatch and context refresh** — attached to `refreshDispatchContext();`

```text
No destination: the manoeuvre is being given up in favour of exploring,
and the next goal is whatever PLAN picks. NB a resume_exploring dispatch
has no matching reconnect_end when the chase was DECLINED — there was no
manoeuvre clock running to stop (see resumeExploring).

The refresh is the whole point on THIS leaf: refreshDispatchContext's own
comment names pursuitFallback as a path that "commits once a chase has
run", so without it peer_record_age_sec is stamped as of the chase TRIGGER
and understates the true age by up to the entire pursuit budget
(pursuit_budget_max_sec = 600 s) — always in the flattering direction, and
only on the post-chase subset, which is worse than uniform noise because it
biases exactly the comparison the field exists to support.
```

## ExploPlannerNode::holdForTeam

### hold-for-team-barrier

**Holding for the team at the current pose** — attached to `void ExploPlannerNode::holdForTeam(const char* why) {`

```text
Raise the RETURN_SYNC barrier at the CURRENT pose. Used by pure-pursuit
endings and by the no-record corner of the pursuit dispatch. PURSUE is a
driving state, so the platform must be stopped explicitly (the guard's
stationary-state assumption; see abandonNavGoal) — ceasing to publish
goal_pose alone leaves the navigator finishing the last goal it accepted. The presence
intent (kept fresh by the heartbeat, which fires in RETURN_SYNC) is what
lets the pursued teammate count us whenever it comes into range.
```

## ExploPlannerNode::publishPresenceIntent

### presence-coast-claim-disc

**Shrinking the claim disc while coasting** — attached to `const double claim_r =`

```text
The disc above is harmless for a PARKED robot: it sits wherever that
robot finished, typically far from the partner and out of radio range.
A COASTING robot drags it toward the partner and stops beside it — which
hands a 10 m frontier veto to the one robot still holding the run clock,
and being stationary it usually wins the proximity tie-break for
candidates near itself. That is a route by which this feature could make
runs SLOWER, so shrink the disc for the duration of the coast.

A token 0.5 m rather than 0.0: the receiver treats 0.0 as "unset" and
substitutes its own match radius (coord_claim_radius_m, which the yaml now
states outright at 10.0 instead of auto-resolving to fov_max_range), so
zero would restore the very disc being removed.
The beacon itself must keep going out — teamComplete counts presence, and
a silent finisher ages out of every peer's table within seconds.
```

## ExploPlannerNode::heartbeatTick

### hb-team-presence-clock

**Team-presence clock on the heartbeat** — attached to `if (coord_) {`

```text
Team-presence clock for the reconnect confirmation gate. It lives here, on
the heartbeat timer, rather than at the exhaustion check, because the whole
point is to have a reading that predates the long PLAN tick — sampling it
inside finishOrRendezvous would sample exactly the stale table the gate
exists to distrust. Note this timer is serialised behind the state machine
on a single-threaded executor, so it too can be starved; that is the
conservative direction (a starved clock looks older, so the gate waits).
```

### hb-gap-before-anchor

**Heartbeat gap read before the anchor** — attached to `const double hb_gap_sec =`

```text
How long since the PREVIOUS heartbeat. hb_last_tick_ is not updated until
the bottom of this function, so this is the real inter-tick interval and
it is read here, before the anchor, because the anchor is the thing it
invalidates. See the starvation note below; the WARN for the same
condition stays at the bottom with the rest of the suppression
accounting.
```

### rzv-mutual-stricter-clock

**Rendezvous runs on mutual contact** — attached to `const bool rzv_mutual = rendezvousTeamMutual();`

```text
THE RENDEZVOUS RUNS ON A STRICTER CLOCK THAN team_last_complete_time_,
and the difference is one-way contact.

teamComplete() counts RECEIVED intents. That is "I can hear them" and
says nothing about whether they can hear me — and doc/limitations.md §10
records one-way contact as a real observed mode, not a hypothetical. Run
the appointment machinery off that count and a single healed direction
is enough to break the design's central claim: the robot that can hear
re-stamps its anchor mid-outage, clears rendezvous_spent_ and re-arms on
a NEW anchor, while the robot that cannot hear keeps the old one. The
two t_meets then differ by the whole length of the outage so far —
unbounded, against a 240 s wait — and each robot's log says it kept the
agreement.

TeamModel::Peer::direct is exactly the missing half: it is true only
when we received from the peer inside the TTL AND the in_range_mask it
sent named us back. Both ends evaluate a symmetric condition, so both
stop stamping on the same physical event rather than on their own half
of it. Under healthy comms it is true continuously and this costs
nothing; the divergence is exactly the case it exists to catch.
```

### rzv-snapshot-before-handshake

**Freezing the snapshot before the handshake** — attached to `if (rzv_mutual) refreshRendezvousSnapshot();`

```text
P5: FREEZE THE PROBLEM **BEFORE** THE HANDSHAKE READS IT (2026-09-17).

This call used to sit below, inside the `if (rzv_mutual)` block, i.e.
AFTER the handshake round that consumes what it produces. The cost was a
whole wasted mutual tick, and not by a subtle route:
deriveRendezvousProposal's second line is `if (!have_rendezvous_snapshot_)
return out;`, so on the first mutual tick of a run the derive returned
empty, the snapshot was taken immediately afterwards, and the earliest a
proposal could exist was the NEXT tick.

At N=2 that is one heartbeat and nobody would notice. At N=4 it is the
difference between having a window and not having one: the whole-team
mutual condition holds for about two seconds at coord_heartbeat_hz=1, so
spending the first of them refreshing a snapshot nobody got to read is
spending half the window. (It does not, by itself, make the N=4 rung
tour-informed — at t<2 s the map has no tours to argmin over and the
fallback is the centroid either way. It stops the ordering from being a
SECOND reason for the same outcome, which is worth having precisely
because the first one is a property of the radio regime and this one is
not.)

Nothing else moves. Both calls run on the same tick, off the same
`pres_now`, under the same gate, so the instant the snapshot describes is
unchanged — this is a reorder within one instant, not a relaxation of
when the freeze may happen. In steady state it is a no-op in the literal
sense: refreshRendezvousSnapshot early-returns on every tick inside the
proposal period, so the order only has any effect on the ticks where a
refresh actually lands, and there it makes the derive read this tick's
world instead of the previous period's.

The anchor is deliberately NOT moved up with it. maintainRendezvousProposal
does not read rendezvous_anchor_time_ — the three anchor refusals that
used to matter are gone (see armAppointment) and t_meet is now the
committed integer rather than anchor+interval — so the stamp stays with
the rest of the mutual-tick bookkeeping below, where it is read.
```

### rzv-handshake-every-heartbeat

**One handshake round every heartbeat** — attached to `maintainRendezvousProposal(rzv_mutual);`

```text
ONE ROUND OF THE HANDSHAKE, EVERY HEARTBEAT, whatever the team looks
like. The gate moved INSIDE (it is the argument) because the three steps
this runs do not share a gate: deriving and committing still require the
whole team to be mutually in contact, but echoing the proposer's pair
must not, or the fleet can never converge in time to use the windows
when it is. See the declaration for the N>=3 deadlock this fixes.
```

### rzv-anchor-stamp

**The rendezvous anchor stamp** — attached to `rendezvous_anchor_time_ = pres_now;`

```text
THE ANCHOR. Both robots will convert the agreed INTERVAL into a t_meet
by adding it to their own last stamp of this line, so this assignment
is the shared origin the whole appointment hangs off. It is stamped in
the same breath as the snapshot and the handshake because all three
describe the same instant: the last moment the team was confirmed
whole.
```

### rzv-mutual-block-gates

**What the mutual block still gates** — attached to `} else if (hb_gap_sec >= team_model_.config().direct_ttl_sec) {`

```text
THE SNAPSHOT USED TO BE TAKEN HERE. It now runs a few lines above, on
the same tick and under the same `rzv_mutual` condition, so that the
handshake round can actually read it — see the comment at that call for
why the ordering was costing a whole mutual tick. The property this
spot existed to defend is unchanged: the freeze happens only while the
team is confirmed mutually in contact, which is the last instant at
which every robot is known to be looking at the same world, and an
appointment derived from anything later is one the peers never
computed.

The handshake used to be called HERE, inside this guard. It now runs
above it on every heartbeat, with this same condition passed in as an
argument. What the move buys is that the ECHO does not take the gate,
which is what lets an N>=3 fleet ever converge: a follower that can
hear the proposer but not yet every sibling has to be able to adopt and
re-publish, or the mask never closes and nobody converges. Whatever was
last COMMITTED is still exactly what the outage inherits.

WHICH STEPS STILL REQUIRE WHAT, precisely, because "they take this gate
internally" was written here when it was true of both and is not:

  derive  takes exactly this gate. The snapshot it argmins over is only
          meaningful if every robot is looking at the same world.
  commit  does NOT test the mask. It requires something strictly
          stronger and of a different kind: a matching echo from every
          other robot, each one first-hand evidence that THAT robot
          holds THESE three integers. The mask is a claim about
          connectivity at one instant; the echoes are the agreement
          itself. Re-adding a mutual test there would only be able to
          veto commits the echoes had already proved sound.

The outage is over, so the one-appointment-per-outage guard is
released. THIS IS THE ONLY SITE THAT CLEARS IT — closeAppointment does
not, deliberately (see its declaration): closing an appointment is
what SPENDS the outage's one arming, so releasing the flag there would
hand out a second one inside the same outage.

The consequence is that the flag outlives the reunion by up to one
heartbeat period, because the reunion paths run on the 10 Hz planning
tick and this runs at coord_heartbeat_hz. That lag is in the safe
direction: while the flag is still set armAppointment refuses, so the
worst case is one missed arming in a window where the team is already
back together and no reconnect manoeuvre is wanted anyway.

THE ONE-APPOINTMENT-PER-OUTAGE RELEASE MOVED OUT OF THIS BLOCK
(2026-09-18). It used to be `if (!appointment_armed_)
rendezvous_spent_ = false;` right here, and being here is what broke
the rendezvous arm at N>=3.

The condition on this block is rendezvousTeamMutual() — EVERY pair in
the fleet in direct contact. At N=2 that is just "the link is up" and
recurs constantly. At N>=3 in this radio regime it holds for a few
seconds at spawn and then never again, because it needs all N(N-1)/2
links up simultaneously and the forest takes them down independently.
So the flag latched true after the FIRST appointment and every later
separation refused with "the agreed pair has already been used in this
outage" — measured in the ts4 N=3 cell as five refusals on bestla and
two on husky, leaving the arm inert from t~170 s to the end of a 604 s
run. 70% of the cell, on the arm the cell exists to measure.

The release now lives below this block, on
teamComplete(accountedPeerCount)
— the same predicate the barrier releases on and the same one that ends
an outage everywhere else in the node. "The outage is over" gets ONE
definition instead of two, and it is the weaker, recurring one, which
is the correct one here: what the flag guards against is re-arming
INSIDE an outage, and an outage has ended as soon as I can hear
everyone again — whether or not every pair is in contact, and whether
or not they can hear me.
```

### rzv-anchor-starved

**Suspect anchor after a heartbeat gap** — attached to `if (!rendezvous_anchor_starved_ && have_rendezvous_anchor_) {`

```text
THE ANCHOR MAY HAVE BEEN MISSED ENTIRELY.

This function is a timer on a single-threaded executor, serialised
behind the state machine and the metrics sampler — hb_late_count_ at
the bottom of this function exists because those genuinely do overrun.
Everywhere else in this node a starved tick is conservative: a clock
that stops reads OLDER, and the gates wait. The anchor is the one place
it is not. It is a LATCH of the last instant mutual contact was
observed, so a gap that straddles the mutual -> not-mutual transition
leaves it pointing at the last tick BEFORE the gap rather than at the
separation, and t_meet = anchor + interval is then wrong by up to the
whole gap — on this robot only. The peer, whose executor was not
blocked, holds the right one. Both keep an exact appointment, at two
different times, and both logs record it as kept.

There is no way to recover the instant after the fact, so the answer is
to stop claiming to know it, and the flag clears on the next healthy
stamp above. The threshold is the TeamWorld direct TTL because that is
the window in which `direct` can go false unobserved: a gap shorter
than it cannot have hidden the whole transition.

THIS NO LONGER AFFECTS THE APPOINTMENT (2026-09-17), and the text below
used to say it did — "Refusing to arm an appointment until mutual
contact is observed again", describing a refusal armAppointment had
already stopped performing. Both halves of the old claim died with the
triple: t_meet is adopted verbatim off the wire rather than computed as
anchor + interval, so a suspect anchor can neither move this robot's
meeting instant nor be a reason to refuse. What it still spoils is the
`anchor_sec` COLUMN, which is how the analysis separates an early
agreement from a late commit — so the WARN stays, saying only what is
true. An operator triaging a bad meeting must not read this line as
evidence that no appointment was armed: one almost certainly was.
```

### rzv-spent-release

**Releasing the one-appointment-per-outage latch** — attached to `const bool team_back_dwelt =`

```text
THE ONE-APPOINTMENT-PER-OUTAGE RELEASE (2026-09-18), moved out of the
rzv_mutual branch above — see the note there for the N>=3 failure that
forced it. Deliberately OUTSIDE that branch and keyed on
teamComplete(accountedPeerCount): every robot I can currently hear.

THERE IS NO RELAY. Until 2026-09-18 this line, and three others in the
file, said the count included "every robot reachable, directly or by
relay". That was never true, and it is the wrong mental model to carry
into an N>=3 result. Three independent confirmations, none of them a
comment: Coordination's claim table is filled only by onIntent(), which
runs on the coord_intent_sub_topics_ subscriptions — the emulator's
per-link copies /<self>/rx/<peer>/... , one per PEER, republished only
while that ORDERED PAIR's link is up; hmr_comms_sim_node computes one row
per unordered pair and forwards nothing, so a message never traverses a
third robot; and RobotIntent carries no peer list, no in_range_mask and
no hop count, so a claim cannot arrive second-hand even in principle.
Transitive closure exists in exactly one place in this system —
TeamModel::inComms(), fed by the in_range_masks on TeamWorld — and
accountedPeerCount is not it.

WHAT IT ACTUALLY IS: for every peer still running, a FIRST-HAND read that
needs contact in at least ONE direction. "I have heard each of them within
one TTL" — on the claim table, which is one-way, or on TeamWorld's
`direct` flag, which is the two-way handshake — "or they told me their run
is over", and THAT last clause is neither first-hand nor TTL'd: the
`finished` bit relays through third robots and never clears, so a finished
peer is counted present on no contact at all. It is still not a relay of
REACHABILITY, which is what the paragraph above is about, and it is why
the sentence says "for every peer still running". Silent on
whether a claim-only peer has heard me, so the union is still strictly
weaker than rendezvousTeamMutual(), and every argument built on this
predicate stands. It is weaker for a different reason than the old
comment claimed, and the difference is observable. At N>=3 with A and B
out of contact while C hears both: under the relay reading A would count
B through C and would NOT arm; under what this count does, C's team reads
complete while A's and B's do not. A reader predicting the run's
behaviour from the old sentence would predict the wrong robots.

WHAT HAPPENS IN THAT TOPOLOGY NOW (generation 23) IS A THIRD THING, and
it is not a relay: A and B announce their own broken reads on
TeamWorld/team_incomplete, C hears them first-hand, and teamSettled folds
that in, so all three arm. The contagion sits BESIDE the count, not
inside it: accountedPeerCount unions two first-hand CHANNELS for the same
question ("can I reach this peer") — plus the `finished` channel, which
answers "is this peer worth reaching" and is the one exception to
first-hand — while peerReportsTeamBreak answers a third question ("does a
peer I can reach say someone else cannot").
Keeping them separate is what makes the paragraph above a correct
description of the COUNT.

WHY THE WEAKER PREDICATE IS THE RIGHT ONE. The flag's whole job is to
stop a second arming INSIDE one outage — without it the closed
appointment re-arms on the next tick and the robot drives back to the
cell it just left. That hazard ends the moment I can hear the team again,
which is what this predicate says. Requiring every pair to be in DIRECT
contact was asking for evidence of something the guard never needed, and
at N>=3 that evidence essentially never arrives.

It is also the same predicate as the barrier release's mesh half and
the outcome classifiers' mesh half, so "the outage is over" has one
meaning here. (The generation-27 reachable door is deliberately ABSENT
from this site: this is the LATCH release, and clearing it on the
weaker count would let the arm re-fire inside the outage — the next
paragraph is the argument, and its holding is exactly what makes the
door safe at the barrier.) Two definitions of one event is what
produced a robot logging a no-show in the same millisecond as its own
reconnection.

AND SINCE GENERATION 23 THAT ONE MEANING IS teamSettled — teamComplete
plus "no peer I can currently hear is announcing a break". It has to move
with the other four sites, and here the direction matters: this is the
LATCH RELEASE, so a predicate weaker than the arming site's would clear
the latch while the arming site still reads the team broken, and the arm
would re-fire inside the same outage — one appointment per outage becomes
one per tick. That is the generation-18 polarity of the same defect, and
it is why the test above must be the complement of the arming test rather
than merely close to it.

NOT WHILE ONE IS STILL STANDING. Unchanged, and still the second lock:
an appointment that is armed has not been closed yet, and releasing the
flag under it would hand out a second arming while the first is live.
closeAppointment still does not clear it — closing is what SPENDS the
arming — so the only path back is a reunion observed here.

AND IT DWELLS (generation 23), for the same reason the barrier does and
with the same parameter. Matching the arming site's PREDICATE is only half
the complement argument; the other half is the evidence threshold, and
until now this site had none. teamSettled can read true for one heartbeat
on a single claim arriving inside the 5 s TTL — and the arming site, which
runs at 10 Hz on a predicate that is false the moment one beacon is
missed, will then re-fire inside the same outage. That is exactly the
one-per-outage guard failing open, on a flicker. Releasing the latch six
seconds late costs at most a delayed NEXT arming in a window where the
team is genuinely back; releasing it on a flicker is the generation-18
defect returning.

THIS IS THE SOLE WRITER of the team-back window, and it is deliberately
outside every test below it — including the `!appointment_armed_` one.
dwellConfirmed measures a CONTINUOUS run, and a window that is not ticked
does not decay, it FREEZES: written as `!appointment_armed_ &&
dwellConfirmed(...)` the clock would stop for the whole life of a standing
appointment, and the first heartbeat after closeAppointment would find
`now - since` already far past the confirm time and release on one sample.
A guard that skips its own clock is worse than no guard, because it reads
as one. Here it is stepped once per heartbeat unconditionally, so the run
it reports is a real one, and doPlan's P5 supersede test reads the same
window through dwellHeld without disturbing it.
```

### hb-suppression-accounting

**Heartbeat suppression accounting** — attached to `const bool beaconing =`

```text
Suppression accounting (comms experiments). The beacon is STATE-GATED, so
a planner busy longer than coord_claim_ttl_sec in a non-beaconing state —
PLAN above all, which can loop indefinitely when every candidate is
rejected — reads as *missing* in every peer's claim table under perfect
comms. That is indistinguishable from a radio outage from the receiver's
side, and it is enough to arm a reconnect manoeuvre against a healthy
teammate. Logging the episodes here is what lets the analysis classify
each peer-missing window as outage (corroborated by the emulator's
link_states) or suppression (corroborated by these lines) instead of
charging planner latency to the radio.
```

### hb-executor-starvation

**Executor starvation as a suppression cause** — attached to `if (hb_last_tick_.nanoseconds() > 0) {`

```text
Second, independent suppression cause: EXECUTOR STARVATION. The block below
keys entirely off state_, so it can only see a beacon that was never
attempted. A beacon that was attempted LATE is invisible to it — and this
node spins a single-threaded executor, so every timer here is serialised
behind the state machine and behind the metrics sampler, whose map ingest +
grid walk grows with the fused map (millions of voxels by late run). If one
of those callbacks runs longer than coord_claim_ttl_sec the beacon simply
does not go out in time, peers age the claim out, and the analysis charges
a healthy link with an outage. Measuring the actual inter-tick interval is
the only way to see it from inside the node.
```

### hb-suppress-warn-episode

**One suppression warning per episode** — attached to `if (!hb_suppress_warned_ && held >= coord_claim_ttl_sec_) {`

```text
One WARN per episode, at the moment peers can first read us as gone.

NOT IN WAIT_FOR_MAP (2026-09-17, generation 22). That state is entered
once, at construction, and never re-entered — line 2223 is the only
assignment and there is no transitionTo(WAIT_FOR_MAP) anywhere — so
`state_ == WAIT_FOR_MAP` means exactly "this robot has not started its
mission yet". Two things follow, and both say the WARN has nothing to
report here. First, the claim is vacuous: a robot that has never
published an intent is not a robot peers have STOPPED hearing, and the
episode it opens cannot overlap any peer-missing window, because
peer_belief_ is populated from the intent stream (expPeerHeard) and is
still empty on every robot in the team — expPeerSweep returns on
`peer_belief_.empty()` before it can measure anything. Second, and the
reason this became a defect rather than a curiosity: gen 22's
mission_start_hold_sec holds every robot here for sixty seconds, twelve
claim TTLs, so before this gate the WARN fired once on EVERY robot of
EVERY cell. A line that fires unconditionally is not a diagnostic; it is
noise that trains an operator to skip the string, and the string is the
only in-node evidence that separates executor-induced suppression from a
radio outage. Suppressing it in the one state where it is guaranteed and
uninformative is what keeps it meaningful everywhere else.

THE EPISODE ITSELF IS NOT SUPPRESSED, only this WARN. hb_suppressed_,
hb_suppress_start_ and the "Heartbeat resumed after %.1f s suppressed"
INFO below are all untouched, so the hold still leaves exactly one line
in the log marking its length — which is the line worth having.

THE EXEMPTION IS EPISODE-SCOPED, NOT INSTANT-SCOPED, and that distinction
is the whole of the 2026-09-18 fix. A first cut tested `state_ !=
WAIT_FOR_MAP` at the WARN site and nothing else, which was wrong by about
one second: the state leaves WAIT_FOR_MAP when the hold expires, but the
heartbeat does not resume until the executor next runs, so there is a
window in which the robot is already in PLAN while `held` is still the
sixty-second hold. A tick landing in that window found a passing state
guard, an unset latch, and held >= TTL, and fired the exact line the gate
existed to prevent — reported as "suppressed 60.1 s in state PLAN". It
was intermittent, which is worse than always: it fired in 2 of 6 banked
gen-22 cells, so a reader who checked one clean cell would have concluded
the gate worked.

CONSUMING the latch instead of skipping the WARN closes it, because the
latch already has exactly the right lifetime. hb_suppress_warned_ is
reset when an episode BEGINS (above), not when one ends, so marking it
spent here silences this episode and only this episode; the next
suppression clears it again and warns normally. No clock, no second
member, no reliance on the ordering of two asynchronous transitions.
```

### hb-beacon-state-gate

**States that keep the beacon going** — attached to `if (state_ != State::NAVIGATE && state_ != State::INTEGRATE &&`

```text
Re-publish while we hold a claim: NAVIGATE/INTEGRATE (exploration), the
EXPLOIT states, the RETURN states, PURSUE, and DONE. The dwell in
particular can outlast the claim TTL, so a peer would otherwise poach the
vantage angle mid-capture; in RETURN/PURSUE the beacon is what lets
teammates coming back into range count us and release the barrier; in
DONE (idle — finishOrRendezvous publishes the presence intent only then)
it is what keeps the FIRST finisher countable, so a teammate finishing
minutes later sees a full team instead of chasing a parked robot and
waiting forever. PROXIMITY_HOLD keeps beating too: the interrupted goal
is resumed after the hold, so its claim must survive, and the beacon
(with the live robot_pos refreshed below) is what feeds the right-of-way
peer's view of us while we sit in its way.
```

### hb-refresh-robot-pos

**Refreshing robot_pos on the heartbeat** — attached to `current_intent_msg_.robot_pos.x = latest_pos_.x();`

```text
Refresh robot_pos too, not just the stamp. Coordination::selfWinsAgainst
compares the receiver's LIVE pose against this field, so a frozen value
breaks the MinPos guarantee that exactly one robot yields per pairwise
conflict — and it breaks it in the harmful direction: as we close on our
own claimed goal we keep advertising the far-away pose we held at claim
time, a peer computes that it is the closer robot, and it poaches a goal
under active pursuit. nav_max_timeout_sec is 180 s in every campaign
(shared_params.yaml:260, and run_explo_sim_rviz.sh passes it explicitly;
the 60.0 at the dp() site below is a fallback default no campaign uses),
which is 36x the claim TTL (5 s) — and the heartbeat exists precisely to
hold a claim across a long hop, so the stale pose was broadcast for that
entire window.
```

## ExploPlannerNode::checkProximityHold

### proximity-stop-rules

**Proximity stop right of way** — attached to `bool ExploPlannerNode::checkProximityHold() {`

```text
==================================================================
Proximity stop (coordinated yield)
==================================================================

While driving (NAVIGATE / RETURN_NAV), yield to a higher-priority teammate
moving nearby: brake with a goal at the robot's own pose, hold still, and
resume the same goal once the teammate has cleared off (hysteresis) or
parked. Right of
way is the lexicographically SMALLER robot_name — the same total order as
the MinPos tiebreak — computed from ids alone, so both robots always agree
on who yields: exactly one of any pair stops, never both (standoff) and
never neither (race). See ProximityGuard for the freshness/parked rules.

This is a best-effort COORDINATION layer, not a certified safety function:
it needs live peer pose data (intents at 1 Hz + the optional localiser
topics at ~10 Hz) and both planners alive, and the stop rests on the brake
goal alone — the nav2 cancel attempted beside it has no server here and never
fires. The right-of-way robot does NOT stop; it relies on its own local
obstacle grid seeing the held robot as an ordinary obstacle. The crewed 1.5 m
panic stop from the experiment script remains the hard backstop.
```

## ExploPlannerNode::enterProximityHold

### prox-hold-brake-goal-stop

**How the proximity hold stops the robot** — attached to `if (nav_cancel_client_ && nav_cancel_client_->action_server_is_ready()) {`

```text
Stop the platform. Merely CEASING to publish goal_pose does not stop the
robot: simple_nav_3d's navigator latches the last goal it accepted and keeps
feeding it to the planner and controller until the robot arrives. The stop
is therefore commanded, by a brake goal at the robot's own pose — which
reads as an immediate arrival, empties the path within ~100-150 ms and puts
a zero Twist on cmd_vel.

The cancel below is a nav2 leftover and NEVER FIRES on this stack; see the
block at nav_cancel_client_'s construction. It is not a second, independent
stop mechanism, so do not read this as belt-and-braces: the brake goal is
the only thing stopping the robot, and the else-branch WARN says so every
time. current_goal_ is left untouched; the resume re-publishes it fresh.
```

## ExploPlannerNode::abandonNavGoal

### abandon-nav-goal-brake

**Why abandoning a hop must brake** — attached to `void ExploPlannerNode::abandonNavGoal(const char* why) {`

```text
Stop the platform on a transition that ABANDONS the in-flight hop instead of
replacing it. Same wire mechanics as enterProximityHold, no hold state.

The invariant: the proximity guard runs ONLY in NAVIGATE / RETURN_NAV /
PURSUE (see tick()), on the explicit assumption that every other state is
stationary. So
any transition out of a driving state that does not IMMEDIATELY publish a
replacement goal must stop the platform itself — because ceasing to publish
goal_pose does NOT stop the robot: simple_nav_3d's navigator latches the last
goal it accepted and drives it to completion. Without this, the robot keeps
rolling in a state the guard has been told is standing still, which is the one
combination the coordination layer cannot see.

That is not hypothetical. In a 2-robot run both robots' rendezvous-synced
dwells ended 3 ms apart, both re-planned in the same instant, and both picked
the last remaining vantage of the target before either had heard the other's
1 Hz claim. The loser took the cross-pick yield in doNavigate, hopped to
EXPLOIT_PLAN — and its re-plan found NOTHING selectable (two vantages
visited, the third claimed by the winner), so it never published another
goal. The navigator spent the next 29 s driving a robot the guard believed was
"planning" 6.6 m across the ring into the peer standing on the very vantage
it had just yielded. They collided.

The brake goal is the stop. The cancel attempted alongside it is a nav2
leftover that never fires here (no action server — see nav_cancel_client_'s
construction), so this is ONE mechanism, not two, and the INFO line below no
longer claims otherwise. The brake is harmlessly superseded the moment a later
tick does select a real goal — it costs one goal_pose message.

Call sites are exits from DRIVING states, plus holdForTeam's barrier entry
(reachable from PLAN/LOG_STEP via the pure-pursuit no-chase corner, where
the cancel is a no-op and the brake zero-travel). All of them run long
after the first pose, so latest_pos_ is live; do not call this from a state
entered before the first pose, where it would command the frame origin.
```

### abandon-nav-warn-once

**Why the abandon cancel warning fires once** — attached to `RCLCPP_WARN_ONCE(get_logger(),`

```text
WARN_ONCE, not WARN. On the sim stack this branch is taken on every
single abandon — 281 identical lines across the mr1 campaign — and a
warning that fires every time is a warning nobody reads. Once per run is
enough to establish the fact; the per-abandon truth now lives in the
INFO line below, which no longer claims a cancel happened.
```

### abandon-nav-info-wording

**Abandon log reports only what happened** — attached to `RCLCPP_INFO(get_logger(),`

```text
Say what was actually done. The old wording — "cancelled + braking in
place" — was printed unconditionally, including on the 281 campaign
abandons where no cancel was sent at all because simple_nav_3d serves no
action server. Reading those logs, every abandon looked like a commanded
stop; the robot was in fact still driving to the old goal until the brake
pose overrode it. That false line cost a diagnosis once already.
```

## ExploPlannerNode::doProximityHold

### prox-hold-escape-grace

**Escape hatch grace window** — attached to `prox_guard_->armEscape(d.peer_id, now);`

```text
Make the hatch an actual escape: the peer is (by construction of this
branch) still inside the trigger disc, so without a grace window the
very next tick's entry check re-holds and the "escape" is one 0.1 s
tick of driving between max_hold_sec holds, forever — exactly the
wedge the yaml's escape-hatch comment promises this parameter breaks.
The guard drops the immunity early if the peer starts moving again.
```

### prox-hold-resume-same-goal

**Resuming the same goal after a hold** — attached to `if (exploit_target_timing_) exploit_target_started_sec_ += held;`

```text
Resume the interrupted drive on the SAME goal. The re-publish is mandatory,
though not for the reason originally written here ("the cancel consumed the
goal"): the cancel never fires. It is mandatory because the BRAKE goal
replaced the real one at the navigator, and the navigator is now parked on
it — only a fresh goal_pose restarts the drive. state_enter_time_ is backdated by the drive time the goal
had already consumed, so the nav budget CONTINUES across the hold; the
progress window starts fresh (held time is not lack of progress). The
exploit give-up timer gets the held time back for the same reason: a
hold is not target stall.
```

### prox-hold-escape-leg-refund

**Refunding held time to the escape leg** — attached to `if (appointment_escape_active_) {`

```text
An appointment escape leg in flight gets it back for the third time and the
same reason: return_escape_leg_sec is 30 s and proximity_max_hold_sec is
120, so without this a single hold expires the leg cap while the robot is
deliberately braked. The first tick after release would then end a detour
that was never driven — and three such holds spend the whole ladder without
the platform moving, which is precisely the ladder failing to do the one
thing it exists for.
```

### prox-hold-approach-watchdog-reset

**Resetting the homing watchdog after a hold** — attached to `approach_check_time_   = now;`

```text
The homing approach watchdog gets the same treatment as the frozen one
directly above, and for the same reason: held time is not lack of
progress. doReturnHome does not run while state_ is PROXIMITY_HOLD, so
without this the 40 s approach window free-runs on the clock while the
robot is deliberately braked, and the first tick after release compares a
window of (real drive + hold) against a threshold calibrated on driving
alone. That fires on release by construction whenever a hold of 34 s or
more lands early in a window — and with proximity_hold_dist_m at 5.0 m
against homes 3 m apart, both robots homing at once is exactly when it
happens. Charging a commanded yield-to-teammate as a homing stall would
put a systematic penalty on the coordination behaviour under study.
```

## ExploPlannerNode::fillCommonMetrics

### csv-fill-common-metrics

**Which columns fillCommonMetrics sets** — attached to `void ExploPlannerNode::fillCommonMetrics(StepMetrics& m) {`

```text
Every column that is a property of the world at this instant, as opposed to
an attribution of the last plan. Shared by the end-of-step row and the timer
row; the plan-attribution columns (plan_time_ms, mean_/selected_*, the
rejection counts, the exploited vantage) are deliberately NOT set here, so a
timer row leaves them zero instead of repeating the last plan's numbers on
every sample and inviting the analysis to average them.
```

### csv-plan-rejections-on-timer-rows

**Plan rejection columns on timer rows** — attached to `m.plan_cand_total    = pending_plan_cand_total_;`

```text
Rejection profile of the last planning attempt. Filled HERE, in the common
path, rather than in doLogStep with the other plan diagnostics — that is
the entire fix. doLogStep runs only when a step completes, and a planner
rejecting every candidate completes none, so these are the columns that
have to survive on a timer row or they are absent exactly when they matter.
```

### csv-team-complete-sentinel

**The team_complete column and its sentinel** — attached to `if (coord_ && rendezvous_expected_peers_ > 0) {`

```text
R5: the two predicates of §3.4, side by side, on every row.

team_complete is written whenever there IS a coordination table to ask,
not only during a manoeuvre — the disagreement is about what the team
looked like at the moment a chase released, and a row that only exists
inside the manoeuvre cannot show the tick after it. -1 stays "there was no
peer table", which is a third thing and not a zero.

Both use accountedPeerCount / peerAccounted, the calls the release predicate
and the outcome classifier make, so the CSV cannot disagree with either.
The expected-peers test is here and not left to teamComplete(), which
folds "not configured" into "not complete": planner_util.cpp returns
`expected_peers > 0 && active >= expected`, so with the yaml default of 0
("0 = inert") every row would read a clean, measured-looking 0 and
mean(team_complete) would come out 0.0 on a run where the question was
never asked. -1 is "there was no team to ask about", a third thing, and the
same sentinel discipline the pursuit columns above use. Invisible on the
campaign path -- the harness always passes N-1 -- and live for any bare
`ros2 run`, which is exactly the case with nobody watching.
```

### csv-pursuit-live-gate

**Gating pursuit columns on a live chase** — attached to `const bool pursuit_live =`

```text
Gated on the pursuit being LIVE, not on the state being PURSUE. A
proximity hold taken mid-chase parks the robot in PROXIMITY_HOLD while the
chase is still running — this file says so itself where the release
refunds the held time to pursue_start_time_, "yielding to a teammate is not
evidence the chase hypothesis is wrong". Keying on state_ alone wrote
pursue_quarry_live=-1 ("not pursuing") on those rows, and they are not a
random sample of the chase: a hold needs a MOVING peer inside hold_dist_m,
i.e. it concentrates in the closing seconds, exactly where the §3.4 case
(quarry live, team still incomplete) has to be read. Worse, right-of-way
goes to the lexicographically smaller robot_id, so the blindness is
deterministic by NAME — atlas chasing bestla never holds, bestla chasing
atlas does — and the bias is unbalanced across id pairs rather than noise.

pursue_peer_id_ is never cleared once set (see startPursuit), so a
non-empty test on its own would keep writing the column for the rest of the
run; the state pair is what bounds it to a live chase.
```

### csv-coverage-milestone-hook

**Coverage milestones share the CSV measurement** — attached to `if (exp_log_) {`

```text
Coverage milestones ride the SAME measurement the CSV row and the DONE
criterion use, taken on the same tick — so "time to reach coverage X" can
never disagree with the curve it is read off. This is the hook, not a
separate sampler: every emitter of a CSV row (the end-of-step row and the
periodic timer row, which runs in every state including the whole of a
reconnect manoeuvre) passes through here, so the ladder is evaluated at the
sampling period even while the step counter is frozen.
```

### csv-cell-census-same-tick

**Cell census sampled with the coverage** — attached to `updateCellWorld(uf, cov_src);`

```text
The coarse cell census, taken here for the same reason the milestones are:
it reads the map that `uf` above was just measured on, on this tick. P1's
gate asks whether the census agrees with `uf` as coverage saturates, and
this sim is nondeterministic enough that sampling the two from different
hooks would have them measuring different maps. No-op when disabled.
```

## ExploPlannerNode::metricsTick

### metrics-tick-periodic-sampler

**Why the CSV has a periodic sampler** — attached to `void ExploPlannerNode::metricsTick() {`

```text
Sample the CSV on a periodic timer, in every state. The period is SIM time
(use_sim_time), so at RTF != 1 it is not a wall-clock period; the adaptive
back-off below is the only part measured on a steady wall clock, because it
bounds executor-thread work. Steps only advance through
the explore loop, so without this a robot that spends four minutes in PURSUE
or RETURN_NAV contributes one flat segment between two step rows across the
exact interval the comms experiment measures.
```

### metrics-tick-self-throttle

**The metrics sampler self-throttle** — attached to `const auto mt_now = std::chrono::steady_clock::now();`

```text
Self-throttle. This callback is NOT cheap and gets more expensive as the
run proceeds: loadLatestMap() reallocates and re-inserts the whole fused
grid whenever a new map has arrived (dscovox publishes at 1 Hz, so at any
sane period one always has), and computeStats() then walks every voxel
evaluating digamma/log terms plus up to six neighbour lookups per free
voxel. Prior campaign CSVs reach ~4M voxels with plan_time_ms ~1700, so on
a single-threaded executor a fixed 5 s period would spend a large and
GROWING fraction of the node's only thread here — delaying the 1 Hz
coordination beacon past coord_claim_ttl_sec and manufacturing exactly the
peer-missing signal this experiment is trying to attribute to the radio.

So the period floats: measure each tick, and if it cost more than
metrics_max_duty_ of the current period, stretch the period until it does
not. The sampling rate degrades (visibly, in the log) instead of the
planner's real-time behaviour degrading (invisibly, in the data).
```

### metrics-tick-reingest-first

**Re-ingesting the map before sampling** — attached to `loadLatestMap();`

```text
RE-INGEST FIRST — the whole reason this function is more than three lines.
total_observed_voxels is read off map_cache_, and map_cache_ is only ever
rebuilt by loadLatestMap(), which outside WAIT_FOR_MAP/PLAN is called by
nothing but the exploit states. A sample taken without this would report the
voxel count frozen at whatever the last PLAN ingested, for the entire
manoeuvre: a coverage curve that goes flat the moment a robot loses contact
whether or not it kept mapping — manufacturing precisely the result the
experiment is supposed to be testing for. Cheap when no new map has arrived
(pointer-identity check, no rebuild).
```

### metrics-tick-realised-rate

**Realised sampling rate accounting** — attached to `++metrics_rows_written_;`

```text
Realised-rate accounting for this sampler, reported in the event log's
run_end. The configured period is still not a guarantee: the gate above
is a WALL-clock deadline (it must be — it bounds executor work), so the
sim-time spacing of rows scales with RTF, and the duty back-off below
stretches it further under load. Measured from the sim stamps of the rows
actually written, which is the quantity an analysis needs.
```

### metrics-tick-phase-locked-deadline

**Arming the next sampler deadline** — attached to `const auto eff_dur =`

```text
Arm the next deadline from THIS tick's scheduled slot, not from "now"
after the row was written: post-work arming adds the tick's cost to every
period, and when the tick source runs at the same period (sim-clock timer
at RTF >= 1) that constant overshoot suppresses every second firing —
halving the realised rate with no config change, by an amount that
tracks how expensive the map walk happens to be at that point of the
run. Advancing the previous deadline keeps the schedule phase-locked to
the tick source; a schedule that has fallen behind re-anchors instead of
firing a catch-up burst.
```

### metrics-tick-coverage-latch-hook

**The coverage latch hook in metricsTick** — attached to `maybeLatchCoverageDone(last_unknown_fraction_, last_coverage_source_);`

```text
THE DECIDING HOOK for done_criterion == "latch". This callback is the only
thing in the node that measures the ROI unknown fraction in EVERY state —
it re-ingests the fused map at the top precisely so the number stays live
during a reconnect manoeuvre — which is exactly the property the criterion
needs and exactly what the old top-of-doPlan test lacked. last_unknown_-
fraction_ was cached by the fillCommonMetrics call above, so the row that
reports the qualifying fraction and the tick that acts on it are the same
tick and cannot disagree. (The cached double, not m.unknown_fraction: that
field is a float and the comparison is against a threshold given in
decimal.)

LAST in the function, not first, for two reasons: the qualifying sample is
data and must reach the CSV whatever the latch then does with it, and the
row/rate accounting above must count that final row or the realised
sampling rate reported in run_end is short by one.

Safe to transition from here — this timer shares the node's default
(mutually-exclusive) callback group with tick(), so no state-machine
callback can be halfway through when this runs.
```

## ExploPlannerNode::expCtx

### explog-ctx-sim-clock

**Event log uses the node clock** — attached to `c.sim_time_sec = this->now().seconds();`

```text
The node's OWN clock, which is sim time under use_sim_time. The single
reason this file exists: every event in the log shares the axis the planner
actually makes decisions on, so nothing downstream has to map wall-clock
log stamps onto sim time through a real-time factor that drifts within a
run.
```

## ExploPlannerNode::startExperimentLog

### explog-start-waits-for-clock

**Event log start waits for /clock** — attached to `const auto now = this->now();`

```text
Under use_sim_time this->now() reads exactly 0 until the first /clock
message lands, and t0 = 0 would turn every t_rel_sec in the file into an
absolute sim time wearing a relative name — the precise class of silent
axis error this log replaces. Wait for a real stamp instead; the tick timer
itself only fires on that same clock, so this costs nothing.
```

## ExploPlannerNode::expPeerHeard

### explog-peer-belief-source

**Where the event log's peer belief comes from** — attached to `void ExploPlannerNode::expPeerHeard(const std::string& peer_id) {`

```text
Peer belief. Deliberately maintained from the intent stream itself rather
than read out of Coordination's claim table: the control arm runs with
coordination_enabled=false, where that table is never consulted, and outage
timing is exactly what the control arm exists to provide a baseline for. The
threshold is coord_claim_ttl_sec, so the belief means the same thing the
barrier's presence test means — and peers_live (which IS read from
Coordination) rides on every peer event so the two can be cross-checked.
```

### explog-peer-belief-before-run-start

**No peer belief before run_start** — attached to `if (!exp_log_->started()) return;`

```text
Do not seed a belief before run_start. Under use_sim_time this->now() is 0
until the first /clock message, and run_start is deliberately held for the
first tick with a live clock — so an intent that arrives in that window
stamps last_heard at time 0 and marks the peer live. The peer_seen event
itself is dropped (logPeerSeen counts it in dropped_before_start_), but the
POISONED BELIEF survives, and expPeerSweep later measures silence against
time 0: g6pilot_hybrid_seed105/bestla logged peer_lost at t_sim=26.91 with
silent_sec=26.91 and t_rel=0.0, a 27 s outage that never happened, and that
run is also the only one of 24 missing its first_contact=true event.

Returning here costs at most one heartbeat period (intents arrive at 1 Hz):
the next intent creates the belief with a real stamp and logs first contact
properly. Nothing else reads peer_belief_ — coordination runs off coord_
and last_contact_ — so this is confined to the event log's own bookkeeping.
```

## ExploPlannerNode::refreshDispatchContext

### explog-dispatch-context-refresh

**Refreshing dispatch context at commit** — attached to `void ExploPlannerNode::refreshDispatchContext() {`

```text
One event per manoeuvre DECISION, emitted by the leaf that commits the
action. The alternative — logging inside dispatchReconnect's branch walk —
cannot see the hold-escalation dispatch (which re-enters startReturnTo
straight from the barrier) or the hybrid chase -> appointment handoff, and
would drift out of step with the behaviour the first time a branch moved.
dispatchReconnect stashes the peer context at the top of its branch walk, so
the leaf it reaches reports exactly what the decision saw. Two leaves are
reached WITHOUT that walk — hold escalation re-enters startReturnTo straight
from the barrier, and pursuitFallback commits once a chase has run — and for
those the stashed age is old by the entire wait or the entire chase budget,
always in the flattering direction. Re-querying here costs one map lookup and
makes "how stale was the record when the robot committed to this manoeuvre"
mean the same thing on every dispatch event.
```

## ExploPlannerNode::logReconnectDispatch

### explog-dispatch-decline-consumed

**Decline reason is consumed per dispatch** — attached to `reconnect_decline_reason_.clear();`

```text
Consumed, not just read. A decline belongs to the dispatch that produced
it; dispatchReconnect clears the field on entry, but the two out-of-band
leaves (hold escalation re-entering startReturnTo from the barrier, and
pursuitFallback committing after a spent chase) never pass through that
entry and would otherwise re-report a decline from minutes earlier as
though it were the reason for THIS manoeuvre.
```

### explog-dispatch-predict-consumed

**Chase prediction is consumed per dispatch** — attached to `e.predictor = pursuit_predictor_mdp_ ? "mdp" : "trail";`

```text
P6. The arm setting is a run-wide fact and is always written; the
prediction itself belongs to ONE chase, so it is consumed exactly like
decline_reason above. Without the consume, the hold or resume_exploring
leaf that closes a spent chase would re-report the intercept as though the
model had been consulted for it, and an offline count of "dispatches the
model aimed" would over-count by however many leaves each chase produced.
```

### explog-dispatch-link-down-lifetime

**Link-down fields live per manoeuvre** — attached to `e.link_down_sec    = dispatch_link_down_sec_;`

```text
Same lifetime and the same reason: a hold or resume_exploring leaf of
this manoeuvre re-reports the link-down duration the trigger fired on.
Terminal dispatches leave it -1 — the link gate only governs the mid-run
trigger, and pretending otherwise would put a number in the column for
decisions it never touched.
```

## ExploPlannerNode::logRunEnd

### explog-run-end-cached-coverage

**run_end uses cached coverage** — attached to `e.unknown_fraction = last_unknown_fraction_;`

```text
The last measured coverage rather than a fresh measurement: this runs on
the DONE transition and (via the destructor) during shutdown, where
map_cache_ may be mid-teardown and a whole-grid walk is the last thing to
start. fillCommonMetrics refreshes it at the sampling period, so it is at
most one metrics period old.
```

### explog-run-end-home-summary

**run_end geometry and homing summary** — attached to `e.have_home = have_home_;`

```text
Final geometry + mission-return summary (schema 2). latest_pos_ rather
than a fresh TF lookup, same teardown rationale as the coverage fields
above; mission_home_result_ stays "" (-> null) when the homing leg never
resolved — including a censored run killed mid-homing, which a reader
must be able to tell apart from an arrival.
```

## ExploPlannerNode::~ExploPlannerNode

### explog-run-end-in-destructor

**Writing run_end from the destructor** — attached to `try {`

```text
The campaign stops runs with SIGTERM (docker stop), which unwinds spin()
and destroys the node without ever reaching DONE — so without this the file
would end mid-stream and be indistinguishable from a truncated one. The
logger ignores a second run_end, so a node that DID reach DONE keeps its
real reason.

Nothing here may throw: a destructor that escapes during shutdown
terminates the process and would lose the flush it was called to perform.
```

## ExploPlannerNode::doLogStep

### logstep-budget-via-finish-or-rendezvous

**Step budget routes through finishOrRendezvous** — attached to `if (!finishOrRendezvous("step-budget")) {`

```text
Route through finishOrRendezvous, NOT straight to DONE. This is the
dominant termination path in dense terrain (the coverage threshold may
never be reached), and going directly to DONE meant a robot that spent
its step budget shut down wherever it happened to stop — never returning
to last_connected_anchor_ and never releasing its teammate's barrier.
The return drive happens after the budget is spent, so it costs no steps.

On a DEFERRED decision go to PLAN, never stay here: doLogStep is
dispatched every tick and writes a metrics row and increments step_ on
each call, so idling in LOG_STEP would forge duplicate steps. PLAN
re-checks the same budget at its head and calls this again, where a
deferral costs nothing.
```

## ExploPlannerNode::onTreeTarget

### exploit-fine-band-arm-on-ingest

**Arming the fine band on ingest** — attached to `if (region_pub_) {`

```text
Arm the fine band the moment the target enters the queue (release ==
exploitation-phase start for the whole team), not on activation: every
robot then fine-maps any released trunk its lidar reaches — including
the approach drive and a tree a peer is circling. Dedup'd re-reports
skip this (their region is already registered).
```

## ExploPlannerNode::removeLiveRefinementRegions

### exploit-remove-live-regions-on-shutdown

**Removing refinement regions on shutdown** — attached to `void ExploPlannerNode::removeLiveRefinementRegions() {`

```text
Called once, on the tick that latches shutdown_requested_ (State::DONE,
done_action=shutdown). DONE-idle deliberately does NOT clean up: its
regions stay armed because a target released while idling resumes the
exploit sub-loop. Removes here are idempotent with the per-target remove
in finishActiveTarget.
```

## ExploPlannerNode::computeApproachGoal

### exploit-approach-goal-march

**The approach goal march toward a trunk** — attached to `bool ExploPlannerNode::computeApproachGoal(const Eigen::Vector3f& center,`

```text
March from just outside the trunk outward along the line toward the robot and
return the point closest to the trunk that is in-ROI, a free planning_map cell,
reachable from the robot via the (already-flooded) cost grid, and not
blacklisted. This is the nearest spot we can actually drive to that makes
progress toward a target whose vantage ring is still unmapped/unreachable;
driving there maps the surroundings so a vantage can pass on a later tick.
Returns false if no such point exists meaningfully nearer the trunk than the
robot already is. Caller must have flooded cost_grid_ from robot_pos.
```

## ExploPlannerNode::exploitZAt

### exploit-z-terrain-nan-policy

**Exploit point height and the NaN policy** — attached to `float ExploPlannerNode::exploitZAt(float x, float y) const {`

```text
Standing / sightline height for an exploitation point at (x, y).

Flat mode: the fixed absolute vantage height, as before.

Terrain mode: the ingest band is robot-relative ([z_robot + roi_min_z,
z_robot + roi_max_z]), so an absolute vantage height leaves the band entirely
once the robot is on ground far from z = 0 — over this AO's ~14 m of relief
that is most of it. generateVantages() puts the vantage at that height and
lineOfSightClear() marches its ray at the vantage z, so the occlusion test
would find no voxels at all and pass every angle trivially: vantages accepted
with no visibility check, and `vantage_los_clear` logged as a meaningless 1.
Snapping to local ground + the candidate clearance keeps the sightline in the
observed volume, the same way exploration candidates are placed.

Returns NaN in terrain mode when no ground is found under (x, y), and the
CALLER decides — because the two consumers need opposite things and the old
shared "fall back to the robot's own z" was wrong for one of them:

  - A vantage's z IS the measurement. lineOfSightClear() marches its ray at
    exactly this height, so standing the ray at the robot's altitude over a
    column whose ground is unknown re-creates the very bug this function was
    added to fix, one level up: the ray leaves the observed volume, finds no
    occluders, and the angle passes with a meaningless vantage_los_clear=1.
    The vantage must be REJECTED instead (rej_noground), and the target left
    open — its own give-up timer closes it PARTIAL, which is the honest
    outcome for a trunk we could never see.
  - An approach waypoint's z is inert: it feeds inRoi (XY-only), isCellFree
    and reachable (both 2D), the failed-goal disc (XY) and then a navigator
    goal,
    which is a 2D navigator. So the robot's own z is a fine stand-in there,
    and rejecting would be actively harmful — the approach march exists to
    go MAP the unmapped ground, so refusing to drive anywhere the ground is
    unmapped deadlocks exactly the case it is for.
```

## ExploPlannerNode::startExploitNavigate

### exploit-intent-on-navigate

**Publishing the exploit intent** — attached to `if (intent_pub_ && coord_) {`

```text
Publish the goal as an exploit intent: peers on the same trunk consult it
in their vantage loop (MinPos, per-vantage radius) to take a different
angle, and the dwelled_mask carries this robot's clear-LoS credit for the
team quota. claim_radius_m ships the per-vantage disc so the claim's
stored radius matches its exploit semantics.
```

## ExploPlannerNode::vantageVetoedByPeers

### exploit-veto-per-vantage-disc

**Vantage veto uses the per-vantage disc** — attached to `const auto* peer = coord_->claimMatching(`

```text
Uses the small per-vantage disc, NOT the exploration-scale
coord_claim_radius_m — one fov-range disc would swallow the whole ring
and veto the tree outright instead of splitting the angles across the
team. Two claim rules, and which one applies is decided by the CLAIM
KIND, not by geometry; a third rule then contests the peers that are
parked on this ring with no claim on this angle yet. NOTE the default
(nullptr) liveness on claimMatching: exploit claims are read as RETAINED
— grace included — because a vantage under recent pursuit must stay
denied across a receive-side delivery gap (see Coordination's ctor doc
for the starved-executor incident that motivated the grace window).
```

### exploit-veto-claim-kind-not-distance

**Exploit claims are not distance-contested** — attached to `if (peer && peer->exploit && peer->target_id == target_id) return true;`

```text
An EXPLOIT claim on THIS trunk is not contested by distance at all: a
ring angle under active pursuit belongs to its claimant until the claim
lapses. Live-distance MinPos is what lost a ring — a robot standing at
the trunk having just finished one angle was, by construction, closer to
every remaining vantage than a peer 12 m out and 19 s into its drive, so
it took the angle the peer was already committed to, both converged on
one point, the proximity right-of-way braked the loser 8 times, and the
winner closed the 3/3 quota solo. The comparison was never meaningful
here: cost-to-go, not cost-so-far, is what a peer mid-hop has left.
```

### exploit-veto-staged-parked-peers

**Contesting peers parked on the ring** — attached to `return coord_->stagedExploitPeerWinning(`

```text
Third rule, and it is not about claims on this angle at all — both
rules above can only see a vantage a peer has ALREADY claimed, and the
race that actually hurts happens BEFORE any claim exists. When the
dwell-sync barrier releases the team, every parked robot re-plans within
milliseconds of every other, and the 1 Hz claim exchange is blind for
that first instant: with one angle left, every free robot picks it.
Observed: two robots picked the same last vantage 3 ms apart, and the
loser's in-flight yield (doNavigate) came 83 ms too late to prevent the
drive — they collided.

So contest the peers we KNOW are parked on this ring. A staged claim is
set at dwell entry and kept while its producer re-plans in place, i.e.
that peer is about to pick from a dead standstill beside the ring;
settle it now with the same total order the in-flight tiebreak would
use later, and if the parked peer strictly wins the angle simply do not
select it — no goal, no drive, no yield to walk back. Because staged
claims come only from stationary producers, its broadcast robot_pos
still matches reality, both sides evaluate the same antisymmetric
comparison, and exactly one robot admits the vantage without exchanging
a message.

DRIVING peers are deliberately not contested by position here — their
claim discs above are their instrument. A mover's wire pose is a
heartbeat stale, and a robot still approaching from far away is farther
from EVERY angle on the ring, so position-contesting movers would freeze
it out of the whole tree and serialize exactly what the barrier exists
to parallelize. The doNavigate tiebreak remains the backstop for the
residual race between two robots that each believed they had won
(comms skew).
```

## ExploPlannerNode::holdDwelledVantage

### exploit-hold-dwelled-vantage

**Holding the dwelled vantage** — attached to `if (!held_vantage_valid_ || held_vantage_target_id_ != tgt->id) return false;`

```text
The requirement, from the operator watching the 2-robot sim live: "both
robot go to their vantage poses[,] the dwell time starts, then one of the
robots get the next vantage pose, it goes there then dwell time starts[,]
then after dwell time finishes the exploitation finishes" — and the robot
that does NOT get the last vantage "should have kept the first vantage
pose". What it did instead (run9) was thrash: every time the winner's
claim aged out of the table mid-delivery-gap, the parked loser re-selected
the winner's vantage, rolled toward it for ~0.6 s, yielded it back, and
braked ~0.15 m further off its own vantage — twice, plus 25 s of retry
log spam, ending 0.3 m off-pose with a random heading.

So: once THIS robot has completed a dwell on this target and every other
angle of the ring is either team-visited or peer-denied (the SAME rules
the selection walk uses — vantageVetoedByPeers), there is nothing left
for it to contribute by driving. Park on the dwelled vantage, keep the
staged claim beating (a peer's dwell barrier reads it as "released"),
and let the tick's earlier stages — team-quota merge, per-target timeout
— end the hold. Exit paths, all above or upstream of this branch:
  * peer's mask merge completes the quota -> finishActiveTarget;
  * the winner dies -> its claim ages out (TTL + grace) -> the angle
    stops being vetoed -> this returns false and the normal walk selects;
  * per-target timeout -> PARTIAL.
This branch also runs BEFORE the fused-map refresh and flood, so a
holding robot's ticks drop from seconds to microseconds — which is what
lets the single-threaded executor actually deliver the peer's heartbeats
while we hold (the receive-side gap that started all of this).
```

### exploit-hold-blacklist-not-enough

**Blacklist alone does not justify a hold** — attached to `if (!vantageVetoedByPeers(tgt->id, v.position, robot_pos)) return false;`

```text
NOT visited: only a peer-denial keeps it off the table. Blacklist alone
is deliberately NOT enough to hold on — a blacklisted-but-unclaimed
angle means nobody is going there, and parking would leave the ring
permanently short; the normal walk's retry/approach/timeout machinery
owns that case.
```

### exploit-hold-reanchor-pose

**Re-anchoring onto the held vantage** — attached to `{`

```text
Ring covered. Re-anchor if the yield-roll drifted us off the pose (the
brake goal in abandonNavGoal stops the platform where it happens to be,
which after a ~0.6 s roll is decimetres off the vantage): publish the
held vantage itself as the nav goal. This is a sub-metre reposition onto
our own angle — the contested angle is a third of the ring away — and
publishGoal alone is correct here: there is no in-flight hop to cancel
(the yield already abandoned it) and EXPLOIT_PLAN is re-entered every
tick, so arrival needs no state transition.
```

### exploit-hold-staged-claim-rebuild

**Keeping the staged hold claim alive** — attached to `if (intent_pub_ && (!have_active_intent_ || !current_intent_msg_.staged ||`

```text
Keep the staged hold-claim beating. After a completed dwell the cached
intent already IS this claim (dwell entry set staged=true, the dwell-end
broadcast patched the mask, and LOG_STEP left it alone) — but after a
YIELD the intent was released (have_active_intent_=false, deliberately:
a kept claim would have been the CONCEDED angle with staged=false, which
would hold the winner's barrier open against us). Rebuild it here as
what we actually are: staged on our own dwelled vantage. The heartbeat
then re-publishes it at 1 Hz for as long as we hold.
```

## ExploPlannerNode::doExploitPlan

### exploit-plan-ordering-before-map

**doExploitPlan work before the map waits** — attached to `Target* tgt = target_queue_.active();`

```text
NOTE the ordering here: target bookkeeping, the team-quota merge and the
per-target timeout all run BEFORE the fused-map / planning_map waits
further down. None of them need a map, and putting them first means a map
that never arrives cannot pin the planner in EXPLOIT_PLAN — a peer
completing the ring or the wall-clock timeout still closes the target and
reverts to EXPLORE.
```

### exploit-plan-target-timer-restart

**Restarting the per-target give-up timer** — attached to `if (!exploit_target_timing_ || exploit_target_started_id_ != tgt->id) {`

```text
(Re)start the per-target give-up timer whenever a new target becomes
active, so the timeout below measures time spent on *this* trunk only.
The !timing_ disjunct also catches a rendezvous stand-down re-activating
the SAME id (startReturnToAnchor clears timing_): the return drive and
barrier wait must not count against the target. Progress events re-arm
the timer elsewhere: approach arrival (doNavigate) and completed dwell
(doExploitDwell); proximity holds refund it (doProximityHold release).
```

### exploit-plan-team-quota-merge

**Per-tick team quota merge** — attached to `if (coord_ && coord_->enabled()) {`

```text
Team quota: fold peers' clear-LoS dwell masks (carried on their exploit
intents) into this target before checking success. The event-driven merge
in onPeerExploitIntent normally gets there first; this per-tick pass
catches the ordering race where a peer's intent arrived before the
TreeTarget was ingested locally.
```

### exploit-plan-target-timeout-placement

**Where the per-target timeout sits** — attached to `if (exploit_target_timeout_sec_ > 0.0) {`

```text
Hard per-target wall-clock bound. This sits ABOVE all vantage-selection
logic AND above the map waits below on purpose:
  * a target can keep producing a "selectable" vantage every tick (e.g.
    the blacklist TTL re-offers vantages faster than the nav budget can
    exhaust the ring) and spin here forever, never reaching the give-up
    path further down;
  * a map that never arrives (field runs publish NO planning_map) would
    otherwise pin the planner in the early returns below with no escape.
Bounding it here makes exploitation of one trunk provably terminate, so
the phase always reverts to EXPLORE.
```

### exploit-plan-hold-branch-placement

**Where the hold branch sits** — attached to `if (coord_ && coord_->enabled() &&`

```text
Hold branch: with a dwell of our own banked and every other ring angle
team-visited or peer-denied, PARK on the dwelled vantage instead of
running the full selection below (see holdDwelledVantage for the run9
thrash this replaces). Placed above the map refresh + flood on purpose:
a holding robot's tick must cost microseconds, not seconds, or the
executor starves the very intent subscription whose claims the hold
decision reads. Quota merge and the per-target timeout stay ABOVE this,
so a hold can always end.
```

### exploit-plan-planning-map-wait

**Waiting for the planning map** — attached to `if (use_planning_map_ && !latest_plan_map_) {`

```text
Vantage validation (free-cell + reachability) and the approach fallback use
the 2D planning_map when it is enabled. With use_planning_map=false there is
never a map: vantages are then validated on straight-line geometry (no
obstacle/reachability check — avoidance is delegated to the navigator,
mirroring exploration's map-less fallback), so we do NOT wait here.
When the map IS enabled but has not arrived yet, wait rather than drive
blind toward the trunk. The wait is BOUNDED: the per-target timeout above
keeps ticking, so a mis-wired planning_map times the target out PARTIAL
instead of hanging in EXPLOIT_PLAN forever.
```

### exploit-plan-unbounded-flood-cache

**Unbounded cached flood for exploitation** — attached to `bool have_cost = false;`

```text
Build the cost grid for reachability + nearest-vantage ordering, but only
when a planning_map is available. Unlike exploration (a bounded local
flood), exploitation may have to drive across the map to reach the trunk,
so the flood is UNBOUNDED (cap <= 0) — a target released while the robot is
far away must still resolve as reachable as long as a known-free path
exists. ~5 ms once per vantage, infrequent. With use_planning_map=false
there is no obstacle layer to flood: have_cost stays false and every
vantage is ordered by straight-line distance with no reachability filter.

Cached on (map identity, source pose). "once per vantage" only held while
a vantage was selectable: when none is, EXPLOIT_PLAN does NOT transition
and is therefore re-entered at the full 10 Hz tick rate until the target
times out — up to ~1200 whole-grid floods per target, all identical, on
the single-threaded executor that also has to service the map and TF
callbacks. Rebuild only when the latched map object or the robot pose
actually changed.
```

### exploit-vantage-reject-no-ground

**Rejecting vantages with no ground** — attached to `if (!std::isfinite(v.position.z())) { ++rej_noground; continue; }`

```text
Terrain mode with no ground under this angle: exploitZAt returned NaN and
there is no honest height to march the LoS ray at. Reject rather than
guess — see exploitZAt. Counted separately from rej_roi because in the
field the two mean completely different things ("outside the AO box" vs
"this column is not mapped yet"), and inRoi is XY-only so it would not
catch a NaN z anyway.
```

### exploit-vantage-peer-veto

**Per-vantage peer deconfliction** — attached to `if (coord_ && coord_->enabled() &&`

```text
Per-vantage deconfliction: yield an angle a peer currently claims
(in-flight or mid-dwell) or that a parked peer is about to win. The
three rules live in vantageVetoedByPeers() — the hold branch above this
walk consults the same implementation, which is what keeps "nothing
selectable, park on my own vantage" and "this vantage is selectable"
mutually exclusive by construction.
```

### exploit-no-vantage-log-throttle

**Throttling the no-vantage log** — attached to `RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,`

```text
Throttled: this branch re-runs at the 10 Hz tick rate while nothing is
selectable, and an unthrottled print turned a 25 s hold into 200
identical lines that buried the two log lines that actually explained
the round (run9). The rejection counters still tell the whole story
once per window.
```

### exploit-approach-toward-trunk

**Approaching the trunk when no vantage passes** — attached to `Eigen::Vector3f approach;`

```text
Quota-met and the per-target timeout are both handled unconditionally at
the top of this function, so neither can apply here. Drive *toward* the
trunk instead: head for the nearest reachable point
on the line to it so the area maps en route and a vantage can pass on a
later tick. Re-planning (not dwelling) resumes on arrival.
```

### exploit-approach-sep-diag-reset

**Resetting separation diagnostics on exploit rows** — attached to `pending_sep_peer_dist_m_    = -1.0f;`

```text
The separation term does not run on the exploit path — vantages are
chosen by sightline, not by utility — so these are reset to their
"nothing measured" values rather than left holding whatever the last
exploration tick saw. A stale peer distance on an exploit row would be
read as a measurement of where this robot was sent, and it is not.
```

### exploit-stage-selection-los-verdict

**Staging the selection-time LoS verdict** — attached to `current_vantage_los_clear_ = true;`

```text
Stage the selection-time LoS verdict (this vantage just passed
lineOfSightClear above). It is what the dwell's map-unavailable fallback
reads — before this was staged here, that fallback silently reused
whatever verdict the PREVIOUS dwell left behind, possibly from another
vantage or another target, and could credit an unverified capture toward
the team quota.
```

## ExploPlannerNode::doExploitDwell

### dwell-sync-ring-barrier

**The vantage-ring rendezvous barrier** — attached to `if (exploit_dwell_sync_enabled_ && coord_ && coord_->enabled() &&`

```text
Vantage-ring rendezvous barrier. Hold here — dwell clock NOT started — while
any peer holds an exploit claim on this trunk and has not yet declared
itself staged on the vantage it claimed. The ring exists to capture ONE
trunk state from several angles at once; a robot that arrives first and
dwells alone spends the team's 3/3 quota on sequential single-robot views,
and the peer that arrives after the target closed dwells an angle nobody
needs.

"Staged" is PRODUCER-DECLARED (each robot sets the flag on its own claim at
dwell entry, i.e. after arrival AND the post-arrival rotation settle) rather
than inferred here from the claim's robot_pos against its goal_pos. Only the
producer knows whether the exploit hop it is holding is a vantage at all (an
approach waypoint is not) and whether it has stopped turning: the geometric
test read a peer that had arrived in XY but was still rotating as staged
~5 s early, and the team's overlap collapsed to ~3 s of an 8 s dwell.

The barrier is a START condition, latched by dwell_sync_started_ once
satisfied. It must not be re-tested per tick: peers go staged=false again
the moment they leave for their next angle, and re-testing then wiped the
accumulated dwell of a robot that had been motionless all along (see the
member's declaration). Started dwells therefore RUN TO COMPLETION.

The wait mechanism is a per-tick re-anchor of state_enter_time_, not a
separate hold state: `elapsed` below therefore still measures real
motionless seconds at the vantage from the moment the team was staged, so
the CSV dwell_sec stays an honest capture length rather than wait time plus
capture. Release paths (unchanged): every claiming peer staged, no peer
claims this trunk, a peer's claim ages out (firstUnstagedExploitPeer filters
expiry itself — prune() runs in the PLAN ticks, which do not happen while we
sit here), or max_wait below. The 1 Hz heartbeat republish of each claim is
what carries a peer's staging to us; it bounds the residual asymmetry to
one heartbeat plus the DDS hop, and the immediate publish at dwell entry
(transitionTo) removes even that for the common case.
```

### dwell-sync-barrier-satisfied-log

**Latching the barrier and logging team staged** — attached to `if (state_enter_time_ > dwell_sync_wait_start_) {`

```text
Barrier satisfied -> latch the start for the rest of this dwell.

Logged only when we actually held for somebody. The probe returns
nullptr for two different situations — every same-target peer staged,
and no peer claiming this trunk at all (the solo capture, which is the
single-robot-equivalent case and must not produce a "team staged" line
per vantage) — and the Coordination API exposes no per-target peer
claim count to separate them. The hold branch's re-anchor IS that
record: state_enter_time_ sits past the entry-time wait clock iff this
barrier held for at least one tick. A peer that was already staged when
we arrived consequently starts its dwell silently too.
```

### dwell-sync-max-wait-and-deadline

**Unbounded wait and the per-target deadline** — attached to `const bool target_deadline_hit =`

```text
max_wait <= 0 means "wait until the peer actually arrives", which is
the default. A wall-clock bound cannot tell a teammate that is merely
FAR from one that is wedged: the ring of the next trunk can be 30 m
away across the plot, and a peer that is still driving toward its own
angle was abandoned at 60 s while it needed 111 s — the capture went
solo for no reason. Waiting indefinitely does not hang the run,
because a robot that is waiting is by definition standing on its
vantage with staged=true already on the wire (published at dwell
entry), so it never holds a peer's barrier: only a DRIVING peer
holds, and that peer is bounded by its own timers, which do tick
while it drives. It arrives, or it goes silent and its claim ages out
of the table on the receipt-time TTL, or its own per-target give-up
closes the trunk and it stops claiming it. All three release us.
The per-target give-up clock is the OUTER bound, and it applies
whether or not max_wait is set. The paragraph above is right that
every release it lists eventually fires -- but every one of them
depends on the PEER: it arrives, it goes silent, or it closes the
trunk. None of them is a clock this robot owns. A peer that keeps
republishing a live claim it will never stage on (wedged in a
proximity hold, say, which still heartbeats) satisfies none of the
three, and with max_wait at its default 0.0 this robot holds a
vantage until the mission cap.

exploit_target_timeout_sec_ already exists for exactly this class of
failure and already runs during the hold -- it is simply never READ
here, only in doExploitPlan, which a holding robot never re-enters.
Reading it costs no new parameter and no new default to get wrong.

Release rather than abandon. doExploitPlan answers this deadline with
finishActiveTarget(false) because there it means "this trunk cannot
be reached"; here the robot is standing ON a valid vantage with a
capture pending, so the honest response is the one the max_wait path
already takes -- stop waiting, dwell solo, keep the capture. The
same latch is used, so the warning below explains it once and the
barrier stays down for the rest of this dwell.
```

### dwell-sync-timeout-latch

**Latching the barrier give-up** — attached to `dwell_sync_timed_out_ = true;`

```text
Give-up bound: a peer wedged in a proximity hold or cycling
EXPLOIT_PLAN with no selectable vantage must not cost this robot its
capture. Latched for the rest of THIS dwell — re-testing would
re-anchor the dwell clock on the next tick and the dwell would never
complete. transitionTo clears the latch on the next EXPLOIT_DWELL
entry, so one timed-out barrier does not disable the feature.
```

### dwell-no-goal-resend

**No goal re-send during the dwell** — attached to `if (elapsed < exploit_dwell_sec_) return;`

```text
NO goal re-send during the dwell. The dwell is only ever entered from
NAVIGATE *after* arrival, so the goal has already been reached and there is
nothing to keep alive; re-sending would re-arm the navigator on a goal the
robot is standing on.

The original argument for this was nav2's (a re-send is a fresh
NavigateToPose, and controller_server::computeControl() calls
computeAndPublishVelocity() before isGoalReached(), so even a satisfied goal
emits a velocity command). That mechanism is not in this stack, and the
honest version is weaker: on simple_nav_3d an identical re-send is inert —
the intake drops it while driving, and after arrival it re-arms for one 50 ms
tick and commands nothing. So this is hygiene, not a defence.

WHAT THE DWELL IS ACTUALLY EXPOSED TO, since the old comment implied the
absence of a re-send made it still: the controller's rotate-to-goal latch is
independent of anything the planner publishes. The planner's arrival gate
(goal_yaw_tolerance 0.4) is deliberately looser than the controller's
(ugv.goal_yaw_tol_rad 0.2) — it has to be, or the planner would wait on a
heading the controller has already declared good enough — so on entry to the
dwell the controller may still be closing the last 0.2 rad. Against the
0.5 rad/s clamp that is a few tenths of a second of in-place rotation at the
START of an 8 s capture window, bounded by the tolerance gap rather than by
anything this function does. Stated because a reader who needs a strictly
still capture should know where the residual motion comes from — and that
closing the gap would trade it for the arrival deadlock the gap prevents.
```

### dwell-reconfirm-los-sightline-z

**Re-confirming LoS at the settled pose** — attached to `bool los_clear = current_vantage_los_clear_;`

```text
Dwell complete. Re-confirm line-of-sight from the pose we actually settled
at (the controller may have stopped slightly off the planned vantage, and
the map has grown during the dwell) — that is the honest "was this a
clear-LoS capture?" answer, and it is what counts toward the success quota
and what the CSV logs. Falls back to the selection-time verdict — staged in
doExploitPlan when THIS vantage was chosen — if the map or active target is
momentarily unavailable.

Settled XY, SIGHTLINE z. latest_pos_.z() is base_link — 0.09-0.13 m on a
UGV, and it bobs by more than a voxel as the platform settles — so marching
the ray at it samples the voxel row the mapped ground surface occupies and
reports BLOCKED for the ray's whole length, on a trunk in the open, at
random depending on where the suspension came to rest. The measurement
height is ground + candidate_z_clearance: exactly what generateVantages()
used at selection time, and what exploitZAt() is for (see its contract
above — "a vantage's z IS the measurement"). Ground unknown under the
settled pose => keep the selection-time verdict rather than march the ray
outside the observed volume, where it would pass trivially.
```

### dwell-remember-held-vantage-pose

**Remembering the held vantage pose** — attached to `if (t) {`

```text
Remember the exact pose this dwell was captured from. If the rest of the
ring ends up covered by the team, doExploitPlan's hold branch parks the
robot back on precisely this position AND yaw — the operator requirement
is that the robot which does not get the last vantage keeps its vantage
pose, not "stops somewhere near it".
```

### dwell-credit-mask-patch-intent

**Broadcasting dwell credit by patching the intent** — attached to `if (intent_pub_ && coord_ && have_active_intent_ && t) {`

```text
Broadcast the updated team-credit mask immediately (don't wait for the
next selection or heartbeat) so a peer picking its next angle right now
already sees this dwell — and so the credit lands before this robot
could release the claim on target completion.

Patch the cached message in place; do NOT rebuild it through buildIntent().
Rebuilding would reset `staged` to false while the robot is still standing
on the vantage, which is exactly wrong for the last round of a ring: a robot
with no selectable vantage left holds here, and the heartbeat republish of
THIS message (staged, at the vantage) is what keeps a peer's barrier
released while that peer takes the final angle.
```

## ExploPlannerNode::updatePoseFromTF

### pose-stale-tf-is-lost

**A stale transform means pose lost** — attached to `if (pose_max_age_sec_ > 0.0) {`

```text
A successful lookup is NOT a fresh pose. TimePointZero returns the newest
stored transform unconditionally, and tf2 prunes only on INSERT — once a
broadcaster dies this lookup keeps succeeding with the same stamp
forever. Everything downstream trusts latest_pos_ (goals, the brake goal,
distance, the trajectory log), so a stale transform must read as "pose
lost", exactly like a failed lookup, until fresh data arrives.
```

### pose-loss-event-log

**Logging pose loss to the event log** — attached to `if (have_pose_ && exp_log_) {`

```text
Edge-triggered into the event log. The WARN above is throttled and
lives only in the ROS log; analysis reads the JSONL, where a dead pose
feed was previously indistinguishable from a robot that had stopped
moving — the same no-progress failures, the same watchdog ladder, the
same park, with nothing recording that the measurements were blind.
```

### mission-return-home-capture

**Capturing home once at first pose** — attached to `if (!have_home_) {`

```text
Mission-return home capture: exactly once, at the FIRST successful fresh
pose. Deliberately its own latch and not have_pose_ — that flag drops and
returns with TF health, and re-recording here would move "home" to
wherever TF last recovered. The coords are logged so the verify script can
check them against the scenario's declared spawns.
```

### mission-return-breadcrumb-trail

**The outbound breadcrumb trail** — attached to `const bool homing_now =`

```text
Breadcrumb trail (retrace fallback, see doReturnHome). Outbound only —
recording stops once homing starts, so the trail cannot grow toward the
robot while it retraces. First crumb is home itself. 2 m spacing keeps a
3000 s run under ~700 points. Recorded whenever the flag is on: the leg
that will need it cannot know that in advance.

PROXIMITY_HOLD has to count as homing when the hold interrupted a homing
leg. It is a distinct state, so testing state_ alone let the trail keep
growing through a hold taken mid-return: a crumb would land at the HIGHEST
index at a mid-homing position, engageRetrace would pick it as nearest,
and the advance loop would then walk outward to trail[N-1] — tens of
metres AWAY from home — while homeApproachMetric folded the whole homing
leg back into the metric the watchdog scores. A hold taken while still
exploring is the opposite case and must keep recording, so this asks what
the hold interrupted rather than blanket-excluding the state.
```

## ExploPlannerNode::trackDistance

### distance-pose-jump-guard

**Rejecting single-tick pose jumps** — attached to `if (max_pose_jump_m_ > 0.0f && step > max_pose_jump_m_) {`

```text
Reject implausible single-tick pose jumps. Localization relocalization
(NDT / EKF corrections) teleports the map->base_link TF by metres in one
tick; at the 10 Hz tick this guard can't reject real motion (even a 1 m/s
robot moves 0.1 m/tick), so anything above max_pose_jump_m_ is a
discontinuity, not travel. Counting it would inflate distance_traveled (a
headline metric) AND let a stationary-but-relocalizing robot satisfy the
no-progress watchdog. prev_pos_ is still advanced so the next tick
measures from the corrected pose. 0 disables the guard.
```

## ExploPlannerNode::publishGoal

### goal-z-flatten

**Goal z and flatten_goal_z** — attached to `goal.pose.position.z = flatten_goal_z_ ? 0.0 : vp.position.z();`

```text
simple_nav_3d's UGV role plans and checks arrival in the plane and ignores
z, so the true 3D waypoint z rides along by default (terrain mode makes it
the ground + clearance elevation). flatten_goal_z zeroes it for consumers
that choke on a non-zero z; markers/logs keep the 3D value either way.
NOTE for any future UAV run: the UAV role's arrival test IS 3D, so there a
wrong goal z is a wrong goal — not a harmless passenger.
```

## ExploPlannerNode::republishGoal

### goal-republish-keepalive

**Goal keep-alive re-send** — attached to `void ExploPlannerNode::republishGoal(const CandidateViewpoint& vp) {`

```text
Keep-alive re-send used by the states that are still DRIVING toward a goal
across ticks (NAVIGATE, RETURN_NAV). Publishes when the pose changed, when a
subscriber has just appeared, or when goal_republish_sec_ has elapsed — never
at the tick rate. The throttle was introduced against a nav2 failure mode that
does not exist on this stack; see goal_republish_sec_ for what it is really
worth here (recovering a goal published while the navigator was down) and why
setting it to 0 is not the improvement the old comment claimed.

Deliberately NOT called from EXPLOIT_DWELL: that state is entered only after
arrival, so there is no in-flight goal to keep alive and a re-send would
re-arm the navigator on a goal the robot is standing on. See doExploitDwell()
for what does and does not hold the platform still during a capture.
```

## ExploPlannerNode::updateCellWorld

### cell-world-edges-from-plan-map

**Rebuilding cell edges from the plan map** — attached to `if (latest_plan_map_) {`

```text
Edges follow the plan map as it fills. Rebuilt every census rather than
once at startup because at startup the plan map is empty and EVERY edge
would be blocked. With no map the probe is null and every edge is enabled,
which is the right default: the graph is a ranking input, and an
all-disabled graph would silently rank nothing.
```

### cell-census-measured-cells-only

**Unknown quantiles over measured cells only** — attached to `std::vector<double> cell_u, cell_ff;`

```text
Unknown fractions of the MEASURED cells, gathered here and reduced below.
Only cells that cleared min_observed_columns go in: an unmeasured cell
reports unknownFraction() == 1 by construction, and letting those into the
distribution would make the median a statement about how far the grid
overhangs the ROI rather than about how well the seen ground is mapped.
```

### cell-census-nearest-rank-pct

**Nearest-rank percentiles for the census** — attached to `auto pct = [&cell_u](double q) {`

```text
Nearest-rank percentiles, so every reported value is a real cell's
measurement rather than an interpolation between two of them. With a
handful of measured cells an interpolated p10 can sit below every cell
on the map, which is exactly the wrong answer to "is the threshold
reachable".
```

## ExploPlannerNode::missionElapsed

### mission-elapsed-clock-not-live

**Mission baseline before the clock is live** — attached to `const auto now = this->now();`

```text
The same guard startExperimentLog uses, for the same reason: under
use_sim_time this->now() reads exactly 0 until the first /clock message
lands, and a baseline of 0 would make every mission-elapsed quantity
this subsystem publishes an absolute sim time wearing a relative name —
which a float32 then quantises to ~128 s and destroys.
```

## ExploPlannerNode::missionElapsedAt

### mission-elapsed-at-no-latch

**Why missionElapsedAt does not latch** — attached to `if (mission_t0_sec_ < 0.0) return -1.0;`

```text
Deliberately NOT latching: this is const because it only ever reads a
baseline that missionElapsed() must already have set. A caller that gets
here before the first live tick gets -1 and has to say so, rather than
silently anchoring an appointment on a baseline invented from a past
stamp.
```

## ExploPlannerNode::publishTeamWorld

### team-world-no-pose-no-publish

**No TeamWorld publish without a pose** — attached to `if (!have_pose_) {`

```text
No pose, no publish. `position` is a mandatory field and is the
allocator's vehicle-start for this robot; the message has no way to say
"unknown", so publishing before the first TF would put this robot at the
map origin in every peer's world model — and it would be a plausible
position, arriving on a healthy link, that nothing downstream could tell
from a real one. A few seconds of silence at startup is the cheaper error.
```

### team-world-direct-mask

**Publishing directMask, not inCommsMask** — attached to `m.in_range_mask = team_model_.directMask();`

```text
directMask(), NOT inCommsMask(). Publishing the closure result would make
the handshake circular: A would claim to hear C because B said C was
reachable, C would conclude the same about A from its own copy of the same
relay, and the mutual test that exists to catch one-way contact would pass
with neither having heard the other. See TeamModel::inCommsMask.
```

### team-world-team-incomplete-bit

**The team_incomplete contagion bit** — attached to `m.team_incomplete =`

```text
Generation 23, the contagion bit: THIS robot's OWN first-hand answer to
"is the team whole?", never its derived armed state — announcing the armed
state makes A and C hold each other armed off their own echo forever. The
argument in full, and the A—B—C bridge it exists for, is in TeamWorld.msg.

Published in all four arms, like my_tour: it is what this robot's PEERS
need, and whether they act on it is their local mode's business.

Guarded on a positive expectation because teamComplete() is false whenever
it is 0, so an inert configuration (a single-robot run, where 0 is the
documented default in shared_params.yaml) would otherwise broadcast a
permanent, unclearable "the team is broken" to a fleet of nobody. No
expectation means the question has no answer here, and the honest wire
value for that is "I am not reporting a break".
```

### team-world-appointment-inbound

**The appointment_inbound bit** — attached to `m.appointment_inbound =`

```text
Generation 23: "I am still driving to the agreed cell", so a robot already
standing there does not call the meeting over while its partner is on the
way. Keyed on the DRIVING state and not on an arrival test on purpose — the
bit has to clear however the drive ended, or an unreachable meeting point
would hold the team at an unbounded barrier. See TeamWorld.msg.
```

### team-world-inbound-seen-one-hop

**The one-hop appointment_inbound_seen report** — attached to `m.appointment_inbound_seen = peerInboundToAppointment();`

```text
Generation 27: the one-hop report that lets the closure door's inbound
veto reach the peers the door admits through a bridge. Derived from the
RAW first-hand bits only — publishing the wider peer-report predicate
here would let A say "seen" because B says "seen", a relayed echo with
nothing able to clear it. See TeamWorld.msg.
```

### team-world-my-tour-live-only

**Broadcasting the tour only while driving it** — attached to `const State tour_state = (state_ == State::PROXIMITY_HOLD)`

```text
P3/P6: this robot's current route, for the peers' interception model. The
wire type is uint16 and a cell id is an int, so ids are bounds-checked
rather than cast — an id that does not survive the round trip would name a
different cell on the receiver, and a chase aimed at it is aimed at ground
nobody chose. Out-of-range TRUNCATES the tour rather than skipping the
offending entry: the consumer walks this as an ordered route and a route
with a hole in it is a route through a cell the producer never planned to
visit. It cannot happen on any grid this planner builds (the ROI would
have to exceed 65535 cells) and it is checked anyway, because the failure
is silent and the check is free.

Broadcast whether or not the local predictor is enabled: it is what THIS
robot's peers need, and gating the send on our own setting would make an
mdp/trail mixed fleet fail in a way that looks like the model refusing.

BUT ONLY WHILE THIS ROBOT IS ACTUALLY DRIVING THAT ROUTE (2026-09-18).
`my_tour_` had exactly one writer — the allocator solve in doPlan — so it
was only ever refreshed inside the exploration loop. (It has two now: the
clear a few lines below is the other, and it is what this block adds.
Nothing else assigns it; grep my_tour_ to confirm.) The moment the robot
leaves that loop for a manoeuvre (PURSUE, RETURN_NAV, RETURN_SYNC), for
the homing leg (RETURN_HOME), or for good (DONE / coverage latched), the
solve stops running and this heartbeat kept re-sending the last route
FOREVER, at a receiver-stamped age of zero. There is no age or stamp
field on TeamWorld.my_tour, so the consumer cannot tell a route the peer
is driving from one it abandoned four minutes ago — and the receive path
anchors the tour at the position from the SAME message, which is now the
peer's live pose on its way somewhere else entirely. That is the exact
failure my_tour_'s own declaration says the model "has no way to detect",
arrived at by a different door: not a refused solve leaving a stale route
on the air, but no solve at all.

It is not hypothetical for this campaign. A robot that departs for an
appointment typically stays in comms for a few seconds before the link
drops, so the LAST tour its peers hold is the frozen exploration route,
anchored at a pose already heading for the meeting point — and the mdp
predictor (non-inert since FIX 7) then extrapolates it along ground it
has stopped driving, aiming the chase away from where it actually went.

Sending an EMPTY tour is the designed encoding for "I have no route", not
a hole: the receive path overwrites unconditionally on an accepted
message "including with an EMPTY tour ... Absence is information here",
and a peer that hears no tour falls back to the trail chase, which is the
documented safe degradation. So this narrows the predictor's input to the
cases where it is true rather than removing it.

PROXIMITY_HOLD defers to prox_resume_state_ because the hold is a pause,
not a decision: it cancels the goal and resumes THE SAME ONE, so a hold
taken out of NAVIGATE is still driving the tour while one taken out of
PURSUE is not. Same treatment, and for the same reason, as the homing
attribution below at the mission_return sites.
```

### team-world-rendezvous-held-pair

**Publishing the held rendezvous pair** — attached to `m.rendezvous_cell_id     = rendezvous_held_.cell;`

```text
P5: the pair this robot currently holds — its own derivation if it is the
proposer, the proposer's pair echoed verbatim otherwise. Written explicitly
at -1/-1 when there is none rather than left to the struct's zero-init,
because 0 is a valid cell id and a zero-init pair would broadcast a
standing proposal to meet in cell 0 immediately after separation.

Publishing is NOT driving: this says "I hold this pair", and the commit
rule downstream is what turns a pair everyone holds into one anyone acts
on. The two are separate on purpose — a robot that drove its held pair
would leave before its peers had even seen it.
```

### team-world-rendezvous-provisional

**The provisional flag travels with the triple** — attached to `m.rendezvous_provisional =`

```text
THE FLAG TRAVELS WITH THE TRIPLE, and it is false whenever the triple is
absent. A peer must be able to tell "I am publishing nothing" from "I am
publishing a placeholder", and the refusal path below rewrites the cell to
-1 without unwinding this line, so it is set from the held state and then
cleared with the cell if the triple turns out not to fit the wire.
```

### team-world-rendezvous-int32-guard

**Refusing a triple that overflows int32** — attached to `constexpr long long kMaxMs =`

```text
int32 ms tops out at ~24 days, so this cannot fire on any mission this
planner will run — but the value is produced by arithmetic over graph
distances and a nonsense speed would make it enormous, and the failure
mode of a silent truncation is a peer meeting at the wrong TIME while
agreeing perfectly on the cell. Refuse the triple whole instead: a robot
publishing (cell, -1, -1) advertises "no proposal", which the commit rule
already handles, where a truncated instant would be believed.

BOTH TIME FIELDS ARE CHECKED TOGETHER and refused together. t_meet is the
larger of the two — it is the interval plus a mission clock that only
grows — so it is the one that overflows first, and a check that covered
only the interval would broadcast a truncated meeting INSTANT beside a
perfectly valid interval. That is the exact failure this guard exists to
prevent, one field further along.
```

### team-world-finished-latch-or-done

**The finished bit: latch or DONE** — attached to `if (coverage_latched_ || state_ == State::DONE) finished_announced_ = true;`

```text
THE LATCH **OR** THE ENDING, and it took both to make the bit honest.

The latch is the EARLY signal and stays first: DONE is reached at the end
of the mission-return leg, minutes after this robot's map stopped gaining,
and a peer keyed on DONE alone would keep planning around a robot that had
already finished exploring.

But the latch ALONE was a censoring bug (2026-09-18, generation 23). It is
written at exactly one site — maybeLatchCoverageDone, first touch only —
and NONE of the other run endings touch it: step-budget, coverage-saturated
and barrier-gave-up all route through startReturnHome and end at
finishNow() with coverage_latched_ still false. publishTeamWorld has no
state gate and its timer is a bare lambda, so such a robot went on
publishing finished=false AND team_incomplete=true, once a second, from
wherever it had parked, for the rest of the run. Every peer then read a
live teammate announcing a broken team: peerReportsTeamBreak's
`if (p.finished) continue;` exemption — written for exactly this case,
see its own comment — could not fire, teamSettled was pinned false
fleet-wide, and the appointment barrier (unbounded on purpose,
rendezvous_appointment_wait_sec = 0) held every other robot at the meeting
point until the duration cap. One robot finishing early censored the cell.

State::DONE is the ending every one of those paths reaches, and it is a
strictly later and strictly stronger statement than the latch, so ORing it
in cannot make the bit fire too early. Under done_criterion=streak the
latch half stays false for the whole run — that criterion has no latch to
report — and the DONE half now carries the bit on its own, which is the
case that used to have no signal at all.

LATCHED, NOT RECOMPUTED, and that is what makes the bit safe to RELAY
(TeamWorld.msg/robot_finished). Relay depends on monotonicity: a relayed
copy is only sound if the bit can never go back to false, because nothing
downstream clears it. `coverage_latched_` is monotonic — one write site,
never reset — but `state_ == State::DONE` on its own is NOT: the DONE
branch in tick() says so itself, since the exploit sub-loop can pull the
planner back out to EXPLORE -> PLAN and land in DONE again. That path is
unreachable for this campaign (exploitation_enabled: false), and relying on
a config value to hold a wire invariant is exactly the kind of coincidence
that stops being true without anything looking wrong. Latching it here
makes the guarantee local to the publisher and independent of the config.
```

### team-world-announced-mode-levels

**The three-level announced mode** — attached to `using TeamWorldMsg = explo_planner_msgs::msg::TeamWorld;`

```text
The same statement on three levels instead of two, latched the same way
and for the same relay reason (see homing_announced_). NEITHER LEVEL IS
READ FROM state_: DONE comes from the bit above, because State::DONE is
leavable through the exploit sub-loop and the announced bit is not, and
HOMING comes from the latch, because state_ has already moved past
RETURN_HOME by the time the robot is parked at home. The ordering is what
makes the field monotone at the source — DONE outranks HOMING, so a robot
that latches coverage without ever homing still only ever rises.

DONE WAITS FOR THE MEETING (2026-09-23). A finished robot keeping its
appointment is still taking part, so it publishes below HOMING until the
appointment manoeuvre ends, and DONE from then on (announcedMode). The
partner's barrier holds for it on exactly that level; see
holdingForFinishedPeer.
```

### team-world-done-no-team-break

**A finished robot reports no team break** — attached to `if (state_ == State::DONE) m.team_incomplete = false;`

```text
A robot whose run is over has no opinion worth acting on about whether the
TEAM is whole, and announcing one is how the censoring above propagated.
Belt and braces with the `finished` bit above on purpose: that bit is what
peerReportsTeamBreak reads, but team_incomplete is a public wire field and
any future consumer reading it directly must not be handed a stale yes.
```

### team-world-gossip-self-entry

**Own entry anchors the gossip array** — attached to `m.robot_positions[i]      = m.position;`

```text
Our own entry is our publish time, and it is the reference point the
whole array is read against — the receiver recovers each peer's age as
(our entry - that entry), an interval, which is the only thing that
transfers between two mission clocks that started at different wall
times. Without it the array is uninterpretable and is dropped whole.
```

### team-world-gossip-first-hand-pair

**Gossip positions: first-hand and paired** — attached to `if (p.last_direct_sec >= 0.0 && p.have_position && p.position_first_hand) {`

```text
last_direct_sec, not last_known_sec: the field is defined as DIRECT
contact, which bounds relay to a single hop and keeps this consistent
with closure's two-hop bound. Relaying our own second-hand knowledge
would extend position gossip further than status can reach, so a
consumer would hold a position for a robot the model cannot place.

Position and freshness are published as a PAIR or not at all. The wire
has no "fresh but positionless" encoding — the receiver validates a
position by its matching last-heard being non-negative — so a lone
freshness entry would hand the peer (0, 0, 0) as a real position.
```

### team-world-gossip-finished-relay

**Relaying the finished bit freely** — attached to `if (p.finished) m.robot_finished[i] = true;`

```text
RELAYED WITHOUT THE PAIRING RULE AND WITHOUT THE FRESHNESS RULE that
govern the two arrays above, because neither applies to a monotonic bit.
There is no companion field to keep it consistent with, and no age at
which it stops being true. It is also relayed whether we hold it
first-hand or by relay ourselves — unlike position, which is deliberately
first-hand-only to bound relay to one hop. That bound exists so a
consumer cannot hold a POSITION for a robot the model cannot place; a
finished bit places nothing and only ever removes a robot from the set
the team waits for, so propagating it further is the point rather than a
hazard. Its own idempotence is what makes that safe: re-relaying a bit
that is already true changes nothing anywhere.
```

## ExploPlannerNode::drainTeamWorld

### team-drain-two-pass-rows

**Two-pass team exchange rows** — attached to `std::vector<TeamExchangeEvent> rows;`

```text
One row per drained message, filled in two passes: the merge fields here,
the comms fields after the single tick below. The split IS the design —
closure is a property of the whole contact graph, so a comms picture
recomputed per message would give every row a different answer to the same
question and which row a reader believed would be an accident of arrival
order. See the declaration and TeamModel::tick.
```

### team-drain-pre-known-age

**Capturing the silence before observe** — attached to `std::map<int, double> pre_known_age;`

```text
THE SILENCE THIS MESSAGE ENDED, captured before a single observe() runs.

Through v7 the row reported team_model_.lastKnownAgeSec(sender) taken in
the second pass, which is structurally 0.0: observe() sets the sender's
last_known_sec to this same `t`, and a row only exists for a sender. It
read 0.0 on 757,677 of 757,677 rows across the banked campaigns — a column
that could not have reported anything else.

The quantity that was wanted is the one just above: how stale our
knowledge of this peer had become at the instant its message landed, i.e.
the receiver-side inter-arrival gap. That is a direct per-peer measure of
dropout length, and it is the only place in the log where a dropout's
duration is observed by the robot that suffered it rather than inferred
by joining two files.

A SEPARATE PRE-PASS, not a read at the top of the drain loop, and the
distinction is load-bearing: observe() gossips third-party last-heard
times, so the first sender's message can refresh the SECOND sender's
last_known_sec before the loop reaches it. Reading inline would silently
shrink the gap of every peer but the first, in arrival order — the exact
shape of bias that survives into a result.
```

### teamworld-config-checks

**TeamWorld fleet and grid hard drops** — attached to `if (sid < 0 || sid >= fleet_.size()) {`

```text
--- config checks, before anything is believed -------------------

Both are hard drops rather than partial merges. A mismatched fleet or
grid does not corrupt the data in a way the data can show: every id is
in range, every status is valid, and the merge succeeds — onto the wrong
ground, or crediting the wrong robot. There is no degraded mode here
that is better than not listening.
```

### team-drain-finished-gossip-copy

**Copying finished gossip separately** — attached to `o.finished_gossip.reserve(msg.robot_finished.size());`

```text
Copied on its own, not folded into the position loop below: it is
sized independently on the wire and TeamModel merges it without any of
the freshness rules that loop applies. An old producer that predates
the field sends it EMPTY, which reads through as "no relayed evidence"
and degrades to the first-hand-only behaviour rather than to a wrong
answer — though the type hash makes that fleet unmatched anyway.
```

### team-merge-local-priority-centre

**Local merge priority anchored on current cell** — attached to `const int centre =`

```text
Local priority is anchored on the cell we are STANDING IN, recomputed
per drain rather than cached: the rule is "the robot is here and the
peer is not", and a stale centre would defend ground this robot left
while conceding the ground it is actually working. -1 outside the ROI
(idAt's out-of-range answer), which disables the rule — correctly, a
robot off the map has no neighbourhood to claim.
```

### teamworld-peer-route

**Recording the peer's route** — attached to `if (e.drop_reason.empty()) {`

```text
--- the peer's route (P6, §3.7) ----------------------------------

AFTER every check, and gated on all of them: the grid_hash test is what
makes these cell ids name the same ground ours do, and a route recorded
from a refused message would aim a chase by arithmetic over somebody
else's map. Recorded even when this robot's own predictor is off, so
switching the arm on mid-fleet does not depend on who was listening.

Overwrites unconditionally on an accepted message, including with an
EMPTY tour: an allocator that refused this tick is telling us it has no
route, and keeping the last good one would let a chase be aimed by a
plan the peer has already abandoned. Absence is information here.
```

### teamworld-peer-proposal

**Recording the peer's proposal** — attached to `if (sid >= 0 && sid < static_cast<int>(rendezvous_peer_.size()) &&`

```text
--- the peer's rendezvous proposal (P5) ------------------------

Gated on the same checks and for a stronger reason than the tour: a
cell id from a mismatched grid names different ground, and the whole
protocol turns on comparing this robot's cell id to the peer's for
EQUALITY. Two robots on different grids would find their integers
matching and stand in different places — the exact failure the
exchange exists to remove, reintroduced by trusting an unchecked
sender.

Overwrites unconditionally, empty pair included: a peer that has
dropped its proposal is telling us so, and holding its last one would
let this robot commit a pair no one is offering any more.
```

### rendezvous-peer-cell-bounds

**Bounding a peer's proposed cell id** — attached to `if (rp.cell >= 0 && !cell_world_.grid().valid(rp.cell)) {`

```text
BOUND THE CELL ID AGAINST OUR OWN GRID. RendezvousProposal::valid()
only asks cell >= 0, and this is the one wire->geometry path in the
node with no other bound on it: an adopted cell flows to
appointment_.cell and then to grid().centre(), which its own header
documents as undefined for an id the caller has not screened. The
grid_hash check above makes a wrong-but-in-range id unlikely and
does nothing at all about an out-of-range one, because a hash match
says the grids agree on GEOMETRY, not that the sender's id is inside
it. A sender whose grid is larger than ours passes the geometry
check on the cells we share and can still name a cell we do not have.
```

### rendezvous-peer-receipt-dating

**Dating peer proposals by local receipt** — attached to `const double at = missionElapsedAt(kv.second.received);`

```text
Dated by LOCAL receipt on the local mission clock, which is the only
scale on which this robot can ask "is that still current?". The
producer's own clock never enters it — that is the whole reason the
wire field is an interval and not a time. Falls back to the drain
instant if the stamp predates the mission baseline, which costs at
most the drain delay and never dates a message into the future.
```

### team-drain-acquire-held-read

**Reading acquire and held times post-tick** — attached to `e.acquire_sec = p.acquire_sec;`

```text
R1 (generation 33). Read here with the rest of the post-tick comms
picture because that is when the transition has been computed, and
it is safe to read here because neither transition can occur on a
tick with no packet from this peer -- so the row that exists for a
transition is this one. See TeamModel::Peer::acquire_sec.
```

## main

### main-single-thread-executor

**Single-threaded executor and refused start** — attached to `try {`

```text
Single-threaded executor: the planner ingests the fused map over a topic
subscription (non-blocking), so there is no blocking service future to
service on a second thread. All callbacks and timers run on one thread,
which also makes the map-callback / state-machine interaction race-free.

Construction can refuse: a malformed fleet definition is a config error the
node must not run past, because every knowledge mask it published would be
indexed against a fleet the rest of the team does not share. Catching it
here rather than letting it escape gives an operator one legible line and
the harness a non-zero exit, instead of a std::terminate backtrace.
```
