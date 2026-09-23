# coordination.hpp — design notes and history

The long comments of `include/explo_planner/coordination.hpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Claim — declarations](#claim--declarations) — 1
- [Coordination — declarations](#coordination--declarations) — 9

## Claim — declarations

### coord-claim-exploit-fields

**Exploit fields on a peer claim** — attached to `bool exploit = false;` (line 48)

```text
Exploitation coupling (see RobotIntent.msg). exploit=true marks an
exploit hop on tree `target_id`; dwelled_mask is the producer's
cumulative clear-LoS dwelled vantage-index set on that target; staged is
the producer's own declaration that it is standing on goal_pos in its
dwell state (false while it is still driving anywhere).
```

## Coordination — declarations

### coord-max-claim-ttl-cap

**Why peer claim TTLs are capped** — attached to `static constexpr float kMaxClaimTtlSec = 60.0f;` (line 59)

```text
Largest TTL (seconds) any peer may claim, however large a value it
advertises. A claim's TTL means "how long since we last HEARD this peer",
and the heartbeat is ~1 Hz, so anything past a minute means the peer is
gone. Without a cap an intent carrying +inf (or a garbage float from a
version-skewed publisher) produced a claim that prune() could never expire
— a permanent veto over a disc of the map, with no way to clear it short of
restarting the node.
```

### coord-ctor-radius-cap-exploit-grace

**Claim radius cap and exploit grace** — attached to `Coordination(bool enabled, std::string self_id,` (line 68)

```text
@param max_claim_radius_m Upper bound applied to every peer-advertised
       claim radius (see onIntent). Pass the local
       `coord_claim_radius_m` — the exploration-scale disc — since that is
       the largest claim this planner itself considers legitimate. <= 0 or
       non-finite disables the bound (test/legacy default).
@param exploit_claim_grace_sec Extra retention applied to EXPLOIT claims
       only: prune() keeps them for `expiry + grace` instead of dropping
       them at `expiry`. The TTL is sized for a healthy 1 Hz heartbeat,
       but the RECEIVER is a single-threaded executor whose EXPLOIT_PLAN
       retry ticks can monopolise it for several seconds at a stretch
       (fused-map refresh + whole-grid floods), so a peer's claims sit
       undelivered while its engagement is as live as ever. In a 2-robot
       sim run that starved window aged the driving winner's claim out of
       the parked loser's table twice — provably between two plan ticks
       7 ms apart — and each time the loser re-selected the very vantage
       the winner was mid-drive to, rolled toward it, and had to be
       yielded back off it. Grace keeps a recently-heard exploit
       engagement contestable across such receive-side gaps. It is a
       VETO-side lenience only: everything that needs "is this peer
       actually alive right now" semantics — the dwell barrier's release,
       rendezvous presence counting — filters on the raw expiry and is
       unaffected. 0 (default) preserves legacy single-TTL behaviour.
```

### coord-intent-local-receipt-expiry

**Claim expiry on the local clock** — attached to `void onIntent(const explo_planner_msgs::msg::RobotIntent& msg,` (line 94)

```text
Process an incoming intent, stamped with the LOCAL receipt time
`now_local` (the caller's node clock). Drops messages whose robot_id
matches self_id_ (echo of our own broadcast). Stores latest claim per
peer, expiring at now_local + msg.ttl_sec — the TTL means "freshness
since we last heard this peer", measured entirely on the local clock.
The producer's header.stamp is deliberately NOT used for expiry: field
robots' clocks are not synchronised (offsets of seconds to hours have
been observed), and an expiry built from the peer's stamp but pruned
against local now() makes claims immortal when the peer's clock is
ahead and stillborn when it is behind by more than the TTL.
```

### coord-prune-exploit-grace

**Pruning with the exploit grace window** — attached to `void prune(const rclcpp::Time& now);` (line 107)

```text
Drop expired claims. Called once per PLAN tick from the planner.
EXPLOIT claims are retained for `expiry + exploit_claim_grace_sec`
rather than dropped at `expiry` (see the constructor doc) — every
lookup that must not see a graced claim filters on the raw expiry
itself instead of relying on this.
```

### coord-claim-matching-own-radius

**claimMatching radius and liveness filter** — attached to `const Claim* claimMatching(const Eigen::Vector3f& candidate_xy,` (line 114)

```text
MinPos lookup primitive. Returns the active peer claim whose disc
overlaps `candidate_xy`, or nullptr if no peer contests this
candidate. If multiple peers contest the same region, returns the
peer whose `robot_pos` is closest to `candidate_xy` — which is why
this works unchanged at the N=3 and N=4 the campaigns run.

EACH CLAIM IS EVALUATED AT ITS OWN RADIUS, not at ours: the disc is
`claim.goal_pos +/- claim.radius_m`, because the radius is the size
of the region that peer is occupying and it is phase-dependent (~8-10 m
exploring, ~0.75 m holding a vantage). `match_radius_m` is the FALLBACK
only, used when a claim arrives with `radius_m <= 0` — an older node
that sent nothing usable — and the caller sets it from
`coord_claim_radius_m`. See the note at the top of claimMatching() in
coordination.cpp for what evaluating everything at the receiver's own
scale got wrong. 2D Euclidean (XY); the simulation is ground-restricted.

`live_after`, when non-null, skips claims whose expiry is at or before
that instant. Exploit claims outlive their expiry by the grace window
(see prune()), which is exactly what the EXPLOIT contest sites want —
a vantage under recent pursuit stays vetoed across a receive-side gap —
but the EXPLORATION MinPos walk wants live claims only: a graced claim
there would keep contesting frontier candidates for a peer we may not
have heard from in 10+ seconds. Exploration passes its plan-tick time;
exploit sites pass nullptr.
```

### coord-dwell-barrier-unstaged-probe

**The dwell barrier's unstaged-peer probe** — attached to `const Claim* firstUnstagedExploitPeer(uint32_t target_id,` (line 157)

```text
Rendezvous-barrier probe. Returns the first active exploit claim on
`target_id` whose producer has not declared itself staged (`staged ==
false`, i.e. it is still driving somewhere), or nullptr once every such
peer is standing on its vantage (and when no peer claims this target at
all). A robot that has arrived at its own vantage holds its dwell for as
long as this keeps returning non-null, so the returned claim is "the peer
we are waiting for" and exists for the caller to log. The ~1 Hz intent
heartbeat is what makes the barrier progress: each republish carries the
producer's current staging, so an inbound peer's claim goes staged the
moment that peer enters its own dwell.

Staging is PRODUCER-DECLARED (set at EXPLOIT_DWELL entry, after nav
arrival and the post-arrival rotation settle) and this probe consults
nothing else — no position, no tolerance. It used to infer staging from
geometry, `robot_pos` within a tolerance of the claim's own `goal_pos`,
which cannot separate three situations that look identical in XY: holding
the vantage; arrived but still rotating in place, which read as staged
~5 s before the robot settled, so peers began dwelling early and the
team's simultaneous-capture overlap collapsed to ~3 s of an 8 s dwell; and
parked at an APPROACH waypoint — an exploit hop whose goal IS the waypoint
— which read as staged while nowhere near a vantage. Only the producer
knows which of its exploit hops is a vantage it is actually holding.

Unlike claimMatching() / peerDwellUnion(), this lookup filters expiry
ITSELF rather than relying on prune(). Its caller sits in EXPLOIT_DWELL,
where the PLAN tick — and therefore prune() — does not run, so nothing
clears the table while we wait: the stale claim of a peer that died on
its way in would stay unstaged and hold the barrier forever, deadlocking
the robot that did arrive. Skipping `c.expiry <= now` here makes the
receipt-time TTL (~5 s, a heartbeat plus margin) the barrier's release
path — a dead peer stops holding the team one TTL after we last heard it.
The exploit-claim GRACE window (see the constructor) is deliberately NOT
applied here: grace lengthens how long a peer's claim can VETO a vantage,
and stretching the barrier's hold on a silent peer by the same window
would triple how long a dead teammate pins a robot mid-dwell.

No enabled_ check, as with claimMatching(): the caller gates on enabled().
```

### coord-staged-peer-selection-contest

**Selection-time contest against parked peers** — attached to `const Claim* stagedExploitPeerWinning(uint32_t target_id,` (line 197)

```text
Selection-time contest against a PARKED peer. Returns the first active
claim that is (a) an exploit claim on `target_id`, (b) staged — its
producer is standing on a vantage of this same ring — and (c) beats us for
`candidate_xy` under selfWinsAgainst()'s total order (strictly closer
`robot_pos`, lexicographic robot_id on an exact tie). nullptr means no such
peer exists and the candidate is ours to take; a non-null result is "do not
select this vantage, that peer is about to", and exists for the caller to
log.

This closes the blind window between two robots' re-plans. When the team's
synchronised dwells end, both robots leave EXPLOIT_DWELL and re-plan within
milliseconds of each other, both see the same single remaining un-dwelled
vantage, and both select it — neither has yet received the other's claim on
it, because the intent heartbeat is only ~1 Hz. The later claim does make
one of them yield, but by then both have driven at the same angle; in a
2-robot sim run they physically collided. Deciding the same contest at
SELECTION time, from claims we already hold, means the yield happens before
either robot moves.

Only STAGED claims are consulted, and that restriction is the whole design.
A staged peer is parked beside this ring and about to re-plan from a dead
standstill, so if it is closer to the candidate angle it will win the race
for it — conceding up front costs nothing. A DRIVING peer is the opposite
case: it contests only through its claim disc (claimMatching(), the
existing rules), because a position contest against a mover would let one
peer mid-drive freeze an approaching robot out of EVERY vantage on the ring
— the robot arriving from far away is farther from all of them — which
destroys the parallelism exploitation exists for. Its wire pose is a
heartbeat stale anyway, so the comparison would not even be meaningful.

Both robots reach COMPLEMENTARY conclusions, which is what makes this safe
to run independently on each: a staged producer is by construction
stationary (the flag is set at EXPLOIT_DWELL entry and re-broadcast while
dwelling and while re-planning in place; a driving robot always publishes
staged=false), so its broadcast `robot_pos` still agrees with its true
position. Both sides therefore evaluate the same total order on the same
positions, and since that order is antisymmetric, exactly one of them
admits the vantage. Nothing is negotiated and no extra message is needed.

Filters expiry itself, as firstUnstagedExploitPeer() does — see its note;
here prune() does run each tick (the caller is in EXPLOIT_PLAN), but the
self-filter keeps the probe correct from any call site. Unlike the
barrier probe this one honours the exploit-claim grace window: it is a
veto (the contested angle stays un-selected), and a parked peer whose
heartbeats sat undelivered for a few seconds is still parked — the flag
only ever flips through a message (a staged producer that starts driving
publishes staged=false in the same tick it selects), so a graced staged
claim is stale in age, not in meaning. A peer that genuinely died on its
vantage stops winning candidates one grace window after the TTL.
```

### coord-build-intent

**Building the outgoing intent** — attached to `explo_planner_msgs::msg::RobotIntent buildIntent(const CandidateViewpoint& goal,` (line 252)

```text
Build the next outgoing intent for `goal`. Caller publishes on the
`coord_intent_topic` publisher. The producer fills `header.stamp`
from `now` and `header.frame_id` from `map_frame`; planner_type_id
is the 0..3 enum from RobotIntent.msg. The trailing exploit fields
default to the exploration claim shape (exploit=false, zeros, unstaged).
`staged` is the producer's declaration that it is standing on `goal` in
its dwell state; only the EXPLOIT_DWELL publish passes true.
```

### coord-live-peer-count-vs-node

**livePeerCount is not the node's count** — attached to `size_t livePeerCount(const rclcpp::Time& now) const;` (line 281)

```text
Number of peer claims that are LIVE at `now` (raw expiry, no grace).
This is the CLAIM-TABLE presence count. With grace retention in the
table, claims_.size() stopped meaning that — a peer 10 s silent would
have kept counting as present at the rendezvous anchor for the whole
grace window, releasing the return barrier on a teammate that may be
face-down in a ditch.

IT IS NO LONGER THE NODE'S PRESENCE COUNT, and this comment claimed it was
until 2026-09-18 ("the rendezvous barrier releases on ... and the CSV
`coord_active_peers` column documents the same thing"). Since generation
23 both of those read ExploPlannerNode::accountedPeerCount, which unions
this table with TeamWorld's `direct` handshake and with a peer's
`finished` bit; the raw call survives at exactly one site, the
TeamModel-not-yet-configured fallback inside that wrapper. Coordination
cannot see the wrapper, so this stays the right thing for the LIBRARY to
expose — it is just not what the barrier or the column now record.
```
