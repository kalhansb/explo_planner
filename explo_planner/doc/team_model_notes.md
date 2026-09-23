# team_model.hpp, team_model.cpp — design notes and history

The long comments of team_model.hpp, team_model.cpp, moved out of the code on 2026-09-23 so the sources carry short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

One section per source file. Each entry names the function (or section) the comment sat in, the line of code it was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [include/explo_planner/team_model.hpp](#includeexplo_plannerteam_modelhpp) — 16
- [src/team_model.cpp](#srcteam_modelcpp) — 12

## include/explo_planner/team_model.hpp

### team-gossip-max-age

**Gossip freshness bound** — in `Config — declarations`, attached to `double gossip_max_age_sec = 120.0;` (line 82)

```text
A peer's gossip about a third robot is only usable while the gossip
itself is fresh. Past this, the third-party entry is treated as never
heard — NOT as "heard a long time ago", because a very stale interval
added to a very stale message is a number with no meaning that would
still compare against thresholds.
```

### team-link-timing-packet-clock

**Link timing fields use the packet clock** — in `Peer — declarations`, attached to `double   one_way_since_sec = -1.0;` (line 115)

```text
The detector is not changed by any of the four fields below; they only
TIME it. Both measurements exist today by offline reconstruction against
an oracle the robot does not have, which is the gap R1 names.

ALL FOUR ARE STAMPED FROM THE PACKET CLOCK (`reported_at_sec_`), never
from the tick clock. A tick is a coarse, jittering sample of a link that
changes on arrivals, so differencing tick times would report the planner's
scheduling noise as link behaviour. Differencing the two packets that
bracket the transition reports the link.
```

### team-one-way-since

**Start of the current one-way period** — in `Peer — declarations`, attached to `double   one_way_since_sec = -1.0;` (line 125)

```text
Packet stamp that opens the CURRENT period without a completed
handshake — the first packet of a receiving run, or the packet that
broke the last handshake while the peer stayed audible. Negative while
not receiving. This is the "first one-way packet" acquisition latency is
measured from. It is stamped whether or not that packet already
completes the handshake, so an instant handshake correctly reads 0.
```

### team-acquire-one-shot

**One-shot handshake acquisition latency** — in `Peer — declarations`, attached to `double   acquire_sec = -1.0;` (line 138)

```text
ONE-SHOT, and the only two fields in this struct that are: set on the
tick that carries the transition and cleared on every other tick, so a
reader counts events by counting non-negative readings instead of
diffing a level. Negative means "no transition this tick".

THE ONE-SHOT IS SOUND BECAUSE NEITHER TRANSITION CAN HAPPEN ON A TICK
WITH NO PACKET FROM THIS PEER. Both `receiving` and the peer's mask move
only on an arrival, so a tick with an empty batch can lower `direct` by
TTL expiry (which is not this measurement) but can never raise it and can
never break a handshake while still receiving. The value is therefore
always readable on a row that exists for this peer in the same drain.

`acquire_sec`: seconds from the first packet of the contact run to the
packet that completed the handshake — acquisition LATENCY.
```

### team-held-sec

**How long a handshake held** — in `Peer — declarations`, attached to `double   held_sec = -1.0;` (line 153)

```text
`held_sec`: seconds a completed handshake survived, from the packet that
made it to the packet that broke it, and set ONLY on a break that
happens WHILE STILL RECEIVING — the peer is still audible and stopped
naming us back. A link that ends because the packets stopped is a
different event with a different cause (§2.7's fleet-wide blackout), it
has no closing packet to stamp, and conflating the two would average a
delivery failure into a handshake statistic.
```

### team-position-sec

**Position age versus last known** — in `Peer — declarations`, attached to `double   position_sec = -1.0;` (line 169)

```text
Mission-elapsed seconds the POSITION dates from. Negative for never.

Tracked separately from last_known_sec, and the difference is rule 3 of
the file header made concrete rather than a redundancy. Both first-hand
and gossiped updates refresh last_known_sec BEFORE testing whether the
message carried a position at all, so a status-only relay about a robot
nobody has seen in minutes leaves last_known_sec reading "fresh" over a
position that is minutes old. A consumer that steers on the position —
a chase, a separation term — must key on this field.
```

### team-peer-finished

**The sticky finished flag and its relay** — in `Peer — declarations`, attached to `bool     finished = false;` (line 180)

```text
The peer's run is over (TeamWorld/finished). MONOTONIC and STICKY: set
first-hand from the peer's own message, or by relay from a third robot
that heard it, and never cleared by either. The publisher latches the
bit, so false here means "no evidence", not "still exploring".

It is relayed — unlike `team_incomplete` below — because it is a
monotonic statement a robot makes about ITSELF, so a relayed copy can
neither contradict a first-hand one nor echo back to its originator.
Without the relay, a robot that cannot hear the finished peer keeps
counting it missing forever and holds the whole team at the unbounded
appointment barrier; see TeamWorld.msg/robot_finished.
```

### team-peer-mode

**Peer mode and what it adds** — in `Peer — declarations`, attached to `uint8_t  mode = 0;` (line 193)

```text
What the peer is doing with the rest of its run (TeamWorld/mode), on the
ordered scale EXPLORING(0) < HOMING(1) < DONE(2). MONOTONIC and STICKY
for `finished`'s reasons exactly, merged by MAX instead of OR because
the fact has three levels rather than two. 0 means "no evidence", not
"confirmed exploring". It follows `finished`'s two-channel rule to the
letter: the FIRST-HAND assignment is authoritative and may therefore
LOWER the level (the restarted-node case `finished` documents applies
here identically), while RELAY can only ever raise it.

WHAT IT BUYS OVER `finished`, which is the whole point of having both:
it fills the window between "left for home" and "arrived and announced
finished". A peer in that window is not coming to the meeting and will
explore no more cells, but `finished` still reads false — so a partner
waits at an unbounded barrier, and the allocator reserves frontier for
a robot that is driving the other way. Read this where the question is
"will this peer participate?"; read `finished` where it is "is its run
over?". See TeamWorld.msg/mode for the levels and their preconditions.
```

### team-peer-team-incomplete

**The peer's team_incomplete bit** — in `Peer — declarations`, attached to `bool     team_incomplete = false;` (line 212)

```text
The peer's own FIRST-HAND answer to "is the team whole?", as it sent it
(TeamWorld/team_incomplete). NOT its derived armed state — see the
field's own documentation in TeamWorld.msg for why announcing the armed
state instead deadlocks.

Stale-safe in one direction only, which is the useful one: this is
whatever the peer last said, with no TTL of its own, so a consumer must
pair it with `direct || heard_one_way` to mean "a peer we are receiving
from RIGHT NOW says the team is broken". Both flags are required, not
`direct` alone: one-way contact (we receive from the peer, it cannot
hear us) is exactly the case this exists to catch — the peer's own read
is broken, it is announcing so, and it has no other way to tell us.
```

### team-peer-appointment-inbound

**The appointment_inbound bit** — in `Peer — declarations`, attached to `bool     appointment_inbound = false;` (line 226)

```text
The peer is in a rendezvous appointment manoeuvre and has not stopped
driving yet (TeamWorld/appointment_inbound). First-hand only, like
team_incomplete, and its reader pairs it with `direct || heard_one_way`
for the same reason: it carries no TTL of its own.

It clears when the DRIVE ends — arrival, nav budget, or no-progress —
and not when the cell is reached, so a peer whose destination turned out
unreachable stops holding the barrier instead of hanging it. That bound
is also why it needs no finished exemption; see TeamWorld.msg.
```

### team-peer-inbound-seen

**The one-hop appointment_inbound_seen bit** — in `Peer — declarations`, attached to `bool     appointment_inbound_seen = false;` (line 237)

```text
The peer reports that some robot IT receives first-hand is still
driving to the agreed cell (TeamWorld/appointment_inbound_seen): the
one-hop companion to the bit above, added with the generation-27
closure door. The sender derives it from raw first-hand
appointment_inbound bits only, never from other robots' copies of this
field, so it cannot echo (see TeamWorld.msg). Same reader contract as
the bit above: pair with `direct || heard_one_way`, no TTL of its own.
```

### team-finished-gossip

**Relayed finished is not age-gated** — in `Observation — declarations`, attached to `std::vector<uint8_t> finished_gossip;` (line 290)

```text
Relayed `finished`, indexed by fleet id. Merged as a pure OR that never
clears, and — unlike everything else in this block — NOT age-gated: a
finished robot goes quiet, so its last-heard entry ages out of
gossip_max_age_sec exactly when the bit matters. A monotonic fact has no
freshness to check. See Peer::finished and TeamWorld.msg/robot_finished.
```

### team-tick-whole-graph

**Why status is recomputed in tick** — in `TeamModel — declarations`, attached to `void tick(double now_sec);` (line 308)

```text
Recompute every peer's status: TTL expiry, handshake, then closure.
Separated from observe() because closure is a property of the whole graph
and must not be recomputed once per arriving message — with three robots
publishing at 1 Hz that would be three different answers per second, and
whichever one a consumer happened to read would be the one that counted.
```

### team-position-age-accessor

**The position age accessor** — in `TeamModel — declarations`, attached to `double positionAgeSec(int id, double now_sec) const;` (line 325)

```text
Seconds since the POSITION we hold for `id` was measured, or negative
for "we hold no position". This is the accessor the file header has
always named and the one every consumer that STEERS on a peer position
must use; lastKnownAgeSec() answers a different question and can read
fresh over a stale position (see Peer::position_sec).
```

### team-in-comms-mask-local

**The closure mask must not be published** — in `TeamModel — declarations`, attached to `uint32_t inCommsMask() const;` (line 337)

```text
Mask of peers currently IN_COMMS, self included — the closure result.
This is a local conclusion and must NOT be published: TeamWorld's
in_range_mask is defined as direct contacts, and putting the closure mask
there would make the handshake circular — A claims to hear C because B
said C was reachable, C concludes the same about A from its own copy of
the same relay, and the mutual test that rule 1 exists for passes without
anybody having heard anybody.
```

### team-direct-mask-published

**The published direct-contact mask** — in `TeamModel — declarations`, attached to `uint32_t directMask() const;` (line 346)

```text
Mask of peers heard DIRECTLY inside the TTL, self included. This is the
mask to publish in TeamWorld.in_range_mask.

Deliberately NOT the handshake result: this is "I receive from them",
which is the half of the handshake only we can observe. Publishing the
mutual result instead would deadlock a healthy link — neither robot could
name the other until the other had already named it.
```

## src/team_model.cpp

### team-gossip-array-length-backstop

**Refusing gossip longer than the fleet** — in `TeamModel::observe`, attached to `const size_t n = static_cast<size_t>(size_);` (line 74)

```text
A gossip array longer than the fleet means the sender is running a
different team_robot_names than we are, so its ids address different
robots than ours do. The team_hash check upstream should have caught it;
this is the backstop, and it refuses rather than truncating, because
truncating would apply the sender's robot 0 to our robot 0.
```

### team-first-hand-finished-clears

**First-hand finished is authoritative** — in `TeamModel::observe`, attached to `sp.finished       = obs.finished;` (line 94)

```text
FIRST-HAND IS AUTHORITATIVE and is a plain assignment, so it can also
CLEAR. The relay below can only ever set the bit, so this is the one site
that can undo it -- which matters for the single case where the bit is not
monotonic in reality rather than on the wire: a node that restarts mid-run
(its mission clock returns to zero, the case the gossip block below also
calls out) genuinely un-finishes, and the robot itself is the only witness
worth believing. Peers that learned the old bit by relay keep it until they
hear the restarted robot first-hand; they are then receiving from it
directly, which is what peerAccounted keys on anyway.
```

### team-incomplete-first-hand-only

**Why team_incomplete is never relayed** — in `TeamModel::observe`, attached to `sp.team_incomplete = obs.team_incomplete;` (line 108)

```text
team_incomplete IS FIRST-HAND ONLY, and it does not follow `finished` above
into the relay. The two are not the same kind of statement. This one is a
NON-MONOTONIC claim a robot makes ABOUT THE TEAM, and relaying it is the
deadlock TeamWorld.msg warns about arriving by a different route -- C would
re-announce A's break as though it were C's own read and A would then see
its own bit come back, with nothing able to clear it. `finished` is a
MONOTONIC claim a robot makes ABOUT ITSELF: it cannot contradict a
first-hand copy, cannot arrive early, and has no path back to its
originator, which is exactly why it is safe to relay and this is not.
```

### team-appointment-inbound-seen

**Storing the one-hop inbound report** — in `TeamModel::observe`, attached to `sp.appointment_inbound_seen = obs.appointment_inbound_seen;` (line 122)

```text
Its one-hop report lands the same way: it is the sender's own statement
about what the sender currently receives, stored under the sender so the
reader believes it exactly while the sender itself is heard. The sender
derives it from raw first-hand bits only (see TeamWorld.msg), so storing
it here cannot start the echo the two comments above forbid.
```

### team-gossip-clock-ages

**Gossip timestamps as ages** — in `TeamModel::observe`, attached to `const size_t nh = obs.last_heard_sec.size();` (line 141)

```text
The sender's entry for ITSELF is its publish time on its own mission
clock. Everything else in the array is an earlier reading on that same
clock, so (sender_now - entry) is an AGE — an interval, which transfers
between clocks unchanged. Without the sender's own entry there is no
reference point and the whole array is uninterpretable; it is dropped
rather than guessed at, because guessing (say, assuming the sender
published at its latest entry) would systematically under-age gossip and
make stale positions look current.
```

### team-gossip-transit-bound

**Gossip transit time and restarts** — in `TeamModel::observe`, attached to `const double at = now_sec - age;` (line 162)

```text
Local mission-elapsed time this information dates from.

Stated bound: this treats transit as instantaneous, so gossip is
under-aged by however long the message took to arrive. Sub-second in
normal operation, and gossip_max_age_sec is set with slack for it.
The pathological case is a message DELAYED or DUPLICATED by minutes,
which would look that much fresher than it is. It is not guarded
against, deliberately: the obvious guard — refuse a message whose
sender entry is not strictly greater than the last we accepted from
that sender — deadlocks that peer permanently the first time its
node restarts and its mission clock returns to zero, which is a real
event and a far worse failure than an over-fresh duplicate.
```

### team-relayed-position-stamp

**Stamping relayed positions** — in `TeamModel::observe`, attached to `p.position_sec        = at;` (line 186)

```text
`at`, not now_sec: this position was measured when the RELAY heard
it, and the whole point of the interval arithmetic above is to
recover that instant. Stamping it with the receipt time would make
a two-minute-old relayed pose look one tick old — which is the
failure this field exists to prevent, reintroduced at the one site
where it actually happens.
```

### team-relayed-finished

**The relayed finished bit** — in `TeamModel::observe`, attached to `for (size_t k = 0; k < obs.finished_gossip.size(); ++k) {` (line 202)

```text
A SEPARATE LOOP, AND DELIBERATELY NOT INSIDE THE ONE ABOVE. Every guard
that block applies would suppress this bit exactly when it is needed. It
skips a peer the sender never heard, skips one older than
gossip_max_age_sec, and skips one whose reading is not strictly fresher
than what we hold -- all of which are freshness rules, and a finished robot
STOPS MOVING AND GOES QUIET, so its last-heard entry is the first thing to
age out. Gating a monotonic fact on freshness would switch the relay off at
the moment it starts to matter. There is no freshness to check: the bit
only ever goes one way.

PURE OR, NEVER CLEARS. A relayed false is "the sender has no evidence", not
"that robot is still exploring" -- the sender may simply not have heard it
either -- so false carries no information and must not overwrite a bit we
already hold. Only the first-hand assignment above can clear.

WHY IT EXISTS: without it, a robot that cannot hear a finished peer counts
it missing forever (nothing ever clears the stale first-hand false), so it
announces the team incomplete forever, and every robot that CAN hear the
finished peer is held by that announcement at the unbounded appointment
barrier. At N>=3 that hangs the run to the harness duration cap. See
TeamWorld.msg/robot_finished for the full trace.
```

### team-relayed-mode

**The relayed mode level** — in `TeamModel::observe`, attached to `for (size_t k = 0; k < obs.mode_gossip.size(); ++k) {` (line 234)

```text
Every argument in the block above transfers verbatim: not age-gated
(a homing robot goes quiet in the same way a finished one does, and its
level matters most once it has), and a merge that can only ever raise
(a relayed EXPLORING is "the sender has no evidence", never "that robot is
still exploring"). The one difference is the operator -- MAX rather than
OR, because this fact has three ordered levels instead of two.

WHY IT EXISTS separately from `finished`: it covers the window between
leaving for home and arriving there, where `finished` is still false and a
peer is nonetheless never coming back to the frontier or to a meeting.
```

### team-r1-instrumentation-edges

**Reading link edges before overwrite** — in `TeamModel::tick`, attached to `p.acquire_sec = -1.0;` (line 272)

```text
Read the edges BEFORE the flags are overwritten: both measurements are
differences across a transition, so they need the previous state, and
the previous state is exactly what the three assignments below destroy.
The two outputs are cleared unconditionally first — they describe THIS
tick, and a stale value left standing would be counted as a second
event by a reader that counts non-negative readings.
```

### team-one-way-period-restart

**Where the next one-way period starts** — in `TeamModel::tick`, attached to `p.one_way_since_sec = at;` (line 297)

```text
AND THE NEXT ONE-WAY PERIOD STARTS HERE, not where the receiving run
did. A link that breaks and re-forms without the packets ever
stopping would otherwise have its second acquisition measured from
the first packet of the whole run — reporting the length of the
preceding HOLD as though it were acquisition latency.
```

### team-closure-two-hops

**Two-hop comms closure** — in `TeamModel::tick`, attached to `for (int i = 0; i < size_; ++i) {` (line 319)

```text
Peer C is IN_COMMS if some robot we are already in comms with hears C
directly. Expansion goes only through rows we have a FRESH first-hand copy
of — which, since a peer's row arrives on its own message, means only
through robots we can hear ourselves. Closure is therefore TWO HOPS, not
unbounded: A—B—C is covered (the case §3.3 is written for), A—B—C—D is
not, because nothing in TeamWorld carries C's mask to A. That is a
limitation of the message, not an oversight here; extending it would mean
relaying third-party masks the way positions are relayed, and the plan
deliberately does not.

The far edge B—C is taken on B's word alone. It cannot be handshaken from
here — we cannot hear C — and requiring it would make closure impossible
rather than careful. The exposure is bounded: a wrong closure suppresses a
reconnection dispatch, and every consumer that would act on C's DATA keys
on lastKnownAgeSec() instead, which no closure can inflate.
```
