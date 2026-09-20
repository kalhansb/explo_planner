# The shared team world model: one robot's full view of the coarse cell world,
# its own global route, its comms picture, and the third-party gossip it is
# relaying. Broadcast at `team_world_hz` on `exploration/team_world` and routed
# through the comms emulator exactly like RobotIntent.
#
# WHY A NEW MESSAGE AND NOT MORE FIELDS ON RobotIntent
# ----------------------------------------------------
# RobotIntent's own header warns why: on Humble (no XTypes type negotiation) a
# consumer built WITH a new field cannot deserialise a producer built WITHOUT
# it — the CDR buffer underflows, rmw drops the message before user code runs,
# and the producer becomes silently and permanently invisible to that consumer
# for EVERYTHING, claims included. A new type fails the other way: a
# mixed-build fleet gets an unmatched type, which is loud. Deconfliction must
# not be able to die because the world model changed shape.
#
# FULL STATE, NOT DELTAS
# ----------------------
# Every publish carries the complete cell census, not a diff. At <=400 cells x
# ~11 bytes this is a few kB even at 1 Hz. The gain is that merge is idempotent
# and loss-free by construction: a dropped message costs latency and nothing
# else, and there is no sequence-number recovery protocol to get wrong. mTARE
# ships a delta-sync protocol (kSyncDelay); we deliberately do not port it.
#
# header.stamp = producer's clock at publish (DIAGNOSTIC ONLY). As with
#   RobotIntent, nothing times anything off it: field robots' clocks have been
#   observed hours apart. Freshness is measured from LOCAL receipt, and neither
#   scheduling quantity on this message is an absolute stamp:
#   robot_last_heard_sec and rendezvous_t_meet_ms are MISSION-ELAPSED on the
#   producer's own clock, and rendezvous_interval_ms is a duration with no
#   origin at all.
# header.frame_id = the planning map frame (e.g. "map").
std_msgs/Header header

# Producer's numeric fleet id: its index in the ordered `team_robot_names`
# param. Every bitmask in this message and in cell_world is indexed by it.
# A numeric id is needed because a free-form string cannot address a bit.
uint8 robot_id

# FNV-1a 32-bit hash of the producer's ordered `team_robot_names`. A receiver
# whose own hash differs is holding a different fleet definition, which means
# every robot_id and every mask bit in this message points somewhere else than
# the sender meant — and, because cell ids are derived from shared ROI params
# too, quite possibly a different cell grid. That is not a condition to
# degrade through: the mismatch is reported loudly and the message dropped.
uint32 team_hash

# FNV-1a 32-bit hash of the producer's CELL GRID geometry (origin, cell size,
# nx, ny), per CellGrid::configHash(). team_hash above cannot cover this: two
# robots can share an identical `team_robot_names` and still derive different
# grids from different ROI params, and then every `id` in `cells` names
# different ground on the two of them. Nothing in the cell data would look
# wrong — the ids are all in range, the statuses are all valid, and the merge
# succeeds — so the receiver would hold a confident census of the wrong
# ground. The only way to catch it is to compare the geometry directly, which
# is what this field is for.
#
# The field is NOT in the plan's §3.2 field list; it was added when the check
# was implemented, because cell_world.hpp's own contract ("a cell id names the
# same ground on every robot ONLY if all of them derived it from the same ROI
# and cell size, which is why configHash() exists and why TeamWorld's config
# check must fold it in") requires a carrier and §3.2 gave it none.
uint32 grid_hash

# Producer's current pose (map frame). The allocator's vehicle-start position
# for this robot, and the freshest first-hand position anyone has for it.
geometry_msgs/Point position

# Bitmask of robot ids the producer can hear DIRECTLY right now (bit k = robot
# k; the producer's own bit is set). This is what makes the comms handshake
# mutual: a receiver treats the producer as in-comms only if it can hear the
# producer AND this mask names the receiver back. One-way contact — documented
# in doc/limitations.md §10 as a real failure mode, not a hypothetical — is
# otherwise indistinguishable from a working link on the receiving side, and a
# robot that "reconnects" over a link it cannot answer on has shared nothing.
uint32 in_range_mask

# The producer's full cell census, normalised per CellState.msg.
CellState[] cells

# Cell ids of the producer's current global route, in visit order. Its first
# entry is the producer's focus cell. Consumed by the MDP pursuit predictor to
# intercept where the peer WILL be rather than where it was last seen.
#
# Only the SENDER'S OWN tour travels; tours are not relayed (see the gossip
# arrays below, which are). A pursuit of a peer never heard first-hand
# therefore degrades to the legacy trail chase, which is the pre-existing
# behaviour and no worse.
#
# "CURRENT" IS LOAD-BEARING AND THERE IS NO STAMP TO CHECK IT AGAINST. The
# field carries no age, so a receiver cannot distinguish a route the producer
# is driving from one it abandoned; it must be able to trust the word. The
# producer therefore sends this EMPTY whenever it is not in the exploration
# loop that produced the route — any reconnect manoeuvre, the mission-return
# leg, the exploitation phase, or after its coverage latch — because that is
# also when the allocator stops re-solving and the route would freeze. Empty
# is the designed encoding for "I have no route", not a gap: the receiver
# overwrites unconditionally, including with an empty array, and falls back to
# the trail chase. Absence is information here.
uint16[] my_tour

# THE PRODUCER'S RUN IS OVER: it has either latched exploration done on its own
# map, or reached State::DONE by any route (step budget, coverage saturation,
# a barrier that gave up, the end of a mission-return leg). A finished robot
# leaves the allocator's vehicle set — its cells return to the pool rather
# than being assigned to a robot that will never drive them — and the
# knowledge gate stops counting it as someone worth reconnecting to.
#
# Generation 23 widened this from the coverage latch alone. The latch is
# written at ONE site and no other ending touches it, so every non-latch
# finisher used to announce finished=false forever from wherever it parked,
# which pinned teamSettled false on every peer and held the whole team at an
# unbounded appointment barrier until the run's duration cap. Widening it is
# safe in the one direction that matters: DONE is strictly later than the
# latch, so the bit cannot now fire before exploration has actually stopped.
bool finished

# THE PRODUCER'S OWN ANSWER TO "IS THE TEAM WHOLE?" — that is,
# !teamComplete(accountedPeerCount, rendezvous_expected_peers_) evaluated over
# what it is receiving right now. Added in generation 23 to carry out the
# rule "if ANY robot is disconnected, ALL robots go to the rendezvous".
#
# NAME THE RIGHT COUNT. This read `livePeerCount` until 2026-09-18 and the
# publisher has never called it: livePeerCount is the raw claim table, and
# accountedPeerCount unions it with TeamWorld's `direct` handshake and with
# `finished`. The difference decides who announces — a robot whose only missing
# peer has finished reads COMPLETE on the count the publisher uses and BROKEN on
# the one this comment named — so it is not a synonym.
#
# WHY IT MUST BE ON THE WIRE. No channel in accountedPeerCount relays
# REACHABILITY: the claim table counts intents this robot received and nothing
# relays a peer's peer list (RobotIntent carries no mask, and
# hmr_comms_sim_node forwards nothing), and `direct` is a first-hand handshake.
# (`finished` IS relayed — robot_finished below — but it is a statement about a
# peer's run ending, not about who can hear whom, so it closes no partition.)
# So in
# an A—B—C bridge, where A and B have lost each other but C still hears both,
# A and B read the team as broken and C reads it as whole. Nor can C learn it
# some other way: dscovox subscribes to peers' raw scovox_bin and publishes only
# fused products for LOCAL consumers, so there is no map relay either — A and B
# are in a genuine information partition and C is the only robot that can close
# it. Without this field C keeps exploring, the meeting never happens, and the
# two robots that need each other wait out the full cap.
#
# ONE HOP IS ENOUGH, at every N and every topology, and the invariant is over
# the UNFINISHED robots: every unfinished robot arms iff some unfinished robot's
# own bit is set. A robot whose own read is BROKEN arms off that read without
# needing anyone's. A robot whose own read is COMPLETE has heard every
# unfinished peer first-hand within one TTL — that is what accountedPeerCount's
# first two channels mean — so it receives their bits itself. No relay, no
# second hop. (Contrast in_range_mask, whose closure is exactly two hops —
# A—B—C is covered, A—B—C—D is not; see team_model.cpp.)
#
# WHY "UNFINISHED" IS IN THE STATEMENT AND NOT A HEDGE. A finished peer counts
# accounted through channel 3 with no contact whatsoever, and that bit can
# arrive relayed, so COMPLETE does NOT imply "hears all N-1 directly". It costs
# nothing because a finished robot's bit is discarded at BOTH ends on purpose:
# the publisher forces team_incomplete=false once state_ == DONE, and
# peerReportsTeamBreak skips any peer with `finished` set. A robot nobody can
# hear and whose announcement nobody would read cannot be the robot whose bit
# goes unseen.
#
# THE ONE RESIDUAL, stated because it is the only way the invariant fails.
# accountedPeerCount accepts a peer on the CLAIM TABLE alone, while
# peerReportsTeamBreak reads the bit only off TeamWorld (`direct ||
# heard_one_way`). A peer whose intents arrive inside coord_claim_ttl_sec while
# its TeamWorld does not arrive inside direct_ttl_sec is therefore counted
# present and its bit is not read. Both are 5 s over the same emulated link and
# TeamWorld publishes on an unconditional 1 Hz timer, so this is a lost-packet
# window, not a topology — but it is a window, and it closes the moment one
# TeamWorld lands.
#
# THIS IS THE PRODUCER'S OWN READ, NOT ITS DERIVED ARMED STATE, AND THE
# DIFFERENCE IS THE WHOLE DESIGN. If a robot announced "I am armed", then C
# (armed only because it heard A) would announce armed, A would see C armed,
# and neither could ever clear: the arm would sustain itself off its own echo
# long after the link came back. That is generation 19's defect in a new dress
# — armed on one predicate, released on another, so the arm never stopped —
# and it cost ~25 spurious regroups per 3000 s cell at N>=3 while leaving N=2
# untouched, i.e. disguised as a team-size effect on exactly the comparison
# this campaign exists to make. Announcing the first-hand read instead makes
# the clear automatic: the moment A can hear everyone again it publishes false,
# and C, which has nothing of its own to arm on, follows one TeamWorld period
# later. The asymmetry is deliberate — A clears on its own read, C clears when
# no peer announces a break.
#
# PUBLISHED UNCONDITIONALLY, in all four arms, for the same reason my_tour is
# (see above): it is what THIS robot's peers need, and whether they act on it
# is their local mode's business. Only the rendezvous and hybrid arms read it.
bool team_incomplete

# THE PRODUCER IS STILL DRIVING TO THE AGREED CELL: it is in an appointment
# manoeuvre and has not stopped yet. Added in generation 23 to make the two
# halves of the appointment compose.
#
# WHAT IT FIXES. The barrier releases on a COMMS test — teamSettled, or since
# generation 27 full closure reachability (explo_planner_node.cpp,
# manoeuvreReleaseEligible; this bit and its one-hop report
# `appointment_inbound_seen` below gate both doors) — while the drive only
# ends on ARRIVAL. The radio reaches tens of metres, so a
# robot standing at the cell sees the team complete the moment its partner comes
# into range and leaves while that partner is still walking — and the partner
# may not release en route, because for an appointment the CELL is the agreed
# thing (explo_planner_node.cpp/doReturnNav). It therefore drives on to a cell
# nobody is in and waits out the unbounded barrier alone. The gen-23 smoke
# measured exactly that: one robot released after its 36 s settle, the other
# arrived 26 s later and stood there 272 s. Both halves behaved as designed;
# the pair did not.
#
# "STILL DRIVING", NOT "NOT YET ARRIVED", AND THE DIFFERENCE IS THE WHOLE SAFETY
# ARGUMENT. Every exit from the drive — arrival, the nav budget, the no-progress
# watchdog — lands in the barrier, and the appointment wait cap is unbounded
# (rendezvous_appointment_wait_sec = 0). A bit meaning "not yet at the cell"
# would hold the entire team forever on a robot whose destination turned out
# unreachable, which is the case those two watchdogs exist to survive and which
# doReturnNav calls routine in the hybrid arm. This bit clears when the DRIVING
# stops, however it stopped, so the hold it can add is bounded by
# nav_budget_sec.
#
# FIRST-HAND ONLY, NEVER RELAYED, like team_incomplete and for the same reason:
# it is non-monotonic, so a relayed copy could hold a barrier off an echo of
# itself. Its reader pairs it with `direct || heard_one_way` for the same reason
# too — the bit carries no TTL, and a robot that goes silent mid-drive must stop
# holding anyone. Its silence already holds the barrier through teamComplete,
# which is the stronger test and the one that can clear.
#
# NO FINISHED EXEMPTION, unlike team_incomplete, and the asymmetry is deliberate
# rather than an oversight to be tidied up. peerReportsTeamBreak must exempt a
# finished peer because that peer's bit never clears and would hang the
# unbounded barrier; this one always clears within nav_budget_sec, so there is
# no hang to prevent. Including finished robots is also the better behaviour: a
# robot that latched coverage on its way to the meeting is still coming, and
# exempting it would reproduce this very defect for exactly that robot.
#
# PUBLISHED UNCONDITIONALLY, in all four arms, like my_tour and team_incomplete.
# It can only be true inside an appointment manoeuvre, which only the rendezvous
# and hybrid arms ever start, so off and pursuit publish a constant false.
bool appointment_inbound

# A PEER THIS PRODUCER RECEIVES FIRST-HAND IS STILL DRIVING TO THE AGREED CELL:
# its one-hop report of the bit above. Added with the generation-27 closure
# door, and it exists because of that door. The door admits peers that are only
# REACHABLE — connected through a bridge robot, not heard directly — and such a
# peer's appointment_inbound cannot arrive at the releasing robot, because the
# bit above is never relayed. Without this report the set of peers the door
# newly admits is exactly the set the still-driving veto cannot see, and the
# gen-23 lopsided release returns through the door: the robot at the cell fills
# its reachable count while a third robot is still walking in behind the
# bridge, and it leaves. With it, the veto's horizon is the door's CLOSURE
# horizon — the closure expands one hop through a direct bridge
# (team_model.cpp/tick), and the bridge robot reports that same hop here. The
# door's other admission channel, `finished`, stays outside both vetoes' reach:
# it relays with no age gate, an exemption the count inherits from
# peerAccounted and one this report cannot close.
#
# DERIVED FROM RAW FIRST-HAND BITS ONLY: it is publishTeamWorld's copy of
# peerInboundToAppointment(), which reads peers' appointment_inbound and
# nothing else. It must never be derived from peers' copies of THIS field —
# A-says-seen-because-B-says-seen is the relay deadlock documented at
# team_incomplete above, arriving one field later, with nothing able to clear
# it. One hop, recomputed on every publish, so it clears within a publish
# period of the drive it reports ending.
#
# Its reader pairs it with `direct || heard_one_way` like the bit above and for
# the same reason: it carries no TTL of its own, and a reporter that goes
# silent must stop holding anyone.
#
# PUBLISHED UNCONDITIONALLY, in all four arms, like appointment_inbound: it can
# only be true while some first-hand peer is in an appointment drive, which
# only the rendezvous and hybrid arms ever start.
bool appointment_inbound_seen

# --- The rendezvous proposal (P5) -------------------------------------------
#
# The triple below IS the appointment, and it travels on the wire because it
# cannot be derived twice. That is measured, not defensive: across the banked
# ts3 n2+n3 cells, where each robot solved the schedule privately from its own
# snapshot, only 21 of 64 separated pairs picked the same cell and the median
# t_meet disagreement was 91 s at N=2 and 125 s at N=3 — at N=3, zero pairs
# agreed and every appointment stood in an empty cell for the full 240 s wait
# cap. Identical arithmetic is not enough when the inputs are private maps.
#
# THE PROTOCOL. The robot with the LOWEST fleet id derives a (cell, interval,
# t_meet) triple from its frozen snapshot and publishes it here. Everyone else
# adopts what that robot published, verbatim — no re-derivation, no local
# repair, no nearest-valid-cell — and echoes the same three numbers back. A
# robot commits the triple and drives the last one it committed. All three are
# integers so the comparison is exact bytes and not a float that has been
# through anybody's arithmetic.
#
# WHAT "COMMITS" MEANS DEPENDS ON THE GENERATION (2026-09-17). A PROVISIONAL
# triple commits only once every peer has been heard publishing it. A FINAL one
# commits on first-hand evidence from the proposer alone, because final is
# terminal and single-authored: every robot that receives it converges on it and
# never leaves, so the echoes only report who ELSE heard it. Requiring them
# split an N=3 fleet that was already unanimous — all three robots had adopted
# cell 73 within 1.6 s, and only the proposer committed it, because each
# FOLLOWER was waiting on the other follower's echo over a link that did not
# exist.
#
# DERIVED ONCE PER RUN, and only while the team is mutually complete. The
# proposer solves when it holds no triple yet and never again: a second
# generation would put some robots on the old cell and some on the new one at
# the moment the link drops, and would invalidate every confirmation latched
# against the first.
#
# THE TRIPLE DOES NOT GO STALE, and since generation 19 the reason is not that
# it recurs. It used to be: t_meet was a phase, interval was a period, and a
# separating robot kept the next occurrence. That is deleted. The agreed thing
# is now the CELL, which cannot expire, and the time is a local countdown from
# each robot's own detection — see rendezvous_t_meet_ms.
#
# WITH ONE BOUNDED EXCEPTION, and it has its own field: rendezvous_provisional.
# A triple derived before the allocator had any tour is a placeholder rather
# than a choice, and it may be replaced exactly once, by a triple that had
# something to choose between. Everything above still holds for the REPLACEMENT,
# which is final. Read that field before reasoning about the latches.
#
# There is no re-derivation during an outage either: the whole point of the
# exchange is that the triple predates the separation, and a robot that never
# committed one has NO appointment and must say so rather than invent a private
# one.
#
# WHAT THIS CANNOT DO. Two processes cannot agree at an arbitrary cut — if the
# link dies in the one message between "the proposer published its triple" and
# "every peer echoed it", the peers that echoed hold an appointment and the
# ones that did not hold none, and the ones that hold none manoeuvre
# unscheduled. That window is one TeamWorld period wide, it is open only during
# the single derive round near the start of the run, and it is the residual the
# design accepts rather than pretends away.

# Which cell to meet in. -1 means "no proposal" — and 0 is a valid cell id, so
# the sentinel is written explicitly and never left to zero-init.
int32 rendezvous_cell_id

# The derived interval, in milliseconds. -1 means "no proposal".
#
# NOTHING READS IT FOR TIMING (generation 19, 2026-09-17), and that is the most
# important sentence in this block. It was the RECURRENCE PERIOD of a standing
# meeting: a separating robot kept the next occurrence of "cell C, every P
# seconds from T". Before that it was a countdown each robot added to its own
# view of when the team stopped being whole. Both are gone.
#
# WHY THE SCHEDULE HAD TO GO. It needed three inequalities at once — worst-case
# drive <= period <= the barrier's wait cap (reconnect_midrun_max_wait_sec), and
# arming spread <= period — and no period satisfies them. The worst-case in-ROI
# drive is ~283 s (100x100 m at 0.5 m/s, 340 s with the safety markup) against a
# 240 s cap, and the reachability FLOOR beats the cap anyway
# (rendezvous_scheduler.cpp clamps to `max(max_interval_ms, floor_ms)`, because
# a period shorter than the drive is unkeepable rather than tighter). Nothing
# bounded the arming spread at all: the N=3 smoke armed three robots at
# 16.1 / 52.5 / 67.4 s — 1.7 periods — against a 30 s period, so they selected
# three different occurrences off byte-identical integers and the cell logged a
# no-show for the robot that arrived alone.
#
# IT IS STILL DERIVED, AGREED, ECHOED AND LOGGED, and it must still match byte
# for byte across the fleet, because it is part of the triple the campaign gate
# scores as evidence that the team agreed on something while it was whole. It
# also remains the honest statement of how long the furthest robot needs to
# reach the cell, which is worth having in the banked logs. It is a diagnostic.
# Do not reintroduce a timing consumer without re-reading the inequalities above.
#
# Milliseconds and int32 for the same reason every quantity in
# rendezvous_scheduler.hpp is integral: the value is produced once and echoed
# verbatim, and a float32 invites a consumer to do arithmetic on it and then
# compare for equality. int32 ms tops out at 24 days.
int32 rendezvous_interval_ms

# The derived meeting instant, in MISSION-ELAPSED milliseconds. -1 means "no
# proposal".
#
# LIKE THE INTERVAL, NOTHING READS IT FOR TIMING (generation 19). It has been
# three different things and is now the third:
#
#   1. Absent. Each robot computed `my_anchor + interval`, where the anchor was
#      the instant ITS view of the team stopped being mutually whole. That is
#      one physical event at N=2 — the ts4 smoke measured the two ends 0.84 s
#      apart — and is NOT one event at N>=3, where the graph comes apart edge by
#      edge and each robot sees a different edge go first. The ts4 N=3 cell
#      anchored its three robots at 13.18 / 34.18 / 33.98 s.
#   2. The phase of a recurring schedule. Published to remove the per-robot
#      term, which it did; it then failed the other way, because selecting an
#      OCCURRENCE reintroduced a per-robot term (see rendezvous_interval_ms).
#   3. A derived number that is agreed and logged and drives nothing.
#
# WHAT ACTUALLY TIMES THE MEETING NOW is a local countdown with a shared
# constant: a robot that has been missing a teammate for
# rendezvous_depart_delay_sec (100 s) drives to the agreed cell and waits there
# until the team is whole. There is no index to disagree about, so the detection
# spread that broke (1) and (2) becomes arrival spread instead, which the
# unbounded barrier wait absorbs by construction rather than by inequality.
#
# The agreement is therefore over the PLACE. That is a weaker agreement than the
# one this protocol was built for and it is stated plainly rather than dressed
# up: what makes it sufficient is that the barrier no longer gives up.
#
# MISSION-ELAPSED, NOT AN ABSOLUTE STAMP, keeping the reason the interval had:
# field clocks drift hours apart (the 2026-07-06 bunker/curt bags were 4531 s
# apart while recording simultaneously), and a meeting time that arrives as an
# absolute stamp is one a drifting clock silently relocates.
int32 rendezvous_t_meet_ms

# Whether the triple above is a PLACEHOLDER that may still be replaced once.
# False on a robot holding no triple at all, and false on every triple that is
# final.
#
# WHY THE FLEET NEEDS THIS ON THE WIRE AND NOT JUST IN THE PROPOSER'S HEAD. The
# proposer derives from a frozen snapshot, and the first instant the team is
# mutually whole is normally BEFORE the allocator has produced a single tour —
# at N=4 the ts4 smoke measured that window lasting about two seconds, starting
# at t+24.8 s. With nothing to insert a meeting cell into, the solve has exactly
# one admissible cell: the centroid of where the team is standing. That is a
# real, keepable appointment and it is published as one, which is what lets
# every N agree a place and a time beforehand instead of agreeing nothing.
#
# But it is a placeholder, not a decision — the argmin had nothing to choose
# between — so the proposer may replace it ONCE, with a triple that did have a
# choice, and only while the team is still mutually whole. `rendezvous_
# provisional` is how the followers can tell that replacement apart from the
# fault it would otherwise look exactly like (a restarted proposer, two
# campaigns on one bus), which they must refuse. A follower adopts an incoming
# triple over the one it holds if and only if it holds a provisional one and the
# incoming one is final. Provisional is never adopted over provisional: the
# centroid drifts as the robots move, so republishing it would start a new
# generation every few seconds and the team would never finish agreeing.
#
# THE TRANSITIONS THIS PERMITS, exhaustively, per robot, per run:
#   empty -> final                     (tours already existed)
#   empty -> provisional -> final      (the bootstrap case above)
#   empty -> provisional               (never upgraded; kept for the whole run)
# `final` is terminal and nothing downgrades. The confirmation latches behind
# the commit rule depend on exactly this: see explo_planner_node.cpp, where a
# latch is now dropped by any first-hand message showing a peer on a DIFFERENT
# triple, not only by one showing it on none.
bool rendezvous_provisional

# --- Third-party gossip (relayed) -------------------------------------------
# All three arrays are indexed by robot id and sized to the team. They carry
# what the producer knows about robots the RECEIVER may not be able to hear,
# which is what makes comms closure useful at N>=3: in an A-B-C chain, B is the
# only one who can tell A where C is.
#
# These are the reason closure is safe. Transitive closure says A's comms
# STATUS for C is IN_COMMS (so A dispatches no reconnection at C, correctly —
# B is bridging). But A's plan for C must use C's actual freshness, not A's
# status for C, or A would plan against a position that is minutes old while
# believing it is current. Every data consumer keys on these arrays; only the
# dispatch decision keys on status.

# Last known position of each robot, by id. The producer's own entry equals
# `position`. Entries for robots never heard of are zero and are meaningless
# unless the matching last-heard entry is non-negative.
geometry_msgs/Point[] robot_positions

# Mission-elapsed seconds (on the PRODUCER's mission clock) at which the
# producer last had DIRECT contact with each robot, by id. Negative means
# "never heard directly". The producer's own entry is its current mission
# time. A receiver ages these by the gap between the producer's mission clock
# and its own reception, which it can do because both are elapsed intervals.
float32[] robot_last_heard_sec

# `finished` (above) for each robot, by id, as best the producer knows it.
# The producer's own entry is its own bit. False means "no evidence", NOT
# "still exploring" — a robot nobody has heard from reads false everywhere.
#
# WHY THIS ONE FIELD IS RELAYED AND team_incomplete IS NOT. The two look alike
# and are not. `team_incomplete` is a NON-MONOTONIC claim a robot makes ABOUT
# THE TEAM, and relaying it is the self-sustaining echo this message's own
# header rules out: C would re-announce A's break as though it were C's read,
# A would see its own bit return, and neither could ever clear. `finished` is a
# MONOTONIC statement a robot makes ABOUT ITSELF. It never goes back to false
# (the publisher latches it, see explo_planner_node.cpp), it is never about
# anyone else, so a relayed copy cannot contradict a first-hand one, cannot
# arrive early, and cannot feed back. The merge rule is therefore a plain OR
# that never clears, and one hop is all it needs.
#
# WHAT IT FIXES, because the gap was real and it hung whole runs. `finished`
# means "no longer someone worth reconnecting to": peerReportsTeamBreak()
# exempts a finished peer, and the appointment barrier is UNBOUNDED
# (rendezvous_appointment_wait_sec = 0). But the exemption was applied by the
# LISTENER to peers the LISTENER had heard finish, while the robot announcing
# the break is by construction the one that CANNOT hear the finished robot. At
# N>=3: A finishes out of contact with B (they were in contact at spawn, so B
# holds a stale finished=false and no TTL ever clears it); B therefore reads
# the team incomplete forever and says so; C hears both, exempts A, but is held
# by B's complaint, so teamSettled(C) is false forever. Both B and C drive to
# the agreed cell and wait at a barrier neither can release — B because it
# genuinely cannot account A, C because B keeps announcing. Neither is latched,
# so the latched-hold cap does not apply and rendezvousWaitExpired() is false
# for an unbounded cap: the cell runs to the harness duration cap with two of
# three robots standing still. Relaying the bit closes it at the source — B
# learns A is done, stops counting it as missing, stops announcing, and both
# barriers release on the ordinary rule.
#
# The same relay also stops B assigning cells to a robot that will never drive
# them (global_allocator.cpp) and stops it dispatching reconnects at a parked
# robot (reconnect_gate.cpp); both already skip `finished` and both were
# reading the stale first-hand copy.
#
# NOT AGE-GATED, unlike the two arrays above, and that is deliberate: a
# finished robot stops moving and goes quiet, so its last-heard entry ages out
# of gossip_max_age_sec exactly when this bit matters most. A monotonic fact
# has no freshness to check.
bool[] robot_finished
