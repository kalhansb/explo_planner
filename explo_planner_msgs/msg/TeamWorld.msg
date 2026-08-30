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
#   observed hours apart. Freshness is measured from LOCAL receipt, and the two
#   scheduling quantities on this message (rendezvous_time_sec,
#   robot_last_heard_sec) are MISSION-ELAPSED seconds, not absolute stamps.
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
uint16[] my_tour

# The producer has latched exploration done on its own map. A finished robot
# leaves the allocator's vehicle set — its cells return to the pool rather
# than being assigned to a robot that will never drive them — and the
# knowledge gate stops counting it as someone worth reconnecting to.
bool finished

# The producer's current rendezvous proposal: which cell to meet in, and when.
# -1 means "no proposal". Everyone adopts the lowest-robot_id live proposal and
# echoes it; agreement is reached when all live peers echo the same pair.
int32 rendezvous_cell_id

# Meeting time in MISSION-ELAPSED seconds since the producer's run_start — NOT
# an absolute stamp, and not comparable as one. Every robot counts down against
# its own mission clock, which is the same reasoning that makes claim TTLs run
# from local receipt: two field clocks hours apart would otherwise turn an
# agreed meeting into an immediate dispatch on one robot and a never-fires on
# the other.
float32 rendezvous_time_sec

# --- Third-party gossip (relayed) -------------------------------------------
# Both arrays are indexed by robot id and sized to the team. They carry what
# the producer knows about robots the RECEIVER may not be able to hear, which
# is what makes comms closure useful at N>=3: in an A-B-C chain, B is the only
# one who can tell A where C is.
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
