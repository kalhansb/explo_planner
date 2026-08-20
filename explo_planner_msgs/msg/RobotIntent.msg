# Multi-robot exploration claim. Each planner publishes its currently
# selected viewpoint so peers can avoid contesting the same hotspot.
# Used by Coordination::onIntent / claimMatching / selfWinsAgainst
# (see explo_planner/coordination.hpp).
#
# header.stamp = producer's clock at publish (diagnostic only). It is NOT
#   used for claim expiry: consumers time the TTL from LOCAL receipt, so
#   unsynchronised fleet clocks cannot corrupt deconfliction. See ttl_sec.
# header.frame_id = the planning map frame (e.g. "map")
std_msgs/Header header

# Producer robot id (e.g. "atlas"). Used by peers to drop self-messages
# and as the lex tiebreak key in MinPos when straight-line distances tie.
string robot_id

# Claimed viewpoint position (XY in the map frame; z = candidate robot z).
geometry_msgs/Point goal_pos

# Planned final yaw at goal_pos (radians).
float32 yaw

# Producer's current pose at the time of the claim. MinPos compares
# (peer.robot_pos -> candidate) against (self.pos -> candidate); we ship
# the producer's pose rather than a precomputed cost so peers can recompute
# the comparison locally with their own metric.
geometry_msgs/Point robot_pos

# Disc radius (m) around goal_pos that the producer reserves. Peers treat
# any candidate within this radius as "contested by the producer".
float32 claim_radius_m

# Time-to-live for this claim (seconds). Peers drop the claim from their
# local table once ttl_sec has elapsed since THEY RECEIVED it (expiry is
# built from the consumer's local clock, NOT header.stamp), so a stalled or
# crashed peer eventually releases its target. Building expiry from
# header.stamp would compare two unsynchronised fleet clocks and silently
# break: a peer ahead makes claims immortal, a peer behind by > ttl makes
# them expire on arrival. The semantic is "how long since we last HEARD
# this peer", which needs only the local clock.
float32 ttl_sec

# Diagnostic: which planner produced this claim.
# 0 = eig, 1 = entropy, 2 = frontier, 3 = random
uint8 planner_type

# --- Exploitation coupling (team-quota vantage sharing) ---
# When exploit is true this claim is an exploit hop (vantage or approach) on
# tree target `target_id`, and goal_pos is the claimed vantage — peers doing
# the same target use MinPos with a small per-vantage radius to take a
# different angle. exploit=false (exploration claims) leaves target_id and
# dwelled_mask zero; target_id alone is not a discriminator because a
# detector may legitimately emit id 0.
bool exploit
uint32 target_id

# Bitmask of vantage indices on target_id that the producer has dwelled at
# with a clear line of sight. The vantage ring is deterministic from the
# trunk (center, radius), so index k names the same pose on every robot.
# Peers OR this into their local target state: the team's union of clear-LoS
# dwells counts toward min_vantages_required (team quota), so the ring is
# captured once, cooperatively. Cumulative-state (not per-event) so a lost
# message never loses credit. Supports n_vantages <= 32.
uint32 dwelled_mask

# Meaningful only when exploit is true. True means the producer is physically
# standing on the vantage in goal_pos, in its dwell state: the flag is set at
# EXPLOIT_DWELL entry, i.e. after nav arrival AND the post-arrival rotation
# settle. False on every claim published while the producer is driving (vantage
# hop, approach hop) and on all exploration claims.
#
# The producer declares this because a consumer cannot infer it. Peers used to
# derive staging from geometry — XY distance from this claim's robot_pos to its
# own goal_pos within a tolerance — and that inference cannot separate "on the
# vantage" from "arrived in XY but still rotating beside it", which read as
# staged ~5 s early and started the team's dwells out of step, nor from
# "standing at an approach waypoint", an intermediate exploit hop whose goal is
# the waypoint and not a vantage at all. Peers use this flag as the
# rendezvous-barrier staging signal, so the team's dwells start together.
bool staged

# --- Map-size beacon (info-gated reconnect trigger) ---
# The producer's current fused-map size (observed voxels) and its recent
# growth rate (voxels per SIM second, winsorised slope over a ~300 s window;
# kRateWindowSec in explo_planner_node.cpp). Consumers snapshot both into
# their per-peer last-contact record; after a link loss each side
# dead-reckons the pair's unshared-map backlog from the SAME snapshot, so the
# mid-run reconnect trigger time derived from it agrees across the pair
# without any in-outage communication. Values are sampled from the metrics
# tick (~metrics period resolution): coarse, but the trigger consumer clamps
# to [min,max] silence bounds so quantization cannot fire it degenerately.
#
# Wire-compatibility warning: these fields do NOT degrade gracefully across
# mixed builds. On Humble (no XTypes type-negotiation) a consumer built WITH
# them cannot deserialise messages from a producer built WITHOUT them — the
# CDR buffer underflows, rmw drops the message before user code sees it, and
# the producer goes silently, permanently invisible to that consumer (claims,
# heartbeats, everything — not just these fields). Rebuild every robot's
# workspace when this file changes. Zero-as-"unknown" (consumers treat a zero
# rate as unknown and fall back to the time-only trigger) only covers a
# SAME-BUILD producer that has not sampled its first metrics tick yet.
uint64 observed_voxels
float32 map_growth_rate
