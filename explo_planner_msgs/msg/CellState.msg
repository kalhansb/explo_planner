# One coarse cell of the shared global world model, as it travels on the wire.
# Produced and consumed by explo_planner's cell_world unit; see
# explo_planner/cell_world.hpp for the local (richer) representation and
# docs/mtare_evolution_plan.md §3.1-§3.2 for the design.
#
# This is the WIRE form, and it is deliberately narrower than the local form.
# The local status machine carries five values; only three of them are
# transmissible (see `status` below). Keeping the two vocabularies separate is
# load-bearing, not tidiness: a relayed `*_BY_OTHERS` is uninterpretable at the
# far end (whose "others"?), and a fleet that relays it cannot satisfy the
# solve-same-take-own premise the allocator rests on.

# Cell index in the sender's cell grid. Row-major over the ROI at
# `cell_size_m`; every robot derives the same grid from the same ROI and cell
# size params, so an id names the same ground on every robot. It is NOT a
# handle into a sender-local table: a mismatched ROI or cell size across the
# fleet silently re-points every id, which is why TeamWorld carries the
# team_hash config check alongside.
uint16 id

# Normalised exploration status. These three values are the ENTIRE wire
# vocabulary; the local-only COVERED_BY_OTHERS / EXPLORING_BY_OTHERS are
# normalised down to COVERED / EXPLORING before send.
uint8 UNSEEN=0
uint8 EXPLORING=1
uint8 COVERED=2
uint8 status

# Bitmask of robot ids (bit k = index k in `team_robot_names`) that the sender
# believes have seen this cell IN ITS CURRENT STATUS. The qualifier is the
# whole point: on every committed status change the producer resets this to
# {self}, because a change invalidates what peers knew about the cell. Without
# that reset the mask only ever grows, and the knowledge gate that asks "does
# my missing peer already know everything I could tell it?" answers "yes"
# forever for any cell the peer ever heard of — a gate that cannot fire.
#
# Consequently this must be OR-merged only between AGREEING statuses. ORing
# across a status disagreement would credit a peer with knowledge of a status
# it has never seen.
uint32 known_by

# The producer's local count of committed status changes it observed FIRST-HAND
# on this cell. Incremented only on self-observation; the merge path never
# touches it.
#
# It is therefore NOT comparable across robots and must never be used as one.
# Two robots' counters advance on different events at different rates, so
# "higher update_id wins" is not an ordering of information quality: a robot
# out of comms for ten minutes legitimately re-commits EXPLORING at a high
# count against a peer's first-hand COVERED, and higher-wins would regress the
# team's census. Conflicts are resolved by the guard table plus a
# more-explored ordering instead (COVERED > EXPLORING > UNSEEN). This field is
# carried for diagnostics and for a producer's own bookkeeping only.
uint32 update_id
