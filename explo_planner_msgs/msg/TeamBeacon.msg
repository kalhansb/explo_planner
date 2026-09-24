# The gen-34 team beacon: everything one robot tells the others, once a second,
# in every state including DONE. See DESIGN_gen34.md §8.2.
#
# ONE MESSAGE, ALWAYS SENT. The beacon never goes quiet while the node runs, so
# "not heard for 10 s" means the radio, not the sender. Presence, contact,
# exchange completion, the meeting plan and "met" are all read from it; nothing
# else in the gen-34 node decides whether a peer is there.
#
# DIRECT ONLY FOR PRESENCE. A beacon is relayed by nobody: it arrives on
# /<me>/rx/<sender>/exploration/team_beacon straight from the emulator link
# between the two robots. Plan fields may still spread robot to robot (each
# robot re-publishes the highest plan it holds, and the plan_known_* table of
# what it knows the others hold), but that is plan data, not presence.
#
# TeamWorld is unchanged and stays the gen-33 node's message.

# header.stamp: the sender's clock at publish (shared sim clock in simulation).
std_msgs/Header header

# ── Identity ─────────────────────────────────────────────────────────────────
# Fleet id: index into team_robot_names.
uint8 robot_id
# teamNamesHash(team_robot_names): a beacon with a different hash is from a
# robot configured with a different fleet and is ignored entirely.
uint32 team_hash
# CellGrid::configHash(): cell ids in cells[], my_tour and plan_cell name the
# same ground only when this matches. A mismatch ignores those fields.
uint32 grid_hash

# ── Where and what ───────────────────────────────────────────────────────────
geometry_msgs/Point position
# The sender's current drive goal, if any (the chase trail's second point).
geometry_msgs/Point goal
bool has_goal
# Bit j set: the sender has heard robot j's beacon directly within the presence
# window. Mutual contact needs both robots to name each other.
uint32 hears_mask

# Activity, one of:
uint8 ACT_EXPLORE = 0
uint8 ACT_MEET    = 1
uint8 ACT_CHASE   = 2
uint8 ACT_FOLLOW  = 3
uint8 ACT_WAIT    = 4
uint8 ACT_HOME    = 5
uint8 ACT_DONE    = 6
uint8 activity
# Exploration finished (coverage latch, step budget or PLAN starvation). Latched;
# never clears.
bool finished
# Every robot finished and known to be: set once heard directly from all with
# finished, or copied from any beacon that carries it. Never clears.
bool team_finished
# Arrived at the start pose (or homing gave up and the robot parked).
bool home

# ── Map exchange (sequence numbers from scovox) ─────────────────────────────
# Newest ScovoxMapBinary.seq this robot has published: its own dscovox's newest
# for itself.
uint64 map_sent_seq
# Indexed by fleet id: newest seq this robot's dscovox has received from robot
# j. Entry [robot_id] equals map_sent_seq. Length = team size.
uint64[] map_rcvd_seq

# Bit j set: the exchange with j in the current contact is done (both sides
# received up to what the other had sent when the contact began). Lets a robot
# judge a pair it is not part of, for the encounter rule.
uint32 exchanged_mask

# ── The meeting plan (rendezvous and hybrid; version 0 = none) ──────────────
uint32 plan_version
int32 plan_cell
geometry_msgs/Point plan_center
# Slot k is at plan_t0_sec + k * plan_interval_sec on the shared clock.
float64 plan_t0_sec
float64 plan_interval_sec
# The slot at whose full meeting this version was issued; -1 for version 1. A
# re-solve that firms a provisional plan carries its predecessor's value.
int32 plan_renewed_at_slot
bool plan_provisional

# ── What this robot knows the others hold (split recovery, DESIGN_gen34 §11.1)
# Indexed by fleet id, length = team size, all three or none: the highest plan
# version this robot knows robot j to hold, and that plan's cell and centre.
# 0 = nothing known. Entry [robot_id] is the sender's own plan. A receiver
# takes the sender's own plan from the plan_* fields above, never from here,
# and never takes an entry for itself. Versions only rise, so an entry is a
# lower bound on what j holds.
uint32[] plan_known_version
int32[] plan_known_cell
geometry_msgs/Point[] plan_known_center

# ── Meeting attendance ──────────────────────────────────────────────────────
# The slot of the booking this robot has departed for, -1 if none.
int32 meeting_slot
# Bit j set: this robot has completed a direct exchange with j during
# meeting_slot.
uint32 met_mask

# ── Cell world, same content as TeamWorld's ─────────────────────────────────
CellState[] cells
uint16[] my_tour
