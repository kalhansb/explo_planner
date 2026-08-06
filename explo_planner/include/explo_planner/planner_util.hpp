#pragma once
/// @file planner_util.hpp
/// @brief Small pure helpers shared by the exploration planner node.

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace explo_planner {

/// Map a planner_type string to the diagnostic enum used in RobotIntent:
/// 0=eig, 1=entropy, 2=frontier, 3=random, 4=ssmi, 255=unknown.
uint8_t plannerTypeId(const std::string& planner_type);

/// Distance-budgeted NAVIGATE timeout (seconds):
///   budget = clamp(dist / max(speed_est, 1e-3) * safety, min_sec, max_sec)
/// so a short hop gets a small budget and a long hop a larger one, instead of
/// every goal sharing one fixed timeout.
double navBudgetSec(double dist_m, double speed_est_mps, double safety_factor,
                    double min_sec, double max_sec);

// --- Rendezvous barrier (see explo_planner_node.cpp) ---
// A robot that exhausts its exploration goals returns to its last-connected
// anchor and holds there until the whole team is back in comms. These three
// pure predicates encode the decisions; the node supplies the live counts.

/// True when the whole team is currently in comms range: `expected` > 0 and at
/// least `expected` unexpired peer claims are held. Doubles as the barrier
/// release test and the "safe to declare DONE" test.
bool teamComplete(int active_peers, int expected_peers);

/// Decision at exploration exhaustion: return to the anchor and wait for the
/// team (true), or finish now (false). Returns true only when the feature is
/// on, a comms anchor has been recorded, and the team is NOT already complete
/// (if it is, we are synced and can finish straight away).
bool shouldRendezvous(bool rendezvous_enabled, bool have_anchor,
                      int active_peers, int expected_peers);

/// Barrier give-up test: with `max_wait_sec` <= 0 the robot waits forever
/// (always false); otherwise true once `waited_sec` >= `max_wait_sec`.
bool rendezvousWaitExpired(double waited_sec, double max_wait_sec);

// --- Mesh reconnection (robot-carried radios; see explo_planner_node.cpp) ---
// With the radios on the robots instead of a fixed router, "where I last heard
// you" is a PAIR of poses (mine and the peer's advertised one), both stale the
// moment the link drops. Three reconnection policies are built on that record.

/// Reconnection policy at exploration exhaustion with a teammate out of comms.
///   RENDEZVOUS: drive to the own-pose anchor and wait (the pre-mesh
///     behaviour; with every robot doing this the pair distance at arrival
///     equals the distance at last contact, i.e. within comms range).
///   PURSUIT: chase the peer's last declared goal (the trail head) on a
///     budget; when the budget is spent, hold in place and beacon.
///   HYBRID: pursue on the budget, then fall back to the deterministic
///     meeting point (both sides compute the same one) and wait there.
enum class ReconnectMode { RENDEZVOUS, PURSUIT, HYBRID };

/// Parse the `reconnect_mode` parameter. Unknown strings map to RENDEZVOUS
/// (the legacy behaviour); the caller warns on the mismatch.
ReconnectMode reconnectModeFromString(const std::string& s);

/// Pursuit spend limit (seconds). 0 means "do not pursue" — the caller falls
/// straight through to its fallback. Non-zero budgets follow the navBudgetSec
/// shape (distance to the trail head at the conservative speed estimate,
/// clamped to [min_sec, max_sec]) scaled by the freshness of the last-contact
/// record: trust in the trail head decays linearly with `staleness_sec` and
/// hits zero at `staleness_max_sec` — by then the peer could be anywhere in
/// the plot and chasing the record is worse than the guaranteed fallback.
///   - max_sec <= 0 disables pursuit outright (always 0).
///   - staleness_max_sec <= 0 disables the staleness gate (freshness = 1).
double pursuitBudgetSec(double trail_head_dist_m, double staleness_sec,
                        double speed_est_mps, double safety_factor,
                        double staleness_max_sec, double min_sec,
                        double max_sec);

/// Deterministic meeting point for the HYBRID fallback: the midpoint of the
/// last-contact pose pair. Each side computes it from its OWN record — the
/// records differ by at most one heartbeat of travel, so the two midpoints
/// land within a couple of metres of each other, and each robot standing at
/// its own midpoint puts the pair well inside the range they had at last
/// contact. No message is exchanged; determinism substitutes for negotiation
/// exactly as in the MinPos tiebreak.
Eigen::Vector3f meetingPoint(const Eigen::Vector3f& self_at_contact,
                             const Eigen::Vector3f& peer_at_contact);

} // namespace explo_planner
