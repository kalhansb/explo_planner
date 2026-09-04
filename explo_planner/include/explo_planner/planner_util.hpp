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

/// Decision at exploration exhaustion: run a reconnect manoeuvre (true), or
/// finish now (false). Returns true only when the feature is on, a comms
/// anchor has been recorded, and the team is NOT already complete (if it is,
/// we are synced and can finish straight away).
///
/// `reconnect_enabled` is the subsystem master switch, not the rendezvous arm:
/// which manoeuvre a true answer dispatches is reconnect_mode's business, and
/// the caller is dispatchReconnect. The name says "Rendezvous" for history
/// only, from before pursuit and hybrid existed.
bool shouldRendezvous(bool reconnect_enabled, bool have_anchor,
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

/// Parse the `reconnect_mode` parameter, case-insensitively. Unknown strings
/// map to RENDEZVOUS (the legacy behaviour); `known`, when non-null, receives
/// whether the string matched a mode, so the caller can warn on the mismatch
/// without duplicating the accepted-string set.
ReconnectMode reconnectModeFromString(const std::string& s,
                                      bool* known = nullptr);

/// Stable, machine-readable mode name — the inverse of
/// reconnectModeFromString, and the arm label the experiment event log stamps
/// into every run. Treat it as an interface exactly like stateName(): these
/// strings are what an analysis groups runs by, so renaming one silently
/// re-buckets every past run. Round-trips through reconnectModeFromString.
const char* reconnectModeName(ReconnectMode m);

/// Pursuit spend limit (seconds). 0 means "do not pursue" — the caller falls
/// straight through to its fallback. Non-zero budgets follow the navBudgetSec
/// shape (distance to the trail head at the conservative speed estimate,
/// clamped to [min_sec, max_sec]) scaled by the freshness of the last-contact
/// record: trust in the trail head decays linearly with `staleness_sec` and
/// hits zero at `staleness_max_sec` — by then the peer could be anywhere in
/// the plot and chasing the record is worse than the guaranteed fallback.
/// Note the clamp: at field-scale distances the raw distance term saturates
/// the ceiling (flatforest params put that at a ~12 m trail head), so in
/// practice the returned budget IS max_sec until staleness eats into it, and
/// it never lands in (0, min(min_sec, max_sec)).
///   - max_sec <= 0 disables pursuit outright (always 0).
///   - staleness_max_sec <= 0 disables the staleness gate (freshness = 1).
///   - max_sec < min_sec: the ceiling wins (the result never exceeds
///     max_sec — it is the bound a waiting teammate relies on; min_sec comes
///     from the unrelated nav-timeout family and must not override it).
double pursuitBudgetSec(double trail_head_dist_m, double staleness_sec,
                        double speed_est_mps, double safety_factor,
                        double staleness_max_sec, double min_sec,
                        double max_sec);

/// May a peer's latched POSITION still hold cells in the allocation problem?
///
/// `age_sec` is what TeamModel::positionAgeSec() returned — seconds since the
/// pose we hold was measured, or NEGATIVE for "we hold no position". Both
/// arguments are on the mission clock.
///
/// Three rules, and the first two are the ones that make the default safe:
///   - `max_age_sec <= 0` is UNBOUNDED and returns true for everything. This
///     is the file-wide convention for "no limit" (see pursuitBudgetSec) and
///     it is what makes `alloc_peer_pos_max_age_sec = 0` reproduce the
///     pre-TTL planner bit-for-bit, so one binary can run both arms.
///   - A negative `age_sec` is never fresh under a live bound. It means no
///     position exists, and a robot with no position is dropped from the
///     problem anyway — but returning true here would make "unknown" read as
///     "recent" the moment a caller forgot the have-position test.
///   - Otherwise fresh means `age_sec <= max_age_sec`, inclusive, so a TTL of
///     exactly N seconds admits a pose measured N seconds ago.
///
/// NaN in either argument returns false (every comparison against NaN is
/// false, and the `<= 0` unbounded test fails too), which drops every peer
/// rather than admitting every peer. That is the safe direction for a
/// misconfiguration, and the node refuses a non-finite parameter at load
/// anyway so it should be unreachable from the campaign harness.
bool allocPeerPositionFresh(double age_sec, double max_age_sec);

/// Meeting point for the HYBRID fallback: the midpoint of the last-contact
/// pose pair (all three axes — on flat worlds the z average is the shared
/// ground height; the arrival test is xy-only either way). Each side computes
/// it from its OWN record, so no message is exchanged; determinism
/// substitutes for negotiation exactly as in the MinPos tiebreak. The two
/// records — and so the two midpoints — agree only as closely as the two
/// directions' last successful receptions were simultaneous: intent traffic
/// is state-gated, so a one-way fly-by can refresh one side's record and not
/// the other's. The barrier releasing on COMMS (not co-location) is what
/// absorbs the ordinary asymmetry; see planner_method.md for the failure
/// mode when it can't.
Eigen::Vector3f meetingPoint(const Eigen::Vector3f& self_at_contact,
                             const Eigen::Vector3f& peer_at_contact);

} // namespace explo_planner
