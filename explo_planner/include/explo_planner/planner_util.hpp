#pragma once
/// @file planner_util.hpp
/// @brief Small pure helpers shared by the exploration planner node.
/// Moved comments: doc/planner_util_notes.md

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace explo_planner {

/// Map a planner_type string to the diagnostic enum used in RobotIntent:
/// 0=eig, 1=entropy, 2=frontier, 3=random, 4=ssmi, 255=unknown.
uint8_t plannerTypeId(const std::string& planner_type);

/// Concatenate `parts` with `delim` between consecutive elements. An empty
/// vector yields an empty string and a single element yields itself, so the
/// delimiter never appears at either end.
std::string join(const std::vector<std::string>& parts, const std::string& delim);

/// NAVIGATE timeout: dist / max(speed_est, 1e-3) * safety, clamped to [min_sec,
/// max_sec]. If max_sec < min_sec the ceiling wins; a NaN budget becomes
/// max_sec, so the watchdog always fires. (notes: util-nav-budget-guarantees)
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

/// At exploration exhaustion, true = run a reconnect manoeuvre: only with
/// reconnect_enabled, an anchor, and the team not complete. reconnect_enabled
/// is the master switch; reconnect_mode picks the manoeuvre.
/// (notes: util-should-rendezvous)
bool shouldRendezvous(bool reconnect_enabled, bool have_anchor,
                      int active_peers, int expected_peers);

/// Barrier give-up test: with `max_wait_sec` <= 0 the robot waits forever
/// (always false); otherwise true once `waited_sec` >= `max_wait_sec`.
bool rendezvousWaitExpired(double waited_sec, double max_wait_sec);

// --- Mesh reconnection (robot-carried radios; see explo_planner_node.cpp) ---
// With the radios on the robots instead of a fixed router, "where I last heard
// you" is a PAIR of poses (mine and the peer's advertised one), both stale the
// moment the link drops. Three reconnection policies are built on that record.

/// OFF is reconnect_enabled=false. RENDEZVOUS: keep the team's agreed recurring
/// (cell, interval, t_meet) appointment. PURSUIT: chase the trail head on a
/// budget. HYBRID: exactly their union, nothing else.
/// (notes: reconnect-mode-arms)
enum class ReconnectMode { RENDEZVOUS, PURSUIT, HYBRID };

/// First t_meet_ms + k*interval_ms (k >= 0) at or after not_before_sec;
/// interval_ms <= 0 returns t_meet_ms even if past. Any term a caller adds to
/// the floor must clamp at zero, or a team that can all attend forks.
/// (notes: util-next-agreed-occurrence)
long long nextAgreedOccurrence(long long t_meet_ms, long long interval_ms,
                               double not_before_sec);

/// Seconds to raise the occurrence floor so this robot is at most
/// max_lateness_sec late, given its marked-up lead_ms. Exactly 0.0 when lead_ms
/// < 0 or within budget; a bad max_lateness_sec counts as 0.
/// (notes: util-arrival-shortfall)
double arrivalShortfallSec(long long lead_ms, double max_lateness_sec);

/// True once eligible has held continuously for confirm_sec; the first eligible
/// tick arms and returns false, false disarms, confirm_sec <= 0 is true. Per
/// window one site calls this, ticked unconditionally.
/// (notes: util-dwell-confirmed)
bool dwellConfirmed(bool eligible, double now_sec, double confirm_sec,
                    bool* armed, double* since_sec);

/// Non-mutating twin of dwellConfirmed, for reading another site's window. Must
/// stay pure and match dwellConfirmed's return expression; change both
/// together. (notes: util-dwell-held)
bool dwellHeld(bool eligible, double now_sec, double confirm_sec, bool armed,
               double since_sec);

/// Case-insensitive parse of reconnect_mode. Unknown strings return RENDEZVOUS
/// with known set false, which the node treats as fatal.
/// (notes: util-reconnect-mode-parse)
ReconnectMode reconnectModeFromString(const std::string& s,
                                      bool* known = nullptr);

/// Stable mode name, the arm label stamped into every run's event log; renaming
/// one re-buckets past runs. Round-trips through reconnectModeFromString.
/// (notes: util-reconnect-mode-name)
const char* reconnectModeName(ReconnectMode m);

/// Pursuit spend limit, s; 0 = do not pursue. navBudgetSec shape scaled by
/// freshness, linear to 0 at staleness_max_sec (<= 0 disables that). max_sec <=
/// 0 disables pursuit; max_sec < min_sec: ceiling wins.
/// (notes: util-pursuit-budget)
double pursuitBudgetSec(double trail_head_dist_m, double staleness_sec,
                        double speed_est_mps, double safety_factor,
                        double staleness_max_sec, double min_sec,
                        double max_sec);

/// May a peer's latched position still hold cells? max_age_sec <= 0 is
/// unbounded (true); a negative age_sec (no position) is never fresh under a
/// bound; else fresh iff age_sec <= max_age_sec.
/// (notes: util-alloc-peer-position-ttl)
bool allocPeerPositionFresh(double age_sec, double max_age_sec);

/// Midpoint of the last-contact pose pair. Not called by the planner; kept for
/// reconstructing banked runs. Do not reintroduce a call: it is a place with no
/// time, and the two ends do not agree on it.
/// (notes: util-meeting-point-retired)
Eigen::Vector3f meetingPoint(const Eigen::Vector3f& self_at_contact,
                             const Eigen::Vector3f& peer_at_contact);

} // namespace explo_planner
