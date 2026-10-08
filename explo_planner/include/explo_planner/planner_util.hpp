#pragma once
/// @file planner_util.hpp
/// @brief Small pure helpers shared by the exploration planner node.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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
// A robot that exhausts its exploration goals returns to the mission start
// point ("home") and holds there until the team is back in comms. These three
// pure predicates encode the decisions; the node supplies the live counts.

/// True when the whole team is currently in comms range: `expected` > 0 and at
/// least `expected` unexpired peer claims are held. Doubles as the barrier
/// release test and the "safe to declare DONE" test.
bool teamComplete(int active_peers, int expected_peers);

/// Decision at exploration exhaustion: return home and wait for the
/// team (true), or finish now (false). Returns true only when the feature is
/// on, the start point has been recorded, and the team is NOT already complete
/// (if it is, we are synced and can finish straight away).
bool shouldRendezvous(bool rendezvous_enabled, bool have_home,
                      int active_peers, int expected_peers);

/// Barrier give-up test: with `max_wait_sec` <= 0 the robot waits forever
/// (always false); otherwise true once `waited_sec` >= `max_wait_sec`.
bool rendezvousWaitExpired(double waited_sec, double max_wait_sec);

// --- nav2 result handling (see onNavStatus in explo_planner_node.cpp) ---

/// Consecutive nav2 aborts of the planner's goal with the robot (nearly)
/// standing still. An abort within `still_m` (XY) of where the streak's first
/// abort happened extends the streak; one farther away starts a new streak of
/// 1 there, because the robot got somewhere in between. The planner keeps
/// re-sending its goal, so nav2 giving up once is not final; giving up again
/// from the same spot is.
class AbortStreak {
public:
  /// Record an abort with the robot at (x, y); returns the new streak length.
  int onAbort(float x, float y, float still_m);
  void reset() { count_ = 0; }
  int count() const { return count_; }
  /// Move the anchor with a localisation jump so the jump is not read as the
  /// robot driving somewhere.
  void shiftAnchor(float dx, float dy) {
    ax_ += dx;
    ay_ += dy;
  }

private:
  int count_ = 0;
  float ax_ = 0.0f;
  float ay_ = 0.0f;
};

/// Where nav2 ran recovery behaviours during one goal, so an abort can be
/// asked "had nav2 already tried its recoveries here?". Keeps the last
/// kMaxSpots records (a wedged robot's full give-up on nav2's default tree is
/// ~15-20 recoveries).
class RecoveryLog {
public:
  /// Record `n` recoveries with the robot at (x, y).
  void add(float x, float y, int n = 1);
  /// Recoveries recorded within `radius_m` (XY) of (x, y).
  int countNear(float x, float y, float radius_m) const;
  void reset() { spots_.clear(); }
  /// Move every record with a localisation jump, as AbortStreak does.
  void shiftAll(float dx, float dy);

private:
  struct Spot {
    float x, y;
    int n;
  };
  static constexpr std::size_t kMaxSpots = 64;
  std::vector<Spot> spots_;
};

/// Recoveries new in one nav2 feedback message. bt_navigator counts
/// number_of_recoveries per goal from 0, and every keep-alive re-send is a
/// new goal id (a preemption, which restarts the count at 0), so a new id's
/// whole count is new. The exception is the first message this node sees: its
/// goal may predate the node, so its count only sets the baseline.
/// `prev_count` is the count last seen for the same goal id.
int newRecoveries(bool first_message, bool same_goal, int prev_count,
                  int count);

/// What an ABORT of the planner's goal means for that goal.
enum class Nav2GiveUp {
  kRetry,          ///< not final: the keep-alive re-send gets another try
  kAbortLimit,     ///< `abort_limit` aborts in a row from one spot
  kAfterRecovery,  ///< nav2 had already run its recoveries where it aborted
};

/// The decision for an ABORT just recorded: `streak` is the AbortStreak
/// length, `recoveries_here` the RecoveryLog count near the abort.
/// `abort_limit` <= 0 means aborts are only logged (always kRetry).
Nav2GiveUp nav2GiveUp(int abort_limit, int streak, bool final_after_recovery,
                      int recoveries_here, int recoveries_needed);

/// Name of a nav2 error code as forwarded in a NavigateToPose result (Jazzy
/// and later): the FollowPath (1xx), ComputePathToPose (2xx), SmoothPath (5xx)
/// and behaviour (7xx) codes. "" for 0 and for codes it does not know.
const char* nav2ErrorName(int code);

} // namespace explo_planner
