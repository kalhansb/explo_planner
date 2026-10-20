#pragma once
/// @file failed_goal_blacklist.hpp
/// @brief TTL + radius blacklist of recently-failed goal positions.

#include <cstddef>
#include <deque>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace explo_planner {

/// Records goal positions the robot failed to reach (navigate timeout / no
/// progress) so the planner can skip re-picking the same unreachable target
/// while it's still "hot". Without this, a physically stuck robot keeps the
/// same pose -> same scores -> re-picks the same goal forever.
///
/// Time is passed in as seconds (the caller's clock) rather than read
/// internally, so the prune/expiry logic is deterministic and unit-testable
/// without a ROS clock. Entries are stored in insertion (time) order.
class FailedGoalBlacklist {
public:
  /// Record `pos` as failed at time `now_sec`.
  void add(const Eigen::Vector3f& pos, double now_sec);

  /// Drop entries older than `ttl_sec` relative to `now_sec`. Pops from the
  /// front (oldest first) until the first non-expired entry.
  void prune(double now_sec, double ttl_sec);

  /// True if the XY of `pos` is within `radius_m` of any stored entry.
  bool isNear(const Eigen::Vector3f& pos, double radius_m) const;

  /// Time of the newest entry within `radius_m` (XY) of `pos`, or -infinity
  /// if there is none.
  double newestTimeNear(const Eigen::Vector3f& pos, double radius_m) const;

  std::size_t size() const { return entries_.size(); }
  bool empty() const { return entries_.empty(); }

private:
  // (xy position, time declared failed, seconds).
  std::deque<std::pair<Eigen::Vector3f, double>> entries_;
};

/// Order for the visited-goal fallback: indices into `positions`, the one
/// whose newest entry within `radius_m` is oldest first, so the robot goes
/// back to the area it left longest ago. Stable: equal times keep the input
/// order (the caller's utility ranking).
std::vector<std::size_t> leastRecentFirst(
    const FailedGoalBlacklist& bl,
    const std::vector<Eigen::Vector3f>& positions, double radius_m);

} // namespace explo_planner
