#pragma once
/// @file failed_goal_blacklist.hpp
/// @brief TTL + radius blacklist of recently-failed goal positions.

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace explo_planner {

/// One clustered site: a position, when it last failed, and how often.
struct FailedGoalSite {
  Eigen::Vector3f pos{Eigen::Vector3f::Zero()};
  double last_fail_time{0.0};
  int count{0};
  bool retired{false};
};

/// Records goal positions the robot failed to reach (navigate timeout / no
/// progress) so the planner can skip re-picking the same unreachable target
/// while it's still "hot". Without this, a physically stuck robot keeps the
/// same pose -> same scores -> re-picks the same goal forever.
///
/// Time is passed in as seconds (the caller's clock) rather than read
/// internally, so the prune/expiry logic is deterministic and unit-testable
/// without a ROS clock.
///
/// Two modes, selected per call site:
///   - cluster_radius_m <= 0 (the default): every add() creates a new record.
///     This is the historical append-only behaviour and is what `visited_goals_`
///     uses; nothing about it changes.
///   - cluster_radius_m > 0: an add() within that radius of an existing record
///     increments that record's failure count instead of appending. This is
///     what turns a bag of points into a per-SITE failure history, which is
///     what retirement needs. The record keeps its first-seen centre so the
///     suppression disc does not drift across repeated failures.
///
/// Retirement (opt-in via setRetireAfter) exists because TTL expiry alone
/// cannot stop a permanent terrain trap: the entry ages out while the robot is
/// busy failing somewhere else, and the same unreachable goal wins the argmax
/// again. A site that has failed `retire_after` times is held for the rest of
/// the run and is only released by actually reaching it (clearNear).
class FailedGoalBlacklist {
public:
  /// Retire a site after this many failures. 0 disables retirement entirely,
  /// which is the default and keeps `visited_goals_` semantics untouched.
  void setRetireAfter(int n) { retire_after_ = n; }
  int retireAfter() const { return retire_after_; }

  /// Record `pos` as failed at time `now_sec`. Returns the site's failure
  /// count after this call (1 for a fresh record).
  int add(const Eigen::Vector3f& pos, double now_sec,
          double cluster_radius_m = 0.0);

  /// Drop entries older than `ttl_sec` relative to `now_sec`. Retired sites
  /// are never dropped.
  void prune(double now_sec, double ttl_sec);

  /// True if the XY of `pos` is within `radius_m` of any stored entry.
  bool isNear(const Eigen::Vector3f& pos, double radius_m) const;

  /// True if ANY entry within `radius_m` is retired. Retirement is a veto, so
  /// it is not overridden by a nearer non-retired record (see the .cpp).
  bool isRetiredNear(const Eigen::Vector3f& pos, double radius_m) const;

  /// Most recent failure time among entries within `radius_m`, or -infinity
  /// if there is none. Used to order amnesty picks (oldest failure first).
  double lastFailTimeNear(const Eigen::Vector3f& pos, double radius_m) const;

  /// Forget every entry within `radius_m` of `pos`, retired ones included.
  /// Returns how many were removed. Arriving at a goal is proof the ground is
  /// reachable, which outranks any amount of failure history.
  std::size_t clearNear(const Eigen::Vector3f& pos, double radius_m);

  std::size_t size() const { return sites_.size(); }
  bool empty() const { return sites_.empty(); }

private:
  std::vector<FailedGoalSite> sites_;
  int retire_after_{0};
};

/// Strict weak ordering for the planner's amnesty valve: true if suppressed
/// candidate `a` should be re-attempted before `b`. Non-retired first, then
/// least-recently-failed.
///
/// A free function rather than a lambda inside the planner because this
/// ordering fails silently: get it wrong and the planner still returns a goal
/// on every call, just the worst available one, with nothing in the logs to
/// distinguish that from the intended pick. Out here it has a known-answer
/// test. Rationale for the partition itself is at the definition.
bool amnestyOrderBefore(const FailedGoalBlacklist& bl,
                        const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                        double radius_m);

} // namespace explo_planner
