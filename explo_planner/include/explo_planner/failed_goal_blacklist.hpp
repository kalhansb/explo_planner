#pragma once
/// @file failed_goal_blacklist.hpp
/// @brief TTL + radius blacklist of recently-failed goal positions.
/// Moved comments: doc/explo_planner_code_notes.md

#include <cstddef>
#include <vector>

#include <Eigen/Core>

namespace explo_planner {

/// One clustered site: position, last failure time, failure count. expired
/// means it no longer suppresses goals but keeps its count, which retirement
/// counts in (see prune()). (notes: failed-goal-site-expired)
struct FailedGoalSite {
  Eigen::Vector3f pos{Eigen::Vector3f::Zero()};
  double last_fail_time{0.0};
  int count{0};
  bool retired{false};
  bool expired{false};
};

/// Failed goal positions the planner skips while hot; time is passed in, in
/// seconds. cluster_radius_m > 0 folds nearby failures into one site's count,
/// <= 0 appends. Retired sites hold until clearNear.
/// (notes: failed-goal-blacklist-design)
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

  /// Age out entries older than ttl_sec; retired sites never age out. With
  /// retirement on, an aged entry stops suppressing but keeps its count; with
  /// it off (the default) it is erased. (notes: failed-goal-prune-expiry)
  void prune(double now_sec, double ttl_sec);

  /// True if the XY of `pos` is within `radius_m` of any stored entry.
  bool isNear(const Eigen::Vector3f& pos, double radius_m) const;

  /// True if ANY entry within `radius_m` is retired. Retirement is a veto, so
  /// it is not overridden by a nearer non-retired record (see the .cpp).
  bool isRetiredNear(const Eigen::Vector3f& pos, double radius_m) const;

  /// Most recent failure time among entries within `radius_m`, or -infinity
  /// if there is none. Used to order amnesty picks (oldest failure first).
  double lastFailTimeNear(const Eigen::Vector3f& pos, double radius_m) const;

  /// Forget every entry within radius_m of pos, retired and expired history
  /// included; returns how many. Reaching the goal proves the ground reachable,
  /// so the next failure there starts at 1. (notes: failed-goal-clear-near)
  std::size_t clearNear(const Eigen::Vector3f& pos, double radius_m);

  /// Sites that are currently SUPPRESSING goals. Expired history does not
  /// count: this is what the planner logs as "active failed-goal sites", and
  /// counting records that no longer veto anything would make that line grow
  /// without bound while meaning less and less.
  std::size_t size() const;
  bool empty() const { return size() == 0; }

  /// Every record held, expired history included. For tests and introspection
  /// — the planner never branches on it.
  std::size_t historySize() const { return sites_.size(); }

private:
  std::vector<FailedGoalSite> sites_;
  int retire_after_{0};
};

/// Strict weak ordering for the amnesty valve: true if a should be re-attempted
/// before b. Non-retired first, then least-recently-failed. A free function so
/// it has a known-answer test. (notes: failed-goal-amnesty-order)
bool amnestyOrderBefore(const FailedGoalBlacklist& bl,
                        const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                        double radius_m);

} // namespace explo_planner
