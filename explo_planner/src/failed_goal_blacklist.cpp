/// @file failed_goal_blacklist.cpp
/// @brief Definitions for FailedGoalBlacklist (see header).
/// Moved comments: doc/explo_planner_code_notes.md

#include "explo_planner/failed_goal_blacklist.hpp"

#include <algorithm>
#include <limits>

namespace explo_planner {

namespace {
inline float dist2_xy(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  const float dx = a.x() - b.x();
  const float dy = a.y() - b.y();
  return dx * dx + dy * dy;
}
} // namespace

int FailedGoalBlacklist::add(const Eigen::Vector3f& pos, double now_sec,
                             double cluster_radius_m) {
  if (cluster_radius_m > 0.0) {
    const float r2 =
        static_cast<float>(cluster_radius_m * cluster_radius_m);
    for (auto& s : sites_) {
      if (dist2_xy(s.pos, pos) < r2) {
        s.last_fail_time = now_sec;
        // Failing here again revives the suppression. The count carries over
        // from before the expiry — that carry-over IS the fix: it is what lets
        // a trap the robot only returns to every few minutes reach
        // retire_after at all.
        s.expired = false;
        ++s.count;
        if (retire_after_ > 0 && s.count >= retire_after_) s.retired = true;
        return s.count;
      }
    }
  }
  FailedGoalSite s;
  s.pos = pos;
  s.last_fail_time = now_sec;
  s.count = 1;
  s.retired = (retire_after_ == 1);
  sites_.push_back(s);
  return 1;
}

void FailedGoalBlacklist::prune(double now_sec, double ttl_sec) {
  // Full scan, not pop-front-until-fresh: under sim time the clock can step
  // back, so insertion order is not age order. Entries stamped in the future
  // (age < 0) count as fresh. (notes: blacklist-prune-full-scan)
  const auto is_stale = [now_sec, ttl_sec](const FailedGoalSite& s) {
    return !s.retired && now_sec - s.last_fail_time > ttl_sec;
  };

  // Retirement OFF: nothing counts failures, so there is no history worth
  // keeping and erasing is strictly cheaper. This is the `visited_goals_`
  // path and it behaves exactly as it always has.
  if (retire_after_ <= 0) {
    sites_.erase(std::remove_if(sites_.begin(), sites_.end(), is_stale),
                 sites_.end());
    return;
  }

  // With retirement on, stale sites are marked expired, not erased, so a trap
  // revisited after the TTL keeps its count and can reach retire_after. One
  // record per failure site keeps the list small.
  // (notes: blacklist-expire-not-erase)
  for (auto& s : sites_) {
    if (is_stale(s)) s.expired = true;
  }
}

std::size_t FailedGoalBlacklist::size() const {
  std::size_t n = 0;
  for (const auto& s : sites_) {
    if (!s.expired) ++n;
  }
  return n;
}

bool FailedGoalBlacklist::isNear(const Eigen::Vector3f& pos,
                                 double radius_m) const {
  // Expired records are history, not vetoes. Suppression still ends exactly at
  // the TTL — keeping the count must not quietly turn every past failure into
  // a permanent no-go zone, which would be a much bigger behaviour change than
  // the one intended and would starve the planner instead of unsticking it.
  const float r2 = static_cast<float>(radius_m * radius_m);
  for (const auto& s : sites_) {
    if (!s.expired && dist2_xy(s.pos, pos) < r2) return true;
  }
  return false;
}

bool FailedGoalBlacklist::isRetiredNear(const Eigen::Vector3f& pos,
                                        double radius_m) const {
  // True if ANY retired site lies within the radius, not just the nearest:
  // retirement is a veto, and add() clusters into the first site within
  // radius, not the nearest. (notes: blacklist-any-retired-near)
  const float r2 = static_cast<float>(radius_m * radius_m);
  for (const auto& s : sites_) {
    if (s.retired && dist2_xy(s.pos, pos) < r2) return true;
  }
  return false;
}

double FailedGoalBlacklist::lastFailTimeNear(const Eigen::Vector3f& pos,
                                             double radius_m) const {
  // Expired records are excluded, to match isNear; its callers (amnesty
  // ordering and the amnesty log line) only pass candidates suppressed this
  // tick. (notes: blacklist-last-fail-excludes-expired)
  const float r2 = static_cast<float>(radius_m * radius_m);
  double latest = -std::numeric_limits<double>::infinity();
  for (const auto& s : sites_) {
    if (!s.expired && dist2_xy(s.pos, pos) < r2)
      latest = std::max(latest, s.last_fail_time);
  }
  return latest;
}

bool amnestyOrderBefore(const FailedGoalBlacklist& bl,
                        const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                        double radius_m) {
  // Retired last, then least-recently-failed. On fail time alone the oldest
  // failure is almost always a retired trap, so amnesty would aim there first;
  // a retired site stays a last resort. (notes: blacklist-amnesty-order)
  const bool ra = bl.isRetiredNear(a, radius_m);
  const bool rb = bl.isRetiredNear(b, radius_m);
  if (ra != rb) return !ra;  // non-retired first
  return bl.lastFailTimeNear(a, radius_m) < bl.lastFailTimeNear(b, radius_m);
}

std::size_t FailedGoalBlacklist::clearNear(const Eigen::Vector3f& pos,
                                           double radius_m) {
  const float r2 = static_cast<float>(radius_m * radius_m);
  const std::size_t before = sites_.size();
  sites_.erase(std::remove_if(sites_.begin(), sites_.end(),
                              [&](const FailedGoalSite& s) {
                                return dist2_xy(s.pos, pos) < r2;
                              }),
               sites_.end());
  return before - sites_.size();
}

} // namespace explo_planner
