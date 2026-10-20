/// @file failed_goal_blacklist.cpp
/// @brief Definitions for FailedGoalBlacklist (see header).

#include "explo_planner/failed_goal_blacklist.hpp"

#include <algorithm>
#include <limits>
#include <numeric>

namespace explo_planner {

void FailedGoalBlacklist::add(const Eigen::Vector3f& pos, double now_sec) {
  entries_.emplace_back(pos, now_sec);
}

void FailedGoalBlacklist::prune(double now_sec, double ttl_sec) {
  // Full scan rather than a pop-front-until-fresh loop. The early-break form
  // assumed insertion order implies age order, which holds only for a
  // monotonic clock — under sim time a bag restart or a /clock step backwards
  // stamps a fresh entry with an *older* timestamp than the one behind it, and
  // the break then left every expired entry after it blacklisting goals
  // forever. Entries stamped in the future (age < 0) are treated as fresh.
  entries_.erase(
      std::remove_if(entries_.begin(), entries_.end(),
                     [now_sec, ttl_sec](const auto& e) {
                       return now_sec - e.second > ttl_sec;
                     }),
      entries_.end());
}

bool FailedGoalBlacklist::isNear(const Eigen::Vector3f& pos,
                                 double radius_m) const {
  const float r2 = static_cast<float>(radius_m * radius_m);
  for (const auto& [p, _t] : entries_) {
    float dx = pos.x() - p.x();
    float dy = pos.y() - p.y();
    if (dx * dx + dy * dy < r2) return true;
  }
  return false;
}

double FailedGoalBlacklist::newestTimeNear(const Eigen::Vector3f& pos,
                                           double radius_m) const {
  const float r2 = static_cast<float>(radius_m * radius_m);
  double newest = -std::numeric_limits<double>::infinity();
  for (const auto& [p, t] : entries_) {
    float dx = pos.x() - p.x();
    float dy = pos.y() - p.y();
    if (dx * dx + dy * dy < r2) newest = std::max(newest, t);
  }
  return newest;
}

std::vector<std::size_t> leastRecentFirst(
    const FailedGoalBlacklist& bl,
    const std::vector<Eigen::Vector3f>& positions, double radius_m) {
  std::vector<double> newest(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    newest[i] = bl.newestTimeNear(positions[i], radius_m);
  }
  std::vector<std::size_t> order(positions.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) {
                     return newest[a] < newest[b];
                   });
  return order;
}

} // namespace explo_planner
