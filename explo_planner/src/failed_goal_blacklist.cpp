/// @file failed_goal_blacklist.cpp
/// @brief Definitions for FailedGoalBlacklist (see header).

#include "explo_planner/failed_goal_blacklist.hpp"

#include <algorithm>

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

} // namespace explo_planner
