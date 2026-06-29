/// @file failed_goal_blacklist.cpp
/// @brief Definitions for FailedGoalBlacklist (see header).

#include "explo_planner/failed_goal_blacklist.hpp"

namespace explo_planner {

void FailedGoalBlacklist::add(const Eigen::Vector3f& pos, double now_sec) {
  entries_.emplace_back(pos, now_sec);
}

void FailedGoalBlacklist::prune(double now_sec, double ttl_sec) {
  while (!entries_.empty()) {
    double age = now_sec - entries_.front().second;
    if (age > ttl_sec) {
      entries_.pop_front();
    } else {
      break;
    }
  }
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
