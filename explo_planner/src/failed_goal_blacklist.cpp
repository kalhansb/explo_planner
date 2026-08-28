/// @file failed_goal_blacklist.cpp
/// @brief Definitions for FailedGoalBlacklist (see header).

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
  // Full scan rather than a pop-front-until-fresh loop. The early-break form
  // assumed insertion order implies age order, which holds only for a
  // monotonic clock — under sim time a bag restart or a /clock step backwards
  // stamps a fresh entry with an *older* timestamp than the one behind it, and
  // the break then left every expired entry after it blacklisting goals
  // forever. Entries stamped in the future (age < 0) are treated as fresh.
  sites_.erase(
      std::remove_if(sites_.begin(), sites_.end(),
                     [now_sec, ttl_sec](const FailedGoalSite& s) {
                       return !s.retired && now_sec - s.last_fail_time > ttl_sec;
                     }),
      sites_.end());
}

bool FailedGoalBlacklist::isNear(const Eigen::Vector3f& pos,
                                 double radius_m) const {
  const float r2 = static_cast<float>(radius_m * radius_m);
  for (const auto& s : sites_) {
    if (dist2_xy(s.pos, pos) < r2) return true;
  }
  return false;
}

bool FailedGoalBlacklist::isRetiredNear(const Eigen::Vector3f& pos,
                                        double radius_m) const {
  // ANY retired site within the radius, not the nearest one. Nearest-wins was
  // wrong on two counts. Semantically, retirement is a veto — "held for the
  // rest of the run" — and a veto is not overturned by a fresher record
  // happening to sit a few centimetres closer. Mechanically, it did not even
  // describe the same site the caller had just written: add() clusters into
  // the FIRST site within radius, not the nearest, so failGoal could increment
  // site A and then log the retired flag of site B. That flag is what
  // nav_goal_failed carries into the analysis, so a mismatch there is a
  // silently wrong event field, not just a confusing WARN.
  const float r2 = static_cast<float>(radius_m * radius_m);
  for (const auto& s : sites_) {
    if (s.retired && dist2_xy(s.pos, pos) < r2) return true;
  }
  return false;
}

double FailedGoalBlacklist::lastFailTimeNear(const Eigen::Vector3f& pos,
                                             double radius_m) const {
  const float r2 = static_cast<float>(radius_m * radius_m);
  double latest = -std::numeric_limits<double>::infinity();
  for (const auto& s : sites_) {
    if (dist2_xy(s.pos, pos) < r2) latest = std::max(latest, s.last_fail_time);
  }
  return latest;
}

bool amnestyOrderBefore(const FailedGoalBlacklist& bl,
                        const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                        double radius_m) {
  // Retired last, then least-recently-failed.
  //
  // The retired partition is not cosmetic. Ordering on fail time ALONE hands
  // the amnesty valve straight back to the permanent traps that retirement
  // exists to hold, and does so systematically rather than occasionally: a
  // retired site is never pruned, so once the robot stops re-failing it its
  // last_fail_time only gets older, while every ordinary site near it either
  // gets re-failed (fresh time) or ages out of the blacklist entirely and
  // stops being a suppressed candidate at all. Give it a few minutes and the
  // oldest failure in the list is essentially always the confirmed trap. The
  // one valve meant to rescue a starved planner would then aim it at the
  // known-unreachable goal first, every single time.
  //
  // A retired candidate is still reachable as a last resort — that is the
  // point of amnesty, and it is what keeps the worst case no worse than the
  // old behaviour — but only once nothing merely-suppressed is left to try.
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
