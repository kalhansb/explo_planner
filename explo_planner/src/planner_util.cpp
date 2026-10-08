/// @file planner_util.cpp
/// @brief Definitions for the small pure planner helpers (see header).

#include "explo_planner/planner_util.hpp"

#include <algorithm>

namespace explo_planner {

uint8_t plannerTypeId(const std::string& planner_type) {
  if (planner_type == "eig")      return 0;
  if (planner_type == "entropy")  return 1;
  if (planner_type == "frontier") return 2;
  if (planner_type == "random")   return 3;
  if (planner_type == "ssmi")     return 4;
  return 255;
}

double navBudgetSec(double dist_m, double speed_est_mps, double safety_factor,
                    double min_sec, double max_sec) {
  double raw = (dist_m / std::max(speed_est_mps, 1e-3)) * safety_factor;
  return std::clamp(raw, min_sec, max_sec);
}

bool teamComplete(int active_peers, int expected_peers) {
  return expected_peers > 0 && active_peers >= expected_peers;
}

bool shouldRendezvous(bool rendezvous_enabled, bool have_home,
                      int active_peers, int expected_peers) {
  if (!rendezvous_enabled || expected_peers <= 0 || !have_home) return false;
  return !teamComplete(active_peers, expected_peers);
}

bool rendezvousWaitExpired(double waited_sec, double max_wait_sec) {
  return max_wait_sec > 0.0 && waited_sec >= max_wait_sec;
}

int AbortStreak::onAbort(float x, float y, float still_m) {
  const float dx = x - ax_;
  const float dy = y - ay_;
  if (count_ > 0 && dx * dx + dy * dy < still_m * still_m) return ++count_;
  ax_ = x;
  ay_ = y;
  count_ = 1;
  return count_;
}

void RecoveryLog::add(float x, float y, int n) {
  if (n <= 0) return;
  if (spots_.size() >= kMaxSpots) spots_.erase(spots_.begin());
  spots_.push_back({x, y, n});
}

int RecoveryLog::countNear(float x, float y, float radius_m) const {
  int total = 0;
  for (const auto& s : spots_) {
    const float dx = s.x - x;
    const float dy = s.y - y;
    if (dx * dx + dy * dy <= radius_m * radius_m) total += s.n;
  }
  return total;
}

void RecoveryLog::shiftAll(float dx, float dy) {
  for (auto& s : spots_) {
    s.x += dx;
    s.y += dy;
  }
}

int newRecoveries(bool first_message, bool same_goal, int prev_count,
                  int count) {
  const int base = same_goal ? prev_count : (first_message ? count : 0);
  return std::max(0, count - base);
}

Nav2GiveUp nav2GiveUp(int abort_limit, int streak, bool final_after_recovery,
                      int recoveries_here, int recoveries_needed) {
  if (abort_limit <= 0) return Nav2GiveUp::kRetry;
  if (streak >= abort_limit) return Nav2GiveUp::kAbortLimit;
  if (final_after_recovery && recoveries_here >= recoveries_needed)
    return Nav2GiveUp::kAfterRecovery;
  return Nav2GiveUp::kRetry;
}

const char* nav2ErrorName(int code) {
  switch (code) {
    // FollowPath (controller server)
    case 100: return "FOLLOW_PATH_UNKNOWN";
    case 101: return "INVALID_CONTROLLER";
    case 102: return "CONTROLLER_TF_ERROR";
    case 103: return "INVALID_PATH";
    case 104: return "PATIENCE_EXCEEDED";
    case 105: return "FAILED_TO_MAKE_PROGRESS";
    case 106: return "NO_VALID_CONTROL";
    case 107: return "CONTROLLER_TIMED_OUT";
    // ComputePathToPose (planner server)
    case 200: return "COMPUTE_PATH_UNKNOWN";
    case 201: return "INVALID_PLANNER";
    case 202: return "PLANNER_TF_ERROR";
    case 203: return "START_OUTSIDE_MAP";
    case 204: return "GOAL_OUTSIDE_MAP";
    case 205: return "START_OCCUPIED";
    case 206: return "GOAL_OCCUPIED";
    case 207: return "PLANNER_TIMEOUT";
    case 208: return "NO_VALID_PATH";
    // SmoothPath
    case 500: return "SMOOTHER_UNKNOWN";
    case 501: return "INVALID_SMOOTHER";
    case 502: return "SMOOTHER_TIMEOUT";
    case 503: return "SMOOTHED_PATH_IN_COLLISION";
    case 504: return "FAILED_TO_SMOOTH_PATH";
    case 505: return "SMOOTHER_INVALID_PATH";
    // Behaviours (recoveries)
    case 700: return "SPIN_UNKNOWN";
    case 701: return "SPIN_TIMEOUT";
    case 702: return "SPIN_TF_ERROR";
    case 703: return "SPIN_COLLISION_AHEAD";
    case 710: return "BACKUP_UNKNOWN";
    case 711: return "BACKUP_TIMEOUT";
    case 712: return "BACKUP_TF_ERROR";
    case 713: return "BACKUP_INVALID_INPUT";
    case 714: return "BACKUP_COLLISION_AHEAD";
    case 720: return "DRIVE_ON_HEADING_UNKNOWN";
    case 721: return "DRIVE_ON_HEADING_TIMEOUT";
    case 722: return "DRIVE_ON_HEADING_TF_ERROR";
    case 723: return "DRIVE_ON_HEADING_COLLISION_AHEAD";
    case 724: return "DRIVE_ON_HEADING_INVALID_INPUT";
    default:  return "";
  }
}

} // namespace explo_planner
