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

} // namespace explo_planner
