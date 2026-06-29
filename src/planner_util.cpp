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

} // namespace explo_planner
