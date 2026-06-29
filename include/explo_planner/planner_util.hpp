#pragma once
/// @file planner_util.hpp
/// @brief Small pure helpers shared by the exploration planner node.

#include <cstdint>
#include <string>

namespace explo_planner {

/// Map a planner_type string to the diagnostic enum used in RobotIntent:
/// 0=eig, 1=entropy, 2=frontier, 3=random, 4=ssmi, 255=unknown.
uint8_t plannerTypeId(const std::string& planner_type);

/// Distance-budgeted NAVIGATE timeout (seconds):
///   budget = clamp(dist / max(speed_est, 1e-3) * safety, min_sec, max_sec)
/// so a short hop gets a small budget and a long hop a larger one, instead of
/// every goal sharing one fixed timeout.
double navBudgetSec(double dist_m, double speed_est_mps, double safety_factor,
                    double min_sec, double max_sec);

} // namespace explo_planner
