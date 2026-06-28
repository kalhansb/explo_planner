#pragma once
/// @file scoring.hpp
/// @brief Viewpoint scoring functions for NBV exploration.

#include <functional>
#include <string>

namespace explo_planner {

struct UnifiedVoxel {
  float a_occ  = 1.0f;
  float a_free = 1.0f;
  float p_occ  = 0.5f;
  bool  observed = false;
};

using ScoreFn = std::function<float(const UnifiedVoxel&)>;

namespace scoring {

/// SCovox Beta EIG via scovox::expectedInformationGain().
/// Unobserved voxels use Beta(1,1) prior → maximum EIG.
float eig(const UnifiedVoxel& v);

/// Shannon entropy H(p) = -p*log(p) - (1-p)*log(1-p).
/// This is the best uncertainty metric available from log-odds.
float entropy(const UnifiedVoxel& v);

/// Frontier score: 1.0 if unobserved, 0.0 if observed.
float frontier(const UnifiedVoxel& v);

/// Random: always returns 0.0 (candidate selected randomly).
float random(const UnifiedVoxel& v);

/// Factory: return score function by planner type name.
ScoreFn create(const std::string& planner_type);

} // namespace scoring
} // namespace explo_planner
