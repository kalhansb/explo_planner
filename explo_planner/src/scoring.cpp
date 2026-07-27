#include "explo_planner/scoring.hpp"
#include <scovox/voxel.hpp>
#include <scovox/uncertainty.hpp>
#include <cmath>
#include <stdexcept>

namespace explo_planner {
namespace scoring {

float eig(const UnifiedVoxel& v) {
  scovox::Voxel sv;
  sv.a_occ  = v.a_occ;
  sv.a_free = v.a_free;
  return scovox::expectedInformationGain(sv);
}

float entropy(const UnifiedVoxel& v) {
  const float p = v.p_occ;
  if (p < 1e-7f || p > 1.0f - 1e-7f) return 0.0f;
  return -p * std::log(p) - (1.0f - p) * std::log(1.0f - p);
}

float frontier(const UnifiedVoxel& v) {
  return v.observed ? 0.0f : 1.0f;
}

float random(const UnifiedVoxel& /*v*/) {
  return 0.0f;
}

ScoreFn create(const std::string& planner_type) {
  if (planner_type == "eig")      return eig;
  if (planner_type == "entropy")  return entropy;
  if (planner_type == "frontier") return frontier;
  if (planner_type == "random")   return random;
  // SSMI uses a dedicated ray-level evaluation path (FovEvaluator::evaluateSSMI),
  // not the per-voxel ScoreFn. Return eig as a fallback for any code path that
  // still calls score_fn_ directly (e.g. diagnostics).
  if (planner_type == "ssmi")     return eig;
  throw std::invalid_argument("Unknown planner type: " + planner_type);
}

} // namespace scoring
} // namespace explo_planner
