#pragma once
/// @file fov_evaluator.hpp
/// @brief Simulated FOV ray-casting for viewpoint evaluation.
/// Moved comments: doc/explo_planner_code_notes.md

#include "explo_planner/scoring.hpp"
#include "explo_planner/candidate_generator.hpp"
#include <Eigen/Core>
#include <vector>

namespace explo_planner {

class MapCache;

/// Not a sensor model: the node overwrites every field from parameters before
/// construction, the sensor from the fov_* params. Defaults are unit-test
/// placeholders; do not correct them to the deployed values.
/// (notes: fov-config-test-placeholders)
struct FovConfig {
  float hfov      = 1.047f;   ///< Horizontal FOV (radians). Test placeholder.
  float vfov      = 0.785f;   ///< Vertical FOV (radians). Test placeholder.
  float min_range  = 0.3f;    ///< Minimum sensor range (m)
  float max_range  = 10.0f;   ///< Maximum sensor range (m). Test placeholder.
  int   h_rays     = 16;      ///< Horizontal ray samples. Test placeholder.
  int   v_rays     = 12;      ///< Vertical ray samples. Test placeholder.
  float occ_stop   = 0.7f;    ///< Stop ray at voxels above this p_occ

  /// XYZ ROI bounds; rays are clipped at them. The z band must match the
  /// volume map_cache_ actually holds, or rays leaving it score absent voxels
  /// as the Beta(1,1) prior and inflate info gain. (notes: fov-roi-z-band)
  float roi_min_x  = -1e9f;
  float roi_max_x  =  1e9f;
  float roi_min_y  = -1e9f;
  float roi_max_y  =  1e9f;
  float roi_min_z  = -1e9f;
  float roi_max_z  =  1e9f;
};

struct EvalResult {
  float total_score      = 0.0f;
  int   unknown_count    = 0;
  int   observed_count   = 0;
  int   total_ray_voxels = 0;
};

class FovEvaluator {
public:
  explicit FovEvaluator(const FovConfig& cfg);

  /// Re-band the vertical ray clip to follow the terrain-relative map z-slab
  /// each PLAN tick, so out-of-band space stays empty. Ray directions depend
  /// only on the FOV; no recompute. (notes: fov-set-roi-z)
  void setRoiZ(float roi_min_z, float roi_max_z) {
    cfg_.roi_min_z = roi_min_z;
    cfg_.roi_max_z = roi_max_z;
  }

  /// Evaluate a single candidate viewpoint against the map.
  EvalResult evaluate(
      const CandidateViewpoint& vp,
      const MapCache& map,
      const ScoreFn& score_fn) const;

  /// Evaluate all candidates and fill their score fields.
  void evaluateAll(
      std::vector<CandidateViewpoint>& candidates,
      const MapCache& map,
      const ScoreFn& score_fn) const;

  /// SSMI-style MI lower bound with ray marginalisation.
  /// Weights per-voxel KL contributions by the probability the ray
  /// reaches each voxel, matching the f(φ,h) formulation in
  /// Asgharivaskasi & Atanasov (TRO 2023) adapted to the Beta model.
  EvalResult evaluateSSMI(
      const CandidateViewpoint& vp,
      const MapCache& map) const;

  /// SSMI evaluation for all candidates.
  void evaluateAllSSMI(
      std::vector<CandidateViewpoint>& candidates,
      const MapCache& map) const;

private:
  FovConfig cfg_;
  std::vector<Eigen::Vector3f> ray_dirs_;  ///< Precomputed in camera frame

  void precomputeRays();
};

} // namespace explo_planner
