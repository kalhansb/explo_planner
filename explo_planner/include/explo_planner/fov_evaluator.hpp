#pragma once
/// @file fov_evaluator.hpp
/// @brief Simulated FOV ray-casting for viewpoint evaluation.

#include "explo_planner/scoring.hpp"
#include "explo_planner/candidate_generator.hpp"
#include <Eigen/Core>
#include <vector>

namespace explo_planner {

class MapCache;

/// NOT a sensor model. Every field below is overwritten from parameters by
/// explo_planner_node before the evaluator is constructed, so these values
/// never reach a run; the deployed sensor lives in `shared_params.yaml`
/// (`fov_*`) and is traceable to the SDF. They are deliberately left at the
/// old narrow-camera numbers so that the unit tests, which construct a
/// FovConfig and set only the fields they are exercising, keep testing the
/// directional geometry they were written for. Do not "correct" them to the
/// deployed values: that would create a second place for the sensor model to
/// drift, and would silently change what ten tests assert.
struct FovConfig {
  float hfov      = 1.047f;   ///< Horizontal FOV (radians). Test placeholder.
  float vfov      = 0.785f;   ///< Vertical FOV (radians). Test placeholder.
  float min_range  = 0.3f;    ///< Minimum sensor range (m)
  float max_range  = 10.0f;   ///< Maximum sensor range (m). Test placeholder.
  int   h_rays     = 16;      ///< Horizontal ray samples. Test placeholder.
  int   v_rays     = 12;      ///< Vertical ray samples. Test placeholder.
  float occ_stop   = 0.7f;    ///< Stop ray at voxels above this p_occ

  /// XYZ ROI bounds.  Rays are clipped at the ROI boundary so the
  /// evaluator never scores voxels outside the region of interest. The z
  /// band must match the volume the local map_cache_ actually holds (in
  /// dscovox mode that is the GetRegion fetch band): otherwise rays leaving
  /// the band traverse absent voxels and score them as the Beta(1,1) prior,
  /// inflating info gain for upward-pointing rays into unfetched space.
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

  /// Re-band the vertical ray clip. Terrain-relative mode moves the map z-slab
  /// with the robot each PLAN tick; the ray clip must track it so out-of-band
  /// space stays "empty" (no score, no occlusion) rather than reading absent
  /// voxels as the max-uncertainty prior. Ray directions only depend on the
  /// FOV so no recompute is needed.
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
