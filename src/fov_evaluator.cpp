#include "explo_planner/fov_evaluator.hpp"
#include "explo_planner/map_cache.hpp"
#include <scovox/ray_iterator.hpp>
#include <scovox/uncertainty.hpp>
#include <cmath>

namespace explo_planner {

FovEvaluator::FovEvaluator(const FovConfig& cfg) : cfg_(cfg) {
  precomputeRays();
}

void FovEvaluator::precomputeRays() {
  ray_dirs_.clear();
  ray_dirs_.reserve(static_cast<size_t>(cfg_.h_rays * cfg_.v_rays));

  const float h_step = cfg_.hfov / static_cast<float>(cfg_.h_rays);
  const float v_step = cfg_.vfov / static_cast<float>(cfg_.v_rays);
  const float h_start = -cfg_.hfov * 0.5f + h_step * 0.5f;
  const float v_start = -cfg_.vfov * 0.5f + v_step * 0.5f;

  for (int vi = 0; vi < cfg_.v_rays; ++vi) {
    float pitch = v_start + static_cast<float>(vi) * v_step;
    float cp = std::cos(pitch);
    float sp = std::sin(pitch);

    for (int hi = 0; hi < cfg_.h_rays; ++hi) {
      float yaw = h_start + static_cast<float>(hi) * h_step;
      // Direction in camera frame (forward = +X in camera, but we use
      // world convention: forward = cos(yaw), sin(yaw))
      // These are relative offsets; the candidate yaw rotates them.
      ray_dirs_.emplace_back(cp * std::cos(yaw), cp * std::sin(yaw), sp);
    }
  }
}

EvalResult FovEvaluator::evaluate(
    const CandidateViewpoint& vp,
    const MapCache& map,
    const ScoreFn& score_fn) const {
  EvalResult result;

  const float cy = std::cos(vp.yaw);
  const float sy = std::sin(vp.yaw);

  auto acc = map.grid().createConstAccessor();

  for (const auto& dir : ray_dirs_) {
    // Rotate ray direction by candidate yaw (around Z axis)
    Eigen::Vector3f world_dir(
        cy * dir.x() - sy * dir.y(),
        sy * dir.x() + cy * dir.y(),
        dir.z());

    Eigen::Vector3f ray_end = vp.position + world_dir * cfg_.max_range;

    // Clamp ray_end to the XYZ ROI.  Origin is inside the ROI
    // (CandidateGenerator filters it), so we only need the exit-t. The z
    // clamp keeps the raycast volume consistent with the band map_cache_
    // holds, so out-of-band space is treated as empty (no contribution, no
    // occlusion) rather than as max-uncertainty prior voxels.
    {
      Eigen::Vector3f d = ray_end - vp.position;
      float t_exit = 1.0f;
      if (d.x() > 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_max_x - vp.position.x()) / d.x());
      else if (d.x() < 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_min_x - vp.position.x()) / d.x());
      if (d.y() > 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_max_y - vp.position.y()) / d.y());
      else if (d.y() < 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_min_y - vp.position.y()) / d.y());
      if (d.z() > 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_max_z - vp.position.z()) / d.z());
      else if (d.z() < 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_min_z - vp.position.z()) / d.z());
      if (t_exit < 1.0f)
        ray_end = vp.position + d * std::max(t_exit, 0.0f);
    }

    // Start the ray at the sensor's minimum range: voxels inside the dead
    // zone the real depth sensor cannot observe must not be scored.
    auto c_origin = map.posToCoord(vp.position + world_dir * cfg_.min_range);
    auto c_end    = map.posToCoord(ray_end);

    scovox::RayIterator(c_origin, c_end,
        [&](const Bonxai::CoordT& c) -> bool {
          result.total_ray_voxels++;
          const UnifiedVoxel* ptr = acc.value(c);

          UnifiedVoxel uv;
          if (ptr) {
            uv = *ptr;
            result.observed_count++;
          } else {
            // Unobserved: default Beta(1,1) prior
            result.unknown_count++;
          }

          result.total_score += score_fn(uv);

          // Stop ray at occupied voxels (simulates occlusion)
          if (ptr && ptr->p_occ >= cfg_.occ_stop) return false;
          return true;
        });
  }
  return result;
}

void FovEvaluator::evaluateAll(
    std::vector<CandidateViewpoint>& candidates,
    const MapCache& map,
    const ScoreFn& score_fn) const {
  for (auto& vp : candidates) {
    auto result = evaluate(vp, map, score_fn);
    vp.score = result.total_score;
  }
}

EvalResult FovEvaluator::evaluateSSMI(
    const CandidateViewpoint& vp,
    const MapCache& map) const {
  EvalResult result;

  const float cy = std::cos(vp.yaw);
  const float sy = std::sin(vp.yaw);

  auto acc = map.grid().createConstAccessor();

  for (const auto& dir : ray_dirs_) {
    Eigen::Vector3f world_dir(
        cy * dir.x() - sy * dir.y(),
        sy * dir.x() + cy * dir.y(),
        dir.z());

    Eigen::Vector3f ray_end = vp.position + world_dir * cfg_.max_range;

    // Clamp ray_end to the XYZ ROI (same as evaluate).
    {
      Eigen::Vector3f d = ray_end - vp.position;
      float t_exit = 1.0f;
      if (d.x() > 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_max_x - vp.position.x()) / d.x());
      else if (d.x() < 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_min_x - vp.position.x()) / d.x());
      if (d.y() > 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_max_y - vp.position.y()) / d.y());
      else if (d.y() < 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_min_y - vp.position.y()) / d.y());
      if (d.z() > 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_max_z - vp.position.z()) / d.z());
      else if (d.z() < 0.0f)
        t_exit = std::min(t_exit, (cfg_.roi_min_z - vp.position.z()) / d.z());
      if (t_exit < 1.0f)
        ray_end = vp.position + d * std::max(t_exit, 0.0f);
    }

    // Start the ray at the sensor's minimum range: voxels inside the dead
    // zone the real depth sensor cannot observe must not be scored.
    auto c_origin = map.posToCoord(vp.position + world_dir * cfg_.min_range);
    auto c_end    = map.posToCoord(ray_end);

    // Per-ray state for SSMI marginalisation.
    float reach = 1.0f;        // P(ray reaches current voxel)
    float free_kl_acc = 0.0f;  // accumulated KL from free observations

    scovox::RayIterator(c_origin, c_end,
        [&](const Bonxai::CoordT& c) -> bool {
          result.total_ray_voxels++;
          const UnifiedVoxel* ptr = acc.value(c);

          // Build a scovox::Voxel for the KL helpers.
          scovox::Voxel sv;
          float p;
          if (ptr) {
            sv.a_occ  = ptr->a_occ;
            sv.a_free = ptr->a_free;
            p = ptr->p_occ;
            result.observed_count++;
          } else {
            // Unobserved: Beta(1,1) prior.
            sv.a_occ  = 1.0f;
            sv.a_free = 1.0f;
            p = 0.5f;
            result.unknown_count++;
          }

          float kl_occ  = scovox::ssmiOccKL(sv);
          float kl_free = scovox::ssmiFreeKL(sv);

          // MI contribution from "ray hits this voxel" event:
          //   P(hit here) × [KL_occ(this) + Σ KL_free(earlier)]
          result.total_score += reach * p * (kl_occ + free_kl_acc);

          // Accumulate free-observation KL for use by later hit events
          // and the final "no hit" event.
          free_kl_acc += kl_free;

          // Update reach probability.
          reach *= (1.0f - p);

          // Hard occlusion stop (same threshold as evaluate).
          if (ptr && ptr->p_occ >= cfg_.occ_stop) return false;
          return true;
        });

    // "No hit" event: ray passes through all voxels, every cell
    // receives a free observation.
    result.total_score += reach * free_kl_acc;
  }
  return result;
}

void FovEvaluator::evaluateAllSSMI(
    std::vector<CandidateViewpoint>& candidates,
    const MapCache& map) const {
  for (auto& vp : candidates) {
    auto result = evaluateSSMI(vp, map);
    vp.score = result.total_score;
  }
}

} // namespace explo_planner
