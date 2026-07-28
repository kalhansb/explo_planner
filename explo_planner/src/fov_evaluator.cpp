#include "explo_planner/fov_evaluator.hpp"
#include "explo_planner/map_cache.hpp"
#include <scovox/ray_iterator.hpp>
#include <scovox/uncertainty.hpp>
#include <cmath>

namespace explo_planner {

namespace {

// Clip the observable segment of one ray — the span from the sensor's minimum
// range out to max_range — against the XYZ ROI box. Standard slab method:
// intersect the per-axis entry/exit intervals with [min_range, max_range] and
// keep what survives.
//
// BOTH ends need clipping, and the entry end for two distinct reasons:
//
//  1. A candidate within min_range of an ROI face, firing outward, has its ROI
//     exit BEFORE min_range. Clamping only the far end left `ray_start` at
//     origin + dir*min_range — outside the box and PAST the clamped far end —
//     so RayIterator walked backwards through cells map_cache_ never ingests,
//     scoring each as the Beta(1,1) max-uncertainty prior and inflating info
//     gain exactly at the ROI boundary. Caught by the (t_exit > t_enter) test.
//
//  2. An origin already OUTSIDE the box on some axis, firing back toward it.
//     In terrain mode this is reachable in the shipped config: a frontier
//     candidate is snapped to ground + z_clearance searched around the
//     CENTROID's z, so it can land up to (ground_search_above + z_clearance)
//     above the ingested band. Every near-horizontal ray from there has
//     d.z ~ 0, so an exit-only clip skipped the z axis entirely (see 3 below)
//     and walked the full max_range through un-ingested space at the prior —
//     the same boundary bias, one axis over. CandidateGenerator now clamps
//     candidate z into the band as well, so this is belt-and-braces.
//
//  3. d[i] == 0 means the ray is parallel to that pair of faces and never
//     crosses either: the axis contributes no bound, and the ray is entirely
//     in or entirely out according to the origin alone. Skipping the axis (the
//     old behaviour) silently treated "entirely out" as "unconstrained".
//
// Returns false when nothing observable survives, in which case the ray must be
// skipped rather than walked.
bool clipRayToRoi(const Eigen::Vector3f& origin,
                  const Eigen::Vector3f& world_dir,
                  const FovConfig& cfg,
                  Eigen::Vector3f& ray_start,
                  Eigen::Vector3f& ray_end) {
  // world_dir is unit length, so t is a distance in metres along the ray.
  float t_enter = cfg.min_range;
  float t_exit  = cfg.max_range;
  const float o[3] = {origin.x(), origin.y(), origin.z()};
  const float d[3] = {world_dir.x(), world_dir.y(), world_dir.z()};
  const float lo[3] = {cfg.roi_min_x, cfg.roi_min_y, cfg.roi_min_z};
  const float hi[3] = {cfg.roi_max_x, cfg.roi_max_y, cfg.roi_max_z};
  for (int i = 0; i < 3; ++i) {
    if (d[i] > 0.0f) {
      t_enter = std::max(t_enter, (lo[i] - o[i]) / d[i]);
      t_exit  = std::min(t_exit,  (hi[i] - o[i]) / d[i]);
    } else if (d[i] < 0.0f) {
      t_enter = std::max(t_enter, (hi[i] - o[i]) / d[i]);
      t_exit  = std::min(t_exit,  (lo[i] - o[i]) / d[i]);
    } else if (o[i] < lo[i] || o[i] > hi[i]) {
      return false;  // parallel to this slab and outside it: never enters
    }
  }

  // Empty span: the ROI exit lands at or before the entry, so there is no
  // observable voxel inside the box on this ray at all.
  if (!(t_exit > t_enter)) return false;

  ray_start = origin + world_dir * t_enter;
  ray_end   = origin + world_dir * t_exit;
  return true;
}

}  // namespace

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

    // Clip the observable span [min_range, max_range] to the XYZ ROI. The z
    // clamp keeps the raycast volume consistent with the band map_cache_
    // holds, so out-of-band space is treated as empty (no contribution, no
    // occlusion) rather than as max-uncertainty prior voxels.
    Eigen::Vector3f ray_start, ray_end;
    if (!clipRayToRoi(vp.position, world_dir, cfg_, ray_start, ray_end))
      continue;  // ROI exits inside the sensor dead zone — nothing observable

    auto c_origin = map.posToCoord(ray_start);
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

    // Clip the observable span [min_range, max_range] to the XYZ ROI (same as
    // evaluate).
    Eigen::Vector3f ray_start, ray_end;
    if (!clipRayToRoi(vp.position, world_dir, cfg_, ray_start, ray_end))
      continue;  // ROI exits inside the sensor dead zone — nothing observable

    auto c_origin = map.posToCoord(ray_start);
    auto c_end    = map.posToCoord(ray_end);

    // Per-ray state for SSMI marginalisation.
    float reach = 1.0f;        // P(ray reaches current voxel)
    float free_kl_acc = 0.0f;  // accumulated KL from free observations
    bool  occluded = false;    // ray terminated early on an occupied voxel

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
          if (ptr && ptr->p_occ >= cfg_.occ_stop) {
            occluded = true;
            return false;
          }
          return true;
        });

    // "No hit" event: the ray passes through every voxel on its span and each
    // cell receives a free observation. Only valid when the walk actually ran
    // to c_end. On the occlusion-stop path the iteration was TRUNCATED, so the
    // residual `reach` is not "passed through cleanly" — it is the unmodelled
    // mass beyond the occluder, and the cells that would carry it were never
    // visited. Adding the term there credited a ray that demonstrably hit an
    // occupied voxel with the full free-observation KL of everything in front
    // of it, biasing the score toward staring at occluders.
    if (!occluded) result.total_score += reach * free_kl_acc;
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
