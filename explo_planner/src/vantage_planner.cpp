// Moved comments: doc/explo_planner_code_notes.md
#include "explo_planner/vantage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "explo_planner/map_cache.hpp"

namespace explo_planner {

float VantagePlanner::standoffFor(float radius) const {
  const float r = std::max(radius, 0.0f);
  float d = r + cfg_.standoff_m;
  // Keep the trunk surface beyond the min range and the trunk axis within the
  // max range. lo > hi only for a trunk wider than the sensor envelope; fall
  // back to max_range (std::clamp is UB when lo > hi).
  const float lo = cfg_.fov_min_range + r;
  const float hi = cfg_.fov_max_range;
  if (lo > hi) return hi;
  return std::clamp(d, lo, hi);
}

std::vector<CandidateViewpoint> VantagePlanner::generateVantages(
    const Eigen::Vector3f& center, float radius) const {
  std::vector<CandidateViewpoint> out;
  const int n = std::max(cfg_.n_vantages, 0);
  if (n == 0) return out;
  out.reserve(static_cast<size_t>(n));

  const float d = standoffFor(radius);
  const float step = 2.0f * static_cast<float>(M_PI) / static_cast<float>(n);
  for (int i = 0; i < n; ++i) {
    const float theta = cfg_.start_angle_rad + static_cast<float>(i) * step;
    CandidateViewpoint vp;
    vp.position = Eigen::Vector3f(center.x() + d * std::cos(theta),
                                  center.y() + d * std::sin(theta),
                                  cfg_.robot_z);
    // Face the trunk: the heading from the vantage back to the centre is
    // theta + pi.
    vp.yaw = theta + static_cast<float>(M_PI);
    vp.is_vantage = true;
    out.push_back(vp);
  }
  return out;
}

bool VantagePlanner::lineOfSightClear(const Eigen::Vector3f& from,
                                      const Eigen::Vector3f& center,
                                      float radius,
                                      const MapCache& map) const {
  // March horizontally at the sightline height toward the trunk axis.
  const Eigen::Vector3f target(center.x(), center.y(), from.z());
  Eigen::Vector3f delta = target - from;
  const float dist = delta.norm();
  if (dist < 1e-3f) return true;  // already at the axis
  const Eigen::Vector3f dir = delta / dist;

  const float res = static_cast<float>(map.resolution());
  const float step = res > 0.0f ? res : 0.1f;
  // Stop one voxel short of the trunk surface: samples from there inward are
  // the trunk itself (an expected hit), not occluders.
  const float surface = std::max(radius, 0.0f) + step;
  const float stop = dist - surface;
  if (stop <= 0.0f) return true;  // standoff is at/inside the trunk surface

  // Amanatides-Woo traversal: one midpoint sample per crossed voxel, which no
  // fixed spacing guarantees. Float boundaries can sit a whole voxel off
  // MapCache keying; do not widen to double or edit without a ground-truth
  // sweep. (notes: vantage-los-voxel-traversal)
  constexpr float kInf = std::numeric_limits<float>::infinity();
  float t_max[3], t_delta[3];
  // Begin one step out from the vantage: the vantage cell itself is the free
  // pose we are standing on. That leading offset is a full resolution and is
  // the own-cell carve-out, unrelated to the traversal below.
  const float t_begin = step;
  if (t_begin >= stop) return true;
  for (int a = 0; a < 3; ++a) {
    const float d = dir[a];
    if (d == 0.0f) {
      t_max[a] = kInf;
      t_delta[a] = kInf;
      continue;
    }
    const float p = from[a] + d * t_begin;
    const float cell = std::floor(p / step);
    // Leaving through the far face for d > 0, the near face for d < 0.
    const float bound = (d > 0.0f ? cell + 1.0f : cell) * step;
    t_max[a] = t_begin + (bound - p) / d;
    t_delta[a] = step / std::abs(d);
  }

  float t_enter = t_begin;
  while (t_enter < stop) {
    const float t_exit = std::min({t_max[0], t_max[1], t_max[2], stop});
    const Eigen::Vector3f p = from + dir * (0.5f * (t_enter + t_exit));
    if (map.getVoxel(p).p_occ >= cfg_.occ_stop) return false;
    if (t_exit >= stop) break;
    // Advance every axis whose boundary this crossing was, not just one: on an
    // exact corner crossing two or three of them coincide, and advancing one
    // would leave the others reporting a boundary already behind us.
    for (int a = 0; a < 3; ++a)
      if (t_max[a] <= t_exit) t_max[a] += t_delta[a];
    t_enter = t_exit;
  }
  return true;
}

}  // namespace explo_planner
