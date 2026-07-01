#include "explo_planner/vantage_planner.hpp"

#include <algorithm>
#include <cmath>

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

  // Begin one step out from the vantage (the vantage cell itself is the free
  // pose we are standing on) and sample up to the trunk surface.
  for (float t = step; t <= stop; t += step) {
    const Eigen::Vector3f p = from + dir * t;
    if (map.getVoxel(p).p_occ >= cfg_.occ_stop) return false;
  }
  return true;
}

}  // namespace explo_planner
