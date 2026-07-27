#include "explo_planner/candidate_generator.hpp"
#include "explo_planner/map_cache.hpp"
#include <cmath>

namespace explo_planner {

CandidateGenerator::CandidateGenerator(const CandidateConfig& cfg)
    : cfg_(cfg) {}

std::vector<CandidateViewpoint> CandidateGenerator::generate(
    const Eigen::Vector3f& robot_pos,
    float robot_yaw,
    const MapCache* map) const {
  std::vector<CandidateViewpoint> candidates;
  if (!cfg_.enable_polar) return candidates;

  candidates.reserve(
      static_cast<size_t>(cfg_.n_rings * cfg_.n_radial * cfg_.n_yaw));

  const float dr = (cfg_.n_rings > 1)
      ? (cfg_.max_radius - cfg_.min_radius) / static_cast<float>(cfg_.n_rings - 1)
      : 0.0f;

  for (int ri = 0; ri < cfg_.n_rings; ++ri) {
    float radius = cfg_.min_radius + static_cast<float>(ri) * dr;

    for (int ai = 0; ai < cfg_.n_radial; ++ai) {
      float angle = robot_yaw +
          2.0f * static_cast<float>(M_PI) * static_cast<float>(ai) /
              static_cast<float>(cfg_.n_radial);

      Eigen::Vector3f pos(
          robot_pos.x() + radius * std::cos(angle),
          robot_pos.y() + radius * std::sin(angle),
          cfg_.robot_z);

      // Filter out-of-ROI candidates
      if (pos.x() < cfg_.roi_min_x || pos.x() > cfg_.roi_max_x ||
          pos.y() < cfg_.roi_min_y || pos.y() > cfg_.roi_max_y) continue;

      const bool terrain = cfg_.terrain_relative && map;
      if (terrain) {
        // Terrain-relative: follow the local ground, referenced to the
        // robot's current altitude (nearby slopes stay inside the window).
        pos.z() = terrainZ(pos.x(), pos.y(), robot_pos.z(), *map);
      }

      // Filter occupied candidates (only when 3D map is available).
      // Flat mode: skip the check for ground-level voxels — they are always
      // occupied. Terrain mode: the candidate sits z_clearance above the
      // detected ground, so an occupied voxel there is a real obstacle
      // (canopy/overhang/wall) — always check.
      if (map && (terrain || pos.z() > cfg_.ground_z)) {
        auto voxel = map->getVoxel(pos);
        if (voxel.observed && voxel.p_occ >= cfg_.occ_thresh) continue;
      }

      for (int yi = 0; yi < cfg_.n_yaw; ++yi) {
        float yaw = angle + static_cast<float>(M_PI) +
            2.0f * static_cast<float>(M_PI) * static_cast<float>(yi) /
                static_cast<float>(cfg_.n_yaw);
        // Normalize yaw to [-pi, pi]
        yaw = std::remainder(yaw, 2.0f * static_cast<float>(M_PI));

        CandidateViewpoint vp;
        vp.position = pos;
        vp.yaw = yaw;
        candidates.push_back(vp);
      }
    }
  }
  return candidates;
}

void CandidateGenerator::addFrontierCandidates(
    std::vector<CandidateViewpoint>& candidates,
    const std::vector<Eigen::Vector3f>& frontier_centroids,
    const Eigen::Vector3f& robot_pos,
    const MapCache* map) const {
  const bool terrain = cfg_.terrain_relative && map;
  for (const auto& fc : frontier_centroids) {
    // Filter out-of-ROI frontier centroids
    if (fc.x() < cfg_.roi_min_x || fc.x() > cfg_.roi_max_x ||
        fc.y() < cfg_.roi_min_y || fc.y() > cfg_.roi_max_y) continue;

    // Yaw facing toward the frontier centroid from robot
    float dx = fc.x() - robot_pos.x();
    float dy = fc.y() - robot_pos.y();
    float yaw = std::atan2(dy, dx);

    CandidateViewpoint vp;
    // Terrain mode references the ground search to the centroid's OWN z (a
    // distant frontier can sit many metres above/below the robot); the
    // centroid z itself is the fallback — frontiers border unobserved
    // columns, so a missing ground there is expected, and the centroid is a
    // real free voxel at a plausible height already.
    const float z = terrain ? terrainZ(fc.x(), fc.y(), fc.z(), *map)
                            : cfg_.robot_z;
    vp.position = Eigen::Vector3f(fc.x(), fc.y(), z);
    vp.yaw = yaw;
    vp.is_frontier = true;
    candidates.push_back(vp);
  }
}

float CandidateGenerator::terrainZ(float x, float y, float z_ref,
                                   const MapCache& map) const {
  const float gz = map.groundZAt(
      x, y,
      z_ref - cfg_.ground_search_below,
      z_ref + cfg_.ground_search_above,
      cfg_.occ_thresh, cfg_.ground_stack_max_m);
  return std::isfinite(gz) ? gz + cfg_.z_clearance : z_ref;
}

} // namespace explo_planner
