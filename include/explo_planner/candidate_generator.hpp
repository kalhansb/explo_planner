#pragma once
/// @file candidate_generator.hpp
/// @brief Polar-grid viewpoint candidate generation.

#include <Eigen/Core>
#include <vector>

namespace explo_planner {

class MapCache;

struct CandidateViewpoint {
  Eigen::Vector3f position;
  float yaw   = 0.0f;
  float score = 0.0f;
  bool  is_frontier = false;  ///< True for frontier centroid candidates
  bool  is_vantage  = false;  ///< True for exploitation vantage points around a target
};

struct CandidateConfig {
  int   n_radial    = 8;      ///< Angular samples around robot
  int   n_rings     = 3;      ///< Distance rings
  float min_radius  = 2.0f;   ///< Closest ring (m)
  float max_radius  = 8.0f;   ///< Farthest ring (m)
  int   n_yaw       = 4;      ///< Yaw samples per position
  float robot_z     = 0.3f;   ///< Fixed candidate height (UGV)
  float occ_thresh  = 0.7f;   ///< Reject candidates with p_occ above this
  float ground_z    = 0.15f;  ///< Voxels at or below this height are ground (ignored for occupancy check)
  bool  enable_polar = true;  ///< When false, generate() returns empty (frontier-only mode)
  /// Terrain-relative (3D) mode. When true AND a map is provided, each
  /// candidate's z is set to the local ground elevation (MapCache::groundZAt,
  /// searched in a window around the reference z: the robot z for polar
  /// candidates, the centroid z for frontier candidates) plus z_clearance.
  /// Columns with no detected ground fall back to the reference z unchanged.
  /// When false (default) the legacy flat-world behaviour is preserved
  /// bit-for-bit: every candidate sits at the fixed absolute robot_z.
  bool  terrain_relative     = false;
  float z_clearance          = 0.5f;  ///< Candidate height above detected ground (m)
  float ground_search_below  = 4.0f;  ///< Ground search window below the reference z (m)
  float ground_search_above  = 1.0f;  ///< Ground search window above the reference z (m)
  float ground_stack_max_m   = 0.6f;  ///< Contiguous occupied-stack walk cap (vertical smear)
  /// Hard XY bounding box on candidate positions (map frame, metres). The
  /// generator drops any candidate (radial or frontier) whose centre falls
  /// outside [roi_min_x, roi_max_x] x [roi_min_y, roi_max_y]. Set the dscovox
  /// planning_map size + origin to match this box so the global planner is
  /// constrained to the same area.
  float roi_min_x   = -1e9f;
  float roi_max_x   =  1e9f;
  float roi_min_y   = -1e9f;
  float roi_max_y   =  1e9f;
};

class CandidateGenerator {
public:
  explicit CandidateGenerator(const CandidateConfig& cfg);

  /// Generate candidate viewpoints around the robot pose.
  /// Filters out candidates in occupied space when map is provided.
  /// Pass nullptr to skip the 3D occupancy check (dscovox mode uses
  /// the 2D planning_map for filtering instead).
  std::vector<CandidateViewpoint> generate(
      const Eigen::Vector3f& robot_pos,
      float robot_yaw,
      const MapCache* map = nullptr) const;

  /// Inject frontier centroids as additional candidates. In flat mode the
  /// centroid z is flattened to robot_z; in terrain-relative mode (with a
  /// map) the candidate is snapped to ground + z_clearance, searched around
  /// the centroid's own z, falling back to the centroid z when no ground is
  /// found (frontiers naturally border unobserved columns).
  void addFrontierCandidates(
      std::vector<CandidateViewpoint>& candidates,
      const std::vector<Eigen::Vector3f>& frontier_centroids,
      const Eigen::Vector3f& robot_pos,
      const MapCache* map = nullptr) const;

private:
  CandidateConfig cfg_;

  /// Terrain z for a candidate at (x, y): ground + z_clearance, searching the
  /// window around z_ref; z_ref when no ground is found.
  float terrainZ(float x, float y, float z_ref, const MapCache& map) const;
};

} // namespace explo_planner
