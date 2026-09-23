#pragma once
/// @file candidate_generator.hpp
/// @brief Polar-grid viewpoint candidate generation.
/// Moved comments: doc/explo_planner_code_notes.md

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
  /// Terrain mode (with a map): candidate z = local ground + z_clearance,
  /// searched around the robot z (polar) or centroid z (frontier); no ground
  /// keeps that reference z. False (default): fixed absolute robot_z.
  /// (notes: cand-terrain-relative)
  bool  terrain_relative     = false;
  float z_clearance          = 0.5f;  ///< Candidate height above detected ground (m)
  float ground_search_below  = 4.0f;  ///< Ground search window below the reference z (m)
  float ground_search_above  = 1.0f;  ///< Ground search window above the reference z (m)
  float ground_stack_max_m   = 0.6f;  ///< Contiguous occupied-stack walk cap (vertical smear)
  /// Hard XY box (map frame, m): candidates whose centre falls outside are
  /// dropped. Match the dscovox planning_map size and origin to it.
  /// (notes: cand-roi-xy-box)
  float roi_min_x   = -1e9f;
  float roi_max_x   =  1e9f;
  float roi_min_y   = -1e9f;
  float roi_max_y   =  1e9f;
  /// Absolute band (m) a terrain-snapped z is clamped into: the band MapCache
  /// ingested, pushed each PLAN tick via setRoiZ. Outside it a viewpoint scores
  /// un-ingested cells at the maximal prior. Inert in flat mode.
  /// (notes: cand-roi-z-clamp)
  float roi_min_z   = -1e9f;
  float roi_max_z   =  1e9f;
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

  /// Adds frontier centroids as candidates: flat mode sets z to robot_z;
  /// terrain mode snaps to ground + z_clearance searched around the centroid z,
  /// or keeps the centroid z when no ground is found.
  /// (notes: cand-frontier-candidates)
  void addFrontierCandidates(
      std::vector<CandidateViewpoint>& candidates,
      const std::vector<Eigen::Vector3f>& frontier_centroids,
      const Eigen::Vector3f& robot_pos,
      const MapCache* map = nullptr) const;

  /// Read-only access to the configuration (mirrors VantagePlanner::config()).
  /// The exploitation path needs the terrain-mode ground-search window and
  /// clearance to place vantages on the local ground the same way exploration
  /// candidates are placed.
  const CandidateConfig& config() const { return cfg_; }

  /// Re-point the candidate z clamp at the map's current ingest band. In
  /// terrain-relative mode that band rides with the robot, so the node calls
  /// this each PLAN tick alongside FovEvaluator::setRoiZ() to keep the
  /// candidate clamp and the ray clip describing the same volume.
  void setRoiZ(float roi_min_z, float roi_max_z) {
    cfg_.roi_min_z = roi_min_z;
    cfg_.roi_max_z = roi_max_z;
  }

private:
  CandidateConfig cfg_;

  /// Terrain z for a candidate at (x, y): ground + z_clearance, searching the
  /// window around z_ref; z_ref when no ground is found.
  float terrainZ(float x, float y, float z_ref, const MapCache& map) const;
};

} // namespace explo_planner
