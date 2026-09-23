#pragma once
/// @file vantage_planner.hpp
/// @brief Geometric next-best-view generation around a tree target, plus the
///        line-of-sight occlusion test used to validate each vantage.
///
/// Exploitation optimises *prescribed angular coverage* of a known trunk, not
/// raw information gain, so the primitive here is a fixed set of viewpoints
/// evenly spaced on a standoff circle around the trunk (e.g. 3 vantages 120°
/// apart). Each is emitted as a CandidateViewpoint (is_vantage = true) so it
/// flows through the same publish / cost / intent machinery the exploration
/// path already uses. Validation (in-ROI, free cell, reachable) reuses the
/// node's existing filters; the occlusion-specific check lives here as
/// lineOfSightClear(). Pure geometry + a read-only MapCache ray-march, so it is
/// unit-tested in isolation (test_vantage_planner.cpp).
/// Moved comments: doc/explo_planner_code_notes.md

#include <vector>

#include <Eigen/Core>

#include "explo_planner/candidate_generator.hpp"

namespace explo_planner {

class MapCache;

/// As with FovConfig: the node sets every field from parameters, so these are
/// unit-test placeholders rather than the deployed geometry. `fov_min_range`
/// and `fov_max_range` in particular are copied from the live FovConfig at
/// construction (explo_planner_node.cpp), not taken from here.
struct VantageConfig {
  int   n_vantages      = 3;       ///< Viewpoints per target (3 => 120° apart).
  float standoff_m      = 2.0f;    ///< Standoff added to the trunk radius (m).
  float start_angle_rad = 0.0f;    ///< Angle of the first vantage (rad).
  float robot_z         = 0.3f;    ///< Vantage height (UGV sensor height, m).
  float fov_min_range   = 0.3f;    ///< Sensor min range (clamps standoff low).
  float fov_max_range   = 10.0f;   ///< Sensor max range (clamps standoff high).
  float occ_stop        = 0.7f;    ///< p_occ at/above which a voxel blocks LoS.
};

class VantagePlanner {
public:
  explicit VantagePlanner(const VantageConfig& cfg) : cfg_(cfg) {}

  /// Standoff distance for a trunk of the given radius: radius + standoff_m,
  /// clamped to the sensor envelope so the trunk surface is past the min range
  /// and the trunk axis is within the max range.
  float standoffFor(float radius) const;

  /// cfg_.n_vantages viewpoints evenly spaced on the standoff circle, facing
  /// the trunk, is_vantage set, z = cfg_.robot_z. The angular order is
  /// deterministic, which the queue's visited-by-proximity tracking relies on.
  /// (notes: vantage-deterministic-order)
  std::vector<CandidateViewpoint> generateVantages(
      const Eigen::Vector3f& center, float radius) const;

  /// Marches a ray at from.z toward the trunk axis; false if a voxel with p_occ
  /// >= occ_stop is hit before radius + one voxel of the axis (the trunk
  /// itself). Unknown space never blocks; an empty map is clear.
  /// (notes: vantage-line-of-sight)
  bool lineOfSightClear(const Eigen::Vector3f& from,
                        const Eigen::Vector3f& center, float radius,
                        const MapCache& map) const;

  const VantageConfig& config() const { return cfg_; }

private:
  VantageConfig cfg_;
};

}  // namespace explo_planner
