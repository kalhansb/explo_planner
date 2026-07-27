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

#include <vector>

#include <Eigen/Core>

#include "explo_planner/candidate_generator.hpp"

namespace explo_planner {

class MapCache;

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

  /// Generate cfg_.n_vantages viewpoints evenly spaced on the standoff circle
  /// around `center`. Each viewpoint's yaw faces the trunk centre and
  /// is_vantage is set. Z is cfg_.robot_z. Angular order is deterministic
  /// (i = 0..n-1 at start_angle + i * 2π/n), so the same trunk yields the same
  /// vantage positions every tick (the queue's visited-by-proximity tracking
  /// relies on this stability).
  std::vector<CandidateViewpoint> generateVantages(
      const Eigen::Vector3f& center, float radius) const;

  /// Line-of-sight test from `from` to the trunk at `center` (radius). Marches
  /// a ray at the sightline height (from.z) toward the trunk axis and returns
  /// false if any known-occupied voxel (p_occ >= occ_stop) is hit *before*
  /// reaching within (radius + one voxel) of the axis — voxels at or inside the
  /// trunk surface are the trunk itself (an expected hit, not an occluder).
  /// Unknown space (Beta(1,1) prior, p_occ = 0.5) never blocks. An empty map is
  /// trivially clear.
  bool lineOfSightClear(const Eigen::Vector3f& from,
                        const Eigen::Vector3f& center, float radius,
                        const MapCache& map) const;

  const VantageConfig& config() const { return cfg_; }

private:
  VantageConfig cfg_;
};

}  // namespace explo_planner
