#pragma once
/// @file map_cache.hpp
/// @brief Read-only Bonxai grid rebuilt from ROS map messages.

#include "explo_planner/scoring.hpp"
#include <bonxai/bonxai.hpp>
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <scovox_msgs/msg/scovox_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace explo_planner {

class MapCache {
public:
  using Grid   = Bonxai::VoxelGrid<UnifiedVoxel>;
  using CoordT = Bonxai::CoordT;

  explicit MapCache(double resolution);

  /// Rebuild grid from a ScovoxMap message (full voxel dump).
  /// @return false (grid left untouched) if msg.resolution is unusable — see
  ///         the ROI overload.
  bool updateFromScovoxMap(const scovox_msgs::msg::ScovoxMap& msg);

  /// Rebuild grid from a ScovoxMap, keeping only voxels whose position lies in
  /// the inclusive AABB [roi_min, roi_max]. The fused-map topic carries the
  /// whole map; the planner re-applies its ROI clip here so map_cache_ stays
  /// bounded to the ROI as the old per-region GetRegion service made it (frontier
  /// extraction and map-stats both walk the whole grid, so the clip matters).
  /// Non-finite positions are dropped. NB: this clips on voxel position; the old
  /// service clipped in coord space, so results match for resolution-aligned ROI
  /// bounds and may differ by one voxel layer at a non-aligned min boundary.
  ///
  /// @return true if the grid was rebuilt. false means msg.resolution was
  ///         positive but not finite (+inf), which would make Bonxai's
  ///         inv_resolution zero and every posToCoord a float->int32 cast of a
  ///         non-finite value — UB that surfaces as garbage coordinates, not a
  ///         crash. Unlike the constructor and updateFromLogOddsCloud, which
  ///         throw on a bad resolution because that is a config error, this
  ///         value arrives over the wire mid-run: the previous grid is kept and
  ///         the message is dropped, so one malformed publish cannot take the
  ///         node down in the field. Callers should log it (throttled) and
  ///         treat it as "no new map this tick".
  bool updateFromScovoxMap(const scovox_msgs::msg::ScovoxMap& msg,
                           const Eigen::Vector3f& roi_min,
                           const Eigen::Vector3f& roi_max);

  /// Rebuild grid from LogOdds PointCloud2 (x,y,z,occupancy_prob).
  void updateFromLogOddsCloud(const sensor_msgs::msg::PointCloud2& msg,
                              double resolution);

  /// Query single voxel. Returns default Beta(1,1) prior if not in grid.
  UnifiedVoxel getVoxel(const Eigen::Vector3f& pos) const;

  /// Look up voxel by grid coordinate. Returns nullptr if not in grid.
  const UnifiedVoxel* getVoxelByCoord(const CoordT& c) const;

  const Grid& grid() const { return *grid_; }
  double resolution() const { return resolution_; }
  size_t voxelCount() const;
  bool empty() const;

  CoordT posToCoord(const Eigen::Vector3f& pos) const;
  Eigen::Vector3f coordToPos(const CoordT& c) const;

  /// Find frontier centroids: free voxels adjacent to unknown space.
  std::vector<Eigen::Vector3f> findFrontierCentroids(
      float min_z, float max_z, float cluster_radius) const;

  /// Estimate the ground elevation at world (x, y): the z of the top FACE of
  /// the ground voxel stack in that column. The search scans the column from
  /// z_low upward to z_high; the FIRST (lowest) occupied voxel
  /// (p_occ >= occ_thresh) anchors the ground, then the walk continues up
  /// through contiguous occupied voxels for at most stack_max_m (absorbs the
  /// residual vertical measurement smear without climbing walls/trunks) and
  /// the top face of the last stack voxel is returned. Lowest-first anchoring
  /// makes the estimate robust to canopy/overhangs higher in the column.
  /// Returns NaN when the window contains no occupied voxel (unobserved or
  /// free-only column).
  float groundZAt(float x, float y, float z_low, float z_high,
                  float occ_thresh, float stack_max_m) const;

  /// Aggregate per-voxel statistics over the whole (already ROI-clipped) grid:
  /// mean expected-information-gain, mean entropy, mean Beta variance, and the
  /// frontier-voxel count (free voxels with >=1 unknown 6-neighbour). Walks
  /// every active cell once. Extracted from the planner's LOG_STEP so it is
  /// unit-testable and shares the frontier-neighbour logic with
  /// findFrontierCentroids.
  struct MapStats {
    int   total_voxels    = 0;
    int   frontier_voxels = 0;
    float mean_eig        = 0.0f;
    float mean_entropy    = 0.0f;
    float mean_variance   = 0.0f;
  };
  MapStats computeStats() const;

  /// Fraction of x/y columns inside [min_x, max_x] x [min_y, max_y] that
  /// contain NO observed voxel — a 2.5D "area coverage" measure over the
  /// (already z-clipped) fused 3D map. A column counts as observed when any
  /// grid cell (free OR occupied) projects into it, so trunk columns and
  /// swept free space both count as covered; only never-seen ground-plane
  /// cells raise the fraction. This deliberately ignores volumetric unknowns
  /// (trunk interiors, canopy shadow are never observed and would put a
  /// permanent floor under a 3D unknown fraction), matching the semantics of
  /// the 2D planning_map unknown fraction used for coverage termination.
  /// Bounds are mapped to coord space with the same floor() convention as
  /// voxel ingest. Returns a value in [0, 1]; an empty grid gives 1.0.
  /// Returns -1.0 on a degenerate box (max <= min) or non-finite bounds —
  /// the caller's "cannot measure" convention.
  double unknownColumnFraction(float min_x, float max_x,
                               float min_y, float max_y) const;

private:
  double resolution_;
  std::unique_ptr<Grid> grid_;
};

} // namespace explo_planner
