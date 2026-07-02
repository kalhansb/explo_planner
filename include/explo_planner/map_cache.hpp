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
  void updateFromScovoxMap(const scovox_msgs::msg::ScovoxMap& msg);

  /// Rebuild grid from a ScovoxMap, keeping only voxels whose position lies in
  /// the inclusive AABB [roi_min, roi_max]. The fused-map topic carries the
  /// whole map; the planner re-applies its ROI clip here so map_cache_ stays
  /// bounded to the ROI as the old per-region GetRegion service made it (frontier
  /// extraction and map-stats both walk the whole grid, so the clip matters).
  /// Non-finite positions are dropped. NB: this clips on voxel position; the old
  /// service clipped in coord space, so results match for resolution-aligned ROI
  /// bounds and may differ by one voxel layer at a non-aligned min boundary.
  void updateFromScovoxMap(const scovox_msgs::msg::ScovoxMap& msg,
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

private:
  double resolution_;
  std::unique_ptr<Grid> grid_;
};

} // namespace explo_planner
