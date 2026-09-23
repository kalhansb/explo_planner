#pragma once
/// @file map_cache.hpp
/// @brief Read-only Bonxai grid rebuilt from ROS map messages.
/// Moved comments: doc/explo_planner_code_notes.md

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

  /// Rebuild keeping voxels whose position is inside the inclusive AABB
  /// [roi_min, roi_max]; non-finite positions dropped. Returns false, keeping
  /// the previous grid, if msg.resolution is +inf; callers log it throttled.
  /// (notes: map-cache-roi-rebuild)
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

  /// Ground z at (x, y): scanning z_low up to z_high, the lowest voxel with
  /// p_occ >= occ_thresh anchors the ground; climb its contiguous stack at most
  /// stack_max_m, return the top face. NaN if none. (notes: map-cache-ground-z)
  float groundZAt(float x, float y, float z_low, float z_high,
                  float occ_thresh, float stack_max_m) const;

  /// Means of EIG, entropy and Beta variance over the already ROI-clipped grid,
  /// plus the frontier-voxel count (free voxels with an unknown 6-neighbour),
  /// sharing that test with findFrontierCentroids. (notes: map-cache-stats)
  struct MapStats {
    int   total_voxels    = 0;
    int   frontier_voxels = 0;
    float mean_eig        = 0.0f;
    float mean_entropy    = 0.0f;
    float mean_variance   = 0.0f;
  };
  MapStats computeStats() const;

  /// Fraction of x/y columns in the box with no observed voxel (free or
  /// occupied), matching the 2D planning_map unknown fraction. In [0, 1]; empty
  /// grid 1.0; -1.0 for a degenerate or non-finite box.
  /// (notes: map-cache-unknown-columns)
  double unknownColumnFraction(float min_x, float max_x,
                               float min_y, float max_y) const;

private:
  double resolution_;
  std::unique_ptr<Grid> grid_;
};

} // namespace explo_planner
