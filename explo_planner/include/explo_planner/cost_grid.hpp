#pragma once
/// @file cost_grid.hpp
/// @brief Bounded 8-connected grid Dijkstra over the planning_map. Used as
///        the path-cost backend for the per-candidate utility function
///        U = alpha * info_gain - beta * cost_grid_.costTo(c.pos).
///
/// One single-source flood from the robot pose, computed once per PLAN tick,
/// then O(1) lookup per candidate. Replaces straight-line distance everywhere
/// inside the planner: the math (~5 k cells flooded * log(5 k) ~ 60 k ops on
/// commodity x86, well under 1 ms) is below 2 % of the existing FOV raycast
/// budget per PLAN tick.
///
/// Doubles as a reachability filter: a candidate sitting in a free pocket
/// surrounded by inflated obstacles will return kInfCost from costTo(...) and
/// false from reachable(...), so the planner can skip it before publishing a
/// goal that the navigator would never reach.
/// Moved comments: doc/explo_planner_code_notes.md

#include <Eigen/Core>
#include <limits>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace explo_planner {

class CostGrid {
public:
  /// Sentinel for unreachable / out-of-bounds / outside-the-flood-radius
  /// cells. Always > any finite cost the flood will produce.
  static constexpr float kInfCost = std::numeric_limits<float>::infinity();

  CostGrid() = default;

  /// Blocks a cell iff known and >= obstacle_threshold (the isCellFree
  /// threshold). Unknown (-1) is traversable on purpose; isCellFree() refuses
  /// unknown goals. Do not make the two filters consistent. Clears any prior
  /// flood. (notes: costgrid-build-unknown-traversable)
  void build(const nav_msgs::msg::OccupancyGrid& planning_map,
             int8_t obstacle_threshold = 50);

  /// Bounded Dijkstra from source_xy; cells beyond radius_cap_m of walked
  /// distance keep kInfCost. radius_cap_m <= 0 or NaN means unbounded; the grid
  /// diagonal is not a safe bound. Diagonal steps cost sqrt(2) * resolution.
  /// (notes: costgrid-flood-radius-cap)
  void floodFrom(const Eigen::Vector3f& source_xy, float radius_cap_m);

  /// O(1) lookup of the shortest-path distance from the source used in the
  /// most recent floodFrom(...) call to the cell containing `xy`. Returns
  /// kInfCost if the cell is impassable, unreachable, outside the bounded
  /// flood radius, or out of bounds.
  float costTo(const Eigen::Vector3f& xy) const;

  /// Walk parent pointers from the flood source to `goal_xy` and return
  /// the path as a sequence of world-frame XY positions (Z = 0).
  /// Returns an empty vector if the goal is unreachable or out of bounds.
  /// The path starts at the flood source and ends at the goal cell centre.
  std::vector<Eigen::Vector3f> extractPath(const Eigen::Vector3f& goal_xy) const;

  /// True iff costTo(xy) < kInfCost. The "reachability bonus" candidate
  /// filter on top of single-cell isCellFree() — catches free pockets that
  /// have no path from the current pose.
  bool reachable(const Eigen::Vector3f& xy) const;

  // Diagnostic accessors. Used by tests; not load-bearing for the planner.
  int dimsX() const { return dims_x_; }
  int dimsY() const { return dims_y_; }
  float resolution() const { return resolution_; }
  bool blockedAt(int gx, int gy) const;
  size_t reachedCellCount() const;

private:
  /// Backing storage for the most recent flood. Sized to dims_x_ * dims_y_.
  /// cost_[gy * dims_x_ + gx] is the shortest path from the flood source to
  /// (gx, gy), or kInfCost if not reached.
  std::vector<float> cost_;

  /// Parent pointers for path reconstruction. parent_[i] is the flat index
  /// of the predecessor on the shortest path, or -1 for the source cell.
  std::vector<int> parent_;

  /// Obstacle layer from build(): set for known cells >= obstacle_threshold;
  /// unknown (-1) is not blocked. If data.size() != dims_x_ * dims_y_, every
  /// cell is blocked so nothing is reachable. (notes: costgrid-blocked-layer)
  std::vector<uint8_t> blocked_;

  int dims_x_ = 0;
  int dims_y_ = 0;
  float resolution_ = 0.0f;
  float origin_x_ = 0.0f;
  float origin_y_ = 0.0f;

  inline int idx(int gx, int gy) const { return gy * dims_x_ + gx; }
  inline bool inBounds(int gx, int gy) const {
    return gx >= 0 && gy >= 0 && gx < dims_x_ && gy < dims_y_;
  }

  /// Convert world XY into integer grid coordinates. Returns false if the
  /// world point falls outside the captured grid extents.
  bool worldToGrid(float x, float y, int& gx, int& gy) const;
};

} // namespace explo_planner
