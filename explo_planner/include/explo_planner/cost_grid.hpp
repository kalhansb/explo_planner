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

  /// Resample the cost grid from a fresh planning_map.
  ///
  /// `obstacle_threshold` matches the planner's existing isCellFree threshold.
  /// A cell is impassable iff it is KNOWN and at or above the threshold, i.e.
  /// `v >= 0 && v >= obstacle_threshold` (`cost_grid.cpp:61-66`).
  ///
  /// UNKNOWN (-1) IS TRAVERSABLE. This is deliberate and is the opposite of
  /// what this comment used to claim. The flood's job is to find free pockets
  /// sealed off by *known* obstacles; it is not a free-space test. Blocking
  /// unknown would make every frontier a wall and the reachability filter
  /// would reject exactly the candidates exploration exists to reach. The
  /// per-candidate `isCellFree()` check in the node is what refuses to stand
  /// a robot on an unknown cell — the two filters have different jobs and
  /// must not be "made consistent".
  /// Origin / resolution / dims are captured from the OccupancyGrid so the
  /// caller can convert candidate world XY to grid coords without keeping a
  /// pointer to the original message.
  ///
  /// Calling build(...) clears any prior flood result.
  void build(const nav_msgs::msg::OccupancyGrid& planning_map,
             int8_t obstacle_threshold = 50);

  /// Run a single-source bounded flood from `source_xy`. Cells whose
  /// shortest-path distance from the source exceeds `radius_cap_m` are not
  /// touched and keep the sentinel kInfCost.
  ///
  /// Setting `radius_cap_m <= 0` (or NaN) runs a genuinely unbounded flood —
  /// the cap becomes +inf. THE GRID DIAGONAL IS NOT AN EQUIVALENT BOUND: these
  /// are *walked* Dijkstra distances, which routinely exceed the straight-line
  /// diagonal around serpentine corridors and U-shaped obstacles, so a cap set
  /// at the diagonal still leaves reachable cells at kInfCost and callers still
  /// read them as unreachable. That was a real defect; see the note at the cap
  /// computation in cost_grid.cpp. The bound is what makes this <1 ms per PLAN
  /// tick on the live system.
  ///
  /// Diagonal moves cost sqrt(2) * resolution; orthogonal cost 1 * resolution.
  /// Out-of-bounds source returns cleanly: every cell stays at kInfCost.
  ///
  /// Calling floodFrom(...) again replaces the previous flood result without
  /// rebuilding the obstacle layer.
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

  /// Obstacle layer sampled from the OccupancyGrid at build() time.
  /// blocked_[gy * dims_x_ + gx] == true for cells that the flood will not
  /// enter: KNOWN cells at or above `obstacle_threshold` (occupied or
  /// inflated). Unknown (-1) cells are NOT blocked — see build() above.
  /// The one exception is a malformed message whose `data.size()` disagrees
  /// with `dims_x_ * dims_y_`, where every cell is marked blocked so the
  /// flood is empty and the caller degrades to "nothing reachable" instead
  /// of indexing past the end of the array.
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
