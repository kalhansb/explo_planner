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
#include <cstdint>
#include <limits>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace explo_planner {

/// Optional cost shaping on top of the binary free/blocked layer. The defaults
/// reproduce the legacy model exactly: every passable cell costs its metric
/// step length and costTo() is a distance in metres.
struct CostModel {
  /// Traversability weighting. A passable cell holding value v in
  /// [0, obstacle_threshold) costs step * (1 + trav_weight * v / 100). On a
  /// graded traversability map (0 = flat, 100 = wall) rough or steep ground
  /// then counts as longer than open ground. Unknown cells (-1) count as
  /// v = unknown_value.
  float trav_weight = 0.0f;
  /// Value an unseen (-1) cell is weighted as, in [0, 100). 0 makes unseen
  /// ground as cheap as open ground, so a path cuts through unmapped scrub
  /// rather than skirting a mapped slope. Only acts with trav_weight > 0.
  float unknown_value = 0.0f;
  /// Clearance zone: passable cells within clearance_m of a blocked cell. They
  /// stay passable — a robot that starts beside a bush must be able to leave —
  /// but cost clearance_penalty times their step. 0 = no zone.
  float clearance_m = 0.0f;
  float clearance_penalty = 1.0f;

  bool shaped() const {
    return trav_weight > 0.0f || (clearance_m > 0.0f && clearance_penalty != 1.0f);
  }
};

class CostGrid {
public:
  /// Sentinel for unreachable / out-of-bounds / outside-the-flood-radius
  /// cells. Always > any finite cost the flood will produce.
  static constexpr float kInfCost = std::numeric_limits<float>::infinity();

  CostGrid() = default;

  /// Resample the cost grid from a fresh planning_map.
  ///
  /// `obstacle_threshold` matches the planner's isCellFree threshold: values
  /// >= threshold are impassable; unknown (-1) cells stay passable.
  /// Origin / resolution / dims are captured from the OccupancyGrid so the
  /// caller can convert candidate world XY to grid coords without keeping a
  /// pointer to the original message.
  ///
  /// `model` shapes the per-cell step cost (see CostModel); with a shaped
  /// model costTo() is a weighted cost in metre-equivalents, not a distance.
  /// The clearance zone is computed whenever model.clearance_m > 0, penalised
  /// or not, so inClearanceZone() can screen goals.
  ///
  /// Calling build(...) clears any prior flood result. O(cells): one pass to
  /// sample the layers, plus a two-pass chamfer distance transform when the
  /// clearance zone is on.
  void build(const nav_msgs::msg::OccupancyGrid& planning_map,
             int8_t obstacle_threshold = 50,
             const CostModel& model = CostModel{});

  /// Run a single-source bounded flood from `source_xy`. Cells whose
  /// shortest-path distance from the source exceeds `radius_cap_m` are not
  /// touched and keep the sentinel kInfCost.
  ///
  /// Setting `radius_cap_m <= 0` (or any value larger than the grid diagonal)
  /// effectively runs an unbounded flood. The bound is what makes this
  /// <1 ms per PLAN tick on the live system.
  ///
  /// Diagonal moves cost sqrt(2) * resolution; orthogonal cost 1 * resolution.
  /// Out-of-bounds source returns cleanly: every cell stays at kInfCost.
  ///
  /// Calling floodFrom(...) again replaces the previous flood result without
  /// rebuilding the obstacle layer.
  ///
  /// `source_clear_m` > 0 makes blocked cells within that distance of the
  /// source cell passable for this flood only. The source is the robot, and
  /// it is standing there: a LiDAR that sees its own body draws a ring of
  /// obstacles around a parked robot (traversability_mapping drops returns
  /// only inside 0.5 m), which otherwise walls the flood in. The cleared
  /// cells stay blocked for blockedAt()/goal checks.
  void floodFrom(const Eigen::Vector3f& source_xy, float radius_cap_m,
                 float source_clear_m = 0.0f);

  /// O(1) lookup of the shortest-path cost from the source used in the most
  /// recent floodFrom(...) call to the cell containing `xy` (metres unless the
  /// CostModel is shaped). Returns
  /// kInfCost if the cell is impassable, unreachable, outside the bounded
  /// flood radius, or out of bounds.
  float costTo(const Eigen::Vector3f& xy) const;

  /// Metric length (m) of the most recent flood's shortest path to the cell
  /// containing `xy`: what the robot drives, where costTo() is the shaped cost
  /// the path minimises. Equal to costTo() for an unshaped CostModel. kInfCost
  /// wherever costTo() is. O(path cells): walks the parent pointers.
  float pathLengthTo(const Eigen::Vector3f& xy) const;

  /// Walk parent pointers from the flood source to `goal_xy` and return
  /// the path as a sequence of world-frame XY positions (Z = 0).
  /// Returns an empty vector if the goal is unreachable or out of bounds.
  /// The path starts at the flood source and ends at the goal cell centre.
  std::vector<Eigen::Vector3f> extractPath(const Eigen::Vector3f& goal_xy) const;

  /// True iff costTo(xy) < kInfCost. The "reachability bonus" candidate
  /// filter on top of single-cell isCellFree() — catches free pockets that
  /// have no path from the current pose.
  bool reachable(const Eigen::Vector3f& xy) const;

  // Cell-level queries for goal placement (CandidateSnapper). Out-of-bounds
  // cells read as blocked, matching costTo().
  bool cellAt(const Eigen::Vector3f& xy, int& gx, int& gy) const {
    return worldToGrid(xy.x(), xy.y(), gx, gy);
  }
  Eigen::Vector2f cellCenter(int gx, int gy) const {
    return {origin_x_ + (static_cast<float>(gx) + 0.5f) * resolution_,
            origin_y_ + (static_cast<float>(gy) + 0.5f) * resolution_};
  }
  bool inBounds(int gx, int gy) const {
    return gx >= 0 && gy >= 0 && gx < dims_x_ && gy < dims_y_;
  }
  bool unknownAt(int gx, int gy) const;
  /// True for passable cells within CostModel::clearance_m of a blocked cell.
  bool inClearanceZoneAt(int gx, int gy) const;
  bool inClearanceZone(const Eigen::Vector3f& xy) const;
  float costAt(int gx, int gy) const {
    return inBounds(gx, gy) ? cost_[idx(gx, gy)] : kInfCost;
  }

  // Diagnostic accessors. Used by tests; not load-bearing for the planner.
  int dimsX() const { return dims_x_; }
  int dimsY() const { return dims_y_; }
  float resolution() const { return resolution_; }
  bool blockedAt(int gx, int gy) const;
  /// Cells the last flood reached outside its source_clear_m disc (all
  /// reached cells, source included, when it had none).
  size_t reachedCellCount() const;

private:
  /// Backing storage for the most recent flood. Sized to dims_x_ * dims_y_.
  /// cost_[gy * dims_x_ + gx] is the shortest path from the flood source to
  /// (gx, gy), or kInfCost if not reached.
  std::vector<float> cost_;

  /// Parent pointers for path reconstruction. parent_[i] is the flat index
  /// of the predecessor on the shortest path, or -1 for the source cell.
  std::vector<int> parent_;

  /// Cell layer sampled from the OccupancyGrid at build() time, as kFlag*
  /// bits. kFlagBlocked marks cells the flood will not enter (value >=
  /// obstacle_threshold); kFlagUnknown marks -1 cells (passable);
  /// kFlagClearance marks passable cells inside the clearance zone.
  static constexpr uint8_t kFlagBlocked   = 1;
  static constexpr uint8_t kFlagUnknown   = 2;
  static constexpr uint8_t kFlagClearance = 4;
  std::vector<uint8_t> flags_;

  /// Per-cell step multiplier, populated only for a shaped CostModel (empty =
  /// every step costs its length).
  std::vector<float> step_mul_;

  /// Source cell and squared source_clear_m radius (cells) of the last flood.
  int flood_sx_ = 0;
  int flood_sy_ = 0;
  long flood_clear_r2_ = -1;

  int dims_x_ = 0;
  int dims_y_ = 0;
  float resolution_ = 0.0f;
  float origin_x_ = 0.0f;
  float origin_y_ = 0.0f;

  inline int idx(int gx, int gy) const { return gy * dims_x_ + gx; }

  /// Mark kFlagClearance on passable cells within `clearance_m` of a blocked
  /// cell (3-4 chamfer distance transform, <6 % from Euclidean).
  void markClearanceZone(float clearance_m);

  /// Convert world XY into integer grid coordinates. Returns false if the
  /// world point falls outside the captured grid extents.
  bool worldToGrid(float x, float y, int& gx, int& gy) const;
};

} // namespace explo_planner
