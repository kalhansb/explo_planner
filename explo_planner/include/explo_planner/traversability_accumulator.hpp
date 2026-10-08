#pragma once
/// @file traversability_accumulator.hpp
/// @brief Stitches a robot-centred rolling OccupancyGrid window into one fixed,
///        ROI-sized planning grid that remembers every cell the robot has seen.
///
/// traversability_mapping publishes /occupancy_map_local as a 20 x 20 m window
/// that moves with the robot (and nav2's costmap built from it is the same
/// window). The planner treats every cell outside its planning_map as blocked
/// (plan_map_query, CostGrid), so fed the raw window it would reject every
/// candidate more than ~10 m away. The accumulator keeps a grid over the whole
/// ROI instead: each window overwrites the cells it has seen, cells it has not
/// seen (-1) keep their earlier value, and never-seen cells stay unknown.
///
/// Values are kept as published (traversability_mapping: 0..100 cost, 100 =
/// hard obstacle). When the accumulator is coarser than the window, the
/// window cells falling in one accumulator cell are max-pooled, so a thin
/// obstacle survives downsampling.
///
/// Bounded by construction: the grid is sized once from the configured box and
/// never grows (the resolution is coarsened to fit max_cells), and a window
/// that is malformed, rotated or larger than max_window_cells is rejected
/// rather than integrated. Integration is O(window cells), allocation-free
/// after the first window.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace explo_planner {

struct AccumulatorConfig {
  /// Map-frame box the grid covers (metres).
  float min_x = 0.0f;
  float max_x = 0.0f;
  float min_y = 0.0f;
  float max_y = 0.0f;
  /// Requested cell size (metres). Coarsened if the box needs > max_cells.
  float resolution = 0.2f;
  size_t max_cells = 4000000;
  /// Windows with more cells than this are rejected (a 20 m window at 0.1 m is
  /// 40 k cells).
  size_t max_window_cells = 4000000;
};

class TraversabilityAccumulator {
public:
  enum class Status {
    kIntegrated,    ///< At least one known window cell landed in the grid.
    kNoOverlap,     ///< Window lies entirely outside the grid box.
    kNoKnownCells,  ///< Overlaps, but every overlapping window cell is -1.
    kMalformed,     ///< Bad resolution / dims / data size / non-finite origin.
    kTooLarge,      ///< More than max_window_cells cells.
    kRotated,       ///< Window origin is rotated; only axis-aligned windows
                    ///< (identity origin orientation) are supported.
  };
  static const char* statusName(Status s);

  explicit TraversabilityAccumulator(const AccumulatorConfig& cfg);

  Status integrate(const nav_msgs::msg::OccupancyGrid& window);

  /// Incremented whenever an integrate() changes at least one cell value.
  uint64_t version() const { return version_; }

  /// Fill `out.info` and `out.data` with the current grid. `out.header` is
  /// left to the caller.
  void toMsg(nav_msgs::msg::OccupancyGrid& out) const;

  int width() const { return width_; }
  int height() const { return height_; }
  float resolution() const { return resolution_; }
  float originX() const { return origin_x_; }
  float originY() const { return origin_y_; }
  /// True when the requested resolution was coarsened to fit max_cells.
  bool coarsened() const { return coarsened_; }
  size_t cellCount() const { return grid_.size(); }
  size_t knownCells() const { return known_cells_; }

private:
  float origin_x_ = 0.0f;
  float origin_y_ = 0.0f;
  float resolution_ = 0.0f;
  int width_ = 0;
  int height_ = 0;
  bool coarsened_ = false;
  size_t max_window_cells_ = 0;
  std::vector<int8_t> grid_;
  /// Per-integrate "already written by this window" marks over the window's
  /// clipped footprint (reused; sized to that footprint, never the grid).
  std::vector<uint8_t> touched_;
  size_t known_cells_ = 0;
  uint64_t version_ = 0;
};

}  // namespace explo_planner
