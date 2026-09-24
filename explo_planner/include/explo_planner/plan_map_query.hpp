#pragma once
/// @file plan_map_query.hpp
/// @brief Pure 2D occupancy-grid (planning_map) cell queries.
///
/// These are the world->grid lookups the exploration planner runs against the
/// latched planning_map: single-cell free/occupied classification and the ROI
/// unknown-fraction used by the coverage-termination check. They are free
/// functions of an OccupancyGrid (no ROS node, no node state) so the geometry
/// is deterministic and unit-testable on hand-built grids.
///
/// The node-side callers guard on a non-null map before calling; these
/// functions only handle the in-grid / out-of-bounds cases.

#include <cstdint>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace explo_planner {

/// Axis-aligned region of interest in world XY (metres). The 2D planning_map
/// queries ignore Z.
struct Roi2D {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
};

/// Sentinel for planMapCellAt(): no data — empty/degenerate grid or `pos` out
/// of bounds. Distinct from the real cell range (-1 unknown, 0..100 occupancy).
inline constexpr int8_t kCellNoData = -2;

/// Raw occupancy value at world XY (-1 unknown, 0..100 cost), or kCellNoData
/// when the grid is degenerate or `pos` is out of bounds.
int8_t planMapCellAt(const nav_msgs::msg::OccupancyGrid& grid,
                     const Eigen::Vector3f& pos);

/// True iff the cell at `pos` is known-free. Out-of-bounds, unknown (-1) and
/// occupied/inflated (>= 50) cells all fail. The map is already inflated by the
/// body radius, so a single-cell check is enough.
bool isCellFree(const nav_msgs::msg::OccupancyGrid& grid,
                const Eigen::Vector3f& pos);

/// True iff the cell at `pos` is known-occupied/inflated (>= 50). Unlike
/// isCellFree, unknown (-1) cells return false (lets frontier centroids pass
/// through unknown territory); out-of-bounds (kCellNoData) is treated as
/// occupied, the conservative default.
bool isCellOccupied(const nav_msgs::msg::OccupancyGrid& grid,
                    const Eigen::Vector3f& pos);

/// No occupied cell (isCellOccupied) on the XY segment a -> b, sampled every
/// 0.2 m at a's Z. At each end, the run of occupied samples that starts at the
/// end is skipped for up to `end_clear_m`, so a robot standing there (mapped
/// as an obstacle, then inflated) does not block the ray; an occupied sample
/// after the first free one always blocks. Ends under 2 * end_clear_m apart
/// whose runs meet read clear: the map cannot tell two robots' discs from a
/// wall between them. Unknown cells do not block.
bool segmentClear(const nav_msgs::msg::OccupancyGrid& grid,
                  const Eigen::Vector3f& a, const Eigen::Vector3f& b,
                  double end_clear_m);

/// Fraction of cells in `roi` (clipped to the grid) whose value is -1
/// (unknown). Returns -1.0 if the grid is degenerate or the ROI doesn't
/// overlap it.
double unknownFractionInRoi(const nav_msgs::msg::OccupancyGrid& grid,
                            const Roi2D& roi);

} // namespace explo_planner
