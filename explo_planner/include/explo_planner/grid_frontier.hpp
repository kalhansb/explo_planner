#pragma once
/// @file grid_frontier.hpp
/// @brief 2D exploration on the fused planning_map (planner_mode 2d_frontier /
///        2d_raycast): frontier extraction and a ray-cast unknown-area gain.
///
/// Pure functions of an OccupancyGrid, like plan_map_query, so they are
/// testable on hand-built grids. Cell classes follow plan_map_query: -1
/// unknown, 0..49 free, >= 50 occupied or inflated.

#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/plan_map_query.hpp"

namespace explo_planner {

/// Inclusive grid-index box. Empty when x0 > x1 or y0 > y1.
struct GridBox {
  int x0 = 0, x1 = -1, y0 = 0, y1 = -1;
  bool empty() const { return x0 > x1 || y0 > y1; }
  bool contains(int gx, int gy) const {
    return gx >= x0 && gx <= x1 && gy >= y0 && gy <= y1;
  }
};

/// Cells whose index floor()s into `roi`, clipped to the grid. Empty on a
/// degenerate grid or one whose data size does not match width * height.
GridBox gridBoxOf(const nav_msgs::msg::OccupancyGrid& g, const Roi2D& roi);

/// The one definition of a 2D frontier cell, shared by the frontier search and
/// the 2D census: free, with an unknown 4-neighbour inside `box`. Unknown
/// outside the box does not count, so the ROI edge is not a frontier.
bool isGridFrontierCell(const nav_msgs::msg::OccupancyGrid& g, int gx, int gy,
                        const GridBox& box);

/// One frontier candidate: the member cell nearest its group's centroid (so it
/// is a free cell, not a point in unknown space) and the group's size.
struct GridFrontier {
  Eigen::Vector3f position;  ///< cell centre, z = 0
  int cells = 0;
};

/// Frontier cells inside `roi`, grouped by 8-connectivity; components under
/// min_cells are dropped as speckle. Each component is then split into
/// bin_m x bin_m bins, so a long frontier yields one candidate per bin like the
/// 3D frontier centroids. Scan order is deterministic.
std::vector<GridFrontier> findGridFrontiers(
    const nav_msgs::msg::OccupancyGrid& g, const Roi2D& roi, float bin_m,
    int min_cells);

/// Distinct unknown cells inside `roi` seen from `pos`: rays spread evenly over
/// hfov about yaw, each walked cell by cell out to max_range and stopped by the
/// first occupied cell or the grid edge. Unknown cells do not stop a ray;
/// cells nearer than min_range are not counted. At least enough rays are cast
/// to space them one cell apart at max_range, so far cells are not skipped.
int gridVisibleUnknown(const nav_msgs::msg::OccupancyGrid& g,
                       const Roi2D& roi, const Eigen::Vector3f& pos, float yaw,
                       float hfov, int h_rays, float min_range,
                       float max_range);

/// censusFromMap's 2D counterpart, measured on the planning_map. A column is a
/// grid cell whose centre lies in the cell and is observed when known (not -1);
/// frontier_voxels counts isGridFrontierCell cells, over observed_voxels = the
/// observed columns. The frontier test sees unknown only inside the cell grid,
/// as findGridFrontiers sees it only inside the ROI the grid is built from.
/// All cells unmeasured (total_columns 0) on a degenerate grid.
std::vector<CellWorld::CellObservation> censusFromPlanMap(
    const nav_msgs::msg::OccupancyGrid& plan_map, const CellGrid& grid);

} // namespace explo_planner
