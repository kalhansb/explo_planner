#include <gtest/gtest.h>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explo_planner/plan_map_query.hpp"

using namespace explo_planner;

namespace {

// Build a planning_map at a given resolution and origin. Every cell defaults
// to free (value 0); set individual cells via the returned grid's data[].
nav_msgs::msg::OccupancyGrid makeGrid(int width, int height, float resolution,
                                      float origin_x = 0.0f,
                                      float origin_y = 0.0f) {
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = width;
  g.info.height = height;
  g.info.resolution = resolution;
  g.info.origin.position.x = origin_x;
  g.info.origin.position.y = origin_y;
  g.data.assign(static_cast<size_t>(width) * height, 0);
  return g;
}

void setCell(nav_msgs::msg::OccupancyGrid& g, int gx, int gy, int8_t v) {
  g.data[gy * g.info.width + gx] = v;
}

Eigen::Vector3f cellCenter(const nav_msgs::msg::OccupancyGrid& g,
                           int gx, int gy) {
  return Eigen::Vector3f(
      static_cast<float>(g.info.origin.position.x +
                         (gx + 0.5f) * g.info.resolution),
      static_cast<float>(g.info.origin.position.y +
                         (gy + 0.5f) * g.info.resolution),
      0.0f);
}

}  // namespace

// planMapCellAt returns the raw value in bounds, kCellNoData out of bounds.
TEST(PlanMapQuery, CellAtInBoundsAndOut) {
  auto g = makeGrid(10, 10, 1.0f, -5.0f, -5.0f);  // origin offset exercised
  setCell(g, 3, 4, 42);
  EXPECT_EQ(planMapCellAt(g, cellCenter(g, 3, 4)), 42);
  EXPECT_EQ(planMapCellAt(g, cellCenter(g, 0, 0)), 0);
  // Far outside the grid extents -> sentinel, not a garbage index.
  EXPECT_EQ(planMapCellAt(g, Eigen::Vector3f(1000.0f, 0.0f, 0.0f)),
            kCellNoData);
  EXPECT_EQ(planMapCellAt(g, Eigen::Vector3f(0.0f, -1000.0f, 0.0f)),
            kCellNoData);
}

// A degenerate grid (no cells / zero resolution) never indexes data[].
TEST(PlanMapQuery, DegenerateGridIsNoData) {
  auto empty = makeGrid(0, 0, 1.0f);
  EXPECT_EQ(planMapCellAt(empty, Eigen::Vector3f::Zero()), kCellNoData);

  auto zero_res = makeGrid(5, 5, 0.0f);
  EXPECT_EQ(planMapCellAt(zero_res, Eigen::Vector3f::Zero()), kCellNoData);
  EXPECT_FALSE(isCellFree(zero_res, Eigen::Vector3f::Zero()));
  EXPECT_TRUE(isCellOccupied(zero_res, Eigen::Vector3f::Zero()));
}

// isCellFree: only known-free (0..49) passes; unknown/occupied/OOB fail.
TEST(PlanMapQuery, IsCellFreeClassification) {
  auto g = makeGrid(5, 5, 1.0f);
  setCell(g, 1, 1, 0);    // free
  setCell(g, 2, 2, 49);   // free (just under threshold)
  setCell(g, 3, 3, 50);   // occupied (threshold)
  setCell(g, 4, 4, 100);  // occupied
  setCell(g, 0, 0, -1);   // unknown

  EXPECT_TRUE(isCellFree(g, cellCenter(g, 1, 1)));
  EXPECT_TRUE(isCellFree(g, cellCenter(g, 2, 2)));
  EXPECT_FALSE(isCellFree(g, cellCenter(g, 3, 3)));
  EXPECT_FALSE(isCellFree(g, cellCenter(g, 4, 4)));
  EXPECT_FALSE(isCellFree(g, cellCenter(g, 0, 0)));
  EXPECT_FALSE(isCellFree(g, Eigen::Vector3f(1000.0f, 0.0f, 0.0f)));
}

// isCellOccupied: only >=50 occupied; unknown is NOT occupied (frontiers pass
// through), but OOB / no-data IS occupied (conservative).
TEST(PlanMapQuery, IsCellOccupiedClassification) {
  auto g = makeGrid(5, 5, 1.0f);
  setCell(g, 3, 3, 50);
  setCell(g, 4, 4, 100);
  setCell(g, 0, 0, -1);   // unknown
  setCell(g, 1, 1, 0);    // free

  EXPECT_TRUE(isCellOccupied(g, cellCenter(g, 3, 3)));
  EXPECT_TRUE(isCellOccupied(g, cellCenter(g, 4, 4)));
  EXPECT_FALSE(isCellOccupied(g, cellCenter(g, 0, 0)));  // unknown passes
  EXPECT_FALSE(isCellOccupied(g, cellCenter(g, 1, 1)));
  EXPECT_TRUE(isCellOccupied(g, Eigen::Vector3f(1000.0f, 0.0f, 0.0f)));
}

// All-free / all-unknown grids give the bounding fractions.
TEST(PlanMapQuery, UnknownFractionExtremes) {
  auto g = makeGrid(10, 10, 1.0f);
  Roi2D roi{0.0f, 10.0f, 0.0f, 10.0f};  // whole grid
  EXPECT_NEAR(unknownFractionInRoi(g, roi), 0.0, 1e-9);

  for (auto& v : g.data) v = -1;
  EXPECT_NEAR(unknownFractionInRoi(g, roi), 1.0, 1e-9);
}

// A known unknown sub-block yields the expected fraction over the ROI.
TEST(PlanMapQuery, UnknownFractionPartial) {
  auto g = makeGrid(10, 10, 1.0f);
  // 5x5 = 25 unknown cells out of 100.
  for (int gy = 0; gy < 5; ++gy)
    for (int gx = 0; gx < 5; ++gx)
      setCell(g, gx, gy, -1);

  EXPECT_NEAR(unknownFractionInRoi(g, {0.0f, 10.0f, 0.0f, 10.0f}), 0.25, 1e-9);
  // ROI fully inside the unknown block (cells 1..3 in x and y) -> all unknown.
  // (The ROI->index mapping is inclusive of the max edge, so we stay clear of
  // the block boundary at cell 4 rather than asserting over it.)
  EXPECT_NEAR(unknownFractionInRoi(g, {1.0f, 3.0f, 1.0f, 3.0f}), 1.0, 1e-9);
}

// ROI is clipped to the grid; a non-overlapping ROI returns -1.0.
TEST(PlanMapQuery, UnknownFractionRoiClippingAndMiss) {
  auto g = makeGrid(10, 10, 1.0f);
  // ROI extends past the grid on the high side -> still valid, clipped.
  EXPECT_NEAR(unknownFractionInRoi(g, {0.0f, 100.0f, 0.0f, 100.0f}), 0.0, 1e-9);
  // ROI entirely outside the grid -> no overlap.
  EXPECT_EQ(unknownFractionInRoi(g, {-50.0f, -40.0f, 0.0f, 10.0f}), -1.0);

  auto empty = makeGrid(0, 0, 1.0f);
  EXPECT_EQ(unknownFractionInRoi(empty, {0.0f, 1.0f, 0.0f, 1.0f}), -1.0);
}
