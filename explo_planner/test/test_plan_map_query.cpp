// Moved comments: doc/explo_planner_code_notes.md
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

// A grid whose metadata claims more cells than data[] holds must read as no
// data (kCellNoData, -1.0), never overread or invent a fraction:
// unknownFractionInRoi feeds the DONE criterion.
// (notes: planmap-short-data-buffer)
TEST(PlanMapQuery, ShortDataBufferIsNoDataNotAnOverread) {
  auto g = makeGrid(10, 10, 1.0f, -5.0f, -5.0f);
  setCell(g, 3, 4, 42);
  // Baseline: intact buffer, the cell reads back.
  ASSERT_EQ(planMapCellAt(g, cellCenter(g, 3, 4)), 42);
  ASSERT_NEAR(unknownFractionInRoi(g, {-5.0f, 5.0f, -5.0f, 5.0f}), 0.0, 1e-9);

  // Truncate the payload; leave info.width/info.height claiming 100 cells.
  g.data.resize(10);
  ASSERT_EQ(g.info.width * g.info.height, 100u);

  EXPECT_EQ(planMapCellAt(g, cellCenter(g, 3, 4)), kCellNoData);
  // Index 9 IS inside the surviving 10 bytes — the guard must reject on the
  // buffer disagreeing, not on the individual index happening to be unsafe,
  // or it only catches the mismatch at cells it would have crashed on anyway.
  EXPECT_EQ(planMapCellAt(g, cellCenter(g, 9, 0)), kCellNoData);
  EXPECT_FALSE(isCellFree(g, cellCenter(g, 3, 4)));
  EXPECT_TRUE(isCellOccupied(g, cellCenter(g, 3, 4)));  // conservative
  EXPECT_EQ(unknownFractionInRoi(g, {-5.0f, 5.0f, -5.0f, 5.0f}), -1.0);

  // An oversized buffer is just as much a disagreement as a short one: the
  // metadata is what the ROI loop trusts, so a grid that does not match it is
  // not a grid this file can read, whichever direction it differs in.
  auto big = makeGrid(10, 10, 1.0f);
  big.data.resize(200);
  EXPECT_EQ(planMapCellAt(big, cellCenter(big, 3, 4)), kCellNoData);
  EXPECT_EQ(unknownFractionInRoi(big, {0.0f, 10.0f, 0.0f, 10.0f}), -1.0);
}

// §11.3: a robot standing at either end of a sight ray is mapped as an
// obstacle. The ray skips the last end_clear_m at each end, so a braked peer
// does not block every ray to it; a wall in the middle still does.
TEST(PlanMapQuery, SegmentClearSkipsTheEndsNotTheMiddle) {
  auto g = makeGrid(40, 10, 0.2f);   // 8 m x 2 m, free
  // Cell centres, not cell edges: y = 1.0 lands in row 4, not 5, because the
  // float resolution 0.2f is a hair over 0.2.
  const Eigen::Vector3f a = cellCenter(g, 2, 5), b = cellCenter(g, 37, 5);
  EXPECT_TRUE(segmentClear(g, a, b, 0.5));
  // The peer's own cell at b, and the asker's at a: still clear.
  setCell(g, 37, 5, 100);   // x 7.4-7.6, holds b
  setCell(g, 2, 5, 100);    // x 0.4-0.6, holds a
  EXPECT_TRUE(segmentClear(g, a, b, 0.5));
  // Without the clearance the endpoint cell blocks.
  EXPECT_FALSE(segmentClear(g, a, b, 0.0));
  // A wall midway blocks with or without it.
  setCell(g, 20, 5, 100);
  EXPECT_FALSE(segmentClear(g, a, b, 0.5));
  // Unknown does not block.
  auto u = makeGrid(40, 10, 0.2f);
  setCell(u, 20, 5, -1);
  EXPECT_TRUE(segmentClear(u, a, b, 0.5));
}

// The sim's plan map inflates obstacles by 1.5 m, so a mapped robot is a disc
// about 2 m in radius around it. The occupied run touching an end is skipped
// up to end_clear_m; past the first free sample an obstacle blocks, even
// inside end_clear_m, and a run longer than end_clear_m blocks too.
TEST(PlanMapQuery, SegmentClearSkipsAnInflatedRobotButNotAWallBeyondIt) {
  auto disc = [](nav_msgs::msg::OccupancyGrid& g, int cx, int cy, int r) {
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx)
        if (dx * dx + dy * dy <= r * r) setCell(g, cx + dx, cy + dy, 100);
  };
  auto g = makeGrid(50, 20, 0.4f);   // 20 m x 8 m, free
  const Eigen::Vector3f a = cellCenter(g, 5, 10), b = cellCenter(g, 20, 10);
  disc(g, 5, 10, 5);                 // 2 m around a
  disc(g, 20, 10, 5);                // and around b, 6 m away
  EXPECT_FALSE(segmentClear(g, a, b, 0.5));
  EXPECT_TRUE(segmentClear(g, a, b, 3.0));
  // A trunk 2.8 m from a, past a free cell: inside end_clear_m, yet it blocks.
  auto t = g;
  setCell(t, 12, 10, 100);
  EXPECT_FALSE(segmentClear(t, a, b, 3.0));
  // A run from a longer than end_clear_m blocks. (With the ends under
  // 2 * end_clear_m apart the two skips would meet, and the ray reads clear.)
  auto w = makeGrid(50, 20, 0.4f);
  const Eigen::Vector3f c = cellCenter(w, 30, 10);   // 10 m from a
  disc(w, 5, 10, 9);                 // 3.6 m around a
  disc(w, 30, 10, 5);
  EXPECT_FALSE(segmentClear(w, a, c, 3.0));
  EXPECT_TRUE(segmentClear(w, a, c, 4.0));
}
