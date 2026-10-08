#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explo_planner/traversability_accumulator.hpp"

using namespace explo_planner;
using Status = TraversabilityAccumulator::Status;

namespace {

nav_msgs::msg::OccupancyGrid makeWindow(int w, int h, float res, double ox,
                                        double oy, int8_t fill) {
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.width = w;
  g.info.height = h;
  g.info.resolution = res;
  g.info.origin.position.x = ox;
  g.info.origin.position.y = oy;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(static_cast<size_t>(w) * h, fill);
  return g;
}

void setCell(nav_msgs::msg::OccupancyGrid& g, int gx, int gy, int8_t v) {
  g.data[static_cast<size_t>(gy) * g.info.width + gx] = v;
}

AccumulatorConfig box(float x0, float x1, float y0, float y1, float res) {
  AccumulatorConfig c;
  c.min_x = x0;
  c.max_x = x1;
  c.min_y = y0;
  c.max_y = y1;
  c.resolution = res;
  return c;
}

// Value of the accumulator cell containing world (x, y).
int8_t at(const TraversabilityAccumulator& acc, double x, double y) {
  nav_msgs::msg::OccupancyGrid m;
  acc.toMsg(m);
  const int gx = static_cast<int>(std::floor((x - m.info.origin.position.x) /
                                             m.info.resolution));
  const int gy = static_cast<int>(std::floor((y - m.info.origin.position.y) /
                                             m.info.resolution));
  if (gx < 0 || gy < 0 || gx >= static_cast<int>(m.info.width) ||
      gy >= static_cast<int>(m.info.height)) {
    return -2;
  }
  return m.data[static_cast<size_t>(gy) * m.info.width + gx];
}

}  // namespace

TEST(TraversabilityAccumulator, SizedFromBoxAndStartsUnknown) {
  TraversabilityAccumulator acc(box(-5.0f, 5.0f, 0.0f, 5.0f, 0.5f));
  EXPECT_EQ(acc.width(), 20);
  EXPECT_EQ(acc.height(), 10);
  EXPECT_FALSE(acc.coarsened());
  EXPECT_EQ(acc.knownCells(), 0u);
  EXPECT_EQ(acc.version(), 0u);
  nav_msgs::msg::OccupancyGrid m;
  acc.toMsg(m);
  EXPECT_EQ(m.info.width, 20u);
  EXPECT_EQ(m.info.height, 10u);
  EXPECT_FLOAT_EQ(m.info.resolution, 0.5f);
  EXPECT_DOUBLE_EQ(m.info.origin.position.x, -5.0);
  EXPECT_DOUBLE_EQ(m.info.origin.orientation.w, 1.0);
  ASSERT_EQ(m.data.size(), 200u);
  for (int8_t v : m.data) EXPECT_EQ(v, -1);
}

TEST(TraversabilityAccumulator, CoarsensToFitMaxCells) {
  auto cfg = box(0.0f, 100.0f, 0.0f, 100.0f, 0.1f);  // 1 M cells requested
  cfg.max_cells = 10000;
  TraversabilityAccumulator acc(cfg);
  EXPECT_TRUE(acc.coarsened());
  EXPECT_LE(acc.cellCount(), 10000u);
  EXPECT_GE(acc.resolution(), 1.0f);
  EXPECT_GE(acc.width() * acc.resolution(), 100.0f);
  EXPECT_GE(acc.height() * acc.resolution(), 100.0f);
}

TEST(TraversabilityAccumulator, DegenerateBoxIsEmpty) {
  TraversabilityAccumulator acc(box(1.0f, 1.0f, 0.0f, 5.0f, 0.5f));
  EXPECT_EQ(acc.cellCount(), 0u);
  EXPECT_EQ(acc.integrate(makeWindow(4, 4, 0.5f, 0.0, 0.0, 0)),
            Status::kNoOverlap);
  TraversabilityAccumulator nan_box(
      box(0.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f, 5.0f, 0.5f));
  EXPECT_EQ(nan_box.cellCount(), 0u);
}

TEST(TraversabilityAccumulator, SameResolutionWindowIsCopied) {
  TraversabilityAccumulator acc(box(0.0f, 10.0f, 0.0f, 10.0f, 0.5f));
  auto w = makeWindow(4, 4, 0.5f, 1.0, 1.0, 10);
  setCell(w, 0, 0, 100);
  setCell(w, 3, 3, -1);
  ASSERT_EQ(acc.integrate(w), Status::kIntegrated);
  EXPECT_EQ(acc.knownCells(), 15u);
  EXPECT_EQ(acc.version(), 1u);
  EXPECT_EQ(at(acc, 1.25, 1.25), 100);
  EXPECT_EQ(at(acc, 1.75, 1.25), 10);
  EXPECT_EQ(at(acc, 2.75, 2.75), -1);  // window cell was unknown
  EXPECT_EQ(at(acc, 0.75, 1.25), -1);  // outside the window
  EXPECT_EQ(at(acc, 3.25, 1.25), -1);
}

TEST(TraversabilityAccumulator, LatestWindowOverwritesUnknownKeepsOld) {
  TraversabilityAccumulator acc(box(0.0f, 10.0f, 0.0f, 10.0f, 0.5f));
  ASSERT_EQ(acc.integrate(makeWindow(4, 4, 0.5f, 1.0, 1.0, 100)),
            Status::kIntegrated);
  // Second window over the same cells: lower values replace (a cell that
  // turned out passable must become passable), unknown keeps the old value.
  auto w = makeWindow(4, 4, 0.5f, 1.0, 1.0, 5);
  setCell(w, 1, 1, -1);
  ASSERT_EQ(acc.integrate(w), Status::kIntegrated);
  EXPECT_EQ(at(acc, 1.25, 1.25), 5);
  EXPECT_EQ(at(acc, 1.75, 1.75), 100);
  EXPECT_EQ(acc.knownCells(), 16u);
}

TEST(TraversabilityAccumulator, FinerWindowIsMaxPooledWithinAWindow) {
  TraversabilityAccumulator acc(box(0.0f, 4.0f, 0.0f, 4.0f, 0.5f));
  // 0.1 m window: a single 0.1 m obstacle inside a 0.5 m cell must survive.
  auto w = makeWindow(20, 20, 0.1f, 0.0, 0.0, 0);
  setCell(w, 7, 7, 100);  // lands in accumulator cell (1, 1)
  ASSERT_EQ(acc.integrate(w), Status::kIntegrated);
  EXPECT_EQ(at(acc, 0.75, 0.75), 100);
  EXPECT_EQ(at(acc, 0.25, 0.25), 0);
  EXPECT_EQ(acc.knownCells(), 16u);  // 2 x 2 m window -> 4 x 4 cells
  // A later window that sees the cell clear replaces the max, not pools with it.
  ASSERT_EQ(acc.integrate(makeWindow(20, 20, 0.1f, 0.0, 0.0, 3)),
            Status::kIntegrated);
  EXPECT_EQ(at(acc, 0.75, 0.75), 3);
}

TEST(TraversabilityAccumulator, CoarserWindowFillsCoveredCells) {
  TraversabilityAccumulator acc(box(0.0f, 4.0f, 0.0f, 4.0f, 0.5f));
  auto w = makeWindow(2, 2, 1.0f, 1.0, 1.0, 20);
  setCell(w, 1, 1, 90);
  ASSERT_EQ(acc.integrate(w), Status::kIntegrated);
  EXPECT_EQ(acc.knownCells(), 16u);  // 2 x 2 m -> 4 x 4 cells of 0.5 m
  EXPECT_EQ(at(acc, 1.25, 1.25), 20);
  EXPECT_EQ(at(acc, 1.75, 1.75), 20);
  EXPECT_EQ(at(acc, 2.25, 2.25), 90);
  EXPECT_EQ(at(acc, 2.75, 2.75), 90);
  EXPECT_EQ(at(acc, 0.75, 0.75), -1);
  EXPECT_EQ(at(acc, 3.25, 3.25), -1);
}

TEST(TraversabilityAccumulator, UnalignedWindowUsesCellCentres) {
  TraversabilityAccumulator acc(box(0.0f, 2.0f, 0.0f, 2.0f, 0.1f));
  // Origin offset 0.04 m (traversability_mapping does not snap its origin):
  // window cell i has its centre at 0.04 + 0.1 i + 0.05 -> accumulator cell i.
  auto w = makeWindow(5, 1, 0.1f, 0.04, 0.04, 0);
  setCell(w, 2, 0, 77);
  ASSERT_EQ(acc.integrate(w), Status::kIntegrated);
  EXPECT_EQ(at(acc, 0.25, 0.05), 77);
  EXPECT_EQ(at(acc, 0.15, 0.05), 0);
  EXPECT_EQ(at(acc, 0.35, 0.05), 0);
  EXPECT_EQ(acc.knownCells(), 5u);
}

TEST(TraversabilityAccumulator, ClipsToTheBox) {
  TraversabilityAccumulator acc(box(0.0f, 2.0f, 0.0f, 2.0f, 0.5f));
  // Window [-1, 1] x [-1, 1]: only the [0, 1]^2 quarter lands.
  ASSERT_EQ(acc.integrate(makeWindow(4, 4, 0.5f, -1.0, -1.0, 7)),
            Status::kIntegrated);
  EXPECT_EQ(acc.knownCells(), 4u);
  EXPECT_EQ(at(acc, 0.25, 0.25), 7);
  EXPECT_EQ(at(acc, 1.25, 0.25), -1);
  // Entirely outside, near and far (a far origin must not overflow).
  EXPECT_EQ(acc.integrate(makeWindow(4, 4, 0.5f, 5.0, 5.0, 7)),
            Status::kNoOverlap);
  EXPECT_EQ(acc.integrate(makeWindow(4, 4, 0.5f, -1e12, 3e15, 7)),
            Status::kNoOverlap);
  EXPECT_EQ(acc.knownCells(), 4u);
}

TEST(TraversabilityAccumulator, AllUnknownWindow) {
  TraversabilityAccumulator acc(box(0.0f, 2.0f, 0.0f, 2.0f, 0.5f));
  EXPECT_EQ(acc.integrate(makeWindow(4, 4, 0.5f, 0.0, 0.0, -1)),
            Status::kNoKnownCells);
  EXPECT_EQ(acc.version(), 0u);
}

TEST(TraversabilityAccumulator, RejectsBadWindows) {
  TraversabilityAccumulator acc(box(0.0f, 10.0f, 0.0f, 10.0f, 0.5f));
  auto bad_size = makeWindow(4, 4, 0.5f, 0.0, 0.0, 0);
  bad_size.data.pop_back();
  EXPECT_EQ(acc.integrate(bad_size), Status::kMalformed);

  auto zero_res = makeWindow(4, 4, 0.0f, 0.0, 0.0, 0);
  EXPECT_EQ(acc.integrate(zero_res), Status::kMalformed);

  auto nan_res = makeWindow(4, 4, std::numeric_limits<float>::quiet_NaN(),
                            0.0, 0.0, 0);
  EXPECT_EQ(acc.integrate(nan_res), Status::kMalformed);

  auto nan_origin = makeWindow(4, 4, 0.5f, 0.0, 0.0, 0);
  nan_origin.info.origin.position.x = std::nan("");
  EXPECT_EQ(acc.integrate(nan_origin), Status::kMalformed);

  auto empty = makeWindow(0, 0, 0.5f, 0.0, 0.0, 0);
  EXPECT_EQ(acc.integrate(empty), Status::kMalformed);

  auto rotated = makeWindow(4, 4, 0.5f, 0.0, 0.0, 0);
  rotated.info.origin.orientation.z = std::sin(M_PI / 4);  // 90 deg yaw
  rotated.info.origin.orientation.w = std::cos(M_PI / 4);
  EXPECT_EQ(acc.integrate(rotated), Status::kRotated);

  // A width x height product that would overflow 32 bits is caught as too
  // large before anything else touches it.
  auto huge = makeWindow(1, 1, 0.5f, 0.0, 0.0, 0);
  huge.info.width = 100000;
  huge.info.height = 100000;
  EXPECT_EQ(acc.integrate(huge), Status::kTooLarge);

  auto cfg = box(0.0f, 10.0f, 0.0f, 10.0f, 0.5f);
  cfg.max_window_cells = 10;
  TraversabilityAccumulator small(cfg);
  EXPECT_EQ(small.integrate(makeWindow(4, 4, 0.5f, 0.0, 0.0, 0)),
            Status::kTooLarge);

  EXPECT_EQ(acc.knownCells(), 0u);
  EXPECT_EQ(acc.version(), 0u);
}

TEST(TraversabilityAccumulator, ZeroQuaternionReadsAsIdentity) {
  TraversabilityAccumulator acc(box(0.0f, 10.0f, 0.0f, 10.0f, 0.5f));
  auto w = makeWindow(4, 4, 0.5f, 0.0, 0.0, 0);
  w.info.origin.orientation.w = 0.0;
  EXPECT_EQ(acc.integrate(w), Status::kIntegrated);
}

TEST(TraversabilityAccumulator, VersionCountsChangesOnly) {
  TraversabilityAccumulator acc(box(0.0f, 10.0f, 0.0f, 10.0f, 0.5f));
  const auto w = makeWindow(4, 4, 0.5f, 0.0, 0.0, 30);
  ASSERT_EQ(acc.integrate(w), Status::kIntegrated);
  EXPECT_EQ(acc.version(), 1u);
  ASSERT_EQ(acc.integrate(w), Status::kIntegrated);  // identical content
  EXPECT_EQ(acc.version(), 1u);
  auto w2 = w;
  setCell(w2, 2, 2, 31);
  ASSERT_EQ(acc.integrate(w2), Status::kIntegrated);
  EXPECT_EQ(acc.version(), 2u);
}

TEST(TraversabilityAccumulator, ClampsAboveHundred) {
  TraversabilityAccumulator acc(box(0.0f, 10.0f, 0.0f, 10.0f, 0.5f));
  auto w = makeWindow(1, 1, 0.5f, 0.0, 0.0, 127);
  ASSERT_EQ(acc.integrate(w), Status::kIntegrated);
  EXPECT_EQ(at(acc, 0.25, 0.25), 100);
}
