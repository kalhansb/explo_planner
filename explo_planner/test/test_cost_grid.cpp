#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explo_planner/cost_grid.hpp"

using namespace explo_planner;

namespace {

// Build a planning_map at a given resolution and origin. By default every
// cell is free (value 0). Pass `obstacles` as (gx, gy) pairs to set them
// to an impassable value (100).
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

void block(nav_msgs::msg::OccupancyGrid& g, int gx, int gy) {
  g.data[gy * g.info.width + gx] = 100;
}

inline Eigen::Vector3f cellCenter(const nav_msgs::msg::OccupancyGrid& g,
                                   int gx, int gy) {
  return Eigen::Vector3f(
      static_cast<float>(g.info.origin.position.x +
                         (gx + 0.5f) * g.info.resolution),
      static_cast<float>(g.info.origin.position.y +
                         (gy + 0.5f) * g.info.resolution),
      0.0f);
}

}  // namespace

// 1. Empty / all-free grid: source at center, opposite corner finite cost.
TEST(CostGrid, AllFreeFloodReachesEverywhere) {
  auto grid = makeGrid(10, 10, 1.0f, 0.0f, 0.0f);
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 5, 5), 100.0f);

  EXPECT_NEAR(cg.costTo(cellCenter(grid, 5, 5)), 0.0f, 1e-6f);
  EXPECT_TRUE(std::isfinite(cg.costTo(cellCenter(grid, 0, 0))));
  EXPECT_TRUE(cg.reachable(cellCenter(grid, 9, 9)));
  EXPECT_EQ(cg.reachedCellCount(), 100u);
}

// 2. Vertical wall splits the grid: right half unreachable from the left.
TEST(CostGrid, WallSplitsGrid) {
  auto grid = makeGrid(11, 11, 1.0f);
  for (int y = 0; y < 11; ++y) block(grid, 5, y);  // wall at x=5
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 1, 5), 100.0f);

  EXPECT_TRUE(cg.reachable(cellCenter(grid, 0, 5)));
  EXPECT_TRUE(cg.reachable(cellCenter(grid, 4, 5)));
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 6, 5)));
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 9, 5)));

  // Reachability and finite-cost agree.
  EXPECT_EQ(std::isfinite(cg.costTo(cellCenter(grid, 6, 5))),
            cg.reachable(cellCenter(grid, 6, 5)));
}

// 3. U-turn around an obstacle costs more than the straight-line distance.
TEST(CostGrid, UTurnLongerThanStraightLine) {
  // 7x5 grid, three-cell wall blocks the direct E-W path between cols 2 and 4.
  auto grid = makeGrid(7, 5, 1.0f);
  block(grid, 3, 1);
  block(grid, 3, 2);
  block(grid, 3, 3);
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 1, 2), 100.0f);

  float c = cg.costTo(cellCenter(grid, 5, 2));
  EXPECT_TRUE(std::isfinite(c));
  // Straight-line distance is 4 m; the U-turn must be strictly longer.
  EXPECT_GT(c, 4.0f);
}

// 4. Diagonal step: cost ~ sqrt(2) * resolution, not 2 * resolution.
TEST(CostGrid, DiagonalCost) {
  auto grid = makeGrid(5, 5, 1.0f);
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 0, 0), 10.0f);

  float diag = cg.costTo(cellCenter(grid, 1, 1));
  EXPECT_NEAR(diag, std::sqrt(2.0f), 1e-3f);
  EXPECT_LT(diag, 1.5f);  // strictly less than two orthogonal steps
}

// 5. Bounded flood: cells outside the cap are kInfCost.
TEST(CostGrid, BoundedFloodLeavesFarCellsUnreached) {
  auto grid = makeGrid(31, 31, 1.0f);
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 15, 15), 5.0f);

  // Inside the cap.
  EXPECT_TRUE(cg.reachable(cellCenter(grid, 16, 15)));
  EXPECT_TRUE(cg.reachable(cellCenter(grid, 15, 18)));
  // Far outside the cap.
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 30, 30)));
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 0, 0)));

  // Roughly disc-shaped: count should be near pi * r^2.
  size_t reached = cg.reachedCellCount();
  EXPECT_GT(reached, 50u);
  EXPECT_LT(reached, 110u);
}

// 6. Reachability differs from "single-cell free": a free pocket sealed
//    on all 8 sides is isCellFree=true but reachable=false.
TEST(CostGrid, FreePocketIsUnreachable) {
  // 5x5 grid; cell (2,2) is free but completely encircled.
  auto grid = makeGrid(5, 5, 1.0f);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      block(grid, 2 + dx, 2 + dy);
    }
  }
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 0, 0), 100.0f);

  EXPECT_FALSE(cg.reachable(cellCenter(grid, 2, 2)));
  // The cell itself is "free" by single-cell rules.
  EXPECT_FALSE(cg.blockedAt(2, 2));
}

// 7. Rebuild discards previous obstacle layer.
TEST(CostGrid, RebuildResamplesObstacles) {
  // First build: open grid, every cell reachable from center.
  auto open = makeGrid(5, 5, 1.0f);
  CostGrid cg;
  cg.build(open);
  cg.floodFrom(cellCenter(open, 2, 2), 100.0f);
  EXPECT_TRUE(cg.reachable(cellCenter(open, 4, 4)));

  // Second build: same dims but a vertical wall splitting it.
  auto walled = makeGrid(5, 5, 1.0f);
  for (int y = 0; y < 5; ++y) block(walled, 2, y);
  cg.build(walled);
  cg.floodFrom(cellCenter(walled, 0, 2), 100.0f);
  // Now (4,4) is unreachable. If state leaked from build #1 we'd see true.
  EXPECT_FALSE(cg.reachable(cellCenter(walled, 4, 2)));
}

// 8. Repeated flood from the same source is deterministic.
TEST(CostGrid, RepeatableFlood) {
  auto grid = makeGrid(8, 8, 1.0f);
  CostGrid cg;
  cg.build(grid);

  cg.floodFrom(cellCenter(grid, 3, 3), 10.0f);
  std::vector<float> first;
  for (int y = 0; y < 8; ++y)
    for (int x = 0; x < 8; ++x)
      first.push_back(cg.costTo(cellCenter(grid, x, y)));

  cg.floodFrom(cellCenter(grid, 3, 3), 10.0f);
  for (int y = 0; y < 8; ++y) {
    for (int x = 0; x < 8; ++x) {
      float c = cg.costTo(cellCenter(grid, x, y));
      float prev = first[y * 8 + x];
      if (std::isfinite(c) && std::isfinite(prev)) {
        EXPECT_NEAR(c, prev, 1e-6f);
      } else {
        EXPECT_EQ(std::isfinite(c), std::isfinite(prev));
      }
    }
  }
}

// 9. Out-of-bounds source returns cleanly: every cell unreachable.
TEST(CostGrid, OutOfBoundsSource) {
  auto grid = makeGrid(5, 5, 1.0f);
  CostGrid cg;
  cg.build(grid);
  // Source far outside the grid extents.
  cg.floodFrom(Eigen::Vector3f(1000.0f, 1000.0f, 0.0f), 100.0f);

  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 5; ++x) {
      EXPECT_FALSE(cg.reachable(cellCenter(grid, x, y)));
    }
  }
}

// 10. Smoke check: 300x300 grid at 0.20 m, 8 m radius cap, < 50 ms.
TEST(CostGrid, PerformanceSmokeCheck) {
  auto grid = makeGrid(300, 300, 0.20f, -30.0f, -30.0f);
  CostGrid cg;
  cg.build(grid);

  auto t0 = std::chrono::steady_clock::now();
  cg.floodFrom(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 8.0f);
  auto t1 = std::chrono::steady_clock::now();

  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  // Loose bound; nominal target is < 1 ms. Failure here means the impl
  // is wrong, not the math.
  EXPECT_LT(ms, 50);
  EXPECT_GT(cg.reachedCellCount(), 1000u);
  EXPECT_LT(cg.reachedCellCount(), 6000u);
}
