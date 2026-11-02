#include <gtest/gtest.h>

#include <algorithm>
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

// 11. An unbounded flood (radius_cap_m <= 0) must reach cells whose PATH cost
//     exceeds the grid's straight-line diagonal. The old implementation
//     silently clamped the "no bound" case to diag + 1, so a serpentine
//     corridor — a walked distance far longer than the diagonal — read as
//     unreachable and the exploitation planner rejected drivable vantages.
TEST(CostGrid, UnboundedFloodExceedsGridDiagonal) {
  // 20 x 20 at 1 m: diagonal is hypot(20,20) = 28.3 m, so the old cap was
  // ~29.3 m. Block every odd row except a single gap that alternates between
  // the two ends, forcing a snake that traverses ~19 m per open row.
  auto grid = makeGrid(20, 20, 1.0f);
  for (int gy = 1; gy < 20; gy += 2) {
    const int gap = ((gy / 2) % 2 == 0) ? 19 : 0;
    for (int gx = 0; gx < 20; ++gx)
      if (gx != gap) block(grid, gx, gy);
  }

  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 0, 0), /*radius_cap_m (unbounded)=*/0.0f);

  const auto far = cellCenter(grid, 0, 18);
  const float diag = std::hypot(20.0f, 20.0f) * 1.0f;
  ASSERT_TRUE(cg.reachable(far));
  // The whole point: the walked cost is well past the diagonal clamp.
  EXPECT_GT(cg.costTo(far), diag + 1.0f);
}

// 12. A POSITIVE cap is still honoured — the fix removed the implicit diagonal
//     clamp, not the caller's explicit bound.
TEST(CostGrid, PositiveCapStillBoundsTheFlood) {
  auto grid = makeGrid(20, 20, 1.0f);
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 0, 0), /*radius_cap_m=*/3.0f);

  EXPECT_TRUE(cg.reachable(cellCenter(grid, 2, 0)));    // 2 m out
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 15, 15))); // way past the cap
}

// --- CostModel ---------------------------------------------------------------

// A default (unshaped) model is the legacy metric flood.
TEST(CostGridModel, DefaultModelIsMetric) {
  auto grid = makeGrid(10, 1, 1.0f);
  grid.data[5] = 40;  // graded but passable: ignored without trav_weight
  CostGrid cg;
  cg.build(grid, 50, CostModel{});
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_NEAR(cg.costTo(cellCenter(grid, 9, 0)), 9.0f, 1e-5f);
  EXPECT_FALSE(cg.inClearanceZone(cellCenter(grid, 5, 0)));
}

// trav_weight: entering a cell holding v costs step * (1 + w * v / 100).
TEST(CostGridModel, TraversabilityWeightScalesStep) {
  auto grid = makeGrid(10, 1, 1.0f);
  grid.data[5] = 40;
  CostModel m;
  m.trav_weight = 2.0f;
  CostGrid cg;
  cg.build(grid, 50, m);
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  // 8 plain steps + one into the v=40 cell at 1 + 2*0.4 = 1.8.
  EXPECT_NEAR(cg.costTo(cellCenter(grid, 9, 0)), 8.0f + 1.8f, 1e-4f);
}

// The flood detours around a rough strip when going round is cheaper.
TEST(CostGridModel, TraversabilityWeightPrefersSmoothDetour) {
  auto grid = makeGrid(9, 5, 1.0f);
  // Rough (v=45, passable) column x=4 for y=0..3; y=4 left smooth.
  for (int y = 0; y < 4; ++y) grid.data[y * 9 + 4] = 45;
  CostModel m;
  m.trav_weight = 10.0f;  // crossing the strip costs 5.5 per step
  CostGrid cg;
  cg.build(grid, 50, m);
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  const float c = cg.costTo(cellCenter(grid, 8, 0));
  EXPECT_LT(c, 8.0f + 4.5f);   // cheaper than crossing the strip
  EXPECT_GT(c, 8.0f);          // but longer than the straight line
  auto path = cg.extractPath(cellCenter(grid, 8, 0));
  bool via_smooth = false;
  for (const auto& p : path) {
    if (std::floor(p.x()) == 4.0f) via_smooth = std::floor(p.y()) == 4.0f;
  }
  EXPECT_TRUE(via_smooth);
}

// Clearance zone: passable cells within clearance_m of an obstacle, flagged
// and penalised but still passable.
TEST(CostGridModel, ClearanceZoneMarksAndPenalises) {
  auto grid = makeGrid(11, 11, 0.5f);
  block(grid, 5, 5);
  CostModel m;
  m.clearance_m = 0.6f;  // orthogonal neighbours (0.5 m) in, diagonal (~0.7 m) out
  m.clearance_penalty = 4.0f;
  CostGrid cg;
  cg.build(grid, 50, m);
  EXPECT_FALSE(cg.inClearanceZone(cellCenter(grid, 5, 5)));  // blocked itself
  EXPECT_TRUE(cg.inClearanceZone(cellCenter(grid, 4, 5)));
  EXPECT_TRUE(cg.inClearanceZone(cellCenter(grid, 5, 6)));
  EXPECT_FALSE(cg.inClearanceZone(cellCenter(grid, 4, 6)));  // diagonal
  EXPECT_FALSE(cg.inClearanceZone(cellCenter(grid, 3, 5)));  // 1.0 m away
  EXPECT_FALSE(cg.inClearanceZone(cellCenter(grid, 0, 0)));

  // Entering a zone cell costs penalty x step.
  cg.floodFrom(cellCenter(grid, 3, 5), 0.0f);
  EXPECT_NEAR(cg.costTo(cellCenter(grid, 4, 5)), 0.5f * 4.0f, 1e-4f);

  // Starting inside the zone, the robot can still leave it.
  cg.floodFrom(cellCenter(grid, 4, 5), 0.0f);
  EXPECT_TRUE(cg.reachable(cellCenter(grid, 0, 0)));
}

// Clearance zone without a penalty: flagged for goal screening, cost metric.
TEST(CostGridModel, ClearanceZoneWithoutPenaltyKeepsMetricCost) {
  auto grid = makeGrid(11, 1, 1.0f);
  block(grid, 5, 0);
  CostModel m;
  m.clearance_m = 1.0f;
  EXPECT_FALSE(m.shaped());
  CostGrid cg;
  cg.build(grid, 50, m);
  EXPECT_TRUE(cg.inClearanceZone(cellCenter(grid, 4, 0)));
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_NEAR(cg.costTo(cellCenter(grid, 4, 0)), 4.0f, 1e-5f);
}

// Unknown cells are passable and flagged; the threshold is honoured.
TEST(CostGridModel, UnknownAndThreshold) {
  auto grid = makeGrid(5, 1, 1.0f);
  grid.data[1] = -1;
  grid.data[3] = 70;
  CostGrid cg;
  cg.build(grid, 80);  // 70 < 80: passable
  EXPECT_TRUE(cg.unknownAt(1, 0));
  EXPECT_FALSE(cg.blockedAt(3, 0));
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_TRUE(cg.reachable(cellCenter(grid, 4, 0)));
  cg.build(grid, 50);  // 70 >= 50: blocked
  EXPECT_TRUE(cg.blockedAt(3, 0));
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 4, 0)));
}

// A ring of obstacles around the robot (its own body seen by the LiDAR) walls
// the flood in; source_clear_m lets the robot's own flood out through it.
TEST(CostGridModel, SourceClearRadiusOpensRingAroundRobot) {
  auto grid = makeGrid(21, 21, 0.2f);
  for (int y = 7; y <= 13; ++y) {
    for (int x = 7; x <= 13; ++x) {
      const int d2 = (x - 10) * (x - 10) + (y - 10) * (y - 10);
      if (d2 >= 4 && d2 <= 9) block(grid, x, y);  // 0.4-0.6 m ring
    }
  }
  CostGrid cg;
  cg.build(grid);
  const auto robot = cellCenter(grid, 10, 10);
  const auto outside = cellCenter(grid, 2, 10);
  cg.floodFrom(robot, 0.0f);
  EXPECT_FALSE(cg.reachable(outside));
  EXPECT_LT(cg.reachedCellCount(), 10u);

  cg.floodFrom(robot, 0.0f, /*source_clear_m=*/0.9f);  // 4 cells
  EXPECT_TRUE(cg.reachable(outside));
  // Straight out through the ring: 8 cells at 0.2 m, no detour.
  EXPECT_NEAR(cg.costTo(outside), 8 * 0.2f, 1e-4f);
  // Ring cells stay obstacles for goal checks.
  EXPECT_TRUE(cg.blockedAt(10, 12));
  // The count excludes the cleared 4-cell disc: 441 cells minus the
  // 49 (dx^2 + dy^2 <= 16) in it.
  EXPECT_EQ(cg.reachedCellCount(), 441u - 49u);

  // A clear radius inside the ring changes nothing.
  cg.floodFrom(robot, 0.0f, 0.3f);
  EXPECT_FALSE(cg.reachable(outside));
}

// Unseen cells cost as if they held unknown_value; with the default 0 they
// cost as open ground.
TEST(CostGridModel, UnknownValueWeightsUnseenCells) {
  auto grid = makeGrid(10, 1, 1.0f);
  grid.data[5] = -1;
  CostModel m;
  m.trav_weight = 2.0f;
  CostGrid cg;
  cg.build(grid, 50, m);
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_NEAR(cg.costTo(cellCenter(grid, 9, 0)), 9.0f, 1e-4f);

  m.unknown_value = 25.0f;  // 1 + 2 * 0.25 = 1.5
  cg.build(grid, 50, m);
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_NEAR(cg.costTo(cellCenter(grid, 9, 0)), 8.0f + 1.5f, 1e-4f);
  EXPECT_TRUE(cg.unknownAt(5, 0));  // still passable, still flagged
  EXPECT_FALSE(cg.blockedAt(5, 0));

  m.unknown_value = 500.0f;  // clamped below the obstacle range
  cg.build(grid, 50, m);
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_TRUE(cg.reachable(cellCenter(grid, 9, 0)));
}

// pathLengthTo() is the metres driven along the shaped path, not its cost.
TEST(CostGridModel, PathLengthIsMetricUnderShapedModel) {
  auto grid = makeGrid(9, 5, 0.5f);
  for (int y = 0; y < 4; ++y) grid.data[y * 9 + 4] = 45;  // rough strip
  CostModel m;
  m.trav_weight = 10.0f;                                    // strip step x5.5
  CostGrid cg;
  cg.build(grid, 50, m);
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  const auto goal = cellCenter(grid, 8, 0);
  const auto path = cg.extractPath(goal);
  ASSERT_GE(path.size(), 2u);
  double walked = 0.0;
  for (size_t i = 1; i < path.size(); ++i) walked += (path[i] - path[i - 1]).norm();
  EXPECT_NEAR(cg.pathLengthTo(goal), walked, 1e-4);
  EXPECT_GT(cg.pathLengthTo(goal), 8 * 0.5f);   // detours round the strip

  // Every cell mildly rough: each step costs twice its length, and the
  // length stays in metres.
  std::fill(grid.data.begin(), grid.data.end(), 10);
  cg.build(grid, 50, m);
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_NEAR(cg.pathLengthTo(goal), 8 * 0.5f, 1e-4);
  EXPECT_NEAR(cg.costTo(goal), 2.0f * cg.pathLengthTo(goal), 1e-3f);

  // Unshaped: the cost is the length.
  cg.build(grid, 50, CostModel{});
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_FLOAT_EQ(cg.pathLengthTo(goal), cg.costTo(goal));
  EXPECT_FLOAT_EQ(cg.pathLengthTo(cellCenter(grid, 0, 0)), 0.0f);
}

TEST(CostGridModel, PathLengthUnreachableIsInfinite) {
  auto grid = makeGrid(5, 1, 1.0f);
  block(grid, 2, 0);
  CostModel m;
  m.trav_weight = 1.0f;
  CostGrid cg;
  cg.build(grid, 50, m);
  cg.floodFrom(cellCenter(grid, 0, 0), 0.0f);
  EXPECT_FALSE(std::isfinite(cg.pathLengthTo(cellCenter(grid, 4, 0))));
  EXPECT_FALSE(std::isfinite(cg.pathLengthTo(Eigen::Vector3f(-5.0f, 0.0f, 0.0f))));
}
