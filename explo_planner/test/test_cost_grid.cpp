// Moved comments: doc/explo_planner_code_notes.md
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <set>
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

// 11. An unbounded flood (radius_cap_m <= 0) must reach cells whose path cost
// exceeds the grid's straight-line diagonal. (notes: costgrid-unbounded-flood)
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

// A map build() rejected (data.size() disagreeing with the dims marks every
// cell blocked) must report nothing reachable, the source included.
// (notes: costgrid-rejected-map-seed)
TEST(CostGrid, MalformedMapLeavesNothingReachableIncludingTheSource) {
  auto grid = makeGrid(10, 10, 1.0f);
  grid.data.resize(10);  // metadata still claims 100 cells
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 5, 5), 100.0f);

  EXPECT_EQ(cg.reachedCellCount(), 0u)
      << "a rejected map must not report a reached cell";
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 5, 5)))
      << "the source is not reachable on a map with no data";
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 0, 0)));
}

// A robot standing on an inflated (blocked) cell must still flood. The seed
// test is whether the flood has anywhere to go, not whether the source is
// traversable; relaxation never passes another blocked cell.
// (notes: costgrid-seed-on-inflation)
TEST(CostGrid, BlockedSourceWithAFreeNeighbourStillFloods) {
  auto grid = makeGrid(10, 10, 1.0f);
  block(grid, 5, 5);  // robot sits on inflation; everything else is free
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 5, 5), 100.0f);

  EXPECT_TRUE(cg.reachable(cellCenter(grid, 5, 5)));
  EXPECT_NEAR(cg.costTo(cellCenter(grid, 5, 5)), 0.0f, 1e-6f);
  EXPECT_TRUE(cg.reachable(cellCenter(grid, 0, 0)))
      << "seeding on inflation must not cost the robot the rest of the map";
}

// Fully walled in: the source and all eight neighbours are blocked, so there is
// genuinely nowhere to go. Distinct from the case above by exactly one free
// neighbour, which is the discrimination the seed test has to make.
TEST(CostGrid, FullyEnclosedSourceReachesNothing) {
  auto grid = makeGrid(10, 10, 1.0f);
  for (int dy = -1; dy <= 1; ++dy)
    for (int dx = -1; dx <= 1; ++dx) block(grid, 5 + dx, 5 + dy);
  CostGrid cg;
  cg.build(grid);
  cg.floodFrom(cellCenter(grid, 5, 5), 100.0f);

  EXPECT_EQ(cg.reachedCellCount(), 0u);
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 5, 5)));
  // The node's kMinReachedForFilter fallback reads this count and skips the
  // reachability filter for the tick; 0 and the old 1 are both below it, so
  // this change is inert there by design. Pinned so that stays true.
  EXPECT_LT(cg.reachedCellCount(), 10u);
}

// ---------------------------------------------------------------------------
// floodUntil
// ---------------------------------------------------------------------------

// On random obstacle grids, every target floodFrom reaches is settled by
// floodUntil with the same cost bit for bit and the same path, the return
// counts exactly those distinct targets, and the early stop never reaches a
// cell floodFrom does not. Catches counting a target on a stale pop (the stop
// then comes before the last target is final) and an off-by-one stop.
TEST(CostGrid, FloodUntilMatchesFloodFromOnSettledTargets) {
  std::mt19937 rng(20261012);
  auto uni = [&](double a, double b) {
    return std::uniform_real_distribution<double>(a, b)(rng);
  };
  auto pick = [&](int n) {
    return std::uniform_int_distribution<int>(0, n - 1)(rng);
  };
  const float kRes[3] = {0.2f, 0.5f, 1.0f};
  int compared = 0, early = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const int w = 30 + pick(91), h = 30 + pick(91);
    const float res = kRes[pick(3)];
    auto grid = makeGrid(w, h, res, static_cast<float>(uni(-5.0, 5.0)),
                         static_cast<float>(uni(-5.0, 5.0)));
    const double p_occ = uni(0.0, 0.35);
    for (auto& v : grid.data) {
      if (uni(0.0, 1.0) < p_occ) v = 100;
      else if (pick(4) == 0) v = -1;          // unknown is passable
    }
    // Cell centres jittered inside the cell, so the cell is unambiguous.
    auto inCell = [&](int gx, int gy) {
      Eigen::Vector3f p = cellCenter(grid, gx, gy);
      p.x() += static_cast<float>(uni(-0.3, 0.3)) * res;
      p.y() += static_cast<float>(uni(-0.3, 0.3)) * res;
      return p;
    };
    const int sgx = pick(w), sgy = pick(h);
    const Eigen::Vector3f src = inCell(sgx, sgy);
    const float cap = pick(3) == 0 ? 0.0f : static_cast<float>(uni(1.0, 40.0));

    std::vector<Eigen::Vector3f> targets;
    std::set<int> waited;  // distinct cells floodUntil waits for
    const int nt = pick(13);
    for (int k = 0; k < nt; ++k) {
      const int kind = pick(10);
      if (kind == 0) {                        // off the grid
        targets.push_back(Eigen::Vector3f(-1000.0f, 3.0f, 0.0f));
        continue;
      }
      const int gx = kind == 1 ? sgx : pick(w);
      const int gy = kind == 1 ? sgy : pick(h);
      targets.push_back(inCell(gx, gy));
      if (kind == 2) targets.push_back(inCell(gx, gy));   // same cell twice
      const bool src_cell = gx == sgx && gy == sgy;
      if (grid.data[static_cast<size_t>(gy) * w + gx] != 100 || src_cell)
        waited.insert(gy * w + gx);
    }

    CostGrid a, b;
    a.build(grid);
    b.build(grid);
    a.floodFrom(src, cap);
    const int ret = b.floodUntil(src, targets, cap);

    int expect_settled = 0;
    for (int ci : waited) {
      const Eigen::Vector3f t = cellCenter(grid, ci % w, ci / w);
      if (std::isfinite(a.costTo(t))) ++expect_settled;
    }
    EXPECT_EQ(ret, expect_settled) << "trial " << trial;
    for (const auto& t : targets) {
      if (!std::isfinite(a.costTo(t))) continue;
      EXPECT_EQ(b.costTo(t), a.costTo(t)) << "trial " << trial;
      EXPECT_EQ(b.extractPath(t), a.extractPath(t)) << "trial " << trial;
      ++compared;
    }
    EXPECT_LE(b.reachedCellCount(), a.reachedCellCount()) << "trial " << trial;
    if (b.reachedCellCount() < a.reachedCellCount()) ++early;
  }
  EXPECT_GT(compared, 1000);
  EXPECT_GT(early, 100) << "the early stop should usually stop early";
}

// An open 40 m grid: a target 2 m away settles after a few hundred cells,
// not the whole map.
TEST(CostGrid, FloodUntilStopsEarly) {
  auto grid = makeGrid(200, 200, 0.2f, -20.0f, -20.0f);
  CostGrid a, b;
  a.build(grid);
  b.build(grid);
  const Eigen::Vector3f src(0.1f, 0.1f, 0.0f), tgt(2.1f, 0.1f, 0.0f);
  a.floodFrom(src, 0.0f);
  EXPECT_EQ(b.floodUntil(src, {tgt}, 0.0f), 1);
  EXPECT_EQ(a.reachedCellCount(), 40000u);
  EXPECT_LT(b.reachedCellCount(), 1000u);
  EXPECT_EQ(b.costTo(tgt), a.costTo(tgt));
  EXPECT_NEAR(b.costTo(tgt), 2.0f, 1e-4f);
}

// What is waited for and what the return counts. Each case pins one rule:
// off-grid and blocked targets are not waited for (the flood still stops at
// the real one); duplicates count once; the source settles at 0 even when
// it is blocked; an enclosed or out-of-cap target exhausts the flood and is
// not counted; no target returns 0 with the last flood cleared.
TEST(CostGrid, FloodUntilTargetEdgeCases) {
  auto grid = makeGrid(60, 60, 0.5f);
  // A sealed free pocket at (50,50): its 8 neighbours blocked.
  for (int dy = -1; dy <= 1; ++dy)
    for (int dx = -1; dx <= 1; ++dx)
      if (dx || dy) block(grid, 50 + dx, 50 + dy);
  block(grid, 20, 10);                      // a lone blocked target cell
  block(grid, 10, 30);                      // the blocked-source case
  const Eigen::Vector3f src = cellCenter(grid, 10, 10);
  const Eigen::Vector3f near = cellCenter(grid, 14, 10);
  const Eigen::Vector3f far = cellCenter(grid, 40, 10);
  const Eigen::Vector3f pocket = cellCenter(grid, 50, 50);
  const Eigen::Vector3f off(-100.0f, 5.0f, 0.0f);

  CostGrid full, cg;
  full.build(grid);
  cg.build(grid);
  full.floodFrom(src, 0.0f);
  const size_t all = full.reachedCellCount();

  EXPECT_EQ(cg.floodUntil(src, {near}, 0.0f), 1);
  const size_t one = cg.reachedCellCount();
  EXPECT_LT(one, all / 4);
  EXPECT_EQ(cg.costTo(near), full.costTo(near));

  // Off-grid and blocked targets: not waited for, so the same early stop.
  EXPECT_EQ(cg.floodUntil(src, {off, cellCenter(grid, 20, 10), near}, 0.0f),
            1);
  EXPECT_EQ(cg.reachedCellCount(), one);
  EXPECT_FALSE(cg.reachable(cellCenter(grid, 20, 10)));

  // Duplicates, including two points in one cell, count once.
  const Eigen::Vector3f near2 = near + Eigen::Vector3f(0.2f, -0.2f, 0.0f);
  EXPECT_EQ(cg.floodUntil(src, {near, near2, near}, 0.0f), 1);
  EXPECT_EQ(cg.reachedCellCount(), one);

  // The source is settled first, at 0, and nothing else is reached.
  EXPECT_EQ(cg.floodUntil(src, {src}, 0.0f), 1);
  EXPECT_EQ(cg.costTo(src), 0.0f);
  EXPECT_EQ(cg.reachedCellCount(), 1u);

  // A blocked source with a free neighbour still seeds and is waited for.
  const Eigen::Vector3f bsrc = cellCenter(grid, 10, 30);
  EXPECT_EQ(cg.floodUntil(bsrc, {bsrc}, 0.0f), 1);
  EXPECT_EQ(cg.costTo(bsrc), 0.0f);

  // An enclosed target: the flood exhausts its component; only `far` counts.
  EXPECT_EQ(cg.floodUntil(src, {pocket, far}, 0.0f), 1);
  EXPECT_EQ(cg.reachedCellCount(), all);
  EXPECT_FALSE(cg.reachable(pocket));
  EXPECT_EQ(cg.costTo(far), full.costTo(far));

  // Beyond the cap: stays unreached and uncounted.
  EXPECT_EQ(cg.floodUntil(src, {far}, 5.0f), 0);
  EXPECT_FALSE(cg.reachable(far));

  // No target (or none it can wait for): 0, and the previous flood is gone.
  cg.floodFrom(src, 0.0f);
  EXPECT_EQ(cg.floodUntil(src, {}, 0.0f), 0);
  EXPECT_EQ(cg.reachedCellCount(), 0u);
  EXPECT_EQ(cg.floodUntil(src, {off}, 0.0f), 0);
  EXPECT_EQ(cg.reachedCellCount(), 0u);
  // An off-grid source reaches nothing.
  EXPECT_EQ(cg.floodUntil(off, {near}, 0.0f), 0);
  EXPECT_EQ(cg.reachedCellCount(), 0u);
}
