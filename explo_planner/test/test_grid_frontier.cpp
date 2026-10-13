#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explo_planner/grid_frontier.hpp"

using namespace explo_planner;

namespace {

// Every cell starts at `fill` (default unknown).
nav_msgs::msg::OccupancyGrid makeGrid(int width, int height, float resolution,
                                      int8_t fill = -1, float origin_x = 0.0f,
                                      float origin_y = 0.0f) {
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = width;
  g.info.height = height;
  g.info.resolution = resolution;
  g.info.origin.position.x = origin_x;
  g.info.origin.position.y = origin_y;
  g.data.assign(static_cast<size_t>(width) * height, fill);
  return g;
}

void setCell(nav_msgs::msg::OccupancyGrid& g, int gx, int gy, int8_t v) {
  g.data[static_cast<size_t>(gy) * g.info.width + gx] = v;
}

void fillRect(nav_msgs::msg::OccupancyGrid& g, int x0, int y0, int x1, int y1,
              int8_t v) {
  for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x) setCell(g, x, y, v);
}

std::pair<int, int> cellOf(const nav_msgs::msg::OccupancyGrid& g,
                           const Eigen::Vector3f& p) {
  return {static_cast<int>(std::floor((p.x() - g.info.origin.position.x) /
                                      g.info.resolution)),
          static_cast<int>(std::floor((p.y() - g.info.origin.position.y) /
                                      g.info.resolution))};
}

const Roi2D kHuge{-1e9f, 1e9f, -1e9f, 1e9f};
constexpr float kTwoPi = 6.28318530718f;

}  // namespace

// ---------------------------------------------------------------------------
// isGridFrontierCell / gridBoxOf
// ---------------------------------------------------------------------------

TEST(GridFrontier, FrontierCellNeedsFreeAndA4NeighbourUnknown) {
  auto g = makeGrid(5, 5, 1.0f, 0);  // all free
  const GridBox box = gridBoxOf(g, kHuge);
  ASSERT_FALSE(box.empty());
  EXPECT_FALSE(isGridFrontierCell(g, 2, 2, box));   // no unknown at all

  setCell(g, 3, 2, -1);                              // unknown to the east
  EXPECT_TRUE(isGridFrontierCell(g, 2, 2, box));
  setCell(g, 3, 2, 0);
  setCell(g, 2, 1, -1);                              // unknown to the south
  EXPECT_TRUE(isGridFrontierCell(g, 2, 2, box));
  setCell(g, 2, 1, 0);
  setCell(g, 3, 3, -1);                              // diagonal only
  EXPECT_FALSE(isGridFrontierCell(g, 2, 2, box));

  setCell(g, 2, 3, -1);                              // north unknown...
  setCell(g, 2, 2, 100);                             // ...but cell occupied
  EXPECT_FALSE(isGridFrontierCell(g, 2, 2, box));
  setCell(g, 2, 2, 60);                              // inflated counts too
  EXPECT_FALSE(isGridFrontierCell(g, 2, 2, box));
  setCell(g, 2, 2, -1);                              // unknown itself
  EXPECT_FALSE(isGridFrontierCell(g, 2, 2, box));
}

TEST(GridFrontier, UnknownOutsideTheBoxIsNotAFrontier) {
  auto g = makeGrid(6, 6, 1.0f, 0);
  fillRect(g, 3, 0, 5, 5, -1);        // right half unknown
  const GridBox left = gridBoxOf(g, {0.0f, 2.9f, 0.0f, 5.9f});
  EXPECT_EQ(left.x1, 2);
  EXPECT_FALSE(isGridFrontierCell(g, 2, 2, left));  // unknown lies past x1
  const GridBox all = gridBoxOf(g, kHuge);
  EXPECT_TRUE(isGridFrontierCell(g, 2, 2, all));
  EXPECT_FALSE(isGridFrontierCell(g, 9, 9, all));   // off the grid
}

TEST(GridFrontier, DegenerateGridHasAnEmptyBox) {
  auto g = makeGrid(4, 4, 1.0f, 0);
  g.data.pop_back();                  // size no longer width * height
  EXPECT_TRUE(gridBoxOf(g, kHuge).empty());
  EXPECT_TRUE(findGridFrontiers(g, kHuge, 5.0f, 1).empty());
  auto z = makeGrid(4, 4, 0.0f, 0);
  EXPECT_TRUE(gridBoxOf(z, kHuge).empty());
  EXPECT_EQ(gridVisibleUnknown(z, kHuge, Eigen::Vector3f(1, 1, 0), 0.0f,
                               kTwoPi, 16, 0.0f, 3.0f), 0);
}

// ---------------------------------------------------------------------------
// findGridFrontiers
// ---------------------------------------------------------------------------

// A free 10x10 square in an unknown sea: its 36-cell rim is one component.
// Unbinned it is one candidate; 2 m bins over a 1 m grid split it, every
// candidate is a real frontier cell, and the sizes add back up to the rim.
TEST(GridFrontier, SquareRimIsOneComponentSplitIntoBins) {
  auto g = makeGrid(20, 20, 1.0f);
  fillRect(g, 5, 5, 14, 14, 0);
  const GridBox box = gridBoxOf(g, kHuge);

  auto whole = findGridFrontiers(g, kHuge, 0.0f, 1);
  ASSERT_EQ(whole.size(), 1u);
  EXPECT_EQ(whole[0].cells, 36);

  auto binned = findGridFrontiers(g, kHuge, 2.0f, 1);
  ASSERT_GT(binned.size(), 4u);
  int total = 0;
  std::set<std::pair<int, int>> seen;
  for (const auto& f : binned) {
    total += f.cells;
    const auto c = cellOf(g, f.position);
    EXPECT_TRUE(isGridFrontierCell(g, c.first, c.second, box))
        << "candidate at cell " << c.first << "," << c.second;
    EXPECT_TRUE(seen.insert(c).second) << "two candidates on one cell";
  }
  EXPECT_EQ(total, 36);
}

// Speckle filter is on the component, and 8-connectivity joins a diagonal
// staircase that 4-connectivity would split.
TEST(GridFrontier, MinCellsDropsSpeckleAndDiagonalsConnect) {
  auto g = makeGrid(12, 12, 1.0f);
  setCell(g, 1, 1, 0);                       // one isolated free cell
  for (int i = 0; i < 4; ++i) setCell(g, 5 + i, 5 + i, 0);  // staircase
  auto keep_all = findGridFrontiers(g, kHuge, 0.0f, 1);
  EXPECT_EQ(keep_all.size(), 2u);
  auto filtered = findGridFrontiers(g, kHuge, 0.0f, 3);
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_EQ(filtered[0].cells, 4);
}

// Two frontiers in the same bin but not connected stay two candidates.
TEST(GridFrontier, SameBinDisconnectedStaysSeparate) {
  auto g = makeGrid(10, 10, 1.0f);
  fillRect(g, 1, 1, 2, 2, 0);
  fillRect(g, 6, 6, 7, 7, 0);
  auto out = findGridFrontiers(g, kHuge, 100.0f, 1);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].cells, 4);
  EXPECT_EQ(out[1].cells, 4);
}

// Nearest-to-centroid is a member cell even when the centroid of an L lies
// off it, and positions carry the grid origin.
TEST(GridFrontier, CandidateIsAMemberCellWithOrigin) {
  auto g = makeGrid(10, 10, 0.5f, -1, -3.0f, 7.0f);
  fillRect(g, 2, 2, 2, 7, 0);   // vertical bar
  fillRect(g, 2, 2, 7, 2, 0);   // horizontal bar -> an L
  const GridBox box = gridBoxOf(g, kHuge);
  auto out = findGridFrontiers(g, kHuge, 0.0f, 1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].cells, 11);
  const auto c = cellOf(g, out[0].position);
  EXPECT_TRUE(isGridFrontierCell(g, c.first, c.second, box));
  EXPECT_FLOAT_EQ(out[0].position.x(), -3.0f + (c.first + 0.5f) * 0.5f);
  EXPECT_FLOAT_EQ(out[0].position.y(), 7.0f + (c.second + 0.5f) * 0.5f);
}

// A straight strip of 7 frontier cells: the pick is the middle one, not an end.
TEST(GridFrontier, CandidateIsTheMemberNearestTheCentroid) {
  auto g = makeGrid(12, 12, 1.0f);
  fillRect(g, 2, 5, 8, 5, 0);
  auto out = findGridFrontiers(g, kHuge, 0.0f, 1);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].cells, 7);
  EXPECT_EQ(cellOf(g, out[0].position), std::make_pair(5, 5));
}

// ---------------------------------------------------------------------------
// gridVisibleUnknown
// ---------------------------------------------------------------------------

// All unknown around a free start cell: the count lies between the cells whose
// centre is within max_range - half a diagonal and those the disc touches.
TEST(GridFrontier, OpenUnknownCountIsTheDisc) {
  auto g = makeGrid(41, 41, 1.0f);
  setCell(g, 20, 20, 0);
  const Eigen::Vector3f p(20.5f, 20.5f, 0.0f);
  const int n = gridVisibleUnknown(g, kHuge, p, 0.0f, kTwoPi, 16, 0.0f, 8.0f);
  int inner = 0, outer = 0;
  for (int y = 0; y < 41; ++y) {
    for (int x = 0; x < 41; ++x) {
      if (x == 20 && y == 20) continue;
      const double d = std::hypot(x + 0.5 - 20.5, y + 0.5 - 20.5);
      if (d <= 8.0 - 0.71) ++inner;
      if (d <= 8.0 + 0.71) ++outer;
    }
  }
  EXPECT_GE(n, inner);
  EXPECT_LE(n, outer);
}

// A wall two cells out hides what is behind it, identically on every side,
// so the walk is not only right along +x.
TEST(GridFrontier, WallOccludesTheSameOnEverySide) {
  const Eigen::Vector3f p(20.5f, 20.5f, 0.0f);
  auto open = makeGrid(41, 41, 1.0f);
  setCell(open, 20, 20, 0);
  const int n_open =
      gridVisibleUnknown(open, kHuge, p, 0.0f, kTwoPi, 64, 0.0f, 10.0f);

  int counts[4];
  for (int side = 0; side < 4; ++side) {
    auto g = open;
    for (int k = 0; k < 41; ++k) {
      switch (side) {
        case 0: setCell(g, 22, k, 100); break;  // east
        case 1: setCell(g, 18, k, 100); break;  // west
        case 2: setCell(g, k, 22, 100); break;  // north
        case 3: setCell(g, k, 18, 100); break;  // south
      }
    }
    counts[side] =
        gridVisibleUnknown(g, kHuge, p, 0.0f, kTwoPi, 64, 0.0f, 10.0f);
    EXPECT_LT(counts[side], n_open - 80) << "side " << side;
  }
  for (int side = 1; side < 4; ++side)
    EXPECT_NEAR(counts[side], counts[0], 4) << "side " << side;
}

// A diagonal wall: rays must not leak through corner-touching cells.
TEST(GridFrontier, DiagonalWallDoesNotLeak) {
  auto g = makeGrid(41, 41, 1.0f);
  setCell(g, 20, 20, 0);
  for (int k = -20; k <= 20; ++k) {
    const int x = 23 + k, y = 20 - k;  // x + y = 43: a NW-SE line to the NE
    if (x >= 0 && x < 41 && y >= 0 && y < 41) setCell(g, x, y, 100);
  }
  const Eigen::Vector3f p(20.5f, 20.5f, 0.0f);
  // Look only to the north-east, straight at the wall.
  const int n = gridVisibleUnknown(g, kHuge, p, 0.7853982f, 0.5f, 64, 0.0f,
                                   15.0f);
  // Everything visible is on the robot's side of the line x + y = 43.
  auto g_behind = g;
  for (int y = 0; y < 41; ++y)
    for (int x = 0; x < 41; ++x)
      if (x + y > 43) setCell(g_behind, x, y, 0);  // make the far side free
  EXPECT_EQ(gridVisibleUnknown(g_behind, kHuge, p, 0.7853982f, 0.5f, 64,
                               0.0f, 15.0f),
            n);
  EXPECT_GT(n, 0);
}

// Unknown does not stop a ray; min_range and the ROI only stop counting.
TEST(GridFrontier, UnknownIsTransparentAndRoiAndMinRangeOnlyCount) {
  auto g = makeGrid(30, 3, 1.0f, 0);  // a free corridor along +x
  fillRect(g, 2, 0, 4, 2, -1);        // unknown near
  fillRect(g, 10, 0, 12, 2, -1);      // unknown far, behind the near block
  const Eigen::Vector3f p(0.5f, 1.5f, 0.0f);
  const int all = gridVisibleUnknown(g, kHuge, p, 0.0f, 0.2f, 32, 0.0f, 20.0f);
  const int far_only =
      gridVisibleUnknown(g, kHuge, p, 0.0f, 0.2f, 32, 6.0f, 20.0f);
  const int near_roi = gridVisibleUnknown(g, {0.0f, 6.0f, 0.0f, 3.0f}, p, 0.0f,
                                          0.2f, 32, 0.0f, 20.0f);
  EXPECT_GT(far_only, 0);             // seen through the near unknown block
  EXPECT_GT(near_roi, 0);
  EXPECT_EQ(all, far_only + near_roi);

  fillRect(g, 7, 0, 7, 2, 100);       // a wall between them
  EXPECT_EQ(gridVisibleUnknown(g, kHuge, p, 0.0f, 0.2f, 32, 6.0f, 20.0f), 0);
}

// A directional sensor sees only its own side; yaw turns it.
TEST(GridFrontier, DirectionalFovFollowsYaw) {
  auto g = makeGrid(41, 41, 1.0f, 0);
  fillRect(g, 18, 26, 22, 28, -1);    // unknown block to the north only
  const Eigen::Vector3f p(20.5f, 20.5f, 0.0f);
  const float half_pi = 1.5707963f;
  EXPECT_GT(gridVisibleUnknown(g, kHuge, p, half_pi, 1.0f, 32, 0.0f, 12.0f),
            0);
  EXPECT_EQ(gridVisibleUnknown(g, kHuge, p, -half_pi, 1.0f, 32, 0.0f, 12.0f),
            0);
  EXPECT_EQ(gridVisibleUnknown(g, kHuge, p, 0.0f, 1.0f, 32, 0.0f, 12.0f), 0);
}

// Starting on an occupied cell sees nothing; off the grid sees nothing.
TEST(GridFrontier, OccupiedOrOffGridStartSeesNothing) {
  auto g = makeGrid(11, 11, 1.0f);
  setCell(g, 5, 5, 100);
  EXPECT_EQ(gridVisibleUnknown(g, kHuge, Eigen::Vector3f(5.5f, 5.5f, 0.0f),
                               0.0f, kTwoPi, 16, 0.0f, 5.0f), 0);
  EXPECT_EQ(gridVisibleUnknown(g, kHuge, Eigen::Vector3f(-3.0f, 5.5f, 0.0f),
                               0.0f, kTwoPi, 16, 0.0f, 5.0f), 0);
}

// The cell list is the count, cell for cell: on random grids, poses and
// parameters the list has exactly count entries, all distinct, unknown, inside
// the ROI box and on the grid, and a non-empty list is only appended to.
// Catches a push outside the dedup, before the min-range or ROI test, or with
// x and y swapped in the index.
TEST(GridFrontier, VisibleCellsMatchCountOnRandomGrids) {
  std::mt19937 rng(20261012);
  auto uni = [&](double a, double b) {
    return std::uniform_real_distribution<double>(a, b)(rng);
  };
  auto pick = [&](int n) {
    return std::uniform_int_distribution<int>(0, n - 1)(rng);
  };
  const float kHfov[3] = {kTwoPi, 1.0f, 0.3f};
  const int kRays[2] = {1, 96};
  const float kMinR[3] = {0.0f, 0.3f, 1.0f};
  const float kMaxR[3] = {2.0f, 5.0f, 20.0f};
  int nonzero = 0;
  for (int trial = 0; trial < 300; ++trial) {
    const int w = 60 + pick(81), h = 60 + pick(81);
    const float res = pick(2) ? 0.2f : 0.5f;
    const float ox = static_cast<float>(uni(-5.0, 5.0));
    const float oy = static_cast<float>(uni(-5.0, 5.0));
    auto g = makeGrid(w, h, res, -1, ox, oy);
    const double p_unk = uni(0.3, 0.7), p_occ = uni(0.0, 0.2);
    for (auto& v : g.data) {
      const double u = uni(0.0, 1.0);
      v = u < p_unk ? -1
        : u < p_unk + p_occ ? 100 : static_cast<int8_t>(pick(50));
    }
    const float ext_x = w * res, ext_y = h * res;
    Eigen::Vector3f pos(ox + static_cast<float>(uni(0.0, ext_x)),
                        oy + static_cast<float>(uni(0.0, ext_y)), 0.0f);
    // 1 in 10 starts off the grid, 1 in 10 on an occupied cell, else free.
    const int mode = pick(10);
    const auto c = cellOf(g, pos);
    if (mode == 0)
      pos.x() = ox - static_cast<float>(uni(0.1, 5.0));
    else if (c.first >= 0 && c.first < w && c.second >= 0 && c.second < h)
      setCell(g, c.first, c.second, mode == 1 ? 100 : 0);
    const float yaw = static_cast<float>(uni(-M_PI, M_PI));
    const float hfov = kHfov[pick(3)];
    const int rays = kRays[pick(2)];
    const float min_r = kMinR[pick(3)], max_r = kMaxR[pick(3)];
    Roi2D roi = kHuge;
    if (pick(4) != 0) {
      // A sub-ROI near the start, so it usually clips what the rays reach.
      const float x0 = pos.x() - static_cast<float>(uni(-2.0, max_r));
      const float y0 = pos.y() - static_cast<float>(uni(-2.0, max_r));
      roi = {x0, x0 + static_cast<float>(uni(0.0, 2.0 * max_r)), y0,
             y0 + static_cast<float>(uni(0.0, 2.0 * max_r))};
      if (pick(10) == 0) std::swap(roi.min_x, roi.max_x);         // empty
    }

    const int n =
        gridVisibleUnknown(g, roi, pos, yaw, hfov, rays, min_r, max_r);
    std::vector<int32_t> cells;
    ASSERT_EQ(gridVisibleUnknown(g, roi, pos, yaw, hfov, rays, min_r, max_r,
                                 &cells), n) << "trial " << trial;
    ASSERT_EQ(static_cast<int>(cells.size()), n) << "trial " << trial;
    if (n > 0) ++nonzero;

    std::vector<int32_t> sorted = cells;
    std::sort(sorted.begin(), sorted.end());
    EXPECT_TRUE(std::adjacent_find(sorted.begin(), sorted.end()) ==
                sorted.end())
        << "trial " << trial << ": a cell listed twice";
    const GridBox box = gridBoxOf(g, roi);
    for (int32_t i : cells) {
      ASSERT_GE(i, 0) << "trial " << trial;
      ASSERT_LT(i, w * h) << "trial " << trial;
      EXPECT_LT(g.data[static_cast<size_t>(i)], 0) << "trial " << trial;
      EXPECT_TRUE(box.contains(i % w, i / w)) << "trial " << trial;
    }

    std::vector<int32_t> pre = {-7, 12345};
    EXPECT_EQ(gridVisibleUnknown(g, roi, pos, yaw, hfov, rays, min_r, max_r,
                                 &pre), n);
    ASSERT_EQ(pre.size(), cells.size() + 2) << "trial " << trial;
    EXPECT_EQ(pre[0], -7);
    EXPECT_EQ(pre[1], 12345);
    EXPECT_TRUE(std::equal(cells.begin(), cells.end(), pre.begin() + 2));
  }
  EXPECT_GT(nonzero, 100) << "too few trials saw anything to compare";
}

// Exact membership. Along a one-row corridor the cells are entered at
// t = gx - 0.5, so with min_range 3 the first counted cell is gx 4, and the
// wall at gx 10 ends the list. In the open, every unknown cell left of a wall
// and clear of min_range is listed, and none at or behind the wall.
TEST(GridFrontier, VisibleCellsStopAtWallsAndRespectMinRange) {
  auto row = makeGrid(30, 1, 1.0f);
  setCell(row, 0, 0, 0);
  setCell(row, 10, 0, 100);
  std::vector<int32_t> cells;
  EXPECT_EQ(gridVisibleUnknown(row, kHuge, Eigen::Vector3f(0.5f, 0.5f, 0.0f),
                               0.0f, 0.01f, 8, 3.0f, 20.0f, &cells), 6);
  EXPECT_EQ(cells, (std::vector<int32_t>{4, 5, 6, 7, 8, 9}));

  auto g = makeGrid(41, 41, 1.0f);
  setCell(g, 20, 20, 0);
  for (int y = 0; y < 41; ++y) setCell(g, 24, y, 100);
  setCell(g, 18, 22, 0);                     // a known free cell is not listed
  cells.clear();
  const int n =
      gridVisibleUnknown(g, kHuge, Eigen::Vector3f(20.5f, 20.5f, 0.0f), 0.0f,
                         kTwoPi, 16, 3.0f, 10.0f, &cells);
  ASSERT_EQ(static_cast<int>(cells.size()), n);
  const std::set<int32_t> got(cells.begin(), cells.end());
  for (int32_t i : cells) {
    const int gx = i % 41, gy = i / 41;
    EXPECT_LT(gx, 24) << "listed at or behind the wall: " << gx << "," << gy;
    const double d = std::hypot(gx - 20.0, gy - 20.0);
    EXPECT_GE(d, 3.0 - 0.71) << gx << "," << gy;   // entered at t >= t_min
    EXPECT_LE(d, 10.0 + 0.71) << gx << "," << gy;
  }
  EXPECT_EQ(got.count(22 * 41 + 18), 0u);
  int must = 0;
  for (int gy = 0; gy < 41; ++gy) {
    for (int gx = 0; gx < 24; ++gx) {
      const double d = std::hypot(gx - 20.0, gy - 20.0);
      if (d < 3.0 + 0.71 || d > 6.0 || (gx == 18 && gy == 22)) continue;
      ++must;
      EXPECT_EQ(got.count(gy * 41 + gx), 1u) << "missed " << gx << "," << gy;
    }
  }
  EXPECT_GT(must, 50);
}

// ---------------------------------------------------------------------------
// censusFromPlanMap
// ---------------------------------------------------------------------------

// A 20 m square at 0.5 m in four 10 m cells, only the south-west cell known
// free. The frontier is its east and north edges; its south and west edges
// border the grid's outside, which is not unknown to the census.
TEST(GridFrontier, CensusCountsColumnsAndInteriorFrontier) {
  auto g = makeGrid(40, 40, 0.5f);
  fillRect(g, 0, 0, 19, 19, 0);
  CellGrid cg;
  cg.min_x = 0.0f; cg.min_y = 0.0f; cg.cell_size_m = 10.0f; cg.nx = 2; cg.ny = 2;
  const auto obs = censusFromPlanMap(g, cg);
  ASSERT_EQ(obs.size(), 4u);
  for (const auto& o : obs) EXPECT_EQ(o.total_columns, 400);
  EXPECT_EQ(obs[0].observed_columns, 400);
  EXPECT_EQ(obs[0].observed_voxels, 400);
  EXPECT_EQ(obs[0].frontier_voxels, 39);
  EXPECT_DOUBLE_EQ(obs[0].unknownFraction(), 0.0);
  for (int i = 1; i < 4; ++i) {
    EXPECT_EQ(obs[i].observed_columns, 0);
    EXPECT_EQ(obs[i].frontier_voxels, 0);
    EXPECT_DOUBLE_EQ(obs[i].unknownFraction(), 1.0);
  }
}

// A cell grid only partly on the plan map counts only the covered part; a
// degenerate map leaves every cell unmeasured.
TEST(GridFrontier, CensusPartialAndDegenerate) {
  auto g = makeGrid(20, 40, 0.5f, 0);   // 10 m wide: covers column 0 only
  CellGrid cg;
  cg.min_x = 0.0f; cg.min_y = 0.0f; cg.cell_size_m = 10.0f; cg.nx = 2; cg.ny = 2;
  auto obs = censusFromPlanMap(g, cg);
  ASSERT_EQ(obs.size(), 4u);
  EXPECT_EQ(obs[0].total_columns, 400);
  EXPECT_EQ(obs[1].total_columns, 0);
  EXPECT_EQ(obs[2].total_columns, 400);
  EXPECT_EQ(obs[3].total_columns, 0);

  g.data.clear();
  obs = censusFromPlanMap(g, cg);
  ASSERT_EQ(obs.size(), 4u);
  for (const auto& o : obs) EXPECT_EQ(o.total_columns, 0);
}
