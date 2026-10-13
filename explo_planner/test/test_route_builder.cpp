#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explo_planner/grid_frontier.hpp"
#include "explo_planner/route_builder.hpp"

using namespace explo_planner;

namespace {

// Every cell starts at `fill`.
nav_msgs::msg::OccupancyGrid makeGrid(int width, int height, float resolution,
                                      int8_t fill, float origin_x,
                                      float origin_y) {
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

Eigen::Vector3f at(float x, float y) { return {x, y, 0.0f}; }

const Roi2D kRoi{-20.0f, 20.0f, -20.0f, 20.0f};
constexpr float kTwoPi = 6.28318530718f;

// Lattice index of a point of the 2x2 world (16 x 16 points, row-major).
int latIdx(float x, float y) {
  return static_cast<int>(std::lround((y + 18.75f) / 2.5f)) * 16 +
         static_cast<int>(std::lround((x + 18.75f) / 2.5f));
}

// An unknown room of (2hw+1) x (2hh+1) cells centred on the pad's cell, in an
// occupied ring with a free door mid south wall (none when sealed) and a free
// 3x3 pad. extra_free top-row cells are freed from the west, leaving
// (2hw+1)(2hh+1) - 9 - extra_free unknown cells.
void addPocket(nav_msgs::msg::OccupancyGrid& g, const Eigen::Vector3f& pad,
               int hw, int hh, int extra_free = 0, bool sealed = false) {
  const auto [cx, cy] = cellOf(g, pad);
  ASSERT_TRUE(cx - hw - 1 >= 0 && cy - hh - 1 >= 0 &&
              cx + hw + 1 < static_cast<int>(g.info.width) &&
              cy + hh + 1 < static_cast<int>(g.info.height));
  fillRect(g, cx - hw - 1, cy - hh - 1, cx + hw + 1, cy + hh + 1, 100);
  fillRect(g, cx - hw, cy - hh, cx + hw, cy + hh, -1);
  fillRect(g, cx - 1, cy - 1, cx + 1, cy + 1, 0);
  if (!sealed) setCell(g, cx, cy - hh - 1, 0);
  for (int k = 0; k < extra_free; ++k) setCell(g, cx - hw + k, cy + hh, 0);
}

// The standard world: a free 40 m map at 0.2 m under a 2x2 grid of 20 m cells
// (0 SW, 1 SE, 2 NW, 3 NE), robot at (1.25, 1.25), route cells 0-3, tier cell
// 3, min_range 0 so pocket counts are exact. Inputs point into it.
struct World {
  nav_msgs::msg::OccupancyGrid map;
  CellGrid grid = makeCellGrid(-20.0f, 20.0f, -20.0f, 20.0f, 20.0f);
  CostGrid flood, pairs;
  RouteInputs in;
  RouteParams p;

  explicit World(float origin = -20.0f)
      : map(makeGrid(200, 200, 0.2f, 0, origin, origin)) {
    in.robot = at(1.25f, 1.25f);
    in.route_cells = {0, 1, 2, 3};
    in.tier_cell[0] = 3;
    p.min_range = 0.0f;
  }
  World(const World&) = delete;

  void prepare() {
    flood.build(map);
    flood.floodFrom(in.robot, 0.0f);
    in.map = &map;
    in.roi = kRoi;
    in.grid = &grid;
    in.robot_flood = &flood;
  }
  RoutePlan build() {
    prepare();
    return buildRoute(in, p, pairs);
  }
  int visible(const Eigen::Vector3f& q) const {
    return gridVisibleUnknown(map, kRoi, q, 0.0f, p.hfov, p.h_rays,
                              p.min_range, p.max_range);
  }
};

// Every point is rejected once or is a tier member.
void expectAccounted(const RoutePlan& r) {
  const RouteRejects& j = r.rejects;
  EXPECT_EQ(j.stand + j.too_close + j.map + j.unreachable + j.visited +
                j.failed + j.claim + j.below_tier + r.n_tier_members,
            r.n_points);
  EXPECT_EQ(r.n_admissible, j.below_tier + r.n_tier_members);
}

std::vector<Eigen::Vector3f> positions(const RoutePlan& r) {
  std::vector<Eigen::Vector3f> out;
  for (const auto& w : r.waypoints) out.push_back(w.position);
  return out;
}

void expectSamePlan(const RoutePlan& a, const RoutePlan& b) {
  EXPECT_EQ(a.outcome, b.outcome);
  EXPECT_EQ(a.tier, b.tier);
  EXPECT_EQ(a.served_cell, b.served_cell);
  EXPECT_EQ(a.covered_m2, b.covered_m2);
  EXPECT_EQ(a.covered_later_m2, b.covered_later_m2);
  EXPECT_EQ(a.route_len_m, b.route_len_m);
  EXPECT_EQ(a.order_kept, b.order_kept);
  EXPECT_EQ(a.n_matched_prev, b.n_matched_prev);
  EXPECT_EQ(a.n_points, b.n_points);
  EXPECT_EQ(a.n_admissible, b.n_admissible);
  EXPECT_EQ(a.n_tier_members, b.n_tier_members);
  EXPECT_EQ(a.n_floods, b.n_floods);
  const RouteRejects &x = a.rejects, &y = b.rejects;
  EXPECT_EQ(x.stand, y.stand);
  EXPECT_EQ(x.too_close, y.too_close);
  EXPECT_EQ(x.map, y.map);
  EXPECT_EQ(x.unreachable, y.unreachable);
  EXPECT_EQ(x.visited, y.visited);
  EXPECT_EQ(x.failed, y.failed);
  EXPECT_EQ(x.claim, y.claim);
  EXPECT_EQ(x.below_tier, y.below_tier);
  EXPECT_EQ(a.geom.width, b.geom.width);
  EXPECT_EQ(a.geom.height, b.geom.height);
  EXPECT_EQ(a.geom.origin_x, b.geom.origin_x);
  EXPECT_EQ(a.geom.origin_y, b.geom.origin_y);
  EXPECT_EQ(a.geom.resolution, b.geom.resolution);
  ASSERT_EQ(a.waypoints.size(), b.waypoints.size());
  for (size_t k = 0; k < a.waypoints.size(); ++k) {
    const RouteWaypoint &u = a.waypoints[k], &v = b.waypoints[k];
    EXPECT_EQ(u.position, v.position);
    EXPECT_EQ(u.yaw, v.yaw);
    EXPECT_EQ(u.is_frontier, v.is_frontier);
    EXPECT_EQ(u.candidate_index, v.candidate_index);
    EXPECT_EQ(u.selection_rank, v.selection_rank);
    EXPECT_EQ(u.marginal_cells, v.marginal_cells);
    EXPECT_EQ(u.marginal_later_cells, v.marginal_later_cells);
    EXPECT_EQ(u.leg_cost_m, v.leg_cost_m);
    EXPECT_EQ(u.residual_cells, v.residual_cells);
  }
}

// Pockets of the standard world: three on one row of cell 3 (south doors on
// one row cannot see into each other) and one in cell 1.
const Eigen::Vector3f kA = at(3.75f, 11.25f);
const Eigen::Vector3f kB = at(8.75f, 11.25f);
const Eigen::Vector3f kC = at(13.75f, 11.25f);
const Eigen::Vector3f kD = at(8.75f, -8.75f);

}  // namespace

// ---------------------------------------------------------------------------
// Lattice and small helpers
// ---------------------------------------------------------------------------

// The half offset keeps every point a quarter map cell off any 0.2 m edge and
// 1.25 m off any 20 m boundary, so float and double lookups agree.
TEST(RouteBuilder, LatticeIsHalfOffsetAndOffEveryBoundary) {
  const Roi2D roi{-50.0f, 50.0f, -50.0f, 50.0f};
  const auto pts = routeLattice(roi, 2.5f);
  ASSERT_EQ(pts.size(), 1600u);
  EXPECT_EQ(pts[0], Eigen::Vector2f(-48.75f, -48.75f));
  EXPECT_EQ(pts[1], Eigen::Vector2f(-46.25f, -48.75f));
  EXPECT_EQ(pts[40], Eigen::Vector2f(-48.75f, -46.25f));
  EXPECT_EQ(pts[1599], Eigen::Vector2f(48.75f, 48.75f));

  const CellGrid grid = makeCellGrid(-50.0f, 50.0f, -50.0f, 50.0f, 20.0f);
  std::vector<int> per_cell(static_cast<size_t>(grid.size()), 0);
  int near_boundary = 0, near_edge = 0, floor_flip = 0;
  for (const auto& q : pts) {
    const int id = grid.idAt(q.x(), q.y());
    ASSERT_TRUE(grid.valid(id));
    ++per_cell[static_cast<size_t>(id)];
    for (const double v : {static_cast<double>(q.x()),
                           static_cast<double>(q.y())}) {
      const double e = (v + 50.0) / 20.0;
      if (std::abs(e - std::round(e)) * 20.0 < 1.25 - 1e-4) ++near_boundary;
      for (const double origin : {-75.0, -75.1, -50.3}) {
        const double c = (v - origin) / static_cast<double>(0.2f);
        if (std::abs(c - std::round(c)) <= 1e-3) ++near_edge;
        const float cf = (static_cast<float>(v) - static_cast<float>(origin)) /
                         0.2f;
        if (std::floor(cf) != std::floor(c)) ++floor_flip;
      }
    }
  }
  for (int n : per_cell) EXPECT_EQ(n, 64);
  EXPECT_EQ(near_boundary, 0);
  EXPECT_EQ(near_edge, 0);
  EXPECT_EQ(floor_flip, 0);

  EXPECT_TRUE(routeLattice(roi, 0.0f).empty());
  EXPECT_TRUE(routeLattice(roi, -2.5f).empty());
  EXPECT_TRUE(routeLattice(roi, std::nanf("")).empty());
  EXPECT_TRUE(
      routeLattice(roi, std::numeric_limits<float>::infinity()).empty());
  EXPECT_TRUE(routeLattice({0.0f, 2.4f, 0.0f, 10.0f}, 2.5f).empty());
  EXPECT_TRUE(routeLattice({10.0f, 0.0f, 0.0f, 10.0f}, 2.5f).empty());
  EXPECT_EQ(routeLattice({0.0f, 5.0f, 0.0f, 4.99f}, 2.5f).size(), 2u);
  EXPECT_TRUE(routeLattice({-1e4f, 1e4f, -1e4f, 1e4f}, 0.01f).empty());
}

// The tier threshold is an integer count; float resolution must not round
// 20 m^2 at 0.2 m to 499 or 501, nor one 0.35 m cell (0.1225 m^2) to two.
TEST(RouteBuilder, MinTierCellsIsExactAtFloatResolution) {
  EXPECT_EQ(minTierCells(20.0f, 0.2f), 500);
  EXPECT_EQ(minTierCells(0.1225f, 0.35f), 1);
  EXPECT_EQ(minTierCells(20.0f, 0.25f), 320);
  EXPECT_EQ(minTierCells(20.0f, 0.1f), 2000);
  EXPECT_EQ(minTierCells(20.0f, 0.4f), 125);
  EXPECT_EQ(minTierCells(20.0f, 0.5f), 80);
  EXPECT_EQ(minTierCells(20.01f, 0.2f), 501);
  EXPECT_EQ(minTierCells(19.99f, 0.2f), 500);
  EXPECT_EQ(minTierCells(0.0f, 0.2f), 1);
  EXPECT_EQ(minTierCells(-5.0f, 0.2f), 1);
  EXPECT_EQ(minTierCells(20.0f, 0.0f), std::numeric_limits<int>::max());
  EXPECT_EQ(minTierCells(20.0f, -0.2f), std::numeric_limits<int>::max());
  EXPECT_EQ(minTierCells(std::nanf(""), 0.2f), std::numeric_limits<int>::max());
}

// Point-to-box distance (R13), not distance to the cell centre.
TEST(RouteBuilder, DistToCellBoxIsPointToBox) {
  const CellGrid g = makeCellGrid(-20.0f, 20.0f, -20.0f, 20.0f, 20.0f);
  EXPECT_FLOAT_EQ(distToCellBox(g, 3, 10.0f, 10.0f), 0.0f);
  EXPECT_FLOAT_EQ(distToCellBox(g, 3, 0.0f, 20.0f), 0.0f);
  EXPECT_FLOAT_EQ(distToCellBox(g, 3, -3.0f, 10.0f), 3.0f);
  EXPECT_FLOAT_EQ(distToCellBox(g, 3, -3.0f, -4.0f), 5.0f);
  EXPECT_FLOAT_EQ(distToCellBox(g, 3, 25.0f, 24.0f), std::hypot(5.0f, 4.0f));
  EXPECT_FLOAT_EQ(distToCellBox(g, 0, 10.0f, 10.0f), std::hypot(10.0f, 10.0f));
  EXPECT_TRUE(std::isinf(distToCellBox(g, -1, 0.0f, 0.0f)));
  EXPECT_TRUE(std::isinf(distToCellBox(g, 4, 0.0f, 0.0f)));
}

// ---------------------------------------------------------------------------
// buildRoute: tiers, stand, coverage, filters
// ---------------------------------------------------------------------------

// One pocket in k is served by tier 1 with exact outputs. The odd origin
// checks the census classification does not assume an aligned map.
TEST(RouteBuilder, Tier1ServesK) {
  World w(-20.1f);
  addPocket(w.map, kA, 11, 11);
  addPocket(w.map, kD, 11, 11);
  ASSERT_EQ(w.visible(kA), 520);
  w.in.tier_cell[1] = 1;
  w.in.later_cells[0] = {1};
  const RoutePlan r = w.build();
  ASSERT_EQ(r.outcome, RoutePlan::Outcome::kSingle);
  EXPECT_EQ(r.tier, 1);
  EXPECT_EQ(r.served_cell, 3);
  EXPECT_EQ(r.n_points, 256);
  EXPECT_EQ(r.n_tier_members, 1);
  EXPECT_EQ(r.n_floods, 0);
  EXPECT_FALSE(r.order_kept);
  EXPECT_EQ(r.n_matched_prev, 0);
  expectAccounted(r);
  ASSERT_EQ(r.waypoints.size(), 1u);
  const RouteWaypoint& wp = r.waypoints[0];
  EXPECT_EQ(wp.position, kA);
  EXPECT_FALSE(wp.is_frontier);
  EXPECT_EQ(wp.candidate_index, latIdx(kA.x(), kA.y()));
  EXPECT_EQ(wp.selection_rank, 0);
  EXPECT_EQ(wp.marginal_cells, 520);
  EXPECT_EQ(wp.marginal_later_cells, 0);
  EXPECT_EQ(wp.leg_cost_m, w.flood.costTo(kA));
  EXPECT_EQ(r.route_len_m, wp.leg_cost_m);
  EXPECT_FLOAT_EQ(wp.yaw, std::atan2(kA.y() - w.in.robot.y(),
                                     kA.x() - w.in.robot.x()));
  EXPECT_FLOAT_EQ(r.covered_m2,
                  static_cast<float>(520 * static_cast<double>(0.2f) * 0.2f));
  EXPECT_EQ(r.covered_later_m2, 0.0f);

  const auto [cx, cy] = cellOf(w.map, kA);
  ASSERT_EQ(wp.residual_cells.size(), 520u);
  EXPECT_TRUE(
      std::is_sorted(wp.residual_cells.begin(), wp.residual_cells.end()));
  EXPECT_EQ(std::adjacent_find(wp.residual_cells.begin(),
                               wp.residual_cells.end()),
            wp.residual_cells.end());
  for (int32_t i : wp.residual_cells) {
    const int gx = i % 200, gy = i / 200;
    EXPECT_TRUE(std::abs(gx - cx) <= 11 && std::abs(gy - cy) <= 11) << i;
    EXPECT_LT(w.map.data[static_cast<size_t>(i)], 0);
  }
  EXPECT_EQ(routeStillUnknown(w.map, r.geom, wp.residual_cells), 520);
  EXPECT_EQ(r.geom.width, 200);
  EXPECT_EQ(r.geom.height, 200);
  EXPECT_EQ(r.geom.origin_x, w.map.info.origin.position.x);
  EXPECT_EQ(r.geom.origin_y, w.map.info.origin.position.y);
  EXPECT_EQ(r.geom.resolution, 0.2f);
}

// A k pocket one cell short of 20 m^2 (499) defers to k+1; at 500 k is served.
TEST(RouteBuilder, Tier2OnlyWhenTier1Empty) {
  for (const int extra : {21, 20}) {
    SCOPED_TRACE(extra);
    World w;
    addPocket(w.map, kA, 11, 11, extra);
    addPocket(w.map, kD, 11, 11);
    ASSERT_EQ(w.visible(kA), 520 - extra);
    w.in.tier_cell[1] = 1;
    w.in.later_cells[0] = {1};
    const RoutePlan r = w.build();
    expectAccounted(r);
    ASSERT_EQ(r.waypoints.size(), 1u);
    EXPECT_EQ(r.n_tier_members, 1);
    if (extra == 21) {
      EXPECT_EQ(r.tier, 2);
      EXPECT_EQ(r.served_cell, 1);
      EXPECT_EQ(r.waypoints[0].position, kD);
      EXPECT_EQ(r.waypoints[0].marginal_cells, 520);
    } else {
      EXPECT_EQ(r.tier, 1);
      EXPECT_EQ(r.served_cell, 3);
      EXPECT_EQ(r.waypoints[0].position, kA);
      EXPECT_EQ(r.waypoints[0].marginal_cells, 500);
      EXPECT_EQ(r.waypoints[0].marginal_later_cells, 0);
    }
  }
}

// No member in either tier: kNone, every admissible point below tier.
TEST(RouteBuilder, NeitherTierReturnsNone) {
  World w;
  addPocket(w.map, kA, 11, 11, 21);
  addPocket(w.map, kD, 11, 11, 21);
  w.in.tier_cell[1] = 1;
  RoutePlan r = w.build();
  EXPECT_EQ(r.outcome, RoutePlan::Outcome::kNone);
  EXPECT_EQ(r.tier, 0);
  EXPECT_EQ(r.served_cell, -1);
  EXPECT_TRUE(r.waypoints.empty());
  EXPECT_EQ(r.n_tier_members, 0);
  EXPECT_GT(r.n_admissible, 100);
  EXPECT_EQ(r.rejects.below_tier, r.n_admissible);
  EXPECT_EQ(r.covered_m2, 0.0f);
  EXPECT_EQ(r.route_len_m, 0.0f);
  EXPECT_EQ(r.n_floods, 0);
  expectAccounted(r);

  // No second tier, or an id that is not a cell, is the same.
  for (const int t2 : {-1, 99}) {
    w.in.tier_cell[1] = t2;
    r = w.build();
    EXPECT_EQ(r.outcome, RoutePlan::Outcome::kNone);
    EXPECT_EQ(r.rejects.below_tier, r.n_admissible);
  }
}

// Unusable inputs give kNone with every counter zero.
TEST(RouteBuilder, GuardsReturnNoneWithZeroCounters) {
  World w;
  addPocket(w.map, kA, 11, 11);
  ASSERT_EQ(w.build().outcome, RoutePlan::Outcome::kSingle);
  auto expectEmpty = [](const RoutePlan& r) {
    EXPECT_EQ(r.outcome, RoutePlan::Outcome::kNone);
    EXPECT_EQ(r.tier, 0);
    EXPECT_TRUE(r.waypoints.empty());
    EXPECT_EQ(r.n_points, 0);
    EXPECT_EQ(r.n_admissible, 0);
    EXPECT_EQ(r.n_tier_members, 0);
    EXPECT_EQ(r.rejects.stand + r.rejects.too_close + r.rejects.map +
                  r.rejects.below_tier, 0);
  };
  const CellGrid empty_grid;
  auto bad_map = w.map;
  bad_map.data.pop_back();
  for (int c = 0; c < 7; ++c) {
    SCOPED_TRACE(c);
    w.prepare();
    RouteInputs in = w.in;
    RouteParams p = w.p;
    switch (c) {
      case 0: in.map = nullptr; break;
      case 1: in.map = &bad_map; break;
      case 2: in.grid = nullptr; break;
      case 3: in.grid = &empty_grid; break;
      case 4: in.robot_flood = nullptr; break;
      case 5: in.tier_cell[0] = 4; break;
      case 6: p.max_viewpoints = 0; break;
    }
    const RoutePlan r = buildRoute(in, p, w.pairs);
    expectEmpty(r);
    EXPECT_EQ(r.geom.width, c == 0 ? 0 : 200);
  }
  w.prepare();
  RouteInputs in = w.in;
  in.tier_cell[0] = -1;
  expectEmpty(buildRoute(in, w.p, w.pairs));
}

// Cell 3 holds a west-open block only cell 2 can see (2400 cells) and a
// pocket whose pad is a frontier centre (520). With route {3, 1} the richer
// cell 2 must not stand; every viewpoint is within max_range of the tier box.
TEST(RouteBuilder, ViewpointsStandOnlyInRouteCellsWithinRange) {
  World w;
  w.p.max_range = 8.0f;
  fillRect(w.map, 100, 109, 130, 190, 100);
  fillRect(w.map, 100, 110, 129, 189, -1);
  const Eigen::Vector3f pad = at(13.1f, 13.1f);
  addPocket(w.map, pad, 11, 11);
  ASSERT_EQ(w.visible(pad), 520);
  ASSERT_GT(w.visible(at(-1.25f, 11.25f)), 1000);
  w.in.frontier_centres = {pad, at(-5.0f, 5.0f), at(25.0f, 0.0f)};

  auto check = [&](const RoutePlan& r) {
    expectAccounted(r);
    EXPECT_EQ(r.n_points, 259);
    ASSERT_FALSE(r.waypoints.empty());
    for (const auto& wp : r.waypoints) {
      const int id = w.grid.idAt(wp.position.x(), wp.position.y());
      EXPECT_NE(std::find(w.in.route_cells.begin(), w.in.route_cells.end(), id),
                w.in.route_cells.end());
      EXPECT_LE(distToCellBox(w.grid, 3, wp.position.x(), wp.position.y()),
                8.0f);
    }
  };

  // 64 + 64 in cells 0 and 2, 40 of cell 1 beyond 8 m, two frontiers.
  w.in.route_cells = {3, 1};
  RoutePlan r = w.build();
  check(r);
  EXPECT_EQ(r.rejects.stand, 170);
  ASSERT_EQ(r.waypoints.size(), 1u);
  EXPECT_EQ(r.waypoints[0].position, pad);
  EXPECT_TRUE(r.waypoints[0].is_frontier);
  EXPECT_EQ(r.waypoints[0].candidate_index, 256);

  // In range of either tier: all of cell 1 is within range of itself.
  w.in.tier_cell[1] = 1;
  r = w.build();
  check(r);
  EXPECT_EQ(r.rejects.stand, 130);
  EXPECT_EQ(r.tier, 1);
  EXPECT_EQ(positions(r), std::vector<Eigen::Vector3f>{pad});

  // Made a route cell, cell 2 wins the block.
  w.in.tier_cell[1] = -1;
  w.in.route_cells = {3, 1, 2};
  r = w.build();
  check(r);
  EXPECT_EQ(r.rejects.stand, 64 + 40 + 40 + 1);
  bool in_cell2 = false;
  for (const auto& wp : r.waypoints)
    in_cell2 |= w.grid.idAt(wp.position.x(), wp.position.y()) == 2;
  EXPECT_TRUE(in_cell2);
}

// A pad in cell 1 sees ~10k unknown cells there but 400 inside k: not a
// member. At 500 inside k it is.
TEST(RouteBuilder, TierGainCountsOnlyInsideTheTierCell) {
  const Eigen::Vector3f pad = at(8.75f, -1.25f);
  for (const int cols : {20, 25}) {
    SCOPED_TRACE(cols);
    World w;
    fillRect(w.map, 0, 0, 199, 99, -1);
    fillRect(w.map, 142, 92, 144, 94, 0);
    fillRect(w.map, 133, 100, 133 + cols - 1, 119, -1);
    ASSERT_GT(w.visible(pad), 10000);
    w.in.route_cells = {1};
    w.in.later_cells[0] = {1};
    const RoutePlan r = w.build();
    expectAccounted(r);
    EXPECT_EQ(r.n_admissible, 1);
    if (cols == 20) {
      EXPECT_EQ(r.outcome, RoutePlan::Outcome::kNone);
      EXPECT_EQ(r.rejects.below_tier, 1);
    } else {
      ASSERT_EQ(r.waypoints.size(), 1u);
      EXPECT_EQ(r.waypoints[0].position, pad);
      EXPECT_EQ(r.waypoints[0].marginal_cells, 500);
      EXPECT_GT(r.waypoints[0].marginal_later_cells, 5000);
      EXPECT_FLOAT_EQ(r.covered_later_m2,
                      r.waypoints[0].marginal_later_cells * 0.2f * 0.2f);
    }
  }
}

// A and B tie on k (520). B's door looks onto unknown in later cell 1, so B
// is picked first. C sees less of k (510) but the most later unknown: tier
// gain still outranks later gain.
TEST(RouteBuilder, LaterGainBreaksTies) {
  World w;
  addPocket(w.map, kA, 11, 11);
  addPocket(w.map, kB, 11, 11);
  addPocket(w.map, kC, 11, 11, 10);
  fillRect(w.map, 136, 85, 147, 99, -1);
  fillRect(w.map, 160, 60, 175, 99, -1);
  w.in.later_cells[0] = {1};
  RoutePlan r = w.build();
  expectAccounted(r);
  ASSERT_EQ(r.waypoints.size(), 3u);
  std::vector<const RouteWaypoint*> by_rank(3);
  for (const auto& wp : r.waypoints) by_rank[wp.selection_rank] = &wp;
  EXPECT_EQ(by_rank[0]->position, kB);
  EXPECT_EQ(by_rank[1]->position, kA);
  EXPECT_EQ(by_rank[2]->position, kC);
  EXPECT_EQ(by_rank[0]->marginal_cells, 520);
  EXPECT_EQ(by_rank[1]->marginal_cells, 520);
  EXPECT_EQ(by_rank[2]->marginal_cells, 510);
  EXPECT_GT(by_rank[0]->marginal_later_cells, 0);
  EXPECT_EQ(by_rank[1]->marginal_later_cells, 0);
  EXPECT_GT(by_rank[2]->marginal_later_cells, by_rank[0]->marginal_later_cells);
  EXPECT_FLOAT_EQ(r.covered_m2, (520 + 520 + 510) * 0.2f * 0.2f);

  // Without later cells the tie goes to the lower candidate index.
  w.in.later_cells[0].clear();
  r = w.build();
  ASSERT_EQ(r.waypoints.size(), 3u);
  for (const auto& wp : r.waypoints) by_rank[wp.selection_rank] = &wp;
  EXPECT_EQ(by_rank[0]->position, kA);
  EXPECT_EQ(by_rank[1]->position, kB);
  EXPECT_EQ(by_rank[0]->marginal_later_cells, 0);
}

// The cover stops at max_viewpoints and when the best marginal k gain falls
// below 20 m^2, even when standalone members remain.
TEST(RouteBuilder, CoverStopsBelowMinGainAndAtMax) {
  const Eigen::Vector3f p4 = at(16.25f, 16.25f);
  auto ranked = [](const RoutePlan& r) {
    std::vector<int> idx(r.waypoints.size());
    for (const auto& wp : r.waypoints)
      idx[static_cast<size_t>(wp.selection_rank)] = wp.candidate_index;
    return idx;
  };
  {
    World w;
    for (const auto& q : {kA, kB, kC, p4}) addPocket(w.map, q, 11, 11);
    w.p.max_viewpoints = 3;
    RoutePlan r = w.build();
    expectAccounted(r);
    EXPECT_EQ(r.n_tier_members, 4);
    EXPECT_EQ(ranked(r), (std::vector<int>{latIdx(3.75f, 11.25f),
                                           latIdx(8.75f, 11.25f),
                                           latIdx(13.75f, 11.25f)}));
    for (const auto& wp : r.waypoints) EXPECT_EQ(wp.marginal_cells, 520);
    w.p.max_viewpoints = 1;
    r = w.build();
    EXPECT_EQ(r.outcome, RoutePlan::Outcome::kSingle);
    EXPECT_EQ(positions(r), std::vector<Eigen::Vector3f>{kA});
    w.p.max_viewpoints = 12;
    r = w.build();
    EXPECT_EQ(r.waypoints.size(), 4u);
    EXPECT_EQ(r.outcome, RoutePlan::Outcome::kRoute);
  }
  {
    // A 352-cell pocket is no member.
    World w;
    for (const auto& q : {kA, kB, kC}) addPocket(w.map, q, 11, 11);
    addPocket(w.map, p4, 9, 9);
    ASSERT_EQ(w.visible(p4), 352);
    const RoutePlan r = w.build();
    EXPECT_EQ(r.n_tier_members, 3);
    EXPECT_EQ(r.waypoints.size(), 3u);
  }
  {
    // Two pads in one wide room: both members, the second adds nothing.
    World w;
    addPocket(w.map, kB, 30, 11);
    const Eigen::Vector3f pad2 = at(6.25f, 11.25f);
    const auto [px, py] = cellOf(w.map, pad2);
    fillRect(w.map, px - 1, py - 1, px + 1, py + 1, 0);
    const int seen = w.visible(pad2);
    ASSERT_GE(seen, 1300);
    const RoutePlan r = w.build();
    expectAccounted(r);
    EXPECT_EQ(r.n_tier_members, 2);
    ASSERT_EQ(r.waypoints.size(), 1u);
    EXPECT_EQ(r.waypoints[0].position, pad2);
    EXPECT_EQ(r.waypoints[0].marginal_cells, seen);
  }
  {
    // Two halves of a room joined by a gap: the second pick counts only what
    // the first left, and covered_m2 is the union, not the sum.
    World w;
    addPocket(w.map, kB, 30, 11);
    const auto [bx, by] = cellOf(w.map, kB);
    fillRect(w.map, bx, by - 7, bx, by + 11, 100);
    const Eigen::Vector3f west = at(6.25f, 11.25f), east = at(11.25f, 11.25f);
    std::vector<int32_t> seen_by[2], uni;
    for (int k = 0; k < 2; ++k) {
      const auto [x, y] = cellOf(w.map, k ? east : west);
      fillRect(w.map, x - 1, y - 1, x + 1, y + 1, 0);
    }
    for (int k = 0; k < 2; ++k) {
      gridVisibleUnknown(w.map, kRoi, k ? east : west, 0.0f, w.p.hfov,
                         w.p.h_rays, 0.0f, w.p.max_range, &seen_by[k]);
      uni.insert(uni.end(), seen_by[k].begin(), seen_by[k].end());
    }
    std::sort(uni.begin(), uni.end());
    uni.erase(std::unique(uni.begin(), uni.end()), uni.end());
    const RoutePlan r = w.build();
    expectAccounted(r);
    ASSERT_EQ(r.waypoints.size(), 2u);
    std::vector<int32_t> residual;
    int sum = 0;
    for (const auto& wp : r.waypoints) {
      sum += wp.marginal_cells;
      EXPECT_EQ(static_cast<int>(wp.residual_cells.size()), wp.marginal_cells);
      residual.insert(residual.end(), wp.residual_cells.begin(),
                      wp.residual_cells.end());
      if (wp.selection_rank == 1) {
        EXPECT_GE(wp.marginal_cells, 500);
        EXPECT_LT(wp.marginal_cells, w.visible(wp.position));
      }
    }
    std::sort(residual.begin(), residual.end());
    EXPECT_EQ(residual, uni);
    EXPECT_EQ(sum, static_cast<int>(uni.size()));
    EXPECT_FLOAT_EQ(r.covered_m2,
                    static_cast<float>(sum * static_cast<double>(0.2f) * 0.2f));
  }
}

// Each walk filter, in order, takes its points and counts them once.
TEST(RouteBuilder, EachFilterRejectsAndIsCounted) {
  World w;
  w.in.robot = at(11.25f, 11.25f);
  w.in.route_cells = {3};
  auto cell = [&](float x, float y) { return cellOf(w.map, at(x, y)); };
  {
    const auto [x, y] = cell(13.75f, 11.25f);  // too close beats occupied
    setCell(w.map, x, y, 100);
  }
  {
    const auto [x, y] = cell(3.75f, 3.75f);
    setCell(w.map, x, y, 100);
  }
  {
    const auto [x, y] = cell(6.25f, 3.75f);
    setCell(w.map, x, y, -1);
  }
  const Eigen::Vector3f sealed = at(16.25f, 16.25f);
  {
    const auto [x, y] = cellOf(w.map, sealed);
    fillRect(w.map, x - 1, y - 1, x + 1, y + 1, 100);
    setCell(w.map, x, y, 0);
  }
  const Eigen::Vector3f top = at(3.75f, 18.75f), failed_pt = at(8.75f, 3.75f),
                        claimed = at(16.25f, 6.25f);
  FailedGoalBlacklist visited, failed;
  visited.add(top + Eigen::Vector3f(0.0f, 2.4f, 0.0f), 0.0);
  visited.add(sealed, 0.0);       // unreachable comes first
  failed.add(failed_pt, 0.0);
  failed.add(top, 0.0);           // visited comes first
  w.in.visited = &visited;
  w.in.failed = &failed;
  auto near = [](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return std::abs(a.x() - b.x()) < 1e-3f && std::abs(a.y() - b.y()) < 1e-3f;
  };
  w.in.lost_to_claim = [&](const Eigen::Vector3f& q) {
    return near(q, claimed) || near(q, failed_pt);  // failed comes first
  };
  auto expectRejects = [](const RoutePlan& r, std::vector<int> want) {
    const RouteRejects& j = r.rejects;
    EXPECT_EQ((std::vector<int>{j.stand, j.too_close, j.map, j.unreachable,
                                j.visited, j.failed, j.claim, j.below_tier}),
              want);
    EXPECT_EQ(r.outcome, RoutePlan::Outcome::kNone);
    EXPECT_EQ(r.n_points, 256);
    EXPECT_EQ(r.n_admissible, want[7]);
  };
  expectRejects(w.build(), {192, 9, 2, 1, 1, 1, 1, 49});

  // 2.6 m from the visited entry is outside the 2.5 m radius.
  FailedGoalBlacklist visited_far;
  visited_far.add(top + Eigen::Vector3f(0.0f, 2.6f, 0.0f), 0.0);
  visited_far.add(sealed, 0.0);
  w.in.visited = &visited_far;
  expectRejects(w.build(), {192, 9, 2, 1, 0, 2, 1, 49});

  w.in.visited = &visited;
  w.in.skip_reachability = true;
  expectRejects(w.build(), {192, 9, 2, 0, 2, 1, 1, 49});

  w.in.skip_reachability = false;
  w.in.visited = nullptr;
  w.in.failed = nullptr;
  w.in.lost_to_claim = nullptr;
  expectRejects(w.build(), {192, 9, 2, 1, 0, 0, 0, 52});

  // too_close is strict: the four points exactly 5 m away stay.
  w.p.min_goal_dist_m = 5.0;
  expectRejects(w.build(), {192, 9, 2, 1, 0, 0, 0, 52});
  w.p.min_goal_dist_m = 5.01;
  expectRejects(w.build(), {192, 13, 2, 1, 0, 0, 0, 48});
}

// ---------------------------------------------------------------------------
// Cover and order helpers
// ---------------------------------------------------------------------------

// The lazy heap must pick exactly what an eager greedy over (tier, later,
// -index) picks, with the same marginals and marginal tier sets.
TEST(RouteBuilder, LazyGreedyEqualsEagerGreedy) {
  std::mt19937 rng(20261012);
  auto uni = [&](int lo, int hi) {
    return std::uniform_int_distribution<int>(lo, hi)(rng);
  };
  auto real = [&]() {
    return std::uniform_real_distribution<double>(0, 1)(rng);
  };
  int multi = 0, stopped_by_gain = 0;
  for (int trial = 0; trial < 600; ++trial) {
    SCOPED_TRACE(trial);
    const int u_tier = uni(1, 200), u_later = uni(0, 200);
    const size_t universe = static_cast<size_t>(u_tier + u_later);
    const int m = uni(0, 40);
    std::vector<CoverSet> sets;
    for (int k = 0; k < m; ++k) {
      if (k > 0 && uni(0, 5) == 0) {  // duplicates force exact ties
        sets.push_back(sets[static_cast<size_t>(uni(0, k - 1))]);
        continue;
      }
      CoverSet s;
      const double dt = 0.5 * real(), dl = 0.5 * real();
      for (int i = 0; i < u_tier; ++i)
        if (real() < dt) s.tier.push_back(i);
      for (int i = 0; i < u_later; ++i)
        if (real() < dl) s.later.push_back(u_tier + i);
      sets.push_back(std::move(s));
    }
    const int min_tier = uni(1, 12), max_picks = uni(0, 15);

    std::vector<uint8_t> covered(universe, 0), used(static_cast<size_t>(m), 0);
    std::vector<int> e_picks, e_t, e_l;
    std::vector<std::vector<int32_t>> e_sets;
    while (static_cast<int>(e_picks.size()) < max_picks) {
      int best = -1, bt = -1, bl = -1;
      for (int k = 0; k < m; ++k) {
        if (used[k]) continue;
        int t = 0, l = 0;
        for (int32_t i : sets[k].tier) t += !covered[i];
        for (int32_t i : sets[k].later) l += !covered[i];
        if (t > bt || (t == bt && l > bl)) { best = k; bt = t; bl = l; }
      }
      if (best < 0) break;
      if (bt < min_tier) { ++stopped_by_gain; break; }
      std::vector<int32_t> marg;
      for (int32_t i : sets[best].tier)
        if (!covered[i]) { covered[i] = 1; marg.push_back(i); }
      for (int32_t i : sets[best].later) covered[i] = 1;
      used[best] = 1;
      e_picks.push_back(best);
      e_t.push_back(bt);
      e_l.push_back(bl);
      e_sets.push_back(std::move(marg));
    }

    std::vector<int> mt{99}, ml{99};
    std::vector<std::vector<int32_t>> ms{{7}};
    const auto picks =
        lazyGreedyCover(sets, universe, min_tier, max_picks, &mt, &ml, &ms);
    EXPECT_EQ(picks, e_picks);
    EXPECT_EQ(mt, e_t);
    EXPECT_EQ(ml, e_l);
    EXPECT_EQ(ms, e_sets);
    EXPECT_EQ(lazyGreedyCover(sets, universe, min_tier, max_picks), e_picks);
    if (e_picks.size() >= 2) ++multi;
  }
  EXPECT_GT(multi, 150);
  EXPECT_GT(stopped_by_gain, 100);
}

namespace {

// Nearest insertion as specified, without 2-opt: the bound orderOpenRoute
// must meet or beat.
std::vector<int> insertionOnly(const std::vector<float>& s,
                               const std::vector<std::vector<float>>& d) {
  const int n = static_cast<int>(s.size());
  std::vector<int> path;
  std::vector<bool> in(static_cast<size_t>(n), false);
  for (int step = 0; step < n; ++step) {
    int v = -1;
    float best = 0.0f;
    for (int u = 0; u < n; ++u) {
      if (in[u]) continue;
      float du = s[u];
      for (int a : path) du = std::min(du, d[a][u]);
      if (v < 0 || du < best) { v = u; best = du; }
    }
    size_t slot = 0;
    if (!path.empty()) {
      float c = s[v] + d[v][path[0]] - s[path[0]];
      for (size_t k = 1; k < path.size(); ++k) {
        const float ck =
            d[path[k - 1]][v] + d[v][path[k]] - d[path[k - 1]][path[k]];
        if (ck < c) { c = ck; slot = k; }
      }
      if (d[path.back()][v] < c) slot = path.size();
    }
    path.insert(path.begin() + static_cast<std::ptrdiff_t>(slot), v);
    in[v] = true;
  }
  return path;
}

}  // namespace

// Random Euclidean instances: a permutation, never longer than insertion
// alone, and 2-opt locally optimal. Collinear points come out in order.
TEST(RouteBuilder, OpenRouteIsAPermutationNoLongerThanInsertion) {
  std::mt19937 rng(20261012);
  auto uni = [&](int lo, int hi) {
    return std::uniform_int_distribution<int>(lo, hi)(rng);
  };
  auto real = [&](float lo, float hi) {
    return std::uniform_real_distribution<float>(lo, hi)(rng);
  };
  int shorter = 0;
  for (int trial = 0; trial < 400; ++trial) {
    SCOPED_TRACE(trial);
    const int n = uni(0, 13);
    std::vector<Eigen::Vector2f> q(static_cast<size_t>(n));
    for (auto& v : q) v = {real(0, 50), real(0, 50)};
    const Eigen::Vector2f st(real(0, 50), real(0, 50));
    std::vector<float> s(static_cast<size_t>(n));
    std::vector<std::vector<float>> d(
        static_cast<size_t>(n), std::vector<float>(static_cast<size_t>(n)));
    for (int i = 0; i < n; ++i) {
      s[i] = (q[i] - st).norm();
      for (int j = 0; j < n; ++j) d[i][j] = (q[i] - q[j]).norm();
    }
    const auto order = orderOpenRoute(s, d);
    std::vector<int> sorted = order, iota(static_cast<size_t>(n));
    std::sort(sorted.begin(), sorted.end());
    std::iota(iota.begin(), iota.end(), 0);
    ASSERT_EQ(sorted, iota);
    const float len = openRouteLength(order, s, d);
    const float ins = openRouteLength(insertionOnly(s, d), s, d);
    EXPECT_LE(len, ins + 1e-3f);
    if (len < ins - 1e-3f) ++shorter;
    for (int i = 0; i + 1 < n; ++i)
      for (int j = i + 1; j < n; ++j) {
        const float before =
            (i == 0 ? s[order[i]] : d[order[i - 1]][order[i]]) +
            (j < n - 1 ? d[order[j]][order[j + 1]] : 0.0f);
        const float after =
            (i == 0 ? s[order[j]] : d[order[i - 1]][order[j]]) +
            (j < n - 1 ? d[order[i]][order[j + 1]] : 0.0f);
        EXPECT_GE(after - before, -1e-3f) << i << "," << j;
      }
  }
  EXPECT_GT(shorter, 20);

  for (int trial = 0; trial < 50; ++trial) {
    const int n = uni(1, 13);
    std::vector<int> xs(static_cast<size_t>(n));
    std::iota(xs.begin(), xs.end(), 1);
    std::shuffle(xs.begin(), xs.end(), rng);
    std::vector<float> s(static_cast<size_t>(n));
    std::vector<std::vector<float>> d(
        static_cast<size_t>(n), std::vector<float>(static_cast<size_t>(n)));
    for (int i = 0; i < n; ++i) {
      s[i] = static_cast<float>(xs[i]);
      for (int j = 0; j < n; ++j)
        d[i][j] = std::abs(static_cast<float>(xs[i] - xs[j]));
    }
    std::vector<int> want(static_cast<size_t>(n));
    std::iota(want.begin(), want.end(), 0);
    std::sort(want.begin(), want.end(),
              [&](int a, int b) { return xs[a] < xs[b]; });
    EXPECT_EQ(orderOpenRoute(s, d), want);
    EXPECT_FLOAT_EQ(openRouteLength(want, s, d), static_cast<float>(n));
  }
  // Ties: the lower index goes in first, at the earliest cheapest slot. All
  // three orders below are 6 long.
  EXPECT_EQ(orderOpenRoute({5.0f, 5.0f}, {{0.0f, 1.0f}, {1.0f, 0.0f}}),
            (std::vector<int>{1, 0}));
  EXPECT_EQ(orderOpenRoute({1.0f, 3.0f, 2.0f}, {{0.0f, 2.0f, 2.0f},
                                                {2.0f, 0.0f, 3.0f},
                                                {2.0f, 3.0f, 0.0f}}),
            (std::vector<int>{2, 0, 1}));
  EXPECT_TRUE(orderOpenRoute({}, {}).empty());
  EXPECT_EQ(openRouteLength({}, {}, {}), 0.0f);
  EXPECT_FLOAT_EQ(
      openRouteLength({1, 0}, {3.0f, 5.0f}, {{0.0f, 2.0f}, {2.0f, 0.0f}}),
      7.0f);
}

// ---------------------------------------------------------------------------
// Continuity, refresh helpers, pair costs, determinism, timing
// ---------------------------------------------------------------------------

// Three pockets A, B, C on one row; the fresh order is A, B, C. A held order
// is kept unless the fresh one is reorder_gain shorter; picks within 1.25 m
// of a held waypoint keep its place and the rest go in at their cheapest slot.
TEST(RouteBuilder, ContinuityKeepsOldOrderUnlessTenPercentShorter) {
  auto plan = [](const std::vector<Eigen::Vector3f>& prev, float gain) {
    World w;
    for (const auto& q : {kA, kB, kC}) addPocket(w.map, q, 11, 11);
    w.in.prev_remaining = prev;
    w.p.reorder_gain = gain;
    return w.build();
  };
  using V = std::vector<Eigen::Vector3f>;
  const V abc{kA, kB, kC}, cba{kC, kB, kA};

  RoutePlan r = plan({}, 0.1f);
  ASSERT_EQ(positions(r), abc);
  EXPECT_FALSE(r.order_kept);
  const float l_fresh = r.route_len_m;

  r = plan(cba, 0.99f);  // kept as held, with no 2-opt
  EXPECT_EQ(positions(r), cba);
  EXPECT_TRUE(r.order_kept);
  EXPECT_EQ(r.n_matched_prev, 3);
  const float l_seed = r.route_len_m;
  ASSERT_LE(l_fresh, 0.9f * l_seed);

  r = plan(cba, 0.1f);
  EXPECT_EQ(positions(r), abc);
  EXPECT_FALSE(r.order_kept);
  EXPECT_EQ(r.n_matched_prev, 3);

  const float ratio = l_fresh / l_seed;
  r = plan(cba, 1.0f - ratio - 0.005f);
  EXPECT_EQ(positions(r), abc);
  EXPECT_FALSE(r.order_kept);
  r = plan(cba, 1.0f - ratio + 0.005f);
  EXPECT_EQ(positions(r), cba);
  EXPECT_TRUE(r.order_kept);

  // Equal lengths keep the held order, unless the gain is 0 (<= holds).
  r = plan({kA, kB}, 0.1f);
  EXPECT_EQ(positions(r), abc);
  EXPECT_TRUE(r.order_kept);
  EXPECT_EQ(r.n_matched_prev, 2);
  r = plan({kA, kB}, 0.0f);
  EXPECT_EQ(positions(r), abc);
  EXPECT_FALSE(r.order_kept);

  // Unmatched picks go in at the cheapest slot: interior, then head.
  r = plan({kA, kC}, 0.99f);
  EXPECT_EQ(positions(r), abc);
  EXPECT_TRUE(r.order_kept);
  r = plan({kB, kC}, 0.99f);
  EXPECT_EQ(positions(r), abc);
  EXPECT_TRUE(r.order_kept);

  // Identity is 1.25 m inclusive; one held waypoint matches one pick.
  const Eigen::Vector3f dx(1.0f, 0.0f, 0.0f);
  for (const float off : {1.24f, 1.25f, 1.26f}) {
    SCOPED_TRACE(off);
    r = plan({kC + off * dx}, 0.99f);
    EXPECT_EQ(r.n_matched_prev, off <= 1.25f ? 1 : 0);
    EXPECT_EQ(r.order_kept, off <= 1.25f);
    EXPECT_EQ(positions(r), abc);
  }
  r = plan({kC + 1.24f * dx, kB, kA}, 0.99f);
  EXPECT_EQ(positions(r), cba);
  EXPECT_EQ(r.n_matched_prev, 3);
  r = plan({kA + 0.5f * dx, kA + 0.2f * dx}, 0.99f);
  EXPECT_EQ(r.n_matched_prev, 1);

  // Two picks 2.4 m apart (frontier pads of tall rooms sharing a wall), one
  // held waypoint 1.2 m from both: it matches one of them.
  World w;
  w.p.reorder_gain = 0.99f;
  const auto centre = [&](int gx, int gy) {
    return at(static_cast<float>(-20.0 + (gx + 0.5) * 0.2f),
              static_cast<float>(-20.0 + (gy + 0.5) * 0.2f));
  };
  const Eigen::Vector3f q1 = centre(125, 150), q2 = centre(137, 150);
  addPocket(w.map, q1, 5, 25);
  addPocket(w.map, q2, 5, 25);
  ASSERT_EQ(w.visible(q1), 552);
  ASSERT_EQ(w.visible(q2), 552);
  w.in.frontier_centres = {q1, q2};
  w.in.prev_remaining = {0.5f * (q1 + q2)};
  r = w.build();
  ASSERT_EQ(r.waypoints.size(), 2u);
  EXPECT_EQ(r.n_matched_prev, 1);
  EXPECT_TRUE(r.order_kept);

  // Unmatched picks go in by candidate index, not pick order. A (505 cells)
  // is picked after C but goes in first, at the end of the held E; C then
  // goes between them. In pick order it would be E, A, C.
  World u;
  const Eigen::Vector3f kE = at(8.75f, 3.75f);
  addPocket(u.map, kA, 11, 11, 15);
  addPocket(u.map, kC, 11, 11);
  addPocket(u.map, kE, 11, 11);
  u.in.prev_remaining = {kE};
  u.p.reorder_gain = 0.99f;
  r = u.build();
  ASSERT_EQ(r.waypoints.size(), 3u);
  EXPECT_EQ(positions(r), (V{kE, kC, kA}));
  EXPECT_EQ(r.waypoints[1].selection_rank, 1);
  EXPECT_EQ(r.waypoints[2].selection_rank, 2);
  EXPECT_LT(r.waypoints[2].candidate_index, r.waypoints[1].candidate_index);
  EXPECT_TRUE(r.order_kept);
}

// RP §5.4: the head moved more than identity, or the length changed by more
// than reorder_gain of the remaining length.
TEST(RouteBuilder, MaterialChangeThresholds) {
  const RouteParams p;
  const Eigen::Vector3f h = at(0.0f, 0.0f);
  EXPECT_FALSE(routeMaterialChange(at(1.24f, 0.0f), 100.0f, h, 100.0f, p));
  EXPECT_FALSE(routeMaterialChange(at(0.0f, 1.25f), 100.0f, h, 100.0f, p));
  EXPECT_TRUE(routeMaterialChange(at(1.26f, 0.0f), 100.0f, h, 100.0f, p));
  EXPECT_TRUE(routeMaterialChange(at(-0.9f, 0.9f), 100.0f, h, 100.0f, p));
  EXPECT_FALSE(
      routeMaterialChange(Eigen::Vector3f(0, 0, 5), 100.0f, h, 100.0f, p));
  EXPECT_FALSE(routeMaterialChange(h, 109.0f, h, 100.0f, p));
  EXPECT_FALSE(routeMaterialChange(h, 91.0f, h, 100.0f, p));
  EXPECT_FALSE(routeMaterialChange(h, 110.0f, h, 100.0f, p));
  EXPECT_TRUE(routeMaterialChange(h, 111.0f, h, 100.0f, p));
  EXPECT_TRUE(routeMaterialChange(h, 89.0f, h, 100.0f, p));
  EXPECT_TRUE(routeMaterialChange(h, 100.0f, h,
                                  std::numeric_limits<float>::infinity(), p));
  EXPECT_TRUE(routeMaterialChange(h, 100.0f, h, std::nanf(""), p));
  RouteParams q;
  q.spacing_m = 2.0f;
  q.reorder_gain = 0.2f;
  EXPECT_TRUE(routeMaterialChange(at(1.01f, 0.0f), 100.0f, h, 100.0f, q));
  EXPECT_FALSE(routeMaterialChange(h, 119.0f, h, 100.0f, q));
}

// Pair legs come from early-stopping floods out of the earlier pick: exactly
// floodFrom's costs. Pairs no flood joins cost the way through the robot.
TEST(RouteBuilder, PairCostsSymmetricAndEqualFloodFrom) {
  World w;
  const Eigen::Vector3f p4 = at(16.25f, 16.25f);
  for (const auto& q : {kA, kB, kC, p4}) addPocket(w.map, q, 11, 11);
  std::mt19937 rng(20261012);
  std::uniform_int_distribution<int> tx(100, 198), ty(100, 141);
  for (int t = 0; t < 150; ++t) {
    const int x = tx(rng), y = ty(rng);
    if (std::abs(x - 106) <= 5 && std::abs(y - 106) <= 5) continue;
    fillRect(w.map, x, y, x + 1, y + 1, 100);
  }
  const RoutePlan r = w.build();
  expectAccounted(r);
  const size_t n = r.waypoints.size();
  ASSERT_GE(n, 3u);
  EXPECT_EQ(r.n_floods, static_cast<int>(n) - 1);
  EXPECT_EQ(w.pairs.dimsX(), 200);
  EXPECT_EQ(r.waypoints[0].leg_cost_m, w.flood.costTo(r.waypoints[0].position));
  float sum = 0.0f;
  for (size_t k = 0; k < n; ++k) {
    sum += r.waypoints[k].leg_cost_m;
    if (k == 0) continue;
    const RouteWaypoint &a = r.waypoints[k - 1], &b = r.waypoints[k];
    const RouteWaypoint& src = a.selection_rank < b.selection_rank ? a : b;
    const RouteWaypoint& dst = a.selection_rank < b.selection_rank ? b : a;
    CostGrid ref;
    ref.build(w.map);
    ref.floodFrom(src.position, 0.0f);
    EXPECT_EQ(b.leg_cost_m, ref.costTo(dst.position)) << k;
    ref.floodFrom(dst.position, 0.0f);
    EXPECT_NEAR(b.leg_cost_m, ref.costTo(src.position), 1e-3f) << k;
    EXPECT_TRUE(std::isfinite(b.leg_cost_m));
  }
  EXPECT_EQ(r.route_len_m, sum);
  // Each yaw faces the next waypoint; the last keeps its arrival bearing.
  for (size_t k = 0; k < n; ++k) {
    const Eigen::Vector3f& from = r.waypoints[k + 1 < n ? k : k - 1].position;
    const Eigen::Vector3f& to = r.waypoints[k + 1 < n ? k + 1 : k].position;
    EXPECT_FLOAT_EQ(r.waypoints[k].yaw,
                    std::atan2(to.y() - from.y(), to.x() - from.x())) << k;
  }

  World s;
  addPocket(s.map, kA, 11, 11, 0, true);
  addPocket(s.map, kB, 11, 11, 0, true);
  s.in.skip_reachability = true;
  const RoutePlan q = s.build();
  ASSERT_EQ(q.waypoints.size(), 2u);
  EXPECT_EQ(q.n_floods, 1);
  const auto robot = s.in.robot;
  auto eu = [&](const Eigen::Vector3f& v) {
    return std::hypot(v.x() - robot.x(), v.y() - robot.y());
  };
  EXPECT_EQ(q.waypoints[0].leg_cost_m, eu(q.waypoints[0].position));
  EXPECT_FLOAT_EQ(q.waypoints[1].leg_cost_m, eu(kA) + eu(kB));
}

// The residual count refuses a map of another geometry (-1).
TEST(RouteBuilder, StillUnknownRefusesOtherGeometry) {
  World w;
  addPocket(w.map, kA, 11, 11);
  const RoutePlan r = w.build();
  ASSERT_EQ(r.waypoints.size(), 1u);
  std::vector<int32_t> cells = r.waypoints[0].residual_cells;
  EXPECT_EQ(routeStillUnknown(w.map, r.geom, cells), 520);
  for (size_t k = 0; k < cells.size(); k += 2)
    w.map.data[static_cast<size_t>(cells[k])] = 0;
  EXPECT_EQ(routeStillUnknown(w.map, r.geom, cells), 260);
  cells.push_back(-5);
  cells.push_back(200 * 200);
  EXPECT_EQ(routeStillUnknown(w.map, r.geom, cells), 260);
  EXPECT_EQ(routeStillUnknown(w.map, r.geom, {}), 0);

  for (int c = 0; c < 6; ++c) {
    SCOPED_TRACE(c);
    auto g = w.map;
    switch (c) {
      case 0: g.info.width = 199; break;
      case 1: g.info.height = 201; break;
      case 2: g.info.origin.position.x += 0.2; break;
      case 3: g.info.origin.position.y -= 1e-9; break;
      case 4: g.info.resolution = 0.25f; break;
      case 5: g.data.pop_back(); break;
    }
    EXPECT_EQ(routeStillUnknown(g, r.geom, cells), -1);
  }
}

// Same inputs, same plan, field by field; the scratch flood's history does
// not leak into the result.
TEST(RouteBuilder, DeterministicTwice) {
  World w;
  const Eigen::Vector3f p4 = at(16.25f, 16.25f);
  for (const auto& q : {kA, kB, kC, p4}) addPocket(w.map, q, 11, 11);
  fillRect(w.map, 136, 85, 147, 99, -1);
  std::mt19937 rng(7);
  std::uniform_int_distribution<int> tx(100, 198), ty(100, 141);
  for (int t = 0; t < 100; ++t) {
    const int x = tx(rng), y = ty(rng);
    if (std::abs(x - 106) <= 5 && std::abs(y - 106) <= 5) continue;
    fillRect(w.map, x, y, x + 1, y + 1, 100);
  }
  for (const auto& f : findGridFrontiers(w.map, kRoi, 2.0f, 3))
    w.in.frontier_centres.push_back(f.position);
  ASSERT_FALSE(w.in.frontier_centres.empty());
  w.in.later_cells[0] = {1};
  const RoutePlan a = w.build();
  ASSERT_GE(a.waypoints.size(), 3u);
  const RoutePlan b = w.build();
  expectSamePlan(a, b);
  CostGrid other;
  expectSamePlan(a, buildRoute(w.in, w.p, other));

  w.in.prev_remaining = {a.waypoints.back().position, a.waypoints[0].position};
  w.p.reorder_gain = 0.99f;
  const RoutePlan c = w.build();
  EXPECT_TRUE(c.order_kept);
  expectSamePlan(c, w.build());
}

// Not a budget check: prints the solve time on a 500 x 500 map (0.2 m) with
// 30-60 % unknown for the report, asserting only a loose ceiling.
TEST(RouteBuilder, TimingSmoke500) {
  const Roi2D roi{-50.0f, 50.0f, -50.0f, 50.0f};
  const CellGrid grid = makeCellGrid(-50.0f, 50.0f, -50.0f, 50.0f, 20.0f);
  const std::vector<int> all = [] {
    std::vector<int> v(25);
    std::iota(v.begin(), v.end(), 0);
    return v;
  }();
  double worst_ms = 0.0;
  for (const double target : {0.30, 0.45, 0.60}) {
    std::mt19937 rng(20261012);
    std::uniform_real_distribution<float> pos(-50.0f, 50.0f), rad(1.0f, 6.0f);
    auto map = makeGrid(500, 500, 0.2f, 0, -50.0f, -50.0f);
    auto disc = [&](float x, float y, float r, int8_t v) {
      const auto [cx, cy] = cellOf(map, at(x, y));
      const int k = static_cast<int>(std::ceil(r / 0.2f));
      for (int gy = std::max(0, cy - k); gy <= std::min(499, cy + k); ++gy)
        for (int gx = std::max(0, cx - k); gx <= std::min(499, cx + k); ++gx)
          if (std::hypot(gx - cx, gy - cy) * 0.2f <= r) setCell(map, gx, gy, v);
    };
    size_t unknown = 0;
    while (unknown < target * map.data.size()) {
      disc(pos(rng), pos(rng), rad(rng), -1);
      unknown = static_cast<size_t>(
          std::count(map.data.begin(), map.data.end(), int8_t{-1}));
    }
    std::uniform_real_distribution<float> trunk(0.2f, 0.4f);
    for (int t = 0; t < 600; ++t) disc(pos(rng), pos(rng), trunk(rng), 100);
    const Eigen::Vector3f robot = at(1.25f, 1.25f);
    disc(robot.x(), robot.y(), 3.0f, 0);

    CostGrid flood, pairs;
    flood.build(map);
    flood.floodFrom(robot, 0.0f);
    RouteInputs in;
    in.map = &map;
    in.roi = roi;
    in.grid = &grid;
    in.robot = robot;
    in.robot_flood = &flood;
    for (const auto& f : findGridFrontiers(map, roi, 2.0f, 3))
      in.frontier_centres.push_back(f.position);
    RouteParams p;
    for (const int tier : {12, 13, 18}) {
      for (const bool whole : {false, true}) {
        in.route_cells = whole ? all : std::vector<int>{12, 13, 18, 17, 16, 11};
        in.tier_cell[0] = tier;
        in.tier_cell[1] = tier == 12 ? 13 : 12;
        in.later_cells[0] = {17, 16};
        const auto t0 = std::chrono::steady_clock::now();
        const RoutePlan r = buildRoute(in, p, pairs);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();
        worst_ms = std::max(worst_ms, ms);
        std::printf("[timing] unknown %.0f%% tier %d route %s: %.1f ms, "
                    "%d points, %d admissible, %d members, %zu waypoints\n",
                    100.0 * unknown / map.data.size(), tier,
                    whole ? "all 25" : "6 cells", ms, r.n_points,
                    r.n_admissible, r.n_tier_members, r.waypoints.size());
        expectAccounted(r);
      }
    }
  }
  std::printf("[timing] worst %.1f ms\n", worst_ms);
  EXPECT_LT(worst_ms, 5000.0);
}
