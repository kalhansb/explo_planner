#include <gtest/gtest.h>

#include <cmath>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "explo_planner/candidate_snap.hpp"
#include "explo_planner/cost_grid.hpp"

using namespace explo_planner;
using Outcome = CandidateSnapper::Outcome;

namespace {

constexpr float kRes = 0.5f;

nav_msgs::msg::OccupancyGrid makeGrid(int w, int h, int8_t fill = 0) {
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = w;
  g.info.height = h;
  g.info.resolution = kRes;
  g.data.assign(static_cast<size_t>(w) * h, fill);
  return g;
}

void set(nav_msgs::msg::OccupancyGrid& g, int gx, int gy, int8_t v) {
  g.data[static_cast<size_t>(gy) * g.info.width + gx] = v;
}

Eigen::Vector3f centre(int gx, int gy) {
  return {(gx + 0.5f) * kRes, (gy + 0.5f) * kRes, 0.3f};
}

const auto kAcceptAll = [](float, float) { return true; };

}  // namespace

TEST(CandidateSnap, GoodCellIsKept) {
  auto g = makeGrid(21, 21);
  CostGrid cg;
  cg.build(g);
  cg.floodFrom(centre(10, 10), 0.0f);
  CandidateSnapper snap(2.0f);
  Eigen::Vector2f out(-1.0f, -1.0f);
  EXPECT_EQ(snap.snap(cg, centre(3, 3), false, true, kAcceptAll, out),
            Outcome::kKept);
  EXPECT_FLOAT_EQ(out.x(), -1.0f);  // untouched
}

TEST(CandidateSnap, BlockedCellSnapsToNearestFreeCell) {
  auto g = makeGrid(21, 21);
  for (int y = 9; y <= 11; ++y)
    for (int x = 9; x <= 11; ++x) set(g, x, y, 100);  // 1.5 m bush
  CostGrid cg;
  cg.build(g);
  cg.floodFrom(centre(2, 2), 0.0f);
  CandidateSnapper snap(2.0f);
  Eigen::Vector2f out;
  ASSERT_EQ(snap.snap(cg, centre(10, 10), true, true, kAcceptAll, out),
            Outcome::kSnapped);
  // Two cells from the bush centre: the first free ring, straight out.
  const float d = (out - centre(10, 10).head<2>()).norm();
  EXPECT_NEAR(d, 2.0f * kRes, 1e-4f);
  int gx = 0, gy = 0;
  ASSERT_TRUE(cg.cellAt(Eigen::Vector3f(out.x(), out.y(), 0.0f), gx, gy));
  EXPECT_FALSE(cg.blockedAt(gx, gy));
}

TEST(CandidateSnap, ClearanceZoneIsNotAGoodCell) {
  auto g = makeGrid(21, 21);
  set(g, 10, 10, 100);
  CostModel m;
  m.clearance_m = 0.6f;  // orthogonal neighbours of the obstacle
  CostGrid cg;
  cg.build(g, 50, m);
  cg.floodFrom(centre(2, 2), 0.0f);
  ASSERT_TRUE(cg.inClearanceZone(centre(11, 10)));
  CandidateSnapper snap(2.0f);
  Eigen::Vector2f out;
  ASSERT_EQ(snap.snap(cg, centre(11, 10), true, true, kAcceptAll, out),
            Outcome::kSnapped);
  EXPECT_FALSE(cg.inClearanceZone(Eigen::Vector3f(out.x(), out.y(), 0.0f)));
  // Nearest out-of-zone cell is one step away (e.g. diagonal to the obstacle).
  EXPECT_NEAR((out - centre(11, 10).head<2>()).norm(), kRes, 1e-4f);
}

TEST(CandidateSnap, UnknownAllowedOnlyWhenAsked) {
  auto g = makeGrid(21, 21, -1);
  for (int x = 0; x <= 8; ++x)
    for (int y = 0; y < 21; ++y) set(g, x, y, 0);  // known free x <= 8
  CostGrid cg;
  cg.build(g);
  cg.floodFrom(centre(2, 10), 0.0f);
  CandidateSnapper snap(3.0f);
  Eigen::Vector2f out;
  EXPECT_EQ(snap.snap(cg, centre(11, 10), /*allow_unknown=*/true, true,
                      kAcceptAll, out),
            Outcome::kKept);
  ASSERT_EQ(snap.snap(cg, centre(11, 10), /*allow_unknown=*/false, true,
                      kAcceptAll, out),
            Outcome::kSnapped);
  EXPECT_NEAR(out.x(), centre(8, 10).x(), 1e-4f);
  EXPECT_NEAR(out.y(), centre(8, 10).y(), 1e-4f);
}

TEST(CandidateSnap, UnreachablePocketSnapsOutsideWhenRequired) {
  auto g = makeGrid(21, 21);
  // Closed ring around (10, 10): the centre cell is free but sealed off.
  for (int x = 9; x <= 11; ++x) { set(g, x, 9, 100); set(g, x, 11, 100); }
  set(g, 9, 10, 100);
  set(g, 11, 10, 100);
  CostGrid cg;
  cg.build(g);
  cg.floodFrom(centre(2, 2), 0.0f);
  ASSERT_FALSE(cg.reachable(centre(10, 10)));
  CandidateSnapper snap(2.0f);
  Eigen::Vector2f out;
  EXPECT_EQ(snap.snap(cg, centre(10, 10), false, /*require_reachable=*/false,
                      kAcceptAll, out),
            Outcome::kKept);
  ASSERT_EQ(snap.snap(cg, centre(10, 10), false, /*require_reachable=*/true,
                      kAcceptAll, out),
            Outcome::kSnapped);
  EXPECT_TRUE(cg.reachable(Eigen::Vector3f(out.x(), out.y(), 0.0f)));
}

TEST(CandidateSnap, AcceptVetoesTargets) {
  auto g = makeGrid(21, 21);
  for (int y = 0; y < 21; ++y) set(g, 10, y, 100);  // wall at x = 10
  CostGrid cg;
  cg.build(g);
  CandidateSnapper snap(2.0f);
  Eigen::Vector2f out;
  // Only x >= 5.25 (cells 11+) accepted: the snap must go right of the wall.
  const auto right = [](float x, float) { return x > 5.25f; };
  ASSERT_EQ(snap.snap(cg, centre(10, 10), true, false, right, out),
            Outcome::kSnapped);
  EXPECT_NEAR(out.x(), centre(11, 10).x(), 1e-4f);
  EXPECT_NEAR(out.y(), centre(11, 10).y(), 1e-4f);
}

TEST(CandidateSnap, NoSpotLeavesCandidateAlone) {
  auto g = makeGrid(21, 21, 100);  // all blocked
  CostGrid cg;
  cg.build(g);
  CandidateSnapper snap(2.0f);
  Eigen::Vector2f out(-1.0f, -1.0f);
  EXPECT_EQ(snap.snap(cg, centre(10, 10), true, false, kAcceptAll, out),
            Outcome::kNoSpot);
  EXPECT_FLOAT_EQ(out.x(), -1.0f);

  // Off the grid: nothing to search from.
  auto free_grid = makeGrid(21, 21);
  cg.build(free_grid);
  EXPECT_EQ(snap.snap(cg, Eigen::Vector3f(-5.0f, 0.0f, 0.0f), true, false,
                      kAcceptAll, out),
            Outcome::kNoSpot);
}

TEST(CandidateSnap, ZeroRadiusOnlyClassifies) {
  auto g = makeGrid(21, 21);
  set(g, 10, 10, 100);
  CostGrid cg;
  cg.build(g);
  CandidateSnapper snap(0.0f);
  Eigen::Vector2f out;
  EXPECT_EQ(snap.snap(cg, centre(3, 3), true, false, kAcceptAll, out),
            Outcome::kKept);
  EXPECT_EQ(snap.snap(cg, centre(10, 10), true, false, kAcceptAll, out),
            Outcome::kNoSpot);
}

TEST(CandidateSnap, RadiusBoundsTheSearch) {
  auto g = makeGrid(41, 41);
  for (int y = 10; y <= 30; ++y)
    for (int x = 10; x <= 30; ++x) set(g, x, y, 100);  // 10.5 m block
  CostGrid cg;
  cg.build(g);
  Eigen::Vector2f out;
  // Centre (20, 20) is 11 cells (5.5 m) from the nearest free cell.
  EXPECT_EQ(CandidateSnapper(2.0f).snap(cg, centre(20, 20), true, false,
                                        kAcceptAll, out),
            Outcome::kNoSpot);
  EXPECT_EQ(CandidateSnapper(6.0f).snap(cg, centre(20, 20), true, false,
                                        kAcceptAll, out),
            Outcome::kSnapped);
}
