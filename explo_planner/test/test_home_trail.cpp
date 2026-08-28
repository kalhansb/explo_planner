/// @file test_home_trail.cpp
/// @brief Coverage for the homing ladder's trail geometry (doc §32.10).
///
/// Design B — the approach-based homing watchdog — shipped with no unit tests
/// at all, and two of the three bugs found in review were the same bug twice:
/// a search that started at crumb 0, which IS home. Neither is visible in a
/// log. A retrace onto crumb 0 republishes the goal that just failed while
/// reporting "retrace"; an escape onto crumb 0 drives at the trap while
/// reporting "escape". Both produce a plausible-looking event stream and a
/// robot that never gets home, so the tests below assert the index directly.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "explo_planner/home_trail.hpp"

using namespace explo_planner;
using explo_planner::home_trail::nearestCrumbFromOne;
using explo_planner::home_trail::pickBandCrumb;
using explo_planner::home_trail::remainingTrailDistance;

namespace {
Eigen::Vector3f p(float x, float y, float z = 0.0f) {
  return Eigen::Vector3f(x, y, z);
}

// An outbound run straight along +x, crumbs every 2 m — the spacing the pose
// callback enforces. Index 0 is home.
std::vector<Eigen::Vector3f> straight_trail(int n) {
  std::vector<Eigen::Vector3f> t;
  for (int i = 0; i < n; ++i) t.push_back(p(2.0f * i, 0));
  return t;
}

// The campaign's escape band, from the node's kEscapeBandMinM/MaxM.
constexpr float kBandMin = 1.5f;
constexpr float kBandMax = 6.0f;
}  // namespace

// ---------------------------------------------------------------------------
// nearestCrumbFromOne
// ---------------------------------------------------------------------------

// THE regression test. A robot that has just failed twice AT home's doorstep is
// nearest to crumb 0 by a wide margin, and that is exactly when engageRetrace
// runs. Returning 0 here is what silently deleted three rungs of the ladder.
TEST(HomeTrail, NearestCrumbNeverReturnsHomeItself) {
  const auto trail = straight_trail(6);  // home at (0,0), crumbs to (10,0)
  // Standing 0.3 m from home: crumb 0 is 0.3 m away, crumb 1 is 1.7 m.
  EXPECT_EQ(nearestCrumbFromOne(trail, p(0.3f, 0)), 1);
  // Standing exactly ON home, the most extreme version of the same case.
  EXPECT_EQ(nearestCrumbFromOne(trail, p(0, 0)), 1);
}

TEST(HomeTrail, NearestCrumbPicksTheActualNearest) {
  const auto trail = straight_trail(6);
  EXPECT_EQ(nearestCrumbFromOne(trail, p(6.2f, 0.4f)), 3);   // crumb at (6,0)
  EXPECT_EQ(nearestCrumbFromOne(trail, p(9.5f, 0)), 5);      // crumb at (10,0)
  // Off to the side of the trail, not on it: still an XY nearest-neighbour.
  EXPECT_EQ(nearestCrumbFromOne(trail, p(4.1f, 7.0f)), 2);   // crumb at (4,0)
}

// Z is ignored everywhere else in this planner's distance work (the UGV moves
// in a plane and the UAV's altitude would otherwise dominate every metric), so
// it must be ignored here too.
TEST(HomeTrail, NearestCrumbIgnoresZ) {
  const auto trail = straight_trail(6);
  EXPECT_EQ(nearestCrumbFromOne(trail, p(6.0f, 0.0f, 40.0f)), 3);
}

// A two-element trail is the smallest thing engageRetrace accepts, and it must
// yield a real retrace of one segment rather than a nominal one.
TEST(HomeTrail, NearestCrumbOnAMinimalTrailIsOne) {
  const std::vector<Eigen::Vector3f> trail = {p(0, 0), p(2, 0)};
  EXPECT_EQ(nearestCrumbFromOne(trail, p(0.1f, 0)), 1);
}

// ---------------------------------------------------------------------------
// remainingTrailDistance — shared by the watchdog metric and the nav budget
// ---------------------------------------------------------------------------

TEST(HomeTrail, RemainingIsLegPlusTrailDownToHome) {
  const auto trail = straight_trail(6);
  // At (7,0) retracing via crumb 3 at (6,0): 1 m back to the crumb, then
  // 6 m of trail (3 segments x 2 m) down to home.
  EXPECT_FLOAT_EQ(remainingTrailDistance(trail, 3, p(7.0f, 0)), 7.0f);
  // Off-trail leg is a straight line: 3-4-5 triangle to the crumb, then 6 m.
  EXPECT_FLOAT_EQ(remainingTrailDistance(trail, 3, p(9.0f, 4.0f)), 11.0f);
}

// A curved trail is the whole reason the metric exists: scored as a straight
// line, a robot faithfully following a dog-leg reads as making no approach.
TEST(HomeTrail, RemainingFollowsTheTrailNotTheStraightLine) {
  const std::vector<Eigen::Vector3f> trail = {
      p(0, 0), p(0, 4), p(4, 4), p(4, 0)};  // three sides of a square
  // Standing on the last crumb, 4 m from home in a straight line but 12 m of
  // trail away.
  EXPECT_FLOAT_EQ(remainingTrailDistance(trail, 3, p(4, 0)), 12.0f);
  EXPECT_GT(remainingTrailDistance(trail, 3, p(4, 0)),
            (p(4, 0) - p(0, 0)).head<2>().norm());
}

// Monotonicity is the property the approach watchdog actually depends on:
// progress along the trail must reduce the metric, or a robot doing exactly
// what it was told trips the no-approach detector.
TEST(HomeTrail, RemainingDecreasesAsTheRobotWalksTheTrailIn) {
  const auto trail = straight_trail(6);
  float prev = remainingTrailDistance(trail, 5, p(10.0f, 0));
  // Walk in crumb by crumb, decrementing the index as the node's arrival check
  // does.
  for (int idx = 4; idx >= 1; --idx) {
    const float now =
        remainingTrailDistance(trail, idx, trail[static_cast<size_t>(idx)]);
    EXPECT_LT(now, prev) << "metric grew while retracing to crumb " << idx;
    prev = now;
  }
  EXPECT_FLOAT_EQ(prev, 2.0f);  // on crumb 1, one 2 m segment left
}

// Out-of-range indices mean "the trail is spent or unset". 0, not a distance:
// the callers substitute the straight line to home, and would mask a real bug
// if this quietly returned the leg length for index 0.
TEST(HomeTrail, RemainingIsZeroForASpentOrUnsetIndex) {
  const auto trail = straight_trail(6);
  EXPECT_FLOAT_EQ(remainingTrailDistance(trail, 0, p(7.0f, 0)), 0.0f);
  EXPECT_FLOAT_EQ(remainingTrailDistance(trail, -1, p(7.0f, 0)), 0.0f);
  EXPECT_FLOAT_EQ(remainingTrailDistance(trail, 6, p(7.0f, 0)), 0.0f);
  EXPECT_FLOAT_EQ(remainingTrailDistance({}, 1, p(7.0f, 0)), 0.0f);
}

// The budget and the watchdog must agree by construction, not by coincidence.
// They call the same function now; this pins that they are given the same
// arguments, i.e. that the budget's straight-line leg is the leg to the CRUMB.
TEST(HomeTrail, BudgetDistanceEqualsTheWatchdogMetricInRetrace) {
  const auto trail = straight_trail(6);
  const Eigen::Vector3f pos = p(8.5f, 1.5f);
  const int idx = 4;  // crumb at (8,0)
  const float watchdog = remainingTrailDistance(trail, idx, pos);
  // What doReturnHome computes: leg to the published goal + trail below it.
  const float leg = (trail[idx] - pos).head<2>().norm();
  float trail_sum = 0.0f;
  for (int i = idx; i > 0; --i) {
    trail_sum += (trail[i] - trail[i - 1]).head<2>().norm();
  }
  EXPECT_FLOAT_EQ(watchdog, leg + trail_sum);
}

// ---------------------------------------------------------------------------
// pickBandCrumb
// ---------------------------------------------------------------------------

// The second half of the crumb-0 bug. A robot stalled 1.5-6 m from home puts
// home squarely in the escape band, where it is both the nearest candidate and
// the wrong direction — an escape that drives at the trap it is escaping.
TEST(HomeTrail, EscapeNeverTargetsHome) {
  const auto trail = straight_trail(6);
  // The position is chosen so home WINS on distance — otherwise the test
  // passes on the broken version too and proves nothing. Standing at (1.6, 0):
  // home is 1.6 m away and in band; crumb 1 at (2,0) is 0.4 m, below the band;
  // the nearest in-band crumb is 2 at (4,0), a full 0.8 m further than home.
  const int i = pickBandCrumb(trail, p(1.6f, 0), kBandMin, kBandMax, -1);
  EXPECT_NE(i, 0) << "escape aimed at home, the goal that just trapped it";
  EXPECT_EQ(i, 2);
}

TEST(HomeTrail, EscapePicksNearestInBandTiesToTheHigherIndex) {
  const auto trail = straight_trail(8);  // crumbs to (14,0)
  // At (8,0) the robot sits on crumb 4. Crumbs 3 and 5 are both exactly 2 m
  // away: the tie must go to 5, the more recently visited one, which is behind
  // a robot that walked out along +x.
  EXPECT_EQ(pickBandCrumb(trail, p(8.0f, 0), kBandMin, kBandMax, -1), 5);
}

TEST(HomeTrail, EscapeRespectsTheBandAtBothEnds) {
  const auto trail = straight_trail(8);
  const Eigen::Vector3f pos = p(9.0f, 0);  // midway between crumbs 4 and 5
  // Ceiling below the nearest crumb: nothing qualifies, and the caller falls
  // through to the rotate-behind fallback.
  EXPECT_EQ(pickBandCrumb(trail, pos, 0.0f, 0.9f, -1), -1);
  // Floor above the two nearest (1 m each): the next ring out, tie to the
  // higher index.
  EXPECT_EQ(pickBandCrumb(trail, pos, 1.1f, 6.0f, -1), 6);
  // Both bounds inclusive, so a crumb sitting exactly on the edge is usable
  // rather than being discarded into the fallback.
  EXPECT_EQ(pickBandCrumb(trail, pos, 1.0f, 1.0f, -1), 5);
}

// The anti-repeat is what stops a second escape aiming at the same blocked
// crumb the first one failed on.
TEST(HomeTrail, EscapeSkipsTheExcludedCrumb) {
  const auto trail = straight_trail(8);
  // Standing on crumb 4: crumbs 3 and 5 are both 2 m away, tie to 5.
  ASSERT_EQ(pickBandCrumb(trail, p(8.0f, 0), kBandMin, kBandMax, -1), 5);
  // Excluding the winner falls to the other 2 m crumb, not out to a 4 m one.
  EXPECT_EQ(pickBandCrumb(trail, p(8.0f, 0), kBandMin, kBandMax, 5), 3);
  // Excluding the loser leaves the winner alone — the exclusion is one index,
  // not "everything at that distance".
  EXPECT_EQ(pickBandCrumb(trail, p(8.0f, 0), kBandMin, kBandMax, 3), 5);
}

// -1 is the fallback signal, and the fallback (rotate 180 deg and drive 2.5 m)
// is the only escape available to a robot with no trail — the case that arises
// when homing begins within 2 m of home.
TEST(HomeTrail, EscapeReturnsMinusOneWhenNoCrumbQualifies) {
  EXPECT_EQ(pickBandCrumb({}, p(0, 0), kBandMin, kBandMax, -1), -1);
  const std::vector<Eigen::Vector3f> home_only = {p(0, 0)};
  EXPECT_EQ(pickBandCrumb(home_only, p(3.0f, 0), kBandMin, kBandMax, -1), -1);
  // A trail exists but the robot is 100 m from all of it.
  EXPECT_EQ(pickBandCrumb(straight_trail(6), p(100.0f, 0), kBandMin, kBandMax,
                          -1),
            -1);
}
