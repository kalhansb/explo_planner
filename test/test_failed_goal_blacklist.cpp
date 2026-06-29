#include <gtest/gtest.h>

#include <Eigen/Core>

#include "explo_planner/failed_goal_blacklist.hpp"

using namespace explo_planner;

namespace {
Eigen::Vector3f p(float x, float y, float z = 0.0f) {
  return Eigen::Vector3f(x, y, z);
}
}  // namespace

TEST(FailedGoalBlacklist, EmptyByDefault) {
  FailedGoalBlacklist bl;
  EXPECT_TRUE(bl.empty());
  EXPECT_EQ(bl.size(), 0u);
  EXPECT_FALSE(bl.isNear(p(0, 0), 5.0));
}

// isNear is a radius query in XY; Z is ignored.
TEST(FailedGoalBlacklist, IsNearRadiusXYOnly) {
  FailedGoalBlacklist bl;
  bl.add(p(10.0f, 10.0f, 3.0f), /*now_sec=*/0.0);
  EXPECT_EQ(bl.size(), 1u);

  // Inside the radius (1.0 m away < 2.0 m), even with a large Z difference.
  EXPECT_TRUE(bl.isNear(p(11.0f, 10.0f, -50.0f), 2.0));
  // Outside the radius (3.0 m away > 2.0 m).
  EXPECT_FALSE(bl.isNear(p(13.0f, 10.0f), 2.0));
  // Exactly on the boundary fails (strict <).
  EXPECT_FALSE(bl.isNear(p(12.0f, 10.0f), 2.0));
}

// prune drops entries strictly older than the TTL relative to now.
TEST(FailedGoalBlacklist, PruneByTtl) {
  FailedGoalBlacklist bl;
  bl.add(p(0, 0), 100.0);
  bl.add(p(5, 5), 105.0);

  // At t=110 with ttl=8: first entry age 10 (>8) expires, second age 5 stays.
  bl.prune(110.0, 8.0);
  EXPECT_EQ(bl.size(), 1u);
  EXPECT_FALSE(bl.isNear(p(0, 0), 1.0));  // expired entry gone
  EXPECT_TRUE(bl.isNear(p(5, 5), 1.0));   // fresh entry remains

  // Advancing far enough expires everything.
  bl.prune(200.0, 8.0);
  EXPECT_TRUE(bl.empty());
}

// Entry exactly at the TTL age is kept (age > ttl is strict).
TEST(FailedGoalBlacklist, PruneBoundaryKeepsExactAge) {
  FailedGoalBlacklist bl;
  bl.add(p(1, 1), 0.0);
  bl.prune(10.0, 10.0);  // age == ttl, not > ttl
  EXPECT_EQ(bl.size(), 1u);
}

// Multiple live entries are all queryable.
TEST(FailedGoalBlacklist, MultipleEntries) {
  FailedGoalBlacklist bl;
  bl.add(p(0, 0), 0.0);
  bl.add(p(20, 0), 0.0);
  bl.add(p(0, 20), 0.0);
  EXPECT_EQ(bl.size(), 3u);
  EXPECT_TRUE(bl.isNear(p(0.5f, 0.0f), 1.0));
  EXPECT_TRUE(bl.isNear(p(20.0f, 0.5f), 1.0));
  EXPECT_TRUE(bl.isNear(p(0.0f, 19.5f), 1.0));
  EXPECT_FALSE(bl.isNear(p(10.0f, 10.0f), 1.0));
}
