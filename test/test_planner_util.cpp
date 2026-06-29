#include <gtest/gtest.h>

#include <cmath>

#include "explo_planner/planner_util.hpp"

using namespace explo_planner;

// The string->id mapping is the wire enum shared with RobotIntent; lock it down.
TEST(PlannerUtil, PlannerTypeIdMapping) {
  EXPECT_EQ(plannerTypeId("eig"), 0);
  EXPECT_EQ(plannerTypeId("entropy"), 1);
  EXPECT_EQ(plannerTypeId("frontier"), 2);
  EXPECT_EQ(plannerTypeId("random"), 3);
  EXPECT_EQ(plannerTypeId("ssmi"), 4);
  // Anything unrecognised maps to the sentinel.
  EXPECT_EQ(plannerTypeId("nonsense"), 255);
  EXPECT_EQ(plannerTypeId(""), 255);
}

// Mid-range distance: budget = dist/speed * safety, untouched by the clamp.
TEST(PlannerUtil, NavBudgetMidRange) {
  // 10 m / 0.5 m/s * 2.0 = 40 s, inside [8, 60].
  EXPECT_NEAR(navBudgetSec(10.0, 0.5, 2.0, 8.0, 60.0), 40.0, 1e-9);
}

// Short hops clamp up to the floor; long hops clamp down to the ceiling.
TEST(PlannerUtil, NavBudgetClamps) {
  // 1 m / 0.5 * 2 = 4 s -> clamped up to 8.
  EXPECT_NEAR(navBudgetSec(1.0, 0.5, 2.0, 8.0, 60.0), 8.0, 1e-9);
  // 100 m / 0.5 * 2 = 400 s -> clamped down to 60.
  EXPECT_NEAR(navBudgetSec(100.0, 0.5, 2.0, 8.0, 60.0), 60.0, 1e-9);
}

// A zero/degenerate speed estimate must not divide by zero; the floor guard
// keeps the result finite (and clamped to the ceiling here).
TEST(PlannerUtil, NavBudgetZeroSpeedGuarded) {
  double b = navBudgetSec(10.0, 0.0, 2.0, 8.0, 60.0);
  EXPECT_TRUE(std::isfinite(b));
  EXPECT_NEAR(b, 60.0, 1e-9);
}
