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

// --- Rendezvous barrier predicates ---

// teamComplete: expected>0 AND active>=expected. Doubles as the barrier release
// test and the "safe to finish" test, so both boundaries matter.
TEST(PlannerUtil, TeamComplete) {
  EXPECT_TRUE(teamComplete(1, 1));    // exactly the last teammate arrived
  EXPECT_TRUE(teamComplete(3, 2));    // more present than required is still complete
  EXPECT_FALSE(teamComplete(0, 1));   // nobody yet
  EXPECT_FALSE(teamComplete(1, 2));   // one still out
  // expected<=0 means "no team to wait for": never complete (the caller uses
  // this to keep single-robot runs out of the rendezvous path).
  EXPECT_FALSE(teamComplete(0, 0));
  EXPECT_FALSE(teamComplete(5, 0));
}

// shouldRendezvous: return-and-wait only when enabled, an anchor exists, and
// the team is NOT already whole.
TEST(PlannerUtil, ShouldRendezvous) {
  // Enabled, anchor known, a teammate still out -> go wait.
  EXPECT_TRUE(shouldRendezvous(true, true, 0, 1));
  EXPECT_TRUE(shouldRendezvous(true, true, 1, 2));
  // Team already complete -> already synced, finish instead of a pointless hop.
  EXPECT_FALSE(shouldRendezvous(true, true, 1, 1));
  EXPECT_FALSE(shouldRendezvous(true, true, 2, 2));
  // Feature off / no anchor yet / no team -> never rendezvous.
  EXPECT_FALSE(shouldRendezvous(false, true, 0, 1));
  EXPECT_FALSE(shouldRendezvous(true, false, 0, 1));
  EXPECT_FALSE(shouldRendezvous(true, true, 0, 0));
}

// rendezvousWaitExpired: 0 (or negative) max-wait means wait forever; a
// positive max-wait fires once the elapsed wait reaches it.
TEST(PlannerUtil, RendezvousWaitExpired) {
  EXPECT_FALSE(rendezvousWaitExpired(0.0, 0.0));       // wait-forever, just arrived
  EXPECT_FALSE(rendezvousWaitExpired(1e6, 0.0));       // wait-forever, long elapsed
  EXPECT_FALSE(rendezvousWaitExpired(1e6, -1.0));      // negative also = forever
  EXPECT_FALSE(rendezvousWaitExpired(9.9, 10.0));      // not yet
  EXPECT_TRUE(rendezvousWaitExpired(10.0, 10.0));      // exactly at the cap
  EXPECT_TRUE(rendezvousWaitExpired(11.0, 10.0));      // past the cap
}
