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

// shouldRendezvous: return-and-wait only when enabled, home is recorded, and
// the team is NOT already whole.
TEST(PlannerUtil, ShouldRendezvous) {
  // Enabled, home recorded, a teammate still out -> go wait.
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

// AbortStreak: aborts from the same spot extend the streak; an abort after the
// robot moved restarts it at 1 from the new spot.
TEST(PlannerUtil, AbortStreak) {
  AbortStreak s;
  EXPECT_EQ(s.count(), 0);
  EXPECT_EQ(s.onAbort(0.0f, 0.0f, 0.25f), 1);
  EXPECT_EQ(s.onAbort(0.1f, 0.1f, 0.25f), 2);   // jitter, not movement
  EXPECT_EQ(s.onAbort(0.0f, 0.2f, 0.25f), 3);
  EXPECT_EQ(s.onAbort(2.0f, 0.0f, 0.25f), 1);   // drove 2 m: new streak there
  EXPECT_EQ(s.onAbort(2.1f, 0.0f, 0.25f), 2);
  s.reset();
  EXPECT_EQ(s.count(), 0);
  EXPECT_EQ(s.onAbort(2.1f, 0.0f, 0.25f), 1);   // a reset never extends
  // A localisation jump moves the anchor with the robot.
  s.shiftAnchor(5.0f, 0.0f);
  EXPECT_EQ(s.onAbort(7.1f, 0.0f, 0.25f), 2);
}

// RecoveryLog counts recoveries around the ABORT spot, not around the first
// recovery, and only those of this goal.
TEST(PlannerUtil, RecoveryLog) {
  RecoveryLog log;
  EXPECT_EQ(log.countNear(0.0f, 0.0f, 1.0f), 0);
  log.add(0.9f, 0.0f);
  log.add(0.9f, 0.1f, 2);                       // two between feedbacks
  log.add(0.0f, 0.0f, 0);                       // nothing to record
  EXPECT_EQ(log.countNear(0.0f, 0.0f, 1.0f), 3);
  EXPECT_EQ(log.countNear(0.9f, 0.0f, 0.05f), 1);  // only the record in r
  // Recoveries 1.8 m from the abort do not count, even though they are
  // within 1 m of a spot between the two.
  EXPECT_EQ(log.countNear(-0.9f, 0.0f, 1.0f), 0);
  // Recovered 20 m back, then aborted here: nothing near.
  EXPECT_EQ(log.countNear(20.9f, 0.0f, 1.0f), 0);
  // A localisation jump moves the records with the robot.
  log.shiftAll(20.0f, 0.0f);
  EXPECT_EQ(log.countNear(20.9f, 0.0f, 1.0f), 3);
  log.reset();
  EXPECT_EQ(log.countNear(20.9f, 0.0f, 1.0f), 0);
  // Bounded: only the most recent records are kept.
  for (int i = 0; i < 200; ++i) log.add(0.0f, 0.0f);
  EXPECT_EQ(log.countNear(0.0f, 0.0f, 1.0f), 64);
}

TEST(PlannerUtil, NewRecoveries) {
  // Same goal id: only increases are new.
  EXPECT_EQ(newRecoveries(false, true, 2, 3), 1);
  EXPECT_EQ(newRecoveries(false, true, 3, 3), 0);
  // A new id (keep-alive preemption) counts from 0, so a first sample of 1
  // is one recovery, even if the 0 sample was missed.
  EXPECT_EQ(newRecoveries(false, false, 5, 1), 1);
  EXPECT_EQ(newRecoveries(false, false, 5, 0), 0);
  // The very first message may be a goal from before the node: baseline.
  EXPECT_EQ(newRecoveries(true, false, 0, 4), 0);
  // Never negative.
  EXPECT_EQ(newRecoveries(false, true, 4, 1), 0);
}

TEST(PlannerUtil, Nav2GiveUp) {
  using G = Nav2GiveUp;
  // Instant refusal, first abort: re-send.
  EXPECT_EQ(nav2GiveUp(2, 1, true, 0, 2), G::kRetry);
  // Second abort from the same spot.
  EXPECT_EQ(nav2GiveUp(2, 2, true, 0, 2), G::kAbortLimit);
  // First abort after nav2's recoveries there.
  EXPECT_EQ(nav2GiveUp(2, 1, true, 2, 2), G::kAfterRecovery);
  EXPECT_EQ(nav2GiveUp(2, 1, true, 1, 2), G::kRetry);   // one is not enough
  EXPECT_EQ(nav2GiveUp(2, 1, false, 9, 2), G::kRetry);  // rule off
  // Limit 0 = log only, whatever nav2 did.
  EXPECT_EQ(nav2GiveUp(0, 5, true, 9, 2), G::kRetry);
  // Both hold: the abort limit is reported.
  EXPECT_EQ(nav2GiveUp(2, 2, true, 9, 2), G::kAbortLimit);
}

TEST(PlannerUtil, Nav2ErrorName) {
  EXPECT_STREQ(nav2ErrorName(204), "GOAL_OUTSIDE_MAP");
  EXPECT_STREQ(nav2ErrorName(105), "FAILED_TO_MAKE_PROGRESS");
  EXPECT_STREQ(nav2ErrorName(208), "NO_VALID_PATH");
  EXPECT_STREQ(nav2ErrorName(0), "");
  EXPECT_STREQ(nav2ErrorName(9999), "");
}
