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

// --- Mesh reconnection helpers ---

// The wire strings are what the yaml/launch pass; anything else must fall back
// to the legacy behaviour, never crash or invent a mode. Matching is
// case-insensitive (a hand-typed "Hybrid" must not silently run legacy
// rendezvous), and the `known` flag is what the node's startup warning keys
// on, so it must be false exactly when the fallback was NOT asked for.
TEST(PlannerUtil, ReconnectModeFromString) {
  EXPECT_EQ(reconnectModeFromString("rendezvous"), ReconnectMode::RENDEZVOUS);
  EXPECT_EQ(reconnectModeFromString("pursuit"), ReconnectMode::PURSUIT);
  EXPECT_EQ(reconnectModeFromString("hybrid"), ReconnectMode::HYBRID);
  EXPECT_EQ(reconnectModeFromString("Hybrid"), ReconnectMode::HYBRID);
  EXPECT_EQ(reconnectModeFromString("PURSUIT"), ReconnectMode::PURSUIT);
  EXPECT_EQ(reconnectModeFromString("nonsense"), ReconnectMode::RENDEZVOUS);
  EXPECT_EQ(reconnectModeFromString(""), ReconnectMode::RENDEZVOUS);

  bool known = false;
  reconnectModeFromString("rendezvous", &known);
  EXPECT_TRUE(known);
  reconnectModeFromString("HYBRID", &known);
  EXPECT_TRUE(known);
  reconnectModeFromString("nonsense", &known);
  EXPECT_FALSE(known);
  reconnectModeFromString("", &known);
  EXPECT_FALSE(known);
}

// Fresh record, mid-range trail head: navBudget shape at full freshness.
// 20 m / 0.15 m/s * 3.0 = 400 s -> clamped to the 240 s ceiling (the
// flatforest defaults: any trail head past ~12 m rides the ceiling).
TEST(PlannerUtil, PursuitBudgetFreshClampsToCeiling) {
  EXPECT_NEAR(pursuitBudgetSec(20.0, 0.0, 0.15, 3.0, 180.0, 30.0, 240.0),
              240.0, 1e-9);
}

// Freshness decays linearly: at half the staleness window the raw budget is
// halved BEFORE the clamp. 10 m / 0.15 * 3.0 = 200 s raw; * 0.5 = 100 s,
// inside [30, 240].
TEST(PlannerUtil, PursuitBudgetScalesWithStaleness) {
  EXPECT_NEAR(pursuitBudgetSec(10.0, 90.0, 0.15, 3.0, 180.0, 30.0, 240.0),
              100.0, 1e-9);
}

// At (or past) the staleness ceiling the record is worthless: budget 0, which
// the caller reads as "skip pursuit, go straight to the fallback". The floor
// clamp must NOT resurrect a gated budget.
TEST(PlannerUtil, PursuitBudgetStalenessGate) {
  EXPECT_NEAR(pursuitBudgetSec(50.0, 180.0, 0.15, 3.0, 180.0, 30.0, 240.0),
              0.0, 1e-9);
  EXPECT_NEAR(pursuitBudgetSec(50.0, 1e6, 0.15, 3.0, 180.0, 30.0, 240.0),
              0.0, 1e-9);
}

// A close, fresh trail head clamps UP to the floor (a viable minimum chase),
// and max_sec <= 0 disables pursuit outright regardless of everything else.
TEST(PlannerUtil, PursuitBudgetFloorAndDisable) {
  EXPECT_NEAR(pursuitBudgetSec(0.5, 0.0, 0.15, 3.0, 180.0, 30.0, 240.0),
              30.0, 1e-9);
  EXPECT_NEAR(pursuitBudgetSec(50.0, 0.0, 0.15, 3.0, 180.0, 30.0, 0.0),
              0.0, 1e-9);
  // staleness_max <= 0 disables the gate: full freshness however old.
  EXPECT_NEAR(pursuitBudgetSec(10.0, 1e6, 0.15, 3.0, 0.0, 30.0, 240.0),
              200.0, 1e-9);
}

// The ceiling wins over the floor. min_sec is the nav-family floor
// (nav_min_timeout_sec, 30 s in the yaml) and max_sec the pursuit-family
// ceiling — nothing orders them, and a short-chase A/B like
// pursuit_budget_max_sec:=15 is legitimate config. The naive
// clamp(raw, 30, 15) is UB (lo > hi) whose libstdc++ artifact returned the
// FLOOR — a budget above the "hard ceiling" the waiting teammate relies on.
// The ceiling must hold from both directions: raw below the floor and raw
// above the ceiling.
TEST(PlannerUtil, PursuitBudgetCeilingBeatsFloor) {
  // raw = 0.5/0.15*3 = 10 s, floor 30 > ceiling 15 -> 15, never 30.
  EXPECT_NEAR(pursuitBudgetSec(0.5, 0.0, 0.15, 3.0, 180.0, 30.0, 15.0),
              15.0, 1e-9);
  // raw = 400 s -> still the 15 s ceiling.
  EXPECT_NEAR(pursuitBudgetSec(20.0, 0.0, 0.15, 3.0, 180.0, 30.0, 15.0),
              15.0, 1e-9);
}

// Degenerate inputs must stay inside [0, max_sec]: a negative staleness
// (clock skew between the record stamp and now on the same local clock is
// impossible, but a caller bug must not inflate the budget past full
// freshness) and a zero/negative speed estimate (guarded to 1e-3, so the raw
// term explodes and the ceiling absorbs it — same guard navBudgetSec tests).
TEST(PlannerUtil, PursuitBudgetDegenerateInputs) {
  // Negative staleness: freshness clamps to 1.0 — identical to fresh.
  EXPECT_NEAR(pursuitBudgetSec(10.0, -50.0, 0.15, 3.0, 180.0, 30.0, 240.0),
              200.0, 1e-9);
  // Zero speed: raw = 10/1e-3*3 = 30000 -> ceiling.
  EXPECT_NEAR(pursuitBudgetSec(10.0, 0.0, 0.0, 3.0, 180.0, 30.0, 240.0),
              240.0, 1e-9);
}

// The meeting point is the plain midpoint of the last-contact pose pair; both
// sides compute it from their own record, so the arithmetic must be exact.
TEST(PlannerUtil, MeetingPointMidpoint) {
  const Eigen::Vector3f a(10.0f, -4.0f, 0.0f);
  const Eigen::Vector3f b(-2.0f, 8.0f, 1.0f);
  const Eigen::Vector3f m = meetingPoint(a, b);
  EXPECT_NEAR(m.x(), 4.0f, 1e-6f);
  EXPECT_NEAR(m.y(), 2.0f, 1e-6f);
  EXPECT_NEAR(m.z(), 0.5f, 1e-6f);
}
