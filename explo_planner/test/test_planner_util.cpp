#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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

// max_age <= 0 is UNBOUNDED. This is the case that matters most: it is the
// default the parameter ships with, and it is what makes the TTL binary
// reproduce the pre-TTL planner, so one build can run both arms of a campaign.
// Nothing about the age may change the answer here — not a huge age, not the
// "no position held" sentinel, not a negative bound.
TEST(PlannerUtil, AllocPeerPositionUnboundedAdmitsEverything) {
  EXPECT_TRUE(allocPeerPositionFresh(0.0, 0.0));
  EXPECT_TRUE(allocPeerPositionFresh(3600.0, 0.0));
  EXPECT_TRUE(allocPeerPositionFresh(-1.0, 0.0));
  // A negative bound reads as "no limit" too, same as pursuit's max_sec.
  EXPECT_TRUE(allocPeerPositionFresh(3600.0, -5.0));
}

// Under a live TTL the boundary is INCLUSIVE: a pose measured exactly N
// seconds ago still holds its cells at a TTL of N. Just past it, the peer is
// dropped from the allocation and its cells return to the pool.
TEST(PlannerUtil, AllocPeerPositionBoundIsInclusive) {
  EXPECT_TRUE(allocPeerPositionFresh(0.0, 120.0));
  EXPECT_TRUE(allocPeerPositionFresh(119.9, 120.0));
  EXPECT_TRUE(allocPeerPositionFresh(120.0, 120.0));
  EXPECT_FALSE(allocPeerPositionFresh(120.1, 120.0));
  EXPECT_FALSE(allocPeerPositionFresh(3600.0, 120.0));
}

// TeamModel::positionAgeSec() returns a NEGATIVE age for "no position held".
// Under a live TTL that must read as NOT fresh. The caller also tests
// have_position, so this is belt-and-braces — but the failure it guards is
// silent: "we have never located this peer" coming out identical to "we heard
// from it a moment ago" would hand the whole map to a robot we cannot find.
TEST(PlannerUtil, AllocPeerPositionUnknownIsNotFresh) {
  EXPECT_FALSE(allocPeerPositionFresh(-1.0, 120.0));
  EXPECT_FALSE(allocPeerPositionFresh(-0.001, 120.0));
}

// Before the mission clock is live, missionElapsed() returns -1 and every age
// computed against it is 0 (TeamModel clamps), i.e. brand new. That is the
// UNBOUNDED direction — start-up plans behave exactly as they did pre-TTL,
// rather than briefly dropping every peer while the clock latches.
TEST(PlannerUtil, AllocPeerPositionPreClockAgeIsFresh) {
  EXPECT_TRUE(allocPeerPositionFresh(0.0, 120.0));
}

// A non-finite input drops the peer instead of admitting it. The node refuses
// a non-finite parameter at load, so this should be unreachable from the
// harness; it is asserted because the NaN answer falls out of comparison
// semantics rather than from any written branch, and the safe direction (drop)
// and the dangerous one (admit) are one operator apart.
TEST(PlannerUtil, AllocPeerPositionNonFiniteIsNotFresh) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(allocPeerPositionFresh(nan, 120.0));
  EXPECT_FALSE(allocPeerPositionFresh(60.0, nan));
  // But an infinite BOUND is a real "never expires" and must admit.
  EXPECT_TRUE(allocPeerPositionFresh(
      1e9, std::numeric_limits<double>::infinity()));
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

// A watchdog that cannot fire is worse than no watchdog.
//
// navBudgetSec is the NAVIGATE timeout. Both properties asserted here are
// guarantees rather than incidental behaviour, and both used to fail:
//
//  - A non-finite input survived the clamp. std::clamp is written as
//    `v < lo ? lo : hi < v ? hi : v`; both comparisons answer false against
//    NaN, so the NaN came straight back out. Every later `elapsed > budget`
//    test is then false too — the timeout never fires, the robot sits on a
//    dead goal, and the cell runs to max_steps. In the campaign record that is
//    indistinguishable from a genuinely slow cell, so a censored run is scored
//    as a completed one.
//  - Nothing orders min_sec against max_sec (they are separate parameters from
//    unrelated families), and std::clamp with lo > hi is undefined behaviour.
TEST(PlannerUtil, NavBudgetIsAlwaysAFiniteFiringDeadline) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  // Non-finite by any route lands on the ceiling: still bounded, still fires.
  // The ceiling and not the floor, because "we do not know how far it is" must
  // not cut a legitimate long drive short.
  EXPECT_DOUBLE_EQ(navBudgetSec(nan, 0.5, 2.0, 8.0, 60.0), 60.0);
  EXPECT_DOUBLE_EQ(navBudgetSec(10.0, nan, 2.0, 8.0, 60.0), 60.0);
  EXPECT_DOUBLE_EQ(navBudgetSec(10.0, 0.5, nan, 8.0, 60.0), 60.0);
  EXPECT_DOUBLE_EQ(navBudgetSec(inf, 0.5, 2.0, 8.0, 60.0), 60.0);

  // Transposed bounds: the CEILING wins, exactly as in pursuitBudgetSec. The
  // floor winning would hand back a budget LONGER than the ceiling the caller
  // asked for, which is backwards for a timeout.
  EXPECT_DOUBLE_EQ(navBudgetSec(10.0, 0.5, 2.0, 600.0, 60.0), 60.0);
  EXPECT_DOUBLE_EQ(navBudgetSec(1.0, 0.5, 2.0, 600.0, 60.0), 60.0);

  // The ordered case is untouched — this fix must not move any live budget.
  EXPECT_NEAR(navBudgetSec(10.0, 0.5, 2.0, 8.0, 60.0), 40.0, 1e-9);
  EXPECT_NEAR(navBudgetSec(1.0, 0.5, 2.0, 8.0, 60.0), 8.0, 1e-9);
  EXPECT_NEAR(navBudgetSec(100.0, 0.5, 2.0, 8.0, 60.0), 60.0, 1e-9);
}

// ===========================================================================
// WHICH OCCURRENCE OF THE AGREED SCHEDULE A ROBOT KEEPS.
// ===========================================================================
//
// Generation 23 made the rendezvous arm a rendezvous again: the team commits a
// (cell, interval, t_meet) triple while whole, and a robot that later separates
// keeps an occurrence of THAT schedule. WHICH one is the caller's floor, and
// the floor has moved twice: generations 23-24 used the robot's own 100 s
// notice, an unconditional per-robot term that forked the ts4 N=3 cell;
// generation 25 made it bare t_now; generation 29 adds back only the amount by
// which this robot's drive overruns rendezvous_max_lateness_sec, clamped at
// zero so a robot that can make the nearest occurrence still gets the bare
// floor. Generations 20-22 used a private per-robot countdown instead, which is
// N robots visiting one cell at N different times.
//
// This is the arithmetic that distinguishes the two, and it is the reason the
// function was lifted out of explo_planner_node.cpp: that file defines main(),
// every gtest target links gtest_main, so nothing in it can be called from a
// test and node coverage there is source-scan only. The scan can pin that the
// call exists; only this can pin that it is right.
//
// THE PROPERTY THAT MUST HOLD, stated once: robots sharing (t_meet, interval)
// and differing only in `not_before` must land on instants that differ by a
// whole number of intervals, never by an arbitrary offset. A private countdown
// fails that for every pair of robots; this must fail it for none.
//
// MUTATION-VERIFIED, 2026-09-18. Six mutants of nextAgreedOccurrence; five were
// killed by the three tests below — floor instead of ceiling, ceiling one
// interval too far, the `interval_ms <= 0` guard weakened to `< 0` (SIGFPE),
// anchoring on `not_before` (the generation-18 bug), and a private countdown
// (the generation-20 bug).
//
// THE SIXTH SURVIVES AND IS EQUIVALENT, recorded here so nobody re-derives it
// and then "fixes" the gap by adding a test that asserts nothing. Relaxing
// `if (t_meet_ms >= not_before_ms)` to `>` changes behaviour only at
// `t_meet_ms == not_before_ms`, and both paths return `t_meet_ms` there: the
// ceiling division gives k = 0, and the `interval_ms <= 0` guard returns
// `t_meet_ms` outright. There is no input that separates them. A surviving
// mutant is normally a missing test; this one is a proof that the branch is a
// short-circuit rather than a decision.

TEST(NextAgreedOccurrence, TheFirstAgreedMeetingIsKeptAsAgreed) {
  // The floor is behind the agreed instant: the team's FIRST meeting is still
  // ahead, and it is the one every robot committed to. Rolling forward here
  // would skip it and send an early robot to the second meeting alone.
  EXPECT_EQ(nextAgreedOccurrence(130000, 30000, 100.0), 130000);
  // Exactly on the instant is not late.
  EXPECT_EQ(nextAgreedOccurrence(130000, 30000, 130.0), 130000);
  // A meeting already agreed for the past is returned unchanged when it is the
  // only one (no recurrence); with a recurrence it rolls, tested below.
  EXPECT_EQ(nextAgreedOccurrence(130000, 0, 500.0), 130000);
  EXPECT_EQ(nextAgreedOccurrence(130000, -1, 500.0), 130000);
}

TEST(NextAgreedOccurrence, RollsForwardOnWholeIntervalsAndNeverLandsShort) {
  // One tick past the agreed instant takes the NEXT occurrence, not a partial
  // one: 130 s + 30 s.
  EXPECT_EQ(nextAgreedOccurrence(130000, 30000, 130.001), 160000);
  // Landing exactly on an occurrence keeps that occurrence rather than the one
  // after it — the +interval-1 ceiling form is exact here, and an off-by-one
  // would cost a robot a full interval of waiting on every arming.
  EXPECT_EQ(nextAgreedOccurrence(130000, 30000, 160.0), 160000);
  EXPECT_EQ(nextAgreedOccurrence(130000, 30000, 190.0), 190000);
  // Far future: k is not clamped.
  EXPECT_EQ(nextAgreedOccurrence(130000, 30000, 3000.0), 3010000);
  // Every result is on the agreed lattice and at or after the floor, and once
  // the floor has passed the agreed instant it is also the SOONEST such point.
  //
  // The overshoot bound is conditional on purpose. Before t_meet there is no
  // earlier occurrence to choose — k = 0 is the first meeting the team agreed
  // to — so the gap to the floor is whatever the agreement made it, and
  // asserting "within one interval" there would be asserting that the function
  // invents a meeting between now and the first agreed one. That is exactly
  // what the countdown did.
  for (int i = 0; i < 400; ++i) {
    const double floor_sec = 100.0 + 0.37 * i;
    const long long floor_ms =
        static_cast<long long>(std::llround(floor_sec * 1000.0));
    const long long got = nextAgreedOccurrence(130000, 30000, floor_sec);
    EXPECT_EQ((got - 130000) % 30000, 0) << "off the agreed lattice at "
                                         << floor_sec;
    EXPECT_GE(got, floor_ms) << "returned an instant already past at "
                             << floor_sec;
    if (floor_ms >= 130000) {
      EXPECT_LT(got - floor_ms, 30000)
          << "overshot by a whole interval at " << floor_sec;
    } else {
      EXPECT_EQ(got, 130000)
          << "skipped the first agreed meeting at " << floor_sec;
    }
  }
}

TEST(NextAgreedOccurrence, TwoRobotsOnOneAgreementDifferByWholeIntervals) {
  // THE WHOLE POINT, as a test. The ts4 cells measure ~1.2 s of skew between
  // robots' mission clocks, and the generation-18 measurement that killed the
  // first attempt at an agreed time was an arming spread of 16.1 / 52.5 /
  // 67.4 s. Feed that spread in against one committed triple: the three robots
  // may choose different k, but every pair differs by a whole interval, so the
  // early ones are standing at the agreed cell when the late one arrives. Under
  // the countdown the differences were 36.4 s and 14.9 s — not multiples of
  // anything, and never resolvable.
  const long long t_meet = 130000, interval = 30000;
  const double notice[] = {16.1, 52.5, 67.4};
  long long got[3];
  for (int i = 0; i < 3; ++i) {
    got[i] = nextAgreedOccurrence(t_meet, interval, notice[i] + 100.0);
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ((got[i] - got[j]) % interval, 0)
          << "robots arming at " << notice[i] << "s and " << notice[j]
          << "s chose instants that are not a whole number of intervals apart "
             "— that is the generation-18 failure, restored";
    }
  }
  // And concretely: 116.1 -> 130, 152.5 -> 160, 167.4 -> 190.
  EXPECT_EQ(got[0], 130000);
  EXPECT_EQ(got[1], 160000);
  EXPECT_EQ(got[2], 190000);
}

// ===========================================================================
// THE PUNCTUALITY CLAMP (generation 29).
// ===========================================================================
//
// t_meet became an ARRIVAL instant on 2026-09-19: a robot leaves early enough
// to be there, and if it cannot be there within rendezvous_max_lateness_sec it
// signs up to a later rung of the same agreed lattice instead. That second half
// is a PER-ROBOT term on nextAgreedOccurrence's floor, and a per-robot term on
// that floor is what broke the arm in generations 19-24.
//
// WHAT SEPARATES THE TWO IS THE CLAMP AT ZERO, and nothing else. The
// generation-19 floor was `now + 100 s` unconditionally, so robots that could
// all attend the same occurrence still split across two. This floor is
// `now + max(0, lead - budget)`, which is `now` exactly for every robot inside
// its budget. These tests exist because that `max(0, ...)` is one token, the
// node's lead comes from a live grid lookup no gtest target can link against,
// and without a pure function here its deletion would be a source-scan question
// rather than an executable one.
//
// MUTATION-VERIFIED, 2026-09-19. Six mutants of arrivalShortfallSec, four
// killed: dropping the clamp (returns negative, pulls the floor BEHIND now and
// can hand back an occurrence already past); dropping the sanitiser; dropping
// only its negative-budget half; and reading the budget in milliseconds against
// a lead in seconds.
//
// TWO SURVIVE, AND BOTH ARE EQUIVALENT RATHER THAN MISSED. `lead_ms < 0`
// weakened to `<= 0` is the expected one: a robot standing on the cell reads a
// zero lead, and "no estimate" and "no drive" happen to want the same answer.
// The second was not expected and is worth stating plainly — DELETING THE
// `lead_ms < 0` GUARD ENTIRELY CHANGES NOTHING, because the clamp already
// subsumes it: the budget is non-negative by the line above, so a negative lead
// can only produce a negative difference, which max() sends to 0.0. The guard
// is kept as the statement of intent (the -1 is a distinct fact, not a small
// number) but it is NOT what protects the sentinel — the clamp is, again.

TEST(ArrivalShortfall, ARobotInsideItsBudgetAddsExactlyNothing) {
  // THE NON-FORKING CASE, and it has to be exactly 0.0 rather than small: the
  // caller adds this to bare `now`, and any positive value moves the robot off
  // the occurrence its peers are keeping.
  EXPECT_EQ(arrivalShortfallSec(0, 60.0), 0.0);
  EXPECT_EQ(arrivalShortfallSec(30'000, 60.0), 0.0);
  // Exactly at the budget is inside it: 60 s of drive against 60 s of allowed
  // lateness arrives exactly on the tolerance, which is what the tolerance is.
  EXPECT_EQ(arrivalShortfallSec(60'000, 60.0), 0.0);
  // NO ESTIMATE IS NOT UNREACHABLE. -1 comes from a robot off the snapshot grid
  // — including one standing right next to the meeting — and a robot that
  // cannot price its own drive must not be the one to roll the team's rung.
  EXPECT_EQ(arrivalShortfallSec(-1, 60.0), 0.0);
  EXPECT_EQ(arrivalShortfallSec(-999'999, 60.0), 0.0);
}

TEST(ArrivalShortfall, AShortRobotRollsByExactlyWhatItIsShort) {
  // 90 s of drive against a 60 s budget: 30 s short, so the floor rises 30 s
  // and no more. Rounding this up to a whole interval would be the scheduler's
  // job, not this one's — nextAgreedOccurrence does the quantising, and doing
  // it twice is how a robot skips a rung it could have made.
  EXPECT_DOUBLE_EQ(arrivalShortfallSec(90'000, 60.0), 30.0);
  EXPECT_DOUBLE_EQ(arrivalShortfallSec(283'000, 60.0), 223.0);
  // Sub-second resolution survives: the lead is integer ms and the floor is a
  // double, so this is the one place the two meet.
  EXPECT_DOUBLE_EQ(arrivalShortfallSec(60'500, 60.0), 0.5);
  // A zero budget is a legal, if harsh, configuration: be exactly on time.
  EXPECT_DOUBLE_EQ(arrivalShortfallSec(90'000, 0.0), 90.0);
}

TEST(ArrivalShortfall, RefusesAnUnusableBudgetRatherThanPropagatingIt) {
  // A NaN budget reaches llround() inside nextAgreedOccurrence if it survives,
  // and llround of a NaN is undefined — so the failure would not be a wrong
  // meeting, it would be a wrong meeting on some builds only.
  EXPECT_EQ(arrivalShortfallSec(30'000, std::nan("")), 30.0);
  EXPECT_EQ(arrivalShortfallSec(-1, std::nan("")), 0.0);
  // Infinite reads as "never late", which is the opposite of what a budget of
  // infinity should mean here; it is refused for the same reason.
  EXPECT_EQ(arrivalShortfallSec(90'000, INFINITY), 90.0);
  // A negative budget demands an EARLY arrival from every robot — an
  // unconditional per-robot term, which is the generation-19 fork with a
  // different sign.
  EXPECT_EQ(arrivalShortfallSec(30'000, -10.0), 30.0);
  EXPECT_EQ(arrivalShortfallSec(0, -10.0), 0.0);
}

TEST(ArrivalShortfall, ATeamThatCanAllAttendDoesNotFork) {
  // The composed property, which is the only one that matters at the call site:
  // feed one committed pair and a spread of drives, and every robot inside its
  // budget must land on the SAME instant — not merely on the same lattice.
  //
  // Drives of 10-60 s against a 60 s budget and a 30 s lattice: all six robots
  // keep t+130 s. Under the generation-19 rule the same six split across three
  // occurrences.
  const long long t_meet = 130'000, interval = 30'000;
  const double t_now = 100.0;
  for (long long lead_ms = 0; lead_ms <= 60'000; lead_ms += 10'000) {
    EXPECT_EQ(nextAgreedOccurrence(t_meet, interval,
                                   t_now + arrivalShortfallSec(lead_ms, 60.0)),
              t_meet)
        << "a robot with a " << lead_ms
        << " ms lead left the occurrence its peers are keeping";
  }
  // And a robot that genuinely cannot make it rolls — onto the lattice, never
  // off it. 200 s of drive from t+100 s cannot reach t+130 s within 60 s, so it
  // takes t+250 s: 140 s short, floor at t+240, first rung at or after that.
  const long long rolled = nextAgreedOccurrence(
      t_meet, interval, t_now + arrivalShortfallSec(200'000, 60.0));
  EXPECT_EQ(rolled, 250'000);
  EXPECT_EQ((rolled - t_meet) % interval, 0) << "rolled off the agreed lattice";
  // THE INVARIANT THE DELETED "deadline already passed" DIAGNOSTIC RESTS ON:
  // the floor can only ever move the instant later, so the result is never
  // behind bare now.
  EXPECT_GE(rolled, static_cast<long long>(t_now * 1000.0));
}

// ===========================================================================
// THE FLICKER DWELL.
// ===========================================================================
//
// Three sites in the node act on "the team came back": the manoeuvre barrier,
// the appointment supersede, and the rendezvous_spent_ latch release. All three
// read that fact from a claim table with a 5 s liveness TTL, where ONE packet
// arriving is enough to make the team look whole for a reading — so all three
// have to see it HOLD. Generation 23 moved the holding rule out of the node and
// into these two functions, which is the only way it can be tested at all: the
// node defines main(), every gtest target links gtest_main, so nothing there is
// callable from here.
//
// dwellConfirmed owns a window; dwellHeld reads one. The split is not cosmetic.
// The supersede site and the latch release share ONE window, written once per
// heartbeat and read from doPlan, because they ask a single question and two
// windows over one predicate is how "the outage is over" gets two answers. A
// second caller into dwellConfirmed would advance or disarm the pair, which
// would mean that asking the question changed it.
//
// THE TWO FAILURES THESE TESTS EXIST FOR, both of which ship silently:
//
//   * A dwell that fires on the arming tick. It reads as a guard, has a
//     parameter, appears in the logs, and filters nothing.
//   * A FROZEN window — dwellConfirmed called from behind a branch that is
//     untaken for minutes. An un-ticked window does not decay; `armed` stays
//     true with a stale `since_sec`, and the first sample after the gap sees
//     `now - since` far past the confirm time and fires on ONE reading. Same
//     visible symptom as no guard at all, but harder to see in the source.

TEST(FlickerDwell, TheFirstEligibleTickArmsAndDoesNotFire) {
  bool armed = false;
  double since = 0.0;
  // A dwell, not a deadline. Firing here would make the whole guard a no-op
  // wearing a parameter, and every caller would keep its old behaviour while
  // the source claimed otherwise.
  EXPECT_FALSE(dwellConfirmed(true, 100.0, 6.0, &armed, &since));
  EXPECT_TRUE(armed);
  EXPECT_DOUBLE_EQ(since, 100.0);
  // Still inside the window.
  EXPECT_FALSE(dwellConfirmed(true, 105.999, 6.0, &armed, &since));
  // The boundary is inclusive: `>=`, so a confirm window shorter than the tick
  // period fires on the second eligible tick rather than never.
  EXPECT_TRUE(dwellConfirmed(true, 106.0, 6.0, &armed, &since));
  // And it keeps firing while the condition holds — this is a level, not an
  // edge. The barrier is polled every tick and must not release exactly once.
  EXPECT_TRUE(dwellConfirmed(true, 200.0, 6.0, &armed, &since));
  EXPECT_DOUBLE_EQ(since, 100.0) << "the window restarted while eligible";
}

TEST(FlickerDwell, IneligibilityDisarmsWithNoPartialCredit) {
  bool armed = false;
  double since = 0.0;
  EXPECT_FALSE(dwellConfirmed(true, 100.0, 6.0, &armed, &since));
  EXPECT_FALSE(dwellConfirmed(true, 105.0, 6.0, &armed, &since));
  // 5 of the 6 seconds served, then one ineligible sample. A predicate that
  // keeps dropping is precisely the flicker being filtered, so the window has
  // to restart from scratch rather than resume.
  EXPECT_FALSE(dwellConfirmed(false, 105.5, 6.0, &armed, &since));
  EXPECT_FALSE(armed);
  // 106.0 would have fired under the original window. It must not now.
  EXPECT_FALSE(dwellConfirmed(true, 106.0, 6.0, &armed, &since));
  EXPECT_DOUBLE_EQ(since, 106.0);
  EXPECT_FALSE(dwellConfirmed(true, 111.999, 6.0, &armed, &since));
  EXPECT_TRUE(dwellConfirmed(true, 112.0, 6.0, &armed, &since));
}

TEST(FlickerDwell, ANonPositiveConfirmIsNoDwellAtAll) {
  bool armed = false;
  double since = 0.0;
  // The documented off switch: a campaign turns the guard off with a parameter
  // and gets the legacy release-on-first-read. Both <= 0 spellings, because the
  // node's other "unbounded" parameters use the same convention and a `== 0`
  // test here would make a negative yaml value mean "dwell forever".
  EXPECT_TRUE(dwellConfirmed(true, 100.0, 0.0, &armed, &since));
  EXPECT_TRUE(dwellConfirmed(true, 100.0, -1.0, &armed, &since));
  // Ineligible still wins over it: "no dwell" means no delay, not no predicate.
  EXPECT_FALSE(dwellConfirmed(false, 100.0, 0.0, &armed, &since));
  EXPECT_TRUE(dwellHeld(true, 100.0, 0.0, false, 0.0));
  EXPECT_FALSE(dwellHeld(false, 100.0, 0.0, true, 0.0));
}

TEST(FlickerDwell, NullStateWordsRefuseRatherThanCrash) {
  bool armed = false;
  double since = 0.0;
  EXPECT_FALSE(dwellConfirmed(true, 100.0, 6.0, nullptr, &since));
  EXPECT_FALSE(dwellConfirmed(true, 100.0, 6.0, &armed, nullptr));
  EXPECT_FALSE(armed) << "a refused call must not half-arm the caller's window";
}

TEST(FlickerDwell, HeldAnswersExactlyWhatConfirmedWouldWithoutTouchingIt) {
  // The contract between the twins, checked rather than asserted in a comment:
  // for every state the writer can be in, the reader agrees — and the reader
  // leaves the window where it found it. The two answering differently is the
  // defect the split exists to remove.
  const double confirm = 6.0;
  bool w_armed = false;
  double w_since = 0.0;
  // Drive the writer through a flickering sequence and shadow it with the
  // reader at every step.
  const bool elig[] = {true, true, false, true, true, true, true, false, true};
  const double t[]  = {100.0, 103.0, 104.0, 105.0, 108.0, 111.0, 120.0, 121.0,
                       122.0};
  for (int i = 0; i < 9; ++i) {
    const bool  before_armed = w_armed;
    const double before_since = w_since;
    // Ask first. The reader must be a pure function of (eligible, now, confirm,
    // armed, since) — nothing it does may change the writer's answer.
    const bool read = dwellHeld(elig[i], t[i], confirm, before_armed,
                                before_since);
    EXPECT_EQ(w_armed, before_armed) << "dwellHeld mutated `armed` at step " << i;
    EXPECT_DOUBLE_EQ(w_since, before_since)
        << "dwellHeld mutated `since` at step " << i;
    const bool wrote = dwellConfirmed(elig[i], t[i], confirm, &w_armed,
                                      &w_since);
    EXPECT_EQ(read, wrote)
        << "the twins disagreed at step " << i << " (t=" << t[i]
        << ", eligible=" << elig[i] << ")";
  }
}

TEST(FlickerDwell, AnUnTickedWindowFreezesRatherThanDecaying) {
  // THE CALLER-SIDE HAZARD, pinned here because no signature can prevent it and
  // the node had it twice in draft: dwellConfirmed behind a branch that is
  // untaken for minutes. This is what the header means by "tick it from the
  // timer, read it from the branch".
  bool armed = false;
  double since = 0.0;
  EXPECT_FALSE(dwellConfirmed(true, 100.0, 6.0, &armed, &since));
  // ... 300 s in which the caller's branch was untaken, during which the real
  // predicate was false the whole time. The window does not know that. It is
  // still armed, and `now - since` is now 300 s.
  EXPECT_TRUE(dwellConfirmed(true, 400.0, 6.0, &armed, &since))
      << "a frozen window is supposed to fire on the first sample after the "
         "gap — if this ever reports false, dwellConfirmed grew a staleness "
         "rule and the node comments about placing the writer are obsolete";
  // Stepped continuously over the same span it never fires early, because the
  // false samples disarm it. Same predicate, same clock, different sampling —
  // which is the entire argument for where the writer goes.
  armed = false;
  since = 0.0;
  bool fired = false;
  for (int i = 0; i <= 3000; ++i) {
    const double now_sec = 100.0 + 0.1 * i;
    const bool eligible = now_sec >= 399.9;  // false for all but the last tick
    if (dwellConfirmed(eligible, now_sec, 6.0, &armed, &since)) fired = true;
  }
  EXPECT_FALSE(fired) << "fired without a held run when sampled continuously";
}
