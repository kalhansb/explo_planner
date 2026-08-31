#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/global_allocator.hpp"
#include "explo_planner/rendezvous_scheduler.hpp"

using namespace explo_planner;

namespace {

CellWorld::Config cfg0() {
  CellWorld::Config c;
  c.covered_max_unknown   = 0.15;
  c.exploring_min_unknown = 0.35;
  c.covered_max_frontier_frac = 0.90;
  c.min_observed_columns  = 4;
  return c;
}

/// A 5x5 world of 10 m cells over [-25, 25]^2. Straight out of configure(),
/// the graph is FULLY connected, so distance() is centroid distance and every
/// number below is hand-checkable: cell id = row*5 + col, centre =
/// (-20 + 10*col, -20 + 10*row). Row 0 is therefore cells 0..4 at x =
/// -20, -10, 0, 10, 20 — ten metres apart, which is why the fixtures live
/// there.
CellWorld world5(int self = 0) {
  CellWorld w;
  const std::string err =
      w.configure(makeCellGrid(-25.0f, 25.0f, -25.0f, 25.0f, 10.0f),
                  cfg0(), self);
  EXPECT_EQ(err, "") << "fixture failed to configure";
  return w;
}

RendezvousScheduler::Config sched0() {
  RendezvousScheduler::Config c;
  c.speed_mm_s = 1000;          // 1 m/s: one metre is one second, exactly
  c.depart_safety_milli = 1000; // 1.0x, so departure arithmetic is readable
  c.depart_margin_ms = 0;
  c.max_interval_ms = 0;        // uncapped
  return c;
}

std::vector<AllocRobot> pair2(int cell_a, int cell_b) {
  return {AllocRobot{0, cell_a, true, false},
          AllocRobot{1, cell_b, true, false}};
}

/// Hand-build an allocation. Most of these tests are about the OBJECTIVE, not
/// about which tours the allocator happens to produce, and a hand-built pair of
/// tours makes every penalty in the comments arithmetic a reader can check.
/// The one test that must not do this — cross-perspective identity — solves for
/// real, from both sides, through the wire codec.
Allocation tours2(const CellWorld& w, const std::vector<AllocRobot>& robots,
                  std::vector<int> t0, std::vector<int> t1) {
  Allocation a;
  a.tours = {std::move(t0), std::move(t1)};
  a.costs_mm = {GlobalAllocator::routeCostMm(w, robots[0].cell, a.tours[0]),
                GlobalAllocator::routeCostMm(w, robots[1].cell, a.tours[1])};
  return a;
}

}  // namespace

// ---------------------------------------------------------------------------
// (b) t_meet is the slowest robot's own arrival — hand-computed
// ---------------------------------------------------------------------------

/// P5 gate (b). Row 0, ten metres between neighbours:
///
///   robot 0 at cell 0, tour [1, 2]   base cost 10 + 10 = 20 m
///   robot 1 at cell 4, tour [3]      base cost 10 m
///   base makespan = 20 m
///
///   c=1: r0 already has it (20 m, reaches at 10 m); r1 best insert is [3,1]
///        = 10 + 20 = 30 m, reaching 1 at 30 m.  makespan 30, penalty 10 m.
///   c=2: r0 already has it (20 m, reaches at 20 m); r1 best insert is [3,2]
///        = 10 + 10 = 20 m, reaching 2 at 20 m.   makespan 20, penalty 0.
///   c=3: r1 already has it (10 m, reaches at 10 m); r0 best insert is
///        [1,2,3] = 30 m, reaching 3 at 30 m.     makespan 30, penalty 10 m.
///
/// So cell 2 wins with penalty ZERO — the meeting costs the team nothing,
/// which is the whole claim of §3.5 — and t_meet is the slower arrival, 20 m
/// at 1 m/s = 20 s. Not a formula with constants: the tours determined it.
TEST(RendezvousObjective, PicksTheZeroPenaltyNearMissAndTimesItByTheSlowerArrival) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 5'000, sched0());

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 2);
  EXPECT_EQ(p.penalty_mm, 0);
  EXPECT_FALSE(p.floor_won);
  EXPECT_FALSE(p.capped);
  EXPECT_EQ(p.candidates, 3);
  EXPECT_EQ(p.interval_ms, 20'000);
  EXPECT_EQ(p.t_meet_ms, 25'000) << "t_meet is mission-elapsed, not an offset";
}

/// The penalty is a DIFFERENCE of makespans over the same cost function, so it
/// can never be negative: inserting a cell into one robot's tour can only
/// lengthen that tour, and the other tours are untouched. A negative penalty
/// would mean the two sides of the subtraction came from different cost
/// models, which is the drift routeCostMm was made public to prevent.
TEST(RendezvousObjective, PenaltyIsNeverNegativeForAnyCandidate) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 24);
  const Allocation a = tours2(w, robots, {1, 7, 13}, {23, 17, 11});
  for (int floor : {-1, 12}) {
    const RendezvousPlan p =
        RendezvousScheduler::solve(w, robots, a, floor, 0, sched0());
    ASSERT_TRUE(p.valid()) << p.refused;
    EXPECT_GE(p.penalty_mm, 0) << "floor=" << floor;
  }
}

/// The base makespan this file computes and the one the allocator reports must
/// be the same number. They are produced by different code paths — recomputed
/// from the tours here, accumulated during insertion there — and if they ever
/// disagree the penalty is a difference between a polished cost and an
/// unpolished one, which is invisible in every log.
TEST(RendezvousObjective, RecomputedRouteCostMatchesTheAllocatorsOwn) {
  CellWorld w = world5(0);
  for (int id : {6, 7, 8, 16, 17, 18}) w.commitSelf(id, CellStatus::EXPLORING);
  const auto robots = pair2(0, 24);
  const Allocation a =
      GlobalAllocator::solve(w, robots, GlobalAllocator::Config{});
  ASSERT_EQ(a.tours.size(), robots.size());
  for (size_t i = 0; i < robots.size(); ++i) {
    EXPECT_EQ(a.costs_mm[i],
              GlobalAllocator::routeCostMm(w, robots[i].cell, a.tours[i]))
        << "robot " << i;
  }
}

// ---------------------------------------------------------------------------
// (a) the midpoint floor, and when it wins
// ---------------------------------------------------------------------------

/// P5 gate (a). Both tours point AWAY from the other robot, so every tour cell
/// costs a 40 m detour:
///
///   robot 0 at cell 0 (-20,-20), tour [20] (-20, 20)  -> 40 m
///   robot 1 at cell 4 ( 20,-20), tour [24] ( 20, 20)  -> 40 m, makespan 40 m
///
///   c=20: r1 must add it. Best is [24,20] = 40 + 40 = 80 m. penalty 40 m.
///   c=24: symmetric.                                    penalty 40 m.
///   floor=2 (0,-20), the midpoint of the two robots: each best-inserts it
///        FIRST, 20 + 44.72 = 64.72 m.                   penalty 24.72 m.
///
/// The floor wins, and that is the guarantee that matters: when no tour cell is
/// worth its detour, hybrid degrades to exactly the destination the shipped
/// implementation already drives to, so this arm cannot come out worse than the
/// current one on fallback cost.
TEST(RendezvousFloor, WinsWhenNoTourCellIsWorthItsDetour) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {20}, {24});

  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, /*floor_cell=*/2, 0, sched0());

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 2);
  EXPECT_TRUE(p.floor_won);
  EXPECT_EQ(p.candidates, 3) << "the floor is a candidate alongside the tours";
  // Both robots reach the floor after 20 m, so the meeting is 20 s away.
  EXPECT_EQ(p.interval_ms, 20'000);
  // 40 m makespan -> 64.72 m makespan.
  EXPECT_NEAR(static_cast<double>(p.penalty_mm), 24721.0, 2.0);
}

/// The same fixture with no floor available (the midpoint fell outside the
/// ROI). Cells 20 and 24 tie at exactly 40 m of penalty, and the tie breaks to
/// the LOWER CELL ID — a value both robots agree on. "Whichever the scan
/// reached first" is not a tie-break; it is a dependency on iteration order,
/// and it is the failure this whole family of tests exists to catch.
TEST(RendezvousFloor, WithNoFloorAnExactTieBreaksToTheLowerCellId) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {20}, {24});

  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 0, sched0());

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 20);
  EXPECT_FALSE(p.floor_won);
  EXPECT_EQ(p.candidates, 2);
}

/// A floor already on somebody's tour is not counted twice. It would not change
/// the argmin, but `candidates` is what the P5 non-vacuity rule reads to tell
/// "only the floor survived" from "the floor won on merit", and a
/// double-counted candidate makes that number a lie.
TEST(RendezvousFloor, AFloorAlreadyOnATourIsNotDoubleCounted) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, /*floor_cell=*/2, 0, sched0());

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 2);
  EXPECT_TRUE(p.floor_won);
  EXPECT_EQ(p.candidates, 3) << "{1,2,3}, with 2 appearing once";
}

// ---------------------------------------------------------------------------
// (c) THE GATE TEST: both robots derive the same appointment, through the wire
// ---------------------------------------------------------------------------

/// P5 gate (c), and the entire agreement argument. There is no proposal, no
/// echo and no adoption anywhere in this design; agreement is supposed to be a
/// CONSEQUENCE of both robots running identical arithmetic over a shared world.
/// That claim is only worth what this test proves, and it must be proved HERE
/// rather than inherited from the allocator's own determinism test — the
/// scheduler adds an argmin, an insertion search and a time conversion, any of
/// which could reintroduce a cross-process difference the allocator does not
/// have.
///
/// So: two worlds built from opposite viewpoints, reconciled ONLY by exchanging
/// wire messages, and the vehicle set handed in reversed on one side to prove
/// the caller's vector order cannot reach the result. Anything that survives
/// the round trip as a local-only difference — a *_BY_OTHERS provenance byte, a
/// known_by mask, a local update_id — gets its chance to move the meeting.
TEST(RendezvousCrossPerspective, IdenticalAppointmentFromBothSides) {
  CellWorld a = world5(0);
  CellWorld b = world5(1);

  // Disjoint discoveries: before the exchange the two hold different worlds.
  for (int id : {6, 7, 8}) a.commitSelf(id, CellStatus::EXPLORING);
  for (int id : {16, 17, 18}) b.commitSelf(id, CellStatus::EXPLORING);

  const auto a_wire = a.toWire();
  const auto b_wire = b.toWire();
  b.mergeWire(0, a_wire);
  a.mergeWire(1, b_wire);
  ASSERT_EQ(a.sharedHash(), b.sharedHash()) << "fixture did not converge";

  // Both robots know both positions, so both build the same vehicle set — but
  // A lists it in fleet order and B in reverse, which is the order-independence
  // half of the claim.
  const std::vector<AllocRobot> ra = pair2(0, 24);
  const std::vector<AllocRobot> rb = {ra[1], ra[0]};

  const GlobalAllocator::Config gc;
  const Allocation aa = GlobalAllocator::solve(a, ra, gc);
  const Allocation ab = GlobalAllocator::solve(b, rb, gc);

  // The premise, checked before the conclusion. Two empty tours would leave
  // only the floor on both sides, and this test would then agree with itself
  // for free while proving nothing about the argmin, the insertion search or
  // the time conversion — the three things the scheduler adds on top of the
  // allocator's own determinism.
  ASSERT_FALSE(aa.tours[0].empty());
  ASSERT_FALSE(aa.tours[1].empty());
  EXPECT_EQ(aa.tours[0], ab.tours[1]) << "reversed input, same tour per robot";
  EXPECT_EQ(aa.tours[1], ab.tours[0]);

  const RendezvousPlan pa =
      RendezvousScheduler::solve(a, ra, aa, /*floor_cell=*/12, 7'000, sched0());
  const RendezvousPlan pb =
      RendezvousScheduler::solve(b, rb, ab, /*floor_cell=*/12, 7'000, sched0());

  ASSERT_TRUE(pa.valid()) << pa.refused;
  ASSERT_TRUE(pb.valid()) << pb.refused;
  EXPECT_GT(pa.candidates, 1) << "the argmin must have had a choice to make";
  EXPECT_EQ(pa.cell, pb.cell);
  EXPECT_EQ(pa.t_meet_ms, pb.t_meet_ms);
  EXPECT_EQ(pa.penalty_mm, pb.penalty_mm) << "bit-identical, not merely close";
  EXPECT_EQ(pa.interval_ms, pb.interval_ms);
  EXPECT_EQ(pa.floor_won, pb.floor_won);
}

// ---------------------------------------------------------------------------
// (d) the departure rule: staggered departures, coincident arrivals
// ---------------------------------------------------------------------------

/// P5 gate (d). The property, stated as an equation rather than a vibe: with
/// safety 1.0 and no margin, each robot departs at exactly t_meet - travel and
/// therefore ARRIVES AT t_meet, whatever its distance. Nothing about the
/// trigger is synchronised; the shared quantity is t_meet alone.
///
/// This is the test mTARE does not have, on the comparison it commented out
/// (rendezvous_manager.cpp:141-142). Its robots depart AT zero, so each is late
/// by its own travel time — a lateness that differs per robot and therefore
/// cannot be absorbed by any single interval constant.
TEST(RendezvousDeparture, StaggersDeparturesSoArrivalsCoincide) {
  const RendezvousScheduler::Config c = sched0();
  const long long t_meet = 100'000;
  const long long near_ms = 10'000, far_ms = 30'000;

  // Far robot leaves at 70 s, near robot at 90 s; both arrive at 100 s.
  EXPECT_FALSE(RendezvousScheduler::shouldDepart(t_meet, 69'999, far_ms, c));
  EXPECT_TRUE (RendezvousScheduler::shouldDepart(t_meet, 70'000, far_ms, c));
  EXPECT_FALSE(RendezvousScheduler::shouldDepart(t_meet, 89'999, near_ms, c));
  EXPECT_TRUE (RendezvousScheduler::shouldDepart(t_meet, 90'000, near_ms, c));

  // And the far robot is still exploring while the near one is not yet due —
  // the interval in which exactly one of them is driving to the meeting.
  EXPECT_TRUE (RendezvousScheduler::shouldDepart(t_meet, 80'000, far_ms, c));
  EXPECT_FALSE(RendezvousScheduler::shouldDepart(t_meet, 80'000, near_ms, c));
}

/// The safety factor buys slack by leaving EARLIER, never by moving the
/// meeting. 1.2x a 30 s drive is 36 s of countdown.
TEST(RendezvousDeparture, SafetyFactorAndMarginOnlyMoveTheDeparture) {
  RendezvousScheduler::Config c = sched0();
  c.depart_safety_milli = 1200;
  EXPECT_FALSE(RendezvousScheduler::shouldDepart(100'000, 63'999, 30'000, c));
  EXPECT_TRUE (RendezvousScheduler::shouldDepart(100'000, 64'000, 30'000, c));

  c.depart_safety_milli = 1000;
  c.depart_margin_ms = 5'000;
  EXPECT_FALSE(RendezvousScheduler::shouldDepart(100'000, 64'999, 30'000, c));
  EXPECT_TRUE (RendezvousScheduler::shouldDepart(100'000, 65'000, 30'000, c));
}

/// An overdue appointment departs NOW rather than waiting for the next one. A
/// robot that was busy through its own deadline — mid-hop, held at a proximity
/// stop — must still go; the peer is on its way regardless.
TEST(RendezvousDeparture, AnOverdueAppointmentDepartsImmediately) {
  const RendezvousScheduler::Config c = sched0();
  EXPECT_TRUE(RendezvousScheduler::shouldDepart(100'000, 150'000, 30'000, c));
}

/// No appointment and no usable travel estimate are both "do not depart", not
/// "depart now". A -1 that read as overdue would send a robot to cell -1.
TEST(RendezvousDeparture, RefusesWithoutAnAppointmentOrATravelEstimate) {
  const RendezvousScheduler::Config c = sched0();
  EXPECT_FALSE(RendezvousScheduler::shouldDepart(-1, 50'000, 10'000, c));
  EXPECT_FALSE(RendezvousScheduler::shouldDepart(100'000, 50'000, -1, c));
}

TEST(RendezvousTravel, IsIntegerAndRefusesANonPositiveSpeed) {
  EXPECT_EQ(RendezvousScheduler::travelMs(20'000, 1'000), 20'000);
  EXPECT_EQ(RendezvousScheduler::travelMs(0, 1'000), 0);
  EXPECT_EQ(RendezvousScheduler::travelMs(-5, 1'000), 0);
  EXPECT_EQ(RendezvousScheduler::travelMs(20'000, 0), -1);
  // Truncating division, deliberately: it costs at most a millisecond and it
  // is the same millisecond on both robots, where a double would not be.
  EXPECT_EQ(RendezvousScheduler::travelMs(1'999, 1'000), 1'999);
  EXPECT_EQ(RendezvousScheduler::travelMs(1, 3'000), 0);
}

// ---------------------------------------------------------------------------
// (e) admissibility
// ---------------------------------------------------------------------------

/// P5 gate (e). A cell no robot can reach on the roadmap is not a place to
/// promise to stand at a particular minute, so it leaves the candidate set —
/// and is COUNTED on the way out, because a silently shrinking candidate set
/// looks exactly like a world with fewer cells in it.
///
/// Note this is the one place that must NOT go through costMm, which
/// deliberately falls back to centroid distance on an unreachable pair. That
/// fallback is right for ranking a cell somebody will eventually clear and
/// wrong for an appointment. mTARE has no check here at all: an unreachable
/// rendezvous reaches a lookup that calls exit(1) (§3.5.1, edge 4).
TEST(RendezvousAdmissibility, AnUnreachableCellIsRejectedAndCounted) {
  CellWorld w = world5(0);
  // 8-connected, except that every edge touching cell 12 (0,0) is blocked.
  w.rebuildEdges([](float ax, float ay, float bx, float by) {
    const bool a_mid = std::abs(ax) < 1e-3f && std::abs(ay) < 1e-3f;
    const bool b_mid = std::abs(bx) < 1e-3f && std::abs(by) < 1e-3f;
    return (a_mid || b_mid) ? 1.0 : 0.0;
  });
  const auto robots = pair2(0, 4);
  ASSERT_LT(w.distance(0, 12), 0.0) << "fixture did not isolate cell 12";
  ASSERT_GE(w.distance(0, 2), 0.0) << "fixture disconnected too much";

  const Allocation a = tours2(w, robots, {12}, {2});
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 0, sched0());

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 2);
  EXPECT_EQ(p.rejected_unreachable, 1);
  EXPECT_EQ(p.candidates, 1);
}

/// The floor is admitted even when the graph calls it unreachable. It is the
/// midpoint of two poses that were in radio contact, the shipped
/// implementation already drives to it, and an over-aggressive edge probe must
/// not be able to take away the one destination that is always available.
TEST(RendezvousAdmissibility, TheFloorSurvivesAnUnreachableVerdict) {
  CellWorld w = world5(0);
  w.rebuildEdges([](float, float, float, float) { return 1.0; });
  ASSERT_LT(w.distance(0, 2), 0.0) << "fixture did not disconnect";

  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {20}, {24});
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, /*floor_cell=*/2, 0, sched0());

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 2);
  EXPECT_TRUE(p.floor_won);
  EXPECT_EQ(p.candidates, 1) << "both tour cells were rejected";
  EXPECT_EQ(p.rejected_unreachable, 2);
}

// ---------------------------------------------------------------------------
// (f) no-show: the anti-deadlock rule
// ---------------------------------------------------------------------------

/// P5 gate (f), the direct regression for §3.5.1 edge 5 — mTARE waits at its
/// rendezvous forever and has no handler for a peer that never arrives. Here a
/// cell that failed to produce a meeting stops being proposed and the argmin
/// moves on to the next one.
TEST(RendezvousNoShow, AnExcludedCellIsDroppedAndTheArgminMovesOn) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  c.exclude = {2};
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 0, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_NE(p.cell, 2);
  EXPECT_EQ(p.cell, 1) << "1 and 3 tie at 10 m; the lower id wins";
  EXPECT_EQ(p.rejected_excluded, 1);
  EXPECT_EQ(p.candidates, 2);
}

/// The rule that makes the no-show handler safe: exclusion can never empty the
/// candidate set, because it never applies to the floor. Excluding EVERY cell
/// in the world — including the floor itself — still yields a plan. There is no
/// sequence of no-shows that produces "nowhere to meet", so there is nothing
/// for a no-show to deadlock on.
TEST(RendezvousNoShow, ExclusionCanNeverEmptyTheCandidateSet) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  for (int id = 0; id < w.size(); ++id) c.exclude.push_back(id);

  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, /*floor_cell=*/12, 0, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 12);
  EXPECT_TRUE(p.floor_won);
  EXPECT_EQ(p.candidates, 1);
  EXPECT_EQ(p.rejected_excluded, 3);
}

/// With no floor, exhausting the tours IS a refusal — and a refusal is a
/// well-formed answer the caller falls back from, not a plan pointing at cell
/// -1. The distinction matters because `refused` is what tells a reader "there
/// was nowhere to meet" apart from "nobody looked".
TEST(RendezvousNoShow, ExhaustingTheToursWithNoFloorRefusesRatherThanReturningMinusOne) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  c.exclude = {1, 2, 3};
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 0, c);

  EXPECT_FALSE(p.valid());
  EXPECT_EQ(p.cell, -1);
  EXPECT_EQ(p.t_meet_ms, -1);
  EXPECT_FALSE(p.refused.empty());
}

// ---------------------------------------------------------------------------
// The divergence cap
// ---------------------------------------------------------------------------

/// The cap pulls the meeting earlier when the maps are separating faster than
/// the tours imply. It is symmetric in the two robots — it comes from the
/// node's rate_sum model, which sums both robots' map-growth rates — so both
/// ends cap to the same value, which is the only reason capping is admissible
/// at all under agreement-by-construction.
///
/// The cost is that the slower robot is structurally late by the difference,
/// and `capped` says so rather than hiding it: a run where this is always true
/// is one where the cap, not the objective, is choosing the meeting time.
TEST(RendezvousCap, PullsTheMeetingEarlierAndAdvertisesThatItDid) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  c.max_interval_ms = 8'000;
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 1'000, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_TRUE(p.capped);
  EXPECT_EQ(p.interval_ms, 20'000) << "the uncapped interval stays readable";
  EXPECT_EQ(p.t_meet_ms, 9'000);
  // How late the slowest robot will be, recoverable from the plan alone.
  EXPECT_EQ(p.interval_ms - (p.t_meet_ms - 1'000), 12'000);
}

TEST(RendezvousCap, DoesNothingWhenTheToursAlreadyMeetSooner) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  c.max_interval_ms = 60'000;
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 0, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_FALSE(p.capped);
  EXPECT_EQ(p.t_meet_ms, 20'000);
}

// ---------------------------------------------------------------------------
// Refusals — every one of them a stated answer, none of them a silent -1
// ---------------------------------------------------------------------------

TEST(RendezvousRefusal, UnconfiguredWorld) {
  CellWorld w;
  const auto robots = pair2(0, 4);
  Allocation a;
  a.tours = {{}, {}};
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 0, sched0());
  EXPECT_FALSE(p.valid());
  EXPECT_NE(p.refused.find("not configured"), std::string::npos) << p.refused;
}

/// A non-positive speed makes every arrival time infinite. There is no safe
/// value to substitute, so the scheduler refuses rather than inventing one —
/// the node's parameter load clamps it too, which is two guards, because the
/// one that matters is whichever the next caller forgets.
TEST(RendezvousRefusal, NonPositiveSpeed) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});
  RendezvousScheduler::Config c = sched0();
  c.speed_mm_s = 0;
  const RendezvousPlan p = RendezvousScheduler::solve(w, robots, a, 2, 0, c);
  EXPECT_FALSE(p.valid());
  EXPECT_NE(p.refused.find("speed_mm_s"), std::string::npos) << p.refused;
}

/// Index alignment between `robots` and `alloc.tours` is the contract that lets
/// a tour be attributed to a starting cell without searching by id. Handing in
/// an allocation solved over a different vehicle set would produce a
/// well-formed plan that neither robot can keep, so it is refused outright.
TEST(RendezvousRefusal, AllocationNotAlignedWithTheVehicleSet) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  Allocation a;
  a.tours = {{1, 2}};                 // one tour, two robots
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, 2, 0, sched0());
  EXPECT_FALSE(p.valid());
  EXPECT_NE(p.refused.find("index-aligned"), std::string::npos) << p.refused;
}

/// A finished or unlocatable robot leaves the problem, exactly as it does in
/// the allocator. If that empties the vehicle set there is nobody to meet, and
/// that is a refusal rather than a meeting with oneself.
TEST(RendezvousRefusal, NoLocatableUnfinishedRobot) {
  CellWorld w = world5(0);
  std::vector<AllocRobot> robots = {AllocRobot{0, -1, true, false},
                                    AllocRobot{1, 4, true, true}};
  Allocation a;
  a.tours = {{}, {3}};
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, 2, 0, sched0());
  EXPECT_FALSE(p.valid());
  EXPECT_NE(p.refused.find("no locatable"), std::string::npos) << p.refused;
}

/// Empty tours and no floor is "nowhere to meet", and it is a NORMAL outcome
/// early in a run before the allocator has produced anything. It must read as a
/// refusal with a reason, because the caller's correct response — keep the
/// pre-P5 behaviour — is not the same as its response to a real plan.
TEST(RendezvousRefusal, EmptyToursAndNoFloor) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {}, {});
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 0, sched0());
  EXPECT_FALSE(p.valid());
  EXPECT_NE(p.refused.find("no admissible"), std::string::npos) << p.refused;
}

/// ...and the same fixture WITH a floor produces a plan. A robot with no tour
/// still has to travel to the meeting, so its arrival is a real number and the
/// appointment is keepable.
TEST(RendezvousRefusal, EmptyToursWithAFloorStillPlans) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {}, {});
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, /*floor_cell=*/2, 0, sched0());

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 2);
  EXPECT_TRUE(p.floor_won);
  EXPECT_EQ(p.penalty_mm, 20'000) << "0 m makespan -> 20 m makespan";
  EXPECT_EQ(p.interval_ms, 20'000);
}
