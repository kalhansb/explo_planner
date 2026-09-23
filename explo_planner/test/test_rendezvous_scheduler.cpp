// Moved comments: doc/test_rendezvous_scheduler_notes.md
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

/// A 5x5 world of 10 m cells over [-25, 25]^2, fully connected after
/// configure(), so distance() is centroid distance. Cell id = row*5 + col,
/// centre (-20 + 10*col, -20 + 10*row). (notes: rzv-fixture-world5)
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
  c.depart_safety_milli = 1000; // 1.0x, so the floor arithmetic is readable
  c.max_interval_ms = 0;        // uncapped
  return c;
}

std::vector<AllocRobot> pair2(int cell_a, int cell_b) {
  return {AllocRobot{0, cell_a, true, false},
          AllocRobot{1, cell_b, true, false}};
}

/// Hand-builds an allocation with costs from GlobalAllocator::routeCostMm, so
/// tests pin the objective rather than the allocator's tours. The
/// cross-perspective test must solve for real instead.
/// (notes: rzv-fixture-hand-built-tours)
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

/// Robot 0 at cell 0 with tour [1, 2], robot 1 at cell 4 with tour [3]: cell 2
/// wins with zero penalty, and the interval is the slower arrival, 20 m at 1
/// m/s = 20 s. (notes: rzv-objective-zero-penalty-case)
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

/// The penalty is a difference of makespans over one cost function, so it can
/// never be negative; a negative one means the two sides came from different
/// cost models. (notes: rzv-penalty-never-negative)
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

/// The allocator's costs_mm must equal routeCostMm recomputed from its tours,
/// the path the scheduler's base makespan uses; if they differ the penalty
/// mixes a polished and an unpolished cost.
/// (notes: rzv-route-cost-matches-allocator)
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

/// Both tours point away from the other robot, so each tour cell costs 40 m of
/// penalty and the floor (cell 2) wins at 24.72 m. When no tour cell is worth
/// its detour, the plan falls back to the floor.
/// (notes: rzv-floor-wins-on-detour)
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

/// With no floor, cells 20 and 24 tie at 40 m of penalty and the tie breaks to
/// the lower cell id, a value both robots agree on, never to scan order.
/// (notes: rzv-tie-breaks-lower-cell-id)
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

/// Proves the search is a pure function of the world: worlds reconciled only
/// via wire messages, vehicle order reversed on one side, give an identical
/// plan. The node's propose/echo/commit handshake is not tested here.
/// (notes: rzv-cross-perspective-scope)
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

  // The premise first: both tours non-empty, since with empty tours only the
  // floor remains and the two sides would agree for free.
  // (notes: rzv-cross-perspective-premise)
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
// (d) the departure rule is GONE (generation 19)
//
// Departure timing is not in this class: staggered departures live in
// ExploPlannerNode::appointmentDue, and the agreed-occurrence properties are
// pinned in test_gen20_rendezvous.cpp. (notes: rzv-departure-rule-removed)
// ---------------------------------------------------------------------------

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

/// An unreachable cell leaves the candidate set and is counted in
/// rejected_unreachable. Reachability here must not go through costMm, whose
/// centroid fallback on an unreachable pair is wrong for an appointment.
/// (notes: rzv-unreachable-rejected-counted)
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

/// The floor is admitted even when the graph calls it unreachable, so an
/// over-aggressive edge probe cannot remove the one destination that is always
/// available. (notes: rzv-floor-survives-unreachable)
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
//
// The node does not drive this: deriveRendezvousProposal clears cfg.exclude
// before every solve, so rejected_excluded is 0 in runs. These tests keep the
// library feature covered. (notes: rzv-no-show-not-driven-by-node)
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

/// Exclusion never applies to the floor, so it can never empty the candidate
/// set: excluding every cell, the floor included, still yields a plan.
/// (notes: rzv-exclusion-never-empties-set)
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
// The interval: the reachability floor and the findability cap
//
// interval_ms is the agreed recurrence period: robots meet at t_meet_ms +
// k*interval_ms. Its floor is the furthest robot's direct drive, its cap the
// barrier's wait, and the floor outranks the cap.
// (notes: rzv-interval-floor-and-cap)
// ---------------------------------------------------------------------------

/// Tour term (84.72 s) well above the direct drive (20 s): robot 0 at cell 0
/// with tour {20, 2}, robot 1 at cell 4 with no tour. Cells 20 and 2 both cost
/// zero penalty; the tie breaks to cell 2. (notes: rzv-fixture-detour-tours)
Allocation detourTours(const CellWorld& w, const std::vector<AllocRobot>& r) {
  return tours2(w, r, {20, 2}, {});
}

/// The cap pulls interval_ms in to the barrier's wait and sets capped;
/// tour_interval_ms keeps the objective's own answer. Read them together before
/// drawing distance conclusions from interval_ms.
/// (notes: rzv-cap-advertises-cut)
TEST(RendezvousCap, PullsTheIntervalInAndAdvertisesThatItDid) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = detourTours(w, robots);

  RendezvousScheduler::Config c = sched0();
  c.max_interval_ms = 40'000;
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 1'000, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 2);
  EXPECT_TRUE(p.capped);
  EXPECT_FALSE(p.floored) << "40 s cap is above the 20 s floor";
  EXPECT_NEAR(static_cast<double>(p.tour_interval_ms), 84'721.0, 2.0)
      << "the objective's own answer stays readable";
  EXPECT_EQ(p.interval_ms, 40'000);
  // In a solve result t_meet = mission_elapsed + interval. The node does not
  // preserve this: arming recomputes t_meet via nextAgreedOccurrence as t_meet
  // + k*interval, k >= 0. (notes: rzv-t-meet-solve-identity)
  EXPECT_EQ(p.t_meet_ms, 41'000) << "derive time + interval";
  // The part of the slowest robot's journey the capped integer drops.
  EXPECT_NEAR(static_cast<double>(p.tour_interval_ms - p.interval_ms),
              44'721.0, 2.0);
}

/// A cap below the furthest robot's drive stops at the floor. Both capped and
/// floored are true here, the broken-findability shape the node warns on; do
/// not relax these assertions. (notes: rzv-floor-outranks-cap)
TEST(RendezvousCap, CannotCutBelowTheReachabilityFloor) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = detourTours(w, robots);

  RendezvousScheduler::Config c = sched0();
  c.max_interval_ms = 5'000;           // below the 20 s direct drive
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 1'000, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.interval_ms, 20'000) << "the direct drive, not the 5 s cap";
  EXPECT_GT(p.interval_ms, c.max_interval_ms) << "the broken-inequality shape";
  EXPECT_TRUE(p.capped) << "the cap cut from the tours' 84.72 s";
  EXPECT_TRUE(p.floored) << "and the reachability floor is what refused it";
  EXPECT_EQ(p.t_meet_ms, 21'000);
}

/// A cap exactly equal to the tour term cuts nothing, so capped must be false
/// (strict <, not <=). The cap is read back from an uncapped solve so the test
/// stays on the boundary. (notes: rzv-cap-on-tour-term-not-a-cut)
TEST(RendezvousCap, ACapExactlyOnTheTourTermIsNotACut) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = detourTours(w, robots);

  RendezvousScheduler::Config c = sched0();
  c.max_interval_ms = 0;                       // uncapped: learn the tour term
  const RendezvousPlan base =
      RendezvousScheduler::solve(w, robots, a, -1, 1'000, c);
  ASSERT_TRUE(base.valid()) << base.refused;
  ASSERT_GT(base.tour_interval_ms, 0);
  ASSERT_FALSE(base.capped) << "an uncapped solve cannot be capped";

  c.max_interval_ms = base.tour_interval_ms;   // exactly on the boundary
  const RendezvousPlan p =
      RendezvousScheduler::solve(w, robots, a, -1, 1'000, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.interval_ms, base.interval_ms)
      << "a cap equal to the tour term changes nothing about the answer";
  EXPECT_FALSE(p.capped)
      << "the cap is equal to the tour term, so it cut nothing — reporting it "
         "as a cut credits the cap with a schedule the objective chose";
  EXPECT_FALSE(p.floored) << "the floor is below the tour term in this fixture";
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

/// Tour term and direct drive are both one 20 s step, so the floor binds only
/// through depart_safety_milli: at 1.5x the period becomes 30 s.
/// (notes: rzv-floor-safety-raises-period)
TEST(RendezvousFloorInterval, DirectDriveWithSafetyRaisesThePeriod) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  c.depart_safety_milli = 1500;
  const RendezvousPlan p = RendezvousScheduler::solve(w, robots, a, -1, 0, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.cell, 2);
  EXPECT_EQ(p.tour_interval_ms, 20'000);
  EXPECT_TRUE(p.floored);
  EXPECT_EQ(p.interval_ms, 30'000) << "1.5 x the 20 m direct drive";
  EXPECT_EQ(p.t_meet_ms, 30'000);
}

/// `min_interval_ms` is the other half of the floor and it is a max, not a
/// sum: a configured minimum below the drive changes nothing.
TEST(RendezvousFloorInterval, MinIntervalIsAFloorNotAnAddend) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  c.min_interval_ms = 5'000;           // under the 20 s drive
  const RendezvousPlan below = RendezvousScheduler::solve(w, robots, a, -1, 0, c);
  ASSERT_TRUE(below.valid()) << below.refused;
  EXPECT_EQ(below.interval_ms, 20'000);
  EXPECT_FALSE(below.floored);

  c.min_interval_ms = 45'000;          // over it
  const RendezvousPlan above = RendezvousScheduler::solve(w, robots, a, -1, 0, c);
  ASSERT_TRUE(above.valid()) << above.refused;
  EXPECT_EQ(above.tour_interval_ms, 20'000) << "the tour term is untouched";
  EXPECT_TRUE(above.floored);
  EXPECT_EQ(above.interval_ms, 45'000);
}

/// Campaign configuration: a 300 s lattice (rendezvous_interval_sec, as
/// min_interval_ms) and an unbounded barrier wait
/// (rendezvous_appointment_wait_sec of 0). capped false is a contract the node
/// announces at startup. (notes: rzv-campaign-lattice-uncapped)
TEST(RendezvousFloorInterval, TheCampaignLatticeHoldsAndTheUncappedBarrierNeverBinds) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  c.min_interval_ms = 300'000;   // RDV_INTERVAL=300
  c.max_interval_ms = 0;         // RDV_APPT_WAIT=0, wait forever
  const RendezvousPlan p = RendezvousScheduler::solve(w, robots, a, -1, 0, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.interval_ms, 300'000)
      << "the 300 s lattice did not survive the solve, so the rung spacing a "
         "late robot rolls to — and hybrid's chase window — is whatever the "
         "tours happened to ask for";
  EXPECT_GE(p.interval_ms - 60'000, 240'000)
      << "the WIDEST hybrid's chase window can be at the campaign's 60 s "
         "lateness budget — a supremum, not a floor: the window is the gap "
         "from the arming instant to the next rung less min(lead, budget), so "
         "an arming that lands just short of a rung gets none of it";
  EXPECT_TRUE(p.floored) << "the lattice is what raised it above the tours";
  EXPECT_FALSE(p.capped)
      << "max_interval_ms=0 is uncapped; a true `capped` here would mean the "
         "0 sentinel had started being read as a 0 ms cap";
  EXPECT_EQ(p.t_meet_ms, 300'000) << "mission_elapsed 0 + one interval";
}

/// A barrier wait below the lattice is overruled by the floor, and capped stays
/// false because the cap cut nothing of the tours' ask. The node's findability
/// WARN reads interval_ms above max_interval_ms; this pins it.
/// (notes: rzv-barrier-under-lattice-overruled)
TEST(RendezvousCap, ABarrierWaitUnderTheLatticeIsOverruledAndCappedDoesNotSaySo) {
  CellWorld w = world5(0);
  const auto robots = pair2(0, 4);
  const Allocation a = tours2(w, robots, {1, 2}, {3});

  RendezvousScheduler::Config c = sched0();
  c.min_interval_ms = 300'000;
  c.max_interval_ms = 240'000;   // below the lattice
  const RendezvousPlan p = RendezvousScheduler::solve(w, robots, a, -1, 0, c);

  ASSERT_TRUE(p.valid()) << p.refused;
  EXPECT_EQ(p.interval_ms, 300'000)
      << "the floor no longer outranks the cap, so a 240 s barrier now cuts "
         "the lattice down to a spacing the furthest robot cannot drive";
  EXPECT_GT(p.interval_ms, c.max_interval_ms)
      << "the shape the node's findability WARN fires on";
  EXPECT_TRUE(p.floored) << "the lattice is what raised it above the tours";
  EXPECT_FALSE(p.capped)
      << "`capped` has gone back to meaning \"the cap was not the answer\" "
         "rather than \"the cap cut the ask\". Under that reading it is true "
         "here AND true whenever the floor alone bound, which is the "
         "conflation that made the flag unable to separate the two";
}

// ---------------------------------------------------------------------------
// occurrenceAtOrAfter is GONE (generation 19); the schedule is not
//
// The occurrence roll-forward lives in planner_util as nextAgreedOccurrence and
// is tested in test_planner_util.cpp.
// (notes: rzv-occurrence-roll-forward-moved)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// The handshake: adopt/upgrade/conflict, and the peer latch.
//
// (notes: rzv-handshake-tests-origin)
// ---------------------------------------------------------------------------

namespace {

using Adopt = RendezvousHandshake::Adopt;

/// `due` defaults to false so that every case below which does not name it is
/// asking the pre-generation-29 question: what happens to a robot that is NOT
/// waiting to be told the next place and time. That is still the overwhelming
/// majority of ticks, and the refusals it must produce are unchanged.
Adopt adopt(bool pv, bool pp, bool hv, bool hp, bool eq, bool due = false) {
  return RendezvousHandshake::adopt(pv, pp, hv, hp, eq, due);
}

/// Stand-in for the node's private RendezvousProposal with only valid() and ==,
/// the two operations the latch template needs; deliberately its own type.
/// (notes: rzv-latch-test-triple-type)
struct Triple {
  int  cell = -1;
  int  when = -1;
  bool valid() const { return cell >= 0 && when >= 0; }
  bool operator==(const Triple& o) const {
    return cell == o.cell && when == o.when;
  }
};

}  // namespace

TEST(RendezvousHandshakeAdopt, AnInvalidProposalIsNeverActedOn) {
  // Silence is not a withdrawal, and it is not a conflict either. Every
  // combination of what THIS robot holds must come back kIgnore, because a
  // proposer that has said nothing has not contradicted anything.
  for (int bits = 0; bits < 8; ++bits) {
    const bool pp = bits & 1, hv = bits & 2, hp = bits & 4;
    EXPECT_EQ(adopt(/*peer_valid=*/false, pp, hv, hp, /*eq=*/false),
              Adopt::kIgnore) << "bits=" << bits;
    EXPECT_EQ(adopt(/*peer_valid=*/false, pp, hv, hp, /*eq=*/true),
              Adopt::kIgnore) << "bits=" << bits;
  }
}

TEST(RendezvousHandshakeAdopt, HoldingNothingTakesWhateverIsOffered) {
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, /*held=*/false, false, false),
            Adopt::kTake);
  // Including a provisional one. Taking the placeholder is the POINT of the
  // bootstrap: a team that waits for a tour-informed proposal has no
  // appointment at all during the window where it is about to need one.
  EXPECT_EQ(adopt(true, /*peer_prov=*/true, /*held=*/false, false, false),
            Adopt::kTake);
}

TEST(RendezvousHandshakeAdopt, TheOneUpgradeFiresAndOnlyInThatDirection) {
  // P -> R: the upgrade.
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/true, false),
            Adopt::kUpgrade);
  // R -> P would be a proposer walking backwards onto the centroid. It is a
  // conflict, never an adoption, whether or not a replacement was asked for:
  // the answer to "tell me the next meeting" is not a placeholder.
  EXPECT_EQ(adopt(true, /*peer_prov=*/true, true, /*held_prov=*/false, false),
            Adopt::kConflict);
  EXPECT_EQ(adopt(true, /*peer_prov=*/true, true, /*held_prov=*/false, false,
                  /*due=*/true), Adopt::kConflict);
  // P -> P is the centroid being republished as a different cell, which the
  // derive gate is supposed to make impossible. If it happens the published
  // meeting place would follow the team's centroid around; refuse it.
  EXPECT_EQ(adopt(true, /*peer_prov=*/true, true, /*held_prov=*/true, false),
            Adopt::kConflict);
  // R -> R is a second real generation, and UNASKED-FOR it is still the shape
  // that once put one robot on cell 56 and its partner on cell 57.
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/false, false),
            Adopt::kConflict);
}

TEST(RendezvousHandshakeAdopt, TheReplacementLandsOnlyWhenItWasAskedFor) {
  // After a kept meeting the next place and time are agreed before anyone
  // resumes; a follower receives R -> R', which must land as kReagree when due
  // is set, or the commit never reaches unanimity.
  // (notes: rzv-adopt-reagree-when-due)
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/false,
                  /*eq=*/false, /*due=*/true), Adopt::kReagree);

  // Equality outranks the request: the proposer republishes every heartbeat,
  // and answering a matching republish would spend the request and leave the
  // robot holding the old meeting. (notes: rzv-adopt-equality-outranks-reagree)
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/false,
                  /*eq=*/true, /*due=*/true), Adopt::kIgnore);

  // The upgrade still outranks it too: a robot owed a replacement that is also
  // still on the placeholder takes the upgrade path, which clears the
  // provisional flag. kReagree would leave that flag stale.
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/true,
                  /*eq=*/false, /*due=*/true), Adopt::kUpgrade);
}

TEST(RendezvousHandshakeAdopt, TheUpgradeOutranksEqualityOfTheIntegers) {
  // The upgrade outranks integer equality: the provisional flag must clear even
  // when the re-derived triple is unchanged, or this robot keeps reporting the
  // placeholder mechanism. (notes: rzv-adopt-upgrade-outranks-equality)
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/true,
                  /*eq=*/true), Adopt::kUpgrade);
}

TEST(RendezvousHandshakeAdopt, RepublishingWhatWeAlreadyHoldIsANoOp) {
  // The overwhelmingly common case: the proposer re-sends its triple on every
  // heartbeat for the whole run. Neither flag state may turn that into a
  // conflict, or the log fills with ERRORs on a healthy run.
  EXPECT_EQ(adopt(true, false, true, false, /*eq=*/true), Adopt::kIgnore);
  EXPECT_EQ(adopt(true, true,  true, true,  /*eq=*/true), Adopt::kIgnore);
}

TEST(RendezvousHandshakeAdopt, EveryInputCombinationHasExactlyOneAnswer) {
  // Exhaustive over every input, to pin that adopt is total: each combination
  // maps to exactly one answer. (notes: rzv-adopt-is-total)
  int taken = 0, upgraded = 0, conflicts = 0, ignored = 0, reagreed = 0;
  for (int bits = 0; bits < 64; ++bits) {
    const bool pv = bits & 1, pp = bits & 2, hv = bits & 4,
               hp = bits & 8, eq = bits & 16, due = bits & 32;
    switch (adopt(pv, pp, hv, hp, eq, due)) {
      case Adopt::kTake:     ++taken;     break;
      case Adopt::kUpgrade:  ++upgraded;  break;
      case Adopt::kConflict: ++conflicts; break;
      case Adopt::kIgnore:   ++ignored;   break;
      case Adopt::kReagree:  ++reagreed;  break;
    }
  }
  EXPECT_EQ(taken + upgraded + conflicts + ignored + reagreed, 64);
  // 32 with peer_valid=false (nothing on offer, whatever we hold and whatever
  // we are waiting for), plus the 3 (pp, hp) combinations that already match
  // and are not the upgrade, at either value of `due`.
  EXPECT_EQ(ignored, 38);
  // Peer valid, hold nothing: pp x hp x eq x due. hp is meaningless when
  // held_valid is false and the function must not care — it is still 16 inputs
  // and they must all take the offer. `due` cannot matter either: a robot
  // holding nothing has no meeting to have kept.
  EXPECT_EQ(taken, 16);
  // Peer valid and final, hold the placeholder, either value of eq and of due.
  EXPECT_EQ(upgraded, 4);
  // EXACTLY ONE INPUT re-agrees, and that narrowness is the point: peer final,
  // holding a final we do not already match, and owed a replacement. Every
  // neighbouring input is a refusal or a no-op.
  EXPECT_EQ(reagreed, 1);
  // R->P at either value of due, P->P at either, and R->R unasked-for.
  EXPECT_EQ(conflicts, 5);
}

TEST(RendezvousHandshakeLatch, AMatchingEchoIsRecorded) {
  const Triple held{7, 100};
  Triple latched;
  RendezvousHandshake::updatePeerLatch(Triple{7, 100}, held, latched);
  EXPECT_TRUE(latched == held);
}

TEST(RendezvousHandshakeLatch, TheRecordSurvivesTheSilenceItWasMadeFor) {
  // The whole reason the latch exists: at N>=3 the echoes do not coincide, so
  // a record has to outlive the tick it was made on. Repeating the same
  // message must not disturb it either.
  const Triple held{7, 100};
  Triple latched;
  RendezvousHandshake::updatePeerLatch(Triple{7, 100}, held, latched);
  for (int i = 0; i < 5; ++i)
    RendezvousHandshake::updatePeerLatch(Triple{7, 100}, held, latched);
  EXPECT_TRUE(latched == held);
}

TEST(RendezvousHandshakeLatch, AnUpgradeAwayClearsTheRecordRatherThanKeepingIt) {
  // A peer that echoed our triple and then moved to another must have its latch
  // record cleared; a stale record would count toward the commit for a triple
  // the peer no longer holds. (notes: rzv-latch-false-commit)
  const Triple held{7, 100};
  Triple latched;
  RendezvousHandshake::updatePeerLatch(Triple{7, 100}, held, latched);
  ASSERT_TRUE(latched == held);
  RendezvousHandshake::updatePeerLatch(Triple{9, 240}, held, latched);
  EXPECT_FALSE(latched.valid())
      << "a peer that has moved on must not still count toward the commit";
}

TEST(RendezvousHandshakeLatch, WalkingBackToNothingAlsoClearsIt) {
  const Triple held{7, 100};
  Triple latched;
  RendezvousHandshake::updatePeerLatch(Triple{7, 100}, held, latched);
  ASSERT_TRUE(latched == held);
  RendezvousHandshake::updatePeerLatch(Triple{}, held, latched);
  EXPECT_FALSE(latched.valid());
}

TEST(RendezvousHandshakeLatch, AnEchoOfSomethingElseIsNeverRecorded) {
  // A peer publishing a triple that is not ours gives no evidence about ours.
  const Triple held{7, 100};
  Triple latched;
  RendezvousHandshake::updatePeerLatch(Triple{9, 240}, held, latched);
  EXPECT_FALSE(latched.valid());
}

TEST(RendezvousHandshakeLatch, OurOwnUpgradeStrandsThePeerRecordsHarmlessly) {
  // When this robot moves on, the peer's record is left alone; it stops
  // counting because the commit rule compares each record with the triple this
  // robot currently holds. Counting valid records instead breaks this.
  // (notes: rzv-latch-own-upgrade-strands-record)
  const Triple placeholder{9, 240};
  Triple latched;
  RendezvousHandshake::updatePeerLatch(placeholder, placeholder, latched);
  ASSERT_TRUE(latched == placeholder);

  const Triple upgraded{7, 100};
  RendezvousHandshake::updatePeerLatch(placeholder, upgraded, latched);
  EXPECT_FALSE(latched == upgraded)
      << "a record against a triple this robot has superseded must not be "
         "equal to the one it now holds, or it would count toward the commit";

  // ...and it re-records the moment that peer echoes the new one.
  RendezvousHandshake::updatePeerLatch(upgraded, upgraded, latched);
  EXPECT_TRUE(latched == upgraded);
}
