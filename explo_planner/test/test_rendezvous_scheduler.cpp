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
  c.depart_safety_milli = 1000; // 1.0x, so the floor arithmetic is readable
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

/// P5 gate (c). READ THE SCOPE CHANGE FIRST.
///
/// This test used to carry the ENTIRE agreement argument: the v4 design had no
/// proposal, no echo and no adoption, and agreement was supposed to be a
/// CONSEQUENCE of both robots running identical arithmetic over a shared world.
/// Gen 10 falsified that — not the arithmetic, which this test still confirms,
/// but the premise that the two robots feed it the same inputs. In the field
/// each merges its own map and 21 of 64 separated pairs picked the same cell.
/// Agreement is now an explicit propose/echo/commit handshake in the node (see
/// rendezvous_scheduler.hpp's header and the node's P5 state block), and this
/// file does not test it — the handshake has no presence in this library.
///
/// What the test still proves, and why it is still worth running: that the
/// SEARCH is a pure function of the world, so a proposal is auditable offline
/// and a disagreement can be attributed to divergent inputs rather than to the
/// solver. It must be proved HERE rather than inherited from the allocator's
/// own determinism test — the scheduler adds an argmin, an insertion search and
/// a time conversion, any of which could reintroduce a cross-process difference
/// the allocator does not have.
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
// (d) the departure rule is GONE (generation 19)
//
// Four tests stood here pinning RendezvousScheduler::shouldDepart: departures
// staggered by each robot's own travel so the ARRIVALS coincided, the safety
// factor and margin moving only the departure, an overdue deadline departing
// immediately, and -1 inputs refusing rather than reading as overdue. They are
// deleted with the function. Generation 29 restored the BEHAVIOUR they
// described — staggered departures, coincident arrivals — but not in this
// class: it is ExploPlannerNode::appointmentDue, aiming at an instant the team
// agreed rather than at a deadline each robot derived.
//
// WHAT TOOK ITS PLACE, and where its tests are. A robot whose reconnect
// trigger fires signs up to the first occurrence of the committed
// (t_meet, interval) it can still arrive at within rendezvous_max_lateness_sec,
// leaves in time to be there, and waits until the team is whole. The properties
// that replace these tests — that every robot's t_meet is nextAgreedOccurrence
// of the ONE committed pair, that the floor only ever rises above bare now by a
// robot's own shortfall, and that the residual spread is absorbed by an
// unbounded barrier — are node-level, and the expressions are pinned in
// test_gen20_rendezvous.cpp
// (Gen23AgreedSchedule.TheDeadlineIsTheAgreedOccurrenceNotACountdown).
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
//
// THE NODE NO LONGER DRIVES ANY OF THIS. deriveRendezvousProposal hard-clears
// cfg.exclude before every solve, so `rejected_excluded` is 0 on every row a
// gen-10 campaign will produce. The write-off list was PER ROBOT, and a
// per-robot input to a value the whole team must share is the exact mistake
// gen 9 made everywhere else. Anti-deadlock is now the wait cap and the
// one-appointment-per-outage rule, neither of which needs anyone's consent.
//
// These stay because the library feature stays and an untested live code path
// is worse than a tested unused one. Do not read a green run here as evidence
// about a campaign — nothing in a campaign reaches them.
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
// The interval: the reachability floor and the findability cap
//
// THE INTERVAL IS THE RECURRENCE PERIOD AGAIN (generation 23, after generations
// 19-22 in which it was not). Every assertion below is unchanged and still
// correct — solve()'s arithmetic never moved — but WHAT THE NUMBERS MEAN has
// moved twice, and this banner has stated the wrong one before. A robot drives
// to the agreed cell for the first occurrence of `t_meet_ms + k*interval_ms`
// it can still arrive at (nextAgreedOccurrence; floored at t_now plus that
// robot's own shortfall, zero for a robot that can make the nearest one) and
// waits there until the team is whole. `interval_ms` is that period, and it is also the agreed,
// exchanged, compared integer, read as "how long the furthest robot needs to
// get there".
//
// The two bounds survive with it, and both still bite:
//
//   floor: the furthest robot's DIRECT drive, so the integer is a journey
//          somebody can actually make. Without it the objective's own winner —
//          the cheapest space-time near-miss — reports a grid step, 28.284 s in
//          the ts4 smoke, for a meeting that is nothing of the sort.
//   cap:   the barrier's own wait: "the last robot to set off can still arrive
//          before the ones already waiting give up". With the occurrences back
//          and the floor making the interval the furthest robot's direct drive,
//          that is also the older reading — "a robot that misses occurrence k
//          is under one period behind peers still standing there at k".
//
// The floor outranks the cap. Both are fed by the node, so unlike the
// map-divergence cap these replaced, both are live on campaign rows.
// ---------------------------------------------------------------------------

/// The fixture for the two cap tests, built so the TOUR TERM IS WELL ABOVE THE
/// DIRECT DRIVE — which most fixtures are not, because a tour that goes
/// straight to the meeting has a prefix equal to the direct distance and the
/// floor then lands exactly on the tour term.
///
/// Robot 0 at cell 0 (-20,-20) with tour {20, 2}: it drives 40 m north to cell
/// 20 (-20,20) first and only then 44.72 m down to cell 2 (0,-20), reaching the
/// meeting at 84.72 m. Robot 1 at cell 4 (20,-20) has no tour, so it inserts
/// cell 2 at its direct 20 m.
///
///   tour term   = max(84.72, 20)     = 84.72 s at 1 m/s
///   direct term = max(0->2, 4->2) = max(20, 20) = 20 s
///
/// Cells 20 and 2 both cost zero penalty (robot 0 already holds both and robot
/// 1's makespan stays under its), so the tie breaks to the lower id — cell 2.
Allocation detourTours(const CellWorld& w, const std::vector<AllocRobot>& r) {
  return tours2(w, r, {20, 2}, {});
}

/// The cap pulls the interval in to the barrier's wait, and `capped` says it
/// did. The tour term stays readable in `tour_interval_ms`, which is the only
/// place the objective's own answer survives — and the difference between the
/// two is exactly how much of the slowest robot's journey the capped integer no
/// longer accounts for, recoverable from the plan alone. Read them together
/// before drawing any distance conclusion from `interval_ms`.
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
  // solve() writes the two together: t_meet = mission_elapsed + interval, one
  // assignment site. The NODE does not preserve this identity — arming
  // recomputes t_meet as the first AGREED OCCURRENCE the robot can reach
  // (nextAgreedOccurrence), which is t_meet + k*interval for some k >= 0 rather
  // than mission_elapsed + interval — so the identity is a property of a solve
  // result only.
  EXPECT_EQ(p.t_meet_ms, 41'000) << "derive time + interval";
  // The part of the slowest robot's journey the capped integer drops.
  EXPECT_NEAR(static_cast<double>(p.tour_interval_ms - p.interval_ms),
              44'721.0, 2.0);
}

/// THE FLOOR OUTRANKS THE CAP. A cap below the furthest robot's drive asks the
/// interval to claim a journey shorter than the one that has to be made; the
/// number would shrink and the drive would not. It stops at the floor instead.
///
/// BOTH FLAGS ARE TRUE HERE, and that pair is the whole point of the test.
/// `capped` is true because the cap did pull the interval in from the tours'
/// 84.72 s; it just did not get the 5 s it asked for. `floored` is true because
/// the floor is what refused it. Both true — equivalently `interval_ms` above
/// the cap — is the shape that says the findability inequality is broken: the
/// furthest robot cannot reach the cell before the barrier gives up on it, and
/// the node warns when it sees it rather than leaving it to an analysis.
///
/// A derivation that reported NEITHER flag here shipped briefly on 2026-09-18
/// and this test is what caught it. Do not "fix" a failure of these two
/// assertions by relaxing them: a plan that says nothing bound, in the one
/// configuration where two things bound, is a silent null.
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

/// A CAP EXACTLY ON THE TOUR TERM CUT NOTHING, AND MUST NOT SAY IT DID.
///
/// The boundary between the two readings of `capped`, and the only mutant the
/// rest of this group did not kill: `max_interval_ms < tour_interval_ms` versus
/// `<=`. They differ on exactly one input and it is not a contrived one.
///
/// It matters because `capped` is a scored column. A run whose cap happens to
/// sit on the tour term would report the cap as having chosen the meeting time
/// when the objective chose it, and the diagnostic that exists to say "the cap,
/// not the objective, is picking the schedule" would say so falsely. Equality
/// is reachable rather than measure-zero because the node feeds the cap from a
/// configured round number — `rendezvous_appointment_wait_sec` since generation
/// 29 — while the tour term is a quantised grid distance at a nominal speed.
///
/// The cap is read back off a first solve rather than hardcoded, so the test
/// stays on the boundary if the fixture's distances ever change.
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

/// THE REACHABILITY FLOOR, on the fixture the objective is happiest with. Both
/// robots are one cell from the meeting on their own tours, so the tour term is
/// a single grid step — and the direct drive is that same step, so the floor
/// binds only once `depart_safety_milli` marks it up. At 1.5x the 20 s drive
/// becomes a 30 s period.
///
/// This is the safety factor doing its stated job at the schedule level rather
/// than only at the departure test: an occurrence exactly one nominal drive
/// apart leaves no room for the difference between a straight line on the cell
/// graph and what a robot actually does through trees.
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

/// THE CAMPAIGN'S OWN CONFIGURATION, as two numbers rather than as a claim in a
/// comment: a 300 s lattice (`rendezvous_interval_sec`) against an unbounded
/// barrier wait (`rendezvous_appointment_wait_sec` = 0, the directive's "be
/// there until all robots are connected").
///
/// Generation 29 exists because of what the spacing does downstream. A robot
/// signs up to the first rung it can reach inside its lateness budget, and
/// hybrid chases only while its appointment is not yet due — so the chase
/// window is roughly `interval_ms` minus that budget. On the banked generation-
/// 28 armings the derived lattice was 30 s and the budget 60 s, which is a
/// NEGATIVE window on 180 of 211 armings: hybrid armed and never chased once.
/// The two assertions below are what make the 60 s budget affordable, and they
/// are about the returned INTERVAL, not about the knob, because it is the
/// interval the departure test reads.
///
/// `capped` false is a contract, not an observation. The node announces it at
/// startup precisely so a reader finds a constant column and knows it was
/// configured rather than broken — see the rendezvous_appointment_wait_sec
/// INFO in ExploPlannerNode's parameter block.
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

/// A BARRIER WAIT SHORTER THAN THE LATTICE IS OVERRULED SILENTLY, AND `capped`
/// IS NOT THE PLACE THAT SAYS SO.
///
/// This is not hypothetical arithmetic: it is what the campaign configuration
/// produces the moment anyone sets RDV_APPT_WAIT to a finite value without also
/// lowering RDV_INTERVAL below it. The floor outranks the cap, so the answer is
/// a flat 300 s and the broken findability inequality — the furthest robot
/// needs longer to arrive than the barrier will wait — holds on every derive.
///
/// The trap this pins is which signal notices. `capped` asks a narrower
/// question than its name suggests: did the cap CUT THE OBJECTIVE'S OWN ASK.
/// Here the tours asked for 20 s, the cap is 240 s, so the cap cut nothing and
/// the flag is false — correctly, by the definition the solver's own comment
/// spends sixty lines establishing, and the alternative reading is the one that
/// was removed for making `capped` unable to separate "the cap bound" from "the
/// floor did". So an analyst tallying `capped` over a campaign misconfigured
/// exactly this way finds a column of zeroes and concludes the barrier was
/// never overrun.
///
/// What DOES notice is the comparison the node's findability WARN makes
/// directly, `interval_ms > max_interval_ms`, asserted below so that the
/// predicate keeps meaning what the WARN reads it to mean.
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
// Five tests stood here pinning RendezvousScheduler::occurrenceAtOrAfter — a
// future phase returned as-is, at-or-after rather than strictly-after, a phase
// long past rolling forward, two robots straddling a boundary landing EXACTLY
// one period apart, and a non-positive period returning the phase.
//
// The fourth one is why the whole thing is deleted rather than kept for a rainy
// day. It pinned the property the barrier was designed around: "the gap two
// robots have to bridge is exactly one period, which the barrier's wait
// covers." That is true of the function and was false of the system, because
// nothing bounded the arming spread to one period. The N=3 smoke armed at
// 16.1 / 52.5 / 67.4 s against a 30 s period — 1.7 periods — and the three
// robots selected three different occurrences off byte-identical integers.
//
// A green unit test for an arithmetic identity, sitting under a comment
// asserting a system property the arithmetic cannot deliver, is worse than no
// test: it reads as coverage of the thing that broke.
//
// GENERATION 23 TAKES THE ARITHMETIC BACK AND LEAVES THE CLAIM BEHIND. The
// roll-forward lives in planner_util as nextAgreedOccurrence, and the barrier
// it feeds is unbounded (rendezvous_appointment_wait_sec = 0), so two robots
// on different occurrences now cost each other WAITING at the agreed cell
// rather than a missed reunion. Its tests say that under their own names, in
// test_planner_util.cpp — including
// NextAgreedOccurrence.TwoRobotsOnOneAgreementDifferByWholeIntervals, which
// pins the honest version of the property the fourth deleted test overstated.
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
// These pin the two decisions that killed the N>=3 rendezvous arm. Both lived
// inside the ROS node until 2026-09-17 and neither could be exercised without
// running a 600 s cell, which is how both survived three code reviews.
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

/// A stand-in for the node's private RendezvousProposal, with the two
/// operations the latch template needs. Deliberately its own type: if the test
/// used the node's struct it could not be a unit test, and if the template
/// silently required more than this it would be depending on something the
/// caller is not obliged to provide.
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
  // GENERATION 29. The team kept the meeting, the maps merged, and the rule's
  // last clause is that the next place and time are agreed before anyone
  // resumes exploring. On a follower that arrives as R -> R', which the shape
  // above refuses — so without the request flag the re-agreement could not land
  // on anyone but the proposer, the commit gate would never see unanimity, and
  // both appointment arms would re-meet at the t=0 cell for the whole run while
  // every follower logged an ERROR.
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/false,
                  /*eq=*/false, /*due=*/true), Adopt::kReagree);

  // EQUALITY STILL OUTRANKS IT, and this is the case that makes the flag safe
  // to leave set across ticks. The proposer republishes on every heartbeat, so
  // between the request and the answer there are many ticks where the numbers
  // still match. Answering those would spend the request on nothing and leave
  // the robot holding the old meeting with no outstanding ask.
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/false,
                  /*eq=*/true, /*due=*/true), Adopt::kIgnore);

  // The upgrade still outranks it too: a robot owed a replacement that is also
  // still on the placeholder takes the upgrade path, which clears the
  // provisional flag. kReagree would leave that flag stale.
  EXPECT_EQ(adopt(true, /*peer_prov=*/false, true, /*held_prov=*/true,
                  /*eq=*/false, /*due=*/true), Adopt::kUpgrade);
}

TEST(RendezvousHandshakeAdopt, TheUpgradeOutranksEqualityOfTheIntegers) {
  // The proposer re-derived and landed on the same three integers, this time
  // with something to choose between. The team does not move, but the run
  // stops being a placeholder run — and since that is the difference between
  // "measured the scheduler" and "measured meet-at-the-centroid", the flag has
  // to clear. kIgnore here would leave this robot reporting the wrong
  // mechanism forever after.
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
  // Exhaustive over all 32 inputs. Not for coverage — for the property that
  // the function is TOTAL. The version this replaced was an `if` with an
  // `else if` and no `else`, and the case it silently dropped (hold a
  // placeholder, peer offers a final one, follower side) was the upgrade
  // itself: it fell through both branches and did nothing at all.
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
  // THE FALSE-COMMIT BUG, pinned. The peer echoed our placeholder, then
  // upgraded off it. The old code dropped a record only on pair -> empty, so
  // this stale record stood, was counted toward fleet-1, and produced a
  // "Rendezvous AGREED by all" for a triple the peer no longer held — an
  // appointment it would never attend, followed by a no-show logged against a
  // robot that was never on that schedule.
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
  // The other direction of the same race, and it is guarded DIFFERENTLY — the
  // distinction is worth a test because getting it backwards is how the false
  // commit happened in the first place.
  //
  // When the PEER moves on, the record is cleared, because nothing downstream
  // would otherwise notice (test above). When THIS robot moves on, the record
  // is deliberately left alone: it is still a true statement about the peer,
  // the peer has not contradicted it, and there is no message to clear it with
  // — the peers are still happily republishing the placeholder. What stops it
  // counting is that the commit rule compares each record against what this
  // robot CURRENTLY holds, so a record naming a superseded triple simply is not
  // equal to anything the commit is asking about.
  //
  // This test pins that property, because it is the one the safety of leaving
  // the record alone rests on. If the commit rule is ever relaxed to counting
  // valid records instead of matching ones, this is what fails.
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
