#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/global_allocator.hpp"

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

/// A 5x5 world of 10 m cells over [-25, 25]^2 for robot `self`.
CellWorld world5(int self = 0) {
  CellWorld w;
  const std::string err =
      w.configure(makeCellGrid(-25.0f, 25.0f, -25.0f, 25.0f, 10.0f),
                  cfg0(), self);
  EXPECT_EQ(err, "") << "fixture failed to configure";
  return w;
}

/// Force a cell to a status first-hand, bypassing the observation machinery.
/// commitSelf is the real first-hand path, so this exercises the same
/// update_id / known_by rules the node would.
void setSelf(CellWorld& w, int id, CellStatus s) { w.commitSelf(id, s); }

std::vector<AllocRobot> pair2(int cell_a, int cell_b) {
  return {AllocRobot{0, cell_a, true, false},
          AllocRobot{1, cell_b, true, false}};
}

/// Every cell in both tours, sorted — the set of work actually allocated.
std::vector<int> allAssigned(const Allocation& a) {
  std::vector<int> v;
  for (const auto& t : a.tours) v.insert(v.end(), t.begin(), t.end());
  std::sort(v.begin(), v.end());
  return v;
}

} // namespace

// ---------------------------------------------------------------------------
// The premise: two robots must solve the SAME problem and get the SAME answer.
// ---------------------------------------------------------------------------

/// THE GATE TEST (plan §4, P3): cross-perspective determinism THROUGH THE WIRE
/// CODEC. Not two copies of one world — that would only prove solve() is a
/// function. The worlds are built from opposite viewpoints and reconciled by
/// exchanging wire messages, so anything that survives the round trip as a
/// local-only difference (a *_BY_OTHERS provenance byte, a known_by mask, a
/// local update_id) gets a chance to reach the allocation and change it.
///
/// This is the test that fails if someone "improves" the allocator by reading
/// isFirstHand(), or by preferring cells this robot discovered, or by breaking
/// a tie on anything that is not (cell id, robot id).
TEST(GlobalAllocatorCrossPerspective, IdenticalAllocationFromBothSides) {
  CellWorld a = world5(0);
  CellWorld b = world5(1);

  // A saw 6, 7, 8 exploring; B saw 16, 17 exploring. Disjoint discoveries, so
  // before the exchange the two hold genuinely different worlds.
  for (int id : {6, 7, 8}) setSelf(a, id, CellStatus::EXPLORING);
  for (int id : {16, 17}) setSelf(b, id, CellStatus::EXPLORING);

  // Exchange, both directions, through the codec.
  const auto a_wire = a.toWire();
  const auto b_wire = b.toWire();
  b.mergeWire(0, a_wire);
  a.mergeWire(1, b_wire);

  // Both must now agree about the shared belief; if they do not, the rest of
  // this test is meaningless rather than merely failing.
  ASSERT_EQ(a.sharedHash(), b.sharedHash());

  const auto robots = pair2(0, 24);
  const Allocation from_a = GlobalAllocator::solve(a, robots, {});
  const Allocation from_b = GlobalAllocator::solve(b, robots, {});

  EXPECT_EQ(from_a.tours, from_b.tours);
  EXPECT_EQ(from_a.costs_mm, from_b.costs_mm);
  EXPECT_EQ(from_a.unassigned, from_b.unassigned);
}

/// The provenance byte must not reach the allocation. A cell A covered itself
/// and one A learned about from B are the same work, and §3.4 says so
/// explicitly: restricting to first-hand EXPLORING would have the robots
/// solving provably different problems after every exchange.
TEST(GlobalAllocatorCrossPerspective, ByOthersCellsAreCandidates) {
  CellWorld w = world5(0);
  setSelf(w, 6, CellStatus::EXPLORING);

  CellWorld peer = world5(1);
  setSelf(peer, 18, CellStatus::EXPLORING);
  w.mergeWire(1, peer.toWire());
  ASSERT_EQ(w.status(18), CellStatus::EXPLORING_BY_OTHERS);

  const Allocation alloc = GlobalAllocator::solve(w, pair2(0, 24), {});
  const std::vector<int> got = allAssigned(alloc);
  EXPECT_EQ(got, (std::vector<int>{6, 18}));
}

/// The caller's vector order is not part of the problem. Two robots that list
/// the fleet differently (a map iteration, a params order) must not thereby
/// allocate differently — and the output must stay aligned to whatever order
/// the caller did pass.
TEST(GlobalAllocatorDeterminism, VehicleOrderDoesNotChangeTheAnswer) {
  CellWorld w = world5(0);
  for (int id : {6, 7, 8, 16, 17}) setSelf(w, id, CellStatus::EXPLORING);

  const std::vector<AllocRobot> fwd = {AllocRobot{0, 0, true, false},
                                       AllocRobot{1, 24, true, false}};
  const std::vector<AllocRobot> rev = {AllocRobot{1, 24, true, false},
                                       AllocRobot{0, 0, true, false}};

  const Allocation f = GlobalAllocator::solve(w, fwd, {});
  const Allocation r = GlobalAllocator::solve(w, rev, {});

  // Index-aligned to the CALLER's order, so the tours swap position...
  EXPECT_EQ(f.tours[0], r.tours[1]);
  EXPECT_EQ(f.tours[1], r.tours[0]);
  // ...and the focus each robot reads for itself is identical either way.
  EXPECT_EQ(f.focusFor(0, fwd), r.focusFor(0, rev));
  EXPECT_EQ(f.focusFor(1, fwd), r.focusFor(1, rev));
}

/// Tie-break totality. Two robots placed symmetrically about a single
/// equidistant cell have exactly equal insertion costs, so nothing but the
/// declared tie-break can decide it. The rule is (cell id, robot id): the
/// lower robot id takes it, on both robots, every time.
TEST(GlobalAllocatorDeterminism, ExactTieBreaksToTheLowerRobotId) {
  CellWorld w = world5(0);
  setSelf(w, 12, CellStatus::EXPLORING);      // centre of the 5x5

  // Cells 2 and 22 are the same graph distance from 12 (two rows either way).
  const auto robots = pair2(2, 22);
  ASSERT_EQ(GlobalAllocator::costMm(w, 2, 12),
            GlobalAllocator::costMm(w, 22, 12))
      << "fixture is not actually a tie";

  const Allocation alloc = GlobalAllocator::solve(w, robots, {});
  EXPECT_EQ(alloc.tours[0], (std::vector<int>{12}));
  EXPECT_TRUE(alloc.tours[1].empty());
}

/// Cost quantisation is the cross-process determinism guarantee, so it is
/// tested at the primitive rather than inferred from a tour. A 10 m cell step
/// is exactly 10000 mm; the diagonal is the rounded Euclidean value, not a
/// truncation.
TEST(GlobalAllocatorCost, QuantisesToWholeMillimetres) {
  CellWorld w = world5(0);
  EXPECT_EQ(GlobalAllocator::costMm(w, 0, 1), 10000);
  EXPECT_EQ(GlobalAllocator::costMm(w, 0, 5), 10000);
  EXPECT_EQ(GlobalAllocator::costMm(w, 0, 0), 0);
  // 8-connected diagonal: sqrt(2) * 10 m = 14142.1356... mm. Within 1 mm
  // rather than exact, because the centroids are floats and the last unit is
  // the compiler's business; what matters is that the RESULT is an integer, so
  // two robots comparing it cannot disagree in a bit nobody can see.
  EXPECT_NEAR(GlobalAllocator::costMm(w, 0, 6), 14142, 1);
}

/// An unreachable cell degrades to straight-line cost rather than dropping out
/// of the problem. A cell nobody can be assigned is a cell nobody clears, and
/// it stays EXPLORING forever — which the rendezvous minimax would then keep
/// pointing at.
TEST(GlobalAllocatorCost, UnreachableFallsBackToCentroidDistance) {
  CellWorld w = world5(0);
  // Disable every edge: the probe reports the whole line blocked.
  w.rebuildEdges([](float, float, float, float) { return 1.0; });
  ASSERT_LT(w.distance(0, 24), 0.0) << "fixture did not actually disconnect";
  EXPECT_EQ(GlobalAllocator::costMm(w, 0, 24),
            static_cast<long long>(std::llround(w.centroidDistance(0, 24) *
                                                1000.0)));
}

// ---------------------------------------------------------------------------
// The comms mask — the port of GetDistanceMatricesNoComms
// ---------------------------------------------------------------------------

/// An out-of-comms robot may only be given cells it already knows about. This
/// is what makes the no_comms/assume_comms cost gap in §3.6 mean something:
/// without the mask both plans are identical and the reconnection value is
/// always zero, which is a P4 gate that cannot fire.
TEST(GlobalAllocatorMask, OutOfCommsRobotOnlyGetsCellsItKnows) {
  CellWorld w = world5(0);
  for (int id : {6, 18}) setSelf(w, id, CellStatus::EXPLORING);
  // Robot 1 knows about 6 but has never heard of 18.
  w.markKnownBy(6, 1);

  GlobalAllocator::Config cfg;
  cfg.comms_mask = true;
  std::vector<AllocRobot> robots = pair2(0, 24);
  robots[1].in_comms = false;

  const Allocation alloc = GlobalAllocator::solve(w, robots, cfg);
  const std::vector<int>& t1 = alloc.tours[1];
  EXPECT_EQ(std::count(t1.begin(), t1.end(), 18), 0)
      << "an out-of-comms robot was assigned a cell it cannot know exists";
}

/// The mask is OFF by default, and the same fixture proves the previous test
/// was testing the mask rather than the geometry. Without this pair, a mask
/// that silently did nothing and a mask that worked would both pass — the
/// no-mask solve has to be shown to differ.
TEST(GlobalAllocatorMask, WithoutTheMaskTheSameFixtureAllocatesFreely) {
  CellWorld w = world5(0);
  for (int id : {6, 18}) setSelf(w, id, CellStatus::EXPLORING);
  w.markKnownBy(6, 1);

  std::vector<AllocRobot> robots = pair2(0, 24);
  robots[1].in_comms = false;

  const Allocation alloc = GlobalAllocator::solve(w, robots, {});
  const std::vector<int>& t1 = alloc.tours[1];
  EXPECT_EQ(std::count(t1.begin(), t1.end(), 18), 1)
      << "cell 18 is nearest robot 1; unmasked it must be robot 1's";
}

/// A cell no vehicle may take is REPORTED, not silently dropped. "Nothing to
/// do" and "the mask starved the allocation" are opposite findings that
/// present identically as empty tours.
TEST(GlobalAllocatorMask, StarvedCandidatesAreReportedNotDiscarded) {
  CellWorld w = world5(0);
  setSelf(w, 18, CellStatus::EXPLORING);   // known_by == {0}: this robot only

  GlobalAllocator::Config cfg;
  cfg.comms_mask = true;
  // Both peers are dark and neither has ever heard of cell 18. Robot 0 is
  // deliberately absent from the vehicle set: it is the one robot that DOES
  // know the cell, so including it would let it take the cell and hide the
  // starvation this test is about.
  const std::vector<AllocRobot> robots = {AllocRobot{1, 0, false, false},
                                          AllocRobot{2, 24, false, false}};

  const Allocation alloc = GlobalAllocator::solve(w, robots, cfg);
  EXPECT_TRUE(allAssigned(alloc).empty());
  EXPECT_EQ(alloc.unassigned, (std::vector<int>{18}));
  EXPECT_TRUE(alloc.refused.empty()) << "starvation is a result, not a refusal";
}

// ---------------------------------------------------------------------------
// Vehicle-set rules
// ---------------------------------------------------------------------------

/// A finished peer leaves the problem and its work returns to the pool. Left
/// in with an empty tour it would keep absorbing cells through the makespan
/// balance — reserving work for a robot that has stopped moving.
TEST(GlobalAllocatorVehicles, FinishedRobotIsRemovedAndItsCellsReturn) {
  CellWorld w = world5(0);
  for (int id : {20, 21, 22}) setSelf(w, id, CellStatus::EXPLORING);

  std::vector<AllocRobot> robots = pair2(0, 24);
  const Allocation live = GlobalAllocator::solve(w, robots, {});
  ASSERT_FALSE(live.tours[1].empty()) << "fixture: robot 1 should get work";

  robots[1].finished = true;
  const Allocation done = GlobalAllocator::solve(w, robots, {});
  EXPECT_TRUE(done.tours[1].empty());
  EXPECT_EQ(allAssigned(done), (std::vector<int>{20, 21, 22}))
      << "the finished robot's cells must come back to the pool";
}

/// An unlocatable robot is dropped rather than defaulted onto some cell: every
/// cost involving it would be fiction, and fiction that moves the makespan
/// moves the OTHER robot's tour too.
TEST(GlobalAllocatorVehicles, UnlocatableRobotIsDroppedNotDefaulted) {
  CellWorld w = world5(0);
  for (int id : {6, 7}) setSelf(w, id, CellStatus::EXPLORING);

  std::vector<AllocRobot> robots = pair2(0, -1);
  const Allocation alloc = GlobalAllocator::solve(w, robots, {});
  EXPECT_TRUE(alloc.tours[1].empty());
  EXPECT_EQ(alloc.focusFor(1, robots), -1)
      << "no tour means no focus, which is the fall-back-to-unrestricted path";
  EXPECT_EQ(allAssigned(alloc), (std::vector<int>{6, 7}));
}

TEST(GlobalAllocatorVehicles, NoUsableVehicleIsARefusalNotAnEmptyAnswer) {
  CellWorld w = world5(0);
  setSelf(w, 6, CellStatus::EXPLORING);

  std::vector<AllocRobot> robots = pair2(-1, -1);
  const Allocation alloc = GlobalAllocator::solve(w, robots, {});
  EXPECT_FALSE(alloc.refused.empty());
}

// ---------------------------------------------------------------------------
// Makespan balance and the focus cell
// ---------------------------------------------------------------------------

/// The point of makespan balance: work goes to the robot that can absorb it
/// most cheaply, so two robots at opposite corners each take their own side
/// rather than one taking everything.
TEST(GlobalAllocatorBalance, EachRobotTakesItsOwnSide) {
  CellWorld w = world5(0);
  for (int id : {0, 1, 5, 6}) setSelf(w, id, CellStatus::EXPLORING);
  for (int id : {18, 19, 23, 24}) setSelf(w, id, CellStatus::EXPLORING);

  const auto robots = pair2(0, 24);
  const Allocation alloc = GlobalAllocator::solve(w, robots, {});

  for (int id : alloc.tours[0]) EXPECT_LT(id, 12) << "cell " << id;
  for (int id : alloc.tours[1]) EXPECT_GT(id, 12) << "cell " << id;
  EXPECT_EQ(allAssigned(alloc).size(), 8u);
}

/// The focus cell is the first cell of MY tour and nothing else. doPlan reads
/// exactly this one value out of the whole solve.
TEST(GlobalAllocatorFocus, FocusIsTheFirstCellOfOwnTour) {
  CellWorld w = world5(0);
  for (int id : {6, 7, 8}) setSelf(w, id, CellStatus::EXPLORING);

  const auto robots = pair2(0, 24);
  const Allocation alloc = GlobalAllocator::solve(w, robots, {});
  ASSERT_FALSE(alloc.tours[0].empty());
  EXPECT_EQ(alloc.focusFor(0, robots), alloc.tours[0].front());
  EXPECT_EQ(alloc.focusFor(7, robots), -1) << "an id not in the fleet";
}

/// Nothing to explore is a SOLVED problem with an empty answer, not a refusal.
/// The two are handled by different code paths in doPlan.
TEST(GlobalAllocatorFocus, NothingToExploreIsSolvedNotRefused) {
  CellWorld w = world5(0);
  setSelf(w, 6, CellStatus::COVERED);

  const auto robots = pair2(0, 24);
  const Allocation alloc = GlobalAllocator::solve(w, robots, {});
  EXPECT_TRUE(alloc.refused.empty());
  EXPECT_EQ(alloc.focusFor(0, robots), -1);
}

// ---------------------------------------------------------------------------
// Bounds
// ---------------------------------------------------------------------------

/// An oversized candidate set is refused with a reason rather than truncated.
/// A truncation would be the one failure mode this design cannot survive: two
/// robots whose candidate sets differ by a single cell would cut the list at
/// different places and solve different problems while both believing they had
/// solved the same one.
TEST(GlobalAllocatorBounds, OversizedCandidateSetIsRefusedNotTruncated) {
  CellWorld w = world5(0);
  for (int id = 0; id < w.size(); ++id) setSelf(w, id, CellStatus::EXPLORING);

  GlobalAllocator::Config cfg;
  cfg.max_candidates = 4;              // 25 cells are EXPLORING
  const Allocation alloc = GlobalAllocator::solve(w, pair2(0, 24), cfg);

  EXPECT_NE(alloc.refused.find("too many candidate cells"), std::string::npos);
  EXPECT_TRUE(allAssigned(alloc).empty());
}

TEST(GlobalAllocatorBounds, UnconfiguredWorldIsRefused) {
  CellWorld w;
  const Allocation alloc = GlobalAllocator::solve(w, pair2(0, 1), {});
  EXPECT_FALSE(alloc.refused.empty());
}

/// The polish may only ever shorten a tour. A 2-opt that accepts a worse route
/// on some path would be a silent, run-dependent cost regression.
// ---------------------------------------------------------------------------
// The staleness rule
// ---------------------------------------------------------------------------

/// The threshold is a floor, not an equality, and it must not fire one tick
/// early. Off by one here is a cell written off for the whole team on its
/// third bad tick instead of its fourth, in a rule whose demotions cannot be
/// undone.
TEST(StaleFocus, FiresAtTheThresholdAndNotBefore) {
  EXPECT_FALSE(shouldDemoteStaleFocus(2, 3, CellStatus::EXPLORING));
  EXPECT_TRUE(shouldDemoteStaleFocus(3, 3, CellStatus::EXPLORING));
  EXPECT_TRUE(shouldDemoteStaleFocus(9, 3, CellStatus::EXPLORING));
}

/// A cell the fresh census cleared is not demoted. This is the re-measure's
/// entire purpose: without it the counter would write off a cell that had
/// already been covered while the planner was working somewhere else.
TEST(StaleFocus, ACellTheRecheckClearedIsNotDemoted) {
  EXPECT_FALSE(shouldDemoteStaleFocus(9, 3, CellStatus::COVERED));
  EXPECT_FALSE(shouldDemoteStaleFocus(9, 3, CellStatus::COVERED_BY_OTHERS));
  // UNSEEN too: a cell with no evidence either way has nothing to write off,
  // and demoting it would claim first-hand coverage of ground no one has been
  // near.
  EXPECT_FALSE(shouldDemoteStaleFocus(9, 3, CellStatus::UNSEEN));
}

/// A relayed EXPLORING is demotable. It has to be — a phantom cell learned
/// from a peer pollutes the allocation and the rendezvous minimax exactly as
/// a first-hand one does, and the peer that reported it may be long out of
/// contact.
TEST(StaleFocus, RelayedExploringIsDemotable) {
  EXPECT_TRUE(shouldDemoteStaleFocus(3, 3, CellStatus::EXPLORING_BY_OTHERS));
}

/// k below 1 is refused here as well as clamped in the node. Without this the
/// rule would fire on the first tick every cell is assigned, when no candidate
/// has been tried yet — a demotion of every cell in the map, in order.
TEST(StaleFocus, NonPositiveThresholdNeverFires) {
  EXPECT_FALSE(shouldDemoteStaleFocus(0, 0, CellStatus::EXPLORING));
  EXPECT_FALSE(shouldDemoteStaleFocus(5, 0, CellStatus::EXPLORING));
  EXPECT_FALSE(shouldDemoteStaleFocus(5, -1, CellStatus::EXPLORING));
}

TEST(GlobalAllocatorPolish, NeverLengthensATour) {
  CellWorld w = world5(0);
  for (int id : {2, 6, 10, 14, 18, 22}) setSelf(w, id, CellStatus::EXPLORING);

  GlobalAllocator::Config off;  off.polish_passes = 0;
  GlobalAllocator::Config on;   on.polish_passes  = 4;

  const Allocation a = GlobalAllocator::solve(w, pair2(0, 24), off);
  const Allocation b = GlobalAllocator::solve(w, pair2(0, 24), on);
  for (size_t i = 0; i < a.costs_mm.size(); ++i)
    EXPECT_LE(b.costs_mm[i], a.costs_mm[i]) << "robot " << i;
}

// ---------------------------------------------------------------------------
// R3 / §3.6 — the problem digest
//
// These do not test the allocator's answer; they test that the thing that will
// be used to decide "were these two robots even solving the same problem" can
// actually tell the difference. Every one of them is a channel that
// `shared_hash` is blind to, and each test below is the proof of one such
// blind channel -- which is the whole claim. (This header used to add "and
// that is why robots with equal `shared_hash` disagreed about the peer's focus
// 23% of the time at N=4"; that figure is withdrawn as unreproducible. These
// tests are the evidence, not a campaign statistic.)
// ---------------------------------------------------------------------------

/// Baseline: the same problem digests the same, twice, from two independently
/// built worlds. Without this every test below could pass on a hash that is
/// simply noise.
TEST(AllocHash, EqualProblemsDigestEqual) {
  CellWorld a = world5(0), b = world5(1);
  for (int id : {2, 6, 10}) { setSelf(a, id, CellStatus::EXPLORING);
                              setSelf(b, id, CellStatus::EXPLORING); }
  GlobalAllocator::Config c;
  const Allocation ra = GlobalAllocator::solve(a, pair2(0, 24), c);
  const Allocation rb = GlobalAllocator::solve(b, pair2(0, 24), c);
  EXPECT_NE(ra.alloc_hash, 0u) << "a formed problem must not digest to the "
                                  "no-problem sentinel";
  EXPECT_EQ(ra.alloc_hash, rb.alloc_hash);
  EXPECT_EQ(ra.edge_hash, rb.edge_hash);
}

/// The vehicle set is not part of the world, so `shared_hash` cannot see it at
/// all. Every field of it has to move the digest.
TEST(AllocHash, EveryVehicleFieldMovesTheDigest) {
  CellWorld w = world5(0);
  for (int id : {2, 6, 10}) setSelf(w, id, CellStatus::EXPLORING);
  GlobalAllocator::Config c;

  const unsigned base = GlobalAllocator::solve(w, pair2(0, 24), c).alloc_hash;

  // ...a different cell for one robot
  EXPECT_NE(base, GlobalAllocator::solve(w, pair2(0, 23), c).alloc_hash);

  // ...one robot out of comms
  std::vector<AllocRobot> oc = pair2(0, 24);
  oc[1].in_comms = false;
  EXPECT_NE(base, GlobalAllocator::solve(w, oc, c).alloc_hash);

  // ...one robot finished. The filter drops it, so the SOLVE is a one-vehicle
  // problem either way — but "my peer has stopped" is precisely the
  // disagreement that has to be visible, so the digest must still move.
  std::vector<AllocRobot> fin = pair2(0, 24);
  fin[1].finished = true;
  EXPECT_NE(base, GlobalAllocator::solve(w, fin, c).alloc_hash);

  // ...a different fleet id
  std::vector<AllocRobot> rid = pair2(0, 24);
  rid[1].id = 2;
  EXPECT_NE(base, GlobalAllocator::solve(w, rid, c).alloc_hash);

  // ...a robot present at all
  std::vector<AllocRobot> solo{AllocRobot{0, 0, true, false}};
  EXPECT_NE(base, GlobalAllocator::solve(w, solo, c).alloc_hash);
}

/// The caller's vector order is the caller's, and must not reach a value two
/// processes compare. The solve already sorts; the digest has to as well.
TEST(AllocHash, CallerVectorOrderDoesNotReachTheDigest) {
  CellWorld w = world5(0);
  for (int id : {2, 6, 10}) setSelf(w, id, CellStatus::EXPLORING);
  GlobalAllocator::Config c;

  std::vector<AllocRobot> fwd = pair2(0, 24);
  std::vector<AllocRobot> rev{fwd[1], fwd[0]};
  EXPECT_EQ(GlobalAllocator::solve(w, fwd, c).alloc_hash,
            GlobalAllocator::solve(w, rev, c).alloc_hash);
}

/// The candidate set. This is the one channel `shared_hash` partly covers, and
/// it is still folded in explicitly rather than leaned on: `shared_hash`
/// normalises statuses, so it cannot distinguish a candidate set from a
/// same-sized one over different ground.
TEST(AllocHash, CandidateSetMovesTheDigest) {
  CellWorld a = world5(0), b = world5(0);
  for (int id : {2, 6, 10}) setSelf(a, id, CellStatus::EXPLORING);
  for (int id : {2, 6, 11}) setSelf(b, id, CellStatus::EXPLORING);
  GlobalAllocator::Config c;
  EXPECT_NE(GlobalAllocator::solve(a, pair2(0, 24), c).alloc_hash,
            GlobalAllocator::solve(b, pair2(0, 24), c).alloc_hash);
}

/// §3.6's headline case, and the reason `edge_hash` is logged separately: two
/// worlds with IDENTICAL statuses, identical masks and an identical
/// `sharedHash()` whose traversability differs. Nothing on the wire carries
/// this and nothing ever detected it.
TEST(AllocHash, TraversabilityDifferenceIsVisibleWhenSharedHashIsNot) {
  CellWorld a = world5(0), b = world5(0);
  for (int id : {2, 6, 10}) { setSelf(a, id, CellStatus::EXPLORING);
                              setSelf(b, id, CellStatus::EXPLORING); }
  ASSERT_EQ(a.sharedHash(), b.sharedHash())
      << "fixture: the two worlds must agree on everything the wire carries";

  // Both rebuild their adjacency; only b's local plan map says one pair of
  // neighbouring cells is impassable. This is exactly the real mechanism —
  // `rebuildEdges` is driven by each robot's OWN map and its result is never
  // exchanged.
  a.rebuildEdges(nullptr);
  b.rebuildEdges([](float ax, float ay, float bx, float by) {
    const bool pair = (ax == -20.0f && ay == -20.0f &&
                       bx == -10.0f && by == -20.0f) ||
                      (bx == -20.0f && by == -20.0f &&
                       ax == -10.0f && ay == -20.0f);
    return pair ? 1.0 : 0.0;   // 1.0 = fully blocked; 0.0 = clear
  });
  ASSERT_TRUE(a.edgeEnabled(0, 1));
  ASSERT_FALSE(b.edgeEnabled(0, 1)) << "fixture: b's probe did not block";
  EXPECT_EQ(a.sharedHash(), b.sharedHash())
      << "the wire digest must STILL agree — that is the whole problem";

  GlobalAllocator::Config c;
  const Allocation ra = GlobalAllocator::solve(a, pair2(0, 24), c);
  const Allocation rb = GlobalAllocator::solve(b, pair2(0, 24), c);
  EXPECT_NE(ra.edge_hash, rb.edge_hash);
  EXPECT_NE(ra.alloc_hash, rb.alloc_hash);
}

/// An unconfigured world forms no problem, and its zero must not be readable
/// as agreement.
TEST(AllocHash, UnconfiguredWorldDigestsToTheNoProblemSentinel) {
  CellWorld w;  // never configured
  const Allocation r =
      GlobalAllocator::solve(w, pair2(0, 1), GlobalAllocator::Config{});
  EXPECT_FALSE(r.refused.empty());
  EXPECT_EQ(r.alloc_hash, 0u);
  EXPECT_EQ(r.edge_hash, 0u);
}

/// A refused solve still had a problem, and has to carry its digest: "these
/// two robots refused for different reasons" is only interpretable once you
/// can establish they were refusing the same thing.
TEST(AllocHash, RefusedSolveStillCarriesTheDigest) {
  CellWorld w = world5(0);
  for (int id = 0; id < w.size(); ++id) setSelf(w, id, CellStatus::EXPLORING);
  GlobalAllocator::Config c;
  c.max_candidates = 2;                       // force the truncation refusal
  const Allocation r = GlobalAllocator::solve(w, pair2(0, 24), c);
  ASSERT_FALSE(r.refused.empty()) << "fixture did not trigger a refusal";
  EXPECT_NE(r.alloc_hash, 0u);
}

/// Config does not go into the digest as a struct, and does not stay out as a
/// struct either. The line is whether a field changes the PROBLEM or only the
/// approach taken to it, and both directions have to be asserted — an
/// all-in-or-all-out rule is the thing this test replaced (2026-09-18), on both
/// of the readings the digest is supposed to support:
///
///   "same value, different tours" => a determinism bug in the solver. Only
///   sound if fields that change nothing but the tours are EXCLUDED, or the
///   finding gets restated as "different problems" and lost.
///
///   "different values" => they were never solving the same problem. Only
///   sound if fields that change the problem are INCLUDED, or two genuinely
///   different problems agree on the key.
TEST(AllocHash, SolverConfigSplitsOnWhatChangesTheProblem) {
  CellWorld w = world5(0);
  for (int id : {2, 6, 10, 14}) setSelf(w, id, CellStatus::EXPLORING);

  GlobalAllocator::Config base;
  const unsigned h =
      GlobalAllocator::solve(w, pair2(0, 24), base).alloc_hash;
  ASSERT_NE(h, 0u) << "fixture formed no problem; the rest proves nothing";

  // 2-opt passes: same vehicles, same candidates, same feasible set, same
  // refusal — a different answer to the SAME problem, which is exactly the
  // case the digest exists to make visible. It must not move.
  GlobalAllocator::Config polish0; polish0.polish_passes = 0;
  GlobalAllocator::Config polish4; polish4.polish_passes = 4;
  EXPECT_EQ(GlobalAllocator::solve(w, pair2(0, 24), polish0).alloc_hash,
            GlobalAllocator::solve(w, pair2(0, 24), polish4).alloc_hash)
      << "polish_passes reached the digest. Two robots that differ only in "
         "2-opt effort are solving one problem and answering it differently; "
         "with this in the key that reads as two problems and the solver "
         "disagreement becomes unreportable.";

  // The no-comms mask restricts which cells a disconnected robot may be given
  // at all. It is also the one comparison the flag was ADDED for — masked
  // versus unmasked over otherwise identical inputs — so if it does not move
  // the digest, that comparison is precisely where the digest lies.
  GlobalAllocator::Config masked; masked.comms_mask = true;
  EXPECT_NE(h, GlobalAllocator::solve(w, pair2(0, 24), masked).alloc_hash)
      << "a masked and an unmasked solve over identical inputs digest the "
         "same; the reconnection-value counterfactual would join to itself.";

  // The candidate cap decides refused versus solved. A refused solve carries
  // its digest so that "we refused the same thing" is checkable; if the cap is
  // invisible, a refusal and a success agree on the key and the cap — the
  // entire difference — is the part that cannot be seen.
  GlobalAllocator::Config capped; capped.max_candidates = 2;
  const Allocation r = GlobalAllocator::solve(w, pair2(0, 24), capped);
  ASSERT_FALSE(r.refused.empty()) << "fixture did not trigger the cap refusal";
  EXPECT_NE(h, r.alloc_hash)
      << "a solve refused on max_candidates carries the same digest as one "
         "that succeeded under a laxer cap.";
}
