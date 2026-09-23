/// Tests for the economic reconnection gate. Each suppression is paired with a
/// control that fires on the same fixture with one lever moved, so a gate stuck
/// closed or stuck open is caught. (notes: gate-tests-paired-controls)
/// Moved comments: doc/explo_planner_code_notes.md

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "explo_planner/reconnect_gate.hpp"

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

/// A 5x5 world of 10 m cells over [-25, 25]^2, owned by `self`. Same fixture
/// the allocator tests use, so a cost that shows up in both files means the
/// same thing.
CellWorld world5(int self = 0) {
  CellWorld w;
  const std::string err =
      w.configure(makeCellGrid(-25.0f, 25.0f, -25.0f, 25.0f, 10.0f), cfg0(), self);
  EXPECT_EQ(err, "") << "fixture failed to configure";
  return w;
}

GlobalAllocator::Config alloc0() {
  GlobalAllocator::Config c;
  c.comms_mask     = false;   // ignored by the gate; set here to prove it
  c.polish_passes  = 2;
  c.max_candidates = 256;
  return c;
}

std::vector<AllocRobot> pair2(int cell_a, int cell_b) {
  return {AllocRobot{0, cell_a, true, false}, AllocRobot{1, cell_b, true, false}};
}

/// Production shape: the missing peer is also out-of-comms in the vehicle list,
/// as explo_planner_node builds missing from the vehicles that are not
/// in_comms. (notes: gate-fixture-production-shape)
std::vector<AllocRobot> pair2Apart(int cell_self, int cell_peer) {
  return {AllocRobot{0, cell_self, true, false},
          AllocRobot{1, cell_peer, false, false}};
}

std::vector<MissingPeer> missing1(int id, int cell, bool finished = false) {
  return {MissingPeer{id, cell, finished}};
}

}  // namespace

// ======================================================================
// The knowledge gate, in both failure directions
// ======================================================================

TEST(KnowledgeGate, UnseenCellsAreNotNews) {
  // A world where nothing has been observed says nothing to anybody. If UNSEEN
  // counted, this would read as 25 cells of unshared knowledge and the gate
  // would be vacuously true from the first tick of every run.
  const CellWorld w = world5(0);
  EXPECT_EQ(unsharedCellCount(w, missing1(1, 24)), 0);
}

TEST(KnowledgeGate, AFirstHandStatusIsUnsharedUntilTheMaskSaysOtherwise) {
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::COVERED);
  w.commitSelf(7, CellStatus::EXPLORING);
  EXPECT_EQ(unsharedCellCount(w, missing1(1, 24)), 2);

  w.markKnownBy(6, 1);
  EXPECT_EQ(unsharedCellCount(w, missing1(1, 24)), 1);
  w.markKnownBy(7, 1);
  EXPECT_EQ(unsharedCellCount(w, missing1(1, 24)), 0);
}

/// Direction (a) of the plan's two smokes: the peer provably knows everything,
/// so the gate must NOT dispatch — and must not have priced anything either.
TEST(KnowledgeGate, PeerKnowsEverythingSuppressesWithoutPricing) {
  CellWorld w = world5(0);
  for (int id : {6, 7, 8, 11, 12}) {
    w.commitSelf(id, CellStatus::EXPLORING);
    w.markKnownBy(id, 1);
  }
  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 24), missing1(1, 24), alloc0());

  EXPECT_FALSE(v.dispatch);
  EXPECT_FALSE(v.knowledge);
  EXPECT_EQ(v.unshared_cells, 0);
  EXPECT_EQ(v.refused, "");
  // Not merely "did not dispatch": it declined BEFORE the value gate, so the
  // makespans were never computed. A -1 here is what distinguishes the
  // knowledge suppression from a value suppression that happened to agree.
  EXPECT_EQ(v.c_no_mm, -1);
  EXPECT_EQ(v.c_re_mm, -1);
}

/// Direction (b), and the one that catches OR-forever: a status committed AFTER
/// the masks were synced must make the gate fire again. If `known_by` ever
/// stops resetting on a committed change (§3.1), this is the test that fails
/// while (a) above keeps passing happily.
TEST(KnowledgeGate, AStatusCommittedAfterSyncMakesItNewsAgain) {
  CellWorld w = world5(0);
  for (int id : {6, 7, 8, 11, 12}) {
    w.commitSelf(id, CellStatus::EXPLORING);
    w.markKnownBy(id, 1);
  }
  ASSERT_EQ(unsharedCellCount(w, missing1(1, 24)), 0) << "fixture not synced";

  // One cell moves on. The peer's copy of cell 6 is now stale, and it is the
  // only thing in the world it does not hold.
  ASSERT_TRUE(w.commitSelf(6, CellStatus::COVERED));
  EXPECT_EQ(unsharedCellCount(w, missing1(1, 24)), 1);

  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 24), missing1(1, 24), alloc0());
  EXPECT_TRUE(v.knowledge);
  EXPECT_EQ(v.unshared_cells, 1);
}

TEST(KnowledgeGate, FinishedPeersAreIgnored) {
  CellWorld w = world5(0);
  for (int id : {6, 7, 8}) w.commitSelf(id, CellStatus::EXPLORING);

  // Unfinished: three cells of news.
  EXPECT_EQ(unsharedCellCount(w, missing1(1, 24, false)), 3);
  // Finished: nothing we say changes a plan it will never make.
  EXPECT_EQ(unsharedCellCount(w, missing1(1, 24, true)), 0);

  // And that is a DECIDED no, not a refusal — the distinction the gate has to
  // keep, because a refusal fails open and would send us chasing a robot that
  // has parked.
  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 24), missing1(1, 24, true), alloc0());
  EXPECT_FALSE(v.dispatch);
  EXPECT_EQ(v.refused, "");
}

TEST(KnowledgeGate, AnyMissingPeerLackingTheCellCounts) {
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::EXPLORING);
  w.markKnownBy(6, 1);          // peer 1 is current, peer 2 is not
  const std::vector<MissingPeer> both = {MissingPeer{1, 20, false},
                                         MissingPeer{2, 24, false}};
  EXPECT_EQ(unsharedCellCount(w, both), 1);
  EXPECT_EQ(unsharedCellCount(w, missing1(1, 20)), 0);
}

// ======================================================================
// The value gate
// ======================================================================

namespace {

/// n EXPLORING cells laid in from cell 24 back towards the origin, furthest
/// from robot 0 at cell 0. All are first-hand, so under the no-comms mask the
/// peer may take none of them. (notes: gate-fixture-world-with-work)
CellWorld worldWithWork(int n) {
  CellWorld w = world5(0);
  for (int id = 24; id > 0 && n > 0; --id, --n)
    w.commitSelf(id, CellStatus::EXPLORING);
  return w;
}

}  // namespace

/// The lever is the amount of work; the peer stays at cell 4 so the leg is 40 m
/// in both tests. Peer distance is a confounded lever: moving the peer also
/// changes what work it can reach. (notes: gate-value-lever-is-work)
TEST(ValueGate, EnoughWorkToDivideIsWorthTheTrip) {
  const CellWorld w = worldWithWork(15);
  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 4), missing1(1, 4), alloc0());

  ASSERT_EQ(v.refused, "");
  EXPECT_TRUE(v.knowledge);
  EXPECT_EQ(v.leg_mm, 40000);                  // four 10 m cell steps
  EXPECT_LT(v.c_re_mm, v.c_no_mm);
  EXPECT_TRUE(v.dispatch) << "C_re=" << v.c_re_mm << " C_no=" << v.c_no_mm;
  // Nothing strands under the mask: the deciding robot is in comms with itself
  // and therefore unmasked, so it can absorb every cell the peer cannot take.
  EXPECT_EQ(v.unassigned, 0);
}

TEST(ValueGate, TooLittleWorkIsNotWorthTheSameTrip) {
  const CellWorld w = worldWithWork(5);
  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 4), missing1(1, 4), alloc0());

  ASSERT_EQ(v.refused, "");
  EXPECT_TRUE(v.knowledge) << "the suppression under test must be the VALUE "
                              "gate's, so the knowledge gate has to pass";
  EXPECT_EQ(v.leg_mm, 40000);                  // same trip as above
  EXPECT_GT(v.c_re_mm, v.c_no_mm);
  EXPECT_FALSE(v.dispatch) << "C_re=" << v.c_re_mm << " C_no=" << v.c_no_mm;
}

TEST(ValueGate, KnowledgeAloneDoesNotDispatch) {
  // The two gates are AND: a COVERED cell to share but no work to divide must
  // not dispatch. With both makespans zero, C_re equals the leg at both
  // distances, one-for-one. (notes: gate-knowledge-and-value-are-and)
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::COVERED);

  for (const int peer_cell : {4, 24}) {
    const GateVerdict v = evaluateReconnectGate(
        w, pair2(0, peer_cell), missing1(1, peer_cell), alloc0());
    ASSERT_EQ(v.refused, "") << "peer_cell=" << peer_cell;
    EXPECT_TRUE(v.knowledge);
    EXPECT_EQ(v.c_no_mm, 0) << "peer_cell=" << peer_cell;
    EXPECT_EQ(v.c_re_mm, v.leg_mm) << "peer_cell=" << peer_cell;
    EXPECT_GT(v.leg_mm, 0) << "peer_cell=" << peer_cell;
    EXPECT_FALSE(v.dispatch) << "peer_cell=" << peer_cell;
  }
}

/// The strictness boundary the plan asks for, constructed so it is EXACT rather
/// than approached: with no candidates both makespans are 0, and with the peer
/// in my own cell the leg is 0 too. C_re == C_no == 0. A `<=` dispatches here;
/// the specified `<` does not.
TEST(ValueGate, AnExactTieDoesNotDispatch) {
  CellWorld w = world5(0);
  w.commitSelf(12, CellStatus::COVERED);

  const GateVerdict v =
      evaluateReconnectGate(w, pair2(12, 12), missing1(1, 12), alloc0());
  ASSERT_EQ(v.refused, "");
  ASSERT_TRUE(v.knowledge) << "fixture must clear the knowledge gate to reach "
                              "the comparison this test is about";
  EXPECT_EQ(v.leg_mm, 0);
  EXPECT_EQ(v.c_no_mm, 0);
  EXPECT_EQ(v.c_re_mm, 0);
  EXPECT_FALSE(v.dispatch);
}

TEST(ValueGate, MakespanIsTheSlowestVehicleNotTheTotal) {
  Allocation a;
  a.costs_mm = {10, 90, 0};
  EXPECT_EQ(makespanMm(a), 90);
  // The distinction that matters: a plan dumping everything on one robot has
  // the same TOTAL as a balanced one and twice the completion time.
  Allocation lopsided;
  lopsided.costs_mm = {100, 0};
  Allocation balanced;
  balanced.costs_mm = {50, 50};
  EXPECT_GT(makespanMm(lopsided), makespanMm(balanced));
}

TEST(ValueGate, ConfigCommsMaskIsIgnored) {
  // The caller's comms_mask must not reach either solve: the gate sets it per
  // side, and honouring the caller would collapse the two futures into one and
  // make C_re == C_no + leg for every input — a gate that never fires.
  const CellWorld w = worldWithWork(15);   // a fixture that DOES dispatch, so
                                           // "both agree" is not both refusing
  GlobalAllocator::Config on = alloc0();
  on.comms_mask = true;
  GlobalAllocator::Config off = alloc0();
  off.comms_mask = false;

  const GateVerdict a = evaluateReconnectGate(w, pair2(0, 4), missing1(1, 4), on);
  const GateVerdict b = evaluateReconnectGate(w, pair2(0, 4), missing1(1, 4), off);
  ASSERT_TRUE(b.dispatch);
  EXPECT_EQ(a.c_no_mm, b.c_no_mm);
  EXPECT_EQ(a.c_re_mm, b.c_re_mm);
  EXPECT_EQ(a.dispatch, b.dispatch);
  // And the two futures really are different futures — if the mask were a
  // no-op the gate would be comparing a plan against itself plus a drive.
  EXPECT_NE(b.c_no_mm, b.c_re_mm - b.leg_mm);
}

// ======================================================================
// Failing open — every path that cannot evaluate must still let the run
// reconnect, and must say so
// ======================================================================

TEST(FailOpen, UnconfiguredWorld) {
  const CellWorld w;   // never configured
  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 1), missing1(1, 1), alloc0());
  EXPECT_TRUE(v.dispatch);
  EXPECT_EQ(v.refused, "world-unconfigured");
}

TEST(FailOpen, SelfNotInTheVehicleSet) {
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::EXPLORING);
  // Robot 0 owns this world but is absent from the problem: we cannot price a
  // leg from a position we do not have.
  const std::vector<AllocRobot> without_self = {AllocRobot{1, 24, true, false}};
  const GateVerdict v =
      evaluateReconnectGate(w, without_self, missing1(1, 24), alloc0());
  EXPECT_TRUE(v.dispatch);
  EXPECT_EQ(v.refused, "self-unlocatable");
}

TEST(FailOpen, SelfUnlocatable) {
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::EXPLORING);
  const GateVerdict v =
      evaluateReconnectGate(w, pair2(-1, 24), missing1(1, 24), alloc0());
  EXPECT_TRUE(v.dispatch);
  EXPECT_EQ(v.refused, "self-unlocatable");
}

TEST(FailOpen, PeerPositionUnknown) {
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::EXPLORING);
  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 24), missing1(1, -1), alloc0());
  EXPECT_TRUE(v.dispatch);
  EXPECT_EQ(v.refused, "peer-position-unknown");
  // The knowledge gate ran first and passed; it is the pricing that could not.
  EXPECT_TRUE(v.knowledge);
}

TEST(FailOpen, AllocatorRefusal) {
  CellWorld w = world5(0);
  for (int id : {20, 21, 22, 23, 24}) w.commitSelf(id, CellStatus::EXPLORING);
  GlobalAllocator::Config tiny = alloc0();
  tiny.max_candidates = 2;      // 5 candidates: the allocator refuses

  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 1), missing1(1, 1), tiny);
  EXPECT_TRUE(v.dispatch);
  EXPECT_EQ(v.refused.rfind("no-comms-solve:", 0), 0u) << v.refused;
}

TEST(FailOpen, PeerIdOutsideTheMaskWidth) {
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::EXPLORING);
  const GateVerdict v =
      evaluateReconnectGate(w, pair2(0, 24), missing1(32, 24), alloc0());
  EXPECT_TRUE(v.dispatch);
  EXPECT_EQ(v.refused, "peer-id-out-of-mask-range");
}

TEST(FailOpen, NoMissingPeerIdentifiedIsAnInabilityToEvaluate) {
  // An empty missing set is a normal outcome, not an upstream bug: the trigger
  // and this list come from different topics with different TTLs. It fails open
  // with a named refusal, distinct from the clean suppression.
  // (notes: gate-empty-missing-fails-open-2)
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::EXPLORING);
  const GateVerdict v = evaluateReconnectGate(w, pair2(0, 24), {}, alloc0());
  EXPECT_TRUE(v.dispatch);
  EXPECT_EQ(v.refused, "no-missing-peer-identified");
}

TEST(KnowledgeGate, EveryMissingPeerFinishedIsStillACleanSuppression) {
  // Every missing peer finished is a computed no that keeps an empty refused.
  // The empty-missing fail-open sits directly upstream and must not swallow
  // this path. (notes: gate-finished-peers-clean-no)
  CellWorld w = world5(0);
  w.commitSelf(6, CellStatus::EXPLORING);
  std::vector<MissingPeer> done = missing1(1, 24);
  done[0].finished = true;
  const GateVerdict v = evaluateReconnectGate(w, pair2(0, 24), done, alloc0());
  EXPECT_FALSE(v.dispatch);
  EXPECT_EQ(v.refused, "");
  EXPECT_EQ(v.unshared_cells, 0);
}

// ======================================================================
// Production shape: the missing peer is out of comms in the vehicle list.
// ======================================================================

TEST(ProductionShape, DispatchIsReachableWhenThePeerIsOutOfComms) {
  // Asserts dispatch is reachable on the shape the node builds (peer out of
  // comms). Both terms must favour it: the peer adjacent at cell 1 for a cheap
  // leg, and the backlog spread across the grid.
  // (notes: gate-dispatch-reachable-apart)
  CellWorld w = world5(0);
  for (int id = 5; id <= 24; ++id) w.commitSelf(id, CellStatus::EXPLORING);
  const GateVerdict v = evaluateReconnectGate(
      w, pair2Apart(0, 1), missing1(1, 1), alloc0());
  EXPECT_EQ(v.refused, "") << "the gate refused rather than deciding";
  EXPECT_TRUE(v.knowledge) << "peer holds no bit for any committed cell";
  EXPECT_GT(v.unshared_cells, 0);
  // The decision itself, and the arithmetic behind it, so a failure says which.
  EXPECT_GT(v.c_no_mm, 0);
  EXPECT_GT(v.c_re_mm, 0);
  EXPECT_GE(v.leg_mm, 0);
  EXPECT_LT(v.c_re_mm, v.c_no_mm) << "c_re " << v.c_re_mm << " c_no "
                                  << v.c_no_mm << " leg " << v.leg_mm;
  EXPECT_TRUE(v.dispatch);
}

TEST(ProductionShape, TheCommsMaskIsWhatMakesStayingApartExpensive) {
  // The gate sets in_comms false on every live missing peer itself, so the
  // caller's flag cannot move C_no. Instead, masked c_no_mm must exceed c_re_mm
  // minus leg_mm, the makespan with the priced peer back.
  // (notes: gate-comms-mask-mechanism)
  CellWorld w = world5(0);
  for (int id = 5; id <= 24; ++id) w.commitSelf(id, CellStatus::EXPLORING);
  const GateVerdict v = evaluateReconnectGate(
      w, pair2Apart(0, 1), missing1(1, 1), alloc0());
  ASSERT_EQ(v.refused, "");
  ASSERT_GT(v.unshared_cells, 0) << "fixture shares nothing to bar";
  EXPECT_GT(v.c_no_mm, v.c_re_mm - v.leg_mm)
      << "masked makespan " << v.c_no_mm << " vs unmasked "
      << (v.c_re_mm - v.leg_mm);
}

// ======================================================================
// Cross-perspective: the gate is arithmetic over a shared world, so two
// robots holding the same world must price the same manoeuvre identically.
// ======================================================================

TEST(GateCrossPerspective, ConvergedWorldsPriceTheSameFutures) {
  CellWorld a = world5(0);
  CellWorld b = world5(1);
  for (int id : {6, 7, 8}) a.commitSelf(id, CellStatus::EXPLORING);
  for (int id : {16, 17}) b.commitSelf(id, CellStatus::EXPLORING);
  const auto a_wire = a.toWire();
  const auto b_wire = b.toWire();
  b.mergeWire(0, a_wire);
  a.mergeWire(1, b_wire);
  ASSERT_EQ(a.sharedHash(), b.sharedHash());

  const auto robots = pair2(0, 24);
  // Each robot asks about the other, from its own end of the same problem.
  const GateVerdict from_a =
      evaluateReconnectGate(a, robots, missing1(1, 24), alloc0());
  const GateVerdict from_b =
      evaluateReconnectGate(b, robots, missing1(0, 0), alloc0());

  ASSERT_EQ(from_a.refused, "");
  ASSERT_EQ(from_b.refused, "");
  // The assume-comms future is a property of the world, not of who is asking.
  EXPECT_EQ(from_a.c_re_mm - from_a.leg_mm, from_b.c_re_mm - from_b.leg_mm);
}
