#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "explo_planner/cell_world.hpp"
#include "explo_planner/pursuit_predictor.hpp"

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

/// A 5x5 world of 10 m cells over [-25, 25]^2. Row-major ids, so the bottom
/// row is 0..4 with centres (-20,-20), (-10,-20), (0,-20), (10,-20), (20,-20)
/// — a straight line of 10 m hops, which is what makes every number below
/// hand-computable.
///
/// No cell edges are set, so CellWorld::distance reports unreachable and
/// GlobalAllocator::costMm falls back to centroid distance. That fallback is
/// the documented behaviour (global_allocator.hpp) and it is what the predictor
/// inherits; a test that quietly depended on a roadmap would be testing a
/// configuration the node does not always have.
CellWorld world5(int self = 0) {
  CellWorld w;
  const std::string err =
      w.configure(makeCellGrid(-25.0f, 25.0f, -25.0f, 25.0f, 10.0f),
                  cfg0(), self);
  EXPECT_EQ(err, "") << "fixture failed to configure";
  return w;
}

/// A chain whose arithmetic is exact: 1 m/s over 10 m legs with no dwell is a
/// 10 s leg, and a 5 s step makes P(go) exactly 1/2. Off-route drain is
/// switched off outright (half-life 0, which the Config defines as "the hazard
/// is off") so the distribution is a clean binomial and any deviation is the
/// model, not the hazard. A merely LARGE half-life would not do: at 1e9 s each
/// step still loses 3e-9 of the mass, which is a thousand times the tolerance
/// these tests need in order to be checking the binomial at all.
PursuitPredictor::Config exact() {
  PursuitPredictor::Config c;
  c.peer_speed_mps = 1.0;
  c.my_speed_mps   = 1.0;
  c.dwell_sec      = 0.0;
  c.step_sec       = 5.0;
  c.offroute_half_life_sec = 0.0;   // off, exactly
  c.max_horizon_sec = 1e6;
  c.min_probability = 0.0;
  return c;
}

PursuitPredictor::PeerTrack atCell(const CellWorld& w, int cell,
                                   std::vector<int> tour, double age = 0.0) {
  PursuitPredictor::PeerTrack p;
  p.tour = std::move(tour);
  float cx = 0.0f, cy = 0.0f;
  w.grid().centre(cell, cx, cy);
  p.x = cx;
  p.y = cy;
  p.have_position = true;
  p.age_sec = age;
  return p;
}

double sum(const std::vector<double>& v) {
  double s = 0.0;
  for (double x : v) s += x;
  return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// The chain: expansion and conservation
// ---------------------------------------------------------------------------

/// Hand-computed chain expansion. With P(go) = 1/2 and no drain, the mass after
/// n steps is the binomial row C(n,k)/2^n for as long as the tail is out of
/// reach. This is the whole model's arithmetic, checked against a number a
/// reader can verify without running anything.
TEST(PursuitPredictorChain, ExpandsAsAHandComputedBinomial) {
  const CellWorld w = world5();
  const std::vector<int> tour{0, 1, 2, 3, 4};
  const auto c = exact();

  std::vector<double> p(tour.size(), 0.0);
  p[0] = 1.0;
  double off = 0.0;

  PursuitPredictor::step(w, tour, p, off, c);
  EXPECT_NEAR(p[0], 0.5, 1e-12);
  EXPECT_NEAR(p[1], 0.5, 1e-12);

  PursuitPredictor::step(w, tour, p, off, c);
  EXPECT_NEAR(p[0], 0.25, 1e-12);
  EXPECT_NEAR(p[1], 0.50, 1e-12);
  EXPECT_NEAR(p[2], 0.25, 1e-12);

  PursuitPredictor::step(w, tour, p, off, c);
  EXPECT_NEAR(p[0], 1.0 / 8.0, 1e-12);
  EXPECT_NEAR(p[1], 3.0 / 8.0, 1e-12);
  EXPECT_NEAR(p[2], 3.0 / 8.0, 1e-12);
  EXPECT_NEAR(p[3], 1.0 / 8.0, 1e-12);
  EXPECT_NEAR(p[4], 0.0, 1e-12);
}

/// THE INVARIANT the whole model rests on, checked AT EVERY STEP rather than
/// once at the end: a conservation error and a compensating one are
/// indistinguishable in a final sum, and the second is the kind a later edit
/// introduces.
TEST(PursuitPredictorChain, ProbabilityIsConservedAtEveryStep) {
  const CellWorld w = world5();
  const std::vector<int> tour{0, 1, 2, 3, 4};
  auto c = exact();
  c.offroute_half_life_sec = 37.0;   // a drain that is neither off nor total

  std::vector<double> p(tour.size(), 0.0);
  p[2] = 1.0;                        // start mid-tour, so both directions run
  double off = 0.0;

  for (int k = 0; k < 40; ++k) {
    PursuitPredictor::step(w, tour, p, off, c);
    EXPECT_NEAR(sum(p) + off, 1.0, 1e-12) << "step " << k;
    for (double x : p) EXPECT_GE(x, 0.0) << "step " << k;
  }
  // And it really did drain: a conservation test passes trivially if nothing
  // moves, so assert the hazard was live.
  EXPECT_GT(off, 0.5);
}

/// The O transition is a hazard with the half-life it says it has. At
/// step == half-life, exactly half the mass is gone after one step — the one
/// point on the curve that can be checked without reproducing the formula.
TEST(PursuitPredictorChain, OffRouteHalfLifeIsExact) {
  const CellWorld w = world5();
  const std::vector<int> tour{0, 1};
  auto c = exact();
  c.offroute_half_life_sec = c.step_sec;

  std::vector<double> p{1.0, 0.0};
  double off = 0.0;
  PursuitPredictor::step(w, tour, p, off, c);

  EXPECT_NEAR(off, 0.5, 1e-12);
  EXPECT_NEAR(sum(p), 0.5, 1e-12);
}

/// The tail absorbs rather than advancing: a peer at the end of the tour we
/// hold has nowhere on the RECORD to go, and inventing a next cell would be
/// inventing a prediction. Mass there drains only through O.
TEST(PursuitPredictorChain, TailNodeDoesNotAdvance) {
  const CellWorld w = world5();
  const std::vector<int> tour{0, 1};
  auto c = exact();

  std::vector<double> p{0.0, 1.0};
  double off = 0.0;
  for (int k = 0; k < 10; ++k) PursuitPredictor::step(w, tour, p, off, c);

  EXPECT_NEAR(p[1], 1.0, 1e-9);   // no drain configured, so it all stays
  EXPECT_NEAR(p[0], 0.0, 1e-12);  // and nothing flows backwards
  EXPECT_NEAR(off, 0.0, 1e-9);
}

/// A mismatched distribution is a no-op, not a crash or a silent resize. The
/// caller that gets this wrong is a caller whose indices no longer mean what
/// the tour's do, and quietly fixing the length would hide that.
TEST(PursuitPredictorChain, MismatchedStateIsANoOp) {
  const CellWorld w = world5();
  const std::vector<int> tour{0, 1, 2};
  const auto c = exact();

  std::vector<double> p{1.0, 0.0};   // too short
  double off = 0.0;
  PursuitPredictor::step(w, tour, p, off, c);
  ASSERT_EQ(p.size(), 2u);
  EXPECT_NEAR(p[0], 1.0, 1e-12);
  EXPECT_NEAR(off, 0.0, 1e-12);
}

// ---------------------------------------------------------------------------
// The anchor
// ---------------------------------------------------------------------------

TEST(PursuitPredictorAnchor, PicksTheNearestTourNode) {
  const CellWorld w = world5();
  const std::vector<int> tour{0, 1, 2, 3, 4};
  float cx = 0.0f, cy = 0.0f;
  w.grid().centre(2, cx, cy);
  EXPECT_EQ(PursuitPredictor::anchorIndex(w, tour, cx, cy), 2);

  // Slightly off-centre still resolves to the same node.
  EXPECT_EQ(PursuitPredictor::anchorIndex(w, tour, cx + 3.0, cy - 2.0), 2);
}

/// A tour that revisits a cell anchors at the EARLIEST occurrence. Anchoring at
/// the later one would skip every cell in between and predict the peer ahead of
/// itself — the exact failure the intercept exists to avoid, arrived at from
/// the other direction.
TEST(PursuitPredictorAnchor, RevisitedCellAnchorsAtTheEarliestVisit) {
  const CellWorld w = world5();
  const std::vector<int> tour{0, 1, 2, 1, 0};
  float cx = 0.0f, cy = 0.0f;
  w.grid().centre(0, cx, cy);
  EXPECT_EQ(PursuitPredictor::anchorIndex(w, tour, cx, cy), 0);
}

TEST(PursuitPredictorAnchor, EmptyOrAllInvalidHasNoAnchor) {
  const CellWorld w = world5();
  EXPECT_EQ(PursuitPredictor::anchorIndex(w, {}, 0.0, 0.0), -1);
  EXPECT_EQ(PursuitPredictor::anchorIndex(w, {999, 1000}, 0.0, 0.0), -1);
}

// ---------------------------------------------------------------------------
// The intercept
// ---------------------------------------------------------------------------

/// THE GATE TEST (plan §4, P6): argmax intercept against a hand-computed case.
///
/// The peer is at cell 0 driving 0->4; I am at cell 4. My drive to each cell is
/// 40/30/20/10/0 m, so at 1 m/s I would arrive after 8/6/4/2/0 steps of 5 s.
/// Reading the binomial rows at those steps:
///
///     cell 0 @ 8 steps: 2^-8            = 0.0039
///     cell 1 @ 6 steps: 0.09375
///     cell 2 @ 4 steps: 6/16            = 0.375   <-- the argmax
///     cell 3 @ 2 steps: 0
///     cell 4 @ 0 steps: 0
///
/// The answer is cell 2: not where the peer WAS (cell 0, which is what the
/// legacy trail would chase) and not where I already am. That gap is the whole
/// point of §3.7, and this test is what would fail if the horizon reverted to
/// the record age alone.
TEST(PursuitPredictorIntercept, MeetsInTheMiddleAgainstAHandComputedChain) {
  const CellWorld w = world5();
  const auto c = exact();
  const auto peer = atCell(w, 0, {0, 1, 2, 3, 4});

  const PursuitTarget t = PursuitPredictor::predict(w, peer, /*my_cell=*/4, c);

  ASSERT_TRUE(t.valid()) << t.refused;
  EXPECT_EQ(t.cell, 2);
  EXPECT_EQ(t.win_index, 2);
  EXPECT_EQ(t.anchor_index, 0);
  EXPECT_NEAR(t.p, 0.375, 1e-12);
  EXPECT_NEAR(t.p_on_route, 1.0, 1e-12);
  EXPECT_EQ(t.my_travel_ms, 20000);
  EXPECT_EQ(t.horizon_ms, 20000);
  EXPECT_EQ(t.candidates, 5);

  // And it is a real cell centre, not a cell id echoed into a coordinate.
  float cx = 0.0f, cy = 0.0f;
  w.grid().centre(2, cx, cy);
  EXPECT_FLOAT_EQ(t.x, cx);
  EXPECT_FLOAT_EQ(t.y, cy);
}

/// The counterfactual for the test above: standing next to a peer whose record
/// is fresh, the intercept IS where it was last seen. "Intercept ahead" is a
/// consequence of travel time, not a bias baked into the model, and a version
/// that always aimed forward would fail here.
TEST(PursuitPredictorIntercept, AFreshRecordNextDoorInterceptsInPlace) {
  const CellWorld w = world5();
  const auto c = exact();
  const auto peer = atCell(w, 0, {0, 1, 2, 3, 4});

  const PursuitTarget t = PursuitPredictor::predict(w, peer, /*my_cell=*/0, c);

  ASSERT_TRUE(t.valid()) << t.refused;
  EXPECT_EQ(t.cell, 0);
  EXPECT_EQ(t.win_index, 0);
  EXPECT_NEAR(t.p, 1.0, 1e-12);
  EXPECT_EQ(t.horizon_ms, 0);
}

/// Staleness is inside the horizon, so a record that has aged moves the
/// intercept forward on its own, with no change to where either robot is. Both
/// robots stand on cell 0 here; the only difference between the two calls is
/// how long ago the tour was heard.
///
/// With the hazard off the tour's last node absorbs, so the aged intercept runs
/// all the way to the tail (30 s of staleness is six steps of a chain whose
/// P(go) is 1/2, and the mass has nowhere else to end up). That is the model
/// being honest rather than a defect: a tour heard long enough ago and never
/// contradicted really does say "it finished". In a run the half-life drains
/// that same mass into O and `min_probability` returns the chase to the trail —
/// which is what AStaleRecordRefusesButStillReports covers.
TEST(PursuitPredictorIntercept, AgeAloneMovesTheInterceptForward) {
  const CellWorld w = world5();
  const auto c = exact();

  const PursuitTarget fresh =
      PursuitPredictor::predict(w, atCell(w, 0, {0, 1, 2, 3, 4}, 0.0), 0, c);
  const PursuitTarget aged =
      PursuitPredictor::predict(w, atCell(w, 0, {0, 1, 2, 3, 4}, 30.0), 0, c);

  ASSERT_TRUE(fresh.valid()) << fresh.refused;
  ASSERT_TRUE(aged.valid()) << aged.refused;
  EXPECT_EQ(fresh.win_index, 0);          // fresh: it is where it was
  EXPECT_GT(aged.win_index, fresh.win_index);
  EXPECT_GT(aged.my_travel_ms, fresh.my_travel_ms);

  // The horizon claim, stated as the relation it is — age plus MY drive to
  // whichever cell won. A literal here would instead pin the argmax, and the
  // argmax moving is the thing this test asserts.
  EXPECT_EQ(fresh.horizon_ms, fresh.my_travel_ms);
  EXPECT_EQ(aged.horizon_ms, aged.my_travel_ms + 30000);
}

/// The tie-break is total and its FIRST rule is "sooner arrival". Both
/// surviving candidates here score exactly zero — the mass cannot reach either
/// in the steps available — so the only thing separating them is the rule.
/// The duplicated tail cell then exercises the second rule (lower tour index)
/// at an identical horizon.
TEST(PursuitPredictorIntercept, TiesBreakOnSoonerThenEarlierIndex) {
  const CellWorld w = world5();
  auto c = exact();
  c.my_speed_mps    = 2.0;   // halves every drive: 20/15/10/5/0 s
  c.max_horizon_sec = 8.0;   // keeps only cells 3 and 4
  c.min_probability = 0.0;

  const auto peer = atCell(w, 0, {0, 1, 2, 3, 4, 4});
  const PursuitTarget t = PursuitPredictor::predict(w, peer, /*my_cell=*/4, c);

  ASSERT_TRUE(t.valid()) << t.refused;
  EXPECT_EQ(t.rejected_horizon, 3);
  EXPECT_EQ(t.candidates, 3);
  EXPECT_NEAR(t.p, 0.0, 1e-12);
  EXPECT_EQ(t.cell, 4);        // 0 ms beats cell 3's 5000 ms
  EXPECT_EQ(t.win_index, 4);   // the earlier of the two cell-4 entries
  EXPECT_EQ(t.horizon_ms, 0);
}

// ---------------------------------------------------------------------------
// Degradation: the floor is the legacy trail, and every refusal names itself
// ---------------------------------------------------------------------------

/// §3.7's hard requirement: with no tour on record, pursuit is exactly what it
/// was before this file existed. The refusal must be a NAMED one — "the model
/// had nothing to say" and "the model was never asked" are different runs, and
/// one empty answer for both would make an arm that never predicted look like
/// one that predicted badly.
TEST(PursuitPredictorDegradation, NoTourRefusesAndSaysWhy) {
  const CellWorld w = world5();
  PursuitPredictor::PeerTrack peer;   // no tour, no position
  const PursuitTarget t = PursuitPredictor::predict(w, peer, 0, exact());

  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.cell, -1);
  EXPECT_EQ(t.refused, "no tour on record");
}

/// A tour whose every cell is off this robot's grid is the peer-on-a-different-
/// map case. It must refuse rather than clamp ids into range.
TEST(PursuitPredictorDegradation, AllInvalidTourRefuses) {
  const CellWorld w = world5();
  PursuitPredictor::PeerTrack peer;
  peer.tour = {999, 1000};
  const PursuitTarget t = PursuitPredictor::predict(w, peer, 0, exact());

  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.refused, "no valid cell on the peer's tour");
}

/// One bad id does not poison the rest: it is counted and skipped, because a
/// truncated tour would silently shorten the prediction horizon.
TEST(PursuitPredictorDegradation, OneInvalidCellIsCountedNotFatal) {
  const CellWorld w = world5();
  const auto peer = atCell(w, 0, {0, 999, 1, 2});
  const PursuitTarget t = PursuitPredictor::predict(w, peer, 4, exact());

  ASSERT_TRUE(t.valid()) << t.refused;
  EXPECT_EQ(t.rejected_invalid, 1);
  EXPECT_EQ(t.candidates, 3);
}

/// A record old enough that the mass has drained into O produces a refusal, NOT
/// a confident-looking intercept on a tour the peer abandoned. The reported
/// numbers survive the refusal: a near miss at 0.09 and a hopeless 0.001 are
/// different findings about the model and a caller needs to be able to tell.
TEST(PursuitPredictorDegradation, AStaleRecordRefusesButStillReports) {
  const CellWorld w = world5();
  auto c = exact();
  c.offroute_half_life_sec = 180.0;
  c.max_horizon_sec        = 3000.0;
  c.min_probability        = 0.10;

  const auto peer = atCell(w, 0, {0, 1, 2, 3, 4}, /*age=*/900.0);
  const PursuitTarget t = PursuitPredictor::predict(w, peer, 4, c);

  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.cell, -1);
  EXPECT_EQ(t.refused, "best intercept probability below min_probability");
  // 900 s is five half-lives, so at most 2^-5 of the mass is still on the tour.
  EXPECT_LE(t.p_on_route, 0.03125 + 1e-9);
  EXPECT_GT(t.p, 0.0);          // it did score something
  EXPECT_LT(t.p, 0.10);         // and the threshold is what rejected it
  EXPECT_GE(t.win_index, 0);    // and the near miss is identifiable
}

/// The same staleness, with the threshold at zero, DOES predict. Without this
/// the test above could be passing because the fixture is broken rather than
/// because the rule fired.
TEST(PursuitPredictorDegradation, TheThresholdIsWhatRefusesTheStaleRecord) {
  const CellWorld w = world5();
  auto c = exact();
  c.offroute_half_life_sec = 180.0;
  c.max_horizon_sec        = 3000.0;
  c.min_probability        = 0.0;

  const auto peer = atCell(w, 0, {0, 1, 2, 3, 4}, /*age=*/900.0);
  const PursuitTarget t = PursuitPredictor::predict(w, peer, 4, c);

  EXPECT_TRUE(t.valid()) << t.refused;
}

/// A record older than the prediction horizon rejects every candidate. The
/// parameter bounds how far ahead the model may claim anything, and a stale
/// record is already that far ahead.
TEST(PursuitPredictorDegradation, AnAgeBeyondTheHorizonRejectsEverything) {
  const CellWorld w = world5();
  auto c = exact();
  c.max_horizon_sec = 100.0;

  const auto peer = atCell(w, 0, {0, 1, 2, 3, 4}, /*age=*/500.0);
  const PursuitTarget t = PursuitPredictor::predict(w, peer, 4, c);

  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.rejected_horizon, 5);
  EXPECT_EQ(t.refused, "every tour cell is off the grid or beyond the horizon");
}

TEST(PursuitPredictorDegradation, OffGridSelfRefuses) {
  const CellWorld w = world5();
  const auto peer = atCell(w, 0, {0, 1, 2});
  const PursuitTarget t = PursuitPredictor::predict(w, peer, -1, exact());

  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.refused, "own cell is off the grid");
}

/// A zero speed is a disabled predictor, not an infinite travel time. The
/// division would produce inf, every candidate would be rejected on horizon,
/// and the refusal would name the wrong cause.
TEST(PursuitPredictorDegradation, ZeroSpeedIsADisabledPredictor) {
  const CellWorld w = world5();
  const auto peer = atCell(w, 0, {0, 1, 2});

  auto c = exact();
  c.my_speed_mps = 0.0;
  EXPECT_EQ(PursuitPredictor::predict(w, peer, 4, c).refused,
            "predictor is disabled by configuration");

  c = exact();
  c.peer_speed_mps = 0.0;
  EXPECT_EQ(PursuitPredictor::predict(w, peer, 4, c).refused,
            "predictor is disabled by configuration");
}

/// A negative age is a clock that ran backwards, not a very fresh record.
/// Believing it would propagate the chain a negative number of steps.
TEST(PursuitPredictorDegradation, NegativeAgeRefuses) {
  const CellWorld w = world5();
  auto peer = atCell(w, 0, {0, 1, 2});
  peer.age_sec = -1.0;

  const PursuitTarget t = PursuitPredictor::predict(w, peer, 4, exact());
  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.refused, "record age is not a usable interval");
}

TEST(PursuitPredictorDegradation, UnconfiguredWorldRefuses) {
  CellWorld w;   // never configured
  PursuitPredictor::PeerTrack peer;
  peer.tour = {0, 1};
  const PursuitTarget t = PursuitPredictor::predict(w, peer, 0, exact());

  EXPECT_FALSE(t.valid());
  EXPECT_EQ(t.refused, "cell world is not configured");
}

/// Without a position the chain starts at the tour head — the peer's focus
/// cell, which is where it told us it was going next. This is the relay case
/// and the pre-first-position case, and it must predict rather than refuse.
TEST(PursuitPredictorDegradation, NoPositionAnchorsAtTheTourHead) {
  const CellWorld w = world5();
  PursuitPredictor::PeerTrack peer;
  peer.tour = {2, 3, 4};
  peer.have_position = false;

  const PursuitTarget t = PursuitPredictor::predict(w, peer, 2, exact());
  ASSERT_TRUE(t.valid()) << t.refused;
  EXPECT_EQ(t.anchor_index, 0);
  EXPECT_EQ(t.cell, 2);
}

// ---------------------------------------------------------------------------
// The dwell parameter is load-bearing
// ---------------------------------------------------------------------------

/// A peer that works its cells advances more slowly than one that drives
/// through them, and the intercept has to move back to meet it. Zero dwell is
/// the setting that predicts the peer far ahead of where it is (see the
/// Config comment), so the two must give different answers or the parameter is
/// decorative.
TEST(PursuitPredictorDwell, WorkingTheCellsPullsTheInterceptBack) {
  const CellWorld w = world5();
  const auto peer = atCell(w, 0, {0, 1, 2, 3, 4});

  auto fast = exact();
  auto slow = exact();
  slow.dwell_sec = 60.0;

  const PursuitTarget a = PursuitPredictor::predict(w, peer, 4, fast);
  const PursuitTarget b = PursuitPredictor::predict(w, peer, 4, slow);

  ASSERT_TRUE(a.valid()) << a.refused;
  ASSERT_TRUE(b.valid()) << b.refused;
  EXPECT_LT(b.win_index, a.win_index);
}
