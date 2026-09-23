// Moved comments: doc/explo_planner_code_notes.md
#include <gtest/gtest.h>
#include "explo_planner/cell_world.hpp"
#include "explo_planner/fleet_identity.hpp"
#include "explo_planner/map_cache.hpp"
#include <scovox_msgs/msg/scovox_map.hpp>
#include <cmath>
#include <limits>
#include <vector>

using namespace explo_planner;

namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

CellWorld::Config goodConfig() {
  CellWorld::Config c;
  c.covered_max_unknown   = 0.15;
  c.exploring_min_unknown = 0.35;
  c.covered_max_frontier_frac = 0.90;
  c.min_observed_columns  = 4;
  return c;
}

/// A 3x3 world of 10 m cells over [-15, 15]^2, self id 0.
CellWorld makeWorld(const CellWorld::Config& cfg = goodConfig()) {
  CellWorld w;
  const std::string err =
      w.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f), cfg, 0);
  EXPECT_EQ(err, "") << "fixture failed to configure";
  return w;
}

/// An observation of `total` columns with `observed` of them seen.
CellWorld::CellObservation obs(int total, int observed, int frontier = 0) {
  CellWorld::CellObservation o;
  o.total_columns    = total;
  o.observed_columns = observed;
  o.frontier_voxels  = frontier;
  o.observed_voxels  = observed;
  return o;
}

// --- map fixtures ----------------------------------------------------------

// Mirrors test_map_cache: `occupied` strongly occupied, `free` strongly free,
// ingested through the ScovoxMap path.
MapCache makeMap(const std::vector<Eigen::Vector3f>& occupied,
                 const std::vector<Eigen::Vector3f>& free = {},
                 float resolution = 0.1f) {
  scovox_msgs::msg::ScovoxMap msg;
  msg.resolution = resolution;
  auto add = [&msg](const Eigen::Vector3f& p, float a_occ, float a_free) {
    scovox_msgs::msg::ScovoxVoxel v;
    v.position.x = p.x();
    v.position.y = p.y();
    v.position.z = p.z();
    v.a_occ = a_occ;
    v.a_free = a_free;
    msg.voxels.push_back(v);
  };
  for (const auto& p : occupied) add(p, 10.0f, 1.0f);
  for (const auto& p : free) add(p, 1.0f, 10.0f);
  MapCache map(static_cast<double>(resolution));
  map.updateFromScovoxMap(msg);
  return map;
}

/// A 2x2 grid of 1 m cells over [0, 2]^2 at 0.1 m resolution: 10x10 = 100
/// voxel columns per cell, so an unknown fraction has 1% granularity.
CellGrid mapTestGrid() { return makeCellGrid(0.0f, 2.0f, 0.0f, 2.0f, 1.0f); }

/// Voxel centre of the k-th column inside a 1 m cell whose low edge is `lo`.
float colCentre(float lo, int k) { return lo + 0.05f + 0.1f * k; }

// --- exactly-representable fixture -----------------------------------------
//
// Tests compared against MapCache::unknownColumnFraction use a power-of-two
// resolution: 0.1f is inexact, so a cell's equivalent box picks up a
// neighbour's column under that function's inclusive bounds.
// (notes: cellworld-test-exact-resolution)
constexpr float kExactRes = 0.125f;   // 8 columns per 1 m cell
constexpr int   kExactCols = 8;
/// Voxel centre of the k-th column of a 1 m cell at 0.125 m resolution.
float exactCentre(float lo, int k) { return lo + 0.0625f + 0.125f * k; }
/// Inclusive high bound of a 1 m cell for unknownColumnFraction: the LAST
/// column's low corner, since that function takes inclusive coord bounds.
float exactHigh(float lo) { return lo + 0.125f * (kExactCols - 1); }

} // namespace

// ===========================================================================
// Geometry
// ===========================================================================

TEST(CellGridBuild, CoversTheRoiAtTheRequestedSize) {
  const CellGrid g = makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f);
  EXPECT_EQ(g.nx, 3);
  EXPECT_EQ(g.ny, 3);
  EXPECT_EQ(g.size(), 9);
  EXPECT_FLOAT_EQ(g.maxX(), 15.0f);
  EXPECT_FLOAT_EQ(g.maxY(), 15.0f);
}

TEST(CellGridBuild, RoundsCellCountUpSoNoGroundIsUncoverable) {
  // 25 m at 10 m cells is 2.5 cells; rounding down would leave a 5 m strip
  // that no cell names, permanently UNSEEN and never completable.
  const CellGrid g = makeCellGrid(0.0f, 25.0f, 0.0f, 25.0f, 10.0f);
  EXPECT_EQ(g.nx, 3);
  EXPECT_EQ(g.ny, 3);
  EXPECT_GE(g.maxX(), 25.0f);
}

TEST(CellGridBuild, RefusesDegenerateAndNonFiniteBounds) {
  EXPECT_EQ(makeCellGrid(0.0f, 0.0f, 0.0f, 10.0f, 1.0f).size(), 0);
  EXPECT_EQ(makeCellGrid(10.0f, 0.0f, 0.0f, 10.0f, 1.0f).size(), 0);
  EXPECT_EQ(makeCellGrid(0.0f, 10.0f, 0.0f, 0.0f, 1.0f).size(), 0);
  EXPECT_EQ(makeCellGrid(kNaN, 10.0f, 0.0f, 10.0f, 1.0f).size(), 0);
  EXPECT_EQ(makeCellGrid(0.0f, kInf, 0.0f, 10.0f, 1.0f).size(), 0);
  EXPECT_EQ(makeCellGrid(0.0f, 10.0f, 0.0f, 10.0f, kNaN).size(), 0);
}

TEST(CellGridBuild, RefusesNonPositiveCellSize) {
  EXPECT_EQ(makeCellGrid(0.0f, 10.0f, 0.0f, 10.0f, 0.0f).size(), 0);
  EXPECT_EQ(makeCellGrid(0.0f, 10.0f, 0.0f, 10.0f, -1.0f).size(), 0);
}

TEST(CellGridBuild, RefusesGridsAboveTheCap) {
  // 1 mm cells over 1 km would overflow an int cell count; the check must
  // happen before the multiply, or an overflowed (negative) count would pass.
  EXPECT_EQ(makeCellGrid(0.0f, 1000.0f, 0.0f, 1000.0f, 0.001f).size(), 0);
  // Just over the cap on area alone: 33 x 33 = 1089 > 1024.
  EXPECT_EQ(makeCellGrid(0.0f, 33.0f, 0.0f, 33.0f, 1.0f).size(), 0);
  // Just under it: 32 x 32 = 1024.
  EXPECT_EQ(makeCellGrid(0.0f, 32.0f, 0.0f, 32.0f, 1.0f).size(), 1024);
}

TEST(CellGridIndex, MapsPositionsToCellsAndRejectsOutside) {
  const CellGrid g = makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f);
  EXPECT_EQ(g.idAt(-14.0f, -14.0f), 0);          // row 0, col 0
  EXPECT_EQ(g.idAt(0.0f, 0.0f), 4);              // centre cell
  EXPECT_EQ(g.idAt(14.0f, 14.0f), 8);            // row 2, col 2
  EXPECT_EQ(g.idAt(-15.0f, -15.0f), 0);          // inclusive low corner
  EXPECT_EQ(g.idAt(-15.001f, 0.0f), -1);
  EXPECT_EQ(g.idAt(15.0f, 0.0f), -1);            // exclusive high edge
  EXPECT_EQ(g.idAt(kNaN, 0.0f), -1);
  EXPECT_EQ(g.idAt(0.0f, kInf), -1);
}

TEST(CellGridIndex, BoundariesGoToTheHigherCellOnEveryRobot) {
  // Not a tie-break preference: floor() is the convention voxel ingest uses,
  // and two robots that split a boundary differently would disagree about
  // which cell a shared observation belongs to.
  const CellGrid g = makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f);
  EXPECT_EQ(g.idAt(-5.0f, -14.0f), 1);
  EXPECT_EQ(g.idAt(-5.001f, -14.0f), 0);
}

TEST(CellGridIndex, CentresRoundTripThroughIdAt) {
  const CellGrid g = makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f);
  for (int id = 0; id < g.size(); ++id) {
    float x, y;
    g.centre(id, x, y);
    EXPECT_EQ(g.idAt(x, y), id) << "cell " << id;
  }
}

TEST(CellGridHash, DiffersOnEveryGeometryParameter) {
  const CellGrid base = makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f);
  EXPECT_NE(base.configHash(),
            makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 5.0f).configHash());
  EXPECT_NE(base.configHash(),
            makeCellGrid(-20.0f, 15.0f, -15.0f, 15.0f, 10.0f).configHash());
  EXPECT_NE(base.configHash(),
            makeCellGrid(-15.0f, 15.0f, -20.0f, 15.0f, 10.0f).configHash());
  // Same origin and cell size, larger extent: different nx/ny, so ids mean
  // different ground and the hash must say so.
  EXPECT_NE(base.configHash(),
            makeCellGrid(-15.0f, 25.0f, -15.0f, 15.0f, 10.0f).configHash());
}

TEST(CellGridHash, IsStableUnderSubMillimetreFloatNoise) {
  // Two robots may reach the same ROI through different arithmetic. Reporting
  // a config mismatch on a correctly configured fleet is the failure that
  // gets checks deleted.
  const CellGrid a = makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f);
  const CellGrid b = makeCellGrid(-15.0000001f, 15.0f, -15.0f, 15.0f, 10.0f);
  EXPECT_EQ(a.configHash(), b.configHash());
}

// ===========================================================================
// configure()
// ===========================================================================

TEST(CellWorldConfigure, AcceptsAWellFormedConfiguration) {
  CellWorld w;
  EXPECT_EQ(w.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                        goodConfig(), 0),
            "");
  EXPECT_TRUE(w.configured());
  EXPECT_EQ(w.size(), 9);
  EXPECT_EQ(w.selfId(), 0);
  for (int i = 0; i < w.size(); ++i)
    EXPECT_EQ(w.status(i), CellStatus::UNSEEN);
}

TEST(CellWorldConfigure, RefusesAnEmptyGrid) {
  CellWorld w;
  EXPECT_NE(w.configure(CellGrid{}, goodConfig(), 0), "");
  EXPECT_FALSE(w.configured());
}

TEST(CellWorldConfigure, RefusesWithoutAFleetId) {
  // An unconfigured fleet gives self_id -1, robotBit(-1) == 0, and every mask
  // this robot sets would be empty — the knowledge gate would then compute a
  // confident answer from nothing. Refusing is what keeps that visible.
  CellWorld w;
  const std::string err = w.configure(
      makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f), goodConfig(), -1);
  EXPECT_NE(err, "");
  EXPECT_NE(err.find("team_robot_names"), std::string::npos) << err;
  EXPECT_FALSE(w.configured());
  EXPECT_NE(w.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                        goodConfig(), kMaxTeamSize),
            "");
}

TEST(CellWorldConfigure, RefusesAZeroWidthHysteresisBand) {
  CellWorld w;
  CellWorld::Config c = goodConfig();
  c.exploring_min_unknown = c.covered_max_unknown;
  EXPECT_NE(w.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f), c, 0),
            "");
  EXPECT_FALSE(w.configured());
  // Inverted is refused too, not silently sorted: an operator who inverted the
  // pair believes they configured the band they typed.
  c.covered_max_unknown = 0.5;
  c.exploring_min_unknown = 0.2;
  EXPECT_NE(w.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f), c, 0),
            "");
  EXPECT_FALSE(w.configured());
}

TEST(CellWorldConfigure, RefusesOutOfRangeThresholds) {
  CellWorld w;
  const CellGrid g = makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f);
  CellWorld::Config c = goodConfig();
  c.covered_max_unknown = -0.1;
  EXPECT_NE(w.configure(g, c, 0), "");
  c = goodConfig();
  c.exploring_min_unknown = 1.5;
  EXPECT_NE(w.configure(g, c, 0), "");
  c = goodConfig();
  c.covered_max_frontier_frac = -0.1;
  EXPECT_NE(w.configure(g, c, 0), "");
  c = goodConfig();
  c.covered_max_frontier_frac = 1.1;
  EXPECT_NE(w.configure(g, c, 0), "")
      << "above 1 is not 'disabled', it is a fraction that cannot occur";
  c = goodConfig();
  c.min_observed_columns = -1;
  EXPECT_NE(w.configure(g, c, 0), "");
  c = goodConfig();
  c.edge_max_blocked_fraction = 2.0;
  EXPECT_NE(w.configure(g, c, 0), "");
}

TEST(CellWorldConfigure, ShippedDefaultsAssumeASaturatingMap) {
  // The defaults encode a saturating-map world model, like the shipped
  // done_unknown_fraction. Values tuned to one world with a coverage floor
  // belong in that world's launch config, not in the defaults.
  // (notes: cellworld-test-defaults-world-model)
  const CellWorld::Config d;
  EXPECT_LT(d.covered_max_unknown, 0.2)
      << "a saturating-map default; a value tuned to a world with a coverage "
         "floor belongs in that world's launch config, cf. DONE_UNKNOWN=0.64 "
         "in sim/run_explo_sim_rviz.sh";
  EXPECT_LT(d.exploring_min_unknown, 0.5)
      << "release must still be well under a typical unexplored cell, or "
         "COVERED stops meaning anything";
  EXPECT_LT(d.covered_max_unknown, d.exploring_min_unknown)
      << "no hysteresis band";
  // Permissive by construction: the frontier measure's discriminating power
  // depends on how densely the map stores free space, so the default must not
  // be the binding constraint in a map where it is near-saturated.
  EXPECT_GE(d.covered_max_frontier_frac, 0.85);
  EXPECT_LE(d.covered_max_frontier_frac, 1.0);
  // And the defaults must be a configuration the library itself accepts —
  // trivially true today, and the thing that silently stops being true when
  // someone adds a validation rule and updates only the test fixture.
  CellWorld w;
  EXPECT_EQ(w.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f), d, 0),
            "");
}

TEST(CellWorldConfigure, ReconfiguringClearsTheCensus) {
  // Cell ids ARE geometry. Keeping statuses across a reconfigure would
  // silently re-point every one of them at different ground.
  CellWorld w = makeWorld();
  ASSERT_TRUE(w.commitSelf(4, CellStatus::COVERED));
  EXPECT_EQ(w.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 5.0f),
                        goodConfig(), 0),
            "");
  EXPECT_EQ(w.size(), 36);
  for (int i = 0; i < w.size(); ++i)
    EXPECT_EQ(w.status(i), CellStatus::UNSEEN);
}

// ===========================================================================
// Wire vocabulary
// ===========================================================================

TEST(CellStatusWire, ProvenanceIsStrippedOnSend) {
  // The BY_OTHERS statuses answer "whose observation is this?", which has no
  // meaning at the far end of a relay. Sending them makes N=3 state
  // uninterpretable.
  EXPECT_EQ(normaliseForWire(CellStatus::UNSEEN), 0);
  EXPECT_EQ(normaliseForWire(CellStatus::EXPLORING), 1);
  EXPECT_EQ(normaliseForWire(CellStatus::COVERED), 2);
  EXPECT_EQ(normaliseForWire(CellStatus::EXPLORING_BY_OTHERS), 1);
  EXPECT_EQ(normaliseForWire(CellStatus::COVERED_BY_OTHERS), 2);
}

TEST(CellStatusWire, UnknownValuesDecodeToTheHarmlessOne) {
  EXPECT_EQ(fromWire(0), CellStatus::UNSEEN);
  EXPECT_EQ(fromWire(1), CellStatus::EXPLORING);
  EXPECT_EQ(fromWire(2), CellStatus::COVERED);
  // 3 and 4 are the local-only statuses: a peer sending them is either buggy
  // or a different build, and UNSEEN is the value that cannot cause a bad
  // merge (peer-UNSEEN changes nothing in the guard table).
  EXPECT_EQ(fromWire(3), CellStatus::UNSEEN);
  EXPECT_EQ(fromWire(4), CellStatus::UNSEEN);
  EXPECT_EQ(fromWire(255), CellStatus::UNSEEN);
}

TEST(CellStatusWire, RankAndProvenanceAgreeWithTheStatuses) {
  EXPECT_EQ(exploredRank(CellStatus::UNSEEN), 0);
  EXPECT_EQ(exploredRank(CellStatus::EXPLORING), 1);
  EXPECT_EQ(exploredRank(CellStatus::EXPLORING_BY_OTHERS), 1);
  EXPECT_EQ(exploredRank(CellStatus::COVERED), 2);
  EXPECT_EQ(exploredRank(CellStatus::COVERED_BY_OTHERS), 2);
  EXPECT_TRUE(isFirstHand(CellStatus::UNSEEN));
  EXPECT_TRUE(isFirstHand(CellStatus::EXPLORING));
  EXPECT_TRUE(isFirstHand(CellStatus::COVERED));
  EXPECT_FALSE(isFirstHand(CellStatus::EXPLORING_BY_OTHERS));
  EXPECT_FALSE(isFirstHand(CellStatus::COVERED_BY_OTHERS));
}

// ===========================================================================
// Status machine
// ===========================================================================

TEST(CellWorldClassify, WeakEvidenceLeavesACellUnseen) {
  const CellWorld w = makeWorld();
  // 3 observed columns is below min_observed_columns: one stray lidar return
  // from 40 m away must not promote a cell the robot has never been near.
  EXPECT_EQ(w.classify(CellStatus::UNSEEN, obs(100, 3)), CellStatus::UNSEEN);
  EXPECT_EQ(w.classify(CellStatus::UNSEEN, obs(100, 4)), CellStatus::EXPLORING);
}

TEST(CellWorldClassify, PartialObservationIsExploring) {
  const CellWorld w = makeWorld();
  EXPECT_EQ(w.classify(CellStatus::UNSEEN, obs(100, 50)),
            CellStatus::EXPLORING);
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100, 50)),
            CellStatus::EXPLORING);
}

TEST(CellWorldClassify, SaturatedObservationIsCovered) {
  const CellWorld w = makeWorld();
  // Deliberately not the exact threshold: 85 of 100 gives 0.15000000000000002
  // in doubles. Behaviour exactly at a threshold is not a contract of this
  // class. (notes: cellworld-test-threshold-rounding)
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100, 86)),
            CellStatus::COVERED);
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100, 84)),
            CellStatus::EXPLORING);
}

TEST(CellWorldClassify, RemainingFrontierVetoesPromotion) {
  // The column measure is 2.5D and cannot see an unknown pocket behind a
  // trunk; the frontier measure can.
  const CellWorld w = makeWorld();  // covered_max_frontier_frac = 0.90
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100, 100, 89)),
            CellStatus::COVERED);
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100, 100, 91)),
            CellStatus::EXPLORING);
}

TEST(CellWorldClassify, FrontierVetoIsAFractionNotACount) {
  // The property under test is scale invariance: the same shape of cell
  // measured at very different sizes must classify the same way. A
  // frontier-voxel count threshold fails this by construction.
  // (notes: cellworld-test-frontier-veto-fraction)
  const CellWorld w = makeWorld();
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100, 100, 50)),
            CellStatus::COVERED)
      << "50 frontier voxels is half a small cell — well inside the veto";
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100000, 100000, 50000)),
            CellStatus::COVERED)
      << "the same half, at 1000x the scale, must classify identically; a "
         "count threshold would have vetoed this and not the line above";
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100000, 100000, 95000)),
            CellStatus::EXPLORING)
      << "and a genuinely boundary-dominated cell must still be vetoed";
}

TEST(CellWorldClassify, UnmeasurableFrontierCannotPromote) {
  // frontierFraction() returns -1.0 for cannot-measure, which passes under any
  // threshold, so it must not promote. censusFromMap cannot produce this state;
  // the test guards the seam for other producers.
  // (notes: cellworld-test-frontier-sentinel)
  const CellWorld w = makeWorld();
  CellWorld::CellObservation o;
  o.total_columns    = 100;
  o.observed_columns = 100;   // clears min_observed_columns
  o.observed_voxels  = 0;     // ...but nothing to compute a fraction from
  o.frontier_voxels  = 0;
  EXPECT_DOUBLE_EQ(o.frontierFraction(), -1.0);
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, o), CellStatus::EXPLORING);
}

TEST(CellObservationFrontierFraction, SentinelAndClamp) {
  CellWorld::CellObservation o;
  EXPECT_DOUBLE_EQ(o.frontierFraction(), -1.0) << "no voxels: cannot measure";
  o.observed_voxels = 200;
  o.frontier_voxels = 50;
  EXPECT_DOUBLE_EQ(o.frontierFraction(), 0.25);
  // Clamped, mirroring unknownFraction(). A producer that counted a voxel as
  // frontier more than once would otherwise report a fraction above 1 and
  // silently disable the veto by overflowing past any threshold.
  o.frontier_voxels = 500;
  EXPECT_DOUBLE_EQ(o.frontierFraction(), 1.0);
}

TEST(CellWorldClassify, HysteresisHoldsCoveredInsideTheBand) {
  const CellWorld w = makeWorld();
  // Inside the band (covered_max_unknown 0.15, exploring_min_unknown 0.35)
  // COVERED holds. Without it a cell at the threshold re-commits every tick,
  // each commit resets known_by, and the knowledge gate never suppresses.
  // (notes: cellworld-test-hysteresis-band)
  EXPECT_EQ(w.classify(CellStatus::COVERED, obs(100, 75)), CellStatus::COVERED);
  EXPECT_EQ(w.classify(CellStatus::COVERED, obs(100, 65)), CellStatus::COVERED);
  EXPECT_EQ(w.classify(CellStatus::COVERED, obs(100, 64)),
            CellStatus::EXPLORING);
}

TEST(CellWorldClassify, FrontierDoesNotDemote) {
  // The frontier measure is the noisier one — it counts absent 6-neighbours,
  // so the ROI boundary contributes. Letting it demote would put the flap back
  // through a channel the hysteresis band does not cover.
  const CellWorld w = makeWorld();
  const CellWorld::CellObservation o = obs(100, 100, 95);  // ff = 0.95 > 0.90
  // Paired control: without this line the test would still pass with the veto
  // deleted outright, since the observation would then be unremarkable.
  ASSERT_EQ(w.classify(CellStatus::EXPLORING, o), CellStatus::EXPLORING)
      << "fixture no longer trips the veto; the demotion check below is void";
  EXPECT_EQ(w.classify(CellStatus::COVERED, o), CellStatus::COVERED);
}

TEST(CellWorldClassify, AnUnmeasuredTickChangesNothing) {
  const CellWorld w = makeWorld();
  EXPECT_EQ(w.classify(CellStatus::COVERED, obs(0, 0)), CellStatus::COVERED);
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(0, 0)),
            CellStatus::EXPLORING);
  EXPECT_EQ(w.classify(CellStatus::COVERED_BY_OTHERS, obs(0, 0)),
            CellStatus::COVERED_BY_OTHERS);
}

TEST(CellWorldClassify, WeakEvidenceCannotUnsayHavingBeenThere) {
  const CellWorld w = makeWorld();
  EXPECT_EQ(w.classify(CellStatus::EXPLORING, obs(100, 1)),
            CellStatus::EXPLORING);
  EXPECT_EQ(w.classify(CellStatus::COVERED, obs(100, 1)), CellStatus::COVERED);
}

TEST(CellWorldClassify, OwnIgnoranceNeverLowersAPeersObservation) {
  // My map saying "unknown here" is evidence I have not been, not evidence the
  // peer that was there is wrong. Demoting relayed belief would let a robot
  // erase the team's map by driving away from it.
  const CellWorld w = makeWorld();
  EXPECT_EQ(w.classify(CellStatus::COVERED_BY_OTHERS, obs(100, 0)),
            CellStatus::COVERED_BY_OTHERS);
  EXPECT_EQ(w.classify(CellStatus::COVERED_BY_OTHERS, obs(100, 50)),
            CellStatus::COVERED_BY_OTHERS);
  EXPECT_EQ(w.classify(CellStatus::EXPLORING_BY_OTHERS, obs(100, 0)),
            CellStatus::EXPLORING_BY_OTHERS);
}

TEST(CellWorldClassify, OwnObservationUpgradesRelayedBelief) {
  const CellWorld w = makeWorld();
  // Strictly better: EXPLORING_BY_OTHERS -> my COVERED.
  EXPECT_EQ(w.classify(CellStatus::EXPLORING_BY_OTHERS, obs(100, 100)),
            CellStatus::COVERED);
  // Equal rank still adopts: upgrading relayed to first-hand is real
  // information for the merge guard table.
  EXPECT_EQ(w.classify(CellStatus::EXPLORING_BY_OTHERS, obs(100, 50)),
            CellStatus::EXPLORING);
  EXPECT_EQ(w.classify(CellStatus::COVERED_BY_OTHERS, obs(100, 100)),
            CellStatus::COVERED);
}

// ===========================================================================
// update_id and known_by
// ===========================================================================

TEST(CellWorldCommit, UpdateIdCountsOnlyRealChanges) {
  CellWorld w = makeWorld();
  EXPECT_EQ(w.cell(4).update_id, 0u);
  EXPECT_TRUE(w.commitSelf(4, CellStatus::EXPLORING));
  EXPECT_EQ(w.cell(4).update_id, 1u);
  EXPECT_FALSE(w.commitSelf(4, CellStatus::EXPLORING));
  EXPECT_EQ(w.cell(4).update_id, 1u) << "a no-op commit must not bump";
  EXPECT_TRUE(w.commitSelf(4, CellStatus::COVERED));
  EXPECT_EQ(w.cell(4).update_id, 2u);
}

TEST(CellWorldCommit, KnownByResetsToSelfOnEveryChange) {
  // The reset is what keeps the §3.6 knowledge gate able to fire. Without it
  // the mask only grows, so "does my peer already know everything I could tell
  // it?" answers yes forever for any cell the peer ever heard of, and the gate
  // silently stops gating.
  CellWorld w = makeWorld();
  ASSERT_TRUE(w.commitSelf(4, CellStatus::EXPLORING));
  w.markKnownBy(4, 1);
  w.markKnownBy(4, 2);
  EXPECT_EQ(maskCount(w.cell(4).known_by), 3);
  ASSERT_TRUE(w.commitSelf(4, CellStatus::COVERED));
  EXPECT_EQ(w.cell(4).known_by, robotBit(0))
      << "a status change invalidates what peers knew about the cell";
}

TEST(CellWorldCommit, MarkKnownByDoesNotTouchUpdateId) {
  // update_id counts FIRST-HAND changes only. If the merge path could bump it,
  // it would start to look like an information-quality ordering, and someone
  // downstream would compare it across robots — which regresses the census.
  CellWorld w = makeWorld();
  ASSERT_TRUE(w.commitSelf(4, CellStatus::EXPLORING));
  const uint32_t before = w.cell(4).update_id;
  w.markKnownBy(4, 3);
  EXPECT_EQ(w.cell(4).update_id, before);
  EXPECT_TRUE(maskHas(w.cell(4).known_by, 3));
}

TEST(CellWorldCommit, IgnoresOutOfRangeIds) {
  CellWorld w = makeWorld();
  EXPECT_FALSE(w.commitSelf(-1, CellStatus::COVERED));
  EXPECT_FALSE(w.commitSelf(9, CellStatus::COVERED));
  w.markKnownBy(9, 1);  // must not write past the end
}

TEST(CellWorldApply, CountsChangesAndClaimsSelfKnowledge) {
  CellWorld w = makeWorld();
  std::vector<CellWorld::CellObservation> o(9, obs(100, 0));
  o[0] = obs(100, 50);
  o[1] = obs(100, 100);
  EXPECT_EQ(w.applyObservation(o), 2);
  EXPECT_EQ(w.status(0), CellStatus::EXPLORING);
  EXPECT_EQ(w.status(1), CellStatus::COVERED);
  EXPECT_EQ(w.status(2), CellStatus::UNSEEN);
  EXPECT_TRUE(maskHas(w.cell(0).known_by, 0));
  // A second identical tick changes nothing.
  EXPECT_EQ(w.applyObservation(o), 0);
}

TEST(CellWorldApply, UnmeasuredCellsAreLeftAlone) {
  CellWorld w = makeWorld();
  std::vector<CellWorld::CellObservation> o(9, obs(100, 100));
  ASSERT_EQ(w.applyObservation(o), 9);
  // A tick with no measurement at all — a map that has not been republished —
  // must not erase the census.
  std::vector<CellWorld::CellObservation> none(9, obs(0, 0));
  EXPECT_EQ(w.applyObservation(none), 0);
  for (int i = 0; i < 9; ++i) EXPECT_EQ(w.status(i), CellStatus::COVERED);
}

TEST(CellWorldApply, RejectsAWronglySizedObservation) {
  CellWorld w = makeWorld();
  std::vector<CellWorld::CellObservation> o(8, obs(100, 100));
  EXPECT_EQ(w.applyObservation(o), 0);
  EXPECT_EQ(w.status(0), CellStatus::UNSEEN);
}

TEST(CellWorldCensus, CountsEveryStatusAndReportsCoveredFraction) {
  CellWorld w = makeWorld();
  ASSERT_TRUE(w.commitSelf(0, CellStatus::EXPLORING));
  ASSERT_TRUE(w.commitSelf(1, CellStatus::COVERED));
  ASSERT_TRUE(w.commitSelf(2, CellStatus::COVERED));
  ASSERT_TRUE(w.commitSelf(3, CellStatus::COVERED_BY_OTHERS));
  ASSERT_TRUE(w.commitSelf(4, CellStatus::EXPLORING_BY_OTHERS));
  const CellWorld::Census c = w.census();
  EXPECT_EQ(c.unseen, 4);
  EXPECT_EQ(c.exploring, 1);
  EXPECT_EQ(c.covered, 2);
  EXPECT_EQ(c.covered_by_others, 1);
  EXPECT_EQ(c.exploring_by_others, 1);
  EXPECT_NEAR(c.coveredFraction(), 3.0 / 9.0, 1e-12);
}

TEST(CellWorldObservation, UnknownFractionUsesTheCannotMeasureConvention) {
  EXPECT_NEAR(obs(100, 0).unknownFraction(), 1.0, 1e-12);
  EXPECT_NEAR(obs(100, 100).unknownFraction(), 0.0, 1e-12);
  EXPECT_NEAR(obs(100, 25).unknownFraction(), 0.75, 1e-12);
  EXPECT_LT(obs(0, 0).unknownFraction(), 0.0);
}

// ===========================================================================
// Adjacency
// ===========================================================================

TEST(CellWorldEdges, EightConnectedWithNoProbe) {
  CellWorld w = makeWorld();  // 3x3
  w.rebuildEdges();
  EXPECT_TRUE(w.edgeEnabled(4, 0));   // centre to every corner and side
  EXPECT_TRUE(w.edgeEnabled(4, 8));
  EXPECT_TRUE(w.edgeEnabled(0, 1));
  EXPECT_TRUE(w.edgeEnabled(0, 4));   // diagonal
  EXPECT_FALSE(w.edgeEnabled(0, 2));  // two apart on a row
  EXPECT_FALSE(w.edgeEnabled(0, 0));  // no self edge
  int corner_degree = 0;
  for (int i = 0; i < 9; ++i)
    if (w.edgeEnabled(0, i)) ++corner_degree;
  EXPECT_EQ(corner_degree, 3);
}

TEST(CellWorldEdges, RowWrapAroundIsNotAnEdge) {
  // Cell 2 (col 2, row 0) and cell 3 (col 0, row 1) are adjacent in the
  // row-major id space but 20 m apart on the ground. A naive id+1 adjacency
  // would connect them.
  CellWorld w = makeWorld();
  w.rebuildEdges();
  EXPECT_FALSE(w.edgeEnabled(2, 3));
}

TEST(CellWorldEdges, ABlockedProbeDisablesEdgesAndDistanceRoutesAround) {
  CellWorld w = makeWorld();
  // Wall down the middle: every edge crossing x = -5 -> 5 is blocked, so the
  // left column is reachable from the right only around the top or bottom...
  // except this is an 8-connected grid, so block every edge whose endpoints
  // straddle the middle column.
  auto blocked = [](float ax, float, float bx, float) -> double {
    const bool a_left = ax < -5.0f, b_left = bx < -5.0f;
    return (a_left != b_left) ? 1.0 : 0.0;
  };
  w.rebuildEdges(blocked);
  EXPECT_FALSE(w.edgeEnabled(0, 1));
  EXPECT_TRUE(w.edgeEnabled(1, 2));
  EXPECT_TRUE(w.edgeEnabled(0, 3));  // within the left column
  EXPECT_LT(w.distance(0, 1), 0.0) << "left column is cut off entirely";
  EXPECT_GT(w.distance(0, 6), 0.0);
}

TEST(CellWorldEdges, DistanceIsShortestPathNotStraightLine) {
  CellWorld w = makeWorld();
  // Block only the diagonals, forcing 4-connected travel.
  auto no_diagonals = [](float ax, float ay, float bx, float by) -> double {
    return (ax != bx && ay != by) ? 1.0 : 0.0;
  };
  w.rebuildEdges(no_diagonals);
  // Cell 0 to cell 4 is 10 m diagonally but 20 m in two axis-aligned hops.
  EXPECT_NEAR(w.centroidDistance(0, 4), std::hypot(10.0, 10.0), 1e-9);
  EXPECT_NEAR(w.distance(0, 4), 20.0, 1e-9);
}

TEST(CellWorldEdges, RebuildingInvalidatesTheCachedDistances) {
  // The table is cached behind a dirty flag; a stale table after an edge
  // change is exactly the kind of silently-wrong answer that never throws.
  CellWorld w = makeWorld();
  w.rebuildEdges();
  ASSERT_NEAR(w.distance(0, 4), std::hypot(10.0, 10.0), 1e-9);
  auto no_diagonals = [](float ax, float ay, float bx, float by) -> double {
    return (ax != bx && ay != by) ? 1.0 : 0.0;
  };
  w.rebuildEdges(no_diagonals);
  EXPECT_NEAR(w.distance(0, 4), 20.0, 1e-9);
}

TEST(CellWorldEdges, ANegativeProbeMeansNoMapAndKeepsTheEdge) {
  // An absent plan map must not disconnect the graph: the allocator would
  // then fall back to straight lines everywhere the moment the map lagged.
  CellWorld w = makeWorld();
  auto no_map = [](float, float, float, float) -> double { return -1.0; };
  w.rebuildEdges(no_map);
  EXPECT_TRUE(w.edgeEnabled(0, 1));
  EXPECT_TRUE(w.edgeEnabled(4, 8));
}

TEST(CellWorldEdges, PartialBlockageIsToleratedUpToTheThreshold) {
  CellWorld w = makeWorld();  // edge_max_blocked_fraction defaults to 0.5
  auto half = [](float, float, float, float) -> double { return 0.5; };
  w.rebuildEdges(half);
  EXPECT_TRUE(w.edgeEnabled(0, 1));
  auto over = [](float, float, float, float) -> double { return 0.51; };
  w.rebuildEdges(over);
  EXPECT_FALSE(w.edgeEnabled(0, 1));
}

TEST(CellWorldEdges, CentroidDistanceSurvivesADisconnectedGraph) {
  // The allocator's fallback: an over-aggressive edge probe must degrade the
  // ranking, not remove cells from consideration.
  CellWorld w = makeWorld();
  auto all_blocked = [](float, float, float, float) -> double { return 1.0; };
  w.rebuildEdges(all_blocked);
  EXPECT_LT(w.distance(0, 8), 0.0);
  EXPECT_NEAR(w.centroidDistance(0, 8), std::hypot(20.0, 20.0), 1e-9);
  EXPECT_NEAR(w.distance(4, 4), 0.0, 1e-12);
}

TEST(CellWorldEdges, OutOfRangeQueriesAreNegativeNotUndefined) {
  CellWorld w = makeWorld();
  w.rebuildEdges();
  EXPECT_LT(w.distance(-1, 0), 0.0);
  EXPECT_LT(w.distance(0, 9), 0.0);
  EXPECT_LT(w.centroidDistance(0, 9), 0.0);
  EXPECT_FALSE(w.edgeEnabled(0, 9));
}

// ===========================================================================
// censusFromMap
// ===========================================================================

TEST(CensusFromMap, RefusesAnUnconfiguredGrid) {
  MapCache map(0.1);
  EXPECT_TRUE(censusFromMap(map, CellGrid{}).empty());
}

TEST(CensusFromMap, AnEmptyMapIsFullyUnknownButMeasurable) {
  MapCache map(0.1);
  const auto o = censusFromMap(map, mapTestGrid());
  ASSERT_EQ(o.size(), 4u);
  for (const auto& c : o) {
    EXPECT_EQ(c.total_columns, 100) << "denominator is geometry, not data";
    EXPECT_EQ(c.observed_columns, 0);
    EXPECT_NEAR(c.unknownFraction(), 1.0, 1e-12);
  }
}

TEST(CensusFromMap, ColumnCountsPartitionTheRoi) {
  // Every voxel column in the ROI belongs to exactly one cell: if the
  // denominators did not sum to the ROI's own column count, the census and
  // MapCache::unknownColumnFraction would be measuring different things and
  // the P1 agreement gate would be vacuous.
  const CellGrid g = mapTestGrid();
  MapCache map(0.1);
  const auto o = censusFromMap(map, g);
  int total = 0;
  for (const auto& c : o) total += c.total_columns;
  EXPECT_EQ(total, 400) << "20 x 20 columns of 0.1 m over a 2 x 2 m ROI";
}

TEST(CensusFromMap, ObservationsLandInTheRightCell) {
  std::vector<Eigen::Vector3f> occ;
  for (int i = 0; i < 10; ++i)
    for (int j = 0; j < 10; ++j)
      occ.emplace_back(colCentre(0.0f, i), colCentre(0.0f, j), 0.5f);
  MapCache map = makeMap(occ);
  const auto o = censusFromMap(map, mapTestGrid());
  ASSERT_EQ(o.size(), 4u);
  EXPECT_EQ(o[0].observed_columns, 100);
  EXPECT_NEAR(o[0].unknownFraction(), 0.0, 1e-12);
  EXPECT_EQ(o[1].observed_columns, 0);
  EXPECT_EQ(o[2].observed_columns, 0);
  EXPECT_EQ(o[3].observed_columns, 0);
}

TEST(CensusFromMap, AgreesWithMapCacheUnknownColumnFractionPerCell) {
  // This is the measurement the P1 gate compares against. The two must share a
  // definition of "observed column", or the smoke-run agreement check would be
  // testing the difference between two conventions rather than the census.
  std::vector<Eigen::Vector3f> occ;
  for (int i = 0; i < kExactCols; ++i)
    for (int j = 0; j < 3; ++j)  // 24 of cell 0's 64 columns
      occ.emplace_back(exactCentre(0.0f, i), exactCentre(0.0f, j), 0.5f);
  for (int i = 0; i < 5; ++i)    // 5 of cell 3's, on the diagonal
    occ.emplace_back(exactCentre(1.0f, i), exactCentre(1.0f, i), 0.5f);
  MapCache map = makeMap(occ, {}, kExactRes);
  const auto o = censusFromMap(map, mapTestGrid());
  ASSERT_EQ(o.size(), 4u);
  for (const auto& c : o) EXPECT_EQ(c.total_columns, kExactCols * kExactCols);

  EXPECT_NEAR(o[0].unknownFraction(),
              map.unknownColumnFraction(0.0f, exactHigh(0.0f), 0.0f,
                                        exactHigh(0.0f)),
              1e-12);
  EXPECT_NEAR(o[1].unknownFraction(),
              map.unknownColumnFraction(1.0f, exactHigh(1.0f), 0.0f,
                                        exactHigh(0.0f)),
              1e-12);
  EXPECT_NEAR(o[3].unknownFraction(),
              map.unknownColumnFraction(1.0f, exactHigh(1.0f), 1.0f,
                                        exactHigh(1.0f)),
              1e-12);
  // And the values are the ones arithmetic says they should be, so a pair of
  // agreeing-but-wrong measures could not pass the comparison above.
  EXPECT_NEAR(o[0].unknownFraction(), 1.0 - 24.0 / 64.0, 1e-12);
  EXPECT_NEAR(o[1].unknownFraction(), 1.0, 1e-12);
  EXPECT_NEAR(o[3].unknownFraction(), 1.0 - 5.0 / 64.0, 1e-12);
}

TEST(CensusFromMap, AgreesWithMapCacheUnknownColumnFractionOverTheWholeRoi) {
  // The aggregate form is what the P1 smoke gate actually compares: the
  // planner's existing ROI-wide coverage measure against the census. Per-cell
  // agreement does not imply it — a census that double-counted a column on a
  // shared boundary would pass the per-cell test and fail this one.
  std::vector<Eigen::Vector3f> occ;
  for (int i = 0; i < kExactCols; ++i)
    for (int j = 0; j < 3; ++j)
      occ.emplace_back(exactCentre(0.0f, i), exactCentre(0.0f, j), 0.5f);
  for (int i = 0; i < 5; ++i)
    occ.emplace_back(exactCentre(1.0f, i), exactCentre(1.0f, i), 0.5f);
  MapCache map = makeMap(occ, {}, kExactRes);
  const auto o = censusFromMap(map, mapTestGrid());

  int total = 0, observed = 0;
  for (const auto& c : o) {
    total += c.total_columns;
    observed += c.observed_columns;
  }
  EXPECT_EQ(total, 4 * kExactCols * kExactCols);
  EXPECT_EQ(observed, 29);
  EXPECT_NEAR(1.0 - static_cast<double>(observed) / total,
              map.unknownColumnFraction(0.0f, exactHigh(1.0f), 0.0f,
                                        exactHigh(1.0f)),
              1e-12);
}

TEST(CensusFromMap, VoxelsOutsideTheRoiAreDropped) {
  MapCache map = makeMap({{5.0f, 5.0f, 0.5f}, {-3.0f, 0.5f, 0.5f},
                          {colCentre(0.0f, 0), colCentre(0.0f, 0), 0.5f}});
  const auto o = censusFromMap(map, mapTestGrid());
  int observed = 0;
  for (const auto& c : o) observed += c.observed_columns;
  EXPECT_EQ(observed, 1);
  EXPECT_EQ(o[0].observed_columns, 1);
}

TEST(CensusFromMap, AColumnCountsOnceHoweverTallItIs) {
  // Column semantics are 2.5D by design: a trunk's whole stack is one observed
  // column, not twenty. Counting voxels instead would make a cell with one
  // tree look better covered than a cell with swept open ground.
  std::vector<Eigen::Vector3f> occ;
  for (int k = 0; k < 20; ++k)
    occ.emplace_back(colCentre(0.0f, 0), colCentre(0.0f, 0), 0.05f + 0.1f * k);
  MapCache map = makeMap(occ);
  const auto o = censusFromMap(map, mapTestGrid());
  EXPECT_EQ(o[0].observed_columns, 1);
  EXPECT_EQ(o[0].observed_voxels, 20);
}

TEST(CensusFromMap, FrontierVoxelsAreCountedPerCell) {
  // A free voxel with an absent 6-neighbour. An isolated free voxel has six,
  // so it counts once (the loop breaks on the first).
  MapCache map = makeMap({}, {{colCentre(1.0f, 3), colCentre(0.0f, 3), 0.5f},
                              {colCentre(1.0f, 5), colCentre(0.0f, 5), 0.5f}});
  const auto o = censusFromMap(map, mapTestGrid());
  EXPECT_EQ(o[1].frontier_voxels, 2);
  EXPECT_EQ(o[0].frontier_voxels, 0);
  EXPECT_EQ(o[3].frontier_voxels, 0);
}

TEST(CensusFromMap, OccupiedVoxelsAreNotFrontier) {
  MapCache map = makeMap({{colCentre(0.0f, 3), colCentre(0.0f, 3), 0.5f}});
  const auto o = censusFromMap(map, mapTestGrid());
  EXPECT_EQ(o[0].observed_columns, 1);
  EXPECT_EQ(o[0].frontier_voxels, 0);
}

TEST(CensusFromMap, DrivesTheStatusMachineEndToEnd) {
  // The path the planner actually runs: measure the map, apply, read the
  // census. Cell 0 fully swept, cell 1 barely touched.
  std::vector<Eigen::Vector3f> occ;
  for (int i = 0; i < 10; ++i)
    for (int j = 0; j < 10; ++j)
      occ.emplace_back(colCentre(0.0f, i), colCentre(0.0f, j), 0.5f);
  for (int i = 0; i < 8; ++i)
    occ.emplace_back(colCentre(1.0f, i), colCentre(0.0f, 0), 0.5f);
  MapCache map = makeMap(occ);

  CellWorld w;
  ASSERT_EQ(w.configure(mapTestGrid(), goodConfig(), 0), "");
  const int changed = w.applyObservation(censusFromMap(map, mapTestGrid()));
  EXPECT_EQ(changed, 2);
  EXPECT_EQ(w.status(0), CellStatus::COVERED);
  EXPECT_EQ(w.status(1), CellStatus::EXPLORING);
  EXPECT_EQ(w.status(2), CellStatus::UNSEEN);
  EXPECT_EQ(w.status(3), CellStatus::UNSEEN);
  EXPECT_NEAR(w.census().coveredFraction(), 0.25, 1e-12);
}

TEST(CensusFromMap, DenominatorAndNumeratorShareTheColumnBinning) {
  // Column corners are double but CellGrid is float, so a boundary column can
  // floor() to different cells; the denominator and numerator must share one
  // binning. 0.05 m columns over 5 m cells expose it; 0.1 m over 1 m do not.
  // (notes: census-test-shared-column-binning)
  const CellGrid g = makeCellGrid(-15.0f, 5.0f, -15.0f, 5.0f, 5.0f);
  ASSERT_EQ(g.size(), 16);
  MapCache map = makeMap({}, {}, 0.05f);
  const auto o = censusFromMap(map, g);
  ASSERT_EQ(o.size(), 16u);
  int total = 0;
  for (int i = 0; i < 16; ++i) {
    // 5 m of 0.05 m columns is exactly 100 per axis, for every cell.
    EXPECT_EQ(o[i].total_columns, 100 * 100) << "cell " << i;
    total += o[i].total_columns;
  }
  EXPECT_EQ(total, 400 * 400) << "the cells must partition the ROI's columns";
}

TEST(CensusFromMap, WorksOnANegativeOriginRoi) {
  // Float-narrowing hazard at a negative-origin ROI: the denominator and
  // numerator must agree on the boundary column, or the west and south edge
  // cells report an unknown fraction below the truth.
  // (notes: census-test-negative-origin-roi)
  const CellGrid g = makeCellGrid(-15.0f, -13.0f, -15.0f, -13.0f, 1.0f);
  ASSERT_EQ(g.size(), 4);
  std::vector<Eigen::Vector3f> occ;
  for (int i = 0; i < 10; ++i)
    for (int j = 0; j < 10; ++j)
      occ.emplace_back(colCentre(-15.0f, i), colCentre(-15.0f, j), 0.5f);
  MapCache map = makeMap(occ);
  const auto o = censusFromMap(map, g);
  ASSERT_EQ(o.size(), 4u);
  int total = 0;
  for (const auto& c : o) total += c.total_columns;
  EXPECT_EQ(total, 400);
  EXPECT_EQ(o[0].total_columns, 100);
  EXPECT_EQ(o[0].observed_columns, 100);
  EXPECT_NEAR(o[0].unknownFraction(), 0.0, 1e-12);
  // The other three cells are empty, not miscounted into cell 0.
  for (int i = 1; i < 4; ++i) EXPECT_EQ(o[i].observed_columns, 0);
}

// ===========================================================================
// The wire codec and the merge guard table (P2, plan §3.2)
// ===========================================================================

namespace {

/// Drive a cell to a first-hand status without going through the map: set the
/// observation the classifier would need. `covered` uses a fully observed,
/// frontier-free cell; `exploring` a half-observed one.
void selfCover(CellWorld& w, int id) {
  std::vector<CellWorld::CellObservation> o(w.size());
  o[static_cast<size_t>(id)] = obs(100, 100, 0);
  w.applyObservation(o);
  ASSERT_EQ(w.status(id), CellStatus::COVERED);
}

void selfExplore(CellWorld& w, int id) {
  std::vector<CellWorld::CellObservation> o(w.size());
  o[static_cast<size_t>(id)] = obs(100, 50, 0);
  w.applyObservation(o);
  ASSERT_EQ(w.status(id), CellStatus::EXPLORING);
}

/// One wire cell, written the way a peer would send it.
CellWorld::WireCell wc(int id, uint8_t status, uint32_t known_by = 0,
                       uint32_t update_id = 0) {
  CellWorld::WireCell w;
  w.id        = static_cast<uint16_t>(id);
  w.status    = status;
  w.known_by  = known_by;
  w.update_id = update_id;
  return w;
}

}  // namespace

TEST(CellWorldWire, NormalisesProvenanceAway) {
  CellWorld w = makeWorld();
  selfCover(w, 0);
  selfExplore(w, 1);
  // Cells 2 and 3 acquire their statuses from a peer, so they are the
  // *_BY_OTHERS pair the wire vocabulary must not carry.
  w.mergeWire(1, {wc(2, 2), wc(3, 1)});
  ASSERT_EQ(w.status(2), CellStatus::COVERED_BY_OTHERS);
  ASSERT_EQ(w.status(3), CellStatus::EXPLORING_BY_OTHERS);

  const std::vector<CellWorld::WireCell> out = w.toWire();
  ASSERT_EQ(out.size(), static_cast<size_t>(w.size()));
  for (size_t i = 0; i < out.size(); ++i)
    EXPECT_EQ(out[i].id, static_cast<uint16_t>(i)) << "full state, in id order";
  EXPECT_EQ(out[0].status, 2u);
  EXPECT_EQ(out[1].status, 1u);
  EXPECT_EQ(out[2].status, 2u) << "covered_by_others must ship as covered";
  EXPECT_EQ(out[3].status, 1u) << "exploring_by_others must ship as exploring";
  for (const CellWorld::WireCell& c : out)
    EXPECT_LE(c.status, 2u) << "no local-only status may reach the wire";
}

// ===========================================================================
// sharedHash — the cross-robot convergence readout
// ===========================================================================

TEST(CellWorldSharedHash, AgreesAcrossRobotsAndSeparatesOneCell) {
  CellWorld a = makeWorld();  // self 0
  CellWorld b;
  ASSERT_EQ(b.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                        goodConfig(), 1), "");
  EXPECT_EQ(a.sharedHash(), b.sharedHash())
      << "two freshly configured worlds hold the same (empty) shared belief";

  selfCover(a, 0);
  selfCover(b, 0);
  EXPECT_EQ(a.sharedHash(), b.sharedHash());

  selfExplore(a, 4);
  EXPECT_NE(a.sharedHash(), b.sharedHash())
      << "one cell apart must not hash the same — this is the whole point";
  selfExplore(b, 4);
  EXPECT_EQ(a.sharedHash(), b.sharedHash());
}

TEST(CellWorldSharedHash, IgnoresProvenanceAndLocalBookkeeping) {
  // The three fields that legitimately differ between two agreeing robots.
  // If any of them reached the digest, a healed pair would never read as
  // converged and the P2 gate could not pass on correct behaviour.
  CellWorld first_hand = makeWorld();   // self 0
  selfCover(first_hand, 0);
  selfExplore(first_hand, 1);

  CellWorld second_hand;
  ASSERT_EQ(second_hand.configure(
                makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                goodConfig(), 1), "");
  second_hand.mergeWire(0, first_hand.toWire());
  ASSERT_EQ(second_hand.status(0), CellStatus::COVERED_BY_OTHERS);
  ASSERT_EQ(second_hand.status(1), CellStatus::EXPLORING_BY_OTHERS);
  // Different provenance, different update_ids (rule 1: the merge never
  // touches them), different known_by masks — same shared belief.
  ASSERT_NE(first_hand.cell(0).update_id, second_hand.cell(0).update_id);
  ASSERT_NE(first_hand.cell(0).known_by, second_hand.cell(0).known_by);
  EXPECT_EQ(first_hand.sharedHash(), second_hand.sharedHash());
}

TEST(CellWorldSharedHash, DoesNotCollideAcrossGridSizes) {
  // Two all-UNSEEN worlds of different extent. Without the length prefix these
  // differ only in how many zero bytes were folded in, and FNV's avalanche is
  // no defence against that: the caller is supposed to compare grid_hash
  // first, and this is what happens when it forgets.
  CellWorld small = makeWorld();                    // 3x3
  CellWorld large;
  ASSERT_EQ(large.configure(makeCellGrid(-25.0f, 25.0f, -25.0f, 25.0f, 10.0f),
                            goodConfig(), 0), "");
  ASSERT_NE(small.size(), large.size());
  EXPECT_NE(small.sharedHash(), large.sharedHash());
}

TEST(CellWorldSharedHash, ConvergesAfterAnOutageHeals) {
  // Two robots diverge with the link down, then exchange full state once it
  // heals. Covered counts stay equal while the worlds disagree on four cells,
  // so only sharedHash can see the divergence.
  // (notes: sharedhash-test-outage-heal)
  CellWorld a = makeWorld();  // self 0
  CellWorld b;
  ASSERT_EQ(b.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                        goodConfig(), 1), "");
  selfCover(a, 0);
  selfCover(a, 1);
  selfCover(b, 7);
  selfCover(b, 8);
  ASSERT_EQ(a.census().covered + a.census().covered_by_others,
            b.census().covered + b.census().covered_by_others)
      << "fixture: the counts must AGREE while the worlds do not";
  EXPECT_NE(a.sharedHash(), b.sharedHash())
      << "the digest must see the divergence the counts cannot";

  // The heal: one full-state message each way, which is all the exchange ever
  // sends. Both directions are needed — one alone leaves the sender still
  // ignorant of the receiver's ground.
  const std::vector<CellWorld::WireCell> from_a = a.toWire();
  const std::vector<CellWorld::WireCell> from_b = b.toWire();
  a.mergeWire(1, from_b);
  b.mergeWire(0, from_a);
  EXPECT_EQ(a.sharedHash(), b.sharedHash()) << "converged after the heal";
  EXPECT_EQ(a.census().covered + a.census().covered_by_others, 4);

  // And it stays converged when the healed link redelivers what both already
  // have, which is what a 1 Hz full-state broadcast does forever after.
  const uint32_t settled = a.sharedHash();
  a.mergeWire(1, b.toWire());
  b.mergeWire(0, a.toWire());
  EXPECT_EQ(a.sharedHash(), settled);
  EXPECT_EQ(b.sharedHash(), settled);
}

TEST(CellWorldMerge, RoundTripThroughTheCodec) {
  // The plan's gate: normalise, transmit, merge — not a copy of in-memory
  // state. A robot that has seen everything hands its census to a robot that
  // has seen nothing, and the receiver must end up agreeing about every cell.
  CellWorld a = makeWorld();  // self id 0
  CellWorld b;
  ASSERT_EQ(b.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                        goodConfig(), 1),
            "");
  selfCover(a, 0);
  selfCover(a, 4);
  selfExplore(a, 8);

  const CellWorld::MergeStats st = b.mergeWire(0, a.toWire());
  EXPECT_EQ(st.refused, "");
  EXPECT_EQ(st.examined(), b.size()) << "every offered cell must land in "
                                        "exactly one bucket";
  EXPECT_EQ(st.applied, 3);
  EXPECT_EQ(b.status(0), CellStatus::COVERED_BY_OTHERS);
  EXPECT_EQ(b.status(4), CellStatus::COVERED_BY_OTHERS);
  EXPECT_EQ(b.status(8), CellStatus::EXPLORING_BY_OTHERS);
  // Rule 1: the merge path never touches update_id, however much it changed.
  for (int i = 0; i < b.size(); ++i) EXPECT_EQ(b.cell(i).update_id, 0u);
  // Rule 2's merge-side form: the mask is the two robots that can vouch.
  EXPECT_EQ(b.cell(0).known_by, robotBit(0) | robotBit(1));
}

TEST(CellWorldMerge, IsIdempotent) {
  CellWorld a = makeWorld();
  CellWorld b;
  ASSERT_EQ(b.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                        goodConfig(), 1),
            "");
  selfCover(a, 0);
  selfExplore(a, 5);
  const std::vector<CellWorld::WireCell> msg = a.toWire();

  b.mergeWire(0, msg);
  std::vector<CellStatus> after1;
  std::vector<uint32_t> mask1;
  for (int i = 0; i < b.size(); ++i) {
    after1.push_back(b.status(i));
    mask1.push_back(b.cell(i).known_by);
  }
  // Full state, not deltas: replaying the same message must be a no-op, which
  // is what makes a dropped or duplicated message cost latency and nothing
  // else. Three replays, because a merge that alternated would pass on one.
  for (int k = 0; k < 3; ++k) {
    const CellWorld::MergeStats st = b.mergeWire(0, msg);
    EXPECT_EQ(st.applied, 0) << "replay " << k << " changed a status";
    EXPECT_EQ(st.known_by_only, 0) << "replay " << k << " changed a mask";
  }
  for (int i = 0; i < b.size(); ++i) {
    EXPECT_EQ(b.status(i), after1[static_cast<size_t>(i)]);
    EXPECT_EQ(b.cell(i).known_by, mask1[static_cast<size_t>(i)]);
  }
}

TEST(CellWorldMerge, NeverDowngradesFirstHandCovered) {
  CellWorld w = makeWorld();
  selfCover(w, 0);
  const uint32_t uid = w.cell(0).update_id;

  const CellWorld::MergeStats st = w.mergeWire(1, {wc(0, 1)});  // peer EXPLORING
  EXPECT_EQ(st.applied, 0);
  EXPECT_EQ(st.refused_guard, 1);
  EXPECT_EQ(w.status(0), CellStatus::COVERED);
  EXPECT_EQ(w.cell(0).update_id, uid) << "the merge path may not move update_id";
}

TEST(CellWorldMerge, PeerCoveredOverridesOwnExploringOutsideTheNeighbourhood) {
  CellWorld w = makeWorld();
  selfExplore(w, 8);
  // Local priority centred on cell 0, which in a 3x3 grid does not reach 8.
  const CellWorld::MergeStats st = w.mergeWire(1, {wc(8, 2)}, /*centre=*/0);
  EXPECT_EQ(st.applied, 1);
  EXPECT_EQ(w.status(8), CellStatus::COVERED_BY_OTHERS);
}

TEST(CellWorldMerge, LocalPriorityRefusesInMyOwnNeighbourhood) {
  CellWorld w = makeWorld();
  selfExplore(w, 4);   // the centre cell of a 3x3 grid
  const uint32_t before = w.cell(4).known_by;

  const CellWorld::MergeStats st = w.mergeWire(1, {wc(4, 2, robotBit(1))},
                                               /*centre=*/4);
  EXPECT_EQ(st.refused_local, 1);
  EXPECT_EQ(st.applied, 0);
  EXPECT_EQ(w.status(4), CellStatus::EXPLORING);
  EXPECT_EQ(w.cell(4).known_by, before)
      << "refused ENTIRELY: crediting the peer with knowing a status we are "
         "standing in and about to change would suppress the very "
         "reconnection that would correct it";

  // Paired control: the same claim, with the rule disabled, must land. Without
  // this the test above would still pass against a merge that ignored peer
  // COVERED altogether.
  CellWorld u = makeWorld();
  selfExplore(u, 4);
  EXPECT_EQ(u.mergeWire(1, {wc(4, 2)}, /*centre=*/-1).applied, 1);
  EXPECT_EQ(u.status(4), CellStatus::COVERED_BY_OTHERS);
}

TEST(CellWorldMerge, LocalPriorityCoversTheEightNeighbours) {
  // The rule is a neighbourhood, not a single cell. Centred on 4 in a 3x3
  // grid, every cell is a neighbour; centred on 0, cells 0,1,3,4 are.
  CellWorld w = makeWorld();
  int n9[9];
  EXPECT_EQ(w.neighbourhood9(4, n9), 9);
  EXPECT_EQ(w.neighbourhood9(0, n9), 4) << "corner: clipped to the grid";
  EXPECT_EQ(w.neighbourhood9(-1, n9), 0);
  EXPECT_EQ(w.neighbourhood9(99, n9), 0);

  selfExplore(w, 1);   // a neighbour of 0, not 0 itself
  EXPECT_EQ(w.mergeWire(1, {wc(1, 2)}, /*centre=*/0).refused_local, 1);
  EXPECT_EQ(w.status(1), CellStatus::EXPLORING);
}

TEST(CellWorldMerge, MatchingStatusesComposeKnowledge) {
  // A and B agree the cell is COVERED, and B's mask says C has it too. A must
  // come away crediting all three — this is what lets the knowledge gate at
  // N>=3 conclude the team is in agreement without hearing from C.
  CellWorld w = makeWorld();
  selfCover(w, 0);
  EXPECT_EQ(w.cell(0).known_by, robotBit(0));

  const CellWorld::MergeStats st =
      w.mergeWire(1, {wc(0, 2, robotBit(1) | robotBit(2))});
  EXPECT_EQ(st.known_by_only, 1);
  EXPECT_EQ(st.applied, 0) << "agreement is not a status change";
  EXPECT_EQ(w.status(0), CellStatus::COVERED) << "and provenance stays "
                                                 "first-hand";
  EXPECT_EQ(w.cell(0).known_by, robotBit(0) | robotBit(1) | robotBit(2));

  // Nothing new the second time: that is agreed_noop, not a guard refusal.
  const CellWorld::MergeStats st2 =
      w.mergeWire(1, {wc(0, 2, robotBit(1) | robotBit(2))});
  EXPECT_EQ(st2.agreed_noop, 1);
  EXPECT_EQ(st2.known_by_only, 0);
}

TEST(CellWorldMerge, AdoptionResetsTheMaskInsteadOfComposing) {
  // The deliberate asymmetry with the test above. Adopting a status is already
  // taking the peer's word for the status; crediting the peer's third parties
  // on top would compound two levels of hearsay, and over-crediting is the
  // direction that silently disables the knowledge gate.
  CellWorld w = makeWorld();
  const CellWorld::MergeStats st =
      w.mergeWire(1, {wc(0, 2, robotBit(1) | robotBit(2) | robotBit(3))});
  EXPECT_EQ(st.applied, 1);
  EXPECT_EQ(w.cell(0).known_by, robotBit(0) | robotBit(1))
      << "only the two robots that can vouch for this status";
}

TEST(CellWorldMerge, ARetractionFromTheSourceIsHonoured) {
  // Peer 1 said COVERED, then its own hysteresis released the cell and it now
  // says EXPLORING. Full-state broadcast means that retraction is what
  // arrives; refusing it would leave our view of peer 1 permanently unable to
  // come back down.
  CellWorld w = makeWorld();
  ASSERT_EQ(w.mergeWire(1, {wc(0, 2)}).applied, 1);
  ASSERT_EQ(w.status(0), CellStatus::COVERED_BY_OTHERS);
  ASSERT_TRUE(maskHas(w.cell(0).known_by, 1));

  const CellWorld::MergeStats st = w.mergeWire(1, {wc(0, 1)});
  EXPECT_EQ(st.applied, 1);
  EXPECT_EQ(w.status(0), CellStatus::EXPLORING_BY_OTHERS);
}

TEST(CellWorldMerge, AThirdPartyCannotUndoSomeoneElsesCovered) {
  // A retraction from a robot not in known_by is a disagreement between
  // second-hand sources; the more-explored ordering refuses it, so reordering
  // cannot regress the census. Unreachable at N=2, hence this test.
  // (notes: merge-test-third-party-retraction)
  CellWorld w = makeWorld();
  ASSERT_EQ(w.mergeWire(1, {wc(0, 2)}).applied, 1);
  ASSERT_EQ(w.status(0), CellStatus::COVERED_BY_OTHERS);
  ASSERT_FALSE(maskHas(w.cell(0).known_by, 2));

  const CellWorld::MergeStats st = w.mergeWire(2, {wc(0, 1)});
  EXPECT_EQ(st.applied, 0);
  EXPECT_EQ(st.refused_guard, 1);
  EXPECT_EQ(w.status(0), CellStatus::COVERED_BY_OTHERS);
}

TEST(CellWorldMerge, NoRegressionUnderInterleavedLoss) {
  // The plan's no-regression property. Two peers publish full state; messages
  // are delivered in every order and some are dropped. The COVERED count must
  // be monotone non-decreasing across the whole schedule — loss and reordering
  // may DELAY the census, never regress it.
  CellWorld a = makeWorld(), b = makeWorld(), c = makeWorld();
  ASSERT_EQ(b.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                        goodConfig(), 1), "");
  ASSERT_EQ(c.configure(makeCellGrid(-15.0f, 15.0f, -15.0f, 15.0f, 10.0f),
                        goodConfig(), 2), "");
  selfCover(b, 1);
  const std::vector<CellWorld::WireCell> early_b = b.toWire();
  selfCover(b, 2);
  const std::vector<CellWorld::WireCell> late_b = b.toWire();
  selfCover(c, 3);
  const std::vector<CellWorld::WireCell> msg_c = c.toWire();

  // Deliver late-then-early (reordered), c in the middle, early_b twice.
  const std::vector<std::pair<int, const std::vector<CellWorld::WireCell>*>>
      schedule = {{1, &late_b}, {2, &msg_c},   {1, &early_b},
                  {1, &early_b}, {2, &msg_c},  {1, &late_b}};
  int covered = 0;
  for (const auto& [sender, msg] : schedule) {
    a.mergeWire(sender, *msg);
    const CellWorld::Census cen = a.census();
    const int now = cen.covered + cen.covered_by_others;
    EXPECT_GE(now, covered) << "the census regressed under reordering";
    covered = now;
  }
  EXPECT_EQ(covered, 3) << "and it still converged to everything sent";
}

TEST(CellWorldMerge, RefusesRatherThanSilentlyDoingNothing) {
  CellWorld w = makeWorld();      // self id 0
  EXPECT_NE(w.mergeWire(-1, {wc(0, 2)}).refused, "");
  EXPECT_NE(w.mergeWire(kMaxTeamSize, {wc(0, 2)}).refused, "");
  EXPECT_NE(w.mergeWire(0, {wc(0, 2)}).refused, "")
      << "our own relayed census must not re-enter through the path that does "
         "not bump update_id";
  CellWorld unconf;
  EXPECT_NE(unconf.mergeWire(1, {wc(0, 2)}).refused, "");
  // A refusal examines nothing, and says so, so a caller cannot read a
  // zero-count refusal as a peer that agreed with us.
  EXPECT_EQ(w.mergeWire(0, {wc(0, 2)}).examined(), 0);
  EXPECT_EQ(w.status(0), CellStatus::UNSEEN);
}

TEST(CellWorldMerge, CountsGarbageRatherThanCoercingIt) {
  CellWorld w = makeWorld();
  const CellWorld::MergeStats st =
      w.mergeWire(1, {wc(0, 3), wc(1, 4), wc(99, 2), wc(2, 2)});
  EXPECT_EQ(st.bad_status, 2) << "3 and 4 are the local-only statuses: a peer "
                                 "sending them did not normalise";
  EXPECT_EQ(st.out_of_range, 1);
  EXPECT_EQ(st.applied, 1);
  EXPECT_EQ(st.examined(), 4);
  EXPECT_EQ(w.status(0), CellStatus::UNSEEN) << "not coerced into something";
  EXPECT_EQ(w.status(1), CellStatus::UNSEEN);
}

TEST(CellWorldMerge, PeerUnseenChangesNothing) {
  CellWorld w = makeWorld();
  selfExplore(w, 0);
  const CellWorld::MergeStats st = w.mergeWire(1, {wc(0, 0)});
  EXPECT_EQ(st.applied, 0);
  EXPECT_EQ(w.status(0), CellStatus::EXPLORING)
      << "a robot that has not seen a cell is not evidence that nobody has";
}

TEST(CellWorldMerge, SelfObservationReclaimsProvenanceAndResetsTheMask) {
  // Merge then observe: the relayed belief is upgraded to first-hand, which is
  // a committed change, so update_id moves and the mask collapses to self.
  CellWorld w = makeWorld();
  ASSERT_EQ(w.mergeWire(1, {wc(0, 2, robotBit(1) | robotBit(2))}).applied, 1);
  ASSERT_EQ(w.cell(0).known_by, robotBit(0) | robotBit(1));
  selfCover(w, 0);
  EXPECT_EQ(w.status(0), CellStatus::COVERED);
  EXPECT_EQ(w.cell(0).update_id, 1u);
  EXPECT_EQ(w.cell(0).known_by, robotBit(0));
}

TEST(CellWorldIntercept, ShrinksAsTheCellsThePeerKnewAboutChange) {
  CellWorld w = makeWorld();
  // Peer 1 tells us about three cells; we now believe it knows those three.
  ASSERT_EQ(w.mergeWire(1, {wc(0, 1), wc(1, 1), wc(2, 1)}).applied, 3);
  EXPECT_EQ(w.interceptCandidates(1).size(), 3u);
  // Self is in the known_by of every cell just adopted, so without the
  // self-id guard this would be the whole search list for the one robot that
  // cannot be missing.
  EXPECT_TRUE(w.interceptCandidates(0).empty());

  // One of them is finished by us. Nobody is working there any more, so it is
  // no longer somewhere to look for a missing peer.
  selfCover(w, 0);
  EXPECT_EQ(w.interceptCandidates(1).size(), 2u);

  // And a cell whose status we CHANGED drops the peer from its mask entirely
  // (rule 2), which is the whole reason this list can shrink to nothing.
  selfExplore(w, 1);
  const std::vector<int> left = w.interceptCandidates(1);
  ASSERT_EQ(left.size(), 1u);
  EXPECT_EQ(left[0], 2);
  selfCover(w, 2);
  EXPECT_TRUE(w.interceptCandidates(1).empty()) << "nowhere left to look";

  EXPECT_TRUE(w.interceptCandidates(-1).empty()) << "no bit, no claim";
  EXPECT_TRUE(w.interceptCandidates(kMaxTeamSize).empty());
}
