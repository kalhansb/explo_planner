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
// For the tests that compare against MapCache::unknownColumnFraction, the
// resolution has to be a power of two. `msg.resolution` is a float, so 0.1f is
// really 0.10000000149011612 and Bonxai's inv_resolution is 9.99999985, not
// its exact inverse — posToCoord(1.0) is then 9, not 10, and a cell's
// "equivalent box" picks up one column belonging to its neighbour. That is a
// property of unknownColumnFraction's inclusive-bounds convention rather than a
// disagreement about what an observed column is, and testing at 0.125 removes
// it so the comparison is about the thing it claims to be about.
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
  // The thresholds are world-calibrated knobs, in the same family as
  // done_unknown_fraction (shipped 0.05, overridden to 0.64 by the sim
  // harness). Both measure "columns with nothing observed in them", and both
  // have a floor above zero in any world where some columns can never be
  // observed. There is therefore no universally correct value, and the thing
  // worth pinning is not the numbers but the world model they encode.
  //
  // What this test asserts is that the defaults belong to the SAME world model
  // as the shipped done_unknown_fraction: a map that saturates, where a swept
  // cell really does approach zero unknown. Raising them to fit one particular
  // world — which is what the flat-forest sim needs, and what an earlier
  // version of this file did — silently redefines COVERED for every other
  // deployment, so it belongs in that world's harness and not here.
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
  // Deliberately NOT the exact threshold. 85 of 100 columns is an unknown
  // fraction of 1.0 - 0.85, which is 0.15000000000000002 in doubles and so
  // lands on the EXPLORING side of a 0.15 threshold. Behaviour exactly at a
  // threshold on a continuous measure is not a contract this class offers, and
  // pinning it in a test would only record which way the rounding happened to
  // fall for one pair of numbers.
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
  // The regression this replaced. The veto used to be an absolute voxel count
  // defaulting to 4, which is not a quantity that can be set correctly: it
  // means something different at every cell size and map resolution, and at
  // the shipped 10 m cells and 0.1 m voxels a thoroughly swept cell holds tens
  // of thousands of voxels and hundreds of boundary ones. Every cell was
  // vetoed for an entire 1800 s run.
  //
  // So the property under test is scale invariance: the same SHAPE of cell,
  // measured at two very different sizes, must classify the same way. A count
  // threshold fails this by construction, which is the point.
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
  // frontierFraction() reports "cannot measure" as -1.0, and -1.0 sails under
  // any threshold. A cell with observed columns but no observed voxels cannot
  // arise from censusFromMap, so this is guarding the seam rather than a
  // reachable state — which is exactly the kind of check that gets dropped as
  // impossible and then becomes reachable when a second producer appears.
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
  // Unknown fraction 0.25: above covered_max_unknown (0.15) so it would not be
  // promoted, but below exploring_min_unknown (0.35) so it is not demoted.
  // Without this band a cell hovering at the threshold re-commits every tick,
  // and since every commit resets known_by, the knowledge gate downstream
  // would never suppress anything.
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
  // The float-narrowing hazard, at a configuration where it actually bites.
  //
  // A voxel column's low corner is c * resolution in double, but CellGrid works
  // in float. At resolution 0.05 and a ROI starting at -15, the column at a
  // cell boundary is -15.000000223517418 in double and exactly -15.0f in
  // float, so the two floor() to different cells. If the per-cell column count
  // (the denominator) and the observed-column tally (the numerator) computed
  // that independently, one cell would be short a column: unknown fractions on
  // the affected cells would sit slightly below the truth, which reads as a
  // threshold that needs tuning rather than as a bug.
  //
  // The configuration matters. At 0.05 m columns over 5 m cells the boundaries
  // diverge unevenly and one cell in each axis loses a column; at 0.1 m over
  // 1 m cells every boundary diverges the same way, the windows shift as a
  // block, and the counts come out identical — which is why the other
  // fixtures here cannot see this and this one can.
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
  // The float-narrowing hazard the shared binning helper exists for: at
  // min_x = -15 and res = 0.2 the boundary column is -15.000000000000002 in
  // double but exactly -15.0f in float. If the denominator and the numerator
  // disagreed about that column, the west and south edge cells would report an
  // unknown fraction slightly below the truth.
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
