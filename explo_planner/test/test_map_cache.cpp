#include <gtest/gtest.h>
#include "explo_planner/map_cache.hpp"
#include <scovox_msgs/msg/scovox_map.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace explo_planner;

namespace {

// Build a MapCache at 0.1 m resolution holding the given voxel positions via
// the ScovoxMap ingest path: `occupied` strongly occupied (p_occ ~ 0.91),
// `free` strongly free (p_occ ~ 0.09). Mirrors test_candidate_generator.
MapCache makeMap(const std::vector<Eigen::Vector3f>& occupied,
                 const std::vector<Eigen::Vector3f>& free = {}) {
  scovox_msgs::msg::ScovoxMap msg;
  msg.resolution = 0.1f;
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
  MapCache map(0.1);
  map.updateFromScovoxMap(msg);
  return map;
}

// The 10x10-column test box: [0, 0.95] x [0, 0.95] at 0.1 m resolution spans
// coords 0..9 on both axes (posToCoord floors), i.e. exactly 100 columns whose
// cell centers are 0.05 + 0.1k, k = 0..9.
constexpr float kBoxMax = 0.95f;
constexpr int kCols = 10;

float center(int k) { return 0.05f + 0.1f * static_cast<float>(k); }

} // namespace

TEST(MapCacheColumnFraction, EmptyGridIsAllUnknown) {
  MapCache map(0.1);
  EXPECT_NEAR(map.unknownColumnFraction(0.0f, kBoxMax, 0.0f, kBoxMax),
              1.0, 1e-9);
}

TEST(MapCacheColumnFraction, FullCoverageIsZero) {
  std::vector<Eigen::Vector3f> occ;
  for (int i = 0; i < kCols; ++i)
    for (int j = 0; j < kCols; ++j)
      occ.emplace_back(center(i), center(j), 0.5f);
  MapCache map = makeMap(occ);
  EXPECT_NEAR(map.unknownColumnFraction(0.0f, kBoxMax, 0.0f, kBoxMax),
              0.0, 1e-9);
}

TEST(MapCacheColumnFraction, HalfCoverageIsHalf) {
  // Fill only the 5 low-x column rows: 50 of 100 columns observed.
  std::vector<Eigen::Vector3f> occ;
  for (int i = 0; i < kCols / 2; ++i)
    for (int j = 0; j < kCols; ++j)
      occ.emplace_back(center(i), center(j), 0.5f);
  MapCache map = makeMap(occ);
  EXPECT_NEAR(map.unknownColumnFraction(0.0f, kBoxMax, 0.0f, kBoxMax),
              0.5, 1e-9);
}

TEST(MapCacheColumnFraction, FreeVoxelsCountAsObserved) {
  // Coverage means "seen", not "occupied": a column holding only a strongly
  // free voxel (swept space) is covered.
  MapCache map = makeMap({}, {{center(3), center(4), 0.5f}});
  EXPECT_NEAR(map.unknownColumnFraction(0.0f, kBoxMax, 0.0f, kBoxMax),
              1.0 - 1.0 / (kCols * kCols), 1e-9);
}

TEST(MapCacheColumnFraction, ColumnStackCountsOnce) {
  // A full vertical stack in one column scores the same as a single voxel.
  std::vector<Eigen::Vector3f> occ;
  for (float z = 0.05f; z <= 1.0f; z += 0.1f)
    occ.emplace_back(center(2), center(7), z);
  MapCache map = makeMap(occ);
  EXPECT_NEAR(map.unknownColumnFraction(0.0f, kBoxMax, 0.0f, kBoxMax),
              1.0 - 1.0 / (kCols * kCols), 1e-9);
}

TEST(MapCacheColumnFraction, VoxelsOutsideBoxIgnored) {
  // Observed columns beyond the box neither shrink nor grow the fraction.
  MapCache map = makeMap({{2.0f, 2.0f, 0.5f}, {-1.0f, 0.5f, 0.5f}});
  EXPECT_NEAR(map.unknownColumnFraction(0.0f, kBoxMax, 0.0f, kBoxMax),
              1.0, 1e-9);
}

TEST(MapCacheColumnFraction, NegativeCoordsCovered) {
  // Box straddling the origin: negative coords pack/count correctly.
  MapCache map = makeMap({{-0.55f, -0.55f, 0.5f}});
  // Box [-0.95, 0.95]^2 -> coords -10..9 -> 20x20 = 400 columns.
  EXPECT_NEAR(map.unknownColumnFraction(-0.95f, kBoxMax, -0.95f, kBoxMax),
              1.0 - 1.0 / 400.0, 1e-9);
}

TEST(MapCacheColumnFraction, DegenerateBoxReturnsMinusOne) {
  MapCache map = makeMap({{0.5f, 0.5f, 0.5f}});
  EXPECT_DOUBLE_EQ(map.unknownColumnFraction(1.0f, 0.0f, 0.0f, 1.0f), -1.0);
  EXPECT_DOUBLE_EQ(map.unknownColumnFraction(0.5f, 0.5f, 0.0f, 1.0f), -1.0);
}

// A non-positive / non-finite voxel size makes Bonxai's inv_resolution inf or
// NaN and every posToCoord then casts a non-finite float to int32 — UB that
// surfaces as garbage coordinates, not a crash. Refuse it at construction the
// way CostGrid::build() refuses a degenerate grid.
TEST(MapCacheIngest, RejectsNonPositiveResolution) {
  EXPECT_THROW(MapCache(0.0), std::invalid_argument);
  EXPECT_THROW(MapCache(-0.1), std::invalid_argument);
  EXPECT_THROW(MapCache(std::numeric_limits<double>::quiet_NaN()),
               std::invalid_argument);
  EXPECT_NO_THROW(MapCache(0.1));
}

// Positions are already screened for NaN/inf; the Beta parameters were not.
// A non-finite a_occ/a_free yields a NaN p_occ (the sum test cannot catch it —
// every comparison against NaN is false, and the division happens anyway),
// which propagates into the per-voxel scorers and the aggregate metrics.
TEST(MapCacheIngest, DropsVoxelsWithNonFiniteBetaParameters) {
  scovox_msgs::msg::ScovoxMap msg;
  msg.resolution = 0.1f;
  auto add = [&msg](float x, float a_occ, float a_free) {
    scovox_msgs::msg::ScovoxVoxel v;
    v.position.x = x;
    v.position.y = 0.0f;
    v.position.z = 0.0f;
    v.a_occ = a_occ;
    v.a_free = a_free;
    msg.voxels.push_back(v);
  };
  const float kNan = std::numeric_limits<float>::quiet_NaN();
  add(0.05f, 10.0f, 1.0f);   // good
  add(0.15f, kNan,  1.0f);   // NaN a_occ
  add(0.25f, 1.0f,  kNan);   // NaN a_free
  add(0.35f, std::numeric_limits<float>::infinity(), 1.0f);
  add(0.45f, -1.0f, 1.0f);   // negative Beta parameter

  MapCache map(0.1);
  map.updateFromScovoxMap(msg);

  // Only the well-formed voxel is ingested; the rest read as unobserved.
  EXPECT_TRUE(map.getVoxel(Eigen::Vector3f(0.05f, 0.0f, 0.0f)).observed);
  for (float x : {0.15f, 0.25f, 0.35f, 0.45f}) {
    const auto v = map.getVoxel(Eigen::Vector3f(x, 0.0f, 0.0f));
    EXPECT_FALSE(v.observed) << "x=" << x;
    EXPECT_TRUE(std::isfinite(v.p_occ)) << "x=" << x;
  }
}

// ==================================================================
// groundZAt — local ground elevation under one column
// ==================================================================
// This is the load-bearing function of terrain_relative_z mode: exploration
// candidates, exploitation vantages and (through the vantage z) every
// line-of-sight occlusion ray are placed at ground + clearance. It had no test
// coverage at all before these.
//
// Contract, from the implementation: coords are voxel corners, so the returned
// elevation is the TOP FACE of the top voxel of the ground stack, i.e.
// (coord_z + 1) * resolution. At 0.1 m a single ground voxel whose centre is
// 0.05 sits in coord 0 and reports 0.1.

namespace {

// One occupied voxel per 0.1 m cell, cell CENTRES from z_from to z_to inclusive.
void addColumn(std::vector<Eigen::Vector3f>& out, float x, float y,
               float z_from, float z_to) {
  for (float z = z_from; z <= z_to + 1e-4f; z += 0.1f)
    out.emplace_back(x, y, z);
}

constexpr float kOcc = 0.7f;

} // namespace

TEST(MapCacheGroundZ, FlatGroundReportsTheTopFaceOfTheGroundVoxel) {
  auto map = makeMap({{0.05f, 0.05f, 0.05f}});
  EXPECT_FLOAT_EQ(map.groundZAt(0.05f, 0.05f, -1.0f, 1.0f, kOcc, 0.6f), 0.1f);
}

TEST(MapCacheGroundZ, ScanIsBottomUpSoAnOverhangAboveIsIgnored) {
  // Ground at 0.05 plus canopy at 3.05 in the same column. The lowest occupied
  // voxel anchors the ground; the canopy must not lift it. This is what keeps a
  // vantage under a tree at head height instead of up in the branches.
  std::vector<Eigen::Vector3f> occ{{0.05f, 0.05f, 0.05f}, {0.05f, 0.05f, 3.05f}};
  auto map = makeMap(occ);
  EXPECT_FLOAT_EQ(map.groundZAt(0.05f, 0.05f, -1.0f, 4.0f, kOcc, 0.6f), 0.1f);
}

// CHARACTERISATION, not an endorsement. With no ground voxel in the column but
// an overhang inside the window (a fallen log, a ledge, a low branch, or simply
// ground that the sensor has not seen yet under something it has), groundZAt
// returns the overhang's top face — a plausible, FINITE, wrong answer. Every
// caller's only defence is std::isfinite, which this passes.
//
// The consequence in terrain mode: the vantage is placed z_clearance above a
// branch, and lineOfSightClear marches its occlusion ray at that height. With
// use_planning_map false, that LoS test is the only real vantage filter, so a
// wrong-but-finite ground silently produces a confidently-wrong capture pose.
// The dwell re-confirms LoS from the settled pose, which limits the damage to a
// wasted vantage rather than a false success, but nothing detects the cause.
// If this behaviour is ever changed (e.g. require a supporting stack, or return
// NaN when the column below the hit is unobserved rather than free), this test
// is the one that should fail and be updated deliberately.
TEST(MapCacheGroundZ, AnOverhangWithNoGroundBelowIsReportedAsGround) {
  auto map = makeMap({{0.05f, 0.05f, 2.05f}});
  const float gz = map.groundZAt(0.05f, 0.05f, -1.0f, 4.0f, kOcc, 0.6f);
  EXPECT_TRUE(std::isfinite(gz));
  EXPECT_FLOAT_EQ(gz, 2.1f);
}

TEST(MapCacheGroundZ, StackCapStopsTheGroundClimbingATrunk) {
  // A 2.1 m contiguous occupied column (trunk / wall). The walk up the
  // contiguous stack is capped at floor(stack_max_m / resolution) voxels, so the
  // reported ground stays near the base instead of jumping to the top.
  std::vector<Eigen::Vector3f> occ;
  addColumn(occ, 0.05f, 0.05f, 0.05f, 2.05f);
  auto map = makeMap(occ);

  // 0.25 m cap -> floor(2.5) = 2 voxels up from the first hit -> top face of
  // coord 2 = 0.3. (0.25 rather than the shipped 0.6 so the floor() is not on a
  // float knife-edge in the test itself.)
  EXPECT_NEAR(map.groundZAt(0.05f, 0.05f, -1.0f, 4.0f, kOcc, 0.25f), 0.3f, 1e-5f);
  // A zero cap degenerates to "the first occupied voxel is the ground".
  EXPECT_NEAR(map.groundZAt(0.05f, 0.05f, -1.0f, 4.0f, kOcc, 0.0f), 0.1f, 1e-5f);
  // A cap wider than the column lets the walk run to the real top (2.1), which
  // is exactly the failure the cap exists to prevent — so the cap, not the
  // geometry, is what bounds the result above.
  EXPECT_NEAR(map.groundZAt(0.05f, 0.05f, -1.0f, 4.0f, kOcc, 3.0f), 2.1f, 1e-5f);
}

TEST(MapCacheGroundZ, GroundOutsideTheSearchWindowIsNotFound) {
  // THE field failure mode the roi_min_z band-floor invariant guards against:
  // ground search is referenced to the robot's z and the window must land inside
  // the ingested slab. Miss it and the answer is "no ground here", which is
  // indistinguishable from genuinely unmapped ground.
  auto map = makeMap({{0.05f, 0.05f, 0.05f}});
  EXPECT_FALSE(std::isfinite(
      map.groundZAt(0.05f, 0.05f, 1.0f, 2.0f, kOcc, 0.6f)));   // window above
  EXPECT_FALSE(std::isfinite(
      map.groundZAt(0.05f, 0.05f, -2.0f, -1.0f, kOcc, 0.6f))); // window below
  // Positive control: a window containing the ground still finds it.
  EXPECT_TRUE(std::isfinite(
      map.groundZAt(0.05f, 0.05f, -1.0f, 1.0f, kOcc, 0.6f)));
}

TEST(MapCacheGroundZ, EmptyColumnAndFreeSpaceAreNotGround) {
  MapCache empty(0.1);
  EXPECT_FALSE(std::isfinite(
      empty.groundZAt(0.05f, 0.05f, -1.0f, 1.0f, kOcc, 0.6f)));

  // A strongly-FREE voxel (p_occ ~ 0.09) is not ground at occ_thresh 0.7.
  auto freemap = makeMap({}, {{0.05f, 0.05f, 0.05f}});
  EXPECT_FALSE(std::isfinite(
      freemap.groundZAt(0.05f, 0.05f, -1.0f, 1.0f, kOcc, 0.6f)));
  // ... but it IS ground once the threshold drops below its p_occ.
  EXPECT_TRUE(std::isfinite(
      freemap.groundZAt(0.05f, 0.05f, -1.0f, 1.0f, 0.05f, 0.6f)));
}

TEST(MapCacheGroundZ, NeighbouringColumnsDoNotLeak) {
  // The scan is one column, selected by the query x/y alone. A ground voxel one
  // cell over must not answer for this column — otherwise a vantage would be
  // placed on its neighbour's terrain.
  auto map = makeMap({{0.05f, 0.05f, 0.05f}});
  EXPECT_TRUE(std::isfinite(
      map.groundZAt(0.05f, 0.05f, -1.0f, 1.0f, kOcc, 0.6f)));
  EXPECT_FALSE(std::isfinite(
      map.groundZAt(0.15f, 0.05f, -1.0f, 1.0f, kOcc, 0.6f)));
  EXPECT_FALSE(std::isfinite(
      map.groundZAt(0.05f, 0.15f, -1.0f, 1.0f, kOcc, 0.6f)));
}

TEST(MapCacheGroundZ, DegenerateAndNonFiniteQueriesReturnNotFinite) {
  auto map = makeMap({{0.05f, 0.05f, 0.05f}});
  const float kNan = std::numeric_limits<float>::quiet_NaN();
  const float kInf = std::numeric_limits<float>::infinity();
  // Empty / inverted window.
  EXPECT_FALSE(std::isfinite(map.groundZAt(0.05f, 0.05f, 0.0f, 0.0f, kOcc, 0.6f)));
  EXPECT_FALSE(std::isfinite(map.groundZAt(0.05f, 0.05f, 1.0f, -1.0f, kOcc, 0.6f)));
  // Non-finite inputs must be rejected before posToCoord, whose float->int32
  // cast is UB on NaN/inf.
  EXPECT_FALSE(std::isfinite(map.groundZAt(kNan, 0.05f, -1.0f, 1.0f, kOcc, 0.6f)));
  EXPECT_FALSE(std::isfinite(map.groundZAt(0.05f, kNan, -1.0f, 1.0f, kOcc, 0.6f)));
  EXPECT_FALSE(std::isfinite(map.groundZAt(0.05f, 0.05f, -kInf, 1.0f, kOcc, 0.6f)));
  EXPECT_FALSE(std::isfinite(map.groundZAt(0.05f, 0.05f, -1.0f, kInf, kOcc, 0.6f)));
}

// ==================================================================
// Ingest rejects an unusable wire resolution instead of trusting it
// ==================================================================
TEST(MapCacheIngest, NonFiniteWireResolutionIsRejectedAndKeepsThePreviousGrid) {
  // A positive-but-infinite msg.resolution passed the old `> 0.0f` guard
  // straight into the Grid, making inv_resolution zero and every posToCoord a
  // float->int32 cast of a non-finite value: UB that shows up as garbage
  // coordinates, not a crash. Reject the MESSAGE (this arrives over the wire
  // mid-run) rather than throwing, so one malformed publish cannot take the
  // planner down in the field.
  scovox_msgs::msg::ScovoxMap good;
  good.resolution = 0.1f;
  scovox_msgs::msg::ScovoxVoxel v;
  v.position.x = 0.05f;
  v.position.y = 0.05f;
  v.position.z = 0.05f;
  v.a_occ = 10.0f;
  v.a_free = 1.0f;
  good.voxels.push_back(v);

  MapCache map(0.1);
  EXPECT_TRUE(map.updateFromScovoxMap(good));
  const size_t before = map.voxelCount();
  EXPECT_GT(before, 0u);

  scovox_msgs::msg::ScovoxMap bad = good;
  bad.resolution = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(map.updateFromScovoxMap(bad));
  // Previous grid intact: still queryable, still the same size, resolution
  // unchanged.
  EXPECT_EQ(map.voxelCount(), before);
  // The wire field is a float, so the stored double is static_cast<double>(0.1f).
  EXPECT_NEAR(map.resolution(), 0.1, 1e-6);
  EXPECT_TRUE(map.getVoxel(Eigen::Vector3f(0.05f, 0.05f, 0.05f)).observed);

  // A non-positive or NaN resolution is not an error: it means "unspecified",
  // and ingest falls back to the cache's current resolution.
  scovox_msgs::msg::ScovoxMap unspecified = good;
  unspecified.resolution = 0.0f;
  EXPECT_TRUE(map.updateFromScovoxMap(unspecified));
  // The wire field is a float, so the stored double is static_cast<double>(0.1f).
  EXPECT_NEAR(map.resolution(), 0.1, 1e-6);
  unspecified.resolution = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(map.updateFromScovoxMap(unspecified));
  // The wire field is a float, so the stored double is static_cast<double>(0.1f).
  EXPECT_NEAR(map.resolution(), 0.1, 1e-6);
}
