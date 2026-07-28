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
