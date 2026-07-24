#include <gtest/gtest.h>
#include "explo_planner/map_cache.hpp"
#include <scovox_msgs/msg/scovox_map.hpp>
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
