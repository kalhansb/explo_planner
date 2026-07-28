#include <gtest/gtest.h>
#include "explo_planner/candidate_generator.hpp"
#include "explo_planner/map_cache.hpp"
#include <scovox_msgs/msg/scovox_map.hpp>
#include <cmath>

using namespace explo_planner;

namespace {

// Build a MapCache holding the given occupied voxel positions (strongly
// occupied: p_occ ~ 0.91) at 0.1 m resolution via the ScovoxMap ingest path.
MapCache makeOccupiedMap(const std::vector<Eigen::Vector3f>& occupied) {
  scovox_msgs::msg::ScovoxMap msg;
  msg.resolution = 0.1f;
  for (const auto& p : occupied) {
    scovox_msgs::msg::ScovoxVoxel v;
    v.position.x = p.x();
    v.position.y = p.y();
    v.position.z = p.z();
    v.a_occ = 10.0f;
    v.a_free = 1.0f;
    msg.voxels.push_back(v);
  }
  MapCache map(0.1);
  map.updateFromScovoxMap(msg);
  return map;
}

// Occupied column at (x, y): one voxel per 0.1 m cell, cell CENTERS from
// z_from to z_to inclusive.
void addColumn(std::vector<Eigen::Vector3f>& out, float x, float y,
               float z_from, float z_to) {
  for (float z = z_from; z <= z_to + 1e-4f; z += 0.1f)
    out.emplace_back(x, y, z);
}

} // namespace

TEST(CandidateGenerator, GeneratesExpectedCount) {
  CandidateConfig cfg;
  cfg.n_radial = 8;
  cfg.n_rings = 3;
  cfg.n_yaw = 4;
  CandidateGenerator gen(cfg);

  // Empty map: no occupied voxels to filter
  MapCache map(0.1);
  Eigen::Vector3f pos(0, 0, 0);
  auto candidates = gen.generate(pos, 0.0f, &map);

  // Should be n_rings * n_radial * n_yaw = 96 candidates
  // (none filtered since map is empty)
  EXPECT_EQ(candidates.size(), 96u);
}

TEST(CandidateGenerator, CandidatesWithinRadiusBounds) {
  CandidateConfig cfg;
  cfg.n_radial = 8;
  cfg.n_rings = 3;
  cfg.min_radius = 2.0f;
  cfg.max_radius = 8.0f;
  cfg.n_yaw = 1;
  CandidateGenerator gen(cfg);

  MapCache map(0.1);
  Eigen::Vector3f pos(0, 0, 0);
  auto candidates = gen.generate(pos, 0.0f, &map);

  for (const auto& c : candidates) {
    float dist = (c.position.head<2>() - pos.head<2>()).norm();
    EXPECT_GE(dist, cfg.min_radius - 0.01f);
    EXPECT_LE(dist, cfg.max_radius + 0.01f);
  }
}

TEST(CandidateGenerator, YawNormalized) {
  CandidateConfig cfg;
  cfg.n_radial = 4;
  cfg.n_rings = 1;
  cfg.n_yaw = 4;
  CandidateGenerator gen(cfg);

  MapCache map(0.1);
  auto candidates = gen.generate(Eigen::Vector3f(0, 0, 0), 0.0f, &map);

  for (const auto& c : candidates) {
    EXPECT_GE(c.yaw, static_cast<float>(-M_PI) - 0.01f);
    EXPECT_LE(c.yaw, static_cast<float>(M_PI) + 0.01f);
  }
}

TEST(CandidateGenerator, FrontierCandidatesAdded) {
  CandidateConfig cfg;
  cfg.n_radial = 4;
  cfg.n_rings = 1;
  cfg.n_yaw = 1;
  CandidateGenerator gen(cfg);

  MapCache map(0.1);
  Eigen::Vector3f robot(0, 0, 0);
  auto candidates = gen.generate(robot, 0.0f, &map);
  size_t before = candidates.size();

  std::vector<Eigen::Vector3f> frontiers = {
      {5, 5, 0}, {-3, 4, 0}};
  gen.addFrontierCandidates(candidates, frontiers, robot);

  EXPECT_EQ(candidates.size(), before + 2);
}

// ------------------------------------------------------------------
// Terrain-relative (3D) mode
// ------------------------------------------------------------------

TEST(MapCacheGround, GroundZAtReturnsStackTopFace) {
  // Ground stack: cells 0..2 (centers 0.05..0.25) -> top face at 0.3.
  std::vector<Eigen::Vector3f> occ;
  addColumn(occ, 1.0f, 1.0f, 0.05f, 0.25f);
  auto map = makeOccupiedMap(occ);

  float gz = map.groundZAt(1.0f, 1.0f, -1.0f, 1.0f, 0.7f, 0.6f);
  EXPECT_NEAR(gz, 0.3f, 1e-4f);
}

TEST(MapCacheGround, GroundZAtAnchorsLowestNotCanopy) {
  // Ground cell 0 (top face 0.1) + detached canopy at 2.05..2.25. The
  // bottom-up anchor must return the ground, not the canopy.
  std::vector<Eigen::Vector3f> occ;
  addColumn(occ, 0.0f, 0.0f, 0.05f, 0.05f);
  addColumn(occ, 0.0f, 0.0f, 2.05f, 2.25f);
  auto map = makeOccupiedMap(occ);

  float gz = map.groundZAt(0.0f, 0.0f, -1.0f, 3.0f, 0.7f, 0.6f);
  EXPECT_NEAR(gz, 0.1f, 1e-4f);
}

TEST(MapCacheGround, GroundZAtStackCapStopsWallClimb) {
  // Wall column occupied 0.05..1.45 (cells 0..14). stack cap 0.6 m allows
  // 6 cells above the anchor -> top = cell 6 -> top face 0.7.
  std::vector<Eigen::Vector3f> occ;
  addColumn(occ, 0.0f, 0.0f, 0.05f, 1.45f);
  auto map = makeOccupiedMap(occ);

  float gz = map.groundZAt(0.0f, 0.0f, -1.0f, 2.0f, 0.7f, 0.6f);
  EXPECT_NEAR(gz, 0.7f, 1e-4f);
}

TEST(MapCacheGround, GroundZAtNaNWhenNoOccupied) {
  auto map = makeOccupiedMap({{5.0f, 5.0f, 0.05f}});
  // Different column: nothing occupied -> NaN.
  float gz = map.groundZAt(0.0f, 0.0f, -1.0f, 1.0f, 0.7f, 0.6f);
  EXPECT_TRUE(std::isnan(gz));
}

TEST(CandidateGenerator, TerrainRelativeSnapsToGroundPlusClearance) {
  CandidateConfig cfg;
  cfg.n_radial = 8;
  cfg.n_rings = 3;
  cfg.n_yaw = 1;
  cfg.terrain_relative = true;
  cfg.z_clearance = 1.5f;
  CandidateGenerator gen(cfg);

  // Flat ground plane at cell z=0 (top face 0.1) covering the candidate disc.
  std::vector<Eigen::Vector3f> occ;
  for (float x = -9.0f; x <= 9.0f; x += 0.1f)
    for (float y = -9.0f; y <= 9.0f; y += 0.1f)
      occ.emplace_back(x, y, 0.05f);
  auto map = makeOccupiedMap(occ);

  // Robot 1.5 m above the ground.
  auto candidates = gen.generate(Eigen::Vector3f(0, 0, 1.6f), 0.0f, &map);
  ASSERT_FALSE(candidates.empty());
  for (const auto& c : candidates)
    EXPECT_NEAR(c.position.z(), 0.1f + 1.5f, 1e-3f);
}

TEST(CandidateGenerator, TerrainRelativeFallsBackToRobotZ) {
  CandidateConfig cfg;
  cfg.n_radial = 4;
  cfg.n_rings = 1;
  cfg.n_yaw = 1;
  cfg.terrain_relative = true;
  cfg.z_clearance = 1.5f;
  CandidateGenerator gen(cfg);

  MapCache map(0.1);  // empty: no ground anywhere
  auto candidates = gen.generate(Eigen::Vector3f(0, 0, 2.0f), 0.0f, &map);
  ASSERT_FALSE(candidates.empty());
  for (const auto& c : candidates)
    EXPECT_NEAR(c.position.z(), 2.0f, 1e-4f);
}

TEST(CandidateGenerator, TerrainRelativeFrontierUsesOwnZ) {
  CandidateConfig cfg;
  cfg.n_yaw = 1;
  cfg.terrain_relative = true;
  cfg.z_clearance = 1.0f;
  CandidateGenerator gen(cfg);

  // Ground stack under the first frontier: cells 15..17 -> top face 1.8.
  std::vector<Eigen::Vector3f> occ;
  addColumn(occ, 5.0f, 5.0f, 1.55f, 1.75f);
  auto map = makeOccupiedMap(occ);

  std::vector<CandidateViewpoint> candidates;
  Eigen::Vector3f robot(0, 0, 0);
  // First frontier: ground found around its own z -> ground + clearance.
  // Second frontier (no ground in its column) -> keeps its centroid z.
  std::vector<Eigen::Vector3f> frontiers = {
      {5.0f, 5.0f, 2.0f}, {-3.0f, 4.0f, 6.5f}};
  gen.addFrontierCandidates(candidates, frontiers, robot, &map);

  ASSERT_EQ(candidates.size(), 2u);
  EXPECT_NEAR(candidates[0].position.z(), 1.8f + 1.0f, 1e-3f);
  EXPECT_NEAR(candidates[1].position.z(), 6.5f, 1e-4f);
}

TEST(CandidateGenerator, FlatModeKeepsFixedZ) {
  CandidateConfig cfg;  // terrain_relative defaults to false
  cfg.n_radial = 4;
  cfg.n_rings = 1;
  cfg.n_yaw = 1;
  cfg.robot_z = 0.3f;
  CandidateGenerator gen(cfg);

  std::vector<Eigen::Vector3f> occ;
  addColumn(occ, 2.0f, 0.0f, 0.05f, 0.25f);
  auto map = makeOccupiedMap(occ);

  auto candidates = gen.generate(Eigen::Vector3f(0, 0, 1.0f), 0.0f, &map);
  for (const auto& c : candidates)
    EXPECT_NEAR(c.position.z(), 0.3f, 1e-4f);
}

// Terrain-snapped candidate z is clamped into the map's ingest band.
//
// The frontier path references the ground search to the CENTROID's own z (a
// distant frontier can sit metres above the robot), so ground + z_clearance can
// land above the band the map was ingested over — by up to
// ground_search_above + z_clearance. A candidate there has its FOV origin
// outside the observed volume, where every ray walks un-ingested cells and
// scores them as the Beta(1,1) prior, which is maximal: the planner would rank
// its own blind spot as the most informative place to go.
TEST(CandidateGenerator, TerrainCandidateZIsClampedIntoTheIngestBand) {
  // Ground at ~2.1 (top face of the voxel whose centre is 2.05).
  auto map = makeOccupiedMap({{5.0f, 5.0f, 2.05f}});

  CandidateConfig cfg;
  cfg.terrain_relative     = true;
  cfg.z_clearance          = 0.3f;
  cfg.ground_search_below  = 4.0f;
  cfg.ground_search_above  = 1.0f;
  cfg.enable_polar         = false;   // frontier candidates only
  const std::vector<Eigen::Vector3f> centroids{{5.0f, 5.0f, 2.0f}};
  const Eigen::Vector3f robot(0.0f, 0.0f, 0.0f);

  // Unclamped (default +/-1e9 band): ground 2.1 + clearance 0.3 = 2.4.
  {
    CandidateGenerator gen(cfg);
    std::vector<CandidateViewpoint> out;
    gen.addFrontierCandidates(out, centroids, robot, &map);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_NEAR(out[0].position.z(), 2.4f, 1e-4f);
  }

  // Band top at 1.0: the same candidate is pulled down onto it, not left at 2.4.
  {
    CandidateConfig tight = cfg;
    tight.roi_min_z = -1.0f;
    tight.roi_max_z =  1.0f;
    CandidateGenerator gen(tight);
    std::vector<CandidateViewpoint> out;
    gen.addFrontierCandidates(out, centroids, robot, &map);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_NEAR(out[0].position.z(), 1.0f, 1e-4f);
  }

  // setRoiZ re-points the clamp at runtime — the node calls it every PLAN tick
  // because in terrain mode the ingest band rides with the robot.
  {
    CandidateGenerator gen(cfg);
    gen.setRoiZ(-1.0f, 1.5f);
    std::vector<CandidateViewpoint> out;
    gen.addFrontierCandidates(out, centroids, robot, &map);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_NEAR(out[0].position.z(), 1.5f, 1e-4f);
  }

  // A degenerate (inverted) band must not clamp at all — std::clamp is UB when
  // hi < lo, so the guard has to short-circuit before it.
  {
    CandidateConfig inverted = cfg;
    inverted.roi_min_z =  1.0f;
    inverted.roi_max_z = -1.0f;
    CandidateGenerator gen(inverted);
    std::vector<CandidateViewpoint> out;
    gen.addFrontierCandidates(out, centroids, robot, &map);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_NEAR(out[0].position.z(), 2.4f, 1e-4f);
  }
}

// Flat mode is unaffected: candidates sit at the fixed absolute robot_z and the
// band clamp never runs (it exists for the terrain-snapped z only).
TEST(CandidateGenerator, FlatModeIgnoresTheZBand) {
  auto map = makeOccupiedMap({{5.0f, 5.0f, 2.05f}});
  CandidateConfig cfg;
  cfg.terrain_relative = false;
  cfg.robot_z          = 0.3f;
  cfg.enable_polar     = false;
  cfg.roi_min_z        = -1.0f;
  cfg.roi_max_z        =  1.0f;
  CandidateGenerator gen(cfg);

  std::vector<CandidateViewpoint> out;
  gen.addFrontierCandidates(out, {{5.0f, 5.0f, 2.0f}}, Eigen::Vector3f::Zero(),
                            &map);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FLOAT_EQ(out[0].position.z(), 0.3f);
}
