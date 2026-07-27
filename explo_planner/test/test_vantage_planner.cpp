#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>
#include <scovox_msgs/msg/scovox_map.hpp>

#include "explo_planner/map_cache.hpp"
#include "explo_planner/vantage_planner.hpp"

using namespace explo_planner;

namespace {

VantageConfig defaultCfg() {
  VantageConfig c;
  c.n_vantages = 3;
  c.standoff_m = 2.0f;
  c.start_angle_rad = 0.0f;
  c.robot_z = 0.3f;
  c.fov_min_range = 0.3f;
  c.fov_max_range = 10.0f;
  c.occ_stop = 0.7f;
  return c;
}

// Add an occupied voxel (a_occ >> a_free -> p_occ ~ 0.9) at (x,y,z).
void addOccupied(scovox_msgs::msg::ScovoxMap& m, float x, float y, float z) {
  scovox_msgs::msg::ScovoxVoxel v;
  v.position.x = x;
  v.position.y = y;
  v.position.z = z;
  v.a_occ = 9.0f;
  v.a_free = 1.0f;
  m.voxels.push_back(v);
}

}  // namespace

TEST(VantagePlanner, GeneratesNViewpoints) {
  VantagePlanner vp(defaultCfg());
  auto vs = vp.generateVantages(Eigen::Vector3f(0, 0, 0), 0.3f);
  EXPECT_EQ(vs.size(), 3u);
  for (const auto& v : vs) EXPECT_TRUE(v.is_vantage);
}

TEST(VantagePlanner, EvenAngularSpacingAndStandoff) {
  VantagePlanner vp(defaultCfg());
  const Eigen::Vector3f c(1.0f, 2.0f, 0.0f);
  const float radius = 0.3f;
  auto vs = vp.generateVantages(c, radius);
  ASSERT_EQ(vs.size(), 3u);

  const float d = vp.standoffFor(radius);
  EXPECT_NEAR(d, radius + 2.0f, 1e-4f);  // 2.3 m, inside [0.6, 10]

  for (size_t i = 0; i < vs.size(); ++i) {
    // Each vantage sits at the standoff distance from the centre (XY).
    const float dx = vs[i].position.x() - c.x();
    const float dy = vs[i].position.y() - c.y();
    EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), d, 1e-3f);
    EXPECT_NEAR(vs[i].position.z(), 0.3f, 1e-6f);

    // Angle of vantage i relative to centre == start_angle + i*120 deg.
    float ang = std::atan2(dy, dx);
    float expected = static_cast<float>(i) * 2.0f * static_cast<float>(M_PI) / 3.0f;
    float diff = std::remainder(ang - expected, 2.0f * static_cast<float>(M_PI));
    EXPECT_NEAR(diff, 0.0f, 1e-3f);
  }
}

TEST(VantagePlanner, YawFacesTrunk) {
  VantagePlanner vp(defaultCfg());
  const Eigen::Vector3f c(0, 0, 0);
  auto vs = vp.generateVantages(c, 0.3f);
  for (const auto& v : vs) {
    // Heading (cos yaw, sin yaw) should point from the vantage to the centre.
    Eigen::Vector2f heading(std::cos(v.yaw), std::sin(v.yaw));
    Eigen::Vector2f to_center(c.x() - v.position.x(), c.y() - v.position.y());
    to_center.normalize();
    EXPECT_NEAR(heading.x(), to_center.x(), 1e-3f);
    EXPECT_NEAR(heading.y(), to_center.y(), 1e-3f);
  }
}

TEST(VantagePlanner, StandoffClamping) {
  // Too close: pushed out to fov_min_range + radius.
  {
    VantageConfig c = defaultCfg();
    c.standoff_m = 0.1f;
    c.fov_min_range = 0.5f;
    VantagePlanner vp(c);
    EXPECT_NEAR(vp.standoffFor(0.0f), 0.5f, 1e-4f);
  }
  // Too far: clamped down to fov_max_range.
  {
    VantageConfig c = defaultCfg();
    c.standoff_m = 100.0f;
    c.fov_max_range = 10.0f;
    VantagePlanner vp(c);
    EXPECT_NEAR(vp.standoffFor(0.0f), 10.0f, 1e-4f);
  }
  // Trunk wider than the sensor envelope (lo > hi): fall back to max_range.
  {
    VantageConfig c = defaultCfg();
    c.fov_min_range = 0.3f;
    c.fov_max_range = 10.0f;
    VantagePlanner vp(c);
    EXPECT_NEAR(vp.standoffFor(20.0f), 10.0f, 1e-4f);
  }
}

TEST(VantagePlanner, LineOfSightClearOnEmptyMap) {
  VantagePlanner vp(defaultCfg());
  MapCache map(0.1);  // empty -> every voxel unknown (p_occ 0.5)
  Eigen::Vector3f from(2.3f, 0.0f, 0.3f);
  Eigen::Vector3f center(0, 0, 0.3f);
  EXPECT_TRUE(vp.lineOfSightClear(from, center, 0.3f, map));
}

TEST(VantagePlanner, LineOfSightBlockedByOccluder) {
  VantagePlanner vp(defaultCfg());
  scovox_msgs::msg::ScovoxMap m;
  m.resolution = 0.1f;
  // A wall of occupied voxels midway between the vantage and the trunk.
  for (float x = 0.8f; x <= 1.2f; x += 0.05f) addOccupied(m, x, 0.0f, 0.3f);
  MapCache map(0.1);
  map.updateFromScovoxMap(m);

  Eigen::Vector3f from(2.3f, 0.0f, 0.3f);
  Eigen::Vector3f center(0, 0, 0.3f);
  EXPECT_FALSE(vp.lineOfSightClear(from, center, 0.3f, map));
}

TEST(VantagePlanner, TrunkSurfaceIsCarvedOut) {
  // An occupied voxel at the trunk surface (within `radius` of the axis) IS the
  // trunk and must not count as an occluder. The carve-out works by stopping the
  // march `radius + one voxel` short of the axis. To make this non-vacuous, the
  // occluder sits ON the sightline at x == radius — a point the march WOULD
  // sample if the radius term were dropped from the stop distance. So this test
  // fails if the carve-out regresses, not merely because the voxel is off-ray.
  VantagePlanner vp(defaultCfg());
  const float radius = 0.5f;
  scovox_msgs::msg::ScovoxMap m;
  m.resolution = 0.1f;
  addOccupied(m, radius, 0.0f, 0.3f);  // dist radius from the axis => the trunk
  MapCache map(0.1);
  map.updateFromScovoxMap(m);

  Eigen::Vector3f from(3.0f, 0.0f, 0.3f);
  Eigen::Vector3f center(0, 0, 0.3f);
  EXPECT_TRUE(vp.lineOfSightClear(from, center, radius, map));
}

TEST(VantagePlanner, OccluderJustBeyondTrunkSurfaceBlocks) {
  // Companion to TrunkSurfaceIsCarvedOut: an occupied voxel just OUTSIDE the
  // carve-out (farther from the axis than radius + one voxel) is a genuine
  // occluder and must block — confirming the carve-out isn't simply swallowing
  // every voxel near the trunk.
  VantagePlanner vp(defaultCfg());
  const float radius = 0.5f;
  scovox_msgs::msg::ScovoxMap m;
  m.resolution = 0.1f;
  // dist 0.7 from the axis > radius(0.5) + one voxel(0.1) => marched, not carved.
  addOccupied(m, 0.7f, 0.0f, 0.3f);
  MapCache map(0.1);
  map.updateFromScovoxMap(m);

  Eigen::Vector3f from(3.0f, 0.0f, 0.3f);
  Eigen::Vector3f center(0, 0, 0.3f);
  EXPECT_FALSE(vp.lineOfSightClear(from, center, radius, map));
}
