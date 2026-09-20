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

// The first of the off-axis LoS tests, and the bearing the four +x ones cannot
// reach.
//
// The four above it — LineOfSightClearOnEmptyMap, LineOfSightBlockedByOccluder,
// TrunkSurfaceIsCarvedOut, OccluderJustBeyondTrunkSurfaceBlocks — all march
// along +x, the one family of bearings where the ray enters exactly one voxel
// per `res` of travel, so stepping the sampler by `res` happens to be
// sufficient and those tests are bit-identical whether the spacing is correct
// or not. Off-axis, a ray enters up to sqrt(3) voxels per `res` (the L1 norm of
// its unit direction) and a `res`-spaced sampler simply never looks at the
// surplus.
//
// This header said "the other four" and "EVERY existing test in this file
// marches along +x" until 2026-09-18, when there were exactly five LoS tests.
// There are eight now and three of the additions are off-axis, immediately
// below this one: LineOfSightDoesNotReadVoxelsTheRayMisses (the over-reading
// half of the guarantee), LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal
// (the exact-corner case) and LineOfSightOnANegativeBearingReadsItsFinalVoxel.
// Counting the file's own tests inside one of them is a claim that goes stale
// on the next TEST() appended, as this one did; what is durable is that the +x
// four cannot discriminate any sampling scheme, which is why this one exists.
//
// Geometry below, worked out at res = 0.1 on the (2,1)/sqrt(5) bearing from
// voxel-centre (0.05, 0.05). A `res`-spaced march samples t = 0.1 in voxel
// (1,0) and t = 0.2 in voxel (2,1) — but the ray crosses y = 0.1 at t = 0.1118
// and x = 0.2 at t = 0.1677, so between those two samples it passes through
// voxel (1,1) and the old sampler never looked. An occluder parked there
// reported CLEAR, sending the exploitation planner to a viewpoint that cannot
// see the trunk it was chosen for.
//
// This voxel is ALSO the case that kills the obvious cheap repair. Spacing the
// march by `res / L1(dir)` — the MEAN voxel-traversal length — does not fix it:
// the ray occupies (1,1) for 0.0559 of travel while that spacing is 0.0745, so
// the sampler lands at t = 0.1 and t = 0.1745 and straddles the window again.
// No fixed spacing can work, because a ray clips a corner for arbitrarily
// little travel; only an exact boundary-to-boundary traversal does. Keep this
// test on this exact bearing: it discriminates between the two, which is the
// whole reason it exists.
TEST(VantagePlanner, LineOfSightOffAxisOccluderIsNotSkipped) {
  VantagePlanner vp(defaultCfg());
  const float inv = 1.0f / std::sqrt(5.0f);
  const Eigen::Vector3f from(0.05f, 0.05f, 0.3f);
  const Eigen::Vector3f dir(2.0f * inv, 1.0f * inv, 0.0f);
  const Eigen::Vector3f center = from + dir * 2.0f;  // trunk 2 m down the ray

  // Control: nothing in the way on this bearing reads clear, so the assertion
  // below is about the occluder and not about the bearing itself.
  {
    scovox_msgs::msg::ScovoxMap empty;
    empty.resolution = 0.1f;
    MapCache map(0.1);
    map.updateFromScovoxMap(empty);
    EXPECT_TRUE(vp.lineOfSightClear(from, center, 0.2f, map));
  }

  // One voxel, in the gap the old sampler stepped over.
  scovox_msgs::msg::ScovoxMap m;
  m.resolution = 0.1f;
  addOccupied(m, 0.15f, 0.15f, 0.3f);
  MapCache map(0.1);
  map.updateFromScovoxMap(m);
  EXPECT_FALSE(vp.lineOfSightClear(from, center, 0.2f, map))
      << "an occluder in a crossed-but-unsampled voxel must block; any FIXED "
         "sample spacing steps over it, so this must be an exact traversal";
}

// The other half of the traversal guarantee, and the failure mode the test
// above cannot see.
//
// "Never miss a crossed voxel" is trivially satisfiable by a sampler that
// reads too much — one that walks the neighbours, or samples exactly on a
// boundary and picks up whichever side rounding lands on. That sampler passes
// LineOfSightOffAxisOccluderIsNotSkipped and is still wrong: it reports every
// vantage blocked whenever anything is parked near the sightline, which
// silently empties the exploitation planner's candidate set instead of
// mis-ranking it. Worse to debug, because nothing is ever chosen badly.
//
// Same bearing, same ray, which has slope dy/dx = 1/2 from (0.05, 0.05): it
// crosses y = 0.1 at x = 0.15 and x = 0.2 at y = 0.125, so the voxel run is
// (1,0) -> (1,1) -> (2,1) -> ... Voxel (0,1), centre (0.05, 0.15), shares an
// edge with the run's start and is not in it; voxel (2,0), centre
// (0.25, 0.05), sits directly under (2,1) and is not in it either. Note that
// (1,0) — centre (0.15, 0.05) — IS crossed and would be the wrong choice here.
TEST(VantagePlanner, LineOfSightDoesNotReadVoxelsTheRayMisses) {
  VantagePlanner vp(defaultCfg());
  const float inv = 1.0f / std::sqrt(5.0f);
  const Eigen::Vector3f from(0.05f, 0.05f, 0.3f);
  const Eigen::Vector3f dir(2.0f * inv, 1.0f * inv, 0.0f);
  const Eigen::Vector3f center = from + dir * 2.0f;

  for (const auto& miss : {Eigen::Vector3f(0.05f, 0.15f, 0.3f),
                           Eigen::Vector3f(0.25f, 0.05f, 0.3f)}) {
    scovox_msgs::msg::ScovoxMap m;
    m.resolution = 0.1f;
    addOccupied(m, miss.x(), miss.y(), miss.z());
    MapCache map(0.1);
    map.updateFromScovoxMap(m);
    EXPECT_TRUE(vp.lineOfSightClear(from, center, 0.2f, map))
        << "voxel at (" << miss.x() << ", " << miss.y() << ") is adjacent to "
           "the ray but not crossed by it; blocking on it makes the sightline "
           "test over-reject";
  }
}

// The exact-corner bearing, which is the only one that reaches the traversal's
// multi-axis advance.
//
// On 45 degrees from a voxel centre, x and y cross their boundaries at the
// same parameter every time: the ray goes corner to corner through (1,1),
// (2,2), (3,3), ... and touches nothing else. What this pins is that the
// traversal visits that diagonal and does NOT read the off-diagonal voxels
// either side of each corner — the direction a sampler errs in if it walks
// neighbours or rounds a corner sample the wrong way.
//
// What it does NOT pin, checked by mutation rather than assumed: replacing the
// all-tied-axes advance with the textbook single-axis `break` form still
// passes. That mutant is equivalent here, not merely uncaught — the axis left
// un-advanced yields a zero-width interval on the next iteration whose
// midpoint IS the corner, and the corner floors into the diagonal voxel the
// ray is entering. It costs a duplicate sample per corner and reads nothing
// new. The loop advances all tied axes anyway because that is the form whose
// invariant ("t_max[a] is always the NEXT crossing ahead of t_enter") can be
// stated, not because a test here would catch the alternative.
TEST(VantagePlanner, LineOfSightAtFortyFiveDegreesCrossesOnlyTheDiagonal) {
  VantagePlanner vp(defaultCfg());
  const float inv = 1.0f / std::sqrt(2.0f);
  const Eigen::Vector3f from(0.05f, 0.05f, 0.3f);
  const Eigen::Vector3f dir(inv, inv, 0.0f);
  const Eigen::Vector3f center = from + dir * 2.0f;

  auto losWithOccluderAt = [&](float x, float y) {
    scovox_msgs::msg::ScovoxMap m;
    m.resolution = 0.1f;
    addOccupied(m, x, y, 0.3f);
    MapCache map(0.1);
    map.updateFromScovoxMap(m);
    return vp.lineOfSightClear(from, center, 0.2f, map);
  };

  EXPECT_FALSE(losWithOccluderAt(0.25f, 0.25f))
      << "voxel (2,2) is on the diagonal and must block";
  EXPECT_FALSE(losWithOccluderAt(0.55f, 0.55f))
      << "voxel (5,5) is further along the same diagonal and must block";
  EXPECT_TRUE(losWithOccluderAt(0.25f, 0.15f))
      << "voxel (2,1) is off the diagonal; a traversal that advances one axis "
         "per corner would walk into it";
  EXPECT_TRUE(losWithOccluderAt(0.15f, 0.25f))
      << "voxel (1,2) is off the diagonal on the other side";
}

// A ray running in the NEGATIVE direction, which no other test in this file
// uses, and the case that makes the midpoint sample load-bearing.
//
// The traversal could sample each interval at its entry instead of its middle,
// and on every other test here that is indistinguishable: the entry is a voxel
// boundary, and floor() keying resolves a boundary to the voxel on its far
// side — which, going positive, is the voxel being ENTERED. Going negative it
// is the voxel being LEFT, so entry-sampling reads the run shifted one voxel
// late and simply never looks at the last voxel before `stop`. A midpoint is
// strictly interior and has no such dependence on which way rounding falls.
//
// Geometry at res = 0.1, bearing (-2,-1)/sqrt(5) from (0.95, 0.95). The
// carve-out puts the first sample at t = 0.1 inside voxel (8,9); the ray
// crosses y = 0.9 at t = 0.1118 and enters (8,8). `stop` is set to 0.14 —
// short of the next crossing — so (8,8) is the FINAL voxel, with no later
// boundary at which a late reader could pick it up by accident.
TEST(VantagePlanner, LineOfSightOnANegativeBearingReadsItsFinalVoxel) {
  VantagePlanner vp(defaultCfg());
  const float inv = 1.0f / std::sqrt(5.0f);
  const Eigen::Vector3f from(0.95f, 0.95f, 0.3f);
  const Eigen::Vector3f dir(-2.0f * inv, -1.0f * inv, 0.0f);
  // dist 0.44, minus radius 0.2 and the one-voxel surface carve-out, is
  // stop = 0.14.
  const Eigen::Vector3f center = from + dir * 0.44f;

  {  // Control: the same short ray over an empty map is clear.
    scovox_msgs::msg::ScovoxMap empty;
    empty.resolution = 0.1f;
    MapCache map(0.1);
    map.updateFromScovoxMap(empty);
    EXPECT_TRUE(vp.lineOfSightClear(from, center, 0.2f, map));
  }

  scovox_msgs::msg::ScovoxMap m;
  m.resolution = 0.1f;
  addOccupied(m, 0.85f, 0.85f, 0.3f);  // voxel (8,8)
  MapCache map(0.1);
  map.updateFromScovoxMap(m);
  EXPECT_FALSE(vp.lineOfSightClear(from, center, 0.2f, map))
      << "the last voxel the ray enters before `stop` must still be read; "
         "sampling each interval at its entry boundary drops it on a negative "
         "bearing";
}
