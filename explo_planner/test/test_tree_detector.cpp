#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "explo_planner/tree_detector.hpp"

using namespace explo_planner;

namespace {

TreeDetectorConfig defaultCfg() {
  TreeDetectorConfig c;
  c.voxel_size = 0.2;
  c.veg_class = 5;
  c.occ_thresh = 0.6f;
  c.min_class_conf = 0.3f;
  c.trunk_band_lo = 0.4f;
  c.trunk_band_hi = 3.0f;
  c.min_trunk_voxels = 6;
  c.min_height = 1.0f;
  c.max_radius = 1.0f;
  c.n_azimuth_bins = 12;
  c.n_height_bins = 6;
  c.deficit_thresh = 0.35f;
  return c;
}

// One occupied, confidently-vegetation voxel.
SemVoxel veg(float x, float y, float z, float p_occ = 0.9f, float conf = 0.9f) {
  SemVoxel v;
  v.pos = Eigen::Vector3f(x, y, z);
  v.p_occ = p_occ;
  v.evidence = 20.0f;
  v.best_class = 5;
  v.class_conf = conf;
  return v;
}

// A vertical trunk column at (cx, cy) with vegetation voxels on the azimuths in
// `angles_deg`. A dense angle list => surface seen all around (well-covered); a
// short adjacent list => seen from one side only.
void addTrunk(std::vector<SemVoxel>& out, float cx, float cy, float base_z,
              float radius, float height, const std::vector<float>& angles_deg,
              float voxel = 0.2f, float p_occ = 0.9f) {
  const int nz = static_cast<int>(height / voxel);
  for (int k = 0; k <= nz; ++k) {
    const float z = base_z + k * voxel;
    for (float a : angles_deg) {
      const float r = a * static_cast<float>(M_PI) / 180.0f;
      out.push_back(veg(cx + radius * std::cos(r), cy + radius * std::sin(r), z,
                        p_occ));
    }
  }
}

// Dense ring of azimuths (every 5 deg) — a surface observed from all sides.
std::vector<float> allAzimuths() {
  std::vector<float> a;
  for (int i = 0; i < 72; ++i) a.push_back(i * 5.0f);
  return a;
}

// Geometric-mode config: a LiDAR-only map with empty semantic records.
TreeDetectorConfig geomCfg() {
  TreeDetectorConfig c = defaultCfg();
  c.use_semantics = false;
  return c;
}

// Erase semantic labels from every voxel, as a LiDAR-only ScovoxMap yields
// (best_class / class_conf stay at their zero defaults).
void stripSemantics(std::vector<SemVoxel>& vox) {
  for (auto& v : vox) {
    v.best_class = 0;
    v.class_conf = 0.0f;
  }
}

// Flat plane of unlabeled occupied ground voxels spanning [x0,x1] x [y0,y1].
void addGround(std::vector<SemVoxel>& out, float x0, float x1, float y0,
               float y1, float z = 0.0f, float voxel = 0.2f) {
  for (float x = x0; x <= x1; x += voxel)
    for (float y = y0; y <= y1; y += voxel) {
      SemVoxel v;
      v.pos = Eigen::Vector3f(x, y, z);
      v.p_occ = 0.9f;
      v.evidence = 20.0f;
      out.push_back(v);
    }
}

}  // namespace

TEST(TreeDetector, EmptyInputYieldsNothing) {
  TreeDetector det(defaultCfg());
  EXPECT_TRUE(det.detect({}).empty());
}

// A trunk observed from all sides is well-informed: high coverage, low deficit,
// NOT flagged for exploitation.
TEST(TreeDetector, WellCoveredTrunkNotUnderInformed) {
  std::vector<SemVoxel> vox;
  addTrunk(vox, 3.0f, -2.0f, 0.0f, 0.4f, 2.5f, allAzimuths());
  TreeDetector det(defaultCfg());
  auto d = det.detect(vox);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_GT(d[0].angular_coverage, 0.85f);
  EXPECT_FALSE(d[0].under_informed);
  EXPECT_NEAR(d[0].center.x(), 3.0f, 0.2f);
  EXPECT_NEAR(d[0].center.y(), -2.0f, 0.2f);
  EXPECT_NEAR(d[0].center.z(), 0.0f, 0.1f);   // base z ~ lowest voxel
  EXPECT_NEAR(d[0].radius, 0.4f, 0.15f);
  EXPECT_GT(d[0].height, 2.0f);
}

// The same trunk seen from only ~one quadrant has low angular coverage and a
// high deficit -> flagged as needing to be circled.
TEST(TreeDetector, OneSidedTrunkIsUnderInformed) {
  std::vector<SemVoxel> vox;
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.3f, 2.5f, {0.0f, 30.0f, 60.0f});
  TreeDetector det(defaultCfg());
  auto d = det.detect(vox);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_LT(d[0].angular_coverage, 0.4f);
  EXPECT_GT(d[0].info_deficit, 0.35f);
  EXPECT_TRUE(d[0].under_informed);
}

// Two well-separated trunks segment into two distinct detections.
TEST(TreeDetector, SeparatesTwoTrunks) {
  std::vector<SemVoxel> vox;
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.3f, 2.5f, {0.0f, 30.0f, 60.0f});
  addTrunk(vox, 6.0f, 6.0f, 0.0f, 0.3f, 2.5f, {180.0f, 210.0f, 240.0f});
  TreeDetector det(defaultCfg());
  auto d = det.detect(vox);
  ASSERT_EQ(d.size(), 2u);
}

// A wide occupied slab (a wall/hedge, radius > max_radius) is rejected.
TEST(TreeDetector, RejectsWideBlob) {
  std::vector<SemVoxel> vox;
  for (float x = -1.5f; x <= 1.5f; x += 0.2f)
    for (float z = 0.0f; z <= 2.0f; z += 0.2f)
      vox.push_back(veg(x, 0.0f, z));  // a flat wall spanning 3 m in x
  TreeDetectorConfig c = defaultCfg();
  c.max_radius = 0.6f;
  TreeDetector det(c);
  auto d = det.detect(vox);
  EXPECT_TRUE(d.empty());
}

// Non-vegetation and low-confidence voxels are ignored by the class gate.
TEST(TreeDetector, IgnoresNonVegetation) {
  std::vector<SemVoxel> vox;
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.3f, 2.5f, allAzimuths());
  // Relabel every voxel as building (class 4) -> nothing left to detect.
  for (auto& v : vox) v.best_class = 4;
  TreeDetector det(defaultCfg());
  EXPECT_TRUE(det.detect(vox).empty());
}

// A too-short cluster is rejected by the min_height gate.
TEST(TreeDetector, RejectsShortCluster) {
  std::vector<SemVoxel> vox;
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.3f, 0.4f, allAzimuths());  // 0.4 m tall
  TreeDetector det(defaultCfg());
  EXPECT_TRUE(det.detect(vox).empty());
}

// ============================ Geometric mode ============================
// LiDAR-only maps carry no semantics, so these scenes have best_class = 0
// everywhere and include the ground plane the semantic gate used to remove.

// An unlabeled trunk standing on unlabeled ground is found from shape alone:
// terrain removal strips the plane, the stem-slice cluster passes the PCA
// gates, and the attach step recovers base/height above the ground margin.
TEST(TreeDetectorGeometric, DetectsUnlabeledTrunkOnGround) {
  std::vector<SemVoxel> vox;
  addGround(vox, -3.0f, 3.0f, -3.0f, 3.0f);
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.4f, 2.8f, allAzimuths());
  stripSemantics(vox);
  TreeDetector det(geomCfg());
  auto d = det.detect(vox);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_NEAR(d[0].center.x(), 0.0f, 0.2f);
  EXPECT_NEAR(d[0].center.y(), 0.0f, 0.2f);
  EXPECT_NEAR(d[0].center.z(), 0.4f, 0.25f);  // base ~ ground_margin_m up
  EXPECT_NEAR(d[0].radius, 0.4f, 0.15f);
  EXPECT_GT(d[0].height, 1.8f);
  // r=0.4 quantises to enough distinct ring cells to fill most sectors (same
  // geometry the semantic WellCoveredTrunk test asserts > 0.85 on).
  EXPECT_GT(d[0].angular_coverage, 0.75f);
  EXPECT_FALSE(d[0].under_informed);
}

// The shared ground plane must NOT merge two trunks. stem_slice_lo is zeroed
// so the clustering slice would include any ground the margin gate failed to
// strip: this passes only if terrain removal actually removes the plane (with
// the default slice, ground below stem_slice_lo never reached the clusterer
// and the test could not fail even with terrain removal ablated).
TEST(TreeDetectorGeometric, GroundDoesNotMergeTrunks) {
  std::vector<SemVoxel> vox;
  addGround(vox, -2.0f, 6.0f, -2.0f, 2.0f);
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.3f, 2.8f, allAzimuths());
  addTrunk(vox, 4.0f, 0.0f, 0.0f, 0.3f, 2.8f, allAzimuths());
  stripSemantics(vox);
  TreeDetectorConfig c = geomCfg();
  c.stem_slice_lo = 0.0f;  // terrain removal, not the slice, must exclude ground
  TreeDetector det(c);
  EXPECT_EQ(det.detect(vox).size(), 2u);
}

// Terrain interpolation keeps ground out on a 27 deg slope (grade 0.5). With
// a piecewise-constant terrain lookup, ground leaked past the margin gate at
// grades above ~ground_margin/terrain_cell (~22 deg): bases dropped below the
// true ground and steeper slopes fused trunks with ground ribbons.
TEST(TreeDetectorGeometric, HandlesSlopedTerrain) {
  std::vector<SemVoxel> vox;
  const float grade = 0.5f;
  for (float x = -3.0f; x <= 7.0f; x += 0.2f)
    for (float y = -2.0f; y <= 2.0f; y += 0.2f) {
      SemVoxel v;
      v.pos = Eigen::Vector3f(x, y, grade * x);
      v.p_occ = 0.9f;
      v.evidence = 20.0f;
      vox.push_back(v);
    }
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.4f, 2.8f, allAzimuths());  // ground z ~ 0
  addTrunk(vox, 4.0f, 0.0f, 2.0f, 0.4f, 2.8f, allAzimuths());  // ground z ~ 2
  stripSemantics(vox);
  TreeDetector det(geomCfg());
  auto d = det.detect(vox);
  ASSERT_EQ(d.size(), 2u);
  // Bases stay within [local ground, ground + margin]. The min-z terrain
  // estimate sits ~grade*cell/2 below true ground, so downslope-side trunk
  // voxels right at the base can survive the margin gate (base = ground
  // exactly) — what must never happen is a base BELOW local ground (the old
  // piecewise-constant lookup reported -0.5 here) or ground voxels attaching.
  std::vector<float> zs = {d[0].center.z(), d[1].center.z()};
  std::sort(zs.begin(), zs.end());
  EXPECT_GE(zs[0], -0.05f);
  EXPECT_LE(zs[0], 0.45f);
  EXPECT_GE(zs[1], 1.95f);
  EXPECT_LE(zs[1], 2.45f);
}

// A wall slab: linearity does NOT reject it (a wall longer than its in-slice
// height reads linear along its length) and its median radius sits exactly AT
// max_radius (1.0, not >), so the verticality gate is the sole rejector.
// The second config ablates that gate (and moves the radius knife-edge out of
// the way) to pin the rejection on verticality specifically.
TEST(TreeDetectorGeometric, RejectsWallByShape) {
  std::vector<SemVoxel> vox;
  addGround(vox, -3.0f, 3.0f, -2.0f, 2.0f);
  for (float x = -2.0f; x <= 2.0f; x += 0.2f)
    for (float z = 0.0f; z <= 2.4f; z += 0.2f) {
      SemVoxel v;
      v.pos = Eigen::Vector3f(x, 0.0f, z);
      v.p_occ = 0.9f;
      v.evidence = 20.0f;
      vox.push_back(v);
    }
  stripSemantics(vox);
  TreeDetector det(geomCfg());
  EXPECT_TRUE(det.detect(vox).empty());

  TreeDetectorConfig no_tilt = geomCfg();
  no_tilt.max_tilt_deg = 90.0f;  // ablate the verticality gate
  no_tilt.max_radius = 1.5f;     // and the radius knife-edge (median |x| ~= 1.0)
  EXPECT_FALSE(TreeDetector(no_tilt).detect(vox).empty());
}

// Structure with no occupancy connection to a stem must NOT donate voxels to
// it, even inside attach_radius_m: nearest-axis-only attachment previously let
// a floating cluster (or a shape-rejected wall) inflate height / vertical
// completeness and flip under_informed.
TEST(TreeDetectorGeometric, DoesNotAttachUnconnectedStructure) {
  std::vector<SemVoxel> vox;
  addGround(vox, -3.0f, 3.0f, -3.0f, 3.0f);
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.4f, 2.8f, allAzimuths());
  // Disconnected blob at z ~ 8, XY offset 1.5 m (inside the 2.0 attach radius).
  for (float x = 1.3f; x <= 1.7f; x += 0.2f)
    for (float y = -0.2f; y <= 0.2f; y += 0.2f)
      for (float z = 8.0f; z <= 8.6f; z += 0.2f) {
        SemVoxel v;
        v.pos = Eigen::Vector3f(x, y, z);
        v.p_occ = 0.9f;
        v.evidence = 20.0f;
        vox.push_back(v);
      }
  stripSemantics(vox);
  TreeDetector det(geomCfg());
  auto d = det.detect(vox);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_LT(d[0].height, 3.0f);  // ~2.4 real; 8+ if the blob were attached
}

// The information scoring is mode-independent: a trunk seen from one side only
// still reads under-informed in geometric mode.
TEST(TreeDetectorGeometric, OneSidedTrunkStillUnderInformed) {
  std::vector<SemVoxel> vox;
  addGround(vox, -3.0f, 3.0f, -3.0f, 3.0f);
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.3f, 2.8f, {0.0f, 30.0f, 60.0f});
  stripSemantics(vox);
  TreeDetector det(geomCfg());
  auto d = det.detect(vox);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_LT(d[0].angular_coverage, 0.4f);
  EXPECT_TRUE(d[0].under_informed);
}

// Default (semantic) mode on the same unlabeled scene finds nothing — the
// documented LiDAR-only trap that use_semantics=false exists to fix.
TEST(TreeDetectorGeometric, SemanticModeIgnoresUnlabeledScene) {
  std::vector<SemVoxel> vox;
  addGround(vox, -3.0f, 3.0f, -3.0f, 3.0f);
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.3f, 2.8f, allAzimuths());
  stripSemantics(vox);
  TreeDetector det(defaultCfg());
  EXPECT_TRUE(det.detect(vox).empty());
}
