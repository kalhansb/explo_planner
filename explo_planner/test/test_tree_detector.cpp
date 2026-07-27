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
