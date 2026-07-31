#include <gtest/gtest.h>

#include <cmath>
#include <limits>
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

// A trunk surface with THICKNESS and jitter, not the idealised single-radius
// shell addTrunk() lays down. This is what a real fused voxel map holds: the
// surface spans a couple of voxels radially and the returns are noisy. It is
// also the case the thin-shell tests could not expose -- see
// OneSidedThickTrunkIsUnderInformed below.
void addThickTrunk(std::vector<SemVoxel>& out, float cx, float cy, float base_z,
                   float radius, float height, float a_lo_deg, float a_hi_deg,
                   float voxel = 0.2f) {
  const int nz = static_cast<int>(height / voxel);
  // Deterministic pseudo-jitter: no <random>, so the test stays reproducible
  // bit-for-bit across platforms.
  uint32_t s = 12345u;
  auto rnd = [&s]() {
    s = s * 1664525u + 1013904223u;
    return static_cast<float>((s >> 8) & 0xFFFF) / 65535.0f - 0.5f;
  };
  for (int k = 0; k <= nz; ++k) {
    const float z = base_z + k * voxel;
    for (float a = a_lo_deg; a <= a_hi_deg; a += 4.0f) {
      const float r = a * static_cast<float>(M_PI) / 180.0f;
      // Two radial shells plus jitter => a thick, noisy surface.
      for (float dr : {-0.5f * voxel, 0.5f * voxel}) {
        const float rr = radius + dr + 0.3f * voxel * rnd();
        out.push_back(veg(cx + rr * std::cos(r), cy + rr * std::sin(r), z));
      }
    }
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

// A non-finite p_occ from an upstream producer must not poison the score.
// The occupancy gates use `p_occ < occ_thresh`, which is FALSE for NaN, so a
// poisoned voxel is kept rather than skipped and reaches the entropy sum. If
// normEntropy let it through, mean_entropy -> info_deficit would go NaN, the
// `deficit > deficit_thresh` test would compare false (the tree silently reads
// well-observed and is never targeted), and the std::sort comparator on
// info_deficit would stop being a strict weak ordering. Both must hold: the
// outputs stay finite, AND the one-sided trunk stays flagged.
TEST(TreeDetector, NonFinitePOccDoesNotPoisonTheScore) {
  std::vector<SemVoxel> vox;
  addTrunk(vox, 0.0f, 0.0f, 0.0f, 0.3f, 2.5f, {0.0f, 30.0f, 60.0f});
  // Poison one height layer INSIDE the trunk band (defaultCfg: base + 0.4 ..
  // base + 3.0) — entropy is summed over the band, not the whole cluster, and
  // adjacent azimuths on a 0.3 m radius share voxel coords and get deduped, so
  // poisoning an arbitrary index can silently miss the code path.
  int poisoned = 0;
  for (auto& v : vox) {
    if (std::abs(v.pos.z() - 1.2f) < 0.05f) {
      v.p_occ = std::numeric_limits<float>::quiet_NaN();
      ++poisoned;
    }
  }
  ASSERT_GT(poisoned, 0);

  TreeDetector det(defaultCfg());
  auto d = det.detect(vox);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_TRUE(std::isfinite(d[0].mean_entropy));
  EXPECT_TRUE(std::isfinite(d[0].info_deficit));
  EXPECT_TRUE(d[0].under_informed);
}

// Two well-separated trunks segment into two distinct detections.
// REGRESSION (map-test-2 bag): a trunk seen from one side only must read low
// angular coverage even when its surface is thick and noisy rather than an
// idealised single-radius arc.
//
// The thin-shell OneSidedTrunkIsUnderInformed above passed throughout the
// period this was broken, because with a one-voxel-thick arc even a biased
// centre leaves the azimuths clustered. Give the surface real thickness and the
// old per-component-median centre lands ON the arc, the voxels fan out around
// it through every sector, and coverage reads ~1.0 -- a half-observed tree
// scoring as fully covered. On the real map that pinned 13 of 17 trunks at
// coverage 1.00 and made the emission gate arithmetically unreachable.
TEST(TreeDetector, OneSidedThickTrunkIsUnderInformed) {
  std::vector<SemVoxel> vox;
  addThickTrunk(vox, 1.0f, 2.0f, 0.0f, 0.4f, 3.0f, -60.0f, 60.0f);
  const auto d = TreeDetector(defaultCfg()).detect(vox);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_TRUE(d[0].axis_fitted);
  // 120 deg of 360 was observed, so the ideal reading is 0.33. The measured
  // value is 0.50: fitting a circle to a partial arc whose radial noise is a
  // sizeable fraction of its radius (a 0.4 m trunk on a 0.2 m grid -- the real
  // regime, not a pathological one) still biases the centre slightly toward the
  // arc, which spreads the azimuths wider than the arc truly spans. That
  // residual bias is why the node prefers bearing coverage, which measures
  // viewing geometry directly instead of inferring it from surface shape.
  //
  // The value this test actually pins down is that it is no longer ~1.0. Before
  // the fit it read 1.00 here, i.e. "fully circled" for a trunk seen from 120
  // degrees, which is what made the emission gate unreachable on real maps.
  EXPECT_LT(d[0].angular_coverage, 0.6f);
  EXPECT_TRUE(d[0].under_informed);
  // The fit must recover the true axis, not the centroid of the observed arc
  // (which sits ~0.4 m away, at the arc itself).
  EXPECT_NEAR(d[0].center.x(), 1.0f, 0.15f);
  EXPECT_NEAR(d[0].center.y(), 2.0f, 0.15f);
  EXPECT_NEAR(d[0].radius, 0.4f, 0.1f);
}

// The other half of the loop: once the same trunk has been circled, coverage
// saturates and it stops being nominated. Without this the fix could pass the
// test above by simply reading low coverage for everything.
TEST(TreeDetector, CircledThickTrunkIsNotUnderInformed) {
  std::vector<SemVoxel> vox;
  addThickTrunk(vox, 1.0f, 2.0f, 0.0f, 0.4f, 3.0f, 0.0f, 356.0f);
  const auto d = TreeDetector(defaultCfg()).detect(vox);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_TRUE(d[0].axis_fitted);
  EXPECT_GT(d[0].angular_coverage, 0.95f);
  EXPECT_FALSE(d[0].under_informed);
  EXPECT_NEAR(d[0].center.x(), 1.0f, 0.1f);
  EXPECT_NEAR(d[0].center.y(), 2.0f, 0.1f);
  EXPECT_NEAR(d[0].radius, 0.4f, 0.1f);
}

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

// ===================================================================
// Emission gate (bearing coverage)
// ===================================================================

namespace {

/// Fold a robot standing at `deg` around a trunk at the origin into `g`.
float viewFrom(EmitGate& g, float deg, int n_bins = 8) {
  const float r = deg * static_cast<float>(M_PI) / 180.0f;
  const Eigen::Vector2f robot(10.0f * std::cos(r), 10.0f * std::sin(r));
  return observeBearing(g, bearingBit(Eigen::Vector2f::Zero(), robot, n_bins),
                        n_bins);
}

}  // namespace

// Each 45-degree sector is one bit, and opposite bearings are distinct bits.
TEST(EmitGate, BearingBitPartitionsTheCircle) {
  const Eigen::Vector2f c = Eigen::Vector2f::Zero();
  EXPECT_EQ(bearingBit(c, Eigen::Vector2f(1.0f, 0.0f), 8), 1u << 0);
  EXPECT_EQ(bearingBit(c, Eigen::Vector2f(0.0f, 1.0f), 8), 1u << 2);
  EXPECT_EQ(bearingBit(c, Eigen::Vector2f(-1.0f, 0.0f), 8), 1u << 4);
  EXPECT_EQ(bearingBit(c, Eigen::Vector2f(0.0f, -1.0f), 8), 1u << 6);
  // A bearing a hair below zero wraps to (just under) 2*pi, which rounds to
  // exactly 2*pi in float and would index bin n. Unclamped that is a shift of
  // 32 -- undefined behaviour -- so it must land in the last bin instead.
  EXPECT_EQ(bearingBit(c, Eigen::Vector2f(1.0f, -1e-7f), 8), 1u << 7);
  EXPECT_FLOAT_EQ(bearingCoverage(0u, 8), 0.0f);
  EXPECT_FLOAT_EQ(bearingCoverage(0xFFu, 8), 1.0f);
  EXPECT_FLOAT_EQ(bearingCoverage(0x0Fu, 8), 0.5f);
}

// Re-observing a tree from a sector it has already been seen from does not
// raise coverage, and each new sector restarts the settle countdown.
TEST(EmitGate, SettleCountsScansWithoutANewSector) {
  EmitGate g;
  EXPECT_FLOAT_EQ(viewFrom(g, 0.0f), 1.0f / 8.0f);
  EXPECT_EQ(g.settle, 0);          // first sector IS growth
  EXPECT_FLOAT_EQ(viewFrom(g, 10.0f), 1.0f / 8.0f);  // same sector
  EXPECT_EQ(g.settle, 1);
  EXPECT_FLOAT_EQ(viewFrom(g, 20.0f), 1.0f / 8.0f);
  EXPECT_EQ(g.settle, 2);
  EXPECT_FLOAT_EQ(viewFrom(g, 90.0f), 2.0f / 8.0f);  // new sector
  EXPECT_EQ(g.settle, 0);                            // countdown restarts
}

// The core of limitation #9: a tree the robot is still walking past must not be
// emitted, however under-informed it reads, until its bearing history goes
// quiet. Then it fires exactly once.
TEST(EmitGate, DefersEmissionUntilTheBearingHistorySettles) {
  EmitGate g;
  // Three scans while the robot sweeps past: a new sector every scan.
  for (float deg : {0.0f, 50.0f, 100.0f}) {
    viewFrom(g, deg);
    EXPECT_FALSE(stepEmitGate(g, /*under_informed=*/true, /*has_bearing=*/true,
                              /*confirm_ticks=*/2, /*settle_ticks=*/3));
  }
  EXPECT_GE(g.confirm, 2);  // confirmation is satisfied; settling is not
  // Robot has moved off: same sector from here on.
  viewFrom(g, 105.0f);
  EXPECT_FALSE(stepEmitGate(g, true, true, 2, 3));
  viewFrom(g, 107.0f);
  EXPECT_FALSE(stepEmitGate(g, true, true, 2, 3));
  viewFrom(g, 109.0f);
  EXPECT_TRUE(stepEmitGate(g, true, true, 2, 3));  // settle == 3
  // Emit-once: still under-informed and still settled, but silent.
  viewFrom(g, 110.0f);
  EXPECT_FALSE(stepEmitGate(g, true, true, 2, 3));
}

// A tree the robot walks all the way around fills enough sectors that it stops
// reading under-informed, and closes itself without ever being nominated —
// the behaviour the deferral exists to make possible.
TEST(EmitGate, ACircledTreeClosesItselfWithoutBeingEmitted) {
  TreeDetectorConfig cfg;  // stock weights: w_coverage 0.75, w_entropy 0.25
  EmitGate g;
  bool emitted = false;
  for (float deg = 0.0f; deg < 360.0f; deg += 20.0f) {
    const float cov = viewFrom(g, deg);
    const float deficit = infoDeficit(cfg, cov, /*mean_entropy=*/0.2f,
                                      /*vertical=*/1.0f);
    emitted |= stepEmitGate(g, deficit > cfg.deficit_thresh, true, 2, 3);
  }
  EXPECT_FLOAT_EQ(bearingCoverage(g.bearing_mask, 8), 1.0f);
  EXPECT_FALSE(emitted);
}

// Without a pose there is no bearing history to settle, so the gate falls back
// to plain confirmation on the detector's map-geometry verdict.
TEST(EmitGate, NoPoseMeansNoDeferral) {
  EmitGate g;
  EXPECT_FALSE(stepEmitGate(g, true, /*has_bearing=*/false, 2, 3));
  EXPECT_TRUE(stepEmitGate(g, true, false, 2, 3));
}

// settle_ticks <= 0 restores the pre-deferral behaviour: emit as soon as the
// confirmation streak is met.
TEST(EmitGate, ZeroSettleTicksDisablesTheWait) {
  EmitGate g;
  viewFrom(g, 0.0f);
  EXPECT_FALSE(stepEmitGate(g, true, true, 2, 0));
  viewFrom(g, 90.0f);  // sector still growing
  EXPECT_TRUE(stepEmitGate(g, true, true, 2, 0));
}

// A well-observed read resets the confirmation streak; the bearing history it
// accumulated is kept, because the robot really did view it from there.
TEST(EmitGate, WellObservedReadResetsConfirmationNotBearings) {
  EmitGate g;
  viewFrom(g, 0.0f);
  EXPECT_FALSE(stepEmitGate(g, true, true, 3, 0));
  viewFrom(g, 5.0f);
  EXPECT_FALSE(stepEmitGate(g, true, true, 3, 0));
  viewFrom(g, 10.0f);
  EXPECT_FALSE(stepEmitGate(g, /*under_informed=*/false, true, 3, 0));
  EXPECT_EQ(g.confirm, 0);
  EXPECT_EQ(g.bearing_mask, 1u << 0);
  // Streak restarts from scratch: two more under-informed scans are not enough.
  viewFrom(g, 12.0f);
  EXPECT_FALSE(stepEmitGate(g, true, true, 3, 0));
  viewFrom(g, 14.0f);
  EXPECT_FALSE(stepEmitGate(g, true, true, 3, 0));
  viewFrom(g, 16.0f);
  EXPECT_TRUE(stepEmitGate(g, true, true, 3, 0));
}
