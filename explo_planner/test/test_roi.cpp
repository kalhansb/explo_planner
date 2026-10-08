#include <gtest/gtest.h>
#include "explo_planner/roi.hpp"
#include <cmath>

using namespace explo_planner;

namespace {

constexpr float kDeg = static_cast<float>(M_PI / 180.0);

// The field ROI from shared_params.yaml: 100 x 25 m, x' along the road
// (25.75 deg in the map frame), origin on the far road edge.
Roi2D siteRoi() {
  return {0.0f, 100.0f, 0.0f, 25.0f, 25.75f * kDeg, -40.30f, -31.54f};
}

}  // namespace

// Defaults are the map frame: every query must be the legacy box test.
TEST(Roi2D, UntransformedIsTheLegacyAxisAlignedBox) {
  const Roi2D r{-2.0f, 3.0f, -1.0f, 4.0f};
  EXPECT_FALSE(r.transformed());
  EXPECT_TRUE(r.contains(-2.0f, -1.0f));   // inclusive corners
  EXPECT_TRUE(r.contains(3.0f, 4.0f));
  EXPECT_FALSE(r.contains(3.0001f, 0.0f));
  EXPECT_FALSE(r.contains(0.0f, -1.0001f));
  const Eigen::Vector2f p = r.toLocal(1.25f, -0.5f);
  EXPECT_EQ(p.x(), 1.25f);
  EXPECT_EQ(p.y(), -0.5f);
  const Roi2D bb = r.mapAabb();
  EXPECT_EQ(bb.min_x, r.min_x);
  EXPECT_EQ(bb.max_x, r.max_x);
  EXPECT_EQ(bb.min_y, r.min_y);
  EXPECT_EQ(bb.max_y, r.max_y);
}

// The site ROI's corners land where the hmr_localisation plot put them, and
// the map origin (the deployment point) is inside it.
TEST(Roi2D, SiteRoiCornersMatchTheMapFrame) {
  const Roi2D r = siteRoi();
  EXPECT_TRUE(r.transformed());
  const Eigen::Vector2f c1 = r.toMap(100.0f, 0.0f);
  const Eigen::Vector2f c2 = r.toMap(100.0f, 25.0f);
  const Eigen::Vector2f c3 = r.toMap(0.0f, 25.0f);
  EXPECT_NEAR(c1.x(), 49.77f, 0.02f);
  EXPECT_NEAR(c1.y(), 11.90f, 0.02f);
  EXPECT_NEAR(c2.x(), 38.91f, 0.02f);
  EXPECT_NEAR(c2.y(), 34.42f, 0.02f);
  EXPECT_NEAR(c3.x(), -51.16f, 0.02f);
  EXPECT_NEAR(c3.y(), -9.02f, 0.02f);

  const Eigen::Vector2f o = r.toLocal(0.0f, 0.0f);
  EXPECT_NEAR(o.x(), 50.0f, 0.02f);
  EXPECT_NEAR(o.y(), 10.9f, 0.02f);
  EXPECT_TRUE(r.contains(0.0f, 0.0f));
}

TEST(Roi2D, ToLocalInvertsToMap) {
  const Roi2D r = siteRoi();
  for (float u : {-5.0f, 0.0f, 37.5f, 100.0f})
    for (float v : {-3.0f, 0.0f, 12.0f, 25.0f}) {
      const Eigen::Vector2f m = r.toMap(u, v);
      const Eigen::Vector2f l = r.toLocal(m.x(), m.y());
      EXPECT_NEAR(l.x(), u, 1e-3f);
      EXPECT_NEAR(l.y(), v, 1e-3f);
    }
}

// Points inside the map-frame bounding box but outside the rotated rectangle
// are rejected — the whole reason for the rotated ROI.
TEST(Roi2D, RejectsBoundingBoxPointsOutsideTheRectangle) {
  const Roi2D r = siteRoi();
  const Roi2D bb = r.mapAabb();
  EXPECT_NEAR(bb.min_x, -51.16f, 0.02f);
  EXPECT_NEAR(bb.max_x, 49.77f, 0.02f);
  EXPECT_NEAR(bb.min_y, -31.54f, 0.02f);
  EXPECT_NEAR(bb.max_y, 34.42f, 0.02f);

  // South-east corner of the bbox: across the road from the ROI.
  EXPECT_FALSE(r.contains(45.0f, -25.0f));
  // North-west corner of the bbox: in front of the building.
  EXPECT_FALSE(r.contains(-45.0f, 30.0f));
  // Just inside / outside the far road edge (v = 0) at u = 50.
  const Eigen::Vector2f in = r.toMap(50.0f, 0.1f);
  const Eigen::Vector2f out = r.toMap(50.0f, -0.1f);
  EXPECT_TRUE(r.contains(in.x(), in.y()));
  EXPECT_FALSE(r.contains(out.x(), out.y()));
}

TEST(Roi2D, DirectionsRotateWithoutTheOffset) {
  const Roi2D r{0.0f, 1.0f, 0.0f, 1.0f, 90.0f * kDeg, 7.0f, -3.0f};
  const Eigen::Vector2f d = r.dirToLocal(0.0f, 1.0f);  // map +y = ROI +x
  EXPECT_NEAR(d.x(), 1.0f, 1e-6f);
  EXPECT_NEAR(d.y(), 0.0f, 1e-6f);
}
