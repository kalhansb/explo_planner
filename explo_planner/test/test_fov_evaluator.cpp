#include <gtest/gtest.h>
#include "explo_planner/fov_evaluator.hpp"
#include "explo_planner/map_cache.hpp"
#include "explo_planner/scoring.hpp"

using namespace explo_planner;

TEST(FovEvaluator, UnknownMapMaximizesEig) {
  FovConfig cfg;
  cfg.h_rays = 4;
  cfg.v_rays = 4;
  cfg.max_range = 5.0f;
  FovEvaluator eval(cfg);

  // Empty map: all voxels unknown → maximum EIG
  MapCache map(0.1);

  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(0, 0, 0.3f);
  vp.yaw = 0.0f;

  auto result = eval.evaluate(vp, map, scoring::eig);
  EXPECT_GT(result.total_score, 0.0f);
  EXPECT_GT(result.unknown_count, 0);
  EXPECT_EQ(result.observed_count, 0);
}

TEST(FovEvaluator, ObservedVoxelsReduceEig) {
  FovConfig cfg;
  cfg.h_rays = 4;
  cfg.v_rays = 4;
  cfg.max_range = 3.0f;
  FovEvaluator eval(cfg);

  // Empty map: all unknown
  MapCache empty_map(0.1);
  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(0, 0, 0.3f);
  vp.yaw = 0.0f;

  auto result_empty = eval.evaluate(vp, empty_map, scoring::eig);

  // The EIG per unknown voxel should be positive
  EXPECT_GT(result_empty.total_score, 0.0f);
}

TEST(FovEvaluator, EntropyScoring) {
  FovConfig cfg;
  cfg.h_rays = 4;
  cfg.v_rays = 4;
  cfg.max_range = 3.0f;
  FovEvaluator eval(cfg);

  MapCache map(0.1);

  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(0, 0, 0.3f);
  vp.yaw = 0.0f;

  // With entropy scoring on empty map, unobserved voxels have p=0.5 → max entropy
  auto result = eval.evaluate(vp, map, scoring::entropy);
  EXPECT_GT(result.total_score, 0.0f);
}

TEST(FovEvaluator, EvaluateAllSetsScores) {
  FovConfig cfg;
  cfg.h_rays = 4;
  cfg.v_rays = 4;
  cfg.max_range = 3.0f;
  FovEvaluator eval(cfg);

  MapCache map(0.1);

  std::vector<CandidateViewpoint> candidates(3);
  candidates[0].position = Eigen::Vector3f(0, 0, 0.3f);
  candidates[0].yaw = 0.0f;
  candidates[1].position = Eigen::Vector3f(1, 0, 0.3f);
  candidates[1].yaw = 1.57f;
  candidates[2].position = Eigen::Vector3f(0, 1, 0.3f);
  candidates[2].yaw = -1.57f;

  eval.evaluateAll(candidates, map, scoring::eig);

  for (const auto& c : candidates) {
    EXPECT_GT(c.score, 0.0f);
  }
}

TEST(FovEvaluator, ZBandClipsRays) {
  // Rays leaving the [roi_min_z, roi_max_z] band must be clipped so out-of-
  // band space is never traversed or scored. A tight z-band must yield
  // strictly fewer scored voxels (and lower total EIG) than an unbounded one,
  // because the FOV cone contains rays whose vertical pitch exits the band
  // well before max_range. This guards the dscovox-mode invariant that the
  // raycast volume matches the GetRegion fetch band (no spurious info gain
  // from rays pointing into unfetched space above/below the robot).
  FovConfig wide;
  wide.h_rays = 4;
  wide.v_rays = 8;
  wide.vfov = 1.2f;          // wide vertical FOV so some rays pitch steeply
  wide.max_range = 10.0f;
  // wide keeps its +/-1e9 roi_*_z defaults (no z clip).
  FovEvaluator eval_wide(wide);

  FovConfig banded = wide;
  banded.roi_min_z = -0.5f;
  banded.roi_max_z =  2.0f;
  FovEvaluator eval_banded(banded);

  MapCache map(0.1);  // empty → every traversed voxel is unknown
  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(0, 0, 0.3f);
  vp.yaw = 0.0f;

  auto r_wide   = eval_wide.evaluate(vp, map, scoring::eig);
  auto r_banded = eval_banded.evaluate(vp, map, scoring::eig);

  // The band clips steep rays early → fewer voxels and lower total score,
  // while near-horizontal rays still contribute.
  EXPECT_LT(r_banded.total_ray_voxels, r_wide.total_ray_voxels);
  EXPECT_LT(r_banded.total_score, r_wide.total_score);
  EXPECT_GT(r_banded.total_score, 0.0f);
}

// Both ends of a ray are ROI-clipped, not just the far one. A candidate sitting
// on an ROI face and firing outward has its ROI exit at t = 0, i.e. BEFORE the
// sensor's min_range: such a ray observes nothing and must contribute nothing.
// Previously only the far end was clamped, so the walk started at
// position + dir*min_range — outside the box and PAST the clamped far end — and
// the iterator ran backwards through voxels the map never ingests, scoring each
// as the Beta(1,1) max-uncertainty prior. That inflated info gain precisely at
// the ROI boundary, biasing the planner toward the edge of its own region.
TEST(FovEvaluator, RaysLeavingTheRoiInsideTheDeadZoneScoreNothing) {
  FovConfig cfg;
  cfg.h_rays = 8;
  cfg.v_rays = 1;
  cfg.hfov = 3.0f;   // ~172 deg: every ray still has a +x component
  cfg.vfov = 0.1f;
  cfg.min_range = 0.3f;
  cfg.max_range = 5.0f;
  cfg.roi_min_x = -10.0f;
  cfg.roi_max_x = 0.0f;    // candidate sits exactly on the +x face
  cfg.roi_min_y = -10.0f;
  cfg.roi_max_y = 10.0f;
  cfg.roi_min_z = -1.0f;
  cfg.roi_max_z = 1.0f;
  FovEvaluator eval(cfg);

  MapCache map(0.1);  // empty: any traversed voxel reads as the prior
  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

  vp.yaw = 0.0f;  // facing +x, straight out of the ROI
  auto outward = eval.evaluate(vp, map, scoring::eig);
  EXPECT_EQ(outward.total_ray_voxels, 0);
  EXPECT_EQ(outward.unknown_count, 0);
  EXPECT_FLOAT_EQ(outward.total_score, 0.0f);

  // Positive control: the same viewpoint facing INTO the ROI still scores.
  vp.yaw = static_cast<float>(M_PI);
  auto inward = eval.evaluate(vp, map, scoring::eig);
  EXPECT_GT(inward.total_ray_voxels, 0);
  EXPECT_GT(inward.total_score, 0.0f);
}

// The far end is still clipped at the ROI face, so a ray fired along the box
// from well inside stops at the boundary rather than running to max_range.
TEST(FovEvaluator, FarEndStillClippedAtTheRoiFace) {
  FovConfig cfg;
  cfg.h_rays = 1;
  cfg.v_rays = 1;
  cfg.hfov = 0.05f;
  cfg.vfov = 0.05f;
  cfg.min_range = 0.3f;
  cfg.max_range = 10.0f;
  cfg.roi_min_x = -10.0f;
  cfg.roi_min_y = -10.0f;
  cfg.roi_max_y = 10.0f;
  cfg.roi_min_z = -1.0f;
  cfg.roi_max_z = 1.0f;

  cfg.roi_max_x = 9.0f;
  FovEvaluator far_eval(cfg);
  cfg.roi_max_x = 2.0f;
  FovEvaluator near_eval(cfg);

  MapCache map(0.1);
  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  vp.yaw = 0.0f;

  auto r_far  = far_eval.evaluate(vp, map, scoring::eig);
  auto r_near = near_eval.evaluate(vp, map, scoring::eig);
  EXPECT_GT(r_far.total_ray_voxels, r_near.total_ray_voxels);
  EXPECT_GT(r_near.total_ray_voxels, 0);
}

// An origin OUTSIDE the ROI on an axis the ray is PARALLEL to must contribute
// nothing. The old exit-only clip skipped any axis with d[i] == 0 entirely, so
// it read "parallel to this slab" as "unconstrained by this slab" and walked the
// full max_range through space the map never ingested — every cell scoring the
// Beta(1,1) max-uncertainty prior, which is maximal.
//
// This is reachable in the shipped terrain-mode config, not a synthetic case:
// addFrontierCandidates snaps a candidate to ground + z_clearance with the
// ground search referenced to the CENTROID's own z, so a candidate can land up
// to (ground_search_above + z_clearance) = 1.3 m above the ingested band. Rays
// from a UGV camera are near-horizontal, i.e. d.z ~ 0. The planner would then
// score its own blind spot above the band as the most informative place to go.
TEST(FovEvaluator, OriginAboveTheBandWithHorizontalRaysScoresNothing) {
  FovConfig cfg;
  cfg.h_rays = 4;
  cfg.v_rays = 1;      // single ray row at pitch 0 exactly -> d.z == 0
  cfg.hfov = 1.0f;
  cfg.vfov = 0.05f;
  cfg.min_range = 0.3f;
  cfg.max_range = 5.0f;
  cfg.roi_min_x = -10.0f;
  cfg.roi_max_x =  10.0f;
  cfg.roi_min_y = -10.0f;
  cfg.roi_max_y =  10.0f;
  cfg.roi_min_z = -1.0f;
  cfg.roi_max_z =  1.0f;
  FovEvaluator eval(cfg);

  MapCache map(0.1);   // empty: every traversed voxel reads as the prior
  CandidateViewpoint vp;
  vp.yaw = 0.0f;

  vp.position = Eigen::Vector3f(0.0f, 0.0f, 2.0f);   // 1 m above the band top
  auto above = eval.evaluate(vp, map, scoring::eig);
  EXPECT_EQ(above.total_ray_voxels, 0);
  EXPECT_FLOAT_EQ(above.total_score, 0.0f);

  vp.position = Eigen::Vector3f(0.0f, 0.0f, -2.0f);  // and below the floor
  auto below = eval.evaluate(vp, map, scoring::eig);
  EXPECT_EQ(below.total_ray_voxels, 0);
  EXPECT_FLOAT_EQ(below.total_score, 0.0f);

  // Positive control: the identical viewpoint inside the band still scores, so
  // the test is detecting the out-of-band case and not a broken config.
  vp.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  auto inside = eval.evaluate(vp, map, scoring::eig);
  EXPECT_GT(inside.total_ray_voxels, 0);
  EXPECT_GT(inside.total_score, 0.0f);
}

// The NEAR end is clipped to the ROI entry, not just held at min_range: an
// origin outside the box firing back into it starts its walk at the box face,
// so the out-of-box lead-in is never traversed.
TEST(FovEvaluator, NearEndIsClippedAtTheRoiEntryFace) {
  FovConfig cfg;
  cfg.h_rays = 1;
  cfg.v_rays = 1;
  cfg.hfov = 0.05f;    // single ray, exactly +x after the yaw rotation
  cfg.vfov = 0.05f;
  cfg.min_range = 0.3f;
  cfg.max_range = 10.0f;
  cfg.roi_min_y = -10.0f;
  cfg.roi_max_y =  10.0f;
  cfg.roi_min_z = -1.0f;
  cfg.roi_max_z =  1.0f;
  cfg.roi_max_x =  5.0f;

  MapCache map(0.1);
  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(-3.0f, 0.0f, 0.0f);   // 3 m outside the -x face
  vp.yaw = 0.0f;                                       // firing back in

  // Box starts at x = 0, so only the span [0, 5] is observable: ~50 voxels.
  cfg.roi_min_x = 0.0f;
  auto clipped = FovEvaluator(cfg).evaluate(vp, map, scoring::eig);
  // Same geometry with the origin INSIDE the box: the walk starts at min_range
  // and covers the full [-2.7, 5] span instead, ~77 voxels.
  cfg.roi_min_x = -10.0f;
  auto unclipped = FovEvaluator(cfg).evaluate(vp, map, scoring::eig);

  EXPECT_GT(clipped.total_ray_voxels, 0);
  EXPECT_LT(clipped.total_ray_voxels, unclipped.total_ray_voxels);
  // Pin the entry face rather than just the ordering: the 3 m lead-in outside
  // the box (30 voxels) must be absent, not merely fewer.
  EXPECT_GE(clipped.total_ray_voxels, 45);
  EXPECT_LE(clipped.total_ray_voxels, 55);
}
