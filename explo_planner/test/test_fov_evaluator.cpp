// Moved comments: doc/explo_planner_code_notes.md
#include <gtest/gtest.h>
#include "explo_planner/fov_evaluator.hpp"
#include "explo_planner/map_cache.hpp"
#include "explo_planner/scoring.hpp"
#include <scovox_msgs/msg/scovox_map.hpp>
#include <vector>

using namespace explo_planner;

// Resolution shared by the T1 occlusion tests at the bottom of this file. The
// tests above construct their own MapCache(0.1) inline; this is the same number,
// named once so the voxel-centre arithmetic and the ingest message cannot drift
// apart from the cache they are built for.
constexpr double kFovTestRes = 0.1;

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
  // Rays leaving the [roi_min_z, roi_max_z] band are clipped, so a tight band
  // scores fewer voxels and less EIG: the raycast volume must match the map's z
  // band, with no info gain from unfetched space. (notes: fov-test-z-band-clip)
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

// Both ends of a ray are ROI-clipped: a candidate on an ROI face firing outward
// exits at t = 0, before min_range, and must score nothing. Out-of-ROI voxels
// would otherwise score as the Beta(1,1) prior. (notes: fov-roi-clip-both-ends)
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

// An origin outside the ROI on an axis the ray is parallel to must score
// nothing: treating that axis as unconstrained walks the full max_range through
// un-ingested space at the maximal Beta(1,1) prior.
// (notes: fov-roi-clip-parallel-axis)
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

// ---------------------------------------------------------------------------
// T1. Occlusion semantics, pinned directly.
//
// The ROI-clipping tests above use an empty MapCache, so they hold only while
// unknown voxels stay transparent: the ptr && guard in the occ_stop check in
// fov_evaluator.cpp. The shipped occ_stop masks its loss.
// (notes: fov-unknown-voxels-transparent)
//
// GEOMETRY. h_rays = v_rays = 1 puts the single ray exactly on the candidate
// yaw at zero pitch; with yaw = 0 it runs along +x, so voxel counts are exact.
// (notes: fov-test-single-ray-geometry)
namespace {

// One voxel at `pos` with the given Beta parameters, ingested through the same
// ScovoxMap path the node uses. p_occ = a_occ / (a_occ + a_free).
MapCache makeOneVoxelMap(const std::vector<Eigen::Vector3f>& positions,
                         float a_occ, float a_free) {
  scovox_msgs::msg::ScovoxMap msg;
  msg.resolution = static_cast<float>(kFovTestRes);
  for (const auto& p : positions) {
    scovox_msgs::msg::ScovoxVoxel v;
    v.position.x = p.x();
    v.position.y = p.y();
    v.position.z = p.z();
    v.a_occ = a_occ;
    v.a_free = a_free;
    msg.voxels.push_back(v);
  }
  MapCache map(kFovTestRes);
  map.updateFromScovoxMap(msg);
  return map;
}

// Centre of voxel coord k on any axis. Cell centres, never faces: a ray walked
// along a cell boundary makes the traversed set ambiguous, and the whole point
// of these tests is an exact count.
float vcenter(int k) {
  return static_cast<float>(0.5 * kFovTestRes + kFovTestRes * k);
}

// The single-ray configuration. min_range 0.3 / max_range 1.0 at 0.1 m puts the
// walk's first voxel at coord 3 (0.35 -> floor 3.5) and its last at coord 10
// (1.05 -> floor 10.5), both mid-cell.
FovConfig oneRayCfg() {
  FovConfig cfg;
  cfg.h_rays = 1;
  cfg.v_rays = 1;
  cfg.min_range = 0.3f;
  cfg.max_range = 1.0f;
  cfg.occ_stop = 0.7f;
  return cfg;
}

CandidateViewpoint originVp() {
  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(vcenter(0), vcenter(0), vcenter(0));
  vp.yaw = 0.0f;
  return vp;
}

// The walk's first voxel, and the coord holding ray_end.
constexpr int kFirstCoord = 3;    // floor(0.35 / 0.1)
constexpr int kEndCoord = 10;     // floor(1.05 / 0.1)

// scovox::RayIterator is half-open: it stops before the end coord, so the last
// scored voxel is kEndCoord - 1 and the voxel at max_range is never scored.
// Place occluders at kLastVisited. (notes: fov-ray-iterator-half-open)
constexpr int kLastVisited = kEndCoord - 1;                    // 9
constexpr int kClearSpan = kLastVisited - kFirstCoord + 1;     // 7

} // namespace

// The geometry every count-based test below is written against, asserted once on
// its own. If RayIterator's endpoint convention or posToCoord's rounding ever
// moves, this fails alone and names the reason, instead of six tests failing
// together with no indication of which shared assumption broke.
TEST(FovOcclusion, SingleRaySpanIsTheExpectedVoxelCount) {
  FovEvaluator eval(oneRayCfg());
  MapCache empty(kFovTestRes);
  auto r = eval.evaluate(originVp(), empty, scoring::eig);
  EXPECT_EQ(r.total_ray_voxels, kClearSpan)
    << "expected the half-open walk over coords [" << kFirstCoord << ", "
    << kEndCoord << ")";
}

// The half-open convention, stated as a behaviour rather than a constant: an
// occluder sitting in the end coord is invisible, while the same occluder one
// voxel nearer truncates the ray. A future switch to an inclusive iterator
// changes the first of these and nothing else in the suite would notice.
TEST(FovOcclusion, TheEndCoordIsNotVisited) {
  FovEvaluator eval(oneRayCfg());
  MapCache in_end = makeOneVoxelMap(
    {Eigen::Vector3f(vcenter(kEndCoord), vcenter(0), vcenter(0))},
    10.0f, 1.0f);
  const auto r_end = eval.evaluate(originVp(), in_end, scoring::eig);
  EXPECT_EQ(r_end.observed_count, 0);
  EXPECT_EQ(r_end.total_ray_voxels, kClearSpan);

  MapCache in_last = makeOneVoxelMap(
    {Eigen::Vector3f(vcenter(kLastVisited), vcenter(0), vcenter(0))},
    10.0f, 1.0f);
  const auto r_last = eval.evaluate(originVp(), in_last, scoring::eig);
  EXPECT_EQ(r_last.observed_count, 1);
}

// The invariant the five clipping tests silently depend on.
TEST(FovOcclusion, UnknownVoxelsAreTransparent) {
  FovEvaluator eval(oneRayCfg());
  MapCache empty(kFovTestRes);
  auto r = eval.evaluate(originVp(), empty, scoring::eig);

  EXPECT_EQ(r.total_ray_voxels, kClearSpan);
  EXPECT_EQ(r.unknown_count, kClearSpan);   // every one of them
  EXPECT_EQ(r.observed_count, 0);
}

// An unknown voxel implies p_occ 0.5, so without the ptr && check any occ_stop
// <= 0.5 stops the ray at its first voxel. The shipped 0.7 masks this, hence
// the test runs below it. (notes: fov-test-null-guard-isolated)
TEST(FovOcclusion, UnknownVoxelsStayTransparentBelowTheirPriorPOcc) {
  FovConfig cfg = oneRayCfg();
  cfg.occ_stop = 0.0f;   // below the 0.5 prior, and below any real p_occ
  FovEvaluator eval(cfg);
  MapCache empty(kFovTestRes);
  auto r = eval.evaluate(originVp(), empty, scoring::eig);

  EXPECT_EQ(r.total_ray_voxels, kClearSpan)
    << "an absent voxel terminated the ray: the `ptr &&` guard in the "
       "occlusion stop is gone, and every ROI-clipping test above is now "
       "measuring a one-voxel ray";
}

// The stop fires, and it fires ON the occluder rather than before it: the
// occluded voxel is itself counted and scored, then iteration ends.
TEST(FovOcclusion, OccupiedVoxelTerminatesTheRayOnItself) {
  constexpr int kOccAt = 6;
  FovEvaluator eval(oneRayCfg());
  MapCache map = makeOneVoxelMap(
    {Eigen::Vector3f(vcenter(kOccAt), vcenter(0), vcenter(0))}, 10.0f, 1.0f);

  auto r = eval.evaluate(originVp(), map, scoring::eig);
  EXPECT_EQ(r.total_ray_voxels, kOccAt - kFirstCoord + 1);   // 3,4,5,6
  EXPECT_EQ(r.observed_count, 1);                            // the occluder
  EXPECT_EQ(r.unknown_count, kOccAt - kFirstCoord);          // 3,4,5
}

// Truncation, not skipping: a voxel behind the occluder is never visited at all.
// Counting observed voxels is what makes this falsifiable -- a ray that kept
// going and merely declined to score would still report 2.
TEST(FovOcclusion, VoxelsBehindAnOccluderAreNeverVisited) {
  FovEvaluator eval(oneRayCfg());
  scovox_msgs::msg::ScovoxMap msg;
  msg.resolution = static_cast<float>(kFovTestRes);
  auto add = [&msg](int cx, float a_occ, float a_free) {
      scovox_msgs::msg::ScovoxVoxel v;
      v.position.x = vcenter(cx);
      v.position.y = vcenter(0);
      v.position.z = vcenter(0);
      v.a_occ = a_occ;
      v.a_free = a_free;
      msg.voxels.push_back(v);
    };
  add(6, 10.0f, 1.0f);   // occluder
  add(9, 1.0f, 10.0f);   // strongly free, behind it
  MapCache map(kFovTestRes);
  map.updateFromScovoxMap(msg);

  auto r = eval.evaluate(originVp(), map, scoring::eig);
  EXPECT_EQ(r.observed_count, 1) << "the voxel behind the occluder was visited";
  EXPECT_EQ(r.total_ray_voxels, 4);
}

// An OBSERVED voxel below the threshold is transparent too, and is booked as
// observed rather than unknown. Without this, "the ray stopped" and "the voxel
// was observed" are not separated by any test.
TEST(FovOcclusion, ObservedFreeVoxelsDoNotTerminateTheRay) {
  FovEvaluator eval(oneRayCfg());
  MapCache map = makeOneVoxelMap(
    {Eigen::Vector3f(vcenter(6), vcenter(0), vcenter(0))}, 1.0f, 10.0f);

  auto r = eval.evaluate(originVp(), map, scoring::eig);
  EXPECT_EQ(r.total_ray_voxels, kClearSpan);
  EXPECT_EQ(r.observed_count, 1);
  EXPECT_EQ(r.unknown_count, kClearSpan - 1);
}

// occ_stop is INCLUSIVE (`>=`). a_occ 7 / a_free 3 gives p_occ = 7.0f/10.0f,
// which is the same float as the literal 0.7f -- asserted, not assumed, because
// if the two ever differ this stops being a boundary test and silently becomes
// a duplicate of the strictly-above case.
TEST(FovOcclusion, OccStopThresholdIsInclusive) {
  const Eigen::Vector3f at(vcenter(6), vcenter(0), vcenter(0));
  MapCache boundary = makeOneVoxelMap({at}, 7.0f, 3.0f);
  ASSERT_FLOAT_EQ(boundary.getVoxel(at).p_occ, 0.7f);

  FovConfig cfg = oneRayCfg();
  cfg.occ_stop = 0.7f;
  FovEvaluator at_threshold(cfg);
  EXPECT_EQ(at_threshold.evaluate(originVp(), boundary, scoring::eig)
              .total_ray_voxels, 4)
    << "p_occ exactly at occ_stop must stop the ray (>= not >)";

  // Strictly below: 6.9/10 < 0.7 by a comfortable float margin.
  MapCache below = makeOneVoxelMap({at}, 6.9f, 3.1f);
  ASSERT_LT(below.getVoxel(at).p_occ, 0.7f);
  EXPECT_EQ(at_threshold.evaluate(originVp(), below, scoring::eig)
              .total_ray_voxels, kClearSpan);
}

// evaluateSSMI duplicates the stop clause rather than sharing it, so it needs
// its own pin: the two estimators must truncate at the same voxel.
TEST(FovOcclusion, SsmiTruncatesAtTheSameVoxelAsEvaluate) {
  FovEvaluator eval(oneRayCfg());
  MapCache map = makeOneVoxelMap(
    {Eigen::Vector3f(vcenter(6), vcenter(0), vcenter(0))}, 10.0f, 1.0f);

  const auto r_eig = eval.evaluate(originVp(), map, scoring::eig);
  const auto r_ssmi = eval.evaluateSSMI(originVp(), map);
  EXPECT_EQ(r_ssmi.total_ray_voxels, r_eig.total_ray_voxels);
  EXPECT_EQ(r_ssmi.observed_count, r_eig.observed_count);
}

// Pins that SSMI drops its no-hit term when the ray is truncated: same map,
// occluder on the last visited voxel, only occ_stop changes, so the two runs
// differ only in that term. (notes: fov-test-ssmi-no-hit-term)
TEST(FovOcclusion, SsmiDropsTheNoHitTermOnlyWhenOccluded) {
  // On kLastVisited, the final voxel the half-open walk reaches: truncating
  // there removes nothing from the visit set, which is what makes the two runs
  // below differ in exactly one term.
  MapCache map = makeOneVoxelMap(
    {Eigen::Vector3f(vcenter(kLastVisited), vcenter(0), vcenter(0))},
    10.0f, 1.0f);                          // p_occ ~ 0.909

  FovConfig not_stopped = oneRayCfg();
  not_stopped.occ_stop = 0.95f;            // above p_occ: walk ends naturally
  FovConfig stopped = oneRayCfg();
  stopped.occ_stop = 0.7f;                 // below p_occ: walk is truncated

  const auto r_open = FovEvaluator(not_stopped).evaluateSSMI(originVp(), map);
  const auto r_occ = FovEvaluator(stopped).evaluateSSMI(originVp(), map);

  // Same voxels either way -- the occluder is the last one the walk reaches, so
  // truncating on it removes nothing from the visit set.
  ASSERT_EQ(r_open.total_ray_voxels, kClearSpan);
  ASSERT_EQ(r_occ.total_ray_voxels, kClearSpan);

  // ... so the entire score difference is the no-hit term, which is positive.
  EXPECT_GT(r_open.total_score, r_occ.total_score);
}
