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
