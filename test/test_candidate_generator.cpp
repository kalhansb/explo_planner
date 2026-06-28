#include <gtest/gtest.h>
#include "explo_planner/candidate_generator.hpp"
#include "explo_planner/map_cache.hpp"
#include <cmath>

using namespace explo_planner;

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
