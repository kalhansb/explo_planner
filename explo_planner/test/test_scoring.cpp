#include <gtest/gtest.h>
#include "explo_planner/scoring.hpp"
#include <cmath>

using namespace explo_planner;

TEST(Scoring, EigUnobservedMaximum) {
  // Beta(1,1) prior should have maximum EIG
  UnifiedVoxel prior;  // default: a_occ=1, a_free=1, observed=false
  float eig_prior = scoring::eig(prior);
  EXPECT_GT(eig_prior, 0.0f);

  // Well-observed voxel should have much lower EIG
  UnifiedVoxel observed;
  observed.a_occ = 100.0f;
  observed.a_free = 100.0f;
  observed.p_occ = 0.5f;
  observed.observed = true;
  float eig_obs = scoring::eig(observed);
  EXPECT_GT(eig_prior, eig_obs * 5.0f);  // Prior EIG >> observed EIG
}

TEST(Scoring, EigConfidentWallNearZero) {
  UnifiedVoxel wall;
  wall.a_occ = 200.0f;
  wall.a_free = 2.0f;
  wall.p_occ = 200.0f / 202.0f;
  wall.observed = true;
  float eig_wall = scoring::eig(wall);
  EXPECT_LT(eig_wall, 0.01f);
}

TEST(Scoring, EigMonotonicallyDecreases) {
  // EIG should decrease as evidence increases
  float prev = 1.0f;
  for (float n = 1.0f; n <= 100.0f; n += 10.0f) {
    UnifiedVoxel v;
    v.a_occ = n;
    v.a_free = n;
    v.p_occ = 0.5f;
    v.observed = true;
    float e = scoring::eig(v);
    EXPECT_LE(e, prev);
    prev = e;
  }
}

TEST(Scoring, EntropyMaxAtHalf) {
  UnifiedVoxel half;
  half.p_occ = 0.5f;
  half.observed = true;
  float h_half = scoring::entropy(half);
  EXPECT_NEAR(h_half, std::log(2.0f), 0.01f);

  // Entropy at extremes should be near zero
  UnifiedVoxel certain;
  certain.p_occ = 0.99f;
  certain.observed = true;
  EXPECT_LT(scoring::entropy(certain), h_half);
}

TEST(Scoring, EntropyCannotDistinguishEvidenceLevels) {
  // This is the key weakness: entropy(p=0.5) is the same whether
  // the voxel has been observed 0 times or 1000 times.
  UnifiedVoxel low_evidence;
  low_evidence.p_occ = 0.5f;
  low_evidence.a_occ = 1.0f;
  low_evidence.a_free = 1.0f;

  UnifiedVoxel high_evidence;
  high_evidence.p_occ = 0.5f;
  high_evidence.a_occ = 500.0f;
  high_evidence.a_free = 500.0f;

  // Shannon entropy only depends on p_occ, not evidence level
  EXPECT_FLOAT_EQ(scoring::entropy(low_evidence),
                  scoring::entropy(high_evidence));
}

TEST(Scoring, EigDistinguishesEvidenceLevels) {
  // EIG *can* distinguish: Beta(1,1) has much higher EIG than Beta(500,500)
  UnifiedVoxel low;
  low.a_occ = 1.0f;
  low.a_free = 1.0f;
  low.p_occ = 0.5f;

  UnifiedVoxel high;
  high.a_occ = 500.0f;
  high.a_free = 500.0f;
  high.p_occ = 0.5f;

  EXPECT_GT(scoring::eig(low), scoring::eig(high) * 10.0f);
}

TEST(Scoring, FrontierScoring) {
  UnifiedVoxel unobs;
  unobs.observed = false;
  EXPECT_FLOAT_EQ(scoring::frontier(unobs), 1.0f);

  UnifiedVoxel obs;
  obs.observed = true;
  EXPECT_FLOAT_EQ(scoring::frontier(obs), 0.0f);
}

TEST(Scoring, RandomAlwaysZero) {
  UnifiedVoxel v;
  EXPECT_FLOAT_EQ(scoring::random(v), 0.0f);
}

TEST(Scoring, FactoryReturnsCorrectFunction) {
  auto fn_eig = scoring::create("eig");
  auto fn_ent = scoring::create("entropy");
  auto fn_fro = scoring::create("frontier");
  auto fn_rnd = scoring::create("random");

  UnifiedVoxel v;
  EXPECT_FLOAT_EQ(fn_rnd(v), 0.0f);
  EXPECT_GT(fn_eig(v), 0.0f);
  EXPECT_FLOAT_EQ(fn_fro(v), 1.0f);  // Unobserved

  EXPECT_THROW(scoring::create("invalid"), std::invalid_argument);
}
