/// @file test_separation.cpp
/// @brief Unit tests for the team-separation discount (separation.hpp).
///
/// The point of this file is not that the arithmetic is right — the ramp is
/// four lines. It is that the term cannot be SILENTLY INERT or silently
/// mis-scaled, which is the failure the whole design is a reaction to: MinPos
/// is a real mechanism whose one observable has read zero for a whole campaign,
/// and nothing in that campaign's data can say whether the veto never mattered
/// or never ran. So the cases below are weighted toward the boundary between
/// "off" and "on", toward configurations a typo would produce, and toward the
/// exact inputs a treated campaign is most likely to run (weight = 1).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "explo_planner/separation.hpp"

using explo_planner::SeparationTerm;

namespace {

std::vector<SeparationTerm::Anchor> anchorsAt(
    std::initializer_list<std::pair<float, float>> pts) {
  std::vector<SeparationTerm::Anchor> out;
  for (const auto& p : pts) out.push_back(SeparationTerm::Anchor{p.first, p.second});
  return out;
}

SeparationTerm::Config cfg(double weight, double radius = 20.0,
                           double max_age = 10.0) {
  SeparationTerm::Config c;
  c.weight = weight;
  c.radius_m = radius;
  c.max_age_sec = max_age;
  return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// configure(): what counts as usable, and what the refusal leaves behind
// ---------------------------------------------------------------------------

TEST(SeparationConfigure, DefaultIsOff) {
  SeparationTerm s;
  EXPECT_FALSE(s.enabled());
  EXPECT_EQ(s.config().weight, 0.0);
  // A default-constructed term must not touch a score, because that is the
  // state every pre-existing arm of every campaign runs in.
  EXPECT_FLOAT_EQ(s.discount(0.0f, 0.0f, anchorsAt({{0.0f, 0.0f}})), 1.0f);
}

TEST(SeparationConfigure, ZeroWeightAcceptedAndOff) {
  SeparationTerm s;
  EXPECT_EQ(s.configure(cfg(0.0)), "");   // not an error: this is the control
  EXPECT_FALSE(s.enabled());
}

TEST(SeparationConfigure, ZeroWeightStillKeepsRadiusAndMaxAge) {
  // The control arm's DIAGNOSTICS are measured on these two numbers, in every
  // arm, and they are the counterfactual a treated arm is read against. If a
  // weight-0 configure quietly reverted them to the defaults, an off arm asked
  // for a 25 m radius would have reported its peer distances against 20 m and
  // the two arms' columns would not have been comparable — a mismatch nothing
  // downstream could see, because both arms would print the requested 25 in
  // their manifests.
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.0, 25.0, 3.0)), "");
  EXPECT_FALSE(s.enabled());
  EXPECT_EQ(s.config().radius_m, 25.0);
  EXPECT_EQ(s.config().max_age_sec, 3.0);
}

TEST(SeparationConfigure, PositiveWeightEnables) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.5, 30.0, 4.0)), "");
  EXPECT_TRUE(s.enabled());
  EXPECT_EQ(s.config().weight, 0.5);
  EXPECT_EQ(s.config().radius_m, 30.0);
  EXPECT_EQ(s.config().max_age_sec, 4.0);
}

TEST(SeparationConfigure, RefusesOutOfRangeWeight) {
  for (double w : {-0.1, 1.0001, 3.0, -1.0}) {
    SeparationTerm s;
    const std::string err = s.configure(cfg(w));
    EXPECT_FALSE(err.empty()) << "weight " << w << " should have been refused";
    EXPECT_FALSE(s.enabled());
  }
}

TEST(SeparationConfigure, RefusesNonFiniteWeight) {
  SeparationTerm s;
  EXPECT_FALSE(s.configure(cfg(std::numeric_limits<double>::quiet_NaN())).empty());
  EXPECT_FALSE(s.enabled());
  EXPECT_FALSE(s.configure(cfg(std::numeric_limits<double>::infinity())).empty());
  EXPECT_FALSE(s.enabled());
}

TEST(SeparationConfigure, RefusesNonPositiveRadius) {
  for (double r : {0.0, -1.0, -20.0}) {
    SeparationTerm s;
    EXPECT_FALSE(s.configure(cfg(0.5, r)).empty()) << "radius " << r;
    EXPECT_FALSE(s.enabled());
  }
  SeparationTerm nan_r;
  EXPECT_FALSE(
      nan_r.configure(cfg(0.5, std::numeric_limits<double>::quiet_NaN())).empty());
  EXPECT_FALSE(nan_r.enabled());
}

TEST(SeparationConfigure, RefusesRadiiThatBreakTheFloatRamp) {
  // Positivity is not enough. The ramp divides by the radius in float: a radius
  // that underflows the cast to 0 makes a candidate sitting exactly on a
  // teammate evaluate 0/0 and log a NaN discount, and one that overflows it to
  // infinity gives every candidate the same discount, so the term takes the
  // information away and steers nothing. Both pass a bare `> 0` check and both
  // look like a working term in the manifest.
  for (double r : {1e-40, 1e-30, 1e-4, 1e7, 1e39, 1e300}) {
    SeparationTerm s;
    EXPECT_FALSE(s.configure(cfg(0.5, r)).empty()) << "radius " << r;
    EXPECT_FALSE(s.enabled()) << "radius " << r;
  }
  // The bounds themselves are accepted — they are a guard against absurdity,
  // not an opinion about plot size.
  for (double r : {1e-3, 1.0, 20.0, 1e6}) {
    SeparationTerm s;
    EXPECT_EQ(s.configure(cfg(0.5, r)), "") << "radius " << r;
    EXPECT_TRUE(s.enabled()) << "radius " << r;
  }
}

TEST(SeparationConfigure, ADegenerateRadiusCannotProduceANaNDiscount) {
  // The behavioural half of the test above: whatever radius survives
  // configure(), a candidate standing exactly on a teammate gets a real number.
  // Written as a property over the accepted range rather than as a check of the
  // bound, so it keeps biting if the bound is ever widened.
  const auto peer = anchorsAt({{0.0f, 0.0f}});
  for (double r : {1e-3, 1e-2, 1.0, 20.0, 1e5, 1e6}) {
    SeparationTerm s;
    ASSERT_EQ(s.configure(cfg(1.0, r)), "") << "radius " << r;
    const float d = s.discount(0.0f, 0.0f, peer);
    EXPECT_FALSE(std::isnan(d)) << "radius " << r;
    EXPECT_GE(d, 0.0f);
    EXPECT_LE(d, 1.0f);
  }
}

TEST(SeparationConfigure, RefusesNonPositiveMaxAge) {
  // A zero or negative freshness bound would make every peer position too old
  // to repel, i.e. a term that is ON in the manifest and inert in the binary.
  // Refusing is the honest reading of the request.
  for (double a : {0.0, -1.0}) {
    SeparationTerm s;
    EXPECT_FALSE(s.configure(cfg(0.5, 20.0, a)).empty()) << "max_age " << a;
    EXPECT_FALSE(s.enabled());
  }
}

TEST(SeparationConfigure, RadiusIsValidatedEvenWhenTheTermIsOff) {
  // Same reason as ZeroWeightStillKeepsRadiusAndMaxAge: the bound is live in an
  // off arm because the diagnostics use it, so a bad one has to be caught there
  // too rather than only on the treated side.
  SeparationTerm s;
  EXPECT_FALSE(s.configure(cfg(0.0, -5.0)).empty());
  EXPECT_FALSE(s.enabled());
}

TEST(SeparationConfigure, RefusalLeavesTheDefaultConfigNotTheRejectedOne) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.5, 30.0, 4.0)), "");
  ASSERT_TRUE(s.enabled());
  ASSERT_FALSE(s.configure(cfg(2.0, 30.0, 4.0)).empty());
  EXPECT_FALSE(s.enabled());
  // config() is what goes into the manifest, so after a refusal it must read as
  // the disabled default and not as the numbers that were thrown out.
  EXPECT_EQ(s.config().weight, 0.0);
  EXPECT_EQ(s.config().radius_m, 20.0);
  EXPECT_EQ(s.config().max_age_sec, 10.0);
}

// ---------------------------------------------------------------------------
// nearestAnchorDist(): static, and usable with the term switched off
// ---------------------------------------------------------------------------

TEST(SeparationNearest, NegativeWhenNoAnchors) {
  EXPECT_LT(SeparationTerm::nearestAnchorDist(1.0f, 2.0f, {}), 0.0f);
}

TEST(SeparationNearest, PicksTheClosest) {
  const auto a = anchorsAt({{10.0f, 0.0f}, {3.0f, 4.0f}, {-20.0f, 0.0f}});
  EXPECT_FLOAT_EQ(SeparationTerm::nearestAnchorDist(0.0f, 0.0f, a), 5.0f);
}

TEST(SeparationNearest, SkipsNonFiniteAnchorsButUsesTheRest) {
  std::vector<SeparationTerm::Anchor> a = anchorsAt({{3.0f, 4.0f}});
  a.push_back(SeparationTerm::Anchor{
      std::numeric_limits<float>::quiet_NaN(), 0.0f});
  EXPECT_FLOAT_EQ(SeparationTerm::nearestAnchorDist(0.0f, 0.0f, a), 5.0f);
}

TEST(SeparationNearest, AllAnchorsNonFiniteReadsAsNoAnchor) {
  std::vector<SeparationTerm::Anchor> a;
  a.push_back(SeparationTerm::Anchor{std::numeric_limits<float>::quiet_NaN(),
                                     0.0f});
  a.push_back(SeparationTerm::Anchor{
      0.0f, std::numeric_limits<float>::infinity()});
  EXPECT_LT(SeparationTerm::nearestAnchorDist(0.0f, 0.0f, a), 0.0f);
}

TEST(SeparationNearest, NonFiniteQueryReadsAsNoAnchor) {
  // Reported as "no anchor" rather than as inf so a reader of the CSV column
  // cannot mistake a broken candidate for a very distant teammate.
  const auto a = anchorsAt({{0.0f, 0.0f}});
  EXPECT_LT(SeparationTerm::nearestAnchorDist(
                std::numeric_limits<float>::quiet_NaN(), 0.0f, a),
            0.0f);
}

// ---------------------------------------------------------------------------
// discount(): the ramp
// ---------------------------------------------------------------------------

TEST(SeparationDiscount, OffTermNeverDiscounts) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.0, 20.0)), "");
  EXPECT_FLOAT_EQ(s.discount(0.0f, 0.0f, anchorsAt({{0.0f, 0.0f}})), 1.0f);
}

TEST(SeparationDiscount, FullWeightAtZeroDistance) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.6, 20.0)), "");
  EXPECT_FLOAT_EQ(s.discount(0.0f, 0.0f, anchorsAt({{0.0f, 0.0f}})), 0.4f);
}

TEST(SeparationDiscount, LinearRampToTheRadius) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.5, 20.0)), "");
  const auto a = anchorsAt({{0.0f, 0.0f}});
  EXPECT_NEAR(s.discount(5.0f, 0.0f, a), 1.0 - 0.5 * 0.75, 1e-6);
  EXPECT_NEAR(s.discount(10.0f, 0.0f, a), 1.0 - 0.5 * 0.50, 1e-6);
  EXPECT_NEAR(s.discount(15.0f, 0.0f, a), 1.0 - 0.5 * 0.25, 1e-6);
}

TEST(SeparationDiscount, ExactlyOneAtTheRadiusAndBeyond) {
  // The finding this term is built on is stated as a threshold ("time within
  // 20 m"), so the ramp has to reach 1 AT the radius with no tail past it.
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.9, 20.0)), "");
  const auto a = anchorsAt({{0.0f, 0.0f}});
  EXPECT_FLOAT_EQ(s.discount(20.0f, 0.0f, a), 1.0f);
  EXPECT_FLOAT_EQ(s.discount(50.0f, 0.0f, a), 1.0f);
  EXPECT_FLOAT_EQ(s.discount(5000.0f, 0.0f, a), 1.0f);
}

TEST(SeparationDiscount, RadiusIsHonouredAndNotHardCoded) {
  // Added after a mutant that replaced cfg_.radius_m with a literal 20.0f
  // survived the whole suite: every other case here happens to use the default
  // radius, so a knob that was parsed, validated, logged into the manifest and
  // then ignored would have looked exactly like a working one. That specific
  // failure — a configured value the binary does not actually use — is the one
  // this project keeps paying for, so it gets its own case.
  const auto peer = anchorsAt({{0.0f, 0.0f}});
  {
    SeparationTerm tight;
    ASSERT_EQ(tight.configure(cfg(0.5, 10.0)), "");
    EXPECT_NEAR(tight.discount(5.0f, 0.0f, peer), 0.75, 1e-6);
    EXPECT_FLOAT_EQ(tight.discount(10.0f, 0.0f, peer), 1.0f);
    EXPECT_FLOAT_EQ(tight.discount(15.0f, 0.0f, peer), 1.0f)
        << "a 10 m radius must be finished well before the 20 m default is";
  }
  {
    SeparationTerm wide;
    ASSERT_EQ(wide.configure(cfg(0.5, 40.0)), "");
    EXPECT_NEAR(wide.discount(20.0f, 0.0f, peer), 0.75, 1e-6)
        << "at the 20 m default's edge a 40 m radius must still be biting";
    EXPECT_NEAR(wide.discount(30.0f, 0.0f, peer), 0.875, 1e-6);
    EXPECT_FLOAT_EQ(wide.discount(40.0f, 0.0f, peer), 1.0f);
  }
}

TEST(SeparationDiscount, NoAnchorsMeansNoDiscountEvenWhenEnabled) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(1.0, 20.0)), "");
  EXPECT_FLOAT_EQ(s.discount(0.0f, 0.0f, {}), 1.0f);
}

TEST(SeparationDiscount, UsesTheNearestOfSeveralPeers) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.5, 20.0)), "");
  const auto a = anchorsAt({{100.0f, 0.0f}, {10.0f, 0.0f}});
  EXPECT_NEAR(s.discount(0.0f, 0.0f, a), 0.75, 1e-6);
}

TEST(SeparationDiscount, StaysInRangeForEveryWeight) {
  for (double w : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    SeparationTerm s;
    ASSERT_EQ(s.configure(cfg(w, 20.0)), "");
    const auto a = anchorsAt({{0.0f, 0.0f}});
    for (float d = 0.0f; d < 60.0f; d += 0.5f) {
      const float g = s.discount(d, 0.0f, a);
      EXPECT_GE(g, 0.0f);
      EXPECT_LE(g, 1.0f);
      EXPECT_TRUE(std::isfinite(g));
    }
  }
}

TEST(SeparationDiscount, MonotoneNonDecreasingInDistance) {
  // Being further from a teammate must never be scored worse than being nearer.
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.7, 20.0)), "");
  const auto a = anchorsAt({{0.0f, 0.0f}});
  float prev = -1.0f;
  for (float d = 0.0f; d < 40.0f; d += 0.25f) {
    const float g = s.discount(d, 0.0f, a);
    EXPECT_GE(g, prev - 1e-6f) << "at d=" << d;
    prev = g;
  }
}

TEST(SeparationDiscount, WeightOneZeroesACandidateOnTopOfTheTeammate) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(1.0, 20.0)), "");
  EXPECT_FLOAT_EQ(s.discount(0.0f, 0.0f, anchorsAt({{0.0f, 0.0f}})), 0.0f);
}

// ---------------------------------------------------------------------------
// apply(): the two guards that stop a discount becoming a reward or a NaN
// ---------------------------------------------------------------------------

TEST(SeparationApply, ScalesAPositiveUtility) {
  EXPECT_FLOAT_EQ(SeparationTerm::apply(10.0f, 0.25f), 2.5f);
}

TEST(SeparationApply, LeavesNegativeInfinityAlone) {
  // Unreachable candidates carry -inf so they sort last. -inf * 0 is NaN, and
  // a NaN score sorts wherever the comparison chain happens to put it. The
  // configuration that would produce it — weight 1, candidate on the teammate —
  // is exactly the one a treated campaign is most likely to run.
  const float neg_inf = -std::numeric_limits<float>::infinity();
  EXPECT_EQ(SeparationTerm::apply(neg_inf, 0.0f), neg_inf);
  EXPECT_EQ(SeparationTerm::apply(neg_inf, 0.5f), neg_inf);
  EXPECT_FALSE(std::isnan(SeparationTerm::apply(neg_inf, 0.0f)));
}

TEST(SeparationApply, LeavesNegativeUtilityAlone) {
  // A discount below 1 makes a negative number LARGER, so the penalty would
  // become a reward. Nothing currently produces a negative finite utility,
  // which is exactly why it would go unnoticed if something started to.
  EXPECT_FLOAT_EQ(SeparationTerm::apply(-4.0f, 0.5f), -4.0f);
}

TEST(SeparationApply, LeavesZeroAlone) {
  EXPECT_FLOAT_EQ(SeparationTerm::apply(0.0f, 0.5f), 0.0f);
}

TEST(SeparationApply, PassesNaNUtilityThroughUnchanged) {
  EXPECT_TRUE(std::isnan(SeparationTerm::apply(
      std::numeric_limits<float>::quiet_NaN(), 0.5f)));
}

TEST(SeparationApply, IgnoresANonFiniteDiscount) {
  EXPECT_FLOAT_EQ(SeparationTerm::apply(
      10.0f, std::numeric_limits<float>::quiet_NaN()), 10.0f);
  EXPECT_FLOAT_EQ(SeparationTerm::apply(
      10.0f, std::numeric_limits<float>::infinity()), 10.0f);
}

TEST(SeparationApply, UnitDiscountIsBitIdentical) {
  // The off configuration must reproduce the pre-separation planner exactly,
  // and "exactly" has to mean bit-for-bit or the binary-generation argument in
  // the campaign gate does not hold.
  for (float u : {1e-9f, 0.5f, 1.0f, 2537.125f, 1e12f}) {
    EXPECT_EQ(SeparationTerm::apply(u, 1.0f), u);
  }
}

// ---------------------------------------------------------------------------
// The composite behaviour the experiment actually depends on
// ---------------------------------------------------------------------------

TEST(SeparationOrdering, DiscountCanFlipTwoCandidates) {
  // The whole reason the term exists: a slightly better candidate near the
  // teammate should lose to a slightly worse one further away. If this does not
  // happen the term is decoration.
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.5, 20.0)), "");
  const auto peer = anchorsAt({{0.0f, 0.0f}});

  const float near_u = 10.0f;   // 2 m from the teammate
  const float far_u  = 9.0f;    // 40 m away, outside the radius
  const float near_scored = SeparationTerm::apply(
      near_u, s.discount(2.0f, 0.0f, peer));
  const float far_scored = SeparationTerm::apply(
      far_u, s.discount(40.0f, 0.0f, peer));

  EXPECT_GT(near_u, far_u);           // undiscounted, the near one leads
  EXPECT_LT(near_scored, far_scored); // discounted, the far one does
}

TEST(SeparationOrdering, OffTermPreservesEveryOrdering) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.0, 20.0)), "");
  const auto peer = anchorsAt({{0.0f, 0.0f}});
  const float a = SeparationTerm::apply(10.0f, s.discount(2.0f, 0.0f, peer));
  const float b = SeparationTerm::apply(9.0f, s.discount(40.0f, 0.0f, peer));
  EXPECT_EQ(a, 10.0f);
  EXPECT_EQ(b, 9.0f);
  EXPECT_GT(a, b);
}

TEST(SeparationOrdering, ADistantTeammateChangesNothing) {
  // Under the 2026-09-03 radio a teammate is only heard within ~30 m, so most
  // of a run has either no eligible anchor or a distant one. The term must be a
  // no-op there, or it would be re-ranking candidates on a peer it cannot act
  // usefully about.
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(1.0, 20.0)), "");
  const auto peer = anchorsAt({{200.0f, 200.0f}});
  for (float d = 0.0f; d < 50.0f; d += 5.0f) {
    EXPECT_FLOAT_EQ(s.discount(d, 0.0f, peer), 1.0f);
  }
}

// ---------------------------------------------------------------------------
// eligibleAnchor(): the freshness bound, and why it lives in this unit
// ---------------------------------------------------------------------------
//
// max_age_sec used to be consumed entirely in the planner node, in a private
// method no test could reach. A review of this file found that replacing the
// comparison there with a constant passed the whole suite — the knob would have
// been parsed, range-checked, written into every run manifest, and then not
// read, which is a failure this project has already paid for more than once.
// The comparison moved in here so these tests can hold it.

TEST(SeparationEligible, HonoursTheConfiguredBoundAndIsNotHardCoded) {
  // Two different bounds, checked either side of each. A constant in place of
  // cfg_.max_age_sec cannot satisfy both.
  {
    SeparationTerm s;
    ASSERT_EQ(s.configure(cfg(0.5, 20.0, 10.0)), "");
    EXPECT_TRUE(s.eligibleAnchor(0.0));
    EXPECT_TRUE(s.eligibleAnchor(9.99));
    EXPECT_TRUE(s.eligibleAnchor(10.0));      // the bound itself is fresh
    EXPECT_FALSE(s.eligibleAnchor(10.01));
    EXPECT_FALSE(s.eligibleAnchor(60.0));
  }
  {
    SeparationTerm s;
    ASSERT_EQ(s.configure(cfg(0.5, 20.0, 3.0)), "");
    EXPECT_TRUE(s.eligibleAnchor(2.9));
    EXPECT_TRUE(s.eligibleAnchor(3.0));
    EXPECT_FALSE(s.eligibleAnchor(3.1));
    EXPECT_FALSE(s.eligibleAnchor(9.99));     // fresh under the bound above
  }
}

TEST(SeparationEligible, NegativeMeansNoPositionHeldNotAVeryFreshOne) {
  // TeamModel::positionAgeSec returns -1 for "no position for this peer". A
  // bare `age <= max_age` would read that as the freshest possible reading and
  // repel from whatever happens to be in the pose fields — which for a peer
  // that has never reported one is the origin.
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.5, 20.0, 10.0)), "");
  EXPECT_FALSE(s.eligibleAnchor(-1.0));
  EXPECT_FALSE(s.eligibleAnchor(-0.001));
  EXPECT_FALSE(s.eligibleAnchor(std::numeric_limits<double>::quiet_NaN()));
}

TEST(SeparationEligible, MeasuredInAnUntreatedArmToo) {
  // The eligible-peer count is logged in every arm, so the bound has to be live
  // with the weight at 0. If this ever gated on enabled(), every control cell
  // would report zero eligible peers and the treated arm would lose the
  // counterfactual it is scored against.
  SeparationTerm off;
  ASSERT_EQ(off.configure(cfg(0.0, 20.0, 10.0)), "");
  ASSERT_FALSE(off.enabled());
  EXPECT_TRUE(off.eligibleAnchor(5.0));
  EXPECT_FALSE(off.eligibleAnchor(15.0));
}

TEST(SeparationEligible, ARefusedConfigFallsBackToTheDefaultBound) {
  SeparationTerm s;
  ASSERT_EQ(s.configure(cfg(0.5, 20.0, 3.0)), "");
  ASSERT_FALSE(s.eligibleAnchor(5.0));           // 3 s bound in force
  ASSERT_FALSE(s.configure(cfg(2.0, 20.0, 3.0)).empty());
  EXPECT_TRUE(s.eligibleAnchor(5.0));            // back to the 10 s default
  EXPECT_FALSE(s.eligibleAnchor(11.0));
}
