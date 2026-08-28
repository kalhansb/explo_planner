#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "explo_planner/failed_goal_blacklist.hpp"

using namespace explo_planner;

namespace {
Eigen::Vector3f p(float x, float y, float z = 0.0f) {
  return Eigen::Vector3f(x, y, z);
}

/// Read a scalar out of the SHIPPED config instead of restating it as a
/// literal.
///
/// A test that hardcodes 240.0 checks arithmetic the blacklist was never going
/// to get wrong. What can actually regress is somebody tuning
/// failed_goal_ttl_sec back down toward the gen-4 value, and against that a
/// literal is inert: it keeps passing while the claim it stands for stops being
/// true of anything that runs. Reading the file makes the guard live.
///
/// Returns NaN when the key is absent, so the caller can ASSERT on it — a
/// silent 0.0 default would make every check downstream vacuous in the other
/// direction, which is the failure mode this helper exists to avoid.
double shipped_param(const std::string& key) {
  std::ifstream in(SHARED_PARAMS_YAML);
  if (!in) return std::numeric_limits<double>::quiet_NaN();
  std::string line;
  while (std::getline(in, line)) {
    const size_t first = line.find_first_not_of(" \t");
    if (first == std::string::npos || line[first] == '#') continue;
    if (line.compare(first, key.size(), key) != 0) continue;
    size_t colon = first + key.size();
    if (colon >= line.size() || line[colon] != ':') continue;  // prefix match
    const std::string rest = line.substr(colon + 1);
    try {
      return std::stod(rest);  // stops at the trailing comment, if any
    } catch (const std::exception&) {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

TEST(FailedGoalBlacklist, EmptyByDefault) {
  FailedGoalBlacklist bl;
  EXPECT_TRUE(bl.empty());
  EXPECT_EQ(bl.size(), 0u);
  EXPECT_FALSE(bl.isNear(p(0, 0), 5.0));
}

// isNear is a radius query in XY; Z is ignored.
TEST(FailedGoalBlacklist, IsNearRadiusXYOnly) {
  FailedGoalBlacklist bl;
  bl.add(p(10.0f, 10.0f, 3.0f), /*now_sec=*/0.0);
  EXPECT_EQ(bl.size(), 1u);

  // Inside the radius (1.0 m away < 2.0 m), even with a large Z difference.
  EXPECT_TRUE(bl.isNear(p(11.0f, 10.0f, -50.0f), 2.0));
  // Outside the radius (3.0 m away > 2.0 m).
  EXPECT_FALSE(bl.isNear(p(13.0f, 10.0f), 2.0));
  // Exactly on the boundary fails (strict <).
  EXPECT_FALSE(bl.isNear(p(12.0f, 10.0f), 2.0));
}

// prune drops entries strictly older than the TTL relative to now.
TEST(FailedGoalBlacklist, PruneByTtl) {
  FailedGoalBlacklist bl;
  bl.add(p(0, 0), 100.0);
  bl.add(p(5, 5), 105.0);

  // At t=110 with ttl=8: first entry age 10 (>8) expires, second age 5 stays.
  bl.prune(110.0, 8.0);
  EXPECT_EQ(bl.size(), 1u);
  EXPECT_FALSE(bl.isNear(p(0, 0), 1.0));  // expired entry gone
  EXPECT_TRUE(bl.isNear(p(5, 5), 1.0));   // fresh entry remains

  // Advancing far enough expires everything.
  bl.prune(200.0, 8.0);
  EXPECT_TRUE(bl.empty());
}

// Entry exactly at the TTL age is kept (age > ttl is strict).
TEST(FailedGoalBlacklist, PruneBoundaryKeepsExactAge) {
  FailedGoalBlacklist bl;
  bl.add(p(1, 1), 0.0);
  bl.prune(10.0, 10.0);  // age == ttl, not > ttl
  EXPECT_EQ(bl.size(), 1u);
}

// Multiple live entries are all queryable.
TEST(FailedGoalBlacklist, MultipleEntries) {
  FailedGoalBlacklist bl;
  bl.add(p(0, 0), 0.0);
  bl.add(p(20, 0), 0.0);
  bl.add(p(0, 20), 0.0);
  EXPECT_EQ(bl.size(), 3u);
  EXPECT_TRUE(bl.isNear(p(0.5f, 0.0f), 1.0));
  EXPECT_TRUE(bl.isNear(p(20.0f, 0.5f), 1.0));
  EXPECT_TRUE(bl.isNear(p(0.0f, 19.5f), 1.0));
  EXPECT_FALSE(bl.isNear(p(10.0f, 10.0f), 1.0));
}

// prune() must not assume insertion order implies age order. Under sim time a
// bag restart or a /clock step backwards stamps a fresh entry with an OLDER
// timestamp than the one already queued behind it; the old pop-front-until-
// fresh loop hit the newer entry first, broke, and left the expired one
// blacklisting its goal forever.
TEST(FailedGoalBlacklist, PrunesExpiredEntriesOutOfTimestampOrder) {
  FailedGoalBlacklist bl;
  bl.add(p(0, 0), /*now_sec=*/1000.0);  // recent, must survive
  bl.add(p(50, 0), /*now_sec=*/10.0);   // clock jumped back: stamped far older

  bl.prune(/*now_sec=*/1005.0, /*ttl_sec=*/60.0);

  EXPECT_EQ(bl.size(), 1u);
  EXPECT_TRUE(bl.isNear(p(0, 0), 1.0));    // 5 s old
  EXPECT_FALSE(bl.isNear(p(50, 0), 1.0));  // 995 s old
}

// An entry stamped in the future (age < 0, e.g. a peer-driven clock step
// forward between add and prune) counts as fresh, not as expired.
TEST(FailedGoalBlacklist, FutureStampedEntryIsNotPruned) {
  FailedGoalBlacklist bl;
  bl.add(p(3, 3), /*now_sec=*/500.0);
  bl.prune(/*now_sec=*/100.0, /*ttl_sec=*/60.0);
  EXPECT_EQ(bl.size(), 1u);
  EXPECT_TRUE(bl.isNear(p(3, 3), 1.0));
}

// ---------------------------------------------------------------------------
// Generation-5 additions: clustering, retirement, arrival-clears.
// ---------------------------------------------------------------------------

// Default construction must be byte-for-byte the historical behaviour, because
// visited_goals_ is the SAME class and nothing about visited-goal suppression
// is in scope for this change: no clustering (every add appends) and no
// retirement (every entry is prunable).
TEST(FailedGoalBlacklist, DefaultsAreAppendOnlyAndNeverRetire) {
  FailedGoalBlacklist bl;
  EXPECT_EQ(bl.retireAfter(), 0);
  for (int i = 0; i < 5; ++i) EXPECT_EQ(bl.add(p(1, 1), 100.0), 1);
  EXPECT_EQ(bl.size(), 5u);  // five records at one point, not one record x5
  EXPECT_FALSE(bl.isRetiredNear(p(1, 1), 2.0));
  bl.prune(1000.0, 60.0);
  EXPECT_TRUE(bl.empty());
}

// With a cluster radius, repeated failures at one SITE fold into one record
// that keeps its first-seen centre (so the suppression disc cannot drift with
// each new failure) and counts up.
TEST(FailedGoalBlacklist, ClusteredAddsCountUpAndKeepFirstCentre) {
  FailedGoalBlacklist bl;
  EXPECT_EQ(bl.add(p(10, 0), 0.0, /*cluster_radius_m=*/2.0), 1);
  EXPECT_EQ(bl.add(p(11, 0), 10.0, 2.0), 2);   // 1 m away: same site
  EXPECT_EQ(bl.add(p(11.5f, 0), 20.0, 2.0), 3);  // 1.5 m from the CENTRE
  EXPECT_EQ(bl.size(), 1u);
  // Centre never moved. Probed as a two-sided fence, because the obvious probe
  // is not a probe at all: a point at (11.9, 0) is inside the disc whether the
  // centre stayed at (10,0), moved to the mean (10.83,0) or jumped to the last
  // add (11.5,0), so asserting it passes on every build and rules out nothing.
  // These two bracket the drift instead. Both adds pushed +x, so a centre that
  // followed them loses the trailing edge and gains the leading one.
  EXPECT_TRUE(bl.isNear(p(8.1f, 0), 2.0));    // trailing edge: lost if drifted
  EXPECT_FALSE(bl.isNear(p(12.1f, 0), 2.0));  // leading edge: gained if drifted
  // A genuinely different site is its own record.
  EXPECT_EQ(bl.add(p(30, 0), 30.0, 2.0), 1);
  EXPECT_EQ(bl.size(), 2u);
}

// Retirement outlives any TTL: a permanent terrain trap cannot be closed by
// expiry, because the entry ages out while the robot is busy failing elsewhere.
TEST(FailedGoalBlacklist, RetiredSiteSurvivesEveryPrune) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(3);
  bl.add(p(0, 0), 0.0, 2.0);
  bl.add(p(0, 0), 100.0, 2.0);
  EXPECT_FALSE(bl.isRetiredNear(p(0, 0), 2.0));  // 2 failures: not yet
  EXPECT_EQ(bl.add(p(0, 0), 200.0, 2.0), 3);
  EXPECT_TRUE(bl.isRetiredNear(p(0, 0), 2.0));
  bl.prune(1e6, 240.0);
  EXPECT_EQ(bl.size(), 1u);
  EXPECT_TRUE(bl.isNear(p(0, 0), 2.0));
}

// Arriving at a site is proof the ground is reachable, and that outranks any
// amount of failure history — including retirement.
TEST(FailedGoalBlacklist, ClearNearReleasesRetiredSites) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(2);
  bl.add(p(5, 5), 0.0, 2.0);
  bl.add(p(5, 5), 10.0, 2.0);
  ASSERT_TRUE(bl.isRetiredNear(p(5, 5), 2.0));
  bl.add(p(40, 40), 10.0, 2.0);  // unrelated site, must survive
  EXPECT_EQ(bl.clearNear(p(5.5f, 5.0f), 2.0), 1u);
  EXPECT_FALSE(bl.isNear(p(5, 5), 2.0));
  EXPECT_TRUE(bl.isNear(p(40, 40), 2.0));
  EXPECT_EQ(bl.clearNear(p(0, 0), 1.0), 0u);  // nothing there
}

// Amnesty ordering input: the planner re-attempts the LEAST recently failed of
// the suppressed candidates, so this must report the most recent failure per
// site and -inf where there is no history at all.
TEST(FailedGoalBlacklist, LastFailTimeNearReportsMostRecentOrNegInf) {
  FailedGoalBlacklist bl;
  bl.add(p(0, 0), 100.0, 2.0);
  bl.add(p(0, 0), 350.0, 2.0);
  bl.add(p(20, 0), 200.0, 2.0);
  EXPECT_DOUBLE_EQ(bl.lastFailTimeNear(p(0, 0), 2.0), 350.0);
  EXPECT_DOUBLE_EQ(bl.lastFailTimeNear(p(20, 0), 2.0), 200.0);
  EXPECT_FALSE(std::isfinite(bl.lastFailTimeNear(p(99, 99), 2.0)));
}

// KNOWN-ANSWER REPLAY, both sides. mr1_hybrid_seed18 atlas re-picked the same
// unreachable site with measured gaps of 210, 203, 224 and 225 s between
// failures; each attempt then burned a full 180 s nav budget, 1441 s in total,
// and the cell was censored 0.021 above the coverage threshold.
//
// The test asserts BOTH directions on purpose: a one-sided "the new TTL
// suppresses it" would also pass on a build where the blacklist suppressed
// everything forever, and a guard that cannot fail is not a guard.
TEST(FailedGoalBlacklist, Seed18GapIsSuppressedAtShippedTtlAndNotAtGen4) {
  // Calibrate the reader first, on a key whose value is not the one under test.
  // A parser that returns the first number in the file, or NaN for everything,
  // would otherwise turn the whole test green by making its assertions
  // unreachable or trivially true.
  ASSERT_DOUBLE_EQ(shipped_param("failed_goal_radius_m"), 2.0)
      << "config reader is miscalibrated; the TTL it reports cannot be trusted";
  ASSERT_TRUE(std::isnan(shipped_param("no_such_param_here")))
      << "config reader invents values for absent keys";

  const double ttl = shipped_param("failed_goal_ttl_sec");
  ASSERT_TRUE(std::isfinite(ttl))
      << "failed_goal_ttl_sec not found in " << SHARED_PARAMS_YAML;

  const Eigen::Vector3f trap = p(-8.0f, 21.0f);
  const double t_fail = 0.0;
  const double gaps[4] = {210.0, 203.0, 224.0, 225.0};

  // The margin, stated as its own assertion. This is the part that can regress
  // in practice: the mechanism below is fine at any TTL, but a TTL edit that
  // lands under the longest measured gap silently reopens seed18 while every
  // per-gap check still passes on whatever value happens to be shipped.
  const double worst_gap = *std::max_element(gaps, gaps + 4);
  EXPECT_GT(ttl, worst_gap)
      << "shipped failed_goal_ttl_sec=" << ttl << " no longer covers the "
      << "longest measured seed18 re-pick gap (" << worst_gap << " s)";

  for (double gap : gaps) {
    // Generation 5, at the SHIPPED ttl: still suppressed when the robot
    // comes back.
    FailedGoalBlacklist neu;
    neu.add(trap, t_fail, 2.0);
    neu.prune(t_fail + gap, ttl);
    EXPECT_TRUE(neu.isNear(trap, 2.0)) << "ttl=" << ttl << " must still "
                                       << "suppress after " << gap << " s";
    // Generation 4 (60 s): expired — this is the defect, reproduced. A literal
    // on purpose: 60 is a historical fact about a campaign that has already
    // run, not a value anything still reads.
    FailedGoalBlacklist old;
    old.add(trap, t_fail, 2.0);
    old.prune(t_fail + gap, 60.0);
    EXPECT_FALSE(old.isNear(trap, 2.0)) << "ttl=60 is expected to have expired "
                                        << "after " << gap << " s";
  }
}

// The two longest seed18 gaps (626 s and ~756 s) are BEYOND the 240 s TTL:
// they are covered by retirement, not by the TTL, and claiming otherwise would
// overstate the fix. Third failure retires the site; the long gap then no
// longer matters.
TEST(FailedGoalBlacklist, LongSeed18GapsNeedRetirementNotTtl) {
  // Shipped TTL again, so that raising it past 626 s fails here rather than
  // leaving a comment claiming retirement is load-bearing when it no longer is.
  const double ttl = shipped_param("failed_goal_ttl_sec");
  ASSERT_TRUE(std::isfinite(ttl));
  const int retire_after =
      static_cast<int>(shipped_param("failed_goal_retire_after"));
  ASSERT_GT(retire_after, 0) << "retirement is off in the shipped config; the "
                             << "long seed18 gaps are then uncovered";

  const Eigen::Vector3f trap = p(-8.0f, 21.0f);
  FailedGoalBlacklist ttl_only;
  ttl_only.add(trap, 0.0, 2.0);
  ttl_only.prune(626.0, ttl);
  EXPECT_FALSE(ttl_only.isNear(trap, 2.0));  // TTL alone does NOT cover this

  FailedGoalBlacklist with_retire;
  with_retire.setRetireAfter(retire_after);
  with_retire.add(trap, 0.0, 2.0);
  with_retire.add(trap, 210.0, 2.0);
  with_retire.add(trap, 413.0, 2.0);
  with_retire.prune(626.0, ttl);
  EXPECT_TRUE(with_retire.isNear(trap, 2.0));
  EXPECT_TRUE(with_retire.isRetiredNear(trap, 2.0));
}

// ---------------------------------------------------------------------------
// Retirement as a veto, and the amnesty ordering built on top of it.
// ---------------------------------------------------------------------------

// add() folds a new failure into the FIRST site within the cluster radius, not
// the nearest one. That is not an accident to be tidied away later: the whole
// retired-veto argument rests on it, because it means "the site the caller just
// wrote" and "the site nearest the caller's position" are different records.
TEST(FailedGoalBlacklist, ClusteredAddTakesTheFirstSiteNotTheNearest) {
  FailedGoalBlacklist bl;
  bl.add(p(10, 0), 0.0, 2.0);    // site A, recorded first
  bl.add(p(12.5f, 0), 1.0, 2.0); // site B, 2.5 m away: outside A's radius
  ASSERT_EQ(bl.size(), 2u);
  // (11.6, 0) is 1.6 m from A and 0.9 m from B — nearest is B, first is A.
  EXPECT_EQ(bl.add(p(11.6f, 0), 2.0, 2.0), 2);  // A's count, not B's
  EXPECT_EQ(bl.size(), 2u);                      // no third site
  EXPECT_DOUBLE_EQ(bl.lastFailTimeNear(p(10, 0), 0.5), 2.0);   // A advanced
  EXPECT_DOUBLE_EQ(bl.lastFailTimeNear(p(12.5f, 0), 0.5), 1.0);  // B did not
}

// isRetiredNear answers "is any of this suppressed for good?", so a nearer
// non-retired record does not overturn it. Under the old nearest-wins form this
// query returned false and the planner treated a confirmed trap as an ordinary
// suppressed site — in the amnesty ordering, in the WARN, and in the event
// field the analysis reads.
TEST(FailedGoalBlacklist, RetiredVetoSurvivesANearerFreshSite) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(2);
  bl.add(p(10, 0), 0.0, 2.0);
  bl.add(p(10, 0), 5.0, 2.0);
  ASSERT_TRUE(bl.isRetiredNear(p(10, 0), 2.0));
  bl.add(p(12.5f, 0), 900.0, 2.0);  // separate, fresher, NOT retired
  ASSERT_EQ(bl.size(), 2u);

  // Probe at (11.5, 0): 1.5 m from the retired site, 1.0 m from the fresh one.
  // Nearest is the fresh site; the answer is still "retired".
  EXPECT_TRUE(bl.isRetiredNear(p(11.5f, 0), 2.0));
  // And the query still respects its radius — this is not a global flag.
  EXPECT_FALSE(bl.isRetiredNear(p(13.5f, 0), 2.0));
}

// Fail time alone would put the trap first, which is precisely the situation
// that arises unprompted: a retired site is never pruned, so its timestamp only
// gets older while everything around it is either re-failed or expires.
TEST(FailedGoalBlacklist, AmnestyOrderPutsRetiredLastEvenWhenOldest) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(3);
  const Eigen::Vector3f trap = p(0, 0);
  const Eigen::Vector3f ordinary = p(20, 0);
  bl.add(trap, 10.0, 2.0);
  bl.add(trap, 20.0, 2.0);
  bl.add(trap, 30.0, 2.0);
  ASSERT_TRUE(bl.isRetiredNear(trap, 2.0));
  bl.add(ordinary, 500.0, 2.0);
  ASSERT_FALSE(bl.isRetiredNear(ordinary, 2.0));
  ASSERT_LT(bl.lastFailTimeNear(trap, 2.0),
            bl.lastFailTimeNear(ordinary, 2.0));  // the trap IS the oldest

  EXPECT_TRUE(amnestyOrderBefore(bl, ordinary, trap, 2.0));
  EXPECT_FALSE(amnestyOrderBefore(bl, trap, ordinary, 2.0));
}

TEST(FailedGoalBlacklist, AmnestyOrderIsLeastRecentlyFailedInsideAPartition) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(2);
  bl.add(p(0, 0), 100.0, 2.0);
  bl.add(p(20, 0), 300.0, 2.0);
  ASSERT_FALSE(bl.isRetiredNear(p(0, 0), 2.0));
  EXPECT_TRUE(amnestyOrderBefore(bl, p(0, 0), p(20, 0), 2.0));
  EXPECT_FALSE(amnestyOrderBefore(bl, p(20, 0), p(0, 0), 2.0));

  // Same tie-break applies inside the retired partition: once both are traps,
  // amnesty still rotates rather than re-picking the most recent failure.
  bl.add(p(0, 0), 101.0, 2.0);
  bl.add(p(20, 0), 301.0, 2.0);
  ASSERT_TRUE(bl.isRetiredNear(p(0, 0), 2.0));
  ASSERT_TRUE(bl.isRetiredNear(p(20, 0), 2.0));
  EXPECT_TRUE(amnestyOrderBefore(bl, p(0, 0), p(20, 0), 2.0));
  EXPECT_FALSE(amnestyOrderBefore(bl, p(20, 0), p(0, 0), 2.0));
}

// std::stable_sort has undefined behaviour on a comparator that is not a strict
// weak ordering, so irreflexivity is not a formality. The no-history case
// matters too: lastFailTimeNear returns -inf there, and -inf < -inf is false
// only because the comparison is strict.
TEST(FailedGoalBlacklist, AmnestyOrderIsIrreflexive) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(2);
  bl.add(p(0, 0), 100.0, 2.0);
  bl.add(p(0, 0), 200.0, 2.0);
  EXPECT_FALSE(amnestyOrderBefore(bl, p(0, 0), p(0, 0), 2.0));   // retired
  EXPECT_FALSE(amnestyOrderBefore(bl, p(50, 0), p(50, 0), 2.0));  // no history
  // Two candidates with no history are equal, not ordered either way.
  EXPECT_FALSE(amnestyOrderBefore(bl, p(50, 0), p(60, 0), 2.0));
  EXPECT_FALSE(amnestyOrderBefore(bl, p(60, 0), p(50, 0), 2.0));
}

// End to end through the same std::stable_sort the planner runs, on indices
// into a candidate list, because that is where the two properties interact: the
// partition has to hold across the whole list, and equal-key candidates have to
// keep the utility order they arrived in (the sort is the last thing standing
// between the amnesty pick and an arbitrary one).
TEST(FailedGoalBlacklist, AmnestySortIsPartitionedAndStable) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(2);
  const Eigen::Vector3f trap = p(0, 0);
  bl.add(trap, 10.0, 2.0);
  bl.add(trap, 20.0, 2.0);   // retired, oldest
  bl.add(p(20, 0), 400.0, 2.0);  // fresh-ish
  bl.add(p(40, 0), 300.0, 2.0);  // older than (20,0)
  ASSERT_TRUE(bl.isRetiredNear(trap, 2.0));

  // Candidates in utility order, as the planner appends them. Indices 0 and 1
  // both sit inside the retired site's disc, so they share every sort key.
  const std::vector<Eigen::Vector3f> candidates = {
      p(-0.5f, 0),  // 0: retired site, first in utility order
      p(0.5f, 0),   // 1: retired site, second
      p(20, 0),     // 2: last failed 400
      p(40, 0),     // 3: last failed 300
  };
  std::vector<size_t> order = {0, 1, 2, 3};
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    return amnestyOrderBefore(bl, candidates[a], candidates[b], 2.0);
  });

  // Non-retired first, oldest failure first within that: 3 (300) then 2 (400).
  // Then the retired pair, still in the utility order they came in: 0 then 1.
  const std::vector<size_t> expected = {3, 2, 0, 1};
  EXPECT_EQ(order, expected);
}
