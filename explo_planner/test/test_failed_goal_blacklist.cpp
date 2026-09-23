// Moved comments: doc/explo_planner_code_notes.md
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

/// Reads a scalar from the shipped config (SHARED_PARAMS_YAML) so the tests
/// guard the value that actually runs. Returns NaN when the key is absent;
/// callers must ASSERT on it. (notes: blacklist-test-shipped-param)
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

// prune() must not assume insertion order is age order: under sim time a bag
// restart or a backwards /clock step can stamp a fresh entry older than one
// already queued. (notes: blacklist-prune-out-of-order-stamps)
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
  // Centre never moved, probed as a two-sided fence: a centre that followed the
  // +x adds would lose the trailing edge (8.1, 0) and gain the leading edge
  // (12.1, 0). (notes: blacklist-test-centre-fence)
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

// Known-answer replay of measured re-pick gaps, asserted both ways: suppressed
// at the shipped TTL, expired at 60 s. A one-sided check would also pass on a
// blacklist that suppresses forever.
// (notes: blacklist-test-known-answer-replay)
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
    // At a 60 s TTL the entry has expired. The literal is deliberate: nothing
    // still reads this value. (notes: blacklist-test-old-ttl-literal)
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
// Failure history has to outlive the suppression window.
//
// prune() runs every PLAN tick. With retirement on it must keep an expired
// record rather than erase it, so the count can reach the retire threshold
// across gaps longer than the TTL. (notes: blacklist-history-outlives-ttl)
// ---------------------------------------------------------------------------

// The core of the fix. Each failure is separated by more than the TTL, with a
// prune between them exactly as the planner does — the count must still climb.
TEST(FailedGoalBlacklist, CountSurvivesTtlExpirySoRetirementCanFire) {
  const double ttl = 240.0;
  FailedGoalBlacklist bl;
  bl.setRetireAfter(3);
  const Eigen::Vector3f trap = p(-42.03f, -23.66f);

  EXPECT_EQ(bl.add(trap, 0.0, 2.0), 1);
  bl.prune(500.0, ttl);                       // 500 s > ttl: suppression ends
  EXPECT_EQ(bl.add(trap, 500.0, 2.0), 2)      // ...but the history does not
      << "a re-failure at a known site restarted the count at 1";
  bl.prune(1000.0, ttl);
  EXPECT_EQ(bl.add(trap, 1000.0, 2.0), 3);
  EXPECT_TRUE(bl.isRetiredNear(trap, 2.0))
      << "three failures at one site must retire it however far apart they are";
  bl.prune(1e6, ttl);
  EXPECT_TRUE(bl.isNear(trap, 2.0)) << "a retired site is never aged out";
}

// The other half of the contract: keeping the count must NOT extend the veto.
// Suppression still ends at the TTL, or every place the robot ever failed
// becomes permanent no-go ground and the planner starves rather than unsticks.
TEST(FailedGoalBlacklist, ExpiredSiteKeepsHistoryButStopsSuppressing) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(3);
  bl.add(p(0, 0), 0.0, 2.0);
  EXPECT_TRUE(bl.isNear(p(0, 0), 2.0));

  bl.prune(300.0, 240.0);
  EXPECT_FALSE(bl.isNear(p(0, 0), 2.0)) << "expiry must still release the veto";
  EXPECT_EQ(bl.size(), 0u) << "an expired site is not an ACTIVE site";
  EXPECT_EQ(bl.historySize(), 1u) << "...but the record is still held";
  EXPECT_TRUE(bl.empty());

  // Failing there again revives the suppression rather than leaving a record
  // that counts up invisibly and never vetoes anything.
  EXPECT_EQ(bl.add(p(0, 0), 300.0, 2.0), 2);
  EXPECT_TRUE(bl.isNear(p(0, 0), 2.0));
  EXPECT_EQ(bl.size(), 1u);
  EXPECT_EQ(bl.historySize(), 1u) << "revival must reuse the record, not append";
}

// Two sites each failed twice with gaps far past the TTL: the second failure
// must count as the second, and a third retires the site.
// (notes: blacklist-test-sw-corner-trap)
TEST(FailedGoalBlacklist, Seed2SwCornerTrapAccumulatesAcrossItsRealGaps) {
  const double ttl = 240.0;
  const Eigen::Vector3f a = p(-42.03f, -23.66f);
  const Eigen::Vector3f b = p(-37.66f, -32.81f);

  FailedGoalBlacklist bl;
  bl.setRetireAfter(3);
  // t=0 / t=128: the first pass through both traps.
  EXPECT_EQ(bl.add(a, 0.0, 2.0), 1);
  EXPECT_EQ(bl.add(b, 128.0, 2.0), 1);
  // The planner prunes every tick throughout the gap; only the last matters.
  bl.prune(818.0, ttl);
  EXPECT_EQ(bl.add(a, 818.0, 2.0), 2) << "seed2 logged this as k=1";
  bl.prune(932.0, ttl);
  EXPECT_EQ(bl.add(b, 932.0, 2.0), 2) << "seed2 logged this as k=1";
  // Neither has failed three times, so neither is written off yet — the fix
  // makes retirement REACHABLE, it does not make it eager.
  EXPECT_FALSE(bl.isRetiredNear(a, 2.0));
  EXPECT_FALSE(bl.isRetiredNear(b, 2.0));
  bl.prune(1700.0, ttl);
  EXPECT_EQ(bl.add(a, 1700.0, 2.0), 3);
  EXPECT_TRUE(bl.isRetiredNear(a, 2.0));
}

// Arrival outranks history. Reaching a goal proves the ground is passable, so
// the carried-over count goes with the record and the next failure there is
// honestly a first failure — otherwise a site the robot has since driven
// through keeps a head start toward being written off for the run.
TEST(FailedGoalBlacklist, ArrivalDropsCarriedOverHistory) {
  FailedGoalBlacklist bl;
  bl.setRetireAfter(3);
  bl.add(p(4, 4), 0.0, 2.0);
  bl.prune(500.0, 240.0);          // expired, but still on the books
  ASSERT_EQ(bl.historySize(), 1u);
  EXPECT_EQ(bl.clearNear(p(4, 4), 2.0), 1u);
  EXPECT_EQ(bl.historySize(), 0u) << "expired history survived an arrival";
  EXPECT_EQ(bl.add(p(4, 4), 600.0, 2.0), 1);
}

// visited_goals_ is this same class with retirement off. It has no count to
// preserve, so it must keep erasing — a growing list of records that veto
// nothing would be pure leak on the hotter of the two call sites.
TEST(FailedGoalBlacklist, RetirementOffStillErasesOnPrune) {
  FailedGoalBlacklist bl;
  ASSERT_EQ(bl.retireAfter(), 0);
  for (int i = 0; i < 5; ++i) bl.add(p(1, 1), 100.0);
  ASSERT_EQ(bl.historySize(), 5u);
  bl.prune(1000.0, 60.0);
  EXPECT_EQ(bl.historySize(), 0u) << "retirement-off must not accumulate";
  EXPECT_EQ(bl.size(), 0u);
  EXPECT_TRUE(bl.empty());
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

// isRetiredNear asks whether any record in the radius is retired, so a nearer
// non-retired record does not overturn it. The planner's amnesty ordering, WARN
// and event field read it. (notes: blacklist-retired-veto-any-record)
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

// Runs the same std::stable_sort the planner runs: the retired partition must
// hold across the whole list, and equal-key candidates must keep their incoming
// utility order. (notes: blacklist-amnesty-sort-end-to-end)
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
