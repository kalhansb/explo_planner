#include <gtest/gtest.h>

#include <Eigen/Core>

#include "explo_planner/target_queue.hpp"

using namespace explo_planner;

namespace {
Eigen::Vector3f P(float x, float y, float z = 0.0f) {
  return Eigen::Vector3f(x, y, z);
}
constexpr float kDedup = 1.5f;
}  // namespace

// Distinct ids, far apart -> all enqueue.
TEST(TargetQueue, IngestDistinctTargets) {
  TargetQueue q;
  EXPECT_TRUE(q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup));
  EXPECT_TRUE(q.ingest(2, P(10, 0), 0.3f, 0.0f, kDedup));
  EXPECT_TRUE(q.ingest(3, P(0, 10), 0.3f, 0.0f, kDedup));
  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.pendingCount(), 3u);
}

// Same id is a duplicate even at a different position.
TEST(TargetQueue, DedupById) {
  TargetQueue q;
  EXPECT_TRUE(q.ingest(7, P(0, 0), 0.3f, 0.0f, kDedup));
  EXPECT_FALSE(q.ingest(7, P(20, 20), 0.5f, 0.0f, kDedup));
  EXPECT_EQ(q.size(), 1u);
}

// Different id but within the dedup radius is a re-reported tree; outside it
// is a new target.
TEST(TargetQueue, DedupByProximity) {
  TargetQueue q;
  EXPECT_TRUE(q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup));
  EXPECT_FALSE(q.ingest(2, P(1.0f, 0.0f), 0.3f, 0.0f, kDedup));  // 1.0 m < 1.5
  EXPECT_TRUE(q.ingest(3, P(2.0f, 0.0f), 0.3f, 0.0f, kDedup));   // 2.0 m > 1.5
  EXPECT_EQ(q.size(), 2u);
}

// Activation lifecycle: PENDING -> ACTIVE -> DONE, then the next target.
TEST(TargetQueue, ActivateLifecycle) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);
  q.ingest(2, P(10, 0), 0.3f, 0.0f, kDedup);

  EXPECT_EQ(q.active(), nullptr);
  Target* a = q.activate();
  ASSERT_NE(a, nullptr);
  EXPECT_EQ(a->id, 1u);
  EXPECT_EQ(a->status, Target::Status::ACTIVE);

  // Idempotent: activate() again returns the same active target.
  EXPECT_EQ(q.activate(), a);
  EXPECT_EQ(q.active()->id, 1u);

  q.markActiveDone();
  EXPECT_EQ(q.active(), nullptr);
  EXPECT_TRUE(q.hasPending());

  Target* b = q.activate();
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->id, 2u);

  q.markActiveDone();
  EXPECT_FALSE(q.hasPending());
  EXPECT_EQ(q.pendingCount(), 0u);
}

// hasPending / pendingCount ignore DONE targets; a DONE target still dedups.
TEST(TargetQueue, DoneTargetsStillDedup) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);
  q.activate();
  q.markActiveDone();
  EXPECT_FALSE(q.hasPending());
  // Re-report of the finished tree (same id) is rejected, not re-queued.
  EXPECT_FALSE(q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup));
  // ...and by proximity too.
  EXPECT_FALSE(q.ingest(99, P(0.5f, 0.0f), 0.3f, 0.0f, kDedup));
  EXPECT_FALSE(q.hasPending());
}

// Vantage dwell bookkeeping: clear_los_dwells counts only clear-LoS dwells;
// visited-by-proximity is tracked per active target.
TEST(TargetQueue, VantageDwellAndVisited) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);
  Target* a = q.activate();
  ASSERT_NE(a, nullptr);

  EXPECT_FALSE(q.isVantageVisited(P(2, 0), 0.5f));
  q.recordVantageDwell(P(2, 0), /*los_clear=*/true);
  EXPECT_TRUE(q.isVantageVisited(P(2, 0), 0.5f));
  EXPECT_TRUE(q.isVantageVisited(P(2.2f, 0.1f), 0.5f));  // within tol
  EXPECT_FALSE(q.isVantageVisited(P(-2, 0), 0.5f));      // far away
  EXPECT_EQ(a->clear_los_dwells, 1);

  q.recordVantageDwell(P(-2, 0), /*los_clear=*/false);   // dwelled, not clear
  EXPECT_EQ(a->clear_los_dwells, 1);
  EXPECT_EQ(a->visited_vantages.size(), 2u);
}

// No active target: visited query is false and dwell recording is a safe no-op.
TEST(TargetQueue, NoActiveTargetSafe) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);  // PENDING, not active
  EXPECT_FALSE(q.isVantageVisited(P(0, 0), 1.0f));
  q.recordVantageDwell(P(0, 0), true);  // must not crash / mutate
  EXPECT_EQ(q.pendingCount(), 1u);
}
