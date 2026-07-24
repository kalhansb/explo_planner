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

// An indexed clear-LoS dwell sets the ring-index bit in clear_mask; dwelling
// the same index again does not double-count (idempotent on the mask).
TEST(TargetQueue, IndexedDwellSetsMaskNoDoubleCount) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);
  Target* a = q.activate();
  ASSERT_NE(a, nullptr);

  q.recordVantageDwell(P(2, 0), /*los_clear=*/true, /*vantage_index=*/0);
  EXPECT_EQ(a->clear_los_dwells, 1);
  EXPECT_EQ(a->clear_mask, 0b001u);

  // Same index revisited -> no extra credit.
  q.recordVantageDwell(P(2, 0), /*los_clear=*/true, /*vantage_index=*/0);
  EXPECT_EQ(a->clear_los_dwells, 1);
  EXPECT_EQ(a->clear_mask, 0b001u);

  // A different index adds one more bit and one more count.
  q.recordVantageDwell(P(0, 2), /*los_clear=*/true, /*vantage_index=*/2);
  EXPECT_EQ(a->clear_los_dwells, 2);
  EXPECT_EQ(a->clear_mask, 0b101u);
}

// Out-of-range indices (>= 32 or -1) keep the pre-mask counter-only behaviour:
// they count locally but never touch the team-credit mask.
TEST(TargetQueue, OutOfRangeIndexCounterOnly) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);
  Target* a = q.activate();
  ASSERT_NE(a, nullptr);

  q.recordVantageDwell(P(2, 0), true, /*vantage_index=*/40);
  EXPECT_EQ(a->clear_los_dwells, 1);
  EXPECT_EQ(a->clear_mask, 0u);

  q.recordVantageDwell(P(0, 2), true, /*vantage_index=*/-1);
  EXPECT_EQ(a->clear_los_dwells, 2);
  EXPECT_EQ(a->clear_mask, 0u);
}

// mergePeerDwells: a peer's dwelled-index mask adds team credit, appends the
// canonical ring poses (so isVantageVisited skips those angles), unions rather
// than overwrites, and is idempotent on re-receipt.
TEST(TargetQueue, MergePeerDwells) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);
  Target* a = q.activate();
  ASSERT_NE(a, nullptr);

  // The deterministic vantage ring the peer indices refer to.
  const std::vector<Eigen::Vector3f> ring = {P(2, 0), P(-1, 1.7f),
                                              P(-1, -1.7f)};

  // Peer dwelled ring indices 0 and 2.
  EXPECT_TRUE(q.mergePeerDwells(1, 0b101u, ring));
  EXPECT_EQ(a->clear_los_dwells, 2);
  EXPECT_EQ(a->clear_mask, 0b101u);
  // Canonical ring poses for the credited indices count as visited now.
  EXPECT_TRUE(q.isVantageVisited(P(2, 0), 0.5f));
  EXPECT_TRUE(q.isVantageVisited(P(-1, -1.7f), 0.5f));
  EXPECT_FALSE(q.isVantageVisited(P(-1, 1.7f), 0.5f));  // index 1 not dwelled

  // Idempotent: the same mask again credits nothing and returns false.
  EXPECT_FALSE(q.mergePeerDwells(1, 0b101u, ring));
  EXPECT_EQ(a->clear_los_dwells, 2);
  EXPECT_EQ(a->clear_mask, 0b101u);

  // Union: a superset mask credits only the newly set bit (index 1).
  EXPECT_TRUE(q.mergePeerDwells(1, 0b111u, ring));
  EXPECT_EQ(a->clear_los_dwells, 3);
  EXPECT_EQ(a->clear_mask, 0b111u);
  EXPECT_TRUE(q.isVantageVisited(P(-1, 1.7f), 0.5f));
}

// A local dwell and a peer merge on the same index credit it exactly once.
TEST(TargetQueue, MergePeerDwellsNoDoubleCountWithLocal) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);
  Target* a = q.activate();
  ASSERT_NE(a, nullptr);

  const std::vector<Eigen::Vector3f> ring = {P(2, 0), P(-1, 1.7f),
                                              P(-1, -1.7f)};

  q.recordVantageDwell(ring[0], /*los_clear=*/true, /*vantage_index=*/0);
  EXPECT_EQ(a->clear_los_dwells, 1);
  EXPECT_EQ(a->clear_mask, 0b001u);

  // Peer reports indices 0 and 1; only index 1 is new to us.
  EXPECT_TRUE(q.mergePeerDwells(1, 0b011u, ring));
  EXPECT_EQ(a->clear_los_dwells, 2);
  EXPECT_EQ(a->clear_mask, 0b011u);
}

// mergePeerDwells is a safe no-op on an unknown target id and on a DONE target.
TEST(TargetQueue, MergePeerDwellsUnknownOrDoneNoOp) {
  TargetQueue q;
  q.ingest(1, P(0, 0), 0.3f, 0.0f, kDedup);
  const std::vector<Eigen::Vector3f> ring = {P(2, 0)};

  // Unknown id -> false, nothing credited.
  EXPECT_FALSE(q.mergePeerDwells(999, 0b1u, ring));

  // DONE target -> false, clear state untouched.
  q.activate();
  q.markActiveDone();
  EXPECT_FALSE(q.mergePeerDwells(1, 0b1u, ring));
  EXPECT_EQ(q.targets().front().clear_mask, 0u);
  EXPECT_EQ(q.targets().front().clear_los_dwells, 0);
}
