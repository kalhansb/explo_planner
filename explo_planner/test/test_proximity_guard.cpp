#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "explo_planner/proximity_guard.hpp"

using namespace explo_planner;

namespace {

/// Local receipt time passed to onPeerPose / evaluate — everything in the
/// guard runs on the LOCAL clock, mirroring Coordination.
rclcpp::Time at(double sec) {
  return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
}

Eigen::Vector3f v(float x, float y, float z = 0.0f) {
  return Eigen::Vector3f(x, y, z);
}

ProximityGuard::Config cfg() {
  ProximityGuard::Config c;
  c.enabled = true;
  c.hold_dist_m = 3.0f;
  c.resume_dist_m = 4.0f;
  c.pose_stale_sec = 3.0f;
  c.peer_static_sec = 10.0f;
  c.parked_keep_dist_m = 1.5f;
  c.peer_static_move_m = 0.3f;
  c.hold_release_stale_sec = 10.0f;
  return c;
}

}  // namespace

// 1. Disabled: never holds, whatever is tracked.
TEST(ProximityGuard, DisabledNeverHolds) {
  auto c = cfg();
  c.enabled = false;
  ProximityGuard g(c, "rama");
  g.onPeerPose("atlas", v(1.0f, 0.0f), at(100.0));
  EXPECT_FALSE(g.evaluate(v(0, 0), at(100.0), false).hold);
  EXPECT_FALSE(g.evaluate(v(0, 0), at(100.0), true).hold);
}

// 2. Right of way is the lex-smaller id: "rama" yields to "atlas" nearby,
//    but "atlas" never yields to "rama" — exactly one of the pair holds.
TEST(ProximityGuard, YieldsOnlyToLexSmallerPeer) {
  ProximityGuard loser(cfg(), "rama");
  loser.onPeerPose("atlas", v(2.0f, 0.0f), at(100.0));
  const auto d = loser.evaluate(v(0, 0), at(100.0), false);
  EXPECT_TRUE(d.hold);
  EXPECT_EQ(d.peer_id, "atlas");
  EXPECT_NEAR(d.dist_m, 2.0f, 1e-4f);

  ProximityGuard winner(cfg(), "atlas");
  winner.onPeerPose("rama", v(2.0f, 0.0f), at(100.0));
  EXPECT_FALSE(winner.evaluate(v(0, 0), at(100.0), false).hold);
}

// 3. Self-echoes and empty ids are dropped.
TEST(ProximityGuard, SelfEchoIgnored) {
  ProximityGuard g(cfg(), "rama");
  g.onPeerPose("rama", v(0.5f, 0.0f), at(100.0));
  g.onPeerPose("", v(0.5f, 0.0f), at(100.0));
  EXPECT_EQ(g.trackedPeerCount(), 0u);
  EXPECT_FALSE(g.evaluate(v(0, 0), at(100.0), false).hold);
}

// 4. Hysteresis: enter below hold_dist only; once holding, stay held inside
//    resume_dist and release only beyond it.
TEST(ProximityGuard, HysteresisBand) {
  ProximityGuard g(cfg(), "rama");

  // 3.5 m: outside hold_dist -> a drive may continue.
  g.onPeerPose("atlas", v(3.5f, 0.0f), at(100.0));
  EXPECT_FALSE(g.evaluate(v(0, 0), at(100.0), false).hold);

  // 2.9 m: inside hold_dist -> hold starts.
  g.onPeerPose("atlas", v(2.9f, 0.0f), at(101.0));
  EXPECT_TRUE(g.evaluate(v(0, 0), at(101.0), false).hold);

  // Holding, peer back at 3.5 m: inside resume_dist -> still held.
  g.onPeerPose("atlas", v(3.5f, 0.0f), at(102.0));
  EXPECT_TRUE(g.evaluate(v(0, 0), at(102.0), true).hold);

  // Holding, peer at 4.1 m: beyond resume_dist -> released.
  g.onPeerPose("atlas", v(4.1f, 0.0f), at(103.0));
  EXPECT_FALSE(g.evaluate(v(0, 0), at(103.0), true).hold);
}

// 5. An inverted hysteresis band is sanitised at construction.
TEST(ProximityGuard, InvertedBandSanitised) {
  auto c = cfg();
  c.resume_dist_m = 2.0f;  // below hold_dist 3.0
  ProximityGuard g(c, "rama");
  EXPECT_FLOAT_EQ(g.config().resume_dist_m, g.config().hold_dist_m);
}

// 6. Stale data cannot START a hold (never brake on a ghost)...
TEST(ProximityGuard, StaleDataCannotStartHold) {
  ProximityGuard g(cfg(), "rama");
  g.onPeerPose("atlas", v(1.0f, 0.0f), at(100.0));
  // 3.5 s later: past pose_stale_sec (3.0) -> no new hold.
  EXPECT_FALSE(g.evaluate(v(0, 0), at(103.5), false).hold);
}

// 7. ...but an active hold survives up to hold_release_stale_sec without
//    data, then releases.
TEST(ProximityGuard, HoldSurvivesShortSilenceThenReleases) {
  ProximityGuard g(cfg(), "rama");
  g.onPeerPose("atlas", v(1.0f, 0.0f), at(100.0));
  EXPECT_TRUE(g.evaluate(v(0, 0), at(100.0), false).hold);
  // 5 s of silence: within the 10 s release window -> still held.
  EXPECT_TRUE(g.evaluate(v(0, 0), at(105.0), true).hold);
  // 11 s of silence: past the window -> released.
  EXPECT_FALSE(g.evaluate(v(0, 0), at(111.0), true).hold);
}

// 8. A clock rewind (bag loop) makes the track stale instead of asserting or
//    holding forever; a fresh observation re-arms it.
TEST(ProximityGuard, ClockRewindTreatedAsStale) {
  ProximityGuard g(cfg(), "rama");
  g.onPeerPose("atlas", v(1.0f, 0.0f), at(100.0));
  EXPECT_FALSE(g.evaluate(v(0, 0), at(50.0), false).hold);
  g.onPeerPose("atlas", v(1.0f, 0.0f), at(50.0));
  EXPECT_TRUE(g.evaluate(v(0, 0), at(50.0), false).hold);
}

// 9. A parked peer (unmoved past peer_static_sec) neither starts nor
//    sustains a hold — it is the costmap's job, and holding deadlocks.
TEST(ProximityGuard, ParkedPeerReleased) {
  ProximityGuard g(cfg(), "rama");
  // Same pose re-observed every second for 11 s (fresh data throughout).
  for (int i = 0; i <= 11; ++i)
    g.onPeerPose("atlas", v(2.0f, 0.0f), at(100.0 + i));
  // First sighting counted as motion, but 11 s have passed since: parked.
  EXPECT_FALSE(g.evaluate(v(0, 0), at(111.0), false).hold);
  EXPECT_FALSE(g.evaluate(v(0, 0), at(111.0), true).hold);
  EXPECT_EQ(g.evaluate(v(0, 0), at(111.0), true).note, "parked");

  // The peer drives off again: motion re-arms the hold immediately.
  g.onPeerPose("atlas", v(2.5f, 0.0f), at(112.0));
  EXPECT_TRUE(g.evaluate(v(0, 0), at(112.0), false).hold);
}

// 10. Localisation jitter around the anchor is not motion: samples wobbling
//     within peer_static_move_m never refresh last_moved.
TEST(ProximityGuard, JitterIsNotMotion) {
  ProximityGuard g(cfg(), "rama");
  // ±0.1 m wobble — every sample within 0.3 m of wherever the anchor
  // latched (the first sample), so last_moved never refreshes.
  for (int i = 0; i <= 11; ++i) {
    const float dx = (i % 2 == 0) ? 0.1f : -0.1f;
    g.onPeerPose("atlas", v(2.0f + dx, 0.0f), at(100.0 + i));
  }
  EXPECT_FALSE(g.evaluate(v(0, 0), at(111.0), false).hold);
}

// 11. First sighting counts as moving: a peer that appears already inside
//     the hold disc is yielded to at once.
TEST(ProximityGuard, FirstSightingCountsAsMoving) {
  ProximityGuard g(cfg(), "rama");
  g.onPeerPose("atlas", v(1.5f, 0.0f), at(100.0));
  EXPECT_TRUE(g.evaluate(v(0, 0), at(100.0), false).hold);
}

// 12. Distances are XY only — a peer on other terrain 10 m up still holds.
TEST(ProximityGuard, DistanceIgnoresZ) {
  ProximityGuard g(cfg(), "rama");
  g.onPeerPose("atlas", v(2.0f, 0.0f, 10.0f), at(100.0));
  const auto d = g.evaluate(v(0, 0, 0), at(100.0), false);
  EXPECT_TRUE(d.hold);
  EXPECT_NEAR(d.dist_m, 2.0f, 1e-4f);
}

// 13. Multiple outranking peers: any one of them holds, and the nearest is
//     the one reported.
TEST(ProximityGuard, NearestOffenderReported) {
  ProximityGuard g(cfg(), "zulu");
  g.onPeerPose("atlas", v(2.5f, 0.0f), at(100.0));
  g.onPeerPose("rama", v(0.0f, 1.5f), at(100.0));
  const auto d = g.evaluate(v(0, 0), at(100.0), false);
  EXPECT_TRUE(d.hold);
  EXPECT_EQ(d.peer_id, "rama");
  EXPECT_NEAR(d.dist_m, 1.5f, 1e-4f);
}

// 14. Latest-per-peer: a peer that moved away updates its single track (no
//     duplicate entries, and the old close pose no longer holds).
TEST(ProximityGuard, LatestPoseWinsPerPeer) {
  ProximityGuard g(cfg(), "rama");
  g.onPeerPose("atlas", v(1.0f, 0.0f), at(100.0));
  g.onPeerPose("atlas", v(8.0f, 0.0f), at(101.0));
  EXPECT_EQ(g.trackedPeerCount(), 1u);
  EXPECT_FALSE(g.evaluate(v(0, 0), at(101.0), false).hold);
}

// 15. The parked release has a floor: a peer parked INSIDE parked_keep_dist_m
//     still holds — "it stopped" is no licence to drive even closer. Beyond
//     the floor test 9 applies.
TEST(ProximityGuard, ParkedPeerInsideFloorStillHolds) {
  ProximityGuard g(cfg(), "rama");
  for (int i = 0; i <= 11; ++i)
    g.onPeerPose("atlas", v(1.0f, 0.0f), at(100.0 + i));
  // Parked (unmoved 11 s) but at 1.0 m < the 1.5 m floor: the hold stands.
  EXPECT_TRUE(g.evaluate(v(0, 0), at(111.0), false).hold);
  EXPECT_TRUE(g.evaluate(v(0, 0), at(111.0), true).hold);
}

// 16. Non-finite peer positions are rejected at ingest — a NaN from a
//     diverged localiser fails every distance comparison, which would read
//     as "no peer nearby" and silently disarm the guard for that peer.
TEST(ProximityGuard, NonFinitePoseRejected) {
  ProximityGuard g(cfg(), "rama");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  g.onPeerPose("atlas", v(nan, 0.0f), at(100.0));
  EXPECT_EQ(g.trackedPeerCount(), 0u);
  // A good fix arrives, then a NaN: the NaN is dropped, the track stands.
  g.onPeerPose("atlas", v(1.0f, 0.0f), at(101.0));
  g.onPeerPose("atlas", v(nan, nan), at(102.0));
  EXPECT_EQ(g.trackedPeerCount(), 1u);
  EXPECT_TRUE(g.evaluate(v(0, 0), at(102.0), false).hold);
}

// 17. A no-hold decision names the nearest outranking peer and says WHY, so
//     a release log can tell cleared-off from parked from comms-lost.
TEST(ProximityGuard, ReleaseNoteExplains) {
  ProximityGuard g(cfg(), "rama");
  EXPECT_EQ(g.evaluate(v(0, 0), at(100.0), true).note, "no-peer");
  g.onPeerPose("atlas", v(8.0f, 0.0f), at(100.0));
  const auto d = g.evaluate(v(0, 0), at(100.0), true);
  EXPECT_FALSE(d.hold);
  EXPECT_EQ(d.note, "clear");
  EXPECT_EQ(d.peer_id, "atlas");
  EXPECT_NEAR(d.dist_m, 8.0f, 1e-4f);
  // Silence past the release window: stale.
  EXPECT_EQ(g.evaluate(v(0, 0), at(111.0), true).note, "stale");
}

// 18. A nearer-but-stale track must not mask a fresh moving peer inside the
//     trigger: filtered peers only feed the note, never veto a hold.
TEST(ProximityGuard, StalePeerDoesNotMaskFreshOne) {
  ProximityGuard g(cfg(), "zulu");
  g.onPeerPose("atlas", v(1.0f, 0.0f), at(100.0));  // will go stale
  g.onPeerPose("rama", v(2.5f, 0.0f), at(104.0));   // fresh, moving, inside
  const auto d = g.evaluate(v(0, 0), at(104.0), false);
  EXPECT_TRUE(d.hold);
  EXPECT_EQ(d.peer_id, "rama");
}

// 19. Garbage config values fall back to the compiled defaults instead of
//     arming the guard with NaN/negative thresholds (a NaN threshold fails
//     every comparison, silently disabling the guard).
TEST(ProximityGuard, GarbageConfigSanitised) {
  auto c = cfg();
  c.hold_dist_m = std::numeric_limits<float>::quiet_NaN();
  c.pose_stale_sec = -1.0f;
  ProximityGuard g(c, "rama");
  const ProximityGuard::Config dflt;
  EXPECT_FLOAT_EQ(g.config().hold_dist_m, dflt.hold_dist_m);
  EXPECT_FLOAT_EQ(g.config().pose_stale_sec, dflt.pose_stale_sec);
  // The band fix runs after: cfg()'s resume (4.0) < the restored hold (5.0).
  EXPECT_FLOAT_EQ(g.config().resume_dist_m, dflt.hold_dist_m);
}
