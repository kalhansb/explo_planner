#include <gtest/gtest.h>

#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <explo_planner_msgs/msg/robot_intent.hpp>

#include "explo_planner/coordination.hpp"

using namespace explo_planner;

namespace {

explo_planner_msgs::msg::RobotIntent makeIntent(const std::string& robot_id,
                                          float gx, float gy,
                                          float rx, float ry,
                                          double stamp_sec,
                                          float ttl_sec = 5.0f,
                                          float radius = 5.0f,
                                          bool exploit = false,
                                          uint32_t target_id = 0,
                                          uint32_t dwelled_mask = 0) {
  explo_planner_msgs::msg::RobotIntent msg;
  msg.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp_sec * 1e9),
                                   RCL_ROS_TIME);
  msg.header.frame_id = "map";
  msg.robot_id = robot_id;
  msg.goal_pos.x = gx;
  msg.goal_pos.y = gy;
  msg.goal_pos.z = 0.0;
  msg.yaw = 0.0f;
  msg.robot_pos.x = rx;
  msg.robot_pos.y = ry;
  msg.robot_pos.z = 0.0;
  msg.claim_radius_m = radius;
  msg.ttl_sec = ttl_sec;
  msg.planner_type = 0;
  msg.exploit = exploit;
  msg.target_id = target_id;
  msg.dwelled_mask = dwelled_mask;
  return msg;
}

/// Local receipt time passed to onIntent (claim expiry = receipt + ttl,
/// measured on the LOCAL clock — the producer's header.stamp is metadata).
rclcpp::Time at(double sec) {
  return rclcpp::Time(static_cast<int64_t>(sec * 1e9), RCL_ROS_TIME);
}

/// Peer exploit claim for the rendezvous-barrier tests: the producer is holding
/// (or driving to) vantage (gx, gy) on `target_id` while its last heartbeat put
/// it at (rx, ry) and declared `staged` — true only if that peer was in its own
/// dwell state on (gx, gy) when it published. The pose is still filled in
/// because MinPos uses it; the barrier does not. `expiry_sec` is on the same
/// local clock as at()/now.
Coordination::Claim exploitClaim(const std::string& robot_id,
                                 uint32_t target_id,
                                 float gx, float gy,
                                 float rx, float ry,
                                 bool staged = false,
                                 double expiry_sec = 1000.0) {
  Coordination::Claim c;
  c.robot_id = robot_id;
  c.goal_pos = Eigen::Vector3f(gx, gy, 0.0f);
  c.robot_pos = Eigen::Vector3f(rx, ry, 0.0f);
  c.radius_m = 0.75f;   // exploit-scale: one vantage angle around a trunk
  c.expiry = at(expiry_sec);
  c.exploit = true;
  c.target_id = target_id;
  c.staged = staged;
  return c;
}

}  // namespace

// 1. Disabled mode: claimMatching always returns nullptr regardless of
//    stored claims; buildIntent still works.
TEST(Coordination, DisabledModeReturnsNullptr) {
  Coordination c(false, "atlas");
  EXPECT_FALSE(c.enabled());

  c.onIntent(makeIntent("rama", 0.0f, 0.0f, 1.0f, 0.0f, 100.0), at(100.0));
  // Even though a claim is stored, claimMatching may match if called -
  // the disabled flag is consulted by the planner, not by the helper -
  // but activePeerCount should still reflect what was stored.
  EXPECT_EQ(c.activePeerCount(), 1u);

  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(2.0f, 2.0f, 0.0f);
  vp.yaw = 0.5f;
  auto msg = c.buildIntent(vp,
                            Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                            rclcpp::Time(static_cast<int64_t>(100e9),
                                         RCL_ROS_TIME),
                            5.0f, 4.0f, 0, "map");
  EXPECT_EQ(msg.robot_id, "atlas");
  EXPECT_FLOAT_EQ(msg.goal_pos.x, 2.0f);
  EXPECT_FLOAT_EQ(msg.claim_radius_m, 4.0f);
}

// 2. Self-intent dropped: no claim added when robot_id matches self.
TEST(Coordination, SelfIntentDropped) {
  Coordination c(true, "atlas");
  c.onIntent(makeIntent("atlas", 0.0f, 0.0f, 1.0f, 0.0f, 100.0), at(100.0));
  EXPECT_EQ(c.activePeerCount(), 0u);
}

// 3. TTL: prune at t < expiry keeps the claim, at t > expiry removes it.
TEST(Coordination, TtlExpiry) {
  Coordination c(true, "atlas");
  c.onIntent(makeIntent("rama", 5.0f, 5.0f, 6.0f, 5.0f, /*stamp=*/100.0,
                         /*ttl=*/5.0f),
              at(100.0));

  c.prune(rclcpp::Time(static_cast<int64_t>(102e9), RCL_ROS_TIME));
  EXPECT_EQ(c.activePeerCount(), 1u);

  c.prune(rclcpp::Time(static_cast<int64_t>(106e9), RCL_ROS_TIME));
  EXPECT_EQ(c.activePeerCount(), 0u);
}

// 3b. Clock-offset robustness: expiry is based on the LOCAL receipt time, not
//     the producer's header.stamp. Field fleets are not clock-synchronised
//     (offsets of hours have been observed), so a peer stamp far ahead of the
//     local clock must not yield an immortal claim, and one far behind must
//     not yield a claim that expires on arrival.
TEST(Coordination, TtlUsesLocalReceiptTimeNotPeerStamp) {
  Coordination c(true, "atlas");

  // Peer clock ~1.25 h AHEAD: stamp 4600 s, received at local t=100 s.
  // Stamp-based expiry (4605 s) would keep this claim alive for over an hour
  // of local time; receipt-based expiry ends it 5 s after we heard the peer.
  c.onIntent(makeIntent("rama", 0, 0, 1, 0, /*stamp=*/4600.0, /*ttl=*/5.0f),
             at(100.0));
  c.prune(at(104.0));
  EXPECT_EQ(c.activePeerCount(), 1u);   // still fresh locally
  c.prune(at(106.0));
  EXPECT_EQ(c.activePeerCount(), 0u);   // expired 5 s after receipt

  // Peer clock far BEHIND: stamp 10 s, received at local t=100 s. Stamp-based
  // expiry (15 s) is already in the local past -> claim dead on arrival, i.e.
  // no deconfliction at all. Receipt-based expiry survives the full TTL.
  c.onIntent(makeIntent("rama", 0, 0, 1, 0, /*stamp=*/10.0, /*ttl=*/5.0f),
             at(100.0));
  c.prune(at(104.0));
  EXPECT_EQ(c.activePeerCount(), 1u);
}

// 4. Latest-per-peer: same robot_id, new claim overwrites old one.
TEST(Coordination, LatestPerPeerReplacement) {
  Coordination c(true, "atlas");
  c.onIntent(makeIntent("rama", 1.0f, 1.0f, 2.0f, 2.0f, 100.0), at(100.0));
  c.onIntent(makeIntent("rama", 9.0f, 9.0f, 8.0f, 8.0f, 101.0), at(101.0));
  EXPECT_EQ(c.activePeerCount(), 1u);

  const auto* claim = c.claimMatching(Eigen::Vector3f(9.0f, 9.0f, 0.0f), 2.0f);
  ASSERT_NE(claim, nullptr);
  EXPECT_NEAR(claim->goal_pos.x(), 9.0f, 1e-3f);
}

// 5. MinPos -- peer closer -> self yields.
TEST(Coordination, MinPosPeerCloserSelfYields) {
  Coordination c(true, "atlas");
  Coordination::Claim peer;
  peer.robot_id = "rama";
  peer.goal_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  peer.robot_pos = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  peer.radius_m = 5.0f;
  EXPECT_FALSE(c.selfWinsAgainst(Eigen::Vector3f(10.0f, 0.0f, 0.0f),
                                  Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                  peer, "atlas"));
}

// 6. MinPos -- self closer -> self wins.
TEST(Coordination, MinPosSelfCloserSelfWins) {
  Coordination c(true, "atlas");
  Coordination::Claim peer;
  peer.robot_id = "rama";
  peer.goal_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  peer.robot_pos = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  peer.radius_m = 5.0f;
  EXPECT_TRUE(c.selfWinsAgainst(Eigen::Vector3f(1.0f, 0.0f, 0.0f),
                                 Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                 peer, "atlas"));
}

// 7. MinPos exact tie -> lex tiebreak; "atlas" < "rama" so atlas wins.
TEST(Coordination, MinPosExactTieLexTiebreak) {
  Coordination c(true, "atlas");
  Coordination::Claim peer;
  peer.robot_id = "rama";
  peer.goal_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  peer.robot_pos = Eigen::Vector3f(-5.0f, 0.0f, 0.0f);
  peer.radius_m = 5.0f;
  EXPECT_TRUE(c.selfWinsAgainst(Eigen::Vector3f(5.0f, 0.0f, 0.0f),
                                 Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                 peer, "atlas"));

  // Reverse the ids -- now self is "rama" against peer "atlas".
  peer.robot_id = "atlas";
  EXPECT_FALSE(c.selfWinsAgainst(Eigen::Vector3f(5.0f, 0.0f, 0.0f),
                                  Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                  peer, "rama"));
}

// 8. claimMatching picks the closest peer when multiple overlap.
TEST(Coordination, ClaimMatchingPicksClosestPeer) {
  Coordination c(true, "atlas");
  // Two peers both claim within match radius of (0,0); peer "rama" is
  // 1 m away, peer "ravana" is 5 m away.
  Coordination::Claim p1;
  p1.robot_id = "rama";
  p1.goal_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  p1.robot_pos = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  p1.radius_m = 5.0f;
  p1.expiry = rclcpp::Time(static_cast<int64_t>(1000e9), RCL_ROS_TIME);
  c.injectClaimForTest(p1);

  Coordination::Claim p2;
  p2.robot_id = "ravana";
  p2.goal_pos = Eigen::Vector3f(0.5f, 0.0f, 0.0f);
  p2.robot_pos = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
  p2.radius_m = 5.0f;
  p2.expiry = rclcpp::Time(static_cast<int64_t>(1000e9), RCL_ROS_TIME);
  c.injectClaimForTest(p2);

  const auto* match = c.claimMatching(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 5.0f);
  ASSERT_NE(match, nullptr);
  EXPECT_EQ(match->robot_id, "rama");
}

// 9. claimMatching returns nullptr when candidate is outside the match
//    radius of every active peer claim.
TEST(Coordination, ClaimMatchingNullptrOutsideRadius) {
  Coordination c(true, "atlas");
  Coordination::Claim p;
  p.robot_id = "rama";
  p.goal_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  p.robot_pos = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
  p.radius_m = 1.0f;
  p.expiry = rclcpp::Time(static_cast<int64_t>(1000e9), RCL_ROS_TIME);
  c.injectClaimForTest(p);

  // Candidate 10 m from any active peer claim with match_radius=5 -> nullptr.
  EXPECT_EQ(c.claimMatching(Eigen::Vector3f(10.0f, 10.0f, 0.0f), 5.0f),
            nullptr);
}

// 10. Two peers with non-overlapping claims: claimMatching returns the
//     right one for each candidate position.
TEST(Coordination, TwoPeersNonOverlappingClaims) {
  Coordination c(true, "atlas");
  Coordination::Claim p1;
  p1.robot_id = "rama";
  p1.goal_pos = Eigen::Vector3f(-10.0f, 0.0f, 0.0f);
  p1.robot_pos = Eigen::Vector3f(-11.0f, 0.0f, 0.0f);
  p1.radius_m = 2.0f;
  p1.expiry = rclcpp::Time(static_cast<int64_t>(1000e9), RCL_ROS_TIME);
  c.injectClaimForTest(p1);

  Coordination::Claim p2;
  p2.robot_id = "ravana";
  p2.goal_pos = Eigen::Vector3f(10.0f, 0.0f, 0.0f);
  p2.robot_pos = Eigen::Vector3f(11.0f, 0.0f, 0.0f);
  p2.radius_m = 2.0f;
  p2.expiry = rclcpp::Time(static_cast<int64_t>(1000e9), RCL_ROS_TIME);
  c.injectClaimForTest(p2);

  const auto* m1 = c.claimMatching(Eigen::Vector3f(-10.0f, 0.0f, 0.0f), 2.0f);
  ASSERT_NE(m1, nullptr);
  EXPECT_EQ(m1->robot_id, "rama");

  const auto* m2 = c.claimMatching(Eigen::Vector3f(10.0f, 0.0f, 0.0f), 2.0f);
  ASSERT_NE(m2, nullptr);
  EXPECT_EQ(m2->robot_id, "ravana");

  // Halfway between -- neither disc covers it.
  EXPECT_EQ(c.claimMatching(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 2.0f),
            nullptr);
}

// 11. peerDwellUnion ORs dwelled_mask across all active exploit claims on the
//     same target_id, and ignores non-exploit claims and other target ids.
TEST(Coordination, PeerDwellUnion) {
  Coordination c(true, "atlas");

  // Two peers exploiting target 7, each having dwelled a different vantage.
  c.onIntent(makeIntent("rama", 0, 0, 1, 0, /*stamp=*/100.0, 5.0f, 5.0f,
                         /*exploit=*/true, /*target_id=*/7,
                         /*dwelled_mask=*/0b001u),
              at(100.0));
  c.onIntent(makeIntent("ravana", 0, 0, 2, 0, /*stamp=*/100.0, 5.0f, 5.0f,
                         /*exploit=*/true, /*target_id=*/7,
                         /*dwelled_mask=*/0b100u),
              at(100.0));
  // A peer exploiting a DIFFERENT trunk must not leak into target 7's union.
  c.onIntent(makeIntent("kumbha", 0, 0, 3, 0, /*stamp=*/100.0, 5.0f, 5.0f,
                         /*exploit=*/true, /*target_id=*/8,
                         /*dwelled_mask=*/0b010u),
              at(100.0));
  // An EXPLORE claim (exploit=false) carrying a stray mask is ignored.
  c.onIntent(makeIntent("vibhishana", 0, 0, 4, 0, /*stamp=*/100.0, 5.0f, 5.0f,
                         /*exploit=*/false, /*target_id=*/7,
                         /*dwelled_mask=*/0b010u),
              at(100.0));

  EXPECT_EQ(c.peerDwellUnion(7), 0b101u);   // rama | ravana, not vibhishana
  EXPECT_EQ(c.peerDwellUnion(8), 0b010u);   // kumbha only
  EXPECT_EQ(c.peerDwellUnion(99), 0u);      // no claims on this trunk
}

// 12. Exploit fields round-trip through buildIntent -> onIntent; the default
//     (explore) build leaves them zeroed so exploration claims are unchanged.
TEST(Coordination, ExploitFieldsRoundTrip) {
  Coordination producer(true, "atlas");
  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(3.0f, 4.0f, 0.0f);
  vp.yaw = 1.0f;
  const rclcpp::Time now(static_cast<int64_t>(100e9), RCL_ROS_TIME);

  auto msg = producer.buildIntent(vp, Eigen::Vector3f(0, 0, 0), now,
                                   5.0f, 0.75f, 2, "map",
                                   /*exploit=*/true, /*target_id=*/42,
                                   /*dwelled_mask=*/0b1011u, /*staged=*/true);
  EXPECT_TRUE(msg.exploit);
  EXPECT_EQ(msg.target_id, 42u);
  EXPECT_EQ(msg.dwelled_mask, 0b1011u);
  EXPECT_TRUE(msg.staged);

  // A peer ingesting that intent stores the exploit fields and surfaces them
  // through peerDwellUnion and the rendezvous barrier.
  Coordination peer(true, "rama");
  peer.onIntent(msg, now);
  EXPECT_EQ(peer.activePeerCount(), 1u);
  EXPECT_EQ(peer.peerDwellUnion(42), 0b1011u);
  EXPECT_EQ(peer.firstUnstagedExploitPeer(42, now), nullptr);

  // The same claim republished while driving holds that peer's barrier: staging
  // travels on the wire, it is not re-derived from the pose in the claim.
  auto driving = producer.buildIntent(vp, Eigen::Vector3f(0, 0, 0), now,
                                       5.0f, 0.75f, 2, "map",
                                       /*exploit=*/true, /*target_id=*/42,
                                       /*dwelled_mask=*/0b1011u,
                                       /*staged=*/false);
  peer.onIntent(driving, now);
  EXPECT_NE(peer.firstUnstagedExploitPeer(42, now), nullptr);

  // Default (explore) build leaves the exploit fields zeroed and unstaged.
  auto explore = producer.buildIntent(vp, Eigen::Vector3f(0, 0, 0), now,
                                       5.0f, 4.0f, 0, "map");
  EXPECT_FALSE(explore.exploit);
  EXPECT_EQ(explore.target_id, 0u);
  EXPECT_EQ(explore.dwelled_mask, 0u);
  EXPECT_FALSE(explore.staged);
}

// 14. claimMatching sizes each claim's exclusion disc by the radius the
//     CLAIMER advertised, not by the receiver's own match radius. The radius
//     is phase-dependent (exploration ~8-10 m, one exploit vantage ~0.75 m),
//     so evaluating every claim at the receiver's scale made an exploring
//     robot veto a whole 8 m disc around a peer that was merely dwelling at a
//     trunk.
TEST(Coordination, ClaimMatchingUsesTheClaimersRadius) {
  Coordination c(true, "atlas");
  Coordination::Claim p;
  p.robot_id = "rama";
  p.goal_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  p.robot_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  p.radius_m = 0.75f;  // exploit-scale: one vantage angle around a trunk
  p.expiry = rclcpp::Time(static_cast<int64_t>(1000e9), RCL_ROS_TIME);
  c.injectClaimForTest(p);

  // 3 m away is far outside the peer's own 0.75 m claim, even though the
  // caller asks with an exploration-scale 8 m match radius.
  EXPECT_EQ(c.claimMatching(Eigen::Vector3f(3.0f, 0.0f, 0.0f), 8.0f), nullptr);
  // Inside the peer's advertised disc it still matches.
  EXPECT_NE(c.claimMatching(Eigen::Vector3f(0.5f, 0.0f, 0.0f), 8.0f), nullptr);
}

// 15. radius_m <= 0 means the peer advertised nothing usable (older node), so
//     fall back to the receiver's own match radius rather than never matching.
TEST(Coordination, ClaimMatchingFallsBackWhenRadiusUnset) {
  Coordination c(true, "atlas");
  Coordination::Claim p;
  p.robot_id = "rama";
  p.goal_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  p.robot_pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  p.radius_m = 0.0f;
  p.expiry = rclcpp::Time(static_cast<int64_t>(1000e9), RCL_ROS_TIME);
  c.injectClaimForTest(p);

  EXPECT_NE(c.claimMatching(Eigen::Vector3f(3.0f, 0.0f, 0.0f), 8.0f), nullptr);
  EXPECT_EQ(c.claimMatching(Eigen::Vector3f(3.0f, 0.0f, 0.0f), 1.0f), nullptr);
}

// ==================================================================
// Peer-advertised scalars are bounded before they enter the claim table
// ==================================================================
// claimMatching() deliberately evaluates each claim at the radius its CLAIMER
// advertised, because the disc size is phase-dependent (~10 m while exploring,
// ~0.75 m while holding one vantage angle around a trunk). That is correct, but
// it means two numbers straight off the wire decide how much of our candidate
// set a peer can veto and for how long. Unbounded, either one is a
// denial-of-service on our own planner from a single malformed or
// version-skewed publish.

TEST(Coordination, HugePeerRadiusIsBoundedByOurOwnExplorationDisc) {
  // 5 m bound = what this planner considers the largest legitimate claim.
  Coordination c(true, "atlas", /*max_claim_radius_m=*/5.0f);
  c.onIntent(makeIntent("rama", 0.0f, 0.0f, 1.0f, 0.0f, 100.0,
                        /*ttl=*/5.0f, /*radius=*/1000.0f),
             at(100.0));
  ASSERT_EQ(c.activePeerCount(), 1u);

  // Inside the bound: still contested, so deconfliction keeps working.
  EXPECT_NE(c.claimMatching(Eigen::Vector3f(3.0f, 0.0f, 0.0f), 1.0f), nullptr);
  // Beyond the bound: the 1000 m disc the peer asked for is gone. Without the
  // clamp this candidate 200 m away was vetoed by a peer standing at the origin.
  EXPECT_EQ(c.claimMatching(Eigen::Vector3f(200.0f, 0.0f, 0.0f), 1.0f), nullptr);
}

TEST(Coordination, InfinitePeerRadiusVetoesNothingBeyondOurOwnMatchRadius) {
  // The specific wire value that defeated the old code: +inf passed the
  // `radius_m > 0` test, so r2 was infinite and EVERY candidate matched — the
  // robot yielded every goal to that peer and stopped exploring, silently.
  // Now it is treated as "peer sent nothing usable" (0), which claimMatching
  // already handles by falling back to our own match radius.
  Coordination c(true, "atlas", /*max_claim_radius_m=*/5.0f);
  const float kInf = std::numeric_limits<float>::infinity();
  c.onIntent(makeIntent("rama", 0.0f, 0.0f, 1.0f, 0.0f, 100.0,
                        /*ttl=*/5.0f, /*radius=*/kInf),
             at(100.0));
  ASSERT_EQ(c.activePeerCount(), 1u);

  EXPECT_NE(c.claimMatching(Eigen::Vector3f(1.0f, 0.0f, 0.0f), 2.0f), nullptr);
  EXPECT_EQ(c.claimMatching(Eigen::Vector3f(50.0f, 0.0f, 0.0f), 2.0f), nullptr);

  // NaN takes the same path.
  Coordination c2(true, "atlas", 5.0f);
  c2.onIntent(makeIntent("rama", 0.0f, 0.0f, 1.0f, 0.0f, 100.0, 5.0f,
                         std::numeric_limits<float>::quiet_NaN()),
              at(100.0));
  EXPECT_EQ(c2.claimMatching(Eigen::Vector3f(50.0f, 0.0f, 0.0f), 2.0f), nullptr);
}

TEST(Coordination, SaneRadiusIsPassedThroughUnchanged) {
  // The clamp must not disturb a same-version peer: an exploration claim at the
  // shared disc size, and the much smaller exploit vantage claim, both survive
  // intact. (Regression guard — a bound that quietly shrank real claims would
  // break MinPos deconfliction rather than protect it.)
  Coordination c(true, "atlas", /*max_claim_radius_m=*/10.0f);
  c.onIntent(makeIntent("rama", 0.0f, 0.0f, 1.0f, 0.0f, 100.0, 5.0f,
                        /*radius=*/10.0f),
             at(100.0));
  const auto* claim = c.claimMatching(Eigen::Vector3f(9.0f, 0.0f, 0.0f), 0.5f);
  ASSERT_NE(claim, nullptr);
  EXPECT_FLOAT_EQ(claim->radius_m, 10.0f);

  Coordination v(true, "atlas", /*max_claim_radius_m=*/10.0f);
  v.onIntent(makeIntent("rama", 0.0f, 0.0f, 1.0f, 0.0f, 100.0, 5.0f,
                        /*radius=*/0.75f, /*exploit=*/true, /*target_id=*/7u),
             at(100.0));
  const auto* vc = v.claimMatching(Eigen::Vector3f(0.5f, 0.0f, 0.0f), 0.75f);
  ASSERT_NE(vc, nullptr);
  EXPECT_FLOAT_EQ(vc->radius_m, 0.75f);
}

TEST(Coordination, HugePeerTtlCannotMakeAClaimImmortal) {
  // Same failure one field over: prune() compares expiry against the local
  // clock, so a peer advertising a huge ttl_sec produced a claim nothing could
  // ever expire — a permanent veto over a disc of the map with no recovery
  // short of restarting the node. Capped at kMaxClaimTtlSec.
  Coordination c(true, "atlas");
  c.onIntent(makeIntent("rama", 0.0f, 0.0f, 1.0f, 0.0f, 100.0,
                        /*ttl=*/1.0e9f),
             at(100.0));
  ASSERT_EQ(c.activePeerCount(), 1u);
  c.prune(at(100.0 + Coordination::kMaxClaimTtlSec - 1.0));
  EXPECT_EQ(c.activePeerCount(), 1u);   // still inside the cap
  c.prune(at(100.0 + Coordination::kMaxClaimTtlSec + 1.0));
  EXPECT_EQ(c.activePeerCount(), 0u);   // and gone once past it
}

TEST(Coordination, NonFiniteTtlExpiresOnTheNextPrune) {
  // A TTL we cannot read must fail SAFE — expire at once rather than persist.
  // Also keeps a non-finite value out of Duration::from_seconds(), and a wild
  // magnitude out of its int64 nanosecond count.
  for (float ttl : {std::numeric_limits<float>::infinity(),
                    -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::quiet_NaN(),
                    -1.0e9f}) {
    Coordination c(true, "atlas");
    c.onIntent(makeIntent("rama", 0.0f, 0.0f, 1.0f, 0.0f, 100.0, ttl),
               at(100.0));
    c.prune(at(100.0));
    EXPECT_EQ(c.activePeerCount(), 0u) << "ttl=" << ttl;
  }
}

// ==================================================================
// Rendezvous barrier: firstUnstagedExploitPeer()
// ==================================================================
// A robot that has reached its exploitation vantage waits in EXPLOIT_DWELL
// until every peer holding an active exploit claim on the SAME target has
// DECLARED itself staged on its own claimed vantage, so the whole team dwells
// simultaneously. The barrier is up for as long as this query returns non-null,
// and the claim it returns is the peer being waited on (logging).

TEST(Coordination, FirstUnstagedExploitPeerNoClaimsIsNullptr) {
  // Solo run, or a target no peer has claimed: nothing to wait for, dwell now.
  Coordination c(true, "atlas");
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(100.0)), nullptr);
}

TEST(Coordination, FirstUnstagedExploitPeerReturnsPeerStillDriving) {
  Coordination c(true, "atlas");
  // rama has claimed the vantage at (10, 0) but its latest heartbeat declared
  // staged=false -- it is driving in, so the barrier holds and names it.
  c.injectClaimForTest(exploitClaim("rama", 7u, 10.0f, 0.0f, 4.0f, 0.0f,
                                    /*staged=*/false));

  const auto* waiting = c.firstUnstagedExploitPeer(7u, at(100.0));
  ASSERT_NE(waiting, nullptr);
  EXPECT_EQ(waiting->robot_id, "rama");
}

TEST(Coordination, FirstUnstagedExploitPeerNullptrOncePeerStages) {
  Coordination c(true, "atlas");
  // Same peer, a later heartbeat: it has entered its dwell on that vantage and
  // says so. Barrier releases.
  c.injectClaimForTest(exploitClaim("rama", 7u, 10.0f, 0.0f, 10.0f, 0.0f,
                                    /*staged=*/true));
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(100.0)), nullptr);
}

TEST(Coordination, FirstUnstagedExploitPeerFlagBeatsGeometry) {
  // The contract: staging is what the PRODUCER declares, and the barrier reads
  // nothing else. Position used to decide it, and could not tell "on the
  // vantage" from "arrived in XY, still rotating beside it" (released ~5 s
  // early, so the team's dwells no longer overlapped) or from "parked at an
  // approach waypoint", which is an exploit hop but not a vantage at all.
  Coordination c(true, "atlas");

  // Sitting EXACTLY on its own goal and still not staged -- the peer has
  // arrived but has not settled its capture yaw. The barrier holds.
  c.injectClaimForTest(exploitClaim("rama", 7u, 3.0f, 0.0f, 3.0f, 0.0f,
                                    /*staged=*/false));
  const auto* waiting = c.firstUnstagedExploitPeer(7u, at(100.0));
  ASSERT_NE(waiting, nullptr);
  EXPECT_EQ(waiting->robot_id, "rama");

  // 50 m from the goal in the claim, but staged=true (e.g. the pose in the
  // claim is one heartbeat stale, or the peer re-anchored): still released,
  // because the distance is never looked at.
  c.injectClaimForTest(exploitClaim("rama", 7u, 3.0f, 0.0f, 53.0f, 0.0f,
                                    /*staged=*/true));
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(100.0)), nullptr);
}

TEST(Coordination, FirstUnstagedExploitPeerIgnoresExploreClaims) {
  Coordination c(true, "atlas");
  // exploit=false: the peer is driving to an exploration goal, not to a vantage
  // on our trunk. It is not part of this rendezvous whatever it declares --
  // waiting for it would couple our dwell to unrelated frontier travel.
  auto explore = exploitClaim("rama", 7u, 10.0f, 0.0f, 0.0f, 0.0f,
                              /*staged=*/false);
  explore.exploit = false;
  c.injectClaimForTest(explore);
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(100.0)), nullptr);
}

TEST(Coordination, FirstUnstagedExploitPeerIgnoresOtherTargets) {
  Coordination c(true, "atlas");
  // Unstaged, but exploiting a DIFFERENT trunk: the barrier is per-target, so
  // that peer's arrival is none of our business.
  c.injectClaimForTest(exploitClaim("rama", 8u, 10.0f, 0.0f, 0.0f, 0.0f,
                                    /*staged=*/false));
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(100.0)), nullptr);
}

TEST(Coordination, FirstUnstagedExploitPeerIgnoresExpiredClaimWithoutPrune) {
  // Dead-peer release path. The caller queries this from EXPLOIT_DWELL, where
  // the PLAN tick -- and therefore prune() -- never runs, so the stale claim of
  // a peer that died on its way in is still in the table, still unstaged.
  // There is deliberately NO prune() call in this test: the query must step
  // over the expired claim itself, otherwise the robot that did arrive dwells
  // forever.
  Coordination c(true, "atlas");
  c.injectClaimForTest(exploitClaim("rama", 7u, 10.0f, 0.0f, 0.0f, 0.0f,
                                    /*staged=*/false, /*expiry_sec=*/100.0));
  ASSERT_EQ(c.activePeerCount(), 1u);   // unpruned, still stored

  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(100.0)), nullptr);  // == now
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(105.0)), nullptr);  // past
  EXPECT_EQ(c.activePeerCount(), 1u);   // and the query is non-mutating

  // Sanity: the very same claim one second BEFORE its expiry does hold us, so
  // the release above is the TTL and not a mis-filtered field.
  EXPECT_NE(c.firstUnstagedExploitPeer(7u, at(99.0)), nullptr);
}

TEST(Coordination, FirstUnstagedExploitPeerReturnsTheUnstagedOfTwo) {
  Coordination c(true, "atlas");
  // rama is dwelling on its vantage; ravana claimed the far side of the trunk
  // and is still driving. Stored in that order, so this also checks that a
  // staged claim is skipped rather than returned.
  c.injectClaimForTest(exploitClaim("rama", 7u, 3.0f, 0.0f, 3.0f, 0.0f,
                                    /*staged=*/true));
  c.injectClaimForTest(exploitClaim("ravana", 7u, -3.0f, 0.0f, 2.0f, 0.0f,
                                    /*staged=*/false));

  const auto* waiting = c.firstUnstagedExploitPeer(7u, at(100.0));
  ASSERT_NE(waiting, nullptr);
  EXPECT_EQ(waiting->robot_id, "ravana");

  // Once ravana stages on its vantage too, the whole team is staged.
  c.injectClaimForTest(exploitClaim("ravana", 7u, -3.0f, 0.0f, -3.0f, 0.0f,
                                    /*staged=*/true));
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(100.0)), nullptr);
}

// ==================================================================
// Selection-time parked-peer contest: stagedExploitPeerWinning()
// ==================================================================
// When the team's synchronised dwells end, both robots re-plan within
// milliseconds and both see the SAME single remaining un-dwelled vantage. Their
// claims on it cross in the air — the intent heartbeat is only ~1 Hz — so both
// drove at the same angle and, in the last field run, collided. This probe
// decides the contest at selection time from claims already in the table: a
// candidate a parked same-target peer would win under MinPos's total order is
// not selectable, so the yield happens before either robot moves. Only PARKED
// (staged) peers contest this way; a driving peer contests through its claim
// disc alone.

TEST(Coordination, StagedExploitPeerWinningNoClaimsIsNullptr) {
  // Solo run, or a target no peer has claimed: nothing contests the angle.
  Coordination c(true, "atlas");
  EXPECT_EQ(c.stagedExploitPeerWinning(7u, Eigen::Vector3f(3.0f, 0.0f, 0.0f),
                                       Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                       "atlas", at(100.0)),
            nullptr);
}

TEST(Coordination, StagedExploitPeerWinningBlocksCandidateWhenPeerCloser) {
  Coordination c(true, "atlas");
  // Trunk 7 at the origin, vantages on a 3 m ring, and (-3, 0) is the last
  // un-dwelled angle. rama is PARKED on the vantage at (0, 3), 4.24 m from that
  // angle; we are parked at (3, 0), a full 6 m away. rama is about to re-plan
  // from a standstill and will take it, so the candidate is not ours to select.
  c.injectClaimForTest(exploitClaim("rama", 7u, 0.0f, 3.0f, 0.0f, 3.0f,
                                    /*staged=*/true));

  const auto* blocking = c.stagedExploitPeerWinning(
      7u, Eigen::Vector3f(-3.0f, 0.0f, 0.0f),
      Eigen::Vector3f(3.0f, 0.0f, 0.0f), "atlas", at(100.0));
  ASSERT_NE(blocking, nullptr);
  EXPECT_EQ(blocking->robot_id, "rama");   // named for the caller's log
  EXPECT_EQ(c.activePeerCount(), 1u);      // and the probe is non-mutating
}

TEST(Coordination, StagedExploitPeerWinningNullptrWhenSelfCloser) {
  // Mirror of the case above — rama is parked on the far side of the trunk and
  // we are the one standing next to the free angle, so it is ours to take. Our
  // id is deliberately the LEXICOGRAPHICALLY LARGER one ("zulu" > "rama"): the
  // tiebreak must not be reached at all when the distances differ, or the
  // farther robot could steal an angle it is nowhere near.
  Coordination c(true, "zulu");
  c.injectClaimForTest(exploitClaim("rama", 7u, 3.0f, 0.0f, 3.0f, 0.0f,
                                    /*staged=*/true));
  EXPECT_EQ(c.stagedExploitPeerWinning(7u, Eigen::Vector3f(-3.0f, 0.0f, 0.0f),
                                       Eigen::Vector3f(0.0f, 3.0f, 0.0f),
                                       "zulu", at(100.0)),
            nullptr);
}

TEST(Coordination, StagedExploitPeerWinningExactTieLexTiebreak) {
  // Exactly equidistant, which happens for real whenever the two robots park on
  // vantages symmetric about the free angle. Candidate at the origin, we at
  // (4, 0), the parked peer at (-4, 0): powers of two, so both squared distances
  // are the SAME float and the comparison genuinely falls through to the id
  // tiebreak rather than being decided by rounding.
  Coordination self_wins(true, "atlas");
  self_wins.injectClaimForTest(exploitClaim("rama", 7u, -4.0f, 0.0f,
                                            -4.0f, 0.0f, /*staged=*/true));
  EXPECT_EQ(self_wins.stagedExploitPeerWinning(
                7u, Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                Eigen::Vector3f(4.0f, 0.0f, 0.0f), "atlas", at(100.0)),
            nullptr);   // "atlas" < "rama" -> ours

  // Swap the ids and nothing else. Now WE lose the tie and the very same
  // geometry blocks the candidate: run on both robots, this pair of outcomes is
  // what guarantees exactly ONE of them admits the vantage.
  Coordination peer_wins(true, "rama");
  peer_wins.injectClaimForTest(exploitClaim("atlas", 7u, -4.0f, 0.0f,
                                            -4.0f, 0.0f, /*staged=*/true));
  const auto* blocking = peer_wins.stagedExploitPeerWinning(
      7u, Eigen::Vector3f(0.0f, 0.0f, 0.0f),
      Eigen::Vector3f(4.0f, 0.0f, 0.0f), "rama", at(100.0));
  ASSERT_NE(blocking, nullptr);
  EXPECT_EQ(blocking->robot_id, "atlas");
}

TEST(Coordination, StagedExploitPeerWinningDrivingPeerNeverBlocks) {
  // Identical geometry to BlocksCandidateWhenPeerCloser — the peer is still the
  // closer robot — but its heartbeat says staged=false, so it is DRIVING and
  // this probe ignores it entirely. A moving peer contests only through its
  // claim disc (claimMatching), because a position contest against a mover would
  // deny an approaching robot every angle on the ring at once: arriving from far
  // away it is farther from all of them than a peer already circling the trunk,
  // and exploitation would serialise instead of running in parallel. The mover's
  // pose on the wire is up to a heartbeat stale anyway.
  Coordination c(true, "atlas");
  c.injectClaimForTest(exploitClaim("rama", 7u, 0.0f, 3.0f, 0.0f, 3.0f,
                                    /*staged=*/false));
  EXPECT_EQ(c.stagedExploitPeerWinning(7u, Eigen::Vector3f(-3.0f, 0.0f, 0.0f),
                                       Eigen::Vector3f(3.0f, 0.0f, 0.0f),
                                       "atlas", at(100.0)),
            nullptr);
}

TEST(Coordination, StagedExploitPeerWinningIgnoresOtherTargetAndExploreClaims) {
  Coordination c(true, "atlas");
  // Both of these peers are staged and both are much closer to the candidate
  // than we are (they sit on it), so only the target/exploit filters can save
  // the candidate. Neither peer is on OUR ring: one is parked at a vantage of a
  // different trunk that happens to be nearby, the other is not exploiting at
  // all. Contesting an angle of trunk 7 against either would strand us with no
  // vantage to take on a trunk nobody else is working.
  c.injectClaimForTest(exploitClaim("rama", 8u, -3.0f, 0.0f, -3.0f, 0.0f,
                                    /*staged=*/true));
  auto explore = exploitClaim("ravana", 7u, -3.0f, 0.0f, -3.0f, 0.0f,
                              /*staged=*/true);
  explore.exploit = false;
  c.injectClaimForTest(explore);

  EXPECT_EQ(c.stagedExploitPeerWinning(7u, Eigen::Vector3f(-3.0f, 0.0f, 0.0f),
                                       Eigen::Vector3f(3.0f, 0.0f, 0.0f),
                                       "atlas", at(100.0)),
            nullptr);
}

TEST(Coordination, StagedExploitPeerWinningIgnoresExpiredClaimWithoutPrune) {
  // A peer that went silent — died, or drove out of comms — must not keep vetoing
  // an angle forever. There is deliberately NO prune() call here: the probe steps
  // over the expired claim itself, so the receipt-time TTL is the release path
  // however the caller is scheduled.
  Coordination c(true, "atlas");
  c.injectClaimForTest(exploitClaim("rama", 7u, 0.0f, 3.0f, 0.0f, 3.0f,
                                    /*staged=*/true, /*expiry_sec=*/100.0));
  ASSERT_EQ(c.activePeerCount(), 1u);   // unpruned, still stored

  EXPECT_EQ(c.stagedExploitPeerWinning(7u, Eigen::Vector3f(-3.0f, 0.0f, 0.0f),
                                       Eigen::Vector3f(3.0f, 0.0f, 0.0f),
                                       "atlas", at(100.0)),
            nullptr);   // == now
  EXPECT_EQ(c.stagedExploitPeerWinning(7u, Eigen::Vector3f(-3.0f, 0.0f, 0.0f),
                                       Eigen::Vector3f(3.0f, 0.0f, 0.0f),
                                       "atlas", at(105.0)),
            nullptr);   // past
  EXPECT_EQ(c.activePeerCount(), 1u);   // and the query is non-mutating

  // Sanity: one second BEFORE its expiry the same claim does block us, so the
  // release above is the TTL and not a mis-filtered field or bad geometry.
  EXPECT_NE(c.stagedExploitPeerWinning(7u, Eigen::Vector3f(-3.0f, 0.0f, 0.0f),
                                       Eigen::Vector3f(3.0f, 0.0f, 0.0f),
                                       "atlas", at(99.0)),
            nullptr);
}

// ---------------------------------------------------------------------------
// Exploit-claim grace window. The TTL is sized for a 1 Hz heartbeat HEARD at
// 1 Hz, but the receiver is a single-threaded executor whose EXPLOIT_PLAN
// ticks can starve the intent subscription for seconds at a stretch. In a
// 2-robot sim run the driving winner's claim aged out of the parked loser's
// table twice — between plan ticks 7 ms apart — and the loser re-selected the
// winner's vantage both times. Grace keeps exploit claims RETAINED (and the
// vantage contests closed) one extra window past expiry, while everything
// with presence semantics — the dwell barrier, rendezvous counting — stays
// on the raw TTL.
// ---------------------------------------------------------------------------

TEST(Coordination, ExploitClaimGraceRetainsThroughPrune) {
  Coordination c(true, "atlas", 0.0f, /*exploit_claim_grace_sec=*/10.0f);
  c.injectClaimForTest(exploitClaim("rama", 7u, 5.0f, 0.0f, 5.0f, 0.0f,
                                    /*staged=*/false, /*expiry_sec=*/100.0));

  // Inside the grace window prune() keeps the claim and the vantage contest
  // (claimMatching with default liveness = retained) still sees it.
  c.prune(at(105.0));
  ASSERT_EQ(c.activePeerCount(), 1u);
  EXPECT_NE(c.claimMatching(Eigen::Vector3f(5.0f, 0.0f, 0.0f), 0.75f),
            nullptr);

  // At expiry + grace the retention ends: pruned, gone.
  c.prune(at(110.0));
  EXPECT_EQ(c.activePeerCount(), 0u);
  EXPECT_EQ(c.claimMatching(Eigen::Vector3f(5.0f, 0.0f, 0.0f), 0.75f),
            nullptr);
}

TEST(Coordination, GraceDoesNotApplyToExploreClaims) {
  // Grace is an EXPLOIT-claim property. An exploration claim held past its
  // TTL would keep contesting frontier candidates for a peer we may not have
  // heard from in 10+ seconds — exploration keeps single-TTL semantics.
  Coordination c(true, "atlas", 0.0f, /*exploit_claim_grace_sec=*/10.0f);
  auto claim = exploitClaim("rama", 0u, 5.0f, 0.0f, 5.0f, 0.0f,
                            /*staged=*/false, /*expiry_sec=*/100.0);
  claim.exploit = false;
  c.injectClaimForTest(claim);

  c.prune(at(99.0));
  ASSERT_EQ(c.activePeerCount(), 1u);   // still live, still stored
  c.prune(at(101.0));
  EXPECT_EQ(c.activePeerCount(), 0u);   // dropped at expiry, no grace
}

TEST(Coordination, GraceZeroPreservesLegacyPrune) {
  // Default-constructed grace (0) must be bit-for-bit the old behaviour:
  // exploit claims drop AT expiry.
  Coordination c(true, "atlas");
  c.injectClaimForTest(exploitClaim("rama", 7u, 5.0f, 0.0f, 5.0f, 0.0f,
                                    /*staged=*/false, /*expiry_sec=*/100.0));
  c.prune(at(100.0));
  EXPECT_EQ(c.activePeerCount(), 0u);
}

TEST(Coordination, LivePeerCountExcludesGracedClaims) {
  // livePeerCount is the PRESENCE count (rendezvous barriers, the CSV
  // column): a claim past its raw expiry no longer proves the peer is alive,
  // however long the table retains it for the vantage contests.
  Coordination c(true, "atlas", 0.0f, /*exploit_claim_grace_sec=*/10.0f);
  c.injectClaimForTest(exploitClaim("rama", 7u, 5.0f, 0.0f, 5.0f, 0.0f,
                                    /*staged=*/false, /*expiry_sec=*/100.0));

  EXPECT_EQ(c.livePeerCount(at(99.0)), 1u);
  c.prune(at(105.0));                        // graced: retained...
  ASSERT_EQ(c.activePeerCount(), 1u);
  EXPECT_EQ(c.livePeerCount(at(105.0)), 0u); // ...but not PRESENT
}

TEST(Coordination, ClaimMatchingLiveAfterFiltersExpired) {
  // The optional liveness filter is what the exploration MinPos walk passes:
  // it must see live claims only, while the exploit contest sites (nullptr)
  // read the table as retained. No prune() here — the filter itself decides.
  Coordination c(true, "atlas", 0.0f, /*exploit_claim_grace_sec=*/10.0f);
  c.injectClaimForTest(exploitClaim("rama", 7u, 5.0f, 0.0f, 5.0f, 0.0f,
                                    /*staged=*/false, /*expiry_sec=*/100.0));

  const auto before = at(99.0);
  const auto after  = at(101.0);
  EXPECT_NE(c.claimMatching(Eigen::Vector3f(5.0f, 0.0f, 0.0f), 0.75f, &before),
            nullptr);
  EXPECT_EQ(c.claimMatching(Eigen::Vector3f(5.0f, 0.0f, 0.0f), 0.75f, &after),
            nullptr);
  // Default (no filter): found regardless — the retained view.
  EXPECT_NE(c.claimMatching(Eigen::Vector3f(5.0f, 0.0f, 0.0f), 0.75f),
            nullptr);
}

TEST(Coordination, StagedExploitPeerWinningHonoursGrace) {
  // The parked-peer contest is a veto, and a parked peer whose heartbeats sat
  // undelivered is still parked — the staged flag only flips through a
  // message. Within grace the peer keeps winning the angle; one grace window
  // past the TTL a genuinely dead peer stops.
  Coordination c(true, "atlas", 0.0f, /*exploit_claim_grace_sec=*/10.0f);
  c.injectClaimForTest(exploitClaim("rama", 7u, 0.0f, 3.0f, 0.0f, 3.0f,
                                    /*staged=*/true, /*expiry_sec=*/100.0));
  const Eigen::Vector3f cand(-3.0f, 0.0f, 0.0f);
  const Eigen::Vector3f self(30.0f, 0.0f, 0.0f);   // far: peer clearly closer

  EXPECT_NE(c.stagedExploitPeerWinning(7u, cand, self, "atlas", at(99.0)),
            nullptr);   // live
  EXPECT_NE(c.stagedExploitPeerWinning(7u, cand, self, "atlas", at(105.0)),
            nullptr);   // expired but graced: still vetoes
  EXPECT_EQ(c.stagedExploitPeerWinning(7u, cand, self, "atlas", at(110.0)),
            nullptr);   // == expiry + grace: released
}

TEST(Coordination, BarrierIgnoresGracedClaims) {
  // THE regression guard for the barrier: grace lengthens how long a claim
  // can VETO a vantage, and must not lengthen how long a silent peer can
  // HOLD a dwell barrier. firstUnstagedExploitPeer releases on the raw TTL
  // exactly as before — a dead teammate frees the dwelling robot in one TTL,
  // not TTL + grace.
  Coordination c(true, "atlas", 0.0f, /*exploit_claim_grace_sec=*/10.0f);
  c.injectClaimForTest(exploitClaim("rama", 7u, 0.0f, 3.0f, 6.0f, 6.0f,
                                    /*staged=*/false, /*expiry_sec=*/100.0));

  EXPECT_NE(c.firstUnstagedExploitPeer(7u, at(99.0)), nullptr);   // held
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(100.0)), nullptr);  // == expiry
  // Still in the table (graced), yet the barrier is already released.
  EXPECT_EQ(c.firstUnstagedExploitPeer(7u, at(105.0)), nullptr);
  EXPECT_EQ(c.activePeerCount(), 1u);
}
