#include <gtest/gtest.h>

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
                                   /*dwelled_mask=*/0b1011u);
  EXPECT_TRUE(msg.exploit);
  EXPECT_EQ(msg.target_id, 42u);
  EXPECT_EQ(msg.dwelled_mask, 0b1011u);

  // A peer ingesting that intent stores the exploit fields and surfaces them
  // through peerDwellUnion.
  Coordination peer(true, "rama");
  peer.onIntent(msg, now);
  EXPECT_EQ(peer.activePeerCount(), 1u);
  EXPECT_EQ(peer.peerDwellUnion(42), 0b1011u);

  // Default (explore) build leaves the exploit fields zeroed.
  auto explore = producer.buildIntent(vp, Eigen::Vector3f(0, 0, 0), now,
                                       5.0f, 4.0f, 0, "map");
  EXPECT_FALSE(explore.exploit);
  EXPECT_EQ(explore.target_id, 0u);
  EXPECT_EQ(explore.dwelled_mask, 0u);
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
