#include "explo_planner/coordination.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace explo_planner {

Coordination::Coordination(bool enabled, std::string self_id)
    : enabled_(enabled), self_id_(std::move(self_id)) {}

void Coordination::onIntent(const explo_planner_msgs::msg::RobotIntent& msg,
                            const rclcpp::Time& now_local) {
  // Drop self-broadcasts. Producers publish their own intent so peers can
  // see it; we read every intent on the topic and need to filter our own.
  if (msg.robot_id == self_id_) return;

  Claim claim;
  claim.robot_id = msg.robot_id;
  claim.goal_pos = Eigen::Vector3f(
      static_cast<float>(msg.goal_pos.x),
      static_cast<float>(msg.goal_pos.y),
      static_cast<float>(msg.goal_pos.z));
  claim.robot_pos = Eigen::Vector3f(
      static_cast<float>(msg.robot_pos.x),
      static_cast<float>(msg.robot_pos.y),
      static_cast<float>(msg.robot_pos.z));
  claim.radius_m = msg.claim_radius_m;
  claim.planner_type = msg.planner_type;
  claim.exploit = msg.exploit;
  claim.target_id = msg.target_id;
  claim.dwelled_mask = msg.dwelled_mask;

  // Expiry = LOCAL receipt time + ttl, so prune(now) compares two timestamps
  // from the SAME clock. Building expiry from the producer's header.stamp
  // breaks on real robots: fleet clocks are not synchronised (offsets of
  // hours have been observed in the field), so a peer clock ahead of ours
  // made its claims immortal and a peer behind by > ttl made them expire on
  // arrival — both silently. The TTL semantic is "how long since we last
  // HEARD this peer", which only needs the local clock.
  claim.expiry = now_local + rclcpp::Duration::from_seconds(msg.ttl_sec);

  // Latest-per-peer replacement. If we already have a claim from this
  // robot_id, overwrite it; otherwise append. Linear scan is fine for
  // team sizes in the single digits.
  for (auto& existing : claims_) {
    if (existing.robot_id == claim.robot_id) {
      existing = claim;
      return;
    }
  }
  claims_.push_back(claim);
}

void Coordination::prune(const rclcpp::Time& now) {
  claims_.erase(
      std::remove_if(
          claims_.begin(), claims_.end(),
          [&now](const Claim& c) { return c.expiry <= now; }),
      claims_.end());
}

const Coordination::Claim* Coordination::claimMatching(
    const Eigen::Vector3f& candidate_xy, float match_radius_m) const {
  if (claims_.empty()) return nullptr;

  const Claim* best = nullptr;
  float best_d2 = std::numeric_limits<float>::infinity();

  for (const auto& c : claims_) {
    // Test against the radius the CLAIMER advertised, not our own. The claim
    // radius is the size of the region that peer is occupying, and it is
    // phase-dependent: an exploration claim is ~8-10 m ("I'm driving to this
    // area"), an exploit vantage claim is ~0.75 m ("I'm holding this one angle
    // around a trunk"). Evaluating every claim at the receiver's own scale
    // meant an exploring robot vetoed an 8 m disc around a peer that was
    // merely dwelling at a tree, and an exploiting robot under-vetoed a peer's
    // exploration goal. radius_m <= 0 means the peer sent nothing usable
    // (older node), so fall back to our own notion of "same goal".
    const float r = (c.radius_m > 0.0f) ? c.radius_m : match_radius_m;
    const float r2 = r * r;

    float dx = c.goal_pos.x() - candidate_xy.x();
    float dy = c.goal_pos.y() - candidate_xy.y();
    float d2 = dx * dx + dy * dy;
    if (d2 > r2) continue;

    // Tie-break overlap by which peer's *robot pose* is closer to the
    // candidate. The closer peer is the one MinPos would award the
    // candidate to under N>2.
    float pdx = c.robot_pos.x() - candidate_xy.x();
    float pdy = c.robot_pos.y() - candidate_xy.y();
    float pd2 = pdx * pdx + pdy * pdy;
    if (pd2 < best_d2) {
      best_d2 = pd2;
      best = &c;
    }
  }
  return best;
}

bool Coordination::selfWinsAgainst(const Eigen::Vector3f& self_pos,
                                   const Eigen::Vector3f& candidate_xy,
                                   const Claim& peer,
                                   const std::string& self_id) const {
  float dxs = self_pos.x() - candidate_xy.x();
  float dys = self_pos.y() - candidate_xy.y();
  float d_self2 = dxs * dxs + dys * dys;

  float dxp = peer.robot_pos.x() - candidate_xy.x();
  float dyp = peer.robot_pos.y() - candidate_xy.y();
  float d_peer2 = dxp * dxp + dyp * dyp;

  if (d_self2 < d_peer2) return true;
  if (d_self2 > d_peer2) return false;
  // Exact tie — lex tiebreak. "atlas" < "rama" so atlas wins on ties.
  return self_id < peer.robot_id;
}

uint32_t Coordination::peerDwellUnion(uint32_t target_id) const {
  uint32_t mask = 0;
  for (const auto& c : claims_) {
    if (c.exploit && c.target_id == target_id) mask |= c.dwelled_mask;
  }
  return mask;
}

explo_planner_msgs::msg::RobotIntent Coordination::buildIntent(
    const CandidateViewpoint& goal,
    const Eigen::Vector3f& self_pos,
    const rclcpp::Time& now,
    float ttl_sec,
    float claim_radius_m,
    uint8_t planner_type_id,
    const std::string& map_frame,
    bool exploit,
    uint32_t target_id,
    uint32_t dwelled_mask) const {
  explo_planner_msgs::msg::RobotIntent msg;
  msg.header.stamp = now;
  msg.header.frame_id = map_frame;
  msg.robot_id = self_id_;

  msg.goal_pos.x = goal.position.x();
  msg.goal_pos.y = goal.position.y();
  msg.goal_pos.z = goal.position.z();
  msg.yaw = goal.yaw;

  msg.robot_pos.x = self_pos.x();
  msg.robot_pos.y = self_pos.y();
  msg.robot_pos.z = self_pos.z();

  msg.claim_radius_m = claim_radius_m;
  msg.ttl_sec = ttl_sec;
  msg.planner_type = planner_type_id;
  msg.exploit = exploit;
  msg.target_id = target_id;
  msg.dwelled_mask = dwelled_mask;
  return msg;
}

void Coordination::injectClaimForTest(Claim claim) {
  for (auto& existing : claims_) {
    if (existing.robot_id == claim.robot_id) {
      existing = std::move(claim);
      return;
    }
  }
  claims_.push_back(std::move(claim));
}

}  // namespace explo_planner
