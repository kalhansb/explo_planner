// Moved comments: doc/explo_planner_code_notes.md
#include "explo_planner/coordination.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace explo_planner {

Coordination::Coordination(bool enabled, std::string self_id,
                           float max_claim_radius_m,
                           float exploit_claim_grace_sec)
    : enabled_(enabled), self_id_(std::move(self_id)),
      max_claim_radius_m_(max_claim_radius_m),
      exploit_grace_sec_(
          std::isfinite(exploit_claim_grace_sec) && exploit_claim_grace_sec > 0.0f
              ? exploit_claim_grace_sec
              : 0.0f) {}

bool Coordination::withinRetention(const Claim& c,
                                   const rclcpp::Time& now) const {
  if (c.expiry > now) return true;
  if (!c.exploit || exploit_grace_sec_ <= 0.0f) return false;
  return c.expiry + rclcpp::Duration::from_seconds(exploit_grace_sec_) > now;
}

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
  // Bound the peer-advertised radius: claimMatching() tests each claim at the
  // claimer's radius, so +inf would veto every candidate. Non-finite or
  // non-positive becomes 0 (use ours); larger clamps to max_claim_radius_m_.
  // (notes: coord-bound-claim-radius)
  claim.radius_m = std::isfinite(msg.claim_radius_m) && msg.claim_radius_m > 0.0f
      ? msg.claim_radius_m
      : 0.0f;
  if (std::isfinite(max_claim_radius_m_) && max_claim_radius_m_ > 0.0f)
    claim.radius_m = std::min(claim.radius_m, max_claim_radius_m_);
  claim.planner_type = msg.planner_type;
  claim.exploit = msg.exploit;
  claim.target_id = msg.target_id;
  claim.dwelled_mask = msg.dwelled_mask;
  claim.staged = msg.staged;

  // Expiry is LOCAL receipt time + ttl, never the producer's header stamp:
  // fleet clocks are not synchronised. The peer's ttl is clamped to [0,
  // kMaxClaimTtlSec] and non-finite becomes 0, so no claim is immortal.
  // (notes: coord-claim-expiry-local-clock)
  const float ttl = std::isfinite(msg.ttl_sec)
      ? std::clamp(msg.ttl_sec, 0.0f, kMaxClaimTtlSec)
      : 0.0f;
  claim.expiry = now_local + rclcpp::Duration::from_seconds(ttl);

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
  // Exploit claims are retained one grace window past expiry (withinRetention)
  // so the vantage contests stay closed across receive-side delivery gaps —
  // see the constructor doc for the incident. Non-exploit claims drop at
  // expiry exactly as before.
  claims_.erase(
      std::remove_if(
          claims_.begin(), claims_.end(),
          [this, &now](const Claim& c) { return !withinRetention(c, now); }),
      claims_.end());
}

size_t Coordination::livePeerCount(const rclcpp::Time& now) const {
  size_t n = 0;
  for (const auto& c : claims_)
    if (c.expiry > now) ++n;
  return n;
}

bool Coordination::peerLive(const std::string& robot_id,
                            const rclcpp::Time& now) const {
  for (const auto& c : claims_)
    if (c.robot_id == robot_id && c.expiry > now) return true;
  return false;
}

const Coordination::Claim* Coordination::claimMatching(
    const Eigen::Vector3f& candidate_xy, float match_radius_m,
    const rclcpp::Time* live_after) const {
  if (claims_.empty()) return nullptr;

  const Claim* best = nullptr;
  float best_d2 = std::numeric_limits<float>::infinity();

  for (const auto& c : claims_) {
    // Liveness filter for callers that must not see graced exploit claims
    // (the exploration MinPos walk — see the header). Exploit contest sites
    // pass nullptr and read the table as retained.
    if (live_after && c.expiry <= *live_after) continue;
    // Test against the radius the claimer advertised, not our own: claim discs
    // are phase-dependent (exploration goal versus one vantage angle). radius_m
    // <= 0 means the peer sent nothing usable; use match_radius_m.
    // (notes: coord-claimer-radius)
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

const Coordination::Claim* Coordination::firstUnstagedExploitPeer(
    uint32_t target_id, const rclcpp::Time& now) const {
  for (const auto& c : claims_) {
    // Expiry is tested here, not left to prune(): the caller sits in
    // EXPLOIT_DWELL, which does not re-plan, so prune() never runs. The
    // receipt-time TTL releases a silent peer after one TTL.
    // (notes: coord-unstaged-expiry-here)
    if (c.expiry <= now) continue;
    if (!c.exploit || c.target_id != target_id) continue;

    // The producer sets staged on entering EXPLOIT_DWELL (after arrival and
    // rotation settle) and clears it while driving; every heartbeat carries it.
    // Do not test geometry: only the producer knows which hop holds a vantage.
    // (notes: coord-staged-producer-flag)
    if (!c.staged) return &c;
  }
  return nullptr;
}

const Coordination::Claim* Coordination::stagedExploitPeerWinning(
    uint32_t target_id, const Eigen::Vector3f& candidate_xy,
    const Eigen::Vector3f& self_pos, const std::string& self_id,
    const rclcpp::Time& now) const {
  for (const auto& c : claims_) {
    // Filtered on the graced retention bound, not raw expiry: this probe is a
    // veto, and a parked peer whose heartbeats went undelivered is still
    // parked. Redundant after a plan-tick prune(); kept so any call site is
    // correct. (notes: coord-staged-probe-retention)
    if (!withinRetention(c, now)) continue;
    if (!c.exploit || c.target_id != target_id) continue;

    // Staged claims only: a parked peer closer to this angle wins it, so
    // concede before either moves. A driving peer must never block a candidate
    // this way; it contests only through its claim disc.
    // (notes: coord-staged-only-contest)
    if (!c.staged) continue;

    // Same total order the yield path uses, so both robots compute complementary
    // answers: strictly closer robot_pos wins, exact ties broken lexicographically
    // by robot_id. Losing to this peer is a veto on the candidate, and we return
    // the claim rather than a bool so the caller can name the peer in its log.
    if (!selfWinsAgainst(self_pos, candidate_xy, c, self_id)) return &c;
  }
  return nullptr;
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
    uint32_t dwelled_mask,
    bool staged) const {
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
  msg.staged = staged;
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
