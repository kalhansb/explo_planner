#pragma once
/// @file coordination.hpp
/// @brief Multi-robot intent table + MinPos primitive used by the planner
///        to deconflict candidate viewpoints across teammates.
///
/// Two responsibilities:
///   1. Maintain the latest-per-peer claim table from incoming RobotIntent
///      messages, with a receipt-time TTL (local clock).
///   2. Provide the MinPos allocation primitive (Bautin, Simonin & Charpillet
///      IROS 2012, restricted to N=2): given a candidate viewpoint, return
///      whether the local robot or a peer is closer to it, with a
///      lexicographic robot-id tiebreak.
///
/// There is no soft discount, no Mode enum, no per-voxel hook. The helper is
/// always constructed; the only thing that depends on enabled() is whether
/// the planner consults claimMatching() / selfWinsAgainst() in its candidate
/// walk. With enabled() == false, single-robot behaviour is bit-for-bit
/// preserved (the helper is plumbed through doPlan unconditionally).

#include "explo_planner/candidate_generator.hpp"

#include <Eigen/Core>
#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <explo_planner_msgs/msg/robot_intent.hpp>

namespace explo_planner {

class Coordination {
public:
  /// One peer's currently-active claim. Stored once per peer robot_id;
  /// later messages from the same robot_id replace the existing entry
  /// (latest-wins, TTL-bounded).
  struct Claim {
    std::string robot_id;          ///< Producer of the claim (peer).
    Eigen::Vector3f goal_pos;      ///< Claimed viewpoint XY in map frame.
    Eigen::Vector3f robot_pos;     ///< Producer's pose at claim time.
    float radius_m = 0.0f;         ///< Disc radius the producer reserved.
    rclcpp::Time expiry;           ///< local receipt time + ttl (local clock).
    uint8_t planner_type = 0;      ///< Diagnostic only.
    // Exploitation coupling (see RobotIntent.msg). exploit=true marks an
    // exploit hop on tree `target_id`; dwelled_mask is the producer's
    // cumulative clear-LoS dwelled vantage-index set on that target.
    bool exploit = false;
    uint32_t target_id = 0;
    uint32_t dwelled_mask = 0;
  };

  /// Largest TTL (seconds) any peer may claim, however large a value it
  /// advertises. A claim's TTL means "how long since we last HEARD this peer",
  /// and the heartbeat is ~1 Hz, so anything past a minute means the peer is
  /// gone. Without a cap an intent carrying +inf (or a garbage float from a
  /// version-skewed publisher) produced a claim that prune() could never expire
  /// — a permanent veto over a disc of the map, with no way to clear it short of
  /// restarting the node.
  static constexpr float kMaxClaimTtlSec = 60.0f;

  /// @param max_claim_radius_m Upper bound applied to every peer-advertised
  ///        claim radius (see onIntent). Pass the local
  ///        `coord_claim_radius_m` — the exploration-scale disc — since that is
  ///        the largest claim this planner itself considers legitimate. <= 0 or
  ///        non-finite disables the bound (test/legacy default).
  Coordination(bool enabled, std::string self_id,
               float max_claim_radius_m = 0.0f);

  /// Process an incoming intent, stamped with the LOCAL receipt time
  /// `now_local` (the caller's node clock). Drops messages whose robot_id
  /// matches self_id_ (echo of our own broadcast). Stores latest claim per
  /// peer, expiring at now_local + msg.ttl_sec — the TTL means "freshness
  /// since we last heard this peer", measured entirely on the local clock.
  /// The producer's header.stamp is deliberately NOT used for expiry: field
  /// robots' clocks are not synchronised (offsets of seconds to hours have
  /// been observed), and an expiry built from the peer's stamp but pruned
  /// against local now() makes claims immortal when the peer's clock is
  /// ahead and stillborn when it is behind by more than the TTL.
  void onIntent(const explo_planner_msgs::msg::RobotIntent& msg,
                const rclcpp::Time& now_local);

  /// Drop expired claims. Called once per PLAN tick from the planner.
  void prune(const rclcpp::Time& now);

  /// MinPos lookup primitive. Returns the active peer claim whose disc
  /// (claim.goal_pos +/- match_radius_m) overlaps `candidate_xy`, or
  /// nullptr if no peer contests this candidate. If multiple peers
  /// contest the same region, returns the peer whose `robot_pos` is
  /// closest to `candidate_xy` — extending naturally to N>2 if we ever
  /// scale up the team.
  ///
  /// `match_radius_m` is set by the caller from `coord_claim_radius_m`,
  /// the disc radius the planner uses for both publish and subscribe.
  /// 2D Euclidean (XY); the simulation is ground-restricted.
  const Claim* claimMatching(const Eigen::Vector3f& candidate_xy,
                             float match_radius_m) const;

  /// MinPos comparator. Returns true iff the local robot should win
  /// `candidate_xy` against `peer`, using strict closer-distance with
  /// a lexicographic robot_id tiebreak when distances are equal. Total
  /// and antisymmetric: exactly one robot yields per pairwise conflict.
  bool selfWinsAgainst(const Eigen::Vector3f& self_pos,
                       const Eigen::Vector3f& candidate_xy,
                       const Claim& peer,
                       const std::string& self_id) const;

  /// OR of dwelled_mask over all active exploit claims on `target_id`.
  /// This is the team's clear-LoS dwelled vantage-index union as currently
  /// visible through unexpired claims; the planner merges it into its local
  /// TargetQueue (persistent), so an expiring claim never revokes credit.
  uint32_t peerDwellUnion(uint32_t target_id) const;

  /// Build the next outgoing intent for `goal`. Caller publishes on the
  /// `coord_intent_topic` publisher. The producer fills `header.stamp`
  /// from `now` and `header.frame_id` from `map_frame`; planner_type_id
  /// is the 0..3 enum from RobotIntent.msg. The trailing exploit fields
  /// default to the exploration claim shape (exploit=false, zeros).
  explo_planner_msgs::msg::RobotIntent buildIntent(const CandidateViewpoint& goal,
                                            const Eigen::Vector3f& self_pos,
                                            const rclcpp::Time& now,
                                            float ttl_sec,
                                            float claim_radius_m,
                                            uint8_t planner_type_id,
                                            const std::string& map_frame,
                                            bool exploit = false,
                                            uint32_t target_id = 0,
                                            uint32_t dwelled_mask = 0) const;

  /// Whether the planner should consult MinPos at all. False in
  /// single-robot launches; true in multi-robot launches.
  bool enabled() const { return enabled_; }

  /// Number of unexpired peer claims currently held. Used as a CSV
  /// diagnostic column (`coord_active_peers`).
  size_t activePeerCount() const { return claims_.size(); }

  const std::string& selfId() const { return self_id_; }

  // Test seam: install a peer claim directly without going through the
  // ROS message conversion. Tests use this to drive deterministic
  // scenarios; the planner never calls it.
  void injectClaimForTest(Claim claim);

private:
  bool enabled_;
  std::string self_id_;
  float max_claim_radius_m_ = 0.0f;  ///< 0 = unbounded (see constructor).
  std::vector<Claim> claims_;  ///< Latest-per-peer, pruned by expiry.
};

}  // namespace explo_planner
