#pragma once
/// @file coordination.hpp
/// @brief Multi-robot intent table + MinPos primitive used by the planner
///        to deconflict candidate viewpoints across teammates.
///
/// Two responsibilities:
///   1. Maintain the latest-per-peer claim table from incoming RobotIntent
///      messages, with a receipt-time TTL (local clock).
///   2. Provide the MinPos allocation primitive (Bautin, Simonin & Charpillet
///      IROS 2012): given a candidate viewpoint, return whether the local
///      robot or a peer is closer to it, with a lexicographic robot-id
///      tiebreak. Neither the paper nor this port is restricted to two
///      robots — claimMatching() walks every live claim and the campaigns run
///      it at N=2, 3 and 4. What IS narrower than the paper is the objective:
///      this is the pairwise "am I closest" test only, not the paper's full
///      frontier-to-robot assignment.
///
/// There is no soft discount, no Mode enum, no per-voxel hook. The helper is
/// always constructed; the only thing that depends on enabled() is whether
/// the planner consults claimMatching() / selfWinsAgainst() in its candidate
/// walk. With enabled() == false, single-robot behaviour is bit-for-bit
/// preserved (the helper is plumbed through doPlan unconditionally).
/// Moved comments: doc/coordination_notes.md

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
    // Exploit fields, as in RobotIntent.msg: exploit marks an exploit hop on
    // tree target_id; dwelled_mask is the producer's cumulative dwelled vantage
    // set there; staged means it stands on goal_pos, never while driving.
    // (notes: coord-claim-exploit-fields)
    bool exploit = false;
    uint32_t target_id = 0;
    uint32_t dwelled_mask = 0;
    bool staged = false;
  };

  /// Largest TTL (s) a peer may claim, whatever it advertises, so every claim
  /// can expire. The TTL means time since we last heard the peer; the heartbeat
  /// is ~1 Hz. (notes: coord-max-claim-ttl-cap)
  static constexpr float kMaxClaimTtlSec = 60.0f;

  /// max_claim_radius_m caps peer claim radii (pass coord_claim_radius_m; <= 0
  /// or non-finite: no cap). exploit_claim_grace_sec keeps EXPLOIT claims past
  /// expiry for vetoes only; presence checks use raw expiry. 0 = none.
  /// (notes: coord-ctor-radius-cap-exploit-grace)
  Coordination(bool enabled, std::string self_id,
               float max_claim_radius_m = 0.0f,
               float exploit_claim_grace_sec = 0.0f);

  /// Stores the latest claim per peer, skipping our own echo. Expiry is
  /// now_local + the clamped ttl_sec on the local clock; never build it from
  /// the peer's header.stamp, as robot clocks are not synchronised.
  /// (notes: coord-intent-local-receipt-expiry)
  void onIntent(const explo_planner_msgs::msg::RobotIntent& msg,
                const rclcpp::Time& now_local);

  /// Drops expired claims; called once per PLAN tick. EXPLOIT claims stay until
  /// expiry + exploit_claim_grace_sec, so lookups that must not see graced
  /// claims check raw expiry themselves. (notes: coord-prune-exploit-grace)
  void prune(const rclcpp::Time& now);

  /// Returns the claim whose disc, at the claim's own radius_m, covers
  /// candidate_xy (XY), closest robot_pos first; else nullptr. match_radius_m
  /// is the fallback for radius_m <= 0. Non-null live_after skips expired
  /// claims. (notes: coord-claim-matching-own-radius)
  const Claim* claimMatching(const Eigen::Vector3f& candidate_xy,
                             float match_radius_m,
                             const rclcpp::Time* live_after = nullptr) const;

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

  /// Returns the first live exploit claim on target_id whose producer has not
  /// declared staged, else nullptr; the dwell barrier holds while non-null.
  /// Checks raw expiry itself, no grace: prune() does not run in EXPLOIT_DWELL.
  /// (notes: coord-dwell-barrier-unstaged-probe)
  const Claim* firstUnstagedExploitPeer(uint32_t target_id,
                                        const rclcpp::Time& now) const;

  /// Returns the first staged exploit claim on target_id that beats us for
  /// candidate_xy under selfWinsAgainst(), else nullptr. Staged peers only: a
  /// driving peer contests via claimMatching() alone. Honours the grace window.
  /// (notes: coord-staged-peer-selection-contest)
  const Claim* stagedExploitPeerWinning(uint32_t target_id,
                                        const Eigen::Vector3f& candidate_xy,
                                        const Eigen::Vector3f& self_pos,
                                        const std::string& self_id,
                                        const rclcpp::Time& now) const;

  /// Builds the intent for goal (stamp from now, frame_id from map_frame); the
  /// caller publishes it. planner_type_id is the 0..3 enum in RobotIntent.msg.
  /// Exploit fields default to an exploration claim; staged: standing on goal.
  /// (notes: coord-build-intent)
  explo_planner_msgs::msg::RobotIntent buildIntent(const CandidateViewpoint& goal,
                                            const Eigen::Vector3f& self_pos,
                                            const rclcpp::Time& now,
                                            float ttl_sec,
                                            float claim_radius_m,
                                            uint8_t planner_type_id,
                                            const std::string& map_frame,
                                            bool exploit = false,
                                            uint32_t target_id = 0,
                                            uint32_t dwelled_mask = 0,
                                            bool staged = false) const;

  /// Whether the planner should consult MinPos at all. False in
  /// single-robot launches; true in multi-robot launches.
  bool enabled() const { return enabled_; }

  /// Number of peer claims currently stored, INCLUDING exploit claims held
  /// past expiry by the grace window. Storage diagnostic only — anything with
  /// presence semantics must use livePeerCount(), or, in the node, the wrapper
  /// named below.
  size_t activePeerCount() const { return claims_.size(); }

  /// Claim-table presence count: claims live at now (raw expiry, no grace). The
  /// node's barrier and coord_active_peers use
  /// ExploPlannerNode::accountedPeerCount instead, which calls this only as a
  /// fallback. (notes: coord-live-peer-count-vs-node)
  size_t livePeerCount(const rclcpp::Time& now) const;

  /// Per-peer liveness on the same raw-expiry semantics as livePeerCount().
  /// The mesh-reconnection planner uses this to tell WHICH teammate the
  /// barrier is waiting on: a recorded last-contact whose producer is not
  /// live is the peer to pursue / meet.
  bool peerLive(const std::string& robot_id, const rclcpp::Time& now) const;

  const std::string& selfId() const { return self_id_; }

  // Test seam: install a peer claim directly without going through the
  // ROS message conversion. Tests use this to drive deterministic
  // scenarios; the planner never calls it.
  void injectClaimForTest(Claim claim);

private:
  /// True iff `c` should still be in the table at `now`: live claims always,
  /// exploit claims additionally through the grace window. This is the
  /// RETENTION rule (prune()'s complement), not a liveness test — liveness is
  /// `c.expiry > now`, which callers with presence semantics check directly.
  bool withinRetention(const Claim& c, const rclcpp::Time& now) const;

  bool enabled_;
  std::string self_id_;
  float max_claim_radius_m_ = 0.0f;  ///< 0 = unbounded (see constructor).
  float exploit_grace_sec_ = 0.0f;   ///< 0 = no grace (see constructor).
  std::vector<Claim> claims_;  ///< Latest-per-peer, pruned by expiry (+grace).
};

}  // namespace explo_planner
