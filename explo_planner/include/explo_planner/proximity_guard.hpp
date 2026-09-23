#pragma once
/// @file proximity_guard.hpp
/// @brief Coordinated proximity-stop arbiter: decides when THIS robot must
///        hold still because a higher-priority teammate is moving nearby.
///
/// The planner consults it every tick while a nav goal is in flight; a "hold"
/// decision parks the robot (State::PROXIMITY_HOLD) until the peer clears off
/// or parks. Three properties carry the design:
///
///   1. Right of way is the lexicographically SMALLER robot_id — the same
///      total order Coordination::selfWinsAgainst uses for its tiebreak. It
///      is computed from ids alone, with no geometry, so both robots of a
///      pair always agree on who yields: the pair can never both stop
///      (standoff) or both proceed (race), even when their pose estimates of
///      each other disagree.
///   2. Hysteresis: a hold STARTS below hold_dist_m but only RELEASES beyond
///      resume_dist_m, so a peer skirting the threshold doesn't chatter the
///      robot between hold and drive.
///   3. A peer that stops moving is released ("parked"): a stationary robot is
///      an ordinary static obstacle in the navigator's local obstacle grid —
///      lidar sees it, so it is mapped like any other solid thing; there is no
///      nav2 costmap in this stack — and holding against one deadlocks, e.g. a
///      teammate waiting at its rendezvous anchor, or one whose planner died
///      mid-run. Motion is
///      measured against an anchor pose (displacement > peer_static_move_m
///      re-arms it), so localisation jitter doesn't count as driving. The
///      release has a floor, parked_keep_dist_m: inside it "the peer parked"
///      is no licence to drive even closer — the hold stands until the peer
///      moves off (or the caller's max-hold escape hatch fires). When that
///      hatch fires, the caller arms armEscape(): for escape_grace_sec the
///      escaped-from peer cannot force a NEW hold, because a peer that is
///      still static inside the disc would otherwise re-hold on the very
///      next tick and the hatch would never actually free the robot. The
///      immunity is per-peer, and cancels early if the peer moves.
///
/// Staleness is asymmetric on purpose: data older than pose_stale_sec cannot
/// START a hold (never brake on a ghost), but an active hold survives up to
/// hold_release_stale_sec without data (the peer that scared us is probably
/// still close; give comms a chance to recover before driving on).
///
/// All timestamps are LOCAL receipt times supplied by the caller, mirroring
/// Coordination: field robot clocks are not synchronised, so peer header
/// stamps are never compared against the local clock. Distances are XY —
/// the whole planner is ground-restricted.
/// Moved comments: doc/explo_planner_code_notes.md

#include <Eigen/Core>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>

namespace explo_planner {

class ProximityGuard {
public:
  struct Config {
    bool  enabled = false;
    float hold_dist_m = 5.0f;    ///< Yield when a moving peer is closer.
    float resume_dist_m = 6.0f;  ///< Release only beyond this (hysteresis).
    float pose_stale_sec = 3.0f; ///< Older data cannot START a hold.
    /// Peer unmoved for this long => "parked", not held against...
    float peer_static_sec = 10.0f;
    /// ...unless it sits INSIDE this floor: a peer parked closer than this
    /// keeps the hold (property 3 above).
    float parked_keep_dist_m = 1.5f;
    /// Displacement from the motion anchor that counts as movement.
    float peer_static_move_m = 0.3f;
    /// An active hold with no data for this long RELEASES (peer presumed
    /// gone / out of comms).
    float hold_release_stale_sec = 10.0f;
    /// Seconds after the max-hold escape hatch fires during which the
    /// escaped-from peer cannot start a new hold; 0 disables. Ends early if
    /// the peer moves. (notes: prox-escape-grace)
    float escape_grace_sec = 30.0f;
  };

  /// When hold, peer_id/dist_m name the nearest offending peer; otherwise the
  /// nearest outranking peer considered, and note says why it did not hold
  /// (e.g. clear, parked, stale, no-peer). (notes: prox-decision-note)
  struct Decision {
    bool hold = false;
    std::string peer_id;
    float dist_m = 0.0f;
    std::string note;
  };

  /// Sanitises the config: non-finite or negative distances/times fall back
  /// to their defaults, and resume_dist_m is raised to hold_dist_m when it
  /// is below it, so the hysteresis band can never be inverted.
  ProximityGuard(Config cfg, std::string self_id);

  /// Record a peer pose stamped with the local receipt time; all sources feed
  /// one per-peer track, latest wins. Self-echoes and non-finite positions are
  /// dropped. (notes: prox-peer-pose-ingest)
  void onPeerPose(const std::string& peer_id, const Eigen::Vector3f& pos,
                  const rclcpp::Time& now_local);

  /// Should the robot hold (holding == false: may a drive continue?) or keep
  /// holding (holding == true)? `holding` selects the hysteresis thresholds:
  /// enter below hold_dist_m on fresh data, stay held inside resume_dist_m
  /// until the peer clears, parks, or goes silent past the release window.
  Decision evaluate(const Eigen::Vector3f& self_pos, const rclcpp::Time& now,
                    bool holding) const;

  /// The caller's max-hold escape hatch fired against this peer: suppress new
  /// holds against it for escape_grace_sec (see Config), so the resumed drive
  /// can actually leave the trigger disc instead of re-holding next tick.
  void armEscape(const std::string& peer_id, const rclcpp::Time& now);

  /// Peers that cannot force a hold until the next call (gen 34, Q65: both
  /// robots are in Meet, driving to their own ring spots). Replaces the whole
  /// set each call. Empty by default, so callers that never set it (gen 33)
  /// are unaffected. An exempt peer still names itself in a no-hold decision,
  /// with note "exempt".
  void setExempt(std::vector<std::string> peer_ids) { exempt_ = std::move(peer_ids); }

  bool enabled() const { return cfg_.enabled; }
  const Config& config() const { return cfg_; }
  size_t trackedPeerCount() const { return peers_.size(); }

private:
  struct PeerTrack {
    std::string id;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    rclcpp::Time last_seen;
    /// Motion anchor: pos the last time the peer had moved > move threshold.
    Eigen::Vector3f move_anchor = Eigen::Vector3f::Zero();
    rclcpp::Time last_moved;
    /// Post-escape immunity (armEscape). Guarded by the flag, never by the
    /// time alone: a default-constructed rclcpp::Time carries a different
    /// clock type than the caller's, and comparing them throws.
    bool escape_active = false;
    rclcpp::Time escape_until;
  };

  Config cfg_;
  std::string self_id_;
  std::vector<PeerTrack> peers_;  ///< Latest-per-peer; team sizes are small.
  std::vector<std::string> exempt_;  ///< setExempt
};

}  // namespace explo_planner
