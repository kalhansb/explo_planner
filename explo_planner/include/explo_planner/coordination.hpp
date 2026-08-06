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
    // cumulative clear-LoS dwelled vantage-index set on that target; staged is
    // the producer's own declaration that it is standing on goal_pos in its
    // dwell state (false while it is still driving anywhere).
    bool exploit = false;
    uint32_t target_id = 0;
    uint32_t dwelled_mask = 0;
    bool staged = false;
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
  /// @param exploit_claim_grace_sec Extra retention applied to EXPLOIT claims
  ///        only: prune() keeps them for `expiry + grace` instead of dropping
  ///        them at `expiry`. The TTL is sized for a healthy 1 Hz heartbeat,
  ///        but the RECEIVER is a single-threaded executor whose EXPLOIT_PLAN
  ///        retry ticks can monopolise it for several seconds at a stretch
  ///        (fused-map refresh + whole-grid floods), so a peer's claims sit
  ///        undelivered while its engagement is as live as ever. In a 2-robot
  ///        sim run that starved window aged the driving winner's claim out of
  ///        the parked loser's table twice — provably between two plan ticks
  ///        7 ms apart — and each time the loser re-selected the very vantage
  ///        the winner was mid-drive to, rolled toward it, and had to be
  ///        yielded back off it. Grace keeps a recently-heard exploit
  ///        engagement contestable across such receive-side gaps. It is a
  ///        VETO-side lenience only: everything that needs "is this peer
  ///        actually alive right now" semantics — the dwell barrier's release,
  ///        rendezvous presence counting — filters on the raw expiry and is
  ///        unaffected. 0 (default) preserves legacy single-TTL behaviour.
  Coordination(bool enabled, std::string self_id,
               float max_claim_radius_m = 0.0f,
               float exploit_claim_grace_sec = 0.0f);

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
  /// EXPLOIT claims are retained for `expiry + exploit_claim_grace_sec`
  /// rather than dropped at `expiry` (see the constructor doc) — every
  /// lookup that must not see a graced claim filters on the raw expiry
  /// itself instead of relying on this.
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
  ///
  /// `live_after`, when non-null, skips claims whose expiry is at or before
  /// that instant. Exploit claims outlive their expiry by the grace window
  /// (see prune()), which is exactly what the EXPLOIT contest sites want —
  /// a vantage under recent pursuit stays vetoed across a receive-side gap —
  /// but the EXPLORATION MinPos walk wants live claims only: a graced claim
  /// there would keep contesting frontier candidates for a peer we may not
  /// have heard from in 10+ seconds. Exploration passes its plan-tick time;
  /// exploit sites pass nullptr.
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

  /// Rendezvous-barrier probe. Returns the first active exploit claim on
  /// `target_id` whose producer has not declared itself staged (`staged ==
  /// false`, i.e. it is still driving somewhere), or nullptr once every such
  /// peer is standing on its vantage (and when no peer claims this target at
  /// all). A robot that has arrived at its own vantage holds its dwell for as
  /// long as this keeps returning non-null, so the returned claim is "the peer
  /// we are waiting for" and exists for the caller to log. The ~1 Hz intent
  /// heartbeat is what makes the barrier progress: each republish carries the
  /// producer's current staging, so an inbound peer's claim goes staged the
  /// moment that peer enters its own dwell.
  ///
  /// Staging is PRODUCER-DECLARED (set at EXPLOIT_DWELL entry, after nav
  /// arrival and the post-arrival rotation settle) and this probe consults
  /// nothing else — no position, no tolerance. It used to infer staging from
  /// geometry, `robot_pos` within a tolerance of the claim's own `goal_pos`,
  /// which cannot separate three situations that look identical in XY: holding
  /// the vantage; arrived but still rotating in place, which read as staged
  /// ~5 s before the robot settled, so peers began dwelling early and the
  /// team's simultaneous-capture overlap collapsed to ~3 s of an 8 s dwell; and
  /// parked at an APPROACH waypoint — an exploit hop whose goal IS the waypoint
  /// — which read as staged while nowhere near a vantage. Only the producer
  /// knows which of its exploit hops is a vantage it is actually holding.
  ///
  /// Unlike claimMatching() / peerDwellUnion(), this lookup filters expiry
  /// ITSELF rather than relying on prune(). Its caller sits in EXPLOIT_DWELL,
  /// where the PLAN tick — and therefore prune() — does not run, so nothing
  /// clears the table while we wait: the stale claim of a peer that died on
  /// its way in would stay unstaged and hold the barrier forever, deadlocking
  /// the robot that did arrive. Skipping `c.expiry <= now` here makes the
  /// receipt-time TTL (~5 s, a heartbeat plus margin) the barrier's release
  /// path — a dead peer stops holding the team one TTL after we last heard it.
  /// The exploit-claim GRACE window (see the constructor) is deliberately NOT
  /// applied here: grace lengthens how long a peer's claim can VETO a vantage,
  /// and stretching the barrier's hold on a silent peer by the same window
  /// would triple how long a dead teammate pins a robot mid-dwell.
  ///
  /// No enabled_ check, as with claimMatching(): the caller gates on enabled().
  const Claim* firstUnstagedExploitPeer(uint32_t target_id,
                                        const rclcpp::Time& now) const;

  /// Selection-time contest against a PARKED peer. Returns the first active
  /// claim that is (a) an exploit claim on `target_id`, (b) staged — its
  /// producer is standing on a vantage of this same ring — and (c) beats us for
  /// `candidate_xy` under selfWinsAgainst()'s total order (strictly closer
  /// `robot_pos`, lexicographic robot_id on an exact tie). nullptr means no such
  /// peer exists and the candidate is ours to take; a non-null result is "do not
  /// select this vantage, that peer is about to", and exists for the caller to
  /// log.
  ///
  /// This closes the blind window between two robots' re-plans. When the team's
  /// synchronised dwells end, both robots leave EXPLOIT_DWELL and re-plan within
  /// milliseconds of each other, both see the same single remaining un-dwelled
  /// vantage, and both select it — neither has yet received the other's claim on
  /// it, because the intent heartbeat is only ~1 Hz. The later claim does make
  /// one of them yield, but by then both have driven at the same angle; in a
  /// 2-robot sim run they physically collided. Deciding the same contest at
  /// SELECTION time, from claims we already hold, means the yield happens before
  /// either robot moves.
  ///
  /// Only STAGED claims are consulted, and that restriction is the whole design.
  /// A staged peer is parked beside this ring and about to re-plan from a dead
  /// standstill, so if it is closer to the candidate angle it will win the race
  /// for it — conceding up front costs nothing. A DRIVING peer is the opposite
  /// case: it contests only through its claim disc (claimMatching(), the
  /// existing rules), because a position contest against a mover would let one
  /// peer mid-drive freeze an approaching robot out of EVERY vantage on the ring
  /// — the robot arriving from far away is farther from all of them — which
  /// destroys the parallelism exploitation exists for. Its wire pose is a
  /// heartbeat stale anyway, so the comparison would not even be meaningful.
  ///
  /// Both robots reach COMPLEMENTARY conclusions, which is what makes this safe
  /// to run independently on each: a staged producer is by construction
  /// stationary (the flag is set at EXPLOIT_DWELL entry and re-broadcast while
  /// dwelling and while re-planning in place; a driving robot always publishes
  /// staged=false), so its broadcast `robot_pos` still agrees with its true
  /// position. Both sides therefore evaluate the same total order on the same
  /// positions, and since that order is antisymmetric, exactly one of them
  /// admits the vantage. Nothing is negotiated and no extra message is needed.
  ///
  /// Filters expiry itself, as firstUnstagedExploitPeer() does — see its note;
  /// here prune() does run each tick (the caller is in EXPLOIT_PLAN), but the
  /// self-filter keeps the probe correct from any call site. Unlike the
  /// barrier probe this one honours the exploit-claim grace window: it is a
  /// veto (the contested angle stays un-selected), and a parked peer whose
  /// heartbeats sat undelivered for a few seconds is still parked — the flag
  /// only ever flips through a message (a staged producer that starts driving
  /// publishes staged=false in the same tick it selects), so a graced staged
  /// claim is stale in age, not in meaning. A peer that genuinely died on its
  /// vantage stops winning candidates one grace window after the TTL.
  const Claim* stagedExploitPeerWinning(uint32_t target_id,
                                        const Eigen::Vector3f& candidate_xy,
                                        const Eigen::Vector3f& self_pos,
                                        const std::string& self_id,
                                        const rclcpp::Time& now) const;

  /// Build the next outgoing intent for `goal`. Caller publishes on the
  /// `coord_intent_topic` publisher. The producer fills `header.stamp`
  /// from `now` and `header.frame_id` from `map_frame`; planner_type_id
  /// is the 0..3 enum from RobotIntent.msg. The trailing exploit fields
  /// default to the exploration claim shape (exploit=false, zeros, unstaged).
  /// `staged` is the producer's declaration that it is standing on `goal` in
  /// its dwell state; only the EXPLOIT_DWELL publish passes true.
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
  /// presence semantics must use livePeerCount().
  size_t activePeerCount() const { return claims_.size(); }

  /// Number of peer claims that are LIVE at `now` (raw expiry, no grace).
  /// This is the presence count: the rendezvous barrier releases on "every
  /// teammate has been HEARD within one TTL", and the CSV `coord_active_peers`
  /// column documents the same thing. With grace retention in the table,
  /// claims_.size() stopped meaning that — a peer 10 s silent would have kept
  /// counting as present at the rendezvous anchor for the whole grace window,
  /// releasing the return barrier on a teammate that may be face-down in a
  /// ditch.
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
