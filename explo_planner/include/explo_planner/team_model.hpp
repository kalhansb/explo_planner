#pragma once
/// @file team_model.hpp
/// @brief Who this robot is in contact with, who it can reach through someone
///        else, and how stale everything it believes about each peer is.
///        Port of the decision core of M-TARE's multi_robot_exploration_manager.
///        See docs/mtare_evolution_plan.md §3.3.
///
/// The planner already had a notion of "is my peer there?", but it was N=2 and
/// it was one-sided: a peer counted as present if its messages were arriving.
/// Three things go wrong with that as soon as the fleet is bigger than two or
/// the radio is worse than perfect, and this unit exists for those three.
///
///   1. ONE-WAY CONTACT. doc/limitations.md §10 records it as observed, not
///      hypothetical: A hears B while B cannot hear A. On A's side that is
///      indistinguishable from a working link, so A cancels its reconnection,
///      drives on, and shares nothing — the whole point of reconnecting. The
///      fix is mTARE's: the link counts only if the peer's own in_range_mask
///      names us back. A handshake, not a receipt.
///
///   2. RELAYS. At N=3 in an A—B—C chain, A cannot hear C at all, yet C is
///      being told everything A knows, by B, continuously. A reconnection
///      dispatch at C would be a long drive to deliver information C already
///      has. Transitive closure over the direct-contact graph says so.
///
///   3. THE TRAP IN 2. Closure is a statement about COMMS, and it is tempting
///      to read it as a statement about DATA. It is not. A's picture of C is
///      still only as fresh as the last time B relayed it, which may be minutes
///      old, while A's status for C says IN_COMMS. A planner that keys on
///      status where it means to key on freshness will chase a position that
///      does not exist and never notice. So this unit reports the two
///      SEPARATELY and names them differently — `inComms()` for the dispatch
///      decision, `positionAgeSec()` / `lastDirectAgeSec()` for every consumer
///      that needs the peer's actual data — and there is no accessor that
///      quietly conflates them. (`lastKnownAgeSec()` exists too and is NOT the
///      freshness accessor a reader wants here; see the warning on its
///      declaration. There has never been a `lastHeardAgeSec()`, which this
///      paragraph named until 2026-09-17 — a paragraph about not confusing two
///      quantities was sending readers to a third that does not exist.)
///
/// Everything here is measured in MISSION-ELAPSED seconds on the LOCAL clock.
/// Nothing times anything off a peer's header stamp: field robots' clocks have
/// been observed hours apart, and a freshness measure denominated in a remote
/// clock is a freshness measure that can read as negative. A peer's gossip
/// about a third robot arrives as an elapsed interval on the PEER's clock, and
/// is converted on receipt by adding the local age of the message that carried
/// it — an interval plus an interval, with no absolute stamp anywhere.

#include <cstdint>
#include <string>
#include <vector>

#include "explo_planner/fleet_identity.hpp"

namespace explo_planner {

/// Per-peer comms state. mTARE's RELAY_COMMS and CONVOY are deliberately not
/// ported: RELAY_COMMS drives a behaviour (park and act as a repeater) that
/// nothing here implements, and a state nothing acts on is a state that will
/// drift out of agreement with reality unnoticed.
enum class CommsStatus : uint8_t {
  LOST_COMMS = 0,
  IN_COMMS   = 1,
};

const char* commsStatusName(CommsStatus s);

class TeamModel {
public:
  struct Config {
    /// How long a direct contact keeps counting as direct. This is the
    /// existing intent-liveness TTL generalised: it must exceed the publish
    /// period by enough that one dropped message is not a dropout, and be
    /// short enough that a real dropout is noticed before the mission ends.
    double direct_ttl_sec = 5.0;

    /// Transitive closure on/off. Off is exactly the N=2 behaviour that
    /// predates this unit, which is what the phase's equivalence gate needs to
    /// be able to reproduce, and it is also the honest setting for a fleet
    /// whose radios are not actually relaying.
    bool closure_enabled = true;

    /// A peer's gossip about a third robot is only usable while the gossip
    /// itself is fresh. Past this, the third-party entry is treated as never
    /// heard — NOT as "heard a long time ago", because a very stale interval
    /// added to a very stale message is a number with no meaning that would
    /// still compare against thresholds.
    double gossip_max_age_sec = 120.0;
  };

  /// One robot as this robot currently sees it.
  struct Peer {
    bool     known = false;      ///< anything at all has been heard about it
    CommsStatus status = CommsStatus::LOST_COMMS;

    /// True when the link was confirmed in BOTH directions: we received from
    /// the peer inside the TTL and its in_range_mask named us. This is the
    /// only thing that makes a peer a direct contact.
    bool     direct = false;
    /// Received inside the TTL, but the peer's mask did not name us back. Kept
    /// as its own flag rather than folded into `direct` because it is the
    /// signature of the §10 failure and an operator needs to see it, not
    /// deduce it from an absence.
    bool     heard_one_way = false;
    /// True when `status` is IN_COMMS only because someone else is bridging.
    /// The dispatch decision does not care; a log line reading "in comms" for
    /// a robot nobody can hear directly needs the qualifier or it looks wrong.
    bool     via_relay = false;

    /// Mission-elapsed seconds when this robot last had DIRECT contact.
    /// Negative for never.
    double   last_direct_sec = -1.0;
    /// Mission-elapsed seconds of the freshest information we hold about the
    /// peer, first-hand or gossiped. Negative for never. This is the field
    /// every DATA consumer keys on.
    double   last_known_sec = -1.0;
    /// Whether `position` came from the peer itself or through a relay.
    bool     position_first_hand = false;
    double   position_x = 0.0, position_y = 0.0, position_z = 0.0;
    bool     have_position = false;
    /// Mission-elapsed seconds the POSITION dates from. Negative for never.
    ///
    /// Tracked separately from last_known_sec, and the difference is rule 3 of
    /// the file header made concrete rather than a redundancy. Both first-hand
    /// and gossiped updates refresh last_known_sec BEFORE testing whether the
    /// message carried a position at all, so a status-only relay about a robot
    /// nobody has seen in minutes leaves last_known_sec reading "fresh" over a
    /// position that is minutes old. A consumer that steers on the position —
    /// a chase, a separation term — must key on this field.
    double   position_sec = -1.0;

    /// The peer's run is over (TeamWorld/finished). MONOTONIC and STICKY: set
    /// first-hand from the peer's own message, or by relay from a third robot
    /// that heard it, and never cleared by either. The publisher latches the
    /// bit, so false here means "no evidence", not "still exploring".
    ///
    /// It is relayed — unlike `team_incomplete` below — because it is a
    /// monotonic statement a robot makes about ITSELF, so a relayed copy can
    /// neither contradict a first-hand one nor echo back to its originator.
    /// Without the relay, a robot that cannot hear the finished peer keeps
    /// counting it missing forever and holds the whole team at the unbounded
    /// appointment barrier; see TeamWorld.msg/robot_finished.
    bool     finished = false;

    /// The peer's own FIRST-HAND answer to "is the team whole?", as it sent it
    /// (TeamWorld/team_incomplete). NOT its derived armed state — see the
    /// field's own documentation in TeamWorld.msg for why announcing the armed
    /// state instead deadlocks.
    ///
    /// Stale-safe in one direction only, which is the useful one: this is
    /// whatever the peer last said, with no TTL of its own, so a consumer must
    /// pair it with `direct || heard_one_way` to mean "a peer we are receiving
    /// from RIGHT NOW says the team is broken". Both flags are required, not
    /// `direct` alone: one-way contact (we receive from the peer, it cannot
    /// hear us) is exactly the case this exists to catch — the peer's own read
    /// is broken, it is announcing so, and it has no other way to tell us.
    bool     team_incomplete = false;

    /// The peer is in a rendezvous appointment manoeuvre and has not stopped
    /// driving yet (TeamWorld/appointment_inbound). First-hand only, like
    /// team_incomplete, and its reader pairs it with `direct || heard_one_way`
    /// for the same reason: it carries no TTL of its own.
    ///
    /// It clears when the DRIVE ends — arrival, nav budget, or no-progress —
    /// and not when the cell is reached, so a peer whose destination turned out
    /// unreachable stops holding the barrier instead of hanging it. That bound
    /// is also why it needs no finished exemption; see TeamWorld.msg.
    bool     appointment_inbound = false;

    /// The peer reports that some robot IT receives first-hand is still
    /// driving to the agreed cell (TeamWorld/appointment_inbound_seen): the
    /// one-hop companion to the bit above, added with the generation-27
    /// closure door. The sender derives it from raw first-hand
    /// appointment_inbound bits only, never from other robots' copies of this
    /// field, so it cannot echo (see TeamWorld.msg). Same reader contract as
    /// the bit above: pair with `direct || heard_one_way`, no TTL of its own.
    bool     appointment_inbound_seen = false;

    /// The peer's last reported direct-contact mask, as it sent it.
    uint32_t in_range_mask = 0;
  };

  TeamModel() = default;

  /// Bind to a fleet. Returns "" on success or the reason for refusing. An
  /// unconfigured identity is refused rather than accepted-as-empty: every
  /// index here is a fleet id, and a model with no ids would answer every
  /// question with a confident LOST_COMMS.
  std::string configure(const FleetIdentity& fleet, const Config& cfg);

  bool configured() const { return size_ > 0; }
  int  size() const { return size_; }
  int  selfId() const { return self_id_; }
  const Config& config() const { return cfg_; }

  /// A TeamWorld message reduced to what this unit needs. Decoupled from the
  /// generated type so the model and its tests do not depend on ROS.
  struct Observation {
    int      sender_id = -1;
    uint32_t in_range_mask = 0;   ///< the sender's direct contacts, incl. itself
    bool     finished = false;
    /// The sender's own first-hand "the team is not whole" bit. Copied through
    /// verbatim; see Peer::team_incomplete.
    bool     team_incomplete = false;
    /// The sender's own "I am still driving to the agreed cell" bit. Copied
    /// through verbatim; see Peer::appointment_inbound.
    bool     appointment_inbound = false;
    /// The sender's one-hop "a peer I receive first-hand is still driving"
    /// report. Copied through verbatim; see Peer::appointment_inbound_seen.
    bool     appointment_inbound_seen = false;
    bool     have_position = false;
    double   x = 0.0, y = 0.0, z = 0.0;

    /// Gossip, indexed by fleet id and sized to the team (shorter is accepted
    /// and treated as absent past the end; longer is a fleet-definition
    /// mismatch and is refused).
    std::vector<double> last_heard_sec;   ///< on the SENDER's mission clock
    std::vector<double> gx, gy, gz;       ///< last known position per robot
    std::vector<uint8_t> have_gossip_pos;

    /// Relayed `finished`, indexed by fleet id. Merged as a pure OR that never
    /// clears, and — unlike everything else in this block — NOT age-gated: a
    /// finished robot goes quiet, so its last-heard entry ages out of
    /// gossip_max_age_sec exactly when the bit matters. A monotonic fact has no
    /// freshness to check. See Peer::finished and TeamWorld.msg/robot_finished.
    std::vector<uint8_t> finished_gossip;
  };

  /// Fold in one received message. `now_sec` is the LOCAL mission-elapsed time
  /// of receipt. Returns "" or a reason the message was dropped.
  std::string observe(const Observation& obs, double now_sec);

  /// Recompute every peer's status: TTL expiry, handshake, then closure.
  /// Separated from observe() because closure is a property of the whole graph
  /// and must not be recomputed once per arriving message — with three robots
  /// publishing at 1 Hz that would be three different answers per second, and
  /// whichever one a consumer happened to read would be the one that counted.
  void tick(double now_sec);

  const Peer& peer(int id) const { return peers_[static_cast<size_t>(id)]; }

  /// Dispatch decision only. See rule 3 in the file header.
  bool inComms(int id) const;

  /// Seconds since the freshest information about `id`, first-hand or
  /// gossiped, or a negative value for "never heard". THIS is what a consumer
  /// that needs the peer's data must key on.
  double lastKnownAgeSec(int id, double now_sec) const;

  /// Seconds since the POSITION we hold for `id` was measured, or negative
  /// for "we hold no position". This is the accessor the file header has
  /// always named and the one every consumer that STEERS on a peer position
  /// must use; lastKnownAgeSec() answers a different question and can read
  /// fresh over a stale position (see Peer::position_sec).
  double positionAgeSec(int id, double now_sec) const;

  /// Seconds since DIRECT contact with `id`, or negative for never. The
  /// knowledge gate keys on this: what a peer has been told depends on when we
  /// last actually talked, not on whether someone else can currently hear it.
  double lastDirectAgeSec(int id, double now_sec) const;

  /// Mask of peers currently IN_COMMS, self included — the closure result.
  /// This is a local conclusion and must NOT be published: TeamWorld's
  /// in_range_mask is defined as direct contacts, and putting the closure mask
  /// there would make the handshake circular — A claims to hear C because B
  /// said C was reachable, C concludes the same about A from its own copy of
  /// the same relay, and the mutual test that rule 1 exists for passes without
  /// anybody having heard anybody.
  uint32_t inCommsMask() const;

  /// Mask of peers heard DIRECTLY inside the TTL, self included. This is the
  /// mask to publish in TeamWorld.in_range_mask.
  ///
  /// Deliberately NOT the handshake result: this is "I receive from them",
  /// which is the half of the handshake only we can observe. Publishing the
  /// mutual result instead would deadlock a healthy link — neither robot could
  /// name the other until the other had already named it.
  uint32_t directMask() const;

  /// Number of peers (excluding self) currently LOST_COMMS.
  int lostCount() const;

private:
  Config cfg_;
  int    self_id_ = -1;
  int    size_    = 0;
  std::vector<Peer> peers_;

  /// Direct-contact adjacency as most recently reported, row = robot id, bit =
  /// peer id. Row `self_id_` is what we measured; other rows are what each
  /// peer told us about itself, and are used only for closure.
  std::vector<uint32_t> reported_mask_;
  std::vector<double>   reported_at_sec_;
};

}  // namespace explo_planner
