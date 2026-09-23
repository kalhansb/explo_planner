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
/// Moved comments: doc/team_model_notes.md

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

    /// Gossip about a third robot is usable only while the gossip itself is
    /// younger than this, seconds; past it the entry is treated as never heard,
    /// not as heard long ago. (notes: team-gossip-max-age)
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

    // --- R1 instrumentation (generation 33) ---------------------------------
    //
    // The four fields below only time the detector; they do not change it. All
    // are stamped from the packet clock (reported_at_sec_), never the tick
    // clock, so scheduling jitter is not reported as link behaviour.
    // (notes: team-link-timing-packet-clock)

    /// Packet stamp opening the current period without a completed handshake:
    /// first packet of a receiving run, or the packet that broke the last
    /// handshake. Negative while not receiving. An instant handshake reads 0.
    /// (notes: team-one-way-since)
    double   one_way_since_sec = -1.0;
    /// Packet stamp of the message that completed the CURRENT handshake.
    /// Negative while not direct. Distinct from `last_direct_sec`, which is the
    /// LAST mutual packet: this one is the first, and a hold's duration is the
    /// difference between them plus whatever ends it.
    double   direct_since_sec = -1.0;

    /// One-shot, like held_sec: set only on the tick carrying the transition,
    /// else -1, so count non-negative readings. Both transitions need a packet
    /// from this peer. Seconds from first contact packet to the handshake
    /// packet. (notes: team-acquire-one-shot)
    double   acquire_sec = -1.0;
    /// Seconds a completed handshake survived, packet to packet; set only on a
    /// break while still receiving (the peer is audible but stopped naming us).
    /// A link that ends because packets stopped is not recorded here.
    /// (notes: team-held-sec)
    double   held_sec = -1.0;
    /// Mission-elapsed seconds of the freshest information we hold about the
    /// peer, first-hand or gossiped. Negative for never. This is the field
    /// every DATA consumer keys on.
    double   last_known_sec = -1.0;
    /// Whether `position` came from the peer itself or through a relay.
    bool     position_first_hand = false;
    double   position_x = 0.0, position_y = 0.0, position_z = 0.0;
    bool     have_position = false;
    /// Mission-elapsed seconds the POSITION dates from; negative for never.
    /// Status-only updates refresh last_known_sec but not this, so a consumer
    /// that steers on the position (chase, separation term) must key on this
    /// field. (notes: team-position-sec)
    double   position_sec = -1.0;

    /// The peer's run is over (TeamWorld/finished). Set first-hand or by relay;
    /// relay never clears it. False means no evidence, not still exploring.
    /// Relayed so an unheard finished peer cannot hold the appointment barrier
    /// forever. (notes: team-peer-finished)
    bool     finished = false;

    /// The peer's TeamWorld/mode: EXPLORING(0) < HOMING(1) < DONE(2); 0 means
    /// no evidence. First-hand is authoritative and may lower it; relay merges
    /// by MAX. Read it for will this peer participate, finished for is its run
    /// over. (notes: team-peer-mode)
    uint8_t  mode = 0;

    /// The peer's own first-hand team_incomplete bit, not its armed state. No
    /// TTL of its own: pair it with direct or heard_one_way (not direct alone),
    /// since one-way contact is the case it exists to catch.
    /// (notes: team-peer-team-incomplete)
    bool     team_incomplete = false;

    /// The peer is in an appointment manoeuvre and still driving
    /// (TeamWorld/appointment_inbound). First-hand only, no TTL: pair with
    /// direct or heard_one_way. Clears when the drive ends, not when the cell
    /// is reached. (notes: team-peer-appointment-inbound)
    bool     appointment_inbound = false;

    /// The peer reports a robot it hears first-hand is still driving to the
    /// agreed cell. Derived from raw first-hand bits only, so it cannot echo.
    /// Pair with direct or heard_one_way; no TTL of its own.
    /// (notes: team-peer-inbound-seen)
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
    /// The sender's own mode level; see Peer::mode. Copied through verbatim.
    uint8_t  mode = 0;
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

    /// Relayed finished, by fleet id: OR-merged, never cleared, and NOT
    /// age-gated, since a finished robot goes quiet and its gossip ages out
    /// exactly when the bit matters. (notes: team-finished-gossip)
    std::vector<uint8_t> finished_gossip;

    /// Relayed `mode`, indexed by fleet id. Merged by MAX that never lowers,
    /// and NOT age-gated, for finished_gossip's reasons exactly — a robot that
    /// is homing or done ages out of gossip_max_age_sec precisely when its
    /// level matters. See Peer::mode and TeamWorld.msg/robot_mode.
    std::vector<uint8_t> mode_gossip;
  };

  /// Fold in one received message. `now_sec` is the LOCAL mission-elapsed time
  /// of receipt. Returns "" or a reason the message was dropped.
  std::string observe(const Observation& obs, double now_sec);

  /// Recompute every peer's status: TTL expiry, handshake, then closure. Kept
  /// out of observe(): closure is a whole-graph property and must not be
  /// recomputed per arriving message. (notes: team-tick-whole-graph)
  void tick(double now_sec);

  const Peer& peer(int id) const { return peers_[static_cast<size_t>(id)]; }

  /// Dispatch decision only. See rule 3 in the file header.
  bool inComms(int id) const;

  /// Seconds since the freshest information about `id`, first-hand or
  /// gossiped, or a negative value for "never heard". THIS is what a consumer
  /// that needs the peer's data must key on.
  double lastKnownAgeSec(int id, double now_sec) const;

  /// Seconds since the POSITION held for id was measured, or negative if none.
  /// Every consumer that steers on a peer position must use this;
  /// lastKnownAgeSec() can read fresh over a stale position.
  /// (notes: team-position-age-accessor)
  double positionAgeSec(int id, double now_sec) const;

  /// Seconds since DIRECT contact with `id`, or negative for never. The
  /// knowledge gate keys on this: what a peer has been told depends on when we
  /// last actually talked, not on whether someone else can currently hear it.
  double lastDirectAgeSec(int id, double now_sec) const;

  /// Mask of peers currently IN_COMMS, self included (the closure result).
  /// Local only: publishing it as TeamWorld in_range_mask, which is direct
  /// contacts, would make the rule-1 handshake circular.
  /// (notes: team-in-comms-mask-local)
  uint32_t inCommsMask() const;

  /// Mask of peers heard directly inside the TTL, self included; this is what
  /// TeamWorld in_range_mask carries. Not the handshake result: publishing the
  /// mutual result would deadlock a healthy link.
  /// (notes: team-direct-mask-published)
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
