#include "explo_planner/team_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace explo_planner {

const char* commsStatusName(CommsStatus s) {
  switch (s) {
    case CommsStatus::IN_COMMS:   return "in_comms";
    case CommsStatus::LOST_COMMS: return "lost_comms";
  }
  return "unknown";
}

std::string TeamModel::configure(const FleetIdentity& fleet,
                                 const Config& cfg) {
  size_    = 0;
  self_id_ = -1;
  peers_.clear();
  reported_mask_.clear();
  reported_at_sec_.clear();

  char buf[160];
  // An unconfigured identity is refused rather than accepted as an empty
  // fleet. Every index here is a fleet id; with none, this model would answer
  // "LOST_COMMS" for every question, confidently and forever, and the caller
  // would have no way to tell that from a genuinely silent radio.
  if (!fleet.configured)
    return "fleet identity is not configured (team_robot_names)";
  if (fleet.size() < 1 || fleet.size() > kMaxTeamSize) {
    std::snprintf(buf, sizeof(buf), "fleet size %d is outside [1, %d]",
                  fleet.size(), kMaxTeamSize);
    return buf;
  }
  if (fleet.self_id < 0 || fleet.self_id >= fleet.size())
    return "fleet identity does not name this robot";
  if (!(cfg.direct_ttl_sec > 0.0))
    return "direct_ttl_sec must be positive";
  if (!(cfg.gossip_max_age_sec > 0.0))
    return "gossip_max_age_sec must be positive";

  size_    = fleet.size();
  self_id_ = fleet.self_id;
  cfg_     = cfg;
  peers_.assign(static_cast<size_t>(size_), Peer{});
  reported_mask_.assign(static_cast<size_t>(size_), 0u);
  reported_at_sec_.assign(static_cast<size_t>(size_), -1.0);

  // Self is always known, always in comms, always current. Saying so here
  // means no consumer needs a special case for "is this me?", and a loop over
  // the fleet that forgot one would show up as this robot reporting itself
  // lost rather than as a subtly wrong count.
  Peer& me = peers_[static_cast<size_t>(self_id_)];
  me.known  = true;
  me.status = CommsStatus::IN_COMMS;
  me.direct = true;
  return std::string();
}

std::string TeamModel::observe(const Observation& obs, double now_sec) {
  char buf[192];
  if (!configured()) return "team model is not configured";
  if (obs.sender_id < 0 || obs.sender_id >= size_) {
    std::snprintf(buf, sizeof(buf), "sender robot id %d is outside [0, %d)",
                  obs.sender_id, size_);
    return buf;
  }
  if (obs.sender_id == self_id_) return "sender is this robot";
  if (!std::isfinite(now_sec) || now_sec < 0.0)
    return "receipt time must be a finite non-negative mission elapsed time";

  // A gossip array longer than the fleet means the sender is running a
  // different team_robot_names than we are, so its ids address different
  // robots than ours do. The team_hash check upstream should have caught it;
  // this is the backstop, and it refuses rather than truncating, because
  // truncating would apply the sender's robot 0 to our robot 0.
  const size_t n = static_cast<size_t>(size_);
  if (obs.last_heard_sec.size() > n || obs.gx.size() > n ||
      obs.gy.size() > n || obs.gz.size() > n ||
      obs.have_gossip_pos.size() > n || obs.finished_gossip.size() > n) {
    std::snprintf(buf, sizeof(buf),
                  "gossip arrays are longer than the fleet (%d): the sender "
                  "is running a different team definition", size_);
    return buf;
  }

  const int s = obs.sender_id;
  Peer& sp = peers_[static_cast<size_t>(s)];
  sp.known          = true;
  sp.in_range_mask  = obs.in_range_mask;
  // FIRST-HAND IS AUTHORITATIVE and is a plain assignment, so it can also
  // CLEAR. The relay below can only ever set the bit, so this is the one site
  // that can undo it -- which matters for the single case where the bit is not
  // monotonic in reality rather than on the wire: a node that restarts mid-run
  // (its mission clock returns to zero, the case the gossip block below also
  // calls out) genuinely un-finishes, and the robot itself is the only witness
  // worth believing. Peers that learned the old bit by relay keep it until they
  // hear the restarted robot first-hand; they are then receiving from it
  // directly, which is what peerAccounted keys on anyway.
  sp.finished       = obs.finished;
  // team_incomplete IS FIRST-HAND ONLY, and it does not follow `finished` above
  // into the relay. The two are not the same kind of statement. This one is a
  // NON-MONOTONIC claim a robot makes ABOUT THE TEAM, and relaying it is the
  // deadlock TeamWorld.msg warns about arriving by a different route -- C would
  // re-announce A's break as though it were C's own read and A would then see
  // its own bit come back, with nothing able to clear it. `finished` is a
  // MONOTONIC claim a robot makes ABOUT ITSELF: it cannot contradict a
  // first-hand copy, cannot arrive early, and has no path back to its
  // originator, which is exactly why it is safe to relay and this is not.
  sp.team_incomplete = obs.team_incomplete;
  // First-hand only for the same reason as team_incomplete directly above: it is
  // non-monotonic, so a relayed copy could hold an appointment barrier off an
  // echo of itself.
  sp.appointment_inbound = obs.appointment_inbound;
  // Its one-hop report lands the same way: it is the sender's own statement
  // about what the sender currently receives, stored under the sender so the
  // reader believes it exactly while the sender itself is heard. The sender
  // derives it from raw first-hand bits only (see TeamWorld.msg), so storing
  // it here cannot start the echo the two comments above forbid.
  sp.appointment_inbound_seen = obs.appointment_inbound_seen;
  sp.last_known_sec = now_sec;
  if (obs.have_position) {
    sp.have_position       = true;
    sp.position_first_hand = true;
    sp.position_sec        = now_sec;
    sp.position_x = obs.x;
    sp.position_y = obs.y;
    sp.position_z = obs.z;
  }
  reported_mask_[static_cast<size_t>(s)]   = obs.in_range_mask;
  reported_at_sec_[static_cast<size_t>(s)] = now_sec;

  // --- third-party gossip ---------------------------------------------------
  //
  // The sender's entry for ITSELF is its publish time on its own mission
  // clock. Everything else in the array is an earlier reading on that same
  // clock, so (sender_now - entry) is an AGE — an interval, which transfers
  // between clocks unchanged. Without the sender's own entry there is no
  // reference point and the whole array is uninterpretable; it is dropped
  // rather than guessed at, because guessing (say, assuming the sender
  // published at its latest entry) would systematically under-age gossip and
  // make stale positions look current.
  const size_t nh = obs.last_heard_sec.size();
  if (nh > static_cast<size_t>(s)) {
    const double sender_now = obs.last_heard_sec[static_cast<size_t>(s)];
    if (std::isfinite(sender_now) && sender_now >= 0.0) {
      for (size_t k = 0; k < nh; ++k) {
        const int r = static_cast<int>(k);
        if (r == self_id_ || r == s) continue;   // we know ourselves; s is above
        const double t = obs.last_heard_sec[k];
        if (!std::isfinite(t) || t < 0.0) continue;   // sender never heard r
        const double age = sender_now - t;
        if (!(age >= 0.0)) continue;             // sender's clock ran backwards
        if (age > cfg_.gossip_max_age_sec) continue;
        // Local mission-elapsed time this information dates from.
        //
        // Stated bound: this treats transit as instantaneous, so gossip is
        // under-aged by however long the message took to arrive. Sub-second in
        // normal operation, and gossip_max_age_sec is set with slack for it.
        // The pathological case is a message DELAYED or DUPLICATED by minutes,
        // which would look that much fresher than it is. It is not guarded
        // against, deliberately: the obvious guard — refuse a message whose
        // sender entry is not strictly greater than the last we accepted from
        // that sender — deadlocks that peer permanently the first time its
        // node restarts and its mission clock returns to zero, which is a real
        // event and a far worse failure than an over-fresh duplicate.
        const double at = now_sec - age;
        Peer& p = peers_[static_cast<size_t>(r)];
        p.known = true;
        // Strictly fresher only. A relay that keeps re-sending the same old
        // reading must not keep refreshing it, and first-hand contact must
        // never be overwritten by a relay's older copy of the same event.
        if (!(at > p.last_known_sec)) continue;
        p.last_known_sec = at;
        if (k < obs.have_gossip_pos.size() && obs.have_gossip_pos[k] &&
            k < obs.gx.size() && k < obs.gy.size() && k < obs.gz.size()) {
          p.have_position       = true;
          p.position_first_hand = false;
          // `at`, not now_sec: this position was measured when the RELAY heard
          // it, and the whole point of the interval arithmetic above is to
          // recover that instant. Stamping it with the receipt time would make
          // a two-minute-old relayed pose look one tick old — which is the
          // failure this field exists to prevent, reintroduced at the one site
          // where it actually happens.
          p.position_sec        = at;
          p.position_x = obs.gx[k];
          p.position_y = obs.gy[k];
          p.position_z = obs.gz[k];
        }
      }
    }
  }

  // --- relayed `finished` ---------------------------------------------------
  //
  // A SEPARATE LOOP, AND DELIBERATELY NOT INSIDE THE ONE ABOVE. Every guard
  // that block applies would suppress this bit exactly when it is needed. It
  // skips a peer the sender never heard, skips one older than
  // gossip_max_age_sec, and skips one whose reading is not strictly fresher
  // than what we hold -- all of which are freshness rules, and a finished robot
  // STOPS MOVING AND GOES QUIET, so its last-heard entry is the first thing to
  // age out. Gating a monotonic fact on freshness would switch the relay off at
  // the moment it starts to matter. There is no freshness to check: the bit
  // only ever goes one way.
  //
  // PURE OR, NEVER CLEARS. A relayed false is "the sender has no evidence", not
  // "that robot is still exploring" -- the sender may simply not have heard it
  // either -- so false carries no information and must not overwrite a bit we
  // already hold. Only the first-hand assignment above can clear.
  //
  // WHY IT EXISTS: without it, a robot that cannot hear a finished peer counts
  // it missing forever (nothing ever clears the stale first-hand false), so it
  // announces the team incomplete forever, and every robot that CAN hear the
  // finished peer is held by that announcement at the unbounded appointment
  // barrier. At N>=3 that hangs the run to the harness duration cap. See
  // TeamWorld.msg/robot_finished for the full trace.
  for (size_t k = 0; k < obs.finished_gossip.size(); ++k) {
    const int r = static_cast<int>(k);
    if (r == self_id_ || r == s) continue;   // we know ourselves; s is above
    if (!obs.finished_gossip[k]) continue;
    Peer& p = peers_[static_cast<size_t>(r)];
    p.known    = true;
    p.finished = true;
  }
  return std::string();
}

void TeamModel::tick(double now_sec) {
  if (!configured()) return;

  // --- 1. direct contact and the handshake ---------------------------------
  for (int i = 0; i < size_; ++i) {
    if (i == self_id_) continue;
    Peer& p = peers_[static_cast<size_t>(i)];
    const double at = reported_at_sec_[static_cast<size_t>(i)];
    const bool receiving = at >= 0.0 && (now_sec - at) <= cfg_.direct_ttl_sec;
    // Rule 1: receiving is only half a link. The peer's own mask must name us
    // back, or this is the one-way contact of doc/limitations.md §10 and a
    // "reconnection" over it would share nothing.
    const bool mutual =
        receiving && maskHas(reported_mask_[static_cast<size_t>(i)], self_id_);
    p.direct        = mutual;
    p.heard_one_way = receiving && !mutual;
    p.via_relay     = false;
    p.status        = mutual ? CommsStatus::IN_COMMS : CommsStatus::LOST_COMMS;
    if (mutual) {
      p.last_direct_sec = at;
      if (at > p.last_known_sec) p.last_known_sec = at;
    }
  }

  if (!cfg_.closure_enabled) return;

  // --- 2. closure ----------------------------------------------------------
  //
  // Peer C is IN_COMMS if some robot we are already in comms with hears C
  // directly. Expansion goes only through rows we have a FRESH first-hand copy
  // of — which, since a peer's row arrives on its own message, means only
  // through robots we can hear ourselves. Closure is therefore TWO HOPS, not
  // unbounded: A—B—C is covered (the case §3.3 is written for), A—B—C—D is
  // not, because nothing in TeamWorld carries C's mask to A. That is a
  // limitation of the message, not an oversight here; extending it would mean
  // relaying third-party masks the way positions are relayed, and the plan
  // deliberately does not.
  //
  // The far edge B—C is taken on B's word alone. It cannot be handshaken from
  // here — we cannot hear C — and requiring it would make closure impossible
  // rather than careful. The exposure is bounded: a wrong closure suppresses a
  // reconnection dispatch, and every consumer that would act on C's DATA keys
  // on lastKnownAgeSec() instead, which no closure can inflate.
  for (int i = 0; i < size_; ++i) {
    if (i == self_id_) continue;
    const Peer& p = peers_[static_cast<size_t>(i)];
    if (p.direct) continue;
    // Is i heard directly by anyone we are in comms with?
    for (int b = 0; b < size_; ++b) {
      if (b == self_id_ || b == i) continue;
      const Peer& bridge = peers_[static_cast<size_t>(b)];
      if (!bridge.direct) continue;   // only first-hand rows may be expanded
      if (!maskHas(reported_mask_[static_cast<size_t>(b)], i)) continue;
      Peer& q = peers_[static_cast<size_t>(i)];
      q.status    = CommsStatus::IN_COMMS;
      q.via_relay = true;
      q.known     = true;
      break;
    }
  }
}

bool TeamModel::inComms(int id) const {
  if (id < 0 || id >= size_) return false;
  return peers_[static_cast<size_t>(id)].status == CommsStatus::IN_COMMS;
}

double TeamModel::lastKnownAgeSec(int id, double now_sec) const {
  if (id < 0 || id >= size_) return -1.0;
  if (id == self_id_) return 0.0;
  const double t = peers_[static_cast<size_t>(id)].last_known_sec;
  if (t < 0.0) return -1.0;
  return std::max(0.0, now_sec - t);
}

double TeamModel::positionAgeSec(int id, double now_sec) const {
  if (id < 0 || id >= size_) return -1.0;
  // Self is deliberately NOT special-cased to 0 the way the other two are:
  // this model never stores a position for self (nothing observes us), so
  // answering "0 seconds old" would be a confident age for a position that
  // does not exist. -1 says what is true — ask the localiser.
  const Peer& p = peers_[static_cast<size_t>(id)];
  if (!p.have_position || p.position_sec < 0.0) return -1.0;
  return std::max(0.0, now_sec - p.position_sec);
}

double TeamModel::lastDirectAgeSec(int id, double now_sec) const {
  if (id < 0 || id >= size_) return -1.0;
  if (id == self_id_) return 0.0;
  const double t = peers_[static_cast<size_t>(id)].last_direct_sec;
  if (t < 0.0) return -1.0;
  return std::max(0.0, now_sec - t);
}

uint32_t TeamModel::inCommsMask() const {
  uint32_t m = 0;
  for (int i = 0; i < size_; ++i)
    if (peers_[static_cast<size_t>(i)].status == CommsStatus::IN_COMMS)
      m |= robotBit(i);
  return m;
}

uint32_t TeamModel::directMask() const {
  uint32_t m = robotBit(self_id_);
  for (int i = 0; i < size_; ++i) {
    if (i == self_id_) continue;
    // "I am receiving from them", not "the handshake completed" — see the
    // header. The mutual test is the RECEIVER's to make, from this mask.
    if (peers_[static_cast<size_t>(i)].direct ||
        peers_[static_cast<size_t>(i)].heard_one_way)
      m |= robotBit(i);
  }
  return m;
}

int TeamModel::lostCount() const {
  int n = 0;
  for (int i = 0; i < size_; ++i) {
    if (i == self_id_) continue;
    if (peers_[static_cast<size_t>(i)].status != CommsStatus::IN_COMMS) ++n;
  }
  return n;
}

}  // namespace explo_planner
