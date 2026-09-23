// Moved comments: doc/team_model_notes.md
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

  // Gossip arrays longer than the fleet mean the sender runs a different
  // team_robot_names; refuse rather than truncate, which would misapply its
  // ids. Backstop to the upstream team_hash check.
  // (notes: team-gossip-array-length-backstop)
  const size_t n = static_cast<size_t>(size_);
  if (obs.last_heard_sec.size() > n || obs.gx.size() > n ||
      obs.gy.size() > n || obs.gz.size() > n ||
      obs.have_gossip_pos.size() > n || obs.finished_gossip.size() > n ||
      obs.mode_gossip.size() > n) {
    std::snprintf(buf, sizeof(buf),
                  "gossip arrays are longer than the fleet (%d): the sender "
                  "is running a different team definition", size_);
    return buf;
  }

  const int s = obs.sender_id;
  Peer& sp = peers_[static_cast<size_t>(s)];
  sp.known          = true;
  sp.in_range_mask  = obs.in_range_mask;
  // First-hand is authoritative: this plain assignment is the only site that
  // can clear finished (the relay below only sets it), which a restarted node
  // that genuinely un-finishes needs. (notes: team-first-hand-finished-clears)
  sp.finished       = obs.finished;
  // Same channel, same rule, one extra level: first-hand is authoritative and
  // is therefore the one site that can lower the level, for the restart reason
  // spelled out directly above. The relay below can only raise it.
  sp.mode           = obs.mode;
  // First-hand only, never relayed: team_incomplete is a non-monotonic claim
  // about the team, and a relayed copy could echo back to its originator with
  // nothing able to clear it. (notes: team-incomplete-first-hand-only)
  sp.team_incomplete = obs.team_incomplete;
  // First-hand only for the same reason as team_incomplete directly above: it is
  // non-monotonic, so a relayed copy could hold an appointment barrier off an
  // echo of itself.
  sp.appointment_inbound = obs.appointment_inbound;
  // The sender's own report of what it currently receives, stored under the
  // sender. The sender derives it from raw first-hand bits only, so storing it
  // cannot start an echo. (notes: team-appointment-inbound-seen)
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
  // The sender's own entry is its publish time; sender_now minus an entry is an
  // age, which transfers between clocks. Without the sender's own entry the
  // array is dropped, not guessed at. (notes: team-gossip-clock-ages)
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
        // Local mission-elapsed time this information dates from; transit
        // counts as instantaneous. Do not reject a non-increasing sender entry:
        // that deadlocks a peer whose node restarts.
        // (notes: team-gossip-transit-bound)
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
          // Stamp with at, not now_sec: a relayed position dates from when the
          // relay heard it, and the receipt time would make an old pose look
          // fresh. (notes: team-relayed-position-stamp)
          p.position_sec        = at;
          p.position_x = obs.gx[k];
          p.position_y = obs.gy[k];
          p.position_z = obs.gz[k];
        }
      }
    }
  }

  // --- relayed `finished` ---------------------------------------------------
  // A separate loop with no freshness gates: a finished robot goes quiet and
  // would age out. Pure OR: a relayed false carries no information, and only
  // the first-hand assignment clears. (notes: team-relayed-finished)
  for (size_t k = 0; k < obs.finished_gossip.size(); ++k) {
    const int r = static_cast<int>(k);
    if (r == self_id_ || r == s) continue;   // we know ourselves; s is above
    if (!obs.finished_gossip[k]) continue;
    Peer& p = peers_[static_cast<size_t>(r)];
    p.known    = true;
    p.finished = true;
  }

  // --- relayed `mode` -------------------------------------------------------
  // Same rules as relayed finished: not age-gated, and the merge only raises
  // (max over the ordered levels). It covers a peer heading home that has not
  // yet finished. (notes: team-relayed-mode)
  for (size_t k = 0; k < obs.mode_gossip.size(); ++k) {
    const int r = static_cast<int>(k);
    if (r == self_id_ || r == s) continue;   // we know ourselves; s is above
    if (obs.mode_gossip[k] == 0) continue;
    Peer& p = peers_[static_cast<size_t>(r)];
    p.known = true;
    p.mode  = std::max(p.mode, obs.mode_gossip[k]);
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

    // --- R1 instrumentation ---------------------------------------------
    // Read the transition edges before the assignments below overwrite the
    // previous state. acquire_sec and held_sec describe this tick only, so they
    // reset to -1 first. (notes: team-r1-instrumentation-edges)
    p.acquire_sec = -1.0;
    p.held_sec    = -1.0;
    if (!receiving) {
      // The run is over, whichever way it ended. Both anchors go with it so
      // the next run measures itself and not the gap between runs.
      p.one_way_since_sec = -1.0;
      p.direct_since_sec  = -1.0;
    } else {
      if (p.one_way_since_sec < 0.0) p.one_way_since_sec = at;
      if (mutual && !p.direct) {
        p.direct_since_sec = at;
        p.acquire_sec      = at - p.one_way_since_sec;
      } else if (!mutual && p.direct && p.direct_since_sec >= 0.0) {
        // STILL RECEIVING, by construction: this branch is inside the
        // `receiving` arm. The peer is audible and has stopped naming us —
        // the §10 one-way failure arriving on a link that was up.
        p.held_sec         = at - p.direct_since_sec;
        p.direct_since_sec = -1.0;
        // The next one-way period starts here, not at the start of the
        // receiving run; otherwise a re-formed link reports the preceding hold
        // as acquisition latency. (notes: team-one-way-period-restart)
        p.one_way_since_sec = at;
      }
    }

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
  // Peer i is IN_COMMS via relay when a directly-connected peer's mask names
  // it: two hops only, since only first-hand rows expand. The far edge is taken
  // on the bridge's word; closure never touches last_known_sec.
  // (notes: team-closure-two-hops)
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
