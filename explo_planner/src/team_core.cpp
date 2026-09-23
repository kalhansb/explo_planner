#include "explo_planner/team_core.hpp"

#include <algorithm>
#include <cmath>

#include "explo_planner/fleet_identity.hpp"

namespace explo_planner {
namespace gen34 {

namespace {
constexpr double kPi = 3.14159265358979323846;
using F = LogField;
}  // namespace

double dist(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

bool parseArm(const std::string& s, Arm* out) {
  if (s == "off")        { *out = Arm::kOff;        return true; }
  if (s == "pursuit")    { *out = Arm::kPursuit;    return true; }
  if (s == "rendezvous") { *out = Arm::kRendezvous; return true; }
  if (s == "hybrid")     { *out = Arm::kHybrid;     return true; }
  return false;
}

const char* armName(Arm a) {
  switch (a) {
    case Arm::kOff:        return "off";
    case Arm::kPursuit:    return "pursuit";
    case Arm::kRendezvous: return "rendezvous";
    case Arm::kHybrid:     return "hybrid";
  }
  return "?";
}

const char* activityName(Activity a) {
  switch (a) {
    case Activity::kExplore: return "explore";
    case Activity::kMeet:    return "meet";
    case Activity::kChase:   return "chase";
    case Activity::kFollow:  return "follow";
    case Activity::kWait:    return "wait";
    case Activity::kHome:    return "home";
    case Activity::kDone:    return "done";
  }
  return "?";
}

const std::vector<std::string>& TeamCore::counterNames() {
  static const std::vector<std::string> names = {
      "beacons_heard", "beacon_bad_id", "beacon_foreign_team",
      "beacon_grid_mismatch", "beacon_stale",
      "exchange_start", "exchange_done", "exchange_gave_up",
      "exchange_done_trivial", "exchange_lost",
      "plan_proposed", "plan_firmed", "plan_renewed", "plan_adopted",
      "plan_solve_refused",
      "booking_booked", "booking_departed", "booking_depart_deferred",
      "booking_arrived", "booking_pair_met", "booking_full_met",
      "booking_retargeted", "booking_split_alternate", "booking_no_slot",
      "booking_renewal_unseen", "booking_met", "booking_partial",
      "booking_missed", "booking_patience", "booking_backstop",
      "booking_encounter", "booking_team-finished", "booking_renewed",
      "reconnect_moves", "reconnect_contact", "reconnect_no_contact",
      "chase_gate_dispatch", "chase_gate_declined", "chase_gate_failopen",
      "chase_start", "chase_first_intercept", "chase_first_goal",
      "chase_first_last-position", "chase_point_reached", "chase_point_expired",
      "chase_contact", "chase_contact_lost", "chase_done", "chase_failed",
      "chase_limit", "chase_pre-empted",
      "homing_start", "homing_arrived", "homing_gave_up", "homing_no-home",
      "homing_interrupted", "team_finished",
      "activity_explore", "activity_meet", "activity_chase", "activity_follow",
      "activity_wait", "activity_home", "activity_done",
  };
  return names;
}

TeamCore::TeamCore(const TeamConfig& cfg, TeamOracle* oracle)
    : cfg_(cfg), oracle_(oracle) {
  if (cfg_.n < 1) cfg_.n = 1;
  if (cfg_.n > kMaxTeamSize) cfg_.n = kMaxTeamSize;
  peers_.resize(cfg_.n);
  present_.assign(cfg_.n, false);
  contact_.assign(cfg_.n, false);
  ex_.resize(cfg_.n);
  for (const std::string& k : counterNames()) counts_[k] = 0;
}

// ── Beacons ─────────────────────────────────────────────────────────────────

bool TeamCore::onBeacon(const Beacon& b, double rx_time) {
  if (b.team_hash != cfg_.team_hash) { count("beacon_foreign_team"); return false; }
  if (b.robot_id < 0 || b.robot_id >= cfg_.n || b.robot_id == cfg_.self_id) {
    count("beacon_bad_id");
    return false;
  }
  PeerRecord& r = peers_[b.robot_id];
  // Best-effort delivery is scheduled by transmission time, so two beacons
  // can land out of order. The older one says nothing new.
  if (r.ever && b.stamp < r.b.stamp) { count("beacon_stale"); return false; }
  r.ever = true;
  r.rx_time = rx_time;
  r.b = b;
  r.grid_ok = (b.grid_hash == cfg_.grid_hash);
  if (r.grid_ok) r.version_seen = b.plan.version;
  else count("beacon_grid_mismatch");
  count("beacons_heard");
  return true;
}

// ── Presence (Q8, Q43, E2) ──────────────────────────────────────────────────

void TeamCore::updatePresence() {
  const int self = cfg_.self_id;
  for (int j = 0; j < cfg_.n; ++j) {
    const PeerRecord& r = peers_[j];
    present_[j] = j != self && r.ever && (now_ - r.rx_time) <= cfg_.presence_window_sec;
    contact_[j] = present_[j] && maskHas(r.b.hears_mask, self);
    if (contact_[j]) peers_[j].last_contact = now_;
  }
  if (allConnected()) team_was_all_connected_ = true;
}

bool TeamCore::present(int j) const {
  return j >= 0 && j < cfg_.n && present_[j];
}

bool TeamCore::inContact(int j) const {
  return j >= 0 && j < cfg_.n && contact_[j];
}

bool TeamCore::pairInContact(int a, int b) const {
  if (a < 0 || b < 0 || a >= cfg_.n || b >= cfg_.n) return false;
  if (a == b) return true;
  const int self = cfg_.self_id;
  if (a == self) return contact_[b];
  if (b == self) return contact_[a];
  return present_[a] && present_[b] &&
         maskHas(peers_[a].b.hears_mask, b) && maskHas(peers_[b].b.hears_mask, a);
}

bool TeamCore::allConnected() const {
  if (cfg_.n < 2) return false;
  for (int a = 0; a < cfg_.n; ++a)
    for (int b = a + 1; b < cfg_.n; ++b)
      if (!pairInContact(a, b)) return false;
  return true;
}

uint32_t TeamCore::othersMask() const {
  return fleetMask(cfg_.n) & ~robotBit(cfg_.self_id);
}

bool TeamCore::lastHeardPosition(int j, Vec2* p, double* age) const {
  if (j < 0 || j >= cfg_.n || j == cfg_.self_id || !peers_[j].ever) return false;
  if (p) *p = peers_[j].b.position;
  if (age) *age = now_ - peers_[j].rx_time;
  return true;
}

bool TeamCore::proximityExempt(int j) const {
  return activity_ == Activity::kMeet && present(j) &&
         peers_[j].b.activity == Activity::kMeet;
}

// ── Exchange (Q18, Q27, Q51) ────────────────────────────────────────────────

namespace {
uint64_t peerRxOf(const Beacon& b, int id) {
  return (id >= 0 && static_cast<size_t>(id) < b.map_rcvd_seq.size())
             ? b.map_rcvd_seq[id] : 0;
}
}  // namespace

void TeamCore::updateExchanges() {
  const int self = cfg_.self_id;
  for (int j = 0; j < cfg_.n; ++j) {
    if (j == self) continue;
    ExchangeRecord& e = ex_[j];
    const uint64_t rx  = in_.rcvd_seq[j];
    const uint64_t prx = peerRxOf(peers_[j].b, self);
    const uint64_t gaps = in_.seq_gaps[j];
    if (contact_[j]) {
      if (!e.active && !in_.seq_valid) continue;
      if (!e.active) {
        e = ExchangeRecord{};
        e.active = true;
        e.state = ExchangeRecord::State::kRunning;
        e.start = now_;
        e.last_progress = now_;
        e.target_rx = peers_[j].b.map_sent_seq;
        e.target_tx = in_.sent_seq;
        e.last_rx = rx;
        e.last_peer_rx = prx;
        e.gaps_start = gaps;
        e.trivial = rx >= e.target_rx && prx >= e.target_tx;
        count("exchange_start");
        emit("exchange", {F::str("action", "start"), F::integer("peer", j),
                          F::integer("target_rx", (long long)e.target_rx),
                          F::integer("target_tx", (long long)e.target_tx),
                          F::integer("rx", (long long)rx),
                          F::integer("peer_rx", (long long)prx)});
      }
      if (e.state == ExchangeRecord::State::kRunning) {
        if (rx > e.last_rx || prx > e.last_peer_rx) e.last_progress = now_;
        e.last_rx = std::max(e.last_rx, rx);
        e.last_peer_rx = std::max(e.last_peer_rx, prx);
        const char* end = nullptr;
        if (rx >= e.target_rx && prx >= e.target_tx) {
          e.state = ExchangeRecord::State::kDone;
          end = "done";
        } else if (now_ - e.last_progress >= cfg_.exchange_stall_sec) {
          e.state = ExchangeRecord::State::kGaveUp;
          end = "gave_up_stall";
        } else if (now_ - e.start >= cfg_.exchange_total_sec) {
          e.state = ExchangeRecord::State::kGaveUp;
          end = "gave_up_total";
        }
        if (end) {
          const bool done = e.state == ExchangeRecord::State::kDone;
          count(done ? "exchange_done" : "exchange_gave_up");
          if (done && e.trivial) count("exchange_done_trivial");
          emit("exchange", {F::str("action", end), F::integer("peer", j),
                            F::num("duration_sec", now_ - e.start),
                            F::integer("target_rx", (long long)e.target_rx),
                            F::integer("target_tx", (long long)e.target_tx),
                            F::integer("rx", (long long)rx),
                            F::integer("peer_rx", (long long)prx),
                            F::integer("gaps", (long long)(gaps - std::min(gaps, e.gaps_start)))});
        }
      }
    } else if (e.active) {
      if (e.state == ExchangeRecord::State::kRunning) {
        count("exchange_lost");
        emit("exchange", {F::str("action", "contact_lost"), F::integer("peer", j),
                          F::num("duration_sec", now_ - e.start),
                          F::integer("target_rx", (long long)e.target_rx),
                          F::integer("target_tx", (long long)e.target_tx),
                          F::integer("rx", (long long)rx),
                          F::integer("peer_rx", (long long)prx)});
      }
      e.active = false;
    }
  }
}

bool TeamCore::exchanged(int j) const {
  return j >= 0 && j < cfg_.n && ex_[j].active &&
         ex_[j].state == ExchangeRecord::State::kDone;
}

bool TeamCore::exchangeGaveUp(int j) const {
  return j >= 0 && j < cfg_.n && ex_[j].active &&
         ex_[j].state == ExchangeRecord::State::kGaveUp;
}

bool TeamCore::anyExchangeRunning() const {
  for (int j = 0; j < cfg_.n; ++j)
    if (ex_[j].active && ex_[j].state == ExchangeRecord::State::kRunning) return true;
  return false;
}

bool TeamCore::allPairsExchanged() const {
  const int self = cfg_.self_id;
  auto lists = [&](int x, int y) {
    if (x == self) return exchanged(y);
    return peers_[x].ever && maskHas(peers_[x].b.exchanged_mask, y);
  };
  for (int a = 0; a < cfg_.n; ++a)
    for (int b = a + 1; b < cfg_.n; ++b)
      if (!lists(a, b) && !lists(b, a)) return false;
  return true;
}

// ── team_finished (Q57a) ────────────────────────────────────────────────────

void TeamCore::updateTeamFinished() {
  if (team_finished_) return;
  const int self = cfg_.self_id;
  const char* why = nullptr;
  bool all = in_.finished;
  for (int j = 0; j < cfg_.n && all; ++j)
    if (j != self) all = present_[j] && peers_[j].b.finished;
  if (all) why = "all-heard";
  for (int j = 0; j < cfg_.n && !why; ++j)
    if (j != self && peers_[j].ever && peers_[j].b.team_finished) why = "beacon";
  if (!why) return;
  team_finished_ = true;
  count("team_finished");
  emit("team_finished", {F::str("reason", why)});
}

// ── Plan (R1, Q47, Q54) ─────────────────────────────────────────────────────

std::vector<TeamRobotView> TeamCore::teamView() const {
  std::vector<TeamRobotView> v(cfg_.n);
  for (int j = 0; j < cfg_.n; ++j) {
    v[j].id = j;
    if (j == cfg_.self_id) {
      v[j].known = in_.have_pose;
      v[j].position = in_.pose;
      v[j].finished = in_.finished;
    } else {
      v[j].known = peers_[j].ever;
      v[j].position = peers_[j].b.position;
      v[j].finished = peers_[j].b.finished;
    }
  }
  return v;
}

void TeamCore::adoptPlan(const Plan& p, int from) {
  prev_plan_ = plan_;
  plan_ = p;
  count("plan_adopted");
  emit("plan", {F::str("action", "adopted"), F::integer("from", from),
                F::integer("version", p.version), F::integer("cell", p.cell),
                F::num("cx", p.center.x), F::num("cy", p.center.y),
                F::num("t0", p.t0), F::num("interval", p.interval),
                F::integer("renewed_at_slot", p.renewed_at_slot),
                F::boolean("provisional", p.provisional)});
}

void TeamCore::proposePlan(const char* why, int renewed_at_slot) {
  const PlanSolve s = oracle_->solvePlan(teamView());
  if (!s.ok && !plan_.valid()) {
    // Retried every plan_resolve_period_sec while the team is together, so
    // only the first refusal and every 12th after it are logged.
    count("plan_solve_refused");
    if (counts_["plan_solve_refused"] % 12 == 1)
      emit("plan", {F::str("action", "refused"), F::str("why", why),
                    F::str("refused", s.refused),
                    F::integer("attempts", counts_["plan_solve_refused"])});
    return;
  }
  // A firming re-solve only replaces a provisional plan with a solved one.
  if (std::string(why) == "firmed" && (!s.ok || s.provisional)) return;
  Plan p;
  p.version = plan_.version + 1;
  if (s.ok) {
    p.cell = s.cell;
    p.center = s.center;
    p.provisional = s.provisional;
  } else {
    // Renewal with a failed solve keeps the same cell.
    p.cell = plan_.cell;
    p.center = plan_.center;
    p.provisional = plan_.provisional;
  }
  if (plan_.valid()) {
    p.t0 = plan_.t0;
    p.interval = plan_.interval;
  } else {
    p.t0 = now_ + cfg_.meeting_interval_sec;
    p.interval = cfg_.meeting_interval_sec;
  }
  // A firming re-solve replaces the cell, not the history: the slot the last
  // renewal proved met stays proved, or every attendee falls to the renewal
  // patience and book() could re-book that slot.
  p.renewed_at_slot = std::string(why) == "firmed" ? plan_.renewed_at_slot : renewed_at_slot;
  prev_plan_ = plan_;
  plan_ = p;
  count(std::string("plan_") + why);
  emit("plan", {F::str("action", why), F::integer("version", p.version),
                F::integer("cell", p.cell), F::num("cx", p.center.x),
                F::num("cy", p.center.y), F::num("t0", p.t0),
                F::num("interval", p.interval),
                F::integer("renewed_at_slot", p.renewed_at_slot),
                F::boolean("provisional", p.provisional),
                F::str("refused", s.ok ? "" : s.refused)});
}

void TeamCore::updatePlan() {
  if (!armBooks(cfg_.arm) || cfg_.n < 2) return;
  const int self = cfg_.self_id;
  for (int j = 0; j < cfg_.n; ++j) {
    if (j == self) continue;
    const PeerRecord& r = peers_[j];
    if (!r.ever || !r.grid_ok) continue;
    if (r.b.plan.valid() && r.b.plan.version > plan_.version) adoptPlan(r.b.plan, j);
  }
  if (self != 0) return;
  if (!allConnected()) return;
  if (plan_.valid() && !plan_.provisional) return;
  if (now_ - last_solve_ < cfg_.plan_resolve_period_sec) return;
  last_solve_ = now_;
  proposePlan(plan_.valid() ? "firmed" : "proposed", -1);
}

void TeamCore::cellForSlot(int k, int* cell, Vec2* center) const {
  *cell = plan_.cell;
  *center = plan_.center;
  if (!prev_plan_.valid() || prev_plan_.version >= plan_.version) return;
  if (prev_plan_.cell == plan_.cell) return;
  if (k % 2 == 0) return;
  for (int j = 0; j < cfg_.n; ++j) {
    if (j == cfg_.self_id) continue;
    const PeerRecord& r = peers_[j];
    if (r.ever && r.grid_ok && r.version_seen == prev_plan_.version) {
      *cell = prev_plan_.cell;
      *center = prev_plan_.center;
      return;
    }
  }
}

// ── Booking (rendezvous, hybrid) ────────────────────────────────────────────

double TeamCore::leadFrom(const Vec2& from, int cell, const Vec2& center) {
  const double d = oracle_->pathDistance(from, cell, center);
  const double v = cfg_.travel_speed_mps > 1e-6 ? cfg_.travel_speed_mps : 1e-6;
  return d / v * cfg_.depart_safety;
}

Vec2 TeamCore::ringSpot(const Vec2& center, double* radius) {
  const double a = 2.0 * kPi * cfg_.self_id / std::max(1, cfg_.n);
  const double step = cfg_.ring_step_m > 1e-3 ? cfg_.ring_step_m : 1.0;
  Vec2 p;
  for (double r = cfg_.meeting_ring_m; r <= cfg_.ring_max_m + 1e-9; r += step) {
    p = {center.x + r * std::cos(a), center.y + r * std::sin(a)};
    if (oracle_->standable(p)) { if (radius) *radius = r; return p; }
  }
  const double r = std::max(cfg_.meeting_ring_m, cfg_.ring_max_m);
  if (radius) *radius = r;
  return {center.x + r * std::cos(a), center.y + r * std::sin(a)};
}

void TeamCore::book() {
  const double interval = plan_.interval;
  int k = std::max(0, static_cast<int>(std::ceil((now_ - plan_.t0) / interval)));
  const int self = cfg_.self_id;
  for (int tries = 0; tries < 1000; ++tries, ++k) {
    const double T = plan_.slotTime(k);
    if (T <= now_) continue;
    int cell;
    Vec2 center;
    cellForSlot(k, &cell, &center);
    const double lead_me = in_.have_pose ? leadFrom(in_.pose, cell, center) : 0.0;
    bool ok = (T - now_) >= lead_me;
    for (int j = 0; j < cfg_.n && ok; ++j) {
      if (j == self || !peers_[j].ever) continue;
      ok = (T - now_) >= leadFrom(peers_[j].b.position, cell, center);
    }
    if (!ok) continue;
    // A slot whose meeting already happened is not booked again, and nothing
    // is booked while the first slot every robot can make is such a slot.
    // Booking the one after it instead would let a team that keeps drifting
    // in and out of range meet slot after slot ahead of time, spending future
    // slots and pushing the next real meeting out by an interval each time.
    // A renewal at slot k is robot 0's proof that the team met k, which covers
    // a robot whose own booking ended as an encounter.
    if (k < std::max(min_slot_, plan_.renewed_at_slot + 1)) return;
    booking_ = Booking{};
    booking_.held = true;
    booking_.slot = k;
    booking_.slot_time = T;
    booking_.cell = cell;
    booking_.center = center;
    booking_.plan_version = plan_.version;
    booking_.booked_at = now_;
    if (cell != plan_.cell) count("booking_split_alternate");
    count("booking_booked");
    emit("booking", {F::str("action", "booked"), F::integer("slot", k),
                     F::num("slot_time", T), F::integer("cell", cell),
                     F::num("cx", center.x), F::num("cy", center.y),
                     F::num("lead_sec", lead_me),
                     F::integer("plan_version", plan_.version),
                     F::integer("missed_streak", missed_streak_)});
    return;
  }
  count("booking_no_slot");
}

void TeamCore::clearBooking(const char* reason) {
  Booking& B = booking_;
  const uint32_t others = othersMask();
  const bool someone_unheard = B.departed && (B.heard_mask & others) != others;
  if (someone_unheard) ++missed_streak_;
  else missed_streak_ = 0;
  if (B.departed || B.full_met || std::string(reason) == "renewed")
    min_slot_ = std::max(min_slot_, B.slot + 1);
  ++booking_clears_;
  count(std::string("booking_") + reason);
  emit("booking", {F::str("action", reason), F::integer("slot", B.slot),
                   F::num("slot_time", B.slot_time), F::integer("cell", B.cell),
                   F::boolean("departed", B.departed),
                   F::boolean("arrived", B.arrived),
                   F::num("held_sec", B.arrived ? now_ - B.arrive_time : 0.0),
                   F::integer("met_mask", B.met_mask),
                   F::integer("heard_mask", B.heard_mask),
                   F::boolean("full_met", B.full_met),
                   F::integer("missed_streak", missed_streak_)});
  booking_ = Booking{};
}

bool TeamCore::pairMet(int a, int b, int slot) const {
  const int self = cfg_.self_id;
  auto lists = [&](int x, int y) {
    if (x == self)
      return booking_.held && booking_.slot == slot && maskHas(booking_.met_mask, y);
    const PeerRecord& r = peers_[x];
    return r.ever && r.b.meeting_slot == slot && maskHas(r.b.met_mask, y);
  };
  return lists(a, b) || lists(b, a);
}

bool TeamCore::fullMet(int slot) const {
  if (cfg_.n < 2) return false;
  for (int a = 0; a < cfg_.n; ++a)
    for (int b = a + 1; b < cfg_.n; ++b)
      if (!pairMet(a, b, slot)) return false;
  return true;
}

void TeamCore::updateBooking() {
  if (!armBooks(cfg_.arm) || cfg_.n < 2) return;
  const long long clears = booking_clears_;
  bookingPass();
  // A booking that ended this tick is followed by the next one in the same
  // tick, so a robot going from one slot to the next shows no one-tick stint
  // of Explore or Wait (and no brake) between the two.
  if (booking_clears_ != clears && !booking_.held) bookingPass();
}

void TeamCore::bookingPass() {
  const int self = cfg_.self_id;
  if (allConnected()) apart_since_ = -1.0;
  else if (apart_since_ < 0.0) apart_since_ = now_;
  const bool apart = apart_since_ >= 0.0 && now_ - apart_since_ >= cfg_.book_apart_sec;

  // Once the whole team is finished, nothing is booked and Home holds (Q57a),
  // a departed booking included: the homes are close, so the maps still move
  // on the way and at home.
  if (booking_.held && team_finished_) clearBooking("team-finished");
  else if (booking_.held && !booking_.departed && allConnected() && allPairsExchanged())
    clearBooking("encounter");
  // The team fully met at this slot (or a later one) without this robot
  // seeing it from its own booking: robot 0 renews only on a full meeting, so
  // the renewal is the proof. It happens when this robot took part by radio
  // from outside the cell, or heard the renewal before its own view of the
  // pairs caught up.
  if (booking_.held && !booking_.full_met && plan_.renewed_at_slot >= booking_.slot)
    clearBooking("renewed");
  if (!booking_.held && plan_.valid() && !team_finished_ && apart && !home_done_ &&
      !(in_.finished && missed_streak_ >= cfg_.finished_missed_slots)) {
    book();
  }
  if (!booking_.held) return;
  Booking& B = booking_;

  // A plan adopted before arrival moves the meeting (Q54).
  if (!B.arrived) {
    int c;
    Vec2 ctr;
    cellForSlot(B.slot, &c, &ctr);
    if (c != B.cell || dist(ctr, B.center) > 1e-6) {
      B.cell = c;
      B.center = ctr;
      if (B.departed) B.spot = ringSpot(B.center, &B.spot_radius);
      count("booking_retargeted");
      emit("booking", {F::str("action", "retargeted"), F::integer("slot", B.slot),
                       F::integer("cell", c), F::integer("plan_version", plan_.version)});
    }
  }

  if (!B.departed) {
    const double lead = in_.have_pose ? leadFrom(in_.pose, B.cell, B.center) : 0.0;
    if (!(in_.finished || now_ + lead >= B.slot_time)) return;
    // An encounter in progress is the meeting: the whole team is in contact
    // and maps are still moving. Leaving for the cell would break it off; the
    // encounter rule above clears the booking once every pair has exchanged,
    // and an exchange that gives up ends the deferral. A presence lapse
    // shorter than book_apart_sec does not end it, as it does not book.
    if (!apart && anyExchangeRunning()) {
      if (!B.depart_deferred) {
        B.depart_deferred = true;
        count("booking_depart_deferred");
        emit("booking", {F::str("action", "depart_deferred"), F::integer("slot", B.slot),
                         F::num("slot_time", B.slot_time), F::num("lead_sec", lead)});
      }
      return;
    }
    B.departed = true;
    B.depart_time = now_;
    B.spot = ringSpot(B.center, &B.spot_radius);
    B.leg_key = next_leg_key_++;
    B.seq_rx = in_.rcvd_seq;
    B.seq_peer_rx.assign(cfg_.n, 0);
    for (int j = 0; j < cfg_.n; ++j)
      if (j != self) B.seq_peer_rx[j] = peerRxOf(peers_[j].b, self);
    count("booking_departed");
    emit("booking", {F::str("action", "departed"), F::integer("slot", B.slot),
                     F::num("slot_time", B.slot_time), F::num("lead_sec", lead),
                     F::boolean("finished", in_.finished),
                     F::num("spot_x", B.spot.x), F::num("spot_y", B.spot.y),
                     F::num("spot_radius", B.spot_radius)});
    if (chase_.held) clearChase("pre-empted");
  }

  if (!B.arrived) B.spot = ringSpot(B.center, &B.spot_radius);

  for (int j = 0; j < cfg_.n; ++j) {
    if (j == self) continue;
    const uint32_t bit = robotBit(j);
    if (present_[j] && !(B.heard_mask & bit)) {
      B.heard_mask |= bit;
      B.last_progress = now_;   // an attendee heard for the first time
    }
    if (contact_[j]) {
      B.contact_mask |= bit;
      B.rc_used_mask &= ~bit;
    }
    // A slot is met only in its own interval. Earlier contact still swaps
    // maps (exchange runs in every activity) but does not spend the slot: a
    // finished robot waits at the cell from the moment it books, and counting
    // a chance contact there as the meeting would spend a slot an interval or
    // more ahead and leave the team a double gap before the next one.
    if (exchanged(j) && !(B.met_mask & bit) && now_ >= B.slot_time - plan_.interval) {
      B.met_mask |= bit;
      count("booking_pair_met");
      emit("booking", {F::str("action", "pair_met"), F::integer("slot", B.slot),
                       F::integer("peer", j)});
    }
    const uint64_t rx = in_.rcvd_seq[j];
    const uint64_t prx = peerRxOf(peers_[j].b, self);
    if (rx > B.seq_rx[j] || prx > B.seq_peer_rx[j]) B.last_progress = now_;
    B.seq_rx[j] = std::max(B.seq_rx[j], rx);
    B.seq_peer_rx[j] = std::max(B.seq_peer_rx[j], prx);
  }

  if (!B.arrived && in_.have_pose && dist(in_.pose, B.spot) <= cfg_.meeting_arrive_m) {
    B.arrived = true;
    B.arrive_time = now_;
    B.last_progress = now_;
    count("booking_arrived");
    emit("booking", {F::str("action", "arrived"), F::integer("slot", B.slot),
                     F::num("early_sec", B.slot_time - now_)});
  }

  if (!B.full_met && fullMet(B.slot)) {
    B.full_met = true;
    B.full_met_time = now_;
    count("booking_full_met");
    emit("booking", {F::str("action", "full_met"), F::integer("slot", B.slot)});
    if (self == 0 && renewed_for_slot_ != B.slot && !team_finished_) {
      renewed_for_slot_ = B.slot;
      proposePlan("renewed", B.slot);
    }
  }
  if (B.full_met) {
    bool renewed = team_finished_ || plan_.renewed_at_slot == B.slot;
    for (int j = 0; j < cfg_.n && renewed; ++j)
      if (j != self) renewed = peers_[j].ever && peers_[j].b.plan.version >= plan_.version;
    if (renewed) { clearBooking("met"); return; }
    // Unseen renewal: patience from the full meeting, capped by the backstop
    // that wait_bound and the drive's deadline report.
    if (now_ - B.full_met_time >= cfg_.meeting_patience_sec ||
        now_ >= std::max(B.slot_time, B.depart_time) + cfg_.meeting_backstop_sec) {
      count("booking_renewal_unseen");
      clearBooking("met");
      return;
    }
    return;
  }

  if (now_ >= std::max(B.slot_time, B.depart_time) + cfg_.meeting_backstop_sec) {
    clearBooking("backstop");
    return;
  }
  if (now_ >= B.slot_time + cfg_.meeting_window_sec && !anyExchangeRunning()) {
    clearBooking(B.met_mask ? "partial" : "missed");
    return;
  }
  // Patience runs from arrival or the slot time, whichever is later: an early
  // arrival waits for the slot as Q45 asks, and the clock bounds the hold
  // after it.
  if (B.arrived && now_ >= B.slot_time) {
    const double from = std::max(B.last_progress, std::max(B.arrive_time, B.slot_time));
    if (now_ - from >= cfg_.meeting_patience_sec) {
      clearBooking("patience");
      return;
    }
  }

  // Q32 / Q53: of a pair that lost each other at the cell, the higher id drives
  // to the other's last heard position; the lower id holds.
  if (!B.arrived) return;
  if (B.rc_active || B.rc_holding) {
    const int j = B.rc_peer;
    if (B.rc_active && contact_[j]) {
      B.rc_active = false;
      B.rc_holding = true;
      count("reconnect_contact");
      emit("booking", {F::str("action", "reconnect_contact"), F::integer("peer", j)});
    } else if (B.rc_active && in_.have_pose &&
               dist(in_.pose, B.rc_point) <= cfg_.leg_arrive_m) {
      B.rc_active = false;
      B.leg_key = next_leg_key_++;
      count("reconnect_no_contact");
      emit("booking", {F::str("action", "reconnect_no_contact"), F::integer("peer", j)});
    }
    if (B.rc_holding && (pairMet(self, j, B.slot) || !contact_[j])) {
      B.rc_holding = false;
      B.leg_key = next_leg_key_++;
    }
  }
  if (!B.rc_active && !B.rc_holding) {
    for (int j = 0; j < cfg_.n; ++j) {
      if (j == self || self < j) continue;
      const uint32_t bit = robotBit(j);
      if (!(B.contact_mask & bit) || contact_[j] || (B.rc_used_mask & bit)) continue;
      if (pairMet(self, j, B.slot) || !peers_[j].ever) continue;
      B.rc_active = true;
      B.rc_peer = j;
      B.rc_point = peers_[j].b.position;
      B.rc_used_mask |= bit;
      B.leg_key = next_leg_key_++;
      count("reconnect_moves");
      emit("booking", {F::str("action", "reconnect_move"), F::integer("peer", j),
                       F::num("x", B.rc_point.x), F::num("y", B.rc_point.y)});
      break;
    }
  }
}

// ── Chase (pursuit, hybrid) ─────────────────────────────────────────────────

ChaseGateView TeamCore::gateView(int j) const {
  ChaseGateView v;
  v.id = j;
  const PeerRecord& r = peers_[j];
  v.known = r.ever;
  v.last_position = r.b.position;
  v.age_sec = r.ever ? now_ - r.rx_time : 0.0;
  v.finished = r.b.finished;
  v.tour = r.b.tour;
  return v;
}

Vec2 TeamCore::followPoint(const Vec2& peer_pos) {
  double dx = in_.pose.x - peer_pos.x, dy = in_.pose.y - peer_pos.y;
  double L = std::hypot(dx, dy);
  if (L < 1e-3) { dx = 1.0; dy = 0.0; L = 1.0; }
  dx /= L;
  dy /= L;
  for (double d = cfg_.follow_distance_m; d >= cfg_.follow_min_distance_m - 1e-9; d -= 1.0) {
    const Vec2 p{peer_pos.x + dx * d, peer_pos.y + dy * d};
    if (oracle_->lineOfSight(peer_pos, p) && oracle_->standable(p)) return p;
  }
  return {peer_pos.x + dx * cfg_.follow_min_distance_m,
          peer_pos.y + dy * cfg_.follow_min_distance_m};
}

void TeamCore::clearChase(const char* reason) {
  count(std::string("chase_") + reason);
  emit("chase", {F::str("action", reason), F::integer("peer", chase_.peer),
                 F::num("duration_sec", now_ - chase_.start),
                 F::boolean("following", chase_.following),
                 F::boolean("contact_seen", chase_.contact_seen)});
  chase_ = Chase{};
  last_chase_end_ = now_;
}

void TeamCore::updateChase() {
  if (!armChases(cfg_.arm)) return;
  const int self = cfg_.self_id;
  const double v = cfg_.travel_speed_mps > 1e-6 ? cfg_.travel_speed_mps : 1e-6;
  auto startPoint = [&]() {
    const Vec2& p = chase_.points[chase_.idx].p;
    const double d = in_.have_pose ? dist(in_.pose, p) : 0.0;
    chase_.point_deadline = now_ + d / v * cfg_.leg_budget_factor + cfg_.leg_budget_slack_sec;
    chase_.leg_key = next_leg_key_++;
  };

  if (chase_.held) {
    if (booking_.held && booking_.departed) { clearChase("pre-empted"); return; }
    if (now_ - chase_.start >= cfg_.chase_limit_sec) { clearChase("limit"); return; }
    const int j = chase_.peer;
    if (contact_[j]) {
      if (!chase_.following) {
        chase_.following = true;
        chase_.leg_key = next_leg_key_++;
        if (!chase_.contact_seen) {
          chase_.contact_seen = true;
          count("chase_contact");
        }
        emit("chase", {F::str("action", "contact"), F::integer("peer", j),
                       F::num("after_sec", now_ - chase_.start)});
      }
      if (exchanged(j)) { clearChase("done"); return; }
      if (exchangeGaveUp(j)) { clearChase("failed"); return; }
      return;
    }
    if (chase_.following) {
      chase_.following = false;
      chase_.points = {{peers_[j].b.position, "last-position"}};
      chase_.idx = 0;
      startPoint();
      count("chase_contact_lost");
      emit("chase", {F::str("action", "contact_lost"), F::integer("peer", j)});
    }
    if (chase_.idx < chase_.points.size()) {
      const Vec2& p = chase_.points[chase_.idx].p;
      const bool reached = in_.have_pose && dist(in_.pose, p) <= cfg_.leg_arrive_m;
      const bool expired = now_ >= chase_.point_deadline;
      if (reached || expired) {
        count(reached ? "chase_point_reached" : "chase_point_expired");
        emit("chase", {F::str("action", reached ? "point_reached" : "point_expired"),
                       F::integer("peer", j),
                       F::str("kind", chase_.points[chase_.idx].kind)});
        ++chase_.idx;
        if (chase_.idx < chase_.points.size()) startPoint();
      }
    }
    if (chase_.idx >= chase_.points.size()) clearChase("failed");
    return;
  }

  // Trigger.
  if (!team_was_all_connected_ || in_.finished) return;
  if (now_ - last_chase_end_ < cfg_.chase_cooldown_sec) return;
  if (booking_.held) {
    if (booking_.departed) return;
    if (cfg_.arm == Arm::kHybrid) {
      const double lead = in_.have_pose ? leadFrom(in_.pose, booking_.cell, booking_.center) : 0.0;
      if (booking_.slot_time - lead - now_ <= cfg_.chase_departure_margin_sec) return;
    }
  }
  std::vector<ChaseGateView> missing;
  for (int j = 0; j < cfg_.n; ++j) {
    if (j == self) continue;
    const PeerRecord& r = peers_[j];
    if (!r.ever || r.b.finished) continue;
    // Silent: not heard for chase_silence_sec. The cut-off is from the last
    // contact (§8.3); the team was all connected, so every peer has one.
    if (now_ - r.rx_time < cfg_.chase_silence_sec) continue;
    if (r.last_contact < 0.0 || now_ - r.last_contact > cfg_.chase_max_contact_age_sec) continue;
    missing.push_back(gateView(j));
  }
  if (missing.empty()) return;
  if (now_ - last_gate_eval_ < cfg_.gate_period_sec) return;
  last_gate_eval_ = now_;
  const ChaseGateVerdict gv = oracle_->chaseGate(missing);
  const bool failopen = !gv.refused.empty();
  const bool go = gv.dispatch || failopen;
  int target = -1;
  for (const ChaseGateView& m : missing)
    if (m.id == gv.peer) target = m.id;
  if (target < 0) {
    double best = 1e18;
    for (const ChaseGateView& m : missing) {
      const double d = in_.have_pose ? dist(in_.pose, m.last_position) : 0.0;
      if (d < best) { best = d; target = m.id; }
    }
  }
  count(failopen ? "chase_gate_failopen" : (gv.dispatch ? "chase_gate_dispatch" : "chase_gate_declined"));
  if (!go) {
    emit("chase", {F::str("action", "declined"), F::integer("peer", target),
                   F::integer("c_no_mm", gv.c_no_mm), F::integer("c_re_mm", gv.c_re_mm),
                   F::integer("leg_mm", gv.leg_mm),
                   F::integer("unshared_cells", gv.unshared_cells)});
    return;
  }
  chase_ = Chase{};
  chase_.held = true;
  chase_.peer = target;
  chase_.start = now_;
  const PeerRecord& r = peers_[target];
  Vec2 ip;
  if (oracle_->intercept(gateView(target), &ip)) chase_.points.push_back({ip, "intercept"});
  if (r.b.has_goal) chase_.points.push_back({r.b.goal, "goal"});
  chase_.points.push_back({r.b.position, "last-position"});
  chase_.idx = 0;
  startPoint();
  count("chase_start");
  count(std::string("chase_first_") + chase_.points[0].kind);
  emit("chase", {F::str("action", "start"), F::integer("peer", target),
                 F::str("first", chase_.points[0].kind),
                 F::integer("points", (long long)chase_.points.size()),
                 F::boolean("failopen", failopen), F::str("refused", gv.refused),
                 F::integer("c_no_mm", gv.c_no_mm), F::integer("c_re_mm", gv.c_re_mm),
                 F::integer("leg_mm", gv.leg_mm),
                 F::num("silence_sec", now_ - r.rx_time)});
}

// ── Mission: the priority list (§8.4) ───────────────────────────────────────

Activity TeamCore::selectActivity() {
  // Done is terminal: a plan that arrives after a no-plan homing does not
  // pull the robot back out (the run may already be judged over).
  if (home_done_) return Activity::kDone;
  if (armBooks(cfg_.arm) && booking_.held && booking_.departed) return Activity::kMeet;
  if (armChases(cfg_.arm) && chase_.held)
    return chase_.following ? Activity::kFollow : Activity::kChase;
  if (!in_.finished) return Activity::kExplore;
  // A finished robot with no departed booking: the team is together, or the
  // next slot is not yet bookable, or a departure is deferred.
  if (armBooks(cfg_.arm) && plan_.valid() && !team_finished_ &&
      missed_streak_ < cfg_.finished_missed_slots)
    return Activity::kWait;
  if (!home_done_) return Activity::kHome;
  return Activity::kDone;
}

void TeamCore::updateHoming(Activity act) {
  if (act == Activity::kHome) {
    if (!homing_) {
      homing_ = true;
      homing_start_ = now_;
      home_leg_key_ = next_leg_key_++;
      const char* why = team_finished_ ? "team-finished"
                      : !armBooks(cfg_.arm) ? "arm"
                      : !plan_.valid() ? "no-plan" : "missed-streak";
      count("homing_start");
      emit("homing", {F::str("action", "start"), F::str("reason", why),
                      F::num("dist_m", in_.have_home && in_.have_pose ? dist(in_.pose, in_.home) : -1.0)});
    }
    const char* end = nullptr;
    if (!in_.have_home) end = "no-home";
    else if (in_.have_pose && dist(in_.pose, in_.home) <= cfg_.home_tol_m) end = "arrived";
    else if (now_ - homing_start_ >= cfg_.home_max_sec) end = "gave_up";
    if (end) {
      home_done_ = true;
      count(std::string("homing_") + end);
      emit("homing", {F::str("action", end), F::num("duration_sec", now_ - homing_start_)});
    }
    return;
  }
  if (act == Activity::kDone) return;
  if (homing_ && !home_done_) {
    count("homing_interrupted");
    emit("homing", {F::str("action", "interrupted"), F::str("by", activityName(act))});
  }
  homing_ = false;
  home_done_ = false;
}

Drive TeamCore::driveFor(Activity act) {
  Drive d;
  switch (act) {
    case Activity::kExplore:
      d.kind = DriveKind::kExplore;
      return d;
    case Activity::kWait:
    case Activity::kDone:
      d.kind = DriveKind::kHold;
      return d;
    case Activity::kHome:
      d.kind = DriveKind::kLeg;
      d.point = in_.home;
      d.tol = cfg_.home_tol_m;
      d.deadline = homing_start_ + cfg_.home_max_sec;
      d.key = home_leg_key_;
      d.purpose = "home";
      return d;
    case Activity::kMeet: {
      const Booking& B = booking_;
      d.deadline = std::max(B.slot_time, B.depart_time) + cfg_.meeting_backstop_sec;
      d.key = B.leg_key;
      if (B.rc_active) {
        d.kind = DriveKind::kLeg;
        d.point = B.rc_point;
        d.tol = cfg_.leg_arrive_m;
        d.purpose = "reconnect";
        return d;
      }
      if (B.rc_holding) { d.kind = DriveKind::kHold; return d; }
      d.kind = DriveKind::kLeg;
      d.point = B.spot;
      d.tol = cfg_.meeting_arrive_m;
      d.purpose = "meet";
      return d;
    }
    case Activity::kChase: {
      d.kind = DriveKind::kLeg;
      d.point = chase_.points[std::min(chase_.idx, chase_.points.size() - 1)].p;
      d.tol = cfg_.leg_arrive_m;
      d.deadline = chase_.point_deadline;
      d.key = chase_.leg_key;
      d.purpose = "chase";
      return d;
    }
    case Activity::kFollow: {
      d.kind = DriveKind::kLeg;
      d.point = followPoint(peers_[chase_.peer].b.position);
      d.tol = cfg_.leg_arrive_m;
      d.deadline = chase_.start + cfg_.chase_limit_sec;
      d.key = chase_.leg_key;
      d.purpose = "follow";
      return d;
    }
  }
  return d;
}

TickOutputs TeamCore::tick(const TickInputs& in) {
  in_ = in;
  now_ = in.now;
  in_.rcvd_seq.resize(cfg_.n, 0);
  in_.seq_gaps.resize(cfg_.n, 0);

  updatePresence();
  updateExchanges();
  updateTeamFinished();
  updatePlan();
  updateBooking();
  updateChase();

  const Activity act = selectActivity();
  if (!have_activity_ || act != activity_) {
    count(std::string("activity_") + activityName(act));
    emit("team_activity", {F::str("from", have_activity_ ? activityName(activity_) : ""),
                           F::str("to", activityName(act)),
                           F::num("dwell_sec", have_activity_ ? now_ - activity_since_ : -1.0)});
    activity_ = act;
    activity_since_ = now_;
    have_activity_ = true;
  }
  updateHoming(act);

  TickOutputs out;
  out.activity = act;
  out.drive = driveFor(act);
  switch (act) {
    case Activity::kMeet:
      out.wait_start = booking_.arrived ? booking_.arrive_time : booking_.depart_time;
      out.wait_bound = std::max(booking_.slot_time, booking_.depart_time) + cfg_.meeting_backstop_sec;
      break;
    case Activity::kChase:
    case Activity::kFollow:
      out.wait_start = chase_.start;
      out.wait_bound = chase_.start + cfg_.chase_limit_sec;
      break;
    case Activity::kHome:
      out.wait_start = homing_start_;
      out.wait_bound = homing_start_ + cfg_.home_max_sec;
      break;
    case Activity::kWait:
      out.wait_start = activity_since_;
      break;
    default:
      break;
  }
  last_drive_ = out.drive;
  return out;
}

Beacon TeamCore::beacon() const {
  Beacon b;
  b.robot_id = cfg_.self_id;
  b.team_hash = cfg_.team_hash;
  b.grid_hash = cfg_.grid_hash;
  b.stamp = now_;
  b.position = in_.pose;
  if (last_drive_.kind == DriveKind::kLeg) {
    b.goal = last_drive_.point;
    b.has_goal = true;
  }
  for (int j = 0; j < cfg_.n; ++j)
    if (present_[j]) b.hears_mask |= robotBit(j);
  b.activity = activity_;
  b.finished = in_.finished;
  b.team_finished = team_finished_;
  b.home = home_done_;
  b.map_sent_seq = in_.sent_seq;
  b.map_rcvd_seq = in_.rcvd_seq;
  if (cfg_.self_id < static_cast<int>(b.map_rcvd_seq.size()))
    b.map_rcvd_seq[cfg_.self_id] = in_.sent_seq;
  for (int j = 0; j < cfg_.n; ++j)
    if (exchanged(j)) b.exchanged_mask |= robotBit(j);
  b.plan = plan_;
  if (booking_.held && booking_.departed) {
    b.meeting_slot = booking_.slot;
    b.met_mask = booking_.met_mask;
  }
  return b;
}

std::vector<TeamEvent> TeamCore::drainEvents() {
  std::vector<TeamEvent> out;
  out.swap(events_);
  return out;
}

void TeamCore::emit(const std::string& kind, LogFields f) {
  events_.push_back(TeamEvent{kind, std::move(f)});
}

}  // namespace gen34
}  // namespace explo_planner
