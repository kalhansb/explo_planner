#pragma once
/// @file team_core.hpp
/// @brief Gen-34 team logic: presence, map exchange, the meeting plan,
///        bookings, chases and the mission priority list. Pure C++, no ROS.
///
/// See DESIGN_gen34.md §8. The node feeds one TickInputs per tick and the
/// beacons as they arrive; TeamCore answers with the activity, what to drive
/// to, the fields of this robot's own beacon, and events for the jsonl log.
/// The in-process harness (test_team_harness.cpp) runs N of these against a
/// toy world, so everything that decides what the team does lives here.
///
/// COMMITMENTS ARE STORED; THE ACTIVITY IS NOT (§8.1). Each component holds its
/// own commitment (an exchange record per peer, the plan, a booking, a chase),
/// set in one place and cleared in one place with a reason. Every tick the
/// activity is the first line of the priority list that holds (§8.4). Nothing
/// stores "what I was doing", so an interrupted activity resumes by re-checking.
///
/// PRESENCE IS DIRECT ONLY (Q8). A beacon arrives straight from its sender over
/// one emulator link; relays count for nothing. Plan data is the exception: a
/// robot adopts a higher plan version from any beacon, since a plan is data,
/// not presence (Q54), and the beacon carries what its sender knows every
/// robot holds (§11.1).
///
/// TIME. Every time here is seconds on the shared sim clock (E20): the node
/// passes now() and the beacons' header stamps on that clock.
///
/// The world questions TeamCore cannot answer alone (the allocator's path
/// cost, the planning map, the rendezvous solve, the value gate, the intercept
/// predictor) go through TeamOracle, which the node implements over its map and
/// cell world and the harness implements over its toy world.

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "explo_planner/log_fields.hpp"

namespace explo_planner {
namespace gen34 {

struct Vec2 {
  double x = 0.0, y = 0.0;
};
double dist(const Vec2& a, const Vec2& b);

// ── Arms and activities ─────────────────────────────────────────────────────

enum class Arm : uint8_t { kOff, kPursuit, kRendezvous, kHybrid };
/// "off", "pursuit", "rendezvous", "hybrid". False on anything else.
bool parseArm(const std::string& s, Arm* out);
const char* armName(Arm a);
inline bool armBooks(Arm a) { return a == Arm::kRendezvous || a == Arm::kHybrid; }
inline bool armChases(Arm a) { return a == Arm::kPursuit || a == Arm::kHybrid; }

/// Values match TeamBeacon.msg's ACT_* constants.
enum class Activity : uint8_t {
  kExplore = 0, kMeet = 1, kChase = 2, kFollow = 3, kWait = 4, kHome = 5, kDone = 6,
};
/// Lower-case event name: "explore", "meet", ...
const char* activityName(Activity a);

// ── The meeting plan ────────────────────────────────────────────────────────

struct Plan {
  uint32_t version = 0;       ///< 0 = no plan
  int cell = -1;
  Vec2 center;
  double t0 = 0.0;            ///< slot 0, shared clock
  double interval = 0.0;
  int renewed_at_slot = -1;
  bool provisional = false;
  bool valid() const { return version > 0 && interval > 0.0; }
  double slotTime(int k) const { return t0 + k * interval; }
};

// ── The beacon, as TeamCore sees it ─────────────────────────────────────────

struct Beacon {
  int robot_id = -1;
  uint32_t team_hash = 0;
  uint32_t grid_hash = 0;
  double stamp = 0.0;
  Vec2 position;
  Vec2 goal;
  bool has_goal = false;
  uint32_t hears_mask = 0;
  Activity activity = Activity::kExplore;
  bool finished = false;
  bool team_finished = false;
  bool home = false;
  uint64_t map_sent_seq = 0;
  std::vector<uint64_t> map_rcvd_seq;   ///< by fleet id
  uint32_t exchanged_mask = 0;
  Plan plan;
  int meeting_slot = -1;
  uint32_t met_mask = 0;
  std::vector<int> tour;                ///< my_tour, for the intercept predictor
  /// §11.1: by fleet id, the highest plan the sender knows robot j holds.
  /// Length n, all three, or the table is ignored. Version 0 = unknown.
  std::vector<uint32_t> plan_known_version;
  std::vector<int> plan_known_cell;
  std::vector<Vec2> plan_known_center;
};

// ── Configuration (§8.7) ────────────────────────────────────────────────────

struct TeamConfig {
  Arm arm = Arm::kOff;
  int self_id = 0;
  int n = 1;                             ///< team size
  uint32_t team_hash = 0;
  uint32_t grid_hash = 0;

  double presence_window_sec = 10.0;
  double exchange_stall_sec = 120.0;
  double exchange_total_sec = 600.0;

  double chase_silence_sec = 90.0;
  double chase_cooldown_sec = 90.0;
  double chase_max_contact_age_sec = 900.0;
  double chase_limit_sec = 600.0;
  double chase_departure_margin_sec = 120.0;
  double follow_distance_m = 10.0;
  double follow_min_distance_m = 6.0;

  double meeting_interval_sec = 300.0;
  double meeting_window_sec = 120.0;
  double meeting_patience_sec = 120.0;
  double meeting_backstop_sec = 600.0;
  double meeting_ring_m = 3.0;
  double meeting_arrive_m = 1.0;
  /// A booking waits until the team has been apart this long without a
  /// break: a pair drifting across the radio edge is not a separation.
  double book_apart_sec = 15.0;

  double travel_speed_mps = 0.40;
  double depart_safety = 1.2;
  int finished_missed_slots = 2;
  double leg_arrive_m = 1.5;
  /// Sizes a chase point's time bound: straight-line time at travel speed
  /// times this, plus leg_budget_slack_sec (Q63: nav_safety_factor).
  double leg_budget_factor = 2.0;
  double leg_budget_slack_sec = 30.0;

  double home_tol_m = 1.0;
  double home_max_sec = 600.0;

  // Fixed by the design, not tuning knobs.
  double plan_resolve_period_sec = 5.0;  ///< §8.3 Plan: re-solve while provisional
  double gate_period_sec = 10.0;         ///< value-gate re-evaluation period
  double ring_step_m = 1.0;              ///< Q65: step outward when blocked
  double ring_max_m = 6.0;
  /// §11.3: the Q65 exemption holds only within this of the booking's centre
  /// (ring_max_m + 2).
  double meet_exempt_radius_m = 8.0;
};

// ── The world questions (implemented by the node and by the harness) ───────

struct TeamRobotView {
  int id = -1;
  bool known = false;       ///< position known (self, or heard directly)
  Vec2 position;
  bool finished = false;
};

struct PlanSolve {
  bool ok = false;
  int cell = -1;
  Vec2 center;
  bool provisional = false;
  std::string refused;
};

struct ChaseGateView {
  int id = -1;
  bool known = false;       ///< last heard position is available
  Vec2 last_position;
  double age_sec = 0.0;
  bool finished = false;
  std::vector<int> tour;
};

struct ChaseGateVerdict {
  bool dispatch = false;
  int peer = -1;            ///< the peer the gate priced; -1 if none
  std::string refused;      ///< non-empty: the gate could not price (fails open)
  long long c_no_mm = -1, c_re_mm = -1, leg_mm = -1;
  int unshared_cells = 0;
};

class TeamOracle {
 public:
  virtual ~TeamOracle() = default;
  /// Robot 0's plan solve (RendezvousScheduler::solve over the allocator, or
  /// the team's centroid cell when there is no tour yet, marked provisional).
  virtual PlanSolve solvePlan(const std::vector<TeamRobotView>& team) = 0;
  /// Path length in metres from `from` to the plan cell (the allocator's
  /// cell-to-cell cost), straight line when there is no cell world.
  virtual double pathDistance(const Vec2& from, int cell, const Vec2& center) = 0;
  /// A robot may stand here: the planning map shows it free (true when there
  /// is no map).
  virtual bool standable(const Vec2& p) = 0;
  /// Clear line of sight between the two points on the map. The last 0.5 m
  /// at each end is not tested: a robot standing there is mapped as an
  /// obstacle (§11.3).
  virtual bool lineOfSight(const Vec2& a, const Vec2& b) = 0;
  /// The value gate over the missing peers (evaluateReconnectGate).
  virtual ChaseGateVerdict chaseGate(const std::vector<ChaseGateView>& missing) = 0;
  /// The intercept point for a missing peer (PursuitPredictor::predict).
  /// False when there is none.
  virtual bool intercept(const ChaseGateView& peer, Vec2* out) = 0;
};

// ── Per-tick input and output ───────────────────────────────────────────────

struct TickInputs {
  double now = 0.0;
  Vec2 pose;
  bool have_pose = false;
  /// Exploration finished: the coverage latch, the step budget or PLAN
  /// starvation, latched by the node (K4, §11.7). The only source of the
  /// beacon's `finished`.
  bool finished = false;
  /// The seq inputs are measured: the node has had a fusion-counters sample
  /// that carries them. Until then no exchange starts, because every target
  /// would read 0 and 0 >= 0 would pass as a finished swap.
  bool seq_valid = true;
  /// My own dscovox's newest seq for me (Q51).
  uint64_t sent_seq = 0;
  /// My dscovox's newest seq and gap count per fleet id (size n; [self] ignored).
  std::vector<uint64_t> rcvd_seq;
  std::vector<uint64_t> seq_gaps;
  Vec2 home;
  bool have_home = false;
};

enum class DriveKind : uint8_t {
  kExplore,   ///< the gen-33 exploration machine drives
  kHold,      ///< stand still
  kLeg,       ///< drive to `point` within `tol`
};

struct Drive {
  DriveKind kind = DriveKind::kExplore;
  Vec2 point;
  double tol = 1.0;
  double deadline = std::numeric_limits<double>::infinity();  ///< caller's bound
  /// A new key starts a new leg (watchdog reset); the same key with a moved
  /// point retargets the leg (the follow point moves with the peer).
  uint64_t key = 0;
  std::string purpose;   ///< "meet", "reconnect", "chase", "follow", "home"
};

struct TeamEvent {
  std::string kind;      ///< team_activity, plan, booking, exchange, chase, homing
  LogFields fields;
};

/// What the priority list chose and why, plus the current wait's bound for
/// the tick event (K20).
struct TickOutputs {
  Activity activity = Activity::kExplore;
  Drive drive;
  double wait_start = -1.0;   ///< -1: not in a bounded wait
  double wait_bound = -1.0;   ///< absolute time the wait must end by; -1 none
};

// ── Component state (public so tests can read it) ───────────────────────────

struct PeerRecord {
  bool ever = false;
  double rx_time = -1.0;        ///< local receipt time of the last beacon
  Beacon b;                     ///< that beacon
  bool grid_ok = false;         ///< its grid hash matches ours
  double last_contact = -1.0;   ///< last tick inContact() held
  bool table_ok = false;        ///< its plan_known_* table has the team's length
};

/// §11.1: the highest plan a robot is known to hold. Version 0 = unknown.
struct KnownPlan {
  uint32_t version = 0;
  int cell = -1;
  Vec2 center;
};

struct ExchangeRecord {
  enum class State : uint8_t { kNone, kRunning, kDone, kGaveUp };
  bool active = false;          ///< a contact is running
  State state = State::kNone;
  double start = 0.0;
  double last_progress = 0.0;
  uint64_t target_rx = 0;       ///< the peer's sent seq when contact began
  uint64_t target_tx = 0;       ///< my sent seq when contact began
  uint64_t last_rx = 0;         ///< my received-from-peer
  uint64_t last_peer_rx = 0;    ///< peer's received-from-me
  uint64_t gaps_start = 0;
  bool trivial = false;         ///< nothing to move at the start: both already current
};

struct Booking {
  bool held = false;
  int slot = -1;
  double slot_time = 0.0;
  int cell = -1;
  Vec2 center;
  uint32_t plan_version = 0;
  double booked_at = 0.0;
  bool depart_deferred = false; ///< held back by an encounter in progress
  bool departed = false;
  double depart_time = 0.0;
  bool arrived = false;
  double arrive_time = 0.0;
  Vec2 spot;
  double spot_radius = 0.0;
  double last_progress = 0.0;
  uint32_t heard_mask = 0;      ///< attendees heard directly since departure
  uint32_t contact_mask = 0;    ///< attendees in contact at some point since departure
  uint32_t met_mask = 0;        ///< Q55: exchanged with j during this booking
  std::vector<uint64_t> seq_rx, seq_peer_rx;
  bool full_met = false;
  double full_met_time = 0.0;
  // Q32 / Q53: one mover per lost pair, the higher id.
  bool rc_active = false;       ///< driving to rc_point
  bool rc_holding = false;      ///< made contact on the way; holding there
  int rc_peer = -1;
  Vec2 rc_point;
  bool rc_sight = false;        ///< §11.3: rc_point is a sight point, not the peer
  uint32_t rc_used_mask = 0;    ///< used for the current lost contact
  uint64_t leg_key = 0;
};

struct Chase {
  bool held = false;
  int peer = -1;
  double start = 0.0;
  bool following = false;
  bool contact_seen = false;
  struct Point { Vec2 p; std::string kind; };
  std::vector<Point> points;
  size_t idx = 0;
  double point_deadline = 0.0;
  uint64_t leg_key = 0;
};

// ── TeamCore ────────────────────────────────────────────────────────────────

class TeamCore {
 public:
  TeamCore(const TeamConfig& cfg, TeamOracle* oracle);

  /// A beacon arrived directly from its sender at local time rx_time. Beacons
  /// from another fleet (team hash) or from out-of-range ids are counted and
  /// ignored. Returns false when ignored.
  bool onBeacon(const Beacon& b, double rx_time);

  /// One decision tick. Call at the node's tick rate.
  TickOutputs tick(const TickInputs& in);

  /// This robot's own beacon, as of the last tick.
  Beacon beacon() const;

  /// Events since the last drain.
  std::vector<TeamEvent> drainEvents();

  // ── Queries (valid after a tick) ──
  bool present(int j) const;
  bool inContact(int j) const;
  bool pairInContact(int a, int b) const;
  bool allConnected() const;
  bool exchanged(int j) const;
  bool exchangeGaveUp(int j) const;
  const PeerRecord& peer(int j) const { return peers_[j]; }
  const ExchangeRecord& exchange(int j) const { return ex_[j]; }
  const Plan& plan() const { return plan_; }
  const Plan& previousPlan() const { return prev_plan_; }
  /// §11.1: the highest plan robot j is known to hold ([self] = my plan).
  const KnownPlan& knownPlan(int j) const { return known_[j]; }
  const Booking& booking() const { return booking_; }
  const Chase& chase() const { return chase_; }
  Activity activity() const { return activity_; }
  bool teamFinished() const { return team_finished_; }
  bool homeDone() const { return home_done_; }
  int missedStreak() const { return missed_streak_; }
  const TeamConfig& config() const { return cfg_; }
  /// The cell (and centre) the plan puts slot k at, with split recovery
  /// (§11.1): odd slots go to the lowest known version's cell while it is
  /// below mine.
  void cellForSlot(int k, int* cell, Vec2* center) const;
  /// Every mechanism's firing count, for run_end (rule 3).
  const std::map<std::string, long long>& counts() const { return counts_; }
  /// Every counter TeamCore can raise. counts() holds each from construction,
  /// at 0 until it fires, so run_end reports a mechanism that never fired.
  static const std::vector<std::string>& counterNames();
  /// A peer's last directly heard position, for the allocator and separation
  /// (K2). False if never heard.
  bool lastHeardPosition(int j, Vec2* p, double* age) const;
  /// Proximity exemption (Q65, §11.3): both this robot and peer j are in
  /// Meet near my booking's centre, and I am not walking to reconnect.
  bool proximityExempt(int j) const;

 private:
  void updatePresence();
  void updateExchanges();
  void updateTeamFinished();
  void updatePlan();
  void updateBooking();
  void bookingPass();
  void updateChase();
  Activity selectActivity();
  void updateHoming(Activity act);
  Drive driveFor(Activity act);

  void book();
  void clearBooking(const char* reason);
  void clearChase(const char* reason);
  void proposePlan(const char* why, int renewed_at_slot);
  void adoptPlan(const Plan& p, int from);
  bool pairMet(int a, int b, int slot) const;
  bool fullMet(int slot) const;
  bool allPairsExchanged() const;
  double leadFrom(const Vec2& from, int cell, const Vec2& center);
  Vec2 ringSpot(const Vec2& center, double* radius);
  Vec2 followPoint(const Vec2& peer_pos);
  /// §11.3: where the walker of a lost pair drives. *sight is true for a
  /// point chosen for line of sight, false for the peer's own position.
  Vec2 reconnectPoint(const Vec2& peer_pos, bool* sight);
  void mergeKnown(int j, uint32_t version, int cell, const Vec2& center);
  void setOwnKnown();
  bool anyExchangeRunning() const;
  uint32_t othersMask() const;
  std::vector<TeamRobotView> teamView() const;
  ChaseGateView gateView(int j) const;

  void emit(const std::string& kind, LogFields f);
  void count(const std::string& what) { ++counts_[what]; }

  TeamConfig cfg_;
  TeamOracle* oracle_;
  TickInputs in_;
  double now_ = 0.0;

  std::vector<PeerRecord> peers_;
  std::vector<bool> present_, contact_;
  std::vector<ExchangeRecord> ex_;
  bool team_finished_ = false;
  bool team_was_all_connected_ = false;

  Plan plan_, prev_plan_;
  std::vector<KnownPlan> known_;   ///< §11.1, by fleet id; [self] = plan_
  double last_solve_ = -1e18;
  int renewed_for_slot_ = -1;

  Booking booking_;
  int missed_streak_ = 0;
  long long booking_clears_ = 0;
  double apart_since_ = -1.0;   ///< when the team last stopped being all connected
  int min_slot_ = 0;   ///< slots below this met; book() waits past them

  Chase chase_;
  double last_chase_end_ = -1e18;
  double last_gate_eval_ = -1e18;

  Activity activity_ = Activity::kExplore;
  bool have_activity_ = false;
  double activity_since_ = 0.0;

  bool homing_ = false;
  double homing_start_ = 0.0;
  bool home_done_ = false;
  uint64_t next_leg_key_ = 1;
  uint64_t home_leg_key_ = 0;
  Drive last_drive_;

  std::vector<TeamEvent> events_;
  std::map<std::string, long long> counts_;
};

}  // namespace gen34
}  // namespace explo_planner
