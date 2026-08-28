#pragma once
/// @file experiment_log.hpp
/// @brief Authoritative, machine-readable event log for exploration
///        experiments — one newline-delimited JSON file per robot per run.
///
/// This is the file the paper's numbers come out of. It exists because the
/// per-step CSV (metrics_logger.hpp) plus the human-readable ROS log could not
/// answer the experiment's own questions without post-hoc reconstruction, and
/// every one of those reconstructions was lossy:
///
///  1. NO USABLE TIME AXIS. Events lived only in ROS log lines, which carry a
///     WALL-CLOCK epoch stamp. Recovering sim time meant fitting wall -> sim
///     against the CSV's per-step anchors, and the sim's real-time factor
///     drifts within a run (0.89 -> 0.81 measured), so a single linear fit was
///     off by up to 54 s. Every cross-file join inherited that error. Here the
///     PRIMARY stamp of every event is the node's own simulation clock, and a
///     `run_start` event records the absolute sim t0, so run-relative time is a
///     subtraction rather than a regression. Wall time rides along as a clearly
///     named secondary field (`t_wall_sec`) and is never the join key.
///
///  2. EVENTS ONLY AS PROSE. Reconnect manoeuvres, state transitions and
///     peer-loss episodes were recoverable only by grepping log text that
///     changes whenever someone improves a sentence. Here every event is a
///     typed method on this class: adding a field is a compile-time change, and
///     a renamed log sentence cannot silently break the analysis.
///
///  3. INCOMPARABLE COMPLETION TIMES. Each experiment arm stops on its own
///     criterion, so "run duration" measures a different thing per arm. The
///     `coverage_milestone` event fixes the endpoint instead of the stopping
///     rule: "sim time at which this robot first drove the ROI unknown
///     fraction below X" means exactly the same thing in every arm, and is read
///     directly out of the file instead of interpolated between CSV rows.
///
///     Worse than incomparable, "completion time" was AMBIGUOUS: four
///     quantities were in circulation for it, differing by up to 155 s, and two
///     separate tools each emitted a column called `makespan`. This file
///     defines it exactly once, as `explore_done_sim_sec` (see below), and
///     deliberately does not use the word makespan anywhere.
///
///  4. SILENT WRITE FAILURE. The CSV stream is checked only at construction, so
///     a mid-run failure (ENOSPC, unmounted output dir) discarded every later
///     row while the node kept logging "Step N logged". Here the stream is
///     checked after EVERY write, the first failure raises one ROS ERROR and
///     latches `healthy()` false, and that flag is reported in `run_end`.
///     Truncation is detectable from the file alone: every line carries a
///     monotonic `seq`, and `run_end` carries the total it should end at.
///
///  5. PRECISION LOSS. ostream default formatting is 6 SIGNIFICANT digits, so a
///     sim timestamp of 1234.5678 was written as 1234.57 and an epoch-scale
///     clock would degrade to ~100 s granularity. Every number here is written
///     with explicit FIXED precision (kDecimals = 6 decimal places), which is
///     microseconds on a timestamp of any magnitude a clock can hold.
///
///  6. CRLF. At least one existing CSV in the campaign tree is written with
///     CRLF line endings, which silently breaks naive column parsing. This file
///     is opened in binary mode so the bytes written are the bytes stored: LF
///     only, UTF-8, no BOM.
///
/// THE COMPLETION TIME, defined once. `explore_done_sim_sec` is the ABSOLUTE
/// simulation time at which this robot's own exploration-exhaustion test first
/// fired (coverage saturation, or the step budget) — the instant that produces
/// the first `exploration_complete` event. It is carried on every
/// `exploration_complete` and again in `run_end`, so it is read, never derived.
///
///   It INCLUDES: everything from this planner's t0 (its first tick with a live
///     clock) to that instant — driving, planning, dwelling, proximity holds,
///     and any reconnect manoeuvre that ran before exhaustion.
///   It EXCLUDES: the harness's post-hoc grace period; the harness's polling
///     quantisation; any bring-up offset (a T0 taken after comms/nav come up,
///     which itself varied 27-58 s between runs); everything after the
///     declaration (barrier waits, the return drive, shutdown); and any
///     wall-clock -> sim mapping, since it is measured on the sim clock to
///     begin with.
///   It is NOT: the last line's t_sim_sec, `log_span_sim_sec`, the maximum sim
///     time in the CSV, or anything named makespan. Those measure when the
///     process stopped, which is a property of the harness and the arm's
///     stopping rule, not of the exploration.
///
/// For a cross-arm endpoint that does not depend on any stopping rule at all,
/// prefer the `coverage_milestone` events; `explore_done_sim_sec` is this
/// robot's own declaration, which is what the arms differ in.
///
/// CLOCK ANCHORS. Every line carries the sim stamp AND the wall stamp taken at
/// the same instant, and `clock_anchor` events emit that pair on a steady SIM
/// cadence regardless of what the planner is doing. That makes the file a
/// sim<->wall conversion table for the whole run directory: any other log in it
/// (console, comms emulator, nav) that carries only wall stamps can be placed
/// on the sim timeline by interpolating between two neighbouring anchors,
/// instead of by a single least-squares fit across a real-time factor that
/// drifts within a run.
///
/// The anchor cadence is driven from SIM TIME ONLY. That is a deliberate
/// avoidance of a bug measured in the metrics sampler beside it: a sim-time
/// timer whose callback also gates itself on a WALL deadline armed after its
/// own work suppresses whole ticks at some real-time factors and none at
/// others, so its realised period silently changed from 10 s to 5 s between
/// campaigns while every log line still asserted the configured value. Nothing
/// here consults a wall clock to decide whether to emit; `run_end` additionally
/// reports the metrics sampler's REALISED period alongside its configured one,
/// so that class of drift is measurable from the data rather than assumed away.
///
/// FORMAT. One JSON object per line, flushed after each event, no trailing
/// commas, no nested arrays except where documented. Non-finite values are
/// never emitted as bare `nan`/`inf` tokens (which no strict JSON parser
/// accepts): they are written as `null` and the offending key is listed in the
/// line's `nonfinite_fields` array, so a broken value is loud rather than
/// missing.
///
/// COST. One formatted line into a reusable buffer, one write, one flush per
/// event. No queues, no threads, no growth: the only per-run allocation is the
/// milestone ladder and the run-parameter list, both fixed at startup.

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

namespace explo_planner {

/// The fields that ride on EVERY event line. The node fills this from its own
/// clock and state machine at the call site (see ExploPlannerNode::expCtx()),
/// so no event can be stamped from a different clock than the one the planner
/// makes decisions on.
struct ExperimentContext {
  /// Absolute simulation time, seconds — this->now().seconds() on the node's
  /// clock (sim time under use_sim_time). THE authoritative stamp.
  double sim_time_sec = 0.0;
  /// Planner state at emit time, from stateName(). Borrowed pointer; must
  /// outlive the call (all call sites pass a string literal).
  const char* state = "UNKNOWN";
  /// Exploration step index at emit time.
  int step = 0;
};

/// `step` event payload — mirrors the CSV row deliberately (the CSV stays; the
/// redundancy is what lets either file be validated against the other).
struct StepEvent {
  double unknown_fraction = -1.0;         ///< ROI unknown fraction, -1 = unmeasurable
  const char* coverage_source = "none";   ///< "scovox" / "planning_map" / "none"
  int    observed_voxels  = 0;
  int    frontier_voxels  = 0;
  double distance_m       = 0.0;          ///< cumulative travel, metres
  /// Pose in the map frame (m, m, m, rad). THE ONLY SURVIVING TRAJECTORY: the
  /// CSV has no pose columns and the campaign runs with bagging disabled, so
  /// without these no figure of a rendezvous, a chase or a hold can be drawn
  /// after the fact. z rides along because it costs one field and terrain mode
  /// makes it meaningful.
  double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
  const char* phase = "explore";          ///< "explore" / "exploit"
  int    peers_live       = 0;            ///< peers heard within one claim TTL
  double plan_time_ms     = 0.0;
};

/// `peer_lost` / `peer_seen` payload.
struct PeerEvent {
  std::string peer;                       ///< teammate robot id
  double silent_sec = 0.0;                ///< how long the previous belief had held
  double last_contact_age_sec = -1.0;     ///< age of the last-contact record, -1 = none
  int    peers_live = 0;
  int    expected_peers = 0;
  bool   first_contact = false;           ///< peer_seen only: never heard before
};

/// `reconnect_dispatch` payload — one per manoeuvre DECISION, emitted where the
/// action becomes definite (not where it is contemplated).
struct ReconnectDispatchEvent {
  const char* mode = "rendezvous";        ///< reconnect_mode: rendezvous/pursuit/hybrid
  bool  terminal = true;                  ///< true = exhaustion trigger, false = mid-run
  const char* reason = "";                ///< dispatch cause, e.g. "coverage-saturated"
  std::string peer;                       ///< teammate the manoeuvre is aimed at ("" = none)
  double peer_record_age_sec = -1.0;      ///< age of that peer's last-contact record
  /// chase | anchor_return | meeting_point | hold | resume_exploring
  const char* action = "hold";
  bool   have_dest = false;               ///< false -> destination fields are null
  double dest_x = 0.0, dest_y = 0.0;
  double budget_sec = -1.0;               ///< pursuit budget; -1 where not applicable
  std::string decline_reason;             ///< why a richer action was NOT taken ("" = none)
  int    attempt = 0;                     ///< mid-run attempt index (0 for terminal)
  int    peers_live = 0;
  int    expected_peers = 0;
  /// Info-gate diagnostics (mid-run dispatches only; -1 elsewhere): the
  /// effective trigger threshold the dispatch fired against (the fixed
  /// silence clock when the gate is off), and the estimated unshared-map
  /// backlog at that moment (own exact delta + peer dead-reckoned). Two
  /// sentinels, deliberately distinct: -1 = the gate was off (time-only
  /// trigger, control arm), -2 = the gate was ON but no contact snapshot
  /// existed, so it fell back to the fixed silence clock. Without the
  /// split a gated run that never got a snapshot logs identically to a
  /// control run and the arms cannot be separated offline. Logged so the
  /// estimator can be scored offline against the transfer actually
  /// measured at the merge.
  ///
  /// Two things to know before scoring against it. (1) Both are stamped at
  /// the TRIGGER; every later leaf of the same manoeuvre (hold,
  /// resume_exploring) re-reports them unchanged, so they date the decision,
  /// not the event. (2) est_unshared_vox is NOT overlap-discounted: it counts
  /// each side's gathering in full, while the two robots often re-observe the
  /// same region. Measured on p14, actual transfer runs ~0.38x this estimate
  /// (median), so it is an upper bound on what a merge will deliver.
  double gate_sec = -1.0;
  double est_unshared_vox = -1.0;
  /// How long the RADIO had been down when a mid-run trigger fired, from the
  /// comms emulator's own connected bit; -1 = the link gate was not in play
  /// (feature off, undecodable robot index, stale samples, or a terminal
  /// dispatch), so the fire was decided on peer_record_age_sec exactly as in
  /// every campaign before this one. Logged BESIDE peer_record_age_sec rather
  /// than replacing it: §30.11 is the finding that the two clocks disagree by a
  /// median of 49 s, and collapsing them into one column would destroy the
  /// measurement that motivated the change. A gated run in which the two
  /// columns agree everywhere is evidence the gate is not doing anything.
  double link_down_sec = -1.0;
};

/// `reconnect_end` payload.
struct ReconnectEndEvent {
  /// reconnected | gave_up | abandoned — classified by the node from the team
  /// state and the destination state at the moment the manoeuvre clock stops.
  /// "Gave up" destinations are the states where exploration is over: DONE,
  /// and RETURN_HOME under mission return (a latch that ends a manoeuvre must
  /// bucket the same way whether the robot then parks or drives home).
  const char* outcome = "abandoned";
  const char* to_state = "UNKNOWN";       ///< state the manoeuvre resolved into
  const char* reason = "";                ///< transition reason string
  double duration_sec = 0.0;              ///< SIM seconds the whole manoeuvre ran
  bool   terminal = true;
  int    peers_live = 0;
  int    expected_peers = 0;
};

/// `exploration_complete` payload — this robot declaring its own exploration
/// exhausted, independently of what the arm then does about it.
struct ExplorationCompleteEvent {
  const char* reason = "";                ///< "coverage-saturated" / "step-budget"
  double unknown_fraction = -1.0;
  const char* coverage_source = "none";
  int    steps = 0;
  double distance_m = 0.0;
  bool   team_complete = false;           ///< was the team in comms at that instant
  int    peers_live = 0;
  int    expected_peers = 0;
  int    occurrence = 1;                  ///< 1 = the first declaration of the run
};

/// `mission_complete` payload — the moment the mission-return homing leg
/// resolves, however it resolves. Emitted at most once per run (the homing leg
/// is entered only from a terminal exploration ending and only resolves into
/// DONE), and only in runs with mission_return_enabled — its absence in a
/// mission-return run whose exploration completed is a defect signal. The
/// mission endpoint (primary metric) is this event's stamp for
/// result=="arrived"; every other result is a flagged, bounded failure that
/// still ends the run.
struct MissionCompleteEvent {
  /// arrived | timeout | budget | no-progress — how the homing leg resolved.
  const char* result = "";
  /// The reason the robot went home, i.e. the terminal exploration ending
  /// that dispatched it ("coverage-latched" / "step-budget" / ...). NOT the
  /// resolution — that is `result`.
  const char* reason = "";
  double home_x = 0.0, home_y = 0.0;      ///< the recorded start pose driven to
  double final_x = 0.0, final_y = 0.0;    ///< where the robot actually stopped
  double dist_to_home_m = -1.0;           ///< XY distance between the two rows above
  double homing_duration_sec = -1.0;      ///< SIM seconds spent in RETURN_HOME
  double homing_distance_m = -1.0;        ///< metres travelled while homing
  bool   latched = false;                 ///< had the coverage latch fired (vs step budget)
};

/// `run_end` payload. The logger appends its own health/accounting fields, the
/// completion time and the clock statistics.
struct RunEndEvent {
  const char* reason = "";                ///< "coverage-done" / "step-budget" / ...
  int    steps = 0;
  double distance_m = 0.0;
  double unknown_fraction = -1.0;
  const char* coverage_source = "none";
  int    peers_live = 0;
  int    expected_peers = 0;
  // --- CSV metrics-sampler accounting ---------------------------------
  // The realised sampling period of the per-step CSV's periodic sampler, and
  // the period it was CONFIGURED with. These differed silently between
  // campaigns (10 s realised against 5 s configured, and later 5 s against 5 s)
  // because that sampler gates a sim-time timer on a wall-clock deadline, so
  // whole ticks are suppressed at some real-time factors and none at others.
  // Recorded here so the sampling rate of any archived run is a fact in the
  // data rather than an assumption.
  double metrics_period_param_sec = -1.0;     ///< configured metrics_period_sec
  double metrics_realised_period_sec = -1.0;  ///< measured mean, -1 if < 2 rows
  double metrics_effective_period_sec = -1.0; ///< the sampler's own current belief
  int    metrics_rows = 0;                    ///< periodic rows actually written
  int    metrics_backoffs = 0;                ///< times it stretched its period
  // --- Mission return / final geometry (schema 2) ---------------------
  // Where the run actually ENDED, in every run — not only mission-return
  // ones. The step trajectory samples at a period, so the true final pose is
  // otherwise unrecorded; with these, "did the robot end where it started"
  // is answerable from run_end alone, and in a mission-return run the pair
  // (home, final) is the audit of the homing leg without replaying events.
  bool   have_home = false;                   ///< a home pose was ever recorded
  double home_x = 0.0, home_y = 0.0;          ///< recorded start pose (junk if !have_home)
  double final_x = 0.0, final_y = 0.0;        ///< last known pose at run end
  /// "" (never resolved / feature off) | arrived | timeout | budget |
  /// no-progress — restates mission_complete so single-line readers need not
  /// scan the event stream. The logger writes "" as null.
  std::string mission_home_result;
  double mission_home_sim_sec = -1.0;         ///< sim stamp of the resolution, -1 = none
};

// ==================================================================
// ExperimentLog
// ==================================================================

class ExperimentLog {
 public:
  /// Schema version stamped into `run_start`. Bump on ANY change to field
  /// names or meanings so an analysis script can refuse a file it predates.
  /// v2: mission_complete event; run_end gains have_home/home_x/home_y/
  /// final_x/final_y/mission_home_result/mission_home_sim_sec.
  static constexpr int kSchemaVersion = 2;

  /// Decimal places used for every number written. Fixed, not significant
  /// digits: 6 decimals is microseconds on a sim timestamp and micrometres on a
  /// coordinate, at any magnitude the value can take (see failure 5 above).
  static constexpr int kDecimals = 6;

  /// Opens `path` truncating, in binary mode (LF endings, byte-exact UTF-8).
  ///
  /// Does NOT throw on failure, deliberately differing from MetricsLogger,
  /// which refuses to construct when its CSV cannot be opened. The trade is
  /// asymmetric: the CSV is the planner's only metric sink, so a run without it
  /// is worthless, whereas this log is additive — a bad `experiment_log_path`
  /// must not take down a robot mid-campaign when the CSV is still being
  /// written. The failure is loud instead: one ROS ERROR here, `open()` and
  /// `healthy()` false forever after, and the node repeats it at startup.
  ExperimentLog(const std::string& path, std::string robot_id,
                rclcpp::Logger logger);
  ~ExperimentLog();

  ExperimentLog(const ExperimentLog&) = delete;
  ExperimentLog& operator=(const ExperimentLog&) = delete;

  /// The output stream was opened successfully.
  bool open() const { return open_; }
  /// No write has failed since the file was opened. Latches false on the first
  /// failure and is reported in `run_end`.
  bool healthy() const { return healthy_; }
  /// `run_start` has been emitted, i.e. t0 is latched and events are being
  /// written. False events are counted, not buffered (see startRun()).
  bool started() const { return started_; }
  /// Absolute sim time of `run_start`, seconds. 0 before startRun().
  double t0Sec() const { return t0_sec_; }

  // ----------------------------------------------------------------
  // Run parameters — accumulated before startRun(), flushed into the
  // `run_start` event's "params" object and then released. Named per type
  // rather than overloaded: addParam("x", "str") on an overload set with a
  // bool member silently selects the bool overload, which is exactly the
  // class of bug this file exists to prevent.
  // ----------------------------------------------------------------
  void addParamNum(const std::string& name, double value);
  void addParamBool(const std::string& name, bool value);
  void addParamStr(const std::string& name, const std::string& value);

  /// Emit `run_start`, latching t0 from `ctx.sim_time_sec`.
  ///
  /// MUST be called once the node's clock is live, NOT from the constructor:
  /// under use_sim_time this->now() reads 0 until the first /clock message, and
  /// a t0 of 0 would make every t_rel in the file an absolute sim time in
  /// disguise. The node calls this from its first tick with a non-zero clock.
  ///
  /// `milestones` is the descending unknown-fraction ladder for
  /// coverage_milestone events; it is sorted descending and de-duplicated here.
  void startRun(const ExperimentContext& ctx,
                const std::vector<double>& milestones);

  // ----------------------------------------------------------------
  // Events. Every one is a no-op before startRun() (counted, and reported as
  // events_dropped_before_start in run_end) and after a failed open().
  // ----------------------------------------------------------------
  void logStep(const ExperimentContext& ctx, const StepEvent& e);
  /// `clock_anchor` — the sim/wall pair plus the real-time factor measured
  /// since the previous anchor and averaged since t0. The caller schedules it
  /// on a SIM-TIME cadence and must not gate it on any wall-clock condition
  /// (see the clock-anchor note in the file header); this method itself reads
  /// the wall clock only to record it, never to decide anything.
  void logClockAnchor(const ExperimentContext& ctx);
  /// `dwell_sec` is time spent in `from`; pass -1 when it is not known (the
  /// very first transition, before state_enter_time_ has ever been stamped).
  void logStateChange(const ExperimentContext& ctx, const char* from,
                      const char* to, const char* reason, double dwell_sec);
  void logPeerLost(const ExperimentContext& ctx, const PeerEvent& e);
  void logPeerSeen(const ExperimentContext& ctx, const PeerEvent& e);
  void logReconnectDispatch(const ExperimentContext& ctx,
                            const ReconnectDispatchEvent& e);
  void logReconnectEnd(const ExperimentContext& ctx,
                       const ReconnectEndEvent& e);
  void logExplorationComplete(const ExperimentContext& ctx,
                              const ExplorationCompleteEvent& e);
  void logMissionComplete(const ExperimentContext& ctx,
                          const MissionCompleteEvent& e);
  /// A navigation attempt gave up. `k` is the site's cumulative failure count
  /// after this one (sites cluster at failed_goal_radius_m), `retired` says the
  /// site is now suppressed for the rest of the run rather than on a TTL.
  /// Emitted so "how much time went into unreachable ground, and where" is a
  /// query over the event log rather than a regex over ROS log lines.
  void logNavGoalFailed(const ExperimentContext& ctx, double x, double y,
                        const char* reason, double elapsed_sec, int k,
                        bool retired);
  /// The failed-goal blacklist suppressed EVERY candidate and the planner
  /// re-attempted one anyway — non-retired first, least-recently-failed within
  /// that. Expected to be absent in a healthy run; each occurrence is a
  /// measurement of how close suppression came to starving the planner, and
  /// `retired` says whether only confirmed traps were left to pick from.
  void logGoalAmnesty(const ExperimentContext& ctx, double x, double y,
                      double last_fail_age_sec, bool retired);
  /// A mission-return homing event. ONE event type covers two different things,
  /// so the vocabulary below is a contract the analysis depends on — do not
  /// widen it without updating the readers.
  ///
  ///   kind = "approach" | "frozen"   a detector fired.
  ///     mode        the homing mode it fired IN (direct|retrace|escape),
  ///                 captured before any transition this fire causes.
  ///     response    what was done: resend | retrace | escape | park.
  ///                 (An "approach" fire never reaches mode=escape: that
  ///                 detector is suppressed during an escape leg.)
  ///     window_sec  the detector's window — progress_window_sec for frozen,
  ///                 return_approach_window_sec for approach.
  ///     next_mode   absent.
  ///
  ///   kind = "escape-end"            an escape leg finished. NOT a detector
  ///                                  fire; do not pool these with the rows
  ///                                  above when counting watchdog fires.
  ///     mode        always "escape" — the mode the event is about, same
  ///                 convention as a detector fire.
  ///     response    why the leg ended: escape-arrived | escape-leg-cap |
  ///                 escape-frozen.
  ///     window_sec  the leg's DURATION in seconds, not a detector window.
  ///     next_mode   the mode homing resumed in: retrace, or direct when there
  ///                 was no usable trail.
  ///
  /// escapes_used is the running count against return_escape_max_attempts and
  /// is post-increment on the row that starts a leg (response="escape").
  void logHomeWatchdog(const ExperimentContext& ctx, const char* kind,
                       const char* mode, const char* response,
                       double dist_home_m, double metric_m, double window_sec,
                       int escapes_used, const char* next_mode = nullptr);
  /// Emits `run_end` (once — later calls are ignored) with the logger's own
  /// health and accounting appended. Safe to call from a destructor: it never
  /// throws and never touches ROS beyond the already-constructed logger.
  void logRunEnd(const ExperimentContext& ctx, const RunEndEvent& e);

  /// Feed the coverage measurement taken this sample. Emits one
  /// `coverage_milestone` event for each ladder threshold crossed for the FIRST
  /// time by this value (a single sample can cross several rungs at once, and
  /// then each gets its own event at the same stamp). Latched per threshold:
  /// unknown fraction that rises again never re-fires a rung, so "time to
  /// coverage X" is a single well-defined instant per run.
  ///
  /// Negative / non-finite fractions (the -1 "cannot measure" sentinel) are
  /// ignored rather than treated as full coverage.
  void noteCoverage(const ExperimentContext& ctx, double unknown_fraction,
                    const char* coverage_source, double distance_m,
                    double x, double y);

  /// Number of ladder rungs already reached. Diagnostic / run_end field.
  int milestonesReached() const;

 private:
  // --- line assembly ---------------------------------------------
  void begin(const char* event, const ExperimentContext& ctx);
  void end();                       ///< closes the object, writes, flushes, checks
  void key(const char* name);       ///< emits `,"name":` (or without the comma first)
  void num(const char* name, double v);
  void integer(const char* name, long long v);
  void boolean(const char* name, bool v);
  void text(const char* name, const std::string& v);
  void text(const char* name, const char* v);
  void appendEscaped(const std::string& s);
  void appendNumber(const char* name_for_flag, double v);
  /// Common tail for the reconnect/peer/step payloads that all carry the team
  /// counts, so the two field names cannot drift apart between event types.
  void teamCounts(int peers_live, int expected_peers);

  std::ofstream file_;
  std::string   path_;
  std::string   robot_id_;
  rclcpp::Logger logger_;

  std::string line_;                       ///< reused per event, never shrinks
  bool        first_field_ = true;
  std::vector<const char*> nonfinite_;     ///< keys of this line's null-ed values

  bool   open_    = false;
  bool   healthy_ = false;
  bool   started_ = false;
  bool   run_end_written_ = false;
  bool   write_error_reported_ = false;
  double t0_sec_ = 0.0;
  long long seq_ = 0;                      ///< monotonic event index, from 0
  long long write_failures_ = 0;
  long long dropped_before_start_ = 0;

  /// THE completion time (see the file header): absolute sim seconds of the
  /// LAST exploration_complete, i.e. when this robot stopped trying. A robot
  /// that exhausts, is pulled into a reconnect manoeuvre, resumes on a merged
  /// map and exhausts again did not finish at the first declaration, and
  /// latching the first one understates it — measured at up to +237 s (15%) on
  /// p8trigger_rendezvous_seed1, in 5 of 19 robot-runs in the reconnecting arms
  /// and 0 of 8 in the control. That bias runs one way and only against the
  /// arms under test, so the last declaration is the headline number and the
  /// first is kept beside it rather than instead of it.
  /// Both are < 0 until an exploration_complete is emitted.
  double explore_done_last_sec_  = -1.0;
  double explore_done_first_sec_ = -1.0;

  /// Clock-anchor state: t0's wall stamp and the previous anchor's pair, for
  /// the mean and instantaneous real-time factors.
  double wall0_sec_ = 0.0;
  double prev_anchor_sim_sec_ = 0.0;
  double prev_anchor_wall_sec_ = 0.0;
  long long anchors_ = 0;
  double rtf_last_ = -1.0;

  /// Sim time of the previous coverage sample, so a milestone event can state
  /// the interval its crossing is bracketed by instead of leaving the reader to
  /// assume the configured sampling period (which is exactly the assumption
  /// that went wrong elsewhere).
  double prev_coverage_sample_sec_ = -1.0;

  /// Descending ladder + the per-rung latch. Parallel arrays rather than a
  /// struct so the ladder can be written straight out as a JSON array in
  /// run_start.
  std::vector<double> milestones_;
  std::vector<bool>   milestone_hit_;

  /// Run parameters awaiting run_start. Values are stored PRE-ENCODED as JSON
  /// scalars (numbers/bools bare, strings quoted+escaped) so the flush is a
  /// concatenation and the list can be cleared the moment it is written.
  std::vector<std::pair<std::string, std::string>> params_;
};

}  // namespace explo_planner
