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
///
/// There is deliberately NO last_contact_age_sec here. It was removed after the
/// g6pilot campaign showed it could not mean the same thing in both arms: the
/// last_contact_ record it aged is written only under `reconnect_enabled_`, so
/// all 160 peer events in the off arm carried the "never contacted" sentinel -1
/// including in runs with dozens of real contacts, while in the hybrid arm all
/// 58 populated values equalled `silent_sec` to the last digit (both are stamped
/// from the same intent receipt). So the field was a lie in one arm and a
/// duplicate in the other — the worst combination, because a cross-arm read of
/// "time since last contact" renders the control arm as robots that never met.
/// Populating it in the off arm was rejected as the fix: last_contact_ feeds
/// missingPeerRecord and hence the manoeuvre dispatch, and altering the control
/// arm's state to improve a log field is the wrong trade in a two-arm
/// experiment. Use `silent_sec`.
struct PeerEvent {
  std::string peer;                       ///< teammate robot id
  double silent_sec = 0.0;                ///< how long the previous belief had held
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
  /// THE QUANTITY THE MID-RUN TRIGGER ACTUALLY TESTED: seconds since the team
  /// last read complete, i.e. `now - team_last_complete_time_`. The firing
  /// condition is exactly `team_incomplete_sec >= gate_sec`; -1 on terminal
  /// dispatches, which this trigger did not decide.
  ///
  /// It exists because peer_record_age_sec is NOT that quantity and scoring the
  /// gate against it silently tests a different inequality.
  /// team_last_complete_time_ is stamped on the 1 Hz heartbeat whenever
  /// livePeerCount() reads the team complete, and a peer counts live until its
  /// coordination claim expires — coord_claim_ttl_sec (5.0 s) after its last
  /// beacon. So this runs BEHIND the peer's record age by the TTL, plus up to
  /// one heartbeat period of quantization:
  ///
  ///     team_incomplete_sec ~= peer_record_age_sec - coord_claim_ttl_sec
  ///
  /// Measured on g6pilot + g7r1: 6/6 dispatches, peer_record_age_sec minus this
  /// quantity fell in [3.24, 5.51] s, straddling the TTL as predicted. The
  /// point estimates are [3.74, 5.01]; the interval is widened because THIS
  /// COLUMN DID NOT EXIST in those campaigns — it is new in generation 8, and
  /// the only banked copy of the quantity is the `%.0f`-rounded number in the
  /// gen-7 dispatch line, worth +/-0.5 s per sample. So the relation is
  /// corroborated, not validated at precision; g8r1 is the first data that can
  /// test it properly. The practical consequence is that a nominal 240 s gate
  /// fires at a peer record age of ~245 s, so a gate quoted from
  /// peer_record_age_sec is ~TTL too high. Both columns are kept for the same reason link_down_sec is: the
  /// disagreement between clocks is a measurement, and collapsing them destroys
  /// it.
  double team_incomplete_sec = -1.0;
  /// How long the RADIO had been down when a mid-run trigger fired, from the
  /// comms emulator's own connected bit; -1 = the link gate was not in play
  /// (feature off, undecodable robot index, stale samples, or a terminal
  /// dispatch), so the fire was decided on the silence clock alone exactly as in
  /// every campaign before this one. That clock is team_incomplete_sec, NOT
  /// peer_record_age_sec — see the note there; this comment named the wrong
  /// column through generation 7. Logged BESIDE peer_record_age_sec rather
  /// than replacing it: §30.11 is the finding that the two clocks disagree by a
  /// median of 49 s, and collapsing them into one column would destroy the
  /// measurement that motivated the change. A gated run in which the two
  /// columns agree everywhere is evidence the gate is not doing anything.
  double link_down_sec = -1.0;
  /// P6 interception (§3.7). `predictor` is the arm's setting — "trail" on
  /// every run before this generation and on every control arm — and the rest
  /// describe what the model said on THIS dispatch. They are stamped at the
  /// trigger and, like gate_sec above, re-reported unchanged by every later
  /// leaf of the same manoeuvre.
  ///
  /// Read `predict_refused` first: non-empty means the chase drove the legacy
  /// trail, and the reason separates "the model was never asked" (predictor
  /// off, no tour on record) from "the model answered and lost" (below
  /// min_probability, or an intercept the budget could not reach). Those are
  /// different findings and an mdp arm that silently never predicted would
  /// otherwise be indistinguishable from one that predicted badly.
  ///
  /// The numeric fields stay populated THROUGH a refusal, on purpose: a
  /// rejection at p = 0.09 against a 0.10 floor and one at p = 0.001 say
  /// different things about the model, and only the first suggests the floor is
  /// mis-set. `predict_candidates` says how far the model got and is the field
  /// that makes the numbers readable: 0 means the chain never ran (the arm is
  /// off, or there was no tour) and the zeros beside it are placeholders, 1
  /// means it ran and the argmax was over a single node, and anything higher is
  /// a real comparison.
  const char* predictor = "trail";
  std::string predict_refused;            ///< "" = the intercept was driven
  double predict_p = -1.0;                ///< P(peer in that cell at arrival)
  double predict_p_on_route = -1.0;       ///< 1 - P(off route) at that horizon
  double predict_horizon_sec = -1.0;      ///< record age + my drive to the cell
  double predict_tour_age_sec = -1.0;     ///< local age of the tour it used
  int    predict_cell = -1;               ///< intercept cell id (-1 = none)
  int    predict_candidates = 0;          ///< tour nodes actually scored
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
  /// Rows written by the PERIODIC SAMPLER ONLY. It is not the CSV's row count
  /// and never was: end-of-step rows are written on the LOG_STEP path, which
  /// does not touch this counter, so the file holds
  ///
  ///     csv_rows (excl. header) == metrics_timer_rows + steps
  ///
  /// verified exact in 24/24 robot-runs of g6pilot. Renamed from `metrics_rows`
  /// in generation 8 because that name reads as "rows in the metrics file" and
  /// invites `assert metrics_rows == wc -l`, which fails by exactly the step
  /// count and looks like dropped rows. `steps` is on this same event, so the
  /// identity is checkable without opening the CSV.
  int    metrics_timer_rows = 0;
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

/// `cell_census` payload: one sample of the coarse cell world's status
/// histogram (see cell_world.hpp), plus the planner's own ROI-wide coverage
/// measure taken on the SAME tick from the SAME map.
///
/// The two coverage numbers ride on one event deliberately. P1's gate asks
/// whether the census converges to COVERED as the ROI's unknown fraction
/// falls, and this sim is nondeterministic enough run-to-run that joining two
/// event streams on a timestamp would be comparing two different ticks and
/// calling the difference a disagreement. Here they are one measurement.
struct CellCensusEvent {
  int cells_total = 0;
  // The five local statuses, counted separately. `*_by_others` is a LOCAL-only
  // vocabulary (it never goes on the wire), so these two counts are the only
  // record of how much of this robot's belief is second-hand.
  int unseen = 0;
  int exploring = 0;
  int covered = 0;
  int exploring_by_others = 0;
  int covered_by_others = 0;
  /// Fraction of cells believed finished by anyone. Comparable to
  /// (1 - roi_unknown_fraction) only up to CELL QUANTISATION: a cell counts as
  /// wholly covered or not at all, so with 10 m cells over a 30 m ROI the
  /// census can only ever take ten values and lags the continuous measure by
  /// up to one cell's worth of ground in each direction.
  double covered_fraction = -1.0;
  /// The planner's existing continuous measure and where it came from
  /// ("planning_map" / "scovox" / "none") — the same pair fed to
  /// noteCoverage() on this tick.
  double roi_unknown_fraction = -1.0;
  const char* coverage_source = "none";
  /// Cells whose status changed on the tick that produced this sample.
  int changed = 0;
  /// Sum over cells of update_id: every first-hand status commit this robot
  /// has ever made. A flap detector. If this climbs while the histogram above
  /// sits still, the hysteresis band is too narrow — and each of those commits
  /// reset a known_by mask that the §3.6 knowledge gate reads, so flapping is
  /// not merely cosmetic churn, it silently re-arms reconnection forever.
  long long commits_total = 0;
  // --- geometry, so a cell id in this file can be located --------------
  double cell_size_m = 0.0;
  int nx = 0, ny = 0;
  /// FNV-1a over the quantised grid geometry. Two robots whose grid_hash
  /// differs do not mean the same ground by the same cell id, and no
  /// cross-robot comparison of their cell ids is valid.
  unsigned int grid_hash = 0;
  /// FNV-1a over every cell's WIRE status in id order, per
  /// CellWorld::sharedHash(). The cross-robot convergence readout: two robots
  /// with the same grid_hash and the same shared_hash hold the same shared
  /// belief cell for cell. The counts above cannot say that — identical
  /// totals over different ground read as agreement — which is exactly the
  /// mistake a post-dropout convergence check would make.
  unsigned int shared_hash = 0;
  /// Enabled / possible edges in the cell adjacency graph. A collapse here is
  /// how a plan-map outage shows up before it becomes an allocation failure.
  int edges_enabled = 0;
  int edges_total = 0;
  // --- is the COVERED threshold reachable at all? ----------------------
  //
  // The status histogram above says how many cells were promoted; it cannot
  // say whether promotion was POSSIBLE. Those look identical in the log — an
  // all-EXPLORING census is what you get both from a census that is broken and
  // from a `covered_max_unknown` set below anything this world's map ever
  // reaches. The first run of P1 landed on exactly that ambiguity: zero
  // COVERED cells over an entire run that reached the harness's completion
  // criterion, with nothing in the row to say which of the two it was.
  //
  // These five fields settle it from the row itself. They are order statistics
  // over the MEASURED cells only, so a grid overhanging the ROI does not drag
  // the distribution toward 1.0 with cells nobody could have seen, and the two
  // promotion vetoes are reported separately because they fail for unrelated
  // reasons and a single "not promoted" count cannot be acted on.
  /// Cells with observed_columns >= min_observed_columns. Denominator for the
  /// four below; the rest of the grid has no evidence either way.
  int cells_measured = 0;
  /// Measured cells clearing covered_max_frontier_frac. Compare against
  /// cell_unknown_p10: if this is high while the unknown fractions sit far
  /// above covered_max_unknown, the column measure is the binding veto, and
  /// vice versa.
  int cells_frontier_ok = 0;
  /// Unknown-column fraction over the measured cells: the best cell, the 10th
  /// percentile, and the median. -1 when nothing was measured.
  ///
  /// cell_unknown_min is the one that answers the reachability question
  /// directly — no cell can ever be promoted if the best-observed cell on the
  /// map is still above covered_max_unknown.
  double cell_unknown_min = -1.0;
  double cell_unknown_p10 = -1.0;
  double cell_unknown_median = -1.0;
  /// Frontier voxels as a fraction of the cell's observed voxels: the best
  /// cell and the median, over the measured cells.
  ///
  /// A fraction and not the raw count, because the raw count is what made the
  /// veto unusable in the first place. The knob used to be an absolute number
  /// of voxels, so its meaning depended on the cell size and the map
  /// resolution — at 10 m cells and 0.1 m voxels a thoroughly swept cell holds
  /// tens of thousands of voxels and hundreds of boundary ones, and the
  /// small-looking default vetoed everything forever. The fraction is the same
  /// quantity with the scale divided out, so a threshold on it means the same
  /// thing at any cell size; see covered_max_frontier_frac.
  double cell_frontier_frac_min = -1.0;
  double cell_frontier_frac_median = -1.0;
  /// The frontier fraction of the cell with the LOWEST unknown fraction —
  /// i.e. of the single best promotion candidate on the map.
  ///
  /// The two medians above cannot answer whether promotion is possible,
  /// because the best-unknown cell and the best-frontier cell need not be the
  /// same cell, and a threshold pair chosen from two independent marginals can
  /// be satisfiable by nothing. This field is the joint question asked
  /// directly: if covered_max_unknown were set just above cell_unknown_min,
  /// would the frontier veto still reject that very cell?
  double cell_frontier_frac_at_best_unknown = -1.0;
};

/// `team_exchange` payload: one received TeamWorld, from the moment it was
/// drained rather than the moment it arrived. Emitted per MESSAGE MERGED, not
/// per message received — see `coalesced`.
///
/// It carries three things that a reader would otherwise have to join three
/// streams to assemble, and the join would be wrong because the sim is
/// nondeterministic enough that two events a tick apart are two different
/// worlds: what the merge did, what the comms model concluded, and where the
/// local census stood immediately afterwards. The last is what makes the P2
/// convergence check readable from one robot's file: after a dropout heals,
/// `covered_by_others` climbing toward the peer's own COVERED count IS the
/// convergence, and reading it off the same line as the merge that caused it
/// needs no timestamp alignment between two robots' logs.
struct TeamExchangeEvent {
  /// Sender's name from `team_robot_names`, and its numeric id. The name is
  /// carried because every other peer-keyed event in this file is keyed by
  /// name, and a file where half the peer references are ids and half are
  /// names cannot be grouped without a lookup table nobody has.
  std::string peer;
  int peer_id = -1;

  /// "" when the message was merged. Otherwise why it was not, verbatim from
  /// whichever check refused it — a config-mismatch drop and a merge that
  /// found nothing to do are opposite diagnoses and must never render alike.
  std::string drop_reason;

  /// Messages from this sender superseded before this one was drained. Full
  /// state means the newest message contains everything the older ones did, so
  /// coalescing loses no information — but it does lose the RECORD, and a run
  /// where this is persistently non-zero is a run whose planning tick is
  /// slower than `team_world_hz`, which is worth seeing rather than inferring.
  int coalesced = 0;
  /// Seconds between local receipt and this merge: the drain latency the
  /// callback-copies-only design trades for. Bounded by the tick period in a
  /// healthy run; large values mean the executor was starved.
  double queue_age_sec = -1.0;

  // --- what the merge did (CellWorld::MergeStats) ---------------------
  int cells_in_msg = 0;
  int applied = 0;
  int known_by_only = 0;
  int agreed_noop = 0;
  int refused_guard = 0;
  int refused_local = 0;
  int out_of_range = 0;
  int bad_status = 0;

  // --- what the comms model concluded, AFTER this message ---------------
  /// The dispatch-decision answer. Never a statement about data freshness:
  /// with closure on, this can be true for a peer whose every byte is minutes
  /// old, which is what the two age fields below are for.
  bool in_comms = false;
  /// The handshake completed both ways.
  bool direct = false;
  /// Received inside the TTL, but the peer's mask did not name us back — the
  /// one-way contact of doc/limitations.md §10, logged as its own condition so
  /// it can be counted rather than deduced from `in_comms == false`.
  bool one_way = false;
  /// IN_COMMS only because someone else is bridging.
  bool via_relay = false;
  double last_direct_age_sec = -1.0;
  double last_known_age_sec = -1.0;
  /// Peers (excluding self) the model currently calls LOST_COMMS.
  int peers_lost = 0;
  /// The sender's published direct-contact mask, and ours. Written as decimal
  /// integers; bit k is robot k.
  unsigned int peer_in_range_mask = 0;
  unsigned int self_direct_mask = 0;

  // --- where the local census stood immediately after the merge ---------
  double covered_fraction = -1.0;
  int covered = 0;
  int covered_by_others = 0;
  int exploring = 0;
  int exploring_by_others = 0;
};

/// `allocation` payload (P3): one global-allocator solve, emitted from the
/// planning tick that used it.
///
/// The P3 smoke gate is MECHANISM-level, not outcome-level — the redundancy
/// metric it would otherwise want needs thousands of cells per arm and cannot
/// resolve a single A/B pair. So this event has to carry enough to answer
/// "did the allocator do its job" from the logs alone, which is three separate
/// questions and three separate groups of fields below.
struct AllocationEvent {
  // --- 1. did both robots solve the SAME problem? -----------------------
  //
  // Solve-same-take-own is an arithmetic claim, and it is only meaningful
  // over a converged world: two robots holding different beliefs SHOULD
  // allocate differently, and counting that as a disagreement would report
  // the comms model's dropouts as an allocator fault. `shared_hash` is the
  // join key that separates the two — restrict to cycles where both robots
  // logged the same value and the remaining disagreements are the allocator's.
  unsigned int shared_hash = 0;
  unsigned int grid_hash = 0;
  /// Vehicles that survived into the problem, and how many candidate cells it
  /// had. Both are needed to read an empty tour: no vehicles, no candidates
  /// and "the mask took them all" are three different empty tours.
  int robots_in_problem = 0;
  int candidates = 0;
  int unassigned = 0;
  /// Empty when solved. Non-empty is a wholesale refusal with its reason —
  /// distinct from "solved, nothing to do", which is an empty tour and no
  /// reason. See GlobalAllocator::Allocation::refused.
  std::string refused;
  double solve_ms = -1.0;

  // --- 2. did the two robots agree about who goes where? ----------------
  //
  // Each robot solves for the WHOLE fleet, so this robot's answer contains
  // its BELIEF about the peer's focus cell as well as its own. Logging both
  // makes the agreement check readable without aligning two files by
  // timestamp: robot A's `peer_focus` against robot B's `focus_cell` is the
  // same comparison, and it is exact rather than tolerance-windowed.
  int focus_cell = -1;
  /// This robot's whole tour, cell ids in visit order, comma-separated. Text
  /// because it is variable-length and a reader that wants only the head has
  /// `focus_cell` already.
  std::string tour;
  /// "id:cell" per OTHER vehicle, comma-separated; a vehicle with no tour is
  /// written as "id:-1" rather than omitted, so an absent peer and a peer
  /// allocated nothing do not render alike.
  std::string peer_focus;
  /// True while every vehicle in the problem is in comms. The gate's coincidence
  /// fraction is only defined over these cycles — out of comms the two robots
  /// are not expected to agree and a coincidence there means nothing.
  bool all_in_comms = false;

  // --- 3. did the allocation actually steer the planner? ----------------
  //
  // The field that stops this gate from becoming another one that cannot
  // fail. An allocator that solves beautifully and never changes which goal
  // is chosen is indistinguishable in every field above from one that works.
  /// Tour rank of the candidate finally selected: 0 = the focus cell's
  /// neighbourhood, 1 = the next tour cell's, and so on; -1 when no goal was
  /// selected this tick, and `kAllocRankUnrestricted` when the goal came from
  /// outside every tour cell (the documented fall-back).
  int picked_rank = -1;
  /// True when the ranking actually reordered the candidate list — i.e. at
  /// least one frontier candidate fell outside the focus neighbourhood. False
  /// means the filter was a no-op this tick and `picked_rank` proves nothing.
  bool reordered = false;
  /// Consecutive ticks the current focus cell has produced no admissible
  /// candidate, and the cells the staleness rule demoted to COVERED on this
  /// tick (comma-separated ids; empty for none).
  int focus_skips = 0;
  std::string demoted;
};

/// `AllocationEvent::picked_rank` for a goal that came from outside every
/// tour cell's neighbourhood. A large sentinel rather than -1 so it sorts
/// after every real rank and cannot be confused with "no goal".
inline constexpr int kAllocRankUnrestricted = 9999;

/// `reconnect_gate` payload: one utility-gate decision (§3.6) and the whole
/// arithmetic behind it.
///
/// Emitted on EVERY evaluation, not only on the ones that dispatch. That is
/// the point of the event: a gate is judged by what it SUPPRESSED, and a log
/// that records only the fires is indistinguishable between "the gate is
/// working" and "the gate is stuck closed". Cross-check the count of
/// `dispatched=false` lines against the arm's `reconnect_dispatch` count.
///
/// The evaluation sits behind the silence clock, the link veto and the attempt
/// budget, so these lines appear only during a confirmed outage and are rare
/// (order tens per robot-run), not per-tick.
struct ReconnectGateEvent {
  /// The verdict actually acted on. False = exploration continued.
  bool dispatched = false;

  /// The knowledge half. `false` with an empty `refused` is the clean
  /// suppression: every missing peer already holds the current status of every
  /// cell we have an opinion about.
  bool knowledge      = false;
  int  unshared_cells = 0;

  /// The value half, in the allocator's quantised unit (mm). All -1 when the
  /// knowledge gate declined first, which is how a knowledge suppression is
  /// told apart from a value suppression that happened to agree.
  long long c_no_mm = -1;
  long long c_re_mm = -1;
  long long leg_mm  = -1;

  /// Candidate cells no vehicle could take under the no-comms mask. Expected
  /// 0; a nonzero value means C_no is understating the cost of staying apart.
  int unassigned = 0;

  /// Non-empty when the gate could not evaluate and therefore FAILED OPEN
  /// (`dispatched` is then true). Counting these separates "the gate let it
  /// through" from "the gate never ran".
  std::string refused;

  /// What the silence clock had already decided when the gate was consulted —
  /// the inequality this gate is layered on top of, so a suppression can be
  /// read against the dispatch that would otherwise have happened.
  double team_incomplete_sec = -1.0;
  double gate_sec            = -1.0;
  int    attempts_used       = 0;

  /// The peers the gate was asked about: "id:cell" comma-separated, cell -1
  /// for a peer we could not locate.
  std::string peers;
};

/// `rendezvous_agreed` payload: one appointment, and everything needed to
/// check offline that the peer derived the same one (§3.5).
///
/// The name is inherited from the v2 design, where agreement was a PROTOCOL
/// and this event marked the moment it converged. Under v4 there is no
/// protocol: agreement is a consequence of both robots running identical
/// arithmetic over a converged world, so this event records a DERIVATION, and
/// the fields exist to make the claim falsifiable rather than to narrate a
/// handshake. `shared_hash` is the join key — restrict to outages where both
/// robots logged the same value and any (cell, t_meet) disagreement that
/// remains is the scheduler's, not the comms model's.
///
/// Emitted on every arming attempt including the refusals, for the same reason
/// `reconnect_gate` logs its suppressions: an appointment that was never armed
/// and one that was armed and ignored are the same silence in the log
/// otherwise.
struct RendezvousAgreedEvent {
  /// The world both sides are claimed to have solved over, and the grid it
  /// sits on. From the FROZEN snapshot, not the live world — the frozen one is
  /// what the arithmetic ran on, and logging the live hash would make a
  /// disagreement look like a scheduler fault when it was a staleness one.
  unsigned int shared_hash = 0;
  unsigned int grid_hash   = 0;
  /// Mission-elapsed seconds at which the snapshot was frozen. The gap between
  /// this and `t_now_sec` is how long the two worlds have had to diverge, which
  /// is the only quantity that can explain a legitimate disagreement.
  double snapshot_age_sec = -1.0;

  /// The appointment. cell -1 with a non-empty `refused` is the refusal shape.
  int    cell        = -1;
  double t_meet_sec  = -1.0;
  double t_now_sec   = -1.0;
  /// Uncapped max-over-robots arrival, in seconds. Differs from
  /// (t_meet_sec - t_now_sec) exactly when `capped` is true, and the difference
  /// IS how late the slowest robot will be.
  double interval_sec = -1.0;
  bool   capped       = false;

  /// The objective's value at the winner, in the allocator's quantised unit.
  /// 0 means the meeting cost the team nothing — somebody was already driving
  /// there. A large value against a small candidate count is the signature of
  /// a scheduler with nothing good to choose from.
  long long penalty_mm = -1;
  /// True when the last-contact midpoint won. Read together with `candidates`:
  /// the floor winning against ten candidates is a verdict, the floor winning
  /// against one is the absence of one.
  bool floor_won = false;
  int  candidates = 0;
  int  rejected_unreachable = 0;
  int  rejected_excluded    = 0;

  /// This robot's own travel estimate to the winner and the departure deadline
  /// derived from it, both in mission-elapsed seconds. Deliberately per-robot
  /// and NOT expected to match the peer's: staggered departures with coincident
  /// arrivals is the design, so two equal deadlines in a pair would be evidence
  /// the mechanism is not doing what it claims.
  double travel_sec  = -1.0;
  double depart_sec  = -1.0;

  /// Non-empty when no appointment could be derived. Distinguishes "no meeting
  /// was scheduled" from "a meeting was scheduled and nothing came of it".
  std::string refused;
  /// Cells already written off by a no-show, comma-separated. Grows within one
  /// outage; empty on the first attempt.
  std::string excluded;
};

/// `rendezvous_outcome` payload: how an armed appointment actually ended.
///
/// One per `rendezvous_agreed` that armed, so the two join 1:1 within an
/// outage and the pair answers the only question that matters about the
/// mechanism: of the meetings it scheduled, how many produced contact. Without
/// this event a scheduler that arms perfectly and never reconnects anything
/// looks identical in the log to one that works.
struct RendezvousOutcomeEvent {
  int    cell       = -1;
  double t_meet_sec = -1.0;
  /// When this robot actually got there (or gave up), mission-elapsed.
  double t_end_sec  = -1.0;
  /// Signed: negative is early, positive is late. The quantity the departure
  /// rule exists to keep near zero, and the one mTARE's depart-at-zero rule
  /// cannot control because its lateness differs per robot.
  double lateness_sec = -1.0;

  /// How it ended. One of:
  ///   "reconnected"   contact restored (the success case)
  ///   "no-show"       arrived, waited out the cap, nobody came
  ///   "unreachable"   the drive to the cell failed or timed out
  ///   "superseded"    abandoned because the manoeuvre ended another way
  std::string outcome;
  /// True when the robot reached the cell at all, whatever the outcome. A
  /// no-show with `arrived` false is a navigation failure wearing a
  /// coordination failure's name.
  bool arrived = false;
  /// Seconds spent waiting at the cell.
  double waited_sec = -1.0;
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
  /// v3: generation 8. Two of the changes are field-INCOMPATIBLE, not additive,
  /// which is the whole reason this constant exists:
  ///   - `PeerEvent::last_contact_age_sec` REMOVED (see the note at the peer
  ///     event above). A v2-era reader that keys on it gets a KeyError, not a
  ///     wrong number, so at least it fails loudly.
  ///   - `RunEndEvent::metrics_rows` RENAMED to `metrics_timer_rows`. This one
  ///     is the dangerous shape: a lenient reader with a `.get("metrics_rows",
  ///     0)` default reads 0 forever and silently reports every run as having
  ///     written no metrics rows.
  /// Additive in the same generation: home_watchdog gains test_delta_m /
  /// test_threshold_m (present only on detector fires, absent on escape-end),
  /// and its VOCABULARY widens — a new kind `frozen-in-escape` and a new
  /// response `abort-leg`. Vocabulary widening is a schema change even though
  /// no field moved: a reader with an exhaustive match on kind now hits an
  /// unhandled case, and one that counts fires by kind silently undercounts.
  /// The gen-7 cells under /home/kalhan/hmr_campaign_void_gen7 are void for
  /// exactly these two incompatibilities; leaving the stamp at 2 would have
  /// made a void file and a live file indistinguishable to a script.
  /// v4: the M-TARE evolution (docs/mtare_evolution_plan.md). Nothing moved and
  /// nothing was removed — every v3 field means exactly what it meant — but the
  /// event VOCABULARY widens by six kinds:
  ///
  ///     cell_census         the coarse cell world's status histogram
  ///     team_exchange       one received TeamWorld: merge result + comms picture
  ///     allocation          a global-allocator solve: tour, focus cell, cost
  ///     rendezvous_agreed   a scheduled (cell, time) reached agreement
  ///     rendezvous_outcome  how a scheduled meeting actually ended
  ///     reconnect_gate      an info-gated dispatch decision and its arithmetic
  ///
  /// `team_exchange` was added by P2 to a v4 that P0 had declared with five
  /// kinds. It is an AMENDMENT to v4 rather than a bump to v5, and the reason
  /// is what the stamp is for: it protects readers of files already written,
  /// and there are none — v4 exists only on this unmerged branch, no campaign
  /// has run against it, and the only v4 artifacts in existence are the phase
  /// equivalence pairs, which are unaffected because the kind cannot appear at
  /// defaults. Once a v4 file exists outside this branch this reasoning
  /// expires and the next widening must bump.
  ///
  /// P6 amended v4 a second time, but only with FIELDS: eight `predict_*`
  /// columns on `reconnect_dispatch` (see ReconnectDispatchEvent). That is a
  /// weaker change than team_exchange's and does not reach the stamp at all —
  /// the vocabulary is unchanged, no field moved, and a JSON-lines reader that
  /// does not know the keys ignores them. It is recorded here anyway so the
  /// list of what v4 accumulated is complete in one place, rather than being
  /// something a reader has to reconstruct by diffing structs.
  ///
  /// Vocabulary widening IS a schema change, on the precedent set by v3's
  /// home_watchdog widening: a reader with an exhaustive match on `event` now
  /// hits an unhandled case, and one that counts by kind silently undercounts.
  /// Bumping the stamp is what lets such a reader refuse rather than guess.
  ///
  /// All five are emitted only when a mechanism is switched ON. At the shipped
  /// defaults a v4 run emits exactly the v3 set — which is not a nicety but the
  /// pre-registered per-phase equivalence gate (see sim/equiv_gate.py): "no new
  /// kinds may appear at defaults" is one of its five run-invariant checks, and
  /// kEventKinds below is the declared universe it scores against.
  static constexpr int kSchemaVersion = 4;

  /// Every `event` value this writer can emit, and the ONLY authority on that
  /// set. It exists because the equivalence gate has to answer "did a new kind
  /// appear?", and the only alternatives were a hand-maintained list in a
  /// Python file (which drifts from the binary the moment someone adds an
  /// event and forgets) or inferring the universe from one run's output (which
  /// cannot distinguish "this kind is new" from "this run did not reach it").
  ///
  /// Ordered as: envelope, then v1-v3 kinds, then the v4 additions. Keep the
  /// v4 block last and append to it — sim/equiv_gate.py reports the tail as the
  /// opt-in set, and a v4 kind hidden in the middle would be scored as legacy.
  static constexpr const char* kEventKinds[] = {
      "run_start", "run_end", "step", "clock_anchor", "state_change",
      "peer_lost", "peer_seen", "reconnect_dispatch", "reconnect_end",
      "exploration_complete", "mission_complete", "nav_goal_failed",
      "pose_health", "goal_amnesty", "home_watchdog", "coverage_milestone",
      // --- v4, all default-off ---
      "cell_census", "allocation", "rendezvous_agreed", "rendezvous_outcome",
      "reconnect_gate", "team_exchange",
  };
  static constexpr size_t kEventKindCount =
      sizeof(kEventKinds) / sizeof(kEventKinds[0]);
  /// Index of the first v4 kind in kEventKinds — the boundary between "a run at
  /// defaults may emit this" and "only an opted-in arm may".
  static constexpr size_t kFirstV4EventKind = 16;

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
  /// `elapsed_sec` is time since NAVIGATE entry — descriptive context on every
  /// row, and NOT in general the quantity that failed. `budget_sec` is the
  /// drive budget (nav_budget_sec), likewise context.
  ///
  /// The comparison that actually fired is carried separately, in
  /// (`test_name`, `test_value`, `test_threshold`), because the three failure
  /// reasons do not test the same thing and two of them do not test
  /// `elapsed_sec` against `budget_sec` at all:
  ///   budget-rotate -> rotate_elapsed_sec vs goal_rotate_timeout_sec
  ///   budget        -> nav_elapsed_sec    vs nav_budget_sec
  ///   no-progress   -> window_progress_m  vs progress_min_distance_m
  /// This split exists because the earlier schema reported only the first pair
  /// and 30 of 31 rows in the g6pilot campaign were budget-rotate: each read
  /// "failed at 43-58 s of a 180 s budget", i.e. ground the robot could not
  /// cross, when the truth was "blew a 15 s rotation timeout". The record was
  /// wrong in the direction that fabricates the more interesting conclusion.
  /// `test_name` names the units, so no consumer has to infer them from
  /// `reason`.
  ///
  /// `pose_stale` is true when TF had gone stale at the moment of failure,
  /// which makes the no-progress verdict a statement about the pose feed
  /// rather than about the robot.
  void logNavGoalFailed(const ExperimentContext& ctx, double x, double y,
                        const char* reason, double elapsed_sec, int k,
                        bool retired, double budget_sec, bool pose_stale,
                        const char* test_name, double test_value,
                        double test_threshold);
  /// The TF pose went stale (`lost=true`) or came back (`lost=false`).
  /// Exists because every downstream symptom of a dead pose feed — frozen
  /// progress metric, no-progress goal failures, the homing watchdog ladder
  /// walking to a park — is recorded in this log as a *physical* stall, and
  /// nothing in the JSONL distinguished "the robot stopped moving" from "the
  /// robot stopped being observed". `age_sec` is the transform's age at the
  /// moment of the transition (0 on recovery).
  void logPoseHealth(const ExperimentContext& ctx, bool lost, double age_sec);
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
  ///   kind = "approach" | "frozen" | "frozen-in-escape"   a detector fired.
  ///     mode        the homing mode it fired IN (direct|retrace|escape),
  ///                 captured before any transition this fire causes.
  ///     response    what was done: resend | retrace | escape | park, plus
  ///                 abort-leg, which only ever appears on frozen-in-escape.
  ///                 (An "approach" fire never reaches mode=escape: that
  ///                 detector is suppressed during an escape leg.)
  ///     window_sec  the detector's window — progress_window_sec for frozen
  ///                 and frozen-in-escape, return_approach_window_sec for
  ///                 approach. Never a leg duration on these kinds; that
  ///                 convention belongs to escape-end alone, and a
  ///                 frozen-in-escape abort is always followed by an
  ///                 escape-end row carrying the leg's duration.
  ///     test_delta_m / test_threshold_m
  ///                 THE FIRED INEQUALITY: the detector fired because
  ///                 test_delta_m < test_threshold_m. delta is window movement
  ///                 for frozen and closing distance for approach; the
  ///                 threshold is the matching param. Present on detector-fire
  ///                 rows only.
  ///     next_mode   absent.
  ///
  ///   kind = "escape-end"            an escape leg finished. NOT a detector
  ///                                  fire; do not pool these with the rows
  ///                                  above when counting watchdog fires.
  ///     mode        always "escape" — the mode the event is about, same
  ///                 convention as a detector fire.
  ///     response    why the leg ended: escape-arrived | escape-leg-cap |
  ///                 escape-frozen. NOTE that escape-frozen is a `response`
  ///                 here and nothing else: the fire that caused this ending
  ///                 is the SEPARATE preceding row, kind=frozen-in-escape.
  ///                 The two rows are one abort. Key on the column, not on
  ///                 the token, or you will count it twice.
  ///     window_sec  the leg's DURATION in seconds, not a detector window.
  ///     next_mode   the mode homing resumed in: retrace, or direct when there
  ///                 was no usable trail.
  ///
  /// escapes_used is the running count against return_escape_max_attempts and
  /// is post-increment on the row that starts a leg (response="escape").
  ///
  /// dist_home_m and metric_m are BOTH instantaneous samples at the fire
  /// instant, and neither is the tested quantity — that is test_delta_m, added
  /// in generation 8. Through generation 7 the fire rows carried no tested
  /// value at all, and metric_m was documented as "the metric over
  /// window_sec", which reads exactly as if it were one. It is not.
  ///
  /// The banked evidence, stated exactly: g6pilot has 7 approach fires, of
  /// which 3 log the tested delta and 4 do not. The delta survives solely in
  /// the planner's "metric moved X m in 40 s" text, which only the DIRECT-mode
  /// fires print; the 4 retrace-mode fires print "fired in retrace mode"
  /// instead and carry no delta at all. On the 3 that log it:
  ///
  ///   metric_m 3.660852, delta  0.89 m  ->   4.11x   (2-dp bound 4.09-4.14)
  ///   metric_m 3.691460, delta -0.03 m  -> 123.05x   (2-dp bound 105.5-147.7)
  ///   metric_m 38.879211, delta 0.24 m  -> 161.997x  (2-dp bound 158.7-165.4)
  ///
  /// so "4.1x to 162x" is the correct min and max, and on the middle one the
  /// robot was RECEDING while the row read 3.69. The deltas are printed to 2 dp
  /// and two of them are near zero, hence the bounds; only the 4.11x is tight.
  ///
  /// This comment has now been wrong in BOTH directions, which is worth leaving
  /// on the record. It first claimed "4x to 162x over all 7 fires" — right
  /// range, wrong n, since 4 of the 7 log nothing to compare against. It was
  /// then "corrected" to "4.1x and ~123x over 2 fires", which dropped the third
  /// fire entirely: 6 of the 7 sit in one robot-run (g6pilot_hybrid_seed103/
  /// bestla) and the 7th is the lone off-arm fire in g6pilot_off_seed102/atlas
  /// — the largest metric_m in the bank, and the one that produces the 162x.
  /// Measuring only the hybrid cells loses it silently. The lesson is narrower
  /// than "check your arithmetic": a population defined by where the events are
  /// dense is not the population, and the arm you are not studying still has
  /// rows in it.
  ///
  /// A reader scoring the detector on the old contract would have called every
  /// fire spurious. The two are kept side by side rather than collapsed
  /// because "how far is left" and "how far it moved" are different questions
  /// and the diagnosis needs both.
  void logHomeWatchdog(const ExperimentContext& ctx, const char* kind,
                       const char* mode, const char* response,
                       double dist_home_m, double metric_m, double window_sec,
                       int escapes_used, double test_delta_m = 0.0,
                       double test_threshold_m = 0.0,
                       const char* next_mode = nullptr);
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

  /// Emits one `cell_census` (schema v4). Unconditional — unlike
  /// noteCoverage() there is no ladder and no latch, because the question this
  /// answers is about the TRAJECTORY of the histogram, not about first
  /// crossings, and a latched sampler cannot show a status going backwards.
  /// The caller decides the rate.
  void logCellCensus(const ExperimentContext& ctx, const CellCensusEvent& e);
  void logTeamExchange(const ExperimentContext& ctx,
                       const TeamExchangeEvent& e);
  /// Emits one `allocation` (schema v4). One per planning tick that ran the
  /// global allocator; nothing is emitted when it is disabled, which is what
  /// makes "no allocation events" a valid allocator-off control rather than an
  /// ambiguous absence.
  void logAllocation(const ExperimentContext& ctx, const AllocationEvent& e);

  /// Emits one `reconnect_gate` (schema v4). One per evaluation, fired or not.
  void logReconnectGate(const ExperimentContext& ctx,
                        const ReconnectGateEvent& e);

  /// Emits one `rendezvous_agreed` (schema v4). One per arming attempt,
  /// including the refusals — see the struct for why the refusals matter.
  void logRendezvousAgreed(const ExperimentContext& ctx,
                           const RendezvousAgreedEvent& e);

  /// Emits one `rendezvous_outcome` (schema v4). One per appointment that
  /// armed, so the two kinds join 1:1 within an outage.
  void logRendezvousOutcome(const ExperimentContext& ctx,
                            const RendezvousOutcomeEvent& e);

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
