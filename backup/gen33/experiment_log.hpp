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
/// Moved comments: doc/experiment_log_notes.md

#include <cstdint>
#include <fstream>
#include <string>
#include <utility>  // std::pair, used by params_
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
  /// Pose in the map frame (m, m, m, rad). The only trajectory record: the CSV
  /// has no pose columns. (notes: explog-step-pose)
  double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
  const char* phase = "explore";          ///< "explore" / "exploit"
  /// accountedPeerCount: peers heard within one TTL on the claim table or
  /// TeamWorld, plus peers that announced finished (no TTL). Every event's
  /// peers_live uses this one definition. (notes: explog-peers-live-definition)
  int    peers_live       = 0;
  double plan_time_ms     = 0.0;
};

/// peer_lost / peer_seen payload. Deliberately no last_contact_age_sec:
/// last_contact_ is written only under reconnect_enabled_, so it cannot mean
/// the same in both arms. Use silent_sec.
/// (notes: explog-peer-event-no-contact-age)
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
  /// chase | appointment | anchor_return | hold | resume_exploring
  ///
  /// The node no longer writes meeting_point; readers still accept it. Change
  /// this vocabulary together with sim/reconnect_value.py (ACTED) and
  /// sim/manoeuvre_events.py (DEST), or manoeuvres misclassify.
  /// (notes: explog-dispatch-action-vocabulary)
  const char* action = "hold";
  bool   have_dest = false;               ///< false -> destination fields are null
  double dest_x = 0.0, dest_y = 0.0;
  double budget_sec = -1.0;               ///< pursuit budget; -1 where not applicable
  std::string decline_reason;             ///< why a richer action was NOT taken ("" = none)
  /// Mid-run chase attempts spent, this one included; not a dispatch index.
  /// Terminal dispatches write 0 and a dispatch spending no chase (e.g. an
  /// appointment departure) the running total, so read terminal for
  /// terminality. (notes: explog-dispatch-attempt-field)
  int    attempt = 0;
  int    peers_live = 0;
  int    expected_peers = 0;
  /// Mid-run dispatches only (-1 elsewhere): trigger threshold and estimated
  /// unshared backlog at the trigger. est_unshared_vox -1 = gate off, -2 = no
  /// contact snapshot; an upper bound.
  /// (notes: explog-dispatch-gate-diagnostics)
  double gate_sec = -1.0;
  double est_unshared_vox = -1.0;
  /// Seconds since the team last read complete (now -
  /// team_last_complete_time_); the mid-run trigger fires when this >=
  /// gate_sec. -1 on terminal dispatches. Lags peer_record_age_sec by the
  /// presence TTL. (notes: explog-team-incomplete-sec)
  double team_incomplete_sec = -1.0;
  /// Seconds the radio had been down when a mid-run trigger fired, from the
  /// comms emulator's connected bit; -1 = link gate not in play. Logged beside
  /// peer_record_age_sec, not instead of it. (notes: explog-link-down-sec)
  double link_down_sec = -1.0;
  /// predictor is the arm's setting. On a chase, non-empty predict_refused
  /// means the trail was driven, and the numbers stay populated through a
  /// refusal. predict_candidates 0 means the model never ran.
  /// (notes: explog-dispatch-predictor-fields)
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
  /// reconnected | gave_up | abandoned, classified from team and destination
  /// state when the manoeuvre clock stops. gave_up means it resolved into DONE
  /// or RETURN_HOME. (notes: explog-reconnect-end-outcome)
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

/// mission_complete payload: how the homing leg resolved. At most once per run,
/// only with mission_return_enabled. Its stamp is the mission endpoint when
/// result is arrived; other results are bounded failures.
/// (notes: explog-mission-complete-event)
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
  /// Sim seconds of the whole homing leg since startReturnHome accepted, holds
  /// included. The budget runs off state_enter_time_, so this can exceed
  /// mission_return_max_sec by up to homing_held_sec.
  /// (notes: explog-homing-duration)
  double homing_duration_sec = -1.0;
  double homing_distance_m = -1.0;        ///< metres travelled while homing
  /// Sim seconds of homing_duration_sec spent in PROXIMITY_HOLD. Subtracting it
  /// gives the interval the homing budget is charged against. 0.0 is a measured
  /// zero, not a missing value. (notes: explog-homing-held-sec)
  double homing_held_sec = 0.0;
  bool   latched = false;                 ///< had the coverage latch fired (vs step budget)
  /// 1 = the first and only mission_complete of the run; a higher value is a
  /// defect. Incremented at the call site on every emission, guard or not, so
  /// it is not a field that can only read 1.
  /// (notes: explog-mission-complete-occurrence)
  int    occurrence = 1;
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
  // Configured and realised period of the CSV's periodic sampler, which gates a
  // sim-time timer on a wall deadline and can drop ticks. Recorded so each
  // run's sampling rate is data, not assumption.
  // (notes: explog-metrics-sampler-period)
  double metrics_period_param_sec = -1.0;     ///< configured metrics_period_sec
  double metrics_realised_period_sec = -1.0;  ///< measured mean, -1 if < 2 rows
  double metrics_effective_period_sec = -1.0; ///< the sampler's own current belief
  /// Rows written by the periodic sampler only, not the CSV row count: csv_rows
  /// (excl. header) == metrics_timer_rows + steps.
  /// (notes: explog-metrics-timer-rows)
  int    metrics_timer_rows = 0;
  int    metrics_backoffs = 0;                ///< times it stretched its period
  // --- Mission return / final geometry (schema 2) ---------------------
  // Where the run actually ended, written in every run, not only mission-return
  // ones; with the home pose it audits the homing leg.
  // (notes: explog-run-end-final-geometry)
  bool   have_home = false;                   ///< a home pose was ever recorded
  double home_x = 0.0, home_y = 0.0;          ///< recorded start pose (junk if !have_home)
  double final_x = 0.0, final_y = 0.0;        ///< last known pose at run end
  /// "" (never resolved / feature off) | arrived | timeout | budget |
  /// no-progress — restates mission_complete so single-line readers need not
  /// scan the event stream. The logger writes "" as null.
  std::string mission_home_result;
  double mission_home_sim_sec = -1.0;         ///< sim stamp of the resolution, -1 = none
  /// Mission-return requests refused after the return had resolved
  /// (ExploPlannerNode::mission_return_done_). 0 in a healthy run; recorded so
  /// a guard that never fired differs from an inert one.
  /// (notes: explog-mission-return-reentries)
  int    mission_return_reentries = 0;

  /// Mid-run dispatches spent against reconnect_midrun_max_attempts (default
  /// 6). Never reset or refunded; at the cap the robot falls back to
  /// terminal-only reconnection. Can exceed the cap by one.
  /// (notes: explog-midrun-attempts-used)
  int    midrun_attempts_used = 0;
};

/// cell_census payload: the cell world's status histogram plus the planner's
/// ROI coverage measure, taken on the same tick from the same map so the two
/// need no timestamp join. (notes: explog-cell-census-event)
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
  /// Fraction of cells believed finished by anyone. Comparable to 1 -
  /// roi_unknown_fraction only up to cell quantisation: a cell counts as wholly
  /// covered or not. (notes: explog-census-covered-fraction)
  double covered_fraction = -1.0;
  /// The planner's existing continuous measure and where it came from
  /// ("planning_map" / "scovox" / "none") — the same pair fed to
  /// noteCoverage() on this tick.
  double roi_unknown_fraction = -1.0;
  const char* coverage_source = "none";
  /// Cells whose status changed on the tick that produced this sample.
  int changed = 0;
  /// Sum over cells of update_id: every first-hand status commit. A flap
  /// detector; each commit resets a known_by mask the knowledge gate reads, so
  /// flapping re-arms reconnection. (notes: explog-census-commits-total)
  long long commits_total = 0;
  // --- geometry, so a cell id in this file can be located --------------
  double cell_size_m = 0.0;
  int nx = 0, ny = 0;
  /// FNV-1a over the quantised grid geometry. Two robots whose grid_hash
  /// differs do not mean the same ground by the same cell id, and no
  /// cross-robot comparison of their cell ids is valid.
  unsigned int grid_hash = 0;
  /// FNV-1a over every cell's wire status in id order
  /// (CellWorld::sharedHash()). Equal grid_hash and shared_hash mean the same
  /// shared belief cell for cell; equal counts do not.
  /// (notes: explog-census-shared-hash)
  unsigned int shared_hash = 0;
  /// Enabled / possible edges in the cell adjacency graph. A collapse here is
  /// how a plan-map outage shows up before it becomes an allocation failure.
  int edges_enabled = 0;
  int edges_total = 0;
  // --- is the COVERED threshold reachable at all? ----------------------
  //
  // These fields tell a broken census from an unreachable covered_max_unknown:
  // order statistics over measured cells only, with the two promotion vetoes
  // reported separately. (notes: explog-census-reachability-stats)
  /// Cells with observed_columns >= min_observed_columns. Denominator for the
  /// four below; the rest of the grid has no evidence either way.
  int cells_measured = 0;
  /// Measured cells clearing covered_max_frontier_frac. Compare against
  /// cell_unknown_p10: if this is high while the unknown fractions sit far
  /// above covered_max_unknown, the column measure is the binding veto, and
  /// vice versa.
  int cells_frontier_ok = 0;
  /// Unknown-column fraction over the measured cells: best, 10th percentile,
  /// median; -1 when nothing was measured. No cell can be promoted while
  /// cell_unknown_min exceeds covered_max_unknown.
  /// (notes: explog-census-cell-unknown-stats)
  double cell_unknown_min = -1.0;
  double cell_unknown_p10 = -1.0;
  double cell_unknown_median = -1.0;
  /// Frontier voxels as a fraction of the cell's observed voxels (best cell and
  /// median, over measured cells). A fraction so a threshold means the same at
  /// any cell size; see covered_max_frontier_frac.
  /// (notes: explog-census-frontier-fraction)
  double cell_frontier_frac_min = -1.0;
  double cell_frontier_frac_median = -1.0;
  /// Frontier fraction of the cell with the lowest unknown fraction, the best
  /// promotion candidate. Asks jointly whether the frontier veto would reject
  /// that very cell, which the medians cannot.
  /// (notes: explog-census-frontier-at-best)
  double cell_frontier_frac_at_best_unknown = -1.0;
};

/// team_exchange payload: one received TeamWorld, stamped when drained, one per
/// message merged (see coalesced). Carries the merge result, comms verdict and
/// census just after, so no join is needed. (notes: explog-team-exchange-event)
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

  /// Messages from this sender superseded before this one was drained. Lossless
  /// under full state; persistently non-zero means the planning tick is slower
  /// than team_world_hz. (notes: explog-exchange-coalesced)
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
  /// Negative on rows without a transition. acquire_sec: first one-way packet
  /// to the handshake-completing one; held_sec: handshake lifetime, set only
  /// when a still-audible peer stops naming us (see TeamModel::Peer).
  /// (notes: explog-exchange-handshake-times)
  double acquire_sec = -1.0;
  double held_sec = -1.0;

  /// How long since the last MUTUAL direct contact with this peer, measured
  /// after the tick. Legitimately ~0 while the link is up — that is the link
  /// being up — and the interesting readings are the non-zero ones, which come
  /// from a sender heard one-way: the message arrived, the handshake did not.
  double last_direct_age_sec = -1.0;
  /// Age of our knowledge of this peer when the message landed, read before it
  /// was believed: the silence it ended. -1.0 = never heard from, distinct from
  /// 0.0. Gossip and one-way receipt also refresh it.
  /// (notes: explog-exchange-last-known-age)
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

/// allocation payload: one global-allocator solve, emitted from the planning
/// tick that used it. The three field groups below check the mechanism from the
/// logs alone. (notes: explog-allocation-event)
struct AllocationEvent {
  // --- 1. did both robots solve the SAME problem? -----------------------
  //
  // Solve-same-take-own is only testable over a converged world: robots holding
  // different beliefs should allocate differently.
  // (notes: explog-alloc-same-problem)
  unsigned int shared_hash = 0;
  unsigned int grid_hash = 0;
  /// Digest of the problem as solved (see the alloc_hash doc in
  /// global_allocator.hpp); join on this, not shared_hash, which covers
  /// statuses only. 0 = no problem formed; skip it, do not match zeros.
  /// (notes: explog-alloc-hash-join-key)
  unsigned int alloc_hash = 0;
  /// The traversability half of `alloc_hash`, logged separately so a
  /// disagreement can be attributed to the cost matrix rather than to the
  /// fleet or the candidate set. This is the channel that does NOT shrink when
  /// the link comes back up, and until now it had no detector at all.
  unsigned int edge_hash = 0;
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
  // Each robot solves for the whole fleet, so it logs its belief about peers'
  // focus too: robot A's peer_focus against robot B's focus_cell is an exact
  // agreement check. (notes: explog-alloc-focus-agreement)
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
  // These fields show whether the allocation actually changed which goal was
  // chosen; nothing above can. (notes: explog-alloc-steering-check)
  /// Tour rank of the candidate finally selected: 0 = the focus cell's
  /// neighbourhood, 1 = the next tour cell's, and so on; -1 when no goal was
  /// selected this tick, and `kAllocRankUnrestricted` when the goal came from
  /// outside every tour cell (the documented fall-back).
  int picked_rank = -1;
  /// True when the ranking actually reordered the candidate list — i.e. at
  /// least one frontier candidate fell outside the focus neighbourhood. False
  /// means the filter was a no-op this tick and `picked_rank` proves nothing.
  bool reordered = false;
  /// Solves in a row, holding this cell as focus, with no admissible candidate
  /// from its neighbourhood. Kept per cell id and not decayed by ticks, so it
  /// bounds nothing about elapsed time. (notes: explog-alloc-focus-skips)
  int focus_skips = 0;
  std::string demoted;
};

/// `AllocationEvent::picked_rank` for a goal that came from outside every
/// tour cell's neighbourhood. A large sentinel rather than -1 so it sorts
/// after every real rank and cannot be confused with "no goal".
inline constexpr int kAllocRankUnrestricted = 9999;

/// reconnect_gate payload: one utility-gate decision and its arithmetic, on
/// every evaluation, not only dispatches. Evaluated only behind the silence
/// clock, link veto and attempt budget, so rare.
/// (notes: explog-reconnect-gate-event)
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

/// rendezvous_agreed payload: an adoption of the (cell, interval, t_meet)
/// triple fleet id 0 publishes, emitted on every arming attempt including
/// refusals. Compare cell + interval_sec directly, not shared_hash.
/// (notes: explog-rzv-agreed-adoption)
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
  /// The agreed occurrence this robot keeps, mission-elapsed
  /// (nextAgreedOccurrence). t_meet_sec - t_now_sec is at least the arrival
  /// shortfall (arrivalShortfallSec); agreement is cell + interval_sec, not
  /// this. (notes: explog-rzv-t-meet-sec)
  double t_meet_sec  = -1.0;
  double t_now_sec   = -1.0;
  /// The committed t_meet this row rolled from, before nextAgreedOccurrence
  /// advanced it; with cell and interval_sec it is the whole triple. One run
  /// can hold several agreements: group by this before folding.
  /// (notes: explog-rzv-agreed-base)
  double agreed_base_sec = -1.0;
  /// The integer the proposer put on the wire, so robots in one outage log it
  /// identically. The meeting recurs every interval_sec; rows on one triple
  /// differ in t_meet_sec by whole multiples. Fold on agreed_base_sec.
  /// (notes: explog-rzv-interval-sec)
  double interval_sec = -1.0;
  /// True when the findability cap pulled interval_sec below the tour term
  /// (RendezvousScheduler::Config::max_interval_ms); see tour_interval_sec.
  /// Proposer rows only; false on a follower means unknown.
  /// (notes: explog-rzv-capped)
  bool   capped       = false;

  /// True when the lattice floor (rendezvous_interval_sec, or the marked-up
  /// drive if larger) set interval_sec rather than the tours; capped cannot
  /// answer this. Proposer only; false on a follower means did not solve.
  /// (notes: explog-rzv-floored)
  bool   floored      = false;

  /// What the objective asked for before the floor and cap: the furthest
  /// robot's marked-up drive to the winning cell. -1 where no solve produced it
  /// (follower rows), distinct from a real 0. (notes: explog-rzv-tour-interval)
  double tour_interval_sec = -1.0;

  /// Objective value at the winner, in the allocator's quantised unit; 0 =
  /// someone was already driving there. Proposer only: a follower writes -1
  /// here and in the counts below; filter on proposer_id == robot.
  /// (notes: explog-rzv-penalty-proposer-only)
  long long penalty_mm = -1;
  /// True when the scheduler's floor cell (centroid of the team's vehicle cells
  /// at proposal time) won the argmin; read with candidates. Proposer only; a
  /// follower writes false, meaning did not search.
  /// (notes: explog-rzv-floor-won)
  bool floor_won = false;
  int  candidates = -1;
  int  rejected_unreachable = -1;
  int  rejected_excluded    = -1;

  /// This robot's travel estimate to the winner, seconds; per-robot, not
  /// expected to match the peer's. A diagnostic, not a departure input: how far
  /// the robot was from the meeting when it armed.
  /// (notes: explog-rzv-travel-sec)
  double travel_sec  = -1.0;

  /// Non-empty when no appointment could be derived. Distinguishes "no meeting
  /// was scheduled" from "a meeting was scheduled and nothing came of it".
  std::string refused;

  // --- the agreement itself (gen 10) --------------------------------------
  //
  // (notes: explog-rzv-agreement-fields-history)

  /// Fleet id of the robot that derived the pair, 0 by design. On the proposer,
  /// penalty_mm, floor_won, candidates and rejected_* describe the argmin; on a
  /// follower they are sentinels. (notes: explog-rzv-proposer-id)
  int proposer_id = -1;

  /// The appointment came from a pair the whole team was seen holding, rather
  /// than from a local solve. False in a refusal row.
  bool from_agreed = false;

  /// The committed triple was the centroid placeholder (the argmin had one
  /// admissible cell). Every robot writes the truth here, since the flag
  /// travels on the wire. False in a refusal row.
  /// (notes: explog-rzv-agreed-provisional)
  bool agreed_provisional = false;

  /// Mission-elapsed instant the team last read mutually complete on this
  /// robot. A diagnostic, no longer an input to t_meet_sec. -1.0 when it never
  /// saw mutual contact, which armed rows can carry too.
  /// (notes: explog-rzv-anchor-sec)
  double anchor_sec = -1.0;

  /// How long this exact triple had stood, team-confirmed, before separation;
  /// stamped on change. It is anchor_sec minus the commit time, so negative
  /// means a late commit; -1.0 is ambiguous, check anchor_sec < 0.
  /// (notes: explog-rzv-agreed-age)
  double agreed_age_sec = -1.0;

  /// Peers holding the committed pair when it was committed. The commit rule
  /// (maintainRendezvousProposal) requires all fleet-1 echoes, so less than
  /// that on an armed row means the rule was relaxed.
  /// (notes: explog-rzv-peers-on-pair)
  int peers_on_pair = 0;

  /// This robot's route to the agreed cell over its own frozen cell graph,
  /// metres, or -1 for no route; read only where from_agreed. Measured,
  /// deliberately not a refusal: no route often means unexplored.
  /// (notes: explog-rzv-own-route)
  double own_route_m = -1.0;
};

/// rendezvous_outcome payload: how an armed appointment ended. One per
/// rendezvous_agreed that armed, so the two join 1:1 within an outage.
/// (notes: explog-rzv-outcome-event)
struct RendezvousOutcomeEvent {
  int    cell       = -1;
  double t_meet_sec = -1.0;
  /// When this robot actually got there (or gave up), mission-elapsed.
  double t_end_sec  = -1.0;
  /// Signed: negative is early, positive is late. The quantity the departure
  /// rule exists to keep near zero, and the one mTARE's depart-at-zero rule
  /// cannot control because its lateness differs per robot.
  double lateness_sec = -1.0;

  /// reconnected, no-show (waited out the cap), unreachable, superseded,
  /// run-ended (run ended under it), unplaceable (cell not placeable; arrived
  /// forced false). The last two rank above arrived/unreachable.
  /// (notes: explog-rzv-outcome-vocabulary)
  std::string outcome;
  /// True when the robot reached the cell at all, whatever the outcome. A
  /// no-show with `arrived` false is a navigation failure wearing a
  /// coordination failure's name.
  bool arrived = false;
  /// Seconds at the cell from the arrival stamp to the close. 0.0 means it
  /// never waited, usually never arrived (superseded always writes 0.0): filter
  /// on arrived before averaging. No emitted row carries -1.
  /// (notes: explog-rzv-waited-sec)
  double waited_sec = -1.0;
  /// True when the reunion was whole-team direct contact, stricter than
  /// outcome, which names what the barrier acted on (relays count). At N=2 it
  /// is true exactly on reconnected rows. (notes: explog-rzv-mutual)
  bool mutual = false;
};

/// One rung of the escape ladder guarding the drive to an agreed cell
/// (appointmentLegWatchdog / resumeAppointmentDrive), on home_watchdog's
/// kind/cause convention. Not a reachability verdict: count rungs.
/// (notes: explog-appointment-leg-event)
struct AppointmentLegEvent {
  /// escape: a detour was dispatched (escapes_used is post-increment).
  /// escape-end: the detour finished; not a rung spent. unreached: ladder
  /// spent, leg abandoned to PLAN; the livelock check keys on it.
  /// (notes: explog-appointment-leg-kinds)
  std::string kind;
  /// On "escape"/"unreached", the watchdog that ended the drive: "budget" |
  /// "no-progress". On "escape-end", why the detour ended: "escape-arrived" |
  /// "escape-leg-cap".
  std::string cause;
  /// Straight-line metres from the robot to the agreed cell at this instant —
  /// to `return_dest_`, which a detour deliberately leaves alone, so it is the
  /// same quantity on all three kinds and comparable across them.
  double dist_m = -1.0;
  /// The agreed cell's position, i.e. `return_dest_`. Written rather than
  /// derived from `cell` because the appointment RECORD can be closed
  /// underneath a live leg (see appointmentLegWatchdog's header) while the
  /// destination stays true.
  double dest_x = 0.0;
  double dest_y = 0.0;
  /// The agreed cell id, or -1 when the record was already closed under the
  /// leg. A -1 here is NOT an unplaceable cell — that case is
  /// `rendezvous_outcome.outcome == "unplaceable"`.
  int cell = -1;
  /// Rungs spent against `rendezvous_escape_max_attempts`, which is the second
  /// field so a reader never has to fetch the param row to normalise.
  int escapes_used = 0;
  int escapes_max  = 0;
  /// The detour's duration, on "escape-end" rows only; -1.0 elsewhere, which
  /// is a "not this kind of row" sentinel and not a reading.
  double leg_sec = -1.0;
  /// On unreached, the occurrence the appointment was rolled to,
  /// mission-elapsed; -1.0 means no record was left and the robot went back to
  /// exploring. -1.0 on every other kind.
  /// (notes: explog-appointment-leg-rolled-to)
  double rolled_to_sec = -1.0;
};

// ==================================================================
// ExperimentLog
// ==================================================================

class ExperimentLog {
 public:
  /// Schema version stamped into `run_start`. Bump on ANY change to field
  /// names or meanings so an analysis script can refuse a file it predates.
  /// A new event kind in kEventKinds also bumps it; purely additive fields
  /// alone do not. A behaviour change may bump it so readers can refuse to
  /// pool. (notes: explog-schema-version-history)
  static constexpr int kSchemaVersion = 12;

  /// Every event value this writer can emit, and the only authority on that
  /// set: the equivalence gate relies on it to tell whether a new kind
  /// appeared. (notes: explog-event-kinds-authority)
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
      "reconnect_gate", "team_exchange", "appointment_leg",
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

  /// Opens path truncating, in binary mode (LF, byte-exact UTF-8). Unlike
  /// MetricsLogger it does not throw on failure: one ROS ERROR, then open() and
  /// healthy() stay false. (notes: explog-open-does-not-throw)
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
  /// Duplicate run_end attempts suppressed: the ROS warning's count as a value
  /// tests can assert on. Non-zero means the node reached a terminal state
  /// twice; endpoint metrics are suspect. (notes: explog-dup-run-ends)
  long long dupRunEnds() const { return dup_run_ends_; }
  /// Duplicate mission_complete attempts suppressed, also written into run_end
  /// as mission_completes_suppressed. Non-zero means something asked to end the
  /// mission twice. (notes: explog-dup-mission-completes)
  long long dupMissionCompletes() const { return dup_mission_completes_; }
  /// Events refused because `run_start` had not been emitted yet, and therefore
  /// never written. Mirrors `events_dropped_before_start` on the run_end row;
  /// exposed so the accounting is testable in the case where the DROPPED event
  /// is itself a run_end and no row exists to report it.
  long long droppedBeforeStart() const { return dropped_before_start_; }
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

  /// Emit run_start, latching t0 from ctx.sim_time_sec. Call only once the
  /// clock is live (under use_sim_time now() reads 0 until /clock). milestones
  /// is sorted descending and de-duplicated here.
  /// (notes: explog-start-run-live-clock)
  void startRun(const ExperimentContext& ctx,
                const std::vector<double>& milestones);

  // ----------------------------------------------------------------
  // Events. Every one is a no-op before startRun() (counted, and reported as
  // events_dropped_before_start in run_end) and after a failed open().
  //
  // A new emitter's early return must increment the dropped-before-start count,
  // or run_end claims nothing was lost. noteCoverage() is the one exemption.
  // (notes: explog-dropped-count-contract)
  // ----------------------------------------------------------------
  void logStep(const ExperimentContext& ctx, const StepEvent& e);
  /// clock_anchor: the sim/wall pair plus the real-time factor since the
  /// previous anchor and since t0. Schedule it on sim time and never gate it on
  /// a wall-clock condition. (notes: explog-clock-anchor-cadence)
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
  /// A navigation attempt gave up; k is the site's cumulative failure count,
  /// retired = suppressed for the rest of the run. The check that fired is
  /// test_name/test_value/test_threshold; elapsed_sec is context.
  /// (notes: explog-nav-goal-failed)
  void logNavGoalFailed(const ExperimentContext& ctx, double x, double y,
                        const char* reason, double elapsed_sec, int k,
                        bool retired, double budget_sec, bool pose_stale,
                        const char* test_name, double test_value,
                        double test_threshold);
  /// The TF pose went stale (lost=true) or came back (lost=false), so a dead
  /// pose feed is not read as a physical stall. age_sec is the transform's age
  /// at the transition, 0 on recovery. (notes: explog-pose-health)
  void logPoseHealth(const ExperimentContext& ctx, bool lost, double age_sec);
  /// Every candidate was suppressed and one was re-attempted anyway
  /// (non-retired first, least recently stamped). source: failed tier first,
  /// visited only when it offers nothing; an absent key means failed.
  /// (notes: explog-goal-amnesty-tiers)
  void logGoalAmnesty(const ExperimentContext& ctx, double x, double y,
                      double last_fail_age_sec, bool retired,
                      const char* source);
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
  /// dist_home_m and metric_m are instantaneous samples at the fire, not the
  /// tested quantity (that is test_delta_m). Both are kept: distance left and
  /// distance moved answer different questions.
  /// (notes: explog-home-watchdog-tested-delta)
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

  /// Feed this sample's coverage. Emits one coverage_milestone per rung crossed
  /// for the first time (several can share a stamp); rungs never re-fire.
  /// Negative or non-finite fractions are ignored.
  /// (notes: explog-note-coverage-milestones)
  void noteCoverage(const ExperimentContext& ctx, double unknown_fraction,
                    const char* coverage_source, double distance_m,
                    double x, double y);

  /// Emits one cell_census (schema v4), unconditionally: no ladder or latch,
  /// since it tracks the histogram's trajectory, which can go backwards. The
  /// caller decides the rate. (notes: explog-cell-census-unlatched)
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

  /// Emits one `appointment_leg` (schema v11). Zero or more per appointment —
  /// a meeting the robot simply drives to writes none — so this does NOT join
  /// 1:1 with `rendezvous_outcome`, and on the give-up path it can outlive it:
  /// a leg whose record was closed underneath it still writes its rungs.
  void logAppointmentLeg(const ExperimentContext& ctx,
                         const AppointmentLegEvent& e);

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
  /// Sim stamp of the run_end that WAS written, so the warning on a suppressed
  /// second attempt can name both instants — "a duplicate happened" is much
  /// less useful than "it happened 61 s after the first".
  double run_end_sim_sec_ = -1.0;
  bool   dup_run_end_reported_ = false;
  /// Suppressed duplicate run_end attempts. Never written in a row (run_end
  /// stays the last line); the destructor reports it to the ROS log, where
  /// gate_g8 check 3n reads it. (notes: explog-dup-run-ends-reporting)
  long long dup_run_ends_ = 0;
  /// Idempotence latch for mission_complete in the FILE; the node guards the
  /// behaviour with mission_return_done_. Keeps exactly one mission_complete
  /// per robot-run. (notes: explog-mission-complete-latch)
  bool   mission_complete_written_ = false;
  /// Sim stamp of the mission_complete that WAS written, so a suppressed
  /// second attempt can name both instants rather than just its own.
  double mission_complete_sim_sec_ = -1.0;
  bool   dup_mission_complete_reported_ = false;
  /// Suppressed duplicate mission_complete attempts. Written into run_end.
  long long dup_mission_completes_ = 0;
  bool   write_error_reported_ = false;
  double t0_sec_ = 0.0;
  long long seq_ = 0;                      ///< monotonic event index, from 0
  long long write_failures_ = 0;
  long long dropped_before_start_ = 0;

  /// Absolute sim seconds of the last and first exploration_complete. The last
  /// is THE completion time: a robot can resume on a merged map and exhaust
  /// again. Both < 0 until one is emitted.
  /// (notes: explog-explore-done-last-first)
  double explore_done_last_sec_  = -1.0;
  double explore_done_first_sec_ = -1.0;
  /// Has a step event arrived since the latest exploration_complete? The
  /// clearable half of the post_latch test in noteCoverage. Set by logStep,
  /// cleared by logExplorationComplete. (notes: explog-explore-resumed-flag)
  bool   explore_resumed_since_done_ = false;

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
