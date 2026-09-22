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
  /// Pose in the map frame (m, m, m, rad). THE ONLY SURVIVING TRAJECTORY: the
  /// CSV has no pose columns and the campaign runs with bagging disabled, so
  /// without these no figure of a rendezvous, a chase or a hold can be drawn
  /// after the fact. z rides along because it costs one field and terrain mode
  /// makes it meaningful.
  double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
  const char* phase = "explore";          ///< "explore" / "exploit"
  /// accountedPeerCount: peers heard within one TTL on EITHER the claim table
  /// or TeamWorld, PLUS peers that announced `finished` (which has no TTL and
  /// may have been relayed). Read "within one claim TTL" here until 2026-09-18;
  /// that has been the narrower livePeerCount since generation 23 and this
  /// field has never carried it. Same definition in every event that has a
  /// `peers_live` — they are all written from the one wrapper.
  int    peers_live       = 0;
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
  /// chase | appointment | anchor_return | hold | resume_exploring
  ///
  /// `meeting_point` was a sixth value until generation 19 and is gone: it named
  /// the midpoint construction, which was deleted. `appointment` replaced it and
  /// is NOT a rename — the midpoint was computed privately by each robot, the
  /// appointment is the cell the whole fleet committed to. Banked logs still
  /// carry the old string, so the readers accept it; the node no longer writes
  /// it. Any change here has to be made in sim/reconnect_value.py (ACTED) and
  /// sim/manoeuvre_events.py (DEST) in the same edit, or the treated arm's main
  /// manoeuvre silently reclassifies as the planner declining to act.
  const char* action = "hold";
  bool   have_dest = false;               ///< false -> destination fields are null
  double dest_x = 0.0, dest_y = 0.0;
  double budget_sec = -1.0;               ///< pursuit budget; -1 where not applicable
  std::string decline_reason;             ///< why a richer action was NOT taken ("" = none)
  /// How many mid-run CHASE attempts this robot has spent, this one included.
  /// Not a dispatch index: `midrun_attempts_` is incremented only on the
  /// gate-approved chase path, so a dispatch that spends no chase reports the
  /// running total unchanged.
  ///
  /// 0 THEREFORE HAS TWO MEANINGS AND `attempt` ALONE CANNOT SEPARATE THEM.
  /// Through v7 this said "0 for terminal", which is half the truth and the
  /// dangerous half. The other producer of a 0 is an APPOINTMENT DEPARTURE:
  /// the robot leaves for a meeting it agreed to earlier, which is a mid-run
  /// manoeuvre (`terminal=false`, `trigger="midrun"`) that costs no chase
  /// budget. Measured on the gen-22 smoke: hybrid wrote 13 such rows against 5
  /// genuine chases, so `attempt >= 1` undercounts hybrid's mid-run
  /// manoeuvres by 72% and `attempt == 0` does not mean terminal there at all.
  /// Read `terminal` for terminality and `reason` for which manoeuvre it was
  /// ("appointment-due" is the departure); use this field only for how much of
  /// `reconnect_midrun_max_attempts` the cell has burned.
  int    attempt = 0;
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
  /// teamComplete(accountedPeerCount(), ...) reads the team complete. It said
  /// livePeerCount() until 2026-09-18 and the heartbeat has called the wider
  /// count since generation 23; see accountedPeerCount for the three channels.
  /// A peer counts present until BOTH the coordination claim expires —
  /// coord_claim_ttl_sec (5.0 s) after its last beacon — and TeamWorld's
  /// `direct` ages out on TeamModel::direct_ttl_sec (5.0 s). So this still runs
  /// BEHIND the peer's record age by the TTL, plus up to one heartbeat period
  /// of quantization:
  ///
  ///     team_incomplete_sec ~= peer_record_age_sec - coord_claim_ttl_sec
  ///
  /// THE RELATION SURVIVES ONLY BECAUSE THE TWO TTLs ARE BOTH 5.0. Move either
  /// and the lag becomes the max of them, not coord_claim_ttl_sec. And the
  /// third channel voids it outright: a peer that announced `finished` counts
  /// present with no TTL at all, so once any peer finishes this column stops
  /// tracking that peer's record age entirely — which is correct for the gate
  /// (a finished robot is not worth reconnecting to) and wrong for anyone
  /// inverting the formula to recover a record age.
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
  /// SIM seconds of WALL time on the homing leg: last instant minus the
  /// instant startReturnHome accepted, so it spans everything the leg spent,
  /// including any proximity holds taken inside it.
  ///
  /// v8 CHANGED WHAT THIS MEASURES. Through v7 it was derived from
  /// `state_enter_time_`, which the proximity-hold release deliberately
  /// backdates by the drive time already spent so that the nav budget and the
  /// `mission_return_max_sec` cap CONTINUE across a hold rather than
  /// restarting. Correct for a budget, wrong for a report: it made this a
  /// drive-only clock shipped beside a whole-leg `homing_distance_m`, so the
  /// implied homing speed was overstated by exactly the held fraction — and
  /// held time is arm-correlated, because the arms that regroup arrive home
  /// together and the hold radius (5.0 m) is larger than the spacing between
  /// homes (~3 m). Do not pool v7 and v8 homing durations.
  ///
  /// The BUDGET still runs off `state_enter_time_` and is unchanged. So this
  /// may legitimately exceed `mission_return_max_sec` by up to
  /// `homing_held_sec` without the cap having failed; subtract before
  /// concluding an overrun.
  double homing_duration_sec = -1.0;
  double homing_distance_m = -1.0;        ///< metres travelled while homing
  /// SIM seconds of `homing_duration_sec` spent parked in PROXIMITY_HOLD —
  /// commanded yields to a teammate, not the robot's own stall. New in v8.
  /// `homing_duration_sec - homing_held_sec` recovers the v7 quantity, which
  /// is also the interval the homing budget is charged against, and
  /// `homing_distance_m / (homing_duration_sec - homing_held_sec)` is the
  /// driving speed. 0.0 is the overwhelmingly common value and a measured one:
  /// it is a difference of two run-cumulative counters, so a leg that took no
  /// hold reports zero rather than nothing.
  double homing_held_sec = 0.0;
  bool   latched = false;                 ///< had the coverage latch fired (vs step budget)
  /// 1 = the first and, per this struct's contract, only `mission_complete` of
  /// the run. Mirrors `ExplorationCompleteEvent::occurrence`, but for the
  /// opposite reason: there a second occurrence is legitimate, here it is a
  /// DEFECT. ts1b shipped 16 robot-runs carrying two `mission_complete` rows
  /// (DONE -> RETURN_HOME re-entry, fixed by the guard at the top of
  /// startReturnHome), and finding that took a 240-cell audit precisely because
  /// the rows were indistinguishable from each other. With this field a
  /// duplicate is one grep.
  ///
  /// Deliberately NOT a self-reporting PASS/FAIL: the node counts and writes,
  /// the offline gate decides. A field that can only ever read 1 would be
  /// another check that stopped checking, so the counter is incremented at the
  /// call site on every emission, guard or no guard, and test_endpoint.cpp
  /// proves it reaches 2 when handed two events.
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
  /// Mission-return requests refused after the return had already resolved
  /// (ExploPlannerNode::mission_return_done_). 0 in a healthy run. Reported
  /// because "the guard never had to fire" and "the guard is inert" are
  /// different claims and a run that records neither cannot tell them apart:
  /// through generation 8 the re-entry was real, happened 16 times in ts1b,
  /// and left no field anywhere saying so.
  int    mission_return_reentries = 0;

  /// Mid-run reconnect dispatches spent this run, against
  /// `reconnect_midrun_max_attempts` (default 6). SCHEMA 8 (2026-09-18).
  ///
  /// THE CAP IS A DOSE TERM AND THIS IS THE ONLY PLACE IT BECOMES VISIBLE. The
  /// counter is run-lifetime with no reset and no refund — a successful mid-run
  /// reconnection costs an attempt exactly like a failed one — so once it
  /// reaches the cap the robot silently reverts to terminal-only reconnection
  /// for the remainder of the run. That is a weaker treatment, and it arrives
  /// sooner in the arms and rungs that produce more outages (pursuit and
  /// hybrid's chase half; N=4 before N=2). Without this field, a cell that ran
  /// half its length under the reduced treatment is indistinguishable from one
  /// that never came close, and the difference would land in the arm means as
  /// noise attributed to the mechanism.
  ///
  /// READ IT AGAINST THE CAP, NOT AS A COUNT OF ANYTHING GOOD. A high value is
  /// not "the mechanism worked hard" — dispatch does not imply reconnection —
  /// it is "this cell was closest to losing the mechanism". `> cap` is
  /// possible by one: the exhaustion warning increments once more so it logs
  /// exactly once.
  int    midrun_attempts_used = 0;
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
  /// R1's two transition measurements (generation 33), negative on every row
  /// that does not carry the transition — which is nearly all of them. Counting
  /// acquisitions is therefore counting non-negative `acquire_sec` values, not
  /// diffing `direct` between consecutive rows and hoping no transition fell in
  /// a gap between them.
  ///
  /// `acquire_sec`: seconds from the first packet of the one-way period to the
  /// packet that completed the handshake. `held_sec`: seconds a completed
  /// handshake survived, set ONLY when it was broken by a peer that stayed
  /// audible and stopped naming us back — never by the packets stopping, which
  /// has no closing packet to stamp and a wholly different cause. Both are
  /// differences of PACKET stamps, so neither carries tick jitter.
  /// See TeamModel::Peer for the full derivation.
  double acquire_sec = -1.0;
  double held_sec = -1.0;

  /// How long since the last MUTUAL direct contact with this peer, measured
  /// after the tick. Legitimately ~0 while the link is up — that is the link
  /// being up — and the interesting readings are the non-zero ones, which come
  /// from a sender heard one-way: the message arrived, the handshake did not.
  double last_direct_age_sec = -1.0;
  /// v8 CHANGED WHAT THIS MEASURES, from a quantity that was always 0.0 to one
  /// that is rarely 0.0. It is now the age of our knowledge of this peer at the
  /// instant the message landed and BEFORE it was believed — the receiver-side
  /// inter-arrival gap, i.e. the length of the silence this message ended.
  ///
  /// v7 read it from the model in the second pass, after observe() had already
  /// stamped the sender's last_known_sec with this same tick time. Since a row
  /// only ever exists for a sender, the subtraction was t - t: it read 0.0 on
  /// 757,677 of 757,677 rows in the banked campaigns, which is not a finding
  /// about comms but the absence of a measurement. Do not pool v7 and v8 on
  /// this field, and do not read a v7 zero as "no gap".
  ///
  /// -1.0 means never heard from, which is distinct from 0.0 (heard on the
  /// immediately preceding drain). Not the same question as
  /// last_direct_age_sec above: gossip and one-way receipt both refresh this
  /// one, so a peer can have a small gap here and a large one there.
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
  /// R3 / §3.6. `shared_hash` cannot serve as the "same problem" join key it
  /// was being used as: it covers cell STATUSES, while the problem also
  /// contains a vehicle set (not part of the world at all) and a cost matrix
  /// (never exchanged — `mergeWire` reconciles status and `known_by` and never
  /// touches `edges_`). Equal `shared_hash` therefore does not imply the same
  /// problem, so every analysis that restricted to equal `shared_hash` and
  /// called the remainder "the allocator's disagreements" was attributing a
  /// world difference to the solver. This follows from what the hash covers;
  /// it needs no measurement to support it, and the "4.5% at N=2 / 23% at N=4"
  /// figures this comment used to quote are withdrawn as unreproducible (see
  /// global_allocator.hpp, the `alloc_hash` doc comment).
  ///
  /// `alloc_hash` digests the problem as solved: the vehicle tuples sorted by
  /// id, the candidate ids, and `edge_hash`. Join on THIS. Two robots with the
  /// same `alloc_hash` and different tours have a determinism bug; two with
  /// different `alloc_hash` were never solving the same problem.
  ///
  /// 0 means no problem was formed (the world was unconfigured), which is not
  /// agreement — skip refused solves rather than matching zeros.
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
  /// How many solves IN A ROW that held the current focus cell as focus have
  /// produced no admissible candidate from its neighbourhood.
  ///
  /// NOT consecutive ticks, which is what this said through v7 — the node's
  /// counter is indexed by cell id and is only touched on a tick where that
  /// cell is the focus, so it survives arbitrarily many intervening ticks
  /// spent on a different focus, and it never decays. Ticks that selected
  /// nothing at all (global starvation) neither advance nor reset it. So a
  /// large value here bounds nothing about elapsed time; pair it with `step`
  /// if you need that. Reading only changed, not the field.
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
/// check offline that every robot is holding the same one (§3.5).
///
/// THE NAME IS NOW LITERAL, AND IT WAS NOT BEFORE. Under v2 agreement was a
/// protocol; under v3/v4 the claim was that no protocol was needed, because
/// both robots ran identical deterministic arithmetic and would therefore land
/// on the same appointment by construction. The arithmetic was fine. The
/// premise that they fed it the same inputs was not — each solved over its own
/// privately merged map — and measured across the banked ts3 cells the two
/// ends picked the same cell in 21 of 64 separated pairs, with a median
/// t_meet disagreement of 91 s at N=2 and 125 s at N=3.
///
/// So under v5 there IS a protocol again: fleet id 0 derives the (cell,
/// interval, t_meet) triple, publishes it on TeamWorld, and everyone else
/// adopts those three integers verbatim and echoes them until the whole team is
/// seen holding the same triple. This event records an ADOPTION, not a
/// derivation,
/// and `from_agreed` / `peers_on_pair` / `proposer_id` are what make that
/// checkable per row.
///
/// DO NOT JOIN ON `shared_hash`. That was the v4 join key, on the theory that
/// an equal hash meant a shared world and any residual disagreement was the
/// scheduler's fault. It is not a complete witness: nine ts3 pairs disagreed
/// on the appointment with IDENTICAL `shared_hash`, and the candidate COUNT
/// differed in 5 of 19. Join on the outage and compare `cell` + `interval_sec`
/// directly — under v5 they are exchanged integers, so equality is the whole
/// question and it needs no proxy.
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
  /// THIS ROBOT'S DEPARTURE INSTANT, mission-elapsed seconds: the first agreed
  /// occurrence at or after bare `t_now_sec` (nextAgreedOccurrence; the floor
  /// was `t_now_sec + rendezvous_depart_delay_sec` through v8, and that
  /// per-robot notice is what forked the ts4 N=3 cell — see the v9 note). The
  /// OCCURRENCE GRID is agreed; WHICH occurrence this robot keeps follows from
  /// its own arming instant, so on v9 this MATCHES the peers' whenever every
  /// robot armed before the agreed instant — the expected case — and differs
  /// by whole intervals only when one armed after it had genuinely passed.
  ///
  /// Equality across rows is still NOT the agreement test, for the v6 reason:
  /// a late-armer's roll is the mechanism working, not the protocol failing.
  /// Agreement is `cell` + `interval_sec` (see the join note above).
  ///
  /// What the column is still worth: `t_meet_sec - t_now_sec` lands in
  /// [shortfall, shortfall + `interval_sec`), where the shortfall is however
  /// far THIS robot's drive to the cell overruns `rendezvous_max_lateness_sec`
  /// and is zero for a robot that can be punctual (arrivalShortfallSec). It was
  /// [0, `interval_sec`) on v9, when a robot kept the nearest rung whether or
  /// not it could get there; a v9 reader applying that upper edge fails every
  /// honest v10 roll, which is the whole reason the version moved. The
  /// shortfall is not logged, so the checkable bound is the LOWER edge alone —
  /// a negative lead is still a row the arming cannot produce. And the SPREAD
  /// of `t_now_sec` across a cell's robots is the direct measurement of how
  /// staggered the armings were, which is what the shared instant then has to
  /// tolerate.
  double t_meet_sec  = -1.0;
  double t_now_sec   = -1.0;
  /// THE COMMITTED `t_meet` THIS ROW ROLLED FROM, unrolled: the integer off the
  /// wire, before nextAgreedOccurrence advanced it to the occurrence this robot
  /// keeps. With `cell` and `interval_sec` it is the whole triple, so equality
  /// across rows is the exact test of whether two robots hold the same
  /// agreement — which `t_meet_sec` alone stopped being at generation 29.
  ///
  /// IT EXISTS BECAUSE THERE IS MORE THAN ONE AGREEMENT PER RUN NOW. Through
  /// v9 a run held at most two triples (the provisional and the one upgrade),
  /// the second of which stood to the end, so a cell's armings all folded onto
  /// one lattice and the base could be recovered as their minimum. Generation
  /// 29 re-agrees the next place and time at every meeting kept, each new base
  /// being `mission_elapsed + interval` at an arbitrary instant, so successive
  /// generations of ONE cell sit on lattices that do not share a phase. A
  /// reader folding by the minimum reads that phase difference as a protocol
  /// failure — a false accusation on the healthiest possible run, one that met
  /// often enough to re-agree. Group by this column first, then fold.
  double agreed_base_sec = -1.0;
  /// THE AGREED QUANTITY: the integer the proposer put on the wire, so two
  /// robots in one outage must log it identically and any difference is a
  /// protocol failure, not a rounding one.
  ///
  /// IT IS THE RECURRENCE PERIOD `t_meet_sec` IS DERIVED FROM: the meeting
  /// repeats every `interval_sec` and each robot keeps the first occurrence at
  /// or after its own departure floor (nextAgreedOccurrence). That was true
  /// through v6, was not on generations 19-22, and is true again on generation
  /// 23. It is also the agreement evidence this gate scores and the provenance
  /// for the argmin, saying which cell was chosen under what reachability
  /// assumption. See RendezvousPlan::interval_ms for the reachability floor
  /// that sets it.
  ///
  /// SO THE MODULO COMPARISON IS THE CROSS-ROBOT TEST AGAIN, and it was
  /// withdrawn here for generations 19-22 because there were no periods to
  /// fold by. Two robots holding the committed triple log `t_meet_sec` values
  /// that differ by a WHOLE MULTIPLE of `interval_sec`; a remainder is a
  /// protocol failure. It is a real test rather than a way to launder one only
  /// because `interval_sec` itself must match exactly first — check that, then
  /// the remainder, and check `t_meet_sec` against `t_now_sec` on the SAME row
  /// for the departure window's lower edge.
  ///
  /// FOLD AGAINST `agreed_base_sec`, NOT AGAINST THE SMALLEST `t_meet_sec` IN
  /// THE CELL. Since generation 29 one cell can carry several agreements and
  /// their lattices need share no phase, so the minimum is a base only when the
  /// run happened to re-agree nothing. See that field for the rest.
  double interval_sec = -1.0;
  /// True when the findability cap pulled `interval_sec` in below what the
  /// tours imply. (It read "the recurrence period" until 2026-09-18; there is
  /// no recurrence, and the cap now bounds the furthest robot's drive against
  /// the barrier's wait — see RendezvousScheduler::Config::max_interval_ms.)
  ///
  /// A TRUE HERE MEANS `interval_sec` UNDERSTATES THE DRIVE. Read
  /// `tour_interval_sec` on the same row for the uncapped tour term.
  ///
  /// ONLY MEANINGFUL ON A PROPOSER ROW, and only there because the wire carries
  /// the three integers and nothing else: a follower has no cap verdict to
  /// report and writes false. False on a follower is therefore "not known",
  /// not "the cap did not fire" — take the value from the proposer's row for
  /// the outage. (The same wire economy is why the derived fields of
  /// RendezvousPlan — this one, `penalty_mm`, the counts — carry sentinels on
  /// the node's `appointment_`, which is that struct reused as a carrier for
  /// the adopted triple rather than the output of a local solve.)
  bool   capped       = false;

  /// True when the LATTICE FLOOR set `interval_sec` — the agreed spacing is
  /// `rendezvous_interval_sec` (or this robot's marked-up drive, whichever is
  /// larger) rather than what the tours asked for.
  ///
  /// THE ONE COLUMN THAT SAYS WHICH TERM CHOSE THE TIMETABLE, and since
  /// generation 29 it is normally true: the campaign runs a 300 s lattice
  /// against tour terms measured in tens of seconds, so the objective's ask
  /// is almost always overruled. Do not reach for `capped` to answer this —
  /// that one asks whether the cap cut the TOUR term, and a cap sitting above
  /// the tours and below the floor reads false while still being overruled.
  /// `floored` false on a campaign row is the interesting case: something
  /// made a robot's drive longer than five minutes.
  ///
  /// Proposer only, like the rest of this block; false on a follower means
  /// "did not solve", not "the floor lost".
  bool   floored      = false;

  /// What the OBJECTIVE asked for, before the floor and the cap touched it —
  /// the furthest robot's marked-up drive to the winning cell. The quantity
  /// `interval_sec` used to be and, at a 300 s lattice, usually is not: with
  /// `floored` true these two differ by the whole policy margin, and it is
  /// this one that measures how far apart the team actually is.
  ///
  /// -1 where no solve produced it (a follower row, or any row predating
  /// generation 29), which is distinguishable from a real 0.
  double tour_interval_sec = -1.0;

  /// The objective's value at the winner, in the allocator's quantised unit.
  /// 0 means the meeting cost the team nothing — somebody was already driving
  /// there. A large value against a small candidate count is the signature of
  /// a scheduler with nothing good to choose from.
  ///
  /// PROPOSER ONLY. This block describes the argmin, and only fleet id 0 runs
  /// one. A follower writes -1 in every count below — a value no real search
  /// can produce, so it cannot be mistaken for "nothing was admissible", which
  /// is what the struct's natural 0 default would have said. Filter on
  /// `proposer_id == robot` before aggregating any of them.
  long long penalty_mm = -1;
  /// True when the scheduler's FLOOR cell won the argmin — the team agreed to
  /// meet somewhere no tour goes. The floor is the centroid of the team's own
  /// vehicle cells at proposal time, NOT the last-contact midpoint: that one
  /// was removed on 2026-09-16 and does not exist at proposal time at all, so
  /// any floor_won rate quoted from before then is measuring a different
  /// destination and does not carry over. Read together with `candidates`:
  /// the floor winning against ten candidates is a verdict, the floor winning
  /// against one is the absence of one. Proposer only; a follower writes false
  /// because no sentinel exists in a bool, so false here means "did not
  /// search" on any row where `proposer_id != robot`.
  bool floor_won = false;
  int  candidates = -1;
  int  rejected_unreachable = -1;
  int  rejected_excluded    = -1;

  /// This robot's own travel estimate to the winner, in mission-elapsed
  /// seconds, or -1 where its frozen cell graph contains no route to the cell.
  /// Deliberately per-robot and NOT expected to match the peer's.
  ///
  /// IT IS NOW A DIAGNOSTIC, NOT AN INPUT (schema 7). It used to feed a
  /// per-robot departure deadline — leave when 1.2x your own drive remains, so
  /// the far robot leaves first and the arrivals coincide — which was reported
  /// alongside it as `depart_sec`. That column is GONE rather than zeroed,
  /// because every robot now departs at `t_meet_sec` exactly — departure is a
  /// bare `now >= t_meet_ms` — and a second column holding the same number
  /// would be read as a second quantity. What travel_sec still answers is "how
  /// far was this robot from the meeting when it armed", which is the covariate
  /// the late/no-show outcomes want.
  double travel_sec  = -1.0;

  /// Non-empty when no appointment could be derived. Distinguishes "no meeting
  /// was scheduled" from "a meeting was scheduled and nothing came of it".
  std::string refused;

  // --- the agreement itself (gen 10) --------------------------------------
  //
  // Before gen 10 each robot DERIVED the appointment privately and the log had
  // no way to say whether the two ends had landed on the same one; the answer,
  // measured after the fact across ts3, was 21 of 64 pairs. These fields make
  // it a first-class, per-row fact rather than something reconstructed by
  // joining two files and hoping.

  /// Fleet id of the robot that derived the pair. Constant (0) by design.
  /// Compare it with the row's own robot to read the rest: on the proposer,
  /// `penalty_mm` / `floor_won` / `candidates` / `rejected_*` describe the
  /// argmin that chose the cell; on a follower they are sentinels, because a
  /// follower does not solve anything — it echoes.
  int proposer_id = -1;

  /// The appointment came from a pair the whole team was seen holding, rather
  /// than from a local solve. False in a refusal row.
  bool from_agreed = false;

  /// The committed triple was the CENTROID PLACEHOLDER: the argmin had exactly
  /// one admissible cell, because the allocator had not produced a tour when the
  /// team was last mutually whole, so the meeting point is where the team was
  /// standing rather than anywhere it chose. False in a refusal row.
  ///
  /// EVERY ROBOT WRITES THE TRUTH HERE, unlike `candidates` and the rest of the
  /// argmin block, which are proposer-only sentinels. The flag travels on the
  /// wire beside the three integers precisely so a follower can tell a permitted
  /// upgrade from a protocol violation, and that makes it the ONE thing a
  /// follower row can say about how the meeting point was chosen.
  ///
  /// WHY IT IS WORTH A COLUMN. A campaign in which every cell committed a
  /// placeholder is a valid rendezvous campaign that never exercised the
  /// scheduler's argmin — the arm measured "meet where we started", which is a
  /// different mechanism from the one the arm name implies, and before this
  /// column nothing in the banked data could distinguish the two. Aggregate it
  /// per cell before reading any meeting-point result.
  bool agreed_provisional = false;

  /// Mission-elapsed seconds of the SEPARATION this appointment is anchored
  /// on — the last instant the team read mutually complete on this robot.
  ///
  /// NO LONGER AN INPUT TO `t_meet_sec` (2026-09-17). It used to be the whole
  /// origin: t_meet was this plus the agreed interval, so two robots agreeing
  /// perfectly on the interval still disagreed on t_meet by exactly their
  /// disagreement about when contact was lost. That term was small at N=2
  /// (0.84 s measured) and large at N>=3 (21 s across three robots in the ts4
  /// cell), because the team coming apart is not one event when there is more
  /// than one link.
  ///
  /// WHAT REPLACED IT IS THE AGREED RECURRENCE, NOT A LOCAL COUNTDOWN. A
  /// countdown stood here briefly and this paragraph described it; since
  /// generation 23 t_meet_sec is nextAgreedOccurrence over the committed
  /// (t_meet_ms, interval_ms), floored since generation 25 at this robot's
  /// own BARE now (through v8 the floor added rendezvous_depart_delay_sec,
  /// and that per-robot term re-manufactured different meetings — the ts4
  /// N=3 fork in the v9 note). The anchor no longer shifts the meeting, and
  /// every robot arming before the agreed instant keeps it outright. So this
  /// column stays a DIAGNOSTIC and its spread across a cell's robots still
  /// measures the detection-spread error directly; what that error can now
  /// cost is waiting at the agreed cell, and a different meeting only when a
  /// robot arms after the instant has genuinely passed.
  ///
  /// -1.0 when this robot never saw mutual contact. That is now reachable on an
  /// ARMED row — arming stopped requiring an anchor when it stopped using one —
  /// so the sentinel is not confined to refusal rows.
  double anchor_sec = -1.0;

  /// Age of the agreement when it was armed: how long THIS EXACT TRIPLE had
  /// been standing, confirmed by the whole team, before the separation froze
  /// it. Stamped on change, not on re-confirmation, so it does not reset every
  /// heartbeat. A large value is not a fault — the triple is deliberately
  /// frozen at separation — but it bounds how stale the meeting cell can be.
  ///
  /// READ IT WITH `anchor_sec`, ALWAYS. This is `anchor_sec - (when the team
  /// committed)`, so a NEGATIVE value is meaningful and is the late-commit
  /// signature — the commit landed after the separation it is being measured
  /// against. But -1.0 is also what this carries when there is no anchor at
  /// all, and the two readings are indistinguishable in this column alone.
  /// `anchor_sec < 0` disambiguates them and is the only thing that does.
  double agreed_age_sec = -1.0;

  /// How many peers were holding the committed pair when it was committed.
  ///
  /// SCHEMA 7 CHANGED WHAT LOW VALUES MEAN. Through schema 6 this was
  /// structurally fleet-1 on every armed row, because the commit rule required
  /// all N-1 echoes, so it was a manipulation check rather than a variable and
  /// anything less meant the rule had been relaxed. It now depends on which
  /// generation the row committed, and BOTH readings are correct:
  ///
  ///   agreed_provisional = 1   still fleet-1 by construction. The placeholder
  ///                            is committed on the full echo round, which runs
  ///                            while the team is co-located and whole.
  ///   agreed_provisional = 0   anywhere in [0, fleet-1]. A final triple is
  ///                            terminal and authored by one robot, so it
  ///                            commits on first-hand evidence from the
  ///                            proposer; the echoes only say who ELSE heard it.
  ///
  /// So read it WITH `agreed_provisional` or not at all. A low value on a final
  /// row is the normal N>=3 shape, not a relaxed rule — see the commit rule in
  /// maintainRendezvousProposal for the N=3 split that produced the change.
  int peers_on_pair = 0;

  /// THIS ROBOT'S OWN ROUTE TO THE AGREED CELL, in metres over its own frozen
  /// cell graph, or -1 for "my map contains no route to it". Read only where
  /// `from_agreed` is true; on a refusal row nothing was evaluated.
  ///
  /// It exists because `travel_sec` above CANNOT express this. That estimate
  /// goes through GlobalAllocator::costMm, which deliberately falls back to
  /// straight-line centroid distance when the graph says unreachable — right
  /// for ranking a cell somebody will eventually clear, wrong for promising to
  /// stand in it at a particular minute. So `travel_sec` is always finite, and
  /// a robot that cannot get to the cell at all is invisible in it.
  ///
  /// That matters most on a FOLLOWER, which adopts the proposer's cell without
  /// solving anything and so never tests reachability. Its appointment is kept
  /// exactly — that is the point of the design — but if its own merge has no
  /// route there it will not arrive, and the outcome row it writes is a plain
  /// `no-show`, indistinguishable from a robot that simply ran out of time.
  /// This column separates the two.
  ///
  /// NOT a refusal condition, and deliberately: the cell graph blocks edges
  /// across UNKNOWN ground as well as occupied ground, so "no route" routinely
  /// means "I have not explored the corridor yet" rather than "it cannot be
  /// reached". Refusing on it would make followers abandon appointments they
  /// could keep, most often in exactly the far-apart, much-unexplored case the
  /// rendezvous arm exists to test. Measure it, do not act on it.
  double own_route_m = -1.0;
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
  ///   "run-ended"     this robot's run ended underneath the appointment
  ///                   (coverage latch teardown mid-drive or mid-wait). Added
  ///                   in generation 21; before it, these were counted as
  ///                   no-shows, which made the rendezvous arm's headline
  ///                   failure count out of its successes.
  ///   "unplaceable"   the agreed cell could not be turned into a position on
  ///                   any grid this robot holds, so appointmentPoint()
  ///                   returned the robot's own position and it never departed
  ///                   for anywhere. Added 2026-09-18 for the same reason as
  ///                   "run-ended": the own-position goal is reached on the
  ///                   next tick, so this used to read as an arrived no-show,
  ///                   i.e. as the PEERS failing to turn up. `arrived` is
  ///                   forced false on this outcome.
  ///
  /// THE LAST TWO SIT ABOVE THE arrived/unreachable SPLIT in the classifier,
  /// below "reconnected", because they answer a prior question: whether this
  /// robot reached the cell is only meaningful once there was a cell to reach
  /// and a run still going in which to reach it.
  std::string outcome;
  /// True when the robot reached the cell at all, whatever the outcome. A
  /// no-show with `arrived` false is a navigation failure wearing a
  /// coordination failure's name.
  bool arrived = false;
  /// Seconds spent AT the cell, measured from the arrival stamp to the close.
  ///
  /// ZERO IS NOT A READING OF ZERO. The one writer (closeAppointment) computes
  /// it only when this robot actually arrived and has an arrival stamp, and
  /// passes a flat 0.0 otherwise — the "superseded" close passes 0.0
  /// unconditionally. So 0.0 means "never waited at all, usually because it
  /// never got there", and it is `arrived` that tells the two apart. Filter on
  /// `arrived` before averaging this column or the no-shows will drag the mean
  /// toward zero.
  ///
  /// The -1.0 initialiser is a struct default that no emitted row carries;
  /// do not read a -1 as a sentinel the writer chose.
  double waited_sec = -1.0;
  /// Was the reunion WHOLE-TEAM DIRECT contact (every pair in range), as
  /// opposed to merely complete (every peer reachable, relays counted)?
  ///
  /// SEPARATE FROM `outcome` SINCE SCHEMA 7 (2026-09-18) and the separation is
  /// the point. `outcome` names what the barrier acted on, because the barrier
  /// is what ended the manoeuvre; this names the stricter condition, which is
  /// the one an analyst wants when asking whether the maps could actually merge
  /// pairwise. They were the SAME field until schema 6, and at N>=3 they
  /// disagree: a robot released by a relayed reunion recorded "no-show" for the
  /// meeting that had just reconnected it. Reading `outcome` alone on a
  /// schema-6 N>=3 file therefore counts successful meetings as failures --
  /// those files are not repairable by re-reading, because only one of the two
  /// facts was ever written.
  ///
  /// At N=2 the two coincide by construction (one pair, no relays) and this is
  /// true on exactly the "reconnected" rows.
  bool mutual = false;
};

/// One rung of the escape ladder that guards the drive to an agreed cell
/// (appointmentLegWatchdog / resumeAppointmentDrive). The homing ladder's
/// `home_watchdog` is the direct analogue and the kind/cause split is its
/// convention, not a new one; the two are kept apart because `dist_home_m` and
/// `next_mode` have no meaning here and overloading them would silently change
/// what a home_watchdog count measures.
///
/// A ROW IS NOT A REACHABILITY VERDICT. simple_nav_3d has no failure-reporting
/// channel, so `cause` names the only two things the planner is ever told, and
/// both are equally consistent with an unreachable cell and with a platform
/// wedged against one trunk on an open approach. Count rungs, not verdicts.
struct AppointmentLegEvent {
  /// Which rung. One of:
  ///   "escape"     a watchdog ended the drive and a detour was dispatched.
  ///                `escapes_used` is POST-increment, so the first row reads 1.
  ///   "escape-end" the detour finished and the cell is the goal again. NOT a
  ///                rung being spent; do not pool these with "escape" when
  ///                counting how much ladder a leg used.
  ///   "unreached"  the ladder was spent and the leg was abandoned to PLAN.
  ///                One per give-up, and the row the livelock check keys on:
  ///                a robot alternating "unreached" with fresh departures and
  ///                never exploring in between is the failure mode this event
  ///                was added to make visible.
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
  /// On "unreached", the occurrence the appointment was rolled to, mission
  /// elapsed. -1.0 means there was no record left to roll, so the robot simply
  /// went back to exploring — the two are different mechanisms and only this
  /// column separates them. -1.0 on every other kind for the same reason
  /// `leg_sec` is.
  double rolled_to_sec = -1.0;
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
  /// The 2026-09 binary window (generation 9, ~/hmr_campaign/CODE_TODO.md)
  /// amended v4 a third time, again with FIELDS only and on exactly the P6
  /// precedent above — kEventKinds is byte-for-byte unchanged, no field moved,
  /// nothing was removed, so the stamp does not move:
  ///
  ///     AllocationEvent::alloc_hash    R3 — the "same problem" join key
  ///     AllocationEvent::edge_hash     R3 — its traversability half
  ///     MissionCompleteEvent::occurrence  T5 — duplicate-endpoint detector
  ///
  /// Note what this means for a reader: a generation-9 file and a generation-8
  /// file BOTH stamp 4, so the stamp cannot separate them and must not be asked
  /// to. Generation is read from run_manifest.txt (tx/scenario/git rev), never
  /// from this constant — the stamp answers "can my parser read this file", not
  /// "may I pool these numbers", and generation 9 changes planner BEHAVIOUR
  /// (D1's 360-degree FOV, R1's terminality fixes, R6's bounded dwell), so
  /// pooling it with generation 8 is wrong however well the parse goes.
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
  ///
  /// v5: generation 10, the exact-rendezvous change. kEventKinds is unchanged —
  /// no kind was added or removed — so this is NOT a vocabulary widening, and on
  /// the P6 / generation-9 precedent the additive half would not have moved the
  /// stamp. One REMOVAL does:
  ///
  ///   - `RendezvousAgreedEvent::excluded` REMOVED. Its source state (the
  ///     node's per-outage no-show list) no longer exists: the appointment is
  ///     now a pair agreed with the whole team BEFORE the separation and frozen
  ///     for its duration, so there is no second arming inside an outage for a
  ///     write-off to influence, and feeding a PRIVATE exclusion list into the
  ///     derivation is precisely the class of private input that made gen 9's
  ///     appointments disagree. Removed rather than left writing "" forever,
  ///     which would read as "nothing has been written off yet" — a live
  ///     mechanism with nothing to report — instead of "the mechanism is gone".
  ///     This is the v3 `last_contact_age_sec` shape exactly: a reader that
  ///     keys on it now gets a KeyError rather than a plausible wrong answer.
  ///
  /// Additive in the same generation, on RendezvousAgreedEvent: `proposer_id`,
  /// `from_agreed`, `anchor_sec`, `agreed_age_sec`, `peers_on_pair`.
  ///
  /// Two v4 fields survive but are STRUCTURALLY INERT AS OF v5, and are kept
  /// only because they are faithful mirrors of RendezvousPlan rather than node
  /// state that has to be maintained: `capped` is false on every row (the
  /// divergence cap is fed by a pairwise last-contact rate model that does not
  /// exist at proposal time, when the team is connected, and does not
  /// generalise past N=2 anyway), and `rejected_excluded` is 0 on every row
  /// (the node passes an empty exclude set). Do not read either as evidence
  /// about a v5 run; see ~/hmr_campaign/CODE_TODO.md, generation 10.
  ///
  /// `capped` IS LIVE AGAIN IN v6 and this paragraph no longer describes it —
  /// see the field's own doc. It stayed false for a second reason the v5 text
  /// did not know about: armAppointment copied five of RendezvousPlan's eight
  /// diagnostics into the struct the row is logged from and silently dropped
  /// `capped`, `floored` and `tour_interval_ms`, so even after the cap was
  /// rewired to the findability bound the column could not have moved. Both
  /// halves of `capped` were fixed in generation 17; the other two were copied
  /// into the struct then but reached no column until generation 29 added
  /// `floored` and `tour_interval_sec` to this event, so any row older than
  /// that simply does not carry them. `rejected_excluded` is still inert.
  ///
  /// v6: generation 17, the agreed-triple change. kEventKinds is unchanged and
  /// nothing was removed, so on the v5 reasoning this would not normally move
  /// the stamp — the additive half alone does not. What moves it is that a v5
  /// column CHANGED MEANING, which is the one thing a reader cannot detect:
  ///
  ///   - `RendezvousAgreedEvent::t_meet_sec` was, in v5, this robot's own
  ///     mission clock at (its private view of the separation) + the agreed
  ///     interval. It is now the AGREED instant, adopted verbatim off the wire,
  ///     and it recurs. The column name, type and units are identical and the
  ///     values are plausible under both readings, so a v5 analysis script run
  ///     against a v6 file produces numbers rather than an error — and the
  ///     numbers mean something else. That is the `checks-that-stopped-checking`
  ///     shape and the stamp is the only thing that can refuse it.
  ///
  /// Additive in the same generation, on RendezvousAgreedEvent:
  /// `agreed_provisional`. TeamWorld gained `rendezvous_provisional` and split
  /// `rendezvous_time_sec` into `rendezvous_interval_ms` + `rendezvous_t_meet_ms`
  /// in the same change, so v5 and v6 binaries cannot share a bus either — do
  /// not pool a v6 campaign with any earlier one.
  /// v7: generation 19 — the rendezvous arm stops being a recurring schedule.
  /// Two changes, and the FIRST is a meaning change with no field movement,
  /// which is the shape that gets read wrong:
  ///
  ///   - `rendezvous_outcome.outcome` now names what the BARRIER acted on
  ///     (teamComplete over live peers) rather than whole-team direct contact.
  ///     No field moved and no name changed, so a v6-era reader parses a v7
  ///     file without error and silently compares two different quantities.
  ///     Worse in the other direction: on v6 N>=3 files the label is simply
  ///     WRONG — a relayed reunion was recorded as "no-show" — and those files
  ///     cannot be repaired by re-reading, because the barrier's own verdict
  ///     was never written down. Do not pool v6 and v7 rendezvous outcomes.
  ///   - additive: `rendezvous_outcome.mutual` carries the strict predicate
  ///     that `outcome` used to carry, so both facts now exist in one row.
  ///
  /// Also v7, and behavioural rather than schematic: `rendezvous_interval_ms`
  /// and `rendezvous_t_meet_ms` are still agreed, echoed and logged, but
  /// NOTHING READS THEM FOR TIMING any more. A v7 robot departs
  /// `rendezvous_depart_delay_sec` after the team reads incomplete and waits at
  /// the cell until the team is whole. An analysis that derives meeting times
  /// from the interval is describing a schedule the binary no longer keeps.
  ///
  /// v8: generation 23 (2026-09-18). One MEANING change and several additive
  /// fields, and as always the meaning change is the one that forces the stamp:
  ///
  ///   - `rendezvous_outcome.outcome` gains "unplaceable", and it is carved out
  ///     of what v7 recorded as "no-show". A robot that could not place the
  ///     agreed cell on any grid it holds had appointmentPoint() hand it its
  ///     OWN position, reached that goal on the next tick, and so logged
  ///     `no-show arrived=true` — the peers blamed for a departure that never
  ///     happened. On v8 that row reads `unplaceable arrived=false`. No column
  ///     moved, both files parse, and a v7-era no-show tally over a v8 file is
  ///     comparing two different populations. Same shape as v7's own note, one
  ///     branch over. Do not pool v7 and v8 rendezvous outcomes.
  ///
  ///   - `mission_complete.homing_duration_sec` becomes WALL time on the leg.
  ///     v7 derived it from `state_enter_time_`, which the proximity-hold
  ///     release backdates on purpose so the homing BUDGET continues across a
  ///     hold — so v7's duration excluded held time while the
  ///     `homing_distance_m` beside it did not, and distance/duration
  ///     overstated homing speed by the held fraction. That bias is
  ///     arm-correlated: robots that regroup arrive home together, and the
  ///     5.0 m hold radius exceeds the ~3 m between homes. v8 measures both
  ///     from the same instant and reports the held time separately, so
  ///     `homing_duration_sec - homing_held_sec` recovers the v7 quantity
  ///     exactly. Do not pool v7 and v8 homing durations.
  ///
  ///   - `team_exchange.last_known_age_sec` becomes the silence this message
  ///     ended, read before the sender is believed. v7 read it after, so it
  ///     was t - t and printed 0.0 on all 757,677 banked rows. This is the
  ///     mildest of the three meaning changes to state and the sharpest to
  ///     act on: the old column supports no conclusion at all, so anything
  ///     resting on it is resting on nothing. Do not pool v7 and v8 here.
  ///
  /// Additive in v8, none of which change an existing column:
  ///   - `run_end.midrun_attempts_used` — how much of the mid-run dispatch cap
  ///     the cell spent. See the field doc: the cap is a dose term and this is
  ///     the only place it is visible.
  ///   - `mission_complete.homing_held_sec` — the held component carved out of
  ///     the duration above, and the only way to tell a leg that legitimately
  ///     outlasted `mission_return_max_sec` from one that overran it.
  ///   - `goal_amnesty.source` — which suppression list granted the reprieve.
  ///     Absent on a v7 file, where only one list could, so a missing key
  ///     reads "failed" and never "unknown". See the field doc.
  ///   - step CSV `plan_rej_visited`, appended LAST in the header, not next to
  ///     `plan_rej_blacklist` where it belongs by meaning — inserting it there
  ///     would shift FIVE columns under every positional reader of every
  ///     banked run: `plan_rej_minpos`, `plan_stall_ticks`, `pursue_peer`,
  ///     `pursue_quarry_live`, `team_complete`. (This said "eight" until
  ///     2026-09-18; MetricsLogger::writeHeader is the only authority on the
  ///     order, and five is what lies between the two names above. One would
  ///     have been enough of an argument.) It is a SUBSET of
  ///     `plan_rej_blacklist`, so the five original rejection columns still
  ///     sum the way they always did.
  ///
  /// ALSO v8 AND BEHAVIOURAL on the planning side: the starvation amnesty now
  /// runs in two tiers. v7 pushed only failed-goal rejections onto the
  /// suppressed pool, so a tick whose every candidate fell inside
  /// `visited_goal_radius_m` of somewhere reached in the last
  /// `visited_goal_ttl_sec` found an empty pool, skipped the amnesty and
  /// returned starved — at 10 Hz, up to ~1200 dead ticks waiting out a TTL,
  /// and the campaign config turns that suppression ON (the compiled default
  /// is off). v8 keeps a second pool and consumes it only when the failed tier
  /// has nothing to offer, so every tick that used to produce a goal still
  /// produces the SAME goal and only the ticks that used to stall change.
  /// Rows with `source="visited"` are exactly those recovered ticks.
  ///
  /// ALSO v8 AND BEHAVIOURAL, i.e. no schema surface at all but a different
  /// binary underneath: generation 23 changed the peer-liveness predicate that
  /// nearly every reconnection decision reads (`accountedPeerCount` unions the
  /// claim table, TeamWorld `direct` and `finished`, where v7 read the beacon
  /// alone), the reconnect gate's value/cost pairing at N>=3, the mid-run
  /// trigger's bookkeeping after a declined dispatch, and what rides on
  /// `TeamWorld.my_tour` (v7 kept broadcasting the last allocator solve while
  /// the robot was manoeuvring, homing or finished — a route it had abandoned,
  /// arriving with no age field and anchored at the sender's live pose; v8
  /// sends an empty tour outside the exploration loop, which the receive path
  /// already treats as "no route" and degrades to the trail chase). None of
  /// that renames a column; all of it changes what the columns describe. Do
  /// not pool a generation-23 campaign with any earlier one on any
  /// reconnection metric.
  ///
  /// v9: generation 25 (2026-09-18). No column moves; two meanings do, and one
  /// vocabulary widens:
  ///
  ///   - `rendezvous_agreed.t_meet_sec` is floored at BARE `t_now_sec`
  ///     (nextAgreedOccurrence), not `t_now_sec +
  ///     rendezvous_depart_delay_sec`: the per-robot notice re-applied a
  ///     shared lead time per arming, and the sum forked the ts4 N=3 cell
  ///     across two occurrences. On v9, `t_meet_sec - t_now_sec` lands in
  ///     [0, `interval_sec`) — a v8 reader checking the old
  ///     [delay, delay + interval) window will fail every honest v9 row — and
  ///     rows across one cell's robots are EXPECTED equal unless a robot armed
  ///     after the instant had genuinely passed. `rendezvous_depart_delay_sec`
  ///     still appears in the param rows but is inert; treat it as schema
  ///     ballast, not a dose term.
  ///   - `state_change` gains reason `return-team-settled` (RETURN_NAV ->
  ///     RETURN_SYNC): an appointment walker whose team has settled for the
  ///     confirm window joins the barrier from where it stands instead of
  ///     driving out the last metres. Consequently `rendezvous_outcome`
  ///     rows with `arrived=false` split into two populations — team-settled
  ///     conversions and budget/watchdog strandings — distinguished by the
  ///     preceding state_change reason, and an `arrived=false` reunion is no
  ///     longer evidence of a navigation failure. Do not pool v8 and v9
  ///     arrival rates.
  ///
  /// v10: generation 29 (2026-09-20). One column is added and two meanings move
  /// under it, both on `rendezvous_agreed`:
  ///
  ///   - `agreed_base_sec` is NEW: the committed `t_meet` this row rolled from.
  ///     It is the generation key. A run now holds one agreement per meeting
  ///     kept rather than at most two for the whole run, because the team
  ///     re-agrees the next place and time before it resumes exploring, and
  ///     successive bases of one cell share no phase. Every cross-robot fold
  ///     groups on this column FIRST; a v9 reader recovering the base as the
  ///     smallest `t_meet_sec` in the cell reports the phase step between two
  ///     healthy generations as a lattice violation.
  ///   - `t_meet_sec` is floored at `t_now_sec` plus this robot's arrival
  ///     SHORTFALL, not at bare `t_now_sec`: a robot whose drive to the cell
  ///     cannot get it there within `rendezvous_max_lateness_sec` rolls to a
  ///     rung it can keep instead of agreeing to arrive late
  ///     (arrivalShortfallSec). So the lead lands in [shortfall, shortfall +
  ///     `interval_sec`) and the v9 upper edge fails every honest roll. The
  ///     shortfall is not logged; check the lower edge, which is unchanged.
  ///   - ALSO BEHAVIOURAL, with no schema surface: `t_meet_sec` is an ARRIVAL
  ///     instant rather than a departure one, and the barrier holds until the
  ///     whole team is present rather than until a wait expires. Rendezvous and
  ///     hybrid timings do not pool across the v9/v10 line on any metric.
  ///
  /// v11: generation 31 (2026-09-21). A VOCABULARY WIDENING, which is what
  /// moves the stamp on the v3 home_watchdog precedent — nothing moved and
  /// nothing was removed, so the additive half alone would not have:
  ///
  ///   - `appointment_leg` is NEW (AppointmentLegEvent). The drive to an agreed
  ///     cell now escalates through an escape ladder instead of falling through
  ///     to the barrier, and every rung of that ladder is a row. Before it the
  ///     ladder was RCLCPP_WARN-only, which is the same as unmeasured: a robot
  ///     cycling give-up -> roll -> re-depart was indistinguishable in the
  ///     tables from one making ordinary dispatches, and escapes could not be
  ///     counted per arm at all.
  ///
  /// A v10 reader is otherwise unaffected — every v10 column means exactly what
  /// it meant — but one with an exhaustive match on `event` now hits an
  /// unhandled case, which is the reason vocabulary widening bumps the stamp at
  /// all. The kind is emitted only under an armed appointment, so a run at the
  /// shipped defaults emits exactly the v10 set and the per-phase equivalence
  /// gate is unmoved.
  ///
  /// v12: FIRST STAMPED BY THE GENERATION 33 BINARY, and it covers the
  /// generation 32 behaviour change as well — gen 32 shipped and ran while the
  /// stamp still said 11, so 11 spans two behaviours and v12 is where the fold
  /// can start refusing to pool. No kind moved. Two columns were ADDED, both on
  /// `team_exchange` and both additive: `acquire_sec` and `held_sec` (see
  /// TeamExchangeEvent), negative on every row that carries no transition. A
  /// v11 reader ignores them; a v12 reader must not assume they are present in
  /// an 11.
  ///
  /// What moved BEHAVIOURALLY across this line is two generations' worth, and
  /// either half alone would justify the refusal.
  ///
  /// From generation 32, the population behind `rendezvous_outcome.outcome`
  /// moved far enough that pooling the two vintages would average two different
  /// behaviours under one name:
  ///
  ///   - A robot that finishes exploring with a standing appointment now KEEPS
  ///     it (keepAppointmentOnFinish) instead of closing it and driving home.
  ///     `superseded` was the modal outcome of a finished robot and is now
  ///     close to absent, replaced by `reconnected` and `run-ended`; the
  ///     mirror-image change is that its peers stop recording the no-shows
  ///     those abandonments caused. Every outcome-mix statistic — and any
  ///     completion time drawn from a cell where one robot waited a barrier
  ///     out that a v11 binary would have abandoned — is a different quantity
  ///     across this line.
  ///   - `appointment_leg` rows appear for the first time in practice. The kind
  ///     is v11's, but no v11 cell ever emitted one: the ladder is only reached
  ///     from a drive to the agreed cell, and v11 had no such drive left to
  ///     make once exploration was over.
  ///
  /// From generation 33, the allocator stops reserving frontier work for a
  /// peer that has turned for home (`off_frontier`), peers carry a `mode` on
  /// the wire, and the at-the-rendezvous hold can be released by the measured
  /// per-peer voxel arrival instead of a fixed settle timer. The last of those
  /// is off at the shipped defaults; the first two are not, and they change
  /// who explores what.
  ///
  /// A v11 reader parses every row correctly and needs no change. The stamp
  /// moves so that a fold can REFUSE to pool, which no additive-compatibility
  /// argument would have let it do.
  static constexpr int kSchemaVersion = 12;

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
  /// Duplicate `run_end` attempts that were suppressed. Exposed rather than
  /// left internal because the warning it also raises goes to the ROS log,
  /// which is prose a test cannot assert on and a script should not parse; this
  /// is the same count as a value. Non-zero means the node reached a terminal
  /// state twice and this run's endpoint metrics are suspect.
  long long dupRunEnds() const { return dup_run_ends_; }
  /// Duplicate `mission_complete` attempts that were suppressed. Same contract
  /// as dupRunEnds(), and unlike that one it is ALSO written into the run_end
  /// row (`mission_completes_suppressed`), because a mission_complete duplicate
  /// necessarily arrives before run_end and so can still be reported in the
  /// file rather than only in prose. Non-zero means something asked to end the
  /// mission twice; the row count alone can no longer show that, which is
  /// exactly why the counter exists.
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
  //
  // THE COUNT IS PART OF THE CONTRACT, NOT A COURTESY. A new emitter that
  // writes `if (!open_ || !started_) return;` without the increment does not
  // merely fail to report — it makes the reported number say "nothing was
  // lost" on a run that lost events, which is worse than no counter at all.
  // Six of the v4 emitters (cell_census, team_exchange, allocation,
  // reconnect_gate, rendezvous_agreed, rendezvous_outcome) shipped exactly
  // that way and were fixed on 2026-09-18. The one deliberate exemption is the
  // coverage-sample updater, which emits no event and carries its own note.
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
  /// A suppression list rejected EVERY candidate and the planner re-attempted
  /// one anyway — non-retired first, least-recently-stamped within that.
  /// Expected to be absent in a healthy run; each occurrence is a measurement
  /// of how close suppression came to starving the planner, and `retired` says
  /// whether only confirmed traps were left to pick from.
  ///
  /// `source` names WHICH list, and is v8. Two tiers run, in this order:
  ///   "failed"   the failed-goal blacklist. The only tier that existed
  ///              through v7, so an absent key on an old file means this one —
  ///              do not read a missing `source` as unknown.
  ///   "visited"  the recently-visited suppression (visited_goal_radius_m /
  ///              visited_goal_ttl_sec). Reached ONLY when the failed tier had
  ///              nothing to offer, i.e. on ticks that before v8 returned
  ///              starved and drove the robot nowhere. A run with rows of this
  ///              source is a run that used to stall for up to a full TTL.
  /// `last_fail_age_sec` is dated against `source`'s own list, so it is an
  /// age-since-visit on the visited tier, not an age-since-failure. `retired`
  /// is structurally false there: visited_goals_ never retires a site.
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
  /// Suppressed duplicate run_end attempts. Not logged in any row -- the row
  /// is already written by the time the first one arrives, and a trailing
  /// event would break both "run_end is the last line" and "the last line's
  /// seq is events_written - 1". Reported instead by the destructor, which is
  /// the last moment the total is known, into the ROS log the harness banks as
  /// planner_<robot>.log; gate_g8 check 3n is the reader.
  long long dup_run_ends_ = 0;
  /// The mission_complete idempotence latch, same shape as run_end_written_
  /// above. The node has its own once-per-run guard on the BEHAVIOUR (see
  /// ExploPlannerNode::mission_return_done_); this one guards the FILE, so the
  /// "exactly one mission_complete per robot-run" contract that every endpoint
  /// analysis reads holds even if some future path reaches the emitter twice.
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
  /// Has a `step` event arrived since the most recent exploration_complete?
  /// This is the clearable half of the post_latch test in noteCoverage: the
  /// two declaration stamps above are one-way (first_) or monotone (last_) and
  /// neither can say that the robot went BACK to exploring in between, which a
  /// reconnect manoeuvre delivering a merged map routinely causes. Set by
  /// logStep, cleared by logExplorationComplete.
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
