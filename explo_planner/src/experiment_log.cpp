#include "explo_planner/experiment_log.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <utility>

#include <rclcpp/logging.hpp>

namespace explo_planner {

namespace {

/// Wall-clock epoch seconds, for the SECONDARY `t_wall_sec` field only. Never
/// used as a stamp anyone joins on — that is the whole point of this file (see
/// failure 1 in the header). std::chrono::system_clock rather than the node
/// clock, so it stays wall time even under use_sim_time.
double wallEpochSec() {
  const auto d = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(d).count();
}

/// %.6f of any finite double fits comfortably: DBL_MAX is 309 integer digits,
/// plus sign, point, 6 decimals and the terminator.
constexpr size_t kNumBuf = 344;

}  // namespace

ExperimentLog::ExperimentLog(const std::string& path, std::string robot_id,
                             rclcpp::Logger logger)
    : file_(path, std::ios::out | std::ios::trunc | std::ios::binary),
      path_(path),
      robot_id_(std::move(robot_id)),
      logger_(std::move(logger)) {
  // Binary mode is not cosmetic: it is what guarantees the LF-only endings the
  // header promises, on any platform whose text mode would translate them.
  if (!file_.is_open()) {
    RCLCPP_ERROR(logger_,
        "ExperimentLog: cannot open '%s': %s. The run continues and the "
        "per-step CSV is unaffected, but this run has NO event log — its "
        "coverage milestones and reconnect events are unrecoverable.",
        path_.c_str(), std::strerror(errno));
    return;
  }
  open_ = true;
  healthy_ = true;
  line_.reserve(1024);
}

ExperimentLog::~ExperimentLog() {
  // No run_end here: it needs run totals only the node can supply, and a
  // destructor that logged a half-known ending would be worse than the missing
  // line an analysis can already detect from `seq` (see the header).
  //
  // dup_run_ends_ IS reported here, and this is the only place it can be. The
  // suppression that produces it necessarily happens after the run_end row is
  // on disk, so unlike dup_mission_completes_ it cannot ride out on a JSON
  // field; and appending a trailing event instead would break two invariants
  // the readers depend on -- run_end is the last line, and the last line's seq
  // is events_written - 1. So it goes to the ROS log, which the harness banks
  // as planner_<robot>.log, where gate_g8 check 3n greps for it. The header's
  // promise that this counter is "kept so a future ... destructor message can
  // report it" is what this is.
  //
  // Only the total: the first duplicate already raised its own warning naming
  // both instants. What this adds is MULTIPLICITY, which that one-shot warning
  // deliberately does not carry.
  if (dup_run_ends_ > 0) {
    RCLCPP_ERROR(logger_,
        "ExperimentLog closing '%s' with %lld SUPPRESSED duplicate run_end "
        "attempt(s). The node reached a terminal state %lld times; the file "
        "holds the FIRST ending only, so this run's endpoint metrics describe "
        "one of them and the run is not scoreable as it stands.",
        path_.c_str(), dup_run_ends_, dup_run_ends_ + 1);
  }
  if (file_.is_open()) file_.close();
}

// ==================================================================
// Line assembly
// ==================================================================

void ExperimentLog::key(const char* name) {
  if (!first_field_) line_ += ',';
  first_field_ = false;
  line_ += '"';
  line_ += name;
  line_ += "\":";
}

void ExperimentLog::appendNumber(const char* name_for_flag, double v) {
  if (!std::isfinite(v)) {
    // NEVER a bare nan/inf token: no strict JSON parser accepts one, so a
    // single NaN would cost the analysis the whole line (or, worse, be
    // silently coerced by a lenient one). Null it and say which key it was.
    line_ += "null";
    nonfinite_.push_back(name_for_flag);
    return;
  }
  char buf[kNumBuf];
  std::snprintf(buf, sizeof(buf), "%.*f", kDecimals, v);
  line_ += buf;
}

void ExperimentLog::num(const char* name, double v) {
  key(name);
  appendNumber(name, v);
}

void ExperimentLog::integer(const char* name, long long v) {
  key(name);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%lld", v);
  line_ += buf;
}

void ExperimentLog::boolean(const char* name, bool v) {
  key(name);
  line_ += v ? "true" : "false";
}

void ExperimentLog::appendEscaped(const std::string& s) {
  line_ += '"';
  for (const char c : s) {
    switch (c) {
      case '"':  line_ += "\\\""; break;
      case '\\': line_ += "\\\\"; break;
      case '\n': line_ += "\\n";  break;
      case '\r': line_ += "\\r";  break;
      case '\t': line_ += "\\t";  break;
      default:
        // Control characters must be escaped; everything else (including every
        // continuation byte of a UTF-8 sequence, which is >= 0x80 and therefore
        // negative as a signed char) passes through byte-exact.
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x",
                        static_cast<unsigned>(static_cast<unsigned char>(c)));
          line_ += buf;
        } else {
          line_ += c;
        }
    }
  }
  line_ += '"';
}

void ExperimentLog::text(const char* name, const std::string& v) {
  key(name);
  appendEscaped(v);
}

void ExperimentLog::text(const char* name, const char* v) {
  key(name);
  appendEscaped(v == nullptr ? std::string() : std::string(v));
}

void ExperimentLog::teamCounts(int peers_live, int expected_peers) {
  integer("peers_live", peers_live);
  integer("expected_peers", expected_peers);
}

void ExperimentLog::begin(const char* event, const ExperimentContext& ctx) {
  line_.clear();
  nonfinite_.clear();
  first_field_ = true;
  line_ += '{';
  text("event", event);
  // Monotonic from 0 with no gaps. This is what makes truncation detectable
  // from the file's own contents: the last line's seq must be
  // events_written - 1 as reported by run_end.
  integer("seq", seq_++);
  text("robot", robot_id_);
  // THE stamp. Absolute simulation seconds on the node's own clock.
  num("t_sim_sec", ctx.sim_time_sec);
  // Seconds since run_start, precomputed so no consumer has to find t0 first.
  //
  // SINGLE-ROBOT AXIS ONLY. t0 is this node's own first tick, and the two
  // planners are launched back-to-back with no sleep, so their t0 values differ
  // — measured at up to 4.93 sim s. Differencing one robot's t_rel_sec against
  // the other's silently inherits that skew. Anything cross-robot (team
  // completion, which robot reached a milestone first, event ordering) must use
  // t_sim_sec, which is the same clock for both. run_start records this node's
  // t0_sim_sec so the offset is recoverable rather than merely warned about.
  num("t_rel_sec", ctx.sim_time_sec - t0_sec_);
  // Secondary, and named so it cannot be mistaken for the primary axis.
  num("t_wall_sec", wallEpochSec());
  text("state", ctx.state);
  integer("step", ctx.step);
}

void ExperimentLog::end() {
  if (!nonfinite_.empty()) {
    key("nonfinite_fields");
    line_ += '[';
    for (size_t i = 0; i < nonfinite_.size(); ++i) {
      if (i) line_ += ',';
      appendEscaped(nonfinite_[i]);
    }
    line_ += ']';
  }
  line_ += "}\n";

  file_.write(line_.data(), static_cast<std::streamsize>(line_.size()));
  file_.flush();
  // Checked after EVERY write, which is the entire point (failure 4). A stream
  // that failed once stays failed until cleared, so clear it: a transient
  // condition (a full disk that is freed, an NFS hiccup) should not cost the
  // rest of the run's events. `healthy_` never comes back, so the run is still
  // marked suspect in run_end whatever happens afterwards.
  if (!file_.good()) {
    ++write_failures_;
    if (!write_error_reported_) {
      write_error_reported_ = true;
      healthy_ = false;
      RCLCPP_ERROR(logger_,
          "ExperimentLog: write to '%s' FAILED (%s) at event seq %lld. The "
          "event log for this run is incomplete from here on; run_end will "
          "report logger_healthy=false. Every later line is best-effort.",
          path_.c_str(), std::strerror(errno), seq_ - 1);
    }
    file_.clear();
  }
}

// ==================================================================
// Run parameters + run_start
// ==================================================================

void ExperimentLog::addParamNum(const std::string& name, double value) {
  if (!open_) return;
  char buf[kNumBuf];
  if (std::isfinite(value)) {
    std::snprintf(buf, sizeof(buf), "%.*f", kDecimals, value);
    params_.emplace_back(name, buf);
  } else {
    params_.emplace_back(name, "null");
  }
}

void ExperimentLog::addParamBool(const std::string& name, bool value) {
  if (!open_) return;
  params_.emplace_back(name, value ? "true" : "false");
}

void ExperimentLog::addParamStr(const std::string& name,
                                const std::string& value) {
  if (!open_) return;
  // Pre-encode through the same escaper the event fields use, so a parameter
  // holding a topic name with a quote in it cannot corrupt the run_start line.
  const std::string saved = std::move(line_);
  line_.clear();
  appendEscaped(value);
  params_.emplace_back(name, line_);
  line_ = saved;
}

void ExperimentLog::startRun(const ExperimentContext& ctx,
                             const std::vector<double>& milestones) {
  if (!open_ || started_) return;

  // Ladder hygiene: descending, unique, and inside (0, 1). A ladder that is not
  // strictly descending would make "the first crossing" ambiguous, and a rung
  // outside the range can never fire but would still appear in run_start as if
  // the run had simply not reached it.
  milestones_.clear();
  for (const double m : milestones) {
    if (!std::isfinite(m) || m <= 0.0 || m >= 1.0) continue;
    milestones_.push_back(m);
  }
  std::sort(milestones_.begin(), milestones_.end(), std::greater<double>());
  milestones_.erase(std::unique(milestones_.begin(), milestones_.end()),
                    milestones_.end());
  milestone_hit_.assign(milestones_.size(), false);

  t0_sec_ = ctx.sim_time_sec;
  wall0_sec_ = wallEpochSec();
  prev_anchor_sim_sec_  = t0_sec_;
  prev_anchor_wall_sec_ = wall0_sec_;
  started_ = true;

  begin("run_start", ctx);
  integer("schema_version", kSchemaVersion);
  // The anchor, stated explicitly as well as through t_sim_sec: every consumer
  // computes run-relative time as (t_sim_sec - t0_sim_sec) with no wall-clock
  // mapping anywhere in the chain.
  num("t0_sim_sec", t0_sec_);
  key("coverage_milestones");
  line_ += '[';
  for (size_t i = 0; i < milestones_.size(); ++i) {
    if (i) line_ += ',';
    appendNumber("coverage_milestones", milestones_[i]);
  }
  line_ += ']';
  key("params");
  line_ += '{';
  for (size_t i = 0; i < params_.size(); ++i) {
    if (i) line_ += ',';
    line_ += '"';
    line_ += params_[i].first;
    line_ += "\":";
    line_ += params_[i].second;
  }
  line_ += '}';
  end();

  params_.clear();
  params_.shrink_to_fit();  // never used again; do not hold the run's memory
}

// ==================================================================
// Events
// ==================================================================

void ExperimentLog::logStep(const ExperimentContext& ctx, const StepEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  // A step after a declaration means exploration RESUMED. See the post_latch
  // block in noteCoverage for why this is the signal and why it is safe on the
  // declaring tick. Set before the early return-free body so it holds for every
  // step, and left alone before the first declaration where it means nothing.
  if (explore_done_last_sec_ >= 0.0) explore_resumed_since_done_ = true;
  begin("step", ctx);
  num("unknown_fraction", e.unknown_fraction);
  text("coverage_source", e.coverage_source);
  integer("observed_voxels", e.observed_voxels);
  integer("frontier_voxels", e.frontier_voxels);
  num("distance_m", e.distance_m);
  // The run's only surviving trajectory (see StepEvent) — the CSV has no pose
  // columns and the campaign bags nothing.
  num("x", e.x);
  num("y", e.y);
  num("z", e.z);
  num("yaw", e.yaw);
  text("phase", e.phase);
  integer("peers_live", e.peers_live);
  num("plan_time_ms", e.plan_time_ms);
  end();
}

void ExperimentLog::logClockAnchor(const ExperimentContext& ctx) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  const double wall = wallEpochSec();
  const double d_sim  = ctx.sim_time_sec - prev_anchor_sim_sec_;
  const double d_wall = wall - prev_anchor_wall_sec_;
  const double span_wall = wall - wall0_sec_;
  begin("clock_anchor", ctx);
  // t_sim_sec and t_wall_sec in the envelope ARE the anchor pair; these are the
  // derived rates, recorded so the drift is visible without differencing.
  num("sim_since_prev_sec", d_sim);
  num("wall_since_prev_sec", d_wall);
  // Instantaneous real-time factor over the last anchor interval. Null (not 0)
  // when the wall interval is degenerate, so a divide-by-zero never
  // masquerades as a stopped simulation.
  //
  // `> 0.0` was not a strong enough guard. The first anchor fires in the same
  // tick as run_start, which has just set prev_anchor_* to t0, so d_sim is
  // exactly 0 and d_wall is a few microseconds: the division succeeds and
  // writes 0.0 — the very "stopped simulation" reading the guard above claims
  // to prevent, once per robot per run. Requiring a real wall interval keeps
  // the genuine stopped-sim case (d_wall large, d_sim 0 -> rtf 0.0) reportable
  // while rejecting the zero-length one.
  constexpr double kMinAnchorWallSec = 1e-3;
  if (d_wall > kMinAnchorWallSec) {
    rtf_last_ = d_sim / d_wall;
    num("rtf", rtf_last_);
  } else {
    key("rtf");
    line_ += "null";
  }
  // Mean since t0. The pair (rtf, rtf_mean) is what shows drift WITHIN a run —
  // measured at 0.89 -> 0.81 in this campaign, which is what made a single
  // linear wall->sim fit wrong by up to 54 s.
  if (span_wall > kMinAnchorWallSec) {
    num("rtf_mean", (ctx.sim_time_sec - t0_sec_) / span_wall);
  } else {
    key("rtf_mean");
    line_ += "null";
  }
  integer("anchor", anchors_);
  end();
  ++anchors_;
  prev_anchor_sim_sec_  = ctx.sim_time_sec;
  prev_anchor_wall_sec_ = wall;
}

void ExperimentLog::logStateChange(const ExperimentContext& ctx,
                                   const char* from, const char* to,
                                   const char* reason, double dwell_sec) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("state_change", ctx);
  text("from", from);
  text("to", to);
  text("reason", reason);
  // -1 = unknown (the first transition of the run, before any state entry has
  // been stamped). 0 is a real value — a transition taken on the entry tick.
  num("from_dwell_sec", dwell_sec);
  end();
}

void ExperimentLog::logPeerLost(const ExperimentContext& ctx,
                                const PeerEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("peer_lost", ctx);
  text("peer", e.peer);
  // How long the peer had been silent when the belief flipped, i.e. the age of
  // its last intent at that instant (>= the claim TTL by construction).
  num("silent_sec", e.silent_sec);
  teamCounts(e.peers_live, e.expected_peers);
  end();
}

void ExperimentLog::logPeerSeen(const ExperimentContext& ctx,
                                const PeerEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("peer_seen", ctx);
  text("peer", e.peer);
  // How long the outage lasted, measured from the last intent heard before it.
  num("silent_sec", e.silent_sec);
  boolean("first_contact", e.first_contact);
  teamCounts(e.peers_live, e.expected_peers);
  end();
}

void ExperimentLog::logReconnectDispatch(const ExperimentContext& ctx,
                                         const ReconnectDispatchEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("reconnect_dispatch", ctx);
  text("mode", e.mode);
  // The two triggers are different experiments in one run: `terminal` is
  // exploration exhaustion (may end in DONE), the mid-run trigger interrupts
  // exploration and must resume it.
  boolean("terminal", e.terminal);
  text("trigger", e.terminal ? "terminal" : "midrun");
  text("reason", e.reason);
  text("peer", e.peer);
  num("peer_record_age_sec", e.peer_record_age_sec);
  text("action", e.action);
  if (e.have_dest) {
    num("dest_x", e.dest_x);
    num("dest_y", e.dest_y);
  } else {
    key("dest_x"); line_ += "null";
    key("dest_y"); line_ += "null";
  }
  num("budget_sec", e.budget_sec);
  text("decline_reason", e.decline_reason);
  integer("attempt", e.attempt);
  num("gate_sec", e.gate_sec);
  num("est_unshared_vox", e.est_unshared_vox);
  // Additive key: the event stream is JSON-lines and sim/event_log.py parses it
  // with json.loads, so older readers ignore it and newer readers can tell a
  // link-gated fire from a record-age one without consulting the manifest.
  num("link_down_sec", e.link_down_sec);
  // Additive for the same reason, and load-bearing: this is the left-hand side
  // of the inequality the mid-run trigger evaluated. Without it the record holds
  // the threshold (gate_sec) but not the quantity compared against it, so the
  // decision cannot be re-derived from the log at all — peer_record_age_sec
  // stands ~coord_claim_ttl_sec clear of it and tests a different inequality.
  num("team_incomplete_sec", e.team_incomplete_sec);
  // P6 interception, additive for the same reason as the two above. `predictor`
  // is written unconditionally — including "trail" on every control run — so
  // the arm can be read off a single dispatch line instead of inferred from the
  // absence of keys, which is what a reader would otherwise have to do and is
  // indistinguishable from an older binary.
  text("predictor", e.predictor);
  text("predict_refused", e.predict_refused);
  num("predict_p", e.predict_p);
  num("predict_p_on_route", e.predict_p_on_route);
  num("predict_horizon_sec", e.predict_horizon_sec);
  num("predict_tour_age_sec", e.predict_tour_age_sec);
  integer("predict_cell", e.predict_cell);
  integer("predict_candidates", e.predict_candidates);
  teamCounts(e.peers_live, e.expected_peers);
  end();
}

void ExperimentLog::logReconnectEnd(const ExperimentContext& ctx,
                                    const ReconnectEndEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("reconnect_end", ctx);
  text("outcome", e.outcome);
  text("to_state", e.to_state);
  text("reason", e.reason);
  num("duration_sec", e.duration_sec);
  boolean("terminal", e.terminal);
  teamCounts(e.peers_live, e.expected_peers);
  end();
}

void ExperimentLog::logExplorationComplete(
    const ExperimentContext& ctx, const ExplorationCompleteEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  // THE completion time: the LAST declaration, so it moves forward when a robot
  // resumes after a reconnect manoeuvre and exhausts again. On this line it is
  // "the latest so far"; run_end carries the final value. The first declaration
  // is latched alongside it — the two answer different questions and the run is
  // not entitled to only one of them. See the member declarations.
  if (explore_done_first_sec_ < 0.0) explore_done_first_sec_ = ctx.sim_time_sec;
  explore_done_last_sec_ = ctx.sim_time_sec;
  // A fresh declaration re-arms the post_latch window; the steps that led to
  // it belong to the exploration that just ended, not to the one after it.
  explore_resumed_since_done_ = false;
  begin("exploration_complete", ctx);
  num("explore_done_sim_sec", explore_done_last_sec_);
  num("explore_done_rel_sec", explore_done_last_sec_ - t0_sec_);
  num("explore_done_first_sim_sec", explore_done_first_sec_);
  text("reason", e.reason);
  num("unknown_fraction", e.unknown_fraction);
  text("coverage_source", e.coverage_source);
  integer("steps", e.steps);
  num("distance_m", e.distance_m);
  boolean("team_complete", e.team_complete);
  // 1 = the first exhaustion of the run. A robot can exhaust, reconnect, get a
  // merged map with new frontiers in it, explore again and exhaust a second
  // time; the analysis wants occurrence 1 for "when did THIS robot finish its
  // own map" and the last one for "when did it stop trying".
  integer("occurrence", e.occurrence);
  teamCounts(e.peers_live, e.expected_peers);
  end();
}

void ExperimentLog::logMissionComplete(const ExperimentContext& ctx,
                                       const MissionCompleteEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  // Idempotence latch, the same construction as run_end_written_ below and for
  // the same reason: a second mission_complete is a duplicate endpoint, and
  // through generation 8 it made 16 ts1b robot-runs report a homing leg whose
  // duration and distance restarted from zero. The node now refuses the
  // re-entry that caused those (mission_return_done_), so this is the second
  // line of defence, sitting at the layer that owns the FILE's contract.
  //
  // Suppressing it silently would be the trap. `occurrence` below was the only
  // offline evidence a duplicate ever happened, and a latch that drops the row
  // also drops the evidence — so the suppression is counted here and written
  // into run_end as mission_completes_suppressed, and the first one warns. A
  // reader cross-checking "one row, occurrence 1, zero suppressed" now gets
  // three agreeing facts instead of one that the fix quietly emptied.
  if (mission_complete_written_) {
    if (!dup_mission_complete_reported_) {
      dup_mission_complete_reported_ = true;
      RCLCPP_WARN(logger_,
          "mission_complete was already written at t_sim=%.3f; this second "
          "attempt at t_sim=%.3f (result=%s, reason=%s, occurrence=%d) is "
          "SUPPRESSED so the run keeps one endpoint. Reaching this means a "
          "mission return resolved twice — treat this run's homing metrics as "
          "suspect and check run_end.mission_return_reentries.",
          mission_complete_sim_sec_, ctx.sim_time_sec,
          e.result ? e.result : "", e.reason ? e.reason : "", e.occurrence);
    }
    ++dup_mission_completes_;
    return;
  }
  mission_complete_written_  = true;
  mission_complete_sim_sec_  = ctx.sim_time_sec;
  begin("mission_complete", ctx);
  text("result", e.result);
  text("reason", e.reason);
  num("mission_home_sim_sec", ctx.sim_time_sec);
  num("mission_home_rel_sec", ctx.sim_time_sec - t0_sec_);
  num("home_x", e.home_x);
  num("home_y", e.home_y);
  num("final_x", e.final_x);
  num("final_y", e.final_y);
  num("dist_to_home_m", e.dist_to_home_m);
  num("homing_duration_sec", e.homing_duration_sec);
  num("homing_distance_m", e.homing_distance_m);
  // Unconditional, like occurrence below: a leg that took no proximity hold
  // must read a measured 0.0, because this field is what separates a homing
  // duration that legitimately exceeds mission_return_max_sec from one that
  // overran, and a missing key cannot make that distinction. See the field doc.
  num("homing_held_sec", e.homing_held_sec);
  boolean("latched", e.latched);
  // 1 = the only mission_complete of the run, which is the contract. Anything
  // above 1 is a DONE->RETURN_HOME re-entry and every row of that run's homing
  // metrics is suspect. Written unconditionally so the count is evidence rather
  // than an assertion the logger makes about itself.
  integer("occurrence", e.occurrence);
  end();
}

void ExperimentLog::logNavGoalFailed(const ExperimentContext& ctx, double x,
                                     double y, const char* reason,
                                     double elapsed_sec, int k, bool retired,
                                     double budget_sec, bool pose_stale,
                                     const char* test_name, double test_value,
                                     double test_threshold) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("nav_goal_failed", ctx);
  num("x", x);
  num("y", y);
  text("reason", reason);
  num("elapsed_sec", elapsed_sec);
  num("budget_sec", budget_sec);
  // The comparison that actually fired. Kept distinct from
  // (elapsed_sec, budget_sec) rather than overwriting them: both pairs are
  // wanted on a budget-rotate row — how long the whole attempt had been
  // running AND how long the rotation had — and collapsing them is what made
  // the old schema misreport 30 of 31 rows.
  text("test_name", test_name);
  num("test_value", test_value);
  num("test_threshold", test_threshold);
  integer("k", k);
  boolean("retired", retired);
  // True means the no-progress / budget verdict was reached while TF was
  // stale. Such a row is evidence about the pose feed, not about the terrain,
  // and must not be pooled with ordinary nav failures.
  boolean("pose_stale", pose_stale);
  end();
}

void ExperimentLog::logPoseHealth(const ExperimentContext& ctx, bool lost,
                                  double age_sec) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("pose_health", ctx);
  boolean("lost", lost);
  num("age_sec", age_sec);
  end();
}

void ExperimentLog::logGoalAmnesty(const ExperimentContext& ctx, double x,
                                   double y, double last_fail_age_sec,
                                   bool retired, const char* source) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("goal_amnesty", ctx);
  num("x", x);
  num("y", y);
  num("last_fail_age_sec", last_fail_age_sec);
  // Whether the amnesty landed on a RETIRED site. These are two different
  // events for analysis: retired=false is the valve doing its job on a merely
  // hot goal, retired=true means nothing but confirmed traps were left, which
  // is a starvation signature and not a routine retry.
  boolean("retired", retired);
  // Unconditional, and never empty: `retired` cannot stand in for the tier
  // (the visited tier reads retired=false by construction), and the two tiers
  // mean opposite things about the run's health — a "failed" row is the valve
  // working, a "visited" row is a tick that would have stalled outright before
  // v8. See the field doc.
  text("source", source);
  end();
}

void ExperimentLog::logHomeWatchdog(const ExperimentContext& ctx,
                                    const char* kind, const char* mode,
                                    const char* response, double dist_home_m,
                                    double metric_m, double window_sec,
                                    int escapes_used, double test_delta_m,
                                    double test_threshold_m,
                                    const char* next_mode) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("home_watchdog", ctx);
  text("kind", kind);
  text("mode", mode);
  text("response", response);
  num("dist_home_m", dist_home_m);
  // INSTANTANEOUS, at the fire instant — the remaining-distance metric as
  // sampled on this tick, in whichever mode was in force (straight-line for
  // direct, along-trail for retrace, which is why it can exceed dist_home_m).
  // It is NOT movement over `window_sec`; this comment said that it was
  // through generation 7, which made it read as the tested quantity. The
  // tested quantity is test_delta_m below.
  num("metric_m", metric_m);
  num("window_sec", window_sec);
  integer("escapes_used", escapes_used);
  // The fired inequality, on detector-fire rows only: the detector fired
  // because test_delta_m < test_threshold_m. Gated on `kind` rather than on a
  // sentinel value because every numeric sentinel collides with a real
  // reading — 0.0 is the canonical frozen fire and negatives are a receding
  // approach fire — so ABSENCE is what has to mean "no inequality here".
  // escape-end rows are leg terminations and evaluate no detector.
  if (std::strcmp(kind, "escape-end") != 0) {
    num("test_delta_m", test_delta_m);
    num("test_threshold_m", test_threshold_m);
  }
  if (next_mode) text("next_mode", next_mode);
  end();
}

void ExperimentLog::logRunEnd(const ExperimentContext& ctx,
                              const RunEndEvent& e) {
  // The three refusals are SEPARATED because they mean different things and two
  // of them used to be silent. Collapsed into one `if`, a run_end that arrived
  // before startRun went unrecorded — `events_dropped_before_start` counted
  // every other event kind and this one alone escaped, so the logger's own
  // accounting was inexact in the one place a reader would trust it.
  if (!open_) return;
  if (!started_) { ++dropped_before_start_; return; }
  // A SECOND run_end is a duplicate endpoint, the same class of defect as the
  // 16 ts1b robot-runs that carried two mission_complete rows (see
  // MissionCompleteEvent::occurrence). Suppressing it is right — a second
  // run_end would give the file two last lines and every "read the tail"
  // consumer a coin flip — but suppressing it SILENTLY makes the guard a check
  // that stopped checking: it would absorb a DONE re-entry indefinitely with
  // nothing anywhere saying so. The row is already written and cannot carry a
  // count, so the evidence goes to the ROS log, which the harness captures per
  // robot. Once, not per call, because the caller may be a tick loop.
  if (run_end_written_) {
    if (!dup_run_end_reported_) {
      dup_run_end_reported_ = true;
      RCLCPP_WARN(logger_,
          "run_end was already written at t_sim=%.3f; this second attempt at "
          "t_sim=%.3f (reason=%s) is SUPPRESSED so the file keeps one last "
          "line. A duplicate run_end means the node reached a terminal state "
          "twice — treat this run's endpoint metrics as suspect.",
          run_end_sim_sec_, ctx.sim_time_sec, e.reason ? e.reason : "");
    }
    ++dup_run_ends_;
    return;
  }
  run_end_written_ = true;
  run_end_sim_sec_ = ctx.sim_time_sec;
  begin("run_end", ctx);
  text("reason", e.reason);
  integer("steps", e.steps);
  num("distance_m", e.distance_m);
  num("unknown_fraction", e.unknown_fraction);
  text("coverage_source", e.coverage_source);
  teamCounts(e.peers_live, e.expected_peers);
  // THE completion time, restated so it can be read without scanning for the
  // exploration_complete line. Null when this robot never declared exhaustion
  // (killed by the duration cap, i.e. a censored run) — which is itself a
  // result, and one that a "max sim time in the file" reading would silently
  // convert into a completion.
  if (explore_done_last_sec_ >= 0.0) {
    num("explore_done_sim_sec", explore_done_last_sec_);
    num("explore_done_rel_sec", explore_done_last_sec_ - t0_sec_);
    num("explore_done_first_sim_sec", explore_done_first_sec_);
    // How much the robot's own completion moved because it resumed after a
    // reconnect. Zero for a robot that declared once. Non-zero only in the
    // reconnecting arms, which is exactly why it is reported rather than
    // silently absorbed into one number or the other.
    num("explore_done_resume_delta_sec",
        explore_done_last_sec_ - explore_done_first_sec_);
  } else {
    key("explore_done_sim_sec"); line_ += "null";
    key("explore_done_rel_sec"); line_ += "null";
    key("explore_done_first_sim_sec"); line_ += "null";
    key("explore_done_resume_delta_sec"); line_ += "null";
  }
  // How long this FILE spans, which is a property of when the process was
  // stopped — deliberately named so it cannot be mistaken for a completion
  // time, and never called makespan.
  num("log_span_sim_sec", ctx.sim_time_sec - t0_sec_);
  // Clock statistics for the whole run (see the clock-anchor note in the
  // header): the mean real-time factor and the last instantaneous one, so
  // drift is visible even in a summary read of the last line.
  {
    const double span_wall = wallEpochSec() - wall0_sec_;
    if (span_wall > 1e-3) {  // see kMinAnchorWallSec in logClockAnchor
      num("rtf_mean", (ctx.sim_time_sec - t0_sec_) / span_wall);
    } else {
      key("rtf_mean"); line_ += "null";
    }
    num("rtf_last", rtf_last_);
    integer("clock_anchors", anchors_);
  }
  // The CSV sampler's configured vs REALISED period (see RunEndEvent).
  num("metrics_period_param_sec", e.metrics_period_param_sec);
  num("metrics_realised_period_sec", e.metrics_realised_period_sec);
  num("metrics_effective_period_sec", e.metrics_effective_period_sec);
  integer("metrics_timer_rows", e.metrics_timer_rows);
  integer("metrics_backoffs", e.metrics_backoffs);
  // Final geometry + mission-return summary (schema 2, see RunEndEvent).
  // home is null-per-field rather than omitted so a reader indexing by key
  // sees the same shape in every file of a campaign.
  boolean("have_home", e.have_home);
  if (e.have_home) {
    num("home_x", e.home_x);
    num("home_y", e.home_y);
  } else {
    key("home_x"); line_ += "null";
    key("home_y"); line_ += "null";
  }
  num("final_x", e.final_x);
  num("final_y", e.final_y);
  if (!e.mission_home_result.empty()) {
    text("mission_home_result", e.mission_home_result);
    num("mission_home_sim_sec", e.mission_home_sim_sec);
  } else {
    key("mission_home_result"); line_ += "null";
    key("mission_home_sim_sec"); line_ += "null";
  }
  // Duplicate-endpoint accounting, both directions. mission_return_reentries
  // is the NODE refusing to start a second homing leg; mission_completes_
  // suppressed is the LOGGER refusing to write a second row. Both are 0 in a
  // healthy run, and both are written unconditionally so that 0 is a measured
  // fact rather than the absence of a field.
  integer("mission_return_reentries", e.mission_return_reentries);
  integer("mission_completes_suppressed", dup_mission_completes_);
  // Unconditional for the same reason as the two above: the mid-run attempt cap
  // silently weakens the treatment once it binds, and a cell that never came
  // close must be a measured 0, not a missing key. See the field's doc.
  integer("midrun_attempts_used", e.midrun_attempts_used);
  integer("milestones_reached", milestonesReached());
  integer("milestones_total", static_cast<long long>(milestones_.size()));
  // Self-accounting. A file whose last line is a run_end with
  // logger_healthy=true and events_written == seq+1 is complete by its own
  // testimony; anything else is not, and says so.
  boolean("logger_healthy", healthy_);
  integer("write_failures", write_failures_);
  integer("events_dropped_before_start", dropped_before_start_);
  integer("events_written", seq_);  // this line included: seq_ was post-incremented
  end();
}

void ExperimentLog::noteCoverage(const ExperimentContext& ctx,
                                 double unknown_fraction,
                                 const char* coverage_source, double distance_m,
                                 double x, double y) {
  if (!open_ || !started_) return;  // not a dropped EVENT; nothing was emitted
  // -1 is the planner's "cannot measure" sentinel (no planning_map, degenerate
  // ROI) and NaN would come from a degenerate footprint. Neither is a coverage
  // of 0, and treating them as one would fire the whole ladder at once.
  if (!std::isfinite(unknown_fraction) || unknown_fraction < 0.0) return;

  // Sim seconds since the PREVIOUS coverage measurement. This is what brackets
  // a crossing — the rung was passed somewhere in (t - gap, t] — and it is
  // measured rather than assumed from the configured sampling period, because
  // the sampler feeding it can and did stretch its own period silently.
  const double gap = prev_coverage_sample_sec_ >= 0.0
      ? ctx.sim_time_sec - prev_coverage_sample_sec_ : -1.0;
  prev_coverage_sample_sec_ = ctx.sim_time_sec;

  for (size_t i = 0; i < milestones_.size(); ++i) {
    if (milestone_hit_[i]) continue;
    if (unknown_fraction > milestones_[i]) continue;
    milestone_hit_[i] = true;
    begin("coverage_milestone", ctx);
    // The rung, and the value that crossed it. The gap between them is the
    // sampling overshoot: with a 5 s sampler the true crossing lies in
    // (t - period, t], and `unknown_fraction` says how far past the rung the
    // sample already was.
    num("threshold", milestones_[i]);
    num("unknown_fraction", unknown_fraction);
    text("coverage_source", coverage_source);
    // The crossing lies in (t_sim_sec - sample_gap_sec, t_sim_sec].
    // -1 on the first sample of the run, which has no predecessor.
    num("sample_gap_sec", gap);
    num("distance_m", distance_m);
    num("x", x);
    num("y", y);
    integer("rung", static_cast<long long>(i));
    integer("rungs_total", static_cast<long long>(milestones_.size()));
    // C1: did this crossing happen after the robot declared exploration
    // complete? A rung crossed post-declaration times post-stop map merging,
    // not exploration, and whether a run reaches such a rung at all correlates
    // with the arm -- so it is a selection artifact, not an endpoint. The
    // ladder is now sized so this should always be false; recording it is how
    // a future ladder/threshold mismatch announces itself instead of quietly
    // contaminating a headline number.
    // Keyed on the LAST declaration and on a flag that a resumed exploration
    // clears -- not on explore_done_first_sec_, which is a one-way latch.
    // Through generation 8 it was the latch, and that made the flag report
    // `true` on genuine exploration: a robot that declares, is pulled into a
    // reconnect manoeuvre, comes back with a merged map and explores on was
    // still "post-latch" for the rest of the run. Resumption happens only
    // where manoeuvres happen, so a flag added to detect an ARM-CORRELATED
    // selection artifact was itself arm-correlated -- it would have attributed
    // the reconnecting arms' late rungs to post-stop map merging and thrown
    // away real coverage.
    //
    // explore_resumed_since_done_ is set by logStep, which is the right
    // signal and needs nothing from the node: the step counter is frozen for
    // the whole of a manoeuvre and for the whole homing leg, so a `step`
    // event arriving after a declaration means this robot is exploring again.
    // On the declaring tick doLogStep writes its step BEFORE routing into
    // recordExplorationComplete, so the flag cannot be set spuriously by the
    // very step that ended.
    const bool post_latch =
        explore_done_last_sec_ >= 0.0 && !explore_resumed_since_done_;
    boolean("post_latch", post_latch);
    // -1.0 is "not applicable", NOT a measured zero and not a negative
    // interval: the robot had not declared, or it had declared and resumed.
    // Same sentinel discipline as the field above it, and stated here because
    // an undocumented -1 in a seconds column is unrecoverable once averaged.
    // Measured from the LAST declaration for the same reason post_latch is.
    num("sec_since_explore_done",
        post_latch ? ctx.sim_time_sec - explore_done_last_sec_ : -1.0);
    end();
  }
}

void ExperimentLog::logCellCensus(const ExperimentContext& ctx,
                                  const CellCensusEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("cell_census", ctx);
  integer("cells_total", e.cells_total);
  integer("unseen", e.unseen);
  integer("exploring", e.exploring);
  integer("covered", e.covered);
  integer("exploring_by_others", e.exploring_by_others);
  integer("covered_by_others", e.covered_by_others);
  num("covered_fraction", e.covered_fraction);
  // The planner's continuous ROI measure, on this same tick and this same map.
  // Kept beside the histogram so the P1 agreement check needs no join.
  num("roi_unknown_fraction", e.roi_unknown_fraction);
  text("coverage_source", e.coverage_source);
  integer("changed", e.changed);
  integer("commits_total", e.commits_total);
  num("cell_size_m", e.cell_size_m);
  integer("nx", e.nx);
  integer("ny", e.ny);
  // Written as a decimal integer, not hex: the JSON number type has no hex
  // literal, and a quoted "0x..." would read as a string in half the tools
  // that open this file.
  integer("grid_hash", static_cast<long long>(e.grid_hash));
  integer("shared_hash", static_cast<long long>(e.shared_hash));
  integer("edges_enabled", e.edges_enabled);
  integer("edges_total", e.edges_total);
  integer("cells_measured", e.cells_measured);
  integer("cells_frontier_ok", e.cells_frontier_ok);
  num("cell_unknown_min", e.cell_unknown_min);
  num("cell_unknown_p10", e.cell_unknown_p10);
  num("cell_unknown_median", e.cell_unknown_median);
  num("cell_frontier_frac_min", e.cell_frontier_frac_min);
  num("cell_frontier_frac_median", e.cell_frontier_frac_median);
  num("cell_frontier_frac_at_best_unknown",
      e.cell_frontier_frac_at_best_unknown);
  end();
}

void ExperimentLog::logTeamExchange(const ExperimentContext& ctx,
                                    const TeamExchangeEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("team_exchange", ctx);
  text("peer", e.peer);
  integer("peer_id", e.peer_id);
  // Always written, including as "": a reader filtering for drops must not
  // have to treat an absent key and an empty one as the same thing.
  text("drop_reason", e.drop_reason);
  integer("coalesced", e.coalesced);
  num("queue_age_sec", e.queue_age_sec);
  integer("cells_in_msg", e.cells_in_msg);
  integer("applied", e.applied);
  integer("known_by_only", e.known_by_only);
  integer("agreed_noop", e.agreed_noop);
  integer("refused_guard", e.refused_guard);
  integer("refused_local", e.refused_local);
  integer("out_of_range", e.out_of_range);
  integer("bad_status", e.bad_status);
  boolean("in_comms", e.in_comms);
  boolean("direct", e.direct);
  boolean("one_way", e.one_way);
  boolean("via_relay", e.via_relay);
  num("last_direct_age_sec", e.last_direct_age_sec);
  num("last_known_age_sec", e.last_known_age_sec);
  integer("peers_lost", e.peers_lost);
  // Decimal, for the same reason grid_hash is: JSON has no hex literal and a
  // quoted "0x.." reads as a string in half the tools that open this file.
  integer("peer_in_range_mask", static_cast<long long>(e.peer_in_range_mask));
  integer("self_direct_mask", static_cast<long long>(e.self_direct_mask));
  num("covered_fraction", e.covered_fraction);
  integer("covered", e.covered);
  integer("covered_by_others", e.covered_by_others);
  integer("exploring", e.exploring);
  integer("exploring_by_others", e.exploring_by_others);
  end();
}

void ExperimentLog::logAllocation(const ExperimentContext& ctx,
                                  const AllocationEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("allocation", ctx);
  // Decimal, like every other hash in this file: JSON has no hex literal.
  integer("shared_hash", static_cast<long long>(e.shared_hash));
  integer("grid_hash", static_cast<long long>(e.grid_hash));
  // R3 / §3.6. Join two robots' allocation cycles on alloc_hash, NOT on
  // shared_hash — see the struct comment for why the latter cannot bear it.
  integer("alloc_hash", static_cast<long long>(e.alloc_hash));
  integer("edge_hash", static_cast<long long>(e.edge_hash));
  integer("robots_in_problem", e.robots_in_problem);
  integer("candidates", e.candidates);
  integer("unassigned", e.unassigned);
  // Always written, including as "": a reader filtering for refusals must not
  // have to treat an absent key and an empty one as the same thing.
  text("refused", e.refused);
  num("solve_ms", e.solve_ms);
  integer("focus_cell", e.focus_cell);
  text("tour", e.tour);
  text("peer_focus", e.peer_focus);
  boolean("all_in_comms", e.all_in_comms);
  integer("picked_rank", e.picked_rank);
  boolean("reordered", e.reordered);
  integer("focus_skips", e.focus_skips);
  text("demoted", e.demoted);
  end();
}

void ExperimentLog::logReconnectGate(const ExperimentContext& ctx,
                                     const ReconnectGateEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("reconnect_gate", ctx);
  boolean("dispatched", e.dispatched);
  boolean("knowledge", e.knowledge);
  integer("unshared_cells", e.unshared_cells);
  integer("c_no_mm", e.c_no_mm);
  integer("c_re_mm", e.c_re_mm);
  integer("leg_mm", e.leg_mm);
  integer("unassigned", e.unassigned);
  // Always written, including as "": a reader counting fail-open passes must
  // not have to treat an absent key and an empty one as the same thing.
  text("refused", e.refused);
  num("team_incomplete_sec", e.team_incomplete_sec);
  num("gate_sec", e.gate_sec);
  integer("attempts_used", e.attempts_used);
  text("peers", e.peers);
  end();
}

void ExperimentLog::logRendezvousAgreed(const ExperimentContext& ctx,
                                        const RendezvousAgreedEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("rendezvous_agreed", ctx);
  // Decimal, like every other hash in this file: JSON has no hex literal.
  integer("shared_hash", static_cast<long long>(e.shared_hash));
  integer("grid_hash", static_cast<long long>(e.grid_hash));
  num("snapshot_age_sec", e.snapshot_age_sec);
  integer("cell", e.cell);
  num("t_meet_sec", e.t_meet_sec);
  num("t_now_sec", e.t_now_sec);
  // The generation key. Written on refusal rows too, at its sentinel, for the
  // reason the agreement block below states: an absent key and a sentinel one
  // must not be the same thing to a reader counting agreements.
  num("agreed_base_sec", e.agreed_base_sec);
  num("interval_sec", e.interval_sec);
  boolean("capped", e.capped);
  boolean("floored", e.floored);
  num("tour_interval_sec", e.tour_interval_sec);
  integer("penalty_mm", e.penalty_mm);
  boolean("floor_won", e.floor_won);
  integer("candidates", e.candidates);
  integer("rejected_unreachable", e.rejected_unreachable);
  integer("rejected_excluded", e.rejected_excluded);
  num("travel_sec", e.travel_sec);
  // Always written, including as "": a reader counting refusals must not have
  // to treat an absent key and an empty one as the same thing.
  text("refused", e.refused);
  // --- the agreement (v5) ---------------------------------------------
  // Written on refusal rows too, at their sentinels. A refusal that says
  // "no pair was agreed" and a refusal that says "the pair was spent" are
  // different mechanisms, and only these columns separate them.
  integer("proposer_id", e.proposer_id);
  boolean("from_agreed", e.from_agreed);
  // The one column in the argmin block a FOLLOWER row can answer, because the
  // flag rides the wire beside the three integers. See the field's doc: a cell
  // whose rows are all true agreed a place without choosing one.
  boolean("agreed_provisional", e.agreed_provisional);
  num("anchor_sec", e.anchor_sec);
  num("agreed_age_sec", e.agreed_age_sec);
  integer("peers_on_pair", e.peers_on_pair);
  num("own_route_m", e.own_route_m);
  end();
}

void ExperimentLog::logRendezvousOutcome(const ExperimentContext& ctx,
                                         const RendezvousOutcomeEvent& e) {
  if (!open_ || !started_) { ++dropped_before_start_; return; }
  begin("rendezvous_outcome", ctx);
  integer("cell", e.cell);
  num("t_meet_sec", e.t_meet_sec);
  num("t_end_sec", e.t_end_sec);
  num("lateness_sec", e.lateness_sec);
  text("outcome", e.outcome);
  boolean("arrived", e.arrived);
  num("waited_sec", e.waited_sec);
  boolean("mutual", e.mutual);
  end();
}

int ExperimentLog::milestonesReached() const {
  return static_cast<int>(
      std::count(milestone_hit_.begin(), milestone_hit_.end(), true));
}

}  // namespace explo_planner
