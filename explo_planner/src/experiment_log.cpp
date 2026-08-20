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
  num("last_contact_age_sec", e.last_contact_age_sec);
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
  num("last_contact_age_sec", e.last_contact_age_sec);
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

void ExperimentLog::logRunEnd(const ExperimentContext& ctx,
                              const RunEndEvent& e) {
  if (!open_ || !started_ || run_end_written_) return;
  run_end_written_ = true;
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
  integer("metrics_rows", e.metrics_rows);
  integer("metrics_backoffs", e.metrics_backoffs);
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
    end();
  }
}

int ExperimentLog::milestonesReached() const {
  return static_cast<int>(
      std::count(milestone_hit_.begin(), milestone_hit_.end(), true));
}

}  // namespace explo_planner
