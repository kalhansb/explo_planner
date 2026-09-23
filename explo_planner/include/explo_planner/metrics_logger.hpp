#pragma once
/// @file metrics_logger.hpp
/// @brief Per-step CSV metric logging for exploration experiments.
/// Moved comments: doc/metrics_logger_notes.md

#include <string>
#include <fstream>

namespace explo_planner {

struct StepMetrics {
  // Exploitation diagnostics. Exploration steps leave these at their defaults
  // ("explore"/-1/0); exploitation steps fill them from the active target and
  // the dwelled vantage so a CSV row attributes itself to a phase + target.
  std::string phase            = "explore";  // "explore" or "exploit"
  int    target_id             = -1;   // active target id, or -1 in explore
  int    vantage_index         = -1;   // index of the dwelled vantage, or -1
  int    n_vantages_valid      = 0;    // valid vantages this exploit plan tick
  int    vantage_los_clear     = 0;    // 1 if the dwelled vantage had clear LoS
  float  dwell_sec             = 0.0f; // dwell time held at the vantage

  int    step                  = 0;
  double sim_time_sec          = 0.0;
  int    total_observed_voxels = 0;
  int    frontier_voxels       = 0;
  float  distance_traveled     = 0.0f;
  float  selected_score        = 0.0f;
  float  plan_time_ms          = 0.0f;
  float  mean_eig              = 0.0f;
  float  mean_entropy          = 0.0f;
  float  mean_variance         = 0.0f;

  // Utility / cost-grid diagnostics. Populated by doPlan() once per step
  // from the candidate set evaluated this tick. mean_* are over all
  // candidates; selected_* are the components for the chosen candidate
  // (so post-hoc analysis can attribute the pick to either term).
  float  mean_info_gain        = 0.0f;
  // Population std of info_gain across this step's candidates; near zero,
  // utility degenerates to argmin-cost. Not adjacent to mean_info_gain in the
  // CSV (append-only order, see metrics_logger.cpp).
  // (notes: metrics-info-gain-std)
  float  info_gain_std         = 0.0f;
  float  mean_path_cost        = 0.0f;
  float  selected_info_gain    = 0.0f;
  float  selected_path_cost    = 0.0f;
  /// Exact alias of selected_score, kept only because the CSV is append-only
  /// and read positionally. Never decompose or regress one on the other;
  /// nothing in this tree enforces the alias.
  /// (notes: metrics-selected-utility-alias)
  float  selected_utility      = 0.0f;

  // Multi-robot coordination diagnostics. Zero in single-robot mode.
  int    coord_active_peers       = 0;
  int    rejected_by_minpos       = 0;
  int    rejected_by_unreachable  = 0;

  // Coordinated proximity stop, CUMULATIVE across the run (like
  // distance_traveled): holds entered so far and total seconds spent held.
  // Per-step deltas come from differencing consecutive rows; without these
  // columns the only record of the yields was planner stdout.
  int    prox_hold_count          = 0;
  float  prox_hold_total_sec      = 0.0f;

  // Timer rows come in every state; state LOG_STEP marks end-of-step rows, and
  // other rows have zero plan-attribution columns. Sim seconds. reconnect_* are
  // -1 with no manoeuvre; range is remaining XY range to the goal.
  // (notes: metrics-reconnect-timer-rows)
  std::string state                    = "UNKNOWN";
  float  reconnect_range_to_goal_m     = -1.0f;
  float  reconnect_elapsed_sec         = -1.0f;

  // The coverage-termination measure this tick, the quantity the DONE criterion
  // compares, and its source (scovox or planning_map). -1 when it cannot be
  // measured, matching coverageUnknownFraction().
  // (notes: metrics-unknown-fraction)
  float  unknown_fraction         = -1.0f;
  std::string coverage_source     = "none";

  // Set on explore LOG_STEP rows only (defaults elsewhere), in every arm. -1
  // means no eligible teammate or not asked; never pool it with 0. sep_discount
  // is already applied to selected_score and selected_utility.
  // (notes: metrics-separation-columns)
  float  sep_peer_dist_m          = -1.0f;
  float  sep_discount             = 1.0f;
  int    sep_reordered            = -1;
  int    sep_eligible_peers       = 0;

  // ---- Last planning attempt: why the candidates were thrown away ----
  // Carried forward from the latest doPlan attempt and written on timer rows
  // too; -1 means no attempt yet. plan_rej_* sum to plan_cand_total when
  // plan_stall_ticks > 0. plan_rej_visited is the last CSV column.
  // (notes: metrics-plan-rejection-columns)
  int    plan_cand_total          = -1;
  int    plan_rej_close           = -1;
  int    plan_rej_map             = -1;
  int    plan_rej_unreach         = -1;
  int    plan_rej_blacklist       = -1;
  int    plan_rej_visited         = -1;
  int    plan_rej_minpos          = -1;
  int    plan_stall_ticks         = -1;

  // ---- R5: who the chase was against, and what the team looked like ----
  // pursue_peer: quarry id, empty if not pursuing. pursue_quarry_live: 1/0, -1
  // not pursuing. team_complete: teamComplete(live, expected), -1 if not asked.
  // Written while the chase is live, incl. PROXIMITY_HOLD.
  // (notes: metrics-pursuit-columns)
  std::string pursue_peer        = "";
  int    pursue_quarry_live      = -1;
  int    team_complete           = -1;
};

class MetricsLogger {
public:
  explicit MetricsLogger(const std::string& csv_path);
  ~MetricsLogger();

  void logStep(const StepMetrics& m);

  /// True while every write has reached the file. The first failure latches
  /// (sticky) and logStep() never throws, so the caller must poll ok() and
  /// report error() through the node's logger.
  /// (notes: metrics-sticky-write-failure)
  bool ok() const { return error_.empty(); }

  /// Description of the first write failure, or empty while ok(). Sticky: it
  /// records the failure that broke the file, not the most recent one.
  const std::string& error() const { return error_; }

private:
  std::string path_;
  std::ofstream file_;
  bool header_written_ = false;
  std::string error_;

  void writeHeader();

  /// Latch the first stream failure seen after a write. `where` names the write
  /// ("header" / "row") so the message says which one lost data.
  void noteStreamState(const char* where);
};

} // namespace explo_planner
