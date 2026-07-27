#pragma once
/// @file metrics_logger.hpp
/// @brief Per-step CSV metric logging for exploration experiments.

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
  float  mean_path_cost        = 0.0f;
  float  selected_info_gain    = 0.0f;
  float  selected_path_cost    = 0.0f;
  float  selected_utility      = 0.0f;

  // Multi-robot coordination diagnostics. Zero in single-robot mode.
  int    coord_active_peers       = 0;
  int    rejected_by_minpos       = 0;
  int    rejected_by_unreachable  = 0;
};

class MetricsLogger {
public:
  explicit MetricsLogger(const std::string& csv_path);
  ~MetricsLogger();

  void logStep(const StepMetrics& m);

private:
  std::ofstream file_;
  bool header_written_ = false;

  void writeHeader();
};

} // namespace explo_planner
