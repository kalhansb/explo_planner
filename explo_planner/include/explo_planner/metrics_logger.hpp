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

  // Coordinated proximity stop, CUMULATIVE across the run (like
  // distance_traveled): holds entered so far and total seconds spent held.
  // Per-step deltas come from differencing consecutive rows; without these
  // columns the only record of the yields was planner stdout.
  int    prox_hold_count          = 0;
  float  prox_hold_total_sec      = 0.0f;

  // Reconnection diagnostics. Rows are emitted on a periodic timer in EVERY
  // state, not just at LOG_STEP, so a minutes-long reconnect manoeuvre samples
  // the coverage curve instead of leaving a gap in it.
  //
  // ALL DURATIONS HERE ARE SIM SECONDS under use_sim_time (the node reads
  // this->now()/get_clock()). At RTF != 1 they are not wall seconds, and the
  // sampling period is likewise a sim-time period.
  //
  // `state` is the planner state at emit time, from stateName(). It is also the
  // row discriminator: a row with state == "LOG_STEP" is an end-of-step row
  // (plan-attribution columns populated); every other value is a timer row
  // (plan-attribution columns are zero by construction — there was no plan on
  // that tick to attribute them to). The timer deliberately skips LOG_STEP ticks
  // so the two kinds never collide.
  //
  // The reconnect_* pair is -1 whenever no manoeuvre is in flight, which keeps 0
  // meaning "arrived / just started" rather than "not applicable".
  // reconnect_elapsed_sec is raw elapsed sim time since the manoeuvre was armed
  // and is NOT bounded by pursuit_budget_max_sec: proximity-hold time is refunded to
  // the pursuit budget clock but not to this one, and in HYBRID the meeting-point
  // leg runs on the same clock after the chase is spent.
  //
  // reconnect_range_to_goal_m is REMAINING straight-line XY range to the
  // manoeuvre goal — a decreasing sample, not a travelled distance. Named for
  // what it is: as `reconnect_distance_m` it read like the manoeuvre's path
  // cost, which inverts the comparison it would be used for (a robot that
  // drives far and arrives ends near 0; one that gives up early stays large).
  // Travelled distance is `distance_traveled` differenced across the rows where
  // reconnect_elapsed_sec >= 0.
  std::string state                    = "UNKNOWN";
  float  reconnect_range_to_goal_m     = -1.0f;
  float  reconnect_elapsed_sec         = -1.0f;

  // Coverage-termination measure, as evaluated this tick, plus which source
  // produced it ("scovox" 2.5D column coverage of the ROI footprint, or
  // "planning_map" 2D cell coverage). This IS the primary endpoint quantity and
  // the one the DONE criterion is compared against, and until now it existed
  // nowhere in the CSV — it reached stdout only inside the branch where it had
  // already dropped below threshold, so a run could not be scored on the
  // criterion it terminates on. total_observed_voxels is not a substitute: it
  // is a 3D active-cell count whose relationship to column coverage varies with
  // occlusion and the z-band.
  //
  // -1 when it cannot be measured (source "planning_map" with no planning_map
  // received, or a degenerate ROI), matching coverageUnknownFraction().
  float  unknown_fraction         = -1.0f;
  std::string coverage_source     = "none";
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
