#pragma once
/// @file metrics_logger.hpp
/// @brief Per-step CSV metric logging for exploration experiments.

#include <limits>
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

  // How the goal behind this row ended (the planner's "Goal end [...]" log
  // line carries the same fields): outcome ("reached", "budget",
  // "no-progress", "nav2-aborted", ...; "" when no goal ended since the last
  // row), seconds spent driving it, metres along its planned path, and what
  // nav2 reported for it (-1 error code = none received, always so on
  // Humble, whose NavigateToPose result has no error code).
  std::string goal_outcome;
  float  nav_elapsed_sec          = 0.0f;
  float  goal_path_length_m       = 0.0f;
  int    nav2_aborts              = 0;
  int    nav2_recoveries          = 0;
  int    nav2_error_code          = -1;

  // How this step's goal was picked when the normal walk left no candidate
  // (EXPLORE only): "hop-relaxed" (candidate_min_goal_dist_m dropped) or
  // "revisit" (every candidate near a recently reached goal; the area left
  // longest ago). "" for a normal pick and in exploit.
  std::string pick_fallback;

  // What kind of row this is. "step": one per LOG_STEP, as above. "status":
  // written every metrics_status_period_sec while no other row has been, so
  // a run that goes quiet still says what the planner was doing; its step is
  // the one in progress, and the map and area columns repeat the last step's.
  // "end": written on entering DONE, with run_end_reason "coverage-saturated"
  // or "step-budget", and step the number of steps done.
  std::string row_type = "step";
  std::string run_end_reason;
  std::string planner_state;
  float  state_elapsed_sec = 0.0f;

  // Where the robot and the goal are. NaN when there is no pose yet, or no
  // goal in play (planning, DONE). goal_trav_value is the planning map's value
  // at the goal (-1 unknown, 0..100, -2 no map or off it).
  float  robot_x = std::numeric_limits<float>::quiet_NaN();
  float  robot_y = std::numeric_limits<float>::quiet_NaN();
  float  robot_z = std::numeric_limits<float>::quiet_NaN();
  float  robot_yaw = std::numeric_limits<float>::quiet_NaN();
  int    robot_in_roi = -1;  // 1 inside the ROI, 0 outside, -1 no pose yet
  float  goal_x = std::numeric_limits<float>::quiet_NaN();
  float  goal_y = std::numeric_limits<float>::quiet_NaN();
  float  goal_z = std::numeric_limits<float>::quiet_NaN();
  float  goal_yaw = std::numeric_limits<float>::quiet_NaN();
  float  goal_dist_m = std::numeric_limits<float>::quiet_NaN();
  int    goal_trav_value = -2;

  // Area covered, all inside the ROI except area_driven_m2. area_mapped_m2 is
  // the fused map's x/y columns with any observed voxel, roi_area_m2 all of
  // them. unknown_fraction is the value the coverage stop compares with
  // done_unknown_fraction (-1 = cannot measure), done_streak its count of
  // passes so far. area_traversable_m2 / area_obstacle_m2 are the planning
  // map's known cells below / at its obstacle threshold (-1 = no map).
  // area_driven_m2 is the 1 m cells the robot's centre has been in.
  float  area_mapped_m2 = 0.0f;
  float  roi_area_m2 = 0.0f;
  float  unknown_fraction = -1.0f;
  int    done_streak = 0;
  float  area_traversable_m2 = -1.0f;
  float  area_obstacle_m2 = -1.0f;
  float  area_driven_m2 = 0.0f;

  // The latest exploration PLAN's candidates and the rejections
  // rejected_by_minpos / rejected_by_unreachable leave out, updated by a PLAN
  // that rejects them all too. 0 in exploit.
  int    n_candidates = 0;
  int    rejected_too_close = 0;
  int    rejected_map = 0;
  int    rejected_clearance = 0;
  int    rejected_blacklist = 0;

  // Whether the things the planner depends on are alive. nav2_status is nav2's
  // latest status for the current goal. nav2_publishers counts publishers on
  // nav_status_action's status topic (0 = nav2 down or the wrong name, and
  // then the nav2_* columns stay at their defaults; -1 = not watched).
  // goal_subscribers counts subscribers on the goal topic. map_age_sec /
  // plan_map_age_sec are the seconds since the last fused map / planning-map
  // window arrived (-1 = none yet, or not accumulating).
  std::string nav2_status;
  int    nav2_publishers = -1;
  int    goal_subscribers = 0;
  float  map_age_sec = -1.0f;
  float  plan_map_age_sec = -1.0f;
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
