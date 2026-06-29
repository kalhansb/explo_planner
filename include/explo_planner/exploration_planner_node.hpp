#pragma once
/// @file exploration_planner_node.hpp
/// @brief NBV (next-best-view) exploration planner node — declaration.
///
/// Backs the single `explo_planner_node` executable. The whole state machine
/// (WAIT_FOR_MAP → PLAN → NAVIGATE → INTEGRATE → LOG_STEP → DONE) lives here;
/// planner_type selects the scoring function (eig/entropy/frontier/random/ssmi,
/// default "eig").
///
/// All definitions — including every parameter, publisher, subscription and
/// timer — live in exploration_planner_node.cpp. This header only declares the
/// interface; nothing here is defined inline.
///
/// Map ingest: in "dscovox" mode the planner SUBSCRIBES to the fused ScovoxMap
/// topic published by the dscovox mapping node (latched QoS) and rebuilds a
/// local, ROI-clipped MapCache from it each PLAN tick. (Previously it pulled the
/// region synchronously via the GetRegion service, which forced a
/// MultiThreadedExecutor; the topic path is non-blocking and runs single-
/// threaded.) In "logodds" mode it subscribes to a log-odds point cloud instead.

#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <scovox_msgs/msg/scovox_map.hpp>
#include <scovox_msgs/msg/robot_intent.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "explo_planner/map_cache.hpp"
#include "explo_planner/candidate_generator.hpp"
#include "explo_planner/fov_evaluator.hpp"
#include "explo_planner/scoring.hpp"
#include "explo_planner/metrics_logger.hpp"
#include "explo_planner/cost_grid.hpp"
#include "explo_planner/coordination.hpp"
#include "explo_planner/failed_goal_blacklist.hpp"

namespace explo_planner {

enum class State {
  WAIT_FOR_MAP,
  PLAN,
  NAVIGATE,
  INTEGRATE,
  LOG_STEP,
  DONE
};

class ExplorationPlannerNode : public rclcpp::Node {
public:
  ExplorationPlannerNode();

private:
  // ================================================================
  // Map ingest
  // ================================================================
  // Cache the latest fused map received on the dscovox topic. The grid is
  // rebuilt from it (ROI-clipped) in loadLatestMap() at the start of each PLAN
  // tick, so a full rebuild doesn't run on every incoming message.
  void onScovoxMap(const scovox_msgs::msg::ScovoxMap::SharedPtr& msg);
  // Rebuild map_cache_ from the latest cached ScovoxMap, clipped to the ROI.
  // Returns false if no map has been received yet.
  bool loadLatestMap();
  void onLogOddsCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

  // Trajectory-level scoring (SSMI ablation; all planner types).
  bool scoreTrajectory(std::vector<CandidateViewpoint>& candidates);

  // ================================================================
  // State machine
  // ================================================================
  void tick();
  void transitionTo(State s);
  void doPlan();
  void doNavigate();
  void doIntegrate();
  void doLogStep();

  // PLAN helpers. The cell classification + ROI unknown-fraction math is pure
  // (see plan_map_query.hpp); these members are thin wrappers that supply the
  // latched planning_map and ROI and handle the no-map case.
  double unknownFractionInRoi() const;
  bool isCellFree(const Eigen::Vector3f& pos) const;
  bool isCellOccupied(const Eigen::Vector3f& pos) const;

  // NAVIGATE helpers
  void failGoal(const char* reason, double elapsed);
  void heartbeatTick();

  // Pose / publishing helpers
  void updatePoseFromTF();
  void trackDistance();
  static geometry_msgs::msg::Quaternion yawToQuat(float yaw);
  void publishGoal(const CandidateViewpoint& vp);
  void publishCandidateViz(const std::vector<CandidateViewpoint>& candidates);

  // --- Parameters ---
  std::string planner_type_;
  std::string map_type_;
  std::string robot_name_;
  std::string output_csv_;
  std::string map_frame_;
  std::string base_frame_;
  int    max_steps_;
  double map_resolution_;
  double goal_xy_tol_;
  double goal_yaw_tol_;
  double integrate_wait_;
  double nav_speed_est_mps_;
  double nav_safety_factor_;
  double nav_min_timeout_sec_;
  double nav_max_timeout_sec_;
  double progress_window_sec_;
  double progress_min_distance_m_;
  double failed_goal_radius_m_;
  double failed_goal_ttl_sec_;
  double done_unknown_fraction_;
  int    done_min_consecutive_steps_;
  // When true (default) the planning_map is a hard startup precondition and
  // drives the candidate free/occupied filter + cost-grid reachability. When
  // false the planning_map is best-effort: still used whenever it is being
  // published, but if absent the planner warns and falls back to straight-line
  // distances (no 2D obstacle / reachability filtering), so it can run on an
  // external map or with no planning map at all.
  bool   require_planning_map_{true};
  // Best-effort grace window (seconds): when require_planning_map_ is false and
  // map + pose are ready but the planning_map hasn't arrived yet, wait this long
  // for it before starting in straight-line fallback. Ignored when
  // require_planning_map_ is true (then we wait for it indefinitely).
  double planning_map_wait_sec_{5.0};
  bool   shutdown_requested_{false};

  // Utility / coordination params (cached so doPlan() doesn't re-query).
  double utility_alpha_info_  = 1.0;
  double utility_beta_cost_   = 1.0;
  double cost_grid_radius_cap_m_ = 0.0;
  bool   trajectory_scoring_   = false;
  double trajectory_sample_spacing_m_ = 1.5;
  bool   coord_enabled_        = false;
  double coord_claim_radius_m_ = 0.0;
  double coord_claim_ttl_sec_  = 5.0;
  double coord_heartbeat_hz_   = 1.0;
  std::string coord_intent_topic_;
  // ROI bounds — used to constrain candidate generation, the FOV raycast and
  // (now) the clip applied when ingesting the fused map topic into map_cache_.
  float roi_min_x_ = -15.0f;
  float roi_max_x_ =  15.0f;
  float roi_min_y_ = -15.0f;
  float roi_max_y_ =  15.0f;
  float roi_min_z_ =  -0.5f;
  float roi_max_z_ =   2.0f;

  // --- Components ---
  std::unique_ptr<MapCache> map_cache_;
  std::unique_ptr<CandidateGenerator> candidate_gen_;
  std::unique_ptr<FovEvaluator> fov_eval_;
  ScoreFn score_fn_;
  std::unique_ptr<MetricsLogger> logger_;
  std::unique_ptr<CostGrid> cost_grid_;
  std::unique_ptr<Coordination> coord_;

  // --- State ---
  State state_ = State::WAIT_FOR_MAP;
  int   step_  = 0;
  bool  have_pose_ = false;
  bool  have_map_  = false;
  bool  have_plan_map_ = false;
  // WAIT_FOR_MAP grace-window anchor: time map + pose first became ready, so the
  // best-effort planning_map wait is measured from then (see planning_map_wait_sec_).
  bool  others_ready_seen_ = false;
  rclcpp::Time others_ready_time_;
  Eigen::Vector3f latest_pos_ = Eigen::Vector3f::Zero();
  float latest_yaw_ = 0.0f;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_plan_map_;
  CandidateViewpoint current_goal_;
  rclcpp::Time state_enter_time_;
  float cumulative_distance_ = 0.0f;
  Eigen::Vector3f prev_pos_ = Eigen::Vector3f::Zero();
  bool  first_pos_ = true;
  std::mt19937 rng_;

  // Recently-failed goals (TTL + radius blacklist).
  FailedGoalBlacklist failed_goals_;

  // Per-navigate-cycle state for the smart timeout.
  double nav_budget_sec_ = 0.0;
  rclcpp::Time progress_check_time_;
  float progress_check_dist_ = 0.0f;

  // Coverage termination streak.
  int coverage_done_streak_ = 0;

  // Per-tick utility / coord diagnostics (filled by doPlan, drained by
  // doLogStep into the StepMetrics row).
  float pending_mean_info_gain_       = 0.0f;
  float pending_mean_path_cost_       = 0.0f;
  float pending_selected_info_gain_   = 0.0f;
  float pending_selected_path_cost_   = 0.0f;
  int   pending_rejected_by_minpos_      = 0;
  int   pending_rejected_by_unreachable_ = 0;

  // Cached active intent so the heartbeat timer can re-publish without
  // touching planning state.
  scovox_msgs::msg::RobotIntent current_intent_msg_;
  bool   have_active_intent_ = false;

  // --- ROS interfaces ---
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // Fused-map topic subscription (dscovox mode) + the latest message received.
  // ingested_scovox_map_ is the message currently built into map_cache_; when it
  // still equals latest_scovox_map_ the grid is up to date and loadLatestMap()
  // skips the (expensive) rebuild.
  rclcpp::Subscription<scovox_msgs::msg::ScovoxMap>::SharedPtr scovox_map_sub_;
  scovox_msgs::msg::ScovoxMap::SharedPtr latest_scovox_map_;
  scovox_msgs::msg::ScovoxMap::SharedPtr ingested_scovox_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr logodds_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr plan_map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<scovox_msgs::msg::RobotIntent>::SharedPtr intent_pub_;
  rclcpp::Subscription<scovox_msgs::msg::RobotIntent>::SharedPtr intent_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

} // namespace explo_planner
