#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <random>
#include <algorithm>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <scovox_msgs/msg/scovox_map.hpp>
#include <scovox_msgs/msg/robot_intent.hpp>
#include <scovox_msgs/srv/get_region.hpp>
#include <scovox/uncertainty.hpp>
#include <scovox/voxel.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "explo_planner/map_cache.hpp"
#include "explo_planner/candidate_generator.hpp"
#include "explo_planner/fov_evaluator.hpp"
#include "explo_planner/scoring.hpp"
#include "explo_planner/metrics_logger.hpp"
#include "explo_planner/cost_grid.hpp"
#include "explo_planner/coordination.hpp"

namespace explo_planner {

enum class State {
  WAIT_FOR_MAP,
  PLAN,
  NAVIGATE,
  INTEGRATE,
  LOG_STEP,
  DONE
};

class ExplorationPlannerCompNode : public rclcpp::Node {
public:
  ExplorationPlannerCompNode()
      : Node("exploration_planner_comp"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_),
        rng_(std::random_device{}()) {
    // --- Parameters ---
    auto dp = [&](auto n, auto d) {
      return this->declare_parameter<decltype(d)>(n, d);
    };

    planner_type_ = dp("planner_type", std::string("eig"));
    map_type_     = dp("map_type", std::string("dscovox"));
    max_steps_    = dp("max_steps", 200);
    robot_name_   = dp("robot_name", std::string("atlas"));
    output_csv_   = dp("output_csv", std::string("/tmp/exploration.csv"));
    map_resolution_ = dp("map_resolution", 0.10);
    map_frame_    = dp("map_frame", std::string("map"));
    base_frame_   = dp("base_frame", std::string(""));
    if (base_frame_.empty()) base_frame_ = robot_name_ + "/base_link";

    // Navigation
    goal_xy_tol_  = dp("goal_xy_tolerance", 0.3);
    goal_yaw_tol_ = dp("goal_yaw_tolerance", 0.2);
    integrate_wait_ = dp("integrate_wait", 2.0);

    // Distance-budgeted navigate timeout. The total budget for a NAVIGATE
    // cycle is computed at entry from straight-line distance to the goal:
    //   budget = clamp(dist / speed_est * safety, nav_min, nav_max)
    // so a 2 m hop gets a small budget and an 8 m hop gets a larger one,
    // instead of every goal sharing the same fixed timeout.
    nav_speed_est_mps_  = dp("nav_speed_estimate_mps", 0.5);
    nav_safety_factor_  = dp("nav_safety_factor", 2.0);
    nav_min_timeout_sec_ = dp("nav_min_timeout_sec", 8.0);
    nav_max_timeout_sec_ = dp("nav_max_timeout_sec", 60.0);

    // No-progress watchdog. Independent of the total budget: if the robot
    // hasn't accumulated at least progress_min_distance_m of travel within
    // progress_window_sec, declare the goal failed early. Catches the
    // wedged-robot case (same pose forever) in seconds instead of waiting
    // out the full budget. Mirrors the navigator's progress check.
    progress_window_sec_   = dp("progress_window_sec", 6.0);
    progress_min_distance_m_ = dp("progress_min_distance_m", 0.3);

    // Failed-goal blacklist. When a navigate cycle times out before reaching
    // the goal, the goal position is parked here for `failed_goal_ttl_sec`
    // seconds; subsequent picks reject any candidate within
    // `failed_goal_radius_m` of a non-expired entry. Prevents the planner
    // from re-picking the same unreachable goal forever when the robot is
    // physically stuck (same pose -> same scores -> same pick loop).
    failed_goal_radius_m_ =
        dp("failed_goal_radius_m", 2.0);
    failed_goal_ttl_sec_ =
        dp("failed_goal_ttl_sec", 60.0);

    // Coverage-based termination. Each PLAN tick we count how many cells
    // inside the ROI of the latched planning_map are still unknown (-1).
    // When the unknown fraction stays below `done_unknown_fraction` for
    // `done_min_consecutive_steps` planning cycles in a row we declare the
    // map saturated and transition to DONE. EIG/entropy scores don't fall
    // sharply as the map saturates (the FOV raycast always finds *some*
    // unobserved voxels at the cone edge), so unknown fraction is the
    // reliable signal here. Set done_unknown_fraction <= 0 to disable.
    done_unknown_fraction_ =
        dp("done_unknown_fraction", 0.05);
    done_min_consecutive_steps_ =
        dp("done_min_consecutive_steps", 3);

    // Candidate generation
    CandidateConfig ccfg;
    ccfg.n_radial   = dp("candidate_n_radial", 8);
    ccfg.n_rings    = dp("candidate_n_rings", 3);
    ccfg.min_radius = static_cast<float>(dp("candidate_min_radius", 2.0));
    ccfg.max_radius = static_cast<float>(dp("candidate_max_radius", 8.0));
    ccfg.n_yaw      = dp("candidate_n_yaw", 4);
    ccfg.robot_z    = static_cast<float>(dp("candidate_robot_z", 0.3));
    ccfg.occ_thresh = static_cast<float>(dp("candidate_occ_thresh", 0.7));
    ccfg.ground_z   = static_cast<float>(dp("candidate_ground_z", 0.15));
    ccfg.enable_polar = dp("candidate_enable_polar", true);
    // Region of interest. Defaults bound the robot to a 30x30 m square
    // centred on the world origin. Must match the dscovox planning_map
    // size + origin so the global planner is constrained to the same area.
    ccfg.roi_min_x  = static_cast<float>(dp("roi_min_x", -15.0));
    ccfg.roi_max_x  = static_cast<float>(dp("roi_max_x",  15.0));
    ccfg.roi_min_y  = static_cast<float>(dp("roi_min_y", -15.0));
    ccfg.roi_max_y  = static_cast<float>(dp("roi_max_y",  15.0));

    // Vertical ROI band. In dscovox mode this is the z-slab GetRegion fetches
    // into the local map_cache_, and it bounds the FOV raycast, frontier
    // extraction, and map-stats volume. Keep it to the robot-relevant band so
    // the spurious vertical LiDAR smear above the robot isn't scored as
    // explorable space; FOV rays leaving the band are clipped (treated as
    // empty — no info gain, no occlusion).
    roi_min_z_ = static_cast<float>(dp("roi_min_z", -0.5));
    roi_max_z_ = static_cast<float>(dp("roi_max_z",  2.0));

    // FOV evaluation
    FovConfig fcfg;
    fcfg.hfov      = static_cast<float>(dp("fov_hfov", 1.047));
    fcfg.vfov      = static_cast<float>(dp("fov_vfov", 0.785));
    fcfg.min_range = static_cast<float>(dp("fov_min_range", 0.3));
    fcfg.max_range = static_cast<float>(dp("fov_max_range", 10.0));
    fcfg.h_rays    = dp("fov_h_rays", 16);
    fcfg.v_rays    = dp("fov_v_rays", 12);
    fcfg.occ_stop  = static_cast<float>(dp("fov_occ_stop", 0.7));
    fcfg.roi_min_x = ccfg.roi_min_x;
    fcfg.roi_max_x = ccfg.roi_max_x;
    fcfg.roi_min_y = ccfg.roi_min_y;
    fcfg.roi_max_y = ccfg.roi_max_y;
    fcfg.roi_min_z = roi_min_z_;
    fcfg.roi_max_z = roi_max_z_;

    // Normalised utility: U = α·(info/max_info) − β·(cost/max_cost).
    // Always on (single-robot too); only the MinPos branch in doPlan is
    // gated on coordination_enabled.
    utility_alpha_info_ = dp("utility_alpha_info", 1.0);
    utility_beta_cost_  = dp("utility_beta_cost",  1.0);

    // 0 = auto -> candidate_max_radius + 2 m slack at flood time.
    cost_grid_radius_cap_m_ = dp("cost_grid_radius_cap_m", 0.0);

    // Trajectory-level scoring (SSMI ablation). When enabled, info_gain
    // for each candidate is the sum of score_fn evaluated at sampled poses
    // along the Dijkstra path, not just the endpoint. Evaluated locally via
    // FovEvaluator on map_cache_ for every map_type.
    trajectory_scoring_ = dp("trajectory_scoring", false);
    trajectory_sample_spacing_m_ = dp("trajectory_sample_spacing_m", 1.5);

    // Multi-robot coordination params. coord_claim_radius_m = 0 -> auto =
    // fov_max_range (Burgard et al. 2005 ties the discount kernel to sensor
    // range).
    coord_enabled_         = dp("coordination_enabled", false);
    coord_claim_radius_m_  = dp("coord_claim_radius_m", 0.0);
    coord_claim_ttl_sec_   = dp("coord_claim_ttl_sec", 5.0);
    coord_intent_topic_    = dp("coord_intent_topic",
                                std::string("/exploration/intents"));
    coord_heartbeat_hz_    = dp("coord_heartbeat_hz", 1.0);

    // Auto-resolve "0 means auto" knobs now that the source values are
    // declared. Cache them so the doPlan tick path doesn't re-query.
    if (cost_grid_radius_cap_m_ <= 0.0) {
      cost_grid_radius_cap_m_ = ccfg.max_radius + 2.0;
    }
    if (coord_claim_radius_m_ <= 0.0) {
      coord_claim_radius_m_ = fcfg.max_range;
    }

    // --- Components ---
    map_cache_ = std::make_unique<MapCache>(map_resolution_);
    candidate_gen_ = std::make_unique<CandidateGenerator>(ccfg);
    fov_eval_ = std::make_unique<FovEvaluator>(fcfg);
    score_fn_ = scoring::create(planner_type_);
    logger_ = std::make_unique<MetricsLogger>(output_csv_);
    cost_grid_ = std::make_unique<CostGrid>();
    coord_ = std::make_unique<Coordination>(coord_enabled_, robot_name_);

    // --- ROS interfaces ---
    std::string logodds_pc_topic = dp("logodds_pointcloud_topic",
        std::string("/" + robot_name_ + "/log_odds_node/pointcloud"));
    std::string goal_topic = dp("goal_topic",
        std::string("/" + robot_name_ + "/goal_pose"));
    // The same OccupancyGrid the global planner consumes. We use it to
    // reject candidate viewpoints whose cell is occupied/inflated/unknown
    // before publishing them as goals.
    std::string planning_map_topic = dp("planning_map_topic",
        std::string("/" + robot_name_ + "/dscovox_node/planning_map"));

    // Cache ROI bounds for the GetRegion service call.
    roi_min_x_ = ccfg.roi_min_x;
    roi_max_x_ = ccfg.roi_max_x;
    roi_min_y_ = ccfg.roi_min_y;
    roi_max_y_ = ccfg.roi_max_y;

    if (map_type_ == "dscovox") {
      // The dscovox mapping node owns voxel fusion (multi-robot consensus).
      // Each PLAN tick the planner pulls the fused occupancy voxels for its
      // ROI via the GetRegion service into a local MapCache, then runs FOV
      // raycasting + scoring + frontier extraction locally. Consensus is
      // preserved (the fused grid is the data source); scoring lives entirely
      // in the planner. Separate callback group so the sync wait in
      // refreshRegion() doesn't deadlock the MultiThreadedExecutor.
      std::string region_srv = dp("get_region_service",
          std::string("/" + robot_name_ + "/dscovox_node/get_region"));
      srv_cb_group_ = create_callback_group(
          rclcpp::CallbackGroupType::MutuallyExclusive);
      get_region_client_ = create_client<scovox_msgs::srv::GetRegion>(
          region_srv, rmw_qos_profile_services_default, srv_cb_group_);
    } else {
      logodds_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          logodds_pc_topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            onLogOddsCloud(msg);
          });
    }

    plan_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        planning_map_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          latest_plan_map_ = msg;
          have_plan_map_ = true;
        });

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10);

    // Direct cmd_vel for in-place rotation after reaching goal XY.
    std::string cmd_vel_topic = dp("cmd_vel_topic",
        std::string("/" + robot_name_ + "/cmd_vel"));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10);

    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/candidates", 10);

    // --- Intent pub/sub (always wired; payload only consumed when
    //     coord_->enabled() is true). Reliable + KeepLast(team_cap) so a
    //     single missed broadcast doesn't drop a peer claim, but the queue
    //     stays bounded. Self-broadcasts are filtered by Coordination::onIntent.
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(8)).reliable();
      intent_pub_ = create_publisher<scovox_msgs::msg::RobotIntent>(
          coord_intent_topic_, qos);
      intent_sub_ = create_subscription<scovox_msgs::msg::RobotIntent>(
          coord_intent_topic_, qos,
          [this](scovox_msgs::msg::RobotIntent::SharedPtr msg) {
            if (coord_) coord_->onIntent(*msg);
          });
    }

    // --- State machine timer (10 Hz, sim time) ---
    tick_timer_ = rclcpp::create_timer(
        this, get_clock(), std::chrono::milliseconds(100),
        [this] { tick(); });

    // --- Coord heartbeat. Re-publishes the active claim while NAVIGATE-ing
    //     so peers don't lose it through TTL. Period = 1 / coord_heartbeat_hz.
    if (coord_enabled_ && coord_heartbeat_hz_ > 0.0) {
      auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / coord_heartbeat_hz_));
      heartbeat_timer_ = rclcpp::create_timer(
          this, get_clock(), period,
          [this] { heartbeatTick(); });
    }

    RCLCPP_INFO(get_logger(),
        "Exploration comparison planner ready: type=%s map=%s max_steps=%d "
        "frame=%s base=%s",
        planner_type_.c_str(), map_type_.c_str(), max_steps_,
        map_frame_.c_str(), base_frame_.c_str());
  }

private:
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
  bool   shutdown_requested_{false};

  // Utility / coordination params (declared in constructor; cached here
  // so doPlan() doesn't re-query the parameter server every tick).
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
  Eigen::Vector3f latest_pos_ = Eigen::Vector3f::Zero();
  float latest_yaw_ = 0.0f;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_plan_map_;
  CandidateViewpoint current_goal_;
  rclcpp::Time state_enter_time_;
  float cumulative_distance_ = 0.0f;
  Eigen::Vector3f prev_pos_ = Eigen::Vector3f::Zero();
  bool  first_pos_ = true;
  std::mt19937 rng_;

  // Recently-failed goals. Each entry is (xy position, time the goal was
  // declared failed). Pruned in pruneFailedGoals(); used by isNearFailedGoal()
  // to filter the candidate picker.
  std::deque<std::pair<Eigen::Vector3f, rclcpp::Time>> failed_goals_;

  // Per-navigate-cycle state for the smart timeout. Recomputed each time
  // we enter NAVIGATE.
  double nav_budget_sec_ = 0.0;
  rclcpp::Time progress_check_time_;
  float progress_check_dist_ = 0.0f;

  // Coverage termination state. Counts consecutive PLAN ticks where the
  // unknown fraction in the planning_map ROI was below the threshold.
  int coverage_done_streak_ = 0;

  // Per-tick utility / coord diagnostics. Populated by doPlan(), drained
  // by doLogStep() into the StepMetrics row.
  float pending_mean_info_gain_       = 0.0f;
  float pending_mean_path_cost_       = 0.0f;
  float pending_selected_info_gain_   = 0.0f;
  float pending_selected_path_cost_   = 0.0f;
  int   pending_rejected_by_minpos_      = 0;
  int   pending_rejected_by_unreachable_ = 0;

  // Cached active intent so the heartbeat timer can re-publish without
  // touching planning state. Owned by doPlan -> publish path.
  scovox_msgs::msg::RobotIntent current_intent_msg_;
  bool   have_active_intent_ = false;

  // --- ROS interfaces ---
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Client<scovox_msgs::srv::GetRegion>::SharedPtr get_region_client_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr logodds_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr plan_map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<scovox_msgs::msg::RobotIntent>::SharedPtr intent_pub_;
  rclcpp::Subscription<scovox_msgs::msg::RobotIntent>::SharedPtr intent_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // ================================================================
  // GetRegion fetch — pull the fused ROI voxels into the local MapCache
  // ================================================================

  // Synchronous call to the dscovox GetRegion service. Pulls the fused
  // (multi-robot consensus) occupancy voxels within the planner's ROI into
  // the local map_cache_, so FOV raycasting + scoring + frontier extraction
  // can all run locally in the planner. Returns false on timeout/error and
  // sets have_map_ on success. Runs on srv_cb_group_ so the blocking wait
  // doesn't deadlock the MultiThreadedExecutor.
  bool refreshRegion() {
    if (!get_region_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "GetRegion service not ready");
      return false;
    }
    auto request = std::make_shared<scovox_msgs::srv::GetRegion::Request>();
    request->min_corner.x = roi_min_x_;
    request->min_corner.y = roi_min_y_;
    request->min_corner.z = roi_min_z_;
    request->max_corner.x = roi_max_x_;
    request->max_corner.y = roi_max_y_;
    request->max_corner.z = roi_max_z_;

    auto future = get_region_client_->async_send_request(request);
    auto status = future.wait_for(std::chrono::seconds(10));
    if (status != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "GetRegion timed out (10s)");
      return false;
    }
    auto response = future.get();
    if (!response) {
      RCLCPP_WARN(get_logger(), "GetRegion: null response");
      return false;
    }
    // Rebuild the local grid from the fused ROI voxels (a_occ/a_free per
    // voxel). This is the only voxel data that crosses the wire — bounded to
    // the ROI bbox and fetched once per PLAN cycle. The [roi_min_z_,
    // roi_max_z_] band defines one consistent observation volume: it is what
    // gets scored (FovEvaluator clips rays to the same band), frontier-
    // extracted, and logged (doLogStep map stats).
    map_cache_->updateFromScovoxMap(response->map);
    // Gate startup on actually having fused data: an empty grid (no robot has
    // published a map yet) leaves have_map_ false so WAIT_FOR_MAP keeps
    // waiting. Once set it stays true; a transient empty fetch mid-run is fine.
    if (!response->map.voxels.empty()) have_map_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "GetRegion: loaded %zu fused voxels into local map_cache_",
        response->map.voxels.size());
    return true;
  }

  // ================================================================
  // Trajectory-level scoring (all planner types)
  // ================================================================

  /// Score candidates by summing per-voxel scores at poses sampled along
  /// the Dijkstra path to each candidate. Evaluates every sample locally via
  /// FovEvaluator on map_cache_ (the fused grid in dscovox mode, the log-odds
  /// cloud otherwise). Always returns true (no service dependency).
  bool scoreTrajectory(std::vector<CandidateViewpoint>& candidates) {
    const float spacing =
        static_cast<float>(trajectory_sample_spacing_m_);

    // Build trajectory samples for all candidates. Each sample remembers
    // which parent candidate it belongs to.
    struct TrajSample {
      CandidateViewpoint vp;
      size_t parent_idx;
    };
    std::vector<TrajSample> samples;
    samples.reserve(candidates.size() * 8);

    for (size_t ci = 0; ci < candidates.size(); ++ci) {
      auto path = cost_grid_->extractPath(candidates[ci].position);
      if (path.size() < 2) {
        candidates[ci].score = 0.0f;  // unreachable → −∞ utility later
        continue;
      }

      float accum = 0.0f;
      for (size_t j = 1; j < path.size(); ++j) {
        float dx = path[j].x() - path[j - 1].x();
        float dy = path[j].y() - path[j - 1].y();
        accum += std::sqrt(dx * dx + dy * dy);

        bool at_end = (j == path.size() - 1);
        if (accum >= spacing || at_end) {
          CandidateViewpoint sample;
          sample.position = path[j];
          sample.position.z() = candidates[ci].position.z();

          if (at_end) {
            sample.yaw = candidates[ci].yaw;
          } else {
            float fdx = path[j + 1].x() - path[j].x();
            float fdy = path[j + 1].y() - path[j].y();
            sample.yaw = std::atan2(fdy, fdx);
          }

          samples.push_back({sample, ci});
          accum = 0.0f;
        }
      }
    }

    if (samples.empty()) return true;  // all candidates unreachable

    // Zero out scores — we'll accumulate from samples.
    for (auto& c : candidates) c.score = 0.0f;

    // Evaluate each trajectory sample on the local map_cache_.
    for (const auto& s : samples) {
      float sample_score;
      if (planner_type_ == "ssmi") {
        sample_score = fov_eval_->evaluateSSMI(s.vp, *map_cache_).total_score;
      } else {
        sample_score = fov_eval_->evaluate(s.vp, *map_cache_, score_fn_).total_score;
      }
      candidates[s.parent_idx].score += sample_score;
    }

    int traj_scored = 0;
    for (const auto& c : candidates) {
      if (c.score > 0.0f) ++traj_scored;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "Trajectory scoring (%s): %zu samples across %d/%zu candidates "
        "(spacing=%.1f m)",
        planner_type_.c_str(),
        samples.size(), traj_scored, candidates.size(), spacing);

    return true;
  }

  // ================================================================
  // Map callbacks
  // ================================================================

  void onLogOddsCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    map_cache_->updateFromLogOddsCloud(*msg, map_resolution_);
    have_map_ = true;
  }

  // ================================================================
  // State machine
  // ================================================================

  void tick() {
    updatePoseFromTF();
    trackDistance();

    switch (state_) {
      case State::WAIT_FOR_MAP:
        // dscovox mode: bootstrap have_map_ by pulling the first fused ROI via
        // GetRegion once pose + planning_map are available. (logodds mode sets
        // have_map_ from the pointcloud callback instead.)
        if (map_type_ == "dscovox" && !have_map_ &&
            have_pose_ && have_plan_map_) {
          refreshRegion();  // sets have_map_ on success
        }
        if (have_map_ && have_pose_ && have_plan_map_) {
          RCLCPP_INFO(get_logger(),
              "Map, planning_map and pose received. Starting exploration.");
          transitionTo(State::PLAN);
        }
        break;

      case State::PLAN:
        doPlan();
        break;

      case State::NAVIGATE:
        doNavigate();
        break;

      case State::INTEGRATE:
        doIntegrate();
        break;

      case State::LOG_STEP:
        doLogStep();
        break;

      case State::DONE:
        if (!shutdown_requested_) {
          shutdown_requested_ = true;
          RCLCPP_INFO(get_logger(),
              "Exploration finished. Shutting down planner node.");
          rclcpp::shutdown();
        }
        break;
    }
  }

  void transitionTo(State s) {
    state_ = s;
    state_enter_time_ = this->now();
  }

  // ================================================================
  // PLAN state
  // ================================================================

  void doPlan() {
    auto plan_start = this->now();

    pruneFailedGoals();
    if (coord_) coord_->prune(plan_start);

    // dscovox mode: pull the fused ROI voxels into the local map_cache_ so
    // frontier extraction + FOV scoring below run on fresh consensus data.
    if (map_type_ == "dscovox" && !refreshRegion()) {
      RCLCPP_WARN(get_logger(), "GetRegion failed; retrying next tick.");
      return;
    }

    // Coverage saturation check. Runs before candidate generation so we
    // can short-circuit out of PLAN entirely once the ROI is fully known.
    // Requires N consecutive low-unknown ticks to avoid premature DONE
    // from a momentary measurement gap.
    if (done_unknown_fraction_ > 0.0) {
      double unk = unknownFractionInRoi();
      if (unk >= 0.0 && unk < done_unknown_fraction_) {
        ++coverage_done_streak_;
        RCLCPP_INFO(get_logger(),
            "Step %d: ROI unknown fraction %.3f < %.3f (streak %d/%d)",
            step_, unk, done_unknown_fraction_,
            coverage_done_streak_, done_min_consecutive_steps_);
        if (coverage_done_streak_ >= done_min_consecutive_steps_) {
          RCLCPP_INFO(get_logger(),
              "Exploration complete: ROI saturated "
              "(unknown=%.3f, %d steps, %.2f m traveled).",
              unk, step_, cumulative_distance_);
          transitionTo(State::DONE);
          return;
        }
      } else {
        coverage_done_streak_ = 0;
      }
    }

    auto robot_pos = latest_pos_;
    float robot_yaw = latest_yaw_;

    // Generate candidates: a polar grid of viewpoints around the robot
    // (local EIG hops) PLUS frontier centroids anywhere in the ROI (long-
    // range targets for escaping local IG maxima). In dscovox mode the 3D
    // occupancy check is skipped (nullptr map); the 2D planning_map filter
    // below handles it. Frontiers are extracted locally from map_cache_ (the
    // fused grid pulled via refreshRegion() in dscovox mode, the log-odds
    // cloud otherwise).
    auto candidates = (map_type_ == "dscovox")
        ? candidate_gen_->generate(robot_pos, robot_yaw, nullptr)
        : candidate_gen_->generate(robot_pos, robot_yaw, map_cache_.get());
    size_t n_radial = candidates.size();
    auto frontiers = map_cache_->findFrontierCentroids(
        roi_min_z_, roi_max_z_, 5.0f);
    candidate_gen_->addFrontierCandidates(candidates, frontiers, robot_pos);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "Candidates: %zu radial + %zu frontier centroids = %zu total",
        n_radial, candidates.size() - n_radial, candidates.size());

    if (candidates.empty()) {
      RCLCPP_WARN(get_logger(), "No valid candidates. Retrying next tick.");
      return;
    }

    // Build the bounded cost grid from the latched planning_map. ~5 ms once
    // per PLAN tick at the auto bound, then O(1) per-candidate lookups.
    // Random planner skips the cost grid entirely (no utility math).
    bool skip_reachability = false;
    if (planner_type_ != "random" && latest_plan_map_) {
      cost_grid_->build(*latest_plan_map_);
      cost_grid_->floodFrom(robot_pos,
                            static_cast<float>(cost_grid_radius_cap_m_));
      size_t reached = cost_grid_->reachedCellCount();
      // If the flood barely escaped the source (e.g. robot is surrounded
      // by inflated cells in a dense forest), the reachability filter would
      // reject every candidate. Fall back to no-reachability filtering for
      // this tick so the planner can still make progress.
      constexpr size_t kMinReachedForFilter = 10;
      if (reached < kMinReachedForFilter) {
        RCLCPP_WARN(get_logger(),
            "CostGrid: flood reached only %zu cells (robot=%.2f,%.2f "
            "robotCost=%.2f map %dx%d origin=%.1f,%.1f). "
            "Skipping reachability filter this tick.",
            reached, robot_pos.x(), robot_pos.y(),
            cost_grid_->costTo(robot_pos),
            cost_grid_->dimsX(), cost_grid_->dimsY(),
            latest_plan_map_->info.origin.position.x,
            latest_plan_map_->info.origin.position.y);
        skip_reachability = true;
      }
    }

    // Evaluate candidates -> populates per-candidate FOV info gain.
    //
    // trajectory_scoring (param, default false): when true, info_gain is the
    // sum of scores at poses sampled along the Dijkstra path, not just the
    // endpoint. Otherwise endpoint scoring. Both run locally via FovEvaluator
    // on map_cache_ (the fused ROI grid in dscovox mode).
    if (planner_type_ != "random") {
      const bool use_traj = trajectory_scoring_
                            && cost_grid_ && !skip_reachability;

      if (use_traj) {
        if (!scoreTrajectory(candidates)) {
          RCLCPP_WARN(get_logger(),
              "Trajectory scoring failed, retrying next tick.");
          return;
        }
      } else if (planner_type_ == "ssmi") {
        // SSMI endpoint scoring on the local map.
        fov_eval_->evaluateAllSSMI(candidates, *map_cache_);
      } else {
        // Endpoint scoring on the local map.
        fov_eval_->evaluateAll(candidates, *map_cache_, score_fn_);
      }
    }
    // Random planner: no scoring; map stats are computed locally in doLogStep.

    // SSMI-style denominator-normalised utility (Asgharivaskasi & Atanasov,
    // TRO 2023):
    //
    //   U(c) = info_gain(c) / (ε + path_cost(c))
    //
    // Information per unit distance — longer paths dilute their score.
    // ε (0.1 m) prevents division-by-zero for candidates at the robot's
    // feet and matches the SSMI reference implementation.
    //
    // Unreachable candidates (inf cost) get U = −∞ and sort to the bottom.
    // Random planner skips utility entirely (shuffled selection).
    constexpr float kCostEpsilon = 0.1f;

    std::vector<float> info_gain(candidates.size(), 0.0f);
    std::vector<float> path_cost(candidates.size(), 0.0f);
    float sum_info = 0.0f;
    float sum_cost_finite = 0.0f;
    int   n_cost_finite = 0;
    if (planner_type_ != "random") {
      // Single pass: collect raw info_gain + path_cost, accumulate the means,
      // and overwrite each candidate's score with the SSMI-style utility.
      // info_gain[i] is cached before the score is overwritten.
      for (size_t i = 0; i < candidates.size(); ++i) {
        info_gain[i] = candidates[i].score;  // from FovEvaluator
        float c;
        if (skip_reachability || !cost_grid_) {
          float dx = candidates[i].position.x() - robot_pos.x();
          float dy = candidates[i].position.y() - robot_pos.y();
          c = std::sqrt(dx * dx + dy * dy);
        } else {
          c = cost_grid_->costTo(candidates[i].position);
        }
        path_cost[i] = c;
        sum_info += info_gain[i];
        if (std::isfinite(c)) {
          sum_cost_finite += c;
          ++n_cost_finite;
          candidates[i].score = info_gain[i] / (kCostEpsilon + c);
        } else {
          // Unreachable (inf cost): U = -inf, sorts to the bottom.
          candidates[i].score = -std::numeric_limits<float>::infinity();
        }
      }
    }

    // Build a selection order: random planner shuffles, scored planners sort
    // by utility descending. The cost-grid reachability filter runs after
    // the sort as one of four candidate filters in the walk below.
    std::vector<size_t> order(candidates.size());
    std::iota(order.begin(), order.end(), 0);
    if (planner_type_ == "random") {
      std::shuffle(order.begin(), order.end(), rng_);
    } else {
      std::sort(order.begin(), order.end(),
          [&candidates](size_t a, size_t b) {
            return candidates[a].score > candidates[b].score;
          });
    }

    int rejected_map = 0;
    int rejected_blacklist = 0;
    int rejected_unreachable = 0;
    int rejected_minpos = 0;
    int rejected_too_close = 0;
    bool found = false;
    size_t selected_idx = 0;
    for (size_t idx : order) {
      const auto& vp = candidates[idx];
      // 0. Skip candidates at the robot's feet — these are "already reached"
      //    by goal_xy_tolerance so they waste a step without any movement.
      {
        float dx = vp.position.x() - robot_pos.x();
        float dy = vp.position.y() - robot_pos.y();
        if (dx * dx + dy * dy < static_cast<float>(
                goal_xy_tol_ * goal_xy_tol_)) {
          ++rejected_too_close;
          continue;
        }
      }
      // 1. Single-cell free check on the planning_map. Frontier centroid
      //    candidates are exempt from the unknown-cell rejection because
      //    they naturally sit at the boundary where the 2D planning_map
      //    cell is still unknown (-1). They are still rejected if the cell
      //    is occupied/inflated (>= 50).
      if (vp.is_frontier) {
        if (isCellOccupied(vp.position)) { ++rejected_map; continue; }
      } else {
        if (!isCellFree(vp.position)) { ++rejected_map; continue; }
      }
      // 2. Cost-grid reachability — catches free pockets sealed off by
      //    inflated obstacles. Random planner skips because it has no
      //    cost grid built. Also skipped when the flood barely reached
      //    anything (robot trapped in inflation zone).
      if (!skip_reachability && planner_type_ != "random" && cost_grid_ &&
          !cost_grid_->reachable(vp.position)) {
        ++rejected_unreachable;
        continue;
      }
      // 3. Failed-goal blacklist (existing).
      if (isNearFailedGoal(vp.position)) { ++rejected_blacklist; continue; }
      // 4. MinPos peer-claim check (only when coordination is enabled).
      if (coord_ && coord_->enabled()) {
        const auto* peer = coord_->claimMatching(
            vp.position, static_cast<float>(coord_claim_radius_m_));
        if (peer && !coord_->selfWinsAgainst(robot_pos, vp.position,
                                              *peer, robot_name_)) {
          ++rejected_minpos;
          continue;
        }
      }
      current_goal_ = vp;
      selected_idx = idx;
      found = true;
      break;
    }
    if (!found) {
      RCLCPP_WARN(get_logger(),
          "Step %d: all %zu candidates rejected (close=%d map=%d unreach=%d "
          "blk=%d minpos=%d). Retrying next tick.",
          step_, candidates.size(), rejected_too_close,
          rejected_map, rejected_unreachable,
          rejected_blacklist, rejected_minpos);
      return;  // stay in PLAN, retry next tick
    }

    // Drain utility / coord diagnostics into pending_* fields for the
    // upcoming LOG_STEP. doLogStep() will copy these into StepMetrics.
    pending_mean_info_gain_ =
        candidates.empty()
            ? 0.0f
            : sum_info / static_cast<float>(candidates.size());
    pending_mean_path_cost_ =
        n_cost_finite > 0
            ? sum_cost_finite / static_cast<float>(n_cost_finite)
            : 0.0f;
    pending_selected_info_gain_ =
        (planner_type_ == "random") ? 0.0f : info_gain[selected_idx];
    pending_selected_path_cost_ =
        (planner_type_ == "random") ? 0.0f : path_cost[selected_idx];
    pending_rejected_by_minpos_      = rejected_minpos;
    pending_rejected_by_unreachable_ = rejected_unreachable;

    auto plan_end = this->now();
    float plan_ms = static_cast<float>(
        (plan_end - plan_start).nanoseconds() * 1e-6);

    RCLCPP_INFO(get_logger(),
        "Step %d: selected goal (%.2f, %.2f) yaw=%.2f U=%.3f "
        "info=%.2f cost=%.2f [%zu cand, close=%d map=%d unreach=%d blk=%d "
        "minpos=%d, peers=%zu, %.1fms]",
        step_, current_goal_.position.x(), current_goal_.position.y(),
        current_goal_.yaw, current_goal_.score,
        pending_selected_info_gain_, pending_selected_path_cost_,
        candidates.size(), rejected_too_close, rejected_map,
        rejected_unreachable, rejected_blacklist, rejected_minpos,
        coord_ ? coord_->activePeerCount() : 0u, plan_ms);

    publishGoal(current_goal_);
    publishCandidateViz(candidates);

    // Build & publish the multi-robot intent for this goal. Always
    // published (even single-robot) for diagnostic visibility; peers only
    // act on it when their own coord_->enabled() is true.
    if (intent_pub_ && coord_) {
      uint8_t ptype = plannerTypeId();
      current_intent_msg_ = coord_->buildIntent(
          current_goal_, robot_pos, plan_end,
          static_cast<float>(coord_claim_ttl_sec_),
          static_cast<float>(coord_claim_radius_m_),
          ptype, map_frame_);
      intent_pub_->publish(current_intent_msg_);
      have_active_intent_ = true;
    }

    transitionTo(State::NAVIGATE);

    // Initialise smart-timeout state for this NAVIGATE cycle.
    float dx = current_goal_.position.x() - robot_pos.x();
    float dy = current_goal_.position.y() - robot_pos.y();
    float dist = std::sqrt(dx * dx + dy * dy);
    double raw_budget = (dist / std::max(nav_speed_est_mps_, 1e-3))
                        * nav_safety_factor_;
    nav_budget_sec_ = std::clamp(raw_budget,
                                 nav_min_timeout_sec_,
                                 nav_max_timeout_sec_);
    // Both anchors mark NAVIGATE entry; reuse the timestamp transitionTo()
    // just stamped rather than re-reading the clock.
    progress_check_time_ = state_enter_time_;
    progress_check_dist_ = cumulative_distance_;
    RCLCPP_DEBUG(get_logger(),
        "Step %d: nav budget %.1fs for %.2f m goal", step_,
        nav_budget_sec_, dist);
  }

  // Drop expired entries from the failed-goal blacklist. Called once per
  // PLAN cycle so the blacklist self-cleans without extra timers.
  void pruneFailedGoals() {
    auto now = this->now();
    while (!failed_goals_.empty()) {
      double age = (now - failed_goals_.front().second).seconds();
      if (age > failed_goal_ttl_sec_) {
        failed_goals_.pop_front();
      } else {
        break;
      }
    }
  }

  // Returns true if `pos` falls within failed_goal_radius_m_ of any
  // non-expired blacklist entry. Used to skip the top-scoring candidate
  // when it's the same unreachable target the robot just failed to reach.
  bool isNearFailedGoal(const Eigen::Vector3f& pos) const {
    const float r2 = static_cast<float>(failed_goal_radius_m_ *
                                        failed_goal_radius_m_);
    for (const auto& [p, _t] : failed_goals_) {
      float dx = pos.x() - p.x();
      float dy = pos.y() - p.y();
      if (dx * dx + dy * dy < r2) return true;
    }
    return false;
  }

  // Fraction of cells in the ROI bounding box of the latched planning_map
  // whose value is -1 (unknown). Returns -1.0 if no map is available or
  // the ROI doesn't overlap the map. Used by the coverage termination
  // check to detect when the explored area is fully classified as
  // free/occupied — at which point further exploration adds nothing.
  double unknownFractionInRoi() const {
    if (!latest_plan_map_) return -1.0;
    const auto& m = *latest_plan_map_;
    if (m.info.resolution <= 0.0f || m.info.width == 0 || m.info.height == 0)
      return -1.0;

    // Convert ROI world bounds (cached members; params never change at
    // runtime) to grid indices, clipped to the map.
    auto to_gx = [&](float x) {
      return static_cast<int>(std::floor(
          (x - m.info.origin.position.x) / m.info.resolution));
    };
    auto to_gy = [&](float y) {
      return static_cast<int>(std::floor(
          (y - m.info.origin.position.y) / m.info.resolution));
    };
    int gx0 = std::max(0, to_gx(roi_min_x_));
    int gx1 = std::min(static_cast<int>(m.info.width) - 1, to_gx(roi_max_x_));
    int gy0 = std::max(0, to_gy(roi_min_y_));
    int gy1 = std::min(static_cast<int>(m.info.height) - 1, to_gy(roi_max_y_));
    if (gx0 > gx1 || gy0 > gy1) return -1.0;

    int total = 0, unknown = 0;
    for (int gy = gy0; gy <= gy1; ++gy) {
      for (int gx = gx0; gx <= gx1; ++gx) {
        int8_t v = m.data[gy * m.info.width + gx];
        ++total;
        if (v < 0) ++unknown;
      }
    }
    if (total == 0) return -1.0;
    return static_cast<double>(unknown) / static_cast<double>(total);
  }

  // Sentinel returned by planMapCellAt when no planning_map is available or
  // the query falls outside the grid. Distinct from the real cell range
  // (-1 unknown, 0..100 occupancy) so callers can apply their own default.
  static constexpr int8_t kCellNoData = -2;

  // Look up the latched planning_map cell value at world XY. Returns the raw
  // occupancy value (-1 unknown, 0..100 cost) or kCellNoData when there is no
  // map or `pos` is out of bounds. Single source of the world->grid math
  // shared by isCellFree/isCellOccupied.
  int8_t planMapCellAt(const Eigen::Vector3f& pos) const {
    if (!latest_plan_map_) return kCellNoData;
    const auto& m = *latest_plan_map_;
    if (m.info.resolution <= 0.0f || m.info.width == 0 || m.info.height == 0)
      return kCellNoData;
    int gx = static_cast<int>(std::floor(
        (pos.x() - m.info.origin.position.x) / m.info.resolution));
    int gy = static_cast<int>(std::floor(
        (pos.y() - m.info.origin.position.y) / m.info.resolution));
    if (gx < 0 || gy < 0 ||
        gx >= static_cast<int>(m.info.width) ||
        gy >= static_cast<int>(m.info.height))
      return kCellNoData;
    return m.data[gy * m.info.width + gx];
  }

  // Returns true iff the (x,y) of `pos` is a free cell in the latched
  // planning_map. Out-of-bounds, unknown (-1), and occupied/inflated (>=50)
  // cells are all rejected. The map is already inflated by the body radius,
  // so a single-cell check is enough — no need for a footprint sweep here.
  bool isCellFree(const Eigen::Vector3f& pos) const {
    int8_t v = planMapCellAt(pos);  // kCellNoData/unknown both fail v >= 0
    return v >= 0 && v < 50;
  }

  // Returns true iff the cell is known-occupied/inflated (value >= 50).
  // Unlike isCellFree, this returns false for unknown (-1) cells, allowing
  // frontier centroid candidates to pass through unknown territory. No map /
  // out-of-bounds is treated as occupied (the conservative default).
  bool isCellOccupied(const Eigen::Vector3f& pos) const {
    int8_t v = planMapCellAt(pos);
    return v == kCellNoData || v >= 50;
  }

  // ================================================================
  // NAVIGATE state
  // ================================================================

  void doNavigate() {
    // Abort early if the map has updated and the goal cell is now inside
    // an obstacle (or its inflation zone). This avoids wasting time
    // navigating toward goals that were valid at selection but became
    // occupied as the map grew.
    if (isCellOccupied(current_goal_.position)) {
      RCLCPP_WARN(get_logger(),
          "Step %d: goal (%.2f, %.2f) is now inside an obstacle — "
          "aborting navigation and re-planning.",
          step_, current_goal_.position.x(),
          current_goal_.position.y());
      have_active_intent_ = false;
      transitionTo(State::PLAN);
      return;
    }

    auto robot_pos = latest_pos_;
    float dx = robot_pos.x() - current_goal_.position.x();
    float dy = robot_pos.y() - current_goal_.position.y();
    float dist = std::sqrt(dx * dx + dy * dy);

    // Goal reached only when BOTH position and orientation are within
    // tolerance. The yaw check ensures the robot is facing the planned
    // direction before we transition to INTEGRATE, so the sensor actually
    // observes the region the planner scored.
    if (dist < goal_xy_tol_) {
      float yaw_err = std::remainder(latest_yaw_ - current_goal_.yaw,
                                     2.0f * static_cast<float>(M_PI));
      if (std::abs(yaw_err) < goal_yaw_tol_) {
        have_active_intent_ = false;
        RCLCPP_INFO(get_logger(),
            "Goal reached: dist=%.2f yaw_err=%.1f deg", dist,
            yaw_err * 180.0f / static_cast<float>(M_PI));
        transitionTo(State::INTEGRATE);
        return;
      }
      // At XY, waiting for controller node to finish rotating.
      auto now = this->now();
      double elapsed = (now - state_enter_time_).seconds();
      if (elapsed > nav_budget_sec_) {
        failGoal("budget-rotate", elapsed);
      }
      return;
    }

    auto now = this->now();
    double elapsed = (now - state_enter_time_).seconds();

    // 1) Distance-budgeted hard timeout.
    if (elapsed > nav_budget_sec_) {
      failGoal("budget", elapsed);
      return;
    }

    // 2) No-progress watchdog.
    double window_elapsed = (now - progress_check_time_).seconds();
    if (window_elapsed > progress_window_sec_) {
      float delta = cumulative_distance_ - progress_check_dist_;
      if (delta < progress_min_distance_m_) {
        failGoal("no-progress", elapsed);
        return;
      }
      progress_check_time_ = now;
      progress_check_dist_ = cumulative_distance_;
    }

    // Re-publish goal periodically to keep navigator alive
    publishGoal(current_goal_);
  }

  // Park the current goal in the failed-goal blacklist with a tagged reason
  // and transition out of NAVIGATE. Centralised so both timeout paths
  // (budget exceeded / no progress) share the same logging + bookkeeping.
  void failGoal(const char* reason, double elapsed) {
    failed_goals_.emplace_back(current_goal_.position, this->now());
    RCLCPP_WARN(get_logger(),
        "Step %d: navigation failed [%s] after %.1fs at goal (%.2f, %.2f). "
        "Blacklisted; %zu active failed-goal entries.",
        step_, reason, elapsed,
        current_goal_.position.x(), current_goal_.position.y(),
        failed_goals_.size());
    have_active_intent_ = false;  // release the claim on failure
    transitionTo(State::INTEGRATE);
  }

  // Re-publish the active intent on a fixed sim-time cadence so peers
  // don't lose the claim through TTL while we're navigating to it.
  // Stamps the message with the current time so peer expiry resets.
  void heartbeatTick() {
    if (!coord_enabled_) return;
    if (!have_active_intent_) return;
    if (state_ != State::NAVIGATE && state_ != State::INTEGRATE) return;
    if (!intent_pub_) return;
    current_intent_msg_.header.stamp = this->now();
    intent_pub_->publish(current_intent_msg_);
  }

  // Map planner_type_ string to the diagnostic enum used in RobotIntent.
  // 0=eig, 1=entropy, 2=frontier, 3=random.
  uint8_t plannerTypeId() const {
    if (planner_type_ == "eig")      return 0;
    if (planner_type_ == "entropy")  return 1;
    if (planner_type_ == "frontier") return 2;
    if (planner_type_ == "random")   return 3;
    if (planner_type_ == "ssmi")     return 4;
    return 255;
  }

  // ================================================================
  // INTEGRATE state
  // ================================================================

  void doIntegrate() {
    double elapsed = (this->now() - state_enter_time_).seconds();
    if (elapsed >= integrate_wait_) {
      transitionTo(State::LOG_STEP);
    }
  }

  // ================================================================
  // LOG_STEP state
  // ================================================================

  void doLogStep() {
    StepMetrics m;
    m.step = step_;
    m.sim_time_sec = this->now().seconds();
    m.distance_traveled = cumulative_distance_;
    m.selected_score = current_goal_.score;

    // Drain utility / coord diagnostics from doPlan().
    m.mean_info_gain      = pending_mean_info_gain_;
    m.mean_path_cost      = pending_mean_path_cost_;
    m.selected_info_gain  = pending_selected_info_gain_;
    m.selected_path_cost  = pending_selected_path_cost_;
    m.selected_utility    = current_goal_.score;
    m.coord_active_peers  = coord_ ? static_cast<int>(coord_->activePeerCount())
                                    : 0;
    m.rejected_by_minpos        = pending_rejected_by_minpos_;
    m.rejected_by_unreachable   = pending_rejected_by_unreachable_;

    // Compute map stats locally from map_cache_ (the fused ROI grid in
    // dscovox mode, the log-odds cloud otherwise). The dscovox node no longer
    // computes these — scoring and stats both live in the planner now.
    m.total_observed_voxels = static_cast<int>(map_cache_->voxelCount());
    {
      float sum_eig = 0, sum_entropy = 0, sum_variance = 0;
      int frontier_count = 0, count = 0;
      auto acc = map_cache_->grid().createConstAccessor();
      static const Bonxai::CoordT offsets[6] = {
          {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
      map_cache_->grid().forEachCell(
          [&](const UnifiedVoxel& uv, const Bonxai::CoordT& c) {
            scovox::Voxel sv;
            sv.a_occ = uv.a_occ;
            sv.a_free = uv.a_free;
            sum_eig += scovox::expectedInformationGain(sv);
            sum_variance += scovox::variance(sv);
            sum_entropy += scoring::entropy(uv);
            count++;
            if (uv.p_occ < 0.5f) {
              for (const auto& off : offsets) {
                Bonxai::CoordT nb{c.x + off.x, c.y + off.y, c.z + off.z};
                if (!acc.value(nb)) { frontier_count++; break; }
              }
            }
          });
      m.frontier_voxels = frontier_count;
      if (count > 0) {
        m.mean_eig      = sum_eig / (float)count;
        m.mean_entropy   = sum_entropy / (float)count;
        m.mean_variance  = sum_variance / (float)count;
      }
    }

    logger_->logStep(m);
    RCLCPP_INFO(get_logger(),
        "Step %d logged: voxels=%d frontiers=%d dist=%.2f mean_eig=%.4f",
        step_, m.total_observed_voxels, m.frontier_voxels,
        m.distance_traveled, m.mean_eig);

    step_++;
    if (step_ >= max_steps_) {
      RCLCPP_INFO(get_logger(),
          "Exploration complete: %d steps, %.2fm traveled, %d final voxels.",
          step_, cumulative_distance_, m.total_observed_voxels);
      transitionTo(State::DONE);
    } else {
      transitionTo(State::PLAN);
    }
  }

  // ================================================================
  // Helpers
  // ================================================================

  // Look up base_frame -> map_frame via TF and cache the robot pose. Replaces
  // the odom subscription so the planner reads the same source-of-truth that
  // the rest of the nav stack uses, and so candidates/goals share the same
  // frame as the dscovox map.
  void updatePoseFromTF() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.05));
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TF lookup %s -> %s failed: %s",
          map_frame_.c_str(), base_frame_.c_str(), e.what());
      return;
    }
    latest_pos_ = Eigen::Vector3f(
        static_cast<float>(tf.transform.translation.x),
        static_cast<float>(tf.transform.translation.y),
        static_cast<float>(tf.transform.translation.z));
    latest_yaw_ = static_cast<float>(tf2::getYaw(tf.transform.rotation));
    have_pose_ = true;
  }

  void trackDistance() {
    if (!have_pose_) return;
    if (!first_pos_) {
      cumulative_distance_ += (latest_pos_ - prev_pos_).norm();
    }
    prev_pos_ = latest_pos_;
    first_pos_ = false;
  }

  // Yaw-only (Z-axis) quaternion message. Shared by goal + candidate viz
  // publishing so the yaw->quat conversion lives in one place.
  static geometry_msgs::msg::Quaternion yawToQuat(float yaw) {
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.z = std::sin(yaw * 0.5);
    return q;
  }

  void publishGoal(const CandidateViewpoint& vp) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = this->now();
    goal.header.frame_id = map_frame_;
    goal.pose.position.x = vp.position.x();
    goal.pose.position.y = vp.position.y();
    goal.pose.position.z = vp.position.z();
    goal.pose.orientation = yawToQuat(vp.yaw);
    goal_pub_->publish(goal);
  }

  void publishCandidateViz(const std::vector<CandidateViewpoint>& candidates) {
    if (viz_pub_->get_subscription_count() == 0) return;

    visualization_msgs::msg::MarkerArray ma;

    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear);

    int id = 0;
    // Compute max over finite scores only to avoid NaN/inf in color ratio.
    float max_score = 0.0f;
    for (const auto& c : candidates)
      if (std::isfinite(c.score))
        max_score = std::max(max_score, c.score);

    for (const auto& c : candidates) {
      if (!std::isfinite(c.position.x()) || !std::isfinite(c.position.y()))
        continue;

      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = this->now();
      m.ns = "candidates";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = c.position.x();
      m.pose.position.y = c.position.y();
      m.pose.position.z = c.position.z();
      m.pose.orientation = yawToQuat(c.yaw);
      m.scale.x = 0.4;
      m.scale.y = 0.1;
      m.scale.z = 0.1;

      // Clamp score for color: -inf/NaN → red, finite → green gradient
      float safe_score = std::isfinite(c.score) ? c.score : 0.0f;
      float ratio = (max_score > 0.0f) ? std::clamp(safe_score / max_score, 0.0f, 1.0f) : 0.0f;
      m.color.r = 1.0f - ratio;
      m.color.g = ratio;
      m.color.b = 0.0f;
      m.color.a = 0.7f;

      ma.markers.push_back(m);
    }

    // Highlight selected goal
    visualization_msgs::msg::Marker sel;
    sel.header.frame_id = map_frame_;
    sel.header.stamp = this->now();
    sel.ns = "selected";
    sel.id = 0;
    sel.type = visualization_msgs::msg::Marker::SPHERE;
    sel.action = visualization_msgs::msg::Marker::ADD;
    sel.pose.position.x = current_goal_.position.x();
    sel.pose.position.y = current_goal_.position.y();
    sel.pose.position.z = current_goal_.position.z() + 0.5;
    sel.scale.x = sel.scale.y = sel.scale.z = 0.5;
    sel.color.r = 0.0f;
    sel.color.g = 1.0f;
    sel.color.b = 1.0f;
    sel.color.a = 1.0f;
    ma.markers.push_back(sel);

    viz_pub_->publish(ma);
  }
};

} // namespace explo_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<explo_planner::ExplorationPlannerCompNode>();
  // MultiThreadedExecutor so the GetRegion service response can complete on
  // a separate thread while refreshRegion() blocks on the future. Without
  // this the single-threaded executor deadlocks.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
