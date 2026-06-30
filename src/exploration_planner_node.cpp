/// @file exploration_planner_node.cpp
/// @brief Definitions for ExplorationPlannerNode (see the header for the
/// interface). Every parameter, publisher, subscription and timer is wired up
/// here in the constructor — the header is declaration-only.

#include "explo_planner/exploration_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include "explo_planner/plan_map_query.hpp"
#include "explo_planner/planner_util.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <scovox/uncertainty.hpp>
#include <scovox/voxel.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // tf2::fromMsg used by tf2::getYaw

namespace explo_planner {

// ==================================================================
// Construction — parameters + ROS interface wiring
// ==================================================================

ExplorationPlannerNode::ExplorationPlannerNode()
    : Node("explo_planner"),
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

  // Vertical ROI band. In dscovox mode this is the z-slab the fused map is
  // clipped to on ingest, and it bounds the FOV raycast, frontier
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
  // Best-effort planning_map: when false the planner does not block on it at
  // startup and gracefully falls back to straight-line distances when none is
  // available (warning each tick). It is still used whenever it is published,
  // so an external occupancy map fed on planning_map_topic works too.
  require_planning_map_ = dp("require_planning_map", true);
  // Best-effort grace window: how long to wait for a late (latched) planning_map
  // after map + pose are ready before starting in straight-line fallback. Only
  // applies when require_planning_map is false.
  planning_map_wait_sec_ = dp("planning_map_wait_sec", 5.0);

  // Cache ROI bounds for the fused-map ingest clip (loadLatestMap()).
  roi_min_x_ = ccfg.roi_min_x;
  roi_max_x_ = ccfg.roi_max_x;
  roi_min_y_ = ccfg.roi_min_y;
  roi_max_y_ = ccfg.roi_max_y;

  if (map_type_ == "dscovox") {
    // The dscovox mapping node fuses every robot's voxels (multi-robot
    // consensus) and publishes the WHOLE fused map as a ScovoxMap topic. We
    // subscribe with the matching latched QoS (KeepLast(1) reliable +
    // transient_local) so the current map is delivered immediately on connect —
    // this replaces the old blocking GetRegion service call, and with it the
    // MultiThreadedExecutor + dedicated callback group that call required.
    // FOV raycasting, scoring and frontier extraction still run locally on the
    // ROI-clipped copy rebuilt into map_cache_ each PLAN tick.
    std::string dscovox_topic = dp("dscovox_topic", std::string(""));
    if (dscovox_topic.empty())
      dscovox_topic = "/" + robot_name_ + "/dscovox_node/scovox";
    scovox_map_sub_ = create_subscription<scovox_msgs::msg::ScovoxMap>(
        dscovox_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](scovox_msgs::msg::ScovoxMap::SharedPtr msg) {
          onScovoxMap(msg);
        });
    RCLCPP_INFO(get_logger(), "Subscribing to fused map (dscovox): %s",
        dscovox_topic.c_str());
  } else {
    logodds_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        logodds_pc_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          onLogOddsCloud(msg);
        });
    RCLCPP_INFO(get_logger(), "Subscribing to log-odds cloud: %s",
        logodds_pc_topic.c_str());
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

  viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/candidates", 10);

  // --- Intent pub/sub (always wired; payload only consumed when
  //     coord_->enabled() is true). Reliable + KeepLast(8) so a single missed
  //     broadcast doesn't drop a peer claim, but the queue stays bounded.
  //     Self-broadcasts are filtered by Coordination::onIntent.
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
      "EIG exploration planner ready: type=%s map=%s max_steps=%d "
      "frame=%s base=%s planning_map=%s goal=%s",
      planner_type_.c_str(), map_type_.c_str(),
      max_steps_, map_frame_.c_str(), base_frame_.c_str(),
      planning_map_topic.c_str(), goal_topic.c_str());
}

// ================================================================
// Map ingest — fused ScovoxMap topic (dscovox) / log-odds cloud (logodds)
// ================================================================

void ExplorationPlannerNode::onScovoxMap(
    const scovox_msgs::msg::ScovoxMap::SharedPtr& msg) {
  // One-time guard: the fused voxels are consumed in their raw frame (no TF
  // applied), so a dscovox map published in a frame other than the planner's
  // map_frame_ would be silently misplaced. Warn once on mismatch.
  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN_ONCE(get_logger(),
        "Fused map frame_id '%s' != planner map_frame '%s' — voxels are used "
        "without reframing and would be misplaced.",
        msg->header.frame_id.c_str(), map_frame_.c_str());
  }
  // Cache only. The grid is rebuilt from this (ROI-clipped) in loadLatestMap(),
  // mirroring the old per-cycle GetRegion fetch and avoiding a full grid rebuild
  // on every incoming message while the robot is NAVIGATE-ing.
  latest_scovox_map_ = msg;
}

bool ExplorationPlannerNode::loadLatestMap() {
  if (!latest_scovox_map_) return false;
  // Rebuild only when a new message has arrived since the last ingest. onScovoxMap
  // just swaps the cached pointer, so pointer identity == unchanged snapshot;
  // repeated PLAN / WAIT_FOR_MAP ticks on the same map reuse the existing grid
  // instead of reallocating it.
  if (latest_scovox_map_ != ingested_scovox_map_) {
    // The topic carries the whole fused map; clipping here keeps map_cache_
    // bounded to the ROI as the old per-region GetRegion service did, so frontier
    // extraction and the doLogStep map stats (both walk the whole grid) stay
    // bounded. The [roi_min_z_, roi_max_z_] band defines one consistent
    // observation volume that is also what FovEvaluator clips rays to.
    map_cache_->updateFromScovoxMap(
        *latest_scovox_map_,
        Eigen::Vector3f(roi_min_x_, roi_min_y_, roi_min_z_),
        Eigen::Vector3f(roi_max_x_, roi_max_y_, roi_max_z_));
    ingested_scovox_map_ = latest_scovox_map_;
    // have_map_ gates the WAIT_FOR_MAP -> PLAN transition; set it from the
    // post-clip count (matching the old service, which returned only in-ROI
    // voxels). Monotonic: once set it stays true.
    if (map_cache_->voxelCount() > 0) have_map_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
        "Loaded fused map into local map_cache_: %zu voxels in ROI (%zu in msg)",
        map_cache_->voxelCount(), latest_scovox_map_->voxels.size());
  }
  // Report whether there is actually in-ROI data to plan on. An empty ROI clip
  // (message received, but nothing inside the ROI band) returns false so doPlan
  // stays in PLAN and retries instead of scoring an all-prior, degenerate map.
  return map_cache_->voxelCount() > 0;
}

void ExplorationPlannerNode::onLogOddsCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  map_cache_->updateFromLogOddsCloud(*msg, map_resolution_);
  have_map_ = true;
}

// ================================================================
// Trajectory-level scoring (all planner types)
// ================================================================

bool ExplorationPlannerNode::scoreTrajectory(
    std::vector<CandidateViewpoint>& candidates) {
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
// State machine
// ================================================================

void ExplorationPlannerNode::tick() {
  updatePoseFromTF();
  trackDistance();

  switch (state_) {
    case State::WAIT_FOR_MAP:
      // dscovox mode: once a fused map has been received on the topic, ingest
      // it (ROI-clipped) so have_map_ flips from the in-ROI voxel count. In
      // logodds mode the cloud callback sets have_map_ directly. Neither path
      // blocks — the map arrives asynchronously via its subscription.
      if (map_type_ == "dscovox" && !have_map_) {
        loadLatestMap();  // sets have_map_ when in-ROI voxels are present
      }
      // planning_map handling:
      //  - require_planning_map_ == true  -> hard precondition; wait for it.
      //  - require_planning_map_ == false -> best-effort: once map + pose are
      //    in, wait up to planning_map_wait_sec_ for the (latched) planning_map
      //    to arrive before starting in straight-line fallback. The grace
      //    window catches a slightly-late planning_map before the first
      //    fallback PLAN tick; it is still adopted later if it arrives after.
      {
        bool start = false;
        if (have_map_ && have_pose_) {
          // Anchor the grace window at the moment map + pose first arrived (not
          // node start), so it measures the wait for the planning_map itself.
          if (!others_ready_seen_) {
            others_ready_time_ = this->now();
            others_ready_seen_ = true;
          }
          if (have_plan_map_) {
            start = true;
          } else if (!require_planning_map_ &&
                     (this->now() - others_ready_time_).seconds() >=
                         planning_map_wait_sec_) {
            start = true;
          }
        }
        if (start) {
          if (have_plan_map_) {
            RCLCPP_INFO(get_logger(),
                "Map, planning_map and pose received. Starting exploration.");
          } else {
            RCLCPP_WARN(get_logger(),
                "Starting WITHOUT a planning_map after a %.1fs grace wait "
                "(require_planning_map=false): no 2D obstacle/reachability "
                "filtering — falling back to straight-line distances. Will "
                "use the planning_map if it arrives.", planning_map_wait_sec_);
          }
          transitionTo(State::PLAN);
        } else {
          // Name the missing precondition so a stuck startup (wrong topic /
          // namespace / QoS, dead mapper, no TF) is diagnosable instead of a
          // silent indefinite wait. In best-effort mode this also covers the
          // grace window while waiting for a late planning_map.
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Waiting to start: map=%d pose=%d planning_map=%d (0 = not yet "
              "received).", have_map_, have_pose_, have_plan_map_);
        }
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

void ExplorationPlannerNode::transitionTo(State s) {
  state_ = s;
  state_enter_time_ = this->now();
}

// ================================================================
// PLAN state
// ================================================================

void ExplorationPlannerNode::doPlan() {
  auto plan_start = this->now();

  failed_goals_.prune(plan_start.seconds(), failed_goal_ttl_sec_);
  if (coord_) coord_->prune(plan_start);

  // dscovox mode: rebuild the local map_cache_ from the latest fused map
  // received on the topic (ROI-clipped) so frontier extraction + FOV scoring
  // below run on fresh consensus data. In logodds mode the cloud callback
  // keeps map_cache_ current, so there is nothing to load.
  if (map_type_ == "dscovox" && !loadLatestMap()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No fused map received yet on the dscovox topic; retrying next tick.");
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
  // fused grid pulled via the topic in dscovox mode, the log-odds cloud
  // otherwise).
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
  // Best-effort mode: no planning_map this tick -> fall back to straight-line
  // distances and skip the reachability filter (there is no grid to flood).
  // The candidate free/occupied filter below is likewise skipped when absent.
  if (planner_type_ != "random" && !latest_plan_map_) {
    skip_reachability = true;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No planning_map available — using straight-line distances and "
        "skipping reachability filtering this tick.");
  }
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
          // NaN-safe descending order. A bare `>` is undefined behaviour for
          // std::sort if any score is NaN (breaks strict-weak-ordering); sort
          // NaNs to the bottom so a degenerate score can never corrupt `order`.
          const float sa = candidates[a].score;
          const float sb = candidates[b].score;
          if (std::isnan(sa)) return false;
          if (std::isnan(sb)) return true;
          return sa > sb;
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
    //    is occupied/inflated (>= 50). Skipped entirely when no planning_map
    //    is available (best-effort mode) — there is nothing to check against.
    if (latest_plan_map_) {
      if (vp.is_frontier) {
        if (isCellOccupied(vp.position)) { ++rejected_map; continue; }
      } else {
        if (!isCellFree(vp.position)) { ++rejected_map; continue; }
      }
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
    if (failed_goals_.isNear(vp.position, failed_goal_radius_m_)) {
      ++rejected_blacklist;
      continue;
    }
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
  pending_plan_ms_ = plan_ms;  // drained into StepMetrics by doLogStep

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
    uint8_t ptype = plannerTypeId(planner_type_);
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
  nav_budget_sec_ = navBudgetSec(dist, nav_speed_est_mps_, nav_safety_factor_,
                                 nav_min_timeout_sec_, nav_max_timeout_sec_);
  // Both anchors mark NAVIGATE entry; reuse the timestamp transitionTo()
  // just stamped rather than re-reading the clock.
  progress_check_time_ = state_enter_time_;
  progress_check_dist_ = cumulative_distance_;
  RCLCPP_DEBUG(get_logger(),
      "Step %d: nav budget %.1fs for %.2f m goal", step_,
      nav_budget_sec_, dist);
}

// Thin wrappers over the pure plan_map_query helpers: supply the latched
// planning_map + ROI and define the no-map behaviour. The grid math itself is
// tested in test_plan_map_query.

double ExplorationPlannerNode::unknownFractionInRoi() const {
  if (!latest_plan_map_) return -1.0;
  return explo_planner::unknownFractionInRoi(
      *latest_plan_map_, {roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_});
}

bool ExplorationPlannerNode::isCellFree(const Eigen::Vector3f& pos) const {
  // No map: not free (matches the old kCellNoData -> v >= 0 failure).
  return latest_plan_map_ &&
         explo_planner::isCellFree(*latest_plan_map_, pos);
}

bool ExplorationPlannerNode::isCellOccupied(const Eigen::Vector3f& pos) const {
  // No map: conservatively occupied (matches the old kCellNoData default).
  return !latest_plan_map_ ||
         explo_planner::isCellOccupied(*latest_plan_map_, pos);
}

// ================================================================
// NAVIGATE state
// ================================================================

void ExplorationPlannerNode::doNavigate() {
  // Abort early if the map has updated and the goal cell is now inside
  // an obstacle (or its inflation zone). This avoids wasting time
  // navigating toward goals that were valid at selection but became
  // occupied as the map grew. Skipped when no planning_map is available
  // (best-effort mode) — isCellOccupied treats "no map" as occupied, which
  // would otherwise abort every goal immediately.
  if (latest_plan_map_ && isCellOccupied(current_goal_.position)) {
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
void ExplorationPlannerNode::failGoal(const char* reason, double elapsed) {
  failed_goals_.add(current_goal_.position, this->now().seconds());
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
void ExplorationPlannerNode::heartbeatTick() {
  if (!coord_enabled_) return;
  if (!have_active_intent_) return;
  if (state_ != State::NAVIGATE && state_ != State::INTEGRATE) return;
  if (!intent_pub_) return;
  current_intent_msg_.header.stamp = this->now();
  intent_pub_->publish(current_intent_msg_);
}

// ================================================================
// INTEGRATE state
// ================================================================

void ExplorationPlannerNode::doIntegrate() {
  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed >= integrate_wait_) {
    transitionTo(State::LOG_STEP);
  }
}

// ================================================================
// LOG_STEP state
// ================================================================

void ExplorationPlannerNode::doLogStep() {
  StepMetrics m;
  m.step = step_;
  m.sim_time_sec = this->now().seconds();
  m.distance_traveled = cumulative_distance_;
  m.selected_score = current_goal_.score;
  m.plan_time_ms = pending_plan_ms_;

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
void ExplorationPlannerNode::updatePoseFromTF() {
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

void ExplorationPlannerNode::trackDistance() {
  if (!have_pose_) return;
  if (!first_pos_) {
    cumulative_distance_ += (latest_pos_ - prev_pos_).norm();
  }
  prev_pos_ = latest_pos_;
  first_pos_ = false;
}

// Yaw-only (Z-axis) quaternion message. Shared by goal + candidate viz
// publishing so the yaw->quat conversion lives in one place.
geometry_msgs::msg::Quaternion ExplorationPlannerNode::yawToQuat(float yaw) {
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.z = std::sin(yaw * 0.5);
  return q;
}

void ExplorationPlannerNode::publishGoal(const CandidateViewpoint& vp) {
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = this->now();
  goal.header.frame_id = map_frame_;
  goal.pose.position.x = vp.position.x();
  goal.pose.position.y = vp.position.y();
  goal.pose.position.z = vp.position.z();
  goal.pose.orientation = yawToQuat(vp.yaw);
  goal_pub_->publish(goal);
}

void ExplorationPlannerNode::publishCandidateViz(
    const std::vector<CandidateViewpoint>& candidates) {
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

} // namespace explo_planner
