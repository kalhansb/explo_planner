/// @file explo_planner_node.cpp
/// @brief Standalone SCovox Beta EIG exploration planner node — self-contained.
///
/// This is the EIG-only planner that the exploration/exploitation system is
/// built on top of. It is a deliberate DUPLICATE of ExplorationPlannerNode
/// (exploration_planner_node.hpp/.cpp): that node is the multi-planner
/// comparison harness (eig/entropy/frontier/random/ssmi) used for experiments;
/// this one is hard-wired to the SCovox Beta expected-information-gain scorer
/// and owns its own copy of the state machine so it can diverge as the
/// exploration/exploitation behaviour grows without disturbing the comparison
/// node. The reusable pieces (scoring, candidate generation, FOV evaluation,
/// cost grid, coordination, map cache, metrics) are still shared via
/// explo_planner_lib.
///
/// Map ingest is topic-based (not the old GetRegion service): the planner
/// SUBSCRIBES to the fused ScovoxMap topic (latched QoS) and rebuilds a local,
/// ROI-clipped MapCache from it each PLAN tick. All parameters, publishers,
/// subscriptions and timers are declared/wired in this file's constructor.
///
/// State machine: WAIT_FOR_MAP -> PLAN -> NAVIGATE -> INTEGRATE -> LOG_STEP -> DONE.
///
/// Exploitation overlay: when a tree target arrives on the targets topic, the
/// planner switches Phase EXPLORE -> EXPLOIT and runs a vantage sub-loop
/// (EXPLOIT_PLAN -> NAVIGATE -> EXPLOIT_DWELL -> LOG_STEP -> EXPLOIT_PLAN) that
/// circles the trunk at occlusion-free vantage points, dwelling at each so the
/// rosbag captures overlapping views (fusion is offline). When the target's
/// vantages are covered and the queue empties, it reverts to EXPLORE. With no
/// targets (or exploitation_enabled=false) behaviour is pure exploration.

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <scovox_msgs/msg/scovox_map.hpp>
#include <explo_planner_msgs/msg/robot_intent.hpp>
#include <explo_planner_msgs/msg/tree_target.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // tf2::fromMsg used by tf2::getYaw

#include "explo_planner/map_cache.hpp"
#include "explo_planner/candidate_generator.hpp"
#include "explo_planner/fov_evaluator.hpp"
#include "explo_planner/scoring.hpp"
#include "explo_planner/metrics_logger.hpp"
#include "explo_planner/cost_grid.hpp"
#include "explo_planner/coordination.hpp"
#include "explo_planner/plan_map_query.hpp"
#include "explo_planner/planner_util.hpp"
#include "explo_planner/failed_goal_blacklist.hpp"
#include "explo_planner/proximity_guard.hpp"
#include "explo_planner/target_queue.hpp"
#include "explo_planner/vantage_planner.hpp"

namespace explo_planner {

enum class State {
  WAIT_FOR_MAP,
  PLAN,
  NAVIGATE,
  INTEGRATE,
  LOG_STEP,
  DONE,
  // Exploitation sub-states. EXPLOIT_PLAN selects the next vantage around the
  // active target; reaching it routes to EXPLOIT_DWELL (NAVIGATE is shared with
  // exploration and branches on phase_).
  EXPLOIT_PLAN,
  EXPLOIT_DWELL,
  // Rendezvous sub-states (multi-robot). On exhausting its exploration goals a
  // robot drives back to its last-connected anchor (RETURN_NAV) and holds there
  // (RETURN_SYNC) until the whole team is back in comms, then re-plans against
  // the merged map. Gated by rendezvous_enabled_; see the rendezvous_* params.
  RETURN_NAV,
  RETURN_SYNC,
  // Coordinated proximity stop (multi-robot). A DRIVING robot that has lost
  // right-of-way to a nearby moving teammate cancels its nav goal and parks
  // here until the peer clears off or parks, then resumes the same goal.
  // Entered only from NAVIGATE / RETURN_NAV; see checkProximityHold().
  PROXIMITY_HOLD
};

// Top-level behaviour mode. NAVIGATE / INTEGRATE / LOG_STEP are shared between
// modes and branch on this to route correctly.
enum class Phase { EXPLORE, EXPLOIT };

// ==================================================================
// ExploPlannerNode — EIG-only NBV exploration planner.
//
// Self-contained on purpose (see file header): the class is declared and
// defined here, with main() at the bottom. The scoring function is fixed to
// SCovox Beta EIG; there is no planner_type parameter.
// ==================================================================
class ExploPlannerNode : public rclcpp::Node {
public:
  ExploPlannerNode();

private:
  // ----------------------------------------------------------------
  // Map ingest
  // ----------------------------------------------------------------
  // Cache the latest fused map received on the dscovox topic. The grid is
  // rebuilt from it (ROI-clipped) in loadLatestMap() at the start of each PLAN
  // tick, so a full rebuild doesn't run on every incoming message.
  void onScovoxMap(const scovox_msgs::msg::ScovoxMap::SharedPtr& msg);
  // Rebuild map_cache_ from the latest cached ScovoxMap, clipped to the ROI.
  // Returns false if no map has been received yet.
  bool loadLatestMap();

  // Trajectory-level scoring (path-integrated EIG; param-gated ablation).
  bool scoreTrajectory(std::vector<CandidateViewpoint>& candidates);

  // ----------------------------------------------------------------
  // State machine
  // ----------------------------------------------------------------
  void tick();
  void transitionTo(State s);
  void doPlan();
  void doNavigate();
  void doIntegrate();
  void doLogStep();

  // Rendezvous (multi-robot reconnection). finishOrRendezvous decides, at
  // exploration exhaustion, between DONE and returning to the anchor;
  // startReturnToAnchor arms the drive back; doReturnNav drives there;
  // doReturnSync holds at the anchor until the whole team is in comms.
  // `reason` is the termination cause, for logging only — EVERY termination
  // path must route through here, not just coverage saturation.
  void finishOrRendezvous(const char* reason);
  void startReturnToAnchor(const char* reason);
  void doReturnNav();
  void doReturnSync();

  // Exploitation. onTreeTarget ingests targets off the shared topic;
  // doExploitPlan generates + validates + selects the next vantage;
  // doExploitDwell holds at it; finishActiveTarget closes a target and routes
  // back to the queue or to exploration. inRoi is the shared ROI box test.
  void onTreeTarget(const explo_planner_msgs::msg::TreeTarget::SharedPtr& msg);
  // Team quota: fold a peer's exploit intent (clear-LoS dwelled vantage mask
  // on its target) into the local target queue the moment it arrives, so
  // credit is never lost to claim TTL while this robot is mid-hop/dwell.
  void onPeerExploitIntent(const explo_planner_msgs::msg::RobotIntent& msg);
  void doExploitPlan();
  void doExploitDwell();
  void finishActiveTarget(bool success);
  bool inRoi(const Eigen::Vector3f& pos) const;
  // Nearest reachable, free, in-ROI point on the line from the robot toward
  // `center` (marched from just outside the trunk outward). Lets the planner
  // drive *toward* a target whose vantage ring isn't reachable yet, mapping en
  // route, instead of abandoning it. Returns false if no such point meaningfully
  // closer than the robot exists (nothing to approach). Requires a flooded
  // cost_grid_.
  bool computeApproachGoal(const Eigen::Vector3f& center, float radius,
                           const Eigen::Vector3f& robot_pos,
                           Eigen::Vector3f& out) const;
  // Publish current_goal_ as the active EXPLOIT goal (+ MinPos intent) and arm
  // the NAVIGATE smart-timeout for the hop from robot_pos. Shared by the vantage
  // and approach paths of doExploitPlan so they stay in lock-step.
  void startExploitNavigate(const Eigen::Vector3f& robot_pos);

  // PLAN helpers. The cell classification + ROI unknown-fraction math is pure
  // (see plan_map_query.hpp); these members are thin wrappers that supply the
  // latched planning_map and ROI and handle the no-map case.
  double unknownFractionInRoi() const;
  // Coverage unknown fraction for termination, dispatched per
  // done_coverage_source_. Sets *source to the label of the source actually
  // used ("planning_map" / "scovox"); returns -1 when it cannot measure.
  double coverageUnknownFraction(const char** source) const;
  bool isCellFree(const Eigen::Vector3f& pos) const;
  bool isCellOccupied(const Eigen::Vector3f& pos) const;

  // NAVIGATE helpers
  void failGoal(const char* reason, double elapsed);
  void heartbeatTick();

  // Coordinated proximity stop. checkProximityHold runs each tick while a nav
  // goal is in flight (NAVIGATE / RETURN_NAV) and enters PROXIMITY_HOLD when
  // the guard says a higher-priority teammate is moving nearby; returns true
  // when it transitioned. doProximityHold holds until the guard releases,
  // then resumes the interrupted drive on the same goal.
  bool checkProximityHold();
  void enterProximityHold(const ProximityGuard::Decision& d);
  void doProximityHold();
  /// Latched operator-facing hold state ("hold peer=... dist=..." / "clear
  /// reason=..."), so a field laptop can see WHO is yielding and why without
  /// grepping planner logs.
  void publishProxState(const std::string& state);

  // Pose / publishing helpers
  void updatePoseFromTF();
  void trackDistance();
  static geometry_msgs::msg::Quaternion yawToQuat(float yaw);
  void publishGoal(const CandidateViewpoint& vp);
  void republishGoal(const CandidateViewpoint& vp);
  void publishCandidateViz(const std::vector<CandidateViewpoint>& candidates);

  /// Standing / sightline height for an exploitation point at (x, y). Flat
  /// mode returns the fixed absolute vantage height. Terrain mode snaps to the
  /// local ground + candidate clearance, the same way exploration candidates
  /// are placed — see the definition for why the absolute height is unusable
  /// once the map z-band is robot-relative.
  float exploitZAt(float x, float y) const;

  // --- Parameters ---
  std::string robot_name_;
  std::string output_csv_;
  std::string map_frame_;
  std::string base_frame_;
  int    max_steps_;
  double map_resolution_;
  double goal_xy_tol_;
  double goal_yaw_tol_;
  // Deadline for the post-arrival in-place rotation, measured from the first
  // tick the robot is inside goal_xy_tol_ — NOT shared with nav_budget_sec_,
  // which budgets the drive (see doNavigate).
  double goal_rotate_timeout_sec_;
  // Minimum interval between re-sends of an *unchanged* goal pose. A changed
  // pose always publishes immediately; this only throttles the keep-alive.
  // Nav2's bt_navigator turns every incoming goal_pose into a fresh
  // NavigateToPose goal, which preempts the running one and rewrites the BT
  // blackboard "goal" — so GoalUpdated (the first child of the default
  // navigate_to_pose_w_replanning_and_recovery RecoveryFallback) returns
  // SUCCESS, the recovery RoundRobin is halted before Spin/Wait/BackUp can
  // finish, and RecoveryNode still counts it as a completed recovery. At the
  // old 10 Hz re-send rate that burned all 6 retries in well under a second,
  // so any transient planning/control failure became an immediate nav2 ABORT
  // instead of a recovery. Set 0 to publish only on change (best once nav2
  // bringup is known reliable — it lets every recovery run to completion).
  double goal_republish_sec_;
  double integrate_wait_;
  double nav_speed_est_mps_;
  double nav_safety_factor_;
  double nav_min_timeout_sec_;
  double nav_max_timeout_sec_;
  double progress_window_sec_;
  double progress_min_distance_m_;
  // Reject single-tick pose jumps larger than this (m) from cumulative_distance_:
  // localization relocalization (NDT/EKF) corrections teleport the
  // map->base_link TF and would otherwise be counted as travel. 0 = disabled.
  float  max_pose_jump_m_ = 1.0f;
  double failed_goal_radius_m_;
  double failed_goal_ttl_sec_;
  double done_unknown_fraction_;
  int    done_min_consecutive_steps_;
  // Where the coverage-termination unknown fraction is measured:
  //  - "planning_map": 2D unknown cells in the latched planning_map (legacy).
  //    Returns -1 (check INACTIVE) when no planning_map is published.
  //  - "scovox": 2.5D column coverage of the ROI footprint on the fused 3D
  //    map in map_cache_ (MapCache::unknownColumnFraction) — works with no
  //    planning_map at all.
  //  - "auto" (default): planning_map when one has been received, else scovox.
  std::string done_coverage_source_{"auto"};
  // What DONE does: "shutdown" (legacy) stops the node; "idle" keeps it
  // spinning so targets released after coverage-done still pull the planner
  // into the exploit sub-loop (field flow: targets are released at the
  // coverage-done cue, which would otherwise race the shutdown).
  std::string done_action_{"shutdown"};
  // Master switch for the 2D planning_map. When true it is subscribed, treated
  // as a hard startup precondition, and drives the candidate free/occupied
  // filter + cost-grid reachability in BOTH exploration and exploitation. When
  // false (default) the planner never subscribes to it and never consults it in
  // either phase: costs fall back to straight-line distances, there is no 2D
  // obstacle / reachability filtering, and obstacle avoidance is delegated to
  // the downstream navigator. Off by default so the planner runs on the fused
  // 3D map alone (the dscovox merger publishes no planning_map).
  bool   use_planning_map_{false};
  bool   shutdown_requested_{false};

  // Utility / coordination params (cached so doPlan() doesn't re-query).
  double cost_grid_radius_cap_m_ = 0.0;
  bool   trajectory_scoring_   = false;
  double trajectory_sample_spacing_m_ = 1.5;
  bool   coord_enabled_        = false;
  double coord_claim_radius_m_ = 0.0;
  double coord_claim_ttl_sec_  = 5.0;
  double coord_heartbeat_hz_   = 1.0;
  std::string coord_intent_topic_;
  // MinPos match radius for EXPLOIT vantage claims. Vantages on one trunk sit
  // only ~(radius + standoff) apart, so the exploration-scale claim disc
  // (fov_max_range) would swallow the whole ring and veto the tree outright
  // instead of assigning different angles. 0 = auto = vantage_visited_tol_m.
  double coord_vantage_claim_radius_m_ = 0.0;

  // --- Rendezvous (multi-robot reconnection) params ---
  // When true (the default), a robot that exhausts its exploration goals does
  // NOT stop while a teammate is still out of comms: it drives back to its
  // last-connected anchor and waits until the whole team is back in range, then
  // re-plans against the merged map. Requires coordination_enabled (peers are
  // what the barrier waits on) and a positive expected-peer count; stays inert
  // otherwise (single-robot runs are unaffected).
  bool   rendezvous_enabled_       = true;
  // How many teammates to wait for at the barrier (team size minus self). The
  // multi_robot launch sets this from the robot list; override in YAML on
  // hardware. <= 0 disables rendezvous.
  int    rendezvous_expected_peers_ = 0;
  // Barrier give-up (seconds). 0 = wait forever (the requested default: STAY
  // until all connected). A positive value is an escape hatch for field trials
  // so a robot whose teammate died doesn't hold the anchor indefinitely.
  double rendezvous_max_wait_sec_  = 0.0;

  // --- Proximity stop (coordinated yield) params ---
  // Thresholds/staleness live in the guard's Config; these are the node-side
  // knobs. max_hold <= 0 = unbounded (the guard's parked-peer and stale-data
  // releases already break every mutual-wait cycle; a positive value is a
  // field escape hatch that resumes the drive even with the peer still near).
  bool   proximity_stop_enabled_ = true;
  double proximity_max_hold_sec_ = 0.0;
  std::string proximity_nav_cancel_action_;
  // ROI bounds — used to constrain candidate generation, the FOV raycast and
  // the clip applied when ingesting the fused map topic into map_cache_.
  float roi_min_x_ = -15.0f;
  float roi_max_x_ =  15.0f;
  float roi_min_y_ = -15.0f;
  float roi_max_y_ =  15.0f;
  float roi_min_z_ =  -0.5f;
  float roi_max_z_ =   2.0f;
  // Terrain-relative (3D) mode. When true:
  //  - roi_min_z_/roi_max_z_ are interpreted RELATIVE to the robot's current
  //    z: the fused-map ingest clip, frontier extraction and the FOV ray
  //    z-clip all use [robot_z + roi_min_z_, robot_z + roi_max_z_], re-banded
  //    as the robot climbs/descends (see loadLatestMap);
  //  - candidates are snapped to local ground + candidate_z_clearance
  //    (CandidateGenerator terrain mode), so published goals carry a real 3D
  //    z. Nav2 consumes only (x, y, yaw) from the goal — the z rides along
  //    for 3D consumers/RViz, or is zeroed when flatten_goal_z_ is set.
  // When false, all z handling is the legacy absolute flat-world behaviour.
  bool  terrain_relative_z_ = false;
  // Effective (absolute) z band currently ingested into map_cache_. Equal to
  // roi_min_z_/roi_max_z_ in flat mode; robot-centred in terrain mode.
  float eff_roi_min_z_ = 0.0f;
  float eff_roi_max_z_ = 0.0f;
  // Robot z the current map_cache_ ingest band was centred on (terrain mode).
  // NaN until the first ingest; used to re-band after vertical travel.
  float ingest_ref_z_ = std::numeric_limits<float>::quiet_NaN();
  // Zero the z of the published goal PoseStamped (strict-2D nav consumers).
  // Markers/logs keep the true 3D z either way.
  bool  flatten_goal_z_ = false;
  // Frontier-centroid clustering bin size (m): findFrontierCentroids bins
  // frontier cells into cubes of this edge length and emits one centroid per
  // bin. Larger -> fewer, coarser long-range targets. The sibling roi_*_z args
  // to findFrontierCentroids are already params; this was the lone hardcoded one.
  float frontier_cluster_radius_m_ = 5.0f;

  // --- Exploitation params ---
  // When false the exploitation overlay is inert (no target subscription is
  // consulted) and behaviour is bit-for-bit pure exploration.
  bool   exploitation_enabled_   = true;
  double target_dedup_radius_m_  = 0.0;   // 0 = dedup by id only (no merge)
  int    min_vantages_required_  = 2;     // clear-LoS dwells for "success"
  double exploit_dwell_sec_      = 8.0;   // hold time at each vantage (s)
  // Proximity tolerance for "this vantage was already dwelled". Well below the
  // chord between adjacent vantages so it never aliases two distinct vantages.
  double vantage_visited_tol_m_  = 0.75;
  // Per-target give-up budget (s): wall-clock on one trunk since the last
  // progress event (activation, approach arrival, completed dwell). When it
  // expires the target closes PARTIAL and exploration resumes. <=0 disables.
  // Must stay ABOVE nav_max_timeout_sec: failed hops keep charging it (that is
  // the termination guarantee for unreachable rings), so a smaller value gives
  // up after the FIRST failed hop with no retry. 300 = one worst-case failed
  // hop (180) + 120 to re-select and reach another angle.
  double exploit_target_timeout_sec_ = 300.0;

  // --- Components ---
  std::unique_ptr<MapCache> map_cache_;
  std::unique_ptr<CandidateGenerator> candidate_gen_;
  std::unique_ptr<FovEvaluator> fov_eval_;
  ScoreFn score_fn_;
  std::unique_ptr<MetricsLogger> logger_;
  std::unique_ptr<CostGrid> cost_grid_;
  std::unique_ptr<Coordination> coord_;
  std::unique_ptr<ProximityGuard> prox_guard_;
  std::unique_ptr<VantagePlanner> vantage_planner_;
  TargetQueue target_queue_;

  // --- State ---
  State state_ = State::WAIT_FOR_MAP;
  Phase phase_ = Phase::EXPLORE;
  // Vantage being navigated to / dwelled at (valid only in EXPLOIT phase).
  int   current_vantage_index_     = -1;
  bool  current_vantage_los_clear_ = false;
  // True when the current EXPLOIT goal is an APPROACH waypoint (driving toward a
  // target whose vantage ring isn't reachable yet) rather than a vantage to
  // dwell at. On reaching an approach goal the planner re-plans instead of
  // dwelling.
  bool  current_is_approach_       = false;
  // Per-target give-up timer: the active target id we started timing and when
  // (sim seconds). Latched when a target becomes active; re-armed on every
  // progress event (approach-waypoint arrival, completed dwell); refunded for
  // proximity-hold time; cleared on rendezvous stand-down so a re-activated
  // target starts fresh. Failed navigation hops deliberately keep charging it
  // — that is what makes an unreachable ring terminate PARTIAL.
  uint32_t exploit_target_started_id_  = 0;
  double   exploit_target_started_sec_ = 0.0;
  bool     exploit_target_timing_      = false;
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

  // Recently-failed goals (TTL + radius blacklist).
  FailedGoalBlacklist failed_goals_;

  // Per-navigate-cycle state for the smart timeout.
  double nav_budget_sec_ = 0.0;
  rclcpp::Time progress_check_time_;
  float progress_check_dist_ = 0.0f;

  // Post-arrival rotation deadline. Armed the first tick the robot is inside
  // goal_xy_tol_ but still outside goal_yaw_tol_; disarmed by transitionTo()
  // on every NAVIGATE entry.
  bool rotate_deadline_armed_ = false;
  rclcpp::Time rotate_start_time_;

  // Cache key for the UNBOUNDED exploitation flood of cost_grid_ (see
  // doExploitPlan). That flood is O(grid) and EXPLOIT_PLAN re-enters at the
  // full tick rate whenever no vantage is selectable, so it is rebuilt only
  // when its inputs change. exploit_flood_map_ is an identity handle for the
  // latched map object — compared, never dereferenced. Invalidated by doPlan's
  // radius-bounded flood, which overwrites the same grid.
  bool exploit_flood_valid_ = false;
  const void* exploit_flood_map_ = nullptr;
  Eigen::Vector3f exploit_flood_pos_ = Eigen::Vector3f::Zero();
  size_t exploit_flood_reached_ = 0;

  // Coverage termination streak.
  int coverage_done_streak_ = 0;

  // Rendezvous anchor: the robot pose the last time it heard a teammate. That
  // pose sits inside the comms bubble, so it is the cheapest point to return to
  // for reconnection. Recorded on every peer intent (see the intent callback);
  // have_anchor_ stays false until the first peer is heard (single-robot runs
  // never rendezvous).
  Eigen::Vector3f last_connected_anchor_ = Eigen::Vector3f::Zero();
  bool  have_anchor_ = false;

  // Proximity-hold bookkeeping: the driving state to resume into (NAVIGATE or
  // RETURN_NAV — current_goal_ is left untouched across the hold), cumulative
  // hold count / held seconds (CSV columns, so post-hoc analysis can correlate
  // holds with the trajectory), and the drive seconds already consumed on the
  // current goal when the hold began — the resume backdates state_enter_time_
  // by it so the nav budget CONTINUES instead of restarting, keeping one
  // goal's total drive time bounded across repeated holds.
  State  prox_resume_state_    = State::NAVIGATE;
  int    prox_hold_count_      = 0;
  double prox_hold_total_sec_  = 0.0;
  double prox_nav_elapsed_sec_ = 0.0;
  // Messages seen across ALL peer pose subs — while 0 with subs configured, a
  // throttled warning says so (a typo'd topic is otherwise indistinguishable
  // from "peer far away").
  size_t peer_pose_msg_count_  = 0;

  // Per-tick utility / coord diagnostics (filled by doPlan, drained by
  // doLogStep into the StepMetrics row).
  float pending_mean_info_gain_       = 0.0f;
  float pending_mean_path_cost_       = 0.0f;
  float pending_selected_info_gain_   = 0.0f;
  float pending_selected_path_cost_   = 0.0f;
  float pending_plan_ms_              = 0.0f;
  int   pending_rejected_by_minpos_      = 0;
  int   pending_rejected_by_unreachable_ = 0;

  // Per-exploit-step diagnostics (filled by doExploitPlan/doExploitDwell,
  // drained by doLogStep when phase_ == EXPLOIT).
  int   pending_exploit_target_id_    = -1;
  int   pending_exploit_vantage_index_ = -1;
  int   pending_exploit_n_valid_      = 0;
  int   pending_exploit_los_clear_    = 0;
  float pending_exploit_dwell_sec_    = 0.0f;

  // Cached active intent so the heartbeat timer can re-publish without
  // touching planning state.
  explo_planner_msgs::msg::RobotIntent current_intent_msg_;
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
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr plan_map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  // Last goal actually put on the wire, for republishGoal()'s change
  // detection + keep-alive throttle. Not valid until have_last_goal_pub_.
  Eigen::Vector3f last_goal_pub_pos_ = Eigen::Vector3f::Zero();
  float           last_goal_pub_yaw_ = 0.0f;
  rclcpp::Time    last_goal_pub_time_;
  bool            have_last_goal_pub_ = false;
  // Subscriber-count edge on goal_pub_. A goal published while the navigator
  // is absent is silently dropped (there is no action feedback to notice it
  // with), so re-send once as soon as one appears — this covers a navigator
  // that is brought up, restarted, or re-configured after the planner.
  bool            goal_had_subscriber_ = false;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<explo_planner_msgs::msg::RobotIntent>::SharedPtr intent_pub_;
  rclcpp::Subscription<explo_planner_msgs::msg::RobotIntent>::SharedPtr intent_sub_;
  // Shared tree-target topic. The time-based scheduler publishes here today; a
  // detector can publish the same message later with no planner change.
  rclcpp::Subscription<explo_planner_msgs::msg::TreeTarget>::SharedPtr target_sub_;
  // Proximity guard inputs + actuation. The pose subs are the peers'
  // localiser outputs (map frame, ~10 Hz) — much fresher than the 1 Hz intent
  // heartbeat that also feeds the guard. The action client exists ONLY to
  // cancel the in-flight NavigateToPose on hold entry: ceasing to publish
  // goal_pose does not stop nav2, the last accepted goal runs to completion.
  std::vector<rclcpp::Subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> peer_pose_subs_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      nav_cancel_client_;
  // Latched (transient_local) hold-state string for the operator: see
  // publishProxState().
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr prox_state_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

// ==================================================================
// Construction — parameters + ROS interface wiring
// ==================================================================

ExploPlannerNode::ExploPlannerNode()
    : Node("explo_planner"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  // --- Parameters ---
  auto dp = [&](auto n, auto d) {
    return this->declare_parameter<decltype(d)>(n, d);
  };

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
  // Separate deadline for the in-place rotation after the XY goal is reached.
  // Sized for a worst-case ~180 deg turn at a slow yaw rate, independent of
  // how far the hop was.
  goal_rotate_timeout_sec_ = dp("goal_rotate_timeout_sec", 15.0);
  goal_republish_sec_ = dp("goal_republish_sec", 5.0);
  // The arrival gate must be strictly LOOSER than the navigator's own goal
  // checker, or the navigator declares success and stops just outside the
  // planner's tolerance, the planner never sees arrival, and failGoal()
  // blacklists a goal the robot is standing on. nav2's shipped
  // general_goal_checker defaults are xy 0.25 / yaw 0.25.
  if (goal_yaw_tol_ < 0.3) {
    RCLCPP_WARN(get_logger(),
        "goal_yaw_tolerance=%.2f rad is at or below nav2's default "
        "yaw_goal_tolerance (0.25) — the navigator can stop inside its own "
        "tolerance but outside this gate, which fails the goal on the "
        "rotate deadline and blacklists it. Use >= 0.4, or match it to this "
        "robot's configured goal checker.", goal_yaw_tol_);
  }
  if (goal_xy_tol_ < 0.3) {
    RCLCPP_WARN(get_logger(),
        "goal_xy_tolerance=%.2f m is at or below nav2's default "
        "xy_goal_tolerance (0.25) — see the goal_yaw_tolerance warning.",
        goal_xy_tol_);
  }
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
  // Teleport guard for cumulative distance (see member doc). At the 10 Hz tick
  // this cannot reject real motion; it filters localization discontinuities so
  // they don't inflate distance_traveled or spoof the no-progress watchdog.
  max_pose_jump_m_ = static_cast<float>(dp("max_pose_jump_m", 1.0));

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

  // Coverage-based termination. Each PLAN tick we measure the unknown
  // fraction of the ROI (source per done_coverage_source below). When it
  // stays below `done_unknown_fraction` for `done_min_consecutive_steps`
  // planning cycles in a row we declare the map saturated and transition to
  // DONE. EIG scores don't fall sharply as the map saturates (the FOV raycast
  // always finds *some* unobserved voxels at the cone edge), so unknown
  // fraction is the reliable signal here. Set done_unknown_fraction <= 0 to
  // disable.
  done_unknown_fraction_ =
      dp("done_unknown_fraction", 0.05);
  done_min_consecutive_steps_ =
      dp("done_min_consecutive_steps", 3);
  // Measurement source: "planning_map" (legacy 2D; INACTIVE when none is
  // published), "scovox" (2.5D column coverage of the fused 3D map — works
  // without a planning_map), or "auto" (planning_map if present, else
  // scovox). NB the two sources measure different things — the 2D grid
  // counts nav-grid cells, the column measure counts ROI-footprint columns
  // with >= 1 observed voxel in the z band — so re-calibrate
  // done_unknown_fraction when switching.
  done_coverage_source_ = dp("done_coverage_source", std::string("auto"));
  if (done_coverage_source_ != "auto" &&
      done_coverage_source_ != "planning_map" &&
      done_coverage_source_ != "scovox") {
    RCLCPP_WARN(get_logger(),
        "Unknown done_coverage_source '%s' — falling back to 'auto'.",
        done_coverage_source_.c_str());
    done_coverage_source_ = "auto";
  }
  // DONE behaviour: "shutdown" (legacy) or "idle" (stay up; late targets are
  // still exploited — required when targets are released at the
  // coverage-done cue, which would otherwise race the shutdown).
  done_action_ = dp("done_action", std::string("shutdown"));
  if (done_action_ != "shutdown" && done_action_ != "idle") {
    RCLCPP_WARN(get_logger(),
        "Unknown done_action '%s' — falling back to 'shutdown'.",
        done_action_.c_str());
    done_action_ = "shutdown";
  }

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

  // Terrain-relative (3D) mode — see the member doc. Off by default: flat
  // mode is bit-for-bit the legacy behaviour.
  terrain_relative_z_ = dp("terrain_relative_z", false);
  ccfg.terrain_relative    = terrain_relative_z_;
  ccfg.z_clearance         = static_cast<float>(dp("candidate_z_clearance", 0.5));
  ccfg.ground_search_below = static_cast<float>(dp("ground_search_below_m", 4.0));
  ccfg.ground_search_above = static_cast<float>(dp("ground_search_above_m", 1.0));
  ccfg.ground_stack_max_m  = static_cast<float>(dp("ground_stack_max_m", 0.6));
  flatten_goal_z_          = dp("flatten_goal_z", false);
  // Until the first ingest the effective band equals the configured one
  // (flat mode keeps it that way permanently).
  eff_roi_min_z_ = roi_min_z_;
  eff_roi_max_z_ = roi_max_z_;
  // Candidate z clamp: same band the map is ingested over, so a terrain-snapped
  // candidate can't sit in space the map holds nothing for. doPlan re-points it
  // at the effective band each tick in terrain mode, exactly as it does the FOV
  // ray clip.
  ccfg.roi_min_z  = eff_roi_min_z_;
  ccfg.roi_max_z  = eff_roi_max_z_;

  // The band-floor invariant the yaml documents: ground search must stay inside
  // the ingested slab from anywhere the robot can sit before loadLatestMap()
  // re-bands. Violated, groundZAt returns NaN on a downslope and every
  // terrain-mode consumer degrades — silently, since NaN reads as "no ground
  // here" and not as "the band is misconfigured". Check it at startup rather
  // than leaving it to a comment.
  if (terrain_relative_z_) {
    const float hyst =
        std::clamp(0.25f * 0.5f * (roi_max_z_ - roi_min_z_), 0.5f, 2.0f);
    const float need_below = ccfg.ground_search_below + hyst;
    const float need_above = ccfg.ground_search_above + hyst;
    if (-roi_min_z_ < need_below || roi_max_z_ < need_above) {
      RCLCPP_WARN(get_logger(),
          "terrain_relative_z: ROI z band [%.2f, %.2f] is too tight for the "
          "ground search window (below %.2f / above %.2f) plus the re-band "
          "hysteresis %.3f — needs roi_min_z <= %.3f and roi_max_z >= %.3f. "
          "Ground search will reach outside the ingested slab on slopes, "
          "groundZAt returns NaN, and vantages get rejected for want of a "
          "ground height.",
          roi_min_z_, roi_max_z_, ccfg.ground_search_below,
          ccfg.ground_search_above, hyst, -need_below, need_above);
    }
  }

  // Frontier clustering bin size (m). See member doc; previously hardcoded 5.0f.
  frontier_cluster_radius_m_ =
      static_cast<float>(dp("frontier_cluster_radius_m", 5.0));

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

  // Trajectory-level scoring (path-integrated EIG ablation). When enabled,
  // info_gain for each candidate is the sum of score_fn evaluated at sampled
  // poses along the Dijkstra path, not just the endpoint. Evaluated locally via
  // FovEvaluator on map_cache_.
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
  coord_vantage_claim_radius_m_ = dp("coord_vantage_claim_radius_m", 0.0);

  // Rendezvous. ON by default: return-to-anchor-and-wait on exploration
  // exhaustion. It only *activates* where it is meaningful — coordination on
  // (the barrier waits on peer claims) and a positive expected-peer count. In
  // single-robot / no-coordination runs (or a one-robot team) it silently
  // stays inert with no behaviour change, so a missing precondition is a plain
  // INFO, not a warning.
  rendezvous_enabled_        = dp("rendezvous_enabled", true);
  rendezvous_expected_peers_ = dp("rendezvous_expected_peers", 0);
  rendezvous_max_wait_sec_   = dp("rendezvous_max_wait_sec", 0.0);
  if (rendezvous_enabled_ &&
      (!coord_enabled_ || rendezvous_expected_peers_ <= 0)) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous inactive (coordination_enabled=%d, expected_peers=%d): "
        "running as plain exploration, finishing when goals are exhausted.",
        coord_enabled_, rendezvous_expected_peers_);
    rendezvous_enabled_ = false;
  }

  // Proximity stop (coordinated yield). ON by default and deliberately NOT
  // tied to coordination_enabled: the guard is inert until it actually tracks
  // a peer (intents from teammates, or the pose topics below), so single-robot
  // runs are bit-for-bit unaffected. Caveat: with coordination_enabled=false
  // the 1 Hz intent heartbeat is also off, so the guard NEEDS the pose topics
  // to see anything — the wiring below warns when that leaves it blind. See
  // the yaml section for the field rationale (documented panic line:
  // robot-robot < 1.5 m closing).
  proximity_stop_enabled_ = dp("proximity_stop_enabled", true);
  ProximityGuard::Config pcfg;
  pcfg.enabled        = proximity_stop_enabled_;
  pcfg.hold_dist_m    = static_cast<float>(dp("proximity_hold_dist_m", 5.0));
  pcfg.resume_dist_m  = static_cast<float>(dp("proximity_resume_dist_m", 6.0));
  pcfg.pose_stale_sec = static_cast<float>(dp("proximity_pose_stale_sec", 3.0));
  pcfg.peer_static_sec =
      static_cast<float>(dp("proximity_peer_static_sec", 10.0));
  pcfg.parked_keep_dist_m =
      static_cast<float>(dp("proximity_parked_keep_dist_m", 1.5));
  pcfg.peer_static_move_m =
      static_cast<float>(dp("proximity_peer_static_move_m", 0.3));
  pcfg.hold_release_stale_sec =
      static_cast<float>(dp("proximity_hold_release_stale_sec", 10.0));
  proximity_max_hold_sec_ = dp("proximity_max_hold_sec", 120.0);
  if (pcfg.resume_dist_m < pcfg.hold_dist_m) {
    RCLCPP_WARN(get_logger(),
        "proximity_resume_dist_m=%.2f < proximity_hold_dist_m=%.2f inverts "
        "the hysteresis band — raising resume to %.2f.",
        pcfg.resume_dist_m, pcfg.hold_dist_m, pcfg.hold_dist_m);
  }
  prox_guard_ = std::make_unique<ProximityGuard>(pcfg, robot_name_);

  // Exploitation. When enabled the planner ingests tree targets off
  // targets_topic and circles each at n_vantages occlusion-free vantage points
  // (default 3 => ~120 deg apart), dwelling exploit_dwell_sec at each. A target
  // is "successfully exploited" once min_vantages_required clear-LoS vantages
  // are dwelled. Standoff = trunk radius + vantage_standoff_m (clamped to the
  // FOV range). Disable to get pure exploration.
  exploitation_enabled_  = dp("exploitation_enabled", true);
  // Dedup by target id only by default (0 => no proximity merge). A positive
  // radius merges re-reports within it, but also collapses genuinely distinct
  // trunks that sit closer than the radius — only enable it when the producer
  // reuses ids unreliably (e.g. a detector with no tracker).
  target_dedup_radius_m_ = dp("target_dedup_radius_m", 0.0);
  int   n_vantages       = dp("n_vantages", 3);
  // Default 2-of-3: tolerate one occluded/unreachable angle and still call a
  // trunk "successfully exploited". Set == n_vantages to require a full ring.
  min_vantages_required_ = dp("min_vantages_required", 2);
  double vantage_standoff_m   = dp("vantage_standoff_m", 2.0);
  double vantage_start_angle_deg = dp("vantage_start_angle_deg", 0.0);
  exploit_dwell_sec_     = dp("exploit_dwell_sec", 8.0);
  vantage_visited_tol_m_ = dp("vantage_visited_tol_m", 0.75);
  // Give-up timer: if an active target has no selectable vantage for this long
  // (e.g. its surroundings stay unmapped / unreachable), close it PARTIAL and
  // revert rather than blocking exploration forever. <=0 disables the timeout.
  exploit_target_timeout_sec_ = dp("exploit_target_timeout_sec", 300.0);
  std::string targets_topic = dp("targets_topic",
                                 std::string("/exploration/targets"));

  // Validate the vantage counts: n_vantages must be >= 1, and
  // min_vantages_required must be in [1, n_vantages] or success is unreachable
  // and every target would close PARTIAL. Clamp + warn rather than silently
  // no-op while logging "Exploitation enabled".
  if (n_vantages < 1) {
    RCLCPP_WARN(get_logger(),
        "n_vantages=%d < 1 — clamping to 1 (exploitation would otherwise be a "
        "no-op).", n_vantages);
    n_vantages = 1;
  }
  if (min_vantages_required_ < 1) min_vantages_required_ = 1;
  if (min_vantages_required_ > n_vantages) {
    RCLCPP_WARN(get_logger(),
        "min_vantages_required=%d > n_vantages=%d — clamping to %d so success "
        "is attainable.", min_vantages_required_, n_vantages, n_vantages);
    min_vantages_required_ = n_vantages;
  }
  if (coord_enabled_ && n_vantages > 32) {
    RCLCPP_WARN(get_logger(),
        "n_vantages=%d > 32 — the team dwell-credit mask covers indices 0..31 "
        "only; higher angles are dwelled but their credit is not shared with "
        "peers.", n_vantages);
  }

  // Auto-resolve "0 means auto" knobs now that the source values are
  // declared. Cache them so the doPlan tick path doesn't re-query.
  if (cost_grid_radius_cap_m_ <= 0.0) {
    cost_grid_radius_cap_m_ = ccfg.max_radius + 2.0;
  }
  if (coord_claim_radius_m_ <= 0.0) {
    coord_claim_radius_m_ = fcfg.max_range;
  }
  // Per-vantage claim disc for exploit MinPos: defaults to the same tolerance
  // that counts a vantage "dwelled", so a claim reserves exactly one angle of
  // the ring rather than the whole tree.
  if (coord_vantage_claim_radius_m_ <= 0.0) {
    coord_vantage_claim_radius_m_ = vantage_visited_tol_m_;
  }

  // --- Components ---
  map_cache_ = std::make_unique<MapCache>(map_resolution_);
  candidate_gen_ = std::make_unique<CandidateGenerator>(ccfg);
  fov_eval_ = std::make_unique<FovEvaluator>(fcfg);
  // Fixed SCovox Beta EIG scorer — this node has no planner_type knob.
  score_fn_ = scoring::eig;
  logger_ = std::make_unique<MetricsLogger>(output_csv_);
  cost_grid_ = std::make_unique<CostGrid>();
  // Bound peer-advertised claim radii by our own exploration disc — the largest
  // claim this planner considers legitimate. Resolved above, so the auto
  // (= fov_max_range) case is already a concrete number here.
  coord_ = std::make_unique<Coordination>(
      coord_enabled_, robot_name_,
      static_cast<float>(coord_claim_radius_m_));

  // Vantage planner. Geometry from the exploit params; sensor envelope + the
  // LoS occupancy threshold reuse the same FOV config as exploration so a
  // vantage frames the trunk within the modelled sensor.
  {
    VantageConfig vcfg;
    vcfg.n_vantages      = n_vantages;
    vcfg.standoff_m      = static_cast<float>(vantage_standoff_m);
    vcfg.start_angle_rad =
        static_cast<float>(vantage_start_angle_deg * M_PI / 180.0);
    vcfg.robot_z         = ccfg.robot_z;
    vcfg.fov_min_range   = fcfg.min_range;
    vcfg.fov_max_range   = fcfg.max_range;
    vcfg.occ_stop        = fcfg.occ_stop;
    vantage_planner_ = std::make_unique<VantagePlanner>(vcfg);

    // The LoS occlusion ray-march runs at the vantage sightline height
    // (vcfg.robot_z). map_cache_ is clipped to [roi_min_z_, roi_max_z_], so if
    // the sightline sits outside that band there are no voxels to hit and the
    // occlusion check silently passes everything. Warn if misconfigured.
    // Terrain mode: the band is robot-relative, so this absolute comparison is
    // meaningless — skip it. The vantage *ring geometry* is still flat-world
    // (one standoff circle, no slope-aware standoff or pitch), but the
    // sightline height is now snapped to the local ground by exploitZAt(), so
    // the LoS ray stays inside the ingested band and the occlusion test is
    // meaningful. See exploitZAt().
    if (exploitation_enabled_ && terrain_relative_z_) {
      RCLCPP_INFO(get_logger(),
          "Terrain mode + exploitation: vantage sightlines snap to local "
          "ground + %.2f m clearance. Ring geometry is still flat-world "
          "(single standoff circle, no slope-aware standoff).",
          static_cast<double>(ccfg.z_clearance));
    }
    if (exploitation_enabled_ && !terrain_relative_z_ &&
        (vcfg.robot_z < roi_min_z_ || vcfg.robot_z > roi_max_z_)) {
      RCLCPP_WARN(get_logger(),
          "Vantage sightline height candidate_robot_z=%.2f is outside the map "
          "z-band [%.2f, %.2f]; line-of-sight occlusion checks will see no "
          "voxels and pass trivially. Widen roi_min_z/roi_max_z or move "
          "candidate_robot_z into the band.",
          vcfg.robot_z, roi_min_z_, roi_max_z_);
    }
  }

  // --- ROS interfaces ---
  std::string goal_topic = dp("goal_topic",
      std::string("/" + robot_name_ + "/goal_pose"));
  // The same OccupancyGrid the global planner consumes. We use it to
  // reject candidate viewpoints whose cell is occupied/inflated/unknown
  // before publishing them as goals.
  std::string planning_map_topic = dp("planning_map_topic",
      std::string("/" + robot_name_ + "/dscovox_node/planning_map"));
  // Master switch for the 2D planning_map (default OFF). When false the planner
  // never subscribes to planning_map_topic and never consults a 2D map in
  // exploration or exploitation — straight-line costs, no obstacle/reachability
  // filtering. Set true (and point planning_map_topic at a publisher, e.g. the
  // scovox_node) to restore 2D free-cell + reachability filtering as a hard
  // startup precondition.
  use_planning_map_ = dp("use_planning_map", false);

  // Cache ROI bounds for the fused-map ingest clip (loadLatestMap()).
  roi_min_x_ = ccfg.roi_min_x;
  roi_max_x_ = ccfg.roi_max_x;
  roi_min_y_ = ccfg.roi_min_y;
  roi_max_y_ = ccfg.roi_max_y;

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

  // Only subscribe when the planning_map is enabled. Leaving the subscription
  // uncreated guarantees latest_plan_map_ stays null for the whole run, so
  // every planning_map use site (all guarded on latest_plan_map_) takes its
  // map-less path — the planner cannot consult a 2D map even if one is being
  // published on the topic.
  if (use_planning_map_) {
    plan_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        planning_map_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          latest_plan_map_ = msg;
          have_plan_map_ = true;
        });
  }

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
    intent_pub_ = create_publisher<explo_planner_msgs::msg::RobotIntent>(
        coord_intent_topic_, qos);
    intent_sub_ = create_subscription<explo_planner_msgs::msg::RobotIntent>(
        coord_intent_topic_, qos,
        [this](explo_planner_msgs::msg::RobotIntent::SharedPtr msg) {
          if (coord_) coord_->onIntent(*msg, this->now());
          // Rendezvous: record where we were the last time we heard a
          // teammate. That pose is inside the comms bubble, so it is the
          // cheapest point to return to for reconnection. onIntent already
          // drops our own echo, but we gate on robot_id here too since we read
          // our live pose. have_pose_ guards the very first ticks before TF.
          if (rendezvous_enabled_ && have_pose_ &&
              msg->robot_id != robot_name_) {
            last_connected_anchor_ = latest_pos_;
            have_anchor_ = true;
          }
          // Proximity guard: the peer's advertised live pose (refreshed by
          // its 1 Hz heartbeat). Coarse but always available in multi-robot
          // runs; the dedicated pose topics below refine it when configured.
          if (prox_guard_ && msg->robot_id != robot_name_) {
            prox_guard_->onPeerPose(
                msg->robot_id,
                Eigen::Vector3f(static_cast<float>(msg->robot_pos.x),
                                static_cast<float>(msg->robot_pos.y),
                                static_cast<float>(msg->robot_pos.z)),
                this->now());
          }
          // Team quota: merge a peer's dwell credit into the local queue
          // immediately (not only on the next EXPLOIT_PLAN tick) so credit
          // broadcast just before the peer releases its claim can't be lost
          // to the claim TTL while this robot is mid-hop or mid-dwell.
          onPeerExploitIntent(*msg);
        });
  }

  // --- Proximity-stop wiring: peer localiser poses + the nav2 cancel client.
  if (proximity_stop_enabled_) {
    // "<robot_name>:<topic>" entries, e.g. "curt:/curt/pcl_pose". These are
    // the peers' localiser poses: already map-frame, ~10 Hz, and independent
    // of the peer's planner state — the intent heartbeat alone is 1 Hz and
    // goes silent in several states, which at 0.8 m/s closing speeds leaves
    // metre-scale pose lag. With nothing configured the guard runs on
    // intents alone (the sim launches, where no localiser runs).
    const auto entries =
        dp("proximity_peer_pose_topics", std::vector<std::string>{});
    for (const auto& e : entries) {
      const auto sep = e.find(':');
      if (sep == 0 || sep == std::string::npos || sep + 1 >= e.size()) {
        RCLCPP_WARN(get_logger(),
            "proximity_peer_pose_topics entry '%s' is not "
            "'<robot_name>:<topic>' — skipped.", e.c_str());
        continue;
      }
      const std::string peer  = e.substr(0, sep);
      const std::string topic = e.substr(sep + 1);
      if (peer == robot_name_) continue;  // own localiser: not a peer
      peer_pose_subs_.push_back(
          create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
              [this, peer](
                  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
                // Consumed in map_frame_ without reframing, same convention
                // as the fused map and tree targets.
                if (!m->header.frame_id.empty() &&
                    m->header.frame_id != map_frame_) {
                  RCLCPP_WARN_ONCE(get_logger(),
                      "Peer pose for '%s' arrives in frame '%s' != map_frame "
                      "'%s' — used without reframing.",
                      peer.c_str(), m->header.frame_id.c_str(),
                      map_frame_.c_str());
                }
                ++peer_pose_msg_count_;
                prox_guard_->onPeerPose(
                    peer,
                    Eigen::Vector3f(
                        static_cast<float>(m->pose.pose.position.x),
                        static_cast<float>(m->pose.pose.position.y),
                        static_cast<float>(m->pose.pose.position.z)),
                    this->now());
              }));
      RCLCPP_INFO(get_logger(),
          "Proximity stop: tracking peer '%s' via %s", peer.c_str(),
          topic.c_str());
    }

    // NavigateToPose client used purely for async_cancel_all_goals() on hold
    // entry. bt_navigator turns every goal_pose into a NavigateToPose goal it
    // sends itself; cancel-all from this client cancels that goal too. Never
    // used to SEND goals — the goal_pose topic remains the only command path.
    proximity_nav_cancel_action_ =
        dp("proximity_nav_cancel_action", std::string(""));
    if (proximity_nav_cancel_action_.empty())
      proximity_nav_cancel_action_ = "/" + robot_name_ + "/navigate_to_pose";
    nav_cancel_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, proximity_nav_cancel_action_);
    RCLCPP_INFO(get_logger(),
        "Proximity stop enabled: hold < %.1f m, resume > %.1f m, cancel via "
        "'%s' (%zu peer pose topic(s); intents always feed the guard).",
        pcfg.hold_dist_m, pcfg.resume_dist_m,
        proximity_nav_cancel_action_.c_str(), peer_pose_subs_.size());

    // Misconfiguration is otherwise SILENT — the guard just never holds, and
    // in the field that is indistinguishable from working. Say what it is
    // actually running on.
    if (!coord_enabled_ && peer_pose_subs_.empty()) {
      RCLCPP_WARN(get_logger(),
          "Proximity stop is enabled but coordination_enabled=false and no "
          "proximity_peer_pose_topics are set: without the 1 Hz intent "
          "heartbeat the guard only hears peers at plan time — older than "
          "pose_stale_sec by the next hop, so it will NEVER hold. Wire the "
          "peer pose topics or enable coordination.");
    } else if (peer_pose_subs_.empty()) {
      RCLCPP_WARN(get_logger(),
          "Proximity stop: no peer pose topics configured — running on the "
          "1 Hz intent heartbeat alone (up to ~1.6 m of unseen closing "
          "between updates at 2x0.8 m/s). Expected in sim; on hardware set "
          "proximity_peer_pose_topics to the peers' localiser poses.");
    }

    // Latched so a field laptop's `ros2 topic echo` shows the CURRENT state
    // immediately, not only the next transition.
    prox_state_pub_ = create_publisher<std_msgs::msg::String>(
        "proximity_hold_state",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    publishProxState("clear");
  }

  // --- Tree-target subscription. Latched (transient_local) + a deep history so
  //     a planner that joins after the scheduler has released several targets
  //     still receives all of them. Inert unless exploitation_enabled_.
  if (exploitation_enabled_) {
    auto tqos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local();
    target_sub_ = create_subscription<explo_planner_msgs::msg::TreeTarget>(
        targets_topic, tqos,
        [this](explo_planner_msgs::msg::TreeTarget::SharedPtr msg) {
          onTreeTarget(msg);
        });
    RCLCPP_INFO(get_logger(),
        "Exploitation enabled: subscribing to tree targets on %s "
        "(n_vantages=%d, min_required=%d, dwell=%.1fs)",
        targets_topic.c_str(), n_vantages, min_vantages_required_,
        exploit_dwell_sec_);
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
      "EIG exploration planner ready: max_steps=%d "
      "frame=%s base=%s planning_map=%s goal=%s",
      max_steps_, map_frame_.c_str(), base_frame_.c_str(),
      use_planning_map_ ? planning_map_topic.c_str() : "(disabled)",
      goal_topic.c_str());
  if (terrain_relative_z_) {
    RCLCPP_INFO(get_logger(),
        "Terrain-relative z ON: map z-band [%+.1f, %+.1f] m about the robot, "
        "candidates at ground + %.2f m (ground search -%.1f/+%.1f m, stack "
        "cap %.2f m), goal z %s.",
        static_cast<double>(roi_min_z_), static_cast<double>(roi_max_z_),
        static_cast<double>(ccfg.z_clearance),
        static_cast<double>(ccfg.ground_search_below),
        static_cast<double>(ccfg.ground_search_above),
        static_cast<double>(ccfg.ground_stack_max_m),
        flatten_goal_z_ ? "flattened to 0 (strict-2D nav)" : "3D (nav2 ignores z)");
  }
}

// ==================================================================
// Map ingest — fused ScovoxMap topic
// ==================================================================

void ExploPlannerNode::onScovoxMap(
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

bool ExploPlannerNode::loadLatestMap() {
  if (!latest_scovox_map_) return false;
  // Terrain mode: the ingest z-band follows the robot. Re-banding forces a
  // grid rebuild even for the same map message, but only after meaningful
  // vertical travel — the hysteresis keeps the 10 Hz PLAN retry ticks from
  // rebuilding a large grid every tick while the robot idles.
  bool reband = false;
  float band_lo = roi_min_z_, band_hi = roi_max_z_;
  if (terrain_relative_z_) {
    const float ref_z = have_pose_ ? latest_pos_.z() : 0.0f;
    // 1/4 of the band half-width, clamped to [0.5, 2] m: tight bands re-band
    // sooner, wide bands tolerate more drift before paying a rebuild.
    const float hysteresis = std::clamp(
        0.25f * 0.5f * (roi_max_z_ - roi_min_z_), 0.5f, 2.0f);
    const float drift = std::isfinite(ingest_ref_z_)
        ? std::abs(ref_z - ingest_ref_z_)
        : std::numeric_limits<float>::infinity();
    const float band_ref = (drift > hysteresis) ? ref_z : ingest_ref_z_;
    reband = (drift > hysteresis);
    band_lo = band_ref + roi_min_z_;
    band_hi = band_ref + roi_max_z_;
  }
  // Rebuild when a new message has arrived since the last ingest (onScovoxMap
  // just swaps the cached pointer, so pointer identity == unchanged snapshot)
  // or when terrain mode moved the band; otherwise repeated PLAN /
  // WAIT_FOR_MAP ticks reuse the existing grid instead of reallocating it.
  if (latest_scovox_map_ != ingested_scovox_map_ || reband) {
    // The topic carries the whole fused map; clipping here keeps map_cache_
    // bounded to the ROI as the old per-region GetRegion service did, so frontier
    // extraction and the doLogStep map stats (both walk the whole grid) stay
    // bounded. The [band_lo, band_hi] z-band defines one consistent
    // observation volume that is also what FovEvaluator clips rays to
    // (doPlan re-syncs the evaluator from eff_roi_*_z_ each tick).
    if (!map_cache_->updateFromScovoxMap(
            *latest_scovox_map_,
            Eigen::Vector3f(roi_min_x_, roi_min_y_, band_lo),
            Eigen::Vector3f(roi_max_x_, roi_max_y_, band_hi))) {
      // Unusable resolution on the wire (see MapCache::updateFromScovoxMap).
      // The previous grid is intact, so keep planning on it; do NOT latch
      // ingested_scovox_map_, so a later good publish still triggers a rebuild.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Fused map rejected: msg.resolution=%.6g is not a usable voxel size. "
          "Keeping the previous grid (%zu voxels). Check the scovox/dscovox "
          "publisher.",
          latest_scovox_map_->resolution, map_cache_->voxelCount());
      return map_cache_->voxelCount() > 0;
    }
    ingested_scovox_map_ = latest_scovox_map_;
    eff_roi_min_z_ = band_lo;
    eff_roi_max_z_ = band_hi;
    if (terrain_relative_z_)
      ingest_ref_z_ = band_lo - roi_min_z_;
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

// ==================================================================
// Trajectory-level scoring (path-integrated EIG)
// ==================================================================

bool ExploPlannerNode::scoreTrajectory(
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
    float sample_score =
        fov_eval_->evaluate(s.vp, *map_cache_, score_fn_).total_score;
    candidates[s.parent_idx].score += sample_score;
  }

  int traj_scored = 0;
  for (const auto& c : candidates) {
    if (c.score > 0.0f) ++traj_scored;
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Trajectory scoring (eig): %zu samples across %d/%zu candidates "
      "(spacing=%.1f m)",
      samples.size(), traj_scored, candidates.size(), spacing);

  return true;
}

// ==================================================================
// State machine
// ==================================================================

void ExploPlannerNode::tick() {
  updatePoseFromTF();
  trackDistance();

  // A configured-but-silent peer pose topic (typo'd name, localiser down) is
  // indistinguishable from "peer far away" to the guard — keep saying so
  // until the first message lands.
  if (proximity_stop_enabled_ && !peer_pose_subs_.empty() &&
      peer_pose_msg_count_ == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "Proximity stop: %zu peer pose topic(s) configured but nothing "
        "received on any of them yet — the guard cannot see those peers.",
        peer_pose_subs_.size());
  }

  // Coordinated proximity stop: while a nav goal is in flight, yield to a
  // higher-priority teammate moving nearby. Checked at the full 10 Hz tick
  // rate, before the state dispatch, so the hold pre-empts everything the
  // driving states would otherwise do this tick. The stationary states are
  // deliberately exempt — a dwelling/integrating/planning robot is already
  // still, and the moving peer's costmap treats it as an ordinary obstacle.
  if ((state_ == State::NAVIGATE || state_ == State::RETURN_NAV) &&
      checkProximityHold()) {
    return;
  }

  switch (state_) {
    case State::WAIT_FOR_MAP:
      // Once a fused map has been received on the topic, ingest it
      // (ROI-clipped) so have_map_ flips from the in-ROI voxel count. The map
      // arrives asynchronously via its subscription; this never blocks.
      if (!have_map_) {
        loadLatestMap();  // sets have_map_ when in-ROI voxels are present
      }
      // planning_map handling:
      //  - use_planning_map_ == true  -> hard precondition; wait for it.
      //  - use_planning_map_ == false -> not subscribed; start as soon as the
      //    fused map + pose are ready and run map-less (straight-line costs, no
      //    2D obstacle/reachability filtering, in both exploration and exploit).
      {
        const bool start = have_map_ && have_pose_ &&
                           (!use_planning_map_ || have_plan_map_);
        if (start) {
          if (use_planning_map_) {
            RCLCPP_INFO(get_logger(),
                "Map, planning_map and pose received. Starting exploration.");
          } else {
            RCLCPP_INFO(get_logger(),
                "Map and pose received; planning_map disabled "
                "(use_planning_map=false) — straight-line costs, no 2D "
                "obstacle/reachability filtering. Starting exploration.");
          }
          transitionTo(State::PLAN);
        } else {
          // Name the missing precondition so a stuck startup (wrong topic /
          // namespace / QoS, dead mapper, no TF) is diagnosable instead of a
          // silent indefinite wait.
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Waiting to start: map=%d pose=%d planning_map=%d (0 = not yet "
              "received; planning_map %s).",
              have_map_, have_pose_, have_plan_map_,
              use_planning_map_ ? "required" : "disabled");
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

    case State::EXPLOIT_PLAN:
      doExploitPlan();
      break;

    case State::EXPLOIT_DWELL:
      doExploitDwell();
      break;

    case State::RETURN_NAV:
      doReturnNav();
      break;

    case State::RETURN_SYNC:
      doReturnSync();
      break;

    case State::PROXIMITY_HOLD:
      doProximityHold();
      break;

    case State::DONE:
      // done_action == "idle": stay alive so targets released after
      // coverage-done still pull the planner into the exploit sub-loop
      // (field flow: the scheduler releases targets AT the coverage-done cue,
      // which would race a shutdown). hasPending() includes a still-ACTIVE
      // target and activate() resumes it, so an exploitation interrupted by
      // the step budget also continues here, vantage by vantage, until the
      // queue drains. When the queue empties the exploit sub-loop reverts to
      // EXPLORE -> PLAN, whose coverage check immediately lands back in DONE
      // (streak already at threshold) unless the map regressed.
      if (done_action_ == "idle") {
        if (exploitation_enabled_ && target_queue_.hasPending()) {
          target_queue_.activate();
          phase_ = Phase::EXPLOIT;
          RCLCPP_INFO(get_logger(),
              "Target arrived while DONE-idle (%zu pending) -> EXPLOIT.",
              target_queue_.pendingCount());
          transitionTo(State::EXPLOIT_PLAN);
        } else {
          RCLCPP_INFO_ONCE(get_logger(),
              "Exploration finished; idling (done_action=idle). Planner "
              "stays up and will exploit any targets that arrive.");
        }
        break;
      }
      if (!shutdown_requested_) {
        shutdown_requested_ = true;
        RCLCPP_INFO(get_logger(),
            "Exploration finished. Shutting down planner node.");
        rclcpp::shutdown();
      }
      break;
  }
}

void ExploPlannerNode::transitionTo(State s) {
  state_ = s;
  state_enter_time_ = this->now();
  // The post-arrival rotation deadline is per-NAVIGATE-cycle and is armed
  // lazily on arrival at the XY goal; disarm it on every entry.
  if (s == State::NAVIGATE) rotate_deadline_armed_ = false;
}

// ==================================================================
// PLAN state
// ==================================================================

void ExploPlannerNode::doPlan() {
  // Step budget: never start a new exploration step past max_steps_. Only
  // reachable with done_action=idle — the exploit sub-loop drains its queue
  // past the budget (deliberate, see State::DONE) and its empty-queue revert
  // lands here; without this it would sneak one stray exploration hop per
  // drain. In shutdown mode LOG_STEP routes to DONE before PLAN ever runs
  // with a spent budget, so legacy behaviour is untouched.
  if (step_ >= max_steps_) {
    finishOrRendezvous("step-budget");
    return;
  }

  auto plan_start = this->now();

  failed_goals_.prune(plan_start.seconds(), failed_goal_ttl_sec_);
  if (coord_) coord_->prune(plan_start);

  // Exploitation takes priority over exploration: the moment a target is
  // waiting, activate it and hand off to the vantage sub-loop. With no targets
  // this is a single bool test and the exploration path below runs unchanged.
  if (exploitation_enabled_ && target_queue_.hasPending()) {
    target_queue_.activate();
    phase_ = Phase::EXPLOIT;
    RCLCPP_INFO(get_logger(),
        "Target queued (%zu pending) -> switching to EXPLOIT.",
        target_queue_.pendingCount());
    transitionTo(State::EXPLOIT_PLAN);
    return;
  }
  phase_ = Phase::EXPLORE;

  // Rebuild the local map_cache_ from the latest fused map received on the
  // topic (ROI-clipped) so frontier extraction + FOV scoring below run on
  // fresh consensus data.
  if (!loadLatestMap()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No fused map received yet on the dscovox topic; retrying next tick.");
    return;
  }

  // Coverage saturation check. Runs before candidate generation so we
  // can short-circuit out of PLAN entirely once the ROI is fully known.
  // Requires N consecutive low-unknown ticks to avoid premature DONE
  // from a momentary measurement gap.
  if (done_unknown_fraction_ > 0.0) {
    const char* cov_src = "";
    double unk = coverageUnknownFraction(&cov_src);
    if (unk >= 0.0 && unk < done_unknown_fraction_) {
      ++coverage_done_streak_;
      RCLCPP_INFO(get_logger(),
          "Step %d: ROI unknown fraction %.3f < %.3f (source=%s, streak %d/%d)",
          step_, unk, done_unknown_fraction_, cov_src,
          coverage_done_streak_, done_min_consecutive_steps_);
      if (coverage_done_streak_ >= done_min_consecutive_steps_) {
        RCLCPP_INFO(get_logger(),
            "Exploration complete: ROI saturated "
            "(unknown=%.3f, %d steps, %.2f m traveled).",
            unk, step_, cumulative_distance_);
        // With rendezvous on and a teammate still out of comms, return to the
        // anchor and wait for the team instead of finishing (see below).
        finishOrRendezvous("coverage-saturated");
        return;
      }
    } else {
      coverage_done_streak_ = 0;
      if (unk < 0.0) {
        // -1 = the selected source cannot measure: done_coverage_source
        // "planning_map" with no planning_map published, or a degenerate ROI
        // box. ("auto" falls back to the scovox column measure, which always
        // yields a value once the fused map is loaded, so it only lands here
        // on a bad ROI.) Surface it instead of silently relying on max_steps.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
            "Coverage termination (done_unknown_fraction=%.3f) INACTIVE: "
            "source '%s' cannot measure the ROI unknown fraction (no "
            "planning_map / degenerate ROI); stopping only at max_steps=%d.",
            done_unknown_fraction_, cov_src, max_steps_);
      }
    }
  }

  auto robot_pos = latest_pos_;
  float robot_yaw = latest_yaw_;

  // Terrain mode: loadLatestMap() may have re-banded the map z-slab around
  // the robot; keep the FOV ray z-clip in lock-step so rays leaving the
  // ingested band are clipped, not scored against absent voxels. The candidate
  // z clamp rides the same band — a candidate above it would put its own FOV
  // origin outside the ingested volume, where every ray scores the maximal
  // Beta(1,1) prior.
  if (terrain_relative_z_) {
    fov_eval_->setRoiZ(eff_roi_min_z_, eff_roi_max_z_);
    candidate_gen_->setRoiZ(eff_roi_min_z_, eff_roi_max_z_);
  }

  // Generate candidates: a polar grid of viewpoints around the robot
  // (local EIG hops) PLUS frontier centroids anywhere in the ROI (long-
  // range targets for escaping local IG maxima). Flat mode passes a nullptr
  // map (3D occupancy check skipped; the 2D planning_map filter below
  // handles it); terrain mode passes map_cache_ so candidates snap to the
  // local ground + clearance (and get the 3D occupancy check at that
  // height). Frontiers are extracted locally from map_cache_ (the fused
  // grid pulled via the topic), over the effective (robot-relative in
  // terrain mode) z band.
  const MapCache* terrain_map =
      terrain_relative_z_ ? map_cache_.get() : nullptr;
  auto candidates =
      candidate_gen_->generate(robot_pos, robot_yaw, terrain_map);
  size_t n_radial = candidates.size();
  auto frontiers = map_cache_->findFrontierCentroids(
      eff_roi_min_z_, eff_roi_max_z_, frontier_cluster_radius_m_);
  candidate_gen_->addFrontierCandidates(candidates, frontiers, robot_pos,
                                        terrain_map);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "Candidates: %zu radial + %zu frontier centroids = %zu total",
      n_radial, candidates.size() - n_radial, candidates.size());

  if (candidates.empty()) {
    RCLCPP_WARN(get_logger(), "No valid candidates. Retrying next tick.");
    return;
  }

  // Build the bounded cost grid from the latched planning_map. ~5 ms once
  // per PLAN tick at the auto bound, then O(1) per-candidate lookups.
  bool skip_reachability = false;
  if (latest_plan_map_) {
    cost_grid_->build(*latest_plan_map_);
    cost_grid_->floodFrom(robot_pos,
                          static_cast<float>(cost_grid_radius_cap_m_));
    // cost_grid_ is shared with the exploitation planner, and this flood is
    // RADIUS-BOUNDED while that one is unbounded — invalidate its cache so it
    // does not reuse a truncated flood as if it were the full one.
    exploit_flood_valid_ = false;
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
  } else {
    // Best-effort mode: no planning_map this tick -> fall back to straight-line
    // distances and skip the reachability filter (there is no grid to flood).
    // The candidate free/occupied filter below is likewise skipped when absent.
    skip_reachability = true;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No planning_map available — using straight-line distances and "
        "skipping reachability filtering this tick.");
  }

  // Evaluate candidates -> populates per-candidate FOV info gain.
  //
  // trajectory_scoring (param, default false): when true, info_gain is the
  // sum of EIG scores at poses sampled along the Dijkstra path, not just the
  // endpoint. Otherwise endpoint scoring. Both run locally via FovEvaluator
  // on map_cache_ (the fused ROI grid in dscovox mode).
  {
    const bool use_traj = trajectory_scoring_
                          && cost_grid_ && !skip_reachability;
    if (use_traj) {
      if (!scoreTrajectory(candidates)) {
        RCLCPP_WARN(get_logger(),
            "Trajectory scoring failed, retrying next tick.");
        return;
      }
    } else {
      // Endpoint EIG scoring on the local map.
      fov_eval_->evaluateAll(candidates, *map_cache_, score_fn_);
    }
  }

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
  constexpr float kCostEpsilon = 0.1f;

  std::vector<float> info_gain(candidates.size(), 0.0f);
  std::vector<float> path_cost(candidates.size(), 0.0f);
  float sum_info = 0.0f;
  float sum_cost_finite = 0.0f;
  int   n_cost_finite = 0;
  {
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

  // Sort the selection order by utility descending. The cost-grid
  // reachability filter runs after the sort as one of the candidate filters
  // in the walk below.
  std::vector<size_t> order(candidates.size());
  std::iota(order.begin(), order.end(), 0);
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
    //    inflated obstacles. Skipped when the flood barely reached
    //    anything (robot trapped in inflation zone).
    if (!skip_reachability && cost_grid_ &&
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
  pending_selected_info_gain_ = info_gain[selected_idx];
  pending_selected_path_cost_ = path_cost[selected_idx];
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
  // act on it when their own coord_->enabled() is true. EIG planner id = 0.
  if (intent_pub_ && coord_) {
    current_intent_msg_ = coord_->buildIntent(
        current_goal_, robot_pos, plan_end,
        static_cast<float>(coord_claim_ttl_sec_),
        static_cast<float>(coord_claim_radius_m_),
        /*planner_type_id (eig)=*/0u, map_frame_);
    intent_pub_->publish(current_intent_msg_);
    have_active_intent_ = true;
  }

  transitionTo(State::NAVIGATE);

  // Initialise smart-timeout state for this NAVIGATE cycle. Budget the DRIVEN
  // distance, not the straight line: the selected candidate's Dijkstra path
  // cost was already computed a few lines above, and a goal 5 m away in a
  // straight line that needs a 15 m detour around an obstacle used to get a
  // 5 m budget, time out, and be blacklisted for being "unreachable" when it
  // was merely far. With no cost grid path_cost IS the straight line, so this
  // is a no-op in the default use_planning_map=false configuration.
  float dx = current_goal_.position.x() - robot_pos.x();
  float dy = current_goal_.position.y() - robot_pos.y();
  float dist = std::sqrt(dx * dx + dy * dy);
  if (std::isfinite(pending_selected_path_cost_))
    dist = std::max(dist, pending_selected_path_cost_);
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

double ExploPlannerNode::unknownFractionInRoi() const {
  if (!latest_plan_map_) return -1.0;
  return explo_planner::unknownFractionInRoi(
      *latest_plan_map_, {roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_});
}

double ExploPlannerNode::coverageUnknownFraction(const char** source) const {
  const bool use_plan_map =
      done_coverage_source_ == "planning_map" ||
      (done_coverage_source_ == "auto" && latest_plan_map_ != nullptr);
  if (use_plan_map) {
    *source = "planning_map";
    return unknownFractionInRoi();
  }
  // scovox source: 2.5D column coverage of the ROI footprint, measured on the
  // fused 3D map already ingested (ROI + z-band clipped) into map_cache_ by
  // loadLatestMap() this tick. NB in flat mode the ingest band is the absolute
  // [roi_min_z, roi_max_z]: on terrain outside that band the cache stays empty
  // and this reads 1.0 (never done) — set a per-area z band or
  // terrain_relative_z when the ground leaves the default band.
  *source = "scovox";
  return map_cache_->unknownColumnFraction(roi_min_x_, roi_max_x_,
                                           roi_min_y_, roi_max_y_);
}

bool ExploPlannerNode::isCellFree(const Eigen::Vector3f& pos) const {
  // No map: not free (matches the old kCellNoData -> v >= 0 failure).
  return latest_plan_map_ &&
         explo_planner::isCellFree(*latest_plan_map_, pos);
}

bool ExploPlannerNode::isCellOccupied(const Eigen::Vector3f& pos) const {
  // No map: conservatively occupied (matches the old kCellNoData default).
  return !latest_plan_map_ ||
         explo_planner::isCellOccupied(*latest_plan_map_, pos);
}

// ==================================================================
// NAVIGATE state
// ==================================================================

void ExploPlannerNode::doNavigate() {
  // Responsiveness: if a target appears while we're mid-hop on an EXPLORE goal,
  // release the goal and switch to exploitation immediately rather than
  // finishing the exploration hop first.
  if (phase_ == Phase::EXPLORE && exploitation_enabled_ &&
      target_queue_.hasPending()) {
    RCLCPP_INFO(get_logger(),
        "Target appeared mid-hop -> releasing exploration goal, switching "
        "to EXPLOIT.");
    have_active_intent_ = false;
    target_queue_.activate();
    phase_ = Phase::EXPLOIT;
    transitionTo(State::EXPLOIT_PLAN);
    return;
  }

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
      if (phase_ == Phase::EXPLOIT) {
        if (current_is_approach_) {
          // Reached an approach waypoint (not a vantage): re-plan from here so
          // the now-better-mapped surroundings can yield a selectable vantage.
          // Keep the claim — we are still working this target. Arriving IS
          // progress on the target, so re-arm the give-up timer exactly like a
          // completed dwell does — otherwise the drive toward a distant trunk
          // (~18 m at 0.15 m/s eats a 120 s budget) closes the target PARTIAL
          // before the ring is ever tried. Termination is preserved: each
          // approach must land meaningfully NEARER the trunk than the last
          // (computeApproachGoal returns false otherwise), so the resets are
          // finite and the hold-and-retry path still runs down the timer.
          if (exploit_target_timing_)
            exploit_target_started_sec_ = this->now().seconds();
          RCLCPP_INFO(get_logger(),
              "Reached approach waypoint for target %d (dist=%.2f) -> "
              "re-planning vantages.", pending_exploit_target_id_, dist);
          transitionTo(State::EXPLOIT_PLAN);
          return;
        }
        // Reached a vantage: dwell. Hold the MinPos claim through the dwell so a
        // peer doesn't poach this angle mid-capture; finishActiveTarget releases
        // it when the target closes.
        RCLCPP_INFO(get_logger(),
            "Reached vantage %d of target %d (dist=%.2f) -> dwelling %.1fs.",
            current_vantage_index_, pending_exploit_target_id_, dist,
            exploit_dwell_sec_);
        transitionTo(State::EXPLOIT_DWELL);
        return;
      }
      // Exploration goal reached: integrate the new observation.
      have_active_intent_ = false;
      RCLCPP_INFO(get_logger(),
          "Goal reached: dist=%.2f yaw_err=%.1f deg", dist,
          yaw_err * 180.0f / static_cast<float>(M_PI));
      transitionTo(State::INTEGRATE);
      return;
    }
    // At XY, waiting for the controller to finish rotating. This gets its OWN
    // deadline, armed on arrival. Reusing nav_budget_sec_ was wrong twice
    // over: that budget covers the DRIVE and is measured from NAVIGATE entry,
    // so a robot that arrived at 15.9 s against a 16 s budget was failed on
    // the very next tick with the goal already underfoot — and failGoal()
    // blacklists current_goal_.position, so the robot then rejected every
    // candidate within failed_goal_radius_m of where it was standing for the
    // whole failed_goal_ttl_sec.
    auto now = this->now();
    if (!rotate_deadline_armed_) {
      rotate_deadline_armed_ = true;
      rotate_start_time_ = now;
    }
    if ((now - rotate_start_time_).seconds() > goal_rotate_timeout_sec_) {
      failGoal("budget-rotate", (now - state_enter_time_).seconds());
      return;
    }
    // Keep re-publishing so the navigator keeps servicing the yaw — the old
    // early return skipped this, so a goal dropped by the controller mid-turn
    // was never re-sent. The no-progress watchdog is deliberately NOT applied
    // here: rotating in place accumulates no translation, so it would fire on
    // every correct rotation.
    republishGoal(current_goal_);
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

  // Keep-alive re-send (throttled; see republishGoal).
  republishGoal(current_goal_);
}

// Park the current goal in the failed-goal blacklist with a tagged reason
// and transition out of NAVIGATE. Centralised so both timeout paths
// (budget exceeded / no progress) share the same logging + bookkeeping.
void ExploPlannerNode::failGoal(const char* reason, double elapsed) {
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

// ==================================================================
// Rendezvous (multi-robot reconnection)
// ==================================================================

// Called at exploration exhaustion (coverage saturated). If rendezvous is on,
// an anchor is known, and a teammate is still out of comms, return to the
// anchor and wait for the team; otherwise finish. When the whole team is
// already present the map is already merged, so exhaustion here means the team
// is genuinely done — everyone reaches this together and lands in DONE.
void ExploPlannerNode::finishOrRendezvous(const char* reason) {
  const int active =
      coord_ ? static_cast<int>(coord_->activePeerCount()) : 0;
  if (shouldRendezvous(rendezvous_enabled_, have_anchor_, active,
                       rendezvous_expected_peers_)) {
    startReturnToAnchor(reason);
    return;
  }
  if (rendezvous_enabled_ && rendezvous_expected_peers_ > 0) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: exploration ended [%s] with full team present "
        "(%d/%d peers) -> DONE.",
        reason, active, rendezvous_expected_peers_);
  }
  transitionTo(State::DONE);
}

// Arm the drive back to the last-connected anchor, reusing the NAVIGATE
// smart-timeout + arrival test. A presence intent is published (and re-sent by
// the heartbeat, which now fires in the RETURN states) so teammates arriving
// later count us at the barrier — without it two robots waiting at their own
// anchors would never see each other and would deadlock.
void ExploPlannerNode::startReturnToAnchor(const char* reason) {
  // The rendezvous barrier is HARD: nothing preempts it. doNavigate()
  // interrupts an exploration hop the moment a target arrives, but the RETURN
  // states deliberately do NOT check target_queue_ — with
  // rendezvous_max_wait_sec <= 0 (wait forever, the default) a robot that
  // serviced trees on the way back would leave its teammate blocked at the
  // anchor indefinitely.
  //
  // Stand the queue down rather than destroying it: the ACTIVE target is
  // demoted to PENDING and the phase reset to EXPLORE, so the exploit claim
  // stops being broadcast (the fresh non-exploit intent published below
  // overwrites current_intent_msg_, clearing exploit/target_id/dwelled_mask —
  // peers must not merge dwell credit from a robot that is driving home) and
  // doPlan() picks the target back up once the barrier releases. Leaving it
  // ACTIVE mattered once the step-budget path started routing through here:
  // that path can fire mid-exploitation, unlike coverage saturation.
  if (target_queue_.active() || target_queue_.hasPending()) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: standing down exploitation (%zu target(s) still open) — "
        "the barrier takes priority; they resume after the team reconnects.",
        target_queue_.pendingCount());
    target_queue_.deactivate();
    phase_ = Phase::EXPLORE;
    // Stop the give-up timer along with the queue. The demoted target comes
    // back with the SAME id after the barrier, so doExploitPlan's re-latch
    // check ("different id?") would keep the old start time — the whole
    // return drive plus the barrier wait would count against the target and
    // it would re-activate already past exploit_target_timeout_sec, closing
    // PARTIAL without a single new dwell attempt.
    exploit_target_timing_ = false;
  }

  current_goal_ = CandidateViewpoint{};
  current_goal_.position = last_connected_anchor_;
  current_goal_.yaw = latest_yaw_;

  RCLCPP_INFO(get_logger(),
      "Rendezvous: exploration ended [%s], team incomplete (%d/%d peers) "
      "-> returning to anchor (%.2f, %.2f).",
      reason,
      coord_ ? static_cast<int>(coord_->activePeerCount()) : 0,
      rendezvous_expected_peers_,
      last_connected_anchor_.x(), last_connected_anchor_.y());

  publishGoal(current_goal_);

  if (intent_pub_ && coord_) {
    current_intent_msg_ = coord_->buildIntent(
        current_goal_, latest_pos_, this->now(),
        static_cast<float>(coord_claim_ttl_sec_),
        static_cast<float>(coord_claim_radius_m_),
        /*planner_type_id (eig)=*/0u, map_frame_);
    intent_pub_->publish(current_intent_msg_);
    have_active_intent_ = true;
  }

  transitionTo(State::RETURN_NAV);

  const float dx = current_goal_.position.x() - latest_pos_.x();
  const float dy = current_goal_.position.y() - latest_pos_.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  nav_budget_sec_ = navBudgetSec(dist, nav_speed_est_mps_, nav_safety_factor_,
                                 nav_min_timeout_sec_, nav_max_timeout_sec_);
  progress_check_time_ = state_enter_time_;
  progress_check_dist_ = cumulative_distance_;
}

// Drive toward the anchor. If the whole team reconnects en route, the barrier
// is already satisfied — re-plan without finishing the drive. On arrival (or if
// the anchor turns out unreachable) hand off to RETURN_SYNC to wait for the
// team from wherever we ended up.
void ExploPlannerNode::doReturnNav() {
  const int active =
      coord_ ? static_cast<int>(coord_->activePeerCount()) : 0;
  if (teamComplete(active, rendezvous_expected_peers_)) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: team reconnected en route (%d/%d) -> re-planning against "
        "merged map.", active, rendezvous_expected_peers_);
    have_active_intent_ = false;
    transitionTo(State::PLAN);
    return;
  }

  const auto robot_pos = latest_pos_;
  const float dx = robot_pos.x() - current_goal_.position.x();
  const float dy = robot_pos.y() - current_goal_.position.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  if (dist < goal_xy_tol_) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: reached anchor (dist=%.2f) -> waiting for team.", dist);
    transitionTo(State::RETURN_SYNC);
    return;
  }

  const auto now = this->now();
  const double elapsed = (now - state_enter_time_).seconds();
  // Distance-budgeted timeout / no-progress watchdog: if the anchor can't be
  // reached, wait for the team from here rather than looping on the drive
  // (we're at least closer to comms than where exploration stranded us).
  if (elapsed > nav_budget_sec_) {
    RCLCPP_WARN(get_logger(),
        "Rendezvous: anchor unreachable within budget (%.1fs, dist=%.2f) "
        "-> waiting for team from current pose.", elapsed, dist);
    transitionTo(State::RETURN_SYNC);
    return;
  }
  const double window_elapsed = (now - progress_check_time_).seconds();
  if (window_elapsed > progress_window_sec_) {
    const float delta = cumulative_distance_ - progress_check_dist_;
    if (delta < progress_min_distance_m_) {
      RCLCPP_WARN(get_logger(),
          "Rendezvous: no progress toward anchor -> waiting for team from "
          "current pose.");
      transitionTo(State::RETURN_SYNC);
      return;
    }
    progress_check_time_ = now;
    progress_check_dist_ = cumulative_distance_;
  }
  republishGoal(current_goal_);
}

// Hold at the anchor until the whole team is back in comms, then re-plan. With
// rendezvous_max_wait_sec <= 0 the wait is unbounded (STAY until all connected,
// the requested default); a positive value is the field escape hatch. The
// heartbeat keeps broadcasting our presence throughout so arriving peers count
// us and release their own barriers.
void ExploPlannerNode::doReturnSync() {
  const int active =
      coord_ ? static_cast<int>(coord_->activePeerCount()) : 0;
  if (teamComplete(active, rendezvous_expected_peers_)) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: full team connected (%d/%d) -> re-planning against "
        "merged map.", active, rendezvous_expected_peers_);
    have_active_intent_ = false;
    transitionTo(State::PLAN);
    return;
  }

  const double waited = (this->now() - state_enter_time_).seconds();
  if (rendezvousWaitExpired(waited, rendezvous_max_wait_sec_)) {
    RCLCPP_WARN(get_logger(),
        "Rendezvous: waited %.0fs for team (%d/%d present); "
        "max_wait=%.0fs reached -> giving up and finishing.",
        waited, active, rendezvous_expected_peers_, rendezvous_max_wait_sec_);
    have_active_intent_ = false;
    transitionTo(State::DONE);
    return;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Rendezvous: waiting at anchor for team (%d/%d present)%s.",
      active, rendezvous_expected_peers_,
      rendezvous_max_wait_sec_ > 0.0 ? "" : " (no timeout)");
}

// Re-publish the active intent on a fixed sim-time cadence so peers
// don't lose the claim through TTL while we're navigating to it.
// Stamps the message with the current time so peer expiry resets.
void ExploPlannerNode::heartbeatTick() {
  if (!coord_enabled_) return;
  if (!have_active_intent_) return;
  // Re-publish while we hold a claim: NAVIGATE/INTEGRATE (exploration), the
  // EXPLOIT states, and the RETURN states. The dwell in particular can outlast
  // the claim TTL, so a peer would otherwise poach the vantage angle
  // mid-capture; in RETURN the beacon is what lets teammates arriving at the
  // rendezvous count us and release the barrier. PROXIMITY_HOLD keeps beating
  // too: the interrupted goal is resumed after the hold, so its claim must
  // survive, and the beacon (with the live robot_pos refreshed below) is what
  // feeds the right-of-way peer's view of us while we sit in its way.
  if (state_ != State::NAVIGATE && state_ != State::INTEGRATE &&
      state_ != State::EXPLOIT_PLAN && state_ != State::EXPLOIT_DWELL &&
      state_ != State::RETURN_NAV && state_ != State::RETURN_SYNC &&
      state_ != State::PROXIMITY_HOLD) {
    return;
  }
  if (!intent_pub_) return;
  current_intent_msg_.header.stamp = this->now();
  // Refresh robot_pos too, not just the stamp. Coordination::selfWinsAgainst
  // compares the receiver's LIVE pose against this field, so a frozen value
  // breaks the MinPos guarantee that exactly one robot yields per pairwise
  // conflict — and it breaks it in the harmful direction: as we close on our
  // own claimed goal we keep advertising the far-away pose we held at claim
  // time, a peer computes that it is the closer robot, and it poaches a goal
  // under active pursuit. nav_max_timeout_sec (60 s) is 12x the claim TTL
  // (5 s), and the heartbeat exists precisely to hold a claim across a long
  // hop, so the stale pose was broadcast for that entire window.
  current_intent_msg_.robot_pos.x = latest_pos_.x();
  current_intent_msg_.robot_pos.y = latest_pos_.y();
  current_intent_msg_.robot_pos.z = latest_pos_.z();
  intent_pub_->publish(current_intent_msg_);
}

// ==================================================================
// Proximity stop (coordinated yield)
// ==================================================================
//
// While driving (NAVIGATE / RETURN_NAV), yield to a higher-priority teammate
// moving nearby: cancel the in-flight nav2 goal, hold still, and resume the
// same goal once the teammate has cleared off (hysteresis) or parked. Right of
// way is the lexicographically SMALLER robot_name — the same total order as
// the MinPos tiebreak — computed from ids alone, so both robots always agree
// on who yields: exactly one of any pair stops, never both (standoff) and
// never neither (race). See ProximityGuard for the freshness/parked rules.
//
// This is a best-effort COORDINATION layer, not a certified safety function:
// it needs live peer pose data (intents at 1 Hz + the optional localiser
// topics at ~10 Hz), both planners alive, and a nav2 that honours the cancel.
// The right-of-way robot does NOT stop — its costmap sees the held robot as an
// ordinary obstacle — and the crewed 1.5 m panic stop from the experiment
// script remains the hard backstop.

bool ExploPlannerNode::checkProximityHold() {
  if (!prox_guard_ || !prox_guard_->enabled() || !have_pose_) return false;
  const auto d =
      prox_guard_->evaluate(latest_pos_, this->now(), /*holding=*/false);
  if (!d.hold) return false;
  enterProximityHold(d);
  return true;
}

void ExploPlannerNode::enterProximityHold(const ProximityGuard::Decision& d) {
  prox_resume_state_ = state_;
  // Bank the drive time already spent on this goal: the resume backdates
  // state_enter_time_ by it, so the nav budget continues rather than
  // restarting — without this, every hold refunded the FULL budget and
  // repeated holds on a crossing route left one goal's drive time unbounded.
  prox_nav_elapsed_sec_ = (this->now() - state_enter_time_).seconds();
  ++prox_hold_count_;
  RCLCPP_WARN(get_logger(),
      "Proximity hold #%d: yielding to '%s' at %.2f m (< %.2f m). Cancelling "
      "the nav goal; resuming beyond %.2f m or when the peer parks.",
      prox_hold_count_, d.peer_id.c_str(), d.dist_m,
      prox_guard_->config().hold_dist_m, prox_guard_->config().resume_dist_m);

  // Stop the platform. Ceasing to publish goal_pose does NOT stop nav2 — the
  // last accepted NavigateToPose goal runs to completion — so the in-flight
  // goal is cancelled through the action interface (bt_navigator's server
  // honours cancel-all from any client, including for the goals it sent
  // itself off the goal_pose topic). The cancel is fire-and-forget on the
  // wire, so a brake goal at the robot's own pose goes out AS WELL: a lost or
  // rejected cancel must not leave nav2 driving while the planner believes it
  // holds, and whichever order bt_navigator processes the pair, the robot
  // ends with either no goal or a zero-travel goal. The cancel response is
  // checked async below purely to say out loud when it did not land.
  // current_goal_ is left untouched; the resume re-publishes it fresh.
  if (nav_cancel_client_ && nav_cancel_client_->action_server_is_ready()) {
    nav_cancel_client_->async_cancel_all_goals(
        [this](auto resp) {
          if (!resp ||
              resp->return_code !=
                  action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
            RCLCPP_WARN(get_logger(),
                "Proximity hold: nav goal cancel returned code %d — the "
                "brake goal is the only thing stopping the robot.",
                resp ? static_cast<int>(resp->return_code) : -1);
          } else {
            RCLCPP_INFO(get_logger(),
                "Proximity hold: nav cancel accepted (%zu goal(s) "
                "cancelling).", resp->goals_canceling.size());
          }
        });
  } else {
    RCLCPP_WARN(get_logger(),
        "Proximity hold: action server '%s' unavailable for cancel — the "
        "brake goal is the only stop command.",
        proximity_nav_cancel_action_.c_str());
  }
  CandidateViewpoint brake;
  brake.position = latest_pos_;
  brake.yaw = latest_yaw_;
  publishGoal(brake);

  char buf[160];
  std::snprintf(buf, sizeof(buf), "hold peer=%s dist=%.2f n=%d",
                d.peer_id.c_str(), d.dist_m, prox_hold_count_);
  publishProxState(buf);
  transitionTo(State::PROXIMITY_HOLD);
}

void ExploPlannerNode::doProximityHold() {
  const auto now = this->now();
  const double held = (now - state_enter_time_).seconds();
  const auto d = prox_guard_->evaluate(latest_pos_, now, /*holding=*/true);

  const bool timed_out =
      proximity_max_hold_sec_ > 0.0 && held >= proximity_max_hold_sec_;
  if (d.hold && !timed_out) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Proximity hold: '%s' at %.2f m (resume > %.2f m); held %.0fs.",
        d.peer_id.c_str(), d.dist_m, prox_guard_->config().resume_dist_m,
        held);
    return;
  }
  // The escape-hatch warning only applies when the peer is genuinely still
  // near — when the timeout lands on the same tick the guard releases, the
  // ordinary release below tells the story.
  if (timed_out && d.hold) {
    RCLCPP_WARN(get_logger(),
        "Proximity hold: max_hold_sec=%.0f reached with '%s' still at %.2f m "
        "— resuming anyway (escape hatch).",
        proximity_max_hold_sec_, d.peer_id.c_str(), d.dist_m);
  }

  // Resume the interrupted drive on the SAME goal. The re-publish is
  // mandatory — the cancel consumed nav2's goal, so only a fresh goal_pose
  // restarts it. state_enter_time_ is backdated by the drive time the goal
  // had already consumed, so the nav budget CONTINUES across the hold; the
  // progress window starts fresh (held time is not lack of progress). The
  // exploit give-up timer gets the held time back for the same reason: a
  // hold is not target stall.
  if (exploit_target_timing_) exploit_target_started_sec_ += held;
  prox_hold_total_sec_ += held;
  const char* why = d.hold ? "max-hold" : d.note.c_str();
  RCLCPP_INFO(get_logger(),
      "Proximity hold released after %.1fs [%s: '%s' at %.2f m] -> resuming "
      "drive to (%.2f, %.2f).",
      held, why, d.peer_id.c_str(), d.dist_m,
      current_goal_.position.x(), current_goal_.position.y());
  char buf[160];
  std::snprintf(buf, sizeof(buf), "clear reason=%s held=%.1f", why, held);
  publishProxState(buf);
  transitionTo(prox_resume_state_);
  state_enter_time_ =
      now - rclcpp::Duration::from_seconds(prox_nav_elapsed_sec_);
  progress_check_time_ = now;
  progress_check_dist_ = cumulative_distance_;
  publishGoal(current_goal_);
}

void ExploPlannerNode::publishProxState(const std::string& state) {
  if (!prox_state_pub_) return;
  std_msgs::msg::String m;
  m.data = state;
  prox_state_pub_->publish(m);
}

// ==================================================================
// INTEGRATE state
// ==================================================================

void ExploPlannerNode::doIntegrate() {
  double elapsed = (this->now() - state_enter_time_).seconds();
  if (elapsed >= integrate_wait_) {
    transitionTo(State::LOG_STEP);
  }
}

// ==================================================================
// LOG_STEP state
// ==================================================================

void ExploPlannerNode::doLogStep() {
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

  // Cumulative proximity-hold columns. LOG_STEP is never reached mid-hold
  // (holds only interrupt driving states), so these are always settled.
  m.prox_hold_count     = prox_hold_count_;
  m.prox_hold_total_sec = static_cast<float>(prox_hold_total_sec_);

  // Exploitation columns. Left at the "explore"/-1/0 defaults for exploration
  // rows; filled from the dwelled vantage for exploitation rows.
  if (phase_ == Phase::EXPLOIT) {
    m.phase             = "exploit";
    m.target_id         = pending_exploit_target_id_;
    m.vantage_index     = pending_exploit_vantage_index_;
    m.n_vantages_valid  = pending_exploit_n_valid_;
    m.vantage_los_clear = pending_exploit_los_clear_;
    m.dwell_sec         = pending_exploit_dwell_sec_;
  }

  // Aggregate map stats from map_cache_ (the fused ROI grid). The dscovox node
  // no longer computes these — scoring and stats both live in the planner now.
  // The grid walk + frontier-neighbour logic lives in MapCache::computeStats()
  // (shared with findFrontierCentroids, unit-testable in isolation).
  const auto stats = map_cache_->computeStats();
  m.total_observed_voxels = stats.total_voxels;
  m.frontier_voxels       = stats.frontier_voxels;
  m.mean_eig              = stats.mean_eig;
  m.mean_entropy          = stats.mean_entropy;
  m.mean_variance         = stats.mean_variance;

  logger_->logStep(m);
  RCLCPP_INFO(get_logger(),
      "Step %d logged: voxels=%d frontiers=%d dist=%.2f mean_eig=%.4f",
      step_, m.total_observed_voxels, m.frontier_voxels,
      m.distance_traveled, m.mean_eig);

  step_++;
  if (step_ >= max_steps_) {
    RCLCPP_INFO(get_logger(),
        "Step budget reached: %d steps, %.2fm traveled, %d final voxels.",
        step_, cumulative_distance_, m.total_observed_voxels);
    // Route through finishOrRendezvous, NOT straight to DONE. This is the
    // dominant termination path in dense terrain (the coverage threshold may
    // never be reached), and going directly to DONE meant a robot that spent
    // its step budget shut down wherever it happened to stop — never returning
    // to last_connected_anchor_ and never releasing its teammate's barrier.
    // The return drive happens after the budget is spent, so it costs no steps.
    finishOrRendezvous("step-budget");
  } else {
    // Route by phase: exploitation steps loop back to the vantage planner,
    // exploration steps to the exploration planner.
    transitionTo(phase_ == Phase::EXPLOIT ? State::EXPLOIT_PLAN
                                          : State::PLAN);
  }
}

// ==================================================================
// EXPLOIT states — vantage-point selection + dwell around a tree target
// ==================================================================

void ExploPlannerNode::onTreeTarget(
    const explo_planner_msgs::msg::TreeTarget::SharedPtr& msg) {
  // STATUS_DONE is a completion broadcast (reserved for a future peer/detector
  // marking a tree already inspected), not a request to inspect — never queue
  // it for circling.
  if (msg->status == explo_planner_msgs::msg::TreeTarget::STATUS_DONE) {
    RCLCPP_DEBUG(get_logger(),
        "Tree target id=%u arrived STATUS_DONE — not queued.", msg->target_id);
    return;
  }

  // Targets are consumed in map_frame_ without reframing (same convention as
  // the fused map). Warn once on a frame mismatch so a misplaced target is
  // diagnosable rather than silently circled at the wrong spot.
  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    RCLCPP_WARN_ONCE(get_logger(),
        "Tree target frame_id '%s' != planner map_frame '%s' — target is used "
        "without reframing and would be misplaced.",
        msg->header.frame_id.c_str(), map_frame_.c_str());
  }
  Eigen::Vector3f center(static_cast<float>(msg->center.x),
                         static_cast<float>(msg->center.y),
                         static_cast<float>(msg->center.z));
  const bool added = target_queue_.ingest(
      msg->target_id, center, msg->radius, msg->height,
      static_cast<float>(target_dedup_radius_m_));
  if (added) {
    RCLCPP_INFO(get_logger(),
        "Ingested tree target id=%u at (%.2f, %.2f) r=%.2f by '%s' "
        "(%zu pending).",
        msg->target_id, center.x(), center.y(), msg->radius,
        msg->discovered_by.c_str(), target_queue_.pendingCount());
  } else {
    RCLCPP_DEBUG(get_logger(),
        "Duplicate tree target id=%u ignored.", msg->target_id);
  }
}

void ExploPlannerNode::onPeerExploitIntent(
    const explo_planner_msgs::msg::RobotIntent& msg) {
  if (!coord_enabled_ || !exploitation_enabled_) return;
  if (!msg.exploit || msg.dwelled_mask == 0u) return;
  if (msg.robot_id == robot_name_) return;  // echo of our own broadcast

  // Find the peer's target in the local queue. Both robots ingest the same
  // global targets topic, so ids agree; if the TreeTarget hasn't arrived here
  // yet (ordering race) the merge silently no-ops and the per-tick merge in
  // doExploitPlan catches up while the peer's claim is still alive.
  const Target* local = nullptr;
  for (const auto& t : target_queue_.targets()) {
    if (t.id == msg.target_id) { local = &t; break; }
  }
  if (!local || local->status == Target::Status::DONE) return;

  // Ring indices are only meaningful on the identical ring, so regenerate it
  // from the locally stored (center, radius) — same inputs on every robot.
  const auto vantages =
      vantage_planner_->generateVantages(local->center, local->radius);
  std::vector<Eigen::Vector3f> ring;
  ring.reserve(vantages.size());
  for (const auto& v : vantages) ring.push_back(v.position);

  if (target_queue_.mergePeerDwells(msg.target_id, msg.dwelled_mask, ring)) {
    RCLCPP_INFO(get_logger(),
        "Merged peer '%s' dwell credit on target %u (mask=0x%x): team "
        "clear-LoS dwells now %d/%d.",
        msg.robot_id.c_str(), msg.target_id, msg.dwelled_mask,
        local->clear_los_dwells, min_vantages_required_);
  }
}

bool ExploPlannerNode::inRoi(const Eigen::Vector3f& pos) const {
  return pos.x() >= roi_min_x_ && pos.x() <= roi_max_x_ &&
         pos.y() >= roi_min_y_ && pos.y() <= roi_max_y_;
}

// Close the active target (success or partial) and route back to the queue or
// to exploration. Shared by both DONE paths in doExploitPlan.
void ExploPlannerNode::finishActiveTarget(bool success) {
  const Target* t = target_queue_.active();
  const uint32_t id = t ? t->id : 0u;
  const int clear = t ? t->clear_los_dwells : 0;
  target_queue_.markActiveDone();
  have_active_intent_ = false;     // release the claim now the target is closed
  exploit_target_timing_ = false;  // next active target re-latches the timer
  current_is_approach_   = false;
  RCLCPP_INFO(get_logger(),
      "Target %u exploitation %s (%d/%d clear-LoS vantages dwelled).",
      id, success ? "COMPLETE" : "PARTIAL", clear, min_vantages_required_);

  if (target_queue_.hasPending()) {
    transitionTo(State::EXPLOIT_PLAN);  // doExploitPlan activates the next one
  } else {
    phase_ = Phase::EXPLORE;
    RCLCPP_INFO(get_logger(),
        "Target queue empty -> reverting to EXPLORE.");
    transitionTo(State::PLAN);
  }
}

// March from just outside the trunk outward along the line toward the robot and
// return the point closest to the trunk that is in-ROI, a free planning_map cell,
// reachable from the robot via the (already-flooded) cost grid, and not
// blacklisted. This is the nearest spot we can actually drive to that makes
// progress toward a target whose vantage ring is still unmapped/unreachable;
// driving there maps the surroundings so a vantage can pass on a later tick.
// Returns false if no such point exists meaningfully nearer the trunk than the
// robot already is. Caller must have flooded cost_grid_ from robot_pos.
bool ExploPlannerNode::computeApproachGoal(const Eigen::Vector3f& center,
                                           float radius,
                                           const Eigen::Vector3f& robot_pos,
                                           Eigen::Vector3f& out) const {
  const float dx = robot_pos.x() - center.x();
  const float dy = robot_pos.y() - center.y();
  const float robot_dist = std::sqrt(dx * dx + dy * dy);
  if (robot_dist < 1e-3f) return false;  // robot sits on the trunk axis
  const float ux = dx / robot_dist;
  const float uy = dy / robot_dist;

  // Step at the cost-grid resolution (fall back if the grid isn't sized yet).
  float step = cost_grid_->resolution();
  if (!(step > 0.0f)) step = 0.25f;

  // Never place the approach goal inside the trunk: start at the vantage
  // standoff for this radius. Require a real move (so we don't re-issue a goal
  // we're effectively already at) by stopping short of the robot.
  const float start_d = vantage_planner_->standoffFor(radius);
  const float min_progress =
      std::max(2.0f * static_cast<float>(goal_xy_tol_), step);
  for (float d = start_d; d <= robot_dist - min_progress; d += step) {
    const float px = center.x() + ux * d;
    const float py = center.y() + uy * d;
    // An approach waypoint's z is inert downstream (see exploitZAt), so an
    // unmapped column falls back to the robot's own altitude rather than
    // dropping the point — the whole purpose of the march is to go map ground
    // we have no height for yet.
    float pz = exploitZAt(px, py);
    if (!std::isfinite(pz)) pz = robot_pos.z();
    Eigen::Vector3f p(px, py, pz);
    if (!inRoi(p)) continue;
    if (latest_plan_map_ && !isCellFree(p)) continue;
    // Reachability only when a planning_map/cost grid exists; with no map,
    // accept the straight-line point (same fallback as the vantage loop).
    if (latest_plan_map_ && !cost_grid_->reachable(p)) continue;
    if (failed_goals_.isNear(p, failed_goal_radius_m_)) continue;
    out = p;
    return true;  // closest-to-trunk reachable point on the ray
  }
  return false;
}

// Standing / sightline height for an exploitation point at (x, y).
//
// Flat mode: the fixed absolute vantage height, as before.
//
// Terrain mode: the ingest band is robot-relative ([z_robot + roi_min_z,
// z_robot + roi_max_z]), so an absolute vantage height leaves the band entirely
// once the robot is on ground far from z = 0 — over this AO's ~14 m of relief
// that is most of it. generateVantages() puts the vantage at that height and
// lineOfSightClear() marches its ray at the vantage z, so the occlusion test
// would find no voxels at all and pass every angle trivially: vantages accepted
// with no visibility check, and `vantage_los_clear` logged as a meaningless 1.
// Snapping to local ground + the candidate clearance keeps the sightline in the
// observed volume, the same way exploration candidates are placed.
//
// Returns NaN in terrain mode when no ground is found under (x, y), and the
// CALLER decides — because the two consumers need opposite things and the old
// shared "fall back to the robot's own z" was wrong for one of them:
//
//   - A vantage's z IS the measurement. lineOfSightClear() marches its ray at
//     exactly this height, so standing the ray at the robot's altitude over a
//     column whose ground is unknown re-creates the very bug this function was
//     added to fix, one level up: the ray leaves the observed volume, finds no
//     occluders, and the angle passes with a meaningless vantage_los_clear=1.
//     The vantage must be REJECTED instead (rej_noground), and the target left
//     open — its own give-up timer closes it PARTIAL, which is the honest
//     outcome for a trunk we could never see.
//   - An approach waypoint's z is inert: it feeds inRoi (XY-only), isCellFree
//     and reachable (both 2D), the failed-goal disc (XY) and then a nav2 goal,
//     which is a 2D navigator. So the robot's own z is a fine stand-in there,
//     and rejecting would be actively harmful — the approach march exists to
//     go MAP the unmapped ground, so refusing to drive anywhere the ground is
//     unmapped deadlocks exactly the case it is for.
float ExploPlannerNode::exploitZAt(float x, float y) const {
  const float flat_z = vantage_planner_->config().robot_z;
  if (!terrain_relative_z_ || !map_cache_) return flat_z;
  const auto& c = candidate_gen_->config();
  const float z_ref = have_pose_ ? latest_pos_.z() : 0.0f;
  const float gz = map_cache_->groundZAt(
      x, y, z_ref - c.ground_search_below, z_ref + c.ground_search_above,
      c.occ_thresh, c.ground_stack_max_m);
  if (!std::isfinite(gz)) return std::numeric_limits<float>::quiet_NaN();
  // Same band clamp exploration candidates get (CandidateConfig::roi_min_z):
  // the LoS ray has to march inside the ingested volume to mean anything.
  const float z = gz + c.z_clearance;
  if (!(eff_roi_max_z_ >= eff_roi_min_z_)) return z;
  return std::clamp(z, eff_roi_min_z_, eff_roi_max_z_);
}

void ExploPlannerNode::startExploitNavigate(const Eigen::Vector3f& robot_pos) {
  publishGoal(current_goal_);

  // Publish the goal as an exploit intent: peers on the same trunk consult it
  // in their vantage loop (MinPos, per-vantage radius) to take a different
  // angle, and the dwelled_mask carries this robot's clear-LoS credit for the
  // team quota. claim_radius_m ships the per-vantage disc so the claim's
  // stored radius matches its exploit semantics.
  if (intent_pub_ && coord_) {
    const Target* t = target_queue_.active();
    current_intent_msg_ = coord_->buildIntent(
        current_goal_, robot_pos, this->now(),
        static_cast<float>(coord_claim_ttl_sec_),
        static_cast<float>(coord_vantage_claim_radius_m_),
        /*planner_type_id (eig)=*/0u, map_frame_,
        /*exploit=*/true,
        /*target_id=*/t ? t->id : 0u,
        /*dwelled_mask=*/t ? t->clear_mask : 0u);
    intent_pub_->publish(current_intent_msg_);
    have_active_intent_ = true;
  }

  transitionTo(State::NAVIGATE);

  // Initialise the smart-timeout state for this NAVIGATE cycle (same as doPlan).
  const float dx = current_goal_.position.x() - robot_pos.x();
  const float dy = current_goal_.position.y() - robot_pos.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  nav_budget_sec_ = navBudgetSec(dist, nav_speed_est_mps_, nav_safety_factor_,
                                 nav_min_timeout_sec_, nav_max_timeout_sec_);
  progress_check_time_ = state_enter_time_;
  progress_check_dist_ = cumulative_distance_;
}

void ExploPlannerNode::doExploitPlan() {
  auto plan_start = this->now();

  // NOTE the ordering here: target bookkeeping, the team-quota merge and the
  // per-target timeout all run BEFORE the fused-map / planning_map waits
  // further down. None of them need a map, and putting them first means a map
  // that never arrives cannot pin the planner in EXPLOIT_PLAN — a peer
  // completing the ring or the wall-clock timeout still closes the target and
  // reverts to EXPLORE.

  // Ensure we have an active target (the doPlan / mid-hop hooks activate one;
  // after a target finishes we promote the next here).
  Target* tgt = target_queue_.active();
  if (!tgt) {
    tgt = target_queue_.activate();
    if (!tgt) {  // queue drained
      phase_ = Phase::EXPLORE;
      transitionTo(State::PLAN);
      return;
    }
  }

  // (Re)start the per-target give-up timer whenever a new target becomes
  // active, so the timeout below measures time spent on *this* trunk only.
  // The !timing_ disjunct also catches a rendezvous stand-down re-activating
  // the SAME id (startReturnToAnchor clears timing_): the return drive and
  // barrier wait must not count against the target. Progress events re-arm
  // the timer elsewhere: approach arrival (doNavigate) and completed dwell
  // (doExploitDwell); proximity holds refund it (doProximityHold release).
  if (!exploit_target_timing_ || exploit_target_started_id_ != tgt->id) {
    exploit_target_started_id_  = tgt->id;
    exploit_target_started_sec_ = this->now().seconds();
    exploit_target_timing_      = true;
  }

  // Generate the fixed angular vantage set around the trunk. Deterministic
  // from (center, radius), so ring index k names the same pose on every
  // robot — generated before the quota check because the peer-credit merge
  // below needs the canonical ring positions.
  auto vantages = vantage_planner_->generateVantages(tgt->center, tgt->radius);
  // Terrain mode: lift each vantage (and therefore its LoS ray, which marches
  // at the vantage z) onto the local ground — see exploitZAt(). Ring identity
  // across robots is unaffected: every ring/visited comparison in TargetQueue
  // is XY-only (dist2xy), so only the angular index has to agree.
  if (terrain_relative_z_) {
    for (auto& v : vantages)
      v.position.z() = exploitZAt(v.position.x(), v.position.y());
  }

  // Team quota: fold peers' clear-LoS dwell masks (carried on their exploit
  // intents) into this target before checking success. The event-driven merge
  // in onPeerExploitIntent normally gets there first; this per-tick pass
  // catches the ordering race where a peer's intent arrived before the
  // TreeTarget was ingested locally.
  if (coord_ && coord_->enabled()) {
    coord_->prune(plan_start);
    const uint32_t peer_mask = coord_->peerDwellUnion(tgt->id);
    if (peer_mask != 0u) {
      std::vector<Eigen::Vector3f> ring;
      ring.reserve(vantages.size());
      for (const auto& v : vantages) ring.push_back(v.position);
      if (target_queue_.mergePeerDwells(tgt->id, peer_mask, ring)) {
        RCLCPP_INFO(get_logger(),
            "Target %u: merged peer dwell credit (mask=0x%x) -> team "
            "clear-LoS dwells %d/%d.",
            tgt->id, peer_mask, tgt->clear_los_dwells,
            min_vantages_required_);
      }
    }
  }

  // Already enough clear-LoS vantages dwelled (team union when coordination
  // is on) -> success for the whole trunk.
  if (tgt->clear_los_dwells >= min_vantages_required_) {
    finishActiveTarget(/*success=*/true);
    return;
  }

  // Hard per-target wall-clock bound. This sits ABOVE all vantage-selection
  // logic AND above the map waits below on purpose:
  //   * a target can keep producing a "selectable" vantage every tick (e.g.
  //     the blacklist TTL re-offers vantages faster than the nav budget can
  //     exhaust the ring) and spin here forever, never reaching the give-up
  //     path further down;
  //   * a map that never arrives (field runs publish NO planning_map) would
  //     otherwise pin the planner in the early returns below with no escape.
  // Bounding it here makes exploitation of one trunk provably terminate, so
  // the phase always reverts to EXPLORE.
  if (exploit_target_timeout_sec_ > 0.0) {
    const double waited = this->now().seconds() - exploit_target_started_sec_;
    if (waited >= exploit_target_timeout_sec_) {
      RCLCPP_WARN(get_logger(),
          "Target %u: exploitation timeout after %.0fs (limit %.0fs) -> PARTIAL.",
          tgt->id, waited, exploit_target_timeout_sec_);
      finishActiveTarget(/*success=*/false);
      return;
    }
  }

  // The LoS ray-march reads the fused grid; refresh it (cheap when unchanged).
  if (!loadLatestMap()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "EXPLOIT_PLAN: no fused map yet; retrying next tick.");
    return;
  }

  // Vantage validation (free-cell + reachability) and the approach fallback use
  // the 2D planning_map when it is enabled. With use_planning_map=false there is
  // never a map: vantages are then validated on straight-line geometry (no
  // obstacle/reachability check — avoidance is delegated to the navigator,
  // mirroring exploration's map-less fallback), so we do NOT wait here.
  // When the map IS enabled but has not arrived yet, wait rather than drive
  // blind toward the trunk. The wait is BOUNDED: the per-target timeout above
  // keeps ticking, so a mis-wired planning_map times the target out PARTIAL
  // instead of hanging in EXPLOIT_PLAN forever.
  if (use_planning_map_ && !latest_plan_map_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "EXPLOIT_PLAN: no planning_map yet; waiting before selecting vantages "
        "(gives up PARTIAL at exploit_target_timeout_sec=%.0fs).",
        exploit_target_timeout_sec_);
    return;
  }

  // Age out stale blacklist entries (same TTL as exploration) so a vantage that
  // failed earlier can be retried once its entry expires.
  failed_goals_.prune(plan_start.seconds(), failed_goal_ttl_sec_);

  const auto robot_pos = latest_pos_;

  // Build the cost grid for reachability + nearest-vantage ordering, but only
  // when a planning_map is available. Unlike exploration (a bounded local
  // flood), exploitation may have to drive across the map to reach the trunk,
  // so the flood is UNBOUNDED (cap <= 0) — a target released while the robot is
  // far away must still resolve as reachable as long as a known-free path
  // exists. ~5 ms once per vantage, infrequent. With use_planning_map=false
  // there is no obstacle layer to flood: have_cost stays false and every
  // vantage is ordered by straight-line distance with no reachability filter.
  //
  // Cached on (map identity, source pose). "once per vantage" only held while
  // a vantage was selectable: when none is, EXPLOIT_PLAN does NOT transition
  // and is therefore re-entered at the full 10 Hz tick rate until the target
  // times out — up to ~1200 whole-grid floods per target, all identical, on
  // the single-threaded executor that also has to service the map and TF
  // callbacks. Rebuild only when the latched map object or the robot pose
  // actually changed.
  bool have_cost = false;
  if (latest_plan_map_) {
    constexpr float kFloodRefreshM = 0.5f;
    const bool stale =
        !exploit_flood_valid_ ||
        exploit_flood_map_ != latest_plan_map_.get() ||
        (robot_pos - exploit_flood_pos_).head<2>().norm() > kFloodRefreshM;
    if (stale) {
      cost_grid_->build(*latest_plan_map_);
      cost_grid_->floodFrom(robot_pos, /*radius_cap_m (unbounded)=*/0.0f);
      exploit_flood_valid_   = true;
      exploit_flood_map_     = latest_plan_map_.get();
      exploit_flood_pos_     = robot_pos;
      exploit_flood_reached_ = cost_grid_->reachedCellCount();
    }
    constexpr size_t kMinReachedForFilter = 10;
    have_cost = exploit_flood_reached_ >= kMinReachedForFilter;
  }

  // Validate every vantage; pick the nearest (by path cost) that is valid,
  // unvisited and not blacklisted. Validation = ROI + free-cell + reachable +
  // LoS-clear (the first three reuse the exploration filters).
  int   best_idx  = -1;
  float best_cost = std::numeric_limits<float>::infinity();
  int   n_valid   = 0;
  int rej_roi = 0, rej_map = 0, rej_unreach = 0, rej_los = 0,
      rej_visited = 0, rej_blk = 0, rej_minpos = 0, rej_noground = 0;

  for (size_t i = 0; i < vantages.size(); ++i) {
    const auto& v = vantages[i];
    // Terrain mode with no ground under this angle: exploitZAt returned NaN and
    // there is no honest height to march the LoS ray at. Reject rather than
    // guess — see exploitZAt. Counted separately from rej_roi because in the
    // field the two mean completely different things ("outside the AO box" vs
    // "this column is not mapped yet"), and inRoi is XY-only so it would not
    // catch a NaN z anyway.
    if (!std::isfinite(v.position.z())) { ++rej_noground; continue; }
    if (!inRoi(v.position)) { ++rej_roi; continue; }
    // Free-cell check only when a planning_map is present; without one
    // isCellFree() is conservatively false and would reject every vantage.
    if (latest_plan_map_ && !isCellFree(v.position)) { ++rej_map; continue; }

    float cost;
    if (have_cost) {
      if (!cost_grid_->reachable(v.position)) { ++rej_unreach; continue; }
      cost = cost_grid_->costTo(v.position);
    } else {
      // Straight-line fallback when no usable cost grid this tick.
      const float dx = v.position.x() - robot_pos.x();
      const float dy = v.position.y() - robot_pos.y();
      cost = std::sqrt(dx * dx + dy * dy);
    }

    if (!vantage_planner_->lineOfSightClear(v.position, tgt->center,
                                            tgt->radius, *map_cache_)) {
      ++rej_los;
      continue;
    }
    ++n_valid;  // a valid (reachable + clear-LoS + in-ROI + free) vantage

    if (target_queue_.isVantageVisited(
            v.position, static_cast<float>(vantage_visited_tol_m_))) {
      ++rej_visited;
      continue;
    }
    // Scope the failed-goal match to ~one vantage (vantage_visited_tol_m_), not
    // the exploration-scale failed_goal_radius_m_, so blacklisting one occluded
    // angle doesn't also veto the other vantages circling the same small trunk.
    if (failed_goals_.isNear(v.position,
                             static_cast<double>(vantage_visited_tol_m_))) {
      ++rej_blk;
      continue;
    }
    // MinPos per-vantage deconfliction: yield an angle a peer currently
    // claims (in-flight or mid-dwell) when the peer is closer. Uses the small
    // per-vantage disc, NOT the exploration-scale coord_claim_radius_m — one
    // fov-range disc would swallow the whole ring and veto the tree outright
    // instead of splitting the angles across the team.
    if (coord_ && coord_->enabled()) {
      const auto* peer = coord_->claimMatching(
          v.position, static_cast<float>(coord_vantage_claim_radius_m_));
      if (peer && !coord_->selfWinsAgainst(robot_pos, v.position, *peer,
                                           robot_name_)) {
        ++rej_minpos;
        continue;
      }
    }
    if (cost < best_cost) {
      best_cost = cost;
      best_idx  = static_cast<int>(i);
    }
  }

  if (best_idx < 0) {
    RCLCPP_INFO(get_logger(),
        "Target %u: no selectable vantage (valid=%d noground=%d roi=%d map=%d "
        "unreach=%d los=%d visited=%d blk=%d minpos=%d).",
        tgt->id, n_valid, rej_noground, rej_roi, rej_map, rej_unreach, rej_los,
        rej_visited, rej_blk, rej_minpos);

    // Quota-met and the per-target timeout are both handled unconditionally at
    // the top of this function, so neither can apply here. Drive *toward* the
    // trunk instead: head for the nearest reachable point
    // on the line to it so the area maps en route and a vantage can pass on a
    // later tick. Re-planning (not dwelling) resumes on arrival.
    Eigen::Vector3f approach;
    // Approach when we have a usable cost grid, OR when the planning_map is
    // disabled (no grid to have — computeApproachGoal then uses straight-line
    // geometry). The remaining case (map enabled but flood reached too little,
    // robot trapped in inflation) still skips the approach as before.
    if ((have_cost || !latest_plan_map_) &&
        computeApproachGoal(tgt->center, tgt->radius, robot_pos, approach)) {
      current_is_approach_   = true;
      current_vantage_index_ = -1;
      // Approach hops carry no verified sightline; keep the staged verdict
      // honest in case a future path ever dwells off an approach goal.
      current_vantage_los_clear_ = false;
      current_goal_ = CandidateViewpoint{};
      current_goal_.position = approach;
      current_goal_.yaw = std::atan2(tgt->center.y() - approach.y(),
                                     tgt->center.x() - approach.x());

      pending_exploit_target_id_     = static_cast<int>(tgt->id);
      pending_exploit_vantage_index_ = -1;  // an approach hop, not a vantage
      pending_exploit_n_valid_       = n_valid;
      pending_exploit_los_clear_     = 0;
      pending_exploit_dwell_sec_     = 0.0f;
      pending_plan_ms_ = static_cast<float>(
          (this->now() - plan_start).nanoseconds() * 1e-6);
      pending_mean_info_gain_          = 0.0f;
      pending_mean_path_cost_          = 0.0f;
      pending_selected_info_gain_      = 0.0f;
      pending_selected_path_cost_      = 0.0f;
      pending_rejected_by_minpos_      = rej_minpos;
      pending_rejected_by_unreachable_ = rej_unreach;

      RCLCPP_INFO(get_logger(),
          "Target %u: no vantage reachable yet -> approaching trunk via "
          "(%.2f, %.2f) to map the area.",
          tgt->id, approach.x(), approach.y());
      startExploitNavigate(robot_pos);
      return;
    }

    // No vantage and no closer reachable approach point this tick. Do NOT
    // abandon the target — the map may still grow (other robots, late sensor
    // returns). Hold in EXPLOIT_PLAN and retry; the give-up timer above is what
    // eventually closes a genuinely unreachable target PARTIAL.
    const double waited = this->now().seconds() - exploit_target_started_sec_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Target %u: no vantage and no reachable approach yet (waited %.0fs / "
        "%.0fs) — retrying.",
        tgt->id, waited, exploit_target_timeout_sec_);
    return;
  }

  current_goal_ = vantages[best_idx];
  current_vantage_index_ = best_idx;
  current_is_approach_   = false;  // a real vantage to dwell at
  // Stage the selection-time LoS verdict (this vantage just passed
  // lineOfSightClear above). It is what the dwell's map-unavailable fallback
  // reads — before this was staged here, that fallback silently reused
  // whatever verdict the PREVIOUS dwell left behind, possibly from another
  // vantage or another target, and could credit an unverified capture toward
  // the team quota.
  current_vantage_los_clear_ = true;

  // Stage the exploit diagnostics for the upcoming LOG_STEP. los_clear is staged
  // 0 here and only set to 1 when the dwell completes and re-confirms LoS from
  // the actual final pose — so a navigation failure logs an honest los_clear=0
  // / dwell_sec=0 rather than claiming a clear dwell that never happened.
  pending_exploit_target_id_     = static_cast<int>(tgt->id);
  pending_exploit_vantage_index_ = best_idx;
  pending_exploit_n_valid_       = n_valid;
  pending_exploit_los_clear_     = 0;
  pending_exploit_dwell_sec_     = 0.0f;
  pending_plan_ms_ = static_cast<float>(
      (this->now() - plan_start).nanoseconds() * 1e-6);

  // Repurpose the shared utility/coord diagnostics for the exploit row so they
  // don't carry stale exploration values: path-cost columns hold the travel
  // cost to the chosen vantage; info-gain is not meaningful here.
  pending_mean_info_gain_          = 0.0f;
  pending_mean_path_cost_          = best_cost;
  pending_selected_info_gain_      = 0.0f;
  pending_selected_path_cost_      = best_cost;
  pending_rejected_by_minpos_      = rej_minpos;
  pending_rejected_by_unreachable_ = rej_unreach;

  RCLCPP_INFO(get_logger(),
      "Target %u: vantage %d/%zu selected at (%.2f, %.2f) yaw=%.2f "
      "cost=%.2f (%d valid, %d clear-LoS dwelled).",
      tgt->id, best_idx, vantages.size(), current_goal_.position.x(),
      current_goal_.position.y(), current_goal_.yaw, best_cost, n_valid,
      tgt->clear_los_dwells);

  publishCandidateViz(vantages);
  startExploitNavigate(robot_pos);
}

void ExploPlannerNode::doExploitDwell() {
  const double elapsed = (this->now() - state_enter_time_).seconds();

  // Clock regression (a bag loop / sim reset rewinds sim time): elapsed goes
  // negative and the dwell would never complete. Re-anchor the dwell start to
  // now and retry next tick rather than hang.
  if (elapsed < 0.0) {
    RCLCPP_WARN(get_logger(),
        "EXPLOIT_DWELL: clock went backwards (elapsed=%.2fs) — re-anchoring "
        "dwell start.", elapsed);
    state_enter_time_ = this->now();
    return;
  }

  // NO goal re-send during the dwell. The dwell is only ever entered from
  // NAVIGATE *after* arrival, so nav2 has already reported the goal reached and
  // the controller has stopped — there is nothing to keep alive, and every
  // re-send is a fresh NavigateToPose that preempts nothing and re-drives a
  // goal the robot is standing on.
  //
  // Throttling it was not enough. goal_republish_sec 5.0 against
  // exploit_dwell_sec 8.0 puts exactly one re-navigation at t~5 s of every
  // capture — the midpoint — and the widened goal_yaw_tolerance (0.4, needed so
  // the planner's arrival gate stays looser than nav2's 0.25 checker) means the
  // pose nav2 stopped at can be up to 0.4 rad off the vantage yaw, so the
  // re-send is a real rotation, not a no-op. controller_server::computeControl()
  // also calls computeAndPublishVelocity() BEFORE isGoalReached(), so even an
  // exactly-satisfied goal emits at least one velocity command. That is a
  // rotation through the middle of the RGB-D/LiDAR capture the dwell exists to
  // take, i.e. motion blur and a viewpoint shift in the one window where the
  // platform is supposed to be still.
  if (elapsed < exploit_dwell_sec_) return;

  // Dwell complete. Re-confirm line-of-sight from the pose we actually settled
  // at (the controller may have stopped slightly off the planned vantage, and
  // the map has grown during the dwell) — that is the honest "was this a
  // clear-LoS capture?" answer, and it is what counts toward the success quota
  // and what the CSV logs. Falls back to the selection-time verdict — staged in
  // doExploitPlan when THIS vantage was chosen — if the map or active target is
  // momentarily unavailable.
  bool los_clear = current_vantage_los_clear_;
  Target* t = target_queue_.active();
  if (t && loadLatestMap()) {
    los_clear = vantage_planner_->lineOfSightClear(
        latest_pos_, t->center, t->radius, *map_cache_);
  }
  current_vantage_los_clear_ = los_clear;
  pending_exploit_los_clear_ = los_clear ? 1 : 0;

  // Record it on the active target and stage the dwell time for LOG_STEP.
  // The ring index makes the credit shareable: peers merge it by index into
  // their own copy of this target (team quota).
  target_queue_.recordVantageDwell(current_goal_.position, los_clear,
                                   current_vantage_index_);
  pending_exploit_dwell_sec_ = static_cast<float>(elapsed);

  // Broadcast the updated team-credit mask immediately (don't wait for the
  // next selection or heartbeat) so a peer picking its next angle right now
  // already sees this dwell — and so the credit lands before this robot
  // could release the claim on target completion.
  if (intent_pub_ && coord_ && have_active_intent_ && t) {
    current_intent_msg_.dwelled_mask = t->clear_mask;
    current_intent_msg_.header.stamp = this->now();
    intent_pub_->publish(current_intent_msg_);
  }

  // Reaching + dwelling a vantage is progress: reset the per-target give-up
  // timer so the timeout only fires on a genuine no-progress stall.
  exploit_target_started_sec_ = this->now().seconds();

  RCLCPP_INFO(get_logger(),
      "Dwelled %.1fs at vantage %d of target %d (LoS %s; clear-LoS dwells now "
      "%d/%d).",
      elapsed, current_vantage_index_, pending_exploit_target_id_,
      los_clear ? "clear" : "BLOCKED",
      t ? t->clear_los_dwells : 0, min_vantages_required_);
  transitionTo(State::LOG_STEP);
}

// ==================================================================
// Helpers
// ==================================================================

// Look up base_frame -> map_frame via TF and cache the robot pose. Replaces
// the odom subscription so the planner reads the same source-of-truth that
// the rest of the nav stack uses, and so candidates/goals share the same
// frame as the dscovox map.
void ExploPlannerNode::updatePoseFromTF() {
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

void ExploPlannerNode::trackDistance() {
  if (!have_pose_) return;
  if (!first_pos_) {
    float step = (latest_pos_ - prev_pos_).norm();
    // Reject implausible single-tick pose jumps. Localization relocalization
    // (NDT / EKF corrections) teleports the map->base_link TF by metres in one
    // tick; at the 10 Hz tick this guard can't reject real motion (even a 1 m/s
    // robot moves 0.1 m/tick), so anything above max_pose_jump_m_ is a
    // discontinuity, not travel. Counting it would inflate distance_traveled (a
    // headline metric) AND let a stationary-but-relocalizing robot satisfy the
    // no-progress watchdog. prev_pos_ is still advanced so the next tick
    // measures from the corrected pose. 0 disables the guard.
    if (max_pose_jump_m_ > 0.0f && step > max_pose_jump_m_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Pose jump of %.2f m in one tick exceeds max_pose_jump_m=%.2f — "
          "treating as a localization discontinuity, not travel.",
          step, max_pose_jump_m_);
    } else {
      cumulative_distance_ += step;
    }
  }
  prev_pos_ = latest_pos_;
  first_pos_ = false;
}

// Yaw-only (Z-axis) quaternion message. Shared by goal + candidate viz
// publishing so the yaw->quat conversion lives in one place.
geometry_msgs::msg::Quaternion ExploPlannerNode::yawToQuat(float yaw) {
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.z = std::sin(yaw * 0.5);
  return q;
}

void ExploPlannerNode::publishGoal(const CandidateViewpoint& vp) {
  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = this->now();
  goal.header.frame_id = map_frame_;
  goal.pose.position.x = vp.position.x();
  goal.pose.position.y = vp.position.y();
  // Nav2's 2D planners consume only (x, y, yaw) and ignore z, so the true
  // 3D waypoint z rides along by default (terrain mode makes it the ground
  // + clearance elevation). flatten_goal_z zeroes it for consumers that
  // choke on a non-zero z; markers/logs keep the 3D value either way.
  goal.pose.position.z = flatten_goal_z_ ? 0.0 : vp.position.z();
  goal.pose.orientation = yawToQuat(vp.yaw);
  // No navigator listening: the goal goes nowhere and, with no action
  // feedback, the only symptom is every goal failing on the nav budget /
  // no-progress watchdog. Say so instead of letting it look like a slow robot.
  if (goal_pub_->get_subscription_count() == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No subscriber on the goal topic — nothing is navigating. Check the "
        "navigator is up and its goal_pose topic matches goal_topic.");
  }
  goal_pub_->publish(goal);
  last_goal_pub_pos_  = vp.position;
  last_goal_pub_yaw_  = vp.yaw;
  last_goal_pub_time_ = goal.header.stamp;
  have_last_goal_pub_ = true;
}

// Keep-alive re-send used by the states that are still DRIVING toward a goal
// across ticks (NAVIGATE, RETURN_NAV). Publishes when the pose changed, when a
// subscriber has just appeared, or when goal_republish_sec_ has elapsed — never
// at the tick rate. See goal_republish_sec_ for why the old unthrottled 10 Hz
// re-send actively provoked nav2 aborts.
//
// Deliberately NOT called from EXPLOIT_DWELL: that state is entered only after
// nav2 has reported arrival, so there is no in-flight goal to keep alive and a
// re-send just re-drives a satisfied goal through the capture window. See
// doExploitDwell().
void ExploPlannerNode::republishGoal(const CandidateViewpoint& vp) {
  const bool has_sub = goal_pub_->get_subscription_count() > 0;
  const bool sub_appeared = has_sub && !goal_had_subscriber_;
  goal_had_subscriber_ = has_sub;

  if (!have_last_goal_pub_ || sub_appeared) {
    publishGoal(vp);
    return;
  }
  // Pose change is compared against what was last put on the wire, so a goal
  // edited in place (e.g. a re-planned vantage on the same ring) still goes
  // out immediately.
  const bool moved =
      (vp.position - last_goal_pub_pos_).squaredNorm() > 1e-6f ||
      std::abs(std::remainder(vp.yaw - last_goal_pub_yaw_,
                              2.0f * static_cast<float>(M_PI))) > 1e-3f;
  if (moved) {
    publishGoal(vp);
    return;
  }
  if (goal_republish_sec_ <= 0.0) return;  // publish-on-change only
  if ((this->now() - last_goal_pub_time_).seconds() >= goal_republish_sec_)
    publishGoal(vp);
}

void ExploPlannerNode::publishCandidateViz(
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

// ==================================================================
// Entry point
// ==================================================================

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Single-threaded executor: the planner ingests the fused map over a topic
  // subscription (non-blocking), so there is no blocking service future to
  // service on a second thread. All callbacks and timers run on one thread,
  // which also makes the map-callback / state-machine interaction race-free.
  rclcpp::spin(std::make_shared<explo_planner::ExploPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
