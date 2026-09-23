/// @file explo_planner_node.cpp
/// @brief Generation 34 of the EIG exploration planner node: the gen-33 node's
///        exploration machine, driven by the gen-34 team layer.
///
/// Carved from gen 33's node (DESIGN_gen34.md §8.1, Q60), which is kept,
/// unbuilt, in backup/gen33/ as the reference for the exploit port. Goal
/// selection, map ingest, the exploration drive (doNavigate, failGoal), the
/// coverage criterion, the step budget, metrics, the event log and the claim
/// stream are copied verbatim. Exploit, rendezvous, pursuit, the reconnect
/// trigger and dispatch, RETURN_NAV / RETURN_SYNC, the link gate, done-seek,
/// hold escalation, TeamWorld and TeamModel are gone.
///
/// Team logic lives in TeamCore (team_core.hpp), pure C++ with no ROS. Every
/// tick the node feeds it the pose, the finished latch and the map sequence
/// numbers; TeamCore picks the activity from its priority list (§8.4):
///
///   Explore      -> PLAN -> NAVIGATE -> INTEGRATE -> LOG_STEP (gen 33's)
///   Meet         -> MEET        Chase -> CHASE      Follow -> FOLLOW
///   Wait         -> WAIT        Home  -> RETURN_HOME
///   Done         -> DONE (the node stays up and keeps beaconing)
///
/// Every drive that is not an exploration goal is a Leg (leg.hpp). The
/// proximity guard covers every drive; two robots both in Meet do not hold
/// for each other (Q65). The TeamBeacon goes out once a second in every state.
///
/// Exploitation is phase 2 (Q66): exploitation_enabled=true is fatal here.
///
/// Comments ending "(notes: <id>)" refer to doc/explo_planner_node_notes.md,
/// written for the gen-33 node this code was copied from.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <map>
#include <numeric>
#include <utility>  // std::pair, std::move

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
#include <scovox_msgs/msg/scovox_fusion_counters.hpp>
#include <explo_planner_msgs/msg/robot_intent.hpp>
#include <explo_planner_msgs/msg/team_beacon.hpp>
#include <explo_planner_msgs/msg/cell_state.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // tf2::fromMsg used by tf2::getYaw

#include "explo_planner/map_cache.hpp"
#include "explo_planner/candidate_generator.hpp"
#include "explo_planner/fov_evaluator.hpp"
#include "explo_planner/scoring.hpp"
#include "explo_planner/metrics_logger.hpp"
#include "explo_planner/experiment_log.hpp"
#include "explo_planner/cost_grid.hpp"
#include "explo_planner/coordination.hpp"
#include "explo_planner/plan_map_query.hpp"
#include "explo_planner/planner_util.hpp"
#include "explo_planner/failed_goal_blacklist.hpp"
#include "explo_planner/fleet_identity.hpp"
#include "explo_planner/cell_world.hpp"
#include "explo_planner/separation.hpp"
#include "explo_planner/global_allocator.hpp"
#include "explo_planner/reconnect_gate.hpp"
#include "explo_planner/pursuit_predictor.hpp"
#include "explo_planner/rendezvous_scheduler.hpp"
#include "explo_planner/proximity_guard.hpp"
#include "explo_planner/team_core.hpp"
#include "explo_planner/leg.hpp"

// Generated into the build tree on every build; defines EXPLO_PLANNER_GIT_REV.
// Guarded because this translation unit must still compile in a tree that has
// not run the generator (an IDE index pass, a standalone syntax check) — the
// #ifdef at the stamping site already handles the revision being unavailable.
#if __has_include("explo_planner_git_rev.h")
#include "explo_planner_git_rev.h"
#endif

namespace explo_planner {

enum class State {
  WAIT_FOR_MAP,
  PLAN,
  NAVIGATE,
  INTEGRATE,
  LOG_STEP,
  DONE,
  // A driving robot that lost right-of-way to a nearby moving teammate brakes
  // and parks until the peer clears or parks, then resumes the same drive.
  // Entered from NAVIGATE and from every Leg state. (notes: state-proximity-hold-entry)
  PROXIMITY_HOLD,
  // The gen-34 team activities (DESIGN_gen34.md §8.4). Each is TeamCore's
  // activity of the same name; MEET, CHASE, FOLLOW and RETURN_HOME drive a
  // Leg, WAIT and DONE stand still.
  MEET,
  CHASE,
  FOLLOW,
  WAIT,
  RETURN_HOME
};

// Wire/CSV state names consumed by offline analysis and the run scripts: do
// not rename one. No default case, so a State without a name here is a compile
// warning. (notes: statename-wire-interface)
inline const char* stateName(State s) {
  switch (s) {
    case State::WAIT_FOR_MAP:   return "WAIT_FOR_MAP";
    case State::PLAN:           return "PLAN";
    case State::NAVIGATE:       return "NAVIGATE";
    case State::INTEGRATE:      return "INTEGRATE";
    case State::LOG_STEP:       return "LOG_STEP";
    case State::DONE:           return "DONE";
    case State::PROXIMITY_HOLD: return "PROXIMITY_HOLD";
    case State::MEET:           return "MEET";
    case State::CHASE:          return "CHASE";
    case State::FOLLOW:         return "FOLLOW";
    case State::WAIT:           return "WAIT";
    case State::RETURN_HOME:    return "RETURN_HOME";
  }
  return "UNKNOWN";
}

/// The Leg-driven states: the drive is TeamCore's, not an exploration goal.
inline bool isLegState(State s) {
  return s == State::MEET || s == State::CHASE || s == State::FOLLOW ||
         s == State::RETURN_HOME;
}

/// The team states, Leg-driven or standing.
inline bool isTeamState(State s) {
  return isLegState(s) || s == State::WAIT || s == State::DONE;
}

// Top-level behaviour mode. NAVIGATE / INTEGRATE / LOG_STEP are shared between
// modes and branch on this to route correctly.
enum class Phase { EXPLORE, EXPLOIT };

// Depth 1, reliable, transient_local for current-value topics; both ends must
// agree. Not for the tree-target pair (KeepLast(50) backlog replay) or the
// comms link-index subscription. (notes: latched-qos-contract)
inline rclcpp::QoS latchedQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

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
  /// Last chance to write run_end when rclcpp::shutdown() unwinds spin() and
  /// destroys the node. Never throws. (notes: dtor-closes-event-log)
  ~ExploPlannerNode() override;

private:
  // ----------------------------------------------------------------
  // Map ingest
  // ----------------------------------------------------------------
  // Cache the latest fused map received on the dscovox topic. The grid is
  // rebuilt from it (ROI-clipped) in loadLatestMap() at the start of each PLAN
  // tick, so a full rebuild doesn't run on every incoming message.
  void onScovoxMap(const scovox_msgs::msg::ScovoxMap::SharedPtr& msg);
  // Fold dscovox's per-source integration counters into peer_fusion_deltas_,
  // the only signal here that says which peer's voxels are arriving (the fused
  // map total cannot). (notes: on-fusion-counters-per-peer)
  void onFusionCounters(
      const scovox_msgs::msg::ScovoxFusionCounters::SharedPtr& msg);
  // Rebuild map_cache_ from the latest cached ScovoxMap, clipped to the ROI.
  // Returns false if no map has been received yet.
  bool loadLatestMap();

  // Trajectory-level scoring (path-integrated EIG; param-gated ablation).
  bool scoreTrajectory(std::vector<CandidateViewpoint>& candidates);

  // ----------------------------------------------------------------
  // State machine
  // ----------------------------------------------------------------
  void tick();
  /// reason is recorded verbatim in the state_change event. Pass a stable,
  /// machine-readable tag: it is an analysis interface like stateName().
  /// (notes: transition-reason-tags)
  void transitionTo(State s, const char* reason = "");
  void doPlan();
  void doNavigate();
  void doIntegrate();
  void doLogStep();

  // fillCommonMetrics fills the world-state columns shared by both emitters and
  // leaves plan-attribution columns at zero; doLogStep adds those on
  // end-of-step rows. metricsTick is the periodic sampler, run in every state.
  // (notes: csv-row-assembly-emitters)
  void fillCommonMetrics(StepMetrics& m);
  void metricsTick();

  // Experiment event log: the authoritative event stream on the node's sim
  // clock; the CSV stays the per-step record. expCtx() is the only place the
  // log's time axis is read. (notes: exp-log-sim-clock-envelope)
  ExperimentContext expCtx();
  /// Emit run_start on the first tick with a live clock — NOT in the
  /// constructor. Under use_sim_time this->now() reads 0 until the first
  /// /clock message, and a t0 of 0 would silently turn every t_rel_sec in the
  /// file into an absolute sim time. No-op once started.
  void startExperimentLog();
  /// Emit clock_anchor on a pure sim-time schedule. Never consult a wall clock
  /// to decide whether to fire. (notes: exp-clock-anchor-sim-schedule)
  void expClockAnchorTick();
  /// peer_seen/peer_lost bookkeeping: expPeerHeard runs from the intent
  /// callback, expPeerSweep (tick) marks a peer LOST past the claim TTL. Uses
  /// the intent stream, not Coordination, so it works with coordination off.
  /// (notes: exp-peer-seen-lost-bookkeeping)
  void expPeerHeard(const std::string& peer_id);
  void expPeerSweep();
  /// Emit run_end exactly once (later calls are ignored by the logger).
  void logRunEnd(const char* reason);

  /// Emit exploration_complete at the instant this robot declares its
  /// exploration exhausted, before anything is decided about what follows.
  /// Shared so the latch criterion records the same event.
  /// (notes: exploration-complete-event)
  void recordExplorationComplete(const char* reason);
  /// Latch exploration finished (K4) and record exploration_complete, once.
  /// Shared by both criteria and the step budget; TeamCore reads the latch and
  /// picks what follows.
  void latchFinished(const char* reason);
  /// Latch completion test (done_criterion == "latch") on a caller-measured ROI
  /// unknown fraction. True means the caller must stop this tick; every path
  /// that published a goal or changed state must return true.
  /// (notes: latch-coverage-done-return-contract)
  bool maybeLatchCoverageDone(double unk, const char* source);
  /// Records the unknown fraction a completion decision tested, for
  /// recordExplorationComplete to stamp. Every termination path calls it except
  /// doLogStep's step-budget ending, where fillCommonMetrics already wrote it.
  /// (notes: coverage-decision-sample-invariant)
  void noteCoverageDecisionSample(double unk, const char* source);
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
  /// Park the current goal in the failed-goal blacklist and leave NAVIGATE.
  /// `elapsed` is time since NAVIGATE entry (context on every row); the
  /// comparison that actually fired is passed separately as
  /// (test_name, test_value, test_threshold) because the three callers do not
  /// test the same quantity — see logNavGoalFailed's contract.
  void failGoal(const char* reason, double elapsed, const char* test_name,
                double test_value, double test_threshold);
  void heartbeatTick();

  // checkProximityHold runs each tick while a nav goal is in flight and enters
  // PROXIMITY_HOLD when a higher-priority teammate moves nearby (true if it
  // transitioned); doProximityHold resumes the same goal on release.
  // (notes: proximity-hold-helpers)
  bool checkProximityHold();
  void enterProximityHold(const ProximityGuard::Decision& d);
  void doProximityHold();
  /// Stop the platform when a transition abandons an in-flight drive without
  /// replacing it: cancel-all plus a brake goal at our own pose, without the
  /// hold bookkeeping. (notes: abandon-nav-goal-stop)
  void abandonNavGoal(const char* why);
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

  /// Re-census the coarse cell world from the current map and emit one
  /// `cell_census`, at most every `cell_census_period_s_` SIM seconds. Takes
  /// the ROI coverage measure of the calling tick so the two ride one event.
  /// No-op unless the cell world is enabled and configured.
  void updateCellWorld(double roi_unknown_fraction, const char* coverage_source);
  /// Draw the cell world as a flat colour-coded grid on its own topic.
  void publishCellViz();

  // --- Parameters ---
  std::string robot_name_;
  // Fleet identity: the ordered `team_robot_names` param resolved against
  // robot_name_. Unconfigured (the default) is legal and is exactly the
  // pre-M-TARE behaviour; every mechanism that needs numeric ids refuses to
  // start without it rather than degrading. See fleet_identity.hpp.
  std::vector<std::string> team_robot_names_;
  FleetIdentity            fleet_;
  // Coarse cell world, off by default (nothing configured, censused, emitted or
  // published). Observation-only: nothing reads cell_world_ to decide, so the
  // census is safe to sample from the metrics path.
  // (notes: cell-world-off-by-default)
  bool       cell_world_enable_ = false;
  CellWorld  cell_world_;

  // Soft team-separation discount on the exploration utility (separation.hpp).
  // OFF by default; `separation_weight = 0` is the pre-separation planner
  // bit-for-bit, because SeparationTerm::apply is not called at all when the
  // term is disabled and discount() would return exactly 1.0f if it were.
  SeparationTerm separation_;
  /// Peers this robot may be repelled by, rebuilt once per PLAN tick. now_sec
  /// is mission-elapsed (missionElapsed()), not this->now(). Built regardless
  /// of separation_.enabled() so the logged columns exist in every arm.
  /// (notes: separation-anchors-every-arm)
  std::vector<SeparationTerm::Anchor> separationAnchors(double now_sec) const;

  /// Allocation vehicle set at (x, y): self plus peers at their last directly
  /// heard beacon position, shared by the allocator and the chase value gate. now_sec is
  /// mission-elapsed (negative reads fresh); only doPlan passes a real age
  /// bound. (notes: alloc-vehicles-definition)
  std::vector<AllocRobot> allocVehicles(float x, float y, bool* all_in_comms,
                                        double now_sec,
                                        double pos_max_age_sec) const;

  // Global allocator, off by default (doPlan never calls solve() or emits
  // allocation). Requires the cell world and, in fleets larger than one, the
  // fleet identity; both are checked at startup and fatal.
  // (notes: global-alloc-off-by-default)
  bool       global_alloc_enable_ = false;
  GlobalAllocator::Config alloc_cfg_;
  /// Consecutive planning ticks each cell has been the focus without yielding
  /// an admissible candidate. Sized with the grid at configure time. The
  /// staleness rule (§3.4) reads it; nothing else does.
  std::vector<int> alloc_focus_skips_;
  /// How many consecutive skips demote a phantom EXPLORING cell to COVERED.
  int        alloc_focus_skip_k_ = 3;
  /// Max age, in mission-elapsed seconds, of a latched peer position in the
  /// allocator solve; <= 0 = unbounded (the pre-TTL behaviour). Applied at the
  /// doPlan solve only (see allocVehicles). (notes: alloc-peer-pos-max-age)
  double     alloc_peer_pos_max_age_sec_ = 0.0;

  // Cumulative voxel deltas from each peer, by fleet id, fed and sized by
  // onFusionCounters. Empty = no counters yet (unmeasured); zeros = nothing
  // arrived. Test against a baseline; a decrease re-baselines.
  // (notes: peer-fusion-deltas-per-peer)
  std::vector<uint64_t> peer_fusion_deltas_;

  // Cells whose local status a PEER's census has changed, over the whole run.
  // The one signal here that no amount of this robot's own driving can move, so
  // a difference across the settle is first-hand evidence that a peer told this
  // robot something it did not know — which is the thing the meeting is for.
  long long            team_merge_applied_total_ = 0;
  PursuitPredictor::Config pursuit_mdp_cfg_;
  // <= 0 means "assume the peer moves as I do" and is the default. It exists
  // as a knob because it is a different KIND of quantity from my own speed: I
  // measure mine and I am guessing at the peer's, and a campaign that wants to
  // test the guess needs to be able to vary it alone.
  double pursuit_mdp_peer_speed_mps_ = -1.0;

  // This robot's global tour from the last allocator solve, broadcast as
  // TeamBeacon.my_tour; empty when the allocator is off or refused. doPlan
  // assigns it per solve; publishBeacon clears it outside the exploration
  // loop. (notes: allocator-my-tour-two-writers)
  std::vector<int> my_tour_;

  bool       team_merge_local_priority_ = true;
  /// Mission-elapsed baseline: sim seconds at the first tick with a live clock,
  /// -1 before. TeamWorld times are mission-elapsed as float32 cannot hold
  /// absolute stamps. Not read from exp_log_, so logging off changes nothing.
  /// (notes: mission-t0-float32-baseline)
  double     mission_t0_sec_ = -1.0;
  /// Sim seconds since mission_t0_sec_, or -1 before the clock is live. Latches
  /// the baseline on its first live call (hence not const), since the publish
  /// timer or the tick may fire first.
  /// (notes: mission-elapsed-latches-baseline)
  double     missionElapsed();
  /// missionElapsed() at a past rclcpp::Time, to put team_last_complete_time_
  /// on the mission-elapsed scale. Returns -1 when the baseline is not latched
  /// or when predates it. (notes: mission-elapsed-at-past-time)
  double     missionElapsedAt(const rclcpp::Time& when) const;
  double     cell_census_period_s_ = 5.0;
  /// Sim-time stamp of the last census, -1 = none yet.
  double     last_cell_census_sec_ = -1.0;
  bool       publish_cell_markers_ = false;
  std::string output_csv_;
  // Event-log output (newline-delimited JSON, one file per robot per run).
  // Empty path = derive from output_csv (see the param load); enabled by
  // default because this file, not the CSV, is what the experiment's primary
  // metric is read from.
  std::string experiment_log_path_;
  bool        experiment_log_enabled_ = true;
  // Descending unknown-fraction ladder for coverage_milestone events. See the
  // param load for the range real runs actually traverse.
  std::vector<double> coverage_milestones_;
  // clock_anchor cadence, in SIM seconds, and the next due stamp. Compared
  // against this->now() and nothing else: gating a sim-time schedule on a wall
  // deadline is the bug that silently halved the CSV sampler's realised rate
  // between campaigns (see experiment_log.hpp).
  double experiment_log_anchor_period_sec_ = 10.0;
  double next_anchor_sim_sec_ = -1.0;
  std::string map_frame_;
  std::string base_frame_;
  int    max_steps_;
  double map_resolution_;
  double goal_xy_tol_;
  // Minimum straight-line XY range from the robot to an acceptable candidate.
  // 0 = off (shipped default). See the param load for why it exists.
  double cand_min_goal_dist_{0.0};
  double goal_yaw_tol_;
  // True when the modelled sensor spans a full circle, derived from fov_hfov at
  // load; then the exploration arrival gate does not block on yaw. Derived, not
  // a param, so it cannot disagree with fov_hfov.
  // (notes: fov-omnidirectional-yaw-gate)
  bool   fov_is_omnidirectional_ = false;
  // Deadline for the post-arrival in-place rotation, measured from the first
  // tick the robot is inside goal_xy_tol_ — NOT shared with nav_budget_sec_,
  // which budgets the drive (see doNavigate).
  double goal_rotate_timeout_sec_;
  // Minimum interval between re-sends of an unchanged goal pose; a changed pose
  // publishes at once. An unchanged re-send is a no-op for simple_nav_3d and
  // recovers a restarted navigator; 0 gives that up.
  // (notes: nav-goal-republish-keepalive)
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
  // Transforms older than this (s) read as pose lost: with a dead broadcaster
  // tf2 keeps serving the last transform. 0 = disabled.
  // (notes: tf-pose-max-age-gate)
  double pose_max_age_sec_ = 5.0;
  double failed_goal_radius_m_;
  double failed_goal_ttl_sec_;
  int    failed_goal_retire_after_{0};
  double done_unknown_fraction_;
  int    done_min_consecutive_steps_;
  // Which rule finishes this robot: "latch" (default) on the first tick the ROI
  // unknown fraction reaches done_unknown_fraction, in any state, never
  // un-finishing; "streak" is the legacy PLAN-tick streak. Either latches
  // finished_. (notes: done-criterion-latch-vs-streak)
  std::string done_criterion_{"latch"};
  // Where the coverage-termination unknown fraction is measured:
  //  - "planning_map": 2D unknown cells in the latched planning_map (legacy).
  //    Returns -1 (check INACTIVE) when no planning_map is published.
  //  - "scovox": 2.5D column coverage of the ROI footprint on the fused 3D
  //    map in map_cache_ (MapCache::unknownColumnFraction) — works with no
  //    planning_map at all.
  //  - "auto" (default): planning_map when one has been received, else scovox.
  std::string done_coverage_source_{"auto"};
  // What DONE does. Gen 34 only idles: the node stays up and keeps beaconing,
  // so a finished robot stays countable to its team (rule 6).
  std::string done_action_{"idle"};
  // Master switch for the 2D planning_map. True: subscribed, a hard startup
  // precondition, used for candidate filtering and cost-grid reachability in
  // both phases. False (default): never read; straight-line costs.
  // (notes: planning-map-master-switch)
  bool   use_planning_map_{false};

  // Utility / coordination params (cached so doPlan() doesn't re-query).
  double cost_grid_radius_cap_m_ = 0.0;
  // Exponent on the path-cost denominator of the utility. 1.0 reproduces the
  // shipped SSMI form exactly (and is short-circuited to do so bit-for-bit);
  // below 1.0 discounts distance so a far-but-informative candidate can win.
  // See the derivation at the utility itself.
  double utility_cost_exponent_ = 1.0;
  bool   trajectory_scoring_   = false;
  double trajectory_sample_spacing_m_ = 1.5;
  bool   coord_enabled_        = false;
  double coord_claim_radius_m_ = 0.0;
  double coord_claim_ttl_sec_  = 5.0;
  // Extra retention for EXPLOIT claims past their TTL, 2x the TTL by default:
  // rides out executor gaps yet frees a dead winner's vantage well inside the
  // 300 s per-target budget. (notes: coord-exploit-claim-grace)
  double coord_claim_grace_sec_ = 10.0;
  double coord_heartbeat_hz_   = 1.0;
  // Heartbeat-suppression episode tracking (see heartbeatTick): lets analysis
  // tell a peer missing for radio loss from one whose planner was busy in a
  // non-beaconing state. (notes: coord-heartbeat-suppression-tracking)
  bool hb_suppressed_ = false;
  bool hb_suppress_warned_ = false;
  // Previous heartbeatTick entry, for late-tick (executor starvation)
  // detection, and a running count of ticks later than the claim TTL.
  rclcpp::Time hb_last_tick_;
  int  hb_late_count_ = 0;
  rclcpp::Time hb_suppress_start_;
  std::string coord_intent_topic_;
  // Split intent stream: publish to coord_intent_pub_topic_, subscribe to
  // coord_intent_sub_topics_ (e.g. a comms emulator's relayed copies). Both
  // empty fall back to coord_intent_topic_, self-echo included.
  // (notes: coord-intent-topic-split)
  std::string coord_intent_pub_topic_;
  std::vector<std::string> coord_intent_sub_topics_;
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
  // When true, roi_min_z_/roi_max_z_ are relative to robot z for the ingest
  // clip, frontier extraction and FOV z-clip (re-banded in loadLatestMap), and
  // candidates snap to ground + candidate_z_clearance.
  // (notes: terrain-relative-z-mode)
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
  // Inset of the FRONTIER search band from the effective ROI band, in metres
  // off the bottom and off the top. Both 0 (the default) = search the whole ROI
  // band, i.e. today's behaviour. See the param load for what these are for.
  bool  frontier_band_set_ = false;
  float frontier_z_lo_off_ = 0.0f;
  float frontier_z_hi_off_ = 0.0f;

  // --- Exploitation ---
  // Phase 2 (DESIGN_gen34.md Q66). Read so a launch file that sets it is
  // checked, and fatal when true; the node explores only.
  bool exploitation_enabled_ = false;

  // --- Components ---
  std::unique_ptr<MapCache> map_cache_;
  std::unique_ptr<CandidateGenerator> candidate_gen_;
  std::unique_ptr<FovEvaluator> fov_eval_;
  ScoreFn score_fn_;
  std::unique_ptr<MetricsLogger> logger_;
  std::unique_ptr<ExperimentLog> exp_log_;
  std::unique_ptr<CostGrid> cost_grid_;
  std::unique_ptr<Coordination> coord_;
  std::unique_ptr<ProximityGuard> prox_guard_;

  // --- State ---
  State state_ = State::WAIT_FOR_MAP;
  Phase phase_ = Phase::EXPLORE;
  int   step_  = 0;
  bool  have_pose_ = false;
  // Edge state for the pose_health event: true between a reported loss and its
  // recovery. Separate from have_pose_ so the pre-first-transform ticks, which
  // also have have_pose_ == false, do not read as a recovery.
  bool  pose_loss_reported_ = false;
  // Unbroken run of doPlan ticks that rejected every candidate. Carried in the
  // throttled starvation WARN, which without it cannot show duration.
  int   consecutive_all_rejected_ = 0;
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
  // Recently-REACHED exploration goals, same structure, opposite trigger. See
  // the param load for why "nearest frontier" needs this to terminate.
  FailedGoalBlacklist visited_goals_;
  double visited_goal_radius_m_ = 0.0;   // 0 = feature off
  double visited_goal_ttl_sec_  = 120.0;

  // Per-navigate-cycle state for the smart timeout.
  double nav_budget_sec_ = 0.0;
  rclcpp::Time progress_check_time_;
  float progress_check_dist_ = 0.0f;

  // Post-arrival rotation deadline. Armed the first tick the robot is inside
  // goal_xy_tol_ but still outside goal_yaw_tol_; disarmed by transitionTo()
  // on every NAVIGATE entry.
  bool rotate_deadline_armed_ = false;
  rclcpp::Time rotate_start_time_;

  // Coverage termination streak (done_criterion == "streak" only).
  int coverage_done_streak_ = 0;

  // Coverage termination latch (done_criterion == "latch"): set on the first
  // qualifying sample, never cleared. The stamps record the latch instant,
  // which is not the run's end. (notes: coverage-latch-one-way)
  bool   coverage_latched_        = false;
  double coverage_latch_t_sim_    = -1.0;
  double coverage_latch_unknown_  = -1.0;

  // --- Mission start and homing ---
  // Homing itself is TeamCore's Home activity (a Leg to home_pos_); these are
  // its tolerance and bound, plus the pre-mission hold. have_home_ is one-shot,
  // unlike have_pose_. (notes: mission-return-home)
  double mission_home_tol_m_      = 1.0;
  double mission_return_max_sec_  = 600.0;

  // No robot plans or navigates until this much mission time has passed, in
  // every arm, so the meeting plan is agreed while co-located. Do not
  // confine the provisional->final upgrade to this hold.
  // (notes: mission-start-hold-all-arms)
  double mission_start_hold_sec_  = 60.0;
  // One-shot, so the "still holding" line cannot be mistaken for a stall and
  // the release is stamped once with the number it waited for.
  bool   mission_start_hold_logged_ = false;
  bool   have_home_               = false;
  Eigen::Vector3f home_pos_       = Eigen::Vector3f::Zero();
  float  home_yaw_                = 0.f;
  // Hold state: the driving state to resume (current_goal_ untouched), CSV hold
  // count/seconds, and drive seconds already used; the resume backdates
  // state_enter_time_ by it so the nav budget continues.
  // (notes: proximity-hold-bookkeeping)
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
  float pending_info_gain_std_        = 0.0f;
  float pending_mean_path_cost_       = 0.0f;
  float pending_selected_info_gain_   = 0.0f;
  float pending_selected_path_cost_   = 0.0f;
  float pending_plan_ms_              = 0.0f;
  int   pending_rejected_by_minpos_      = 0;
  int   pending_rejected_by_unreachable_ = 0;

  // Rejection profile of the latest doPlan attempt, drained by
  // fillCommonMetrics so it reaches every row, including a starved planner's
  // timer rows. -1 = no planning attempt yet, never a measured zero.
  // (notes: plan-rejection-profile-metrics)
  int   pending_plan_cand_total_    = -1;
  int   pending_plan_rej_close_     = -1;
  int   pending_plan_rej_map_       = -1;
  int   pending_plan_rej_unreach_   = -1;
  int   pending_plan_rej_blacklist_ = -1;
  // The visited half of pending_plan_rej_blacklist_, not a sixth disjoint
  // bucket: every candidate counted here is ALSO counted there, so the five
  // original rejection columns still sum to the candidates the filter refused
  // and nothing about the old arithmetic changes. See metrics_logger.hpp.
  int   pending_plan_rej_visited_   = -1;
  int   pending_plan_rej_minpos_    = -1;
  int   pending_plan_stall_ticks_   = -1;

  // Separation-term diagnostics (separation.hpp), filled by doPlan each
  // exploration tick. sep_reordered: 1 yes, 0 no, -1 not asked. Peer distance
  // and eligible peers are logged even when the term is off.
  // (notes: separation-manipulation-check-columns)
  int   pending_sep_eligible_peers_   = 0;
  float pending_sep_peer_dist_m_      = -1.0f;  // -1 = no eligible peer
  float pending_sep_discount_         = 1.0f;
  int   pending_sep_reordered_        = -1;
  // Cached active intent so the heartbeat timer can re-publish without
  // touching planning state.
  explo_planner_msgs::msg::RobotIntent current_intent_msg_;
  bool   have_active_intent_ = false;

  // (t_sim_sec, total_observed_voxels) samples fed by fillCommonMetrics give
  // the cached growth rate; stampMapInfo() copies count and rate onto outgoing
  // intents. The window is 300 s, not the contact length.
  // (notes: map-size-beacon-rate-window)
  static constexpr double kRateWindowSec = 300.0;
  static constexpr double kRateWinsorK   = 2.0;
  std::vector<std::pair<double, double>> map_size_hist_;
  double latest_map_voxels_ = 0.0;
  double map_growth_rate_   = 0.0;   // voxels / sim-second, >= 0
  void   noteMapSize(double t_sim_sec, double voxels);
  void   stampMapInfo();
  /// The only way this node publishes an intent. Stamps the map-size beacon
  /// onto the cached message first, so no publish carries a stale or zeroed
  /// count and rate. Callers must have checked intent_pub_.
  /// (notes: publish-intent-stamps-beacon)
  void   publishIntent();
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
  // Per-source integration counters from the same dscovox node, on their own
  // topic and their own timer. Created only when the fleet is configured:
  // without fleet ids there is nothing to key the per-peer totals by.
  rclcpp::Subscription<scovox_msgs::msg::ScovoxFusionCounters>::SharedPtr
      fusion_counters_sub_;
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
  // Separate topic from viz_pub_: publishCandidateViz() clears with DELETEALL,
  // which would wipe a shared topic. Created only when the cell world is
  // enabled and markers are requested. (notes: viz-cell-world-marker-topic)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      cell_viz_pub_;
  rclcpp::Publisher<explo_planner_msgs::msg::RobotIntent>::SharedPtr intent_pub_;
  // One subscription per configured source topic. Single-element in the
  // default (shared-bus) wiring; one entry per peer when the stream is split
  // across an emulator's per-link relays.
  std::vector<rclcpp::Subscription<explo_planner_msgs::msg::RobotIntent>::SharedPtr>
      intent_subs_;
  // Peer localiser poses (map frame, ~10 Hz) feed the proximity guard.
  // nav_cancel_client_ has no server on this stack and never fires; the brake
  // goal does the stopping, since the navigator drives its last goal.
  // (notes: proximity-guard-cancel-client)
  std::vector<rclcpp::Subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> peer_pose_subs_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      nav_cancel_client_;
  // Latched (transient_local) hold-state string for the operator: see
  // publishProxState().
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr prox_state_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  // Periodic CSV sampler (SIM seconds); see metricsTick(). <= 0 disables it and restores the
  // pre-experiment behaviour of one row per completed exploration step.
  double metrics_period_sec_ = 5.0;
  rclcpp::TimerBase::SharedPtr metrics_timer_;
  // Adaptive back-off state for metricsTick (see the comment there). Steady
  // clock, deliberately: this bounds executor-thread work, which is wall-clock
  // work regardless of use_sim_time.
  double metrics_max_duty_ = 0.2;
  double metrics_effective_period_ = 5.0;
  int    metrics_backoffs_ = 0;
  std::chrono::steady_clock::time_point metrics_next_{};
  // Realised CSV sampling, reported in run_end: the steady-clock deadline in
  // metricsTick can suppress sim-time ticks, so the achieved period can differ
  // from the configured one. (notes: metrics-realised-sampling)
  int    metrics_rows_written_ = 0;
  double metrics_first_row_sim_sec_ = -1.0;
  double metrics_last_row_sim_sec_  = -1.0;

  // --- Event-log bookkeeping (see experiment_log.hpp) ---
  // state_enter_time_ is default-constructed on the SYSTEM clock and only
  // becomes a sim-time stamp at the first transitionTo, and subtracting times
  // from two different sources THROWS. This flag is what lets the state_change
  // event report the dwell in the state being left without that landing in a
  // ROS callback as an exception. Same bool-guard pattern as midrun_end_armed_.
  bool have_state_enter_ = false;
  // Keyed on step_: one exploration_complete per exhaustion episode. The
  // finished latch makes a second one impossible in gen 34; the key stays as a
  // guard.
  // (notes: exploration-complete-dedup)
  int exp_complete_step_  = -1;
  int exp_complete_count_ = 0;
  // Why the node reached DONE, kept for the run_end the destructor writes in
  // done_action=idle (where DONE is not the end of the file). Empty = DONE was
  // never reached, i.e. the run was cut short from outside.
  std::string done_reason_;
  // Per-peer belief behind peer_lost / peer_seen: when this robot last heard
  // the peer, and whether it currently believes it live. Bounded by the team
  // size (one entry per robot id ever heard).
  struct PeerBelief {
    rclcpp::Time last_heard;   ///< local receipt time of its last intent
    bool         live = false;
  };
  std::map<std::string, PeerBelief> peer_belief_;
  // Latest coverage measurement seen by fillCommonMetrics, so the terminal
  // events can report the same number the milestones were judged against
  // instead of paying for another whole-grid measurement at shutdown.
  double      last_unknown_fraction_ = -1.0;
  const char* last_coverage_source_  = "none";

  // ----------------------------------------------------------------
  // Team layer (gen 34, DESIGN_gen34.md §8)
  // ----------------------------------------------------------------
  // TeamCore asks the node its world questions through this: the allocator,
  // the scheduler, the value gate, the intercept predictor and the plan map.
  class NodeOracle : public gen34::TeamOracle {
   public:
    explicit NodeOracle(ExploPlannerNode* n) : n_(n) {}
    gen34::PlanSolve solvePlan(
        const std::vector<gen34::TeamRobotView>& team) override {
      return n_->oracleSolvePlan(team);
    }
    double pathDistance(const gen34::Vec2& from, int cell,
                        const gen34::Vec2& center) override {
      return n_->oraclePathDistance(from, cell, center);
    }
    bool standable(const gen34::Vec2& p) override {
      return n_->oracleStandable(p);
    }
    bool lineOfSight(const gen34::Vec2& a, const gen34::Vec2& b) override {
      return n_->oracleLineOfSight(a, b);
    }
    gen34::ChaseGateVerdict chaseGate(
        const std::vector<gen34::ChaseGateView>& missing) override {
      return n_->oracleChaseGate(missing);
    }
    bool intercept(const gen34::ChaseGateView& peer,
                   gen34::Vec2* out) override {
      return n_->oracleIntercept(peer, out);
    }
   private:
    ExploPlannerNode* n_;
  };
  gen34::PlanSolve oracleSolvePlan(const std::vector<gen34::TeamRobotView>& team);
  double oraclePathDistance(const gen34::Vec2& from, int cell,
                            const gen34::Vec2& center) const;
  bool oracleStandable(const gen34::Vec2& p) const;
  bool oracleLineOfSight(const gen34::Vec2& a, const gen34::Vec2& b) const;
  gen34::ChaseGateVerdict oracleChaseGate(
      const std::vector<gen34::ChaseGateView>& missing) const;
  bool oracleIntercept(const gen34::ChaseGateView& peer, gen34::Vec2* out) const;

  /// One TeamCore tick and its events; runs every tick from the first pose.
  void teamTick();
  /// Move the state machine to the state TeamCore's activity names.
  void applyActivity();
  /// The team states' drive: a Leg through leg_, or stand still.
  void doTeamState();
  /// Close a proximity hold that the team activity, not the guard, ended.
  void leaveProximityHold(const char* why);
  static State stateForActivity(gen34::Activity a);
  bool pickEscape(const gen34::Vec2& from, const gen34::Vec2& toward,
                  gen34::Vec2* out);
  void publishLegGoal(const gen34::Vec2& p);
  /// A goal at the robot's own pose: the navigator stops and holds.
  void brakeHere();
  void publishBeacon();
  void drainBeacons();
  void logTickEvent(double now_sec);
  /// Peers TeamCore counts present (a beacon heard directly in the window).
  int presentPeerCount() const;
  /// The team size less this robot; 0 without a fleet.
  int expectedPeers() const;

  gen34::Arm          arm_ = gen34::Arm::kOff;
  gen34::TeamConfig   team_cfg_;
  gen34::LegConfig    leg_cfg_;
  std::unique_ptr<NodeOracle>       oracle_;
  std::unique_ptr<gen34::TeamCore>  team_;
  gen34::LegTracker   leg_;
  /// The Leg goal last published, for the keep-alive republish.
  CandidateViewpoint  leg_goal_;
  /// TeamCore's latest output; valid once team_ticked_.
  gen34::TickOutputs  team_out_;
  bool                team_ticked_ = false;
  /// Exploration finished (K4): the coverage latch or the step budget. Never
  /// cleared. Carried on the beacon.
  bool                finished_ = false;
  std::string         finished_reason_;
  /// The hold drive's one brake goal has been sent.
  bool                hold_braked_ = false;
  /// Rotates the escape picker's first offset, as gen 33's fallback did.
  int                 escape_rot_ = 0;

  // Beacon (TeamBeacon.msg): published on its own sim-clock timer so a busy
  // planning tick never silences it; received copy-only and drained on the
  // tick, one pending slot per sender (each beacon is full state).
  double beacon_hz_ = 1.0;
  std::string beacon_pub_topic_;
  std::vector<std::string> beacon_sub_topics_;
  rclcpp::Publisher<explo_planner_msgs::msg::TeamBeacon>::SharedPtr beacon_pub_;
  std::vector<rclcpp::Subscription<explo_planner_msgs::msg::TeamBeacon>::SharedPtr>
      beacon_subs_;
  rclcpp::TimerBase::SharedPtr beacon_timer_;
  struct PendingBeacon {
    explo_planner_msgs::msg::TeamBeacon::SharedPtr msg;
    rclcpp::Time received;
  };
  std::map<int, PendingBeacon> beacon_pending_;
  long long beacons_sent_ = 0;
  long long beacons_received_ = 0;
  long long beacons_superseded_ = 0;
  long long beacons_rejected_ = 0;
  long long cell_merges_refused_ = 0;

  // Map-exchange progress (Q51), by fleet id, from the fusion counters: the
  // newest ScovoxMapBinary.seq this robot's dscovox has from each robot
  // (itself included) and the gaps it saw. Empty until the first sample.
  std::vector<uint64_t> seq_newest_;
  std::vector<uint64_t> seq_gaps_;
  // Where they come from, for the warning when they never do, and when the
  // team first ticked without them (-1: not waiting).
  std::string counters_topic_;
  double seq_wait_since_ = -1.0;

  // K20 tick event cadence, sim seconds; <= 0 disables it.
  double tick_event_period_sec_ = 2.0;
  double next_tick_event_sec_   = -1.0;

  // Homing outcome for run_end: "arrived", "timeout" or "no-home"; "" while
  // unresolved, including a run killed mid-homing.
  std::string mission_home_result_;
  double      mission_home_sim_sec_ = -1.0;
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
  // Sibling of dp for float members: ROS 2 has no float parameter type, so
  // these declare a double and narrow. The default is typed double so float
  // literals at call sites still compile. (notes: param-dp-f-float-narrowing)
  auto dp_f = [&](auto n, double d) {
    return static_cast<float>(dp(n, d));
  };

  max_steps_    = dp("max_steps", 200);
  robot_name_   = dp("robot_name", std::string("atlas"));
  // Ordered list of every robot, identical on every robot: a robot's id is its
  // index, used by every knowledge mask, gossip slot and tie-break. Empty =
  // unconfigured, not an error; malformed = refuse to start.
  // (notes: param-fleet-identity)
  team_robot_names_ =
      dp("team_robot_names", std::vector<std::string>{});
  fleet_ = makeFleetIdentity(team_robot_names_, robot_name_);
  if (!fleet_.error.empty()) {
    RCLCPP_FATAL(this->get_logger(), "team_robot_names: %s",
                 fleet_.error.c_str());
    throw std::runtime_error("team_robot_names: " + fleet_.error);
  }
  if (fleet_.configured) {
    RCLCPP_INFO(this->get_logger(),
                "Fleet identity: robot_id=%d of %d, team_hash=0x%08x",
                fleet_.self_id, fleet_.size(), fleet_.team_hash);
  }
  output_csv_   = dp("output_csv", std::string("/tmp/exploration.csv"));
  // One JSONL event log per robot per run. Empty (default) derives <output_csv
  // minus .csv>.events.jsonl, so each log sits beside its own CSV and robots
  // never share a file; an explicit value is used verbatim.
  // (notes: param-experiment-log-path)
  experiment_log_path_    = dp("experiment_log_path", std::string(""));
  experiment_log_enabled_ = dp("experiment_log_enabled", true);
  if (experiment_log_path_.empty()) {
    experiment_log_path_ = output_csv_;
    const std::string suffix = ".csv";
    if (experiment_log_path_.size() >= suffix.size() &&
        experiment_log_path_.compare(experiment_log_path_.size() - suffix.size(),
                                     suffix.size(), suffix) == 0) {
      experiment_log_path_.erase(experiment_log_path_.size() - suffix.size());
    }
    experiment_log_path_ += ".events.jsonl";
  }
  // Descending unknown-fraction ladder for coverage_milestone events, which
  // make time-to-coverage comparable across arms. Invariant: every rung sits
  // strictly above done_unknown_fraction; move one, move the other.
  // (notes: param-coverage-milestone-ladder)
  coverage_milestones_ = dp("coverage_milestones",
      std::vector<double>{0.90, 0.85, 0.80, 0.75, 0.70, 0.65});
  // clock_anchor cadence in SIM seconds. Each anchor is a (sim, wall) pair plus
  // the RTF since the last, the conversion table for wall-stamped logs. <= 0
  // disables anchors. (notes: param-clock-anchor-cadence)
  experiment_log_anchor_period_sec_ =
      dp("experiment_log_anchor_period_sec", 10.0);
  // CSV sampling period; steps do not advance during a team activity, so
  // step-end rows alone would flatten it. 0 restores step-rows-only behaviour.
  // (notes: param-metrics-period)
  metrics_period_sec_ = dp("metrics_period_sec", 5.0);
  // Ceiling on the fraction of wall time the metrics sampler may consume. It is
  // a real-time budget, not a preference: the sampler shares one thread with
  // the coordination beacon, and a beacon delayed past coord_claim_ttl_sec is
  // read by peers as this robot having vanished.
  metrics_max_duty_ = std::clamp(dp("metrics_max_duty", 0.2), 0.01, 1.0);
  metrics_effective_period_ = metrics_period_sec_;
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
  // The arrival gate must be strictly looser than simple_nav_3d's stop
  // condition, or the navigator stops just outside it and failGoal() blacklists
  // the goal the robot stands on. Hence the warnings below 0.3.
  // (notes: param-arrival-gate-vs-navigator)
  if (goal_yaw_tol_ < 0.3) {
    RCLCPP_WARN(get_logger(),
        "goal_yaw_tolerance=%.2f rad is below 0.3 — the navigator stops as soon "
        "as it is inside its OWN yaw tolerance (simple_nav_3d "
        "ugv.goal_yaw_tol_rad, 0.2 as shipped), which can be outside this gate; "
        "that fails the goal on the rotate deadline and blacklists the pose the "
        "robot is standing on. Use >= 0.4, or match it to this robot's "
        "navigator.", goal_yaw_tol_);
  }
  if (goal_xy_tol_ < 0.3) {
    RCLCPP_WARN(get_logger(),
        "goal_xy_tolerance=%.2f m is below 0.3 — the median measured park is "
        "0.269 m from the commanded point, so this gate would miss half of all "
        "arrivals; see the goal_yaw_tolerance warning.", goal_xy_tol_);
  }
  // doPlan rejects candidates nearer than this, which stops a near-frontier
  // two-point oscillation. Scale it to the sensor's vertical-FOV standoff (a
  // few metres for a VLP-16). Default 0.0 keeps shipped behaviour.
  // (notes: param-candidate-min-goal-dist)
  cand_min_goal_dist_ = dp("candidate_min_goal_dist_m", 0.0);
  if (cand_min_goal_dist_ > 0.0 && cand_min_goal_dist_ <= goal_xy_tol_) {
    RCLCPP_WARN(get_logger(),
        "candidate_min_goal_dist_m=%.2f is not above goal_xy_tolerance=%.2f, "
        "so it rejects nothing the arrival gate did not already reject and "
        "the near-frontier oscillation it exists to prevent is still live.",
        cand_min_goal_dist_, goal_xy_tol_);
  }
  integrate_wait_ = dp("integrate_wait", 2.0);

  // NAVIGATE budget, computed at entry from straight-line distance to the goal:
  // budget = clamp(dist / speed_est * safety, nav_min, nav_max).
  // (notes: param-nav-distance-budget)
  nav_speed_est_mps_  = dp("nav_speed_estimate_mps", 0.5);
  nav_safety_factor_  = dp("nav_safety_factor", 2.0);
  nav_min_timeout_sec_ = dp("nav_min_timeout_sec", 8.0);
  nav_max_timeout_sec_ = dp("nav_max_timeout_sec", 60.0);

  // No-progress watchdog, independent of the total budget: fail the goal early
  // if travel within progress_window_sec is under progress_min_distance_m.
  // Mirrors the navigator's progress check. (notes: param-no-progress-watchdog)
  progress_window_sec_   = dp("progress_window_sec", 6.0);
  progress_min_distance_m_ = dp("progress_min_distance_m", 0.3);
  // Teleport guard for cumulative distance (see member doc). At the 10 Hz tick
  // this cannot reject real motion; it filters localization discontinuities so
  // they don't inflate distance_traveled or spoof the no-progress watchdog.
  max_pose_jump_m_ = dp_f("max_pose_jump_m", 1.0);
  // See the member: a dead TF chain keeps "succeeding" with the same stamp,
  // so freshness is checked explicitly in updatePoseFromTF. Sized for the
  // slowest healthy publisher in the map->base chain (field SLAM's map->odom
  // at well under 1 Hz), not for the 10 Hz tick.
  pose_max_age_sec_ = dp("pose_max_age_sec", 5.0);

  // A timed-out goal is parked for failed_goal_ttl_sec; candidates within
  // failed_goal_radius_m of a live entry are rejected. The TTL must outlast one
  // worst-case nav budget elsewhere plus travel.
  // (notes: param-failed-goal-blacklist-ttl)
  failed_goal_radius_m_ =
      dp("failed_goal_radius_m", 2.0);
  failed_goal_ttl_sec_ =
      dp("failed_goal_ttl_sec", 240.0);
  // After failed_goal_retire_after failures at one site it is suppressed for
  // the run; 0 = pure TTL. Arriving inside the radius clears it (clearNear),
  // and amnesty re-offers retired sites when nothing else is left.
  // (notes: param-failed-goal-retirement)
  failed_goal_retire_after_ = dp("failed_goal_retire_after", 3);
  failed_goals_.setRetireAfter(failed_goal_retire_after_);
  // Make the drift that caused seed18 impossible to repeat silently. The two
  // numbers are coupled — a blacklist entry has to outlive one attempt
  // elsewhere — and nothing enforced that coupling, so raising one of them in
  // the campaign yaml quietly disarmed the other.
  if (failed_goal_ttl_sec_ < nav_max_timeout_sec_ + 30.0) {
    RCLCPP_WARN(get_logger(),
        "failed_goal_ttl_sec=%.0f is below nav_max_timeout_sec=%.0f + 30: a "
        "blacklisted site can expire while the robot is still burning one full "
        "nav budget somewhere else, which re-opens the trap it was meant to "
        "close. Use >= %.0f.",
        failed_goal_ttl_sec_, nav_max_timeout_sec_, nav_max_timeout_sec_ + 30.0);
  }
  // A reached goal is parked for visited_goal_ttl_sec and EXPLORE candidates
  // within visited_goal_radius_m are skipped; 0 = off. Size the radius near the
  // cluster radius; the TTL must outlast a there-and-back trip.
  // (notes: param-visited-goal-suppression)
  visited_goal_radius_m_ = dp("visited_goal_radius_m", 0.0);
  visited_goal_ttl_sec_  = dp("visited_goal_ttl_sec", 120.0);

  // Streak termination: DONE when ROI unknown fraction stays below
  // done_unknown_fraction for done_min_consecutive_steps plan cycles; <= 0
  // disables. The default is generic; each world needs its own calibrated
  // value. (notes: param-done-unknown-fraction)
  done_unknown_fraction_ =
      dp("done_unknown_fraction", 0.05);
  // Every coverage milestone must sit strictly above done_unknown_fraction, or
  // it only times post-stop merging. Report-only: rungs still fire and events
  // carry post_latch. (notes: milestones-above-done-threshold)
  if (done_unknown_fraction_ > 0.0) {
    std::vector<double> below;
    for (double m : coverage_milestones_)
      if (m <= done_unknown_fraction_) below.push_back(m);
    if (!below.empty()) {
      std::ostringstream ss;
      for (size_t i = 0; i < below.size(); ++i)
        ss << (i ? ", " : "") << below[i];
      RCLCPP_WARN(this->get_logger(),
          "coverage_milestones: %zu rung(s) [%s] are at or below "
          "done_unknown_fraction=%.3f. Those can only be crossed AFTER this "
          "robot declares exploration complete, so they time post-stop map "
          "merging, not exploration, and their reach rate tracks the arm. "
          "Do not report them as endpoints (each event carries post_latch). "
          "Fix by truncating the ladder, not by lowering the threshold.",
          below.size(), ss.str().c_str(), done_unknown_fraction_);
    }
  }
  done_min_consecutive_steps_ =
      dp("done_min_consecutive_steps", 3);
  // "latch" (default): own fused-map ROI unknown fraction, tested every metrics
  // tick in every state, on first touch, no rendezvous gate. "streak": the
  // doPlan-top test described above. (notes: param-done-criterion-latch)
  done_criterion_ = dp("done_criterion", std::string("latch"));
  if (done_criterion_ != "latch" && done_criterion_ != "streak") {
    RCLCPP_WARN(get_logger(),
        "Unknown done_criterion '%s' — falling back to 'latch'.",
        done_criterion_.c_str());
    done_criterion_ = "latch";
  }
  // "planning_map" (2D, inactive when none is published), "scovox" (2.5D column
  // coverage of the fused map) or "auto". They measure different things:
  // recalibrate done_unknown_fraction when switching.
  // (notes: param-done-coverage-source)
  done_coverage_source_ = dp("done_coverage_source", std::string("auto"));
  if (done_coverage_source_ != "auto" &&
      done_coverage_source_ != "planning_map" &&
      done_coverage_source_ != "scovox") {
    RCLCPP_WARN(get_logger(),
        "Unknown done_coverage_source '%s' — falling back to 'auto'.",
        done_coverage_source_.c_str());
    done_coverage_source_ = "auto";
  }
  // The gen-34 node never shuts down (DESIGN_gen34.md §8.6, rule 6): DONE
  // idles and keeps beaconing, so a finished robot stays countable and a late
  // meeting can still find it. Read only so a launch file asking for
  // "shutdown" is told it does not get it.
  done_action_ = dp("done_action", std::string("idle"));
  if (done_action_ != "idle") {
    RCLCPP_WARN(get_logger(),
        "done_action='%s' is not supported by the gen-34 node: DONE idles and "
        "keeps beaconing. Using 'idle'.", done_action_.c_str());
    done_action_ = "idle";
  }

  // Candidate generation
  CandidateConfig ccfg;
  ccfg.n_radial   = dp("candidate_n_radial", 8);
  ccfg.n_rings    = dp("candidate_n_rings", 3);
  ccfg.min_radius = dp_f("candidate_min_radius", 2.0);
  ccfg.max_radius = dp_f("candidate_max_radius", 8.0);
  // 1, not the historical 4: with a full-azimuth FOV model the yaw samples at
  // one position score the same view repeatedly (see fov_hfov below), so the
  // extra three are aliasing noise the ranking would otherwise sort on.
  ccfg.n_yaw      = dp("candidate_n_yaw", 1);
  ccfg.robot_z    = dp_f("candidate_robot_z", 0.3);
  ccfg.occ_thresh = dp_f("candidate_occ_thresh", 0.7);
  ccfg.ground_z   = dp_f("candidate_ground_z", 0.15);
  ccfg.enable_polar = dp("candidate_enable_polar", true);
  // Region of interest. Defaults bound the robot to a 30x30 m square
  // centred on the world origin. Must match the dscovox planning_map
  // size + origin so the global planner is constrained to the same area.
  ccfg.roi_min_x  = dp_f("roi_min_x", -15.0);
  ccfg.roi_max_x  = dp_f("roi_max_x",  15.0);
  ccfg.roi_min_y  = dp_f("roi_min_y", -15.0);
  ccfg.roi_max_y  = dp_f("roi_max_y",  15.0);

  // In dscovox mode the z-slab the fused map is clipped to on ingest; it bounds
  // the FOV raycast, frontier extraction and map stats. FOV rays leaving it are
  // clipped (no info gain, no occlusion). (notes: param-vertical-roi-band)
  roi_min_z_ = dp_f("roi_min_z", -0.5);
  roi_max_z_ = dp_f("roi_max_z",  2.0);

  // Terrain-relative (3D) mode — see the member doc. Off by default: flat
  // mode is bit-for-bit the legacy behaviour.
  terrain_relative_z_ = dp("terrain_relative_z", false);
  ccfg.terrain_relative    = terrain_relative_z_;
  ccfg.z_clearance         = dp_f("candidate_z_clearance", 0.5);
  ccfg.ground_search_below = dp_f("ground_search_below_m", 4.0);
  ccfg.ground_search_above = dp_f("ground_search_above_m", 1.0);
  ccfg.ground_stack_max_m  = dp_f("ground_stack_max_m", 0.6);
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

  // Ground search must stay inside the ingested slab before loadLatestMap()
  // re-bands, else groundZAt returns NaN on downslopes and terrain-mode
  // consumers degrade silently. Checked at startup.
  // (notes: terrain-band-floor-check)
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
  frontier_cluster_radius_m_ = dp_f("frontier_cluster_radius_m", 5.0);
  // Insets the frontier search band from the ROI floor and ceiling; both 0 =
  // whole ROI band. Set it to the heights the sensor sweeps while driving. Only
  // narrows: intersected with the ingested ROI band.
  // (notes: param-frontier-z-band)
  const double f_lo_off = dp("frontier_z_lo_offset_m", 0.0);
  const double f_hi_off = dp("frontier_z_hi_offset_m", 0.0);
  frontier_z_lo_off_ = static_cast<float>(std::max(0.0, f_lo_off));
  frontier_z_hi_off_ = static_cast<float>(std::max(0.0, f_hi_off));
  frontier_band_set_ = (frontier_z_lo_off_ > 0.0f || frontier_z_hi_off_ > 0.0f);
  if (frontier_band_set_) {
    RCLCPP_INFO(get_logger(),
        "Frontier search band inset by %.2f m from the ROI floor and %.2f m "
        "from its ceiling (ROI z [%.2f, %.2f]).",
        frontier_z_lo_off_, frontier_z_hi_off_, roi_min_z_, roi_max_z_);
  }

  // FOV evaluation. Defaults are the VLP-16 the robots carry, matching the SDF;
  // they are what a launch that skips shared_params.yaml gets.
  // (notes: fov-default-vlp16-model)
  FovConfig fcfg;
  fcfg.hfov      = dp_f("fov_hfov", 6.28318);
  fcfg.vfov      = dp_f("fov_vfov", 0.5236);
  fcfg.min_range = dp_f("fov_min_range", 0.3);
  fcfg.max_range = dp_f("fov_max_range", 20.0);
  fcfg.h_rays    = dp("fov_h_rays", 96);
  fcfg.v_rays    = dp("fov_v_rays", 16);
  fcfg.occ_stop  = dp_f("fov_occ_stop", 0.7);
  // Omnidirectional if hfov is within one ray step of 2*pi. Not an exact == on
  // 2*pi: the yaml and SDF carry rounded values that would read as directional.
  // (notes: fov-omnidirectional-tolerance)
  {
    const float h_step = (fcfg.h_rays > 0)
        ? fcfg.hfov / static_cast<float>(fcfg.h_rays) : 0.0f;
    fov_is_omnidirectional_ =
        fcfg.hfov >= (2.0f * static_cast<float>(M_PI) - h_step);
    if (fov_is_omnidirectional_) {
      RCLCPP_INFO(get_logger(),
          "FOV model is omnidirectional (hfov=%.4f rad over %d rays, %.2f deg "
          "apart): exploration arrival does NOT gate on yaw. Exploitation "
          "vantages still do.",
          fcfg.hfov, fcfg.h_rays,
          h_step * 180.0f / static_cast<float>(M_PI));
    } else {
      RCLCPP_INFO(get_logger(),
          "FOV model is directional (hfov=%.4f rad = %.1f deg): exploration "
          "arrival gates on yaw within %.2f rad.",
          fcfg.hfov, fcfg.hfov * 180.0f / static_cast<float>(M_PI),
          goal_yaw_tol_);
    }
  }
  fcfg.roi_min_x = ccfg.roi_min_x;
  fcfg.roi_max_x = ccfg.roi_max_x;
  fcfg.roi_min_y = ccfg.roi_min_y;
  fcfg.roi_max_y = ccfg.roi_max_y;
  fcfg.roi_min_z = roi_min_z_;
  fcfg.roi_max_z = roi_max_z_;

  // 0 = auto -> candidate_max_radius + 2 m slack at flood time.
  cost_grid_radius_cap_m_ = dp("cost_grid_radius_cap_m", 0.0);

  // Distance exponent on the utility denominator; 1.0 = shipped behaviour.
  // Clamped to [0, 2]: a negative exponent rewards distance without bound, and
  // above 2 the candidate filters do it more cheaply.
  // (notes: param-utility-cost-exponent)
  utility_cost_exponent_ = dp("utility_cost_exponent", 1.0);
  if (!std::isfinite(utility_cost_exponent_)) {
    // NaN passes both range tests below (every comparison on NaN is false)
    // and reaches every U(c) as pow(cost, nan) = nan — an arbitrary pick per
    // tick once the unstable sort permutes the nan-keyed candidates. YAML
    // makes this reachable: both `nan` and `.nan` parse as double params.
    RCLCPP_WARN(get_logger(),
        "utility_cost_exponent=%f is not finite; using the default 1.0.",
        utility_cost_exponent_);
    utility_cost_exponent_ = 1.0;
  } else if (utility_cost_exponent_ < 0.0 || utility_cost_exponent_ > 2.0) {
    const double raw = utility_cost_exponent_;
    utility_cost_exponent_ = std::clamp(utility_cost_exponent_, 0.0, 2.0);
    RCLCPP_WARN(get_logger(),
        "utility_cost_exponent=%.3f is outside [0, 2]; clamped to %.3f.",
        raw, utility_cost_exponent_);
  }

  // Soft team-separation multiplier on U(c) (separation.hpp); weight 0 = off
  // (default). configure() refuses invalid values rather than clamping, and the
  // refusal logs at ERROR. (notes: param-separation-term)
  {
    SeparationTerm::Config scfg;
    scfg.weight      = dp("separation_weight", 0.0);
    scfg.radius_m    = dp("separation_radius_m", 20.0);
    scfg.max_age_sec = dp("separation_max_age_sec", 10.0);
    const std::string serr = separation_.configure(scfg);
    if (!serr.empty()) {
      RCLCPP_ERROR(get_logger(), "Separation term DISABLED: %s.", serr.c_str());
    } else if (separation_.enabled()) {
      RCLCPP_INFO(get_logger(),
          "Separation term ON: weight=%.3f radius=%.1fm max_age=%.1fs "
          "(utility is multiplied by 1 - weight*(1 - d/radius) for a "
          "candidate d metres from the nearest fresh teammate).",
          separation_.config().weight, separation_.config().radius_m,
          separation_.config().max_age_sec);
    }
  }

  // TTL on the peer position the allocator solves over; 0 = unbounded
  // (default). Not clamped or refused, but NaN would drop every peer from the
  // allocation, so it is caught and reset to 0.
  // (notes: param-alloc-peer-pos-ttl)
  alloc_peer_pos_max_age_sec_ = dp("alloc_peer_pos_max_age_sec", 0.0);
  if (!std::isfinite(alloc_peer_pos_max_age_sec_)) {
    RCLCPP_WARN(get_logger(),
        "alloc_peer_pos_max_age_sec=%f is not finite; using 0 (unbounded).",
        alloc_peer_pos_max_age_sec_);
    alloc_peer_pos_max_age_sec_ = 0.0;
  } else if (alloc_peer_pos_max_age_sec_ > 0.0) {
    RCLCPP_INFO(get_logger(),
        "Allocator peer-position TTL ON: %.1f s (a peer whose pose is older "
        "than this is dropped from the allocation and its cells return to the "
        "pool). Applies to the doPlan solve only.",
        alloc_peer_pos_max_age_sec_);
  }

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
  coord_claim_grace_sec_ = dp("coord_claim_grace_sec", 10.0);
  coord_intent_topic_    = dp("coord_intent_topic",
                              std::string("/exploration/intents"));
  // Split intent stream; empty = shared bus. A non-absolute topic resolves
  // under /<robot_name>/ since this node is not namespaced by the launch files;
  // a relative one would miss the emulator's per-robot relays.
  // (notes: param-split-intent-topics)
  auto resolve_topic = [this](const std::string& t) {
    if (t.empty() || t.front() == '/' || t.front() == '~') return t;
    return "/" + robot_name_ + "/" + t;
  };
  coord_intent_pub_topic_ =
      resolve_topic(dp("coord_intent_pub_topic", std::string("")));
  if (coord_intent_pub_topic_.empty())
    coord_intent_pub_topic_ = coord_intent_topic_;
  coord_intent_sub_topics_ =
      dp("coord_intent_sub_topics", std::vector<std::string>{});
  for (auto& t : coord_intent_sub_topics_) t = resolve_topic(t);
  // Drop empties before the fallback test, so a stray "" in the list cannot
  // create a subscription on the empty topic (rclcpp throws) and an
  // all-empty list still degrades to the shared bus rather than leaving the
  // planner deaf.
  coord_intent_sub_topics_.erase(
      std::remove(coord_intent_sub_topics_.begin(),
                  coord_intent_sub_topics_.end(), ""),
      coord_intent_sub_topics_.end());
  if (coord_intent_sub_topics_.empty())
    coord_intent_sub_topics_ = {coord_intent_topic_};
  coord_heartbeat_hz_    = dp("coord_heartbeat_hz", 1.0);

  // Homing is TeamCore's Home activity, in every arm (§8.4): a Leg to the
  // start pose within mission_home_tol_m, given up (parked, result "timeout")
  // after mission_return_max_sec. There is no switch: a finished robot always
  // goes home, and the bound must be real.
  mission_home_tol_m_     = dp("mission_home_tol_m", 1.0);
  mission_return_max_sec_ = dp("mission_return_max_sec", 600.0);
  if (!std::isfinite(mission_home_tol_m_) || mission_home_tol_m_ <= 0.0 ||
      !std::isfinite(mission_return_max_sec_) ||
      mission_return_max_sec_ <= 0.0) {
    RCLCPP_FATAL(get_logger(),
        "mission_home_tol_m=%.3f / mission_return_max_sec=%.3f: both must be "
        "finite and positive. Homing runs in every gen-34 arm, and a zero "
        "bound would give up the instant it started.",
        mission_home_tol_m_, mission_return_max_sec_);
    throw std::runtime_error(
        "mission_home_tol_m and mission_return_max_sec must be positive");
  }
  // Defaulted ON: it closes a defect that split a fleet.
  // (notes: mission-start-hold-default-on)
  mission_start_hold_sec_ = dp("mission_start_hold_sec", 60.0);
  if (!std::isfinite(mission_start_hold_sec_) ||
      mission_start_hold_sec_ < 0.0) {
    RCLCPP_FATAL(get_logger(),
        "mission_start_hold_sec=%.3f is not a finite non-negative duration. "
        "A negative or NaN hold would compare false against every elapsed time "
        "and silently restore the generation-21 behaviour this parameter "
        "exists to replace — which is a fleet that can disperse mid-agreement "
        "and meet in two places.",
        mission_start_hold_sec_);
    throw std::runtime_error("mission_start_hold_sec must be finite and >= 0");
  }

  // Proximity stop is ON by default and not tied to coordination_enabled; it is
  // inert until it tracks a peer. With coordination_enabled=false the intent
  // heartbeat is off, so the guard needs the pose topics.
  // (notes: prox-stop-default-on-pose-topics)
  proximity_stop_enabled_ = dp("proximity_stop_enabled", true);
  ProximityGuard::Config pcfg;
  pcfg.enabled                = proximity_stop_enabled_;
  pcfg.hold_dist_m            = dp_f("proximity_hold_dist_m", 5.0);
  pcfg.resume_dist_m          = dp_f("proximity_resume_dist_m", 6.0);
  pcfg.pose_stale_sec         = dp_f("proximity_pose_stale_sec", 3.0);
  pcfg.peer_static_sec        = dp_f("proximity_peer_static_sec", 10.0);
  pcfg.parked_keep_dist_m     = dp_f("proximity_parked_keep_dist_m", 1.5);
  pcfg.peer_static_move_m     = dp_f("proximity_peer_static_move_m", 0.3);
  pcfg.hold_release_stale_sec = dp_f("proximity_hold_release_stale_sec", 10.0);
  pcfg.escape_grace_sec       = dp_f("proximity_escape_grace_sec", 30.0);
  proximity_max_hold_sec_ = dp("proximity_max_hold_sec", 120.0);
  if (pcfg.resume_dist_m < pcfg.hold_dist_m) {
    RCLCPP_WARN(get_logger(),
        "proximity_resume_dist_m=%.2f < proximity_hold_dist_m=%.2f inverts "
        "the hysteresis band — raising resume to %.2f.",
        pcfg.resume_dist_m, pcfg.hold_dist_m, pcfg.hold_dist_m);
  }
  prox_guard_ = std::make_unique<ProximityGuard>(pcfg, robot_name_);

  // Exploitation is phase 2 of gen 34 (DESIGN_gen34.md Q66). Read so a launch
  // file that turns it on is refused rather than silently ignored: an
  // exploit-on run waits for the port.
  exploitation_enabled_ = dp("exploitation_enabled", false);
  if (exploitation_enabled_) {
    RCLCPP_FATAL(get_logger(),
        "exploitation_enabled=true: the planner explores only (gen 34 "
        "phase 1, Q66); exploitation waits for the phase-2 port.");
    throw std::runtime_error(
        "exploitation_enabled=true is not supported by the gen-34 node");
  }

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
  // Fixed SCovox Beta EIG scorer — this node has no planner_type knob.
  score_fn_ = scoring::eig;
  logger_ = std::make_unique<MetricsLogger>(output_csv_);
  // Experiment event log. Constructed here, but run_start is NOT emitted yet:
  // under use_sim_time the clock reads 0 until the first /clock message, and
  // t0 must be a real sim time (see startExperimentLog).
  if (experiment_log_enabled_) {
    exp_log_ = std::make_unique<ExperimentLog>(
        experiment_log_path_, robot_name_, get_logger());
    if (!exp_log_->open()) {
      // The logger already emitted the ERROR with errno; say what it costs, so
      // an operator watching the console knows the run is scientifically
      // degraded before it burns an hour of battery.
      RCLCPP_ERROR(get_logger(),
          "Experiment event log DISABLED by open failure ('%s'). The CSV is "
          "unaffected, but this run will have no coverage milestones and no "
          "structured team events.", experiment_log_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
          "Experiment event log: %s (%zu coverage milestone(s); sim-clock "
          "stamped, flushed per event).",
          experiment_log_path_.c_str(), coverage_milestones_.size());
    }
    // Run parameters and binary provenance are recorded in the event log
    // itself. EXPLO_PLANNER_GIT_REV comes from explo_planner_git_rev.h,
    // regenerated on every build by cmake/StampGitRev.cmake.
    // (notes: explog-provenance-git-rev)
#ifdef EXPLO_PLANNER_GIT_REV
    exp_log_->addParamStr("git_rev", EXPLO_PLANNER_GIT_REV);
#else
    exp_log_->addParamStr("git_rev", "unknown");
#endif
    // Compile time of THIS translation unit: the one identifier that is always
    // exactly right about which binary is running.
    exp_log_->addParamStr("build_stamp",
                          std::string(__DATE__) + " " + __TIME__);
    exp_log_->addParamStr("node_name", std::string(this->get_name()));
    exp_log_->addParamStr("robot_name", robot_name_);
    // Fleet identity is logged as configured (team_robot_names) and as resolved
    // (robot_id, team_hash). robot_id -1 / team_hash 0 is the unconfigured run.
    // (notes: explog-fleet-identity-both-forms)
    exp_log_->addParamStr("team_robot_names", join(team_robot_names_, ","));
    exp_log_->addParamNum("robot_id", fleet_.self_id);
    exp_log_->addParamNum("team_hash", fleet_.team_hash);
    exp_log_->addParamBool("use_sim_time", this->get_parameter("use_sim_time")
                                               .as_bool());
    exp_log_->addParamStr("output_csv", output_csv_);
    exp_log_->addParamNum("max_steps", max_steps_);
    exp_log_->addParamNum("metrics_period_sec", metrics_period_sec_);
    exp_log_->addParamNum("metrics_max_duty", metrics_max_duty_);
    exp_log_->addParamNum("experiment_log_anchor_period_sec",
                          experiment_log_anchor_period_sec_);
    // Sensor model, logged from fcfg (what the evaluator was built with), not
    // the raw params, plus the derived fov_is_omnidirectional flag that decides
    // the arrival gate. (notes: explog-sensor-model-from-fcfg)
    exp_log_->addParamNum("fov_hfov", fcfg.hfov);
    exp_log_->addParamNum("fov_vfov", fcfg.vfov);
    exp_log_->addParamNum("fov_h_rays", fcfg.h_rays);
    exp_log_->addParamNum("fov_v_rays", fcfg.v_rays);
    exp_log_->addParamNum("fov_min_range", fcfg.min_range);
    exp_log_->addParamNum("fov_max_range", fcfg.max_range);
    exp_log_->addParamBool("fov_is_omnidirectional", fov_is_omnidirectional_);
    exp_log_->addParamNum("candidate_n_yaw", ccfg.n_yaw);
    // Logged beside candidate_n_yaw because n_yaw applies only to polar
    // candidates; with enable_polar=false the frontier path sets its own count
    // and ignores it. (notes: explog-polar-beside-n-yaw)
    exp_log_->addParamBool("candidate_enable_polar", ccfg.enable_polar);
    exp_log_->addParamNum("goal_xy_tolerance", goal_xy_tol_);
    exp_log_->addParamNum("goal_yaw_tolerance", goal_yaw_tol_);
    exp_log_->addParamBool("coordination_enabled", coord_enabled_);
    // Logged from separation_.config(), not the raw params: configure() refuses
    // an out-of-range weight and leaves the term off, so log the outcome, not
    // the request. (notes: explog-separation-from-config)
    exp_log_->addParamNum("separation_weight", separation_.config().weight);
    exp_log_->addParamNum("separation_radius_m", separation_.config().radius_m);
    exp_log_->addParamNum("separation_max_age_sec",
                          separation_.config().max_age_sec);
    // Allocator peer-position TTL, from the member the solve actually reads,
    // for the reason the separation block above gives: a NaN is coerced to 0
    // at load time, and recording the request rather than the outcome would
    // index a cell that ran unbounded as a treated one.
    exp_log_->addParamNum("alloc_peer_pos_max_age_sec",
                          alloc_peer_pos_max_age_sec_);
    // The 0-means-auto knobs are logged after their resolution above, so the
    // record holds the value used, not the sentinel. coord_claim_radius_m 0
    // resolves to fov_max_range. (notes: explog-auto-knobs-after-resolution)
    exp_log_->addParamNum("coord_claim_radius_m", coord_claim_radius_m_);
    exp_log_->addParamNum("cost_grid_radius_cap_m", cost_grid_radius_cap_m_);
    exp_log_->addParamNum("coord_claim_ttl_sec", coord_claim_ttl_sec_);
    exp_log_->addParamNum("coord_heartbeat_hz", coord_heartbeat_hz_);
    exp_log_->addParamNum("done_unknown_fraction", done_unknown_fraction_);
    exp_log_->addParamNum("done_min_consecutive_steps",
                          done_min_consecutive_steps_);
    exp_log_->addParamStr("done_coverage_source", done_coverage_source_);
    // Which rule ended the run. Recorded because it is not recoverable from any
    // other field, and a campaign that mixes the two criteria is comparing two
    // different endpoints under one column name.
    exp_log_->addParamStr("done_criterion", done_criterion_);
    exp_log_->addParamStr("done_action", done_action_);
    exp_log_->addParamNum("mission_home_tol_m", mission_home_tol_m_);
    exp_log_->addParamNum("mission_return_max_sec", mission_return_max_sec_);
    // Failed-goal blacklist and no-progress knobs, logged because the analysis
    // cannot infer them from the events.
    exp_log_->addParamNum("failed_goal_ttl_sec", failed_goal_ttl_sec_);
    exp_log_->addParamNum("failed_goal_radius_m", failed_goal_radius_m_);
    exp_log_->addParamNum("failed_goal_retire_after", failed_goal_retire_after_);
    exp_log_->addParamNum("progress_window_sec", progress_window_sec_);
    exp_log_->addParamNum("progress_min_distance_m", progress_min_distance_m_);
    // The thresholds every nav_goal_failed row is tested against.
    // nav_budget_sec is per-goal (navBudgetSec, from goal distance), so its
    // four inputs are logged instead. (notes: explog-nav-failure-thresholds)
    exp_log_->addParamNum("goal_rotate_timeout_sec", goal_rotate_timeout_sec_);
    exp_log_->addParamNum("nav_speed_estimate_mps", nav_speed_est_mps_);
    exp_log_->addParamNum("nav_safety_factor", nav_safety_factor_);
    exp_log_->addParamNum("nav_min_timeout_sec", nav_min_timeout_sec_);
    exp_log_->addParamNum("nav_max_timeout_sec", nav_max_timeout_sec_);
    exp_log_->addParamBool("exploitation_enabled", exploitation_enabled_);
    exp_log_->addParamBool("proximity_stop_enabled", proximity_stop_enabled_);
    exp_log_->addParamBool("terrain_relative_z", terrain_relative_z_);
    // From ccfg, not the roi_*_ members: those are cached from it further down
    // (with the ROS interfaces) and are still at their in-class defaults here.
    exp_log_->addParamNum("roi_min_x", ccfg.roi_min_x);
    exp_log_->addParamNum("roi_max_x", ccfg.roi_max_x);
    exp_log_->addParamNum("roi_min_y", ccfg.roi_min_y);
    exp_log_->addParamNum("roi_max_y", ccfg.roi_max_y);
  } else {
    RCLCPP_WARN(get_logger(),
        "Experiment event log DISABLED by parameter "
        "(experiment_log_enabled=false): no coverage milestones, no structured "
        "events. Only the per-step CSV will be written.");
  }
  cost_grid_ = std::make_unique<CostGrid>();
  // Bound peer-advertised claim radii by our own exploration disc — the largest
  // claim this planner considers legitimate. Resolved above, so the auto
  // (= fov_max_range) case is already a concrete number here.
  coord_ = std::make_unique<Coordination>(
      coord_enabled_, robot_name_,
      static_cast<float>(coord_claim_radius_m_),
      static_cast<float>(coord_claim_grace_sec_));

  // --- ROS interfaces ---
  std::string goal_topic = dp("goal_topic",
      std::string("/" + robot_name_ + "/goal_pose"));
  // The same OccupancyGrid the global planner consumes. We use it to
  // reject candidate viewpoints whose cell is occupied/inflated/unknown
  // before publishing them as goals.
  std::string planning_map_topic = dp("planning_map_topic",
      std::string("/" + robot_name_ + "/dscovox_node/planning_map"));
  // Master switch for the 2D planning_map, default off: no subscription,
  // straight-line costs, no free-cell or reachability filtering. When true the
  // map is a hard startup precondition. (notes: planning-map-param)
  use_planning_map_ = dp("use_planning_map", false);
  // Logged here because these params are read after the main param dump;
  // run_start is held until the first live-clock tick, so they still land in
  // the same row. (notes: explog-planning-map-config-late)
  if (exp_log_) {
    exp_log_->addParamBool("use_planning_map", use_planning_map_);
    exp_log_->addParamStr("planning_map_topic",
                          use_planning_map_ ? planning_map_topic
                                            : std::string("(unsubscribed)"));
  }

  // Cache ROI bounds for the fused-map ingest clip (loadLatestMap()).
  roi_min_x_ = ccfg.roi_min_x;
  roi_max_x_ = ccfg.roi_max_x;
  roi_min_y_ = ccfg.roi_min_y;
  roi_max_y_ = ccfg.roi_max_y;

  // --- Coarse cell world (P1) ---------------------------------------
  // The ROI diced into cells, each with a status from this robot's own map. Off
  // by default and not configured when off. The grid derives from the ROI so
  // the census covers exactly the ROI. (notes: cellworld-p1-coarse-grid)
  cell_world_enable_ = dp("cell_world_enable", false);
  if (cell_world_enable_) {
    // Cell ids are only comparable between robots that agree on the geometry,
    // so every input here is logged (below) and hashed into grid_hash.
    const double cell_size_m = dp("cell_size_m", 10.0);
    CellWorld::Config ccw;
    // World-calibrated status thresholds (see CellWorldConfig). The band from
    // covered_max_unknown to exploring_min_unknown is load-bearing hysteresis:
    // every status change resets the cell's known_by mask.
    // (notes: cellworld-status-thresholds-hysteresis)
    ccw.covered_max_unknown   = dp("cell_covered_max_unknown", 0.15);
    ccw.exploring_min_unknown = dp("cell_exploring_min_unknown", 0.35);
    ccw.covered_max_frontier_frac =
        dp("cell_covered_max_frontier_frac", 0.90);
    ccw.min_observed_columns  = dp("cell_min_observed_columns", 4);
    ccw.edge_max_blocked_fraction =
        dp("cell_edge_max_blocked_fraction", 0.5);
    const CellGrid grid = makeCellGrid(roi_min_x_, roi_max_x_,
                                       roi_min_y_, roi_max_y_,
                                       static_cast<float>(cell_size_m));
    // Requires a configured fleet identity: without one self_id is -1, every
    // known_by mask this robot sets would be empty, and the gate downstream
    // would compute a confident answer out of nothing. Refuse, don't degrade.
    const std::string err = cell_world_.configure(grid, ccw, fleet_.self_id);
    if (!err.empty()) {
      RCLCPP_FATAL(this->get_logger(), "cell_world: %s", err.c_str());
      throw std::runtime_error("cell_world: " + err);
    }
    cell_census_period_s_ = dp("cell_census_period_s", 5.0);
    publish_cell_markers_ = dp("publish_cell_markers", false);
    RCLCPP_INFO(this->get_logger(),
                "Cell world: %dx%d cells of %.2f m over ROI "
                "[%.1f,%.1f]x[%.1f,%.1f], grid_hash=0x%08x",
                grid.nx, grid.ny, static_cast<double>(grid.cell_size_m),
                static_cast<double>(roi_min_x_),
                static_cast<double>(roi_max_x_),
                static_cast<double>(roi_min_y_),
                static_cast<double>(roi_max_y_), grid.configHash());
    if (publish_cell_markers_) {
      cell_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          "~/cell_world", 10);
    }
  }
  if (exp_log_) {
    // Logged even when off, so a run record says the feature EXISTED and was
    // disabled — which is what distinguishes an equivalence-gate control run
    // from a run of the parent binary that never had the knob.
    exp_log_->addParamBool("cell_world_enable", cell_world_enable_);
    if (cell_world_enable_) {
      exp_log_->addParamNum("cell_size_m", cell_world_.grid().cell_size_m);
      exp_log_->addParamNum("cell_covered_max_unknown",
                            cell_world_.config().covered_max_unknown);
      exp_log_->addParamNum("cell_exploring_min_unknown",
                            cell_world_.config().exploring_min_unknown);
      exp_log_->addParamNum("cell_covered_max_frontier_frac",
                            cell_world_.config().covered_max_frontier_frac);
      exp_log_->addParamNum("cell_min_observed_columns",
                            cell_world_.config().min_observed_columns);
      exp_log_->addParamNum("cell_edge_max_blocked_fraction",
                            cell_world_.config().edge_max_blocked_fraction);
      exp_log_->addParamNum("cell_census_period_s", cell_census_period_s_);
      exp_log_->addParamBool("publish_cell_markers", publish_cell_markers_);
      // Derived, not a dp() param, but the one number that says whether two
      // robots' cell ids name the same ground. Kept beside the inputs it is
      // computed from so a mismatch can be traced to which input differed.
      exp_log_->addParamNum("cell_grid_hash", cell_world_.grid().configHash());
      exp_log_->addParamNum("cell_nx", cell_world_.grid().nx);
      exp_log_->addParamNum("cell_ny", cell_world_.grid().ny);
    }
  }

  // --- Team layer (gen 34, §8.7) -------------------------------------------
  // One beacon, always sent, replaces TeamWorld, the comms model and every
  // gen-33 reconnect switch; `arm` is the one knob that picks what the team
  // does about a gap. Beacons need fleet ids, so they are wired only when the
  // fleet is configured; without one the robot runs alone and TeamCore still
  // drives homing and DONE.
  //
  // Separation reads peer positions only from beacons: with no fleet there is
  // nobody to be repelled by, and the term would silently reproduce the
  // untreated planner. Fatal, as in gen 33.
  // (notes: separation-requires-teamworld)
  if (separation_.enabled() && !fleet_.configured) {
    const std::string e =
        "separation_weight > 0 requires team_robot_names: peer positions "
        "reach the planner only through the team beacon, which needs fleet "
        "ids, so the separation term would run with no teammate to be "
        "repelled by and silently reproduce the untreated planner.";
    RCLCPP_FATAL(get_logger(), "%s", e.c_str());
    throw std::runtime_error(e);
  }
  // mTARE's local-priority rule for the cell census merge, as in gen 33.
  team_merge_local_priority_ = dp("cell_merge_local_priority", true);

  {
    const std::string arm_s = dp("arm", std::string("off"));
    if (!gen34::parseArm(arm_s, &arm_)) {
      RCLCPP_FATAL(get_logger(),
          "arm='%s' is not one of off, pursuit, rendezvous, hybrid.",
          arm_s.c_str());
      throw std::runtime_error("unknown arm '" + arm_s + "'");
    }
  }
  if (arm_ != gen34::Arm::kOff) {
    const std::string idw = requireFleetIdentity(fleet_, "arm");
    if (!idw.empty()) {
      RCLCPP_FATAL(get_logger(), "%s", idw.c_str());
      throw std::runtime_error(idw);
    }
    if (fleet_.size() < 2) {
      RCLCPP_FATAL(get_logger(),
          "arm=%s needs a team of at least two robots; team_robot_names "
          "names %d.", gen34::armName(arm_), fleet_.size());
      throw std::runtime_error("arm needs a team of two or more");
    }
  }

  // Beacon stream, split like the intents: publish on one topic, subscribe to
  // the emulator's per-link relays. Both unset is the shared bus, where no
  // external process can gate the stream and every peer reads present.
  beacon_hz_ = dp("beacon_hz", 1.0);
  beacon_pub_topic_ =
      resolve_topic(dp("team_beacon_pub_topic", std::string("")));
  if (beacon_pub_topic_.empty())
    beacon_pub_topic_ = "/exploration/team_beacon";
  beacon_sub_topics_ =
      dp("team_beacon_sub_topics", std::vector<std::string>{});
  for (auto& t : beacon_sub_topics_) t = resolve_topic(t);
  beacon_sub_topics_.erase(
      std::remove(beacon_sub_topics_.begin(), beacon_sub_topics_.end(), ""),
      beacon_sub_topics_.end());
  if (beacon_sub_topics_.empty())
    beacon_sub_topics_ = {beacon_pub_topic_};

  team_cfg_.arm                        = arm_;
  team_cfg_.presence_window_sec        = dp("presence_window_sec", 10.0);
  team_cfg_.exchange_stall_sec         = dp("exchange_stall_sec", 120.0);
  team_cfg_.exchange_total_sec         = dp("exchange_total_sec", 600.0);
  team_cfg_.chase_silence_sec          = dp("chase_silence_sec", 90.0);
  team_cfg_.chase_cooldown_sec         = dp("chase_cooldown_sec", 90.0);
  team_cfg_.chase_max_contact_age_sec  = dp("chase_max_contact_age_sec", 900.0);
  team_cfg_.chase_limit_sec            = dp("chase_limit_sec", 600.0);
  team_cfg_.chase_departure_margin_sec = dp("chase_departure_margin_sec", 120.0);
  team_cfg_.follow_distance_m          = dp("follow_distance_m", 10.0);
  team_cfg_.follow_min_distance_m      = dp("follow_min_distance_m", 6.0);
  team_cfg_.meeting_interval_sec       = dp("meeting_interval_sec", 300.0);
  team_cfg_.meeting_window_sec         = dp("meeting_window_sec", 120.0);
  team_cfg_.meeting_patience_sec       = dp("meeting_patience_sec", 120.0);
  team_cfg_.meeting_backstop_sec       = dp("meeting_backstop_sec", 600.0);
  team_cfg_.meeting_ring_m             = dp("meeting_ring_m", 3.0);
  team_cfg_.meeting_arrive_m           = dp("meeting_arrive_m", 1.0);
  team_cfg_.travel_speed_mps           = dp("travel_speed_mps", 0.40);
  team_cfg_.depart_safety              = dp("depart_safety", 1.2);
  team_cfg_.finished_missed_slots      = dp("finished_missed_slots", 2);
  team_cfg_.leg_arrive_m               = dp("leg_arrive_m", 1.5);
  tick_event_period_sec_               = dp("tick_event_period_sec", 2.0);
  {
    // Every duration and distance here bounds a wait or places a robot; zero,
    // negative or NaN would end a wait at once, never end it, or stack robots.
    const std::pair<const char*, double> positive[] = {
        {"beacon_hz", beacon_hz_},
        {"presence_window_sec", team_cfg_.presence_window_sec},
        {"exchange_stall_sec", team_cfg_.exchange_stall_sec},
        {"exchange_total_sec", team_cfg_.exchange_total_sec},
        {"chase_silence_sec", team_cfg_.chase_silence_sec},
        {"chase_cooldown_sec", team_cfg_.chase_cooldown_sec},
        {"chase_max_contact_age_sec", team_cfg_.chase_max_contact_age_sec},
        {"chase_limit_sec", team_cfg_.chase_limit_sec},
        {"follow_distance_m", team_cfg_.follow_distance_m},
        {"follow_min_distance_m", team_cfg_.follow_min_distance_m},
        {"meeting_interval_sec", team_cfg_.meeting_interval_sec},
        {"meeting_window_sec", team_cfg_.meeting_window_sec},
        {"meeting_patience_sec", team_cfg_.meeting_patience_sec},
        {"meeting_backstop_sec", team_cfg_.meeting_backstop_sec},
        {"meeting_ring_m", team_cfg_.meeting_ring_m},
        {"meeting_arrive_m", team_cfg_.meeting_arrive_m},
        {"travel_speed_mps", team_cfg_.travel_speed_mps},
        {"depart_safety", team_cfg_.depart_safety},
        {"leg_arrive_m", team_cfg_.leg_arrive_m},
    };
    for (const auto& kv : positive) {
      if (!std::isfinite(kv.second) || kv.second <= 0.0) {
        RCLCPP_FATAL(get_logger(),
            "%s=%.3f must be finite and positive.", kv.first, kv.second);
        throw std::runtime_error(std::string(kv.first) +
                                 " must be finite and positive");
      }
    }
    if (!std::isfinite(team_cfg_.chase_departure_margin_sec) ||
        team_cfg_.chase_departure_margin_sec < 0.0) {
      RCLCPP_FATAL(get_logger(),
          "chase_departure_margin_sec=%.3f must be finite and >= 0.",
          team_cfg_.chase_departure_margin_sec);
      throw std::runtime_error("chase_departure_margin_sec must be >= 0");
    }
    if (team_cfg_.follow_min_distance_m > team_cfg_.follow_distance_m) {
      RCLCPP_FATAL(get_logger(),
          "follow_min_distance_m=%.2f exceeds follow_distance_m=%.2f: the "
          "follow point would sit inside the distance the follower backs off "
          "from.", team_cfg_.follow_min_distance_m,
          team_cfg_.follow_distance_m);
      throw std::runtime_error(
          "follow_min_distance_m must not exceed follow_distance_m");
    }
    if (team_cfg_.finished_missed_slots < 1) {
      RCLCPP_FATAL(get_logger(),
          "finished_missed_slots=%d must be at least 1.",
          team_cfg_.finished_missed_slots);
      throw std::runtime_error("finished_missed_slots must be >= 1");
    }
  }
  // A Leg's time bound is the exploration drive budget's formula: the same
  // safety factor over the straight-line time, plus a fixed slack.
  team_cfg_.leg_budget_factor = nav_safety_factor_;
  team_cfg_.home_tol_m        = mission_home_tol_m_;
  team_cfg_.home_max_sec      = mission_return_max_sec_;
  // The Leg watchdog is the exploration drive's, same pair of knobs.
  leg_cfg_.progress_window_sec     = progress_window_sec_;
  leg_cfg_.progress_min_distance_m = progress_min_distance_m_;

  team_cfg_.self_id   = fleet_.configured ? fleet_.self_id : 0;
  team_cfg_.n         = fleet_.configured ? fleet_.size() : 1;
  team_cfg_.team_hash = fleet_.team_hash;
  team_cfg_.grid_hash =
      cell_world_.configured() ? cell_world_.grid().configHash() : 0u;

  if (exp_log_) {
    exp_log_->setSchemaVersion(ExperimentLog::kGen34SchemaVersion);
    exp_log_->addParamStr("arm", gen34::armName(arm_));
    exp_log_->addParamNum("expected_peers", expectedPeers());
    exp_log_->addParamBool("cell_merge_local_priority",
                           team_merge_local_priority_);
    exp_log_->addParamNum("beacon_hz", beacon_hz_);
    exp_log_->addParamStr("team_beacon_pub_topic", beacon_pub_topic_);
    exp_log_->addParamStr("team_beacon_sub_topics",
                          join(beacon_sub_topics_, ","));
    exp_log_->addParamNum("presence_window_sec", team_cfg_.presence_window_sec);
    exp_log_->addParamNum("exchange_stall_sec", team_cfg_.exchange_stall_sec);
    exp_log_->addParamNum("exchange_total_sec", team_cfg_.exchange_total_sec);
    exp_log_->addParamNum("chase_silence_sec", team_cfg_.chase_silence_sec);
    exp_log_->addParamNum("chase_cooldown_sec", team_cfg_.chase_cooldown_sec);
    exp_log_->addParamNum("chase_max_contact_age_sec",
                          team_cfg_.chase_max_contact_age_sec);
    exp_log_->addParamNum("chase_limit_sec", team_cfg_.chase_limit_sec);
    exp_log_->addParamNum("chase_departure_margin_sec",
                          team_cfg_.chase_departure_margin_sec);
    exp_log_->addParamNum("follow_distance_m", team_cfg_.follow_distance_m);
    exp_log_->addParamNum("follow_min_distance_m",
                          team_cfg_.follow_min_distance_m);
    exp_log_->addParamNum("meeting_interval_sec",
                          team_cfg_.meeting_interval_sec);
    exp_log_->addParamNum("meeting_window_sec", team_cfg_.meeting_window_sec);
    exp_log_->addParamNum("meeting_patience_sec",
                          team_cfg_.meeting_patience_sec);
    exp_log_->addParamNum("meeting_backstop_sec",
                          team_cfg_.meeting_backstop_sec);
    exp_log_->addParamNum("meeting_ring_m", team_cfg_.meeting_ring_m);
    exp_log_->addParamNum("meeting_arrive_m", team_cfg_.meeting_arrive_m);
    exp_log_->addParamNum("travel_speed_mps", team_cfg_.travel_speed_mps);
    exp_log_->addParamNum("depart_safety", team_cfg_.depart_safety);
    exp_log_->addParamNum("finished_missed_slots",
                          team_cfg_.finished_missed_slots);
    exp_log_->addParamNum("leg_arrive_m", team_cfg_.leg_arrive_m);
    exp_log_->addParamNum("leg_budget_factor", team_cfg_.leg_budget_factor);
    exp_log_->addParamNum("leg_budget_slack_sec",
                          team_cfg_.leg_budget_slack_sec);
    exp_log_->addParamNum("tick_event_period_sec", tick_event_period_sec_);
  }

  // ---- Global allocator (P3) -----------------------------------------
  //
  // The first phase in which the cell world DECIDES something. Off by default;
  // when off, doPlan takes exactly the path it took before this block existed.
  global_alloc_enable_ = dp("global_alloc_enable", false);
  if (global_alloc_enable_) {
    if (!cell_world_enable_) {
      RCLCPP_FATAL(get_logger(),
          "global_alloc_enable=true but cell_world_enable=false: the cell "
          "world IS the allocation problem, so there would be nothing to "
          "solve.");
      throw std::runtime_error("global_alloc_enable requires cell_world_enable");
    }
    // A single-robot fleet has nothing to allocate against, and the identity
    // is what supplies the ids. Refused rather than degraded for the reason in
    // the member comment: a fleet of one solves cleanly and logs cleanly.
    const std::string idw = requireFleetIdentity(fleet_, "global_alloc_enable");
    if (!idw.empty()) {
      RCLCPP_FATAL(get_logger(), "%s", idw.c_str());
      throw std::runtime_error(idw);
    }
    // global_alloc_comms_mask is a separate knob, default off: it is the P4
    // reconnection-value machinery, not part of P3. See
    // GlobalAllocator::Config::comms_mask.
    // (notes: alloc-comms-mask-separate-knob)
    alloc_cfg_.comms_mask     = dp("global_alloc_comms_mask", false);
    alloc_cfg_.polish_passes  = dp("global_alloc_polish_passes", 2);
    alloc_cfg_.max_candidates = dp("global_alloc_max_candidates", 256);
    alloc_focus_skip_k_       = dp("global_alloc_focus_skip_k", 3);
    if (alloc_focus_skip_k_ < 1) {
      RCLCPP_WARN(get_logger(),
          "global_alloc_focus_skip_k=%d is below 1; a cell would be demoted "
          "the first tick it produced no candidate, which every cell does on "
          "the tick it is first assigned. Clamping to 1.",
          alloc_focus_skip_k_);
      alloc_focus_skip_k_ = 1;
    }
    alloc_focus_skips_.assign(static_cast<size_t>(cell_world_.size()), 0);
  }
  if (exp_log_) {
    exp_log_->addParamBool("global_alloc_enable", global_alloc_enable_);
    if (global_alloc_enable_) {
      exp_log_->addParamBool("global_alloc_comms_mask", alloc_cfg_.comms_mask);
      exp_log_->addParamNum("global_alloc_polish_passes",
                            alloc_cfg_.polish_passes);
      exp_log_->addParamNum("global_alloc_max_candidates",
                            alloc_cfg_.max_candidates);
      exp_log_->addParamNum("global_alloc_focus_skip_k", alloc_focus_skip_k_);
    }
  }

  // --- Intercept predictor (P6) --------------------------------------------
  // Read in every arm, so a run record carries the same keys whatever the
  // arm; consulted only by a chase (pursuit, hybrid) with a cell world.
  // The speed the prediction assumes is deliberately its own knob, not
  // nav_speed_est_mps_ (conservative; the budget still uses it).
  // (notes: pursuit-mdp-speed-not-nav-est)
  pursuit_mdp_cfg_.my_speed_mps = dp("pursuit_mdp_speed_mps", 0.40);
  if (!(pursuit_mdp_cfg_.my_speed_mps > 0.0)) {
    RCLCPP_FATAL(get_logger(),
        "pursuit_mdp_speed_mps=%.3f: every travel time would be infinite and "
        "there is no safe value to substitute.",
        pursuit_mdp_cfg_.my_speed_mps);
    throw std::runtime_error("pursuit_mdp_speed_mps must be positive");
  }
  if (std::fabs(pursuit_mdp_cfg_.my_speed_mps - team_cfg_.travel_speed_mps) >
      1e-6) {
    RCLCPP_WARN(get_logger(),
        "pursuit_mdp_speed_mps=%.3f but travel_speed_mps=%.3f: the same robot "
        "is modelled at two speeds. Deliberate is fine — sharing the mistake "
        "is what this warns about.",
        pursuit_mdp_cfg_.my_speed_mps, team_cfg_.travel_speed_mps);
  }
  pursuit_mdp_peer_speed_mps_ = dp("pursuit_mdp_peer_speed_mps", -1.0);
  pursuit_mdp_cfg_.peer_speed_mps = pursuit_mdp_peer_speed_mps_ > 0.0
      ? pursuit_mdp_peer_speed_mps_
      : pursuit_mdp_cfg_.my_speed_mps;
  pursuit_mdp_cfg_.dwell_sec = dp("pursuit_mdp_dwell_sec", 45.0);
  pursuit_mdp_cfg_.step_sec  = dp("pursuit_mdp_step_sec", 5.0);
  pursuit_mdp_cfg_.offroute_half_life_sec =
      dp("pursuit_mdp_offroute_half_life_sec", 180.0);
  pursuit_mdp_cfg_.max_horizon_sec = dp("pursuit_mdp_max_horizon_sec", 600.0);
  pursuit_mdp_cfg_.min_probability =
      dp("pursuit_mdp_min_probability", 0.10);
  if (!(pursuit_mdp_cfg_.step_sec > 0.0)) {
    RCLCPP_FATAL(get_logger(),
        "pursuit_mdp_step_sec=%.3f: the chain cannot advance.",
        pursuit_mdp_cfg_.step_sec);
    throw std::runtime_error("pursuit_mdp_step_sec must be positive");
  }
  // A zero floor is a diagnostic setting (every chase MDP-aimed), not a
  // campaign one: it removes the fall-through to the trail.
  if (pursuit_mdp_cfg_.min_probability <= 0.0) {
    RCLCPP_WARN(get_logger(),
        "pursuit_mdp_min_probability=%.3f: the model will aim every chase, "
        "including ones with no information behind them. The fall-through to "
        "the trail is what bounds this arm's downside.",
        pursuit_mdp_cfg_.min_probability);
  }
  if (exp_log_) {
    exp_log_->addParamNum("pursuit_mdp_speed_mps",
                          pursuit_mdp_cfg_.my_speed_mps);
    exp_log_->addParamNum("pursuit_mdp_peer_speed_mps",
                          pursuit_mdp_peer_speed_mps_);
    exp_log_->addParamNum("pursuit_mdp_dwell_sec", pursuit_mdp_cfg_.dwell_sec);
    exp_log_->addParamNum("pursuit_mdp_step_sec", pursuit_mdp_cfg_.step_sec);
    exp_log_->addParamNum("pursuit_mdp_offroute_half_life_sec",
                          pursuit_mdp_cfg_.offroute_half_life_sec);
    exp_log_->addParamNum("pursuit_mdp_max_horizon_sec",
                          pursuit_mdp_cfg_.max_horizon_sec);
    exp_log_->addParamNum("pursuit_mdp_min_probability",
                          pursuit_mdp_cfg_.min_probability);
  }

  // Without the allocator and a cell world the plan falls back to the team's
  // centroid, the value gate fails open (every chase is worth it) and the
  // intercept predictor has no tours. Legal, but not the arm as designed.
  if ((gen34::armBooks(arm_) || gen34::armChases(arm_)) &&
      !(global_alloc_enable_ && cell_world_.configured())) {
    RCLCPP_WARN(get_logger(),
        "arm=%s without global_alloc_enable and cell_world_enable: meeting "
        "cells fall back to the team centroid, the chase value gate fails "
        "open and the intercept predictor is unused.",
        gen34::armName(arm_));
  }

  oracle_ = std::make_unique<NodeOracle>(this);
  team_   = std::make_unique<gen34::TeamCore>(team_cfg_, oracle_.get());
  leg_    = gen34::LegTracker(leg_cfg_);
  RCLCPP_INFO(get_logger(),
      "Team layer: arm=%s, robot %d of %d, presence window %.0fs, meetings "
      "every %.0fs, homing within %.1f m or %.0fs.",
      gen34::armName(arm_), team_cfg_.self_id, team_cfg_.n,
      team_cfg_.presence_window_sec, team_cfg_.meeting_interval_sec,
      team_cfg_.home_tol_m, team_cfg_.home_max_sec);

  // The whole fused map from dscovox, subscribed with its latched QoS
  // (KeepLast(1) reliable + transient_local) so the current map arrives on
  // connect. Planning uses the ROI-clipped copy in map_cache_.
  // (notes: dscovox-fused-map-latched-sub)
  std::string dscovox_topic = dp("dscovox_topic", std::string(""));
  if (dscovox_topic.empty())
    dscovox_topic = "/" + robot_name_ + "/dscovox_node/scovox";
  scovox_map_sub_ = create_subscription<scovox_msgs::msg::ScovoxMap>(
      dscovox_topic, latchedQos(),
      [this](scovox_msgs::msg::ScovoxMap::SharedPtr msg) {
        onScovoxMap(msg);
      });
  RCLCPP_INFO(get_logger(), "Subscribing to fused map (dscovox): %s",
      dscovox_topic.c_str());

  // Per-source fusion counters (whose voxels arrive), latched like the map.
  // Fleet-gated: peer_fusion_deltas_ is indexed by fleet id and sized on the
  // first sample, so empty means no sample yet.
  // (notes: dscovox-fusion-counters-fleet-gated)
  if (fleet_.configured) {
    std::string counters_topic = dp("dscovox_counters_topic", std::string(""));
    if (counters_topic.empty())
      counters_topic = "/" + robot_name_ + "/dscovox_node/fusion_counters";
    counters_topic_ = counters_topic;
    fusion_counters_sub_ =
        create_subscription<scovox_msgs::msg::ScovoxFusionCounters>(
            counters_topic, latchedQos(),
            [this](scovox_msgs::msg::ScovoxFusionCounters::SharedPtr msg) {
              onFusionCounters(msg);
            });
    RCLCPP_INFO(get_logger(),
        "Subscribing to per-source fusion counters (dscovox): %s",
        counters_topic.c_str());
  }

  // Subscribe only when use_planning_map_ is set. Otherwise latest_plan_map_
  // stays null all run and every use site, all guarded on it, takes its
  // map-less path. (notes: planning-map-sub-only-when-enabled)
  if (use_planning_map_) {
    plan_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        planning_map_topic, latchedQos(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          // Warn once if the grid's frame is not map_frame_:
          // planMapCellAt/isCellFree/unknownFractionInRoi index it by raw world
          // XY with no TF, so it is only correct under an identity map->odom.
          // (notes: planning-map-frame-check)
          if (!msg->header.frame_id.empty() &&
              msg->header.frame_id != map_frame_) {
            RCLCPP_WARN_ONCE(get_logger(),
                "planning_map is in frame '%s' but the planner works in '%s'. "
                "2D cell lookups apply NO transform, so this is only correct "
                "while the two frames are numerically identical (e.g. an "
                "identity map->odom). Candidate free/occupied and reachability "
                "results are unreliable otherwise.",
                msg->header.frame_id.c_str(), map_frame_.c_str());
          }
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
        coord_intent_pub_topic_, qos);
    auto on_intent =
        [this](explo_planner_msgs::msg::RobotIntent::SharedPtr msg) {
          if (coord_) coord_->onIntent(*msg, this->now());
          // Event log: this is the ONE place a teammate broadcast is received,
          // in every arm and whether or not coordination consumes it, so it is
          // where peer_seen / peer_lost belief is maintained from. Self-echo is
          // filtered here as onIntent does it internally.
          if (msg->robot_id != robot_name_) expPeerHeard(msg->robot_id);
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
        };
    for (const auto& topic : coord_intent_sub_topics_) {
      intent_subs_.push_back(
          create_subscription<explo_planner_msgs::msg::RobotIntent>(
              topic, qos, on_intent));
    }
    // Logged unconditionally: a split-stream run that silently fell back to
    // the shared bus looks exactly like a run with perfect comms, and the
    // per-link relay topics are the first thing to check when every peer
    // reads present (leak) or absent (typo / QoS mismatch) for a whole run.
    const std::string subs = join(coord_intent_sub_topics_, ", ");
    RCLCPP_INFO(get_logger(),
        "Intents: pub '%s' <- KeepLast(8).reliable() -> sub [%s]%s",
        coord_intent_pub_topic_.c_str(), subs.c_str(),
        (coord_intent_sub_topics_.size() == 1 &&
         coord_intent_sub_topics_[0] == coord_intent_pub_topic_)
            ? " (shared bus: no external process can gate this stream)" : "");
  }

  // --- Team beacon pub/sub (§8.2). Wired whenever the fleet is configured,
  //     in every arm: presence, positions, exchange progress and the census
  //     all ride it, and a finished robot keeps sending it from DONE.
  if (fleet_.configured) {
    // KeepLast(2): each beacon is full state, so a deeper queue only delivers
    // stale ones. Two, not one, so a beacon arriving during a planning pass
    // survives until the copy-only callback runs.
    // (notes: ctor-teamworld-qos-keeplast-two)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    beacon_pub_ = create_publisher<explo_planner_msgs::msg::TeamBeacon>(
        beacon_pub_topic_, qos);
    auto on_beacon =
        [this](explo_planner_msgs::msg::TeamBeacon::SharedPtr msg) {
          // COPY ONLY: the checks, the merge and TeamCore run in drainBeacons
          // on the tick, one pending slot per sender.
          const int sid = static_cast<int>(msg->robot_id);
          // Our own beacon looped back by a shared bus.
          if (sid == fleet_.self_id) return;
          if (sid < 0 || sid >= fleet_.size()) return;
          auto& slot = beacon_pending_[sid];
          if (slot.msg) {
            ++beacons_superseded_;
            // Best effort can reorder: of two in one tick, the older says
            // nothing new, and TeamCore would take it as the latest.
            if (rclcpp::Time(msg->header.stamp).nanoseconds() <
                rclcpp::Time(slot.msg->header.stamp).nanoseconds())
              return;
          }
          slot.msg      = msg;
          slot.received = this->now();
        };
    for (const auto& topic : beacon_sub_topics_) {
      beacon_subs_.push_back(
          create_subscription<explo_planner_msgs::msg::TeamBeacon>(
              topic, qos, on_beacon));
    }
    // Own timer, on the SIM clock: presence is judged in sim seconds, and a
    // busy planning tick must not silence the robot.
    // (notes: ctor-teamworld-sim-clock-timer)
    const std::chrono::duration<double> period(1.0 / beacon_hz_);
    beacon_timer_ = rclcpp::create_timer(
        this, get_clock(), period, [this]() { publishBeacon(); });
    const std::string subs = join(beacon_sub_topics_, ", ");
    RCLCPP_INFO(get_logger(),
        "Beacon: %.2f Hz, pub '%s' -> sub [%s]%s",
        beacon_hz_, beacon_pub_topic_.c_str(), subs.c_str(),
        (beacon_sub_topics_.size() == 1 &&
         beacon_sub_topics_[0] == beacon_pub_topic_)
            ? " (shared bus: no external process can gate this stream, and "
              "every peer reads present)" : "");
  } else {
    RCLCPP_INFO(get_logger(),
        "Beacon: off (no team_robot_names). This robot runs alone.");
  }

  // --- Proximity-stop wiring: peer localiser poses + the (inert) nav2 cancel
  // client.
  if (proximity_stop_enabled_) {
    // Entries are <robot_name>:<topic> peer localiser poses (map frame, ~10
    // Hz), fresher than the 1 Hz intent heartbeat. With none configured the
    // guard runs on intents alone. (notes: ctor-proxstop-peer-pose-topics)
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

    // Dead path: simple_nav_3d serves no NavigateToPose action, so the
    // readiness guard means the cancel is never sent and the brake goal is the
    // only stop. Never used to send goals; goal_pose is the only command path.
    // (notes: ctor-proxstop-dead-nav-cancel)
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
        "proximity_hold_state", latchedQos());
    publishProxState("clear");
  }

  // --- State machine timer (10 Hz, sim time) ---
  tick_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(100),
      [this] { tick(); });

  // --- Periodic CSV sampler (sim time, same clock as the state machine).
  //     Shares the node's default (mutually-exclusive) callback group with
  //     tick(), so a row can never be assembled from half-updated state.
  if (metrics_period_sec_ > 0.0) {
    const std::chrono::duration<double> period(metrics_period_sec_);
    metrics_timer_ = rclcpp::create_timer(
        this, get_clock(), period, [this] { metricsTick(); });
    RCLCPP_INFO(get_logger(),
        "Metrics: sampling every %.1f s in ALL states (rows where state != "
        "LOG_STEP); end-of-step rows unchanged.", metrics_period_sec_);
  } else {
    RCLCPP_WARN(get_logger(),
        "Metrics: periodic sampling DISABLED (metrics_period_sec=0) — the CSV "
        "will have no rows during team activities.");
  }

  // --- Coord heartbeat. Re-publishes the active claim while NAVIGATE-ing
  //     so peers don't lose it through TTL. Period = 1 / coord_heartbeat_hz.
  if (coord_enabled_ && coord_heartbeat_hz_ > 0.0) {
    const std::chrono::duration<double> period(1.0 / coord_heartbeat_hz_);
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
        // simple_nav_3d's UGV arrival test is XY-only, but its UAV role
        // measures 3D distance, so on a UAV a wrong goal z is a wrong goal.
        // (notes: ctor-startup-log-goal-z-roles)
        flatten_goal_z_ ? "flattened to 0 (strict-2D nav)"
                        : "3D (ignored by the UGV arrival test, USED by the UAV's)");
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

void ExploPlannerNode::onFusionCounters(
    const scovox_msgs::msg::ScovoxFusionCounters::SharedPtr& msg) {
  // The arrays are parallel by contract (see ScovoxFusionCounters.msg). A
  // length mismatch is a producer that has been changed out from under this
  // node, and reading past the shorter one would attribute a peer's traffic to
  // whichever robot happened to sort next.
  if (msg->source_frame.size() != msg->deltas_received.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "Fusion counters: %zu source_frame vs %zu deltas_received — parallel "
        "arrays disagree, ignoring this sample.",
        msg->source_frame.size(), msg->deltas_received.size());
    return;
  }
  // Sized on first receipt (even from a sample naming no sources), not at
  // construction: empty means unmeasured, zeros mean measured and zero,
  // non-zero means a peer sent data.
  // (notes: fusion-counters-sized-first-receipt)
  if (peer_fusion_deltas_.size() != static_cast<size_t>(fleet_.size()))
    peer_fusion_deltas_.assign(static_cast<size_t>(fleet_.size()), 0);
  for (size_t i = 0; i < msg->source_frame.size(); ++i) {
    // "<robot>/odom" is what simple_nav_3d stamps as the integration frame and
    // what dscovox files its per-source grids under. The emulator relays the
    // serialized bytes without deserializing, so this still names the robot
    // that SENSED the voxels even when another robot bridged them.
    const std::string& frame = msg->source_frame[i];
    const size_t slash = frame.find('/');
    const int id = fleet_.idOf(
        slash == std::string::npos ? frame : frame.substr(0, slash));
    // idOf returns -1 for a producer the fleet list does not name. Counting it
    // is not possible (there is no index) and it is not an error either: a
    // robot outside the configured team can legitimately be on the bus.
    if (id < 0 || id >= static_cast<int>(peer_fusion_deltas_.size())) continue;
    const uint64_t total = msg->deltas_received[i];
    // A decrease means dscovox restarted: take the new value as the baseline
    // (do not subtract or keep the old one) and warn.
    // (notes: fusion-counters-decrease-rebaseline)
    if (total < peer_fusion_deltas_[static_cast<size_t>(id)]) {
      RCLCPP_WARN(get_logger(),
          "Fusion counters: source '%s' went backwards (%llu -> %llu) — "
          "dscovox restarted. Re-baselining; deltas across the restart are "
          "unmeasured.",
          frame.c_str(),
          static_cast<unsigned long long>(
              peer_fusion_deltas_[static_cast<size_t>(id)]),
          static_cast<unsigned long long>(total));
    }
    peer_fusion_deltas_[static_cast<size_t>(id)] = total;
  }

  // Map-exchange progress (Q51): newest seq and gaps per fleet id, read by
  // TeamCore's exchange test and carried on the beacon. The arrays are
  // parallel to source_frame; a dscovox that predates them sends them empty,
  // and then no exchange can see progress (each gives up on its stall bound).
  if (msg->newest_seq.size() == msg->source_frame.size() &&
      msg->seq_gaps.size() == msg->source_frame.size()) {
    const size_t n = static_cast<size_t>(fleet_.size());
    if (seq_newest_.size() != n) seq_newest_.assign(n, 0);
    if (seq_gaps_.size() != n) seq_gaps_.assign(n, 0);
    for (size_t i = 0; i < msg->source_frame.size(); ++i) {
      const std::string& frame = msg->source_frame[i];
      const size_t slash = frame.find('/');
      const int id = fleet_.idOf(
          slash == std::string::npos ? frame : frame.substr(0, slash));
      if (id < 0 || static_cast<size_t>(id) >= n) continue;
      seq_newest_[static_cast<size_t>(id)] = msg->newest_seq[i];
      seq_gaps_[static_cast<size_t>(id)]   = msg->seq_gaps[i];
    }
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "Fusion counters carry no map sequence numbers (newest_seq %zu, "
        "seq_gaps %zu, sources %zu): this dscovox predates gen 34, so no map "
        "exchange can see progress.",
        msg->newest_seq.size(), msg->seq_gaps.size(),
        msg->source_frame.size());
  }
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
    // Clip the whole fused map to the ROI so map_cache_ (walked by frontier
    // extraction and doLogStep) stays bounded. The [band_lo, band_hi] z-band is
    // also what FovEvaluator clips rays to; doPlan re-syncs it from
    // eff_roi_*_z_. (notes: loadmap-roi-clip-zband)
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
  // Event log first: run_start must be the file's first line and needs a live
  // clock. The peer sweep runs here, not in the coordination heartbeat, so
  // outage timing is recorded with coordination off.
  // (notes: tick-event-log-first)
  startExperimentLog();
  expClockAnchorTick();
  expPeerSweep();

  updatePoseFromTF();
  trackDistance();

  // Beacons, then TeamCore: after the pose (the local-priority merge anchors
  // on the current cell, and TeamCore reads the pose) and before the dispatch
  // and the proximity-hold return, so a held robot keeps tracking its team.
  // (notes: tick-drain-teamworld-order)
  drainBeacons();
  teamTick();

  // Any tick outside PLAN ends an all-rejected episode. Clear it here, not only
  // on goal selection: doPlan has other exits and the team states leave PLAN.
  // (notes: tick-all-rejected-reset-off-plan)
  if (state_ != State::PLAN) {
    consecutive_all_rejected_ = 0;
  }

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

  // Q65: two robots both in Meet drive to their own spots on the ring and do
  // not hold for each other. Set every tick, so the exemption ends on the
  // tick either robot leaves Meet.
  if (prox_guard_) {
    std::vector<std::string> exempt;
    if (team_ && fleet_.configured) {
      for (int j = 0; j < fleet_.size(); ++j)
        if (j != fleet_.self_id && team_->proximityExempt(j))
          exempt.push_back(fleet_.nameOf(j));
    }
    prox_guard_->setExempt(std::move(exempt));
  }

  // TeamCore's activity picks the state before the dispatch.
  applyActivity();

  // Coordinated proximity stop: checked every tick before the state dispatch so
  // the hold pre-empts every drive, an exploration goal or a moving Leg. A Leg
  // that has arrived, the standing activities and the exploration machine's
  // stationary states are already still.
  // (notes: tick-proximity-hold-driving-states)
  if ((state_ == State::NAVIGATE ||
       (isLegState(state_) && leg_.active())) &&
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
      // use_planning_map_ true: planning_map is a hard precondition. False: not
      // subscribed; start once fused map and pose are ready and run map-less
      // (straight-line costs, no 2D filtering).
      // (notes: tick-wait-planning-map-handling)
      {
        // Pre-mission hold, ANDed with the preconditions so release is at
        // max(preconditions, hold). Timed on missionElapsed(), each robot's own
        // baseline; -1 before the clock is live, which holds.
        // (notes: tick-wait-pre-mission-hold)
        const double held_for = missionElapsed();
        const bool hold_done  = held_for >= mission_start_hold_sec_;
        const bool preconds_met = have_map_ && have_pose_ &&
                           (!use_planning_map_ || have_plan_map_);
        const bool start = preconds_met && hold_done;
        if (!hold_done) {
          // Deliberately its own line rather than another missing precondition
          // in the WARN below. Holding is the design working; the WARN names
          // faults, and a 60 s wait appearing there every 5 s would read as a
          // stuck startup in exactly the logs someone scans for one.
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 15000,
              "Pre-mission hold: %.0fs of %.0fs. Not planning or navigating "
              "yet — the team agrees its meeting plan while it is still "
              "co-located. Applied in every arm so the cost is shared.",
              std::max(0.0, held_for), mission_start_hold_sec_);
        } else if (!mission_start_hold_logged_ &&
                   mission_start_hold_sec_ > 0.0) {
          mission_start_hold_logged_ = true;
          RCLCPP_INFO(get_logger(),
              "Pre-mission hold complete at t+%.0fs (configured %.0fs).",
              held_for, mission_start_hold_sec_);
        }
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
          transitionTo(State::PLAN, "startup-preconditions-met");
        } else if (!preconds_met) {
          // Names the missing precondition. Gated on !preconds_met, not !start:
          // the pre-mission hold is not a fault and has its own log lines.
          // (notes: tick-wait-start-warn-gating)
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

    case State::PROXIMITY_HOLD:
      doProximityHold();
      break;

    case State::MEET:
    case State::CHASE:
    case State::FOLLOW:
    case State::WAIT:
    case State::RETURN_HOME:
      doTeamState();
      break;

    case State::DONE:
      // The node never shuts down (rule 6): DONE stands still and the beacon
      // keeps going, so a finished robot stays countable to its team.
      doTeamState();
      RCLCPP_INFO_ONCE(get_logger(),
          "Mission done [%s]: idling and beaconing. This node stays up.",
          done_reason_.c_str());
      break;
  }
}

void ExploPlannerNode::transitionTo(State s, const char* reason) {
  const State from = state_;
  // Emitted before the new state is installed, so from_dwell_sec measures the
  // state being left. -1 on the first transition, where state_enter_time_ is
  // still system-clock and subtracting would throw.
  // (notes: transition-state-change-dwell-event)
  if (exp_log_) {
    exp_log_->logStateChange(
        expCtx(), stateName(from), stateName(s), reason,
        have_state_enter_ ? (this->now() - state_enter_time_).seconds() : -1.0);
  }

  state_ = s;
  state_enter_time_ = this->now();
  have_state_enter_ = true;
  // DONE's reason is kept for the run_end the destructor writes: the node
  // stays up in DONE, so DONE is not the end of the file.
  // (notes: transition-done-run-end-placement)
  if (s == State::DONE) done_reason_ = reason;
  // The post-arrival rotation deadline is per-NAVIGATE-cycle and is armed
  // lazily on arrival at the XY goal; disarm it on every entry.
  if (s == State::NAVIGATE) rotate_deadline_armed_ = false;
}

// ==================================================================
// Team layer (gen 34): TeamCore's activity drives the state machine
// ==================================================================

// One TeamCore tick on this tick's pose, finished latch and map sequence
// numbers, then its events into the log. Runs from the first pose on, in every
// state, so presence, exchange and the plan keep time while the robot explores.
void ExploPlannerNode::teamTick() {
  // have_home_ is the one-shot "a pose has ever arrived" latch. After that a
  // lost pose still ticks, with have_pose false, so the team's clocks run.
  if (!team_ || !have_home_) return;
  const auto now = this->now();
  if (now.nanoseconds() <= 0) return;

  gen34::TickInputs in;
  in.now       = now.seconds();
  in.pose      = gen34::Vec2{latest_pos_.x(), latest_pos_.y()};
  in.have_pose = have_pose_;
  in.finished  = finished_;
  const size_t n = static_cast<size_t>(team_cfg_.n);
  in.rcvd_seq.assign(n, 0);
  in.seq_gaps.assign(n, 0);
  for (size_t j = 0; j < n && j < seq_newest_.size(); ++j)
    in.rcvd_seq[j] = seq_newest_[j];
  for (size_t j = 0; j < n && j < seq_gaps_.size(); ++j)
    in.seq_gaps[j] = seq_gaps_[j];
  const int self = team_cfg_.self_id;
  in.sent_seq = (self >= 0 && static_cast<size_t>(self) < seq_newest_.size())
                    ? seq_newest_[static_cast<size_t>(self)] : 0;
  // Without a counters sample every seq reads 0, and TeamCore would take
  // 0 >= 0 for a finished swap; it starts no exchange instead. Said out loud
  // after a grace, so a wrong counters topic or a silent dscovox is not a run
  // with no exchanges and no sign of why.
  in.seq_valid = !seq_newest_.empty();
  if (in.seq_valid || team_cfg_.n < 2) {   // alone, there is nothing to swap
    seq_wait_since_ = -1.0;
  } else {
    if (seq_wait_since_ < 0.0) seq_wait_since_ = in.now;
    if (in.now - seq_wait_since_ > 20.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "No map sequence numbers from the fusion counters for %.0f s "
          "(topic '%s'): map exchanges wait for them.",
          in.now - seq_wait_since_,
          counters_topic_.empty() ? "<not subscribed>" : counters_topic_.c_str());
    }
  }
  in.home      = gen34::Vec2{home_pos_.x(), home_pos_.y()};
  in.have_home = have_home_;

  team_out_    = team_->tick(in);
  team_ticked_ = true;

  for (const gen34::TeamEvent& ev : team_->drainEvents()) {
    // The homing result rides run_end as well as its own event: it is the
    // mission endpoint's outcome, and a run killed mid-homing must read "".
    if (ev.kind == "homing") {
      const LogField* a = findField(ev.fields, "action");
      const std::string action = a ? a->s : "";
      if (action == "start") {
        mission_home_result_.clear();
        mission_home_sim_sec_ = -1.0;
      } else if (action == "arrived" || action == "gave_up" ||
                 action == "no-home") {
        mission_home_result_ = (action == "gave_up") ? "timeout" : action;
        mission_home_sim_sec_ = in.now;
        RCLCPP_INFO(get_logger(),
            "Homing resolved [%s] at t_sim=%.1f, %.2f m from home.",
            mission_home_result_.c_str(), in.now,
            (latest_pos_ - home_pos_).head<2>().norm());
      }
    }
    if (exp_log_) exp_log_->logTeamEvent(expCtx(), ev.kind, ev.fields);
  }

  if (exp_log_ && tick_event_period_sec_ > 0.0 &&
      in.now >= next_tick_event_sec_) {
    logTickEvent(in.now);
    next_tick_event_sec_ = in.now + tick_event_period_sec_;
  }
}

// The periodic tick event (K20): what the robot is doing, how long it may go
// on, and how close its nearest teammate is.
void ExploPlannerNode::logTickEvent(double now_sec) {
  (void)now_sec;
  TickEvent e;
  e.x = latest_pos_.x();
  e.y = latest_pos_.y();
  e.activity = gen34::activityName(team_out_.activity);
  switch (team_out_.drive.kind) {
    case gen34::DriveKind::kExplore: e.drive = "explore"; break;
    case gen34::DriveKind::kHold:    e.drive = "hold";    break;
    case gen34::DriveKind::kLeg:     e.drive = "leg";     break;
  }
  if (team_out_.drive.kind == gen34::DriveKind::kLeg)
    e.purpose = team_out_.drive.purpose;
  e.leg_status = gen34::LegTracker::statusName(leg_.status());
  double best = std::numeric_limits<double>::infinity();
  uint32_t contact = 0;
  for (int j = 0; j < team_cfg_.n; ++j) {
    if (j == team_cfg_.self_id) continue;
    if (team_->inContact(j)) contact |= (1u << j);
    gen34::Vec2 p;
    double age = -1.0;
    if (!team_->lastHeardPosition(j, &p, &age)) continue;
    const double d = std::hypot(p.x - latest_pos_.x(), p.y - latest_pos_.y());
    if (d < best) {
      best = d;
      e.nearest_peer = j;
      e.nearest_peer_m = d;
      e.nearest_peer_age_sec = age;
    }
  }
  e.wait_start_sec = team_out_.wait_start;
  e.wait_bound_sec = team_out_.wait_bound;
  e.all_connected  = team_->allConnected();
  e.contact_mask   = contact;
  e.booking_slot   = team_->booking().held ? team_->booking().slot : -1;
  e.proximity_hold = (state_ == State::PROXIMITY_HOLD);
  exp_log_->logTick(expCtx(), e);
}

State ExploPlannerNode::stateForActivity(gen34::Activity a) {
  switch (a) {
    case gen34::Activity::kExplore: return State::PLAN;
    case gen34::Activity::kMeet:    return State::MEET;
    case gen34::Activity::kChase:   return State::CHASE;
    case gen34::Activity::kFollow:  return State::FOLLOW;
    case gen34::Activity::kWait:    return State::WAIT;
    case gen34::Activity::kHome:    return State::RETURN_HOME;
    case gen34::Activity::kDone:    return State::DONE;
  }
  return State::PLAN;
}

// TeamCore's activity picks the state (§8.4). Explore hands the robot to the
// exploration machine, which runs its own PLAN -> NAVIGATE -> INTEGRATE ->
// LOG_STEP loop as gen 33 did; every other activity is one state. A step
// already in INTEGRATE or LOG_STEP finishes first (two seconds at most), so a
// step is never half-logged.
void ExploPlannerNode::applyActivity() {
  if (!team_ticked_ || state_ == State::WAIT_FOR_MAP) return;
  const State want = stateForActivity(team_out_.activity);
  const bool holding = (state_ == State::PROXIMITY_HOLD);
  const State cur = holding ? prox_resume_state_ : state_;
  const bool in_explore = cur == State::PLAN || cur == State::NAVIGATE ||
                          cur == State::INTEGRATE || cur == State::LOG_STEP;

  if (want == State::PLAN) {
    if (in_explore) return;
    if (holding) leaveProximityHold("team-explore");
    // Leaving a Leg: the navigator drives its last goal until told otherwise
    // (see abandonNavGoal), and PLAN may take a few ticks to publish the next.
    if (isLegState(cur)) abandonNavGoal("team-explore");
    leg_.reset();
    transitionTo(State::PLAN, "team-explore");
    return;
  }

  if (cur == want) return;
  if (in_explore) {
    if (cur == State::INTEGRATE || cur == State::LOG_STEP) return;
    // The exploration claim is released: a peer may take the goal this robot
    // is leaving.
    have_active_intent_ = false;
  }
  const char* act = gen34::activityName(team_out_.activity);
  if (holding) leaveProximityHold(act);
  leg_.reset();
  hold_braked_ = false;
  char reason[32];
  std::snprintf(reason, sizeof(reason), "team-%s", act);
  // The navigator drives its last goal until told otherwise. With a live pose
  // the new state replaces it this tick (a Leg goal, or the hold's brake); a
  // Leg without a pose publishes nothing, so the old goal is stopped here.
  if (!have_pose_ && have_home_ && isLegState(want) &&
      (cur == State::NAVIGATE || isLegState(cur)))
    abandonNavGoal(reason);
  transitionTo(want, reason);
  // run_end's reason: how the mission ended, not the transition tag.
  if (want == State::DONE && !mission_home_result_.empty())
    done_reason_ = "home-" + mission_home_result_;
}

// A PROXIMITY_HOLD that the team activity ends: close the hold's books the way
// doProximityHold does on a release.
void ExploPlannerNode::leaveProximityHold(const char* why) {
  const double held = (this->now() - state_enter_time_).seconds();
  prox_hold_total_sec_ += held;
  RCLCPP_INFO(get_logger(),
      "Proximity hold ended after %.1fs by the team activity [%s].", held, why);
  char buf[160];
  std::snprintf(buf, sizeof(buf), "clear reason=activity held=%.1f", held);
  publishProxState(buf);
}

// Every team state runs TeamCore's drive: a Leg through the tracker, or stand
// still. The drive may change kind inside one state (a Meet that holds on
// contact), so this reads the drive, not the state.
void ExploPlannerNode::doTeamState() {
  const gen34::Drive& d = team_out_.drive;
  if (d.kind == gen34::DriveKind::kLeg) {
    hold_braked_ = false;
    if (!have_pose_) return;
    const gen34::Vec2 pose{latest_pos_.x(), latest_pos_.y()};
    const gen34::LegTracker::Command cmd = leg_.update(
        this->now().seconds(), pose, d,
        [this](const gen34::Vec2& from, const gen34::Vec2& toward,
               gen34::Vec2* out) { return pickEscape(from, toward, out); });
    if (cmd.publish) {
      publishLegGoal(cmd.goal);
    } else if (cmd.brake) {
      brakeHere();
    } else if (leg_.active()) {
      republishGoal(leg_goal_);  // throttled keep-alive
    }
    return;
  }
  // Hold (Explore cannot reach here: applyActivity moves it to PLAN first).
  // One brake goal; the navigator keeps it.
  leg_.reset();
  if (!hold_braked_ && have_home_) {
    brakeHere();
    hold_braked_ = true;
  }
}

void ExploPlannerNode::publishLegGoal(const gen34::Vec2& p) {
  CandidateViewpoint vp;
  vp.position = Eigen::Vector3f(static_cast<float>(p.x),
                                static_cast<float>(p.y), latest_pos_.z());
  // Face the direction of travel; a point underfoot keeps the current heading.
  const double dx = p.x - latest_pos_.x();
  const double dy = p.y - latest_pos_.y();
  vp.yaw = (std::hypot(dx, dy) < 0.1) ? latest_yaw_
                                      : static_cast<float>(std::atan2(dy, dx));
  leg_goal_ = vp;
  publishGoal(vp);
}

void ExploPlannerNode::brakeHere() {
  CandidateViewpoint brake;
  brake.position = latest_pos_;
  brake.yaw = latest_yaw_;
  publishGoal(brake);
}

// A free, visible point 1.5-6 m away for a Leg's escape move: gen 33's
// fallback, 2.5 m behind the robot at a rotating offset, generalised to five
// offsets and four radii and checked against the plan map. With nothing free
// it still answers with the plain fallback, as gen 33 did.
bool ExploPlannerNode::pickEscape(const gen34::Vec2& from,
                                  const gen34::Vec2& /*toward*/,
                                  gen34::Vec2* out) {
  static constexpr double kOffsetRad[5] = {0.0, 1.047, -1.047, 2.094, -2.094};
  static constexpr double kRadiusM[4] = {2.5, 1.5, 4.0, 6.0};
  const double base = static_cast<double>(latest_yaw_) + M_PI;
  const int start = escape_rot_ % 5;
  escape_rot_ = (escape_rot_ + 1) % 5;
  for (int k = 0; k < 5; ++k) {
    const double a = base + kOffsetRad[(start + k) % 5];
    for (double r : kRadiusM) {
      const gen34::Vec2 p{from.x + r * std::cos(a), from.y + r * std::sin(a)};
      if (oracleStandable(p) && oracleLineOfSight(from, p)) {
        *out = p;
        return true;
      }
    }
  }
  const double a = base + kOffsetRad[start];
  *out = gen34::Vec2{from.x + 2.5 * std::cos(a), from.y + 2.5 * std::sin(a)};
  return true;
}

// Exploration is finished, latched (K4): the coverage criterion or the step
// budget. The beacon's `finished` and TeamCore's priority list read it; it is
// never cleared.
void ExploPlannerNode::latchFinished(const char* reason) {
  if (finished_) return;
  finished_        = true;
  finished_reason_ = reason;
  recordExplorationComplete(reason);
}

int ExploPlannerNode::presentPeerCount() const {
  if (!team_ || !fleet_.configured) return 0;
  int k = 0;
  for (int j = 0; j < fleet_.size(); ++j)
    if (j != fleet_.self_id && team_->present(j)) ++k;
  return k;
}

int ExploPlannerNode::expectedPeers() const {
  return fleet_.configured ? fleet_.size() - 1 : 0;
}

// Vehicle set: self first-hand at (x, y), peers at their last DIRECTLY heard
// beacon position (K2: no gossip, no relay). Only the allocator passes
// pos_max_age_sec; the value gate and the plan solve pass 0 (unbounded).
// (notes: alloc-vehicles-vehicle-set)
std::vector<AllocRobot> ExploPlannerNode::allocVehicles(
    float x, float y, bool* all_in_comms,
    double /*now_sec*/, double pos_max_age_sec) const {
  std::vector<AllocRobot> out;
  if (all_in_comms) *all_in_comms = true;
  if (!cell_world_.configured() || !team_) return out;

  const CellGrid& g = cell_world_.grid();
  for (int id = 0; id < fleet_.size(); ++id) {
    AllocRobot r;
    r.id = id;
    if (id == fleet_.self_id) {
      r.cell = g.idAt(x, y);
      r.in_comms = true;
      r.finished = false;   // we are planning, so we are not done
      // And still on the frontier: this is only ever asked to decide where
      // THIS robot works next.
      r.off_frontier = false;
    } else {
      gen34::Vec2 p;
      double age = -1.0;
      const bool heard = team_->lastHeardPosition(id, &p, &age);
      // A missing or expired position gives cell -1: the robot drops out and
      // its cells return to the pool. It stays in the list, still counting
      // toward all_in_comms and fleet size.
      // (notes: alloc-vehicles-unlocatable-cell)
      r.cell = (heard && allocPeerPositionFresh(age, pos_max_age_sec))
                   ? g.idAt(static_cast<float>(p.x), static_cast<float>(p.y))
                   : -1;
      r.in_comms = team_->present(id);
      const gen34::PeerRecord& pr = team_->peer(id);
      r.finished = pr.ever && pr.b.finished;
      // Homing or done is off the frontier too; `finished` is ORed in because
      // a finished robot may still be meeting (Meet or Wait).
      r.off_frontier = r.finished ||
          (pr.ever && (pr.b.activity == gen34::Activity::kHome ||
                       pr.b.activity == gen34::Activity::kDone));
      // all_in_comms stays on `finished` alone: a robot driving home out of
      // contact is a real break in the team.
      if (!r.in_comms && !r.finished && all_in_comms) *all_in_comms = false;
    }
    out.push_back(r);
  }
  return out;
}

std::vector<SeparationTerm::Anchor> ExploPlannerNode::separationAnchors(
    double /*now_sec*/) const {
  std::vector<SeparationTerm::Anchor> out;
  if (!team_ || !fleet_.configured) return out;

  for (int id = 0; id < fleet_.size(); ++id) {
    if (id == fleet_.self_id) continue;
    gen34::Vec2 p;
    double age = -1.0;
    if (!team_->lastHeardPosition(id, &p, &age)) continue;
    // Finished teammates are excluded, not down-weighted: they clear no ground.
    // (notes: separation-anchors-skip-finished)
    const gen34::PeerRecord& pr = team_->peer(id);
    if (pr.ever && pr.b.finished) continue;
    if (!separation_.eligibleAnchor(age)) continue;
    out.push_back(SeparationTerm::Anchor{static_cast<float>(p.x),
                                         static_cast<float>(p.y)});
  }
  return out;
}

// ==================================================================
// PLAN state
// ==================================================================

void ExploPlannerNode::doPlan() {
  // A finished robot explores no more (K4). doPlan is reached after the latch
  // only for the tick or two before TeamCore's activity moves the state on.
  if (finished_) return;
  // Step budget: never start an exploration step past max_steps_. doLogStep
  // latches it at the step that spends it; this catches any other way into
  // PLAN. (notes: plan-step-budget-guard)
  if (step_ >= max_steps_) {
    // Measure coverage fresh for the event. Keep the two statements: nested,
    // argument order is unsequenced and budget_src can be read before it is
    // filled. (notes: plan-step-budget-coverage-sample)
    const char* budget_src = "";
    const double budget_unk = coverageUnknownFraction(&budget_src);
    noteCoverageDecisionSample(budget_unk, budget_src);
    latchFinished("step-budget");
    return;
  }

  auto plan_start = this->now();

  failed_goals_.prune(plan_start.seconds(), failed_goal_ttl_sec_);
  visited_goals_.prune(plan_start.seconds(), visited_goal_ttl_sec_);
  if (coord_) coord_->prune(plan_start);

  phase_ = Phase::EXPLORE;

  // Rebuild the local map_cache_ from the latest fused map received on the
  // topic (ROI-clipped) so frontier extraction + FOV scoring below run on
  // fresh consensus data.
  if (!loadLatestMap()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No fused map received yet on the dscovox topic; retrying next tick.");
    return;
  }

  // Checked before candidate generation; streak mode needs N consecutive
  // low-unknown ticks. Under latch it repeats the metrics-tick check so it
  // works with the sampler off; maybeLatchCoverageDone is idempotent.
  // (notes: plan-coverage-saturation-check)
  if (done_criterion_ == "latch") {
    if (done_unknown_fraction_ > 0.0) {
      const char* cov_src = "";
      const double unk = coverageUnknownFraction(&cov_src);
      if (maybeLatchCoverageDone(unk, cov_src)) return;
      if (unk < 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
            "Coverage termination (done_unknown_fraction=%.3f) INACTIVE: "
            "source '%s' cannot measure the ROI unknown fraction (no "
            "planning_map / degenerate ROI); stopping only at max_steps=%d.",
            done_unknown_fraction_, cov_src, max_steps_);
      }
    }
  } else if (done_unknown_fraction_ > 0.0) {
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
        // Record the fraction this decision was taken on, not the last sampler
        // tick's. (notes: plan-streak-decision-sample)
        noteCoverageDecisionSample(unk, cov_src);
        latchFinished("coverage-saturated");
        return;
      }
    } else {
      coverage_done_streak_ = 0;
      if (unk < 0.0) {
        // -1: the selected source cannot measure (planning_map source with none
        // published, or a degenerate ROI box; auto only on a bad ROI). Warn
        // rather than silently rely on max_steps.
        // (notes: plan-coverage-source-unmeasurable)
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

  // Terrain mode: loadLatestMap() may re-band the map z-slab, so keep the FOV
  // ray z-clip and candidate z clamp on that band; out-of-band rays and
  // candidates score the maximal Beta(1,1) prior.
  // (notes: plan-terrain-roi-z-lockstep)
  if (terrain_relative_z_) {
    fov_eval_->setRoiZ(eff_roi_min_z_, eff_roi_max_z_);
    candidate_gen_->setRoiZ(eff_roi_min_z_, eff_roi_max_z_);
  }

  // Terrain mode passes map_cache_ so candidates snap to ground + clearance
  // with a 3D occupancy check. Flat mode passes nullptr: only the 2D
  // planning_map filter, when enabled, checks occupancy.
  // (notes: plan-candidate-generation)
  const MapCache* terrain_map =
      terrain_relative_z_ ? map_cache_.get() : nullptr;
  auto candidates =
      candidate_gen_->generate(robot_pos, robot_yaw, terrain_map);
  size_t n_radial = candidates.size();
  // Frontier search band. Defaults to the effective ROI band (unchanged
  // behaviour); frontier_z_lo_/hi_ narrow it, and can only ever narrow it.
  float f_lo = eff_roi_min_z_, f_hi = eff_roi_max_z_;
  if (frontier_band_set_) {
    // Offsets ride the ROI band so terrain_relative_z keeps working: in flat
    // mode eff_roi_* are the absolute yaml values and these are absolute
    // heights; in terrain mode eff_roi_* track the robot and so does this.
    f_lo = std::max(f_lo, eff_roi_min_z_ + frontier_z_lo_off_);
    f_hi = std::min(f_hi, eff_roi_max_z_ - frontier_z_hi_off_);
    if (f_lo >= f_hi) {          // degenerate: fall back rather than find none
      f_lo = eff_roi_min_z_; f_hi = eff_roi_max_z_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "frontier_z_*_offset_m collapse the band to nothing — ignoring them "
          "for this tick and searching the full ROI band [%.2f, %.2f].",
          f_lo, f_hi);
    }
  }
  auto frontiers = map_cache_->findFrontierCentroids(
      f_lo, f_hi, frontier_cluster_radius_m_);
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

  // Evaluate per-candidate FOV info gain. With trajectory_scoring (default
  // false) info_gain sums EIG along the Dijkstra path; otherwise endpoint
  // scoring. Both run locally on map_cache_. (notes: plan-trajectory-scoring)
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

  // U = info_gain / (kCostEpsilon + path_cost)^kGamma, kGamma =
  // utility_cost_exponent_. kGamma == 1 skips std::pow to stay bit-identical to
  // SSMI. kCostEpsilon sits inside the power; inf cost gives U = -inf.
  // (notes: plan-utility-cost-exponent)
  constexpr float kCostEpsilon = 0.1f;
  // Hoisted out of the loop: γ is fixed for the life of the node, and the
  // equality test is on the same value the branch uses, so no candidate can
  // take a different path from its neighbour within one planning tick.
  const float kGamma = static_cast<float>(utility_cost_exponent_);
  const bool  kUnitGamma = (kGamma == 1.0f);

  // ---- Team-separation discount (separation.hpp) -----------------------
  // Built once per tick, before the utility loop; sep_anchors drives both the
  // discount and the sep_* columns, written in every arm. missionElapsed(), not
  // now(): peer positions carry mission-clock stamps.
  // (notes: sep-discount-anchors)
  const std::vector<SeparationTerm::Anchor> sep_anchors =
      separationAnchors(missionElapsed());
  // Kept per candidate so the SELECTED one's discount can be logged after the
  // walk below picks it — the walk can reject the top-scoring candidate for a
  // dozen reasons, so the discount that mattered is not knowable here.
  std::vector<float> sep_discount(candidates.size(), 1.0f);
  // The undiscounted utility, kept only to answer "would this tick have
  // preferred a different candidate without the term?" — the manipulation
  // check. Not used for any decision.
  std::vector<float> utility_undiscounted(candidates.size(), 0.0f);

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
        candidates[i].score = kUnitGamma
            ? info_gain[i] / (kCostEpsilon + c)
            : info_gain[i] / std::pow(kCostEpsilon + c, kGamma);
      } else {
        // Unreachable (inf cost): U = -inf, sorts to the bottom.
        candidates[i].score = -std::numeric_limits<float>::infinity();
      }
      utility_undiscounted[i] = candidates[i].score;
      // The discount scales U, not info_gain, so selected_info_gain stays the
      // FOV evaluator's value. Guarded by enabled() so the off configuration
      // executes no arithmetic at all. (notes: sep-discount-applied-to-utility)
      if (separation_.enabled()) {
        sep_discount[i] = separation_.discount(candidates[i].position.x(),
                                               candidates[i].position.y(),
                                               sep_anchors);
        candidates[i].score =
            SeparationTerm::apply(candidates[i].score, sep_discount[i]);
      }
    }
  }

  // ---- Global allocator focus (P3) ------------------------------------
  // Rank 0 = focus cell's 9-neighbourhood, k = k-th tour cell's, others last.
  // Primary sort key: ordering, not filtering, so the allocator cannot starve
  // doPlan. The 0 initialiser is inert with no tour. (notes: alloc-focus-rank)
  std::vector<int> alloc_rank(candidates.size(), 0);
  std::vector<AllocRobot> alloc_robots;
  Allocation alloc;
  AllocationEvent alloc_ev;
  bool alloc_ran = false;
  if (global_alloc_enable_ && cell_world_.configured()) {
    alloc_ran = true;
    const CellGrid& g = cell_world_.grid();
    const auto t_solve0 = std::chrono::steady_clock::now();

    bool all_in_comms = true;
    // The ONE call site that bounds peer-position staleness. A pose older than
    // the TTL stops holding cells here and they return to the pool; 0 (the
    // default) is unbounded and is the pre-TTL planner exactly.
    alloc_robots = allocVehicles(robot_pos.x(), robot_pos.y(), &all_in_comms,
                                 missionElapsed(),
                                 alloc_peer_pos_max_age_sec_);

    alloc = GlobalAllocator::solve(cell_world_, alloc_robots, alloc_cfg_);
    const double solve_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t_solve0).count();

    // Only frontiers are steered: the polar ring takes kAllocRankUnrestricted
    // with the frontiers no tour cell claimed and competes with them on
    // utility. Not rank 0: that band always wins. tour is copied.
    // (notes: alloc-ring-unrestricted-band)
    const int self_idx = fleet_.self_id;
    std::vector<int> tour;
    if (self_idx >= 0 && self_idx < static_cast<int>(alloc.tours.size()))
      tour = alloc.tours[static_cast<size_t>(self_idx)];
    // Broadcast copy, assigned on every solve including empty ones, so a
    // refused allocation clears the route on the air rather than leaving a
    // stale one. (notes: alloc-broadcast-tour-every-solve)
    my_tour_ = tour;
    bool reordered = false;
    if (!tour.empty()) {
      // cell id -> tour rank of the earliest tour cell whose neighbourhood
      // contains it. Built once per tick; the alternative is a linear scan of
      // the tour per candidate, which is the same work with worse constants.
      std::vector<int> rank_of_cell(static_cast<size_t>(cell_world_.size()),
                                    kAllocRankUnrestricted);
      for (size_t k = 0; k < tour.size(); ++k) {
        int nb[9];
        const int n = cell_world_.neighbourhood9(tour[k], nb);
        for (int i = 0; i < n; ++i) {
          int& slot = rank_of_cell[static_cast<size_t>(nb[i])];
          if (slot == kAllocRankUnrestricted) slot = static_cast<int>(k);
        }
      }
      for (size_t i = 0; i < candidates.size(); ++i) {
        if (!candidates[i].is_frontier) {
          alloc_rank[i] = kAllocRankUnrestricted;   // polar ring: unrestricted
          continue;
        }
        const int cid = g.idAt(candidates[i].position.x(),
                               candidates[i].position.y());
        alloc_rank[i] = g.valid(cid)
                            ? rank_of_cell[static_cast<size_t>(cid)]
                            // Off the cell grid entirely: outside every tour
                            // cell by definition, and the grid is derived from
                            // the ROI, so this is a candidate the mission does
                            // not want anyway.
                            : kAllocRankUnrestricted;
        if (alloc_rank[i] != 0) reordered = true;
      }
    }

    alloc_ev.shared_hash       = cell_world_.sharedHash();
    alloc_ev.grid_hash         = g.configHash();
    // Taken from the solve's own result, not recomputed here. A digest of the
    // inputs reassembled at the call site is a digest of a second thing
    // believed to be equal, and the drift between them would be invisible in
    // exactly the way this exists to make visible.
    alloc_ev.alloc_hash        = alloc.alloc_hash;
    alloc_ev.edge_hash         = alloc.edge_hash;
    for (int cid = 0; cid < cell_world_.size(); ++cid) {
      const CellStatus s = cell_world_.status(cid);
      if (s == CellStatus::EXPLORING || s == CellStatus::EXPLORING_BY_OTHERS)
        ++alloc_ev.candidates;
    }
    alloc_ev.unassigned        = static_cast<int>(alloc.unassigned.size());
    alloc_ev.refused           = alloc.refused;
    alloc_ev.solve_ms          = solve_ms;
    alloc_ev.focus_cell        = tour.empty() ? -1 : tour.front();
    alloc_ev.all_in_comms      = all_in_comms;
    alloc_ev.reordered         = reordered;
    // Mirrors the allocator's vehicle filter exactly, `off_frontier` included.
    // The column's only job is to say how many vehicles the solve was actually
    // given; a count that used a narrower test than the solve would report a
    // robot as present in a problem it had been dropped from.
    for (const AllocRobot& r : alloc_robots)
      if (r.cell >= 0 && !r.finished && !r.off_frontier)
        ++alloc_ev.robots_in_problem;
    for (int cid : tour)
      alloc_ev.tour += (alloc_ev.tour.empty() ? "" : ",") + std::to_string(cid);
    for (size_t i = 0; i < alloc_robots.size(); ++i) {
      if (static_cast<int>(i) == self_idx) continue;
      // Written even for a peer with no tour ("id:-1"), so an absent peer and
      // a peer allocated nothing do not render alike in the log.
      alloc_ev.peer_focus +=
          (alloc_ev.peer_focus.empty() ? "" : ",") +
          std::to_string(alloc_robots[i].id) + ":" +
          std::to_string(alloc.focusFor(alloc_robots[i].id, alloc_robots));
    }
    if (!alloc_ev.refused.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "Global allocator refused this tick (%s); planning unrestricted.",
          alloc_ev.refused.c_str());
    }
  }

  // Sort the selection order by utility descending, under the allocator's
  // tour rank when it is enabled (rank is 0 everywhere when it is not, so this
  // is the pre-P3 comparator exactly). The cost-grid reachability filter runs
  // after the sort as one of the candidate filters in the walk below.
  std::vector<size_t> order(candidates.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
      [&candidates, &alloc_rank](size_t a, size_t b) {
        // Tour rank first: the focus neighbourhood before the next tour cell's
        // before everything else. Equal ranks — which is every pair when the
        // allocator is off — fall through to the utility comparison unchanged.
        if (alloc_rank[a] != alloc_rank[b]) return alloc_rank[a] < alloc_rank[b];
        // NaN-safe descending order. A bare `>` is undefined behaviour for
        // std::sort if any score is NaN (breaks strict-weak-ordering); sort
        // NaNs to the bottom so a degenerate score can never corrupt `order`.
        const float sa = candidates[a].score;
        const float sb = candidates[b].score;
        if (std::isnan(sa)) return false;
        if (std::isnan(sb)) return true;
        return sa > sb;
      });

  bool found = false;
  size_t selected_idx = 0;

  // Rejection counters and suppressed lists, bundled so the walk can run a
  // second time (the separation counterfactual) without polluting the reported
  // numbers. suppressed holds blacklist-only rejections for the amnesty.
  // (notes: plan-walk-tally-and-amnesty)
  struct WalkTally {
    int too_close = 0;
    int map = 0;
    int unreachable = 0;
    int blacklist = 0;
    int minpos = 0;
    // `blacklist` is the union the per-step `blk` column has always reported;
    // `visited` is the recently-visited half of it, broken out because the two
    // halves get different treatment at the amnesty and a reader cannot tell
    // from the union which one starved the tick.
    int visited = 0;
    // Candidates rejected by the failed-goal blacklist, and by the visited
    // suppression, kept apart on purpose — see the rejection sites. The
    // amnesty consumes `suppressed` first and falls back to
    // `visited_suppressed` only when there is nothing else at all.
    std::vector<size_t> suppressed;
    std::vector<size_t> visited_suppressed;
  };

  // The filter chain shared by the real walk and the separation counterfactual.
  // Filters only read node state and write only t, and none looks at score, so
  // both walks see the same admissible set.
  // (notes: plan-admissible-filter-chain)
  const auto admissible = [&](size_t idx, WalkTally& t) -> bool {
    const auto& vp = candidates[idx];
    // 0. Skip candidates closer than max(goal_xy_tol_, cand_min_goal_dist_):
    // they waste a step, or sit too close to observe the frontier that
    // generated them. (notes: plan-filter-too-close)
    {
      const double near = std::max(goal_xy_tol_, cand_min_goal_dist_);
      float dx = vp.position.x() - robot_pos.x();
      float dy = vp.position.y() - robot_pos.y();
      if (dx * dx + dy * dy < static_cast<float>(near * near)) {
        ++t.too_close;
        return false;
      }
    }
    // 1. Single-cell check on the planning_map: frontier candidates may sit on
    // unknown (-1) cells but not occupied/inflated ones; others must be free.
    // Skipped with no planning_map. (notes: plan-filter-planning-map-cell)
    if (latest_plan_map_) {
      if (vp.is_frontier) {
        if (isCellOccupied(vp.position)) { ++t.map; return false; }
      } else {
        if (!isCellFree(vp.position)) { ++t.map; return false; }
      }
    }
    // 2. Cost-grid reachability — catches free pockets sealed off by
    //    inflated obstacles. Skipped when the flood barely reached
    //    anything (robot trapped in inflation zone).
    if (!skip_reachability && cost_grid_ &&
        !cost_grid_->reachable(vp.position)) {
      ++t.unreachable;
      return false;
    }
    // 3. Recently-visited suppression, then the failed-goal blacklist. Both
    // count as blk but go to separate lists, because the amnesty tries the
    // failed tier first and visited only as a last resort.
    // (notes: plan-filter-visited-vs-failed)
    if (visited_goal_radius_m_ > 0.0 &&
        visited_goals_.isNear(vp.position, visited_goal_radius_m_)) {
      ++t.blacklist;
      ++t.visited;
      t.visited_suppressed.push_back(idx);
      return false;
    }
    if (failed_goals_.isNear(vp.position, failed_goal_radius_m_)) {
      ++t.blacklist;
      t.suppressed.push_back(idx);
      return false;
    }
    // 4. MinPos peer-claim check, only with coordination enabled. claimMatching
    // gets plan_start so exploration contests LIVE claims only; graced exploit
    // claims do not count here. (notes: plan-filter-minpos-live-claims)
    if (coord_ && coord_->enabled()) {
      const auto* peer = coord_->claimMatching(
          vp.position, static_cast<float>(coord_claim_radius_m_),
          &plan_start);
      if (peer && !coord_->selfWinsAgainst(robot_pos, vp.position,
                                            *peer, robot_name_)) {
        ++t.minpos;
        return false;
      }
    }
    return true;
  };

  WalkTally tally;
  for (size_t idx : order) {
    if (!admissible(idx, tally)) continue;
    current_goal_ = candidates[idx];
    selected_idx = idx;
    found = true;
    break;
  }
  // This walk is the one whose rejections are real; the counterfactual's are
  // discarded. Aliased rather than copied so the amnesty block below, which
  // consumes the suppressed list, needs no change.
  const int rejected_too_close   = tally.too_close;
  const int rejected_map         = tally.map;
  const int rejected_unreachable = tally.unreachable;
  const int rejected_blacklist   = tally.blacklist;
  // A SUBSET of rejected_blacklist, not a peer of it — see the member doc.
  const int rejected_visited     = tally.visited;
  const int rejected_minpos      = tally.minpos;
  std::vector<size_t>& suppressed_only = tally.suppressed;
  std::vector<size_t>& visited_only    = tally.visited_suppressed;
  // Whether the pick came from the walk itself rather than from the amnesty
  // fallback below. sep_reordered is only defined for a walk pick — see there.
  const bool found_in_walk = found;
  // One amnesty for both suppression tiers. bl is the blacklist that suppressed
  // the pool and so orders and dates it; source is the log word for the tier
  // granting the reprieve. (notes: plan-amnesty-single-helper)
  const auto tryAmnesty = [&](std::vector<size_t>& pool,
                              FailedGoalBlacklist& bl, double radius_m,
                              const char* source, const char* prose,
                              const char* stamp_verb) {
    if (found || pool.empty()) return;
    // Retired last, then least-recently-failed (see amnestyOrderBefore for why
    // the retired partition has to be there). stable_sort so equal keys keep
    // the score order they inherited from `order`.
    std::stable_sort(pool.begin(), pool.end(), [&](size_t a, size_t b) {
      return amnestyOrderBefore(bl, candidates[a].position,
                                candidates[b].position, radius_m);
    });
    for (size_t idx : pool) {
      const auto& vp = candidates[idx];
      // Re-run the peer-claim check: blacklisted candidates were rejected
      // before MinPos ran, and skipping it would hand this robot a goal a peer
      // already claimed. (notes: plan-amnesty-recheck-minpos)
      if (coord_ && coord_->enabled()) {
        const auto* peer = coord_->claimMatching(
            vp.position, static_cast<float>(coord_claim_radius_m_),
            &plan_start);
        if (peer && !coord_->selfWinsAgainst(robot_pos, vp.position,
                                              *peer, robot_name_)) {
          continue;  // not counted again; it was already counted as blk
        }
      }
      const double last_fail = bl.lastFailTimeNear(vp.position, radius_m);
      const double age = std::isfinite(last_fail)
                             ? plan_start.seconds() - last_fail
                             : -1.0;
      // Only the failed-goal blacklist retires sites; visited_goals_ never
      // calls setRetireAfter, so this reads false on that tier by
      // construction rather than by accident. `source` is what tells the two
      // apart in the log — do not read retired=false as "the failed tier".
      const bool amnesty_retired = bl.isRetiredNear(vp.position, radius_m);
      current_goal_ = vp;
      selected_idx = idx;
      found = true;
      RCLCPP_WARN(get_logger(),
          "Step %d: all %zu candidates suppressed by the %s; AMNESTY "
          "re-attempt of the least-recently-%s%s goal (%.2f, %.2f), last %s "
          "%.1fs ago.",
          step_, candidates.size(), prose, stamp_verb,
          amnesty_retired ? " RETIRED" : "",
          vp.position.x(), vp.position.y(), stamp_verb, age);
      if (exp_log_) {
        exp_log_->logGoalAmnesty(expCtx(), vp.position.x(), vp.position.y(),
                                 age, amnesty_retired, source);
      }
      break;
    }
  };
  // Order is the behavioural contract: the failed tier runs first; the visited
  // tier only when it found nothing. Visited picks re-cover ground, so they
  // stay countable via source and plan_rej_visited.
  // (notes: plan-amnesty-tier-order)
  tryAmnesty(suppressed_only, failed_goals_, failed_goal_radius_m_, "failed",
             "failed-goal blacklist", "failed");
  tryAmnesty(visited_only, visited_goals_, visited_goal_radius_m_, "visited",
             "recently-visited suppression", "visited");

  // ---- Allocator bookkeeping (P3) -------------------------------------
  // Before the starvation return on purpose: a tick where every candidate was
  // rejected must still log the allocation.
  // (notes: alloc-bookkeeping-placement)
  if (alloc_ran) {
    alloc_ev.picked_rank = found ? alloc_rank[selected_idx] : -1;

    // alloc_focus_skips_ is per cell id and counts failed appearances as focus,
    // not consecutive ticks; no decay. !found neither advances nor resets it; a
    // rank-0 pick resets it, any other pick increments it.
    // (notes: alloc-focus-staleness-counter)
    const int focus = alloc_ev.focus_cell;
    if (focus >= 0 && focus < static_cast<int>(alloc_focus_skips_.size())) {
      if (found && alloc_ev.picked_rank == 0) {
        alloc_focus_skips_[focus] = 0;
      } else if (found) {
        ++alloc_focus_skips_[focus];
      }
      alloc_ev.focus_skips = alloc_focus_skips_[focus];

      if (alloc_focus_skips_[focus] >= alloc_focus_skip_k_) {
        // Re-measure before believing the counter. The census runs on its own
        // rate limiter, so the cell's observation can be a full
        // cell_census_period_s old, and demoting on stale evidence is exactly
        // how a cell that HAS been cleared gets written off.
        if (map_cache_) {
          cell_world_.applyObservation(
              censusFromMap(*map_cache_, cell_world_.grid()));
        }
        if (shouldDemoteStaleFocus(alloc_focus_skips_[focus],
                                   alloc_focus_skip_k_,
                                   cell_world_.status(focus))) {
          // A first-hand COVERED that no peer can lower again, so it is logged
          // on every fire. Requires the focus cell, k failed appearances as
          // focus, and a fresh census still reading it unexplored.
          // (notes: alloc-focus-demotion-one-way)
          cell_world_.commitSelf(focus, CellStatus::COVERED);
          alloc_ev.demoted = std::to_string(focus);
          RCLCPP_WARN(get_logger(),
              "Global allocator: focus cell %d produced no admissible "
              "candidate on %d consecutive solves that held it as focus, and "
              "still reads unexplored after a fresh census — demoting it to "
              "COVERED.",
              focus, alloc_focus_skips_[focus]);
        }
        alloc_focus_skips_[focus] = 0;
      }
    }
    if (exp_log_) exp_log_->logAllocation(expCtx(), alloc_ev);
  }

  // ---- Rejection profile of THIS attempt ------------------------------
  // Above the starvation return, so it covers starved and successful ticks
  // alike. (notes: rejection-profile-placement)
  pending_plan_cand_total_    = static_cast<int>(candidates.size());
  pending_plan_rej_close_     = rejected_too_close;
  pending_plan_rej_map_       = rejected_map;
  pending_plan_rej_unreach_   = rejected_unreachable;
  pending_plan_rej_blacklist_ = rejected_blacklist;
  pending_plan_rej_visited_   = rejected_visited;
  pending_plan_rej_minpos_    = rejected_minpos;

  if (!found) {
    // The WARN is throttled, so it carries consecutive_all_rejected_: the count
    // separates one unlucky tick from sustained starvation, which the throttle
    // alone would hide. (notes: plan-starvation-consecutive-count)
    ++consecutive_all_rejected_;
    // After the increment, so the row shows the stall length INCLUDING this
    // tick. The WARN below is throttled to one line per 5 s; this column is
    // not, which is the point — the throttle is what made the log an
    // unreliable place to measure stall length from.
    pending_plan_stall_ticks_ = consecutive_all_rejected_;
    // blk is the union and vis its visited half; reaching here means both
    // amnesty tiers declined. blk==vis means MinPos vetoed the whole visited
    // pool; blk>vis means real failed goals are in play.
    // (notes: plan-starvation-blk-vis-reading)
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Step %d: all %zu candidates rejected (close=%d map=%d unreach=%d "
        "blk=%d vis=%d minpos=%d). Retrying next tick; %d consecutive "
        "rejected ticks.",
        step_, candidates.size(), rejected_too_close,
        rejected_map, rejected_unreachable,
        rejected_blacklist, rejected_visited, rejected_minpos,
        consecutive_all_rejected_);
    return;  // stay in PLAN, retry next tick
  }
  // Reset only on a tick that actually selected a goal, so the counter measures
  // an unbroken starvation run rather than resetting on any code path that
  // happens to reach here.
  if (consecutive_all_rejected_ > 0) {
    RCLCPP_INFO(get_logger(),
        "Step %d: planning recovered after %d consecutive all-rejected ticks.",
        step_, consecutive_all_rejected_);
    consecutive_all_rejected_ = 0;
  }
  // Unconditional, not inside the branch above: a tick that selected a goal is
  // a tick with zero stall, whether or not it followed a stall. Setting this
  // only on recovery would leave the last stall's length standing on every
  // healthy row after it.
  pending_plan_stall_ticks_ = 0;

  // Drain utility / coord diagnostics into pending_* fields for the
  // upcoming LOG_STEP. doLogStep() will copy these into StepMetrics.
  pending_mean_info_gain_ =
      candidates.empty()
          ? 0.0f
          : sum_info / static_cast<float>(candidates.size());
  // Population std of info_gain over the same denominator as the mean. Two-pass
  // on purpose: the sum-of-squares shortcut cancels away most of the answer in
  // float. (notes: plan-info-gain-std-two-pass)
  {
    double ss = 0.0;
    const double mean = pending_mean_info_gain_;
    for (float g : info_gain) {
      const double d = static_cast<double>(g) - mean;
      ss += d * d;
    }
    pending_info_gain_std_ =
        info_gain.empty()
            ? 0.0f
            : static_cast<float>(std::sqrt(ss / static_cast<double>(
                                                    info_gain.size())));
  }
  pending_mean_path_cost_ =
      n_cost_finite > 0
          ? sum_cost_finite / static_cast<float>(n_cost_finite)
          : 0.0f;
  pending_selected_info_gain_ = info_gain[selected_idx];
  pending_selected_path_cost_ = path_cost[selected_idx];
  pending_rejected_by_minpos_      = rejected_minpos;
  pending_rejected_by_unreachable_ = rejected_unreachable;

  // ---- Separation manipulation checks ---------------------------------
  // The first two are measured whether or not the term is on: in an untreated
  // arm they are the counterfactual. (notes: sep-manipulation-checks)
  pending_sep_eligible_peers_ = static_cast<int>(sep_anchors.size());
  pending_sep_peer_dist_m_ = SeparationTerm::nearestAnchorDist(
      current_goal_.position.x(), current_goal_.position.y(), sep_anchors);
  // The discount on the candidate that was actually taken, which is not
  // necessarily the most-discounted one or the one that led: the walk above
  // can reject the leader on the map, reachability, the blacklist or MinPos.
  pending_sep_discount_ = sep_discount[selected_idx];
  // pending_sep_reordered_: 1 yes, 0 no, -1 not asked (term off, nothing
  // selected, or amnesty pick). Compares picks by re-walking the same filter
  // chain on undiscounted utility; exact as admissibility ignores score.
  // (notes: sep-reordered-counterfactual-walk)
  pending_sep_reordered_ = -1;
  if (separation_.enabled() && found_in_walk) {
    std::vector<size_t> order_undisc(candidates.size());
    std::iota(order_undisc.begin(), order_undisc.end(), 0);
    // Character-for-character the comparator used for the real `order` above,
    // except for reading utility_undiscounted[] in place of the live score —
    // same rank-primary rule, same NaN-to-the-bottom handling. If that
    // comparator changes, this one has to change with it.
    std::sort(order_undisc.begin(), order_undisc.end(),
        [&utility_undiscounted, &alloc_rank](size_t a, size_t b) {
          if (alloc_rank[a] != alloc_rank[b]) return alloc_rank[a] < alloc_rank[b];
          const float sa = utility_undiscounted[a];
          const float sb = utility_undiscounted[b];
          if (std::isnan(sa)) return false;
          if (std::isnan(sb)) return true;
          return sa > sb;
        });
    WalkTally cf_tally;            // discarded: `tally` holds the real numbers
    long cf_pick = -1;
    for (size_t idx : order_undisc) {
      if (!admissible(idx, cf_tally)) continue;
      cf_pick = static_cast<long>(idx);
      break;
    }
    pending_sep_reordered_ =
        (cf_pick != static_cast<long>(selected_idx)) ? 1 : 0;
  }

  auto plan_end = this->now();
  float plan_ms = static_cast<float>(
      (plan_end - plan_start).nanoseconds() * 1e-6);
  pending_plan_ms_ = plan_ms;  // drained into StepMetrics by doLogStep

  RCLCPP_INFO(get_logger(),
      "Step %d: selected goal (%.2f, %.2f) yaw=%.2f U=%.3f "
      "info=%.2f cost=%.2f field=%.2f±%.2f "
      "[%zu cand, close=%d map=%d unreach=%d blk=%d vis=%d "
      "minpos=%d, peers=%zu, %.1fms]",
      step_, current_goal_.position.x(), current_goal_.position.y(),
      current_goal_.yaw, current_goal_.score,
      pending_selected_info_gain_, pending_selected_path_cost_,
      // `field` is mean±std of info_gain over ALL candidates, against which
      // `info` (the winner's) reads as a z-score by eye. A std that collapses
      // toward zero means the utility has stopped choosing on information.
      pending_mean_info_gain_, pending_info_gain_std_,
      candidates.size(), rejected_too_close, rejected_map,
      rejected_unreachable, rejected_blacklist, rejected_visited,
      rejected_minpos,
      static_cast<size_t>(presentPeerCount()), plan_ms);

  // Throttled, and only when the term is on: shows a pilot the treatment acting
  // without the CSV, without doubling the planner log.
  // (notes: sep-console-line-throttled)
  if (separation_.enabled()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 15000,
        "Separation: %d eligible peer(s), pick sits %.1fm from the nearest, "
        "discounted x%.3f, goal moved by the term: %s.",
        pending_sep_eligible_peers_, pending_sep_peer_dist_m_,
        pending_sep_discount_,
        pending_sep_reordered_ > 0 ? "yes"
                                   : (pending_sep_reordered_ == 0
                                          ? "no" : "not asked"));
  }

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
    publishIntent();
    have_active_intent_ = true;
  }

  transitionTo(State::NAVIGATE, "goal-selected");

  // Initialise smart-timeout state. Budget the driven (path_cost) distance, not
  // the straight line, so a detoured goal is not blacklisted as unreachable.
  // With no cost grid path_cost is the straight line.
  // (notes: nav-budget-driven-distance)
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
  // scovox source: 2.5D column coverage of the ROI on map_cache_ as ingested by
  // loadLatestMap(). In flat mode, ground outside [roi_min_z, roi_max_z] leaves
  // the cache empty and this reads 1.0 (never done).
  // (notes: coverage-scovox-z-band)
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
  // Abort if the goal cell is now inside an obstacle or its inflation zone.
  // Skipped without a planning_map: isCellOccupied treats no map as occupied
  // and would abort every goal. (notes: navigate-goal-inside-obstacle)
  if (latest_plan_map_ && isCellOccupied(current_goal_.position)) {
    RCLCPP_WARN(get_logger(),
        "Step %d: goal (%.2f, %.2f) is now inside an obstacle — "
        "aborting navigation and re-planning.",
        step_, current_goal_.position.x(),
        current_goal_.position.y());
    have_active_intent_ = false;
    // PLAN usually re-goals on the next tick, but nothing guarantees it does —
    // and until it does the navigator is still driving INTO a now-mapped obstacle
    // (invariant: see abandonNavGoal).
    abandonNavGoal("goal inside obstacle");
    transitionTo(State::PLAN, "goal-inside-obstacle");
    return;
  }

  auto robot_pos = latest_pos_;
  float dx = robot_pos.x() - current_goal_.position.x();
  float dy = robot_pos.y() - current_goal_.position.y();
  float dist = std::sqrt(dx * dx + dy * dy);

  // An omnidirectional sensor sees the same whichever way it faces, so the
  // arrival gate does not wait on yaw (see fov_is_omnidirectional_).
  const bool yaw_required = !fov_is_omnidirectional_;
  if (dist < goal_xy_tol_) {
    float yaw_err = std::remainder(latest_yaw_ - current_goal_.yaw,
                                   2.0f * static_cast<float>(M_PI));
    if (!yaw_required || std::abs(yaw_err) < goal_yaw_tol_) {
      // Exploration goal reached: integrate the new observation.
      have_active_intent_ = false;
      // Record the reached goal in visited_goals_ so the next PLAN does not
      // re-pick it: in a forest the frontier set never drains, and only
      // recording where the robot has been breaks the ping-pong.
      // (notes: navigate-visited-goals-break-cycle)
      visited_goals_.add(current_goal_.position, this->now().seconds());
      // Arriving inside a failed site's radius is proof the ground is
      // reachable, which outranks any amount of failure history — including a
      // retirement, which otherwise lasts the whole run. This is the main
      // re-entry path that keeps retirement from being permanent by accident.
      if (const std::size_t cleared = failed_goals_.clearNear(
              current_goal_.position, failed_goal_radius_m_)) {
        RCLCPP_INFO(get_logger(),
            "Goal reached inside a failed-goal site; %zu record(s) cleared.",
            cleared);
      }
      RCLCPP_INFO(get_logger(),
          "Goal reached: dist=%.2f yaw_err=%.1f deg", dist,
          yaw_err * 180.0f / static_cast<float>(M_PI));
      transitionTo(State::INTEGRATE, "goal-reached");
      return;
    }
    // At XY, waiting for the rotation, on its own deadline armed on arrival. Do
    // not reuse nav_budget_sec_: it times the drive from NAVIGATE entry, and
    // failGoal blacklists the spot underfoot.
    // (notes: navigate-rotate-own-deadline)
    auto now = this->now();
    if (!rotate_deadline_armed_) {
      rotate_deadline_armed_ = true;
      rotate_start_time_ = now;
    }
    const double rotate_elapsed = (now - rotate_start_time_).seconds();
    if (rotate_elapsed > goal_rotate_timeout_sec_) {
      failGoal("budget-rotate", (now - state_enter_time_).seconds(),
               "rotate_elapsed_sec", rotate_elapsed, goal_rotate_timeout_sec_);
      return;
    }
    // Keep re-publishing so the navigator keeps servicing the yaw. The
    // no-progress watchdog is deliberately not applied here: rotating in place
    // accumulates no translation. (notes: navigate-rotate-republish)
    republishGoal(current_goal_);
    return;
  }

  auto now = this->now();
  double elapsed = (now - state_enter_time_).seconds();

  // 1) Distance-budgeted hard timeout.
  if (elapsed > nav_budget_sec_) {
    failGoal("budget", elapsed, "nav_elapsed_sec", elapsed, nav_budget_sec_);
    return;
  }

  // 2) No-progress watchdog.
  double window_elapsed = (now - progress_check_time_).seconds();
  if (window_elapsed > progress_window_sec_) {
    float delta = cumulative_distance_ - progress_check_dist_;
    if (delta < progress_min_distance_m_) {
      // The test here is a DISTANCE, not a time: metres travelled inside the
      // progress window against progress_min_distance_m. test_name carries the
      // units so no consumer has to infer them from `reason` and then get them
      // wrong.
      failGoal("no-progress", elapsed, "window_progress_m",
               static_cast<double>(delta),
               static_cast<double>(progress_min_distance_m_));
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
void ExploPlannerNode::failGoal(const char* reason, double elapsed,
                                const char* test_name, double test_value,
                                double test_threshold) {
  const double fail_time = this->now().seconds();
  const int k = failed_goals_.add(current_goal_.position, fail_time,
                                  failed_goal_radius_m_);
  const bool retired =
      failed_goals_.isRetiredNear(current_goal_.position, failed_goal_radius_m_);
  char status[48];
  if (retired) {
    std::snprintf(status, sizeof(status), "RETIRED for run");
  } else {
    std::snprintf(status, sizeof(status), "ttl=%.0fs", failed_goal_ttl_sec_);
  }
  // Log at WARN the check that actually fired (test_name, test_value vs
  // test_threshold), not elapsed vs nav_budget_sec_. pose_stale marks progress
  // measured against a pose that stopped updating.
  // (notes: failgoal-log-tested-pair)
  RCLCPP_WARN(get_logger(),
      "Step %d: navigation failed [%s] on %s=%.1f vs %.1f (%.1fs since "
      "NAVIGATE entry) at goal (%.2f, %.2f)%s. "
      "Blacklisted [k=%d, %s]; %zu active failed-goal sites.",
      step_, reason, test_name, test_value, test_threshold, elapsed,
      current_goal_.position.x(), current_goal_.position.y(),
      have_pose_ ? "" : " [POSE STALE]",
      k, status, failed_goals_.size());
  if (exp_log_) {
    exp_log_->logNavGoalFailed(expCtx(), current_goal_.position.x(),
                               current_goal_.position.y(), reason, elapsed, k,
                               retired, nav_budget_sec_, !have_pose_,
                               test_name, test_value, test_threshold);
  }
  have_active_intent_ = false;  // release the claim on failure
  // The nav budget / no-progress watchdogs give up on this goal; the navigator
  // does not
  // know that — the goal is still accepted and still driving (invariant: see
  // abandonNavGoal), and INTEGRATE is one of the states presumed stationary.
  abandonNavGoal(reason);
  transitionTo(State::INTEGRATE, reason);
}

// ==================================================================
// Exploration completion
// ==================================================================

// (notes: explore-complete-entry-point)
void ExploPlannerNode::recordExplorationComplete(const char* reason) {
  const int active = presentPeerCount();
  // exploration_complete is this robot declaring its exploration exhausted,
  // emitted here, not at the endings. Keyed on step_: re-entry during a
  // deferral is one event; re-saturating later is a new occurrence.
  // (notes: explore-complete-event-step-key)
  if (exp_complete_step_ != step_) {
    exp_complete_step_ = step_;
    // Printed from this single funnel every ending routes through, because the
    // harness hang gate disarms on "Exploration complete". Keep it even though
    // it duplicates the latch path's line.
    // (notes: explore-complete-log-hang-gate)
    RCLCPP_INFO(get_logger(),
        "Exploration complete [%s]: declared at step %d, %.2f m traveled "
        "(unknown=%.3f, source=%s, peers_live=%d).",
        reason, step_, cumulative_distance_, last_unknown_fraction_,
        last_coverage_source_ ? last_coverage_source_ : "?", active);
    // Nested, not a second top-level test: the step_ key is consumed by the
    // line above, so the event has to share this scope to keep firing on the
    // same tick it always did. Hoisting the key past `exp_log_` only means a
    // build with no event log now latches it too, which no caller reads.
    if (!exp_log_) return;
    ExplorationCompleteEvent e;
    e.reason            = reason;
    e.unknown_fraction  = last_unknown_fraction_;
    e.coverage_source   = last_coverage_source_;
    e.steps             = step_;
    e.distance_m        = cumulative_distance_;
    e.team_complete     = (fleet_.size() > 1 && team_ && team_->allConnected());
    e.peers_live        = active;
    e.expected_peers    = expectedPeers();
    e.occurrence        = ++exp_complete_count_;
    exp_log_->logExplorationComplete(expCtx(), e);
  }
}

// Publishes the deciding sample into last_unknown_fraction_ /
// last_coverage_source_, which the exploration_complete event stamps from. A
// negative unk (cannot measure) is never published.
// (notes: coverage-decision-sample-cache)
void ExploPlannerNode::noteCoverageDecisionSample(double unk,
                                                  const char* source) {
  if (unk < 0.0) return;
  last_unknown_fraction_ = unk;
  if (source && *source) last_coverage_source_ = source;
}

// The latched, state-blind criterion. Everything this does NOT do is the point:
// no streak to accumulate, no peer count consulted, no state excluded, and no
// path back out. It answers one question — has this robot's own map of the ROI
// reached the threshold yet — and the answer is monotone in time.
bool ExploPlannerNode::maybeLatchCoverageDone(double unk, const char* source) {
  if (done_criterion_ != "latch") return false;
  if (coverage_latched_) return false;           // first touch only
  if (done_unknown_fraction_ <= 0.0) return false;
  // unk < 0 is "cannot measure", NOT "fully explored". Guarding on >= 0 is what
  // keeps a degenerate ROI or an unpublished planning_map from reading as an
  // instant finish on the first tick of the run.
  if (!(unk >= 0.0 && unk <= done_unknown_fraction_)) return false;

  coverage_latched_       = true;
  coverage_latch_t_sim_   = this->now().seconds();
  coverage_latch_unknown_ = unk;
  noteCoverageDecisionSample(unk, source);
  RCLCPP_INFO(get_logger(),
      "Exploration complete [latch]: ROI unknown fraction %.3f <= %.3f "
      "(source=%s) in state %s at t_sim=%.1f — %d steps, %.2f m traveled. "
      "Latched; this robot does not un-finish.",
      unk, done_unknown_fraction_, source, stateName(state_),
      coverage_latch_t_sim_, step_, cumulative_distance_);

  // Order matters: record the declaration against the state we were actually
  // in, before any transitionTo moves us. The latch can fire mid-drive: an
  // ending that is not meant to keep the nav goal must abandon it.
  // (notes: latch-order-record-then-stop)
  recordExplorationComplete("coverage-latched");

  // Finished is all the latch decides (K4); exploration_complete went out just
  // above. What follows (wait, meet, home) is TeamCore's, and the next tick's
  // activity switches the state. true stops the caller, so doPlan publishes
  // no goal on the latch tick.
  if (!finished_) {
    finished_        = true;
    finished_reason_ = "coverage-latched";
  }
  return true;
}

void ExploPlannerNode::noteMapSize(double t_sim_sec, double voxels) {
  latest_map_voxels_ = voxels;
  map_size_hist_.emplace_back(t_sim_sec, voxels);
  size_t keep_from = 0;
  while (keep_from + 1 < map_size_hist_.size() &&
         map_size_hist_[keep_from + 1].first < t_sim_sec - kRateWindowSec) {
    ++keep_from;
  }
  if (keep_from > 0) {
    map_size_hist_.erase(map_size_hist_.begin(),
                         map_size_hist_.begin() + keep_from);
  }
  // Winsorised slope: per-interval deltas floored at 0 and capped at
  // kRateWinsorK x the window median, over the window span, so a merged peer
  // map does not count as this robot's own gathering.
  // (notes: midrun-winsorised-growth-rate)
  std::vector<double> deltas;
  deltas.reserve(map_size_hist_.size());
  double span = 0.0;
  for (size_t i = 1; i < map_size_hist_.size(); ++i) {
    const double dt = map_size_hist_[i].first - map_size_hist_[i - 1].first;
    if (dt <= 0.0) continue;
    deltas.push_back(
        std::max(0.0, map_size_hist_[i].second - map_size_hist_[i - 1].second));
    span += dt;
  }
  if (deltas.size() < 2 || span <= 0.0) {
    map_growth_rate_ = 0.0;
    return;
  }
  std::vector<double> sorted = deltas;
  std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2,
                   sorted.end());
  const double med = sorted[sorted.size() / 2];
  const double cap = (med > 0.0) ? kRateWinsorK * med
                                 : std::numeric_limits<double>::infinity();
  double total = 0.0;
  for (double d : deltas) total += std::min(d, cap);
  map_growth_rate_ = total / span;
}

void ExploPlannerNode::publishIntent() {
  stampMapInfo();
  intent_pub_->publish(current_intent_msg_);
}

void ExploPlannerNode::stampMapInfo() {
  current_intent_msg_.observed_voxels =
      static_cast<uint64_t>(std::max(0.0, latest_map_voxels_));
  current_intent_msg_.map_growth_rate =
      static_cast<float>(map_growth_rate_);
}

// The intent heartbeat: re-publishes the exploration goal claim while it is
// held. Presence is the team beacon's job in gen 34; this stream carries only
// claims (and the guard's intent-pose fallback).
void ExploPlannerNode::heartbeatTick() {
  if (!coord_enabled_) return;
  // A hold is judged by the drive it interrupted.
  const State drive_state =
      (state_ == State::PROXIMITY_HOLD) ? prox_resume_state_ : state_;
  // Suppression accounting: the claim stream is state-gated, so a planner
  // stuck in a non-claiming state reads as unclaimed to peers under perfect
  // comms. The team states are not suppression: they hold no claim by design
  // and the beacon keeps them present.
  // (notes: hb-suppression-accounting)
  const bool beaconing =
      isTeamState(drive_state) ||
      (have_active_intent_ && intent_pub_ &&
       (state_ == State::NAVIGATE || state_ == State::INTEGRATE ||
        state_ == State::PROXIMITY_HOLD));
  const auto hb_now = this->now();
  // Executor starvation: an inter-tick heartbeat gap >= coord_claim_ttl_sec_
  // lets peers age our claim out with the link up. Only the measured
  // inter-tick interval can reveal it. (notes: hb-executor-starvation)
  if (hb_last_tick_.nanoseconds() > 0) {
    const double gap = (hb_now - hb_last_tick_).seconds();
    if (gap >= coord_claim_ttl_sec_) {
      ++hb_late_count_;
      RCLCPP_WARN(get_logger(),
          "Heartbeat tick LATE: %.2f s since the previous tick (>= claim TTL "
          "%.1f s), state %s. The executor was blocked, not the radio — peers "
          "may have aged this robot's claim out with the link up. Late ticks "
          "so far: %d.", gap, coord_claim_ttl_sec_, stateName(state_),
          hb_late_count_);
    }
  }
  hb_last_tick_ = hb_now;
  if (!beaconing) {
    if (!hb_suppressed_) {
      hb_suppressed_ = true;
      hb_suppress_start_ = hb_now;
      hb_suppress_warned_ = false;
    }
    const double held = (hb_now - hb_suppress_start_).seconds();
    // One WARN per episode once held >= the claim TTL. The latch is consumed
    // even in WAIT_FOR_MAP, where the WARN is skipped, so the exemption covers
    // the whole episode; the episode itself is still tracked.
    // (notes: hb-suppress-warn-episode)
    if (!hb_suppress_warned_ && held >= coord_claim_ttl_sec_) {
      hb_suppress_warned_ = true;
      if (state_ != State::WAIT_FOR_MAP) {
        RCLCPP_WARN(get_logger(),
            "Heartbeat suppressed %.1f s in state %s (>= claim TTL %.1f s): "
            "peers no longer see this robot's goal claim. Its presence is "
            "unaffected (the team beacon carries that).",
            held, stateName(state_), coord_claim_ttl_sec_);
      }
    }
  } else if (hb_suppressed_) {
    hb_suppressed_ = false;
    const double held = (hb_now - hb_suppress_start_).seconds();
    RCLCPP_INFO(get_logger(),
        "Heartbeat resumed after %.1f s suppressed (now %s)%s.",
        held, stateName(state_),
        held >= coord_claim_ttl_sec_ ? " [exceeded claim TTL]" : "");
  }
  if (!have_active_intent_) return;
  // Re-publish while holding an exploration claim: driving, integrating, and
  // PROXIMITY_HOLD (the interrupted goal's claim must survive). The team
  // states cleared the claim on entry.
  // (notes: hb-beacon-state-gate)
  if (state_ != State::NAVIGATE && state_ != State::INTEGRATE &&
      state_ != State::PROXIMITY_HOLD) {
    return;
  }
  if (!intent_pub_) return;
  current_intent_msg_.header.stamp = this->now();
  // Refresh robot_pos, not just the stamp: Coordination::selfWinsAgainst
  // compares a peer's live pose against it, and a stale value lets a peer poach
  // a goal under active pursuit. (notes: hb-refresh-robot-pos)
  current_intent_msg_.robot_pos.x = latest_pos_.x();
  current_intent_msg_.robot_pos.y = latest_pos_.y();
  current_intent_msg_.robot_pos.z = latest_pos_.z();
  publishIntent();
}

// ==================================================================
// Proximity stop (coordinated yield)
// ==================================================================
// While driving, yield to a higher-priority teammate nearby: brake at own pose,
// hold, resume when it clears or parks. Smaller robot_name has right of way.
// Best-effort coordination, not a safety function.
// (notes: proximity-stop-rules)

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
      "Proximity hold #%d: yielding to '%s' at %.2f m (< %.2f m). Publishing a "
      "brake goal at the current pose; resuming beyond %.2f m or when the peer "
      "parks.",
      prox_hold_count_, d.peer_id.c_str(), d.dist_m,
      prox_guard_->config().hold_dist_m, prox_guard_->config().resume_dist_m);

  // Stop via a brake goal at the robot's own pose: simple_nav_3d latches the
  // last goal, so ceasing to publish does not stop it. The nav2 cancel never
  // fires here; current_goal_ is left for the resume.
  // (notes: prox-hold-brake-goal-stop)
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
  transitionTo(State::PROXIMITY_HOLD, "proximity-hold");
}

// Brakes in place when leaving a driving state without publishing a replacement
// goal; the proximity guard assumes non-driving states are stationary. Needs a
// live latest_pos_: never call before the first pose.
// (notes: abandon-nav-goal-brake)
void ExploPlannerNode::abandonNavGoal(const char* why) {
  bool cancel_sent = false;
  if (nav_cancel_client_ && nav_cancel_client_->action_server_is_ready()) {
    // The reason is copied into the callback, not captured as a pointer: the
    // response lands ticks later and one caller forwards failGoal's `reason`
    // argument, so nothing here may assume the string outlives this call.
    nav_cancel_client_->async_cancel_all_goals(
        [this, tag = std::string(why)](auto resp) {
          if (!resp ||
              resp->return_code !=
                  action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
            RCLCPP_WARN(get_logger(),
                "Nav abandon [%s]: cancel returned code %d — the brake goal is "
                "the only stop command.",
                tag.c_str(), resp ? static_cast<int>(resp->return_code) : -1);
          }
        });
    cancel_sent = true;
    RCLCPP_INFO(get_logger(), "Nav cancel request sent to '%s'.",
                proximity_nav_cancel_action_.c_str());
  } else {
    // WARN_ONCE: on the sim stack this branch is taken on every abandon; the
    // per-abandon record is the INFO line below. (notes: abandon-nav-warn-once)
    RCLCPP_WARN_ONCE(get_logger(),
        "Nav abandon [%s]: cancel client for '%s' unavailable (proximity stop "
        "disabled, or the action server is not up) — the brake goal is the "
        "only stop command. Further occurrences suppressed.",
        why, proximity_nav_cancel_action_.c_str());
  }
  // Brake in place. current_goal_ is deliberately left alone: callers still
  // read it (failGoal blacklists it) and the states we transition INTO
  // overwrite it when they select their own goal.
  CandidateViewpoint brake;
  brake.position = latest_pos_;
  brake.yaw = latest_yaw_;
  publishGoal(brake);

  // Report only what was done: claim a cancel only when one was actually sent.
  // (notes: abandon-nav-info-wording)
  RCLCPP_INFO(get_logger(),
      "Abandoning nav goal [%s]: brake goal published at current pose%s.", why,
      cancel_sent ? " (cancel also requested)" : "");
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
    // Grant an escape grace window, or the next tick's entry check re-holds on
    // the peer still inside the trigger disc. The guard drops the immunity
    // early if the peer moves again. (notes: prox-hold-escape-grace)
    prox_guard_->armEscape(d.peer_id, now);
  }

  prox_hold_total_sec_ += held;
  const char* why = d.hold ? "max-hold" : d.note.c_str();
  RCLCPP_INFO(get_logger(),
      "Proximity hold released after %.1fs [%s: '%s' at %.2f m] -> resuming "
      "%s.", held, why, d.peer_id.c_str(), d.dist_m,
      stateName(prox_resume_state_));
  char buf[160];
  std::snprintf(buf, sizeof(buf), "clear reason=%s held=%.1f", why, held);
  publishProxState(buf);
  transitionTo(prox_resume_state_, "proximity-hold-released");
  // A Leg restarts from here: the brake goal replaced it at the navigator, and
  // a fresh leg re-publishes its point and restarts the watchdog window. The
  // Leg's time bound is TeamCore's and kept running through the hold.
  if (isLegState(state_)) {
    leg_.reset();
    return;
  }
  // Resume the same exploration goal; the re-publish is mandatory because the
  // brake goal replaced it at the navigator. Nav budget continues (backdated),
  // progress window restarts. (notes: prox-hold-resume-same-goal)
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
    transitionTo(State::LOG_STEP, "integrate-complete");
  }
}

// ==================================================================
// CSV rows — shared assembly, and the periodic sampler
// ==================================================================

// Fills the world-state columns shared by the end-of-step and timer rows.
// Plan-attribution columns are deliberately not set here, so timer rows leave
// them zero. (notes: csv-fill-common-metrics)
void ExploPlannerNode::fillCommonMetrics(StepMetrics& m) {
  const auto now = this->now();
  m.step              = step_;
  m.sim_time_sec      = now.seconds();
  m.distance_traveled = cumulative_distance_;
  // Peers present on the team beacon (heard directly within the presence
  // window), the number TeamCore decides on.
  m.coord_active_peers  = presentPeerCount();
  // Cumulative proximity-hold columns.
  m.prox_hold_count     = prox_hold_count_;
  m.prox_hold_total_sec = static_cast<float>(prox_hold_total_sec_);
  m.phase = (phase_ == Phase::EXPLOIT) ? "exploit" : "explore";
  m.state = stateName(state_);

  // Rejection profile of the last planning attempt, filled here rather than in
  // doLogStep so it survives on timer rows when no step completes.
  // (notes: csv-plan-rejections-on-timer-rows)
  m.plan_cand_total    = pending_plan_cand_total_;
  m.plan_rej_close     = pending_plan_rej_close_;
  m.plan_rej_map       = pending_plan_rej_map_;
  m.plan_rej_unreach   = pending_plan_rej_unreach_;
  m.plan_rej_blacklist = pending_plan_rej_blacklist_;
  m.plan_rej_visited   = pending_plan_rej_visited_;
  m.plan_rej_minpos    = pending_plan_rej_minpos_;
  m.plan_stall_ticks   = pending_plan_stall_ticks_;

  // Reconnect columns stay at their -1 sentinel: gen 34 has no reconnect
  // manoeuvre (the team activities are in the event log's tick events).
  //
  // team_complete: every robot in mutual contact with every other (TeamCore's
  // allConnected); -1 without a team to ask about.
  // (notes: csv-team-complete-sentinel)
  if (fleet_.size() > 1 && team_) {
    m.team_complete = team_->allConnected() ? 1 : 0;
  }
  // The chase columns, gated on the chase being driven (CHASE or FOLLOW, or a
  // hold resuming one) so mid-chase holds are recorded.
  // (notes: csv-pursuit-live-gate)
  if (team_ && team_->chase().held) {
    const State ds = (state_ == State::PROXIMITY_HOLD) ? prox_resume_state_
                                                       : state_;
    if (ds == State::CHASE || ds == State::FOLLOW) {
      const int q = team_->chase().peer;
      m.pursue_peer = fleet_.nameOf(q);
      m.pursue_quarry_live = team_->present(q) ? 1 : 0;
    }
  }

  // Coverage-termination measure, on every row. This is the primary endpoint
  // quantity AND what the DONE criterion is compared against, so a run that
  // does not carry it cannot be scored on its own stopping rule. Same call the
  // termination check makes, so the CSV and the decision can never disagree.
  const char* cov_src = "none";
  const double uf = coverageUnknownFraction(&cov_src);
  m.unknown_fraction = static_cast<float>(uf);
  m.coverage_source  = cov_src;
  last_unknown_fraction_ = uf;
  last_coverage_source_  = cov_src;

  // Coverage milestones use the same measurement and tick as the CSV row and
  // DONE criterion. Every CSV row emitter passes here, so the ladder runs at
  // the sampling period even while step_ is frozen.
  // (notes: csv-coverage-milestone-hook)
  if (exp_log_) {
    exp_log_->noteCoverage(expCtx(), uf, cov_src,
                           static_cast<double>(cumulative_distance_),
                           static_cast<double>(latest_pos_.x()),
                           static_cast<double>(latest_pos_.y()));
  }

  // Cell census taken on the same tick and map as uf, so the two can be
  // compared. No-op when disabled. (notes: csv-cell-census-same-tick)
  updateCellWorld(uf, cov_src);

  // Aggregate map stats from map_cache_ (the fused ROI grid). The dscovox node
  // no longer computes these — scoring and stats both live in the planner now.
  // The grid walk + frontier-neighbour logic lives in MapCache::computeStats()
  // (shared with findFrontierCentroids, unit-testable in isolation).
  const auto stats = map_cache_->computeStats();
  m.total_observed_voxels = stats.total_voxels;
  // Every CSV row passes through here (see the milestone comment above), so
  // this is the one place the map-size beacon's count/rate cache is fed —
  // the beacon can never disagree with the logged series.
  noteMapSize(now.seconds(), static_cast<double>(stats.total_voxels));
  m.frontier_voxels       = stats.frontier_voxels;
  m.mean_eig              = stats.mean_eig;
  m.mean_entropy          = stats.mean_entropy;
  m.mean_variance         = stats.mean_variance;
}

// Samples a CSV row on a periodic timer in every state, since steps only
// advance in the explore loop. The period is sim time; only the adaptive
// back-off uses the steady wall clock. (notes: metrics-tick-periodic-sampler)
void ExploPlannerNode::metricsTick() {
  if (!logger_ || !map_cache_) return;
  // Self-throttle: this tick (map re-ingest plus whole-grid stats) runs on the
  // single-threaded executor, so the period stretches until a tick costs at
  // most metrics_max_duty_ of it. (notes: metrics-tick-self-throttle)
  const auto mt_now = std::chrono::steady_clock::now();
  if (metrics_next_.time_since_epoch().count() != 0 && mt_now < metrics_next_)
    return;
  // LOG_STEP emits its own, strictly richer row on this same tick. Skipping it
  // here is what makes state=="LOG_STEP" a sound discriminator for end-of-step
  // rows instead of a coin flip on timer phase.
  if (state_ == State::LOG_STEP) return;

  // Re-ingest first: map_cache_ is otherwise rebuilt only in WAIT_FOR_MAP/PLAN
  // and the exploit states, so samples would freeze the voxel count. Cheap when
  // no new map has arrived. (notes: metrics-tick-reingest-first)
  loadLatestMap();

  StepMetrics m;
  fillCommonMetrics(m);
  logger_->logStep(m);

  // Realised-rate accounting for run_end, from the sim stamps of rows actually
  // written; the configured period is not guaranteed (wall-clock gate, RTF,
  // duty back-off). (notes: metrics-tick-realised-rate)
  ++metrics_rows_written_;
  if (metrics_first_row_sim_sec_ < 0.0)
    metrics_first_row_sim_sec_ = m.sim_time_sec;
  metrics_last_row_sim_sec_ = m.sim_time_sec;

  const double cost_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - mt_now).count();
  // Steady clock throughout: this bounds work on the executor thread, which is
  // wall-clock work whatever use_sim_time says. Using the ROS clock here would
  // make the back-off scale with RTF and stop bounding anything.
  const double budget = std::max(1e-3, metrics_period_sec_ * metrics_max_duty_);
  double eff_period = metrics_period_sec_;
  if (cost_s > budget) {
    eff_period = cost_s / metrics_max_duty_;
    if (eff_period > metrics_effective_period_ * 1.5 || metrics_backoffs_ == 0) {
      ++metrics_backoffs_;
      RCLCPP_WARN(get_logger(),
          "Metrics sampler backing off: a tick cost %.2f s (> %.0f%% of the "
          "%.1f s period); sampling every %.1f s until it gets cheaper. Rows "
          "during this window are sparser — the coverage curve is undersampled, "
          "not flat. (backoff #%d)",
          cost_s, metrics_max_duty_ * 100.0, metrics_period_sec_, eff_period,
          metrics_backoffs_);
    }
  }
  metrics_effective_period_ = eff_period;
  // Arm the next deadline from this tick's scheduled slot, not from now after
  // the work, so the schedule stays phase-locked to the tick source. A schedule
  // that has fallen behind re-anchors, no catch-up burst.
  // (notes: metrics-tick-phase-locked-deadline)
  const auto eff_dur =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(eff_period));
  auto next = (metrics_next_.time_since_epoch().count() != 0)
                  ? metrics_next_ + eff_dur
                  : mt_now + eff_dur;
  if (next <= mt_now) next = mt_now + eff_dur;
  metrics_next_ = next;

  // Decides done_criterion == "latch" on the cached double from
  // fillCommonMetrics. Must stay last so the qualifying row is logged and
  // counted first. Safe to transition: same exclusive callback group as tick().
  // (notes: metrics-tick-coverage-latch-hook)
  maybeLatchCoverageDone(last_unknown_fraction_, last_coverage_source_);
}

// ==================================================================
// Experiment event log — see experiment_log.hpp
// ==================================================================

ExperimentContext ExploPlannerNode::expCtx() {
  ExperimentContext c;
  // The node's own clock (sim time under use_sim_time), so every logged event
  // shares the axis the planner decides on. (notes: explog-ctx-sim-clock)
  c.sim_time_sec = this->now().seconds();
  c.state = stateName(state_);
  c.step  = step_;
  return c;
}

void ExploPlannerNode::startExperimentLog() {
  if (!exp_log_ || exp_log_->started()) return;
  // Under use_sim_time now() reads 0 until the first /clock; wait for a real
  // stamp so t0 is not 0. (notes: explog-start-waits-for-clock)
  const auto now = this->now();
  if (now.nanoseconds() <= 0) return;
  ExperimentContext ctx = expCtx();
  exp_log_->startRun(ctx, coverage_milestones_);
  RCLCPP_INFO(get_logger(),
      "Experiment event log started at sim t0=%.3f s; all event times in "
      "'%s' are on this clock.", ctx.sim_time_sec,
      experiment_log_path_.c_str());
}

void ExploPlannerNode::expClockAnchorTick() {
  if (!exp_log_ || !exp_log_->started()) return;
  if (experiment_log_anchor_period_sec_ <= 0.0) return;
  const double now_sec = this->now().seconds();
  if (next_anchor_sim_sec_ < 0.0) {
    // First anchor goes out immediately after run_start, so the very first
    // interval of the run is bracketed like every other one.
    next_anchor_sim_sec_ = now_sec;
  }
  if (now_sec < next_anchor_sim_sec_) return;
  exp_log_->logClockAnchor(expCtx());
  // Scheduled forward from NOW, not from the missed deadline: a sim clock that
  // jumps (a paused or fast-forwarded simulator) must not produce a burst of
  // back-dated anchors, all of which would carry the same wall stamp and so
  // report an infinite real-time factor.
  next_anchor_sim_sec_ = now_sec + experiment_log_anchor_period_sec_;
}

// Peer belief, kept from the intent stream rather than Coordination's claim
// table so it also works with coordination_enabled=false. Threshold is
// coord_claim_ttl_sec, as in the barrier's presence test.
// (notes: explog-peer-belief-source)
void ExploPlannerNode::expPeerHeard(const std::string& peer_id) {
  if (!exp_log_) return;
  // Do not seed a belief before run_start: now() is 0 before the first /clock
  // and a belief stamped 0 later reports a false outage. Costs at most one
  // heartbeat; only the event log reads peer_belief_.
  // (notes: explog-peer-belief-before-run-start)
  if (!exp_log_->started()) return;
  const auto now = this->now();
  auto it = peer_belief_.find(peer_id);
  if (it == peer_belief_.end()) {
    // First time this teammate has ever been heard. Worth an event of its own
    // (first_contact=true): it is when the pair's link came up, which is not
    // recoverable from anything else in the file.
    PeerBelief b;
    b.last_heard = now;
    b.live = true;
    peer_belief_.emplace(peer_id, b);
    PeerEvent e;
    e.peer = peer_id;
    e.silent_sec = 0.0;
    e.first_contact = true;
    e.peers_live = presentPeerCount();
    e.expected_peers = expectedPeers();
    exp_log_->logPeerSeen(expCtx(), e);
    return;
  }
  if (!it->second.live) {
    PeerEvent e;
    e.peer = peer_id;
    // The outage this message ends, measured from the last one that preceded
    // it — NOT from when the sweep noticed, which lags by up to a claim TTL.
    e.silent_sec = (now - it->second.last_heard).seconds();
    e.peers_live = presentPeerCount();
    e.expected_peers = expectedPeers();
    exp_log_->logPeerSeen(expCtx(), e);
    it->second.live = true;
  }
  it->second.last_heard = now;
}

void ExploPlannerNode::expPeerSweep() {
  if (!exp_log_ || peer_belief_.empty()) return;
  const auto now = this->now();
  for (auto& [peer_id, belief] : peer_belief_) {
    if (!belief.live) continue;
    const double silent = (now - belief.last_heard).seconds();
    if (silent < coord_claim_ttl_sec_) continue;
    belief.live = false;
    PeerEvent e;
    e.peer = peer_id;
    e.silent_sec = silent;
    e.peers_live = presentPeerCount();
    e.expected_peers = expectedPeers();
    exp_log_->logPeerLost(expCtx(), e);
  }
}

void ExploPlannerNode::logRunEnd(const char* reason) {
  if (!exp_log_) return;
  RunEndEvent e;
  e.reason     = reason;
  e.steps      = step_;
  e.distance_m = cumulative_distance_;
  // Last measured coverage, not a fresh walk: this runs on DONE and during
  // shutdown, when map_cache_ may be mid-teardown. At most one metrics period
  // old. (notes: explog-run-end-cached-coverage)
  e.unknown_fraction = last_unknown_fraction_;
  e.coverage_source  = last_coverage_source_;
  e.peers_live = presentPeerCount();
  e.expected_peers = expectedPeers();
  // Configured vs REALISED CSV sampling rate. The realised value needs two rows
  // to be a period at all; -1 says "not measurable", never a fabricated 0.
  e.metrics_period_param_sec     = metrics_period_sec_;
  e.metrics_effective_period_sec = metrics_effective_period_;
  e.metrics_timer_rows           = metrics_rows_written_;
  e.metrics_backoffs             = metrics_backoffs_;
  e.metrics_realised_period_sec =
      (metrics_rows_written_ >= 2)
          ? (metrics_last_row_sim_sec_ - metrics_first_row_sim_sec_) /
                static_cast<double>(metrics_rows_written_ - 1)
          : -1.0;
  // Final geometry and mission-return summary. latest_pos_, not a TF lookup
  // (teardown). mission_home_result_ stays "" (null) when homing never
  // resolved, including a run killed mid-homing.
  // (notes: explog-run-end-home-summary)
  e.have_home = have_home_;
  e.home_x    = home_pos_.x();
  e.home_y    = home_pos_.y();
  e.final_x   = latest_pos_.x();
  e.final_y   = latest_pos_.y();
  e.mission_home_result  = mission_home_result_;
  e.mission_home_sim_sec = mission_home_sim_sec_;
  // Gen 33's once-per-run homing guard and mid-run attempts do not exist here;
  // the columns stay, at zero, so one reader handles both generations.
  e.mission_return_reentries = 0;
  e.midrun_attempts_used     = 0;
  // Every mechanism's firing count (rule 3). TeamCore's hold every counter
  // from construction, so one that never fired reads 0, not absent.
  if (team_) e.mechanism_counts = team_->counts();
  e.mechanism_counts["beacons_sent"]        = beacons_sent_;
  e.mechanism_counts["beacons_received"]    = beacons_received_;
  e.mechanism_counts["beacons_superseded"]  = beacons_superseded_;
  e.mechanism_counts["beacons_rejected"]    = beacons_rejected_;
  e.mechanism_counts["cell_merges_refused"] = cell_merges_refused_;
  e.mechanism_counts["leg_legs"]            = leg_.legs();
  e.mechanism_counts["leg_escapes"]         = leg_.escapes();
  e.mechanism_counts["team_events_unknown"] = exp_log_->teamEventsUnknown();
  exp_log_->logRunEnd(expCtx(), e);
}

ExploPlannerNode::~ExploPlannerNode() {
  // Writes run_end on SIGTERM shutdown, which never reaches DONE; the logger
  // ignores a second run_end. Nothing here may throw.
  // (notes: explog-run-end-in-destructor)
  try {
    // A DONE-idle robot already knows why it finished; only a node killed
    // before ever reaching DONE (the duration cap, i.e. a censored run) falls
    // back to the signal itself as the reason.
    logRunEnd(done_reason_.empty() ? "node-destroyed" : done_reason_.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "ExperimentLog: run_end failed: %s", e.what());
  } catch (...) {
  }
}

// ==================================================================
// LOG_STEP state
// ==================================================================

void ExploPlannerNode::doLogStep() {
  StepMetrics m;
  fillCommonMetrics(m);
  m.selected_score = current_goal_.score;
  m.plan_time_ms = pending_plan_ms_;

  // Drain utility / coord diagnostics from doPlan().
  m.mean_info_gain      = pending_mean_info_gain_;
  m.info_gain_std       = pending_info_gain_std_;
  m.mean_path_cost      = pending_mean_path_cost_;
  m.selected_info_gain  = pending_selected_info_gain_;
  m.selected_path_cost  = pending_selected_path_cost_;
  m.selected_utility    = current_goal_.score;
  m.rejected_by_minpos        = pending_rejected_by_minpos_;
  m.rejected_by_unreachable   = pending_rejected_by_unreachable_;
  m.sep_peer_dist_m     = pending_sep_peer_dist_m_;
  m.sep_discount        = pending_sep_discount_;
  m.sep_reordered       = pending_sep_reordered_;
  m.sep_eligible_peers  = pending_sep_eligible_peers_;

  logger_->logStep(m);
  // The same row into the event log. Deliberate redundancy with the CSV, which
  // is unchanged and stays: two independently written files that must agree is
  // what lets either be validated against the other, and the JSON copy carries
  // the pose and the sim-clock envelope the CSV has no columns for.
  if (exp_log_) {
    StepEvent e;
    e.unknown_fraction = m.unknown_fraction;
    e.coverage_source  = m.coverage_source.c_str();
    e.observed_voxels  = m.total_observed_voxels;
    e.frontier_voxels  = m.frontier_voxels;
    e.distance_m       = m.distance_traveled;
    e.x   = latest_pos_.x();
    e.y   = latest_pos_.y();
    e.z   = latest_pos_.z();
    e.yaw = latest_yaw_;
    e.phase = m.phase.c_str();
    e.peers_live = m.coord_active_peers;
    e.plan_time_ms = m.plan_time_ms;
    exp_log_->logStep(expCtx(), e);
  }
  RCLCPP_INFO(get_logger(),
      "Step %d logged: voxels=%d frontiers=%d dist=%.2f mean_eig=%.4f",
      step_, m.total_observed_voxels, m.frontier_voxels,
      m.distance_traveled, m.mean_eig);

  step_++;
  if (step_ >= max_steps_) {
    RCLCPP_INFO(get_logger(),
        "Step budget reached: %d steps, %.2f m traveled, %d final voxels.",
        step_, cumulative_distance_, m.total_observed_voxels);
    // Finished, latched (K4). To PLAN, never staying in LOG_STEP (each call
    // logs and bumps step_); PLAN does nothing once finished, and the next
    // tick's activity moves the state on.
    latchFinished("step-budget");
    transitionTo(State::PLAN, "step-budget");
  } else {
    transitionTo(State::PLAN, "step-logged");
  }
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
  // A successful lookup is not a fresh pose: TimePointZero keeps returning the
  // last transform after a broadcaster dies. A transform older than
  // pose_max_age_sec_ reads as pose lost. (notes: pose-stale-tf-is-lost)
  if (pose_max_age_sec_ > 0.0) {
    const double age = (this->now() - rclcpp::Time(tf.header.stamp)).seconds();
    if (age > pose_max_age_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TF %s -> %s is %.1f s old (pose_max_age_sec=%.1f) — treating the "
          "pose as lost until the transform updates.",
          map_frame_.c_str(), base_frame_.c_str(), age, pose_max_age_sec_);
      // Edge-triggered into the JSONL event log (the WARN is throttled and
      // ROS-log only) so a dead pose feed is distinguishable from a stopped
      // robot. (notes: pose-loss-event-log)
      if (have_pose_ && exp_log_) {
        exp_log_->logPoseHealth(expCtx(), /*lost=*/true, age);
        pose_loss_reported_ = true;
      }
      have_pose_ = false;
      return;
    }
  }
  // Recovery is gated on a REPORTED loss, not on !have_pose_, which is also
  // false for every tick before the first transform ever arrives — that would
  // stamp a "pose recovered" event on the healthy start of every single run.
  if (pose_loss_reported_ && exp_log_) {
    exp_log_->logPoseHealth(expCtx(), /*lost=*/false, 0.0);
    pose_loss_reported_ = false;
  }
  latest_pos_ = Eigen::Vector3f(
      static_cast<float>(tf.transform.translation.x),
      static_cast<float>(tf.transform.translation.y),
      static_cast<float>(tf.transform.translation.z));
  latest_yaw_ = static_cast<float>(tf2::getYaw(tf.transform.rotation));
  have_pose_ = true;
  // Home is captured exactly once, at the first fresh pose. Its own latch, not
  // have_pose_, which toggles with TF health and would move home.
  // (notes: mission-return-home-capture)
  if (!have_home_) {
    have_home_ = true;
    home_pos_  = latest_pos_;
    home_yaw_  = latest_yaw_;
    RCLCPP_INFO(get_logger(),
        "MISSION-RETURN home recorded: (%.2f, %.2f, %.2f) yaw %.2f.",
        home_pos_.x(), home_pos_.y(), home_pos_.z(), home_yaw_);
  }
}

void ExploPlannerNode::trackDistance() {
  if (!have_pose_) return;
  if (!first_pos_) {
    float step = (latest_pos_ - prev_pos_).norm();
    // Steps above max_pose_jump_m_ are localization discontinuities, not
    // travel: not counted in distance or progress. prev_pos_ still advances; 0
    // disables the guard. (notes: distance-pose-jump-guard)
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
  // The 3D goal z is sent by default (UGV arrival ignores z); flatten_goal_z_
  // zeroes it for consumers that reject non-zero z. A UAV arrival test is 3D,
  // so there z must be right. (notes: goal-z-flatten)
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

// Keep-alive for states still driving (NAVIGATE and the Leg states): publishes
// on pose change, new subscriber, or after goal_republish_sec_, never at tick
// rate. (notes: goal-republish-keepalive)
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

// ==================================================================
// Coarse cell world (P1)
// ==================================================================

void ExploPlannerNode::updateCellWorld(double roi_unknown_fraction,
                                       const char* coverage_source) {
  if (!cell_world_enable_ || !cell_world_.configured() || !map_cache_) return;

  // Rate-limited on SIM time, the same axis every event in the log lives on.
  // The census walks the whole voxel grid once — the cost of a single
  // unknownColumnFraction call, which this tick already paid — so the limit is
  // about not doubling that on every CSV row, not about it being expensive.
  const double t = this->now().seconds();
  if (last_cell_census_sec_ >= 0.0 &&
      t - last_cell_census_sec_ < cell_census_period_s_)
    return;
  last_cell_census_sec_ = t;

  const int changed =
      cell_world_.applyObservation(censusFromMap(*map_cache_,
                                                 cell_world_.grid()));

  // Edges are rebuilt every census from the plan map as it fills. With no plan
  // map every edge is enabled; an all-disabled graph would rank nothing.
  // (notes: cell-world-edges-from-plan-map)
  if (latest_plan_map_) {
    const nav_msgs::msg::OccupancyGrid& pm = *latest_plan_map_;
    cell_world_.rebuildEdges(
        [&pm](float x0, float y0, float x1, float y1) -> double {
          // Sample the straight line between two cell centroids. 16 intervals:
          // at a 10 m cell an 8-neighbour span is <= 14.1 m, so samples are
          // under a metre apart — coarse next to the plan map's resolution,
          // but this decides an edge, not a path.
          constexpr int kSamples = 16;
          int blocked = 0;
          for (int i = 0; i <= kSamples; ++i) {
            const float u = static_cast<float>(i) / kSamples;
            const Eigen::Vector3f p(x0 + (x1 - x0) * u,
                                    y0 + (y1 - y0) * u, 0.0f);
            // NOT isCellOccupied: unknown must count as blocked here. An
            // unexplored corridor is exactly the case where a straight-line
            // edge would claim a connection nobody has verified.
            if (!explo_planner::isCellFree(pm, p)) ++blocked;
          }
          return static_cast<double>(blocked) / (kSamples + 1);
        });
  } else {
    cell_world_.rebuildEdges();
  }

  const CellWorld::Census cs = cell_world_.census();
  CellCensusEvent ev;
  ev.cells_total          = cell_world_.size();
  ev.unseen               = cs.unseen;
  ev.exploring            = cs.exploring;
  ev.covered              = cs.covered;
  ev.exploring_by_others  = cs.exploring_by_others;
  ev.covered_by_others    = cs.covered_by_others;
  ev.covered_fraction     = cs.coveredFraction();
  ev.roi_unknown_fraction = roi_unknown_fraction;
  ev.coverage_source      = coverage_source;
  ev.changed              = changed;
  ev.cell_size_m          = cell_world_.grid().cell_size_m;
  ev.nx                   = cell_world_.grid().nx;
  ev.ny                   = cell_world_.grid().ny;
  ev.grid_hash            = cell_world_.grid().configHash();
  ev.shared_hash          = cell_world_.sharedHash();

  // Commits and edges in one sweep over the unordered neighbour pairs. Only
  // the +x, +y and the two diagonals are visited, so each edge is counted
  // once; the mirror pairs are the same edge.
  const CellGrid& g = cell_world_.grid();
  static const int kDx[4] = {1, 0, 1,  1};
  static const int kDy[4] = {0, 1, 1, -1};
  // Only cells with observed_columns >= min_observed_columns enter the
  // distribution; an unmeasured cell reports unknownFraction() == 1 and would
  // skew the quantiles. (notes: cell-census-measured-cells-only)
  std::vector<double> cell_u, cell_ff;
  cell_u.reserve(static_cast<size_t>(cell_world_.size()));
  cell_ff.reserve(static_cast<size_t>(cell_world_.size()));
  double best_u = 2.0, ff_at_best_u = -1.0;
  for (int id = 0; id < cell_world_.size(); ++id) {
    const CellWorld::Cell& c = cell_world_.cell(id);
    ev.commits_total += c.update_id;
    if (c.obs.observed_columns >= cell_world_.config().min_observed_columns) {
      ++ev.cells_measured;
      const double u = c.obs.unknownFraction();
      // Frontier as a fraction of the cell's own observed voxels. observed
      // cannot be 0 here: a cell with no voxels at all has no observed columns
      // either and never reaches this branch. Guarded anyway, because the
      // consequence of being wrong is a division that poisons a quantile.
      const double ff =
          c.obs.observed_voxels > 0
              ? static_cast<double>(c.obs.frontier_voxels) /
                    static_cast<double>(c.obs.observed_voxels)
              : 0.0;
      cell_u.push_back(u);
      cell_ff.push_back(ff);
      // Strict <, so ties keep the FIRST cell rather than the last. Which cell
      // wins a tie does not matter; that the choice is deterministic does,
      // since this field is read across rows as if it tracked one candidate.
      if (u < best_u) { best_u = u; ff_at_best_u = ff; }
      if (ff <= cell_world_.config().covered_max_frontier_frac)
        ++ev.cells_frontier_ok;
    }
    const int cx = g.col(id), cy = g.row(id);
    for (int k = 0; k < 4; ++k) {
      const int nx2 = cx + kDx[k], ny2 = cy + kDy[k];
      if (nx2 < 0 || nx2 >= g.nx || ny2 < 0 || ny2 >= g.ny) continue;
      ++ev.edges_total;
      if (cell_world_.edgeEnabled(id, ny2 * g.nx + nx2)) ++ev.edges_enabled;
    }
  }
  if (!cell_u.empty()) {
    std::sort(cell_u.begin(), cell_u.end());
    // Nearest-rank percentiles, so every reported value is a real cell's
    // measurement rather than an interpolation.
    // (notes: cell-census-nearest-rank-pct)
    auto pct = [&cell_u](double q) {
      const size_t n = cell_u.size();
      size_t k = static_cast<size_t>(std::ceil(q * static_cast<double>(n)));
      if (k == 0) k = 1;
      return cell_u[std::min(k, n) - 1];
    };
    ev.cell_unknown_min    = cell_u.front();
    ev.cell_unknown_p10    = pct(0.10);
    ev.cell_unknown_median = pct(0.50);
    // Sorted separately and deliberately: these are the frontier marginal, not
    // the frontier of the cells that produced the unknown quantiles. The joint
    // question is the field below, which was captured before either sort.
    std::sort(cell_ff.begin(), cell_ff.end());
    ev.cell_frontier_frac_min    = cell_ff.front();
    ev.cell_frontier_frac_median =
        cell_ff[std::min(static_cast<size_t>(std::ceil(0.5 * cell_ff.size())),
                         cell_ff.size()) - 1];
    ev.cell_frontier_frac_at_best_unknown = ff_at_best_u;
  }

  if (exp_log_) exp_log_->logCellCensus(expCtx(), ev);
  publishCellViz();
}

void ExploPlannerNode::publishCellViz() {
  if (!cell_viz_pub_ || cell_viz_pub_->get_subscription_count() == 0) return;

  const CellGrid& g = cell_world_.grid();
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(clear);

  for (int id = 0; id < cell_world_.size(); ++id) {
    float cx = 0.0f, cy = 0.0f;
    g.centre(id, cx, cy);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = this->now();
    m.ns = "cells";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = cx;
    m.pose.position.y = cy;
    // Below the robot and below the candidate arrows, and flat: this is a
    // ground shading layer, not an obstacle.
    m.pose.position.z = -0.05;
    m.pose.orientation.w = 1.0;
    // Inset so the cell boundaries stay legible without a separate line list.
    m.scale.x = m.scale.y = g.cell_size_m * 0.92;
    m.scale.z = 0.02;
    // First-hand statuses are saturated, second-hand ones are the same hue
    // desaturated — so "I saw this" and "somebody told me" are distinguishable
    // at a glance, which is the whole point of keeping them separate locally.
    switch (cell_world_.status(id)) {
      case CellStatus::UNSEEN:
        m.color.r = 0.35f; m.color.g = 0.35f; m.color.b = 0.35f; break;
      case CellStatus::EXPLORING:
        m.color.r = 0.95f; m.color.g = 0.75f; m.color.b = 0.10f; break;
      case CellStatus::COVERED:
        m.color.r = 0.10f; m.color.g = 0.85f; m.color.b = 0.25f; break;
      case CellStatus::EXPLORING_BY_OTHERS:
        m.color.r = 0.60f; m.color.g = 0.55f; m.color.b = 0.35f; break;
      case CellStatus::COVERED_BY_OTHERS:
        m.color.r = 0.35f; m.color.g = 0.60f; m.color.b = 0.40f; break;
    }
    m.color.a = 0.35f;
    ma.markers.push_back(m);
  }

  cell_viz_pub_->publish(ma);
}

// ==================================================================
// Mission clock
// ==================================================================

double ExploPlannerNode::missionElapsed() {
  if (mission_t0_sec_ < 0.0) {
    // Under use_sim_time this->now() is 0 until the first /clock; return -1
    // rather than anchor the baseline at 0 (the same guard startExperimentLog
    // uses). (notes: mission-elapsed-clock-not-live)
    const auto now = this->now();
    if (now.nanoseconds() <= 0) return -1.0;
    mission_t0_sec_ = now.seconds();
  }
  // Clamped at 0 rather than allowed negative. A sim clock that jumps
  // backwards (a reset simulator) would otherwise hand out negative elapsed
  // times, which every age test downstream would misread — a condition that
  // ought to cost one tick.
  return std::max(0.0, this->now().seconds() - mission_t0_sec_);
}

double ExploPlannerNode::missionElapsedAt(const rclcpp::Time& when) const {
  // Does not latch: only reads the baseline missionElapsed() sets. Returns -1
  // before the first live tick, and callers must handle that.
  // (notes: mission-elapsed-at-no-latch)
  if (mission_t0_sec_ < 0.0) return -1.0;
  if (when.nanoseconds() <= 0) return -1.0;
  const double e = when.seconds() - mission_t0_sec_;
  // Negative means `when` predates the baseline, which for the rendezvous
  // anchor means the team was already complete before this node's clock went
  // live. Refusing is right: the interval would be anchored before the mission
  // started and every deadline computed from it would be in the past.
  return e < 0.0 ? -1.0 : e;
}


// ==================================================================
// Team beacon (gen 34, §8.2)
// ==================================================================

// Once a second, in every state including DONE: TeamCore's beacon plus the
// node's parts (the exploration goal, the census and the tour). Nothing is
// sent before TeamCore's first tick, which needs a live clock and a pose.
void ExploPlannerNode::publishBeacon() {
  if (!beacon_pub_ || !team_ || !team_ticked_) return;
  const gen34::Beacon b = team_->beacon();

  explo_planner_msgs::msg::TeamBeacon m;
  m.header.stamp    = this->now();
  m.header.frame_id = map_frame_;
  m.robot_id  = static_cast<uint8_t>(b.robot_id);
  m.team_hash = b.team_hash;
  m.grid_hash = b.grid_hash;
  m.position.x = b.position.x;
  m.position.y = b.position.y;
  m.position.z = latest_pos_.z();
  // The goal is the chase trail's second point: a Leg's point from TeamCore,
  // else the exploration goal while it is being driven (or held).
  const State ds = (state_ == State::PROXIMITY_HOLD) ? prox_resume_state_
                                                     : state_;
  if (b.has_goal) {
    m.goal.x = b.goal.x;
    m.goal.y = b.goal.y;
    m.goal.z = latest_pos_.z();
    m.has_goal = true;
  } else if (ds == State::NAVIGATE) {
    m.goal.x = current_goal_.position.x();
    m.goal.y = current_goal_.position.y();
    m.goal.z = current_goal_.position.z();
    m.has_goal = true;
  }
  m.hears_mask    = b.hears_mask;
  m.activity      = static_cast<uint8_t>(b.activity);
  m.finished      = b.finished;
  m.team_finished = b.team_finished;
  m.home          = b.home;
  m.map_sent_seq  = b.map_sent_seq;
  m.map_rcvd_seq  = b.map_rcvd_seq;
  m.exchanged_mask = b.exchanged_mask;
  m.plan_version  = b.plan.version;
  m.plan_cell     = b.plan.cell;
  m.plan_center.x = b.plan.center.x;
  m.plan_center.y = b.plan.center.y;
  m.plan_t0_sec   = b.plan.t0;
  m.plan_interval_sec    = b.plan.interval;
  m.plan_renewed_at_slot = b.plan.renewed_at_slot;
  m.plan_provisional     = b.plan.provisional;
  m.meeting_slot  = b.meeting_slot;
  m.met_mask      = b.met_mask;

  if (cell_world_.configured()) {
    const std::vector<CellWorld::WireCell> wire = cell_world_.toWire();
    m.cells.reserve(wire.size());
    for (const CellWorld::WireCell& w : wire) {
      explo_planner_msgs::msg::CellState c;
      c.id        = w.id;
      c.status    = w.status;
      c.known_by  = w.known_by;
      c.update_id = w.update_id;
      m.cells.push_back(c);
    }
  }

  // Send my_tour_ only while this robot explores it (a hold judged by the
  // drive it interrupted), else clear it; empty means no route. Ids beyond
  // uint16 truncate the tour. (notes: team-world-my-tour-live-only)
  const bool tour_is_live =
      !finished_ &&
      (ds == State::PLAN || ds == State::NAVIGATE ||
       ds == State::INTEGRATE || ds == State::LOG_STEP);
  if (!tour_is_live) my_tour_.clear();
  m.my_tour.reserve(my_tour_.size());
  for (int cid : my_tour_) {
    if (cid < 0 ||
        cid > static_cast<int>(std::numeric_limits<uint16_t>::max())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "Beacon: tour cell %d does not fit the uint16 wire field; "
          "broadcasting the route truncated at %zu of %zu cells.",
          cid, m.my_tour.size(), my_tour_.size());
      break;
    }
    m.my_tour.push_back(static_cast<uint16_t>(cid));
  }

  beacon_pub_->publish(m);
  ++beacons_sent_;
}

// Every pending beacon into TeamCore, then its census into the cell world.
// Runs on the tick before TeamCore does. A beacon TeamCore refuses (another
// fleet, a bad id, an older stamp) is not merged: its ids do not mean ours.
void ExploPlannerNode::drainBeacons() {
  if (!team_ || beacon_pending_.empty()) return;
  // Nothing can be dated before the clock is live; the slots are left, not
  // dropped, since each is full state.
  if (this->now().nanoseconds() <= 0) return;

  std::map<int, PendingBeacon> batch;
  batch.swap(beacon_pending_);
  const bool grid_on = cell_world_.configured();
  const uint32_t my_grid = grid_on ? cell_world_.grid().configHash() : 0u;

  for (auto& kv : batch) {
    const int sid = kv.first;
    if (!kv.second.msg || sid < 0 || sid >= fleet_.size()) {
      ++beacons_rejected_;
      continue;
    }
    const explo_planner_msgs::msg::TeamBeacon& msg = *kv.second.msg;
    if (msg.activity >
        static_cast<uint8_t>(gen34::Activity::kDone)) {
      ++beacons_rejected_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "Beacon from robot %d: activity %u is not one this node knows. "
          "Dropping it.", sid, static_cast<unsigned>(msg.activity));
      continue;
    }
    const bool same_grid = grid_on && msg.grid_hash == my_grid;

    gen34::Beacon b;
    b.robot_id  = sid;
    b.team_hash = msg.team_hash;
    b.grid_hash = msg.grid_hash;
    b.stamp     = rclcpp::Time(msg.header.stamp).seconds();
    b.position  = gen34::Vec2{msg.position.x, msg.position.y};
    b.goal      = gen34::Vec2{msg.goal.x, msg.goal.y};
    b.has_goal  = msg.has_goal;
    b.hears_mask = msg.hears_mask;
    b.activity  = static_cast<gen34::Activity>(msg.activity);
    b.finished  = msg.finished;
    b.team_finished = msg.team_finished;
    b.home      = msg.home;
    b.map_sent_seq = msg.map_sent_seq;
    b.map_rcvd_seq = msg.map_rcvd_seq;
    b.exchanged_mask = msg.exchanged_mask;
    b.plan.version  = msg.plan_version;
    b.plan.cell     = msg.plan_cell;
    b.plan.center   = gen34::Vec2{msg.plan_center.x, msg.plan_center.y};
    b.plan.t0       = msg.plan_t0_sec;
    b.plan.interval = msg.plan_interval_sec;
    b.plan.renewed_at_slot = msg.plan_renewed_at_slot;
    b.plan.provisional     = msg.plan_provisional;
    b.meeting_slot = msg.meeting_slot;
    b.met_mask     = msg.met_mask;
    // Tour cell ids name our ground only on the same grid; an id off our grid
    // is dropped rather than handed to the predictor.
    if (same_grid) {
      b.tour.reserve(msg.my_tour.size());
      for (uint16_t c : msg.my_tour) {
        const int id = static_cast<int>(c);
        if (cell_world_.grid().valid(id)) b.tour.push_back(id);
      }
    }

    if (!team_->onBeacon(b, kv.second.received.seconds())) {
      ++beacons_rejected_;
      continue;
    }

    // The census, after TeamCore accepted the sender. Local priority anchors
    // on the cell this robot stands in; -1 disables the rule.
    // (notes: team-merge-local-priority-centre)
    if (same_grid) {
      std::vector<CellWorld::WireCell> wire;
      wire.reserve(msg.cells.size());
      for (const auto& c : msg.cells) {
        CellWorld::WireCell w;
        w.id        = c.id;
        w.status    = c.status;
        w.known_by  = c.known_by;
        w.update_id = c.update_id;
        wire.push_back(w);
      }
      const int centre =
          team_merge_local_priority_
              ? cell_world_.grid().idAt(latest_pos_.x(), latest_pos_.y())
              : -1;
      const CellWorld::MergeStats st =
          cell_world_.mergeWire(sid, wire, centre);
      if (!st.refused.empty()) {
        ++cell_merges_refused_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
            "Cell census from %s refused: %s", fleet_.nameOf(sid).c_str(),
            st.refused.c_str());
      } else {
        team_merge_applied_total_ += st.applied;
      }
    }

    // A beacon position is a fresh peer pose for the guard (it is sent from
    // every state, where the intent heartbeat is not).
    if (prox_guard_) {
      prox_guard_->onPeerPose(
          fleet_.nameOf(sid),
          Eigen::Vector3f(static_cast<float>(msg.position.x),
                          static_cast<float>(msg.position.y),
                          static_cast<float>(msg.position.z)),
          kv.second.received);
    }
    expPeerHeard(fleet_.nameOf(sid));
    ++beacons_received_;
  }
}

// ==================================================================
// TeamCore's world questions (NodeOracle)
// ==================================================================

// Robot 0's plan solve: the scheduler's meeting cell over the allocator's
// tours. Before any robot has a tour, and without the allocator, the team's
// centroid cell; provisional only when a solve could later replace it.
gen34::PlanSolve ExploPlannerNode::oracleSolvePlan(
    const std::vector<gen34::TeamRobotView>& team) {
  gen34::PlanSolve out;
  double sx = 0.0, sy = 0.0;
  int k = 0;
  for (const gen34::TeamRobotView& r : team) {
    if (!r.known) continue;
    sx += r.position.x;
    sy += r.position.y;
    ++k;
  }
  if (k == 0) {
    out.refused = "no robot located";
    return out;
  }
  const gen34::Vec2 centroid{sx / k, sy / k};
  out.ok = true;
  out.cell = -1;
  out.center = centroid;
  if (!cell_world_.configured()) return out;

  const CellGrid& g = cell_world_.grid();
  auto cellCentre = [&g](int id) {
    float x = 0.f, y = 0.f;
    g.centre(id, x, y);
    return gen34::Vec2{x, y};
  };
  const int floor_cell = g.idAt(static_cast<float>(centroid.x),
                                static_cast<float>(centroid.y));
  if (floor_cell >= 0) {
    out.cell = floor_cell;
    out.center = cellCentre(floor_cell);
  }
  if (!global_alloc_enable_) return out;

  std::vector<AllocRobot> robots;
  robots.reserve(team.size());
  for (const gen34::TeamRobotView& r : team) {
    AllocRobot a;
    a.id = r.id;
    a.cell = r.known ? g.idAt(static_cast<float>(r.position.x),
                              static_cast<float>(r.position.y))
                     : -1;
    // A team plan: every robot counts as reachable.
    a.in_comms = true;
    a.finished = r.finished;
    a.off_frontier = r.finished;
    robots.push_back(a);
  }
  const Allocation alloc = GlobalAllocator::solve(cell_world_, robots,
                                                  alloc_cfg_);
  bool any_tour = false;
  for (const auto& t : alloc.tours) any_tour = any_tour || !t.empty();
  if (!alloc.refused.empty() || !any_tour) {
    out.provisional = true;
    return out;
  }
  // Finished robots come to meetings too, so the scheduler sees everyone.
  std::vector<AllocRobot> meet = robots;
  for (AllocRobot& a : meet) {
    a.finished = false;
    a.off_frontier = false;
  }
  RendezvousScheduler::Config rc;
  rc.speed_mm_s =
      std::max(1LL, std::llround(team_cfg_.travel_speed_mps * 1000.0));
  rc.depart_safety_milli =
      static_cast<int>(std::lround(team_cfg_.depart_safety * 1000.0));
  rc.min_interval_ms = 0;
  rc.max_interval_ms = 0;
  const RendezvousPlan rp = RendezvousScheduler::solve(
      cell_world_, meet, alloc, floor_cell, 0, rc);
  if (!rp.valid() || !g.valid(rp.cell)) {
    out.provisional = true;
    return out;
  }
  out.cell = rp.cell;
  out.center = cellCentre(rp.cell);
  out.provisional = false;
  return out;
}

// Metres to the plan cell: the allocator's cell-to-cell cost, never less than
// the straight line; the straight line without a cell world.
double ExploPlannerNode::oraclePathDistance(const gen34::Vec2& from, int cell,
                                            const gen34::Vec2& center) const {
  const double straight = std::hypot(center.x - from.x, center.y - from.y);
  if (!cell_world_.configured() || !cell_world_.grid().valid(cell))
    return straight;
  const int from_cell = cell_world_.grid().idAt(static_cast<float>(from.x),
                                                static_cast<float>(from.y));
  if (from_cell < 0) return straight;
  const double path_m =
      static_cast<double>(GlobalAllocator::costMm(cell_world_, from_cell,
                                                  cell)) / 1000.0;
  return std::max(straight, path_m);
}

// Known free on the planning map; anywhere without one.
bool ExploPlannerNode::oracleStandable(const gen34::Vec2& p) const {
  if (!latest_plan_map_) return true;
  return isCellFree(Eigen::Vector3f(static_cast<float>(p.x),
                                    static_cast<float>(p.y), latest_pos_.z()));
}

// No occupied cell on the segment, sampled every 0.2 m; clear without a map.
// Unknown cells do not block, as for the frontier goals.
bool ExploPlannerNode::oracleLineOfSight(const gen34::Vec2& a,
                                         const gen34::Vec2& b) const {
  if (!latest_plan_map_) return true;
  const double len = std::hypot(b.x - a.x, b.y - a.y);
  const int n = std::max(1, static_cast<int>(std::ceil(len / 0.2)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const Eigen::Vector3f q(static_cast<float>(a.x + t * (b.x - a.x)),
                            static_cast<float>(a.y + t * (b.y - a.y)),
                            latest_pos_.z());
    if (isCellOccupied(q)) return false;
  }
  return true;
}

// The value gate over the missing peers (evaluateReconnectGate). Without the
// allocator and a cell world it cannot price, and TeamCore fails open.
gen34::ChaseGateVerdict ExploPlannerNode::oracleChaseGate(
    const std::vector<gen34::ChaseGateView>& missing) const {
  gen34::ChaseGateVerdict v;
  if (!global_alloc_enable_ || !cell_world_.configured()) {
    v.dispatch = true;
    v.refused = "allocator off";
    return v;
  }
  const CellGrid& g = cell_world_.grid();
  std::vector<MissingPeer> mp;
  mp.reserve(missing.size());
  for (const gen34::ChaseGateView& m : missing) {
    MissingPeer p;
    p.id = m.id;
    p.cell = m.known ? g.idAt(static_cast<float>(m.last_position.x),
                              static_cast<float>(m.last_position.y))
                     : -1;
    p.finished = m.finished;
    mp.push_back(p);
  }
  const std::vector<AllocRobot> robots =
      allocVehicles(latest_pos_.x(), latest_pos_.y(), nullptr, 0.0, 0.0);
  const GateVerdict gv =
      evaluateReconnectGate(cell_world_, robots, mp, alloc_cfg_);
  v.dispatch = gv.dispatch;
  v.peer = gv.leg_peer_id;
  v.refused = gv.refused;
  v.c_no_mm = gv.c_no_mm;
  v.c_re_mm = gv.c_re_mm;
  v.leg_mm = gv.leg_mm;
  v.unshared_cells = gv.unshared_cells;
  return v;
}

// The intercept point on the missing peer's announced tour
// (PursuitPredictor). False sends the chase down the trail.
bool ExploPlannerNode::oracleIntercept(const gen34::ChaseGateView& peer,
                                       gen34::Vec2* out) const {
  if (!cell_world_.configured() || peer.tour.empty() || !have_pose_)
    return false;
  PursuitPredictor::PeerTrack t;
  t.tour = peer.tour;
  t.x = peer.last_position.x;
  t.y = peer.last_position.y;
  t.have_position = peer.known;
  t.age_sec = peer.age_sec;
  const int my_cell =
      cell_world_.grid().idAt(latest_pos_.x(), latest_pos_.y());
  const PursuitTarget pt =
      PursuitPredictor::predict(cell_world_, t, my_cell, pursuit_mdp_cfg_);
  if (!pt.valid()) return false;
  *out = gen34::Vec2{pt.x, pt.y};
  return true;
}

} // namespace explo_planner

// ==================================================================
// Entry point
// ==================================================================

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Single-threaded executor: all callbacks and timers share one thread, so the
  // map callback and state machine cannot race. Construction can throw on a
  // malformed fleet; caught for one FATAL line and exit 2.
  // (notes: main-single-thread-executor)
  try {
    rclcpp::spin(std::make_shared<explo_planner::ExploPlannerNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("explo_planner"),
                 "refusing to start: %s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
