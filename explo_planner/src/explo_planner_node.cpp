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
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <scovox_msgs/msg/scovox_map.hpp>
#include <scovox_msgs/msg/refinement_region.hpp>
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
#include "explo_planner/experiment_log.hpp"
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
  // Mesh-reconnection pursuit (robot-carried radios). Instead of driving home
  // to the anchor, chase the missing peer's last declared goal (and its last
  // heard pose) on a staleness-scaled budget; the mesh re-forms the moment we
  // come within range, no arrival needed. Budget spent -> fall back to the
  // deterministic meeting point (hybrid) or hold in place (pursuit). Gated by
  // reconnect_mode_; see the pursuit_* params.
  PURSUE,
  // Coordinated proximity stop (multi-robot). A DRIVING robot that has lost
  // right-of-way to a nearby moving teammate cancels its nav goal and parks
  // here until the peer clears off or parks, then resumes the same goal.
  // Entered only from NAVIGATE / RETURN_NAV / PURSUE; see checkProximityHold().
  PROXIMITY_HOLD
};

// Stable, machine-readable state names. These are wire/CSV values consumed by
// the offline analysis (merge attribution classifies each contact event by the
// planner state at contact time), so treat them as an interface: renaming one
// silently re-buckets every past run's events. No default case — adding a
// State without a name here is a compile warning, not a mystery at analysis
// time.
inline const char* stateName(State s) {
  switch (s) {
    case State::WAIT_FOR_MAP:   return "WAIT_FOR_MAP";
    case State::PLAN:           return "PLAN";
    case State::NAVIGATE:       return "NAVIGATE";
    case State::INTEGRATE:      return "INTEGRATE";
    case State::LOG_STEP:       return "LOG_STEP";
    case State::DONE:           return "DONE";
    case State::EXPLOIT_PLAN:   return "EXPLOIT_PLAN";
    case State::EXPLOIT_DWELL:  return "EXPLOIT_DWELL";
    case State::RETURN_NAV:     return "RETURN_NAV";
    case State::RETURN_SYNC:    return "RETURN_SYNC";
    case State::PURSUE:         return "PURSUE";
    case State::PROXIMITY_HOLD: return "PROXIMITY_HOLD";
  }
  return "UNKNOWN";
}

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
  /// Last chance to close the event log. rclcpp::shutdown() (SIGTERM from the
  /// campaign harness, or the DONE-shutdown path) unwinds spin() and destroys
  /// the node, and a run that ends that way would otherwise have no run_end
  /// line at all — indistinguishable, from the file alone, from a truncated
  /// one. Never throws (see the definition).
  ~ExploPlannerNode() override;

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
  /// `reason` names WHY the transition happened and is recorded verbatim in the
  /// event log's state_change event. Defaulted so a new call site compiles, but
  /// every existing one passes a stable, machine-readable tag: these strings are
  /// an analysis interface exactly as stateName() is (see the event log header
  /// on why grepping prose was the thing this replaces).
  void transitionTo(State s, const char* reason = "");
  void doPlan();
  void doNavigate();
  void doIntegrate();
  void doLogStep();

  // CSV row assembly. fillCommonMetrics populates every column that is a
  // property of the world right now (map stats, odometry, peers, holds, state,
  // reconnect clock) and is shared by both emitters; the plan-attribution
  // columns are left at zero for it to stay honest on a timer row, and doLogStep
  // adds them for genuine end-of-step rows. metricsTick is the periodic
  // sampler that runs in every state.
  void fillCommonMetrics(StepMetrics& m);
  void metricsTick();

  // Experiment event log (experiment_log.hpp). The CSV above is unchanged and
  // stays the per-step record; this is the authoritative, machine-readable
  // event stream on the node's own sim clock.
  //
  // expCtx() is the per-event envelope (sim stamp + state + step) and is the
  // ONLY place the log's time axis is read, so no event can be stamped from a
  // clock the planner does not make decisions on.
  ExperimentContext expCtx();
  /// Emit run_start on the first tick with a live clock — NOT in the
  /// constructor. Under use_sim_time this->now() reads 0 until the first
  /// /clock message, and a t0 of 0 would silently turn every t_rel_sec in the
  /// file into an absolute sim time. No-op once started.
  void startExperimentLog();
  /// Emit clock_anchor on a pure SIM-TIME schedule. Never consults a wall clock
  /// to decide whether to fire — that is the failure mode next door in
  /// metricsTick, where a wall deadline armed after the tick's own work
  /// suppresses whole sim-time ticks at some real-time factors and none at
  /// others, silently changing the realised rate between campaigns.
  void expClockAnchorTick();
  /// Peer-belief bookkeeping behind peer_seen / peer_lost. expPeerHeard is
  /// called from the intent callback (every teammate broadcast, in every arm —
  /// the subscription is wired even with coordination off); expPeerSweep runs
  /// on the state-machine tick and flips a peer to LOST once its last intent is
  /// older than the claim TTL. Deliberately computed from the intent stream
  /// rather than Coordination's table so the control arm, which runs with
  /// coordination_enabled=false, still records outage timing.
  void expPeerHeard(const std::string& peer_id);
  void expPeerSweep();
  /// Emit reconnect_dispatch for the manoeuvre just chosen. Called from the
  /// leaf that COMMITS the action (startPursuit / startReturnTo / holdForTeam /
  /// pursuitExploreFallback), never from the branch that contemplates it, so
  /// the event set and the behaviour cannot disagree. `dest` is null for
  /// actions with no destination.
  void logReconnectDispatch(const char* action, const Eigen::Vector3f* dest,
                            double budget_sec, const char* reason);
  /// Re-read the missing-peer context (id and record age) from the CURRENT
  /// contact table, for leaves that commit out of band from
  /// dispatchReconnect's branch walk and would otherwise report the age that
  /// walk saw — stale by a whole barrier wait or a whole chase budget.
  void refreshDispatchContext();
  /// Emit run_end exactly once (later calls are ignored by the logger).
  void logRunEnd(const char* reason);

  // Rendezvous (multi-robot reconnection). finishOrRendezvous decides, at
  // exploration exhaustion, between DONE and the reconnect_mode_ manoeuvre
  // (return-to-anchor, pursuit, or pursuit-then-meeting-point);
  // startReturnTo arms a drive to any barrier destination (anchor or meeting
  // point); doReturnNav drives there; doReturnSync holds until the whole team
  // is in comms. `reason` is the termination cause, for logging only — EVERY
  // termination path must route through here, not just coverage saturation.
  /// Decide the ending at exhaustion. Returns false when the decision was
  /// DEFERRED by the reconnect confirmation gate and no transition happened —
  /// the caller must then leave the node somewhere that re-runs this check.
  bool finishOrRendezvous(const char* reason);
  /// The `exploration_complete` event — THIS robot declaring its own
  /// exploration exhausted, emitted at the instant of the declaration and
  /// before anything is decided about what happens next. Factored out of
  /// finishOrRendezvous so the latch criterion, which skips the rendezvous
  /// decision entirely, still records the same event at the same instant.
  void recordExplorationComplete(const char* reason);
  /// The ending itself: keep beaconing if DONE-idle, then transition. Shared by
  /// both criteria so there is exactly one place a run can end.
  bool finishNow(const char* reason);
  /// Latched, state-blind completion test (done_criterion == "latch"). Takes an
  /// already-measured ROI unknown fraction — every caller has just computed one
  /// and a second ROI walk is the most expensive thing in the tick. Returns
  /// true on the call that latched, false on every other call.
  bool maybeLatchCoverageDone(double unk, const char* source);
  /// The manoeuvre dispatch itself (mode -> pursuit / meeting point / anchor /
  /// hold), factored out of finishOrRendezvous so the mid-run trigger in
  /// doPlan can arm the same manoeuvres without the DONE fallthrough. Always
  /// transitions into a manoeuvre state and returns true (kept boolean for
  /// call-site symmetry with startPursuit).
  bool dispatchReconnect(const char* reason);
  /// Flicker guard for manoeuvre release: true once `eligible` has held
  /// continuously for reconnect_release_confirm_sec (immediately when the
  /// window is 0). Resets whenever eligible drops or the state changes.
  bool releaseConfirmed(bool eligible);
  void startReturnTo(const Eigen::Vector3f& dest, const char* what,
                     const char* reason);
  void doReturnNav();
  void doReturnSync();
  // Mesh-reconnection pursuit (see the PURSUE state). startPursuit arms the
  // chase along the missing peer's last-contact trail (returns false when the
  // record is too stale to be worth chasing — pursuitBudgetSec() == 0);
  // armPursuitWaypoint publishes the current trail waypoint as the nav goal;
  // doPursue drives the trail under the budget; pursuitFallback routes a
  // spent/failed chase to the mode's fallback (meeting point or hold-here);
  // holdForTeam raises the RETURN_SYNC barrier at the CURRENT pose.
  // standDownExploitation is the shared open-target demotion every barrier
  // entry performs (factored out of the old startReturnToAnchor).
  struct LastContact;  // defined with the members below
  bool startPursuit(const std::string& peer_id, const LastContact& rec,
                    const char* reason);
  void armPursuitWaypoint();
  void doPursue();
  void pursuitFallback(const char* why);
  void holdForTeam(const char* why);
  // Abandon the current manoeuvre and go back to exploring. Shared by pure
  // pursuit's fallback and the mid-run barrier expiry — both mean "this
  // attempt is over and there is still map to cover".
  void resumeExploring(const char* why);
  // Pure pursuit's fallback, tried before holdForTeam. Returns false when the
  // fallback is disabled or its budget is spent, and the caller holds.
  bool pursuitExploreFallback(const char* why);
  void standDownExploitation();
  // Presence-only intent (goal = own pose), kept fresh by the heartbeat.
  // Published at every barrier hold and at DONE-idle entry: a parked robot
  // must stay countable by livePeerCount or a teammate that finishes later
  // waits forever on a robot that is metres away and silent.
  void publishPresenceIntent();
  // Freshest last-contact record whose producer is NOT currently live — the
  // teammate the barrier is actually waiting on. nullptr when every recorded
  // peer is live (the missing one was never heard at all). `peer_id_out`
  // receives the record's robot id when non-null.
  const LastContact* missingPeerRecord(std::string* peer_id_out);

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
  // The three peer-claim rules of the vantage filter (same-target exploit
  // claim = unconditional veto; other claim shapes = distance contest; parked
  // staged peer = position contest), extracted so doExploitPlan's candidate
  // walk and its hold branch consult ONE implementation — two hand-copies of
  // these rules would drift, and a hold decision computed from different rules
  // than the selection it suppresses could deadlock a ring. Returns true if a
  // peer claim denies `v_pos`. Caller gates on coord_->enabled().
  bool vantageVetoedByPeers(uint32_t target_id, const Eigen::Vector3f& v_pos,
                            const Eigen::Vector3f& robot_pos);
  // Hold branch of doExploitPlan: true when this robot should PARK on the
  // vantage it already dwelled (held_vantage_*) because every other angle of
  // the ring is either team-visited or peer-denied. Publishes the staged hold
  // claim / re-anchor goal as needed; the caller returns from the tick when
  // this returns true.
  bool holdDwelledVantage(Target* tgt,
                          const std::vector<CandidateViewpoint>& vantages,
                          const Eigen::Vector3f& robot_pos);
  void finishActiveTarget(bool success);
  // Fine-TSDF region relay: register (remove=false) / unregister (remove=true)
  // target `id` as a refinement cylinder on this robot's scovox_node.
  void publishRefinementRegion(uint32_t id, const Eigen::Vector3f& center,
                               float radius, bool remove);
  // Terminal cleanup: unregister every region whose target never reached
  // DONE (finishActiveTarget already removed the DONE ones).
  void removeLiveRefinementRegions();
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
  /// Stop the platform when a state transition ABANDONS an in-flight drive
  /// without immediately replacing it with another goal. Same stop mechanics as
  /// enterProximityHold (cancel-all + brake goal at our own pose), none of the
  /// hold bookkeeping. See the definition for the invariant this enforces and
  /// the collision that motivated it.
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

  /// Standing / sightline height for an exploitation point at (x, y). Flat
  /// mode returns the fixed absolute vantage height. Terrain mode snaps to the
  /// local ground + candidate clearance, the same way exploration candidates
  /// are placed — see the definition for why the absolute height is unusable
  /// once the map z-band is robot-relative.
  float exploitZAt(float x, float y) const;

  // --- Parameters ---
  std::string robot_name_;
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
  // A TF lookup that SUCCEEDS can still be a corpse: with a dead broadcaster
  // tf2 serves the newest stored transform forever (TimePointZero reads never
  // prune), so without an age gate the planner keeps planning, logging and
  // braking against a pose frozen at the moment the localiser died.
  // Transforms older than this (sec) read as pose lost. 0 = disabled.
  double pose_max_age_sec_ = 5.0;
  double failed_goal_radius_m_;
  double failed_goal_ttl_sec_;
  double done_unknown_fraction_;
  int    done_min_consecutive_steps_;
  // Which rule decides that THIS robot has finished exploring:
  //  - "latch" (default): the first tick on which the ROI unknown fraction
  //    reaches done_unknown_fraction, in ANY state and regardless of who is in
  //    comms range. Latched: it never un-finishes. No streak, no rendezvous.
  //  - "streak": the legacy rule — done_min_consecutive_steps consecutive PLAN
  //    ticks below threshold, then finishOrRendezvous, which may spend a
  //    reconnect manoeuvre before DONE. Kept so banked campaigns remain
  //    reproducible; see the param load for why the default changed.
  std::string done_criterion_{"latch"};
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
  // Extra retention for EXPLOIT claims past their TTL (Coordination ctor doc
  // has the full receive-side-starvation incident). 2x the TTL by default:
  // long enough to ride out the multi-second executor gaps observed in sim,
  // short enough that a genuinely dead winner frees its vantage well inside
  // the 300 s per-target budget.
  double coord_claim_grace_sec_ = 10.0;
  double coord_heartbeat_hz_   = 1.0;
  // Heartbeat-suppression episode tracking (see heartbeatTick). Lets the
  // analysis separate "peer missing because the radio was down" from "peer
  // missing because its planner was busy in a non-beaconing state" — the two
  // are identical in coord_active_peers, and only one of them is a comms
  // result.
  bool hb_suppressed_ = false;
  bool hb_suppress_warned_ = false;
  // Previous heartbeatTick entry, for late-tick (executor starvation)
  // detection, and a running count of ticks later than the claim TTL.
  rclcpp::Time hb_last_tick_;
  int  hb_late_count_ = 0;
  rclcpp::Time hb_suppress_start_;
  std::string coord_intent_topic_;
  // Intent stream, split. By default the planner pubs and subs the SAME
  // global topic (coord_intent_topic_), which means no external process can
  // sit between two robots' intents — and a peer that reads as *missing* is
  // the trigger for every reconnect manoeuvre, so a comms emulator that
  // cannot gate this stream cannot exercise them at all. These two params
  // separate the ends: publish to one topic, subscribe to a list of others
  // (typically the emulator's relayed copies, /<self>/rx/<peer>/...). Both
  // default empty -> fall back to coord_intent_topic_, i.e. today's exact
  // behaviour, self-echo included (filtered downstream as before).
  std::string coord_intent_pub_topic_;
  std::vector<std::string> coord_intent_sub_topics_;
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
  // --- Mesh reconnection (robot-carried radios) params ---
  // With the radios on the robots (peer-to-peer mesh, no base station) the
  // anchor loses its router-bubble meaning: the link existed because the PAIR
  // of poses was within range, and both endpoints have moved since. The mode
  // picks the reconnection manoeuvre at exploration exhaustion:
  //   rendezvous — the legacy return-to-own-anchor barrier. Still sound on a
  //     mesh BY SYMMETRY: every robot returning to its own last-contact pose
  //     restores the pair distance the link had when it last worked.
  //   pursuit   — chase the missing peer's last declared goal (trail head) on
  //     a staleness-scaled budget; budget spent -> hold in place and beacon.
  //   hybrid    — pursue on the budget, then fall back to the deterministic
  //     meeting point (midpoint of the last-contact pose pair — both sides
  //     compute the same one from their own record) and wait there. Worst
  //     case degenerates to rendezvous' guarantee; best case wins early.
  // Code default is the legacy mode; the yaml/sim opt into hybrid.
  ReconnectMode reconnect_mode_ = ReconnectMode::RENDEZVOUS;
  // Hard ceiling on one chase (s). <= 0 disables pursuit (pursuit/hybrid then
  // behave like their fallback). Also the worst-case bound a WAITING teammate
  // can assume about its pursuer, so it wants a config'd cap, not a formula.
  double pursuit_budget_max_sec_    = 240.0;
  // Last-contact record age (s) beyond which the trail head is worthless and
  // pursuit is skipped outright; freshness scales the budget linearly down to
  // zero across this window. <= 0 = no staleness gate.
  //
  // Sized to real outages, not to goal freshness: dense-forest separations of
  // 186-861 s were measured, and at the old 180 s the chase declined every
  // single time it was asked — pursuit was dead code in the world it was
  // written for. What makes the wider window safe is pursuit_goal_stale_sec
  // below: past 180 s the chase stops trusting the peer's declared goal and
  // drives to its last CONTACT POSE, a target whose value does not decay with
  // age, and declines outright when the budget cannot cover that trail.
  double pursuit_staleness_max_sec_ = 900.0;
  // Record age beyond which the peer's declared GOAL is a dead hypothesis
  // (the peer has re-planned since) but its CONTACT POSE is still worth
  // driving to: two chasers that both complete a contact-pose trail end at
  // the swapped contact pair, which was mutually within link range — the same
  // geometric argument the anchor return rests on, and one that does not
  // decay with staleness. Past this age the chase drops the goal waypoint,
  // targets peer_pose alone, and budgets by distance (no freshness discount);
  // it declines instead when that distance-true budget would not cover the
  // trail (an uncoverable trail ends the chase at an arbitrary disconnected
  // point — worse than the mode's own fallback). <= 0 = never split.
  double pursuit_goal_stale_sec_ = 180.0;
  // Pure pursuit's fallback when the chase cannot start or is spent: keep
  // EXPLORING rather than park.
  //
  // Parking is a fixed point. Two robots that both hold cannot reconnect —
  // neither is moving, so the geometry that broke the link never changes —
  // and p7modes measured exactly that: 5 of 6 holds never reconnected, the
  // single recovery came from the PEER still driving, and one mutual hold
  // cost a mission whose maps were complete but split. A robot that goes
  // back to exploring is still covering ground, still earning the mission's
  // objective, and can regain the link by luck; a parked one can only be
  // found. Strictly dominated, so pursuit stops doing it.
  //
  // Bounded, because the terminal dispatch is the run's ending: reaching DONE
  // needs coverage saturation AND a complete team, so an unbounded
  // explore-fallback would re-saturate, re-dispatch and re-explore until the
  // duration cap, converting runs that would have finished into censored
  // ones. After this many fallbacks the robot reverts to holdForTeam and the
  // barrier (plus hold_escalate) guarantees an ending. The default matches
  // reconnect_midrun_max_attempts so a mid-run chase that declines can resume
  // exploring on every one of its attempts.
  bool pursuit_explore_fallback_ = true;
  int  pursuit_explore_max_      = 6;
  int  pursuit_explores_         = 0;
  // How long the team must have been INCOMPLETE before a manoeuvre may arm.
  //
  // Without this the arm test (peer missing, one read of the claim table) and
  // the release test (peer live, the same read one tick later) disagree inside
  // a single cycle, and the manoeuvre fires and dissolves before the robot
  // moves. Measured across the p3b/p4mild campaigns: 8 of 24 firings ended
  // within 5 s having travelled under a metre, two of them logging "ended
  // after 0.0 s sim", and 9 of 24 armed while the emulator had the pair
  // connected with 5-12 messages/s flowing. The cause is that the decision is
  // taken on a claim table which has not absorbed already-delivered intents:
  // the planner has just spent a long tick in PLAN (which is also what
  // suppresses its OWN beacon, see heartbeatTick), and drains the queue
  // immediately afterwards.
  //
  // This is the same guard the coverage criterion already carries
  // (done_min_consecutive_steps_): do not act on one sample of a noisy test.
  // The wait is on top of coord_claim_ttl_sec, so the peer must be silent for
  // ttl + this before a manoeuvre commits. <= 0 restores the old
  // arm-on-first-read behaviour.
  double reconnect_confirm_sec_ = 3.0;
  // Last time the team was observed complete, and whether that ever happened.
  // Maintained on the heartbeat timer so it advances in every state, including
  // the long PLAN ticks that cause the race. A run whose team was NEVER
  // complete (total blackout) must not be made to wait for a confirmation that
  // can never arrive, hence the flag.
  rclcpp::Time team_last_complete_time_;
  bool         team_seen_complete_ = false;
  // --- Mid-exploration reconnect trigger ---
  // 0 (the field default) keeps the manoeuvre strictly terminal: a robot only
  // considers reconnection once its own exploration is exhausted. A positive
  // value arms the same dispatch DURING exploration, whenever the team has
  // been continuously incomplete for this long — the point being that a
  // mid-run reconnection delivers the peer's queued map deltas while they can
  // still prune this robot's remaining frontiers. Must exceed the worst
  // heartbeat-suppression episode (measured ~180 s in the sim campaigns, see
  // heartbeatTick): below that, a healthy teammate stuck in a long PLAN loop
  // reads as missing and the trigger drives a manoeuvre at a robot that is in
  // range and fine. 0 restores the legacy terminal-only trigger.
  //
  // Default 240: comfortably above that 180 s suppression tail (measured max
  // over 2307 heartbeat episodes) and well below the outage tail, so it fires
  // on genuine separation and not on a busy teammate.
  double reconnect_midrun_silence_sec_  = 240.0;
  // Barrier give-up for MID-RUN manoeuvres only. A mid-run attempt that waits
  // rendezvous_max_wait_sec (600 in the sim harness) costs 2.5x its own
  // trigger threshold in lost exploration per failure; a short cap keeps the
  // attempt proportionate. Terminal manoeuvres keep rendezvous_max_wait_sec.
  double reconnect_midrun_max_wait_sec_ = 240.0;
  // Per-run cap on mid-run attempts. Every dispatch costs exploration time;
  // after this many failures the policy has had its chance and the robot
  // reverts to terminal-only behaviour (logged, so the analysis can see it).
  int    reconnect_midrun_max_attempts_ = 6;
  // --- Link-state gating for the mid-run trigger (see §30.11 and §30.24) ---
  // THE DEFECT THIS FIXES. Everything above measures silence with a RECORD-AGE
  // clock: team_last_complete_time_ advances only while peer intents arrive,
  // and the intent beacon is conditional twice over (it needs an active intent
  // and a state outside PLAN), so a healthy in-range teammate stuck in a long
  // PLAN loop is indistinguishable from one behind a hill. Measured against the
  // emulator's own link trace the record clock ran a median +49.1 s ahead of
  // real link-down, and 41-42 % of all mid-run fires bought nothing — 16-21 %
  // of them fired while the radio was UP. A chase is real distance debited from
  // exploration, so those are pure loss.
  //
  // WHAT IS AND IS NOT LEGITIMATE TO READ. The emulator publishes one row per
  // robot pair with [i, j, distance_m, trees_on_link, path_loss_db, snr_db,
  // ber, bandwidth_mbps, connected]. Only two of those may be touched here:
  // `connected` for pairs involving THIS robot, and `path_loss_db` solely as
  // the startup mask (a row the emulator has not computed yet reads
  // path_loss_db <= 0 with every physical field zeroed, and counting it as a
  // real disconnection manufactures a reconnection event at t=0 — same rule as
  // link_logger.py). `connected` is the stand-in for what a real mesh radio
  // genuinely exposes: a per-neighbour link indication kept alive by MAC-level
  // keepalives that are unconditional and fast, which is exactly what the
  // app-layer beacon is not. Reading distance_m, trees_on_link, path_loss_db
  // as a signal, snr_db, or any pair not involving self would be peer position
  // through the back door, and using link data to PREDICT reconnection or to
  // steer the chase would be oracle-driven. None of that is done below.
  // The topic is a global side channel and reaches a robot the emulator
  // considers disconnected; that discipline is by convention here, not
  // enforced by the transport.
  //
  // "" (the default) leaves the feature OFF and the trigger bit-identical to
  // the campaigns already banked. Nothing about the `off` arm can reach this:
  // the whole mid-run block requires rendezvous_enabled_, which is false there.
  std::string comms_link_states_topic_;
  std::string comms_link_robot_index_topic_;
  // Newest sample older than this and the gate stands down to the legacy clock
  // rather than acting on a stale belief. The emulator publishes at 5 Hz, so
  // 3 s is 15 missed samples: a real gap, not jitter.
  double comms_link_stale_sec_ = 3.0;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      link_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr link_index_sub_;
  int  link_self_idx_      = -1;     ///< own row index in the emulator's table
  bool link_index_usable_  = false;  ///< a 2-robot index naming us has arrived
  bool link_index_warned_  = false;  ///< N != 2 complaint is emitted once
  bool link_connected_     = false;  ///< newest own-pair connected bit
  bool link_have_sample_   = false;
  bool link_clock_anchored_ = false;
  rclcpp::Time link_last_sample_time_;  ///< receipt of the newest usable sample
  // Receipt time at which the link was last OBSERVED up. Down-duration is
  // measured from here rather than from a stored up->down edge, because
  // Float64MultiArray carries no header: if the executor stalls in a long PLAN
  // tick, a backlog delivered afterwards is all stamped at receipt, which would
  // compress the history and read the down-duration as ~0. That is also why the
  // subscription below is KeepLast(1) and not deep — with only the newest
  // sample the current state is always true. The residual error is honest and
  // one-sided: a stall that hides an up->down edge makes the down-duration read
  // too LONG by at most the stall, so the trigger can fire on a genuine outage
  // younger than the gate. It can never fire on a link that is up, which is the
  // 16-21 % this change exists to remove.
  rclcpp::Time link_up_last_seen_;
  // --- Info-gated mid-run trigger (voxels, not seconds) ---
  // 0 (the default) keeps the fixed silence clock above, bit-identical legacy
  // behaviour. Positive: the trigger time becomes "when the pair's estimated
  // unshared map crosses this many voxels", dead-reckoned from the last
  // contact: T = min_share / (my_rate + peer_rate), both rates frozen in the
  // LastContact snapshot so the two robots derive (approximately) the SAME
  // trigger time with no in-outage communication — asymmetric firing is the
  // lone-waiter failure p7modes measured (5 of 6 holds never reconnected).
  // T is clamped to [min_silence, max_silence]; rates that read 0 (peer
  // predates the beacon field, or no samples yet) push T to max_silence, i.e.
  // "nothing known to share -> wait for the backstop", never a divide-by-0.
  //
  // Calibrated against p14 (36 comms-on cells, dense world): a chance merge
  // delivers ~16k voxels median, a commanded one ~332k; by the 240 s clock
  // the pair is ALWAYS 300-850k apart — so as a deferral filter this gate is
  // inert in that world, and its live use is EARLIER, timeliness-driven
  // triggering. Sizing follows from the measured pair divergence (~2.3k
  // vox/s under this estimator): T ~ V*/2300, so 200k ~ 84 s, 300k ~ 126 s,
  // 500k ~ 210 s. Anything below ~140k lands under the 60 s floor and the
  // clamp — not V* — becomes the trigger, which is a fixed clock wearing the
  // gate's name; pick V* above that or the arm tests nothing.
  double reconnect_min_share_voxels_     = 0.0;
  // Clamp bounds for the derived trigger time. The floor guards the radio-
  // flicker regime the release-confirm machinery expects (a healthy teammate
  // in a long PLAN loop can read missing ~180 s; an info trigger BELOW that
  // knowingly accepts some chases at busy-not-lost teammates — that is the
  // eager variant's cost, so the floor is a parameter and not 200 hardcoded).
  // The ceiling is the insurance policy: the ONE effect p14 proved is that
  // mid-run reconnection caps the worst outage (515 -> 308 s, p=0.0095), and
  // no info gate is allowed to trade that away by deferring forever. It
  // therefore defaults to the LEGACY CLOCK, not above it: at 240 the gated
  // arm can only ever fire earlier than the control, so the proven cap is a
  // floor on its behaviour and the comparison carries no "gated runs waited
  // longer" confound. A ceiling above reconnect_midrun_silence_sec is a
  // deliberate choice to give that up.
  double reconnect_midrun_min_silence_sec_ = 60.0;
  double reconnect_midrun_max_silence_sec_ = 240.0;
  // True while the CURRENT manoeuvre was dispatched from exploration
  // exhaustion (the only kind that may end in DONE); false for mid-run
  // dispatches, which must always resume exploring instead. Default true so
  // every pre-existing path behaves exactly as before.
  bool   reconnect_terminal_ = true;
  int    midrun_attempts_    = 0;
  // Cooldown stamped when a mid-run manoeuvre ENDS (transitionTo, where
  // reconnect_active_ falls) — never at dispatch: missing_for stays satisfied
  // for the whole outage, so a dispatch-stamped cooldown expires DURING the
  // manoeuvre and the "resume" becomes a one-tick interlude in an endless
  // re-dispatch loop. Bool-guarded: rclcpp::Time default-constructs on the
  // system clock and subtracting it from a sim-time now() throws.
  rclcpp::Time midrun_last_end_;
  bool         midrun_end_armed_ = false;
  // --- Hold escalation (mutual-hold deadlock break) ---
  // When a TERMINAL barrier wait expires, drive once to the last-connected
  // anchor and wait hold_escalate_wait_sec more before giving up. Both robots
  // converging on their own last-contact poses restores the pair geometry the
  // link last worked at, which breaks the pure-hold fixed point (observed: 5
  // of 6 holds never reconnected; the parked pair can only be rescued by peer
  // motion). Sticky per-manoeuvre flag, NOT a position test: an unreachable
  // anchor must not re-escalate on every expiry forever. Inert under the
  // field default rendezvous_max_wait_sec=0 (wait forever, so no terminal
  // barrier ever expires): it can only act where an escape hatch is already
  // configured, and there it converts a give-up into one more attempt.
  bool   hold_escalate_          = true;
  double hold_escalate_wait_sec_ = 300.0;
  bool   hold_escalated_         = false;
  // (reconnect_rec_, the pair the current manoeuvre was armed from, is declared
  // with the other LastContact members below — the struct is only forward
  // declared here.)
  //
  // Arrival tolerance for a manoeuvre destination. NOT goal_xy_tolerance (0.4 m,
  // an exploration figure): a manoeuvre destination's whole value is
  // CONNECTIVITY, not position, and 0.4 m demands a precision the point does not
  // deserve. Two robots that each stop within this of the same meeting point are
  // at most 2x this apart — 8 m at the default, against a link that was still
  // carrying traffic at 54 m in the run that motivated the meeting point.
  //
  // This is also what makes an unreachable meeting point cheap. The midpoint is
  // synthetic and never checked against the map, and the comms emulator kills a
  // link by counting trunks within the Fresnel radius of the segment BETWEEN the
  // pair — so a foliage-killed link puts the trunks on that segment and the
  // midpoint at their centre. Demanding 0.4 m there means grinding against an
  // obstacle until the budget expires; 4 m means arriving beside it and waiting,
  // which is all the manoeuvre ever needed.
  double reconnect_arrive_tol_m_ = 4.0;
  // Ceiling on ONE manoeuvre drive leg. The distance-true budget in
  // startReturnTo is deliberately exempt from nav_max_timeout_sec (see there),
  // but "exempt" was unbounded: at nav_speed_estimate 0.15 x safety 3.0 = 20 s/m
  // a 40 m leg authorises 800 s, and the meeting point sits about half the pair
  // separation further out than the own-pose anchor it replaced. The invariant
  // this restores: no single leg may cost more than the barrier it is driving
  // toward, so a manoeuvre cannot outspend its own purpose. <= 0 = unbounded.
  double reconnect_nav_max_sec_ = 600.0;
  // --- Release confirmation (flicker guard) ---
  // A single live claim releases a manoeuvre and resets the silence clock,
  // crediting a "reconnection" on a range-edge flicker that drained no map
  // deltas. A positive value requires the release condition to hold
  // continuously this long.
  //
  // MUST EXCEED coord_claim_ttl_sec, and the old default (3 s, matched to
  // reconnect_confirm_sec) did not. Liveness is `receipt + ttl > now`, so ONE
  // packet at t holds the peer live until t+5 unaided — and a 3 s window is
  // satisfied at t+3 by that single packet. The guard let through exactly the
  // flicker it was written to stop. 6 s needs the claim genuinely refreshed at
  // least once (the beacon is 1 Hz), which a real reconnection does and a
  // range-edge blip does not. 0 = release on first read (legacy).
  double reconnect_release_confirm_sec_ = 6.0;
  rclcpp::Time release_ok_since_;
  bool         release_ok_armed_ = false;

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
  // Inset of the FRONTIER search band from the effective ROI band, in metres
  // off the bottom and off the top. Both 0 (the default) = search the whole ROI
  // band, i.e. today's behaviour. See the param load for what these are for.
  bool  frontier_band_set_ = false;
  float frontier_z_lo_off_ = 0.0f;
  float frontier_z_hi_off_ = 0.0f;

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
  // Vantage-ring rendezvous barrier. When true (the default) a robot standing on
  // its vantage does NOT start its dwell clock until every peer holding an
  // exploit claim on the same trunk is standing on the vantage IT claimed: the
  // team requirement is simultaneous capture of one trunk state, not three
  // sequential single-robot dwells. Inert with coordination off or with no peer
  // claim on this trunk (single-robot runs are bit-for-bit unaffected).
  bool   exploit_dwell_sync_enabled_ = true;
  // Barrier give-up (s), measured from EXPLOIT_DWELL entry and NOT from the
  // re-anchored dwell start. <= 0 (the default) waits until the peer actually
  // arrives; see the release-path argument in doExploitDwell for why that
  // terminates. Set a positive value only to force a deadline: a wall-clock
  // bound cannot distinguish a distant teammate from a wedged one, and 60 s
  // abandoned one that needed 111 s to cross the plot.
  double exploit_dwell_sync_max_wait_sec_ = 0.0;
  // Fine-TSDF region relay. When true, every ingested TreeTarget is registered
  // as a RefinementRegion on this robot's scovox_node the moment it arrives
  // (target release == exploitation-phase start) and unregistered when
  // finishActiveTarget closes it — so fine grids exist only while a tree is
  // under exploitation. Removal keeps already-fused fine voxels (the region
  // gate is integration-time policy, not storage); a scovox_node running with
  // fine_ratio_log2 = 0 ignores the messages, so this is safe to leave on.
  bool   publish_refinement_regions_ = true;
  // Radius forwarded to the scovox gate. TreeTarget.radius is the vantage
  // standoff radius (root flare — 1.8 m in the exploit schedules), far wider
  // than the breast-height trunk the fine slab measures, so the relay
  // substitutes this trunk-scale radius. <= 0 forwards TreeTarget.radius.
  double fine_region_radius_m_       = 0.5;

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
  // Dwell-sync barrier bookkeeping, re-initialised on every EXPLOIT_DWELL entry
  // (transitionTo). The wait clock CANNOT be state_enter_time_: the barrier
  // holds by re-anchoring that to now every tick, so a wait measured from it
  // would read ~0 forever and max_wait would never fire. The latch is what
  // stops a timed-out barrier from re-entering and re-anchoring the dwell clock
  // on the next tick, which would leave the dwell never completing.
  //
  // dwell_sync_started_ is the same guarantee for the SUCCESSFUL release: the
  // barrier is a start condition, not a per-tick precondition, so once the team
  // is staged the dwell runs to completion. Re-testing the probe every tick
  // makes a running dwell hostage to the team's schedule — a peer that finishes
  // its own capture and drives off to its next angle publishes staged=false
  // again, and the barrier then re-anchored the dwell clock of a robot that had
  // been motionless on its vantage for seconds (observed: ~3 s of accumulated
  // dwell discarded and re-dwelled from zero when the peer hopped v1 -> v0,
  // i.e. a full extra 8 s capture charged per peer departure).
  rclcpp::Time dwell_sync_wait_start_;
  bool         dwell_sync_timed_out_ = false;
  bool         dwell_sync_started_   = false;
  // The vantage THIS robot last completed a dwell on — pose (position + the
  // capture yaw), ring index, and which target it belongs to. This is the
  // pose the robot parks on when the rest of the ring is covered by the team
  // (doExploitPlan's hold branch): the requirement, verbatim from the field
  // operator watching the 2-robot sim, is that the robot which does NOT get
  // the last vantage "should have kept the first vantage pose". Validity is
  // the id match — target ids are unique for the life of the queue, so a
  // stale entry from a finished target can never match the next one and no
  // explicit invalidation is needed.
  CandidateViewpoint held_vantage_pose_;
  int      held_vantage_index_     = -1;
  uint32_t held_vantage_target_id_ = 0;
  bool     held_vantage_valid_     = false;
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

  // Cache key for the UNBOUNDED exploitation flood of cost_grid_ (see
  // doExploitPlan). That flood is O(grid) and EXPLOIT_PLAN re-enters at the
  // full tick rate whenever no vantage is selectable, so it is rebuilt only
  // when its inputs change. exploit_flood_map_ is an identity handle for the
  // latched map object — compared, never dereferenced. Invalidated by doPlan's
  // radius-bounded flood, which overwrites the same grid.
  bool exploit_flood_valid_ = false;
  const void* exploit_flood_map_ = nullptr;
  // Pointer identity alone is an ABA hazard: the allocator can hand a new
  // OccupancyGrid the address a freed one had, and the cache would then skip
  // the rebuild against a genuinely different map. The stamp breaks the tie.
  builtin_interfaces::msg::Time exploit_flood_stamp_;
  Eigen::Vector3f exploit_flood_pos_ = Eigen::Vector3f::Zero();
  size_t exploit_flood_reached_ = 0;

  // Coverage termination streak (done_criterion == "streak" only).
  int coverage_done_streak_ = 0;

  // Coverage termination latch (done_criterion == "latch"). One-way: set on the
  // first qualifying sample and never cleared, so a map that wobbles back above
  // threshold — or a merge that re-frontiers the ROI — cannot un-finish a robot
  // that has already reported finished. The two stamps are recorded because the
  // latch instant is the endpoint this criterion defines, and it is NOT the same
  // as the run's t_sim end (which is set by the SLOWER robot, plus teardown
  // grace); an analysis needs both to separate the two.
  bool   coverage_latched_        = false;
  double coverage_latch_t_sim_    = -1.0;
  double coverage_latch_unknown_  = -1.0;

  // Rendezvous anchor: the robot pose the last time it heard a teammate. That
  // pose sits inside the comms bubble, so it is the cheapest point to return to
  // for reconnection. Recorded on every peer intent (see the intent callback);
  // have_anchor_ stays false until the first peer is heard (single-robot runs
  // never rendezvous).
  Eigen::Vector3f last_connected_anchor_ = Eigen::Vector3f::Zero();
  bool  have_anchor_ = false;

  // Per-peer last-contact record (mesh reconnection). The mobile-radio
  // generalisation of the anchor: at every received intent, the PAIR of poses
  // that made the link — mine and the peer's advertised one — plus the peer's
  // declared goal, which is the freshest hypothesis of where it went (the
  // pursuit trail head). Keyed by robot_id; stamped with LOCAL receipt time
  // (the same clock discipline as claim expiry — peer stamps are untrusted).
  struct LastContact {
    Eigen::Vector3f self_pose = Eigen::Vector3f::Zero();
    Eigen::Vector3f peer_pose = Eigen::Vector3f::Zero();
    Eigen::Vector3f peer_goal = Eigen::Vector3f::Zero();
    rclcpp::Time    stamp;
    // Map-size half of the snapshot (info-gated mid-run trigger). Both sides
    // of the pair at the moment of contact: my cached count/rate and the
    // peer's beaconed ones. The peer's record of ME holds my last-BEACONED
    // values while mine holds my cached-at-receipt values — up to one beacon
    // period apart, which is noise against the ~metrics-period sampling both
    // are quantized to and the 15-60 s PLAN-loop dispatch jitter. rate 0.0
    // means "unknown" (pre-field peer or no samples yet), and the gate falls
    // back to the time-only trigger rather than divide by it.
    double self_voxels = 0.0, peer_voxels = 0.0;
    double self_rate   = 0.0, peer_rate   = 0.0;
  };
  std::map<std::string, LastContact> last_contact_;

  // Pursuit bookkeeping (valid while state_ == PURSUE). The trail is the
  // waypoint list startPursuit builds from the missing peer's record — goal
  // first (where it was heading), then its last heard pose (sweeps the leg it
  // was driving; the mesh lights up the moment any point of it is in range).
  // The budget clock runs from pursue_start_time_ across ALL waypoints;
  // proximity-hold time is refunded to it (a hold is not chase progress lost).
  std::vector<Eigen::Vector3f> pursue_waypoints_;
  size_t       pursue_wp_index_   = 0;
  double       pursue_budget_sec_ = 0.0;
  rclcpp::Time pursue_start_time_;
  std::string  pursue_peer_id_;
  // Snapshot of the record the chase was armed from. pursuitFallback derives
  // the hybrid meeting point from THIS pair, not a re-read of last_contact_:
  // a one-way packet heard mid-chase would move our midpoint away from the
  // one the peer computes from its own (un-refreshed) record of us.
  LastContact  pursue_rec_;

  // The pair the CURRENT manoeuvre was armed from — the same discipline as
  // pursue_rec_, applied to the whole manoeuvre rather than just the chase.
  // Every target a manoeuvre steers to is derived from THIS snapshot and never
  // from a re-read of last_contact_: the peer computes its midpoint from its
  // own record of the SAME contact event, so a one-way packet heard while we
  // drove or waited would move our midpoint off the one the peer is driving to
  // — which is exactly the non-convergence the meeting point exists to remove.
  // The hold escalation is the site that needed it: it re-queried live, and a
  // refresh between dispatch and barrier expiry made it escalate to a point
  // the peer had no reason to be at. One snapshot per manoeuvre, taken by
  // dispatchReconnect, cleared in transitionTo when reconnect_active_ falls.
  LastContact  reconnect_rec_;
  bool         have_reconnect_rec_ = false;

  // Human label of the current RETURN_NAV destination ("last-connected
  // anchor" or "meeting point"), set by startReturnTo for doReturnNav's logs.
  std::string  return_dest_label_;

  // Reconnect-manoeuvre clock, for the CSV's reconnect_elapsed_sec. Deliberately
  // NOT state_enter_time_: the manoeuvre spans state changes that must not
  // restart it — RETURN_NAV -> RETURN_SYNC on arrival, a PROXIMITY_HOLD taken
  // mid-drive, and in HYBRID the whole PURSUE -> meeting-point handoff. Armed
  // once by whichever of startReturnTo/startPursuit fires first (hence the
  // already-active guard in both) and cleared in transitionTo on leaving the
  // manoeuvre states, so the column measures one thing: wall seconds this robot
  // spent trying to re-establish contact rather than exploring.
  bool         reconnect_active_ = false;
  rclcpp::Time reconnect_start_time_;

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
  float pending_info_gain_std_        = 0.0f;
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

  // Map-size beacon state (info-gated mid-run trigger). History of
  // (t_sim_sec, total_observed_voxels) samples fed by fillCommonMetrics —
  // i.e. at the CSV sampling period, the same series every offline analysis
  // reads — from which the growth-rate slope is cached. stampMapInfo()
  // copies count+rate onto every outgoing intent; the on_intent lambda
  // snapshots both sides into LastContact. A vector trimmed in place, not a
  // deque: it holds a few dozen samples and is touched at the metrics
  // period, so contiguity beats pop_front.
  //
  // The window is 300 s, NOT the contact duration: connected windows have a
  // median of 24 s and 72% are under 90 s (measured, p14), so a window sized
  // to a contact contains nothing but that contact's merge inflow. Reaching
  // back across the PRECEDING OUTAGE is what supplies samples of the robot's
  // own unaided gathering. Validated against p14's measured pair divergence
  // (2,134 vox/s): the pair's summed rate under this estimator reads 1.12x
  // truth, against 1.40x for a 90 s two-point slope, at better coverage.
  static constexpr double kRateWindowSec = 300.0;
  static constexpr double kRateWinsorK   = 2.0;
  std::vector<std::pair<double, double>> map_size_hist_;
  double latest_map_voxels_ = 0.0;
  double map_growth_rate_   = 0.0;   // voxels / sim-second, >= 0
  void   noteMapSize(double t_sim_sec, double voxels);
  void   stampMapInfo();
  /// The ONLY way this node puts an intent on the wire. Stamps the map-size
  /// beacon onto the cached message first, so a publish can never carry a
  /// stale — or, at the sites that rebuild current_intent_msg_ via
  /// buildIntent(), a ZEROED — count and rate. A peer that snapshots a zeroed
  /// beacon as its last contact reads the pair as gathering nothing and defers
  /// to the ceiling, so this is not cosmetic. Callers must have checked
  /// intent_pub_.
  void   publishIntent();
  /// Effective mid-run trigger threshold (seconds of team-incomplete before
  /// dispatch). The fixed silence clock when the info gate is off or the
  /// snapshot is unusable; otherwise the dead-reckoned crossing time of
  /// reconnect_min_share_voxels, clamped to [min,max] silence. Also computes
  /// the live unshared-backlog estimate for the dispatch event's diagnostics.
  double midrunGateSec(double missing_for, double* est_unshared_out);
  // Gate diagnostics stashed at the trigger decision, consumed by the next
  // reconnect_dispatch event (same discipline as dispatch_peer_id_): -1 =
  // not applicable (terminal dispatch, or gate off); -2 = gate ON but no
  // usable contact snapshot, so the trigger fell back to the time-only
  // clock (otherwise that fallback logs byte-identically to a control
  // dispatch, which also has gate_sec = the fixed silence clock).
  double dispatch_gate_sec_     = -1.0;
  double dispatch_est_unshared_ = -1.0;
  // How long the RADIO had been down when the mid-run trigger fired, stashed on
  // the same discipline. -1 = the link gate was not in play (feature off, no
  // usable robot index, stale samples, or a terminal dispatch), in which case
  // the fire was decided on peer_record_age_sec exactly as before. Logged
  // alongside rather than instead of the record age: the whole point of §30.11
  // is that the two clocks disagree, so collapsing them into one column would
  // destroy the measurement that motivated the change.
  double dispatch_link_down_sec_ = -1.0;
  // True when the link gate has a fresh, decodable sample for our own pair and
  // may therefore override the record-age clock. Logs (throttled) when a topic
  // is configured but unusable, so a typo degrades loudly rather than silently
  // reverting to the behaviour this change exists to replace.
  bool linkGateReady(const rclcpp::Time& now);

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
  // One subscription per configured source topic. Single-element in the
  // default (shared-bus) wiring; one entry per peer when the stream is split
  // across an emulator's per-link relays.
  std::vector<rclcpp::Subscription<explo_planner_msgs::msg::RobotIntent>::SharedPtr>
      intent_subs_;
  // Shared tree-target topic. The time-based scheduler publishes here today; a
  // detector can publish the same message later with no planner change.
  rclcpp::Subscription<explo_planner_msgs::msg::TreeTarget>::SharedPtr target_sub_;
  // Fine-TSDF region relay to this robot's own scovox_node (topic built
  // absolute from robot_name_ — this node is not namespaced by the packaged
  // launch files). Null unless exploitation and publish_refinement_regions
  // are both enabled.
  rclcpp::Publisher<scovox_msgs::msg::RefinementRegion>::SharedPtr region_pub_;
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
  // REALISED sampling accounting for the periodic CSV sampler. The configured
  // period is not what it achieves: the sim-time timer above is gated by the
  // steady-clock deadline in metricsTick, so at some real-time factors whole
  // ticks are suppressed and at others none are — a realised period of 10 s
  // against a configured 5 s was measured in an earlier campaign, with every
  // log line still asserting 5 s. Counted here and reported in run_end so the
  // rate of an archived run is a recorded fact.
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
  // exploration_complete de-duplication. finishOrRendezvous is re-entered every
  // tick while the reconnect confirmation gate defers, and again if a robot
  // re-saturates after a manoeuvre delivered a merged map. Keying on step_
  // emits exactly one event per exhaustion EPISODE (step_ cannot advance during
  // a deferral) while still recording a genuine second exhaustion later.
  int exp_complete_step_  = -1;
  int exp_complete_count_ = 0;
  // Why the node reached DONE, kept for the run_end the destructor writes in
  // done_action=idle (where DONE is not the end of the file). Empty = DONE was
  // never reached, i.e. the run was cut short from outside.
  std::string done_reason_;
  // The peer + record age the current dispatch was decided from, stashed by
  // dispatchReconnect so the leaf that commits the action can report them
  // without re-querying a table that may have changed in between.
  std::string dispatch_peer_id_;
  double      dispatch_peer_age_sec_ = -1.0;
  // Why a richer manoeuvre was NOT taken, set by whichever guard declined
  // (startPursuit's staleness / trail-length gates, the spent explore-fallback
  // budget) and consumed by the next reconnect_dispatch event. Cleared at every
  // dispatch entry so a decline can never be attributed to a later manoeuvre.
  std::string reconnect_decline_reason_;
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
  // Experiment event log — one newline-delimited JSON file per robot per run.
  // See experiment_log.hpp for what it exists to fix; in short, the CSV plus
  // the ROS log could not answer "when did this robot reach coverage X" without
  // reconstructing sim time from wall-clock log stamps against a drifting RTF.
  //
  // The path DEFAULTS TO EMPTY, meaning "derive from output_csv": <csv without
  // its .csv suffix>.events.jsonl. That is deliberate and is the one place this
  // pair diverges from output_csv's flat default. Every harness already passes
  // a per-run, per-robot output_csv (run_explo_sim_rviz.sh: -p
  // output_csv:=$OUTDIR/planner_$r.csv), so deriving puts the event log beside
  // its own CSV automatically — whereas a fixed default like
  // /tmp/exploration_events.jsonl would have every robot of every arm truncate
  // the same file, which is exactly the silent data loss this class is about.
  // An explicit value is always used verbatim.
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
  // Descending unknown-fraction ladder for the coverage_milestone events, which
  // are how time-to-coverage is made comparable ACROSS ARMS: each arm stops on
  // its own criterion, so run duration measures a different thing per arm,
  // while "sim time this robot first drove unknown fraction below X" means the
  // same thing everywhere.
  //
  // The default is sized from the campaign data rather than guessed. Across 98
  // planner CSVs under /tmp/hmr_campaign the first measured unknown fraction is
  // 0.930-1.000 and the final one is 0.497-0.554 in 97 of them (the exception
  // is a 21-row run aborted at 0.817). So: 0.95 is degenerate — for the robots
  // whose first sample is already 0.93 it would fire at t0 by construction and
  // measure the initial map, not exploration — and nothing below 0.50 is ever
  // reached. 0.90 down to 0.50 is the informative band (in the p8trigger runs
  // 0.90 lands at ~35-43 s and 0.55 at ~820-1490 s, a spread that separates the
  // arms), and 0.45 is carried as a guard rung so a future arm that covers more
  // ground still records the crossing instead of silently having no data point.
  // Rungs that never fire simply never appear in the file.
  // Ladder rationale (and why the tail is NOT 0.55/0.50/0.45) in
  // config/shared_params.yaml — measured, those bottom rungs were dead columns
  // and the deepest one that fired was done_unknown_fraction itself.
  coverage_milestones_ = dp("coverage_milestones",
      std::vector<double>{0.90, 0.85, 0.80, 0.75, 0.70,
                          0.65, 0.62, 0.60, 0.58, 0.56});
  // clock_anchor cadence in SIM seconds. Each anchor is a (sim, wall) pair plus
  // the real-time factor since the previous one, which turns this file into the
  // conversion table for every other log in the run directory — they carry wall
  // stamps only, and the current practice of fitting one line through the whole
  // run is wrong by 10-54 s because the RTF drifts within a run (0.89 -> 0.81
  // measured). 10 s bounds the interpolation error to well under a second at
  // any plausible drift rate and costs one short line per anchor. <= 0
  // disables anchors (every other event still carries its own pair).
  experiment_log_anchor_period_sec_ =
      dp("experiment_log_anchor_period_sec", 10.0);
  // Wall-clock CSV sampling period. The end-of-step row is the only row a
  // pre-experiment run produced, and steps do not advance during a reconnect
  // manoeuvre — so a run that spent four minutes chasing a peer recorded that
  // interval as a single flat segment between two step rows, which is precisely
  // the interval the comms experiment is measuring. 0 restores the old
  // step-rows-only behaviour.
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
  // Minimum useful hop. Candidates nearer than this are rejected in doPlan
  // alongside the ones inside goal_xy_tolerance.
  //
  // Without it the planner deadlocks into a two-point oscillation, and the
  // mechanism is structural rather than a tuning accident. Utility is the
  // SSMI-style info_gain / (eps + path_cost). In a mostly-unknown map every
  // candidate's raycast terminates in unknown space, so info_gain is nearly
  // constant across the whole candidate set — measured spread was 6% while
  // path_cost varied ninefold — and argmax(U) degenerates to argmin(cost),
  // i.e. "drive to the closest frontier". The closest frontier is typically
  // under a metre away, and here is why that never resolves: a VLP-16 has a
  // +-15 deg vertical field of view, so at 0.6 m of standoff it sees a band
  // barely 0.3 m tall. Voxels beside the robot at any other height are
  // physically unobservable from that range. Driving there reveals nothing,
  // the frontier survives, and the pair of candidates either side of the
  // robot regenerate every step forever. Neither guard already in the loop
  // catches it: goal_xy_tolerance only skips candidates at arm's length, and
  // the failed-goal blacklist never fires because the robot REACHES each goal.
  //
  // So this is not a heuristic to break ties; it encodes that a goal closer
  // than the sensor's useful standoff cannot reduce uncertainty where it
  // stands. Scale it to the sensor, not the robot: a few metres for a VLP-16
  // with fov_max_range 10 m.
  //
  // Default 0.0 keeps the shipped behaviour bit-identical — every field
  // config that predates this parameter selects exactly the goals it did
  // before.
  cand_min_goal_dist_ = dp("candidate_min_goal_dist_m", 0.0);
  if (cand_min_goal_dist_ > 0.0 && cand_min_goal_dist_ <= goal_xy_tol_) {
    RCLCPP_WARN(get_logger(),
        "candidate_min_goal_dist_m=%.2f is not above goal_xy_tolerance=%.2f, "
        "so it rejects nothing the arrival gate did not already reject and "
        "the near-frontier oscillation it exists to prevent is still live.",
        cand_min_goal_dist_, goal_xy_tol_);
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
  // See the member: a dead TF chain keeps "succeeding" with the same stamp,
  // so freshness is checked explicitly in updatePoseFromTF. Sized for the
  // slowest healthy publisher in the map->base chain (field SLAM's map->odom
  // at well under 1 Hz), not for the 10 Hz tick.
  pose_max_age_sec_ = dp("pose_max_age_sec", 5.0);

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
  // Visited-goal suppression. A goal the robot REACHED is parked for
  // visited_goal_ttl_sec, and EXPLORE candidates within visited_goal_radius_m
  // of a live entry are skipped. 0 (default) = off, shipped behaviour.
  //
  // Needed because frontier exploration has no fixed point in a forest. A
  // frontier is a free voxel beside an unknown one, and every trunk casts a
  // permanently unknown shadow, so frontier clusters regenerate no matter how
  // thoroughly an area is observed. Utility is info/(eps+cost) and info is
  // near-constant while the map is mostly unknown, so selection collapses to
  // argmin(cost) -- and two neighbouring clusters then trade places as
  // "nearest" forever. Measured on flatforest: after narrowing the frontier
  // band the robot advanced in bursts but still alternated between two goals
  // 4.7 m apart for six consecutive steps with no net movement.
  //
  // The failed-goal blacklist cannot cover this: it only fires when a goal is
  // NOT reached, and here every goal is reached, on time, successfully.
  //
  // Size the radius near the frontier cluster radius, so suppressing a visited
  // goal suppresses the cluster that produced it rather than a point inside
  // it. The TTL must outlast a there-and-back trip or the entry expires before
  // the oscillation it prevents can recur; it is a TTL rather than permanent so
  // a genuinely re-frontiered area can be revisited late in a run.
  visited_goal_radius_m_ = dp("visited_goal_radius_m", 0.0);
  visited_goal_ttl_sec_  = dp("visited_goal_ttl_sec", 120.0);

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
  // WHICH RULE DECIDES FINISHED. The default is "latch" and the paragraph above
  // describes "streak", which is now opt-in. The change is deliberate and the
  // reason is measurable: the streak test runs only at the top of doPlan, so a
  // robot inside a reconnect manoeuvre cannot declare itself finished however
  // saturated its map is. Measured on the tr1 campaign, that blind spot plus
  // re-earning the streak from zero after the manoeuvre put ~60 s of pure
  // detection latency on the hybrid arm and 0 s on the off arm — an endpoint
  // that moves with the treatment is not an endpoint, it is part of the
  // treatment. "latch" measures the same quantity in both arms:
  //   * the robot's OWN fused-map ROI unknown fraction (nothing team-wide),
  //   * tested on every metrics tick in EVERY state, manoeuvres included,
  //   * on first touch — no confirmation streak,
  //   * with no rendezvous gate: being in comms is not required to be finished.
  // The run then ends when BOTH robots have latched independently, which the
  // harness already implements by waiting for every planner's state to read
  // DONE. Set "streak" to reproduce a pre-2026-08-24 campaign.
  done_criterion_ = dp("done_criterion", std::string("latch"));
  if (done_criterion_ != "latch" && done_criterion_ != "streak") {
    RCLCPP_WARN(get_logger(),
        "Unknown done_criterion '%s' — falling back to 'latch'.",
        done_criterion_.c_str());
    done_criterion_ = "latch";
  }
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
  // Narrow the FRONTIER search band relative to the ROI band, from the bottom
  // and from the top. Both 0 = search the whole ROI band (shipped behaviour).
  //
  // The ROI band is sized for terrain and canopy — the yaml ships a 9.5 m slab,
  // -5.5 to +4.0 — and searching a slab that tall for frontiers produces a
  // candidate set dominated by voxels no robot can ever observe. A frontier is
  // a free voxel with an unknown neighbour, and a lidar's free space is a wedge
  // bounded by its vertical FOV, so the ENTIRE upper and lower surface of that
  // wedge qualifies, at every range, forever: a VLP-16 at +-15 deg simply has
  // no ray that reaches 3 m up at 4 m out. Those frontiers cannot be consumed
  // by driving to them, which is what makes them poison rather than noise —
  // they regenerate beside the robot every tick, they are always the nearest
  // ones, and (utility being info/cost with near-constant info) they are
  // therefore always chosen. Measured on flatforest: frontier_voxels stayed at
  // ~94% of all observed voxels and GREW monotonically as the map grew, while
  // both robots ping-ponged between two adjacent goals indefinitely.
  //
  // Set these so the band covers only the heights the sensor sweeps as it
  // drives — roughly the navigable slice. Then a frontier means "ground I have
  // not been past", which driving there does clear, and the candidate set
  // drains as the ROI is covered, which is also what makes coverage
  // termination reachable at all.
  //
  // Only ever NARROWS: the result is intersected with the ROI band, because a
  // frontier outside the ingested band would reference voxels map_cache_ never
  // loaded.
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

  // Distance discount for the utility denominator. 1.0 = shipped behaviour.
  // Clamped rather than trusted: a negative exponent inverts the denominator
  // into a REWARD for distance (the further the better, without bound), which
  // is not a weaker preference but a different and unbounded objective, and a
  // typo in a sweep script should not be able to express it. Above 2.0 the
  // planner is more distance-averse than nearest-frontier, which the candidate
  // filters already enforce more cheaply.
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
  // Split intent stream (see member comments). Empty -> today's shared bus.
  // A non-absolute value is resolved under /<robot_name>/, matching this
  // file's convention for every other cross-node topic (see
  // refinement_region_topic below): the packaged launch files pass robot_name
  // as a parameter but do NOT namespace this node, so a relative topic would
  // otherwise land in the global scope and silently never match the
  // emulator's per-robot relays — which reads as "peer missing forever",
  // indistinguishable from the outage the run is trying to measure.
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
                  coord_intent_sub_topics_.end(), std::string("")),
      coord_intent_sub_topics_.end());
  if (coord_intent_sub_topics_.empty())
    coord_intent_sub_topics_ = {coord_intent_topic_};
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
  // The barrier counts peers by their claim beacons, and only a DONE-idle
  // robot keeps beaconing after it finishes (finishOrRendezvous publishes the
  // presence intent, the heartbeat refreshes it). A done_action=shutdown
  // robot exits the process instead: the first finisher becomes permanently
  // invisible and a teammate finishing later runs the whole reconnect
  // manoeuvre against a robot that no longer exists.
  if (rendezvous_enabled_ && done_action_ != "idle") {
    RCLCPP_WARN(get_logger(),
        "Rendezvous barrier with done_action='%s': the first robot to finish "
        "exits and stops beaconing, so a teammate finishing later can never "
        "count it (it waits the full rendezvous_max_wait_sec, or forever). "
        "Use done_action=idle for reconnect runs.", done_action_.c_str());
  }

  // Mesh reconnection mode. Default "rendezvous" = the legacy return-to-anchor
  // barrier, bit-for-bit; "pursuit"/"hybrid" are the robot-carried-radio
  // manoeuvres (see the param comments above). Gated by the same
  // rendezvous_enabled_ preconditions — the mode only picks WHICH manoeuvre
  // runs once shouldRendezvous() says one should.
  {
    const std::string mode_str =
        dp("reconnect_mode", std::string("rendezvous"));
    bool mode_known = false;
    reconnect_mode_ = reconnectModeFromString(mode_str, &mode_known);
    if (!mode_known) {
      RCLCPP_WARN(get_logger(),
          "Unknown reconnect_mode '%s' — falling back to 'rendezvous'.",
          mode_str.c_str());
    }
  }
  pursuit_budget_max_sec_    = dp("pursuit_budget_max_sec", 240.0);
  pursuit_staleness_max_sec_ = dp("pursuit_staleness_max_sec", 900.0);
  pursuit_goal_stale_sec_    = dp("pursuit_goal_stale_sec", 180.0);
  pursuit_explore_fallback_  = dp("pursuit_explore_fallback", true);
  pursuit_explore_max_       = dp("pursuit_explore_max", 6);
  reconnect_confirm_sec_     = dp("reconnect_confirm_sec", 3.0);
  if (reconnect_confirm_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
        "reconnect_confirm_sec <= 0: manoeuvres arm on a single read of the "
        "claim table. Expect firings that dissolve before the robot moves, "
        "and treat any reconnection timing from this run as unusable.");
  }
  // Mid-run trigger family (see the member comments). ON by default: the
  // terminal-only trigger it replaces could not reconnect a team before its
  // exploration was already over, which is the whole value of reconnecting.
  // Set reconnect_midrun_silence_sec to 0 to restore the legacy behaviour.
  reconnect_midrun_silence_sec_  = dp("reconnect_midrun_silence_sec", 240.0);
  reconnect_midrun_max_wait_sec_ = dp("reconnect_midrun_max_wait_sec", 240.0);
  reconnect_midrun_max_attempts_ = dp("reconnect_midrun_max_attempts", 6);
  // Link-state gate (see the member comments). "" = off, bit-identical legacy
  // record-age clock, no subscription created at all.
  comms_link_states_topic_ =
      dp("comms_link_states_topic", std::string(""));
  comms_link_robot_index_topic_ =
      dp("comms_link_robot_index_topic", std::string(""));
  comms_link_stale_sec_ = dp("comms_link_stale_sec", 3.0);
  if (!comms_link_states_topic_.empty() &&
      comms_link_robot_index_topic_.empty()) {
    // The index is what turns a pair row into "my pair". Without it every
    // sample is undecodable and the gate would stand down on every tick while
    // looking configured — the failure mode this project keeps rediscovering.
    RCLCPP_WARN(get_logger(),
        "comms_link_states_topic is set to '%s' but "
        "comms_link_robot_index_topic is empty: link rows cannot be matched to "
        "this robot, so the mid-run trigger will keep using the record-age "
        "clock. Set both or neither.",
        comms_link_states_topic_.c_str());
  }
  // Info gate (see the member comments). 0 = off, bit-identical legacy clock.
  reconnect_min_share_voxels_      = dp("reconnect_min_share_voxels", 0.0);
  reconnect_midrun_min_silence_sec_ =
      dp("reconnect_midrun_min_silence_sec", 60.0);
  reconnect_midrun_max_silence_sec_ =
      dp("reconnect_midrun_max_silence_sec", 240.0);
  if (reconnect_min_share_voxels_ > 0.0 &&
      reconnect_midrun_min_silence_sec_ >
          reconnect_midrun_max_silence_sec_) {
    // min(max, max(min, t)) with min > max collapses to MAX, whatever t is.
    RCLCPP_WARN(get_logger(),
        "reconnect_midrun_min_silence_sec=%.0f exceeds max=%.0f: the clamp "
        "degenerates to a fixed %.0f s clock (the MAX wins) and the info gate "
        "is inert.",
        reconnect_midrun_min_silence_sec_, reconnect_midrun_max_silence_sec_,
        reconnect_midrun_max_silence_sec_);
  }
  if (reconnect_min_share_voxels_ > 0.0 &&
      reconnect_midrun_max_silence_sec_ > reconnect_midrun_silence_sec_ &&
      reconnect_midrun_silence_sec_ > 0.0) {
    RCLCPP_WARN(get_logger(),
        "reconnect_midrun_max_silence_sec=%.0f exceeds the legacy clock "
        "%.0f s: a gated run can now wait LONGER than the ungated control, "
        "so the proven longest-outage cap is no longer guaranteed and an A/B "
        "against that control gains a confound.",
        reconnect_midrun_max_silence_sec_, reconnect_midrun_silence_sec_);
  }
  if (reconnect_min_share_voxels_ > 0.0 &&
      reconnect_midrun_silence_sec_ <= 0.0) {
    // The info gate lives INSIDE the mid-run trigger: silence_sec = 0 turns
    // the whole trigger off, so a configured gate silently never runs — the
    // run then looks exactly like a control (no gated dispatches, ever)
    // while its manifest says it was gated.
    RCLCPP_WARN(get_logger(),
        "reconnect_min_share_voxels=%.0f is set but "
        "reconnect_midrun_silence_sec=0 disables the whole mid-run trigger — "
        "the info gate can never fire in this configuration.",
        reconnect_min_share_voxels_);
  }
  reconnect_release_confirm_sec_ = dp("reconnect_release_confirm_sec", 6.0);
  hold_escalate_                 = dp("hold_escalate", true);
  hold_escalate_wait_sec_        = dp("hold_escalate_wait_sec", 300.0);
  reconnect_arrive_tol_m_        = dp("reconnect_arrive_tol_m", 4.0);
  reconnect_nav_max_sec_         = dp("reconnect_nav_max_sec", 600.0);
  // A release window at or below the claim TTL is not a flicker guard: one
  // packet keeps the peer live for the whole TTL, so any window inside it is
  // satisfied without the claim ever being refreshed. Warn rather than clamp —
  // 0 is a legitimate "legacy behaviour" setting for an A/B, and silently
  // moving a configured value would make the manifest a lie.
  if (reconnect_release_confirm_sec_ > 0.0 &&
      reconnect_release_confirm_sec_ <= coord_claim_ttl_sec_) {
    RCLCPP_WARN(get_logger(),
        "reconnect_release_confirm_sec=%.1f is <= coord_claim_ttl_sec=%.1f: a "
        "SINGLE packet holds the peer live for the whole TTL, so this window "
        "is satisfied without the claim being refreshed and the flicker guard "
        "is inert. Use a value above %.1f.",
        reconnect_release_confirm_sec_, coord_claim_ttl_sec_,
        coord_claim_ttl_sec_);
  }
  if (reconnect_midrun_silence_sec_ > 0.0 &&
      reconnect_midrun_silence_sec_ < 200.0) {
    RCLCPP_WARN(get_logger(),
        "reconnect_midrun_silence_sec=%.0f is below the measured "
        "heartbeat-suppression tail (~180 s): a healthy teammate stuck in a "
        "long PLAN loop can read as missing that long, and the trigger would "
        "drive a manoeuvre at a robot that is in range and fine.",
        reconnect_midrun_silence_sec_);
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
  pcfg.escape_grace_sec =
      static_cast<float>(dp("proximity_escape_grace_sec", 30.0));
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
  // Vantage-ring rendezvous barrier. ON by default: the whole point of the ring
  // is overlapping simultaneous views of one trunk state, and without the
  // barrier the first robot to arrive burns its dwell alone while the peer is
  // still driving — on a 3/3 quota the early robot can close the target solo
  // and the peer arrives to dwell an angle nobody needs. Distinct from the
  // `rendezvous_*` params above, which are the comms-reconnection barrier at
  // the anchor pose; these two are the per-vantage capture barrier.
  exploit_dwell_sync_enabled_ = dp("exploit_dwell_sync_enabled", true);
  exploit_dwell_sync_max_wait_sec_ =
      dp("exploit_dwell_sync_max_wait_sec", 0.0);
  std::string targets_topic = dp("targets_topic",
                                 std::string("/exploration/targets"));
  // Fine-TSDF region relay (see the member comments). The default topic is
  // built ABSOLUTE from robot_name_, like every other cross-node topic here
  // (goal_pose, dscovox_node/*): the packaged launch files pass robot_name as
  // a parameter but do NOT namespace this node, so a relative default would
  // resolve to the global scope and silently never match the namespaced
  // scovox_node. An explicit param value is used verbatim.
  publish_refinement_regions_ = dp("publish_refinement_regions", true);
  fine_region_radius_m_       = dp("fine_region_radius_m", 0.5);
  std::string refinement_region_topic = dp(
      "refinement_region_topic", std::string(""));
  if (refinement_region_topic.empty()) {
    refinement_region_topic =
        "/" + robot_name_ + "/scovox_node/refinement_region";
  }

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
          "structured reconnect events.", experiment_log_path_.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
          "Experiment event log: %s (%zu coverage milestone(s); sim-clock "
          "stamped, flushed per event).",
          experiment_log_path_.c_str(), coverage_milestones_.size());
    }
    // The independent variables of the experiment, recorded IN the data file
    // rather than only in the harness manifest — a file that cannot say which
    // arm produced it has to be trusted to a directory name.
    // PROVENANCE. No derived artefact in this project currently carries any:
    // the CSV schema changed silently between campaigns and nothing in the data
    // recorded which version produced it. schema_version (written by the logger
    // itself) covers the event format; these identify the binary.
    //
    // EXPLO_PLANNER_GIT_REV is injected by CMake at configure time — so it
    // identifies the checkout the build was CONFIGURED from, and a source edit
    // without a reconfigure leaves it stale. It is a strong hint, not a
    // guarantee; the build stamp below is what disambiguates two binaries built
    // from the same revision.
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
    exp_log_->addParamStr("reconnect_mode", reconnectModeName(reconnect_mode_));
    // THE arm this run belongs to, and the field an analysis must group by.
    //
    // reconnect_mode alone is NOT the arm. The control arm is "no reconnection
    // at all", which is not a reconnect_mode value — it is expressed as
    // rendezvous_enabled=false, and the harness has to pass SOME mode alongside
    // it (it passes "hybrid"). So a control run is stamped reconnect_mode
    // "hybrid", and anything grouping on that column pools the control into the
    // hybrid cell: the hybrid mean becomes the average of treatment and
    // control, and the control arm ceases to exist. The information was always
    // in the file, split across two fields; nothing was reading both. This
    // collapses them once, here, where the planner knows the answer.
    exp_log_->addParamStr("arm", rendezvous_enabled_
                                     ? reconnectModeName(reconnect_mode_)
                                     : "off");
    exp_log_->addParamBool("use_sim_time", this->get_parameter("use_sim_time")
                                               .as_bool());
    exp_log_->addParamStr("output_csv", output_csv_);
    exp_log_->addParamNum("max_steps", max_steps_);
    exp_log_->addParamNum("metrics_period_sec", metrics_period_sec_);
    exp_log_->addParamNum("metrics_max_duty", metrics_max_duty_);
    exp_log_->addParamNum("experiment_log_anchor_period_sec",
                          experiment_log_anchor_period_sec_);
    exp_log_->addParamBool("coordination_enabled", coord_enabled_);
    exp_log_->addParamNum("coord_claim_ttl_sec", coord_claim_ttl_sec_);
    exp_log_->addParamNum("coord_heartbeat_hz", coord_heartbeat_hz_);
    exp_log_->addParamBool("rendezvous_enabled", rendezvous_enabled_);
    exp_log_->addParamNum("rendezvous_expected_peers",
                          rendezvous_expected_peers_);
    exp_log_->addParamNum("rendezvous_max_wait_sec", rendezvous_max_wait_sec_);
    exp_log_->addParamNum("reconnect_confirm_sec", reconnect_confirm_sec_);
    exp_log_->addParamNum("reconnect_release_confirm_sec",
                          reconnect_release_confirm_sec_);
    exp_log_->addParamNum("reconnect_midrun_silence_sec",
                          reconnect_midrun_silence_sec_);
    exp_log_->addParamNum("reconnect_midrun_max_wait_sec",
                          reconnect_midrun_max_wait_sec_);
    exp_log_->addParamNum("reconnect_midrun_max_attempts",
                          reconnect_midrun_max_attempts_);
    exp_log_->addParamNum("reconnect_min_share_voxels",
                          reconnect_min_share_voxels_);
    exp_log_->addParamNum("reconnect_midrun_min_silence_sec",
                          reconnect_midrun_min_silence_sec_);
    exp_log_->addParamNum("reconnect_midrun_max_silence_sec",
                          reconnect_midrun_max_silence_sec_);
    exp_log_->addParamNum("pursuit_budget_max_sec", pursuit_budget_max_sec_);
    exp_log_->addParamNum("pursuit_staleness_max_sec",
                          pursuit_staleness_max_sec_);
    exp_log_->addParamNum("pursuit_goal_stale_sec", pursuit_goal_stale_sec_);
    exp_log_->addParamBool("pursuit_explore_fallback",
                           pursuit_explore_fallback_);
    exp_log_->addParamNum("pursuit_explore_max", pursuit_explore_max_);
    exp_log_->addParamBool("hold_escalate", hold_escalate_);
    exp_log_->addParamNum("hold_escalate_wait_sec", hold_escalate_wait_sec_);
    exp_log_->addParamNum("reconnect_arrive_tol_m", reconnect_arrive_tol_m_);
    exp_log_->addParamNum("reconnect_nav_max_sec", reconnect_nav_max_sec_);
    exp_log_->addParamNum("done_unknown_fraction", done_unknown_fraction_);
    exp_log_->addParamNum("done_min_consecutive_steps",
                          done_min_consecutive_steps_);
    exp_log_->addParamStr("done_coverage_source", done_coverage_source_);
    // Which rule ended the run. Recorded because it is not recoverable from any
    // other field, and a campaign that mixes the two criteria is comparing two
    // different endpoints under one column name.
    exp_log_->addParamStr("done_criterion", done_criterion_);
    exp_log_->addParamStr("done_action", done_action_);
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
          // Frame check, matching the one the ScovoxMap path already does.
          // planMapCellAt/isCellFree/unknownFractionInRoi index this grid with
          // raw world XY and no TF at all, so if the publisher stamps it in a
          // frame that is not numerically the planner's map frame, every
          // lookup silently reads the wrong cell — candidates rejected as
          // occupied on the strength of geometry from somewhere else. The
          // usual sim publisher is scovox_node, which stamps its integration
          // frame (<robot>/odom); that is only safe because map->odom is
          // published as identity. Nothing enforces that, so say so out loud
          // when it stops being true.
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
          // Rendezvous: record where we were the last time we heard a
          // teammate. That pose is inside the comms bubble, so it is the
          // cheapest point to return to for reconnection. onIntent already
          // drops our own echo, but we gate on robot_id here too since we read
          // our live pose. have_pose_ guards the very first ticks before TF.
          if (rendezvous_enabled_ && have_pose_ &&
              msg->robot_id != robot_name_) {
            last_connected_anchor_ = latest_pos_;
            have_anchor_ = true;
            // Mesh reconnection: with robot-carried radios BOTH endpoints of
            // the lost link have moved, so keep the whole last-contact pair
            // per peer — my pose (the anchor generalised), its advertised
            // pose, and its declared goal, the pursuit trail head. Stamped
            // with local receipt time, same clock discipline as claim expiry.
            auto& rec = last_contact_[msg->robot_id];
            rec.self_pose = latest_pos_;
            rec.peer_pose =
                Eigen::Vector3f(static_cast<float>(msg->robot_pos.x),
                                static_cast<float>(msg->robot_pos.y),
                                static_cast<float>(msg->robot_pos.z));
            rec.peer_goal =
                Eigen::Vector3f(static_cast<float>(msg->goal_pos.x),
                                static_cast<float>(msg->goal_pos.y),
                                static_cast<float>(msg->goal_pos.z));
            rec.stamp = this->now();
            // Map-size half of the snapshot: the peer's beaconed count/rate
            // and my cached ones, frozen together so both robots derive the
            // same info-gate trigger time from THIS contact (see LastContact).
            rec.peer_voxels = static_cast<double>(msg->observed_voxels);
            rec.peer_rate   = static_cast<double>(msg->map_growth_rate);
            rec.self_voxels = latest_map_voxels_;
            rec.self_rate   = map_growth_rate_;
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
    std::string subs;
    for (const auto& t : coord_intent_sub_topics_) subs += (subs.empty() ? "" : ", ") + t;
    RCLCPP_INFO(get_logger(),
        "Intents: pub '%s' <- KeepLast(8).reliable() -> sub [%s]%s",
        coord_intent_pub_topic_.c_str(), subs.c_str(),
        (coord_intent_sub_topics_.size() == 1 &&
         coord_intent_sub_topics_[0] == coord_intent_pub_topic_)
            ? " (shared bus: no external process can gate this stream)" : "");
  }

  // --- Link-state gate wiring (§30.11). Only two fields of the emulator's
  // table are ever read here — see the member comments for why the rest would
  // be an oracle. Nothing is created when the topic is unset, so a legacy run
  // does not even subscribe.
  if (!comms_link_states_topic_.empty()) {
    if (!comms_link_robot_index_topic_.empty()) {
      link_index_sub_ = create_subscription<std_msgs::msg::String>(
          comms_link_robot_index_topic_,
          // Must match the emulator's transient_local publisher or the latched
          // index never arrives and every link sample stays undecodable.
          rclcpp::QoS(1).transient_local(),
          [this](const std_msgs::msg::String::SharedPtr msg) {
            // Minimal scan of {"robots":["a","b"],...}. A JSON dependency for
            // one flat array of strings is not worth the build cost, and the
            // emitter is a fixed ostringstream in the emulator, not arbitrary
            // JSON: PublishRobotIndex writes exactly this shape.
            const std::string& s = msg->data;
            const auto key = s.find("\"robots\"");
            if (key == std::string::npos) return;
            const auto open  = s.find('[', key);
            if (open == std::string::npos) return;
            const auto close = s.find(']', open);
            if (close == std::string::npos) return;
            std::vector<std::string> names;
            size_t p = open;
            while (true) {
              const auto q1 = s.find('"', p);
              if (q1 == std::string::npos || q1 > close) break;
              const auto q2 = s.find('"', q1 + 1);
              if (q2 == std::string::npos || q2 > close) break;
              names.push_back(s.substr(q1 + 1, q2 - q1 - 1));
              p = q2 + 1;
            }
            // Deliberately restricted to two robots. With three or more,
            // "connected to at least one peer" and "the team is complete" stop
            // being the same statement, and the mid-run trigger is written
            // against the latter. Guessing a meaning here would be a silent
            // wrong answer in the heterogeneous campaign that is already
            // planned, so refuse loudly and keep the legacy clock instead.
            if (names.size() != 2) {
              if (!link_index_warned_) {
                link_index_warned_ = true;
                RCLCPP_WARN(get_logger(),
                    "Link gate: robot index lists %zu robots; the gate is only "
                    "defined for a pair, so the mid-run trigger keeps the "
                    "record-age clock for this run.",
                    names.size());
              }
              link_index_usable_ = false;
              return;
            }
            const auto it = std::find(names.begin(), names.end(), robot_name_);
            if (it == names.end()) {
              if (!link_index_warned_) {
                link_index_warned_ = true;
                RCLCPP_WARN(get_logger(),
                    "Link gate: robot index does not name '%s' (it lists '%s', "
                    "'%s'); keeping the record-age clock.",
                    robot_name_.c_str(), names[0].c_str(), names[1].c_str());
              }
              link_index_usable_ = false;
              return;
            }
            const int idx = static_cast<int>(it - names.begin());
            if (!link_index_usable_ || link_self_idx_ != idx) {
              RCLCPP_INFO(get_logger(),
                  "Link gate: armed as robot index %d of [%s, %s]; the mid-run "
                  "trigger now fires on radio link-down, not record age.",
                  idx, names[0].c_str(), names[1].c_str());
            }
            link_self_idx_     = idx;
            link_index_usable_ = true;
          });
    }
    link_states_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        comms_link_states_topic_,
        // KeepLast(1) ON PURPOSE — see link_up_last_seen_. These messages carry
        // no header, so a backlog delivered after an executor stall would be
        // stamped at receipt and read as a link that just came back.
        rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (!link_index_usable_) return;
          constexpr size_t kCols = 9;   // emulator's kLinkStateCols
          const auto& d = msg->data;
          bool found = false, connected = false;
          for (size_t k = 0; k + kCols <= d.size(); k += kCols) {
            const int i = static_cast<int>(d[k]);
            const int j = static_cast<int>(d[k + 1]);
            if (i != link_self_idx_ && j != link_self_idx_) continue;
            // Startup mask, same discriminator as link_logger.py: the path-loss
            // floor is ~49 dB at one metre and only grows, so <= 0 is a row the
            // emulator never computed. Treating it as a disconnection would
            // manufacture a reconnection the instant poses arrive.
            if (d[k + 4] <= 0.0) continue;
            found = true;
            if (d[k + 8] != 0.0) connected = true;   // `connected` column
          }
          if (!found) return;
          const auto now = this->now();
          link_have_sample_      = true;
          link_last_sample_time_ = now;
          link_connected_        = connected;
          // Anchor on the first usable sample whatever its state: with no
          // observed "up" to measure from, the earliest defensible claim is
          // "down since we started watching", which under-states the outage and
          // therefore delays rather than invents a fire.
          if (connected || !link_clock_anchored_) {
            link_up_last_seen_    = now;
            link_clock_anchored_  = true;
          }
        });
    RCLCPP_INFO(get_logger(),
        "Link gate: subscribed to '%s' (KeepLast(1)) with index from '%s', "
        "stale after %.1fs. Mid-run reconnect fires on link-down duration; "
        "peer_record_age_sec is still logged unchanged.",
        comms_link_states_topic_.c_str(),
        comms_link_robot_index_topic_.c_str(), comms_link_stale_sec_);
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
    if (publish_refinement_regions_) {
      // Latched (transient_local) both ends, mirroring the scovox_node
      // subscription: an ADD fires exactly once per target id (ingest dedup —
      // no retry path exists), so it must survive a DDS discovery race at
      // startup-with-backlog and a restarted scovox subscription. Replay is
      // safe: adds are keyed-replace and removes are idempotent, so a late
      // joiner converges to the correct region set.
      region_pub_ = create_publisher<scovox_msgs::msg::RefinementRegion>(
          refinement_region_topic,
          rclcpp::QoS(rclcpp::KeepLast(50)).reliable().transient_local());
      RCLCPP_INFO(get_logger(),
          "Relaying tree targets as fine refinement regions on %s (r=%.2fm).",
          region_pub_->get_topic_name(), fine_region_radius_m_);
    }
  }

  // --- State machine timer (10 Hz, sim time) ---
  tick_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(100),
      [this] { tick(); });

  // --- Periodic CSV sampler (sim time, same clock as the state machine).
  //     Shares the node's default (mutually-exclusive) callback group with
  //     tick(), so a row can never be assembled from half-updated state.
  if (metrics_period_sec_ > 0.0) {
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(metrics_period_sec_));
    metrics_timer_ = rclcpp::create_timer(
        this, get_clock(), period, [this] { metricsTick(); });
    RCLCPP_INFO(get_logger(),
        "Metrics: sampling every %.1f s in ALL states (rows where state != "
        "LOG_STEP); end-of-step rows unchanged.", metrics_period_sec_);
  } else {
    RCLCPP_WARN(get_logger(),
        "Metrics: periodic sampling DISABLED (metrics_period_sec=0) — the CSV "
        "will have no rows during reconnect manoeuvres.");
  }

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
  // Event log first: run_start must be the file's first line, and it can only
  // be emitted once the clock is live (see startExperimentLog). The peer sweep
  // rides the same tick rather than the coordination heartbeat, which returns
  // early when coordination_enabled is false — the control arm needs its
  // outage timing recorded too.
  startExperimentLog();
  expClockAnchorTick();
  expPeerSweep();

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
  if ((state_ == State::NAVIGATE || state_ == State::RETURN_NAV ||
       state_ == State::PURSUE) &&
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
          transitionTo(State::PLAN, "startup-preconditions-met");
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

    case State::PURSUE:
      doPursue();
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
      //
      // The latch criterion overrides that: "finished" is a one-way property of
      // this robot, and leaving DONE would contradict it — the run-completion
      // rule (every planner reads DONE) is evaluated on the state column, so a
      // latched robot that flicks back out to EXPLOIT_PLAN would re-open a run
      // it already ended. Targets are not part of this experiment's completion
      // question; if a configuration ever needs both, it wants done_criterion=
      // streak, where DONE is genuinely revocable.
      if (done_action_ == "idle") {
        if (coverage_latched_) {
          RCLCPP_INFO_ONCE(get_logger(),
              "Exploration finished [latch] at t_sim=%.1f; idling and "
              "beaconing. This robot does not leave DONE.",
              coverage_latch_t_sim_);
          break;
        }
        if (exploitation_enabled_ && target_queue_.hasPending()) {
          target_queue_.activate();
          phase_ = Phase::EXPLOIT;
          RCLCPP_INFO(get_logger(),
              "Target arrived while DONE-idle (%zu pending) -> EXPLOIT.",
              target_queue_.pendingCount());
          transitionTo(State::EXPLOIT_PLAN, "target-arrived-done-idle");
        } else {
          RCLCPP_INFO_ONCE(get_logger(),
              "Exploration finished; idling (done_action=idle). Planner "
              "stays up and will exploit any targets that arrive.");
        }
        break;
      }
      if (!shutdown_requested_) {
        shutdown_requested_ = true;
        // Disarm any fine-band regions whose target never reached DONE:
        // the step-budget checks and the rendezvous give-up land here
        // WITHOUT passing finishActiveTarget, and shutting down with their
        // regions live would leave scovox fine-integrating those trunks for
        // the rest of the run. The actual shutdown is deferred one tick so
        // DDS gets a cycle to flush the removes (a reliable publisher's
        // unsent history dies with the process).
        removeLiveRefinementRegions();
        RCLCPP_INFO(get_logger(),
            "Exploration finished. Shutting down planner node.");
        break;
      }
      rclcpp::shutdown();
      break;
  }
}

void ExploPlannerNode::transitionTo(State s, const char* reason) {
  const State from = state_;
  // Reconnect clock. The manoeuvre owns RETURN_NAV / RETURN_SYNC / PURSUE, plus
  // any PROXIMITY_HOLD taken while already inside one. Staying within that set
  // keeps the clock running, which is what lets a single manoeuvre span the
  // RETURN_NAV -> RETURN_SYNC arrival and the HYBRID chase -> meeting-point
  // handoff without restarting; leaving it means the manoeuvre resolved, one
  // way or the other, so the clock stops and the CSV reverts to its sentinel.
  const bool manoeuvre = (s == State::RETURN_NAV || s == State::RETURN_SYNC ||
                          s == State::PURSUE || s == State::PROXIMITY_HOLD);
  if (reconnect_active_ && !manoeuvre) {
    const double manoeuvre_sec = (this->now() - reconnect_start_time_).seconds();
    RCLCPP_INFO(get_logger(),
        "Reconnect manoeuvre ended after %.1f s sim (-> %s).",
        manoeuvre_sec, stateName(s));
    if (exp_log_) {
      const int live =
          coord_ ? static_cast<int>(coord_->livePeerCount(this->now())) : 0;
      ReconnectEndEvent e;
      // Classified mechanically from the two facts that decide it, with the
      // raw fields alongside so an analysis can re-classify: the team being
      // complete AT THIS INSTANT is what "reconnected" means (every release
      // path in the manoeuvre states tests exactly that), and landing in DONE
      // instead of PLAN is what "gave up" means.
      e.outcome = teamComplete(live, rendezvous_expected_peers_)
                      ? "reconnected"
                      : (s == State::DONE ? "gave_up" : "abandoned");
      e.to_state       = stateName(s);
      e.reason         = reason;
      e.duration_sec   = manoeuvre_sec;
      e.terminal       = reconnect_terminal_;
      e.peers_live     = live;
      e.expected_peers = rendezvous_expected_peers_;
      exp_log_->logReconnectEnd(expCtx(), e);
    }
    reconnect_active_ = false;
    hold_escalated_ = false;
    // The armed-from pair belongs to the manoeuvre that just ended; the next
    // dispatch takes its own snapshot from its own query.
    have_reconnect_rec_ = false;
    // Mid-run cooldown starts HERE, at manoeuvre end — missing_for stays
    // satisfied for the whole outage, so a dispatch-stamped cooldown would
    // expire during the manoeuvre and re-dispatch on the first PLAN tick.
    if (!reconnect_terminal_) {
      midrun_last_end_ = this->now();
      midrun_end_armed_ = true;
      reconnect_terminal_ = true;
    }
  }
  // The release-confirm dwell never survives a state change: a flicker that
  // straddles e.g. a PROXIMITY_HOLD must restart its window.
  release_ok_armed_ = false;

  // state_change, emitted before the new state is installed so `from_dwell_sec`
  // still measures the state being LEFT. -1 when the dwell is unknowable: the
  // very first transition of the run, where state_enter_time_ is still the
  // default-constructed SYSTEM-clock value and subtracting it from a sim-time
  // now() would throw inside a timer callback.
  if (exp_log_) {
    exp_log_->logStateChange(
        expCtx(), stateName(from), stateName(s), reason,
        have_state_enter_ ? (this->now() - state_enter_time_).seconds() : -1.0);
  }

  state_ = s;
  state_enter_time_ = this->now();
  have_state_enter_ = true;
  // DONE is the single funnel for every ending the node reaches while running
  // (coverage, step budget, barrier give-up), so the ending's REASON is
  // captured here rather than at each of those sites.
  //
  // Whether run_end is written here depends on what DONE means for this
  // configuration, because run_end must be the LAST line of the file — an
  // analysis reads it to decide whether the file is complete, and lines after
  // it would make `events_written` a lie:
  //   done_action=shutdown — DONE is terminal, the node exits within a tick, so
  //     write it now while the run totals are still meaningful.
  //   done_action=idle     — the node stays up and a target arriving later
  //     pulls it back into the exploit sub-loop, which produces more events.
  //     Defer to the destructor, which the last of them precedes by
  //     construction, and carry this reason there so the ending is still named
  //     properly rather than degrading to "node-destroyed".
  if (s == State::DONE) {
    done_reason_ = reason;
    if (done_action_ != "idle") logRunEnd(reason);
  }
  // The post-arrival rotation deadline is per-NAVIGATE-cycle and is armed
  // lazily on arrival at the XY goal; disarm it on every entry.
  if (s == State::NAVIGATE) rotate_deadline_armed_ = false;
  // The dwell-sync barrier is per-DWELL: its wait clock starts at entry (the
  // moment the robot is physically staged on the vantage) and neither latch may
  // survive into the next capture — a timed-out barrier would otherwise disable
  // the barrier for every remaining vantage of the run, and a started one would
  // skip the wait at the next vantage entirely.
  if (s == State::EXPLOIT_DWELL) {
    dwell_sync_wait_start_ = state_enter_time_;
    dwell_sync_timed_out_  = false;
    dwell_sync_started_    = false;
    // Declare this robot staged on the claim its teammates hold their barriers
    // against. This entry is the only point that can honestly say it: the dwell
    // is reached from NAVIGATE only after BOTH the XY-arrival gate and the
    // post-arrival yaw settle, so the platform is standing still on the vantage
    // it claimed — not merely inside a tolerance of it, and not parked on an
    // approach waypoint. Published on the spot instead of waiting for the next
    // ~1 Hz heartbeat, for the same reason as the post-dwell dwelled_mask
    // broadcast: a peer already parked on its own angle is re-anchoring its
    // dwell clock every tick until it hears this, so a second of stale staging
    // is a second shaved off the team's simultaneous-capture window.
    if (intent_pub_ && have_active_intent_) {
      current_intent_msg_.staged = true;
      current_intent_msg_.header.stamp = state_enter_time_;
      publishIntent();
    }
  }
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
  visited_goals_.prune(plan_start.seconds(), visited_goal_ttl_sec_);
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
    transitionTo(State::EXPLOIT_PLAN, "target-queued");
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
  //
  // Under done_criterion == "latch" this site is NOT the deciding one — the
  // metrics tick is, because it runs in every state — but the test is repeated
  // here anyway so the criterion does not silently depend on the sampler being
  // enabled (metrics_period_sec <= 0 disables that timer entirely), and so a
  // robot that saturates between two sampler ticks finishes on the PLAN tick
  // rather than waiting out the sampler period. maybeLatchCoverageDone is
  // idempotent, so evaluating it from both hooks latches exactly once.
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

  // Mid-exploration reconnect trigger (off unless reconnect_midrun_silence_sec
  // > 0). Sits AFTER the exploit branch (targets are the mission deliverable
  // and defer the trigger — remember that when reading firing times) and AFTER
  // the coverage check (a saturated robot must route through the terminal
  // path). Only evaluated in PLAN, i.e. between hops: detection latency past
  // the silence crossing is one residual hop (typically 15-60 s), which is
  // per-robot jitter the analysis inherits. livePeerCount is re-read here
  // because the heartbeat-maintained clock is quantized at 1 Hz and starvable
  // — without the re-check a peer that reconnected within the last heartbeat
  // period still reads missing and we brake for a manoeuvre that dissolves on
  // its first tick.
  if (reconnect_midrun_silence_sec_ > 0.0 && rendezvous_enabled_ &&
      have_anchor_ && team_seen_complete_) {
    if (midrun_attempts_ < reconnect_midrun_max_attempts_) {
      const auto trig_now = this->now();
      const int live =
          coord_ ? static_cast<int>(coord_->livePeerCount(trig_now)) : 0;
      const double missing_for =
          (trig_now - team_last_complete_time_).seconds();
      const bool cooldown_ok =
          !midrun_end_armed_ ||
          (trig_now - midrun_last_end_).seconds() >=
              reconnect_midrun_silence_sec_;
      // Threshold from the info gate when configured (dead-reckoned unshared
      // backlog crossing, clamped), else the legacy fixed clock — same value
      // exactly when reconnect_min_share_voxels=0. The COOLDOWN above stays
      // on the fixed clock either way: it paces retry pressure after a
      // failed manoeuvre, which has nothing to do with how much map the pair
      // holds. Gate maths runs only once the team actually reads incomplete:
      // midrunGateSec walks the peer table, and a fully-connected gated run
      // would otherwise pay that walk on every PLAN tick of the whole run.
      if (!teamComplete(live, rendezvous_expected_peers_) && cooldown_ok) {
        // WHICH CLOCK THE GATE COMPARES AGAINST (§30.11, §30.24). missing_for
        // is record age: it ages whenever the peer is not SENDING, which
        // includes a teammate sitting in a long PLAN tick two metres away. When
        // the link gate is armed the radio answers instead, and the two
        // possible answers are different in kind:
        //   link UP   -> there is nothing to reconnect to. Do not fire at all,
        //                however old the record is. This is the 16-21 % of
        //                fires that were pure loss.
        //   link DOWN -> compare the gate against how long the RADIO has been
        //                down, not how long the mailbox has been quiet.
        // -1 is a sentinel for "stand down this tick" and is checked before any
        // gate maths runs, because midrunGateSec walks the peer table and a
        // quiet-but-connected teammate would otherwise buy that walk on every
        // PLAN tick for the rest of the run.
        double link_down_for = missing_for;   // legacy clock unless gated
        bool   link_gated    = false;
        if (linkGateReady(trig_now)) {
          link_gated = true;
          if (link_connected_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                "Reconnect (mid-run): standing down — peer record silent "
                "%.0fs but the radio link is UP, so a chase would be spent on "
                "a peer that is already reachable.",
                missing_for);
            link_down_for = -1.0;
          } else {
            link_down_for = (trig_now - link_up_last_seen_).seconds();
          }
        }
        if (link_down_for >= 0.0) {
          double est_unshared = -1.0;
          // Fed missing_for ON PURPOSE, not link_down_for: the unshared-map
          // backlog accrues from the last time the pair actually exchanged
          // anything, which is last CONTACT. A peer that was connected but
          // quiet was still not sending deltas, so dating the backlog from
          // link-down would under-count it.
          const double gate_sec = midrunGateSec(missing_for, &est_unshared);
          if (link_down_for >= gate_sec) {
            ++midrun_attempts_;
            reconnect_terminal_ = false;
            hold_escalated_ = false;
            dispatch_gate_sec_      = gate_sec;
            dispatch_est_unshared_  = est_unshared;
            dispatch_link_down_sec_ = link_gated ? link_down_for : -1.0;
            RCLCPP_INFO(get_logger(),
                "Reconnect (mid-run): %s %.0fs >= gate %.0fs "
                "(record age %.0fs, est unshared %.0f vox, attempt %d/%d) -> "
                "interrupting exploration for the reconnect manoeuvre.",
                link_gated ? "radio link down" : "peer silent",
                link_down_for, gate_sec, missing_for, est_unshared,
                midrun_attempts_, reconnect_midrun_max_attempts_);
            if (dispatchReconnect("peer-lost")) return;
          }
        }
      }
    } else if (midrun_attempts_ == reconnect_midrun_max_attempts_) {
      ++midrun_attempts_;  // log the exhaustion exactly once
      RCLCPP_WARN(get_logger(),
          "Reconnect (mid-run): attempt budget exhausted (%d) — reverting to "
          "terminal-only reconnection for the rest of the run.",
          reconnect_midrun_max_attempts_);
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
  // TRO 2023), generalised by an exponent γ on the denominator:
  //
  //   U(c) = info_gain(c) / (ε + path_cost(c))^γ
  //
  // γ = 1 is the shipped form: information per unit distance, which at constant
  // speed is information per SECOND. That is the correct greedy objective when
  // the metric is time to completion, so γ = 1 is not an arbitrary default and
  // the burden of proof is on moving it. It is also the default here, and the
  // γ == 1 branch below skips std::pow so the shipped path stays bit-identical
  // rather than merely close.
  //
  // WHY THE KNOB EXISTS. Measured over 703 logged decisions on flatforest_dense
  // (campaign p14, off arm, to the 0.60 unknown rung): across the candidate set
  // at a single decision, path_cost spans roughly 5.8x while info_gain spans
  // only ~0.35 sd/mean. Cost enters linearly and varies far more, so argmax(U)
  // collapses to argmin(cost) — the planner chose goals at a median 7.8 m when
  // the mean candidate was 45.2 m away, i.e. it ran as nearest-frontier. The
  // same collapse is described from the other direction in shared_params.yaml
  // at candidate_min_goal_dist_m ("6% spread against a ninefold spread in
  // path_cost"). Solving for the γ at which a mean+2sd-information candidate at
  // the field's mean distance overtakes the one actually chosen gives a median
  // of 0.25 (p10 0.15, p90 0.42).
  //
  // WHAT IT DOES NOT FIX, stated so γ is not mistaken for a repair. Against the
  // map actually gained afterwards, info_gain has Spearman ρ ≈ +0.18 — real
  // (the null, raw distance, is ≈ 0) but weak, and its ~1.7x span cannot
  // separate outcomes that range over 600x. Lowering γ stops a nearly-flat
  // information term from being overruled by cost; it does not make that term
  // discriminate. The repair is the information model, not this exponent.
  //
  // ε (0.1 m) prevents division-by-zero for candidates at the robot's
  // feet and matches the SSMI reference implementation. It sits INSIDE the
  // power so the guard survives any γ: at γ = 0 the denominator is exactly 1
  // and U reduces to pure info_gain with distance ignored.
  //
  // Unreachable candidates (inf cost) get U = −∞ and sort to the bottom.
  constexpr float kCostEpsilon = 0.1f;
  // Hoisted out of the loop: γ is fixed for the life of the node, and the
  // equality test is on the same value the branch uses, so no candidate can
  // take a different path from its neighbour within one planning tick.
  const float kGamma = static_cast<float>(utility_cost_exponent_);
  const bool  kUnitGamma = (kGamma == 1.0f);

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
    //    by goal_xy_tolerance so they waste a step without any movement —
    //    and, when candidate_min_goal_dist_m is set, everything inside the
    //    sensor's useful standoff as well: a goal too close to observe from
    //    cannot clear the frontier that generated it, which is the whole
    //    near-frontier oscillation (see the param load).
    {
      const double near = std::max(goal_xy_tol_, cand_min_goal_dist_);
      float dx = vp.position.x() - robot_pos.x();
      float dy = vp.position.y() - robot_pos.y();
      if (dx * dx + dy * dy < static_cast<float>(near * near)) {
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
    // 3. Failed-goal blacklist (existing) + recently-visited suppression.
    //    Both counted as `blk` in the per-step log: they reject for the same
    //    reason from the planner's point of view -- do not go back there yet.
    if (visited_goal_radius_m_ > 0.0 &&
        visited_goals_.isNear(vp.position, visited_goal_radius_m_)) {
      ++rejected_blacklist;
      continue;
    }
    if (failed_goals_.isNear(vp.position, failed_goal_radius_m_)) {
      ++rejected_blacklist;
      continue;
    }
    // 4. MinPos peer-claim check (only when coordination is enabled).
    // live_after: exploration contests LIVE claims only. Exploit claims are
    // retained past expiry by the grace window so the vantage contests stay
    // closed across delivery gaps, but out here a graced claim would keep a
    // frontier candidate yielded to a peer we have not heard from in 10+
    // seconds — exploration keeps the original TTL semantics.
    if (coord_ && coord_->enabled()) {
      const auto* peer = coord_->claimMatching(
          vp.position, static_cast<float>(coord_claim_radius_m_),
          &plan_start);
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
  // Spread of info_gain across the candidate set. Second pass over the vector
  // already built above — no extra raycast — and deliberately two-pass rather
  // than the sum-of-squares shortcut: info_gain runs ~2.5e3 with a spread two
  // orders of magnitude smaller, so E[x^2] - E[x]^2 in float cancels away most
  // of the answer. Population (not sample) std, over the same denominator as
  // the mean above, so the two are directly comparable.
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

  auto plan_end = this->now();
  float plan_ms = static_cast<float>(
      (plan_end - plan_start).nanoseconds() * 1e-6);
  pending_plan_ms_ = plan_ms;  // drained into StepMetrics by doLogStep

  RCLCPP_INFO(get_logger(),
      "Step %d: selected goal (%.2f, %.2f) yaw=%.2f U=%.3f "
      "info=%.2f cost=%.2f field=%.2f±%.2f "
      "[%zu cand, close=%d map=%d unreach=%d blk=%d "
      "minpos=%d, peers=%zu, %.1fms]",
      step_, current_goal_.position.x(), current_goal_.position.y(),
      current_goal_.yaw, current_goal_.score,
      pending_selected_info_gain_, pending_selected_path_cost_,
      // `field` is mean±std of info_gain over ALL candidates, against which
      // `info` (the winner's) reads as a z-score by eye. A std that collapses
      // toward zero means the utility has stopped choosing on information.
      pending_mean_info_gain_, pending_info_gain_std_,
      candidates.size(), rejected_too_close, rejected_map,
      rejected_unreachable, rejected_blacklist, rejected_minpos,
      coord_ ? coord_->livePeerCount(plan_end) : 0u, plan_ms);

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
    // The released hop must be stopped, not just forgotten: EXPLOIT_PLAN can
    // sit on "no vantage and no reachable approach yet — retrying" for as long
    // as the give-up timer allows without ever publishing a goal, and nav2
    // would drive out the abandoned exploration hop underneath it (invariant:
    // see abandonNavGoal).
    abandonNavGoal("target released mid-hop");
    transitionTo(State::EXPLOIT_PLAN, "target-released-mid-hop");
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
    // PLAN usually re-goals on the next tick, but nothing guarantees it does —
    // and until it does nav2 is still driving INTO a now-mapped obstacle
    // (invariant: see abandonNavGoal).
    abandonNavGoal("goal inside obstacle");
    transitionTo(State::PLAN, "goal-inside-obstacle");
    return;
  }

  // In-flight cross-pick tiebreak, exploit VANTAGE hops only. The claim
  // heartbeat is 1 Hz, so around a target release both robots can select the
  // SAME ring angle before either has heard the other's claim — the
  // selection-time yield in doExploitPlan cannot see a claim that has not
  // arrived yet, and with that yield in place neither would ever release the
  // angle afterwards. Settle it with the same total order MinPos uses:
  // selfWinsAgainst is strict closer-distance with a lexicographic robot_id
  // tiebreak, so on the same pair of inputs exactly one of the two abandons —
  // never both (the ring silently loses an angle) and never neither (they
  // re-converge on one point and the proximity guard brakes the loser).
  // Approach hops carry no ring angle to contest; exploration hops are
  // deconflicted at selection time by the exploration-scale MinPos disc.
  if (phase_ == Phase::EXPLOIT && !current_is_approach_ &&
      pending_exploit_target_id_ >= 0 && coord_ && coord_->enabled()) {
    const auto* peer = coord_->claimMatching(
        current_goal_.position,
        static_cast<float>(coord_vantage_claim_radius_m_));
    if (peer && peer->exploit &&
        peer->target_id == static_cast<uint32_t>(pending_exploit_target_id_) &&
        !coord_->selfWinsAgainst(latest_pos_, current_goal_.position, *peer,
                                 robot_name_)) {
      RCLCPP_INFO(get_logger(),
          "Yielding vantage %d of target %d to '%s' (simultaneous pick) -> "
          "re-planning another angle.",
          current_vantage_index_, pending_exploit_target_id_,
          peer->robot_id.c_str());
      // Release the claim WITH the hop. The winner does not need it — its
      // symmetric check computes the same total order whether it sees our claim
      // or none at all — but its dwell-sync barrier reads it: a kept claim is
      // this yielded angle with staged=false, republished by the heartbeat for
      // as long as we sit in EXPLOIT_PLAN with nothing else selectable (the
      // 3-vantage / 2-robot final round), so the robot standing on the angle we
      // yielded would hold its dwell against US until our per-target give-up
      // fired, ~minutes for a hop we abandoned in one tick. Our next real claim
      // is whatever the re-plan picks. The DRIVE is stopped as well, and for
      // the same final round: the re-plan may find nothing selectable at all
      // (visited + claimed exhaust the ring between two robots), and a planner
      // retrying in EXPLOIT_PLAN must not still be rolling toward the angle it
      // just conceded — that heading is a collision course with the winner
      // standing on it.
      have_active_intent_ = false;
      abandonNavGoal("yielded vantage");
      transitionTo(State::EXPLOIT_PLAN, "vantage-yielded");
      return;
    }
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
          transitionTo(State::EXPLOIT_PLAN, "approach-waypoint-reached");
          return;
        }
        // Reached a vantage: dwell. Hold the MinPos claim through the dwell so a
        // peer doesn't poach this angle mid-capture; finishActiveTarget releases
        // it when the target closes.
        RCLCPP_INFO(get_logger(),
            "Reached vantage %d of target %d (dist=%.2f) -> dwelling %.1fs.",
            current_vantage_index_, pending_exploit_target_id_, dist,
            exploit_dwell_sec_);
        transitionTo(State::EXPLOIT_DWELL, "vantage-reached");
        return;
      }
      // Exploration goal reached: integrate the new observation.
      have_active_intent_ = false;
      // Park it in the visited set so the next PLAN does not immediately pick
      // it again. In a forest the frontier set never drains -- every trunk
      // casts a permanently unknown shadow -- so "go to the nearest frontier"
      // has no natural stopping point and the planner ping-pongs between two
      // neighbouring clusters indefinitely. Recording where it has just BEEN
      // is what breaks the cycle; recording only where it failed (below) never
      // could, because these goals are reached successfully every time.
      visited_goals_.add(current_goal_.position, this->now().seconds());
      RCLCPP_INFO(get_logger(),
          "Goal reached: dist=%.2f yaw_err=%.1f deg", dist,
          yaw_err * 180.0f / static_cast<float>(M_PI));
      transitionTo(State::INTEGRATE, "goal-reached");
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
  // The nav budget / no-progress watchdogs give up on this goal; nav2 does not
  // know that — the goal is still accepted and still driving (invariant: see
  // abandonNavGoal), and INTEGRATE is one of the states presumed stationary.
  abandonNavGoal(reason);
  transitionTo(State::INTEGRATE, reason);
}

// ==================================================================
// Rendezvous (multi-robot reconnection)
// ==================================================================

// Called at exploration exhaustion (coverage saturated). If rendezvous is on,
// an anchor is known, and a teammate is still out of comms, run the
// reconnect_mode_ manoeuvre; otherwise finish. When the whole team is
// already present the map is already merged, so exhaustion here means the team
// is genuinely done — everyone reaches this together and lands in DONE.
void ExploPlannerNode::recordExplorationComplete(const char* reason) {
  const int active =
      coord_ ? static_cast<int>(coord_->livePeerCount(this->now())) : 0;
  // exploration_complete — THIS ROBOT declaring its own exploration exhausted,
  // recorded here and not at any of the endings below. That is the point: what
  // happens next is the independent variable (finish / return / chase / hold),
  // so an event emitted at the ending would measure a different thing in every
  // arm, while this instant means the same thing in all of them.
  //
  // Keyed on step_ rather than a plain once-per-run latch: this function is
  // re-entered on every tick while the reconnect confirmation gate defers
  // (step_ cannot advance during a deferral, so that is still ONE event), and
  // again if a manoeuvre delivers a merged map with new frontiers in it and the
  // robot explores on and re-saturates (which is a genuine second declaration
  // and gets its own event, with occurrence > 1).
  if (exp_log_ && exp_complete_step_ != step_) {
    exp_complete_step_ = step_;
    ExplorationCompleteEvent e;
    e.reason            = reason;
    e.unknown_fraction  = last_unknown_fraction_;
    e.coverage_source   = last_coverage_source_;
    e.steps             = step_;
    e.distance_m        = cumulative_distance_;
    e.team_complete     = teamComplete(active, rendezvous_expected_peers_);
    e.peers_live        = active;
    e.expected_peers    = rendezvous_expected_peers_;
    e.occurrence        = ++exp_complete_count_;
    exp_log_->logExplorationComplete(expCtx(), e);
  }
}

bool ExploPlannerNode::finishNow(const char* reason) {
  // A DONE-idle robot must keep announcing itself: teammates that finish
  // LATER count peers via claim TTLs, and a silent finisher ages out of every
  // table within seconds — its teammate would then run the whole reconnect
  // manoeuvre against a robot that is parked in range, and wait at the
  // barrier forever. done_action=shutdown robots genuinely disappear; that
  // combination gets a startup warning (see the param load).
  if (done_action_ == "idle") {
    publishPresenceIntent();
  }
  transitionTo(State::DONE, reason);
  return true;
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
  RCLCPP_INFO(get_logger(),
      "Exploration complete [latch]: ROI unknown fraction %.3f <= %.3f "
      "(source=%s) in state %s at t_sim=%.1f — %d steps, %.2f m traveled. "
      "Latched; this robot does not un-finish.",
      unk, done_unknown_fraction_, source, stateName(state_),
      coverage_latch_t_sim_, step_, cumulative_distance_);

  // Order matters. Record the declaration against the state we were actually in
  // (transitionTo would otherwise have already moved us), then stop the
  // platform, then end. The abandon is what the streak path never needed: it
  // finished from PLAN, where the robot is stationary and nav2 holds no goal.
  // This one can fire mid-drive, and nav2 does not know the run is over — an
  // uncancelled goal keeps driving a "finished" robot around.
  recordExplorationComplete("coverage-latched");
  abandonNavGoal("coverage-latched");
  return finishNow("coverage-latched");
}

bool ExploPlannerNode::finishOrRendezvous(const char* reason) {
  // livePeerCount, not the raw table size: exploit claims are retained past
  // expiry by the grace window (vantage-contest lenience), and a graced claim
  // must not count a 10-s-silent teammate as "present" for the barrier.
  const int active =
      coord_ ? static_cast<int>(coord_->livePeerCount(this->now())) : 0;
  recordExplorationComplete(reason);
  if (shouldRendezvous(rendezvous_enabled_, have_anchor_, active,
                       rendezvous_expected_peers_)) {
    // Confirmation gate. `active` above is one read of a claim table that may
    // not yet have absorbed intents already delivered to this node, so a
    // manoeuvre committed on it can be released by the very next tick. Require
    // the team to have been continuously incomplete for reconnect_confirm_sec
    // first. Returning here leaves the node in PLAN with the coverage streak
    // already satisfied, so the next tick re-runs this check: the deferral
    // resolves either into a manoeuvre (peer still gone) or into DONE (peer
    // was there all along), and cannot loop, because team_last_complete_time_
    // only advances while the team IS complete — in which case
    // shouldRendezvous is false and we never reach this branch.
    if (reconnect_confirm_sec_ > 0.0 && team_seen_complete_) {
      const double missing_for =
          (this->now() - team_last_complete_time_).seconds();
      if (missing_for < reconnect_confirm_sec_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Reconnect: team incomplete (%d/%d) for only %.1fs of the %.1fs "
            "confirmation window [%s] — deferring the manoeuvre.",
            active, rendezvous_expected_peers_, missing_for,
            reconnect_confirm_sec_, reason);
        return false;
      }
    }
    // This is the only dispatch that may end the run: mark the manoeuvre
    // terminal so a failed barrier wait is allowed to reach DONE.
    reconnect_terminal_ = true;
    hold_escalated_ = false;
    return dispatchReconnect(reason);
  }
  if (rendezvous_enabled_ && rendezvous_expected_peers_ > 0) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: exploration ended [%s] with full team present "
        "(%d/%d peers) -> DONE.",
        reason, active, rendezvous_expected_peers_);
  }
  return finishNow(reason);
}

// Mode dispatch (mesh radios). Pursuit and hybrid try the chase first;
// startPursuit declines when the missing peer's record is too stale for
// its trail head to mean anything (pursuitBudgetSec == 0), and each mode
// then falls through to its fallback. rec can be null even here: the
// missing teammate may never have been heard at all (the anchor came from
// a DIFFERENT peer) — then there is nothing to chase and no pair to
// midpoint, so hybrid and rendezvous degrade to the own-anchor return
// while pure pursuit holds in place (below).
//
// Callers own reconnect_terminal_: finishOrRendezvous sets true (its barrier
// may end in DONE), the mid-run trigger in doPlan sets false (its barrier
// must resume exploring).
// ==================================================================
// Info-gated mid-run trigger (map-size beacon)
// ==================================================================

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
  // Winsorised slope: per-interval deltas, each capped at kRateWinsorK x the
  // window's median delta, summed over the window's total elapsed time. The
  // cap is what makes this a PRIVATE gathering rate rather than a total one:
  // a reconnection merges the peer's map into this same cumulative count, a
  // step of up to 243k voxels in ONE sample (measured, p14), which a plain
  // two-point slope reports as growth this robot never sensed. Deltas are
  // floored at 0 for the mirror artifact (a map reload shrinking the count).
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

bool ExploPlannerNode::linkGateReady(const rclcpp::Time& now) {
  if (comms_link_states_topic_.empty()) return false;   // feature off
  if (!link_index_usable_ || !link_have_sample_ || !link_clock_anchored_) {
    // Configured but not delivering. Say so, throttled: silently reverting to
    // the record-age clock is exactly the "check that stopped checking" shape —
    // the run would log as though the gate were in force while behaving like
    // the binary this change replaces.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000,
        "Link gate configured on '%s' but not usable yet (index %s, samples "
        "%s): the mid-run trigger is running on the record-age clock.",
        comms_link_states_topic_.c_str(),
        link_index_usable_ ? "ok" : "missing",
        link_have_sample_ ? "ok" : "none");
    return false;
  }
  const double age = (now - link_last_sample_time_).seconds();
  if (age > comms_link_stale_sec_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000,
        "Link gate: newest link sample is %.1fs old (> %.1fs); standing down to "
        "the record-age clock rather than acting on a stale belief.",
        age, comms_link_stale_sec_);
    return false;
  }
  return true;
}

double ExploPlannerNode::midrunGateSec(double missing_for,
                                       double* est_unshared_out) {
  if (est_unshared_out != nullptr) *est_unshared_out = -1.0;
  if (reconnect_min_share_voxels_ <= 0.0) {
    return reconnect_midrun_silence_sec_;   // gate off: legacy fixed clock
  }
  std::string peer_id;
  const LastContact* rec = missingPeerRecord(&peer_id);
  if (rec == nullptr) {
    // No contact snapshot to reckon from (anchor predates the record) —
    // the time-only clock is the only trigger that remains meaningful.
    // est_unshared = -2, not -1: a gated dispatch that fell back here would
    // otherwise log gate_sec = the fixed silence clock and est_unshared = -1,
    // byte-identical to a control run's dispatch — the one situation these
    // diagnostics exist to tell apart.
    if (est_unshared_out != nullptr) *est_unshared_out = -2.0;
    return reconnect_midrun_silence_sec_;
  }
  // Diagnostic backlog estimate: my delta is EXACT (own cached count), the
  // peer's is dead-reckoned at its beaconed rate. Logged on the dispatch
  // event so the offline analysis can score the estimator against the
  // transfer actually measured at the merge.
  if (est_unshared_out != nullptr) {
    *est_unshared_out = (latest_map_voxels_ - rec->self_voxels) +
                        std::max(0.0, rec->peer_rate) * missing_for;
  }
  // The TRIGGER TIME, by contrast, uses only snapshot-frozen quantities so
  // both robots derive the same value (see LastContact). rate 0 = unknown
  // (pre-field peer / no samples): the quotient blows past the ceiling and
  // the clamp turns it into "wait for the backstop", which is the correct
  // reading of "nothing known to share".
  //
  // SUM, not mean: once each side's rate is a PRIVATE gathering rate (the
  // winsorised estimator in noteMapSize — a total-count rate would already
  // include the peer's contribution and summing it would double-count), the
  // backlog is the union of two disjoint gatherings and its growth is their
  // sum. Checked, not assumed: summed = 1.12x p14's measured pair divergence,
  // where the mean would read 0.56x and fire roughly twice too late.
  const double rate_sum =
      std::max(0.0, rec->self_rate) + std::max(0.0, rec->peer_rate);
  const double t = (rate_sum > 1e-9)
                       ? reconnect_min_share_voxels_ / rate_sum
                       : reconnect_midrun_max_silence_sec_;
  return std::min(reconnect_midrun_max_silence_sec_,
                  std::max(reconnect_midrun_min_silence_sec_, t));
}

bool ExploPlannerNode::dispatchReconnect(const char* reason) {
  std::string peer_id;
  const LastContact* rec = missingPeerRecord(&peer_id);
  // Event-log context for whichever leaf commits the action below. Stashed
  // from the query the DECISION was taken on, so the event can never report a
  // peer or a record age the dispatch did not actually see; cleared decline
  // reason so a decline from an earlier dispatch cannot be charged to this one.
  dispatch_peer_id_      = peer_id;
  dispatch_peer_age_sec_ =
      rec ? (this->now() - rec->stamp).seconds() : -1.0;
  reconnect_decline_reason_.clear();
  // Freeze the pair THIS manoeuvre is armed from. Everything downstream that
  // needs a meeting point — the RENDEZVOUS/HYBRID dispatch below, and the hold
  // escalation when this manoeuvre's barrier expires — reads the snapshot, so a
  // packet arriving mid-manoeuvre cannot move our midpoint away from the one
  // the peer computes from the same contact event. See reconnect_rec_.
  have_reconnect_rec_ = (rec != nullptr);
  if (rec != nullptr) reconnect_rec_ = *rec;
  if (rec == nullptr) {
    // The missing teammate was never heard at all (the anchor came from a
    // different peer), so there is nothing to chase and no pair to midpoint.
    reconnect_decline_reason_ = "no-peer-record";
  }
  if (reconnect_mode_ != ReconnectMode::RENDEZVOUS && rec != nullptr &&
      startPursuit(peer_id, *rec, reason)) {
    return true;
  }
  if (reconnect_mode_ == ReconnectMode::HYBRID && rec != nullptr) {
    startReturnTo(meetingPoint(rec->self_pose, rec->peer_pose),
                  "meeting point", reason);
    return true;
  }
  if (reconnect_mode_ == ReconnectMode::PURSUIT) {
    // Pure pursuit has no agreed fallback point by design (that is the A/B
    // against hybrid). A chase that never started therefore goes back to
    // exploring while its fallback budget lasts, and only parks here once
    // that is spent — parking early is the mutual-hold fixed point that cost
    // a p7modes mission (see pursuit_explore_fallback_).
    if (pursuitExploreFallback(reason)) return true;
    holdForTeam(reason);
    return true;
  }
  // RENDEZVOUS. Both robots drive to the SAME point: the midpoint of the two
  // poses at last contact, which each end computes from its own last_contact_
  // record without any further exchange (I hold my pose and the peer's; it
  // holds the mirror image of the same pair, so both midpoints agree to
  // whatever the robots moved between the two receipt instants).
  //
  // It used to be last_connected_anchor_ — each robot returning to where IT was
  // standing at last contact. That cannot converge, and measurably did not:
  // the link dies at the edge of range, so the two anchors are one comms range
  // apart BY CONSTRUCTION. In p9log_rendezvous_seed1 atlas returned to
  // (3.50, -26.23) and bestla to (-3.22, 27.46) — 54 m apart — five times
  // between them, every manoeuvre timed out to PLAN having reconnected
  // nothing, and the pair burned 918 s and 1137 s of a 3439 s run doing it.
  // Two robots waiting for each other at opposite ends of the gap that
  // separated them is the failure mode, not bad luck.
  //
  // The midpoint is the same construction HYBRID's fallback already uses
  // above; sharing it is deliberate, so the arms differ in WHEN they go to a
  // meeting point and not in where the meeting point is.
  if (rec != nullptr) {
    startReturnTo(meetingPoint(rec->self_pose, rec->peer_pose),
                  "meeting point", reason);
    return true;
  }
  // No last-contact record (the peer was never heard, so there is no pair to
  // take a midpoint of). The own-pose anchor is all this robot has.
  startReturnTo(last_connected_anchor_, "last-connected anchor", reason);
  return true;
}

// Flicker guard: a single live claim (one intent inside the 5 s TTL) is
// enough to read the team complete for a tick, release a manoeuvre and reset
// the silence clock — crediting a "reconnection" on a range-edge flicker that
// drained no map deltas. With a positive confirm window the release condition
// must hold continuously that long. The dwell is cheap where it runs: PURSUE
// keeps driving (budget still ticking), the barriers keep waiting.
bool ExploPlannerNode::releaseConfirmed(bool eligible) {
  if (!eligible) {
    release_ok_armed_ = false;
    return false;
  }
  if (reconnect_release_confirm_sec_ <= 0.0) return true;
  const auto now = this->now();
  if (!release_ok_armed_) {
    release_ok_armed_ = true;
    release_ok_since_ = now;
    return false;
  }
  return (now - release_ok_since_).seconds() >= reconnect_release_confirm_sec_;
}

// Shared barrier-entry stand-down. The rendezvous/pursuit barrier is HARD:
// nothing preempts it. doNavigate() interrupts an exploration hop the moment a
// target arrives, but the RETURN/PURSUE states deliberately do NOT check
// target_queue_ — with rendezvous_max_wait_sec <= 0 (wait forever, the
// default) a robot that serviced trees on the way would leave its teammate
// blocked at the barrier indefinitely.
//
// Stand the queue down rather than destroying it: the ACTIVE target is
// demoted to PENDING and the phase reset to EXPLORE, so the exploit claim
// stops being broadcast (the fresh non-exploit intent the caller publishes
// overwrites current_intent_msg_, clearing exploit/target_id/dwelled_mask and
// staged — peers must not merge dwell credit from a robot that is driving
// home, nor hold a vantage barrier open for one) and
// doPlan() picks the target back up once the barrier releases. Leaving it
// ACTIVE mattered once the step-budget path started routing through here:
// that path can fire mid-exploitation, unlike coverage saturation.
void ExploPlannerNode::standDownExploitation() {
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
}

// Arm the drive to a barrier destination — the own-pose anchor (rendezvous
// mode) or the pair midpoint (hybrid fallback) — reusing the NAVIGATE
// smart-timeout + arrival test. A presence intent is published (and re-sent by
// the heartbeat, which fires in the RETURN states) so teammates arriving
// later count us at the barrier — without it two robots waiting at their own
// anchors would never see each other and would deadlock.
void ExploPlannerNode::startReturnTo(const Eigen::Vector3f& dest,
                                     const char* what, const char* reason) {
  standDownExploitation();

  // Arm the reconnect clock only if nothing is running it yet. In HYBRID this
  // is reached as the fallback leg of a spent pursuit, and the column has to
  // report the total time spent trying to reconnect — not just the last leg.
  if (!reconnect_active_) {
    reconnect_active_ = true;
    reconnect_start_time_ = this->now();
  }

  return_dest_label_ = what;
  current_goal_ = CandidateViewpoint{};
  current_goal_.position = dest;
  current_goal_.yaw = latest_yaw_;

  RCLCPP_INFO(get_logger(),
      "Rendezvous: dispatched [%s], team incomplete (%d/%d peers) "
      "-> returning to %s (%.2f, %.2f).",
      reason,
      coord_ ? static_cast<int>(coord_->livePeerCount(this->now())) : 0,
      rendezvous_expected_peers_, what, dest.x(), dest.y());

  // startReturnTo has exactly two destinations across all three call sites (the
  // hybrid fallback's meeting point; the own-pose anchor, both at dispatch and
  // on hold-escalation), and `what` is what already distinguishes them in the
  // log line above — so it is what the event's action is derived from.
  refreshDispatchContext();
  logReconnectDispatch(
      std::strcmp(what, "meeting point") == 0 ? "meeting_point"
                                              : "anchor_return",
      &dest, /*budget_sec=*/-1.0, reason);

  publishGoal(current_goal_);

  if (intent_pub_ && coord_) {
    current_intent_msg_ = coord_->buildIntent(
        current_goal_, latest_pos_, this->now(),
        static_cast<float>(coord_claim_ttl_sec_),
        static_cast<float>(coord_claim_radius_m_),
        /*planner_type_id (eig)=*/0u, map_frame_);
    publishIntent();
    have_active_intent_ = true;
  }

  transitionTo(State::RETURN_NAV, reason);

  const float dx = current_goal_.position.x() - latest_pos_.x();
  const float dy = current_goal_.position.y() - latest_pos_.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  // Manoeuvre legs are exempt from nav_max_timeout_sec: the smart-timeout
  // ceiling was sized for exploration hops, and a cross-world return clipped
  // to it dies tens of metres short of a destination whose whole value is
  // ARRIVING (the connected geometry). Distance-true budget, floor kept; the
  // no-progress window remains the stuck-robot watchdog.
  //
  // Exempt from THAT ceiling, but not unbounded — reconnect_nav_max_sec stops a
  // leg outspending the barrier it drives toward. The 20 s/m model rate is 3x
  // conservative, so this binds only on a leg failing SLOWLY; one failing fast
  // still exits on the 15 s no-progress window, the primary guard, unchanged.
  nav_budget_sec_ = std::max(
      nav_min_timeout_sec_,
      static_cast<double>(dist) * nav_safety_factor_ /
          std::max(nav_speed_est_mps_, 1e-3));
  if (reconnect_nav_max_sec_ > 0.0) {
    nav_budget_sec_ = std::min(nav_budget_sec_, reconnect_nav_max_sec_);
  }
  progress_check_time_ = state_enter_time_;
  progress_check_dist_ = cumulative_distance_;
}

// Drive toward the anchor. If the whole team reconnects en route, the barrier
// is already satisfied — re-plan without finishing the drive. On arrival (or if
// the anchor turns out unreachable) hand off to RETURN_SYNC to wait for the
// team from wherever we ended up.
void ExploPlannerNode::doReturnNav() {
  // livePeerCount — presence semantics, same note as finishOrRendezvous().
  const int active =
      coord_ ? static_cast<int>(coord_->livePeerCount(this->now())) : 0;
  if (releaseConfirmed(teamComplete(active, rendezvous_expected_peers_))) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: team reconnected en route (%d/%d) -> re-planning against "
        "merged map.", active, rendezvous_expected_peers_);
    // RETURN_NAV is a driving state: stop the platform before PLAN. doPlan
    // can spend ticks retrying (map load, all candidates rejected) with the
    // proximity guard off, and nav2 would keep executing the barrier goal
    // underneath it the whole time (see abandonNavGoal).
    abandonNavGoal("return-released");
    // Re-confirm saturation against the post-merge map instead of finishing
    // off the pre-barrier streak on the first PLAN tick.
    coverage_done_streak_ = 0;
    have_active_intent_ = false;
    transitionTo(State::PLAN, "return-released");
    return;
  }

  const auto robot_pos = latest_pos_;
  const float dx = robot_pos.x() - current_goal_.position.x();
  const float dy = robot_pos.y() - current_goal_.position.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  // reconnect_arrive_tol_m_, NOT goal_xy_tol_: the destination's value is
  // connectivity, not position (see the member). Both robots stopping within
  // this of the same meeting point leaves them <= 2x it apart, and it turns an
  // obstructed midpoint from a budget burn into an arrival beside it.
  if (dist < static_cast<float>(reconnect_arrive_tol_m_)) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: reached %s (dist=%.2f, tol=%.1f) -> waiting for team.",
        return_dest_label_.c_str(), dist, reconnect_arrive_tol_m_);
    // RETURN_SYNC assumes a stationary robot, but arrival-by-tolerance lands
    // BEFORE nav2 finishes its own goal: without an explicit stop the
    // controller keeps driving to the exact goal pose underneath the barrier
    // — and underneath whatever state the release transitions into next.
    // Same rationale as the release path above.
    abandonNavGoal("return-arrived");
    transitionTo(State::RETURN_SYNC, "return-arrived");
    return;
  }

  const auto now = this->now();
  const double elapsed = (now - state_enter_time_).seconds();
  // Distance-budgeted timeout / no-progress watchdog: if the destination
  // can't be reached, wait for the team from here rather than looping on the
  // drive (we're at least closer to comms than where exploration stranded
  // us). RETURN_SYNC assumes a stationary robot, so the failed drive must be
  // stopped explicitly — the meeting point in particular is a synthetic
  // coordinate that can be unreachable, making these exits routine in hybrid.
  if (elapsed > nav_budget_sec_) {
    RCLCPP_WARN(get_logger(),
        "Rendezvous: %s unreachable within budget (%.1fs, dist=%.2f) "
        "-> waiting for team from current pose.",
        return_dest_label_.c_str(), elapsed, dist);
    abandonNavGoal("return-budget");
    transitionTo(State::RETURN_SYNC, "return-budget");
    return;
  }
  const double window_elapsed = (now - progress_check_time_).seconds();
  if (window_elapsed > progress_window_sec_) {
    const float delta = cumulative_distance_ - progress_check_dist_;
    if (delta < progress_min_distance_m_) {
      RCLCPP_WARN(get_logger(),
          "Rendezvous: no progress toward %s -> waiting for team from "
          "current pose.", return_dest_label_.c_str());
      abandonNavGoal("return-no-progress");
      transitionTo(State::RETURN_SYNC, "return-no-progress");
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
  // livePeerCount — presence semantics, same note as finishOrRendezvous().
  const int active =
      coord_ ? static_cast<int>(coord_->livePeerCount(this->now())) : 0;
  if (releaseConfirmed(teamComplete(active, rendezvous_expected_peers_))) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: full team connected (%d/%d) -> re-planning against "
        "merged map.", active, rendezvous_expected_peers_);
    // Re-confirm saturation against the post-merge map instead of finishing
    // off the pre-barrier streak on the first PLAN tick.
    coverage_done_streak_ = 0;
    have_active_intent_ = false;
    transitionTo(State::PLAN, "barrier-released");
    return;
  }

  // Mid-run barriers give up on their own (short) cap; terminal barriers keep
  // the field cap, shortened after an escalation (the second wait is a
  // confirmation of failure, not a second full vigil).
  const double wait_cap =
      !reconnect_terminal_
          ? reconnect_midrun_max_wait_sec_
          : (hold_escalated_ ? hold_escalate_wait_sec_
                             : rendezvous_max_wait_sec_);
  const double waited = (this->now() - state_enter_time_).seconds();
  if (rendezvousWaitExpired(waited, wait_cap)) {
    if (!reconnect_terminal_) {
      // A mid-run attempt must never end the run: the map is not saturated
      // (the trigger only fires from an unsaturated PLAN tick), so give the
      // manoeuvre back its time and go explore. The cooldown stamps in
      // transitionTo when reconnect_active_ falls.
      RCLCPP_INFO(get_logger(),
          "Reconnect (mid-run): gave up after %.0fs at the barrier "
          "(%d/%d present) -> resuming exploration.",
          waited, active, rendezvous_expected_peers_);
      coverage_done_streak_ = 0;
      have_active_intent_ = false;
      transitionTo(State::PLAN, "midrun-barrier-expired");
      return;
    }
    if (hold_escalate_ && !hold_escalated_) {
      hold_escalated_ = true;  // sticky: an unreachable target must not
                               // re-escalate on every expiry forever
      // Escalate to the SAME point the dispatch picked — the midpoint of the
      // pair THIS manoeuvre was armed from (reconnect_rec_, not a re-read:
      // a packet heard while we waited must not move the target off the one
      // the peer is driving to). Escalating to this robot's own anchor sent a
      // waiting robot to a point one comms range from where its waiting peer
      // would go, which is the non-convergence documented in dispatchReconnect;
      // it has to be fixed in both places or a hold reintroduces it after the
      // dispatch avoided it.
      //
      // EXCEPT under pure PURSUIT, which has no agreed fallback point BY
      // DESIGN — that absence is the A/B against hybrid (see dispatchReconnect
      // and pursuitFallback, which both say so). A mode-blind escalation makes
      // pursuit perform hybrid's fallback ~300 s later and erases the contrast
      // the arm exists to measure, so pursuit keeps the own-anchor escalation
      // it had before the meeting point existed and is unchanged by it.
      const bool use_meeting =
          reconnect_mode_ != ReconnectMode::PURSUIT && have_reconnect_rec_;
      Eigen::Vector3f esc_target = Eigen::Vector3f::Zero();
      const char* esc_what = nullptr;
      if (use_meeting) {
        esc_target = meetingPoint(reconnect_rec_.self_pose,
                                  reconnect_rec_.peer_pose);
        esc_what = "meeting point";
      } else if (have_anchor_) {
        esc_target = last_connected_anchor_;
        esc_what = "last-connected anchor";
      }
      // No anchor at all means the peer was never heard this run; there is
      // nowhere to escalate TO, and the zero vector is the world origin, not a
      // destination. Fall through and give up.
      if (esc_what != nullptr) {
        const float dx = esc_target.x() - latest_pos_.x();
        const float dy = esc_target.y() - latest_pos_.y();
        // Same arrival tolerance the drive would use: escalating to a point we
        // are already "at" by that standard would re-wait in place.
        if (std::sqrt(dx * dx + dy * dy) >
            static_cast<float>(reconnect_arrive_tol_m_)) {
          RCLCPP_WARN(get_logger(),
              "Rendezvous: waited %.0fs for team (%d/%d present) -> escalating "
              "to the %s (%.2f, %.2f) before giving up.",
              waited, active, rendezvous_expected_peers_, esc_what,
              esc_target.x(), esc_target.y());
          startReturnTo(esc_target, esc_what, "hold-escalate");
          return;
        }
      }
      // Already there (or nowhere to go): fall through and give up now.
    }
    RCLCPP_WARN(get_logger(),
        "Rendezvous: waited %.0fs for team (%d/%d present); "
        "max_wait=%.0fs reached -> giving up and finishing.",
        waited, active, rendezvous_expected_peers_, wait_cap);
    // Same presence rule as finishOrRendezvous's DONE branch: a given-up
    // idle robot is still parked and countable — its late-arriving pursuer
    // must be able to release its own barrier on contact.
    if (done_action_ == "idle") {
      publishPresenceIntent();
    } else {
      have_active_intent_ = false;
    }
    transitionTo(State::DONE, "barrier-gave-up");
    return;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Rendezvous: waiting for team at the barrier (%d/%d present)%s.",
      active, rendezvous_expected_peers_,
      rendezvous_max_wait_sec_ > 0.0 ? "" : " (no timeout)");
}

// ==================================================================
// Mesh-reconnection pursuit (robot-carried radios)
// ==================================================================

// The teammate the barrier is actually waiting on: among the peers we have
// EVER heard (last_contact_), the one not currently live, freshest record
// first. Liveness is the raw claim expiry — the same presence semantics as
// livePeerCount, and deliberately not the exploit grace window.
const ExploPlannerNode::LastContact*
ExploPlannerNode::missingPeerRecord(std::string* peer_id_out) {
  const auto now = this->now();
  const LastContact* best = nullptr;
  for (const auto& [id, rec] : last_contact_) {
    if (coord_ && coord_->peerLive(id, now)) continue;
    if (best == nullptr || rec.stamp > best->stamp) {
      best = &rec;
      if (peer_id_out) *peer_id_out = id;
    }
  }
  return best;
}

// Arm the chase. The trail is the missing peer's last declared goal (the
// trail head — the freshest hypothesis of where it went) and then its last
// heard pose (sweeping the leg it was driving; on a mesh the link re-forms
// the moment ANY point of the sweep comes within range of it, so partial
// coverage of the leg is already useful). Returns false without touching any
// state when the budget comes back 0 — record too stale, or pursuit disabled
// by pursuit_budget_max_sec <= 0 — and the caller falls through to the
// mode's fallback.
bool ExploPlannerNode::startPursuit(const std::string& peer_id,
                                    const LastContact& rec,
                                    const char* reason) {
  const auto now = this->now();
  const double staleness = (now - rec.stamp).seconds();
  if (pursuit_staleness_max_sec_ > 0.0 &&
      staleness >= pursuit_staleness_max_sec_) {
    RCLCPP_INFO(get_logger(),
        "Pursuit: record of '%s' is %.0fs old (max %.0fs) — trail head "
        "worthless, skipping the chase.",
        peer_id.c_str(), staleness, pursuit_staleness_max_sec_);
    // Carried on the fallback's own dispatch event: "this arm held instead of
    // chasing" and "this arm chose to hold" are different results, and only the
    // decline reason separates them.
    reconnect_decline_reason_ = "record-stale";
    return false;
  }

  // Trail selection (see pursuit_goal_stale_sec_): a fresh record chases the
  // declared goal then the contact pose (the legacy trail); a stale one drops
  // the dead goal hypothesis and drives to the contact pose alone.
  const bool goal_stale = pursuit_goal_stale_sec_ > 0.0 &&
                          staleness > pursuit_goal_stale_sec_;
  std::vector<Eigen::Vector3f> trail;
  if (!goal_stale) {
    trail.push_back(rec.peer_goal);
    // The last heard pose only earns a waypoint when it is meaningfully apart
    // from the goal — a peer claiming a goal beside itself would produce two
    // coincident hops.
    if ((rec.peer_pose - rec.peer_goal).head<2>().norm() > 1.0f) {
      trail.push_back(rec.peer_pose);
    }
  } else {
    trail.push_back(rec.peer_pose);
  }

  double budget = 0.0;
  if (goal_stale) {
    // Distance-true budget: the contact-pose endpoint's value is geometric
    // (the swapped contact pair was a connected configuration), so it earns
    // the full model time — but ONLY if the cap covers the whole trail. A
    // partial chase ends at an arbitrary disconnected point, which is worse
    // than the mode's fallback (meeting point / hold-and-beacon).
    double trail_m = 0.0;
    Eigen::Vector3f prev = latest_pos_;
    for (const auto& wp : trail) {
      trail_m += (wp - prev).head<2>().norm();
      prev = wp;
    }
    const double trail_model_sec =
        trail_m * nav_safety_factor_ / std::max(nav_speed_est_mps_, 1e-3);
    if (trail_model_sec > pursuit_budget_max_sec_) {
      RCLCPP_INFO(get_logger(),
          "Pursuit: record of '%s' is %.0fs old (goal stale) and the contact "
          "pose is %.1f m away (needs %.0fs > cap %.0fs) — chase would die "
          "mid-trail, skipping to the fallback.",
          peer_id.c_str(), staleness, trail_m, trail_model_sec,
          pursuit_budget_max_sec_);
      reconnect_decline_reason_ = "trail-exceeds-budget-cap";
      return false;
    }
    budget = std::max(std::min(nav_min_timeout_sec_, pursuit_budget_max_sec_),
                      trail_model_sec);
  } else {
    const float dx = rec.peer_goal.x() - latest_pos_.x();
    const float dy = rec.peer_goal.y() - latest_pos_.y();
    const float trail_head_dist = std::sqrt(dx * dx + dy * dy);
    budget = pursuitBudgetSec(
        trail_head_dist, staleness, nav_speed_est_mps_, nav_safety_factor_,
        pursuit_staleness_max_sec_, nav_min_timeout_sec_,
        pursuit_budget_max_sec_);
    if (budget <= 0.0) {
      RCLCPP_INFO(get_logger(),
          "Pursuit: record of '%s' is %.0fs old (max %.0fs) — trail head "
          "worthless, skipping the chase.",
          peer_id.c_str(), staleness, pursuit_staleness_max_sec_);
      // Zero budget is either the staleness discount eating it or pursuit
      // disabled outright (pursuit_budget_max_sec <= 0); name which, because
      // the second means the arm never chased at all.
      reconnect_decline_reason_ = pursuit_budget_max_sec_ <= 0.0
          ? "pursuit-disabled" : "budget-zero-stale";
      return false;
    }
  }

  standDownExploitation();

  // See startReturnTo: first leg of the manoeuvre arms the clock, later legs
  // inherit it. Distinct from pursue_start_time_, which is the budget clock and
  // is refunded proximity-hold time — this one is never refunded, because the
  // hold really was time spent not exploring.
  if (!reconnect_active_) {
    reconnect_active_ = true;
    reconnect_start_time_ = now;
  }

  pursue_budget_sec_ = budget;
  pursue_peer_id_ = peer_id;
  pursue_rec_ = rec;
  pursue_start_time_ = now;
  pursue_waypoints_ = std::move(trail);
  pursue_wp_index_ = 0;

  RCLCPP_INFO(get_logger(),
      "Pursuit: dispatched [%s], '%s' out of comms (record %.0fs old%s) "
      "-> chasing its trail head (%.2f, %.2f), budget %.0fs, %zu waypoint(s).",
      reason, peer_id.c_str(), staleness,
      goal_stale ? ", goal stale — contact pose only" : "",
      pursue_waypoints_.front().x(), pursue_waypoints_.front().y(),
      pursue_budget_sec_, pursue_waypoints_.size());

  // The chase is committed here (state is armed, nothing below can decline it).
  // dispatch_peer_id_ is normally already this peer, but a pursuit re-armed
  // from anywhere else must still name its quarry, so set it from the argument.
  dispatch_peer_id_      = peer_id;
  dispatch_peer_age_sec_ = staleness;
  logReconnectDispatch("chase", &pursue_waypoints_.front(), pursue_budget_sec_,
                       reason);

  armPursuitWaypoint();
  return true;
}

// Publish the current trail waypoint as the nav goal and (re)enter PURSUE.
// Called for the first waypoint by startPursuit and again on each advance —
// re-entering resets state_enter_time_, so the per-waypoint nav budget and
// progress window restart while the pursuit budget keeps running from
// pursue_start_time_. The presence intent keeps the claim/beacon fresh for
// the same reason as the RETURN states: the pursued robot must be able to
// count us the moment the mesh re-forms. Known wart: the claim disc this
// broadcasts sits on the CHASED peer's own goal (RobotIntent has no flag to
// mark a chase beacon), so a returning peer can briefly MinPos-yield its own
// goal to us — self-limiting, since contact releases the chase within a tick
// and the stray claim ages out with the TTL.
void ExploPlannerNode::armPursuitWaypoint() {
  current_goal_ = CandidateViewpoint{};
  current_goal_.position = pursue_waypoints_[pursue_wp_index_];
  current_goal_.yaw = latest_yaw_;

  publishGoal(current_goal_);

  if (intent_pub_ && coord_) {
    current_intent_msg_ = coord_->buildIntent(
        current_goal_, latest_pos_, this->now(),
        static_cast<float>(coord_claim_ttl_sec_),
        static_cast<float>(coord_claim_radius_m_),
        /*planner_type_id (eig)=*/0u, map_frame_);
    publishIntent();
    have_active_intent_ = true;
  }

  transitionTo(State::PURSUE, "pursuit-waypoint");

  const float dx = current_goal_.position.x() - latest_pos_.x();
  const float dy = current_goal_.position.y() - latest_pos_.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  // Manoeuvre legs are exempt from nav_max_timeout_sec: the smart-timeout
  // ceiling was sized for exploration hops, and a cross-world return clipped
  // to it dies tens of metres short of a destination whose whole value is
  // ARRIVING (the connected geometry). Distance-true budget, floor kept; the
  // no-progress window remains the stuck-robot watchdog.
  //
  // Same reconnect_nav_max_sec ceiling as startReturnTo. Usually slack here —
  // pursue_budget_sec_ caps the whole chase and normally binds first — but a
  // single waypoint leg must not be able to exceed it either.
  nav_budget_sec_ = std::max(
      nav_min_timeout_sec_,
      static_cast<double>(dist) * nav_safety_factor_ /
          std::max(nav_speed_est_mps_, 1e-3));
  if (reconnect_nav_max_sec_ > 0.0) {
    nav_budget_sec_ = std::min(nav_budget_sec_, reconnect_nav_max_sec_);
  }
  progress_check_time_ = state_enter_time_;
  progress_check_dist_ = cumulative_distance_;
}

// Drive the trail. Release conditions, in priority order: the team is back
// in comms (the whole point — re-plan against the merged map, no arrival
// needed); the pursuit budget is spent (the chase hypothesis is dead — hand
// off to the mode's fallback); the current waypoint is reached or judged
// unreachable (advance to the next, or fall back when the trail is
// exhausted). Per-waypoint nav failures advance rather than abort: waypoint
// 2 can be reachable when waypoint 1 is not.
void ExploPlannerNode::doPursue() {
  const auto now = this->now();
  // livePeerCount — presence semantics, same note as finishOrRendezvous().
  const int active =
      coord_ ? static_cast<int>(coord_->livePeerCount(now)) : 0;
  // Release when the whole team is back — or when the CHASED peer alone is:
  // the chase has done its job either way, and on a 3+ team burning the rest
  // of the budget driving at a teammate that is already in comms only delays
  // dispatching on whoever is still missing (PLAN re-runs finishOrRendezvous
  // once saturation re-confirms, and missingPeerRecord picks the next one).
  const bool quarry_heard = coord_ && !pursue_peer_id_.empty() &&
                            coord_->peerLive(pursue_peer_id_, now);
  if (releaseConfirmed(teamComplete(active, rendezvous_expected_peers_) ||
                       quarry_heard)) {
    RCLCPP_INFO(get_logger(),
        "Pursuit: %s mid-chase (%d/%d) -> re-planning against merged map.",
        quarry_heard && !teamComplete(active, rendezvous_expected_peers_)
            ? "chased peer back in comms"
            : "team reconnected",
        active, rendezvous_expected_peers_);
    // PURSUE is a driving state: stop the platform before PLAN. doPlan can
    // spend ticks retrying (map load, all candidates rejected) with the
    // proximity guard off, and nav2 would keep driving at the missing
    // teammate's own last pose underneath it (see abandonNavGoal).
    abandonNavGoal("pursuit-released");
    // Re-confirm saturation against the post-merge map instead of finishing
    // off the pre-chase streak on the first PLAN tick.
    coverage_done_streak_ = 0;
    have_active_intent_ = false;
    transitionTo(State::PLAN, "pursuit-released");
    return;
  }
  const double chased = (now - pursue_start_time_).seconds();
  if (chased >= pursue_budget_sec_) {
    RCLCPP_WARN(get_logger(),
        "Pursuit: budget spent (%.0fs of %.0fs) without contact with '%s'.",
        chased, pursue_budget_sec_, pursue_peer_id_.c_str());
    pursuitFallback("pursuit-budget");
    return;
  }

  const float dx = latest_pos_.x() - current_goal_.position.x();
  const float dy = latest_pos_.y() - current_goal_.position.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  const double elapsed = (now - state_enter_time_).seconds();
  const double window_elapsed = (now - progress_check_time_).seconds();
  bool no_progress = false;
  if (window_elapsed > progress_window_sec_) {
    no_progress =
        (cumulative_distance_ - progress_check_dist_) < progress_min_distance_m_;
    progress_check_time_ = now;
    progress_check_dist_ = cumulative_distance_;
  }

  if (dist < goal_xy_tol_ || elapsed > nav_budget_sec_ || no_progress) {
    const bool arrived = dist < goal_xy_tol_;
    if (!arrived) {
      RCLCPP_WARN(get_logger(),
          "Pursuit: waypoint %zu/%zu unreachable (%s, dist=%.2f).",
          pursue_wp_index_ + 1, pursue_waypoints_.size(),
          elapsed > nav_budget_sec_ ? "nav budget" : "no progress", dist);
    }
    ++pursue_wp_index_;
    if (pursue_wp_index_ < pursue_waypoints_.size()) {
      RCLCPP_INFO(get_logger(),
          "Pursuit: %s waypoint %zu -> driving trail waypoint %zu/%zu "
          "(%.2f, %.2f).",
          arrived ? "reached" : "skipping", pursue_wp_index_,
          pursue_wp_index_ + 1, pursue_waypoints_.size(),
          pursue_waypoints_[pursue_wp_index_].x(),
          pursue_waypoints_[pursue_wp_index_].y());
      armPursuitWaypoint();
    } else {
      RCLCPP_INFO(get_logger(),
          "Pursuit: trail exhausted (%zu waypoint(s)) without contact with "
          "'%s'.", pursue_waypoints_.size(), pursue_peer_id_.c_str());
      pursuitFallback("trail-exhausted");
    }
    return;
  }

  republishGoal(current_goal_);
}

// Route a spent or exhausted chase to the mode's fallback. HYBRID drives to
// the deterministic meeting point and waits there — the pursued robot's own
// hybrid dispatch computes (approximately) the same point from its own
// record, so the worst case degenerates to the rendezvous guarantee. Pure
// PURSUIT waits wherever the chase ended: no agreed point is part of that
// method, which is exactly the A/B against hybrid.
void ExploPlannerNode::pursuitFallback(const char* why) {
  // A fresh decision point: the chase ran and ended (`why` says how), so
  // whatever was declined when it was ARMED is no longer the explanation for
  // what happens next.
  reconnect_decline_reason_.clear();
  if (reconnect_mode_ == ReconnectMode::HYBRID) {
    // pursue_rec_ is the pair the chase was armed from (snapshotted in
    // startPursuit): the midpoint must come from the SAME contact event the
    // peer computes its own midpoint from, not from a record a mid-chase
    // one-way packet may have refreshed on our side only.
    startReturnTo(meetingPoint(pursue_rec_.self_pose, pursue_rec_.peer_pose),
                  "meeting point", why);
    return;
  }
  if (pursuitExploreFallback(why)) return;
  holdForTeam(why);
}

// Abandon the manoeuvre and go back to exploring. Mirrors holdForTeam's exit
// hygiene (a driving state must be stopped explicitly; an open vantage claim
// must be demoted) but lands in PLAN instead of at a barrier. The coverage
// streak is cleared because the robot is genuinely resuming, not finishing:
// leaving it satisfied would re-trigger the same dispatch on the next tick.
void ExploPlannerNode::resumeExploring(const char* why) {
  standDownExploitation();
  abandonNavGoal(why);
  coverage_done_streak_ = 0;
  have_active_intent_   = false;
  transitionTo(State::PLAN, why);
  // Close the mid-run bookkeeping HERE and not only in transitionTo. That
  // clock block is gated on reconnect_active_, and a fallback whose chase was
  // DECLINED never armed a manoeuvre, so reconnect_active_ is false and the
  // block does not run. Without this the cooldown would never stamp — every
  // PLAN tick still sees missing_for past the threshold, so the robot would
  // re-dispatch and re-decline until it had burned all its attempts within
  // seconds — and reconnect_terminal_ would stay false, letting a LATER
  // terminal barrier resume exploring instead of ending the run.
  //
  // AFTER the transition, not before: transitionTo stamps reconnect_end with
  // reconnect_terminal_, so setting it here first would label a mid-run
  // manoeuvre that actually ran (chase armed, budget spent, then fell back to
  // exploring) as terminal — contradicting its own dispatch event and moving
  // its duration into the wrong bucket. Idempotent by construction: when the
  // chase DID arm, transitionTo's block has already stamped and the guard
  // below is false.
  if (!reconnect_terminal_) {
    midrun_last_end_    = this->now();
    midrun_end_armed_   = true;
    reconnect_terminal_ = true;
  }
}

// See pursuit_explore_fallback_ for why parking is the dominated option.
bool ExploPlannerNode::pursuitExploreFallback(const char* why) {
  if (!pursuit_explore_fallback_) {
    // Appended, not overwritten: the chase's own decline (record stale, trail
    // too long) is why we are here at all, and the hold that follows should
    // carry both halves of the explanation.
    if (!reconnect_decline_reason_.empty()) reconnect_decline_reason_ += "+";
    reconnect_decline_reason_ += "explore-fallback-disabled";
    return false;
  }
  if (pursuit_explores_ >= pursuit_explore_max_) {
    RCLCPP_INFO(get_logger(),
        "Pursuit: explore-fallback budget spent (%d/%d) [%s] -> holding for "
        "the team instead.",
        pursuit_explores_, pursuit_explore_max_, why);
    if (!reconnect_decline_reason_.empty()) reconnect_decline_reason_ += "+";
    reconnect_decline_reason_ += "explore-fallback-budget-spent";
    return false;
  }
  ++pursuit_explores_;
  RCLCPP_INFO(get_logger(),
      "Pursuit: no chase available [%s] -> resuming exploration "
      "(fallback %d/%d). A moving robot can still regain the link; a parked "
      "one can only be found.",
      why, pursuit_explores_, pursuit_explore_max_);
  // No destination: the manoeuvre is being given up in favour of exploring,
  // and the next goal is whatever PLAN picks. NB a resume_exploring dispatch
  // has no matching reconnect_end when the chase was DECLINED — there was no
  // manoeuvre clock running to stop (see resumeExploring).
  logReconnectDispatch("resume_exploring", nullptr, /*budget_sec=*/-1.0, why);
  resumeExploring(why);
  return true;
}

// Raise the RETURN_SYNC barrier at the CURRENT pose. Used by pure-pursuit
// endings and by the no-record corner of the pursuit dispatch. PURSUE is a
// driving state, so the platform must be stopped explicitly (the guard's
// stationary-state assumption; see abandonNavGoal) — ceasing to publish
// goal_pose alone leaves nav2 finishing the last accepted goal. The presence
// intent (kept fresh by the heartbeat, which fires in RETURN_SYNC) is what
// lets the pursued teammate count us whenever it comes into range.
void ExploPlannerNode::holdForTeam(const char* why) {
  standDownExploitation();
  abandonNavGoal(why);

  // Same idempotent arm as startReturnTo/startPursuit. Reached both as the
  // fallback of a spent chase (clock already running) and directly in PURSUIT
  // mode when the record was too stale to chase at all (clock not yet running,
  // and this wait is still the robot's reconnect attempt — it must be timed).
  if (!reconnect_active_) {
    reconnect_active_ = true;
    reconnect_start_time_ = this->now();
  }

  publishPresenceIntent();

  RCLCPP_INFO(get_logger(),
      "Reconnect: holding for the team at the current pose (%.2f, %.2f) "
      "[%s].", latest_pos_.x(), latest_pos_.y(), why);
  // The hold's "destination" is where it holds, which is where the robot
  // already is — recorded so a hold and an arrival are comparable geometry.
  refreshDispatchContext();
  logReconnectDispatch("hold", &latest_pos_, /*budget_sec=*/-1.0, why);
  transitionTo(State::RETURN_SYNC, why);
}

// Presence-only intent: goal = own pose. The claim disc this puts on the
// parked pose is deliberate — peers should not plan goals on top of a
// stationary robot — and the heartbeat keeps it fresh (RETURN_SYNC and
// DONE-idle are both in its state gate).
void ExploPlannerNode::publishPresenceIntent() {
  current_goal_ = CandidateViewpoint{};
  current_goal_.position = latest_pos_;
  current_goal_.yaw = latest_yaw_;

  if (intent_pub_ && coord_) {
    current_intent_msg_ = coord_->buildIntent(
        current_goal_, latest_pos_, this->now(),
        static_cast<float>(coord_claim_ttl_sec_),
        static_cast<float>(coord_claim_radius_m_),
        /*planner_type_id (eig)=*/0u, map_frame_);
    publishIntent();
    have_active_intent_ = true;
  }
}

// Re-publish the active intent on a fixed sim-time cadence so peers
// don't lose the claim through TTL while we're navigating to it.
// Stamps the message with the current time so peer expiry resets.
void ExploPlannerNode::heartbeatTick() {
  if (!coord_enabled_) return;
  // Team-presence clock for the reconnect confirmation gate. It lives here, on
  // the heartbeat timer, rather than at the exhaustion check, because the whole
  // point is to have a reading that predates the long PLAN tick — sampling it
  // inside finishOrRendezvous would sample exactly the stale table the gate
  // exists to distrust. Note this timer is serialised behind the state machine
  // on a single-threaded executor, so it too can be starved; that is the
  // conservative direction (a starved clock looks older, so the gate waits).
  if (coord_) {
    const auto pres_now = this->now();
    if (teamComplete(static_cast<int>(coord_->livePeerCount(pres_now)),
                     rendezvous_expected_peers_)) {
      team_last_complete_time_ = pres_now;
      team_seen_complete_ = true;
    }
  }
  // Suppression accounting (comms experiments). The beacon is STATE-GATED, so
  // a planner busy longer than coord_claim_ttl_sec in a non-beaconing state —
  // PLAN above all, which can loop indefinitely when every candidate is
  // rejected — reads as *missing* in every peer's claim table under perfect
  // comms. That is indistinguishable from a radio outage from the receiver's
  // side, and it is enough to arm a reconnect manoeuvre against a healthy
  // teammate. Logging the episodes here is what lets the analysis classify
  // each peer-missing window as outage (corroborated by the emulator's
  // link_states) or suppression (corroborated by these lines) instead of
  // charging planner latency to the radio.
  const bool beaconing =
      have_active_intent_ && intent_pub_ &&
      (state_ == State::NAVIGATE || state_ == State::INTEGRATE ||
       state_ == State::EXPLOIT_PLAN || state_ == State::EXPLOIT_DWELL ||
       state_ == State::RETURN_NAV || state_ == State::RETURN_SYNC ||
       state_ == State::PURSUE || state_ == State::PROXIMITY_HOLD ||
       state_ == State::DONE);
  const auto hb_now = this->now();
  // Second, independent suppression cause: EXECUTOR STARVATION. The block below
  // keys entirely off state_, so it can only see a beacon that was never
  // attempted. A beacon that was attempted LATE is invisible to it — and this
  // node spins a single-threaded executor, so every timer here is serialised
  // behind the state machine and behind the metrics sampler, whose map ingest +
  // grid walk grows with the fused map (millions of voxels by late run). If one
  // of those callbacks runs longer than coord_claim_ttl_sec the beacon simply
  // does not go out in time, peers age the claim out, and the analysis charges
  // a healthy link with an outage. Measuring the actual inter-tick interval is
  // the only way to see it from inside the node.
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
    // One WARN per episode, at the moment peers can first read us as gone.
    if (!hb_suppress_warned_ && held >= coord_claim_ttl_sec_) {
      hb_suppress_warned_ = true;
      RCLCPP_WARN(get_logger(),
          "Heartbeat suppressed %.1f s in state %s (>= claim TTL %.1f s): "
          "peers now read this robot as MISSING with the link up. Classify "
          "any peer-missing window overlapping this as suppression, not "
          "outage.", held, stateName(state_), coord_claim_ttl_sec_);
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
  // Re-publish while we hold a claim: NAVIGATE/INTEGRATE (exploration), the
  // EXPLOIT states, the RETURN states, PURSUE, and DONE. The dwell in
  // particular can outlast the claim TTL, so a peer would otherwise poach the
  // vantage angle mid-capture; in RETURN/PURSUE the beacon is what lets
  // teammates coming back into range count us and release the barrier; in
  // DONE (idle — finishOrRendezvous publishes the presence intent only then)
  // it is what keeps the FIRST finisher countable, so a teammate finishing
  // minutes later sees a full team instead of chasing a parked robot and
  // waiting forever. PROXIMITY_HOLD keeps beating too: the interrupted goal
  // is resumed after the hold, so its claim must survive, and the beacon
  // (with the live robot_pos refreshed below) is what feeds the right-of-way
  // peer's view of us while we sit in its way.
  if (state_ != State::NAVIGATE && state_ != State::INTEGRATE &&
      state_ != State::EXPLOIT_PLAN && state_ != State::EXPLOIT_DWELL &&
      state_ != State::RETURN_NAV && state_ != State::RETURN_SYNC &&
      state_ != State::PURSUE && state_ != State::PROXIMITY_HOLD &&
      state_ != State::DONE) {
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
  // Map size rides the heartbeat too: this is the 1 Hz beacon that maintains
  // peer belief, so it is the freshest value a peer can snapshot at the last
  // exchange before an outage — exactly the sample the info gate runs on.
  publishIntent();
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
  transitionTo(State::PROXIMITY_HOLD, "proximity-hold");
}

// Stop the platform on a transition that ABANDONS the in-flight hop instead of
// replacing it. Same wire mechanics as enterProximityHold, no hold state.
//
// The invariant: the proximity guard runs ONLY in NAVIGATE / RETURN_NAV /
// PURSUE (see tick()), on the explicit assumption that every other state is
// stationary. So
// any transition out of a driving state that does not IMMEDIATELY publish a
// replacement goal must stop the platform itself — because ceasing to publish
// goal_pose does NOT stop nav2: the last accepted NavigateToPose goal runs to
// completion. Without this, the robot keeps rolling in a state the guard has
// been told is standing still, which is the one combination the coordination
// layer cannot see.
//
// That is not hypothetical. In a 2-robot run both robots' rendezvous-synced
// dwells ended 3 ms apart, both re-planned in the same instant, and both picked
// the last remaining vantage of the target before either had heard the other's
// 1 Hz claim. The loser took the cross-pick yield in doNavigate, hopped to
// EXPLOIT_PLAN — and its re-plan found NOTHING selectable (two vantages
// visited, the third claimed by the winner), so it never published another
// goal. nav2 spent the next 29 s driving a robot the guard believed was
// "planning" 6.6 m across the ring into the peer standing on the very vantage
// it had just yielded. They collided.
//
// Belt and braces, for the same reason enterProximityHold uses both: the cancel
// is fire-and-forget on the wire (a dropped or rejected cancel must not leave
// nav2 driving), so a brake goal at our own pose goes out as well, and
// whichever order bt_navigator processes the pair the robot ends with either no
// goal or a zero-travel one. The brake is harmlessly preempted the moment a
// later tick does select a real goal — it costs one goal_pose message.
//
// Call sites are exits from DRIVING states, plus holdForTeam's barrier entry
// (reachable from PLAN/LOG_STEP via the pure-pursuit no-chase corner, where
// the cancel is a no-op and the brake zero-travel). All of them run long
// after the first pose, so latest_pos_ is live; do not call this from a state
// entered before the first pose, where it would command the frame origin.
void ExploPlannerNode::abandonNavGoal(const char* why) {
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
  } else {
    RCLCPP_WARN(get_logger(),
        "Nav abandon [%s]: cancel client for '%s' unavailable (proximity stop "
        "disabled, or the action server is not up) — the brake goal is the "
        "only stop command.",
        why, proximity_nav_cancel_action_.c_str());
  }
  // Brake in place. current_goal_ is deliberately left alone: callers still
  // read it (failGoal blacklists it) and the states we transition INTO
  // overwrite it when they select their own goal.
  CandidateViewpoint brake;
  brake.position = latest_pos_;
  brake.yaw = latest_yaw_;
  publishGoal(brake);

  RCLCPP_INFO(get_logger(),
      "Abandoning nav goal [%s]: cancelled + braking in place.", why);
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
    // Make the hatch an actual escape: the peer is (by construction of this
    // branch) still inside the trigger disc, so without a grace window the
    // very next tick's entry check re-holds and the "escape" is one 0.1 s
    // tick of driving between max_hold_sec holds, forever — exactly the
    // wedge the yaml's escape-hatch comment promises this parameter breaks.
    // The guard drops the immunity early if the peer starts moving again.
    prox_guard_->armEscape(d.peer_id, now);
  }

  // Resume the interrupted drive on the SAME goal. The re-publish is
  // mandatory — the cancel consumed nav2's goal, so only a fresh goal_pose
  // restarts it. state_enter_time_ is backdated by the drive time the goal
  // had already consumed, so the nav budget CONTINUES across the hold; the
  // progress window starts fresh (held time is not lack of progress). The
  // exploit give-up timer gets the held time back for the same reason: a
  // hold is not target stall.
  if (exploit_target_timing_) exploit_target_started_sec_ += held;
  // The pursuit budget gets the held time back too: yielding to a teammate is
  // not evidence the chase hypothesis is wrong, and charging it would let one
  // crossing-route hold convert a viable pursuit into its fallback.
  if (prox_resume_state_ == State::PURSUE) {
    pursue_start_time_ =
        pursue_start_time_ + rclcpp::Duration::from_seconds(held);
  }
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
  transitionTo(prox_resume_state_, "proximity-hold-released");
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

// Every column that is a property of the world at this instant, as opposed to
// an attribution of the last plan. Shared by the end-of-step row and the timer
// row; the plan-attribution columns (plan_time_ms, mean_/selected_*, the
// rejection counts, the exploited vantage) are deliberately NOT set here, so a
// timer row leaves them zero instead of repeating the last plan's numbers on
// every sample and inviting the analysis to average them.
void ExploPlannerNode::fillCommonMetrics(StepMetrics& m) {
  const auto now = this->now();
  m.step              = step_;
  m.sim_time_sec      = now.seconds();
  m.distance_traveled = cumulative_distance_;
  // livePeerCount: the column documents "peers heard within one TTL", and
  // grace-retained exploit claims must not inflate it (CSV schema unchanged,
  // only the count's honesty restored).
  m.coord_active_peers  = coord_
      ? static_cast<int>(coord_->livePeerCount(now))
      : 0;
  // Cumulative proximity-hold columns.
  m.prox_hold_count     = prox_hold_count_;
  m.prox_hold_total_sec = static_cast<float>(prox_hold_total_sec_);
  m.phase = (phase_ == Phase::EXPLOIT) ? "exploit" : "explore";
  m.state = stateName(state_);

  // Reconnect columns. Both stay at their -1 "not applicable" sentinel outside
  // a manoeuvre — 0.0 would read as "arrived", which is a real and different
  // thing to record.
  if (reconnect_active_) {
    m.reconnect_elapsed_sec =
        static_cast<float>((now - reconnect_start_time_).seconds());
    if (state_ == State::RETURN_SYNC) {
      // The barrier itself: the robot has stopped where it is going to wait, so
      // the distance left to run is zero by definition — not the range to
      // whatever goal it last drove toward.
      m.reconnect_range_to_goal_m = 0.0f;
    } else {
      // current_goal_ is the live manoeuvre destination in RETURN_NAV and
      // PURSUE, and survives a PROXIMITY_HOLD untouched (the hold publishes a
      // separate brake goal), so it is correct in all three.
      const float dx = current_goal_.position.x() - latest_pos_.x();
      const float dy = current_goal_.position.y() - latest_pos_.y();
      m.reconnect_range_to_goal_m = std::sqrt(dx * dx + dy * dy);
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

  // Coverage milestones ride the SAME measurement the CSV row and the DONE
  // criterion use, taken on the same tick — so "time to reach coverage X" can
  // never disagree with the curve it is read off. This is the hook, not a
  // separate sampler: every emitter of a CSV row (the end-of-step row and the
  // periodic timer row, which runs in every state including the whole of a
  // reconnect manoeuvre) passes through here, so the ladder is evaluated at the
  // sampling period even while the step counter is frozen.
  if (exp_log_) {
    exp_log_->noteCoverage(expCtx(), uf, cov_src,
                           static_cast<double>(cumulative_distance_),
                           static_cast<double>(latest_pos_.x()),
                           static_cast<double>(latest_pos_.y()));
  }

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

// Sample the CSV on a periodic timer, in every state. The period is SIM time
// (use_sim_time), so at RTF != 1 it is not a wall-clock period; the adaptive
// back-off below is the only part measured on a steady wall clock, because it
// bounds executor-thread work. Steps only advance through
// the explore loop, so without this a robot that spends four minutes in PURSUE
// or RETURN_NAV contributes one flat segment between two step rows across the
// exact interval the comms experiment measures.
void ExploPlannerNode::metricsTick() {
  if (!logger_ || !map_cache_) return;
  // Self-throttle. This callback is NOT cheap and gets more expensive as the
  // run proceeds: loadLatestMap() reallocates and re-inserts the whole fused
  // grid whenever a new map has arrived (dscovox publishes at 1 Hz, so at any
  // sane period one always has), and computeStats() then walks every voxel
  // evaluating digamma/log terms plus up to six neighbour lookups per free
  // voxel. Prior campaign CSVs reach ~4M voxels with plan_time_ms ~1700, so on
  // a single-threaded executor a fixed 5 s period would spend a large and
  // GROWING fraction of the node's only thread here — delaying the 1 Hz
  // coordination beacon past coord_claim_ttl_sec and manufacturing exactly the
  // peer-missing signal this experiment is trying to attribute to the radio.
  //
  // So the period floats: measure each tick, and if it cost more than
  // metrics_max_duty_ of the current period, stretch the period until it does
  // not. The sampling rate degrades (visibly, in the log) instead of the
  // planner's real-time behaviour degrading (invisibly, in the data).
  const auto mt_now = std::chrono::steady_clock::now();
  if (metrics_next_.time_since_epoch().count() != 0 && mt_now < metrics_next_)
    return;
  // LOG_STEP emits its own, strictly richer row on this same tick. Skipping it
  // here is what makes state=="LOG_STEP" a sound discriminator for end-of-step
  // rows instead of a coin flip on timer phase.
  if (state_ == State::LOG_STEP) return;

  // RE-INGEST FIRST — the whole reason this function is more than three lines.
  // total_observed_voxels is read off map_cache_, and map_cache_ is only ever
  // rebuilt by loadLatestMap(), which outside WAIT_FOR_MAP/PLAN is called by
  // nothing but the exploit states. A sample taken without this would report the
  // voxel count frozen at whatever the last PLAN ingested, for the entire
  // manoeuvre: a coverage curve that goes flat the moment a robot loses contact
  // whether or not it kept mapping — manufacturing precisely the result the
  // experiment is supposed to be testing for. Cheap when no new map has arrived
  // (pointer-identity check, no rebuild).
  loadLatestMap();

  StepMetrics m;
  fillCommonMetrics(m);
  logger_->logStep(m);

  // Realised-rate accounting for this sampler, reported in the event log's
  // run_end. The configured period is still not a guarantee: the gate above
  // is a WALL-clock deadline (it must be — it bounds executor work), so the
  // sim-time spacing of rows scales with RTF, and the duty back-off below
  // stretches it further under load. Measured from the sim stamps of the rows
  // actually written, which is the quantity an analysis needs.
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
  // Arm the next deadline from THIS tick's scheduled slot, not from "now"
  // after the row was written: post-work arming adds the tick's cost to every
  // period, and when the tick source runs at the same period (sim-clock timer
  // at RTF >= 1) that constant overshoot suppresses every second firing —
  // halving the realised rate with no config change, by an amount that
  // tracks how expensive the map walk happens to be at that point of the
  // run. Advancing the previous deadline keeps the schedule phase-locked to
  // the tick source; a schedule that has fallen behind re-anchors instead of
  // firing a catch-up burst.
  const auto eff_dur =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(eff_period));
  auto next = (metrics_next_.time_since_epoch().count() != 0)
                  ? metrics_next_ + eff_dur
                  : mt_now + eff_dur;
  if (next <= mt_now) next = mt_now + eff_dur;
  metrics_next_ = next;

  // THE DECIDING HOOK for done_criterion == "latch". This callback is the only
  // thing in the node that measures the ROI unknown fraction in EVERY state —
  // it re-ingests the fused map at the top precisely so the number stays live
  // during a reconnect manoeuvre — which is exactly the property the criterion
  // needs and exactly what the old top-of-doPlan test lacked. last_unknown_-
  // fraction_ was cached by the fillCommonMetrics call above, so the row that
  // reports the qualifying fraction and the tick that acts on it are the same
  // tick and cannot disagree. (The cached double, not m.unknown_fraction: that
  // field is a float and the comparison is against a threshold given in
  // decimal.)
  //
  // LAST in the function, not first, for two reasons: the qualifying sample is
  // data and must reach the CSV whatever the latch then does with it, and the
  // row/rate accounting above must count that final row or the realised
  // sampling rate reported in run_end is short by one.
  //
  // Safe to transition from here — this timer shares the node's default
  // (mutually-exclusive) callback group with tick(), so no state-machine
  // callback can be halfway through when this runs.
  maybeLatchCoverageDone(last_unknown_fraction_, last_coverage_source_);
}

// ==================================================================
// Experiment event log — see experiment_log.hpp
// ==================================================================

ExperimentContext ExploPlannerNode::expCtx() {
  ExperimentContext c;
  // The node's OWN clock, which is sim time under use_sim_time. The single
  // reason this file exists: every event in the log shares the axis the planner
  // actually makes decisions on, so nothing downstream has to map wall-clock
  // log stamps onto sim time through a real-time factor that drifts within a
  // run.
  c.sim_time_sec = this->now().seconds();
  c.state = stateName(state_);
  c.step  = step_;
  return c;
}

void ExploPlannerNode::startExperimentLog() {
  if (!exp_log_ || exp_log_->started()) return;
  // Under use_sim_time this->now() reads exactly 0 until the first /clock
  // message lands, and t0 = 0 would turn every t_rel_sec in the file into an
  // absolute sim time wearing a relative name — the precise class of silent
  // axis error this log replaces. Wait for a real stamp instead; the tick timer
  // itself only fires on that same clock, so this costs nothing.
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

// Peer belief. Deliberately maintained from the intent stream itself rather
// than read out of Coordination's claim table: the control arm runs with
// coordination_enabled=false, where that table is never consulted, and outage
// timing is exactly what the control arm exists to provide a baseline for. The
// threshold is coord_claim_ttl_sec, so the belief means the same thing the
// barrier's presence test means — and peers_live (which IS read from
// Coordination) rides on every peer event so the two can be cross-checked.
void ExploPlannerNode::expPeerHeard(const std::string& peer_id) {
  if (!exp_log_) return;
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
    e.last_contact_age_sec = -1.0;
    e.first_contact = true;
    e.peers_live = coord_ ? static_cast<int>(coord_->livePeerCount(now)) : 0;
    e.expected_peers = rendezvous_expected_peers_;
    exp_log_->logPeerSeen(expCtx(), e);
    return;
  }
  if (!it->second.live) {
    PeerEvent e;
    e.peer = peer_id;
    // The outage this message ends, measured from the last one that preceded
    // it — NOT from when the sweep noticed, which lags by up to a claim TTL.
    e.silent_sec = (now - it->second.last_heard).seconds();
    const auto rec = last_contact_.find(peer_id);
    e.last_contact_age_sec = (rec != last_contact_.end())
        ? (now - rec->second.stamp).seconds() : -1.0;
    e.peers_live = coord_ ? static_cast<int>(coord_->livePeerCount(now)) : 0;
    e.expected_peers = rendezvous_expected_peers_;
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
    const auto rec = last_contact_.find(peer_id);
    e.last_contact_age_sec = (rec != last_contact_.end())
        ? (now - rec->second.stamp).seconds() : -1.0;
    e.peers_live = coord_ ? static_cast<int>(coord_->livePeerCount(now)) : 0;
    e.expected_peers = rendezvous_expected_peers_;
    exp_log_->logPeerLost(expCtx(), e);
  }
}

// One event per manoeuvre DECISION, emitted by the leaf that commits the
// action. The alternative — logging inside dispatchReconnect's branch walk —
// cannot see the hold-escalation dispatch (which re-enters startReturnTo
// straight from the barrier) or the hybrid chase -> meeting-point handoff, and
// would drift out of step with the behaviour the first time a branch moved.
// dispatchReconnect stashes the peer context at the top of its branch walk, so
// the leaf it reaches reports exactly what the decision saw. Two leaves are
// reached WITHOUT that walk — hold escalation re-enters startReturnTo straight
// from the barrier, and pursuitFallback commits once a chase has run — and for
// those the stashed age is old by the entire wait or the entire chase budget,
// always in the flattering direction. Re-querying here costs one map lookup and
// makes "how stale was the record when the robot committed to this manoeuvre"
// mean the same thing on every dispatch event.
void ExploPlannerNode::refreshDispatchContext() {
  std::string peer_id;
  const LastContact* rec = missingPeerRecord(&peer_id);
  dispatch_peer_id_      = peer_id;
  dispatch_peer_age_sec_ = rec ? (this->now() - rec->stamp).seconds() : -1.0;
}

void ExploPlannerNode::logReconnectDispatch(const char* action,
                                            const Eigen::Vector3f* dest,
                                            double budget_sec,
                                            const char* reason) {
  if (!exp_log_) return;
  const auto now = this->now();
  ReconnectDispatchEvent e;
  e.mode     = reconnectModeName(reconnect_mode_);
  e.terminal = reconnect_terminal_;
  e.reason   = reason;
  e.peer     = dispatch_peer_id_;
  e.peer_record_age_sec = dispatch_peer_age_sec_;
  e.action   = action;
  if (dest != nullptr) {
    e.have_dest = true;
    e.dest_x = dest->x();
    e.dest_y = dest->y();
  }
  e.budget_sec     = budget_sec;
  e.decline_reason = reconnect_decline_reason_;
  // Consumed, not just read. A decline belongs to the dispatch that produced
  // it; dispatchReconnect clears the field on entry, but the two out-of-band
  // leaves (hold escalation re-entering startReturnTo from the barrier, and
  // pursuitFallback committing after a spent chase) never pass through that
  // entry and would otherwise re-report a decline from minutes earlier as
  // though it were the reason for THIS manoeuvre.
  reconnect_decline_reason_.clear();
  e.attempt        = reconnect_terminal_ ? 0 : midrun_attempts_;
  e.peers_live     = coord_ ? static_cast<int>(coord_->livePeerCount(now)) : 0;
  e.expected_peers = rendezvous_expected_peers_;
  // Gate diagnostics belong to the manoeuvre (same lifetime as
  // reconnect_rec_, NOT consumed-per-event like decline_reason): an
  // out-of-band leaf of a mid-run manoeuvre re-reports the gate decision it
  // was armed from, while terminal dispatches never carry a stale one.
  if (!reconnect_terminal_) {
    e.gate_sec         = dispatch_gate_sec_;
    e.est_unshared_vox = dispatch_est_unshared_;
    // Same lifetime and the same reason: a hold or resume_exploring leaf of
    // this manoeuvre re-reports the link-down duration the trigger fired on.
    // Terminal dispatches leave it -1 — the link gate only governs the mid-run
    // trigger, and pretending otherwise would put a number in the column for
    // decisions it never touched.
    e.link_down_sec    = dispatch_link_down_sec_;
  }
  exp_log_->logReconnectDispatch(expCtx(), e);
}

void ExploPlannerNode::logRunEnd(const char* reason) {
  if (!exp_log_) return;
  RunEndEvent e;
  e.reason     = reason;
  e.steps      = step_;
  e.distance_m = cumulative_distance_;
  // The last measured coverage rather than a fresh measurement: this runs on
  // the DONE transition and (via the destructor) during shutdown, where
  // map_cache_ may be mid-teardown and a whole-grid walk is the last thing to
  // start. fillCommonMetrics refreshes it at the sampling period, so it is at
  // most one metrics period old.
  e.unknown_fraction = last_unknown_fraction_;
  e.coverage_source  = last_coverage_source_;
  e.peers_live =
      coord_ ? static_cast<int>(coord_->livePeerCount(this->now())) : 0;
  e.expected_peers = rendezvous_expected_peers_;
  // Configured vs REALISED CSV sampling rate. The realised value needs two rows
  // to be a period at all; -1 says "not measurable", never a fabricated 0.
  e.metrics_period_param_sec     = metrics_period_sec_;
  e.metrics_effective_period_sec = metrics_effective_period_;
  e.metrics_rows                 = metrics_rows_written_;
  e.metrics_backoffs             = metrics_backoffs_;
  e.metrics_realised_period_sec =
      (metrics_rows_written_ >= 2)
          ? (metrics_last_row_sim_sec_ - metrics_first_row_sim_sec_) /
                static_cast<double>(metrics_rows_written_ - 1)
          : -1.0;
  exp_log_->logRunEnd(expCtx(), e);
}

ExploPlannerNode::~ExploPlannerNode() {
  // The campaign stops runs with SIGTERM (docker stop), which unwinds spin()
  // and destroys the node without ever reaching DONE — so without this the file
  // would end mid-stream and be indistinguishable from a truncated one. The
  // logger ignores a second run_end, so a node that DID reach DONE keeps its
  // real reason.
  //
  // Nothing here may throw: a destructor that escapes during shutdown
  // terminates the process and would lose the flush it was called to perform.
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

  // Exploitation columns. Left at the -1/0 defaults for exploration rows;
  // filled from the dwelled vantage for exploitation rows (m.phase itself is
  // set from phase_ in fillCommonMetrics).
  if (phase_ == Phase::EXPLOIT) {
    m.target_id         = pending_exploit_target_id_;
    m.vantage_index     = pending_exploit_vantage_index_;
    m.n_vantages_valid  = pending_exploit_n_valid_;
    m.vantage_los_clear = pending_exploit_los_clear_;
    m.dwell_sec         = pending_exploit_dwell_sec_;
  }

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
        "Step budget reached: %d steps, %.2fm traveled, %d final voxels.",
        step_, cumulative_distance_, m.total_observed_voxels);
    // Route through finishOrRendezvous, NOT straight to DONE. This is the
    // dominant termination path in dense terrain (the coverage threshold may
    // never be reached), and going directly to DONE meant a robot that spent
    // its step budget shut down wherever it happened to stop — never returning
    // to last_connected_anchor_ and never releasing its teammate's barrier.
    // The return drive happens after the budget is spent, so it costs no steps.
    //
    // On a DEFERRED decision go to PLAN, never stay here: doLogStep is
    // dispatched every tick and writes a metrics row and increments step_ on
    // each call, so idling in LOG_STEP would forge duplicate steps. PLAN
    // re-checks the same budget at its head and calls this again, where a
    // deferral costs nothing.
    if (!finishOrRendezvous("step-budget")) {
      transitionTo(State::PLAN, "finish-deferred");
    }
  } else {
    // Route by phase: exploitation steps loop back to the vantage planner,
    // exploration steps to the exploration planner.
    transitionTo(
        phase_ == Phase::EXPLOIT ? State::EXPLOIT_PLAN : State::PLAN,
        "step-logged");
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
    // Arm the fine band the moment the target enters the queue (release ==
    // exploitation-phase start for the whole team), not on activation: every
    // robot then fine-maps any released trunk its lidar reaches — including
    // the approach drive and a tree a peer is circling. Dedup'd re-reports
    // skip this (their region is already registered).
    if (region_pub_) {
      publishRefinementRegion(msg->target_id, center, msg->radius,
                              /*remove=*/false);
    }
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
  // Disarm the fine band for this trunk — integration stops, the fine voxels
  // already fused stay in the lattice for offline post-processing. Only on
  // DONE: a stand-down (deactivate) keeps the region live because the target
  // returns to PENDING and the phase is still exploitation.
  if (t && region_pub_) {
    publishRefinementRegion(t->id, t->center, t->radius, /*remove=*/true);
  }
  target_queue_.markActiveDone();
  have_active_intent_ = false;     // release the claim now the target is closed
  exploit_target_timing_ = false;  // next active target re-latches the timer
  current_is_approach_   = false;
  RCLCPP_INFO(get_logger(),
      "Target %u exploitation %s (%d/%d clear-LoS vantages dwelled).",
      id, success ? "COMPLETE" : "PARTIAL", clear, min_vantages_required_);

  if (target_queue_.hasPending()) {
    transitionTo(State::EXPLOIT_PLAN, "target-closed-next-pending");  // doExploitPlan activates the next one
  } else {
    phase_ = Phase::EXPLORE;
    RCLCPP_INFO(get_logger(),
        "Target queue empty -> reverting to EXPLORE.");
    transitionTo(State::PLAN, "target-queue-empty");
  }
}

// Fine-TSDF region relay (RefinementRegion is field-compatible with TreeTarget
// by design — see scovox_msgs/msg/RefinementRegion.msg). center.z forwards as
// base_z, the trunk base the scovox slab params offset from; the radius is
// swapped for the trunk-scale fine_region_radius_m unless that is <= 0.
void ExploPlannerNode::publishRefinementRegion(uint32_t id,
                                               const Eigen::Vector3f& center,
                                               float radius, bool remove) {
  scovox_msgs::msg::RefinementRegion m;
  m.id = id;
  m.x = center.x();
  m.y = center.y();
  m.base_z = center.z();
  m.radius = fine_region_radius_m_ > 0.0
                 ? static_cast<float>(fine_region_radius_m_)
                 : radius;
  m.remove = remove;
  region_pub_->publish(m);
  RCLCPP_INFO(get_logger(),
      "%s fine refinement region id=%u at (%.2f, %.2f) r=%.2f.",
      remove ? "Removed" : "Registered", id, m.x, m.y, m.radius);
}

// Called once, on the tick that latches shutdown_requested_ (State::DONE,
// done_action=shutdown). DONE-idle deliberately does NOT clean up: its
// regions stay armed because a target released while idling resumes the
// exploit sub-loop. Removes here are idempotent with the per-target remove
// in finishActiveTarget.
void ExploPlannerNode::removeLiveRefinementRegions() {
  if (!region_pub_) return;
  for (const auto& t : target_queue_.targets()) {
    if (t.status == Target::Status::DONE) continue;
    publishRefinementRegion(t.id, t.center, t.radius, /*remove=*/true);
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
    publishIntent();
    have_active_intent_ = true;
  }

  transitionTo(State::NAVIGATE, "exploit-goal-selected");

  // Initialise the smart-timeout state for this NAVIGATE cycle (same as doPlan).
  const float dx = current_goal_.position.x() - robot_pos.x();
  const float dy = current_goal_.position.y() - robot_pos.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  nav_budget_sec_ = navBudgetSec(dist, nav_speed_est_mps_, nav_safety_factor_,
                                 nav_min_timeout_sec_, nav_max_timeout_sec_);
  progress_check_time_ = state_enter_time_;
  progress_check_dist_ = cumulative_distance_;
}

bool ExploPlannerNode::vantageVetoedByPeers(uint32_t target_id,
                                            const Eigen::Vector3f& v_pos,
                                            const Eigen::Vector3f& robot_pos) {
  // Uses the small per-vantage disc, NOT the exploration-scale
  // coord_claim_radius_m — one fov-range disc would swallow the whole ring
  // and veto the tree outright instead of splitting the angles across the
  // team. Two claim rules, and which one applies is decided by the CLAIM
  // KIND, not by geometry; a third rule then contests the peers that are
  // parked on this ring with no claim on this angle yet. NOTE the default
  // (nullptr) liveness on claimMatching: exploit claims are read as RETAINED
  // — grace included — because a vantage under recent pursuit must stay
  // denied across a receive-side delivery gap (see Coordination's ctor doc
  // for the starved-executor incident that motivated the grace window).
  const auto* peer = coord_->claimMatching(
      v_pos, static_cast<float>(coord_vantage_claim_radius_m_));
  // An EXPLOIT claim on THIS trunk is not contested by distance at all: a
  // ring angle under active pursuit belongs to its claimant until the claim
  // lapses. Live-distance MinPos is what lost a ring — a robot standing at
  // the trunk having just finished one angle was, by construction, closer to
  // every remaining vantage than a peer 12 m out and 19 s into its drive, so
  // it took the angle the peer was already committed to, both converged on
  // one point, the proximity right-of-way braked the loser 8 times, and the
  // winner closed the 3/3 quota solo. The comparison was never meaningful
  // here: cost-to-go, not cost-so-far, is what a peer mid-hop has left.
  if (peer && peer->exploit && peer->target_id == target_id) return true;
  // Any other claim shape keeps the distance contest. An EXPLORATION claim
  // carries the fov-range disc (~8-10 m), wide enough to cover the entire
  // ring, so yielding to it unconditionally would veto every vantage of the
  // tree for as long as a peer explores anywhere near it.
  if (peer && !coord_->selfWinsAgainst(robot_pos, v_pos, *peer, robot_name_))
    return true;
  // Third rule, and it is not about claims on this angle at all — both
  // rules above can only see a vantage a peer has ALREADY claimed, and the
  // race that actually hurts happens BEFORE any claim exists. When the
  // dwell-sync barrier releases the team, every parked robot re-plans within
  // milliseconds of every other, and the 1 Hz claim exchange is blind for
  // that first instant: with one angle left, every free robot picks it.
  // Observed: two robots picked the same last vantage 3 ms apart, and the
  // loser's in-flight yield (doNavigate) came 83 ms too late to prevent the
  // drive — they collided.
  //
  // So contest the peers we KNOW are parked on this ring. A staged claim is
  // set at dwell entry and kept while its producer re-plans in place, i.e.
  // that peer is about to pick from a dead standstill beside the ring;
  // settle it now with the same total order the in-flight tiebreak would
  // use later, and if the parked peer strictly wins the angle simply do not
  // select it — no goal, no drive, no yield to walk back. Because staged
  // claims come only from stationary producers, its broadcast robot_pos
  // still matches reality, both sides evaluate the same antisymmetric
  // comparison, and exactly one robot admits the vantage without exchanging
  // a message.
  //
  // DRIVING peers are deliberately not contested by position here — their
  // claim discs above are their instrument. A mover's wire pose is a
  // heartbeat stale, and a robot still approaching from far away is farther
  // from EVERY angle on the ring, so position-contesting movers would freeze
  // it out of the whole tree and serialize exactly what the barrier exists
  // to parallelize. The doNavigate tiebreak remains the backstop for the
  // residual race between two robots that each believed they had won
  // (comms skew).
  return coord_->stagedExploitPeerWinning(
             target_id, v_pos, robot_pos, robot_name_, this->now()) != nullptr;
}

bool ExploPlannerNode::holdDwelledVantage(
    Target* tgt, const std::vector<CandidateViewpoint>& vantages,
    const Eigen::Vector3f& robot_pos) {
  // The requirement, from the operator watching the 2-robot sim live: "both
  // robot go to their vantage poses[,] the dwell time starts, then one of the
  // robots get the next vantage pose, it goes there then dwell time starts[,]
  // then after dwell time finishes the exploitation finishes" — and the robot
  // that does NOT get the last vantage "should have kept the first vantage
  // pose". What it did instead (run9) was thrash: every time the winner's
  // claim aged out of the table mid-delivery-gap, the parked loser re-selected
  // the winner's vantage, rolled toward it for ~0.6 s, yielded it back, and
  // braked ~0.15 m further off its own vantage — twice, plus 25 s of retry
  // log spam, ending 0.3 m off-pose with a random heading.
  //
  // So: once THIS robot has completed a dwell on this target and every other
  // angle of the ring is either team-visited or peer-denied (the SAME rules
  // the selection walk uses — vantageVetoedByPeers), there is nothing left
  // for it to contribute by driving. Park on the dwelled vantage, keep the
  // staged claim beating (a peer's dwell barrier reads it as "released"),
  // and let the tick's earlier stages — team-quota merge, per-target timeout
  // — end the hold. Exit paths, all above or upstream of this branch:
  //   * peer's mask merge completes the quota -> finishActiveTarget;
  //   * the winner dies -> its claim ages out (TTL + grace) -> the angle
  //     stops being vetoed -> this returns false and the normal walk selects;
  //   * per-target timeout -> PARTIAL.
  // This branch also runs BEFORE the fused-map refresh and flood, so a
  // holding robot's ticks drop from seconds to microseconds — which is what
  // lets the single-threaded executor actually deliver the peer's heartbeats
  // while we hold (the receive-side gap that started all of this).
  if (!held_vantage_valid_ || held_vantage_target_id_ != tgt->id) return false;

  for (const auto& v : vantages) {
    if (target_queue_.isVantageVisited(
            v.position, static_cast<float>(vantage_visited_tol_m_)))
      continue;
    // NOT visited: only a peer-denial keeps it off the table. Blacklist alone
    // is deliberately NOT enough to hold on — a blacklisted-but-unclaimed
    // angle means nobody is going there, and parking would leave the ring
    // permanently short; the normal walk's retry/approach/timeout machinery
    // owns that case.
    if (!vantageVetoedByPeers(tgt->id, v.position, robot_pos)) return false;
  }

  // Ring covered. Re-anchor if the yield-roll drifted us off the pose (the
  // brake goal in abandonNavGoal stops the platform where it happens to be,
  // which after a ~0.6 s roll is decimetres off the vantage): publish the
  // held vantage itself as the nav goal. This is a sub-metre reposition onto
  // our own angle — the contested angle is a third of the ring away — and
  // publishGoal alone is correct here: there is no in-flight hop to cancel
  // (the yield already abandoned it) and EXPLOIT_PLAN is re-entered every
  // tick, so arrival needs no state transition.
  {
    const float dx = robot_pos.x() - held_vantage_pose_.position.x();
    const float dy = robot_pos.y() - held_vantage_pose_.position.y();
    const float yaw_err = std::remainder(
        latest_yaw_ - held_vantage_pose_.yaw, 2.0f * static_cast<float>(M_PI));
    if (dx * dx + dy * dy > goal_xy_tol_ * goal_xy_tol_ ||
        std::abs(yaw_err) > goal_yaw_tol_) {
      publishGoal(held_vantage_pose_);
    }
  }

  // Keep the staged hold-claim beating. After a completed dwell the cached
  // intent already IS this claim (dwell entry set staged=true, the dwell-end
  // broadcast patched the mask, and LOG_STEP left it alone) — but after a
  // YIELD the intent was released (have_active_intent_=false, deliberately:
  // a kept claim would have been the CONCEDED angle with staged=false, which
  // would hold the winner's barrier open against us). Rebuild it here as
  // what we actually are: staged on our own dwelled vantage. The heartbeat
  // then re-publishes it at 1 Hz for as long as we hold.
  if (intent_pub_ && (!have_active_intent_ || !current_intent_msg_.staged ||
                      current_intent_msg_.target_id != tgt->id)) {
    current_intent_msg_ = coord_->buildIntent(
        held_vantage_pose_, robot_pos, this->now(),
        static_cast<float>(coord_claim_ttl_sec_),
        static_cast<float>(coord_vantage_claim_radius_m_),
        /*planner_type_id (eig)=*/0u, map_frame_,
        /*exploit=*/true, tgt->id, tgt->clear_mask, /*staged=*/true);
    publishIntent();
    have_active_intent_ = true;
  } else if (have_active_intent_) {
    // Team credit can grow while we hold (the winner's dwell lands in our
    // clear_mask via the merge above this branch); keep the broadcast mask
    // current so OUR heartbeat also carries the newest union.
    current_intent_msg_.dwelled_mask = tgt->clear_mask;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Target %u: holding vantage %d (staged) — ring covered by team; "
      "waiting for quota (%d/%d clear-LoS dwells).",
      tgt->id, held_vantage_index_, tgt->clear_los_dwells,
      min_vantages_required_);
  return true;
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
      transitionTo(State::PLAN, "exploit-queue-drained");
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

  // Hold branch: with a dwell of our own banked and every other ring angle
  // team-visited or peer-denied, PARK on the dwelled vantage instead of
  // running the full selection below (see holdDwelledVantage for the run9
  // thrash this replaces). Placed above the map refresh + flood on purpose:
  // a holding robot's tick must cost microseconds, not seconds, or the
  // executor starves the very intent subscription whose claims the hold
  // decision reads. Quota merge and the per-target timeout stay ABOVE this,
  // so a hold can always end.
  if (coord_ && coord_->enabled() &&
      holdDwelledVantage(tgt, vantages, latest_pos_)) {
    return;
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
  visited_goals_.prune(plan_start.seconds(), visited_goal_ttl_sec_);

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
        exploit_flood_stamp_ != latest_plan_map_->header.stamp ||
        (robot_pos - exploit_flood_pos_).head<2>().norm() > kFloodRefreshM;
    if (stale) {
      cost_grid_->build(*latest_plan_map_);
      cost_grid_->floodFrom(robot_pos, /*radius_cap_m (unbounded)=*/0.0f);
      exploit_flood_valid_   = true;
      exploit_flood_map_     = latest_plan_map_.get();
      exploit_flood_stamp_   = latest_plan_map_->header.stamp;
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
    // Per-vantage deconfliction: yield an angle a peer currently claims
    // (in-flight or mid-dwell) or that a parked peer is about to win. The
    // three rules live in vantageVetoedByPeers() — the hold branch above this
    // walk consults the same implementation, which is what keeps "nothing
    // selectable, park on my own vantage" and "this vantage is selectable"
    // mutually exclusive by construction.
    if (coord_ && coord_->enabled() &&
        vantageVetoedByPeers(tgt->id, v.position, robot_pos)) {
      ++rej_minpos;  // all three rules fold here: the CSV schema is unchanged
      continue;
    }
    if (cost < best_cost) {
      best_cost = cost;
      best_idx  = static_cast<int>(i);
    }
  }

  if (best_idx < 0) {
    // Throttled: this branch re-runs at the 10 Hz tick rate while nothing is
    // selectable, and an unthrottled print turned a 25 s hold into 200
    // identical lines that buried the two log lines that actually explained
    // the round (run9). The rejection counters still tell the whole story
    // once per window.
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
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
      pending_info_gain_std_           = 0.0f;
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
  pending_info_gain_std_           = 0.0f;
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
    // Carry the dwell-sync wait clock back with it. That one is anchored ONCE at
    // state entry and never re-anchored per tick, so a rewind leaves it in the
    // future: the wait reads negative, stays below max_wait forever, and the
    // barrier holds the robot at the vantage for the rest of the run.
    dwell_sync_wait_start_ = state_enter_time_;
    return;
  }

  // Vantage-ring rendezvous barrier. Hold here — dwell clock NOT started — while
  // any peer holds an exploit claim on this trunk and has not yet declared
  // itself staged on the vantage it claimed. The ring exists to capture ONE
  // trunk state from several angles at once; a robot that arrives first and
  // dwells alone spends the team's 3/3 quota on sequential single-robot views,
  // and the peer that arrives after the target closed dwells an angle nobody
  // needs.
  //
  // "Staged" is PRODUCER-DECLARED (each robot sets the flag on its own claim at
  // dwell entry, i.e. after arrival AND the post-arrival rotation settle) rather
  // than inferred here from the claim's robot_pos against its goal_pos. Only the
  // producer knows whether the exploit hop it is holding is a vantage at all (an
  // approach waypoint is not) and whether it has stopped turning: the geometric
  // test read a peer that had arrived in XY but was still rotating as staged
  // ~5 s early, and the team's overlap collapsed to ~3 s of an 8 s dwell.
  //
  // The barrier is a START condition, latched by dwell_sync_started_ once
  // satisfied. It must not be re-tested per tick: peers go staged=false again
  // the moment they leave for their next angle, and re-testing then wiped the
  // accumulated dwell of a robot that had been motionless all along (see the
  // member's declaration). Started dwells therefore RUN TO COMPLETION.
  //
  // The wait mechanism is a per-tick re-anchor of state_enter_time_, not a
  // separate hold state: `elapsed` below therefore still measures real
  // motionless seconds at the vantage from the moment the team was staged, so
  // the CSV dwell_sec stays an honest capture length rather than wait time plus
  // capture. Release paths (unchanged): every claiming peer staged, no peer
  // claims this trunk, a peer's claim ages out (firstUnstagedExploitPeer filters
  // expiry itself — prune() runs in the PLAN ticks, which do not happen while we
  // sit here), or max_wait below. The 1 Hz heartbeat republish of each claim is
  // what carries a peer's staging to us; it bounds the residual asymmetry to
  // one heartbeat plus the DDS hop, and the immediate publish at dwell entry
  // (transitionTo) removes even that for the common case.
  if (exploit_dwell_sync_enabled_ && coord_ && coord_->enabled() &&
      !dwell_sync_started_ && !dwell_sync_timed_out_) {
    const Target* synced_target = target_queue_.active();
    if (synced_target) {
      const auto now = this->now();
      const auto* waiting_on =
          coord_->firstUnstagedExploitPeer(synced_target->id, now);
      if (!waiting_on) {
        // Barrier satisfied -> latch the start for the rest of this dwell.
        //
        // Logged only when we actually held for somebody. The probe returns
        // nullptr for two different situations — every same-target peer staged,
        // and no peer claiming this trunk at all (the solo capture, which is the
        // single-robot-equivalent case and must not produce a "team staged" line
        // per vantage) — and the Coordination API exposes no per-target peer
        // claim count to separate them. The hold branch's re-anchor IS that
        // record: state_enter_time_ sits past the entry-time wait clock iff this
        // barrier held for at least one tick. A peer that was already staged when
        // we arrived consequently starts its dwell silently too.
        if (state_enter_time_ > dwell_sync_wait_start_) {
          RCLCPP_INFO(get_logger(),
              "dwell-sync: team staged on target %u — dwell runs to "
              "completion.", synced_target->id);
        }
        dwell_sync_started_ = true;
      } else {
        const double waited = (now - dwell_sync_wait_start_).seconds();
        // max_wait <= 0 means "wait until the peer actually arrives", which is
        // the default. A wall-clock bound cannot tell a teammate that is merely
        // FAR from one that is wedged: the ring of the next trunk can be 30 m
        // away across the plot, and a peer that is still driving toward its own
        // angle was abandoned at 60 s while it needed 111 s — the capture went
        // solo for no reason. Waiting indefinitely does not hang the run,
        // because a robot that is waiting is by definition standing on its
        // vantage with staged=true already on the wire (published at dwell
        // entry), so it never holds a peer's barrier: only a DRIVING peer
        // holds, and that peer is bounded by its own timers, which do tick
        // while it drives. It arrives, or it goes silent and its claim ages out
        // of the table on the receipt-time TTL, or its own per-target give-up
        // closes the trunk and it stops claiming it. All three release us.
        const bool wait_bounded = exploit_dwell_sync_max_wait_sec_ > 0.0;
        if (!wait_bounded || waited < exploit_dwell_sync_max_wait_sec_) {
          state_enter_time_ = now;
          if (wait_bounded) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "dwell-sync: holding at vantage %d, waiting for '%s' to stage "
                "on target %u (%.0fs / %.0fs).",
                current_vantage_index_, waiting_on->robot_id.c_str(),
                synced_target->id, waited, exploit_dwell_sync_max_wait_sec_);
          } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "dwell-sync: holding at vantage %d, waiting for '%s' to stage "
                "on target %u (%.0fs, no deadline — until it arrives).",
                current_vantage_index_, waiting_on->robot_id.c_str(),
                synced_target->id, waited);
          }
          return;
        }
        // Give-up bound: a peer wedged in a proximity hold or cycling
        // EXPLOIT_PLAN with no selectable vantage must not cost this robot its
        // capture. Latched for the rest of THIS dwell — re-testing would
        // re-anchor the dwell clock on the next tick and the dwell would never
        // complete. transitionTo clears the latch on the next EXPLOIT_DWELL
        // entry, so one timed-out barrier does not disable the feature.
        dwell_sync_timed_out_ = true;
        RCLCPP_WARN(get_logger(),
            "dwell-sync: barrier timed out after %.0fs waiting for '%s' on "
            "target %u (limit %.0fs) — DWELLING SOLO. This capture is not "
            "simultaneous with the team's; check that peer for a proximity hold "
            "or an unreachable ring.",
            waited, waiting_on->robot_id.c_str(), synced_target->id,
            exploit_dwell_sync_max_wait_sec_);
      }
    }
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
  //
  // Settled XY, SIGHTLINE z. latest_pos_.z() is base_link — 0.09-0.13 m on a
  // UGV, and it bobs by more than a voxel as the platform settles — so marching
  // the ray at it samples the voxel row the mapped ground surface occupies and
  // reports BLOCKED for the ray's whole length, on a trunk in the open, at
  // random depending on where the suspension came to rest. The measurement
  // height is ground + candidate_z_clearance: exactly what generateVantages()
  // used at selection time, and what exploitZAt() is for (see its contract
  // above — "a vantage's z IS the measurement"). Ground unknown under the
  // settled pose => keep the selection-time verdict rather than march the ray
  // outside the observed volume, where it would pass trivially.
  bool los_clear = current_vantage_los_clear_;
  Target* t = target_queue_.active();
  if (t && loadLatestMap()) {
    const float sight_z = exploitZAt(latest_pos_.x(), latest_pos_.y());
    if (std::isfinite(sight_z)) {
      const Eigen::Vector3f from(latest_pos_.x(), latest_pos_.y(), sight_z);
      los_clear = vantage_planner_->lineOfSightClear(
          from, t->center, t->radius, *map_cache_);
    }
  }
  current_vantage_los_clear_ = los_clear;
  pending_exploit_los_clear_ = los_clear ? 1 : 0;

  // Record it on the active target and stage the dwell time for LOG_STEP.
  // The ring index makes the credit shareable: peers merge it by index into
  // their own copy of this target (team quota).
  target_queue_.recordVantageDwell(current_goal_.position, los_clear,
                                   current_vantage_index_);
  pending_exploit_dwell_sec_ = static_cast<float>(elapsed);

  // Remember the exact pose this dwell was captured from. If the rest of the
  // ring ends up covered by the team, doExploitPlan's hold branch parks the
  // robot back on precisely this position AND yaw — the operator requirement
  // is that the robot which does not get the last vantage keeps its vantage
  // pose, not "stops somewhere near it".
  if (t) {
    held_vantage_pose_      = current_goal_;
    held_vantage_index_     = current_vantage_index_;
    held_vantage_target_id_ = t->id;
    held_vantage_valid_     = true;
  }

  // Broadcast the updated team-credit mask immediately (don't wait for the
  // next selection or heartbeat) so a peer picking its next angle right now
  // already sees this dwell — and so the credit lands before this robot
  // could release the claim on target completion.
  //
  // Patch the cached message in place; do NOT rebuild it through buildIntent().
  // Rebuilding would reset `staged` to false while the robot is still standing
  // on the vantage, which is exactly wrong for the last round of a ring: a robot
  // with no selectable vantage left holds here, and the heartbeat republish of
  // THIS message (staged, at the vantage) is what keeps a peer's barrier
  // released while that peer takes the final angle.
  if (intent_pub_ && coord_ && have_active_intent_ && t) {
    current_intent_msg_.dwelled_mask = t->clear_mask;
    current_intent_msg_.header.stamp = this->now();
    publishIntent();
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
  transitionTo(State::LOG_STEP, "dwell-complete");
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
  // A successful lookup is NOT a fresh pose. TimePointZero returns the newest
  // stored transform unconditionally, and tf2 prunes only on INSERT — once a
  // broadcaster dies this lookup keeps succeeding with the same stamp
  // forever. Everything downstream trusts latest_pos_ (goals, the brake goal,
  // distance, the trajectory log), so a stale transform must read as "pose
  // lost", exactly like a failed lookup, until fresh data arrives.
  if (pose_max_age_sec_ > 0.0) {
    const double age = (this->now() - rclcpp::Time(tf.header.stamp)).seconds();
    if (age > pose_max_age_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TF %s -> %s is %.1f s old (pose_max_age_sec=%.1f) — treating the "
          "pose as lost until the transform updates.",
          map_frame_.c_str(), base_frame_.c_str(), age, pose_max_age_sec_);
      have_pose_ = false;
      return;
    }
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
