/// @file explo_planner_node.cpp
/// @brief Standalone SCovox Beta EIG exploration planner node — self-contained.
///
/// This is the EIG-only planner that the exploration/exploitation system is
/// built on top of. It began as a fork of a multi-planner comparison node
/// (eig/entropy/frontier/random/ssmi) that NO LONGER EXISTS in this workspace:
/// do not go looking for exploration_planner_node.hpp/.cpp, it is gone, and
/// this is now the only planner node. What survives from that fork is the
/// shape of this file — hard-wired to the SCovox Beta expected-information-gain
/// scorer and owning its own copy of the state machine, which is why there is
/// no planner_type knob anywhere below (see the note at the class declaration).
/// The reusable pieces (scoring, candidate generation, FOV evaluation, cost
/// grid, coordination, map cache, metrics) are shared via explo_planner_lib.
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
///
/// Design rationale and history for this file are in
/// doc/explo_planner_node_notes.md. A comment ending "(notes: <id>)" has
/// a section headed <id> there.

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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <scovox_msgs/msg/scovox_map.hpp>
#include <scovox_msgs/msg/scovox_fusion_counters.hpp>
#include <scovox_msgs/msg/refinement_region.hpp>
#include <explo_planner_msgs/msg/robot_intent.hpp>
#include <explo_planner_msgs/msg/team_world.hpp>
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
#include "explo_planner/fleet_identity.hpp"
#include "explo_planner/cell_world.hpp"
#include "explo_planner/team_model.hpp"
#include "explo_planner/exchange_drain.hpp"
#include "explo_planner/meeting_attendance.hpp"
#include "explo_planner/separation.hpp"
#include "explo_planner/global_allocator.hpp"
#include "explo_planner/reconnect_gate.hpp"
#include "explo_planner/pursuit_predictor.hpp"
#include "explo_planner/rendezvous_scheduler.hpp"
#include "explo_planner/home_trail.hpp"
#include "explo_planner/proximity_guard.hpp"
#include "explo_planner/target_queue.hpp"
#include "explo_planner/vantage_planner.hpp"

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
  // Exploitation sub-states. EXPLOIT_PLAN selects the next vantage around the
  // active target; reaching it routes to EXPLOIT_DWELL (NAVIGATE is shared with
  // exploration and branches on phase_).
  EXPLOIT_PLAN,
  EXPLOIT_DWELL,
  // Rendezvous sub-states (multi-robot). On exhausting its exploration goals a
  // robot drives back to its last-connected anchor (RETURN_NAV) and holds there
  // (RETURN_SYNC) until the whole team is back in comms, then re-plans against
  // the merged map. Gated by reconnect_enabled_; see the rendezvous_* params.
  RETURN_NAV,
  RETURN_SYNC,
  // Chase the missing peer's last declared goal or heard pose on a
  // staleness-scaled budget, then pursuitExploreFallback, then holdForTeam.
  // Same ladder in both arms; hybrid's appointment pre-empts the chase from
  // doPursue. (notes: state-pursue-reconnect-ladder)
  PURSUE,
  // A driving robot that lost right-of-way to a nearby moving teammate cancels
  // its goal and parks until the peer clears or parks, then resumes the same
  // goal. Entered only from NAVIGATE / RETURN_NAV / PURSUE / RETURN_HOME.
  // (notes: state-proximity-hold-entry)
  PROXIMITY_HOLD,
  // Any terminal exploration ending drives back to the recorded start pose
  // (mission_return_enabled). exploration_complete is stamped before entry;
  // resolves only into DONE (arrival, give-up or mission_return_max_sec).
  // (notes: state-return-home-mission-return)
  RETURN_HOME
};

// Wire/CSV state names consumed by offline analysis: do not rename one. No
// default case, so a State without a name here is a compile warning.
// (notes: statename-wire-interface)
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
    case State::RETURN_HOME:    return "RETURN_HOME";
  }
  return "UNKNOWN";
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
  /// Emit reconnect_dispatch for the chosen manoeuvre. Call only from the leaf
  /// that commits the action, never from a branch that contemplates it. dest is
  /// null for actions with no destination.
  /// (notes: reconnect-dispatch-log-at-commit)
  void logReconnectDispatch(const char* action, const Eigen::Vector3f* dest,
                            double budget_sec, const char* reason);
  /// Re-read the missing-peer context (id and record age) from the CURRENT
  /// contact table, for leaves that commit out of band from
  /// dispatchReconnect's branch walk and would otherwise report the age that
  /// walk saw — stale by a whole barrier wait or a whole chase budget.
  void refreshDispatchContext();
  /// Emit run_end exactly once (later calls are ignored by the logger).
  void logRunEnd(const char* reason);

  /// At exhaustion, decide DONE or the reconnect_mode_ manoeuvre; every
  /// termination path must route through here. Returns false when the
  /// confirmation gate deferred, and the caller must then re-run this check.
  /// (notes: finish-or-rendezvous-routing)
  bool finishOrRendezvous(const char* reason);
  /// Emit exploration_complete at the instant this robot declares its
  /// exploration exhausted, before anything is decided about what follows.
  /// Shared so the latch criterion records the same event.
  /// (notes: exploration-complete-event)
  void recordExplorationComplete(const char* reason);
  /// The ending itself: keep beaconing if DONE-idle, then transition. Shared by
  /// both criteria so there is exactly one place a run can end.
  bool finishNow(const char* reason);
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
  /// Dispatch the reconnect manoeuvre (pursuit / meeting point / anchor / hold)
  /// without the DONE fallthrough, for finishOrRendezvous and doPlan's mid-run
  /// trigger. Always transitions and returns true.
  /// (notes: dispatch-reconnect-factoring)
  bool dispatchReconnect(const char* reason);

  // ---- Scheduled rendezvous (P5, §3.5) --------------------------------
  /// The exchanged (cell, interval) pair. Declared here only so the handshake
  /// methods below can name it; it is DEFINED with the rest of the P5 state,
  /// where the comment explaining why the appointment travels on the wire
  /// instead of being derived twice belongs.
  struct RendezvousProposal;

  /// True when every peer is in mutual direct contact (TeamModel::Peer::direct:
  /// heard inside the TTL and its in_range_mask names us). Stricter than
  /// teamComplete on purpose, so both robots stop on the same event.
  /// (notes: rendezvous-team-mutual-gate)
  bool rendezvousTeamMutual() const;

  /// True if any peer received first-hand (direct || heard_one_way) announces
  /// team_incomplete: how a robot learns two peers cannot hear each other. Do
  /// not narrow to direct; that drops the one-way robot.
  /// (notes: peer-reports-team-break-contagion)
  bool peerReportsTeamBreak() const;

  /// True if any first-hand peer (direct || heard_one_way) is still driving to
  /// the agreed cell (appointment_inbound), so the barrier does not release
  /// before it arrives. No finished exemption, deliberately.
  /// (notes: peer-inbound-to-appointment)
  bool peerInboundToAppointment() const;

  /// True if any first-hand peer reports a still-driving robot on its own
  /// horizon (appointment_inbound_seen): the witness for peers the release door
  /// admits through a bridge. Same no-TTL, no-finished-exemption rules.
  /// (notes: peer-reports-inbound-one-hop)
  bool peerReportsInboundToAppointment() const;

  /// Peers accounted for now, the liveness number decisions run on: unexpired
  /// coordination claim OR TeamWorld direct link OR announced finished
  /// (relayed, no TTL). Also written as peers_live and coord_active_peers.
  /// (notes: accounted-peer-count-channels)
  int accountedPeerCount(const rclcpp::Time& now) const;

  /// accountedPeerCount for one peer, named as the claim table names it. Every
  /// single-teammate question goes through this so team-whole and peer-back
  /// share one predicate. (notes: peer-accounted-single-predicate)
  bool peerAccounted(const std::string& name, const rclcpp::Time& now) const;

  /// Peers reachable over the comms closure now (TeamModel::inComms) or
  /// finished; 0 before TeamModel is configured. Only the appointment release
  /// door in manoeuvreReleaseEligible reads it; the arm must never.
  /// (notes: reachable-peer-count-closure)
  int reachablePeerCount() const;

  /// Team predicate P: this robot hears everyone and nobody it hears says
  /// otherwise. Arm on !P and end on P at all five sites; the only sanctioned
  /// asymmetry is the barrier release's reachable door.
  /// (notes: team-settled-predicate)
  bool teamSettled(int live_peers) const;

  /// Appointment: (teamSettled or every expected peer reachable) and nobody
  /// still driving in; otherwise plain teamComplete. Reads
  /// appointment_manoeuvre_, which transitionTo() clears after its classifiers
  /// run. (notes: manoeuvre-release-eligible-split)
  bool manoeuvreReleaseEligible(int live_peers) const;

  /// True while a finished, not-leaving, unheard peer may still be coming and
  /// the wait since finished_peer_wait_start_sec_ is under
  /// rendezvous_latched_hold_sec. A veto inside manoeuvreReleaseEligible's
  /// appointment branch. (notes: holding-for-finished-peer)
  bool holdingForFinishedPeer() const;

  /// Freeze the world the next appointment is derived from. Called on the
  /// heartbeat while the team reads complete, so it holds the last
  /// bidirectionally confirmed state. No-op unless the schedule is enabled.
  /// (notes: rendezvous-snapshot-freeze)
  void refreshRendezvousSnapshot();
  /// One round of the propose/echo/commit handshake, called unconditionally
  /// from the heartbeat. Only DERIVE (proposer) requires team_mutual; ECHO and
  /// COMMIT do not. Gating ECHO on it deadlocks at N>=3.
  /// (notes: rendezvous-proposal-handshake-gates)
  void maintainRendezvousProposal(bool team_mutual);
  /// Proposer only: solve over the frozen snapshot and return the (cell,
  /// interval, t_meet) triple, invalid if no solve is possible. The plan lands
  /// in rendezvous_held_provenance_.plan; candidates <= 1 marks the
  /// placeholder. (notes: derive-rendezvous-proposal)
  RendezvousProposal deriveRendezvousProposal();
  /// Turn the committed triple into a standing appointment: cell verbatim,
  /// t_meet_ms the first agreed occurrence this robot can reach within
  /// rendezvous_max_lateness_sec. One rendezvous_agreed per call; does not
  /// solve. (notes: arm-appointment-agreed-occurrence)
  bool armAppointment(const char* reason);
  /// This robot's drive time to cell from here, or -1 with no usable estimate.
  /// Optimistic (a straight line where no route exists, no nav overhead), so
  /// decisions use appointmentLeadMs instead.
  /// (notes: travel-ms-to-cell-optimistic)
  long long travelMsToCell(int cell) const;
  /// travelMsToCell marked up by rzv_cfg_.depart_safety_milli, or -1 when there
  /// is no estimate. ONE function because the rung choice at arming and the
  /// departure trigger must price the same drive the same way — if they drift,
  /// a robot signs up to a rung on one arithmetic and leaves for it on another.
  long long appointmentLeadMs(int cell) const;
  /// Live travel time from here to the standing appointment; -1 with no
  /// appointment or usable cell. Logged as travel_sec; the departure rule reads
  /// appointmentLeadMs, not this. (notes: appointment-travel-covariate)
  long long appointmentTravelMs() const;
  /// Departure test: now + appointmentLeadMs >= t_meet_ms (bare now >=
  /// t_meet_ms with no estimate), guarded by the armed flag. The lead is live,
  /// so each robot leaves on its own and arrivals coincide.
  /// (notes: appointment-due-departure)
  bool appointmentDue();
  /// Centre of the appointment cell, as a drive destination. Not const: it
  /// latches appointment_unplaceable_ on the branch where no grid can place
  /// the cell, which is the only place that fact is observable.
  Eigen::Vector3f appointmentPoint();
  /// Emit the appointment's single rendezvous_outcome row and disarm it; no-op
  /// when nothing is armed. Does not release rendezvous_spent_. mutual is
  /// whole-team two-way contact, distinct from outcome.
  /// (notes: close-appointment-outcome)
  void closeAppointment(const char* outcome, bool arrived, double waited_sec,
                        bool mutual);

  /// Flicker guard for manoeuvre release: true once `eligible` has held
  /// continuously for reconnect_release_confirm_sec (immediately when the
  /// window is 0). Resets whenever eligible drops or the state changes.
  bool releaseConfirmed(bool eligible);
  /// Non-mutating twin of releaseConfirmed, for callers that need the
  /// barrier's answer without advancing its dwell. Keep the two in step.
  bool releaseHeld(bool eligible) const;
  void startReturnTo(const Eigen::Vector3f& dest, const char* what,
                     const char* reason);
  void doReturnNav();
  /// Shared by doReturnNav's budget and no-progress watchdogs for an
  /// appointment leg stopped too far from the cell. True means it took the leg
  /// over (escape or roll back to PLAN) and the caller must not transition.
  /// (notes: appointment-leg-watchdog)
  bool appointmentLegWatchdog(const char* what_failed, float dist);
  /// End an escape leg and aim at the agreed cell again, with a fresh budget
  /// and a fresh no-progress window. The other half of the escape rung above;
  /// `why` is what ended the leg, for the log.
  void resumeAppointmentDrive(const char* why);
  /// Size the nav budget and open the no-progress window for a leg of dist
  /// metres, at departure and on every escape resume. Reads state_enter_time_
  /// as the leg start; stamp it first when not entering a state.
  /// (notes: manoeuvre-leg-budget)
  void armManoeuvreLegBudget(float dist);
  /// Write one appointment_leg row for the rung just taken. leg_sec and
  /// rolled_to_sec default to -1.0 (not this kind of row). Call before any
  /// transition, which can close the cell id and manoeuvre.
  /// (notes: appointment-leg-row)
  void logAppointmentLegRow(const char* kind, const char* cause, float dist,
                            double leg_sec = -1.0,
                            double rolled_to_sec = -1.0);
  void doReturnSync();
  /// Keep a standing appointment instead of ending the run. On true the caller
  /// must NOT end the run; doReturnSync owns the ending. Called from the
  /// coverage latch and finishOrRendezvous, ahead of mission return.
  /// (notes: keep-appointment-on-finish)
  bool keepAppointmentOnFinish(const char* reason);
  // startReturnHome enters RETURN_HOME without publishing (a same-tick goal can
  // be swallowed by the in-flight cancel); doReturnHome publishes once
  // home_pub_not_before_ passes and resolves into DONE via finishMissionReturn.
  // (notes: return-home-deferred-publish)
  bool startReturnHome(const char* reason);
  void doReturnHome();
  bool finishMissionReturn(const char* result, const char* end_reason);
  // homeApproachMetric: straight-line in DIRECT, remaining trail in RETRACE.
  // homeWatchdogFire is the only place a homing watchdog decision is made.
  // HomeMode is declared here because these signatures need it.
  // (notes: home-watchdog-helpers)
  enum class HomeMode { DIRECT, RETRACE, ESCAPE };
  float homeApproachMetric() const;
  static const char* homeModeName(HomeMode m);
  /// test_delta is the left-hand side the firing detector evaluated (window
  /// movement for frozen, closing distance for approach); not interchangeable
  /// with metric. The threshold is re-derived from kind.
  /// (notes: home-watchdog-test-delta)
  void homeWatchdogFire(const char* kind, float metric, float dist_home,
                        float test_delta);
  bool engageRetrace();
  bool pickEscapeTarget();
  void startEscapeLeg();
  void resumeRetrace(const char* why);
  void republishHomeGoal(const char* why);
  // startPursuit arms the chase (false when pursuitBudgetSec() == 0); doPursue
  // drives it under budget; pursuitFallback routes a spent chase; holdForTeam
  // raises RETURN_SYNC at the current pose. (notes: pursuit-helpers-overview)
  struct LastContact;  // defined with the members below
  bool startPursuit(const std::string& peer_id, const LastContact& rec,
                    const char* reason);
  /// Where `peer_id` will be when this robot could get there, from the tour it
  /// last broadcast (P6, §3.7). An invalid/refused target means "chase the
  /// trail", which is every case the predictor is not switched on for and
  /// every case it is switched on for and cannot serve.
  PursuitTarget predictIntercept(const std::string& peer_id,
                                 double* tour_age_sec = nullptr) const;
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
  // Published at every barrier hold and at DONE-idle entry so a parked robot
  // stays countable by teammates. (notes: presence-intent-beacon)
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
  // The vantage filter's three peer-claim rules, shared by doExploitPlan's
  // candidate walk and hold branch so both use one implementation. True if a
  // peer claim denies v_pos. Caller gates on coord_->enabled().
  // (notes: vantage-peer-veto-rules)
  bool vantageVetoedByPeers(uint32_t target_id, const Eigen::Vector3f& v_pos,
                            const Eigen::Vector3f& robot_pos);
  // True when this robot should park on its dwelled vantage (held_vantage_*)
  // because every other ring angle is team-visited or peer-denied; publishes
  // the hold claim or re-anchor goal. Caller returns from the tick on true.
  // (notes: hold-dwelled-vantage)
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
  // Nearest reachable, free, in-ROI point on the line toward center, so the
  // planner can approach a target whose ring is unreachable. False if none is
  // meaningfully closer. Requires a flooded cost_grid_.
  // (notes: exploit-approach-goal)
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

  /// Broadcast this robot's cell census, comms mask and gossip. Called only
  /// from the team_world_hz_ timer; do not piggyback it on the planning tick,
  /// or peers read a busy planner as silent.
  /// (notes: team-world-publish-timer-only)
  void publishTeamWorld();
  /// Drain team_world_pending_ into the cell world and comms model, then
  /// advance the comms clock once. Once per tick, not per message, so every
  /// consumer sees one closure per tick.
  /// (notes: drain-team-world-once-per-tick)
  void drainTeamWorld();

  /// Standing/sightline height for an exploitation point at (x, y): the fixed
  /// absolute vantage height in flat mode, local ground + candidate clearance
  /// in terrain mode. (notes: exploit-z-terrain-mode)
  float exploitZAt(float x, float y) const;

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
  // team_world_hz_ is the TeamWorld switch: 0 = no publisher, subscription,
  // timer, comms model or event. Requires the cell world and a configured fleet
  // identity; both are checked at startup and fatal.
  // (notes: team-world-hz-switch)
  double     team_world_hz_ = 0.0;
  TeamModel  team_model_;

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

  /// Allocation vehicle set at (x, y): self plus team-model peers, shared by
  /// allocator, reconnect gate and rendezvous snapshot. now_sec is
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

  // false = reconnect_gate silence: the mid-run silence clock alone decides.
  // true = info: the knowledge + value gate on top, which can only suppress a
  // dispatch the clock allowed, never bring one forward.
  // (notes: reconnect-gate-info-flag)
  bool       reconnect_gate_info_ = false;

  // ---- Scheduled rendezvous (P5, §3.5) --------------------------------
  // Off by default, and off must leave dispatchReconnect byte-identical to the
  // pre-P5 node. On, it replaces only where and when a RENDEZVOUS or HYBRID
  // manoeuvre goes; PURSUIT is untouched. (notes: p5-schedule-scope)
  bool       rendezvous_schedule_enable_ = false;
  RendezvousScheduler::Config rzv_cfg_;

  // The frozen problem, copied not referenced, refreshed only while the team
  // reads complete (age logged as snapshot_age_sec). The proposer solves over
  // it; every robot reads it for the cell centre and its own travel.
  // (notes: rendezvous-frozen-snapshot)
  CellWorld               rendezvous_world_;
  std::vector<AllocRobot> rendezvous_vehicles_;
  bool                    have_rendezvous_snapshot_ = false;
  double                  rendezvous_snapshot_at_sec_ = -1.0;

  // ---- The agreed proposal (the wire protocol above) -------------------
  // A (cell, interval) pair compared as exact integers; interval_ms is measured
  // from the separation, not a clock origin, so it can be echoed verbatim.
  // (notes: p5-agreed-proposal)
  struct RendezvousProposal {
    int       cell        = -1;
    long long interval_ms = -1;
    /// The meeting instant in mission-elapsed ms on the proposer's clock,
    /// echoed verbatim like cell and interval_ms. Mission-elapsed, not an
    /// absolute stamp, because field clocks drift.
    /// (notes: proposal-t-meet-instant)
    long long t_meet_ms   = -1;
    /// A zero interval is not a schedule, so valid() requires interval_ms > 0;
    /// armAppointment relies on a positive committed interval. Do not relax it
    /// to >= 0. (notes: proposal-zero-interval-invalid)
    bool valid() const {
      return cell >= 0 && interval_ms > 0 && t_meet_ms >= 0;
    }
    /// ALL THREE INTEGERS. This is the commit comparison — a robot holds the
    /// pair only when every live peer is publishing exactly this — so a field
    /// left out here is a field two robots may silently disagree on while the
    /// protocol reports agreement.
    bool operator==(const RendezvousProposal& o) const {
      return cell == o.cell && interval_ms == o.interval_ms &&
             t_meet_ms == o.t_meet_ms;
    }
    bool operator!=(const RendezvousProposal& o) const { return !(*this == o); }
  };

  // The proposer is fleet id 0, a constant, not an election: a lowest-live-id
  // scan can answer differently on two robots. The config block refuses to
  // start unless team-complete means the whole fleet.
  // (notes: rendezvous-proposer-constant)
  static constexpr int kRendezvousProposerId = 0;

  // Proposer's derive retry period before the team has ever agreed a pair, fine
  // enough to land in the opening co-located window. Deliberately not a
  // parameter. (notes: rendezvous-bootstrap-period)
  static constexpr double kRendezvousBootstrapPeriodSec = 5.0;

  // How long a gathered team stands on the agreed cell waiting for the next
  // pair to commit before leaving on the one it has. Bounded at six bootstrap
  // periods; deliberately not a parameter. (notes: rendezvous-reagree-wait)
  static constexpr double kRendezvousReagreeWaitSec =
      6.0 * kRendezvousBootstrapPeriodSec;

  // What this robot publishes: its own derivation on the proposer, the
  // proposer's pair verbatim (the echo) elsewhere. Not what it drives; driving
  // before peers echo it back is the race the commit removes.
  // (notes: rendezvous-held-published-pair)
  RendezvousProposal rendezvous_held_;

  // Diagnostics for one pair, copied at the commit site so they always describe
  // the appointment the row logs. Proposer only; a follower's copy stays at the
  // -1 sentinels. (notes: rendezvous-provenance-with-pair)
  struct RendezvousProvenance {
    RendezvousPlan plan;
    // Hashes of the frozen snapshot the argmin actually ran over, and when it
    // ran; not the robot's latest snapshot, which refreshes more often than the
    // argmin runs. (notes: provenance-snapshot-hashes)
    unsigned int shared_hash     = 0;
    unsigned int grid_hash       = 0;
    double       derived_at_sec  = -1.0;
  };
  RendezvousProvenance rendezvous_held_provenance_;
  double               rendezvous_held_at_sec_ = -1.0;

  // True when the held pair is the centroid-fallback placeholder (no tour yet).
  // The only pair the proposer may replace before the team keeps it: once, and
  // only by a pair with candidates > 1.
  // (notes: rendezvous-held-provisional-upgrade)
  bool rendezvous_held_provisional_ = false;

  // What this robot DRIVES: the pair it has heard every peer confirm.
  // Monotone — a pair that has been committed is never withdrawn, only
  // replaced by a later committed one — so an outage that starts mid-handshake
  // falls back to the previous agreement rather than to nothing.
  RendezvousProposal   rendezvous_agreed_;
  RendezvousProvenance rendezvous_agreed_provenance_;
  double               rendezvous_agreed_at_sec_ = -1.0;
  // Whether the committed triple was the centroid placeholder. Stamped at
  // commit, unlike rendezvous_held_provisional_, so it records what the team
  // actually agreed to. (notes: rendezvous-agreed-provisional)
  bool                 rendezvous_agreed_provisional_ = false;
  // Peers that had confirmed the pair at commit. A final triple may read
  // anywhere in [0, fleet-1]; a provisional one is fleet-1. Read with
  // rendezvous_agreed_provisional_; not a liveness count.
  // (notes: rendezvous-agreed-peers-count)
  int                  rendezvous_agreed_peers_  = 0;

  // Set where the barrier releases the gathered team, cleared when the proposer
  // adopts the new pair; retried on the bootstrap period until then. Set on
  // every robot, read only by the proposer. (notes: rendezvous-reagree-due)
  bool                 rendezvous_reagree_due_   = false;

  // True while the gathered team waits on the cell for the next pair; releases
  // once rendezvous_agreed_ moves off rendezvous_reagree_from_. Timed off the
  // settle clock (settled_sec - rendezvous_settle_sec_).
  // (notes: rendezvous-reagree-waiting-stage)
  bool                 rendezvous_reagree_waiting_ = false;
  RendezvousProposal   rendezvous_reagree_from_;

  // What the map looked like when the gathered team started its settle, so the
  // release can report what the exchange actually moved instead of asserting
  // that it moved something. `taken` is false when no settle ran — a zero delta
  // and an unmeasured one are different facts and the log says which it has.
  struct MapExchangeBaseline {
    bool      taken  = false;
    double    voxels = 0.0;   ///< dense map, own sensing included
    uint32_t  hash   = 0;     ///< cell census, shared part only
    long long merged = 0;     ///< cells a peer's census has changed, cumulative
    /// peer_fusion_deltas_ at the stamp, by fleet id. Empty when dscovox has
    /// not published counters yet, which is a third answer and not a zero.
    std::vector<uint64_t> peer_deltas;
  };
  MapExchangeBaseline  rendezvous_exchange_;

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

  // Last pair heard from each peer, by fleet id, with the mission-elapsed
  // receipt time that dates it. Written in drainTeamWorld AFTER the team_hash
  // and grid_hash checks, so a cell id in here is guaranteed to name the same
  // ground ours does.
  std::vector<RendezvousProposal> rendezvous_peer_;
  std::vector<double>             rendezvous_peer_at_sec_;
  // Provisional flag received with each peer's pair, parallel-indexed. Must not
  // be a RendezvousProposal field (operator== is the commit comparison).
  // uint8_t because std::vector<bool> is a bit-proxy.
  // (notes: rendezvous-peer-provisional-flags)
  std::vector<uint8_t>            rendezvous_peer_provisional_;

  // The pair each peer was last observed holding, by fleet id; the commit rule
  // counts these. A latch counts only while it equals rendezvous_held_, and the
  // latch loop clears it when a peer is seen holding another pair.
  // (notes: rendezvous-peer-confirmed-latch)
  std::vector<RendezvousProposal> rendezvous_peer_confirmed_;

  // Minimum period between new proposer pairs, and the snapshot refresh period
  // appointmentLeadMs costs against. Do not raise it to space meetings out;
  // that is rendezvous_interval_sec_. (notes: rendezvous-proposal-period)
  double rendezvous_proposal_period_sec_ = 30.0;

  // Timetable spacing, fed to RendezvousScheduler::Config::min_interval_ms: the
  // floor on the recurrence, which the scheduler may only widen. Widens
  // hybrid's chase window but does not guarantee one.
  // (notes: rendezvous-interval-timetable)
  double rendezvous_interval_sec_ = 300.0;

  // Separation anchor, advanced only while every peer is in mutual direct
  // contact (rendezvousTeamMutual), so it stops on both robots at once. Do not
  // fold into team_last_complete_time_, which every arm uses.
  // (notes: rendezvous-anchor-time)
  rclcpp::Time rendezvous_anchor_time_;
  bool         have_rendezvous_anchor_ = false;

  // Set when a starved heartbeat tick finds the team already apart (the anchor
  // may be wrong on this robot only); cleared by the next healthy stamp.
  // Diagnostic only: its one reader is the WARN it de-duplicates.
  // (notes: rendezvous-anchor-starved)
  bool rendezvous_anchor_starved_ = false;

  // One appointment per outage: stops the robot re-arming the same agreed pair
  // once it closes. Cleared when the team reads complete, next to the
  // agreed-pair refresh. (notes: rendezvous-spent-one-per-outage)
  bool rendezvous_spent_ = false;

  // The standing appointment. Armed by armAppointment, cleared by
  // closeAppointment; while armed it suppresses the mid-run trigger, which
  // would otherwise burn an attempt every PLAN tick.
  // (notes: appointment-armed-suppresses-trigger)
  RendezvousPlan appointment_;
  bool           appointment_armed_    = false;
  bool           appointment_departed_ = false;
  bool           appointment_arrived_  = false;
  /// True when appointmentPoint() could not place appointment_.cell on any grid
  /// and fell back to latest_pos_. Written only by appointmentPoint(), at
  /// departure, so a robot that never moved is not read as a peer no-show.
  /// (notes: appointment-unplaceable-cell)
  bool           appointment_unplaceable_ = false;
  /// True while in a manoeuvre started to keep an appointment. Unlike
  /// appointment_armed_, it survives the appointment record being closed;
  /// cleared only when the manoeuvre ends.
  /// (notes: member-appointment-manoeuvre)
  bool           appointment_manoeuvre_ = false;
  /// True only when doReturnNav joined the barrier early because the team
  /// settled while walking; lets doReturnSync resume the leg if that premise
  /// lapses. Cleared in startReturnTo, so a lapse costs at most one resume.
  /// (notes: appointment-settle-converted-resume)
  bool           appointment_settle_converted_ = false;
  /// True while an appointment leg detours around a stall: current_goal_ is the
  /// escape target, return_dest_ the cell. Bookkeeping only; cleared per leg in
  /// startReturnTo, kept by the proximity-hold resume.
  /// (notes: appointment-escape-detour-flag)
  bool           appointment_escape_active_ = false;
  /// Escapes spent on the CURRENT appointment leg, against
  /// rendezvous_escape_max_attempts_. Reset per leg in startReturnTo, so a
  /// resumed leg gets its own ladder instead of inheriting a spent one.
  int            appointment_escapes_used_ = 0;
  double         appointment_armed_at_sec_   = -1.0;
  double         appointment_arrived_at_sec_ = -1.0;
  /// HYBRID only: the mid-run trigger has had its one chase for the standing
  /// appointment. Cleared wherever appointment_armed_ is set or cleared, and
  /// nowhere else, so a declining chase cannot drain the budget.
  /// (notes: hybrid-appointment-chase-tried)
  bool           appointment_chase_tried_ = false;
  // (notes: rdv-no-show-list-removed)

  // ---- MDP interception (P6, §3.7) ------------------------------------
  // pursuit_predictor trail (default) is the shipped chase; mdp changes only
  // which point the chase drives to first. Every predictor refusal falls
  // through to the trail code below. (notes: pursuit-mdp-interception-scope)
  bool pursuit_predictor_mdp_ = false;
  PursuitPredictor::Config pursuit_mdp_cfg_;
  // <= 0 means "assume the peer moves as I do" and is the default. It exists
  // as a knob because it is a different KIND of quantity from my own speed: I
  // measure mine and I am guessing at the peer's, and a campaign that wants to
  // test the guess needs to be able to vary it alone.
  double pursuit_mdp_peer_speed_mps_ = -1.0;

  // This robot's global tour from the last allocator solve, broadcast as
  // TeamWorld.my_tour; empty when the allocator is off or refused. doPlan
  // assigns it per solve; publishTeamWorld clears it outside the exploration
  // loop. (notes: allocator-my-tour-two-writers)
  std::vector<int> my_tour_;

  /// A peer's first-hand route from TeamWorld plus its position from that same
  /// message; never anchor it on the RobotIntent pose. Keyed by fleet id,
  /// stamped with local receipt time (peer stamps are untrusted).
  /// (notes: peer-tour-anchor-consistency)
  struct PeerTour {
    std::vector<int> cells;
    Eigen::Vector3f  pos = Eigen::Vector3f::Zero();
    rclcpp::Time     stamp;
  };
  std::map<int, PeerTour> peer_tours_;

  /// The prediction the current chase was aimed by, for the dispatch event.
  /// Consumed by logReconnectDispatch like reconnect_decline_reason_, so a
  /// later hold or resume does not re-report it.
  /// (notes: pursuit-dispatch-predict-consumed)
  PursuitTarget dispatch_predict_;
  bool          have_dispatch_predict_ = false;
  /// Age (s) of the tour the prediction ran on. Kept beside the target rather
  /// than in PursuitTarget because it describes the input, which explains most
  /// refusals. (notes: pursuit-dispatch-tour-age)
  double dispatch_predict_tour_age_sec_ = -1.0;

  bool       team_merge_local_priority_ = true;
  std::string team_world_pub_topic_;
  std::vector<std::string> team_world_sub_topics_;
  rclcpp::Publisher<explo_planner_msgs::msg::TeamWorld>::SharedPtr
      team_world_pub_;
  std::vector<rclcpp::Subscription<explo_planner_msgs::msg::TeamWorld>::SharedPtr>
      team_world_subs_;
  rclcpp::TimerBase::SharedPtr team_world_timer_;
  /// Newest undrained TeamWorld per sender id, with local receipt time and
  /// superseded count. Newest-only is safe because the message is full state.
  /// The subscription callback must only write here, never merge.
  /// (notes: teamworld-pending-newest-only)
  struct PendingTeamWorld {
    explo_planner_msgs::msg::TeamWorld::SharedPtr msg;
    rclcpp::Time received;
    int superseded = 0;
  };
  std::map<int, PendingTeamWorld> team_world_pending_;
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
  // un-finishing; "streak" is the legacy PLAN-tick streak then
  // finishOrRendezvous. (notes: done-criterion-latch-vs-streak)
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
  // Master switch for the 2D planning_map. True: subscribed, a hard startup
  // precondition, used for candidate filtering and cost-grid reachability in
  // both phases. False (default): never read; straight-line costs.
  // (notes: planning-map-master-switch)
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
  bool   reconnect_enabled_       = true;
  // How many teammates to wait for at the barrier (team size minus self). The
  // multi_robot launch sets this from the robot list; override in YAML on
  // hardware. <= 0 disables rendezvous.
  int    rendezvous_expected_peers_ = 0;
  // Barrier give-up (seconds). 0 = wait forever (the requested default: STAY
  // until all connected). A positive value is an escape hatch for field trials
  // so a robot whose teammate died doesn't hold the anchor indefinitely.
  double rendezvous_max_wait_sec_  = 0.0;
  // --- Mesh reconnection (robot-carried radios) params ---
  // reconnect_mode_ picks the manoeuvre at exploration exhaustion (rendezvous,
  // pursuit, hybrid); the canonical arm definitions live in planner_util.hpp.
  // The yaml/sim opt into hybrid. (notes: reconnect-mode-arm-table)
  ReconnectMode reconnect_mode_ = ReconnectMode::RENDEZVOUS;
  // Hard ceiling on one chase (s). <= 0 disables pursuit (pursuit/hybrid then
  // behave like their fallback). Also the worst-case bound a WAITING teammate
  // can assume about its pursuer, so it wants a config'd cap, not a formula.
  double pursuit_budget_max_sec_    = 240.0;
  // Measured travel speed (m/s), used only to decide whether a chase is
  // feasible; not nav_speed_est_mps_, which is a watchdog estimate. No safety
  // factor; <= 0 is clamped at the use site.
  // (notes: pursuit-measured-speed-gate)
  double pursuit_speed_measured_mps_ = 0.40;
  // Last-contact record age (s) past which pursuit is skipped; freshness scales
  // the budget linearly to zero across this window. <= 0 = no gate. Safe this
  // wide because of pursuit_goal_stale_sec_. (notes: pursuit-staleness-window)
  double pursuit_staleness_max_sec_ = 900.0;
  // Record age past which the chase drops the peer's declared goal, targets
  // peer_pose alone and budgets by distance, declining if that budget cannot
  // cover the trail. <= 0 = never split. (notes: pursuit-goal-stale-split)
  double pursuit_goal_stale_sec_ = 180.0;
  // Pursuit's fallback when the chase cannot start or is spent: keep exploring
  // rather than park. Bounded by pursuit_explore_max_, after which the robot
  // reverts to holdForTeam so the run still ends.
  // (notes: pursuit-explore-fallback)
  bool pursuit_explore_fallback_ = true;
  int  pursuit_explore_max_      = 6;
  int  pursuit_explores_         = 0;
  // How long the team must have been incomplete before a manoeuvre may arm, on
  // top of coord_claim_ttl_sec_, so one noisy claim-table read cannot arm it.
  // <= 0 arms on the first read. (notes: reconnect-confirm-incomplete-dwell)
  double reconnect_confirm_sec_ = 3.0;
  // Last time the team was observed complete, and whether it ever was. Updated
  // on the heartbeat timer so it advances in every state; the flag stops a
  // never-complete team waiting for a confirmation.
  // (notes: team-last-complete-heartbeat)
  rclcpp::Time team_last_complete_time_;
  bool         team_seen_complete_ = false;
  // --- Mid-exploration reconnect trigger ---
  // Seconds. 0 keeps reconnection terminal-only; > 0 arms the same dispatch
  // during exploration once the team has been continuously incomplete this
  // long. (notes: midrun-silence-threshold-90s)
  double reconnect_midrun_silence_sec_  = 90.0;
  // Barrier give-up for mid-run manoeuvres only; terminal manoeuvres keep
  // rendezvous_max_wait_sec_. Deliberately not a multiple of
  // reconnect_midrun_silence_sec_. (notes: midrun-barrier-max-wait)
  double reconnect_midrun_max_wait_sec_ = 240.0;

  // Inert: declared, read and logged only so manifests stay comparable; not
  // validated and decides nothing. The appointment arming floor is bare t_now.
  // (notes: rdv-depart-delay-inert)
  double rendezvous_depart_delay_sec_ = 100.0;

  // Barrier give-up for an appointment; <= 0 waits at the agreed cell until
  // every robot is present. Separate from reconnect_midrun_max_wait_sec_, which
  // bounds a pursuit. (notes: rdv-appointment-wait-unbounded)
  double rendezvous_appointment_wait_sec_ = 0.0;

  // Lateness budget for choosing a rung of t_meet + k*interval: the first rung
  // reachable within it at arming (appointmentLeadMs). It only pushes k later
  // and bounds the sign-up, not the realised arrival.
  // (notes: rdv-max-lateness-rung)
  double rendezvous_max_lateness_sec_ = 60.0;

  // Cap on the at-the-rendezvous hold; deliberately not
  // rendezvous_appointment_wait_sec_. Must outlast one rolled rung plus
  // lateness; the value is derived in RDV_LATCHED_HOLD in
  // run_explo_sim_rviz.sh. (notes: rdv-latched-hold-cap)
  double rendezvous_latched_hold_sec_ = 300.0;

  // Hold at the meeting point after the team reads complete, before exploring
  // again, so the map exchange can finish before both robots re-plan.
  // (notes: rdv-settle-hold-purpose)
  double rendezvous_settle_sec_ = 30.0;

  // Instant the settle hold started, on ROS time (a duration needs no epoch),
  // and whether one is running. The bool guards subtracting a default
  // system-clock rclcpp::Time from sim now(), which throws.
  // (notes: rdv-settle-start-ros-time)
  rclcpp::Time rendezvous_settle_start_;
  bool         rendezvous_settling_ = false;

  // Release the settle hold on the observed map exchange instead of the clock.
  // Default off; R and W are -1 (unset) and configure() refuses to run with the
  // release enabled and either unset. (notes: rdv-drain-release-thresholds)
  bool   rendezvous_drain_release_        = false;  ///< the treatment
  double rendezvous_drain_rate_vox_sec_   = -1.0;   ///< R, per peer
  double rendezvous_drain_window_sec_     = -1.0;   ///< W, the trailing window

  // A TUMBLING WINDOW, ON THE SETTLE'S CLOCK, and the hard floor — see
  // DrainWindow in exchange_drain.hpp. Owned here, advanced only by
  // stepDrainRelease, and reset to a default DrainWindow wherever the settle is.
  DrainWindow rendezvous_drain_window_;

  // Mission-clock start of the appointment-barrier wait, floored at t_meet; -1
  // when not waiting. Bounds holdingForFinishedPeer
  // (rendezvous_latched_hold_sec_). Cleared with appointment_manoeuvre_ and by
  // non-appointment legs. (notes: rdv-finished-peer-wait-bound)
  double finished_peer_wait_start_sec_       = -1.0;
  bool   finished_peer_wait_expired_logged_  = false;

  // Once the drain fires, the hold does not re-enter on a late burst within the
  // visit. Cleared wherever rendezvous_settling_ is, so the next visit re-arms.
  // (notes: rdv-drain-release-monotone)
  bool rendezvous_drain_released_ = false;

  // Per-run cap on mid-run dispatches, successes included; midrun_attempts_ is
  // never refunded or reset. Once spent, the robot reverts to terminal-only
  // reconnection (logged). (notes: midrun-attempt-budget-dispatches)
  int    reconnect_midrun_max_attempts_ = 6;
  // --- Link-state gating for the mid-run trigger (see §30.11 and §30.24) ---
  // Read only connected for pairs involving this robot, and path_loss_db only
  // as the startup mask. "" (default) leaves the gate off.
  // (notes: link-gate-legitimate-reads)
  std::string comms_link_states_topic_;
  std::string comms_link_robot_index_topic_;
  // Newest sample older than this and the gate stands down to the legacy clock
  // rather than acting on a stale belief. The emulator publishes at 5 Hz, so
  // 3 s is 15 missed samples: a real gap, not jitter.
  double comms_link_stale_sec_ = 3.0;
  // How long the radio must have been continuously down before the veto lets a
  // mid-run chase through. Separate from reconnect_confirm_sec_. 0 (default)
  // vetoes only a peer on the radio right now.
  // (notes: link-down-confirm-debounce)
  double reconnect_link_down_confirm_sec_ = 0.0;
  /// The two link_states table widths this node knows how to read. Keep in
  /// step with hmr_comms_sim_node.cpp's kLinkStateCols.
  static constexpr size_t kLinkColsLegacy    = 9;
  static constexpr size_t kLinkColsWithValid = 10;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      link_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr link_index_sub_;
  int  link_self_idx_      = -1;     ///< own row index in the emulator's table
  int  link_robot_count_   = 0;      ///< robots in that table, self included
  bool link_index_usable_  = false;  ///< an index naming us has arrived
  bool link_index_warned_  = false;  ///< unusable-index complaint is emitted once
  /// Columns per pair in the link_states table, read off the message layout
  /// rather than assumed. 9 and 10 (with valid) are accepted; 0 = nothing
  /// parsed yet. (notes: link-cols-read-from-layout)
  size_t link_cols_        = 0;
  bool link_cols_warned_   = false;  ///< unparseable-layout complaint, once
  /// Newest reading of my links: true only when every peer in the index has a
  /// usable row and all are up. All, not any: at N>=3 any would veto a dispatch
  /// while one peer is still out. (notes: link-connected-all-not-any)
  bool link_connected_     = false;
  bool link_have_sample_   = false;
  bool link_clock_anchored_ = false;
  /// One-shot latch for the "gate went live" console line, the only record that
  /// a usable sample arrived and the veto was in force. gate_g8.py check 3f
  /// asserts the line is present. (notes: link-gate-live-log-latch)
  bool link_gate_live_logged_ = false;
  rclcpp::Time link_last_sample_time_;  ///< receipt of the newest usable sample
  // Receipt time the link was last observed up; down-duration is measured from
  // here, not a stored edge, as Float64MultiArray has no header. Hence the
  // subscription is KeepLast(1). (notes: link-up-last-seen-keeplast)
  rclcpp::Time link_up_last_seen_;
  // --- Info-gated mid-run trigger (voxels, not seconds) ---
  // 0 (default) keeps the silence clock. > 0: T = min_share / (my_rate +
  // peer_rate) from the frozen LastContact rates, clamped to [min_silence,
  // max_silence]; zero rates give max_silence.
  // (notes: midrun-info-gated-trigger)
  double reconnect_min_share_voxels_     = 0.0;
  // Clamp bounds for the derived trigger time. The ceiling defaults to the
  // resolved reconnect_midrun_silence_sec_ at the parameter-read site; its
  // initialiser only mirrors that. Inert while reconnect_min_share_voxels_ is
  // 0. (notes: midrun-info-trigger-clamps)
  double reconnect_midrun_min_silence_sec_ = 60.0;
  double reconnect_midrun_max_silence_sec_ = 90.0;  // mirrors the clock
  // True while the CURRENT manoeuvre was dispatched from exploration
  // exhaustion (the only kind that may end in DONE); false for mid-run
  // dispatches, which must always resume exploring instead. Default true so
  // every pre-existing path behaves exactly as before.
  bool   reconnect_terminal_ = true;
  int    midrun_attempts_    = 0;
  // Cooldown stamped when a mid-run manoeuvre ends (transitionTo, where
  // reconnect_active_ falls), never at dispatch. Bool-guarded: a default
  // rclcpp::Time is on the system clock and subtracting it throws.
  // (notes: midrun-cooldown-stamped-at-end)
  rclcpp::Time midrun_last_end_;
  bool         midrun_end_armed_ = false;
  // --- Hold escalation (mutual-hold deadlock break) ---
  // When a terminal barrier wait expires, drive once to the last-connected
  // anchor and wait hold_escalate_wait_sec_ more. hold_escalated_ is sticky per
  // manoeuvre, not a position test. (notes: hold-escalation-deadlock-break)
  bool   hold_escalate_          = true;
  double hold_escalate_wait_sec_ = 300.0;
  bool   hold_escalated_         = false;
  // reconnect_rec_ is declared with the LastContact members below. Arrival
  // tolerance for a manoeuvre destination, looser than goal_xy_tol_ because its
  // value is connectivity, not position. (notes: reconnect-arrive-tolerance)
  double reconnect_arrive_tol_m_ = 4.0;
  // How far from the agreed cell a failed appointment drive may stop and still
  // count as at the meeting; looser than reconnect_arrive_tol_m_, which decides
  // success. Sized against the link horizon.
  // (notes: rdv-present-tolerance-failed-drive)
  double rendezvous_present_tol_m_ = 10.0;
  // Escapes one appointment leg may spend before rolling to the next rung;
  // separate from return_escape_max_attempts_. 0 disables the ladder (straight
  // to roll). (notes: rdv-escape-ladder-attempts)
  int    rendezvous_escape_max_attempts_ = 3;
  // Ceiling on one manoeuvre drive leg, as startReturnTo's distance-true budget
  // is exempt from nav_max_timeout_sec_. No leg may cost more than the barrier
  // it drives toward. <= 0 = unbounded. (notes: reconnect-nav-leg-ceiling)
  double reconnect_nav_max_sec_ = 600.0;
  // --- Release confirmation (flicker guard) ---
  // The release condition must hold continuously this long. Must exceed
  // coord_claim_ttl_sec_, or one packet satisfies it. 0 = release on first
  // read. (notes: reconnect-release-confirm-flicker)
  double reconnect_release_confirm_sec_ = 6.0;
  // Dwell windows: release_ok_* for the manoeuvre barrier, team_back_ok_* for
  // the other team-came-back sites; one confirm parameter. Written only in
  // heartbeatTick at 1 Hz; doPlan and doReturnNav read via dwellHeld. Seconds.
  // (notes: team-back-dwell-windows)
  double release_ok_since_sec_    = 0.0;
  bool   release_ok_armed_        = false;
  double team_back_ok_since_sec_  = 0.0;
  bool   team_back_ok_armed_      = false;

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
  // Per-target give-up budget (s) since the last progress event; on expiry the
  // target closes PARTIAL. <= 0 disables. Must stay above nav_max_timeout_sec_,
  // since failed hops keep charging it. (notes: exploit-target-timeout-budget)
  double exploit_target_timeout_sec_ = 300.0;
  // Vantage-ring barrier: the dwell clock does not start until every peer with
  // an exploit claim on this trunk stands on its claimed vantage. Inert with
  // coordination off or no peer claim here. (notes: exploit-dwell-sync-barrier)
  bool   exploit_dwell_sync_enabled_ = true;
  // Barrier give-up (s), measured from EXPLOIT_DWELL entry, not the re-anchored
  // dwell start. <= 0 (default) waits until the peer arrives; doExploitDwell
  // argues why that terminates. (notes: exploit-dwell-sync-max-wait)
  double exploit_dwell_sync_max_wait_sec_ = 0.0;
  // Fine-TSDF region relay: each ingested TreeTarget is registered as a
  // RefinementRegion on scovox_node and unregistered when finishActiveTarget
  // closes it. Safe to leave on. (notes: exploit-fine-region-relay)
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
  // Per-target give-up timer (sim s): latched on activation, re-armed on each
  // progress event, refunded for proximity holds, cleared on rendezvous
  // stand-down. Failed hops keep charging it.
  // (notes: exploit-target-timer-rules)
  uint32_t exploit_target_started_id_  = 0;
  double   exploit_target_started_sec_ = 0.0;
  bool     exploit_target_timing_      = false;
  // Dwell-sync barrier state, reset on every EXPLOIT_DWELL entry. The wait
  // clock must not be state_enter_time_, which the barrier re-anchors each
  // tick. The latches stop it re-anchoring a running dwell.
  // (notes: exploit-dwell-sync-bookkeeping)
  rclcpp::Time dwell_sync_wait_start_;
  bool         dwell_sync_timed_out_ = false;
  bool         dwell_sync_started_   = false;
  // The vantage this robot last completed a dwell on; it parks here when the
  // team has covered the rest of the ring. Valid only when
  // held_vantage_target_id_ matches, as target ids are never reused.
  // (notes: exploit-held-vantage-pose)
  CandidateViewpoint held_vantage_pose_;
  int      held_vantage_index_     = -1;
  uint32_t held_vantage_target_id_ = 0;
  bool     held_vantage_valid_     = false;
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

  // Cache key for the unbounded exploitation flood of cost_grid_, rebuilt only
  // when inputs change. exploit_flood_map_ is compared, never dereferenced.
  // doPlan's bounded flood invalidates it. (notes: exploit-flood-cache-key)
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

  // Coverage termination latch (done_criterion == "latch"): set on the first
  // qualifying sample, never cleared. The stamps record the latch instant,
  // which is not the run's end. (notes: coverage-latch-one-way)
  bool   coverage_latched_        = false;
  double coverage_latch_t_sim_    = -1.0;
  double coverage_latch_unknown_  = -1.0;

  // Monotonic "this robot's run is over"; the only source of
  // TeamWorld/finished. Latched once (coverage_latched_ || state_ ==
  // State::DONE) first holds, never cleared, because peers relay the bit.
  // (notes: teamworld-finished-announced-latch)
  bool   finished_announced_      = false;

  // Monotonic "left for home"; the only source of TeamWorld/mode at the HOMING
  // level. Latched once (state_ == State::RETURN_HOME || mission_return_done_)
  // first holds; never cleared, as peers relay it.
  // (notes: teamworld-homing-announced-latch)
  bool   homing_announced_        = false;

  // The DONE level's latch, separate from finished_announced_: latched once
  // this robot is finished and no longer keeping an appointment (announcedMode,
  // meeting_attendance.hpp). (notes: teamworld-done-announced-latch)
  bool   done_announced_          = false;

  // --- The at-the-rendezvous hold (2026-09-17, generation 21) ---
  // coverage_latch_hold_start_sec_ is mission-elapsed at the later of the latch
  // and reaching the barrier. coverage_latch_teardown_ marks a run ended at the
  // meeting; sticky. (notes: rdv-at-rendezvous-hold)
  double coverage_latch_hold_start_sec_ = -1.0;
  bool   coverage_latch_teardown_       = false;

  // --- Post-latch coast (done_seek_enabled) ---
  // A latch mid-manoeuvre keeps the existing navigator goal so the robot
  // finishes its drive to the partner, while state_ reads DONE from that tick.
  // Off by default; a runtime switch. (notes: done-seek-post-latch-coast)
  bool   done_seek_enabled_       = false;
  double done_seek_max_sec_       = 600.0;
  // Live coast state. done_seek_coasting_ is the ONLY thing that distinguishes
  // a DONE robot that is still rolling from one that is parked, so every exit
  // path must clear it — an uncleared flag would shrink the presence claim for
  // the rest of the run (see publishPresenceIntent).
  bool   done_seek_coasting_      = false;
  double done_seek_start_sim_     = -1.0;
  double done_seek_dist_at_start_ = 0.0;
  double done_seek_last_dist_     = 0.0;
  double done_seek_last_move_sim_ = -1.0;

  // --- Mission return (mission_return_enabled) ---
  // At any terminal ending drive to the start pose, pre-empting park-in-place
  // and the done_seek coast. have_home_ is one-shot, unlike have_pose_. Own
  // tolerance: reconnect_arrive_tol_m_ would accept the partner's home.
  // (notes: mission-return-home)
  bool   mission_return_enabled_  = false;
  double mission_home_tol_m_      = 1.0;
  double mission_return_max_sec_  = 600.0;

  // No robot plans or navigates until this much mission time has passed, in
  // every arm, so the rendezvous placeholder is agreed while co-located. Do not
  // confine the provisional->final upgrade to this hold.
  // (notes: mission-start-hold-all-arms)
  double mission_start_hold_sec_  = 60.0;
  // One-shot, so the "still holding" line cannot be mistaken for a stall and
  // the release is stamped once with the number it waited for.
  bool   mission_start_hold_logged_ = false;
  bool   have_home_               = false;
  Eigen::Vector3f home_pos_       = Eigen::Vector3f::Zero();
  float  home_yaw_                = 0.f;
  // Live homing-leg state. return_home_goal_sent_ defers the goal publish
  // past the cancel-all; the rest feed the mission_complete event and the
  // run_end summary.
  bool   return_home_goal_sent_   = false;
  int    return_home_retries_     = 0;
  double return_home_dist_at_start_ = 0.0;
  // Wall and hold clocks for the homing leg, stamped with
  // return_home_dist_at_start_ so all three measure one interval; the budget
  // still runs off state_enter_time_. Explicitly RCL_ROS_TIME: mixed clocks
  // throw. (notes: homing-leg-clocks-same-interval)
  rclcpp::Time return_home_start_time_{0, 0, RCL_ROS_TIME};
  double return_home_hold_at_start_ = 0.0;
  std::string return_home_reason_;
  std::string mission_home_result_;      // empty until resolved
  double mission_home_sim_sec_    = -1.0;
  // Publish gate for the deferred home goal: under a backlogged executor "next
  // tick" is not a gap, so publish only after this time. ROS time, compared
  // against this->now(). (notes: homing-goal-publish-gate)
  rclcpp::Time home_pub_not_before_{0, 0, RCL_ROS_TIME};
  // Breadcrumb trail for the retrace fallback: own outbound positions at >= 2 m
  // spacing, recorded from home capture until homing starts. The second
  // no-progress retry follows it back crumb by crumb.
  // (notes: homing-breadcrumb-trail)
  std::vector<Eigen::Vector3f> home_trail_;
  int  home_trail_idx_      = -1;

  // ---- Approach-based homing watchdog (binary generation 5) ----
  // Measures remaining distance home, independently of the gross-travel frozen
  // watchdog; fires report as kind=approach vs kind=frozen. home_mode_ selects
  // among three publishable targets. (notes: home-approach-watchdog-rationale)
  HomeMode home_mode_ = HomeMode::DIRECT;
  rclcpp::Time approach_check_time_{0, 0, RCL_ROS_TIME};
  float  approach_check_metric_ = 0.f;
  // Band clock for the near-home stall detector. Armed on entry to the
  // suppression band and cleared on leaving it, on an escape leg, and on a
  // relocalization jump -- a jump can teleport the robot into the band, and
  // dwell time measured across one is not dwell time.
  bool         near_home_armed_ = false;
  rclcpp::Time near_home_since_{0, 0, RCL_ROS_TIME};
  int    home_escapes_used_     = 0;
  int    escape_last_crumb_     = -1;  // anti-repeat: never twice in a row
  int    escape_fallback_n_     = 0;   // rotates the behind-robot fallback
  rclcpp::Time escape_leg_start_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3f escape_target_ = Eigen::Vector3f::Zero();
  double return_approach_window_sec_ = 40.0;
  double return_approach_min_m_      = 1.0;
  // Radius inside which the approach detector is muted, and the cap on how long
  // a robot may sit inside it without arriving. Was a file-scope constexpr
  // (kApproachSuppressM), which made the one number this defect turns on
  // untunable without a rebuild -- see the band note below.
  double return_approach_suppress_m_ = 3.0;
  double return_near_home_stall_sec_ = 60.0;
  int    return_escape_max_attempts_ = 3;
  double return_escape_leg_sec_      = 30.0;
  // Escape geometry is tied to the 2 m breadcrumb spacing. The approach check
  // is muted inside return_approach_suppress_m_, where 1.0 m is a large share
  // of what is left; the near-home band clock covers that band.
  // (notes: home-escape-geometry-band)
  static constexpr float kEscapeBandMinM      = 1.5f;
  static constexpr float kEscapeBandMaxM      = 6.0f;
  static constexpr float kEscapeArriveM       = 1.5f;
  static constexpr float kEscapeFallbackM     = 2.5f;
  // Placeholder on home_watchdog rows with no detector inequality
  // (kind="escape-end"). Not a sentinel: the writer omits both test fields on
  // those rows, keyed on kind; 0.0 and negatives are real measurements.
  // (notes: home-watchdog-no-test-delta)
  static constexpr double kNoTestDelta        = 0.0;
  // Raised by trackDistance's teleport guard, consumed by doReturnHome: an
  // approach delta measured across a relocalization discontinuity is a
  // measurement artefact, not a stall.
  bool pose_jump_seen_ = false;

  // Robot pose when a teammate was last heard (inside the comms bubble, so the
  // cheapest reconnection point). Recorded on every peer intent; have_anchor_
  // stays false until the first peer is heard.
  // (notes: rendezvous-last-connected-anchor)
  Eigen::Vector3f last_connected_anchor_ = Eigen::Vector3f::Zero();
  bool  have_anchor_ = false;

  // Per-peer record at each received intent, keyed by robot_id: my pose, the
  // peer's advertised pose and its declared goal. Stamped with LOCAL receipt
  // time; peer stamps are untrusted. (notes: mesh-last-contact-record)
  struct LastContact {
    Eigen::Vector3f self_pose = Eigen::Vector3f::Zero();
    Eigen::Vector3f peer_pose = Eigen::Vector3f::Zero();
    Eigen::Vector3f peer_goal = Eigen::Vector3f::Zero();
    rclcpp::Time    stamp;
    // Map size at contact: my cached voxel count/rate and the peer's beaconed
    // ones. rate 0.0 means unknown; the gate then falls back to the time-only
    // trigger rather than divide by it. (notes: last-contact-map-size-snapshot)
    double self_voxels = 0.0, peer_voxels = 0.0;
    double self_rate   = 0.0, peer_rate   = 0.0;
  };
  std::map<std::string, LastContact> last_contact_;

  // Valid while state_ == PURSUE. startPursuit builds the trail: the peer's
  // goal first, then its last heard pose. The budget runs from
  // pursue_start_time_ across all waypoints; proximity-hold time is refunded.
  // (notes: pursuit-trail-and-budget)
  std::vector<Eigen::Vector3f> pursue_waypoints_;
  size_t       pursue_wp_index_   = 0;
  double       pursue_budget_sec_ = 0.0;
  rclcpp::Time pursue_start_time_;
  std::string  pursue_peer_id_;
  // Snapshot of the record the chase was armed from; write-only, kept to
  // document the snapshot discipline. To remove it, delete startPursuit's write
  // with it. (notes: pursuit-rec-write-only-snapshot)
  LastContact  pursue_rec_;

  // The pair the current manoeuvre was armed from; targets derived from a
  // per-peer record use this, never a re-read of last_contact_. Taken by
  // dispatchReconnect, cleared in transitionTo when reconnect_active_ falls.
  // (notes: reconnect-rec-manoeuvre-snapshot)
  LastContact  reconnect_rec_;
  bool         have_reconnect_rec_ = false;

  // Log label for the current RETURN_NAV destination, set by startReturnTo and
  // read only by doReturnNav's logs; not state. Values: "appointment" or
  // "last-connected anchor". (notes: return-dest-label-log-only)
  std::string  return_dest_label_;
  // Where the current RETURN_NAV leg is going (appointment cell or
  // last-connected anchor), set by startReturnTo with the label. Every distance
  // test in doReturnNav uses this, never current_goal_, which escapes borrow.
  // (notes: return-dest-vs-current-goal)
  Eigen::Vector3f return_dest_ = Eigen::Vector3f::Zero();

  // Clock for reconnect_elapsed_sec; not state_enter_time_, as the manoeuvre
  // spans state changes. Armed once by startReturnTo/startPursuit
  // (already-active guard in both), cleared in transitionTo on leaving them.
  // (notes: reconnect-manoeuvre-clock)
  bool         reconnect_active_ = false;
  rclcpp::Time reconnect_start_time_;

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
  /// Seconds of team-incomplete before dispatch: the fixed silence clock if the
  /// info gate is off or the snapshot unusable, else the crossing time of
  /// reconnect_min_share_voxels clamped to [min,max] silence.
  /// (notes: midrun-gate-sec-threshold)
  double midrunGateSec(double missing_for, double* est_unshared_out);
  // Stashed at the trigger decision, consumed by the next reconnect_dispatch
  // event. -1 = not applicable (terminal dispatch or gate off); -2 = gate on
  // but no usable contact snapshot, time-only fallback.
  // (notes: dispatch-gate-diagnostics-sentinels)
  double dispatch_gate_sec_     = -1.0;
  double dispatch_est_unshared_ = -1.0;
  // Radio-down time when the mid-run trigger fired; -1 = link gate not in play.
  // Diagnostic only, never the trigger clock: the gate only vetoes, so a gated
  // dispatch reads >= reconnect_link_down_confirm_sec.
  // (notes: dispatch-link-down-diagnostic)
  double dispatch_link_down_sec_ = -1.0;
  /// The mid-run trigger's own left-hand side, held for the whole manoeuvre like
  /// the other dispatch_* diagnostics so every leaf re-reports the decision it
  /// was armed from. See ReconnectDispatch::team_incomplete_sec for why this is
  /// not peer_record_age_sec.
  double dispatch_team_incomplete_sec_ = -1.0;
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
  // Shared tree-target topic. The time-based scheduler publishes here today; a
  // detector can publish the same message later with no planner change.
  rclcpp::Subscription<explo_planner_msgs::msg::TreeTarget>::SharedPtr target_sub_;
  // Fine-TSDF region relay to this robot's own scovox_node (topic built
  // absolute from robot_name_ — this node is not namespaced by the packaged
  // launch files). Null unless exploitation and publish_refinement_regions
  // are both enabled.
  rclcpp::Publisher<scovox_msgs::msg::RefinementRegion>::SharedPtr region_pub_;
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
  // Keyed on step_: one exploration_complete per exhaustion episode, although
  // finishOrRendezvous re-enters every tick while the confirmation gate defers;
  // a later genuine re-exhaustion still records.
  // (notes: exploration-complete-dedup)
  int exp_complete_step_  = -1;
  int exp_complete_count_ = 0;
  // Exactly one mission_complete per run, enforced by mission_return_done_
  // (run-scoped) and the state_ == RETURN_HOME test (leg-scoped) in
  // startReturnHome. Incremented on every emission unconditionally.
  // (notes: mission-complete-exactly-once)
  int mission_complete_count_ = 0;
  // Makes the mission return once per run: set when finishMissionReturn commits
  // the ending; startReturnHome refuses every later request. The state test
  // cannot, since a second request arrives from DONE.
  // (notes: mission-return-once-per-run)
  bool mission_return_done_ = false;
  // Requests refused by mission_return_done_, reported on run_end. The first
  // refusal WARNs, once only, since the caller can be a tick loop.
  // (notes: mission-return-reentry-counter)
  int mission_return_reentries_ = 0;
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
  // CSV sampling period; steps do not advance during a reconnect manoeuvre, so
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
  coord_vantage_claim_radius_m_ = dp("coord_vantage_claim_radius_m", 0.0);

  // Master switch for the whole reconnect subsystem (false = control arm); not
  // rendezvous_schedule_enable or reconnect_mode. The deprecated alias
  // rendezvous_enabled wins when explicitly set; a disagreement WARNs.
  // (notes: param-reconnect-enabled-alias)
  {
    const auto& ovr =
        this->get_node_parameters_interface()->get_parameter_overrides();
    const bool has_new = ovr.count("reconnect_enabled") > 0;
    const bool has_old = ovr.count("rendezvous_enabled") > 0;
    const bool v_new = dp("reconnect_enabled", true);
    const bool v_old = dp("rendezvous_enabled", true);
    reconnect_enabled_ = has_old ? v_old : v_new;
    if (has_old && has_new && v_new != v_old) {
      RCLCPP_WARN(get_logger(),
          "Both reconnect_enabled=%d and the deprecated alias "
          "rendezvous_enabled=%d were set, and they DISAGREE. The alias wins "
          "(reconnect subsystem %s). Drop one of them.",
          v_new, v_old, v_old ? "ON" : "OFF");
    } else if (has_old) {
      RCLCPP_WARN(get_logger(),
          "Parameter 'rendezvous_enabled' is deprecated — it is the master "
          "switch for the whole reconnect subsystem, not the rendezvous arm. "
          "Use 'reconnect_enabled' instead (value %d carried over).", v_old);
    }
  }
  // The subsystem activates only with coordination on and
  // rendezvous_expected_peers > 0; otherwise all manoeuvres (rendezvous,
  // pursuit, hybrid) are off. WARN, not INFO: it voids a treated arm.
  // (notes: reconnect-preconditions-warn)
  rendezvous_expected_peers_ = dp("rendezvous_expected_peers", 0);
  rendezvous_max_wait_sec_   = dp("rendezvous_max_wait_sec", 0.0);
  if (reconnect_enabled_ &&
      (!coord_enabled_ || rendezvous_expected_peers_ <= 0)) {
    RCLCPP_WARN(get_logger(),
        "RECONNECT SUBSYSTEM DISABLED by preconditions "
        "(coordination_enabled=%d, rendezvous_expected_peers=%d). "
        "reconnect_enabled was requested but ALL manoeuvres — rendezvous, "
        "pursuit and hybrid — are now off, and this robot runs as plain "
        "exploration, finishing when goals are exhausted. Any manifest field "
        "naming a reconnect mode for this run describes the REQUEST, not the "
        "behaviour: read reconnect_enabled, not reconnect_mode_param.",
        coord_enabled_, rendezvous_expected_peers_);
    reconnect_enabled_ = false;
  }
  // The barrier counts peers by claim beacons, and only a DONE-idle robot keeps
  // beaconing; a done_action=shutdown finisher exits and becomes invisible to
  // teammates. Hence the warning. (notes: reconnect-needs-done-idle)
  if (reconnect_enabled_ && done_action_ != "idle") {
    RCLCPP_WARN(get_logger(),
        "Rendezvous barrier with done_action='%s': the first robot to finish "
        "exits and stops beaconing, so a teammate finishing later can never "
        "count it (it waits the full rendezvous_max_wait_sec, or forever). "
        "Use done_action=idle for reconnect runs.", done_action_.c_str());
  }

  // "rendezvous" (default, legacy return-to-anchor barrier), "pursuit" or
  // "hybrid". Gated by the reconnect_enabled_ preconditions; the mode only
  // picks which manoeuvre runs once shouldRendezvous() says one should.
  // (notes: param-reconnect-mode)
  {
    const std::string mode_str =
        dp("reconnect_mode", std::string("rendezvous"));
    bool mode_known = false;
    reconnect_mode_ = reconnectModeFromString(mode_str, &mode_known);
    // FATAL: reconnectModeFromString keeps its tested fallback to RENDEZVOUS,
    // but the node refuses it, since the fallback silently runs the cell as a
    // different arm under the requested name.
    // (notes: reconnect-mode-unknown-fatal)
    if (!mode_known) {
      RCLCPP_FATAL(get_logger(),
          "Unknown reconnect_mode '%s'. The only modes are 'rendezvous', "
          "'pursuit' and 'hybrid'; the off arm is reconnect_enabled=false, "
          "NOT reconnect_mode='off'. Refusing to start rather than silently "
          "running this cell as 'rendezvous' under the requested name.",
          mode_str.c_str());
      throw std::runtime_error("unknown reconnect_mode '" + mode_str + "'");
    }
    // reconnect_mode_ is parsed and logged even when the subsystem is disabled;
    // warn here, naming the mode, so the console log separates a mode that ran
    // from one that was voided. (notes: reconnect-mode-inert-warning)
    if (!reconnect_enabled_ && reconnect_mode_ != ReconnectMode::RENDEZVOUS) {
      RCLCPP_WARN(get_logger(),
          "reconnect_mode='%s' is INERT this run: the reconnect subsystem is "
          "disabled, so no %s manoeuvre can fire. Do not score this cell as a "
          "treated cell.", mode_str.c_str(), mode_str.c_str());
    }
  }
  pursuit_budget_max_sec_    = dp("pursuit_budget_max_sec", 240.0);
  pursuit_speed_measured_mps_ = dp("pursuit_speed_measured_mps", 0.40);
  if (pursuit_speed_measured_mps_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
        "pursuit_speed_measured_mps=%.3f is not a speed; the pursuit "
        "feasibility gate will price every chase as instantaneous and refuse "
        "nothing. Set it to the measured traverse speed.",
        pursuit_speed_measured_mps_);
  }
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
  reconnect_midrun_silence_sec_  = dp("reconnect_midrun_silence_sec", 90.0);
  reconnect_midrun_max_wait_sec_ = dp("reconnect_midrun_max_wait_sec", 240.0);
  if (!std::isfinite(reconnect_midrun_max_wait_sec_) ||
      reconnect_midrun_max_wait_sec_ <= 0.0) {
    // Zero is not "do not wait": rendezvousWaitExpired tests max_wait_sec >
    // 0.0, so zero, negative or NaN never expire, and this cap is the robot's
    // only way back to exploring. Reset to 240 s.
    // (notes: midrun-max-wait-zero-guard)
    RCLCPP_WARN(get_logger(),
        "reconnect_midrun_max_wait_sec %.1f would never expire, which parks a "
        "robot mid-run -> 240 s.",
        reconnect_midrun_max_wait_sec_);
    reconnect_midrun_max_wait_sec_ = 240.0;
  }
  reconnect_midrun_max_attempts_ = dp("reconnect_midrun_max_attempts", 6);
  // The notice/wait/settle family (see the member comments). These three ARE
  // the rendezvous arm's definition and are logged as params so a cell can be
  // attributed to them without reading the binary.
  rendezvous_depart_delay_sec_   = dp("rendezvous_depart_delay_sec", 100.0);
  rendezvous_appointment_wait_sec_ =
      dp("rendezvous_appointment_wait_sec", 0.0);
  rendezvous_settle_sec_         = dp("rendezvous_settle_sec", 30.0);
  rendezvous_max_lateness_sec_   = dp("rendezvous_max_lateness_sec", 60.0);
  if (!std::isfinite(rendezvous_max_lateness_sec_) ||
      rendezvous_max_lateness_sec_ < 0.0) {
    // NaN would propagate through the rung floor into nextAgreedOccurrence's
    // llround (undefined); a negative budget demands early arrival and rolls
    // rungs needlessly. Reset to 60 s. (notes: max-lateness-nan-guard)
    RCLCPP_WARN(get_logger(),
        "rendezvous_max_lateness_sec %.1f is not a usable budget -> 60 s.",
        rendezvous_max_lateness_sec_);
    rendezvous_max_lateness_sec_ = 60.0;
  }
  rendezvous_latched_hold_sec_   = dp("rendezvous_latched_hold_sec", 300.0);
  if (!std::isfinite(rendezvous_latched_hold_sec_) ||
      rendezvous_latched_hold_sec_ <= 0.0) {
    // Zero is rejected, not "no cap": guards read > 0.0, so zero disables the
    // hold, and this cap is the only thing that ends a latched keeper's wait
    // (keepAppointmentOnFinish). Reset to 300 s.
    // (notes: latched-hold-zero-rejected)
    RCLCPP_WARN(get_logger(),
        "rendezvous_latched_hold_sec %.1f is not a usable cap -> 300 s. A "
        "non-positive cap would read as 'no cap' and strand a finished robot "
        "at the meeting for the rest of the cell.",
        rendezvous_latched_hold_sec_);
    rendezvous_latched_hold_sec_ = 300.0;
  }
  if (rendezvous_latched_hold_sec_ <= rendezvous_settle_sec_) {
    // Same incoherence rendezvous_appointment_wait_sec is checked for below: a
    // hold shorter than the settle gives up before the map exchange it is
    // holding for could finish, so the hold pays its whole cost and collects
    // none of its benefit.
    RCLCPP_WARN(get_logger(),
        "rendezvous_latched_hold_sec (%.1f) <= rendezvous_settle_sec (%.1f): a "
        "finished robot would leave the meeting before the merge it is waiting "
        "for could complete.",
        rendezvous_latched_hold_sec_, rendezvous_settle_sec_);
  }
  // rendezvous_depart_delay_sec is not validated: it gates nothing and is
  // logged as passed. Non-finite values compare false and silently change the
  // arm, so refuse to the documented default.
  // (notes: rendezvous-nonfinite-param-guard)
  if (!std::isfinite(rendezvous_settle_sec_)) {
    RCLCPP_ERROR(get_logger(),
        "rendezvous_settle_sec is not finite; the map-exchange hold would be "
        "skipped silently. Falling back to 30 s.");
    rendezvous_settle_sec_ = 30.0;
  }
  // Negative is behaviourally identical to zero at both use sites, but the
  // value is written into the manifest as this arm's definition, so normalise
  // it: a cell whose params say -1 and whose barrier waited forever is a cell
  // whose record does not describe it.
  if (rendezvous_settle_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(),
        "rendezvous_settle_sec %.1f < 0 -> 0 (no map-exchange hold).",
        rendezvous_settle_sec_);
    rendezvous_settle_sec_ = 0.0;
  }
  // THE DRAIN RELEASE (generation 33, R3) — see the members. Read here so the
  // manifest carries the arm's definition whether or not it is on.
  rendezvous_drain_release_      = dp("rendezvous_drain_release", false);
  rendezvous_drain_rate_vox_sec_ = dp("rendezvous_drain_rate_vox_sec", -1.0);
  rendezvous_drain_window_sec_   = dp("rendezvous_drain_window_sec", -1.0);
  // Refused, not defaulted: R and W must come from the measured per-peer
  // arrival distribution. R > 0 strictly: rate is clamped at zero, so R = 0
  // makes every peer undrained and the release never fires.
  // (notes: drain-release-thresholds-refused)
  if (rendezvous_drain_release_ &&
      (!std::isfinite(rendezvous_drain_rate_vox_sec_) ||
       rendezvous_drain_rate_vox_sec_ <= 0.0 ||
       !std::isfinite(rendezvous_drain_window_sec_) ||
       rendezvous_drain_window_sec_ <= 0.0)) {
    RCLCPP_ERROR(get_logger(),
        "rendezvous_drain_release is on but the thresholds are unset "
        "(rate=%.3f vox/s, window=%.1f s). Both must come from the measured "
        "per-peer arrival distribution; there is no defensible default.",
        rendezvous_drain_rate_vox_sec_, rendezvous_drain_window_sec_);
    throw std::runtime_error(
        "rendezvous_drain_release requires rendezvous_drain_rate_vox_sec > 0 "
        "and rendezvous_drain_window_sec > 0");
  }
  // Coupling: the drain hold is capped by rendezvous_latched_hold_sec (the
  // doReturnSync gate), so a window at or above the cap means the drain is
  // never evaluated. (notes: drain-window-vs-latched-hold)
  if (rendezvous_drain_release_ &&
      rendezvous_drain_window_sec_ >= rendezvous_latched_hold_sec_) {
    RCLCPP_ERROR(get_logger(),
        "rendezvous_drain_window_sec (%.1f) >= rendezvous_latched_hold_sec "
        "(%.1f): every meeting would hit the cap before the drain could ever "
        "be evaluated.",
        rendezvous_drain_window_sec_, rendezvous_latched_hold_sec_);
    throw std::runtime_error(
        "rendezvous_drain_window_sec must be < rendezvous_latched_hold_sec");
  }
  if (!std::isfinite(rendezvous_appointment_wait_sec_) ||
      rendezvous_appointment_wait_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(),
        "rendezvous_appointment_wait_sec %.1f is not a usable cap -> 0 "
        "(wait without bound, the default).",
        rendezvous_appointment_wait_sec_);
    rendezvous_appointment_wait_sec_ = 0.0;
  }
  // Coupling: a positive appointment cap at or below the settle gives up before
  // the map-exchange hold can finish. Unreachable on the default (0 =
  // unbounded). (notes: appointment-wait-vs-settle)
  if (rendezvous_appointment_wait_sec_ > 0.0 &&
      rendezvous_appointment_wait_sec_ <= rendezvous_settle_sec_) {
    RCLCPP_WARN(get_logger(),
        "rendezvous_appointment_wait_sec (%.1f) <= rendezvous_settle_sec "
        "(%.1f): a robot that waits out the cap gives up before the map "
        "exchange hold can complete, so the meeting can never pay off.",
        rendezvous_appointment_wait_sec_, rendezvous_settle_sec_);
  }
  // This wait is also the scheduler's findability cap
  // (deriveRendezvousProposal), so unbounded means RendezvousPlan::capped is
  // false on every row by configuration; say so.
  // (notes: appointment-wait-pins-capped)
  if (rendezvous_appointment_wait_sec_ <= 0.0) {
    RCLCPP_INFO(get_logger(),
        "rendezvous_appointment_wait_sec=0: a robot at the agreed cell waits "
        "without bound, so the schedule's findability cap cannot bind and "
        "RendezvousPlan::capped is false for the whole run by construction.");
  }
  // Post-latch coast (see the member comments). OFF by default so this binary
  // reproduces every banked campaign bit-for-bit on the control side.
  done_seek_enabled_ = dp("done_seek_enabled", false);
  done_seek_max_sec_ = dp("done_seek_max_sec", 600.0);
  // Announced unconditionally, both directions, once per run, so a treated run
  // with no line and a control run with one are both detectable from the
  // console log. (notes: done-seek-announce-both-ways)
  RCLCPP_INFO(get_logger(),
      "DONE-SEEK %s (done_seek_enabled=%s, done_seek_max_sec=%.0f).",
      done_seek_enabled_ ? "ENABLED" : "DISABLED",
      done_seek_enabled_ ? "true" : "false", done_seek_max_sec_);
  if (done_seek_enabled_ && done_seek_max_sec_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
        "done_seek_enabled=true with done_seek_max_sec=%.1f: the coast has no "
        "upper bound but the no-progress exit still applies. An unreachable "
        "goal will drive this robot until it stops making headway.",
        done_seek_max_sec_);
  }
  // Mission return (see the member comments). OFF by default for the same
  // reason as the coast: this binary must reproduce banked behaviour exactly
  // unless the campaign explicitly opts in.
  mission_return_enabled_ = dp("mission_return_enabled", false);
  mission_home_tol_m_     = dp("mission_home_tol_m", 1.0);
  mission_return_max_sec_ = dp("mission_return_max_sec", 600.0);
  // Defaulted ON, unlike the two opt-in knobs above: those change what the
  // robot does, whereas this one closes a defect that split a fleet.
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
  // Requires return_approach_min_m of net approach per
  // return_approach_window_sec. Escapes are capped so the whole ladder stays
  // bounded well inside the homing cap (290 s worst case at defaults).
  // (notes: approach-watchdog-defaults)
  return_approach_window_sec_ = dp("return_approach_window_sec", 40.0);
  return_approach_min_m_      = dp("return_approach_min_m", 1.0);
  return_approach_suppress_m_ = dp("return_approach_suppress_m", 3.0);
  return_near_home_stall_sec_ = dp("return_near_home_stall_sec", 60.0);
  return_escape_max_attempts_ = dp("return_escape_max_attempts", 3);
  return_escape_leg_sec_      = dp("return_escape_leg_sec", 30.0);
  // Same unconditional both-directions announce contract as DONE-SEEK above:
  // every run states which side it is on, so a treated cell with no line and a
  // control cell with one are both detectable from the console log alone.
  RCLCPP_INFO(get_logger(),
      "MISSION-RETURN %s (mission_return_enabled=%s, mission_home_tol_m=%.1f, "
      "mission_return_max_sec=%.0f).",
      mission_return_enabled_ ? "ENABLED" : "DISABLED",
      mission_return_enabled_ ? "true" : "false",
      mission_home_tol_m_, mission_return_max_sec_);
  if (mission_return_enabled_ && done_seek_enabled_) {
    RCLCPP_WARN(get_logger(),
        "mission_return_enabled=true makes the done_seek coast unreachable: "
        "the mission-return branch pre-empts the coast gate at every terminal "
        "ending. done_seek_enabled=true is harmless but inert.");
  }
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
  reconnect_link_down_confirm_sec_ =
      dp("reconnect_link_down_confirm_sec", 0.0);
  if (reconnect_link_down_confirm_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(),
        "reconnect_link_down_confirm_sec=%.1f is negative; clamping to 0 "
        "(veto blocks only while the radio is actually UP).",
        reconnect_link_down_confirm_sec_);
    reconnect_link_down_confirm_sec_ = 0.0;
  }
  // Info gate (see the member comments). 0 = off, bit-identical legacy clock.
  reconnect_min_share_voxels_      = dp("reconnect_min_share_voxels", 0.0);
  reconnect_midrun_min_silence_sec_ =
      dp("reconnect_midrun_min_silence_sec", 60.0);
  // Defaulted to the RESOLVED clock, not to a literal: the ceiling is defined
  // as tracking reconnect_midrun_silence_sec, and expressing that as a copied
  // constant is what let it sit at 240 for a run whose clock had moved. An
  // explicit parameter still overrides, which is the deliberate opt-out.
  reconnect_midrun_max_silence_sec_ =
      dp("reconnect_midrun_max_silence_sec", reconnect_midrun_silence_sec_);
  // With the mid-run trigger off (clock 0) the derived ceiling falls below the
  // floor; pin it to the floor so the clamp never sees min > max. Inert in
  // practice. (notes: midrun-silence-ceiling-pin)
  if (reconnect_midrun_silence_sec_ <= 0.0 &&
      reconnect_midrun_max_silence_sec_ < reconnect_midrun_min_silence_sec_) {
    reconnect_midrun_max_silence_sec_ = reconnect_midrun_min_silence_sec_;
  }
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
  // Deliberately not conjoined with reconnect_min_share_voxels_ > 0:
  // ceiling/clock drift is worth reporting even with the info gate off.
  // (notes: midrun-ceiling-drift-warning)
  if (reconnect_midrun_max_silence_sec_ > reconnect_midrun_silence_sec_ &&
      reconnect_midrun_silence_sec_ > 0.0) {
    RCLCPP_WARN(get_logger(),
        "reconnect_midrun_max_silence_sec=%.0f exceeds the mid-run clock "
        "%.0f s. With the info gate ON a gated run could then wait LONGER "
        "than the ungated control, giving up the proven longest-outage cap; "
        "with it OFF this is inert but signals the two have drifted apart.",
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
  rendezvous_present_tol_m_      = dp("rendezvous_present_tol_m", 10.0);
  rendezvous_escape_max_attempts_ = dp("rendezvous_escape_max_attempts", 3);
  reconnect_nav_max_sec_         = dp("reconnect_nav_max_sec", 600.0);
  // rendezvous_present_tol_m must be >= reconnect_arrive_tol_m, or give-ups
  // close enough to count as arrival would roll. Clamped rather than
  // warned-and-kept: the inverted order has no valid A/B reading.
  // (notes: present-tol-vs-arrive-tol)
  if (!std::isfinite(rendezvous_present_tol_m_) ||
      rendezvous_present_tol_m_ < reconnect_arrive_tol_m_) {
    RCLCPP_WARN(get_logger(),
        "rendezvous_present_tol_m=%.2f is below reconnect_arrive_tol_m=%.2f: a "
        "robot cannot be too far from the cell to wait there and close enough "
        "to have arrived there. Clamping to %.2f.",
        rendezvous_present_tol_m_, reconnect_arrive_tol_m_,
        reconnect_arrive_tol_m_);
    rendezvous_present_tol_m_ = reconnect_arrive_tol_m_;
  }
  // A release window at or below the claim TTL is met by a single packet, so
  // the flicker guard is inert. Warn rather than clamp: 0 is a legitimate
  // legacy setting and the manifest must stay true.
  // (notes: release-confirm-vs-claim-ttl)
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
  // Below the ~180 s heartbeat-suppression tail the record-age clock cannot
  // tell a silent teammate from an absent one. Warn only if the subsystem is on
  // and the link veto lacks either topic.
  // (notes: midrun-silence-without-link-veto)
  if (reconnect_midrun_silence_sec_ > 0.0 && reconnect_enabled_ &&
      reconnect_midrun_silence_sec_ < 200.0 &&
      (comms_link_states_topic_.empty() ||
       comms_link_robot_index_topic_.empty())) {
    RCLCPP_WARN(get_logger(),
        "reconnect_midrun_silence_sec=%.0f is below the measured "
        "heartbeat-suppression tail (~180 s) AND %s, so the link veto cannot "
        "run: a healthy teammate stuck in a long PLAN loop can read as missing "
        "that long, and the trigger would drive a manoeuvre at a robot that is "
        "in range and fine. Either set BOTH link-gate topics or raise the "
        "threshold above 200.",
        reconnect_midrun_silence_sec_,
        comms_link_states_topic_.empty()
            ? "comms_link_states_topic is unset"
            : "comms_link_robot_index_topic is unset (so link rows cannot be "
              "matched to this robot)");
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

  // Exploitation circles each targets_topic target at n_vantages occlusion-free
  // vantages, exploit_dwell_sec each; success is min_vantages_required
  // clear-LoS dwells. Standoff = trunk radius + vantage_standoff_m,
  // FOV-clamped. (notes: exploit-vantage-ring-params)
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
  // Per-vantage capture barrier, ON by default: an early robot waits for the
  // peer before dwelling, so ring views of one trunk are simultaneous. Distinct
  // from the rendezvous_* comms-reconnection barrier.
  // (notes: exploit-dwell-sync-param)
  exploit_dwell_sync_enabled_ = dp("exploit_dwell_sync_enabled", true);
  exploit_dwell_sync_max_wait_sec_ =
      dp("exploit_dwell_sync_max_wait_sec", 0.0);
  std::string targets_topic = dp("targets_topic",
                                 std::string("/exploration/targets"));
  // The default topic is built absolute from robot_name_: this node is not
  // namespaced, so a relative default would never match the namespaced
  // scovox_node. An explicit param value is used verbatim.
  // (notes: refinement-region-topic-absolute)
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
    exp_log_->addParamStr("reconnect_mode", reconnectModeName(reconnect_mode_));
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
    exp_log_->addParamNum("coord_vantage_claim_radius_m",
                          coord_vantage_claim_radius_m_);
    exp_log_->addParamNum("cost_grid_radius_cap_m", cost_grid_radius_cap_m_);
    exp_log_->addParamNum("coord_claim_ttl_sec", coord_claim_ttl_sec_);
    exp_log_->addParamNum("coord_heartbeat_hz", coord_heartbeat_hz_);
    // Both names, one resolved value. `reconnect_enabled` is current;
    // `rendezvous_enabled` is kept verbatim and forever, so the cr3-cr5
    // analysis and the gate scripts read a new log exactly as they read an
    // old one, whichever name the harness passed.
    exp_log_->addParamBool("reconnect_enabled", reconnect_enabled_);
    exp_log_->addParamBool("rendezvous_enabled", reconnect_enabled_);
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
    // Logged on every arm, including those that do not use them.
    // rendezvous_depart_delay_sec is inert and logged only so the param rows
    // keep their schema. (notes: explog-rendezvous-arm-timing-params)
    exp_log_->addParamNum("rendezvous_depart_delay_sec",
                          rendezvous_depart_delay_sec_);
    exp_log_->addParamNum("rendezvous_appointment_wait_sec",
                          rendezvous_appointment_wait_sec_);
    exp_log_->addParamNum("rendezvous_settle_sec", rendezvous_settle_sec_);
    // Generation 33's replacement for that trigger. Logged unconditionally and
    // with its thresholds, because a cell whose hold ended on the drain and a
    // cell whose hold ended on the clock are two different arms, and the
    // thresholds are what make two drain cells comparable to each other.
    exp_log_->addParamNum("rendezvous_drain_release",
                          rendezvous_drain_release_ ? 1.0 : 0.0);
    exp_log_->addParamNum("rendezvous_drain_rate_vox_sec",
                          rendezvous_drain_rate_vox_sec_);
    exp_log_->addParamNum("rendezvous_drain_window_sec",
                          rendezvous_drain_window_sec_);
    // Generation 29's addition to that definition, and it is not optional
    // bookkeeping: it decides which rung of the agreed lattice each robot signs
    // up to, so two cells with different values are two different rendezvous
    // arms. Same unconditional logging as the rest of the family.
    exp_log_->addParamNum("rendezvous_max_lateness_sec",
                          rendezvous_max_lateness_sec_);
    exp_log_->addParamNum("reconnect_midrun_max_attempts",
                          reconnect_midrun_max_attempts_);
    exp_log_->addParamNum("reconnect_min_share_voxels",
                          reconnect_min_share_voxels_);
    exp_log_->addParamNum("reconnect_midrun_min_silence_sec",
                          reconnect_midrun_min_silence_sec_);
    exp_log_->addParamNum("reconnect_midrun_max_silence_sec",
                          reconnect_midrun_max_silence_sec_);
    // Logged because an unset comms_link topic makes linkGateReady() return
    // false and silently removes the link veto from the reconnect trigger.
    // (notes: explog-link-veto-topics)
    exp_log_->addParamNum("reconnect_link_down_confirm_sec",
                          reconnect_link_down_confirm_sec_);
    exp_log_->addParamStr("comms_link_states_topic", comms_link_states_topic_);
    exp_log_->addParamStr("comms_link_robot_index_topic",
                          comms_link_robot_index_topic_);
    // Named configured, not active: true when both topics are set, written at
    // startRun before any sample. Liveness is the one-shot link_gate_live
    // console line. States without index stands down every tick.
    // (notes: explog-link-gate-configured-not-active)
    exp_log_->addParamBool("link_gate_configured",
                           !comms_link_states_topic_.empty() &&
                               !comms_link_robot_index_topic_.empty());
    exp_log_->addParamNum("comms_link_stale_sec", comms_link_stale_sec_);
    exp_log_->addParamNum("pursuit_budget_max_sec", pursuit_budget_max_sec_);
    // Echoed beside the cap because the two together are the refusal
    // threshold: a chase is refused past cap * speed metres, and neither
    // number says that on its own.
    exp_log_->addParamNum("pursuit_speed_measured_mps",
                          pursuit_speed_measured_mps_);
    exp_log_->addParamNum("pursuit_staleness_max_sec",
                          pursuit_staleness_max_sec_);
    exp_log_->addParamNum("pursuit_goal_stale_sec", pursuit_goal_stale_sec_);
    exp_log_->addParamBool("pursuit_explore_fallback",
                           pursuit_explore_fallback_);
    exp_log_->addParamNum("pursuit_explore_max", pursuit_explore_max_);
    exp_log_->addParamBool("hold_escalate", hold_escalate_);
    exp_log_->addParamNum("hold_escalate_wait_sec", hold_escalate_wait_sec_);
    exp_log_->addParamNum("reconnect_arrive_tol_m", reconnect_arrive_tol_m_);
    exp_log_->addParamNum("rendezvous_present_tol_m", rendezvous_present_tol_m_);
    exp_log_->addParamNum("rendezvous_escape_max_attempts",
                          rendezvous_escape_max_attempts_);
    exp_log_->addParamNum("reconnect_nav_max_sec", reconnect_nav_max_sec_);
    exp_log_->addParamNum("done_unknown_fraction", done_unknown_fraction_);
    exp_log_->addParamNum("done_min_consecutive_steps",
                          done_min_consecutive_steps_);
    exp_log_->addParamStr("done_coverage_source", done_coverage_source_);
    // Which rule ended the run. Recorded because it is not recoverable from any
    // other field, and a campaign that mixes the two criteria is comparing two
    // different endpoints under one column name.
    exp_log_->addParamStr("done_criterion", done_criterion_);
    // Stamped unconditionally so the arm is recoverable from the run's own
    // params rather than from a campaign script that may have moved on.
    exp_log_->addParamBool("done_seek_enabled", done_seek_enabled_);
    exp_log_->addParamNum("done_seek_max_sec", done_seek_max_sec_);
    exp_log_->addParamStr("done_action", done_action_);
    exp_log_->addParamBool("mission_return_enabled", mission_return_enabled_);
    exp_log_->addParamNum("mission_home_tol_m", mission_home_tol_m_);
    exp_log_->addParamNum("mission_return_max_sec", mission_return_max_sec_);
    // Homing watchdog and failed-goal blacklist knobs, logged because the
    // analysis cannot infer them. (notes: explog-homing-blacklist-knobs)
    exp_log_->addParamNum("return_approach_window_sec",
                          return_approach_window_sec_);
    exp_log_->addParamNum("return_approach_min_m", return_approach_min_m_);
    exp_log_->addParamNum("return_approach_suppress_m",
                          return_approach_suppress_m_);
    exp_log_->addParamNum("return_near_home_stall_sec",
                          return_near_home_stall_sec_);
    exp_log_->addParamNum("return_escape_max_attempts",
                          return_escape_max_attempts_);
    exp_log_->addParamNum("return_escape_leg_sec", return_escape_leg_sec_);
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

    // LoS checks ray-march at vcfg.robot_z through map_cache_, clipped to
    // [roi_min_z_, roi_max_z_]; outside that band they pass trivially, so warn.
    // Terrain mode skips this: exploitZAt() snaps sightlines to ground.
    // (notes: exploit-los-sightline-z-band)
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

  // --- TeamWorld exchange + comms model (P2, plan §3.2/§3.3) --------------
  // team_world_hz is the switch; 0.0, the default, creates nothing. Cell world
  // and fleet identity are hard prerequisites (fatal, not degraded).
  // (notes: teamworld-p2-switch-prereqs)
  team_world_hz_ = dp("team_world_hz", 0.0);
  // Separation reads peer positions only from the comms model, which only
  // TeamWorld feeds; with the exchange off the term would be silently inert.
  // Fatal, not a warning. (notes: separation-requires-teamworld)
  if (separation_.enabled() && !(team_world_hz_ > 0.0)) {
    const std::string e =
        "separation_weight > 0 requires team_world_hz > 0: peer positions "
        "reach the planner only through the TeamWorld exchange, so with it "
        "off the separation term would run with no teammate to be repelled "
        "by and silently reproduce the untreated planner.";
    RCLCPP_FATAL(get_logger(), "%s", e.c_str());
    throw std::runtime_error(e);
  }
  if (team_world_hz_ > 0.0) {
    if (!cell_world_enable_) {
      RCLCPP_FATAL(get_logger(),
          "team_world_hz=%.3f but cell_world_enable=false: the TeamWorld "
          "message IS the cell census, so there would be nothing to send.",
          team_world_hz_);
      throw std::runtime_error("team_world_hz requires cell_world_enable");
    }
    const std::string idw = requireFleetIdentity(fleet_, "team_world_hz");
    if (!idw.empty()) {
      RCLCPP_FATAL(get_logger(), "%s", idw.c_str());
      throw std::runtime_error(idw);
    }
    TeamModel::Config tmc;
    // Defaults to the intent-claim TTL (coord_claim_ttl_sec_) so peer_lost and
    // LOST_COMMS judge liveness on one clock and cannot disagree about the same
    // outage. (notes: teamworld-comms-ttl-default)
    tmc.direct_ttl_sec     = dp("team_comms_ttl_sec", coord_claim_ttl_sec_);
    tmc.closure_enabled    = dp("team_comms_closure", true);
    tmc.gossip_max_age_sec = dp("team_gossip_max_age_sec", 120.0);
    const std::string terr = team_model_.configure(fleet_, tmc);
    if (!terr.empty()) {
      RCLCPP_FATAL(get_logger(), "team_model: %s", terr.c_str());
      throw std::runtime_error("team_model: " + terr);
    }
    // mTARE's local-priority rule: while this robot is standing in a cell and
    // exploring it, a peer's claim about that cell is refused. On by default
    // because the alternative — accepting it — silently credits the peer with
    // knowing a status we are about to change.
    team_merge_local_priority_ = dp("cell_merge_local_priority", true);

    // Split pub/sub, mirroring the intent stream: the comms emulator can gate a
    // stream per link only when publisher and subscribers use different topics.
    // A shared bus shows perfect comms. (notes: teamworld-split-pub-sub)
    team_world_pub_topic_ =
        resolve_topic(dp("team_world_pub_topic", std::string("")));
    if (team_world_pub_topic_.empty())
      team_world_pub_topic_ = "/exploration/team_world";
    team_world_sub_topics_ =
        dp("team_world_sub_topics", std::vector<std::string>{});
    for (auto& t : team_world_sub_topics_) t = resolve_topic(t);
    team_world_sub_topics_.erase(
        std::remove(team_world_sub_topics_.begin(),
                    team_world_sub_topics_.end(), ""),
        team_world_sub_topics_.end());
    if (team_world_sub_topics_.empty())
      team_world_sub_topics_ = {team_world_pub_topic_};
  }
  if (exp_log_) {
    // Written even when off, for the reason the cell-world block is: a run
    // record must say the feature EXISTED and was disabled, or an equivalence
    // control run is indistinguishable from a run of a binary that never had
    // the knob.
    exp_log_->addParamNum("team_world_hz", team_world_hz_);
    if (team_world_hz_ > 0.0) {
      exp_log_->addParamNum("team_comms_ttl_sec",
                            team_model_.config().direct_ttl_sec);
      exp_log_->addParamBool("team_comms_closure",
                             team_model_.config().closure_enabled);
      exp_log_->addParamNum("team_gossip_max_age_sec",
                            team_model_.config().gossip_max_age_sec);
      exp_log_->addParamBool("cell_merge_local_priority",
                             team_merge_local_priority_);
      exp_log_->addParamStr("team_world_pub_topic", team_world_pub_topic_);
      std::string subs;
      for (const auto& t : team_world_sub_topics_)
        subs += (subs.empty() ? "" : ",") + t;
      exp_log_->addParamStr("team_world_sub_topics", subs);
    }
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
    // Without the TeamWorld exchange every robot allocates the whole map to
    // itself while its allocation events look healthy. Fatal, because
    // downstream cannot catch it. (notes: alloc-p3-requires-teamworld)
    if (team_world_hz_ <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "global_alloc_enable=true but team_world_hz=0: with no exchange "
          "there is no shared cell world to solve over, so the allocator "
          "would divide the map among a fleet of one.");
      throw std::runtime_error("global_alloc_enable requires team_world_hz>0");
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

  // ---- Utility-gated reconnection (P4) --------------------------------
  // With silence the mid-run silence clock decides alone; info adds a knowledge
  // + value gate that only suppresses dispatches the clock allowed. Not
  // midrunGateSec()'s gate. (notes: reconnect-gate-p4-silence-vs-info)
  {
    const std::string rg = dp("reconnect_gate", std::string("silence"));
    if (rg == "silence") {
      reconnect_gate_info_ = false;
    } else if (rg == "info") {
      reconnect_gate_info_ = true;
    } else {
      const std::string msg =
          "reconnect_gate='" + rg + "' is not one of {silence, info}";
      RCLCPP_FATAL(get_logger(), "%s", msg.c_str());
      throw std::runtime_error(msg);
    }
  }
  if (reconnect_gate_info_) {
    if (!cell_world_enable_) {
      RCLCPP_FATAL(get_logger(),
          "reconnect_gate=info but cell_world_enable=false: the knowledge gate "
          "reads cell statuses and known_by masks, and the value gate solves "
          "the allocation problem. Neither exists without the cell world.");
      throw std::runtime_error("reconnect_gate=info requires cell_world_enable");
    }
    const std::string idw = requireFleetIdentity(fleet_, "reconnect_gate=info");
    if (!idw.empty()) {
      RCLCPP_FATAL(get_logger(), "%s", idw.c_str());
      throw std::runtime_error(idw);
    }
    // Without the exchange known_by stays {self} on every cell, so the
    // knowledge gate is vacuously true while logging healthy verdicts. Fatal,
    // as in P3. (notes: reconnect-gate-requires-teamworld)
    if (team_world_hz_ <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "reconnect_gate=info but team_world_hz=0: with no exchange no peer "
          "ever enters a known_by mask, so the knowledge gate would answer "
          "'they know nothing' on every cell for the entire run.");
      throw std::runtime_error("reconnect_gate=info requires team_world_hz>0");
    }
    // Not fatal: a run with reconnection disabled entirely is a legitimate
    // control, and the gate simply never evaluates. Warned because asking for
    // `info` in that configuration is almost certainly a harness mistake.
    if (!reconnect_enabled_) {
      RCLCPP_WARN(get_logger(),
          "reconnect_gate=info with reconnect_enabled=false: there is no "
          "mid-run reconnection to gate, so the gate will never evaluate.");
    }
    // The gate prices futures with the allocator's solver, so it needs
    // alloc_cfg_ even with the allocator off. Read the params only if P3 did
    // not (declaring twice throws); comms_mask is set per solve.
    // (notes: reconnect-gate-alloc-cfg-params)
    if (!global_alloc_enable_) {
      alloc_cfg_.polish_passes  = dp("global_alloc_polish_passes", 2);
      alloc_cfg_.max_candidates = dp("global_alloc_max_candidates", 256);
    }
  }
  if (exp_log_) {
    exp_log_->addParamStr("reconnect_gate",
                          reconnect_gate_info_ ? "info" : "silence");
  }

  // ---- Scheduled rendezvous (P5) --------------------------------------
  // The meeting is a constraint on the tours being driven: pick the cell whose
  // forced insertion costs the team's makespan least, and meet when the slower
  // robot reaches it. (notes: rzv-schedule-p5-meeting-constraint)
  rendezvous_schedule_enable_ = dp("rendezvous_schedule_enable", false);
  // ---- THE INTERLOCK (2026-09-16) -------------------------------------
  // With reconnect enabled, rendezvous or hybrid mode requires
  // rendezvous_schedule_enable, else rendezvous silently acts as the off arm
  // and hybrid as pursuit. Fatal at startup; neither shows afterwards.
  // (notes: rzv-schedule-interlock)
  if (reconnect_enabled_ && !rendezvous_schedule_enable_ &&
      (reconnect_mode_ == ReconnectMode::RENDEZVOUS ||
       reconnect_mode_ == ReconnectMode::HYBRID)) {
    RCLCPP_FATAL(get_logger(),
        "reconnect_mode='%s' with rendezvous_schedule_enable=false: this arm "
        "IS the agreed meeting (place AND time), and without the scheduler "
        "there is no agreement to make — %s. The cell would run to completion "
        "looking healthy while carrying no treatment and scoring as a "
        "different arm. Set rendezvous_schedule_enable=true, or run the off "
        "arm with reconnect_enabled=false.",
        reconnectModeName(reconnect_mode_),
        reconnect_mode_ == ReconnectMode::RENDEZVOUS
            ? "rendezvous would never arm an appointment and would behave as "
              "the off arm"
            : "hybrid would lose its rendezvous half and behave as the "
              "pursuit arm");
    throw std::runtime_error(
        std::string("reconnect_mode=") + reconnectModeName(reconnect_mode_) +
        " requires rendezvous_schedule_enable=true");
  }
  if (rendezvous_schedule_enable_) {
    // The problem IS the cell world, same as P3/P4, and for the same reason
    // those two are fatal rather than degrading: a scheduler with no world
    // would refuse on every arming and log a perfectly healthy stream of
    // refusals that reads like a mechanism deciding not to fire.
    if (!cell_world_enable_) {
      RCLCPP_FATAL(get_logger(),
          "rendezvous_schedule_enable=true but cell_world_enable=false: the "
          "meeting is chosen by inserting a cell into the solved tours, and "
          "neither the cells nor the tours exist without the cell world.");
      throw std::runtime_error(
          "rendezvous_schedule_enable requires cell_world_enable");
    }
    const std::string idw =
        requireFleetIdentity(fleet_, "rendezvous_schedule_enable=true");
    if (!idw.empty()) {
      RCLCPP_FATAL(get_logger(), "%s", idw.c_str());
      throw std::runtime_error(idw);
    }
    // Without the TeamWorld exchange each robot would derive a private
    // appointment and drive to a different cell while logging a well-formed
    // rendezvous_agreed. Fatal. (notes: rzv-schedule-requires-teamworld)
    if (team_world_hz_ <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "rendezvous_schedule_enable=true but team_world_hz=0: with no world "
          "exchange each robot would solve a private allocation and derive a "
          "private appointment, and the two ends would meet nowhere.");
      throw std::runtime_error(
          "rendezvous_schedule_enable requires team_world_hz>0");
    }
    // Not fatal: a control run with reconnection off is legitimate and the
    // scheduler simply never arms. Warned because asking for it there is
    // almost certainly a harness mistake.
    if (!reconnect_enabled_) {
      RCLCPP_WARN(get_logger(),
          "rendezvous_schedule_enable=true with reconnect_enabled=false: "
          "there is no reconnect manoeuvre to schedule, so no appointment "
          "will ever be armed.");
    }
    // Pure pursuit is deliberately untouched — see rendezvous_schedule_enable_.
    if (reconnect_mode_ == ReconnectMode::PURSUIT) {
      RCLCPP_WARN(get_logger(),
          "rendezvous_schedule_enable=true under reconnect_mode=pursuit: pure "
          "pursuit has no agreed meeting point by design (that absence is the "
          "A/B against hybrid), so the scheduler will not arm in this arm.");
    }
    // The scheduler solves the allocation problem itself, so it needs a
    // Config even when the allocator and the §3.6 gate are both off.
    // Declaring a ROS parameter twice throws, so read them only where neither
    // of the two blocks above already did.
    if (!global_alloc_enable_ && !reconnect_gate_info_) {
      alloc_cfg_.polish_passes  = dp("global_alloc_polish_passes", 2);
      alloc_cfg_.max_candidates = dp("global_alloc_max_candidates", 256);
    }

    // The schedule's honest cruise speed, deliberately not nav_speed_est_mps_
    // (conservative; would push t_meet too late). Must be identical on all
    // robots: both ends derive values from it.
    // (notes: rzv-schedule-speed-not-nav-est)
    rzv_cfg_.speed_mm_s =
        static_cast<long long>(dp("rendezvous_speed_mm_s", 500));
    // Safety multiplier that marks up the reachability floor in
    // RendezvousScheduler::solve. Fleet-wide, so it MUST match across the team
    // or the derived interval differs between robots.
    // (notes: rzv-schedule-depart-safety)
    rzv_cfg_.depart_safety_milli = static_cast<int>(
        std::lround(dp("rendezvous_depart_safety", 1.2) * 1000.0));
    if (rzv_cfg_.speed_mm_s <= 0) {
      RCLCPP_FATAL(get_logger(),
          "rendezvous_speed_mm_s=%lld: every arrival time would be infinite "
          "and there is no safe value to substitute.", rzv_cfg_.speed_mm_s);
      throw std::runtime_error("rendezvous_speed_mm_s must be positive");
    }

    // Fleet id 0 derives the pair and everyone else echoes it; the handshake
    // runs only while the team reads complete. So rendezvous_expected_peers
    // must equal fleet size - 1, or robot 0 may be left out.
    // (notes: rzv-schedule-proposer-rule)
    if (rendezvous_expected_peers_ != fleet_.size() - 1) {
      RCLCPP_FATAL(get_logger(),
          "rendezvous_schedule_enable=true with rendezvous_expected_peers=%d "
          "but a fleet of %d: the proposal handshake runs only while the team "
          "reads complete and assumes that means every robot, so a partial "
          "quorum could exclude the proposer (fleet id 0) and no appointment "
          "would ever be agreed.",
          rendezvous_expected_peers_, fleet_.size());
      throw std::runtime_error(
          "rendezvous_schedule_enable requires rendezvous_expected_peers == "
          "team size - 1");
    }
    rendezvous_peer_.assign(static_cast<size_t>(fleet_.size()),
                            RendezvousProposal{});
    rendezvous_peer_at_sec_.assign(static_cast<size_t>(fleet_.size()), -1.0);
    rendezvous_peer_provisional_.assign(static_cast<size_t>(fleet_.size()), 0);
    rendezvous_peer_confirmed_.assign(static_cast<size_t>(fleet_.size()),
                                      RendezvousProposal{});

    // How often the proposer may issue a new pair. Each re-issue costs one
    // TeamWorld period of disagreement, so this trades agreement probability
    // against meeting-cell staleness. (notes: rzv-schedule-proposal-period)
    rendezvous_proposal_period_sec_ =
        dp("rendezvous_proposal_period_sec", 30.0);
    if (rendezvous_proposal_period_sec_ <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "rendezvous_proposal_period_sec=%.3f: a non-positive period re-derives "
          "the pair on every heartbeat, so the peers can never finish echoing "
          "one and nothing is ever agreed.",
          rendezvous_proposal_period_sec_);
      throw std::runtime_error(
          "rendezvous_proposal_period_sec must be positive");
    }

    // The spacing of the timetable, and deliberately NOT the period above: see
    // the member for why widening the recurrence through the proposal period
    // would age the snapshot the punctuality estimate is costed against.
    rendezvous_interval_sec_ = dp("rendezvous_interval_sec", 300.0);
    if (!std::isfinite(rendezvous_interval_sec_) ||
        rendezvous_interval_sec_ <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "rendezvous_interval_sec=%.3f: the timetable spacing is the gap "
          "between two legal meeting instants and a non-positive one names no "
          "timetable at all — every occurrence would fall at the same instant, "
          "and a robot that missed it would have no later rung to roll to.",
          rendezvous_interval_sec_);
      throw std::runtime_error("rendezvous_interval_sec must be positive");
    }

    // The handshake (refreshRendezvousSnapshot, maintainRendezvousProposal)
    // runs only from heartbeatTick, which exists only with coord_enabled_ and
    // coord_heartbeat_hz > 0. Without it nothing is agreed.
    // (notes: rzv-schedule-requires-heartbeat)
    if (!coord_enabled_ || coord_heartbeat_hz_ <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "rendezvous_schedule_enable=true with coord_enable=%s and "
          "coord_heartbeat_hz=%.3f: the propose/echo/commit handshake runs "
          "only on the coordination heartbeat, so no pair could ever be "
          "agreed and every appointment would refuse.",
          coord_enabled_ ? "true" : "false", coord_heartbeat_hz_);
      throw std::runtime_error(
          "rendezvous_schedule_enable requires coord_enable with a positive "
          "coord_heartbeat_hz");
    }

    // The agreed pair travels only in TeamWorld, and rendezvousTeamMutual()
    // reads only the TeamModel that TeamWorld feeds. With team_world_hz 0 (the
    // default) nothing is agreed or anchored.
    // (notes: rzv-schedule-teamworld-only-wire)
    if (team_world_hz_ <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "rendezvous_schedule_enable=true with team_world_hz=%.3f: the agreed "
          "(cell, interval) pair is exchanged only in TeamWorld, and the "
          "mutual-contact anchor is read only from the model TeamWorld feeds. "
          "With the exchange off nothing is agreed, nothing is anchored, and "
          "every appointment refuses — a silently null rendezvous arm.",
          team_world_hz_);
      throw std::runtime_error(
          "rendezvous_schedule_enable requires a positive team_world_hz");
    }

    // The commit latches confirmations with no TTL, but mutual contact
    // (TeamModel direct, rendezvousTeamMutual(), the anchor) runs on
    // direct_ttl_sec. Warn, not fatal, if under 3 TeamWorld messages fit in it.
    // (notes: rzv-schedule-mutual-contact-ttl)
    const double mutual_ttl = team_model_.config().direct_ttl_sec;
    if (team_world_hz_ > 0.0 && mutual_ttl > 0.0 &&
        team_world_hz_ * mutual_ttl < 3.0) {
      RCLCPP_WARN(get_logger(),
          "rendezvous: team_world_hz=%.3f against a direct-contact TTL of "
          "%.1f s leaves only %.1f message(s) inside the window mutual contact "
          "is judged on. Expect the team never to read whole, no pair to be "
          "derived, and every appointment to refuse with 'no (cell, interval) "
          "pair was agreed'.",
          team_world_hz_, mutual_ttl, team_world_hz_ * mutual_ttl);
    }

    RCLCPP_INFO(get_logger(),
        "rendezvous: proposer is fleet id 0 (%s); this robot is id %d (%s), "
        "so it %s. Re-proposal period %.1f s.",
        fleet_.nameOf(0).c_str(), fleet_.self_id, robot_name_.c_str(),
        fleet_.self_id == 0 ? "DERIVES the pair" : "ECHOES the proposer's pair",
        rendezvous_proposal_period_sec_);
  }
  if (exp_log_) {
    exp_log_->addParamBool("rendezvous_schedule_enable",
                           rendezvous_schedule_enable_);
    if (rendezvous_schedule_enable_) {
      exp_log_->addParamNum("rendezvous_speed_mm_s",
                            static_cast<double>(rzv_cfg_.speed_mm_s));
      exp_log_->addParamNum("rendezvous_depart_safety",
                            rzv_cfg_.depart_safety_milli / 1000.0);
      exp_log_->addParamNum("rendezvous_proposal_period_sec",
                            rendezvous_proposal_period_sec_);
      exp_log_->addParamNum("rendezvous_interval_sec",
                            rendezvous_interval_sec_);
    }
  }

  // ---- MDP interception (P6, §3.7) ------------------------------------
  // trail is the shipped default chase; mdp aims the first waypoint at the
  // intercept. A string, not a bool, so the log names which aiming rule ran.
  // (notes: pursuit-mdp-p6-predictor-string)
  {
    const std::string pp = dp("pursuit_predictor", std::string("trail"));
    if (pp == "mdp") {
      pursuit_predictor_mdp_ = true;
    } else if (pp != "trail") {
      RCLCPP_FATAL(get_logger(),
          "pursuit_predictor='%s': expected 'trail' or 'mdp'. Refusing to "
          "start rather than silently running the default — a typo here is a "
          "whole arm that measured the control.", pp.c_str());
      throw std::runtime_error("pursuit_predictor must be 'trail' or 'mdp'");
    }
  }
  if (pursuit_predictor_mdp_) {
    // Hard requirements, fatal: otherwise the predictor refuses every dispatch
    // while looking configured. Tours are only the allocator's output, so mdp
    // needs global_alloc_enable. (notes: pursuit-mdp-hard-requirements)
    if (!global_alloc_enable_) {
      RCLCPP_FATAL(get_logger(),
          "pursuit_predictor=mdp but global_alloc_enable=false: the tour the "
          "model propagates is the allocator's output and nothing else "
          "produces one, so no robot would ever broadcast a route and every "
          "chase would silently fall back to the trail.");
      throw std::runtime_error("pursuit_predictor=mdp requires "
                               "global_alloc_enable");
    }
    if (!cell_world_enable_) {
      RCLCPP_FATAL(get_logger(),
          "pursuit_predictor=mdp but cell_world_enable=false: a tour is a "
          "list of cells and the chain walks it through the cell grid.");
      throw std::runtime_error("pursuit_predictor=mdp requires "
                               "cell_world_enable");
    }
    // And they have to TRAVEL. my_tour rides on TeamWorld and on nothing else
    // — it is deliberately not gossiped (TeamWorld.msg), so a peer heard only
    // through a relay has no tour on record and degrades to the trail. With
    // the exchange off outright, every peer is in that position.
    if (team_world_hz_ <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "pursuit_predictor=mdp but team_world_hz=0: tours ride on TeamWorld, "
          "so no robot would ever hear one and the model would refuse every "
          "chase it was asked about.");
      throw std::runtime_error("pursuit_predictor=mdp requires "
                               "team_world_hz>0");
    }
    // Not fatal: a control run with reconnection off is legitimate and the
    // predictor is simply never consulted. Warned because asking for it there
    // is almost certainly a harness mistake.
    if (!reconnect_enabled_) {
      RCLCPP_WARN(get_logger(),
          "pursuit_predictor=mdp with reconnect_enabled=false: there is no "
          "chase to aim, so the model will never be consulted.");
    }
    if (reconnect_mode_ == ReconnectMode::RENDEZVOUS) {
      RCLCPP_WARN(get_logger(),
          "pursuit_predictor=mdp under reconnect_mode=rendezvous: that mode "
          "never chases, so the model will never be consulted.");
    }

    // The speed the prediction assumes, deliberately not nav_speed_est_mps_
    // (conservative; the budget still uses it). Same quantity as
    // rendezvous_speed_mm_s, kept separate so P6 runs with P5 off.
    // (notes: pursuit-mdp-speed-not-nav-est)
    pursuit_mdp_cfg_.my_speed_mps = dp("pursuit_mdp_speed_mps", 0.5);
    if (pursuit_mdp_cfg_.my_speed_mps <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "pursuit_mdp_speed_mps=%.3f: every travel time would be infinite and "
          "there is no safe value to substitute.",
          pursuit_mdp_cfg_.my_speed_mps);
      throw std::runtime_error("pursuit_mdp_speed_mps must be positive");
    }
    if (rendezvous_schedule_enable_ &&
        std::fabs(pursuit_mdp_cfg_.my_speed_mps -
                  static_cast<double>(rzv_cfg_.speed_mm_s) / 1000.0) > 1e-6) {
      RCLCPP_WARN(get_logger(),
          "pursuit_mdp_speed_mps=%.3f but rendezvous_speed_mm_s=%lld (%.3f "
          "m/s): the same robot is modelled at two speeds. Deliberate is fine "
          "— sharing the mistake is what this warns about.",
          pursuit_mdp_cfg_.my_speed_mps, rzv_cfg_.speed_mm_s,
          static_cast<double>(rzv_cfg_.speed_mm_s) / 1000.0);
    }
    pursuit_mdp_peer_speed_mps_ = dp("pursuit_mdp_peer_speed_mps", -1.0);
    pursuit_mdp_cfg_.peer_speed_mps = pursuit_mdp_peer_speed_mps_ > 0.0
        ? pursuit_mdp_peer_speed_mps_
        : pursuit_mdp_cfg_.my_speed_mps;

    pursuit_mdp_cfg_.dwell_sec = dp("pursuit_mdp_dwell_sec", 45.0);
    pursuit_mdp_cfg_.step_sec  = dp("pursuit_mdp_step_sec", 5.0);
    pursuit_mdp_cfg_.offroute_half_life_sec =
        dp("pursuit_mdp_offroute_half_life_sec", 180.0);
    pursuit_mdp_cfg_.max_horizon_sec =
        dp("pursuit_mdp_max_horizon_sec", 600.0);
    pursuit_mdp_cfg_.min_probability =
        dp("pursuit_mdp_min_probability", 0.10);
    if (pursuit_mdp_cfg_.step_sec <= 0.0) {
      RCLCPP_FATAL(get_logger(),
          "pursuit_mdp_step_sec=%.3f: the chain cannot advance.",
          pursuit_mdp_cfg_.step_sec);
      throw std::runtime_error("pursuit_mdp_step_sec must be positive");
    }
    // A zero floor is a legitimate diagnostic setting (it makes every chase
    // MDP-aimed, which is how you measure what the floor is buying) but it is
    // not a legitimate campaign setting, because it removes the fall-through
    // that makes "never worse than the trail" true.
    if (pursuit_mdp_cfg_.min_probability <= 0.0) {
      RCLCPP_WARN(get_logger(),
          "pursuit_mdp_min_probability=%.3f: the model will aim every chase, "
          "including ones with no information behind them. The fall-through to "
          "the trail is what bounds this arm's downside.",
          pursuit_mdp_cfg_.min_probability);
    }
  }
  if (exp_log_) {
    exp_log_->addParamStr("pursuit_predictor",
                          pursuit_predictor_mdp_ ? "mdp" : "trail");
    if (pursuit_predictor_mdp_) {
      exp_log_->addParamNum("pursuit_mdp_speed_mps",
                            pursuit_mdp_cfg_.my_speed_mps);
      exp_log_->addParamNum("pursuit_mdp_peer_speed_mps",
                            pursuit_mdp_peer_speed_mps_);
      exp_log_->addParamNum("pursuit_mdp_dwell_sec",
                            pursuit_mdp_cfg_.dwell_sec);
      exp_log_->addParamNum("pursuit_mdp_step_sec", pursuit_mdp_cfg_.step_sec);
      exp_log_->addParamNum("pursuit_mdp_offroute_half_life_sec",
                            pursuit_mdp_cfg_.offroute_half_life_sec);
      exp_log_->addParamNum("pursuit_mdp_max_horizon_sec",
                            pursuit_mdp_cfg_.max_horizon_sec);
      exp_log_->addParamNum("pursuit_mdp_min_probability",
                            pursuit_mdp_cfg_.min_probability);
    }
  }

  // The arm analyses group by, stamped here downstream of every flag it reads.
  // reconnect_enabled=false is off; any of P3-P6 adds the mtare_ prefix and mdp
  // adds _mdp, matching the harness arm tokens. (notes: explog-arm-stamp)
  if (exp_log_) {
    const bool mtare = global_alloc_enable_ || reconnect_gate_info_ ||
                       rendezvous_schedule_enable_ || pursuit_predictor_mdp_;
    std::string arm = reconnect_enabled_
                          ? std::string(reconnectModeName(reconnect_mode_))
                          : std::string("off");
    if (mtare) arm = "mtare_" + arm;
    if (pursuit_predictor_mdp_) arm += "_mdp";
    exp_log_->addParamStr("arm", arm);
  }

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
          // Rendezvous anchor: record our pose when a teammate is heard; it
          // lies inside the comms bubble. Gated on robot_id because it reads
          // our live pose; have_pose_ guards the first ticks before TF.
          // (notes: ctor-intent-rendezvous-anchor)
          if (reconnect_enabled_ && have_pose_ &&
              msg->robot_id != robot_name_) {
            last_connected_anchor_ = latest_pos_;
            have_anchor_ = true;
            // Keep the whole last-contact pair per peer (my pose, its
            // advertised pose, its declared goal), since both endpoints move.
            // Stamped with local receipt time, like claim expiry.
            // (notes: ctor-intent-last-contact-pair)
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
    const std::string subs = join(coord_intent_sub_topics_, ", ");
    RCLCPP_INFO(get_logger(),
        "Intents: pub '%s' <- KeepLast(8).reliable() -> sub [%s]%s",
        coord_intent_pub_topic_.c_str(), subs.c_str(),
        (coord_intent_sub_topics_.size() == 1 &&
         coord_intent_sub_topics_[0] == coord_intent_pub_topic_)
            ? " (shared bus: no external process can gate this stream)" : "");
  }

  // --- TeamWorld pub/sub. Nothing is created when the switch is off, so a
  //     defaults run does not even advertise the topic.
  if (team_world_hz_ > 0.0) {
    // KeepLast(2): each message is full state, so a deeper queue only merges
    // stale censuses. Two, not one, so a message arriving during a planning
    // pass survives until the copy-only callback runs.
    // (notes: ctor-teamworld-qos-keeplast-two)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    team_world_pub_ = create_publisher<explo_planner_msgs::msg::TeamWorld>(
        team_world_pub_topic_, qos);
    auto on_team_world =
        [this](explo_planner_msgs::msg::TeamWorld::SharedPtr msg) {
          // COPY ONLY. Everything that could take time — the config check, the
          // merge, the comms recompute, the event — happens in drainTeamWorld
          // on the tick. See PendingTeamWorld.
          const int sid = static_cast<int>(msg->robot_id);
          // Our own broadcast, looped back by a shared bus. Dropped here
          // rather than in the merge so it never occupies the single pending
          // slot a real peer would have used.
          if (sid == fleet_.self_id) return;
          auto& slot = team_world_pending_[sid];
          if (slot.msg) ++slot.superseded;
          slot.msg      = msg;
          slot.received = this->now();
        };
    for (const auto& topic : team_world_sub_topics_) {
      team_world_subs_.push_back(
          create_subscription<explo_planner_msgs::msg::TeamWorld>(
              topic, qos, on_team_world));
    }
    // Own timer, not the planning tick, so a busy planner does not go silent to
    // peers. Must be the SIM clock: direct_ttl_sec is measured in sim seconds
    // off this->now(). (notes: ctor-teamworld-sim-clock-timer)
    const std::chrono::duration<double> period(1.0 / team_world_hz_);
    team_world_timer_ = rclcpp::create_timer(
        this, get_clock(), period, [this]() { publishTeamWorld(); });
    const std::string subs = join(team_world_sub_topics_, ", ");
    RCLCPP_INFO(get_logger(),
        "TeamWorld: %.2f Hz, pub '%s' -> sub [%s]%s; comms ttl %.1fs, "
        "closure %s, gossip max age %.0fs",
        team_world_hz_, team_world_pub_topic_.c_str(), subs.c_str(),
        (team_world_sub_topics_.size() == 1 &&
         team_world_sub_topics_[0] == team_world_pub_topic_)
            ? " (shared bus: no external process can gate this stream)" : "",
        team_model_.config().direct_ttl_sec,
        team_model_.config().closure_enabled ? "on" : "off",
        team_model_.config().gossip_max_age_sec);
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
            // Any team of two or more is accepted; a team of one has no link to
            // read and is refused. The row scan requires ALL of this robot's
            // links to be up. (notes: ctor-linkgate-index-team-of-two-plus)
            const std::string joined = join(names, ", ");
            if (names.size() < 2) {
              if (!link_index_warned_) {
                link_index_warned_ = true;
                RCLCPP_WARN(get_logger(),
                    "Link gate: robot index lists %zu robot(s) [%s]; there is "
                    "no link to read, so the mid-run trigger keeps the "
                    "record-age clock for this run.",
                    names.size(), joined.c_str());
              }
              link_index_usable_ = false;
              return;
            }
            const auto it = std::find(names.begin(), names.end(), robot_name_);
            if (it == names.end()) {
              if (!link_index_warned_) {
                link_index_warned_ = true;
                RCLCPP_WARN(get_logger(),
                    "Link gate: robot index does not name '%s' (it lists [%s]); "
                    "keeping the record-age clock.",
                    robot_name_.c_str(), joined.c_str());
              }
              link_index_usable_ = false;
              return;
            }
            const int idx = static_cast<int>(it - names.begin());
            if (!link_index_usable_ || link_self_idx_ != idx) {
              RCLCPP_INFO(get_logger(),
                  "Link gate: armed as robot index %d of [%s]; the mid-run "
                  "trigger still fires on record age, but a fire is vetoed "
                  "while EVERY one of this robot's %zu link(s) is up — at BOTH "
                  "the trigger and the exhausted-chase escalation.",
                  idx, joined.c_str(), names.size() - 1);
            }
            link_self_idx_     = idx;
            link_robot_count_  = static_cast<int>(names.size());
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
          const auto& d = msg->data;
          // Stride from the publisher's own layout (dim[1].size is the
          // emulator's kLinkStateCols), falling back to whichever known width
          // divides the payload. Decided once and then held, so a malformed
          // message cannot silently re-interpret the table mid-run.
          if (link_cols_ == 0) {
            const size_t declared = msg->layout.dim.size() >= 2
                ? static_cast<size_t>(msg->layout.dim[1].size) : 0;
            for (const size_t cand : {declared, kLinkColsWithValid,
                                      kLinkColsLegacy}) {
              if ((cand == kLinkColsWithValid || cand == kLinkColsLegacy) &&
                  !d.empty() && d.size() % cand == 0) {
                link_cols_ = cand;
                break;
              }
            }
            if (link_cols_ == 0) {
              if (!link_cols_warned_) {
                link_cols_warned_ = true;
                RCLCPP_ERROR(get_logger(),
                    "link_states on '%s' carries %zu values, which is not a "
                    "whole number of %zu- or %zu-column rows. The link gate "
                    "will take NO samples: it stays unready, and the reconnect "
                    "trigger falls back to the record-age clock it would use "
                    "with the gate off. Fix the emulator/planner version skew.",
                    comms_link_states_topic_.c_str(), d.size(),
                    kLinkColsWithValid, kLinkColsLegacy);
              }
              return;
            }
            RCLCPP_INFO(get_logger(),
                "link_states table on '%s': %zu columns per pair%s.",
                comms_link_states_topic_.c_str(), link_cols_,
                link_cols_ == kLinkColsLegacy
                    ? " (no `valid` column — pre-generation-9 emulator, masking "
                      "on the path-loss floor instead)"
                    : " (masking on `valid`)");
          }
          const size_t kCols = link_cols_;
          // Per-PEER, not a running OR over the rows: see link_connected_. The
          // vectors are indexed by the peer's own row index so a table that
          // repeats or omits a pair cannot be miscounted by an accumulator.
          const size_t n = static_cast<size_t>(link_robot_count_);
          std::vector<char> seen(n, 0), up(n, 0);
          bool found = false;
          for (size_t k = 0; k + kCols <= d.size(); k += kCols) {
            const int i = static_cast<int>(d[k]);
            const int j = static_cast<int>(d[k + 1]);
            if (i != link_self_idx_ && j != link_self_idx_) continue;
            // Skip rows the emulator did not compute (same test and order as
            // link_logger.py), or leaving them would fake a reconnection:
            // column 9 valid when present, else a non-positive column 4
            // (path-loss floor). (notes: ctor-linkgate-row-validity-mask)
            if (kCols == kLinkColsWithValid && d[k + 9] == 0.0) continue;
            if (d[k + 4] <= 0.0) continue;
            const int other = (i == link_self_idx_) ? j : i;
            if (other < 0 || other >= static_cast<int>(n) ||
                other == link_self_idx_) {
              continue;   // a row that does not name a peer we know about
            }
            found = true;
            seen[static_cast<size_t>(other)] = 1;
            if (d[k + 8] != 0.0)              // `connected` column
              up[static_cast<size_t>(other)] = 1;
          }
          if (!found) return;
          // A peer with no usable row is unknown and counts as not connected.
          // That clears the veto (fallback to the record-age clock), so an
          // unwarmed table cannot suppress dispatches.
          // (notes: ctor-linkgate-unknown-peer-down)
          bool connected = true;
          for (size_t q = 0; q < n; ++q) {
            if (static_cast<int>(q) == link_self_idx_) continue;
            if (!seen[q] || !up[q]) { connected = false; break; }
          }
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
          if (!link_gate_live_logged_) {
            link_gate_live_logged_ = true;
            // The token is matched verbatim by gate_g8.py check 3f; do not
            // reword it without updating the gate. Emitted here, not from
            // linkGateReady(), which runs only when the gate is consulted.
            // (notes: ctor-linkgate-live-token-site)
            RCLCPP_INFO(get_logger(),
                "link_gate_live: first usable link sample on '%s' (index ok, "
                "link %s); the mid-run veto can now run.",
                comms_link_states_topic_.c_str(),
                connected ? "up" : "down");
          }
        });
    RCLCPP_INFO(get_logger(),
        "Link gate: subscribed to '%s' (KeepLast(1)) with index from '%s', "
        "stale after %.1fs. The mid-run trigger clock is unchanged "
        "(peer_record_age_sec); the link state only vetoes a fire aimed at a "
        "peer that is already reachable.",
        comms_link_states_topic_.c_str(),
        comms_link_robot_index_topic_.c_str(), comms_link_stale_sec_);
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
      // Latched both ends, matching the scovox_node subscription: an ADD is
      // sent once per target id with no retry, so it must survive discovery
      // races. Replay is safe (keyed-replace adds, idempotent removes).
      // (notes: ctor-refinement-region-latched-qos)
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
    const std::chrono::duration<double> period(metrics_period_sec_);
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

  // Must run after updatePoseFromTF (local priority anchors on the current
  // cell) and before the state dispatch and the proximity-hold return, so
  // planning sees the merged census and a held robot keeps tracking peers.
  // (notes: tick-drain-teamworld-order)
  drainTeamWorld();

  // Any tick outside PLAN ends an all-rejected episode. Clear it here, not only
  // on goal selection: doPlan has other exits and reconnect manoeuvres leave
  // PLAN. (notes: tick-all-rejected-reset-off-plan)
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

  // Coordinated proximity stop: checked every tick before the state dispatch so
  // the hold pre-empts the driving states (incl. RETURN_HOME). Stationary
  // states are exempt; they are already still.
  // (notes: tick-proximity-hold-driving-states)
  if ((state_ == State::NAVIGATE || state_ == State::RETURN_NAV ||
       state_ == State::PURSUE || state_ == State::RETURN_HOME) &&
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
              "yet — the team agrees its rendezvous place while it is still "
              "co-located, and dispersing mid-agreement is what splits a "
              "fleet. Applied in every arm so the cost is shared.",
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

    case State::RETURN_HOME:
      doReturnHome();
      break;

    case State::DONE:
      // done_action == idle: stay alive so targets released at coverage-done
      // still enter the exploit sub-loop (hasPending() includes an ACTIVE
      // target). Under the coverage latch the robot never leaves DONE.
      // (notes: tick-done-idle-and-latch)
      if (done_action_ == "idle") {
        if (coverage_latched_) {
          // Coast watchdog, the only bound on a goal done_seek left standing.
          // [arrived] fires because the robot already stopped and just parks;
          // only [timeout], still moving, brakes via abandonNavGoal.
          // (notes: tick-done-seek-coast-watchdog)
          if (done_seek_coasting_) {
            const double tnow  = this->now().seconds();
            const double coast = tnow - done_seek_start_sim_;
            if (cumulative_distance_ > done_seek_last_dist_ + 0.25) {
              done_seek_last_dist_     = cumulative_distance_;
              done_seek_last_move_sim_ = tnow;
            }
            const double still = tnow - done_seek_last_move_sim_;
            if (still >= 30.0) {
              done_seek_coasting_ = false;
              RCLCPP_INFO(get_logger(),
                  "DONE-SEEK coast END [arrived]: stopped moving %.0f s ago "
                  "after %.1f s and %.2f m. Parking here.",
                  still, coast,
                  cumulative_distance_ - done_seek_dist_at_start_);
            } else if (done_seek_max_sec_ > 0.0 &&
                       coast >= done_seek_max_sec_) {
              done_seek_coasting_ = false;
              RCLCPP_WARN(get_logger(),
                  "DONE-SEEK coast END [timeout]: %.1f s reached the %.0f s cap "
                  "while still moving (%.2f m travelled) — braking. The goal "
                  "was probably unreachable.",
                  coast, done_seek_max_sec_,
                  cumulative_distance_ - done_seek_dist_at_start_);
              abandonNavGoal("done-seek-timeout");
            }
          }
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
        // Remove regions whose target never reached DONE, since these exits
        // skip finishActiveTarget. rclcpp::shutdown() is deferred one tick so
        // DDS can flush the removes. (notes: tick-done-shutdown-remove-regions)
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
  // The reconnect clock keeps running while s stays in RETURN_NAV, RETURN_SYNC,
  // PURSUE or PROXIMITY_HOLD, so one manoeuvre spans handoffs. Leaving the set
  // stops it and the CSV reverts to its sentinel.
  // (notes: transition-reconnect-clock-states)
  const bool manoeuvre = (s == State::RETURN_NAV || s == State::RETURN_SYNC ||
                          s == State::PURSUE || s == State::PROXIMITY_HOLD);
  // Carries the barrier's own verdict from the manoeuvre-end block to the
  // appointment classifier below, which runs after appointment_manoeuvre_ is
  // cleared and so cannot re-ask manoeuvreReleaseEligible itself. False
  // whenever no appointment manoeuvre ends on this transition.
  bool appointment_barrier_released = false;
  if (reconnect_active_ && !manoeuvre) {
    const double manoeuvre_sec = (this->now() - reconnect_start_time_).seconds();
    const int live = accountedPeerCount(this->now());
    // reconnected iff the ending barrier would release
    // (releaseHeld(manoeuvreReleaseEligible)); else gave_up into DONE or
    // RETURN_HOME, else abandoned. appointment_manoeuvre_ is cleared only after
    // both classifiers. (notes: transition-manoeuvre-outcome-classify)
    const bool manoeuvre_released = releaseHeld(manoeuvreReleaseEligible(live));
    const char* outcome =
        manoeuvre_released
                    ? "reconnected"
                    : ((s == State::DONE || s == State::RETURN_HOME)
                           ? "gave_up" : "abandoned");
    // The outcome is in the line because the destination state is no proxy:
    // under mission return every manoeuvre ends in RETURN_HOME.
    // (notes: transition-outcome-in-log-line)
    RCLCPP_INFO(get_logger(),
        "Reconnect manoeuvre ended after %.1f s sim: %s (-> %s).",
        manoeuvre_sec, outcome, stateName(s));
    if (exp_log_) {
      ReconnectEndEvent e;
      e.outcome        = outcome;
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
    // Captured before appointment_manoeuvre_ is cleared: the appointment
    // classifier below needs this appointment's own barrier verdict, which can
    // release with teamSettled false. (notes: transition-appt-barrier-verdict)
    appointment_barrier_released = appointment_manoeuvre_ && manoeuvre_released;
    // The manoeuvre no longer serves an appointment, whatever happens to the
    // record. This is the only clear site; clearing it in closeAppointment()
    // would defeat the latch. (notes: transition-appt-manoeuvre-only-clear)
    appointment_manoeuvre_ = false;
    // The wait for a finished peer belonged to that manoeuvre's barrier.
    finished_peer_wait_start_sec_      = -1.0;
    finished_peer_wait_expired_logged_ = false;
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

  // ---- P5: appointment bookkeeping -------------------------------------
  // Kept separate from the manoeuvre-end block: a deferred appointment stands
  // while exploring. Every armed appointment must produce exactly one outcome
  // event. (notes: transition-appt-bookkeeping-block)
  if (appointment_armed_) {
    // Label with the barrier's decision, not its input: releaseHeld (never the
    // mutating releaseConfirmed) of teamSettled, or
    // appointment_barrier_released. rendezvousTeamMutual() goes to its own
    // mutual column. (notes: transition-appt-label-barrier-decision)
    const int live_now = accountedPeerCount(this->now());
    const bool team_back = releaseHeld(teamSettled(live_now)) ||
                           appointment_barrier_released;
    const bool team_back_mutual = rendezvousTeamMutual();
    const bool run_over  = (s == State::DONE || s == State::RETURN_HOME);
    // A departed appointment resolves when the manoeuvre keeping it ends. An
    // undeparted one must survive leaving the manoeuvre states; returning to
    // explore before the deadline is the mechanism.
    // (notes: transition-appt-departed-resolution)
    const bool kept_and_done = appointment_departed_ && !manoeuvre;
    if (team_back || run_over || kept_and_done) {
      const double t_close = missionElapsed();
      const double waited =
          (appointment_arrived_ && appointment_arrived_at_sec_ >= 0.0 &&
           t_close >= 0.0)
              ? std::max(0.0, t_close - appointment_arrived_at_sec_)
              : 0.0;
      // Derived, not re-decided. team_back wins; run-ended and unplaceable
      // answer prior questions, so they rank above the arrived/unreachable
      // split. unplaceable reports arrived=false.
      // (notes: transition-appt-outcome-derivation)
      const char* appt_outcome = team_back                ? "reconnected"
                                 : !appointment_departed_ ? "superseded"
                                 : coverage_latch_teardown_ ? "run-ended"
                                 : appointment_unplaceable_ ? "unplaceable"
                                 : appointment_arrived_   ? "no-show"
                                                          : "unreachable";
      closeAppointment(appt_outcome,
                       appointment_arrived_ && !appointment_unplaceable_,
                       waited, team_back_mutual);
    }
  }

  // The release-confirm dwell never survives a state change: a flicker that
  // straddles e.g. a PROXIMITY_HOLD must restart its window.
  release_ok_armed_ = false;

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
  // DONE is the single funnel for every ending, so its reason is captured here.
  // run_end must be the file's last line: written now for shutdown, deferred to
  // the destructor for done_action=idle.
  // (notes: transition-done-run-end-placement)
  if (s == State::DONE) {
    done_reason_ = reason;
    if (done_action_ != "idle") logRunEnd(reason);
  }
  // The post-arrival rotation deadline is per-NAVIGATE-cycle and is armed
  // lazily on arrival at the XY goal; disarm it on every entry.
  if (s == State::NAVIGATE) rotate_deadline_armed_ = false;
  // The dwell-sync barrier is per-DWELL: its wait clock starts at entry and
  // neither latch may carry into the next vantage.
  // (notes: transition-dwell-sync-per-dwell)
  if (s == State::EXPLOIT_DWELL) {
    dwell_sync_wait_start_ = state_enter_time_;
    dwell_sync_timed_out_  = false;
    dwell_sync_started_    = false;
    // Declare staged here: dwell entry follows the XY-arrival gate and the yaw
    // settle, so the robot is still on its vantage. Published now, not on the
    // heartbeat: waiting peers re-anchor their dwell clocks until they hear it.
    // (notes: transition-dwell-entry-staged-publish)
    if (intent_pub_ && have_active_intent_) {
      current_intent_msg_.staged = true;
      current_intent_msg_.header.stamp = state_enter_time_;
      publishIntent();
    }
  }
}

// Vehicle set: self first-hand at (x, y), peers at their freshest position from
// team_model_ (gossip included). Only the allocator passes pos_max_age_sec; the
// reconnect gate and rendezvous snapshot pass 0.
// (notes: alloc-vehicles-vehicle-set)
std::vector<AllocRobot> ExploPlannerNode::allocVehicles(
    float x, float y, bool* all_in_comms,
    double now_sec, double pos_max_age_sec) const {
  std::vector<AllocRobot> out;
  if (all_in_comms) *all_in_comms = true;
  if (!cell_world_.configured()) return out;

  const CellGrid& g = cell_world_.grid();
  for (int id = 0; id < fleet_.size(); ++id) {
    AllocRobot r;
    r.id = id;
    if (id == fleet_.self_id) {
      r.cell = g.idAt(x, y);
      r.in_comms = true;
      r.finished = false;   // we are planning, so we are not done
      // And by the same token still on the frontier: this function is only
      // ever called to decide where THIS robot works next. Reading our own
      // homing latch here would delete us from our own allocation problem.
      r.off_frontier = false;
    } else {
      const TeamModel::Peer& p = team_model_.peer(id);
      // TTL keys on POSITION age, not lastKnownAgeSec: a relayed status update
      // refreshes the latter but not the pose (team_model.hpp rule 3). A -1 age
      // is already covered by have_position.
      // (notes: alloc-vehicles-position-age-ttl)
      const double age = team_model_.positionAgeSec(id, now_sec);
      const bool fresh = allocPeerPositionFresh(age, pos_max_age_sec);
      // A missing or expired position gives cell -1: the robot drops out and
      // its cells return to the pool, never defaulted onto a cell. It stays in
      // the list, still counting toward all_in_comms and fleet size.
      // (notes: alloc-vehicles-unlocatable-cell)
      r.cell = (p.have_position && fresh)
                   ? g.idAt(static_cast<float>(p.position_x),
                            static_cast<float>(p.position_y))
                   : -1;
      r.in_comms = team_model_.inComms(id);
      r.finished = p.finished;
      // The superset (generation 33, R4): homing or done. `finished` is ORed
      // in rather than assumed to imply it, because the two arrive by
      // different routes — a peer heard only through the relay can carry the
      // finished bit while its mode level has not reached us.
      r.off_frontier =
          p.finished ||
          p.mode >= explo_planner_msgs::msg::TeamWorld::MODE_HOMING;
      // all_in_comms stays on `finished` ALONE and is not widened. It answers
      // "is the team whole", and a robot driving home out of contact is a real
      // break in the team — it is carrying a map nobody can reach. Only a run
      // that is OVER excuses the silence.
      if (!r.in_comms && !r.finished && all_in_comms) *all_in_comms = false;
    }
    out.push_back(r);
  }
  return out;
}

std::vector<SeparationTerm::Anchor> ExploPlannerNode::separationAnchors(
    double now_sec) const {
  std::vector<SeparationTerm::Anchor> out;
  if (!team_model_.configured()) return out;

  for (int id = 0; id < team_model_.size(); ++id) {
    if (id == fleet_.self_id) continue;
    const TeamModel::Peer& p = team_model_.peer(id);
    if (!p.known || !p.have_position) continue;
    // Finished teammates are excluded, not down-weighted: they clear no ground.
    // finished is set only from first-hand messages, so a relay-only peer may
    // still repel after it parks. (notes: separation-anchors-skip-finished)
    if (p.finished) continue;
    // POSITION age, not lastKnownAgeSec: a relayed status update refreshes
    // "when did we last hear about this robot" while leaving the pose we hold
    // for it untouched, and steering on the second while filtering on the
    // first is exactly the trap team_model.hpp's rule 3 is written about.
    const double age = team_model_.positionAgeSec(id, now_sec);
    if (!separation_.eligibleAnchor(age)) continue;
    out.push_back(SeparationTerm::Anchor{
        static_cast<float>(p.position_x), static_cast<float>(p.position_y)});
  }
  return out;
}

// ==================================================================
// PLAN state
// ==================================================================

void ExploPlannerNode::doPlan() {
  // Step budget: never start an exploration step past max_steps_. Only
  // reachable with done_action=idle, where the exploit sub-loop's empty-queue
  // revert lands in PLAN. (notes: plan-step-budget-guard)
  if (step_ >= max_steps_) {
    // Measure coverage fresh for the event. Keep the two statements: nested,
    // argument order is unsequenced and budget_src can be read before it is
    // filled. A deferred finishOrRendezvous re-walks the ROI each tick.
    // (notes: plan-step-budget-coverage-sample)
    const char* budget_src = "";
    const double budget_unk = coverageUnknownFraction(&budget_src);
    noteCoverageDecisionSample(budget_unk, budget_src);
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
        // With rendezvous on and a teammate still out of comms, return to the
        // anchor and wait for the team instead of finishing (see below).
        finishOrRendezvous("coverage-saturated");
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

  // A standing appointment is a clock, not a decision: re-running the mid-run
  // trigger would re-derive it and burn an attempt every PLAN tick. Keep
  // exploring; depart when the departure rule says the robot can still make it.
  // (notes: rzv-appointment-is-a-clock)
  if (appointment_armed_ && !appointment_departed_) {
    // Supersede on the arming test's predicate (teamSettled), held for
    // reconnect_release_confirm_sec via dwellHeld, so start and cancel are one
    // event. dwellHeld only reads the window heartbeatTick steps; never tick it
    // here. (notes: rzv-appointment-supersede-dwell)
    const int live_for_supersede = accountedPeerCount(this->now());
    if (dwellHeld(teamSettled(live_for_supersede), this->now().seconds(),
                  reconnect_release_confirm_sec_, team_back_ok_armed_,
                  team_back_ok_since_sec_)) {
      // The outage ended before the deadline: close as superseded so the
      // rendezvous_agreed row gets an outcome. mutual is a genuine second read
      // (all-pairs two-way), not the branch condition restated.
      // (notes: rzv-supersede-close-mutual-read)
      closeAppointment("superseded", /*arrived=*/false, /*waited_sec=*/0.0,
                       /*mutual=*/rendezvousTeamMutual());
    } else if (appointmentDue()) {
      RCLCPP_INFO(get_logger(),
          "Rendezvous: time to leave for cell %d (meeting at t+%.0fs, my "
          "travel %.0fs) -> breaking off exploration for the appointment.",
          appointment_.cell, appointment_.t_meet_ms / 1000.0,
          appointmentTravelMs() / 1000.0);
      appointment_departed_ = true;
      // A deadline departure is mid-run: clear reconnect_terminal_ (resting
      // value true) so the barrier takes the mid-run wait cap. Must precede
      // startReturnTo, which stamps reconnect_end with this flag.
      // (notes: rzv-deadline-departure-not-terminal)
      reconnect_terminal_ = false;
      startReturnTo(appointmentPoint(), "appointment", "appointment-due");
      return;
    }
  }

  // Rendezvous and hybrid both arm at the separation, not behind the mid-run
  // gate. Arming does not move the robot or t_meet; the departure rule owns
  // departure. Not gated on have_rendezvous_anchor_.
  // (notes: rzv-appointment-arms-on-separation)
  if ((reconnect_mode_ == ReconnectMode::RENDEZVOUS ||
       reconnect_mode_ == ReconnectMode::HYBRID) && reconnect_enabled_ &&
      rendezvous_schedule_enable_ && !appointment_armed_ &&
      !rendezvous_spent_ && rendezvous_agreed_.valid()) {
    // Arm on one sample of !teamSettled. The supersede above, transitionTo()'s
    // outcome classifier and the rendezvous_spent_ release must use the same
    // predicate. Preconditions are pre-tested: armAppointment logs refusals.
    // (notes: rzv-arm-predicate-team-settled)
    const int live_for_arm = accountedPeerCount(this->now());
    if (!teamSettled(live_for_arm)) {
      // The reason string only records which half fired: peer-separation means
      // this robot hears everyone and armed on a peer's announced break. It
      // reaches the plaintext log only; nothing in the tree matches on it.
      // (notes: rzv-arm-reason-peer-separation)
      armAppointment(teamComplete(live_for_arm, rendezvous_expected_peers_)
                         ? "peer-separation"
                         : "separation");
    }
  }

  // Off unless reconnect_midrun_silence_sec > 0; stays after the exploit branch
  // and coverage check, and re-reads the peer count. Hybrid gets one chase
  // (appointment_chase_tried_) while its appointment is pending.
  // (notes: reconnect-midrun-trigger-placement)
  const bool hybrid_may_chase =
      reconnect_mode_ == ReconnectMode::HYBRID && appointment_armed_ &&
      !appointment_departed_ && !appointment_chase_tried_;
  if (reconnect_midrun_silence_sec_ > 0.0 && reconnect_enabled_ &&
      have_anchor_ && team_seen_complete_ &&
      (!appointment_armed_ || hybrid_may_chase)) {
    if (midrun_attempts_ < reconnect_midrun_max_attempts_) {
      const auto trig_now = this->now();
      const int live = accountedPeerCount(trig_now);
      const double missing_for =
          (trig_now - team_last_complete_time_).seconds();
      const bool cooldown_ok =
          !midrun_end_armed_ ||
          (trig_now - midrun_last_end_).seconds() >=
              reconnect_midrun_silence_sec_;
      // Threshold comes from the info gate when configured, else the fixed
      // clock; the cooldown always uses the fixed clock. Gate maths runs only
      // once the team reads incomplete, since midrunGateSec walks the peer
      // table. (notes: reconnect-midrun-gate-threshold)
      if (!teamComplete(live, rendezvous_expected_peers_) && cooldown_ok) {
        // missing_for is the team-presence clock, lagging peer record age by
        // the claim TTL. The link veto only refuses fires at a peer already on
        // the radio; it is also applied in pursuitFallback.
        // (notes: reconnect-link-gate-veto)
        bool   link_veto     = false;   // stand down this tick
        double link_down_for = -1.0;    // diagnostic; -1 = gate not in play
        if (linkGateReady(trig_now)) {
          link_down_for = link_connected_
                              ? 0.0
                              : (trig_now - link_up_last_seen_).seconds();
          // Debounced on reconnect_link_down_confirm_sec, the radio debounce
          // only (ships at 0). Keep the link_connected_ disjunct: at 0 the bare
          // comparison is 0.0 < 0.0 and would pass a chase at a reachable peer.
          // (notes: reconnect-link-down-debounce)
          if (link_connected_ ||
              link_down_for < reconnect_link_down_confirm_sec_) {
            link_veto = true;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                "Reconnect (mid-run): standing down — team incomplete "
                "%.0fs but the radio link is %s, so a chase would be spent on "
                "a peer that is already reachable.",
                missing_for,
                link_connected_ ? "UP" : "only just down");
          }
        }
        if (!link_veto) {
          double est_unshared = -1.0;
          // Gate maths runs only past the veto: midrunGateSec walks the peer
          // table, and a quiet-but-connected teammate would otherwise buy that
          // walk on every PLAN tick for the rest of the run.
          const double gate_sec = midrunGateSec(missing_for, &est_unshared);
          bool gate_refuses = false;
          // Declared here because the row is written after the dispatch.
          // Written only when gate_evaluated is set: with the gate off there is
          // no verdict to report. (notes: reconnect-gate-event-hoisted)
          ReconnectGateEvent ev;
          bool gate_evaluated = false;
          if (missing_for >= gate_sec) {
            // Knowledge + value gate, strictly downstream of the silence clock,
            // so it can only refuse a dispatch that clock allowed. Only the
            // mid-run trigger is gated, not the pursuitFallback escalation.
            // (notes: reconnect-knowledge-value-gate)
            if (reconnect_gate_info_) {
              // Same vehicle set the allocator solves over, at the current
              // pose; the missing list is read off it. Pose age is unbounded
              // (0) on purpose: expiring the stale pose would drop the very
              // peer the gate values.
              // (notes: reconnect-gate-unbounded-peer-age)
              const std::vector<AllocRobot> vehicles =
                  allocVehicles(latest_pos_.x(), latest_pos_.y(), nullptr,
                                missionElapsed(), 0.0);
              std::vector<MissingPeer> missing;
              std::string peers_str;
              for (const AllocRobot& r : vehicles) {
                if (r.id == fleet_.self_id || r.in_comms) continue;
                missing.push_back(MissingPeer{r.id, r.cell, r.finished});
                peers_str += (peers_str.empty() ? "" : ",") +
                             std::to_string(r.id) + ":" +
                             std::to_string(r.cell);
              }
              const GateVerdict gv = evaluateReconnectGate(
                  cell_world_, vehicles, missing, alloc_cfg_);

              // Logged on every evaluation, fired or not. Built here but
              // written after the dispatch, so dispatched reports whether a
              // manoeuvre started, not the gate verdict.
              // (notes: reconnect-gate-row-after-dispatch)
              gate_evaluated = true;
              {
                ev.knowledge           = gv.knowledge;
                ev.unshared_cells      = gv.unshared_cells;
                ev.c_no_mm             = gv.c_no_mm;
                ev.c_re_mm             = gv.c_re_mm;
                ev.leg_mm              = gv.leg_mm;
                ev.unassigned          = gv.unassigned;
                ev.refused             = gv.refused;
                ev.team_incomplete_sec = missing_for;
                ev.gate_sec            = gate_sec;
                // Pre-increment on purpose: attempts already used when this
                // evaluation ran. Moving the read past the increment below
                // shifts the whole column by one.
                // (notes: reconnect-gate-attempts-pre-increment)
                ev.attempts_used       = midrun_attempts_;
                ev.peers               = peers_str;
              }

              if (!gv.dispatch) {
                // No attempt consumed and no cooldown armed, since nothing was
                // dispatched; otherwise correctly suppressed evaluations could
                // exhaust the budget.
                // (notes: reconnect-gate-refusal-no-attempt)
                gate_refuses = true;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                    "Reconnect (mid-run): gate says stay — %d unshared cells, "
                    "C_re %lld mm vs C_no %lld mm (leg %lld mm).",
                    gv.unshared_cells, gv.c_re_mm, gv.c_no_mm, gv.leg_mm);
              }
            }

            bool midrun_dispatched = false;
            if (!gate_refuses) {
              ++midrun_attempts_;
              // Spent whether or not the chase starts, because this is the one
              // chase hybrid_may_chase allows for the standing appointment and
              // a declined chase must not be retried into the attempt budget.
              // Set before dispatchReconnect so an early return cannot skip it.
              if (appointment_armed_) appointment_chase_tried_ = true;
              reconnect_terminal_ = false;
              hold_escalated_ = false;
              dispatch_gate_sec_      = gate_sec;
              dispatch_est_unshared_  = est_unshared;
              dispatch_link_down_sec_ = link_down_for;
              dispatch_team_incomplete_sec_ = missing_for;
              // Says team incomplete, not peer silent: missing_for is the
              // team-presence clock, which lags the peer's record age by
              // coord_claim_ttl_sec. Logged beside gate_sec so the inequality
              // is readable. (notes: reconnect-log-team-incomplete-wording)
              RCLCPP_INFO(get_logger(),
                  "Reconnect (mid-run): team incomplete %.0fs >= gate %.0fs "
                  "(radio down %.0fs, est unshared %.0f vox, attempt %d/%d) -> "
                  "interrupting exploration for the reconnect manoeuvre.",
                  missing_for, gate_sec, link_down_for, est_unshared,
                  midrun_attempts_, reconnect_midrun_max_attempts_);
              midrun_dispatched = dispatchReconnect("peer-lost");
              if (!midrun_dispatched) {
                // The attempt stays spent (hybrid's one chase). Arm the
                // cooldown and restore reconnect_terminal_ to true here, since
                // transitionTo and resumeExploring are not reached; else the
                // trigger re-fires every tick.
                // (notes: reconnect-declined-dispatch-bookkeeping)
                midrun_last_end_    = this->now();
                midrun_end_armed_   = true;
                reconnect_terminal_ = true;
                RCLCPP_INFO(get_logger(),
                    "Reconnect (mid-run): attempt %d spent but no manoeuvre "
                    "started — %s. Exploration continues.",
                    midrun_attempts_,
                    appointment_armed_
                        ? "a standing appointment already owns this outage"
                        : "no arm-appropriate manoeuvre was available");
              }
            }
            if (gate_evaluated && exp_log_) {
              ev.dispatched = midrun_dispatched;
              exp_log_->logReconnectGate(expCtx(), ev);
            }
            if (midrun_dispatched) return;
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
      static_cast<size_t>(accountedPeerCount(plan_end)), plan_ms);

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
    // Stop the released hop, not just forget it: EXPLOIT_PLAN may retry without
    // publishing a goal while the navigator still drives the abandoned hop
    // (invariant: see abandonNavGoal).
    // (notes: navigate-target-release-stops-hop)
    abandonNavGoal("target released mid-hop");
    transitionTo(State::EXPLOIT_PLAN, "target-released-mid-hop");
    return;
  }

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

  // Exploit vantage hops only: peers can pick the same ring angle before either
  // hears the other's 1 Hz claim. selfWinsAgainst is a strict total order, so
  // exactly one of the pair yields.
  // (notes: navigate-vantage-cross-pick-tiebreak)
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
      // Release the claim and stop the drive with the hop: a kept claim holds
      // the winner's dwell-sync barrier, and a retrying EXPLOIT_PLAN must not
      // keep rolling toward the conceded angle.
      // (notes: navigate-yielded-vantage-release)
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

  // Yaw is required only where heading matters: always for EXPLOIT vantages,
  // otherwise only when the sensor model is not omnidirectional. Keep the
  // exception a phase test, not a separate parameter.
  // (notes: navigate-goal-yaw-requirement)
  const bool yaw_required =
      (phase_ == Phase::EXPLOIT) || !fov_is_omnidirectional_;
  if (dist < goal_xy_tol_) {
    float yaw_err = std::remainder(latest_yaw_ - current_goal_.yaw,
                                   2.0f * static_cast<float>(M_PI));
    if (!yaw_required || std::abs(yaw_err) < goal_yaw_tol_) {
      if (phase_ == Phase::EXPLOIT) {
        if (current_is_approach_) {
          // Approach waypoint reached: keep the claim, re-arm the give-up timer
          // as a completed dwell does, and re-plan vantages. Finite because
          // computeApproachGoal requires each approach to land nearer the
          // trunk. (notes: exploit-approach-waypoint-rearm)
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
// Rendezvous (multi-robot reconnection)
// ==================================================================

// (notes: explore-complete-entry-point)
void ExploPlannerNode::recordExplorationComplete(const char* reason) {
  const int active = accountedPeerCount(this->now());
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
    e.team_complete     = teamComplete(active, rendezvous_expected_peers_);
    e.peers_live        = active;
    e.expected_peers    = rendezvous_expected_peers_;
    e.occurrence        = ++exp_complete_count_;
    exp_log_->logExplorationComplete(expCtx(), e);
  }
}

bool ExploPlannerNode::finishNow(const char* reason) {
  // A DONE-idle robot keeps publishing presence so teammates finishing later
  // still count it via claim TTLs; a silent finisher ages out and strands them
  // at the barrier. (notes: done-idle-keeps-presence)
  if (done_action_ == "idle") {
    publishPresenceIntent();
  }
  transitionTo(State::DONE, reason);
  return true;
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

  // A robot already standing on the agreed cell (appointment_arrived_) stays in
  // RETURN_SYNC instead of ending. Returning false does not un-finish it;
  // doReturnSync ends the run on release or the latched cap.
  // (notes: latch-stay-on-agreed-cell)
  if (state_ == State::RETURN_SYNC && appointment_manoeuvre_ &&
      appointment_arrived_) {
    // Stamp the hold's clock at max(now, t_meet), not at arrival: the
    // latched-hold cap is sized from t_meet. The stand stays bounded by
    // interval + cap. doReturnSync makes the same stamp.
    // (notes: latch-hold-clock-from-t-meet)
    const double t_now = missionElapsed();
    double t_hold = (t_now >= 0.0) ? t_now : 0.0;
    if (appointment_.valid())
      t_hold = std::max(t_hold, appointment_.t_meet_ms / 1000.0);
    coverage_latch_hold_start_sec_ = t_hold;
    RCLCPP_INFO(get_logger(),
        "Rendezvous: map saturated while holding the agreed cell -> STAYING. "
        "Exploration is recorded complete at t_sim=%.1f; this robot keeps the "
        "appointment for up to %.0fs more so the team can still meet and merge.",
        coverage_latch_t_sim_, rendezvous_latched_hold_sec_);
    return false;
  }

  // A finisher with an appointment still to reach drives there instead of home.
  // Must return true: doPlan reads false as carry on planning and would publish
  // a frontier over the meeting goal. (notes: latch-go-to-unreached-meeting)
  if (keepAppointmentOnFinish("coverage-latched")) return true;

  // Every ending below IS a coverage-latch teardown, so the appointment outcome
  // classifier must not read one as a no-show. Set before any of them, because
  // each one reaches transitionTo, which is where the classifier runs.
  coverage_latch_teardown_ = true;

  // Mission return pre-empts both legacy endings (park and the done_seek
  // coast); do not run the coast before it. exploration_complete is already
  // stamped above. (notes: latch-mission-return-preempts)
  if (mission_return_enabled_ && have_home_) {
    abandonNavGoal("coverage-latched");
    return startReturnHome("coverage-latched");
  }
  if (mission_return_enabled_) {
    RCLCPP_ERROR(get_logger(),
        "mission_return_enabled with no recorded home pose — the first TF "
        "pose never arrived, which cannot happen after exploration started. "
        "Falling back to the park-in-place ending.");
  }
  // Coast (keep the nav goal) only if a reconnect manoeuvre is live, gated on
  // reconnect_active_, not the state name. Sample it here: finishNow's
  // transitionTo to DONE clears reconnect_active_.
  // (notes: latch-done-seek-coast-gate)
  if (done_seek_enabled_ && reconnect_active_) {
    done_seek_coasting_      = true;
    done_seek_start_sim_     = this->now().seconds();
    done_seek_dist_at_start_ = cumulative_distance_;
    done_seek_last_dist_     = cumulative_distance_;
    done_seek_last_move_sim_ = done_seek_start_sim_;
    RCLCPP_INFO(get_logger(),
        "DONE-SEEK coast START: latched in state %s with a reconnect manoeuvre "
        "live — keeping the nav goal instead of braking, cap %.0f s. This robot "
        "reads DONE from here on, so the run clock is unaffected.",
        stateName(state_), done_seek_max_sec_);
  } else {
    abandonNavGoal("coverage-latched");
  }
  return finishNow("coverage-latched");
}

// Defers mission return rather than skipping it: every doReturnSync ending
// (barrier release or the latched hold's cap) routes through startReturnHome.
// (notes: keep-appointment-defers-homing)
bool ExploPlannerNode::keepAppointmentOnFinish(const char* reason) {
  // Ask appointment_manoeuvre_ first: the record can close on the tick its
  // drive departs (see startReturnTo). No dispatch: re-issuing the leg would
  // refill the escape ladder. (notes: keep-appointment-manoeuvre-first)
  if (appointment_manoeuvre_) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: run ended [%s] while already on the way to the agreed "
        "cell -> finishing the drive. Exploration is recorded complete; this "
        "robot homes after the meeting, not instead of it.", reason);
    return true;
  }
  if (!appointment_armed_ || !appointment_.valid()) return false;

  // Solve the goal before committing: if appointmentPoint cannot place the cell
  // it sets appointment_unplaceable_ and returns our own position, so decline
  // and let the caller end the run. (notes: keep-appointment-unplaceable-goal)
  const Eigen::Vector3f goal = appointmentPoint();
  if (appointment_unplaceable_) return false;

  // Read the agreed pair BEFORE the dispatch: the transition startReturnTo
  // tails into is exactly where the record can be closed out from under us.
  RCLCPP_INFO(get_logger(),
      "Rendezvous: map saturated with agreed cell %d still to keep (meeting at "
      "t+%.0fs) -> leaving for it now [%s]. This is the departure the deadline "
      "would have triggered later; the deadline exists to spend the wait on "
      "exploring, and there is none left to spend.",
      appointment_.cell, appointment_.t_meet_ms / 1000.0, reason);

  // A coverage-latched keeper goes non-terminal (bounded by the latched hold);
  // any other keeper takes the terminal cap. Assign both flags BEFORE
  // startReturnTo: its transition reads them.
  // (notes: keep-appointment-bounded-keeper)
  reconnect_terminal_   = !coverage_latched_;
  appointment_departed_ = true;
  startReturnTo(goal, "appointment", reason);
  return true;
}

bool ExploPlannerNode::finishOrRendezvous(const char* reason) {
  // accountedPeerCount, not the raw table size: exploit claims are retained
  // past expiry by the grace window (vantage-contest lenience), and a graced
  // claim must not count a 10-s-silent teammate as "present" for the barrier.
  const int active = accountedPeerCount(this->now());
  recordExplorationComplete(reason);
  // A standing appointment outranks the homing traverse, so this must precede
  // the mission-return branch it pre-empts.
  // (notes: finish-appointment-outranks-homing)
  if (keepAppointmentOnFinish(reason)) return true;
  // Mission return replaces the terminal manoeuvre (mid-run manoeuvres from
  // doPlan are untouched), including the step-budget ending, so no robot is
  // stranded where its budget ran out. (notes: finish-mission-return-replaces)
  if (mission_return_enabled_ && have_home_) {
    abandonNavGoal(reason);
    return startReturnHome(reason);
  }
  if (shouldRendezvous(reconnect_enabled_, have_anchor_, active,
                       rendezvous_expected_peers_)) {
    // Dispatch only after the team has been incomplete for
    // reconnect_confirm_sec. This is the only false this function returns; it
    // cannot loop, as team_last_complete_time_ advances only while the team is
    // complete. (notes: finish-reconnect-confirm-gate)
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
    if (dispatchReconnect(reason)) return true;
    // A terminal dispatch that declines must still end the run: returning false
    // would livelock PLAN (step_ frozen, armAppointment cannot change its
    // mind), so finish as DONE. (notes: finish-declined-dispatch-ends-run)
    RCLCPP_WARN(get_logger(),
        "Rendezvous [%s]: terminal dispatch declined (no appointment and no "
        "unscheduled fallback) -> DONE with %d/%d peers live.",
        reason, active, rendezvous_expected_peers_);
    return finishNow(reason);
  }
  if (reconnect_enabled_ && rendezvous_expected_peers_ > 0) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: exploration ended [%s] with full team present "
        "(%d/%d peers) -> DONE.",
        reason, active, rendezvous_expected_peers_);
  }
  return finishNow(reason);
}

// (notes: dispatch-mode-fallbacks)
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
  // The "link_gate_live:" proof line is emitted from the link-states
  // subscription, not from here: this function is reached only when something
  // actually consults the gate, and a run whose team never went silent would
  // otherwise leave no evidence that the veto was able to run.
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
    // No contact snapshot: fall back to the fixed silence clock. est_unshared =
    // -2, not -1, so this fallback is distinguishable from a control run's
    // dispatch. (notes: midrun-gate-no-snapshot-fallback)
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
  // Trigger time uses only snapshot-frozen quantities so both robots derive the
  // same value (see LastContact). Rates are SUMMED (each is a private rate);
  // rate 0 = unknown clamps to the max silence.
  // (notes: midrun-gate-trigger-rate-sum)
  const double rate_sum =
      std::max(0.0, rec->self_rate) + std::max(0.0, rec->peer_rate);
  const double t = (rate_sum > 1e-9)
                       ? reconnect_min_share_voxels_ / rate_sum
                       : reconnect_midrun_max_silence_sec_;
  return std::min(reconnect_midrun_max_silence_sec_,
                  std::max(reconnect_midrun_min_silence_sec_, t));
}

// ==================================================================
// Scheduled rendezvous (P5, §3.5)
// ==================================================================

bool ExploPlannerNode::rendezvousTeamMutual() const {
  if (!team_model_.configured()) return false;
  // A one-robot fleet would pass the loop below VACUOUSLY and stamp an anchor
  // it can never keep an appointment against. Refused explicitly rather than
  // left to the loop, because "no peers" and "every peer confirmed" are the
  // same iteration count and only one of them is a team.
  if (team_model_.size() < 2) return false;
  // Requires every edge of the team graph, not just ours: at N>=3 a
  // self-centred test lets a bridging robot keep re-stamping the anchor during
  // a split. Peers' in_range_mask supply the other edges.
  // (notes: rzv-mutual-every-graph-edge)
  const uint32_t full = fleetMask(team_model_.size());
  // An empty `full` would make condition 2 below pass for every peer — the
  // check would still run and still print as satisfied while testing nothing.
  // A team size fleetMask() refuses is a configuration fault, not a degraded
  // mode to keep scheduling through.
  if (full == 0u) return false;
  for (int id = 0; id < team_model_.size(); ++id) {
    if (id == team_model_.selfId()) continue;
    // 1. Our own edge to the peer: direct, not inComms(), which is true through
    // a relay; the two ends of a relayed link are not synchronised on when the
    // chain breaks. (notes: rzv-mutual-direct-not-relay)
    if (!team_model_.peer(id).direct) return false;
    // 2. The peer's own edges, as the peer itself reported them. Read only
    //    under (1), so the mask is first-hand and inside the TTL by
    //    construction; a mask from a peer we cannot currently hear is a stale
    //    claim about a graph that has since changed.
    if ((team_model_.peer(id).in_range_mask & full) != full) return false;
  }
  return true;
}

bool ExploPlannerNode::peerAccounted(const std::string& name,
                                     const rclcpp::Time& now) const {
  if (!coord_) return false;
  const int id = fleet_.idOf(name);
  if (team_model_.configured() && id >= 0 && id < team_model_.size()) {
    const TeamModel::Peer& p = team_model_.peer(id);
    // `direct`, not `heard_one_way`: this count feeds teamComplete, which asks
    // whether the team can actually EXCHANGE, and a peer that cannot hear us
    // has not reconnected. peerReportsTeamBreak takes the one-way case on
    // purpose and is the right place for it.
    if (p.direct || p.finished) return true;
  }
  // The claim table is still consulted for every peer, including one the fleet
  // list does not name: an unknown producer that is beaconing is a robot in
  // range, and dropping it here would make it permanently "missing".
  return coord_->peerLive(name, now);
}

int ExploPlannerNode::accountedPeerCount(const rclcpp::Time& now) const {
  if (!coord_) return 0;
  // Before TeamModel is configured there is no second channel and no
  // `finished` bit to read, so the union collapses to the claim table — the
  // pre-generation-23 answer, which is the right one when it is the only one.
  if (!team_model_.configured() || fleet_.size() < 2)
    return static_cast<int>(coord_->livePeerCount(now));
  int n = 0;
  for (int id = 0; id < team_model_.size(); ++id) {
    if (id == team_model_.selfId()) continue;
    if (peerAccounted(fleet_.nameOf(id), now)) ++n;
  }
  return n;
}

int ExploPlannerNode::reachablePeerCount() const {
  // Same guard shape as accountedPeerCount, but the collapse is to ZERO, not
  // to the claim table: before TeamModel there is no closure to read, and a
  // zero here makes the generation-27 door in manoeuvreReleaseEligible
  // unsatisfiable, so the release falls back to the mesh predicate it had.
  if (!team_model_.configured() || fleet_.size() < 2) return 0;
  int n = 0;
  for (int id = 0; id < team_model_.size(); ++id) {
    if (id == team_model_.selfId()) continue;
    // Counts peers in the comms closure (inComms) or finished; the finished
    // exemption is an accepted residual. Deliberately no claim-table channel:
    // this count describes the radio graph only.
    // (notes: reachable-count-closure-finished)
    if (team_model_.inComms(id) || team_model_.peer(id).finished) ++n;
  }
  return n;
}

bool ExploPlannerNode::peerReportsTeamBreak() const {
  if (!team_model_.configured()) return false;
  for (int id = 0; id < team_model_.size(); ++id) {
    if (id == team_model_.selfId()) continue;
    const TeamModel::Peer& p = team_model_.peer(id);
    // Receiving from it first-hand right now. Without this the bit would be
    // whatever the peer last said before it went silent, which is a latch: a
    // robot that announced a break and then dropped off the air entirely would
    // hold the team armed forever off a message nobody can refresh.
    if (!p.direct && !p.heard_one_way) continue;
    // A finished or homing peer (p.mode >= MODE_HOMING) never arms the team:
    // the appointment barrier is unbounded. Arm and release both read this
    // function. Not copied to peerAccounted or reachablePeerCount.
    // (notes: team-break-finished-homing-exempt)
    if (p.finished ||
        p.mode >= explo_planner_msgs::msg::TeamWorld::MODE_HOMING)
      continue;
    if (p.team_incomplete) return true;
  }
  return false;
}

bool ExploPlannerNode::peerInboundToAppointment() const {
  if (!team_model_.configured()) return false;
  for (int id = 0; id < team_model_.size(); ++id) {
    if (id == team_model_.selfId()) continue;
    const TeamModel::Peer& p = team_model_.peer(id);
    // Receiving from it first-hand right now, exactly as peerReportsTeamBreak
    // requires and for the same reason: the bit has no TTL of its own.
    if (!p.direct && !p.heard_one_way) continue;
    // No `finished` skip here, unlike peerReportsTeamBreak — see the
    // declaration for why the exemption that function needs would be a defect
    // in this one.
    if (p.appointment_inbound) return true;
  }
  return false;
}

bool ExploPlannerNode::peerReportsInboundToAppointment() const {
  if (!team_model_.configured()) return false;
  for (int id = 0; id < team_model_.size(); ++id) {
    if (id == team_model_.selfId()) continue;
    const TeamModel::Peer& p = team_model_.peer(id);
    // The same liveness gate as peerInboundToAppointment, for the same
    // reason: the report carries no TTL of its own, so a reporter that goes
    // silent must stop holding the barrier.
    if (!p.direct && !p.heard_one_way) continue;
    if (p.appointment_inbound_seen) return true;
  }
  return false;
}

bool ExploPlannerNode::teamSettled(int live_peers) const {
  return teamComplete(live_peers, rendezvous_expected_peers_) &&
         !peerReportsTeamBreak();
}

bool ExploPlannerNode::manoeuvreReleaseEligible(int live_peers) const {
  // On an appointment: release on teamSettled OR every expected peer reachable,
  // vetoed while a peer is inbound or a finished peer is still coming. Keep
  // these terms out of teamSettled, which has other consumers.
  // (notes: release-eligible-reachable-door)
  return appointment_manoeuvre_
             ? ((teamSettled(live_peers) ||
                 teamComplete(reachablePeerCount(), rendezvous_expected_peers_)) &&
                !peerInboundToAppointment() &&
                !peerReportsInboundToAppointment() &&
                !holdingForFinishedPeer())
             : teamComplete(live_peers, rendezvous_expected_peers_);
}

bool ExploPlannerNode::holdingForFinishedPeer() const {
  // Veto: a finished peer with an appointment still drives to the cell, so wait
  // until it says it is leaving. Only the appointment barrier waits; do not
  // move this into peerAccounted. (notes: finished-peer-still-coming-veto)
  if (!appointment_manoeuvre_) return false;
  // Bounded by rendezvous_latched_hold_sec_ from finished_peer_wait_start_sec_,
  // since the leaving may never arrive. An unstamped (negative) wait has not
  // started, so it has not expired. (notes: finished-peer-wait-bounded)
  if (finished_peer_wait_start_sec_ >= 0.0) {
    const double t = missionElapsedAt(this->now());
    if (t >= 0.0 &&
        t - finished_peer_wait_start_sec_ >= rendezvous_latched_hold_sec_)
      return false;
  }
  return finishedPeerStillComing(team_model_, fleet_.self_id) >= 0;
}

void ExploPlannerNode::refreshRendezvousSnapshot() {
  if (!rendezvous_schedule_enable_) return;
  if (!cell_world_.configured()) return;
  // Refresh once per rendezvous_proposal_period_sec_, not per heartbeat: the
  // CellWorld copy is costly on the single-threaded executor. Followers refresh
  // too; appointmentTravelMs and appointmentPoint read it.
  // (notes: rzv-snapshot-refresh-period)
  const double t_snap = missionElapsed();
  if (have_rendezvous_snapshot_ && t_snap >= 0.0 &&
      rendezvous_snapshot_at_sec_ >= 0.0 &&
      (t_snap - rendezvous_snapshot_at_sec_) < rendezvous_proposal_period_sec_) {
    return;
  }
  rendezvous_world_    = cell_world_;
  // TTL 0 (unbounded): the snapshot is refreshed only while the whole team is
  // in comms, so every peer position is seconds old by construction.
  // (notes: rzv-snapshot-ttl-zero)
  rendezvous_vehicles_ = allocVehicles(latest_pos_.x(), latest_pos_.y(),
                                       nullptr, missionElapsed(), 0.0);
  // Force finished and off_frontier off for every snapshot vehicle: the
  // scheduler drops such vehicles, and both robots must solve the same
  // allocation to agree on a meeting cell.
  // (notes: rzv-snapshot-force-flags-off)
  for (AllocRobot& v : rendezvous_vehicles_) {
    v.finished     = false;
    v.off_frontier = false;
  }
  have_rendezvous_snapshot_   = true;
  rendezvous_snapshot_at_sec_ = missionElapsed();
}

ExploPlannerNode::RendezvousProposal
ExploPlannerNode::deriveRendezvousProposal() {
  rendezvous_held_provenance_ = RendezvousProvenance{};
  RendezvousProposal out;
  if (!have_rendezvous_snapshot_) return out;
  // The origin the published instant is measured from. Refused rather than
  // defaulted: a negative mission clock means this node has not ticked live
  // yet, and an appointment stamped in a frame that does not exist yet is one
  // the peers would evaluate against a different zero.
  const double t_derive = missionElapsed();
  if (t_derive < 0.0) return out;

  // The allocation the schedule is inserted into. Solved HERE over the frozen
  // snapshot rather than reused from doPlan's live solve: doPlan solves the
  // world as it is now, which is the right problem for this robot's next hop
  // and the wrong one for a value that has to describe the whole team.
  const Allocation alloc = GlobalAllocator::solve(
      rendezvous_world_, rendezvous_vehicles_, alloc_cfg_);

  RendezvousScheduler::Config cfg = rzv_cfg_;
  // No exclusions: a per-robot no-show list cannot feed a value the whole team
  // must share. (notes: rzv-derive-no-exclusions)
  cfg.exclude.clear();
  // max_interval_ms is the appointment barrier's wait
  // (rendezvous_appointment_wait_sec), not the mid-run reconnect wait. 0 (wait
  // forever) means an uncapped interval, so the cap and capped are inert.
  // (notes: rzv-derive-interval-cap)
  cfg.max_interval_ms =
      static_cast<long long>(rendezvous_appointment_wait_sec_ * 1000.0);
  // The floor cell is the centroid of the snapshot's vehicle cells, so the
  // candidate set is never empty while a robot is locatable. The team may then
  // meet where no tour goes; floor_won records it.
  // (notes: rzv-derive-centroid-floor)
  int floor_cell = -1;
  {
    double sx = 0.0, sy = 0.0;
    int    n  = 0;
    for (const AllocRobot& v : rendezvous_vehicles_) {
      if (!rendezvous_world_.grid().valid(v.cell)) continue;
      float cx = 0.0f, cy = 0.0f;
      rendezvous_world_.grid().centre(v.cell, cx, cy);
      sx += cx; sy += cy; ++n;
    }
    if (n > 0) {
      floor_cell = rendezvous_world_.grid().idAt(
          static_cast<float>(sx / n), static_cast<float>(sy / n));
      // Convexity says this cannot happen; if the grid ever stops being a
      // rectangle it will, and standing on somebody's own cell is a meeting
      // point where a -1 is the empty candidate set all over again.
      if (!rendezvous_world_.grid().valid(floor_cell)) {
        for (const AllocRobot& v : rendezvous_vehicles_) {
          if (rendezvous_world_.grid().valid(v.cell)) { floor_cell = v.cell; break; }
        }
      }
    }
  }

  // Policy floor on rung spacing (the scheduler already floors at the furthest
  // direct drive). It sets the wait for the next occurrence and hybrid's chase
  // window; read per derive, not baked into rzv_cfg_.
  // (notes: rzv-derive-timetable-floor)
  cfg.min_interval_ms =
      static_cast<long long>(rendezvous_interval_sec_ * 1000.0);

  // Pass the real mission clock: t_meet is an absolute mission-elapsed instant
  // adopted verbatim over the wire, so every robot must evaluate it in the same
  // frame. (notes: rzv-derive-real-mission-clock)
  rendezvous_held_provenance_.plan = RendezvousScheduler::solve(
      rendezvous_world_, rendezvous_vehicles_, alloc, floor_cell,
      static_cast<long long>(t_derive * 1000.0), cfg);
  // Stamped HERE, from the snapshot the solve above actually read, so the
  // hashes on the eventual event name the world the argmin ran over rather
  // than whichever refresh happened last. See RendezvousProvenance.
  rendezvous_held_provenance_.shared_hash    = rendezvous_world_.sharedHash();
  rendezvous_held_provenance_.grid_hash      = rendezvous_world_.grid().configHash();
  rendezvous_held_provenance_.derived_at_sec = rendezvous_snapshot_at_sec_;

  if (!rendezvous_held_provenance_.plan.valid()) return out;
  if (rendezvous_held_provenance_.plan.t_meet_ms < 0) return out;

  // Warn when the agreed interval exceeds the barrier's wait. Keyed on the
  // inequality, not on capped (which only reports a cut tour term). Unreachable
  // while rendezvous_appointment_wait_sec is 0.
  // (notes: rzv-derive-findability-warn)
  if (cfg.max_interval_ms > 0 &&
      rendezvous_held_provenance_.plan.interval_ms > cfg.max_interval_ms) {
    RCLCPP_WARN(get_logger(),
        "Rendezvous: the agreed interval is %.0fs but the barrier only waits "
        "%.0fs — a robot setting off last is NOT guaranteed to find anyone "
        "still there, and will be logged as a no-show. Held at the floor "
        "anyway (furthest robot's drive %.0fs, floored=%d): a shorter interval "
        "does not shorten the drive.",
        rendezvous_held_provenance_.plan.interval_ms / 1000.0,
        cfg.max_interval_ms / 1000.0,
        rendezvous_held_provenance_.plan.tour_interval_ms / 1000.0,
        static_cast<int>(rendezvous_held_provenance_.plan.floored));
  }

  out.cell        = rendezvous_held_provenance_.plan.cell;
  out.interval_ms = rendezvous_held_provenance_.plan.interval_ms;
  out.t_meet_ms   = rendezvous_held_provenance_.plan.t_meet_ms;
  return out;
}

void ExploPlannerNode::maintainRendezvousProposal(bool team_mutual) {
  if (!rendezvous_schedule_enable_) return;
  // Pure pursuit has no agreed meeting point BY DESIGN — that absence is the
  // A/B against hybrid. Returning here keeps the arm's wire traffic honest as
  // well as its behaviour: a pursuit robot publishes no proposal, so a mixed
  // fleet cannot accidentally commit a pair half of it will never keep.
  if (reconnect_mode_ == ReconnectMode::PURSUIT) return;
  // No snapshot gate here: only deriveRendezvousProposal reads the snapshot, so
  // it is gated there. Adopting, echoing and committing must not wait on the
  // mutual-contact window. (notes: rzv-proposal-snapshot-gate-derive)
  const double t = missionElapsed();
  if (t < 0.0) return;
  if (static_cast<int>(rendezvous_peer_.size()) != fleet_.size()) return;
  if (static_cast<int>(rendezvous_peer_at_sec_.size()) != fleet_.size()) return;
  if (static_cast<int>(rendezvous_peer_confirmed_.size()) != fleet_.size()) return;
  // Check all four parallel vectors, including rendezvous_peer_provisional_,
  // which is indexed unconditionally below; do not rely on the caller resizing
  // them together. (notes: rzv-proposal-fourth-vector-guard)
  if (static_cast<int>(rendezvous_peer_provisional_.size()) != fleet_.size()) return;
  // A one-robot fleet would pass the commit loop vacuously and commit a private
  // appointment; refuse it. (notes: rzv-proposal-single-robot-guard)
  if (fleet_.size() < 2) return;

  const bool proposer = (fleet_.self_id == kRendezvousProposerId);

  // Only the proposer derives, only while team_mutual, and only with no pair
  // held, a provisional pair, or rendezvous_reagree_due_. The latch and commit
  // run on every robot. Do not gate the upgrade on mission_start_hold_sec_.
  // (notes: rzv-derive-gate-one-pair-per-meeting)
  if (proposer && team_mutual && have_rendezvous_snapshot_ &&
      (!rendezvous_held_.valid() || rendezvous_held_provisional_ ||
       rendezvous_reagree_due_)) {
    // Paces attempts, not successes: retry every
    // min(kRendezvousBootstrapPeriodSec, rendezvous_proposal_period_sec_),
    // never per heartbeat. The first attempt is immediate while
    // rendezvous_held_at_sec_ < 0. (notes: rzv-derive-attempt-pacing)
    const double derive_period =
        std::min(kRendezvousBootstrapPeriodSec, rendezvous_proposal_period_sec_);
    const bool due = rendezvous_held_at_sec_ < 0.0 ||
                     (t - rendezvous_held_at_sec_) >= derive_period;
    if (due) {
      // Stamped BEFORE the solve and on both outcomes: the period measures
      // time since the last DERIVATION ATTEMPT. Stamping only on success is
      // what let the refusal path spin.
      rendezvous_held_at_sec_ = t;
      // deriveRendezvousProposal resets rendezvous_held_provenance_ before
      // solving; saved so an attempt this block declines can restore it.
      // (notes: rzv-derive-saves-prev-provenance)
      const RendezvousProvenance prev_prov = rendezvous_held_provenance_;
      const RendezvousProposal p = deriveRendezvousProposal();
      if (p.valid()) {
        // PROVISIONAL means the argmin had nothing to choose between: the
        // allocator had produced no tour, so the only admissible cell was the
        // centroid fallback deriveRendezvousProposal always supplies. Keepable,
        // but not a decision — see rendezvous_held_provisional_.
        const bool now_provisional = rendezvous_held_provenance_.plan.candidates <= 1;
        // Adopt when nothing is held, or to upgrade a provisional pair or
        // re-agree after a kept meeting; both replacements need
        // !now_provisional so a centroid fallback never replaces a
        // tour-informed pair. (notes: rzv-proposer-adopt-rule)
        const bool adopt = !rendezvous_held_.valid() ||
                           ((rendezvous_held_provisional_ ||
                             rendezvous_reagree_due_) && !now_provisional);
        if (adopt) {
          if (p != rendezvous_held_) {
            RCLCPP_INFO(get_logger(),
                "Rendezvous proposal%s: cell %d, every %.0fs from t+%.0fs "
                "(penalty %lld mm over %d candidate(s)).",
                !rendezvous_held_.valid()
                    ? ""
                    : (rendezvous_held_provisional_
                           ? " (upgraded from the centroid fallback)"
                           : " (re-agreed after the team met)"),
                p.cell, p.interval_ms / 1000.0, p.t_meet_ms / 1000.0,
                rendezvous_held_provenance_.plan.penalty_mm,
                rendezvous_held_provenance_.plan.candidates);
          }
          rendezvous_held_             = p;
          rendezvous_held_provisional_ = now_provisional;
          // rendezvous_reagree_due_ is cleared by the adoption, not by the
          // attempt, so a declined provisional result retries while the team is
          // still gathered. (notes: rzv-reagree-due-consumed-by-adoption)
          rendezvous_reagree_due_      = false;
        } else {
          // Provisional over provisional: keep the published pair and restore
          // prev_prov. Re-adopting the drifting centroid would publish a new
          // generation every period and agreement would never finish.
          // (notes: rzv-keep-first-centroid-fallback)
          rendezvous_held_provenance_ = prev_prov;
        }
      } else {
        // A refused solve keeps the standing pair. A refusal means no locatable
        // robot or an unconfigured world; the WARN carries the candidate and
        // rejection counts to say why. (notes: rzv-refused-derive-keeps-pair)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
            "Rendezvous proposal: nothing derivable from the frozen snapshot "
            "(%s; %d candidate(s) survived, %d rejected unreachable, %d "
            "excluded). Keeping the standing pair.",
            rendezvous_held_provenance_.plan.refused.empty()
                ? "no plan" : rendezvous_held_provenance_.plan.refused.c_str(),
            rendezvous_held_provenance_.plan.candidates,
            rendezvous_held_provenance_.plan.rejected_unreachable,
            rendezvous_held_provenance_.plan.rejected_excluded);
        // Restore prev_prov after the WARN, which needs the refused search's
        // counts: the commit below can fire this tick and copy the provenance
        // into rendezvous_agreed_provenance_.
        // (notes: rzv-restore-provenance-after-warn)
        rendezvous_held_provenance_ = prev_prov;
      }
    }
  } else if (!proposer) {
    // else if (!proposer), not a bare else, which would drop the proposer into
    // the adopt path. Followers adopt the proposer's pair verbatim, not gated
    // on team_mutual; staleness does not clear the held pair.
    // (notes: rzv-follower-adopts-verbatim)
    const RendezvousProposal& p = rendezvous_peer_[kRendezvousProposerId];
    const bool p_prov = rendezvous_peer_provisional_[kRendezvousProposerId] != 0;
    // The decision is RendezvousHandshake::adopt (five booleans in, one of four
    // answers out), unit-tested there; this branch only logs and writes.
    // (notes: rzv-adopt-decision-in-handshake)
    using Adopt = RendezvousHandshake::Adopt;
    const Adopt decision = RendezvousHandshake::adopt(
        p.valid(), p_prov, rendezvous_held_.valid(),
        rendezvous_held_provisional_, p == rendezvous_held_,
        rendezvous_reagree_due_);
    if (decision == Adopt::kTake || decision == Adopt::kUpgrade ||
        decision == Adopt::kReagree) {
      if (decision == Adopt::kReagree) {
        // Spent on adoption, as the proposer does; left set, this robot would
        // accept every later generation unconditionally.
        // (notes: rzv-follower-spends-reagree-flag)
        rendezvous_reagree_due_ = false;
        RCLCPP_INFO(get_logger(),
            "Rendezvous proposal: adopting robot %d's next triple (re-agreed "
            "after the team met) — cell %d, every %.0fs from t+%.0fs (was "
            "cell %d at t+%.0fs).",
            kRendezvousProposerId, p.cell, p.interval_ms / 1000.0,
            p.t_meet_ms / 1000.0, rendezvous_held_.cell,
            rendezvous_held_.t_meet_ms / 1000.0);
      } else if (decision == Adopt::kUpgrade) {
        RCLCPP_INFO(get_logger(),
            "Rendezvous proposal: robot %d upgraded off the centroid fallback "
            "— adopting cell %d, every %.0fs from t+%.0fs (was cell %d).",
            kRendezvousProposerId, p.cell, p.interval_ms / 1000.0,
            p.t_meet_ms / 1000.0, rendezvous_held_.cell);
      } else {
        RCLCPP_INFO(get_logger(),
            "Rendezvous proposal: adopting robot %d's triple — cell %d, every "
            "%.0fs from t+%.0fs%s.", kRendezvousProposerId, p.cell,
            p.interval_ms / 1000.0, p.t_meet_ms / 1000.0,
            p_prov ? " (provisional)" : "");
      }
      rendezvous_held_             = p;
      rendezvous_held_provisional_ = p_prov;
      rendezvous_held_at_sec_      = t;
    } else if (decision == Adopt::kConflict) {
      // kConflict: the proposer's pair is not the adopted one, not the upgrade,
      // and not an owed replacement. A real fault: keep the adopted pair and
      // log both provisional flags loudly.
      // (notes: rzv-adopt-conflict-is-a-fault)
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 30000,
          "Rendezvous proposal: robot %d is publishing cell %d / every %.0fs "
          "from t+%.0fs (provisional=%d) but this robot already adopted cell "
          "%d / every %.0fs from t+%.0fs (provisional=%d). This is neither the "
          "permitted upgrade nor a replacement this robot is owed (it is not "
          "waiting on one); KEEPING the adopted triple. The commit gate will "
          "refuse until the fleet agrees again.",
          kRendezvousProposerId, p.cell, p.interval_ms / 1000.0,
          p.t_meet_ms / 1000.0, p_prov ? 1 : 0,
          rendezvous_held_.cell, rendezvous_held_.interval_ms / 1000.0,
          rendezvous_held_.t_meet_ms / 1000.0,
          rendezvous_held_provisional_ ? 1 : 0);
    }
  }

  // --- latch what the peers have been HEARD to hold ----------------------
  // Runs after the adopt branch so a follower latches on the same tick.
  // updatePeerLatch clears on any disagreement; keep its clear-then-relatch
  // order. (notes: rzv-peer-latch-update)
  for (int id = 0; id < fleet_.size(); ++id) {
    if (id == fleet_.self_id) continue;
    // Skip peers never heard from (rendezvous_peer_at_sec_ < 0): a
    // default-constructed slot is not a message and must not clear a latch.
    // (notes: rzv-latch-skip-never-heard-peer)
    if (rendezvous_peer_at_sec_[id] < 0.0) continue;
    RendezvousHandshake::updatePeerLatch(rendezvous_peer_[id],
                                         rendezvous_held_,
                                         rendezvous_peer_confirmed_[id]);
  }

  // --- the commit rule ---------------------------------------------------
  // Symmetric on every robot: commit when every peer's latch matches
  // rendezvous_held_. Freshness and mutual contact are deliberately not tested;
  // latches are first-hand evidence. (notes: rzv-commit-rule-symmetric)
  if (!rendezvous_held_.valid()) return;
  int on_pair = 0;
  for (int id = 0; id < fleet_.size(); ++id) {
    if (id == fleet_.self_id) continue;
    // Exact equality ignores latches for a pair this robot has superseded;
    // latches for a pair the peer superseded are cleared in the loop above.
    // (notes: rzv-latch-scoped-to-held-pair)
    if (rendezvous_peer_confirmed_[id] == rendezvous_held_) ++on_pair;
  }
  // Commit only when every peer has echoed this pair (on_pair == fleet-1), for
  // every commit and every N. Counted, then tested, so unanimity is a real
  // gate. (notes: rzv-commit-needs-every-echo)
  if (on_pair != fleet_.size() - 1) return;

  // rendezvous_agreed_at_sec_ is stamped only when the pair changes, so it is
  // the agreement's age. Reported, never enforced: a recurring schedule does
  // not expire. (notes: rzv-agreed-at-stamp-on-change)
  if (rendezvous_agreed_ != rendezvous_held_) {
    // "Rendezvous AGREED by all" is grepped by sim/rendezvous_agreement.py and
    // rendered by sim/rendezvous_agreement_calib.py; change all together. The
    // only text-log record of the agreed meeting time.
    // (notes: rzv-agreed-log-line-contract)
    RCLCPP_INFO(get_logger(),
        "Rendezvous AGREED by all %d robots: cell %d, interval %.0fs from "
        "t+%.0fs (provisional=%d, echoes %d/%d).",
        static_cast<int>(fleet_.size()), rendezvous_held_.cell,
        rendezvous_held_.interval_ms / 1000.0,
        rendezvous_held_.t_meet_ms / 1000.0,
        rendezvous_held_provisional_ ? 1 : 0,
        static_cast<int>(on_pair), static_cast<int>(fleet_.size()) - 1);
    rendezvous_agreed_        = rendezvous_held_;
    rendezvous_agreed_provisional_ = rendezvous_held_provisional_;
    // Triple and provenance are copied together here, the one place they
    // describe each other. On a follower the provenance is the empty plan.
    // (notes: rzv-agreed-provenance-copied-together)
    rendezvous_agreed_provenance_   = rendezvous_held_provenance_;
    rendezvous_agreed_at_sec_ = t;
  }
  // Re-asserted every tick, outside the change test. Always fleet-1 given the
  // return above, so a logged value below that means the commit rule was
  // bypassed. (notes: rzv-agreed-peers-assertion)
  rendezvous_agreed_peers_ = on_pair;
}

// The agreed triple is a recurring schedule: meetings at t_meet_ms + k *
// interval_ms. The occurrence arithmetic lives in planner_util.hpp so it is
// unit-testable; a non-positive interval yields the single agreed instant.
// (notes: rzv-arm-recurring-schedule)

bool ExploPlannerNode::armAppointment(const char* reason) {
  if (!rendezvous_schedule_enable_) return false;
  // Pure pursuit has no agreed meeting point BY DESIGN — see
  // rendezvous_schedule_enable_. Silent rather than a refusal event: the arm
  // is not choosing not to meet, the mechanism is not part of that arm at all,
  // and a stream of "refused: wrong mode" lines would make it look like one.
  if (reconnect_mode_ == ReconnectMode::PURSUIT) return false;

  // An armed appointment is kept, never re-derived: the peer holds the same
  // deadline. Only closeAppointment releases it. No event here, or the
  // agreed/outcome join breaks. (notes: rzv-arm-keep-standing-appointment)
  if (appointment_armed_) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous [%s]: an appointment at cell %d for t+%.0fs already "
        "stands; keeping it rather than re-deriving a deadline the peer is "
        "not going to move.", reason ? reason : "?", appointment_.cell,
        appointment_.t_meet_ms / 1000.0);
    return true;
  }

  // Nothing is solved here: arming takes the pair the team committed while
  // connected and reports what it did. (notes: rzv-arm-nothing-solved)
  RendezvousAgreedEvent ev;
  ev.proposer_id = kRendezvousProposerId;

  const double t_now = missionElapsed();
  ev.t_now_sec = t_now;
  double anchor = -1.0;
  if (t_now < 0.0) {
    // Every quantity here is mission-elapsed. Before the clock is live there
    // is no origin to measure a deadline from, and an absolute stamp would
    // not survive the pair (the two nodes start seconds apart).
    ev.refused = "mission clock is not live yet";
  } else if (rendezvous_spent_) {
    // One appointment per outage: re-arming a spent pair would set a past
    // t_meet, fire appointmentDue() at once and drive back to the cell just
    // left. (notes: rzv-arm-one-appointment-per-outage)
    ev.refused = "the agreed pair has already been used in this outage";
  } else if (!rendezvous_agreed_.valid()) {
    // No pair was ever committed by the whole team — early in a run, or a
    // fleet that never read complete. Refusing is the honest answer and the
    // caller falls through to the unscheduled manoeuvre; inventing a private
    // appointment here is exactly the behaviour being removed.
    ev.refused = "no (cell, interval, t_meet) triple was agreed by the whole "
                 "team before contact was lost";
  } else {
    // No anchor refusals: t_meet is the committed integer, so a suspect anchor
    // cannot move it. The anchor is still logged to tell an early agreement
    // from a late commit. (notes: rzv-arm-anchor-refusals-removed)
    if (have_rendezvous_anchor_) anchor = missionElapsedAt(
                                     rendezvous_anchor_time_);
    // No lateness refusal: arming attends the first occurrence not already past
    // (nextAgreedOccurrence), so the armed meeting cannot be behind the robot.
    // (notes: rzv-arm-lateness-refusal-removed)
  }

  if (ev.refused.empty()) {
    // Adopt cell, t_meet_ms and interval_ms as committed; attend the first
    // occurrence at or after t_now + arrival shortfall (zero inside the
    // lateness budget or with no travel estimate). Add no fixed notice to the
    // floor. (notes: rzv-arm-occurrence-floor)
    appointment_             = RendezvousPlan{};
    appointment_.cell        = rendezvous_agreed_.cell;
    appointment_.interval_ms = rendezvous_agreed_.interval_ms;
    const double shortfall_sec = arrivalShortfallSec(
        appointmentLeadMs(appointment_.cell), rendezvous_max_lateness_sec_);
    appointment_.t_meet_ms   = nextAgreedOccurrence(
        rendezvous_agreed_.t_meet_ms, rendezvous_agreed_.interval_ms,
        t_now + shortfall_sec);
    // Diagnostics, PROPOSER ONLY, and read from the provenance that was copied
    // alongside the pair at the commit site rather than from whatever the last
    // solve left behind — see RendezvousProvenance for the two ways the live
    // read got it wrong.
    if (fleet_.self_id == kRendezvousProposerId) {
      const RendezvousPlan& p = rendezvous_agreed_provenance_.plan;
      appointment_.penalty_mm           = p.penalty_mm;
      appointment_.floor_won            = p.floor_won;
      appointment_.candidates           = p.candidates;
      appointment_.rejected_unreachable = p.rejected_unreachable;
      appointment_.rejected_excluded    = p.rejected_excluded;
      // capped, floored and tour_interval_ms must be copied like the five
      // above: the logged row reads them off appointment_.
      // (notes: rzv-arm-copy-capped-floored)
      appointment_.capped               = p.capped;
      appointment_.floored              = p.floored;
      appointment_.tour_interval_ms     = p.tour_interval_ms;
    } else {
      // A follower never searched, so its counts get -1 (0 is a real search
      // result). floor_won has no sentinel.
      // (notes: rzv-arm-follower-row-sentinels)
      appointment_.penalty_mm           = -1;
      appointment_.candidates           = -1;
      appointment_.rejected_unreachable = -1;
      appointment_.rejected_excluded    = -1;
      appointment_.floor_won            = false;   // no sentinel exists; see doc
    }

    // Asked of the graph directly: costMm masks unreachable with a straight
    // line. Reported, never enforced; a follower keeps the team's pair even
    // with no route. (notes: rzv-arm-own-route-check)
    if (rendezvous_world_.configured()) {
      const int here = rendezvous_world_.grid().idAt(latest_pos_.x(),
                                                     latest_pos_.y());
      if (rendezvous_world_.grid().valid(here)) {
        const double d = rendezvous_world_.distance(here, appointment_.cell);
        ev.own_route_m = d;   // the graph's own negative sentinel passes through
        if (!(d >= 0.0)) {
          RCLCPP_WARN(get_logger(),
              "Rendezvous [%s]: keeping the agreed appointment at cell %d, but "
              "this robot's own map has NO ROUTE to it from cell %d — it will "
              "very likely record a no-show. Kept anyway: the pair is the "
              "team's, and an unreachable reading is as often unexplored "
              "ground as blocked ground.",
              reason ? reason : "?", appointment_.cell, here);
        }
      }
    }

    ev.from_agreed        = true;
    // Stamped with the COMMIT, not with what is held now: this row says what the
    // team agreed to, and an upgrade that lands after the commit does not
    // retroactively change what was armed here.
    ev.agreed_provisional = rendezvous_agreed_provisional_;
    ev.anchor_sec     = anchor;
    // Not clamped at zero: a negative age means the commit landed after the
    // separation. -1 when there is no anchor, since arming no longer requires
    // one. (notes: rzv-arm-agreed-age-unclamped)
    ev.agreed_age_sec = anchor >= 0.0 ? anchor - rendezvous_agreed_at_sec_
                                      : -1.0;
    ev.peers_on_pair  = rendezvous_agreed_peers_;
    // Proposer: the world the argmin ran over, from the provenance copied with
    // the pair, not the live snapshot. Follower: stays 0/-1.
    // (notes: rzv-arm-derivation-world-hashes)
    ev.shared_hash      = rendezvous_agreed_provenance_.shared_hash;
    ev.grid_hash        = rendezvous_agreed_provenance_.grid_hash;
    ev.snapshot_age_sec = rendezvous_agreed_provenance_.derived_at_sec >= 0.0
                              ? t_now - rendezvous_agreed_provenance_.derived_at_sec
                              : -1.0;

    ev.cell                 = appointment_.cell;
    ev.penalty_mm           = appointment_.penalty_mm;
    ev.floor_won            = appointment_.floor_won;
    ev.candidates           = appointment_.candidates;
    ev.rejected_unreachable = appointment_.rejected_unreachable;
    ev.rejected_excluded    = appointment_.rejected_excluded;
    // Read from the appointment, not hardcoded: deriveRendezvousProposal passes
    // the barrier wait as max_interval_ms, so capped means the interval was
    // pulled in. (notes: rzv-arm-capped-column)
    ev.capped               = appointment_.capped;
    // floored says whether the lattice floor rather than the objective set
    // interval_sec; capped cannot stand in for it.
    // (notes: rzv-arm-floored-column)
    ev.floored              = appointment_.floored;
    ev.tour_interval_sec    = appointment_.tour_interval_ms >= 0
                                  ? appointment_.tour_interval_ms / 1000.0
                                  : -1.0;
    ev.interval_sec         = appointment_.interval_ms / 1000.0;
    ev.t_meet_sec           = appointment_.t_meet_ms / 1000.0;
    // The committed t_meet before the roll; it differs from
    // appointment_.t_meet_ms by whole intervals exactly when
    // nextAgreedOccurrence rolled. (notes: rzv-arm-agreed-base-unrolled)
    ev.agreed_base_sec      = rendezvous_agreed_.t_meet_ms / 1000.0;

    // t_meet_ms cannot precede t_now: the floor is t_now plus a shortfall
    // clamped at zero, and intervals are positive. Keep the clamp.
    // (notes: rzv-arm-deadline-passed-unreachable)
  }

  const bool armed = ev.refused.empty() && appointment_.valid();
  if (armed) {
    appointment_armed_          = true;
    appointment_departed_       = false;
    appointment_arrived_        = false;
    appointment_chase_tried_    = false;
    appointment_unplaceable_    = false;
    appointment_armed_at_sec_   = t_now;
    appointment_arrived_at_sec_ = -1.0;
    rendezvous_spent_           = true;
    const long long travel = appointmentTravelMs();
    if (travel >= 0) {
      // Travel, not a departure deadline: the robot aims at ev.t_meet_sec and
      // departs when its live lead says so. Travel separates arrived-late from
      // never-close. (notes: rzv-arm-travel-covariate)
      ev.travel_sec = travel / 1000.0;
    }
  } else {
    // Write both flag and plan: they are read together downstream, and
    // appointment_armed_ being false here rests only on the early return at the
    // top. (notes: rzv-arm-refusal-clears-flag-and-plan)
    appointment_armed_       = false;
    appointment_unplaceable_ = false;
    appointment_             = RendezvousPlan{};
  }

  if (exp_log_) exp_log_->logRendezvousAgreed(expCtx(), ev);

  if (armed) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous schedule [%s]: cell %d, meeting at t+%.0fs — the pair "
        "agreed by this robot and %d peer(s) %.0fs before separation; my "
        "travel %.0fs.",
        reason, appointment_.cell, ev.t_meet_sec, ev.peers_on_pair,
        ev.agreed_age_sec, ev.travel_sec);
  } else {
    RCLCPP_WARN(get_logger(),
        "Rendezvous schedule [%s]: no appointment — %s. This arm has no "
        "unscheduled fallback; the robot keeps exploring.",
        reason, ev.refused.c_str());
  }
  return armed;
}

long long ExploPlannerNode::travelMsToCell(int cell) const {
  if (!rendezvous_world_.configured()) return -1;
  // BOTH ENDS BOUNDS-CHECKED. `here` has always been screened; `cell` was not,
  // because the only caller passed appointment_.cell and the arming site
  // bounds-checks that against cell_world_ when it comes off the wire. The rung
  // chooser now asks this BEFORE arming, so the screen has to live here.
  const int here = rendezvous_world_.grid().idAt(latest_pos_.x(),
                                                 latest_pos_.y());
  if (!rendezvous_world_.grid().valid(here)) return -1;
  if (!rendezvous_world_.grid().valid(cell)) return -1;
  return RendezvousScheduler::travelMs(
      GlobalAllocator::costMm(rendezvous_world_, here, cell),
      rzv_cfg_.speed_mm_s);
}

long long ExploPlannerNode::appointmentLeadMs(int cell) const {
  const long long travel = travelMsToCell(cell);
  if (travel < 0) return -1;
  // Must match the scheduler's reachability-floor markup
  // (rendezvous_scheduler.cpp: floor_ms); a different rate could need more lead
  // than the lattice spacing allows. (notes: rzv-lead-markup-matches-scheduler)
  return (travel * std::max(0, rzv_cfg_.depart_safety_milli)) / 1000;
}

long long ExploPlannerNode::appointmentTravelMs() const {
  // Either condition returns -1 (not armed: nothing to travel to; invalid: no
  // cell). Keep ||, not &&, so an armed-with-blank-plan state cannot reach
  // costMm on cell -1. (notes: rzv-travel-guard-either-condition)
  if (!appointment_armed_ || !appointment_.valid()) return -1;
  // From where the robot is now, not where it armed: the departure lead and the
  // travel_sec covariate both need it fresh.
  // (notes: rzv-travel-from-current-position)
  return travelMsToCell(appointment_.cell);
}

bool ExploPlannerNode::appointmentDue() {
  if (!appointment_armed_) return false;
  const double t = missionElapsed();
  if (t < 0.0) return false;
  const long long now_ms = static_cast<long long>(t * 1000.0);
  // Due once now + lead reaches t_meet, so the arrival lands on t_meet; with no
  // estimate (-1) it leaves at t_meet. Never re-choose the rung here. Use >=,
  // not ==: it is polled on the plan tick.
  // (notes: rzv-appointment-due-arrive-on-time)
  const long long lead_ms = appointmentLeadMs(appointment_.cell);
  if (lead_ms < 0) return now_ms >= appointment_.t_meet_ms;
  return now_ms + lead_ms >= appointment_.t_meet_ms;
}

Eigen::Vector3f ExploPlannerNode::appointmentPoint() {
  float x = 0.0f, y = 0.0f;
  // Screen with valid() first: CellGrid::col() is id % nx and nx is 0 on an
  // unconfigured grid. Prefer the snapshot grid; fall back to cell_world_,
  // which has the same geometry. (notes: rzv-appointment-point-grid-fallback)
  const CellWorld* w = nullptr;
  if (rendezvous_world_.configured() &&
      rendezvous_world_.grid().valid(appointment_.cell)) {
    w = &rendezvous_world_;
  } else if (cell_world_.configured() &&
             cell_world_.grid().valid(appointment_.cell)) {
    w = &cell_world_;
  }
  if (w == nullptr) {
    // Neither grid can place the cell: hold position (anything else is
    // undefined behaviour) and latch appointment_unplaceable_ so the row is not
    // read as a no-show blamed on the peers.
    // (notes: rzv-appointment-unplaceable-hold)
    appointment_unplaceable_ = true;
    RCLCPP_ERROR(get_logger(),
        "Rendezvous: appointment cell %d cannot be placed on either the "
        "snapshot grid (configured=%d) or the live grid (configured=%d). "
        "Holding position instead of departing — this robot will record a "
        "no-show. The appointment was adopted off the wire without this robot "
        "ever having frozen a snapshot.",
        appointment_.cell, static_cast<int>(rendezvous_world_.configured()),
        static_cast<int>(cell_world_.configured()));
    return latest_pos_;
  }
  // Placeable after all — clear the latch rather than leave a stale true. The
  // cell does not change within one appointment, so in practice this only
  // matters if a grid became configured between two departure reads, but a
  // latch nobody clears is a latch that eventually reports the wrong run.
  appointment_unplaceable_ = false;
  w->grid().centre(appointment_.cell, x, y);
  // z from the robot's own frame: the drive and its arrival test are planar
  // (goal_xy_tol_), and the cell grid carries no height.
  return Eigen::Vector3f(x, y, latest_pos_.z());
}

void ExploPlannerNode::closeAppointment(const char* outcome, bool arrived,
                                        double waited_sec, bool mutual) {
  if (!appointment_armed_) return;
  const double t_end = missionElapsed();

  RendezvousOutcomeEvent ev;
  ev.cell         = appointment_.cell;
  ev.t_meet_sec   = appointment_.t_meet_ms / 1000.0;
  ev.t_end_sec    = t_end;
  ev.lateness_sec = (t_end >= 0.0) ? (t_end - ev.t_meet_sec) : 0.0;
  ev.outcome      = outcome;
  ev.arrived      = arrived;
  ev.waited_sec   = waited_sec;
  ev.mutual       = mutual;
  if (exp_log_) exp_log_->logRendezvousOutcome(expCtx(), ev);

  RCLCPP_INFO(get_logger(),
      "Rendezvous appointment at cell %d closed: %s (%s, %.0fs %s, waited "
      "%.0fs).", appointment_.cell, outcome,
      arrived ? "arrived" : "never arrived", std::fabs(ev.lateness_sec),
      ev.lateness_sec >= 0.0 ? "late" : "early", waited_sec);

  // A no-show does not write the cell off: the pair is frozen on separation and
  // there is no second arming, so a per-robot write-off could only push the two
  // ends apart. (notes: rzv-close-no-cell-writeoff)

  appointment_armed_       = false;
  appointment_departed_    = false;
  appointment_arrived_     = false;
  appointment_chase_tried_ = false;
  appointment_unplaceable_ = false;
  appointment_             = RendezvousPlan{};
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
  // Frozen so nothing re-derives a destination off a mid-manoeuvre packet. Only
  // the hold escalation's own-anchor fallback reads it; the destination is the
  // agreed appointment cell. (notes: reconnect-rec-frozen-per-manoeuvre)
  have_reconnect_rec_ = (rec != nullptr);
  if (rec != nullptr) reconnect_rec_ = *rec;
  if (rec == nullptr) {
    // The missing teammate was never heard at all (the anchor came from a
    // different peer), so there is nothing to chase. The appointment is
    // unaffected — it never read this record.
    reconnect_decline_reason_ = "no-peer-record";
  }

  // ---- P5: the appointment, and the timeline it partitions -------------
  // Arm first: the chase owns time before the departure deadline, the
  // appointment after. Terminal dispatch skips the deferral; hybrid still
  // chases if not due. (notes: dispatch-p5-timeline-partition)
  const bool have_appointment = armAppointment(reason);
  const bool may_defer = have_appointment && !reconnect_terminal_;
  const bool hybrid_terminal_chase =
      have_appointment && reconnect_terminal_ &&
      reconnect_mode_ == ReconnectMode::HYBRID && !appointmentDue();
  if (have_appointment && (reconnect_terminal_ || appointmentDue()) &&
      !hybrid_terminal_chase) {
    // Either exploration is over, or the deadline has already passed while
    // the silence clock was still counting — a long outage against a nearby
    // meeting. Go now.
    startReturnTo(appointmentPoint(), "appointment", reason);
    appointment_departed_ = true;
    return true;
  }

  if (reconnect_mode_ != ReconnectMode::RENDEZVOUS && rec != nullptr &&
      startPursuit(peer_id, *rec, reason)) {
    if (appointment_armed_) appointment_chase_tried_ = true;
    return true;
  }
  if (reconnect_mode_ == ReconnectMode::HYBRID) {
    // Chase declined or no record: hybrid does exactly what pursuit does (these
    // lines copy that branch; add no midpoint drive). may_defer keeps exploring
    // until the appointment is due. (notes: dispatch-hybrid-mirrors-pursuit)
    if (may_defer) return false;
    // The terminal chase declined, so depart for the appointment. Must precede
    // the explore fallback: exploration is already over at a terminal dispatch.
    // (notes: dispatch-hybrid-terminal-chase-declined)
    if (hybrid_terminal_chase) {
      startReturnTo(appointmentPoint(), "appointment", reason);
      appointment_departed_    = true;
      appointment_chase_tried_ = true;
      return true;
    }
    if (pursuitExploreFallback(reason)) return true;
    holdForTeam(reason);
    return true;
  }
  if (reconnect_mode_ == ReconnectMode::PURSUIT) {
    // Pure pursuit has no agreed fallback point by design. A chase that never
    // started explores while its fallback budget lasts and parks only once it
    // is spent (see pursuit_explore_fallback_).
    // (notes: dispatch-pursuit-explore-before-park)
    if (pursuitExploreFallback(reason)) return true;
    holdForTeam(reason);
    return true;
  }
  // RENDEZVOUS: the arm is the appointment or nothing. may_defer means keep
  // exploring until due; otherwise armAppointment refused (and logged a
  // rendezvous_agreed row). No midpoint drive.
  // (notes: dispatch-rendezvous-no-fallback)
  if (!may_defer) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
        "Rendezvous [%s]: no appointment stands and this arm has no "
        "unscheduled fallback — continuing to explore.", reason);
  }
  return false;
}

// Flicker guard: the release condition must hold for
// reconnect_release_confirm_sec_ (dwellConfirmed in planner_util). release_ok_*
// are the barrier's alone; do not fold them into team_back_ok_*.
// (notes: release-confirm-flicker-dwell)
bool ExploPlannerNode::releaseConfirmed(bool eligible) {
  return dwellConfirmed(eligible, this->now().seconds(),
                        reconnect_release_confirm_sec_, &release_ok_armed_,
                        &release_ok_since_sec_);
}

// Read-only twin of releaseConfirmed (dwellHeld) for callers such as the
// outcome classifier that must not advance the dwell. Must pass the same three
// inputs as releaseConfirmed. (notes: release-held-read-only-twin)
bool ExploPlannerNode::releaseHeld(bool eligible) const {
  return dwellHeld(eligible, this->now().seconds(),
                   reconnect_release_confirm_sec_, release_ok_armed_,
                   release_ok_since_sec_);
}

// Barrier-entry stand-down. The barrier is hard: RETURN/PURSUE never check
// target_queue_. Demote the active target to PENDING and reset to EXPLORE;
// doPlan() resumes it after the release.
// (notes: barrier-stand-down-exploitation)
void ExploPlannerNode::standDownExploitation() {
  if (target_queue_.active() || target_queue_.hasPending()) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: standing down exploitation (%zu target(s) still open) — "
        "the barrier takes priority; they resume after the team reconnects.",
        target_queue_.pendingCount());
    target_queue_.deactivate();
    phase_ = Phase::EXPLORE;
    // Stop the give-up timer too: the demoted target returns with the same id,
    // so the drive and barrier wait would otherwise count against
    // exploit_target_timeout_sec.
    // (notes: barrier-stand-down-stops-exploit-timer)
    exploit_target_timing_ = false;
  }
}

// Arm the drive to a barrier destination (the agreed appointment cell, or this
// robot's last-connected anchor as fallback) using NAVIGATE's timeout and
// arrival test. The presence intent prevents a two-anchor deadlock.
// (notes: start-return-to-barrier-drive)
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
  return_dest_       = dest;
  current_goal_ = CandidateViewpoint{};
  current_goal_.position = dest;
  current_goal_.yaw = latest_yaw_;

  // Derived once from what, the only input that survives into RETURN_NAV. Must
  // not be folded into appointment_armed_: transitionTo(RETURN_NAV) can close
  // the appointment on this departure tick.
  // (notes: start-return-appointment-manoeuvre-latch)
  appointment_manoeuvre_ = (std::strcmp(what, "appointment") == 0);
  // A new leg is on the road, not at a barrier and not mid-manoeuvre: whatever
  // brought the last one to a stop is spent, and so is the escape ladder it
  // spent getting there. Cleared here rather than at the manoeuvre end so a
  // resumed leg cannot resume itself.
  appointment_settle_converted_ = false;
  appointment_escape_active_    = false;
  appointment_escapes_used_     = 0;
  // The finished-peer wait is not restarted by a new leg, only dropped by a
  // non-appointment leg; restarting it would let a robot cycling
  // stop-short/resume wait forever. (notes: start-return-finished-peer-wait)
  if (!appointment_manoeuvre_) {
    finished_peer_wait_start_sec_      = -1.0;
    finished_peer_wait_expired_logged_ = false;
  }

  RCLCPP_INFO(get_logger(),
      "Rendezvous: dispatched [%s], team incomplete (%d/%d peers) "
      "-> returning to %s (%.2f, %.2f).",
      reason,
      accountedPeerCount(this->now()),
      rendezvous_expected_peers_, what, dest.x(), dest.y());

  // The action string names where this leg went: the appointment cell or this
  // robot's own anchor. Analyses read this field, so it must match the
  // manoeuvre. (notes: reconnect-dispatch-action-label)
  refreshDispatchContext();
  logReconnectDispatch(
      appointment_manoeuvre_ ? "appointment" : "anchor_return",
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
  armManoeuvreLegBudget(std::sqrt(dx * dx + dy * dy));
}

// Manoeuvre legs are exempt from nav_max_timeout_sec: distance-true budget with
// the nav_min_timeout_sec_ floor, capped by reconnect_nav_max_sec_ when > 0.
// The no-progress window stays the primary stuck guard.
// (notes: manoeuvre-leg-budget-exempt-cap)
void ExploPlannerNode::armManoeuvreLegBudget(float dist) {
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

// Drive to the appointment cell or own anchor, then hand off to RETURN_SYNC. An
// anchor return re-plans if the team reconnects en route; an appointment never
// releases here but enters RETURN_SYNC once the team settles.
// (notes: return-nav-appointment-settle-join)
void ExploPlannerNode::doReturnNav() {
  // accountedPeerCount — presence semantics, same note as finishOrRendezvous().
  const int active = accountedPeerCount(this->now());
  // Guard before calling releaseConfirmed: it arms and disarms its dwell as a
  // side effect. manoeuvreReleaseEligible is deliberately redundant with the
  // guard so every release site shares one predicate.
  // (notes: return-nav-release-guard-order)
  if (!appointment_manoeuvre_ && releaseConfirmed(manoeuvreReleaseEligible(active))) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: team reconnected en route (%d/%d) -> re-planning against "
        "merged map.", active, rendezvous_expected_peers_);
    // RETURN_NAV is a driving state: stop the platform before PLAN. doPlan
    // can spend ticks retrying (map load, all candidates rejected) with the
    // proximity guard off, and the navigator would keep executing the barrier goal
    // underneath it the whole time (see abandonNavGoal).
    abandonNavGoal("return-released");
    // Re-confirm saturation against the post-merge map instead of finishing
    // off the pre-barrier streak on the first PLAN tick.
    coverage_done_streak_ = 0;
    have_active_intent_ = false;
    transitionTo(State::PLAN, "return-released");
    return;
  }

  // Measure against return_dest_, not current_goal_.position: an escape leg
  // borrows current_goal_ to steer around a stall. The two are equal on an
  // ordinary drive. (notes: return-nav-measure-return-dest)
  const auto robot_pos = latest_pos_;
  const float dx = robot_pos.x() - return_dest_.x();
  const float dy = robot_pos.y() - return_dest_.y();
  const float dist = std::sqrt(dx * dx + dy * dy);

  // reconnect_arrive_tol_m_, NOT goal_xy_tol_: the destination's value is
  // connectivity, not position (see the member). Both robots stopping within
  // this of the same destination leaves them <= 2x it apart, and it turns an
  // obstructed destination from a budget burn into an arrival beside it.
  if (dist < static_cast<float>(reconnect_arrive_tol_m_)) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: reached %s (dist=%.2f, tol=%.1f) -> waiting for team.",
        return_dest_label_.c_str(), dist, reconnect_arrive_tol_m_);
    // Stop explicitly: arrival by tolerance precedes the navigator finishing
    // its goal. The arrival stamp keys on appointment_manoeuvre_, not armed &&
    // departed, since the record can close on the departure tick.
    // (notes: return-nav-arrival-stop-and-stamp)
    if (appointment_manoeuvre_ && !appointment_arrived_) {
      appointment_arrived_        = true;
      appointment_arrived_at_sec_ = missionElapsed();
    }
    abandonNavGoal("return-arrived");
    transitionTo(State::RETURN_SYNC, "return-arrived");
    return;
  }

  // An appointment walker whose team has settled enters RETURN_SYNC from here.
  // Read team_back_ok_* via dwellHeld (heartbeatTick owns the clock), not
  // release_ok_*, which is frozen here; appointment_arrived_ stays false.
  // (notes: return-nav-settle-conversion)
  if (appointment_manoeuvre_ &&
      dwellHeld(teamSettled(active), this->now().seconds(),
                reconnect_release_confirm_sec_, team_back_ok_armed_,
                team_back_ok_since_sec_)) {
    RCLCPP_INFO(get_logger(),
        "Rendezvous: team settled while driving to %s (dist=%.2f) -> joining "
        "the barrier from here.", return_dest_label_.c_str(), dist);
    appointment_settle_converted_ = true;
    abandonNavGoal("return-team-settled");
    transitionTo(State::RETURN_SYNC, "return-team-settled");
    return;
  }

  // Escape-leg bookkeeping, placed after the arrival and settle exits so those
  // win. The leg ends at escape_target_ or its cap; resumeAppointmentDrive
  // restores the goal. kEscapeArriveM ends the detour, not the meeting.
  // (notes: return-nav-escape-leg-termination)
  if (appointment_escape_active_) {
    const float d = (latest_pos_ - escape_target_).head<2>().norm();
    const double leg = (this->now() - escape_leg_start_).seconds();
    if (d < kEscapeArriveM || leg > return_escape_leg_sec_) {
      resumeAppointmentDrive(d < kEscapeArriveM ? "escape-arrived"
                                                : "escape-leg-cap");
    } else {
      republishGoal(current_goal_);
    }
    return;
  }

  const auto now = this->now();
  const double elapsed = (now - state_enter_time_).seconds();
  // Budget or no-progress: stop the drive explicitly (RETURN_SYNC assumes a
  // stationary robot) and wait from here. appointmentLegWatchdog is asked
  // first, since stopping short of an appointment is not arriving.
  // (notes: return-nav-watchdog-exits)
  if (elapsed > nav_budget_sec_) {
    if (appointmentLegWatchdog("budget", dist)) return;
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
      if (appointmentLegWatchdog("no-progress", dist)) return;
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

// Gated on appointment_manoeuvre_, not the record. A watchdog fire escalates to
// escape legs, then rolls to a later makeable rung and exits to PLAN, clearing
// appointment_departed_ first so transitionTo keeps the appointment.
// (notes: appointment-leg-watchdog-ladder)
bool ExploPlannerNode::appointmentLegWatchdog(const char* what_failed,
                                              float dist) {
  if (!appointment_manoeuvre_) return false;
  // Close enough that the barrier still means something — see
  // rendezvous_present_tol_m_. Escalating these would spend the ladder to fix a
  // few metres, and the robot is already where the others are heading.
  if (dist <= static_cast<float>(rendezvous_present_tol_m_)) return false;

  // RUNG 1..N: escape, then aim at the same cell again. No guard on
  // appointment_escape_active_ is needed — doReturnNav's escape block returns
  // before both watchdogs, so this line is only ever reached between legs and
  // the counter is always a count of ENDED ones.
  if (appointment_escapes_used_ < rendezvous_escape_max_attempts_) {
    const bool on_crumb = pickEscapeTarget();
    ++appointment_escapes_used_;
    appointment_escape_active_ = true;
    escape_leg_start_          = this->now();
    // return_dest_ is deliberately NOT touched: the detour changes where the
    // wheels point, not where the meeting is, and keeping the two apart is what
    // makes the arrival test above immune to this whole manoeuvre.
    current_goal_.position     = escape_target_;
    current_goal_.yaw          = latest_yaw_;
    // Log return_dest_, not appointment_.cell: the record can be closed
    // underneath a live leg, leaving the cell -1.
    // (notes: appointment-escape-log-uses-dest)
    RCLCPP_WARN(get_logger(),
        "Rendezvous: the %s watchdog ended the drive to the agreed cell at "
        "(%.2f, %.2f) %.1f m out -> escape %d/%d, driving %.2f m to %s "
        "(%.2f, %.2f) before aiming at the cell again.",
        what_failed, return_dest_.x(), return_dest_.y(), dist,
        appointment_escapes_used_, rendezvous_escape_max_attempts_,
        (escape_target_ - latest_pos_).head<2>().norm(),
        on_crumb ? "a breadcrumb" : "a pose behind the robot",
        escape_target_.x(), escape_target_.y());
    logAppointmentLegRow("escape", what_failed, dist);
    // No brake first: the replacement goal goes out this tick and supersedes
    // the old one; a brake published alongside it would be overwritten.
    // (notes: appointment-escape-no-brake)
    publishGoal(current_goal_);
    // The budget and the no-progress window are deliberately left as they are.
    // Both are read only by the watchdogs below, which this leg cannot reach,
    // and resumeAppointmentDrive re-arms both from the escape's end pose.
    // Re-arming here as well would be dead state overwritten within the leg.
    return true;
  }

  // THE TERMINAL RUNG, taken with or without a record to roll: the exit to PLAN
  // is what drops the unbounded barrier patience, so a leg whose appointment was
  // closed underneath it takes the exit too and simply has no rung to move.
  char rung[128];
  double rolled_to_sec = -1.0;
  if (appointment_armed_ && appointment_.valid()) {
    // Roll strictly forward: the floor is max(t_meet_ms, now) + 1 ms
    // (nextAgreedOccurrence keeps >= floor), raised by arrivalShortfallSec. An
    // unpriceable cell gives shortfall 0.0, the nearest rung.
    // (notes: appointment-roll-strict-floor)
    const long long now_ms =
        static_cast<long long>(std::llround(missionElapsed() * 1000.0));
    const long long floor_ms = std::max(appointment_.t_meet_ms, now_ms) + 1;
    const double shortfall_sec = arrivalShortfallSec(
        appointmentLeadMs(appointment_.cell), rendezvous_max_lateness_sec_);
    // Strictly later than the abandoned rung: the floor is above it and valid()
    // ensures interval_ms > 0. Do not add a return false here; that falls
    // through into the unbounded barrier. (notes: appointment-roll-lands-later)
    appointment_.t_meet_ms =
        nextAgreedOccurrence(appointment_.t_meet_ms, appointment_.interval_ms,
                             floor_ms / 1000.0 + shortfall_sec);
    rolled_to_sec = appointment_.t_meet_ms / 1000.0;
    std::snprintf(rung, sizeof(rung),
                  "rolling to t+%.0fs and exploring until then", rolled_to_sec);
  } else {
    std::snprintf(rung, sizeof(rung),
                  "the appointment was already closed, so back to exploring");
  }

  RCLCPP_WARN(get_logger(),
      "Rendezvous: the %s watchdog ended the drive to the agreed cell at "
      "(%.2f, %.2f) %.1f m out (present within %.1f m) with %d/%d escapes "
      "spent -> not at the meeting; %s.",
      what_failed, return_dest_.x(), return_dest_.y(), dist,
      rendezvous_present_tol_m_, appointment_escapes_used_,
      rendezvous_escape_max_attempts_, rung);
  // Before the exit, not after: transitionTo(PLAN) clears the manoeuvre and can
  // close the record, and both are columns on this row.
  logAppointmentLegRow("unreached", what_failed, dist, -1.0, rolled_to_sec);

  // A coverage_latched_ robot does not re-plan: end the run (home or finish)
  // instead. The rolled rung is written and logged first, since the rest of the
  // team still keeps it. (notes: appointment-unreached-latched-ends)
  if (coverage_latched_) {
    coverage_latch_hold_start_sec_ = -1.0;
    coverage_latch_teardown_ = true;
    abandonNavGoal("appointment-unreached");
    RCLCPP_WARN(get_logger(),
        "Rendezvous: this robot had already finished exploring when the drive "
        "to the agreed cell ran out of rungs -> ending the run here instead of "
        "re-planning. It has no exploring left to return to.");
    if (mission_return_enabled_ && have_home_) {
      (void)startReturnHome("appointment-unreached");
      return true;
    }
    (void)finishNow("appointment-unreached");
    return true;
  }

  appointment_departed_ = false;
  // RETURN_NAV is a driving state and PLAN can spend ticks before it publishes
  // anything, so the navigator must be stopped explicitly or it keeps executing
  // the abandoned meeting goal underneath the re-plan (see abandonNavGoal).
  abandonNavGoal("appointment-unreached");
  // Same reason as the en-route release above: the saturation streak was
  // accumulated before the outage and must be re-confirmed against whatever the
  // map looks like now.
  coverage_done_streak_ = 0;
  have_active_intent_ = false;
  transitionTo(State::PLAN, "appointment-unreached");
  return true;
}

// Resume the drive to return_dest_, never a fresh appointmentPoint(), which can
// answer latest_pos_. No state change: RETURN_NAV is already the state.
// (notes: appointment-resume-same-cell)
void ExploPlannerNode::resumeAppointmentDrive(const char* why) {
  const double leg_sec = (this->now() - escape_leg_start_).seconds();
  appointment_escape_active_ = false;
  current_goal_.position     = return_dest_;
  current_goal_.yaw          = latest_yaw_;
  const float dist = (current_goal_.position - latest_pos_).head<2>().norm();
  RCLCPP_WARN(get_logger(),
      "Rendezvous: appointment escape %d/%d ended [%s] after %.1f s -> driving "
      "at the agreed cell (%.2f, %.2f) again, %.2f m out.",
      appointment_escapes_used_, rendezvous_escape_max_attempts_, why, leg_sec,
      return_dest_.x(), return_dest_.y(), dist);
  logAppointmentLegRow("escape-end", why, dist, leg_sec);
  publishGoal(current_goal_);
  // A resumed leg is a new leg: re-arm the budget and progress window or the
  // ladder burns in consecutive ticks. armManoeuvreLegBudget reads
  // state_enter_time_, so stamp it directly.
  // (notes: appointment-resume-rearm-budget)
  state_enter_time_ = this->now();
  armManoeuvreLegBudget(dist);
}

// Logs one AppointmentLegEvent per ladder rung. The cell comes from the record
// (-1 once closed), the position from return_dest_, which stays true after the
// record closes. (notes: appointment-leg-row-instrument)
void ExploPlannerNode::logAppointmentLegRow(const char* kind,
                                            const char* cause, float dist,
                                            double leg_sec,
                                            double rolled_to_sec) {
  if (!exp_log_) return;
  AppointmentLegEvent ev;
  ev.kind          = kind;
  ev.cause         = cause;
  ev.dist_m        = dist;
  ev.dest_x        = return_dest_.x();
  ev.dest_y        = return_dest_.y();
  ev.cell          = appointment_armed_ ? appointment_.cell : -1;
  ev.escapes_used  = appointment_escapes_used_;
  ev.escapes_max   = rendezvous_escape_max_attempts_;
  ev.leg_sec       = leg_sec;
  ev.rolled_to_sec = rolled_to_sec;
  exp_log_->logAppointmentLeg(expCtx(), ev);
}

// Hold until the whole team is back in comms, then re-plan; a wait cap <= 0 is
// unbounded. The heartbeat keeps broadcasting presence so arriving peers can
// release their own barriers. (notes: return-sync-overview)
void ExploPlannerNode::doReturnSync() {
  // accountedPeerCount — presence semantics, same note as finishOrRendezvous().
  const int active = accountedPeerCount(this->now());
  // THE WAIT FOR A FINISHED PEER STARTS HERE, OR WHEN THE MEETING FALLS DUE,
  // WHICHEVER IS LATER — the same floor the latched hold's own stamp takes, for
  // the same reason: an early arrival must not spend the bound before the team
  // is due. See finished_peer_wait_start_sec_ and holdingForFinishedPeer.
  if (appointment_manoeuvre_ && finished_peer_wait_start_sec_ < 0.0) {
    const double t_now = missionElapsed();
    double t_wait = (t_now >= 0.0) ? t_now : 0.0;
    if (appointment_.valid())
      t_wait = std::max(t_wait, appointment_.t_meet_ms / 1000.0);
    finished_peer_wait_start_sec_ = t_wait;
  }

  // Log the finished-peer hold, and its expiry once. Placed above the release
  // gate: the expiry is what opens the gate, and the settle below returns
  // before later lines run. (notes: return-sync-finished-peer-log)
  if (appointment_manoeuvre_ && finished_peer_wait_start_sec_ >= 0.0) {
    const int coming = finishedPeerStillComing(team_model_, fleet_.self_id);
    const double t_now = missionElapsed();
    const double on_it =
        (t_now >= 0.0) ? t_now - finished_peer_wait_start_sec_ : 0.0;
    if (coming >= 0 && holdingForFinishedPeer()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
          "Rendezvous: %s finished exploring but has not said it is leaving "
          "-> still waiting for it here (%.0fs of %.0fs).",
          fleet_.nameOf(coming).c_str(), std::max(0.0, on_it),
          rendezvous_latched_hold_sec_);
    } else if (coming >= 0 && !finished_peer_wait_expired_logged_) {
      finished_peer_wait_expired_logged_ = true;
      RCLCPP_WARN(get_logger(),
          "Rendezvous: stopped waiting for %s after %.0fs: it finished "
          "exploring but neither arrived nor said it was leaving.",
          fleet_.nameOf(coming).c_str(), rendezvous_latched_hold_sec_);
    }
  }

  // Release on manoeuvreReleaseEligible: for an appointment, (teamSettled or
  // every expected peer reachable in the comms closure) and no peer still
  // inbound; for midrun or terminal reconnects, teamComplete.
  // (notes: return-sync-five-site-barrier)
  if (releaseConfirmed(manoeuvreReleaseEligible(active))) {
    // After the barrier releases, hold in RETURN_SYNC (not a sleep) so the maps
    // can merge while the robot keeps heartbeating and re-checking the team.
    // Only an appointment settles; timed on this->now().
    // (notes: return-sync-settle-hold)
    const auto settle_now = this->now();
    // Mesh count for a mesh release, closure count for a door release.
    // sim/manoeuvre_events.py RE_REJOIN greps the prefix "Rendezvous: team
    // reachable"; keep it. jsonl rows carry the raw mesh count.
    // (notes: return-sync-rejoin-log-count)
    const int present = std::max(active, reachablePeerCount());
    double settled_sec = 0.0;
    if (appointment_manoeuvre_ && rendezvous_settle_sec_ > 0.0) {
      if (!rendezvous_settling_) {
        rendezvous_settling_    = true;
        rendezvous_settle_start_ = settle_now;
        // STAMPED WHERE THE HOLD STARTS, because that is the instant the
        // exchange starts: the team has just become reachable and none of the
        // peer's voxels have crossed the emulator yet. Anything that arrives
        // between here and the release is what the meeting bought.
        rendezvous_exchange_ = MapExchangeBaseline{
            true, latest_map_voxels_, cell_world_.sharedHash(),
            team_merge_applied_total_, peer_fusion_deltas_};
        RCLCPP_INFO(get_logger(),
            "Rendezvous: team reachable (%d/%d) -> holding %.0fs for the "
            "map exchange before re-planning.",
            present, rendezvous_expected_peers_, rendezvous_settle_sec_);
      }
      settled_sec = (settle_now - rendezvous_settle_start_).seconds();
      // Only the hold's end condition differs: the clock path waits
      // rendezvous_settle_sec_; the drain path waits until every
      // believed-present peer has delivered. Everything below is shared.
      // (notes: return-sync-hold-trigger)
      if (!rendezvous_drain_released_) {
        if (!rendezvous_drain_release_) {
          if (settled_sec < rendezvous_settle_sec_) {
            return;   // still settling; stay put, keep heartbeating.
          }
        } else {
          // The predicate is stepDrainRelease (exchange_drain.cpp): a level
          // against the hold-start baseline, then a rate over a tumbling
          // window, per believed-present peer. This block keeps the state, logs
          // and latch. (notes: return-sync-drain-predicate)
          const DrainReading drain = stepDrainRelease(
              rendezvous_drain_window_, settled_sec, peer_fusion_deltas_,
              rendezvous_exchange_.peer_deltas, team_model_, fleet_.size(),
              fleet_.self_id, rendezvous_drain_rate_vox_sec_,
              rendezvous_drain_window_sec_, rendezvous_latched_hold_sec_);
          if (drain.evaluated && !drain.measurable) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "Rendezvous: drain release has no per-peer counters to read "
                "(dscovox has published none this meeting). Holding to the "
                "%.0fs cap.", rendezvous_latched_hold_sec_);
          }
          if (drain.step == DrainStep::kHold) {
            return;   // keep standing on the cell; keep heartbeating.
          }
          if (drain.step == DrainStep::kUnfinished) {
            // Leaving on the cap with a mute peer is not a drain release, so it
            // is logged as UNFINISHED EXCHANGE.
            // (notes: return-sync-unfinished-exchange)
            RCLCPP_WARN(get_logger(),
                "Rendezvous: UNFINISHED EXCHANGE — %.0fs cap reached; %d of %d "
                "readable peer(s) delivered nothing at all since the hold "
                "began. Leaving on the cap, not on the drain.",
                rendezvous_latched_hold_sec_, drain.mute, drain.examined);
          } else if (drain.step == DrainStep::kAllPeersLeaving) {
            // NEITHER DRAINED NOR UNFINISHED: no exchange was left to happen.
            // Every peer said it is leaving, so there is nobody to wait for —
            // the partner protocol's "stop when it says homing".
            RCLCPP_INFO(get_logger(),
                "Rendezvous: every peer has said it is leaving (%d of %d "
                "homing or done, none here) -> nothing left to exchange; "
                "leaving after %.1fs.",
                drain.leaving, drain.others, settled_sec);
          } else {
            RCLCPP_INFO(get_logger(),
                "Rendezvous: exchange drained after %.1fs (every believed "
                "peer spoke, then fell below %.3f vox/s over %.0fs).",
                settled_sec, rendezvous_drain_rate_vox_sec_,
                rendezvous_drain_window_sec_);
          }
        }
        rendezvous_drain_released_ = true;
      }
      // After the maps merge, request the next place and time. Requested here,
      // not at the release, so the derive and every peer's echo happen while
      // the fleet is still on the cell. (notes: return-sync-reagree-on-cell)
      if (!rendezvous_reagree_waiting_) {
        rendezvous_reagree_waiting_ = true;
        rendezvous_reagree_from_    = rendezvous_agreed_;
        rendezvous_reagree_due_     = true;
        RCLCPP_INFO(get_logger(),
            "Rendezvous: maps merged after %.0fs -> holding for the next place "
            "and time (up to %.0fs).",
            settled_sec, kRendezvousReagreeWaitSec);
      }
      if (rendezvous_agreed_ == rendezvous_reagree_from_) {
        // Release on rendezvous_agreed_ moving (the commit rule's output, same
        // on proposer and follower), not on the request flag. Bounded by
        // kRendezvousReagreeWaitSec; on expiry leave on the pair already held.
        // (notes: return-sync-reagree-commit-test)
        if (settled_sec - rendezvous_settle_sec_ < kRendezvousReagreeWaitSec) {
          return;   // keep standing on the cell; keep heartbeating.
        }
        RCLCPP_WARN(get_logger(),
            "Rendezvous: no new pair committed in %.0fs -> leaving the meeting "
            "on the pair the team already holds (cell %d, t+%.0fs).",
            kRendezvousReagreeWaitSec, rendezvous_agreed_.cell,
            rendezvous_agreed_.t_meet_ms / 1000.0);
      }
    }
    // Report what the exchange moved: peer cells applied, whether the shared
    // census hash moved, dense voxel gain (loosest: includes own sensing).
    // Needs a baseline; settled_sec is the total time on the cell.
    // (notes: return-sync-exchange-report)
    if (rendezvous_exchange_.taken) {
      const long long merged_cells =
          team_merge_applied_total_ - rendezvous_exchange_.merged;
      const double gained_voxels =
          latest_map_voxels_ - rendezvous_exchange_.voxels;
      const bool census_moved =
          cell_world_.sharedHash() != rendezvous_exchange_.hash;
      RCLCPP_INFO(get_logger(),
          "Rendezvous: team reachable (%d/%d) -> re-planning against merged "
          "map (held %.1fs; the exchange applied %lld peer cell(s), the "
          "shared census %s, the dense map gained %.0f voxel(s)).",
          present, rendezvous_expected_peers_, settled_sec, merged_cells,
          census_moved ? "moved" : "did not move", gained_voxels);
      // Per-peer voxel deltas, since team-wide totals hide which peer
      // delivered. No counters is logged as unmeasured; a zero delta is a
      // silent peer, a finding. (notes: return-sync-per-peer-deltas)
      if (rendezvous_exchange_.peer_deltas.size() == peer_fusion_deltas_.size()
          && !peer_fusion_deltas_.empty()) {
        std::ostringstream ss;
        int silent = 0;
        bool first = true;
        for (int id = 0; id < fleet_.size(); ++id) {
          if (id == fleet_.self_id) continue;
          const size_t k = static_cast<size_t>(id);
          // Unsigned, and re-baselined on a dscovox restart, so the baseline
          // can legitimately exceed the current total. Clamp rather than wrap.
          const uint64_t base = rendezvous_exchange_.peer_deltas[k];
          const uint64_t nowv = peer_fusion_deltas_[k];
          const uint64_t d = (nowv >= base) ? (nowv - base) : 0;
          if (d == 0) ++silent;
          ss << (first ? "" : ", ") << fleet_.nameOf(id) << "=+"
             << static_cast<unsigned long long>(d);
          first = false;
        }
        RCLCPP_INFO(get_logger(),
            "Rendezvous: voxel deltas ingested per peer over the %.1fs "
            "meeting: %s (%d peer(s) sent nothing at all).",
            settled_sec, ss.str().c_str(), silent);
      } else {
        RCLCPP_INFO(get_logger(),
            "Rendezvous: per-peer voxel arrival is unmeasured for this "
            "meeting — dscovox published no fusion counters to baseline "
            "against.");
      }
      // Not a gate: a meeting that moved nothing still earns its new place and
      // time. The warning is for reading only.
      // (notes: return-sync-empty-exchange-not-gate)
      if (merged_cells == 0 && !census_moved && gained_voxels <= 0.0) {
        RCLCPP_WARN(get_logger(),
            "Rendezvous: the %.1fs meeting hold moved nothing at all — "
            "the manoeuvre was paid for and returned no map.", settled_sec);
      }
    } else {
      RCLCPP_INFO(get_logger(),
          "Rendezvous: team reachable (%d/%d) -> re-planning against merged "
          "map (no settle configured, so the exchange is unmeasured).",
          present, rendezvous_expected_peers_);
    }
    rendezvous_exchange_ = MapExchangeBaseline{};
    // Raise rendezvous_reagree_due_ only if the settle stage did not already
    // ask (rendezvous_reagree_waiting_): a second raise authorises a second
    // derive that slides t_meet. Settle-less barriers re-agree while driving.
    // (notes: return-sync-reagree-single-raise)
    if (!rendezvous_reagree_waiting_) {
      rendezvous_reagree_due_ = true;
    }
    rendezvous_settling_ = false;
    // The drain state is the settle's, so it dies with it — including the
    // release latch, which is what makes it per-visit rather than per-run.
    rendezvous_drain_released_ = false;
    rendezvous_drain_window_   = DrainWindow{};
    rendezvous_reagree_waiting_ = false;
    // A coverage_latched_ robot must not return to PLAN: it would record a
    // second exploration_complete at the step budget. The team met and the
    // settle ran, so end the run here.
    // (notes: return-sync-latched-release-ends)
    if (coverage_latched_) {
      // Read the elapsed hold before the start stamp is cleared on the next
      // line. -1 when there is no start stamp.
      // (notes: return-sync-latched-elapsed-hold)
      const double t_release_now = missionElapsed();
      const double held_to_release =
          (coverage_latch_hold_start_sec_ >= 0.0 && t_release_now >= 0.0)
              ? std::max(0.0, t_release_now - coverage_latch_hold_start_sec_)
              : -1.0;
      coverage_latch_hold_start_sec_ = -1.0;
      coverage_latch_teardown_ = true;
      abandonNavGoal("coverage-latched-released");
      RCLCPP_INFO(get_logger(),
          "Rendezvous: team met and the map settled while this robot was "
          "already finished -> ending the run now (held %.0fs past the latch, "
          "cap %.0fs).",
          held_to_release, rendezvous_latched_hold_sec_);
      if (mission_return_enabled_ && have_home_) {
        (void)startReturnHome("coverage-latched-released");
        return;
      }
      (void)finishNow("coverage-latched-released");
      return;
    }
    // Re-confirm saturation against the post-merge map instead of finishing
    // off the pre-barrier streak on the first PLAN tick.
    coverage_done_streak_ = 0;
    have_active_intent_ = false;
    transitionTo(State::PLAN, "barrier-released");
    return;
  }
  // Team not complete: void any partial settle and the re-agreement wait (its
  // second stage), so a peer dropping mid-settle restarts the hold rather than
  // shortening it. (notes: return-sync-incomplete-voids-settle)
  rendezvous_settling_ = false;
  rendezvous_reagree_waiting_ = false;
  // Void with it, and the LATCH most of all: a team that came apart and
  // re-gathered is a new visit, and a drain that fired for the old one must
  // not release the new one on the first tick.
  rendezvous_drain_released_ = false;
  rendezvous_drain_window_   = DrainWindow{};

  // A settle-converted walker resumes its drive once the team is no longer
  // together. The terms negate manoeuvreReleaseEligible's first half (change
  // both together); copy current_goal_ since startReturnTo overwrites it.
  // (notes: return-sync-conversion-reversible)
  // A finished peer the veto holds for is not together, although
  // reachablePeerCount counts it: without that term two finished walkers
  // stopped short both hold and neither drives on (DESIGN_gen33 §10, known
  // issue 2).
  if (appointment_manoeuvre_ && appointment_settle_converted_ &&
      !appointment_arrived_ && !teamSettled(active) &&
      (!teamComplete(reachablePeerCount(), rendezvous_expected_peers_) ||
       holdingForFinishedPeer())) {
    const Eigen::Vector3f resume_target = current_goal_.position;
    const float rdx = resume_target.x() - latest_pos_.x();
    const float rdy = resume_target.y() - latest_pos_.y();
    if (std::sqrt(rdx * rdx + rdy * rdy) >
        static_cast<float>(reconnect_arrive_tol_m_)) {
      // Name the peer when the veto is why: the count includes it.
      const int coming = holdingForFinishedPeer()
          ? finishedPeerStillComing(team_model_, fleet_.self_id) : -1;
      const std::string held_for =
          coming >= 0 ? "; " + fleet_.nameOf(coming) + " finished, unheard"
                      : "";
      RCLCPP_WARN(get_logger(),
          "Rendezvous: the settle that stopped me here has lapsed (%d/%d "
          "present%s) -> resuming the drive to the agreed cell (%.2f, %.2f).",
          active, rendezvous_expected_peers_, held_for.c_str(),
          resume_target.x(), resume_target.y());
      startReturnTo(resume_target, "appointment", "return-settle-lapsed");
      return;
    }
  }

  // Patience: terminal takes the field cap (shorter once escalated); otherwise
  // an appointment waits rendezvous_appointment_wait_sec (<= 0 unbounded), else
  // the mid-run cap. Keyed on appointment_manoeuvre_.
  // (notes: return-sync-wait-cap-by-manoeuvre)
  const double wait_cap =
      reconnect_terminal_
          ? (hold_escalated_ ? hold_escalate_wait_sec_
                             : rendezvous_max_wait_sec_)
          : (appointment_manoeuvre_ ? rendezvous_appointment_wait_sec_
                                    : reconnect_midrun_max_wait_sec_);
  double waited = (this->now() - state_enter_time_).seconds();

  // Log the contagion hold: the team is complete here but a peer still reports
  // its view broken. Throttled to 30 s. (notes: return-sync-contagion-hold-log)
  if (appointment_manoeuvre_ &&
      teamComplete(active, rendezvous_expected_peers_) &&
      peerReportsTeamBreak()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
        "Rendezvous: %d/%d present here but a peer still reports the team "
        "incomplete -> holding the appointment (%.0fs).",
        active, rendezvous_expected_peers_, waited);
  }

  // Stamp the latched-hold start at max(stop tick, t_meet) for a robot that
  // finished on the road. Travel is outside the cap. coverage_latch_teardown_
  // (sticky) stops an ended hold being re-armed.
  // (notes: return-sync-latched-hold-stamp)
  if (coverage_latched_ && appointment_manoeuvre_ &&
      !coverage_latch_teardown_ && coverage_latch_hold_start_sec_ < 0.0 &&
      rendezvous_latched_hold_sec_ > 0.0) {
    const double t_stop = missionElapsed();
    double t_hold = (t_stop >= 0.0) ? t_stop : 0.0;
    if (appointment_.valid())
      t_hold = std::max(t_hold, appointment_.t_meet_ms / 1000.0);
    coverage_latch_hold_start_sec_ = t_hold;
  }

  // The latched hold's own cap, checked before and independently of
  // rendezvousWaitExpired (an appointment wait_cap is 0 = unbounded by
  // default). Measured from the latch or stop stamp, not from state entry.
  // (notes: return-sync-latched-hold-cap)
  if (coverage_latched_ && coverage_latch_hold_start_sec_ >= 0.0 &&
      rendezvous_latched_hold_sec_ > 0.0) {
    const double t_hold_now = missionElapsed();
    const double held = (t_hold_now >= 0.0)
                            ? t_hold_now - coverage_latch_hold_start_sec_
                            : 0.0;
    if (held >= rendezvous_latched_hold_sec_) {
      RCLCPP_WARN(get_logger(),
          "Rendezvous: finished robot held the agreed cell %.0fs (cap %.0fs) "
          "with %d/%d present and the team never completed -> ending the run.",
          held, rendezvous_latched_hold_sec_, active,
          rendezvous_expected_peers_);
      coverage_latch_hold_start_sec_ = -1.0;
      coverage_latch_teardown_ = true;
      abandonNavGoal("latched-hold-expired");
      if (mission_return_enabled_ && have_home_) {
        (void)startReturnHome("latched-hold-expired");
        return;
      }
      // Same presence rule the give-up path uses below: a finished idle robot
      // is still parked and countable, so a peer that arrives late can still
      // release its own barrier on contact.
      if (done_action_ == "idle") {
        publishPresenceIntent();
      } else {
        have_active_intent_ = false;
      }
      transitionTo(State::DONE, "latched-hold-expired");
      return;
    }
  }

  // `waited` is deliberately not rewound to max(arrival, t_meet): an
  // appointment wait_cap is 0 = unbounded by default, and the bounded latched
  // hold is floored at t_meet where it is stamped.
  // (notes: return-sync-no-waited-rewind)
  if (rendezvousWaitExpired(waited, wait_cap)) {
    if (!reconnect_terminal_ && !coverage_latched_) {
      // A mid-run attempt never ends the run: give up and explore; the cooldown
      // stamps in transitionTo when reconnect_active_ falls. A
      // coverage_latched_ robot falls through to the escalate/give-up ladder
      // instead. (notes: return-sync-midrun-give-up)
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
      // Escalate to the point the dispatch picked (the appointment cell), never
      // a freshly derived one; otherwise to this robot's own anchor. All arms
      // escalate identically; there is no midpoint escalation.
      // (notes: hold-escalate-same-target)
      Eigen::Vector3f esc_target = Eigen::Vector3f::Zero();
      const char* esc_what = nullptr;
      // An appointment manoeuvre escalates to its agreed cell: a robot already
      // within tolerance gives up on the spot; one that never reached it
      // re-attempts the same drive. (notes: hold-escalate-appointment-cell)
      if (appointment_manoeuvre_) {
        // Use the leg's own destination, not appointmentPoint():
        // closeAppointment() blanks appointment_, often on the departure tick.
        // RETURN_SYNC does not overwrite current_goal_.
        // (notes: hold-escalate-leg-destination)
        esc_target = current_goal_.position;
        esc_what   = "appointment";
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
    // Should be unreachable under mission return (finishOrRendezvous routes
    // home instead); defensively reroute home rather than strand the robot at a
    // dead barrier. (notes: barrier-give-up-reroute-home)
    if (mission_return_enabled_ && have_home_) {
      startReturnHome("barrier-gave-up");
      return;
    }
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
// Mission return (RETURN_HOME)
// ==================================================================

bool ExploPlannerNode::startReturnHome(const char* reason) {
  // Every assignment below is a reset, so re-entry is refused. Guard 1
  // (run-scoped): the return already resolved; return true, as the robot went
  // home. The caller's recordExplorationComplete is unaffected.
  // (notes: return-home-reentry-guards)
  if (mission_return_done_) {
    ++mission_return_reentries_;
    if (mission_return_reentries_ == 1) {
      RCLCPP_WARN(get_logger(),
          "MISSION-RETURN: the return already resolved [%s -> %s]; this later "
          "request [%s] is REFUSED so the run keeps ONE homing leg and ONE "
          "mission_complete. Counted into run_end.mission_return_reentries.",
          return_home_reason_.c_str(),
          mission_home_result_.empty() ? "?" : mission_home_result_.c_str(),
          reason);
    }
    return true;
  }
  // Guard 2 (leg-scoped): a homing leg is already in flight and further along
  // than a new one. DEBUG, not WARN: this fires on ordinary tick-loop re-entry.
  // (notes: return-home-inflight-guard)
  if (state_ == State::RETURN_HOME) {
    RCLCPP_DEBUG(get_logger(),
        "MISSION-RETURN: already homing [%s]; ignoring re-entry [%s].",
        return_home_reason_.c_str(), reason);
    return true;
  }
  standDownExploitation();
  return_home_reason_        = reason;
  return_home_goal_sent_     = false;
  return_home_retries_       = 0;
  return_home_dist_at_start_ = cumulative_distance_;
  // Same instant, same leg: see the declarations. Stamped here rather than
  // read from state_enter_time_ at the end because this is the only point that
  // is guaranteed to be the leg's start — transitionTo re-stamps
  // state_enter_time_ on every proximity hold and release.
  return_home_start_time_    = this->now();
  return_home_hold_at_start_ = prox_hold_total_sec_;
  home_mode_                 = HomeMode::DIRECT;
  home_trail_idx_            = -1;
  home_escapes_used_         = 0;
  escape_last_crumb_         = -1;
  escape_fallback_n_         = 0;
  pose_jump_seen_            = false;
  approach_check_time_       = this->now();
  approach_check_metric_     = (latest_pos_ - home_pos_).head<2>().norm();
  near_home_armed_           = false;
  // 0.3 s (3 ticks) covers the cancel-all round trip from this call site AND
  // from transitionTo below, whichever fires it.
  home_pub_not_before_ = this->now() + rclcpp::Duration::from_seconds(0.3);
  RCLCPP_INFO(get_logger(),
      "MISSION-RETURN: heading home to (%.2f, %.2f) [%s] — tol %.1f m, "
      "cap %.0f s.",
      home_pos_.x(), home_pos_.y(), reason,
      mission_home_tol_m_, mission_return_max_sec_);
  // Same-binary control for an absence claim: "zero watchdog fires" only means
  // "correctly did not fire" if the watchdog is provably armed in that run.
  // Emitted for EVERY homing, so a log with a homing and no ARMED line is a
  // detectable defect rather than a silent one.
  RCLCPP_INFO(get_logger(),
      "MISSION-RETURN WATCHDOG ARMED: approach %.1f m / %.0f s (suppressed "
      "inside %.1f m, where the stall cap is %.0f s), frozen %.2f m / %.0f s, "
      "escapes 0/%d.",
      return_approach_min_m_, return_approach_window_sec_,
      return_approach_suppress_m_, return_near_home_stall_sec_,
      progress_min_distance_m_, progress_window_sec_,
      return_escape_max_attempts_);
  // No goal publish here: the caller's abandonNavGoal cancel-all is still in
  // flight; the first doReturnHome tick publishes. transitionTo closes any live
  // reconnect manoeuvre, emitting reconnect_end.
  // (notes: return-home-no-publish-at-start)
  transitionTo(State::RETURN_HOME, reason);
  return true;
}

void ExploPlannerNode::doReturnHome() {
  if (!return_home_goal_sent_) {
    // Deferred (re-)publish, wall-clock gated: waiting "one tick" is not a
    // gap when the executor is backlogged (mr0pilot logs: 0.2-0.4 ms), so
    // the gate holds this branch until the cancel-all is safely behind us.
    // Reached at entry and again after each no-progress retry.
    if (this->now() < home_pub_not_before_) return;
    current_goal_ = CandidateViewpoint{};
    // Three publishable targets, one per mode. RETRACE with the index at or
    // below 0 means the trail is spent, which is the home goal again.
    const bool retracing = home_mode_ == HomeMode::RETRACE &&
                           home_trail_idx_ > 0 &&
                           home_trail_idx_ < static_cast<int>(home_trail_.size());
    const char* what = "home goal";
    if (home_mode_ == HomeMode::ESCAPE) {
      current_goal_.position = escape_target_;
      what = "escape goal";
    } else if (retracing) {
      current_goal_.position = home_trail_[home_trail_idx_];
      what = "retrace waypoint";
    } else {
      current_goal_.position = home_pos_;
    }
    if (!retracing && home_mode_ != HomeMode::ESCAPE) {
      // Target IS home: keep the recorded home yaw, as the final approach has
      // always done.
      current_goal_.yaw = home_yaw_;
    } else {
      current_goal_.yaw =
          std::atan2(current_goal_.position.y() - latest_pos_.y(),
                     current_goal_.position.x() - latest_pos_.x());
    }
    publishGoal(current_goal_);
    if (intent_pub_ && coord_) {
      // 0.5 m claim, the same figure the coast used: home is a fixed point,
      // not a contested frontier, and 0.0 reads as "unset" to receivers.
      current_intent_msg_ = coord_->buildIntent(
          current_goal_, latest_pos_, this->now(),
          static_cast<float>(coord_claim_ttl_sec_),
          0.5f, /*planner_type_id (eig)=*/0u, map_frame_);
      publishIntent();
      have_active_intent_ = true;
    }
    const float dx = current_goal_.position.x() - latest_pos_.x();
    const float dy = current_goal_.position.y() - latest_pos_.y();
    const float dist = std::sqrt(dx * dx + dy * dy);
    // Budget distance is the straight line, except in retrace, where it covers
    // the remaining trail to home. Keep it the same function the approach
    // watchdog scores with. (notes: return-home-budget-distance)
    const double budget_dist =
        retracing ? home_trail::remainingTrailDistance(home_trail_,
                                                       home_trail_idx_,
                                                       latest_pos_)
                  : dist;
    const double elapsed = (this->now() - state_enter_time_).seconds();
    // Distance-true budget, as for manoeuvre legs, measured from now (elapsed
    // added) since a retry re-arms mid-leg. No min() against the mission cap:
    // the cap check runs first every tick. (notes: return-home-budget-from-now)
    double leg_budget = std::max(
        nav_min_timeout_sec_,
        budget_dist * nav_safety_factor_ /
            std::max(nav_speed_est_mps_, 1e-3));
    if (home_mode_ == HomeMode::ESCAPE) {
      // An escape leg must expire as an escape: return_escape_leg_sec_ is the
      // authority, and the nav budget sits kEscapeBudgetMarginSec behind it
      // only to catch a leg whose termination never runs.
      // (notes: return-home-escape-budget-margin)
      constexpr double kEscapeBudgetMarginSec = 10.0;
      leg_budget = return_escape_leg_sec_ + kEscapeBudgetMarginSec;
    }
    nav_budget_sec_ = elapsed + leg_budget;
    // BOTH watchdog windows restart at every publish. The approach baseline in
    // particular must be re-taken here: a fire brakes the robot, and measuring
    // the next window from a metric sampled before the brake would charge the
    // new leg for the old leg's stall.
    progress_check_time_ = this->now();
    progress_check_dist_ = cumulative_distance_;
    approach_check_time_ = this->now();
    approach_check_metric_ = homeApproachMetric();
    pose_jump_seen_ = false;
    return_home_goal_sent_ = true;
    RCLCPP_INFO(get_logger(),
        "MISSION-RETURN: %s published (dist=%.2f m, budget %.0f s%s).",
        what, dist, nav_budget_sec_,
        return_home_retries_ > 0 || home_escapes_used_ > 0 ? ", retry" : "");
    return;
  }

  const float dx = latest_pos_.x() - home_pos_.x();
  const float dy = latest_pos_.y() - home_pos_.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  // mission_home_tol_m, NOT reconnect_arrive_tol_m: the two homes are only
  // 3 m apart, so the 4 m manoeuvre tolerance would accept the partner's.
  if (dist < static_cast<float>(mission_home_tol_m_)) {
    finishMissionReturn("arrived", "mission-home");
    return;
  }

  const auto now = this->now();
  const double elapsed = (now - state_enter_time_).seconds();
  // Overall cap first: it is the field guarantee that a mission-return run
  // still ends. The robot parks where it is; the analysis reads the result
  // from mission_complete, not from where the robot stopped.
  if (mission_return_max_sec_ > 0.0 && elapsed >= mission_return_max_sec_) {
    RCLCPP_WARN(get_logger(),
        "MISSION-RETURN: %.0f s cap reached %.2f m short of home — parking "
        "here.", mission_return_max_sec_, dist);
    finishMissionReturn("timeout", "home-timeout");
    return;
  }
  if (elapsed > nav_budget_sec_) {
    RCLCPP_WARN(get_logger(),
        "MISSION-RETURN: home unreachable within budget (%.1f s, "
        "dist=%.2f m) — parking here.", elapsed, dist);
    finishMissionReturn("budget", "home-gave-up");
    return;
  }
  // Within 3 m of the current crumb, advance the index toward home past crumbs
  // inside that circle. Plain goal replacement, no cancel; an advance only
  // shrinks the approach metric. (notes: return-home-retrace-advance)
  if (home_mode_ == HomeMode::RETRACE && home_trail_idx_ > 0) {
    if ((latest_pos_ - home_trail_[home_trail_idx_]).head<2>().norm() <
        3.0f) {
      while (home_trail_idx_ > 0 &&
             (latest_pos_ - home_trail_[home_trail_idx_]).head<2>().norm() <
                 3.0f) {
        --home_trail_idx_;
      }
      if (home_trail_idx_ > 0) {
        const Eigen::Vector3f& wp = home_trail_[home_trail_idx_];
        current_goal_.position = wp;
        current_goal_.yaw = std::atan2(wp.y() - latest_pos_.y(),
                                       wp.x() - latest_pos_.x());
      } else {
        // Trail exhausted — final exact approach on home itself.
        current_goal_.position = home_pos_;
        current_goal_.yaw = home_yaw_;
      }
      publishGoal(current_goal_);
    }
  }
  // ---- Three independent watchdogs ----
  // FROZEN (gross metres), APPROACH (remaining distance home) and NEAR-HOME
  // STALL (time inside the muted radius) are evaluated together; both windows
  // advance whichever fires, and `kind` names it. (notes: home-watchdogs-three)
  const float metric = homeApproachMetric();
  bool frozen = false;
  bool no_approach = false;
  float approach_delta = 0.f;
  // Capture the frozen detector's tested delta before progress_check_dist_ is
  // re-baselined, so homeWatchdogFire can log the window movement actually
  // tested. (notes: return-home-frozen-delta-capture)
  float frozen_delta = 0.f;

  const double move_window = (now - progress_check_time_).seconds();
  if (move_window > progress_window_sec_) {
    frozen_delta = cumulative_distance_ - progress_check_dist_;
    frozen = frozen_delta < progress_min_distance_m_;
    progress_check_time_ = now;
    progress_check_dist_ = cumulative_distance_;
  }

  const double appr_window = (now - approach_check_time_).seconds();
  // Suppressed in ESCAPE (the leg moves away from home on purpose) and inside
  // return_approach_suppress_m_, where the near-home stall detector takes over.
  // (notes: return-home-approach-suppression)
  if (home_mode_ != HomeMode::ESCAPE &&
      metric > static_cast<float>(return_approach_suppress_m_) &&
      appr_window > return_approach_window_sec_) {
    approach_delta = approach_check_metric_ - metric;
    no_approach = approach_delta < static_cast<float>(return_approach_min_m_);
    approach_check_time_ = now;
    approach_check_metric_ = metric;
  }
  // A relocalization jump makes the approach delta a measurement artefact, not
  // a stall. Re-baseline and forfeit this window rather than fire on it. The
  // frozen detector is already jump-proof: trackDistance refuses to add the
  // jump to cumulative_distance_.
  if (pose_jump_seen_) {
    pose_jump_seen_ = false;
    approach_check_time_ = now;
    approach_check_metric_ = metric;
    no_approach = false;
    // Likewise disarm the near-home band clock: a jump can place the robot
    // inside the band without driving there. Forfeit the window rather than
    // fire. (notes: return-home-jump-disarms-band)
    near_home_armed_ = false;
  }

  // Updated on EVERY tick, including ticks where another detector fires below;
  // skipping it would restart the clock each time the ladder ran. Near home the
  // test is time in band, not approach rate.
  // (notes: home-near-stall-band-clock)
  bool near_home_stalled = false;
  if (home_mode_ != HomeMode::ESCAPE &&
      metric <= static_cast<float>(return_approach_suppress_m_)) {
    if (!near_home_armed_) {
      near_home_armed_ = true;
      near_home_since_ = now;
    } else if (return_near_home_stall_sec_ > 0.0 &&
               (now - near_home_since_).seconds() >
                   return_near_home_stall_sec_) {
      near_home_stalled = true;
    }
  } else {
    near_home_armed_ = false;
  }

  if (frozen || no_approach) {
    // Frozen wins the label when both fire. The logged delta is selected by the
    // same condition as the label, so it always belongs to the detector named
    // in kind. (notes: home-watchdog-frozen-wins-label)
    homeWatchdogFire(frozen ? "frozen" : "approach", metric, dist,
                     frozen ? frozen_delta : approach_delta);
    return;
  }

  // Checked last: a tick where frozen or approach also fires belongs to them.
  // Disarm before firing, not after: the ladder can leave the robot in the
  // band, and re-arming stops one stall firing every tick.
  // (notes: home-near-stall-fires-last)
  if (near_home_stalled) {
    near_home_armed_ = false;
    homeWatchdogFire("near-home-stall", metric, dist, metric);
    return;
  }

  // Escape-leg termination. Arrival at the escape target, or the leg cap.
  // Either way the answer is the same: go back to retracing — see resumeRetrace
  // for why an escape never resumes DIRECT.
  if (home_mode_ == HomeMode::ESCAPE) {
    const float d = (latest_pos_ - escape_target_).head<2>().norm();
    const double leg = (now - escape_leg_start_).seconds();
    if (d < kEscapeArriveM || leg > return_escape_leg_sec_) {
      resumeRetrace(d < kEscapeArriveM ? "escape-arrived" : "escape-leg-cap");
      return;
    }
  }
  republishGoal(current_goal_);
}

const char* ExploPlannerNode::homeModeName(HomeMode m) {
  switch (m) {
    case HomeMode::DIRECT:  return "direct";
    case HomeMode::RETRACE: return "retrace";
    case HomeMode::ESCAPE:  return "escape";
  }
  return "?";
}

/// Distance the approach watchdog holds the robot accountable for.
/// DIRECT: straight line to home. RETRACE: remaining trail length — the same
/// model the nav budget uses — so that following a curved trail away from the
/// straight line is not scored as failing to approach.
float ExploPlannerNode::homeApproachMetric() const {
  // The index check stays here rather than leaning on remainingTrailDistance's
  // 0.0 return: a spent trail means "the goal is home again", which is a
  // straight line, not a zero distance.
  if (home_mode_ != HomeMode::RETRACE || home_trail_idx_ <= 0 ||
      home_trail_idx_ >= static_cast<int>(home_trail_.size())) {
    return (latest_pos_ - home_pos_).head<2>().norm();
  }
  return home_trail::remainingTrailDistance(home_trail_, home_trail_idx_,
                                            latest_pos_);
}

/// Switch to retracing the outbound trail from the crumb nearest the robot.
/// Every metre of that trail was driven once already this run, so a local
/// minimum on unexplored geometry cannot block it. False if there is no usable
/// trail (a homing that began within 2 m of home).
bool ExploPlannerNode::engageRetrace() {
  // size() <= 1 is "no trail at all", which is a genuine false: there is
  // nothing to retrace. A single usable crumb is not — that is a retrace of one
  // segment. nearestCrumbFromOne never returns 0 (crumb 0 IS home; see its
  // comment for what selecting it does to the ladder).
  if (home_trail_.size() <= 1) return false;
  home_trail_idx_ = home_trail::nearestCrumbFromOne(home_trail_, latest_pos_);
  home_mode_ = HomeMode::RETRACE;
  return true;
}

/// Choose the escape target: the nearest breadcrumb 1.5-6.0 m away (ties toward
/// the higher index), else a pose 2.5 m behind the robot rotated 0/+60/-60 deg
/// per attempt. True only if a crumb was used.
/// (notes: home-escape-target-choice)
bool ExploPlannerNode::pickEscapeTarget() {
  // escape_last_crumb_ is the anti-repeat: never escape twice onto the same
  // crumb. Search starts at 1 because crumb 0 is home — see pickBandCrumb for
  // why aiming an escape there is worse than not escaping at all.
  const int best_i = home_trail::pickBandCrumb(home_trail_, latest_pos_,
                                               kEscapeBandMinM, kEscapeBandMaxM,
                                               escape_last_crumb_);
  if (best_i >= 0) {
    escape_last_crumb_ = best_i;
    escape_target_ = home_trail_[best_i];
    return true;
  }
  static constexpr float kFallbackOffsetRad[3] = {0.f, 1.047f, -1.047f};
  const float off = kFallbackOffsetRad[escape_fallback_n_ % 3];
  ++escape_fallback_n_;
  const float th = latest_yaw_ + static_cast<float>(M_PI) + off;
  escape_target_ = latest_pos_ + Eigen::Vector3f(kEscapeFallbackM * std::cos(th),
                                                 kEscapeFallbackM * std::sin(th),
                                                 0.f);
  escape_last_crumb_ = -1;
  return false;  // used the fallback, not a crumb
}

void ExploPlannerNode::startEscapeLeg() {
  const bool on_crumb = pickEscapeTarget();
  ++home_escapes_used_;
  home_mode_ = HomeMode::ESCAPE;
  escape_leg_start_ = this->now();
  // No frozen-window re-baseline here on purpose: republishHomeGoal clears
  // return_home_goal_sent_, and the publish block that follows resets
  // progress_check_time_/_dist_ itself. (notes: home-escape-no-rebaseline)
  RCLCPP_WARN(get_logger(),
      "MISSION-RETURN: escape %d/%d — driving %.2f m to %s (%.2f, %.2f) to "
      "break the stall, then resuming the retrace.",
      home_escapes_used_, return_escape_max_attempts_,
      (escape_target_ - latest_pos_).head<2>().norm(),
      on_crumb ? "breadcrumb" : "a pose behind the robot",
      escape_target_.x(), escape_target_.y());
}

/// End an escape leg and go back to retracing. NEVER back to DIRECT: needing an
/// escape at all is evidence the direct goal is the thing that trapped the
/// robot. Falls back to the direct goal only when there is no usable trail.
void ExploPlannerNode::resumeRetrace(const char* why) {
  // Snapshot the approach metric BEFORE engageRetrace() flips home_mode_: this
  // event is about the escape leg that just ended, whose metric was the
  // straight line. (notes: home-escape-end-metric-snapshot)
  const float escape_metric = homeApproachMetric();
  const bool ok = engageRetrace();
  if (!ok) home_mode_ = HomeMode::DIRECT;
  RCLCPP_WARN(get_logger(),
      "MISSION-RETURN: escape leg ended [%s] after %.1f s — %s.", why,
      (this->now() - escape_leg_start_).seconds(),
      ok ? "resuming the retrace from the nearest crumb"
         : "no usable trail, resuming the direct home goal");
  if (exp_log_) {
    // mode is the mode this event is about, always escape; the resumed mode
    // goes in next_mode. The kNoTestDelta arguments are inert placeholders, not
    // sentinels: logHomeWatchdog omits the tested pair on escape-end rows.
    // (notes: home-escape-end-log-convention)
    exp_log_->logHomeWatchdog(expCtx(), "escape-end", "escape",
                              why, (latest_pos_ - home_pos_).head<2>().norm(),
                              escape_metric,
                              (this->now() - escape_leg_start_).seconds(),
                              home_escapes_used_, kNoTestDelta, kNoTestDelta,
                              homeModeName(home_mode_));
  }
  republishHomeGoal("home-escape-end");
}

/// Brake at the current pose, then let the next tick publish the mode's goal.
/// The brake goal, not the no-op nav cancel, stops the platform and clears the
/// navigator's active goal so a positional repeat is accepted.
/// (notes: home-republish-brake-goal)
void ExploPlannerNode::republishHomeGoal(const char* why) {
  abandonNavGoal(why);
  return_home_goal_sent_ = false;  // wall-clock-gated re-publish
  home_pub_not_before_ = this->now() + rclcpp::Duration::from_seconds(0.3);
}

/// Single decision point for a homing watchdog fire. Graduated, never fatal on
/// a first fire: resend, then retrace, then escape legs; frozen escapes at
/// once. Only an exhausted escape budget parks.
/// (notes: home-watchdog-fire-ladder)
void ExploPlannerNode::homeWatchdogFire(const char* kind, float metric,
                                        float dist_home, float test_delta) {
  const char* mode = homeModeName(home_mode_);
  const char* response = "resend";
  const bool is_frozen = std::strcmp(kind, "frozen") == 0;
  const bool is_stall  = std::strcmp(kind, "near-home-stall") == 0;
  // kind is a three-valued enum spelled as a string (frozen, approach,
  // near-home-stall); the selections below are exhaustive over it and an
  // unknown kind is logged as an error. (notes: home-watchdog-kind-exhaustive)
  if (!is_frozen && !is_stall && std::strcmp(kind, "approach") != 0) {
    RCLCPP_ERROR(get_logger(),
        "home watchdog fired with unknown kind '%s' — logging it against the "
        "approach window/threshold, which is almost certainly wrong. The "
        "test_threshold_m on this row must not be trusted.", kind);
  }
  const double window = is_frozen ? progress_window_sec_
                      : is_stall  ? return_near_home_stall_sec_
                                  : return_approach_window_sec_;
  // Selected by kind, like window, so the fired inequality (test_delta_m <
  // test_threshold_m) is re-derivable from the row alone. For the stall
  // detector it is band membership against return_approach_suppress_m_.
  // (notes: home-watchdog-test-threshold)
  const double test_threshold = is_frozen ? progress_min_distance_m_
                              : is_stall  ? return_approach_suppress_m_
                                          : return_approach_min_m_;

  if (home_mode_ == HomeMode::ESCAPE) {
    // Only `frozen` can reach here — approach is suppressed during an escape.
    RCLCPP_WARN(get_logger(),
        "MISSION-RETURN: %s during the escape leg (%.2f m from home) — "
        "aborting the leg.", kind, dist_home);
    // Log the fire before aborting the leg, as kind frozen-in-escape (not
    // frozen, escape-end or escape-frozen, which collide) so the tested pair is
    // recorded. window is the detector's window, not the leg's duration.
    // (notes: home-frozen-in-escape-row)
    if (exp_log_) {
      exp_log_->logHomeWatchdog(expCtx(), "frozen-in-escape", mode, "abort-leg",
                                dist_home, metric, window, home_escapes_used_,
                                test_delta, test_threshold);
    }
    resumeRetrace("escape-frozen");
    return;  // resumeRetrace emits its own event and republishes
  }

  // In DIRECT a stall, like approach, takes the resend ladder first; in RETRACE
  // it escapes at once, since the retrace already had its resend. Frozen always
  // escapes. (notes: home-stall-resend-before-escape)
  const bool want_escape = is_frozen || home_mode_ == HomeMode::RETRACE;

  if (want_escape) {
    if (home_escapes_used_ >= return_escape_max_attempts_) {
      // The tested delta is in this line because this is the fire that ENDS a
      // run: "home-gave-up" is the censoring reason, and classifying a park as
      // a genuine stall rather than a detector artefact needs the number the
      // detector compared, not the distance still to go.
      RCLCPP_WARN(get_logger(),
          "MISSION-RETURN: %s watchdog fired in %s mode with all %d escapes "
          "spent (%.2f m from home, metric %.2f m, moved %.2f m of the %.2f m "
          "needed in %.0f s) — parking here.",
          kind, mode, return_escape_max_attempts_, dist_home, metric,
          test_delta, test_threshold, window);
      if (exp_log_) {
        exp_log_->logHomeWatchdog(expCtx(), kind, mode, "park", dist_home,
                                  metric, window, home_escapes_used_,
                                  test_delta, test_threshold);
      }
      finishMissionReturn("no-progress", "home-gave-up");
      return;
    }
    RCLCPP_WARN(get_logger(),
        "MISSION-RETURN: %s watchdog fired in %s mode — %.2f m from home, "
        "metric %.2f m, moved %.2f m of the %.2f m needed over a %.0f s "
        "window.",
        kind, mode, dist_home, metric, test_delta, test_threshold, window);
    startEscapeLeg();
    response = "escape";
  } else {
    ++return_home_retries_;
    // `kind` and test_threshold, not the words "no approach" and
    // return_approach_min_m_: two detectors reach this ladder now, and a stall
    // fire printed against the approach detector's threshold is a console line
    // that contradicts the jsonl row written four lines below it.
    RCLCPP_WARN(get_logger(),
        "MISSION-RETURN: %s watchdog fired %.2f m from home (tested %.2f m "
        "against %.2f m over %.0f s) — retry %d/2: %s.",
        kind, dist_home, test_delta, test_threshold, window,
        return_home_retries_,
        return_home_retries_ >= 2 ? "retracing the outbound trail"
                                  : "braking and re-sending the home goal");
    if (return_home_retries_ >= 2) {
      if (engageRetrace()) {
        response = "retrace";
        RCLCPP_WARN(get_logger(),
            "MISSION-RETURN: retracing from crumb %d/%zu (%.2f m away).",
            home_trail_idx_, home_trail_.size(),
            (home_trail_[home_trail_idx_] - latest_pos_).head<2>().norm());
      } else {
        // No trail to retrace (homing began within one crumb spacing of home).
        // Escape instead of resending a goal that has already failed twice.
        if (home_escapes_used_ >= return_escape_max_attempts_) {
          if (exp_log_) {
            exp_log_->logHomeWatchdog(expCtx(), kind, mode, "park", dist_home,
                                      metric, window, home_escapes_used_,
                                      test_delta, test_threshold);
          }
          RCLCPP_WARN(get_logger(),
              "MISSION-RETURN: no trail to retrace and all %d escapes spent "
              "(moved %.2f m of the %.2f m needed in %.0f s) — parking here.",
              return_escape_max_attempts_, test_delta, test_threshold, window);
          finishMissionReturn("no-progress", "home-gave-up");
          return;
        }
        startEscapeLeg();
        response = "escape";
      }
    }
  }
  if (exp_log_) {
    exp_log_->logHomeWatchdog(expCtx(), kind, mode, response, dist_home, metric,
                              window, home_escapes_used_, test_delta,
                              test_threshold);
  }
  republishHomeGoal("home-watchdog");
}

bool ExploPlannerNode::finishMissionReturn(const char* result,
                                           const char* end_reason) {
  const auto now = this->now();
  const float dx = latest_pos_.x() - home_pos_.x();
  const float dy = latest_pos_.y() - home_pos_.y();
  const float dist = std::sqrt(dx * dx + dy * dy);
  // Both off the leg's own baselines, so the ratio of the two is a speed.
  // NOT state_enter_time_, which a proximity hold backdates to keep the nav
  // budget running across the hold — see return_home_start_time_'s doc.
  const double homing_sec  = (now - return_home_start_time_).seconds();
  const double homing_m    = cumulative_distance_ - return_home_dist_at_start_;
  const double homing_held = prox_hold_total_sec_ - return_home_hold_at_start_;
  // Spend the leg BEFORE anything can re-open it. Set here, not in finishNow
  // (which also ends runs that never homed), and unconditionally ahead of the
  // exp_log_ block: re-entry damage is behavioural.
  // (notes: home-return-done-latch)
  mission_return_done_  = true;
  mission_home_result_  = result;
  mission_home_sim_sec_ = now.seconds();
  if (exp_log_) {
    MissionCompleteEvent e;
    e.result              = result;
    e.reason              = return_home_reason_.c_str();
    e.home_x              = home_pos_.x();
    e.home_y              = home_pos_.y();
    e.final_x             = latest_pos_.x();
    e.final_y             = latest_pos_.y();
    e.dist_to_home_m      = dist;
    e.homing_duration_sec = homing_sec;
    e.homing_distance_m   = homing_m;
    e.homing_held_sec     = homing_held;
    e.latched             = coverage_latched_;
    e.occurrence          = ++mission_complete_count_;
    exp_log_->logMissionComplete(expCtx(), e);
  }
  RCLCPP_INFO(get_logger(),
      "MISSION-RETURN %s [%s]: %.2f m from home after %.1f s / %.2f m of "
      "homing (%.1f s of that held for proximity).",
      result, end_reason, dist, homing_sec, homing_m, homing_held);
  // Stop the platform before parking: on the give-up paths the navigator still
  // holds the unreachable goal, and an arrival-by-tolerance lands before it
  // finishes driving to the exact pose (same rationale as doReturnNav).
  abandonNavGoal(end_reason);
  return finishNow(end_reason);
}

// ==================================================================
// Mesh-reconnection pursuit (robot-carried radios)
// ==================================================================

// The peer the barrier is waiting on: among peers in last_contact_, the
// freshest one not peerAccounted. Liveness is peerAccounted, deliberately not
// the exploit grace window. (notes: pursuit-missing-peer-record)
const ExploPlannerNode::LastContact*
ExploPlannerNode::missingPeerRecord(std::string* peer_id_out) {
  const auto now = this->now();
  const LastContact* best = nullptr;
  for (const auto& [id, rec] : last_contact_) {
    if (peerAccounted(id, now)) continue;
    if (best == nullptr || rec.stamp > best->stamp) {
      best = &rec;
      if (peer_id_out) *peer_id_out = id;
    }
  }
  return best;
}

// Predict the missing peer's intercept. tour_age_sec is set to the age of the
// record the prediction ran on, or left at -1 when there was none.
// (notes: pursuit-predict-intercept-doc)
PursuitTarget ExploPlannerNode::predictIntercept(const std::string& peer_id,
                                                 double* tour_age_sec) const {
  if (tour_age_sec) *tour_age_sec = -1.0;
  PursuitTarget out;
  // The two "the model was never asked" cases, named as distinctly as the
  // model's own refusals are, and for the same reason: a column that reads
  // empty for "off", "no such peer" and "the chain had nothing to say" cannot
  // separate an arm that never predicted from one that predicted badly.
  if (!pursuit_predictor_mdp_) { out.refused = "predictor off"; return out; }
  const int pid = fleet_.idOf(peer_id);
  if (pid < 0) { out.refused = "peer is not in the fleet"; return out; }
  const auto it = peer_tours_.find(pid);
  if (it == peer_tours_.end()) { out.refused = "no tour on record"; return out; }

  // These cell ids name OUR ground, which is drainTeamWorld's grid_hash gate
  // and not an assumption made here: the predictor reads them straight into
  // our own CellWorld and has no way to notice a foreign grid.
  PursuitPredictor::PeerTrack track;
  track.tour = it->second.cells;
  track.x = it->second.pos.x();
  track.y = it->second.pos.y();
  track.have_position = true;
  track.age_sec = (this->now() - it->second.stamp).seconds();
  if (tour_age_sec) *tour_age_sec = track.age_sec;

  // The cell this robot is standing in, live. Not the cell the manoeuvre was
  // armed from: the intercept is scored at MY travel time, and the only pose
  // that makes that a travel time is the one I am about to drive from.
  const int my_cell = cell_world_.grid().idAt(latest_pos_.x(), latest_pos_.y());
  return PursuitPredictor::predict(cell_world_, track, my_cell,
                                   pursuit_mdp_cfg_);
}

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

  // --- the intercept, when the model is on (P6, §3.7) --------------------
  // Computed once per chase, never refreshed. It replaces only the first
  // waypoint and re-sizes budget at the swap, under the same
  // pursuit_budget_max_sec_ cap. have_dispatch_predict_ is armed only at the
  // commit point. (notes: pursuit-intercept-snapshot)
  have_dispatch_predict_ = false;
  const PursuitTarget mdp =
      predictIntercept(peer_id, &dispatch_predict_tour_age_sec_);
  dispatch_predict_ = mdp;

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

  // Path length of a waypoint list from here, and model seconds at the
  // conservative NAV speed (the watchdog's question). The budget rule and the
  // intercept affordability test must both use these.
  // (notes: pursuit-model-sec-nav-speed)
  auto trailMetres = [&](const std::vector<Eigen::Vector3f>& t) {
    double m = 0.0;
    Eigen::Vector3f prev = latest_pos_;
    for (const auto& wp : t) {
      m += (wp - prev).head<2>().norm();
      prev = wp;
    }
    return m;
  };
  auto modelSec = [&](double metres) {
    return metres * nav_safety_factor_ / std::max(nav_speed_est_mps_, 1e-3);
  };
  // travelSec answers whether the chase can be done at all: measured speed, no
  // safety factor. It is not the watchdog's rate.
  // (notes: pursuit-travel-sec-measured)
  auto travelSec = [&](double metres) {
    return metres / std::max(pursuit_speed_measured_mps_, 1e-3);
  };

  double budget = 0.0;
  if (goal_stale) {
    // The contact pose earns the full model time only if the cap covers the
    // whole trail. trail_travel_sec decides whether to chase; trail_model_sec
    // sizes the watchdog. Neither substitutes for the other.
    // (notes: pursuit-stale-trail-budget)
    const double trail_m = trailMetres(trail);
    const double trail_model_sec  = modelSec(trail_m);
    const double trail_travel_sec = travelSec(trail_m);
    if (trail_travel_sec > pursuit_budget_max_sec_) {
      RCLCPP_INFO(get_logger(),
          "Pursuit: record of '%s' is %.0fs old (goal stale) and the contact "
          "pose is %.1f m away (%.0fs of travel at %.2f m/s > cap %.0fs) — "
          "chase would die mid-trail, skipping to the fallback.",
          peer_id.c_str(), staleness, trail_m, trail_travel_sec,
          pursuit_speed_measured_mps_, pursuit_budget_max_sec_);
      reconnect_decline_reason_ = "trail-exceeds-budget-cap";
      return false;
    }
    // The upper clamp is load-bearing: the model rate can price a feasible
    // trail past the cap. Same ceiling as pursuitBudgetSec() in the other
    // branch; lo <= hi by construction.
    // (notes: pursuit-stale-budget-upper-clamp)
    budget = std::clamp(trail_model_sec,
                        std::min(nav_min_timeout_sec_, pursuit_budget_max_sec_),
                        pursuit_budget_max_sec_);
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

  // --- swap the intercept in front, if it fits the budget just granted ------
  // Swap the intercept in front only if it fits. An unaffordable prediction is
  // downgraded to the legacy trail, never declined, so the model cannot
  // suppress a chase the control arm performs.
  // (notes: pursuit-intercept-downgrade-rule)
  if (mdp.valid()) {
    std::vector<Eigen::Vector3f> mdp_trail;
    mdp_trail.push_back(Eigen::Vector3f(mdp.x, mdp.y, latest_pos_.z()));
    // Same 1 m coincidence rule the legacy trail uses: an intercept that lands
    // on the contact pose is one waypoint, not two. The contact pose stays
    // behind it as the sweep leg — the dead goal does not, because it is the
    // hypothesis the model was built to replace.
    if ((rec.peer_pose - mdp_trail.front()).head<2>().norm() > 1.0f)
      mdp_trail.push_back(rec.peer_pose);

    const double mdp_m      = trailMetres(mdp_trail);
    const double mdp_travel = travelSec(mdp_m);
    const double mdp_model  = modelSec(mdp_m);
    // Priced in travel seconds against pursuit_budget_max_sec_, the same
    // yardstick as the trail refusal. Do not test modelSec against budget: that
    // comparison can essentially never pass.
    // (notes: pursuit-intercept-priced-travel)
    if (mdp_travel <= pursuit_budget_max_sec_) {
      trail = std::move(mdp_trail);
      // Resize the watchdog to the route actually driven, at the conservative
      // modelSec rate. The non-stale branch's staleness discount is
      // deliberately not carried over.
      // (notes: pursuit-intercept-resize-watchdog)
      budget = std::clamp(mdp_model,
                          std::min(nav_min_timeout_sec_, pursuit_budget_max_sec_),
                          pursuit_budget_max_sec_);
    } else {
      // Report it as a refusal so the dispatch event says the model was asked,
      // answered, and lost — distinct from the model having nothing to say. The
      // cell/probability fields stay populated on purpose: a downgrade at
      // p = 0.8 and one at p = 0.11 are different findings.
      dispatch_predict_.refused = "intercept exceeds the chase budget";
      RCLCPP_INFO(get_logger(),
          "Pursuit: intercept for '%s' at cell %d (p=%.2f, %.1f m, needs "
          "%.0fs of travel at %.2f m/s > cap %.0fs) — falling back to the "
          "trail.",
          peer_id.c_str(), mdp.cell, mdp.p, mdp_m, mdp_travel,
          pursuit_speed_measured_mps_, pursuit_budget_max_sec_);
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
  // The chase is now real, so the prediction that aimed it is reportable.
  have_dispatch_predict_ = true;

  // The aim is named in the line, because "chasing its trail head" was true of
  // every chase this node had ever dispatched until P6 and is now true of only
  // some of them — a reader grepping these lines to tell the arms apart would
  // otherwise find them identical.
  const bool aimed_by_model = dispatch_predict_.valid();
  RCLCPP_INFO(get_logger(),
      "Pursuit: dispatched [%s], '%s' out of comms (record %.0fs old%s) "
      "-> chasing %s (%.2f, %.2f), budget %.0fs, %zu waypoint(s).",
      reason, peer_id.c_str(), staleness,
      aimed_by_model ? "" :
          (goal_stale ? ", goal stale — contact pose only" : ""),
      aimed_by_model ? "the predicted intercept" : "its trail head",
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
// Re-entry restarts the per-waypoint nav budget and progress window; the
// pursuit budget keeps running from pursue_start_time_.
// (notes: pursuit-arm-waypoint)
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
  // Manoeuvre legs are exempt from nav_max_timeout_sec: distance-true budget,
  // floor kept, capped by reconnect_nav_max_sec as in startReturnTo. The
  // no-progress window stays the stuck-robot watchdog.
  // (notes: pursuit-leg-nav-budget)
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

// Drive the trail. Releases on the team back in comms, the pursuit budget spent
// (to the fallback), or the waypoint reached or unreachable (advance, or fall
// back when exhausted). Nav failures advance, not abort.
// (notes: pursuit-dopursue-release-conditions)
void ExploPlannerNode::doPursue() {
  const auto now = this->now();
  // accountedPeerCount — presence semantics, same note as finishOrRendezvous().
  const int active = accountedPeerCount(now);
  // Release when the whole team or the chased peer alone is back. teamComplete,
  // deliberately not teamSettled: another robot's outage report must not hold
  // this chase open. The trigger site uses the same predicate.
  // (notes: pursuit-release-quarry-or-team)
  const bool quarry_heard = !pursue_peer_id_.empty() &&
                            peerAccounted(pursue_peer_id_, now);
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
    // proximity guard off, and the navigator would keep driving at the missing
    // teammate's own last pose underneath it (see abandonNavGoal).
    abandonNavGoal("pursuit-released");
    // Re-confirm saturation against the post-merge map instead of finishing
    // off the pre-chase streak on the first PLAN tick.
    coverage_done_streak_ = 0;
    have_active_intent_ = false;
    transitionTo(State::PLAN, "pursuit-released");
    return;
  }
  // The departure deadline ends the chase and is tested BEFORE the budget and
  // the trail. appointmentDue() leaves early enough to arrive, so a chase that
  // drives away from the meeting cell breaks off sooner.
  // (notes: pursuit-departure-deadline-first)
  if (appointment_armed_ && !appointment_departed_ && appointmentDue()) {
    RCLCPP_INFO(get_logger(),
        "Pursuit: time to leave for cell %d after %.0fs of chase "
        "(meeting at t+%.0fs, my travel %.0fs) -> breaking off for the "
        "appointment.",
        appointment_.cell, (now - pursue_start_time_).seconds(),
        appointment_.t_meet_ms / 1000.0, appointmentTravelMs() / 1000.0);
    appointment_departed_ = true;
    // PURSUE is a driving state and startReturnTo publishes a new goal on top
    // of the chase waypoint; abandon the old one first, same reason the
    // release path above does.
    abandonNavGoal("appointment-due");
    // Mid-run, for the same reason as doPlan's departure -- see the long note
    // there. A chase broken off for the deadline has not finished exploring.
    reconnect_terminal_ = false;
    startReturnTo(appointmentPoint(), "appointment", "appointment-due");
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

// Route a spent or exhausted chase to its fallback.
// (notes: pursuit-fallback-routing)
void ExploPlannerNode::pursuitFallback(const char* why) {
  // A fresh decision point: the chase ran and ended (`why` says how), so
  // whatever was declined when it was ARMED is no longer the explanation for
  // what happens next.
  reconnect_decline_reason_.clear();
  // Second site of the link veto, with the same freshness bound and debounce as
  // the trigger site: with the radio up or only just down, stand down and
  // resume exploring instead of escalating. (notes: pursuit-fallback-link-veto)
  const rclcpp::Time fb_now = this->now();
  if (linkGateReady(fb_now)) {
    const double link_down_for =
        link_connected_ ? 0.0 : (fb_now - link_up_last_seen_).seconds();
    if (link_connected_ ||
        link_down_for < reconnect_link_down_confirm_sec_) {
      RCLCPP_INFO(get_logger(),
          "Pursuit fallback (%s): standing down — the radio link to '%s' is "
          "%s, so escalating would commit travel toward a peer that is already "
          "reachable. Resuming exploration instead.",
          why, pursue_peer_id_.c_str(),
          link_connected_ ? "UP" : "only just down");
      // resumeExploring, not a bare transition: it stamps the mid-run cooldown
      // and sets reconnect_terminal_; without that every PLAN tick
      // re-dispatches until the attempt budget is burned.
      // (notes: pursuit-fallback-resume-not-transition)
      resumeExploring(why);
      return;
    }
  }
  // With an undeparted appointment standing, a chase that died early resumes
  // exploring rather than parking; doPlan's departure block owns the outage
  // from here. A departed appointment cannot reach here.
  // (notes: pursuit-fallback-appointment-standing)
  if (appointment_armed_ && !appointment_departed_) {
    RCLCPP_INFO(get_logger(),
        "Pursuit fallback (%s): appointment at cell %d still stands (t_meet "
        "t+%.0fs) -> resuming exploration until the departure deadline.",
        why, appointment_.cell, appointment_.t_meet_ms / 1000.0);
    resumeExploring(why);
    return;
  }
  // No hybrid branch: with no appointment standing, hybrid takes pursuit's
  // ladder unchanged. (notes: pursuit-fallback-no-hybrid-branch)
  if (pursuitExploreFallback(why)) return;
  holdForTeam(why);
}

// Abandon the manoeuvre and resume exploring, with holdForTeam's exit hygiene
// but landing in PLAN. coverage_done_streak_ is cleared, or the same dispatch
// re-triggers next tick. (notes: resume-exploring-exit-hygiene)
void ExploPlannerNode::resumeExploring(const char* why) {
  standDownExploitation();
  abandonNavGoal(why);
  coverage_done_streak_ = 0;
  have_active_intent_   = false;
  transitionTo(State::PLAN, why);
  // Close the mid-run bookkeeping here too: transitionTo's block is gated on
  // reconnect_active_, false for a declined chase. Must run AFTER the
  // transition, which stamps reconnect_end with reconnect_terminal_.
  // (notes: resume-exploring-midrun-bookkeeping)
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
  // No destination: PLAN picks the next goal. refreshDispatchContext() is
  // required here, or peer_record_age_sec is stamped as of the chase trigger. A
  // declined chase has no matching reconnect_end.
  // (notes: pursuit-explore-fallback-refresh)
  refreshDispatchContext();
  logReconnectDispatch("resume_exploring", nullptr, /*budget_sec=*/-1.0, why);
  resumeExploring(why);
  return true;
}

// Raise the RETURN_SYNC barrier at the current pose. PURSUE is a driving state,
// so the platform must be stopped explicitly; the presence intent lets the
// pursued teammate count us. (notes: hold-for-team-barrier)
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
    // While coasting, shrink the claim disc to a token 0.5 m (0.0 means unset
    // and restores the full radius) so it cannot veto the partner's frontiers.
    // The beacon must keep going out: teamComplete counts presence.
    // (notes: presence-coast-claim-disc)
    const double claim_r =
        done_seek_coasting_ ? 0.5 : coord_claim_radius_m_;
    current_intent_msg_ = coord_->buildIntent(
        current_goal_, latest_pos_, this->now(),
        static_cast<float>(coord_claim_ttl_sec_),
        static_cast<float>(claim_r),
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
  // Team-presence clock for the reconnect confirmation gate, sampled here on
  // the heartbeat so it predates the long PLAN tick. A starved heartbeat reads
  // older, which is the conservative direction. (notes: hb-team-presence-clock)
  if (coord_) {
    const auto pres_now = this->now();
    // Real interval since the previous heartbeat: hb_last_tick_ is updated only
    // at the bottom of this function. Read here, before the anchor it
    // invalidates. (notes: hb-gap-before-anchor)
    const double hb_gap_sec =
        hb_last_tick_.nanoseconds() > 0 ? (pres_now - hb_last_tick_).seconds()
                                        : 0.0;
    if (teamComplete(accountedPeerCount(pres_now),
                     rendezvous_expected_peers_)) {
      team_last_complete_time_ = pres_now;
      team_seen_complete_ = true;
    }
    // The rendezvous runs on rendezvousTeamMutual(), stricter than
    // teamComplete(): a peer counts only when direct (heard inside the TTL and
    // naming us back), so both ends stop stamping on the same event.
    // (notes: rzv-mutual-stricter-clock)
    const bool rzv_mutual = rendezvousTeamMutual();
    // Freeze the snapshot BEFORE maintainRendezvousProposal reads it, on the
    // same tick and gate. The anchor deliberately stays below with the rest of
    // the mutual-tick bookkeeping. (notes: rzv-snapshot-before-handshake)
    if (rzv_mutual) refreshRendezvousSnapshot();
    // One handshake round every heartbeat, with the mutual-contact gate passed
    // as the argument: deriving requires it, but echoing the proposer's pair
    // must not. (notes: rzv-handshake-every-heartbeat)
    maintainRendezvousProposal(rzv_mutual);
    if (rzv_mutual) {
      // The anchor: the last instant the team was confirmed mutually whole,
      // stamped on the same tick as the snapshot and the handshake.
      // (notes: rzv-anchor-stamp)
      rendezvous_anchor_time_ = pres_now;
      have_rendezvous_anchor_ = true;
      // Freshly sampled under mutual contact, so whatever doubt a previous
      // starved tick raised about the anchor is settled.
      rendezvous_anchor_starved_ = false;
      // derive takes this gate; commit must not test the mask, it needs a
      // matching echo from every other robot. rendezvous_spent_ is released
      // below, outside this branch, not here. (notes: rzv-mutual-block-gates)
    } else if (hb_gap_sec >= team_model_.config().direct_ttl_sec) {
      // A heartbeat gap >= the direct TTL may hide the separation instant, so
      // the anchor is marked suspect until the next healthy stamp. Only the
      // anchor_sec column is affected; the appointment still arms.
      // (notes: rzv-anchor-starved)
      if (!rendezvous_anchor_starved_ && have_rendezvous_anchor_) {
        RCLCPP_WARN(get_logger(),
            "Rendezvous anchor SUSPECT: %.2f s heartbeat gap (>= direct TTL "
            "%.1f s) and the team is no longer in mutual contact. The "
            "separation instant may lie inside the gap, so the anchor_sec "
            "logged by this robot may be up to that much too early. The "
            "appointment is UNAFFECTED and will still arm — t_meet is the "
            "committed integer, not anchor + interval — so treat this as a "
            "caveat on the anchor column alone.",
            hb_gap_sec, team_model_.config().direct_ttl_sec);
      }
      rendezvous_anchor_starved_ = true;
    }

    // Release rendezvous_spent_ only when no appointment stands and teamSettled
    // has dwelt reconnect_release_confirm_sec_. Step dwellConfirmed every
    // heartbeat, outside the !appointment_armed_ test, or its clock freezes.
    // (notes: rzv-spent-release)
    const bool team_back_dwelt =
        dwellConfirmed(teamSettled(accountedPeerCount(pres_now)),
                       pres_now.seconds(), reconnect_release_confirm_sec_,
                       &team_back_ok_armed_, &team_back_ok_since_sec_);
    if (!appointment_armed_ && team_back_dwelt) {
      rendezvous_spent_ = false;
    }
  }
  // Suppression accounting: the beacon is state-gated, so a planner stuck in a
  // non-beaconing state reads as missing to peers under perfect comms. These
  // logs separate suppression from radio outage.
  // (notes: hb-suppression-accounting)
  const bool beaconing =
      have_active_intent_ && intent_pub_ &&
      (state_ == State::NAVIGATE || state_ == State::INTEGRATE ||
       state_ == State::EXPLOIT_PLAN || state_ == State::EXPLOIT_DWELL ||
       state_ == State::RETURN_NAV || state_ == State::RETURN_SYNC ||
       state_ == State::PURSUE || state_ == State::PROXIMITY_HOLD ||
       state_ == State::RETURN_HOME || state_ == State::DONE);
  const auto hb_now = this->now();
  // Second suppression cause, executor starvation: an inter-tick heartbeat gap
  // >= coord_claim_ttl_sec_ lets peers age our claim out with the link up. Only
  // the measured inter-tick interval can reveal it.
  // (notes: hb-executor-starvation)
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
            "peers now read this robot as MISSING with the link up. Classify "
            "any peer-missing window overlapping this as suppression, not "
            "outage.", held, stateName(state_), coord_claim_ttl_sec_);
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
  // Re-publish while holding a claim in these states: long dwells,
  // RETURN/PURSUE barriers, DONE (keeps the first finisher countable) and
  // PROXIMITY_HOLD (the interrupted goal's claim must survive).
  // (notes: hb-beacon-state-gate)
  if (state_ != State::NAVIGATE && state_ != State::INTEGRATE &&
      state_ != State::EXPLOIT_PLAN && state_ != State::EXPLOIT_DWELL &&
      state_ != State::RETURN_NAV && state_ != State::RETURN_SYNC &&
      state_ != State::PURSUE && state_ != State::PROXIMITY_HOLD &&
      state_ != State::RETURN_HOME && state_ != State::DONE) {
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
  // Map size rides the heartbeat too: this is the 1 Hz beacon that maintains
  // peer belief, so it is the freshest value a peer can snapshot at the last
  // exchange before an outage — exactly the sample the info gate runs on.
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

  // Resume the same goal; the re-publish is mandatory because the brake goal
  // replaced it at the navigator. Nav budget continues (backdated), progress
  // window restarts, exploit give-up timer gets the held time back.
  // (notes: prox-hold-resume-same-goal)
  if (exploit_target_timing_) exploit_target_started_sec_ += held;
  // The pursuit budget gets the held time back too: yielding to a teammate is
  // not evidence the chase hypothesis is wrong, and charging it would let one
  // crossing-route hold convert a viable pursuit into its fallback.
  if (prox_resume_state_ == State::PURSUE) {
    pursue_start_time_ =
        pursue_start_time_ + rclcpp::Duration::from_seconds(held);
  }
  // An in-flight appointment escape leg gets the held time back too, or a hold
  // can expire the leg cap (return_escape_leg_sec) while the robot is braked.
  // (notes: prox-hold-escape-leg-refund)
  if (appointment_escape_active_) {
    escape_leg_start_ = escape_leg_start_ + rclcpp::Duration::from_seconds(held);
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
  // Restart the homing approach watchdog too: doReturnHome does not run during
  // PROXIMITY_HOLD, so held time must not count as a homing stall.
  // (notes: prox-hold-approach-watchdog-reset)
  approach_check_time_   = now;
  approach_check_metric_ = homeApproachMetric();
  // A hold taken on the RETURN_HOME entry tick interrupted nothing: the home
  // goal is deferred to the first doReturnHome tick (see startReturnHome) and
  // current_goal_ still holds the pre-latch drive — republishing THAT would
  // send a finished robot back toward its old frontier for a tick.
  if (state_ == State::RETURN_HOME && !return_home_goal_sent_) return;
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
  // accountedPeerCount, so the column and the decisions cannot disagree. As of
  // schema 8 it documents "peers heard within one TTL on EITHER the claim table
  // or TeamWorld, plus peers that announced their run is over"; grace-retained
  // exploit claims still must not inflate it.
  m.coord_active_peers  = accountedPeerCount(now);
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

  // team_complete is written whenever a coordination table exists and
  // rendezvous_expected_peers_ > 0; -1 means no team to ask about
  // (teamComplete() alone folds that into 0).
  // (notes: csv-team-complete-sentinel)
  if (coord_ && rendezvous_expected_peers_ > 0) {
    const int live_now = accountedPeerCount(now);
    m.team_complete =
        teamComplete(live_now, rendezvous_expected_peers_) ? 1 : 0;
  }
  // Gated on the pursuit being live (PURSUE, or PROXIMITY_HOLD resuming
  // PURSUE), not on state_ alone, so mid-chase holds are recorded.
  // pursue_peer_id_ is never cleared, so the state pair bounds it.
  // (notes: csv-pursuit-live-gate)
  const bool pursuit_live =
      !pursue_peer_id_.empty() &&
      (state_ == State::PURSUE ||
       (state_ == State::PROXIMITY_HOLD &&
        prox_resume_state_ == State::PURSUE));
  if (pursuit_live) {
    m.pursue_peer = pursue_peer_id_;
    m.pursue_quarry_live = peerAccounted(pursue_peer_id_, now) ? 1 : 0;
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
    e.peers_live = accountedPeerCount(now);
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
    e.peers_live = accountedPeerCount(now);
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
    e.peers_live = accountedPeerCount(now);
    e.expected_peers = rendezvous_expected_peers_;
    exp_log_->logPeerLost(expCtx(), e);
  }
}

// Re-queries the missing-peer record at commit time, so every dispatch event
// reports its age then, including leaves that skip dispatchReconnect's walk
// (hold escalation, pursuitFallback). (notes: explog-dispatch-context-refresh)
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
  // Consumed, not just read: a decline belongs to one dispatch, and the
  // out-of-band leaves skip dispatchReconnect's clearing entry.
  // (notes: explog-dispatch-decline-consumed)
  reconnect_decline_reason_.clear();
  // predictor is run-wide and always written; the prediction belongs to one
  // chase and is consumed like decline_reason, so later leaves do not re-report
  // it. (notes: explog-dispatch-predict-consumed)
  e.predictor = pursuit_predictor_mdp_ ? "mdp" : "trail";
  if (have_dispatch_predict_) {
    e.predict_refused       = dispatch_predict_.refused;
    e.predict_p             = dispatch_predict_.p;
    e.predict_p_on_route    = dispatch_predict_.p_on_route;
    e.predict_cell          = dispatch_predict_.cell;
    e.predict_candidates    = dispatch_predict_.candidates;
    if (dispatch_predict_.horizon_ms >= 0)
      e.predict_horizon_sec = dispatch_predict_.horizon_ms / 1000.0;
    e.predict_tour_age_sec  = dispatch_predict_tour_age_sec_;
    have_dispatch_predict_  = false;
  } else {
    // An empty refusal on a chase means "the intercept was driven", so a leaf
    // that never asked the model must not leave the field empty.
    e.predict_refused = "not a chase";
  }
  e.attempt        = reconnect_terminal_ ? 0 : midrun_attempts_;
  e.peers_live     = accountedPeerCount(now);
  e.expected_peers = rendezvous_expected_peers_;
  // Gate diagnostics belong to the manoeuvre (same lifetime as
  // reconnect_rec_, NOT consumed-per-event like decline_reason): an
  // out-of-band leaf of a mid-run manoeuvre re-reports the gate decision it
  // was armed from, while terminal dispatches never carry a stale one.
  if (!reconnect_terminal_) {
    e.gate_sec         = dispatch_gate_sec_;
    e.est_unshared_vox = dispatch_est_unshared_;
    // Same manoeuvre lifetime: later leaves re-report the link-down duration
    // the trigger fired on. Terminal dispatches leave it -1; the link gate only
    // governs the mid-run trigger. (notes: explog-dispatch-link-down-lifetime)
    e.link_down_sec    = dispatch_link_down_sec_;
    e.team_incomplete_sec = dispatch_team_incomplete_sec_;
  }
  exp_log_->logReconnectDispatch(expCtx(), e);
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
  e.peers_live = accountedPeerCount(this->now());
  e.expected_peers = rendezvous_expected_peers_;
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
  // Evidence that the once-per-run guard is a guard and not a comment. Zero in
  // a healthy run; non-zero says the second request came and was refused, and
  // names how many times. Read it next to mission_complete's `occurrence`.
  e.mission_return_reentries = mission_return_reentries_;
  e.midrun_attempts_used     = midrun_attempts_;
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
    // Route through finishOrRendezvous, not straight to DONE, so a budget-spent
    // robot still returns and releases its teammate's barrier. On deferral go
    // to PLAN, never stay in LOG_STEP (each call logs and bumps step_).
    // (notes: logstep-budget-via-finish-or-rendezvous)
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
    // Arm the fine band when the target enters the queue, not on activation, so
    // every robot fine-maps released trunks its lidar reaches. Dedup'd
    // re-reports skip this. (notes: exploit-fine-band-arm-on-ingest)
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

// Called once, on the tick that latches shutdown_requested_. DONE-idle
// deliberately keeps regions armed. Idempotent with the per-target remove in
// finishActiveTarget. (notes: exploit-remove-live-regions-on-shutdown)
void ExploPlannerNode::removeLiveRefinementRegions() {
  if (!region_pub_) return;
  for (const auto& t : target_queue_.targets()) {
    if (t.status == Target::Status::DONE) continue;
    publishRefinementRegion(t.id, t.center, t.radius, /*remove=*/true);
  }
}

// Returns the in-ROI, free, reachable, non-blacklisted point on the
// trunk-to-robot ray closest to the trunk, or false if none is meaningfully
// nearer than the robot. Caller must flood cost_grid_ from robot_pos.
// (notes: exploit-approach-goal-march)
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

// Standing/sightline z for an exploit point: fixed height in flat mode, local
// ground plus clearance in terrain mode. Returns NaN when no ground is found;
// callers decide (vantages reject, approach uses robot z).
// (notes: exploit-z-terrain-nan-policy)
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

  // Publish the goal as an exploit intent: peers use it to take a different
  // angle, dwelled_mask carries this robot's clear-LoS credit, and
  // claim_radius_m ships the per-vantage disc.
  // (notes: exploit-intent-on-navigate)
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
  // Use the per-vantage disc, not coord_claim_radius_m, or one claim vetoes the
  // whole ring. Exploit claims are read as retained (grace included) via the
  // default nullptr liveness. (notes: exploit-veto-per-vantage-disc)
  const auto* peer = coord_->claimMatching(
      v_pos, static_cast<float>(coord_vantage_claim_radius_m_));
  // An exploit claim on this trunk is not contested by distance: the angle
  // belongs to its claimant until the claim lapses.
  // (notes: exploit-veto-claim-kind-not-distance)
  if (peer && peer->exploit && peer->target_id == target_id) return true;
  // Any other claim shape keeps the distance contest. An EXPLORATION claim
  // carries the fov-range disc (~8-10 m), wide enough to cover the entire
  // ring, so yielding to it unconditionally would veto every vantage of the
  // tree for as long as a peer explores anywhere near it.
  if (peer && !coord_->selfWinsAgainst(robot_pos, v_pos, *peer, robot_name_))
    return true;
  // Third rule: contest peers parked on this ring (staged claims) with the same
  // total order, so exactly one robot admits the angle before any claim exists.
  // Driving peers are not position-contested here.
  // (notes: exploit-veto-staged-parked-peers)
  return coord_->stagedExploitPeerWinning(
             target_id, v_pos, robot_pos, robot_name_, this->now()) != nullptr;
}

bool ExploPlannerNode::holdDwelledVantage(
    Target* tgt, const std::vector<CandidateViewpoint>& vantages,
    const Eigen::Vector3f& robot_pos) {
  // After this robot's own dwell, with every other angle team-visited or
  // peer-denied (vantageVetoedByPeers), park on the dwelled vantage with a
  // staged claim. Quota merge, claim lapse or target timeout end the hold.
  // (notes: exploit-hold-dwelled-vantage)
  if (!held_vantage_valid_ || held_vantage_target_id_ != tgt->id) return false;

  for (const auto& v : vantages) {
    if (target_queue_.isVantageVisited(
            v.position, static_cast<float>(vantage_visited_tol_m_)))
      continue;
    // Only a peer-denial keeps an unvisited angle off the table; blacklist
    // alone must not hold, or the ring stays short.
    // (notes: exploit-hold-blacklist-not-enough)
    if (!vantageVetoedByPeers(tgt->id, v.position, robot_pos)) return false;
  }

  // Ring covered: re-anchor onto the held vantage if the yield drifted us off
  // it. publishGoal alone suffices: no hop to cancel, and EXPLOIT_PLAN
  // re-enters every tick. (notes: exploit-hold-reanchor-pose)
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

  // Keep the staged hold-claim beating: after a yield the intent was released,
  // so rebuild it as staged on our own dwelled vantage; the heartbeat
  // re-publishes it at 1 Hz. (notes: exploit-hold-staged-claim-rebuild)
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

  // Ordering: target bookkeeping, the team-quota merge and the per-target
  // timeout run before the map waits below, so a missing map cannot pin
  // EXPLOIT_PLAN. (notes: exploit-plan-ordering-before-map)

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

  // (Re)start the per-target give-up timer on a new active target or after a
  // stand-down cleared exploit_target_timing_. Also re-armed by doNavigate and
  // doExploitDwell, refunded by doProximityHold.
  // (notes: exploit-plan-target-timer-restart)
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

  // Team quota: fold peers' clear-LoS dwell masks into this target before
  // checking success. Per-tick backup for onPeerExploitIntent when a peer
  // intent beat the TreeTarget ingest. (notes: exploit-plan-team-quota-merge)
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

  // Hard per-target wall-clock bound. Must stay above vantage selection and the
  // map waits so exploitation of one trunk always terminates and the phase
  // reverts to EXPLORE. (notes: exploit-plan-target-timeout-placement)
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

  // Hold branch (see holdDwelledVantage). Stays above the map refresh and flood
  // so a holding tick is cheap; quota merge and timeout stay above it so a hold
  // can always end. (notes: exploit-plan-hold-branch-placement)
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

  // With use_planning_map=false there is never a map: vantages use
  // straight-line geometry, so do not wait. With it enabled, wait for the map;
  // the per-target timeout bounds the wait.
  // (notes: exploit-plan-planning-map-wait)
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

  // Cost grid only with a planning_map; the flood is unbounded (cap <= 0) since
  // the trunk may be far. Cached on map identity and pose: EXPLOIT_PLAN
  // re-enters every tick when nothing is selectable.
  // (notes: exploit-plan-unbounded-flood-cache)
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
    // A NaN z means exploitZAt found no ground under this angle: reject rather
    // than guess. Counted in rej_noground, apart from rej_roi; inRoi is XY-only
    // and would not catch it. (notes: exploit-vantage-reject-no-ground)
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
    // Yield an angle a peer claims or a parked peer is about to win.
    // vantageVetoedByPeers() is shared with the hold branch above, keeping
    // parking and selectability mutually exclusive.
    // (notes: exploit-vantage-peer-veto)
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
    // Throttled: this branch re-runs every tick while nothing is selectable;
    // the rejection counters summarize each window.
    // (notes: exploit-no-vantage-log-throttle)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Target %u: no selectable vantage (valid=%d noground=%d roi=%d map=%d "
        "unreach=%d los=%d visited=%d blk=%d minpos=%d).",
        tgt->id, n_valid, rej_noground, rej_roi, rej_map, rej_unreach, rej_los,
        rej_visited, rej_blk, rej_minpos);

    // Quota-met and the per-target timeout are handled at the top of this
    // function. Head for the nearest reachable point on the line to the trunk
    // so the area maps; re-planning resumes on arrival.
    // (notes: exploit-approach-toward-trunk)
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
      // The separation term does not run on the exploit path; reset its
      // diagnostics to nothing-measured values so an exploit row never carries
      // a stale peer distance. (notes: exploit-approach-sep-diag-reset)
      pending_sep_peer_dist_m_    = -1.0f;
      pending_sep_discount_       = 1.0f;
      pending_sep_reordered_      = -1;
      pending_sep_eligible_peers_ = 0;

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
  // lineOfSightClear); the dwell's map-unavailable fallback reads it, so it
  // must be set on every selection.
  // (notes: exploit-stage-selection-los-verdict)
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
  // As on the approach path above: the separation term has no say in vantage
  // selection, so the exploit row must not inherit an exploration row's
  // measurement of it.
  pending_sep_peer_dist_m_    = -1.0f;
  pending_sep_discount_       = 1.0f;
  pending_sep_reordered_      = -1;
  pending_sep_eligible_peers_ = 0;

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

  // Ring barrier: while a peer claiming this trunk is not yet staged
  // (producer-declared), re-anchor state_enter_time_ so the dwell clock does
  // not start. Latched by dwell_sync_started_; started dwells run to
  // completion. (notes: dwell-sync-ring-barrier)
  if (exploit_dwell_sync_enabled_ && coord_ && coord_->enabled() &&
      !dwell_sync_started_ && !dwell_sync_timed_out_) {
    const Target* synced_target = target_queue_.active();
    if (synced_target) {
      const auto now = this->now();
      const auto* waiting_on =
          coord_->firstUnstagedExploitPeer(synced_target->id, now);
      if (!waiting_on) {
        // Barrier satisfied: latch the start for this dwell. Log only if the
        // barrier held at least one tick (state_enter_time_ >
        // dwell_sync_wait_start_), so solo captures stay silent.
        // (notes: dwell-sync-barrier-satisfied-log)
        if (state_enter_time_ > dwell_sync_wait_start_) {
          RCLCPP_INFO(get_logger(),
              "dwell-sync: team staged on target %u — dwell runs to "
              "completion.", synced_target->id);
        }
        dwell_sync_started_ = true;
      } else {
        const double waited = (now - dwell_sync_wait_start_).seconds();
        // max_wait <= 0 (the default) waits until the peer stages.
        // exploit_target_timeout_sec_ (if > 0) bounds the hold either way; when
        // hit, release and dwell solo rather than abandon the target.
        // (notes: dwell-sync-max-wait-and-deadline)
        const bool target_deadline_hit =
            exploit_target_timeout_sec_ > 0.0 &&
            (now.seconds() - exploit_target_started_sec_) >=
                exploit_target_timeout_sec_;
        const bool wait_bounded = exploit_dwell_sync_max_wait_sec_ > 0.0;
        if (!target_deadline_hit &&
            (!wait_bounded || waited < exploit_dwell_sync_max_wait_sec_)) {
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
        // Latched for the rest of this dwell; re-testing would re-anchor the
        // dwell clock and the dwell would never complete. transitionTo clears
        // it on the next EXPLOIT_DWELL entry. (notes: dwell-sync-timeout-latch)
        dwell_sync_timed_out_ = true;
        // Which deadline fired is worth saying out loud: the max_wait one is a
        // tuning question, the per-target one means this robot has spent its
        // whole budget for the trunk sitting still, which is a different thing
        // to go and look at.
        if (target_deadline_hit) {
          RCLCPP_WARN(get_logger(),
              "dwell-sync: per-target budget spent after %.0fs of waiting for "
              "'%s' on target %u (exploit_target_timeout %.0fs) — DWELLING "
              "SOLO. That peer is claiming the trunk without ever staging on "
              "it; check it for a proximity hold or an unreachable ring.",
              waited, waiting_on->robot_id.c_str(), synced_target->id,
              exploit_target_timeout_sec_);
        } else {
          RCLCPP_WARN(get_logger(),
              "dwell-sync: barrier timed out after %.0fs waiting for '%s' on "
              "target %u (limit %.0fs) — DWELLING SOLO. This capture is not "
              "simultaneous with the team's; check that peer for a proximity "
              "hold or an unreachable ring.",
              waited, waiting_on->robot_id.c_str(), synced_target->id,
              exploit_dwell_sync_max_wait_sec_);
        }
      }
    }
  }

  // No goal re-send during the dwell; the goal is already reached. The
  // controller may still rotate briefly at dwell start (its yaw tolerance is
  // tighter than the planner's); closing that gap deadlocks arrival.
  // (notes: dwell-no-goal-resend)
  if (elapsed < exploit_dwell_sec_) return;

  // Re-confirm LoS from the settled XY at sightline z (exploitZAt, not
  // base_link z); this verdict counts toward the quota and the CSV. Keep the
  // selection-time verdict if map, target or ground is unavailable.
  // (notes: dwell-reconfirm-los-sightline-z)
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

  // Remember the exact pose this dwell was captured from; if the rest of the
  // ring gets covered, doExploitPlan's hold branch parks back on this position
  // and yaw. (notes: dwell-remember-held-vantage-pose)
  if (t) {
    held_vantage_pose_      = current_goal_;
    held_vantage_index_     = current_vantage_index_;
    held_vantage_target_id_ = t->id;
    held_vantage_valid_     = true;
  }

  // Publish the credit mask now so peers see this dwell before the claim is
  // released. Patch current_intent_msg_ in place; do not rebuild via
  // buildIntent(), which would reset staged while still on the vantage.
  // (notes: dwell-credit-mask-patch-intent)
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
  // Outbound-only breadcrumb trail for the retrace fallback: first crumb is
  // home, 2 m spacing. PROXIMITY_HOLD counts as homing only when it interrupted
  // RETURN_HOME or DONE. (notes: mission-return-breadcrumb-trail)
  const bool homing_now =
      state_ == State::RETURN_HOME || state_ == State::DONE ||
      (state_ == State::PROXIMITY_HOLD &&
       (prox_resume_state_ == State::RETURN_HOME ||
        prox_resume_state_ == State::DONE));
  if (mission_return_enabled_ && have_home_ && !homing_now &&
      (home_trail_.empty() ||
       (latest_pos_ - home_trail_.back()).head<2>().norm() >= 2.0f)) {
    home_trail_.push_back(latest_pos_);
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
      // Latched for doReturnHome: the approach watchdog measures a DELTA in
      // distance-to-home, and a teleport moves that delta by metres in one
      // tick in either direction. The homing tick clears the flag.
      pose_jump_seen_ = true;
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

// Keep-alive for states still driving (NAVIGATE, RETURN_NAV): publishes on pose
// change, new subscriber, or after goal_republish_sec_, never at tick rate. Not
// called from EXPLOIT_DWELL. (notes: goal-republish-keepalive)
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
// TeamWorld exchange (P2)
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
  // times, which TeamModel::observe refuses outright — so the whole exchange
  // would go silent on a condition that ought to cost one tick.
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

void ExploPlannerNode::publishTeamWorld() {
  if (!team_world_pub_ || !cell_world_.configured()) return;

  const double t = missionElapsed();
  if (t < 0.0) return;   // clock not live yet; nothing can be dated

  // No pose, no publish: position is mandatory and cannot express unknown, so
  // an early publish would put this robot at the map origin for every peer.
  // (notes: team-world-no-pose-no-publish)
  if (!have_pose_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "TeamWorld: not publishing yet — no pose. The message cannot express "
        "'position unknown', so an early publish would place this robot at "
        "the map origin for the whole fleet.");
    return;
  }

  explo_planner_msgs::msg::TeamWorld m;
  // Diagnostic only, as the message header says. Nothing times anything off
  // it; the two scheduling quantities below are mission-elapsed.
  m.header.stamp    = this->now();
  m.header.frame_id = map_frame_;
  m.robot_id  = static_cast<uint8_t>(fleet_.self_id);
  m.team_hash = fleet_.team_hash;
  m.grid_hash = cell_world_.grid().configHash();
  m.position.x = latest_pos_.x();
  m.position.y = latest_pos_.y();
  m.position.z = latest_pos_.z();

  // directMask(), not inCommsMask(): publishing the relay closure would make
  // the mutual-contact handshake circular. (notes: team-world-direct-mask)
  m.in_range_mask = team_model_.directMask();

  // This robot's own first-hand answer to is the team whole, never its derived
  // armed state. Published whatever the local mode. Requires
  // rendezvous_expected_peers_ > 0, so an inert config reports no break.
  // (notes: team-world-team-incomplete-bit)
  m.team_incomplete =
      rendezvous_expected_peers_ > 0 &&
      !teamComplete(
          accountedPeerCount(this->now()),
          rendezvous_expected_peers_);

  // Set while driving to the agreed cell. Keyed on RETURN_NAV with
  // appointment_manoeuvre_, not an arrival test, so the bit clears however the
  // drive ends. (notes: team-world-appointment-inbound)
  m.appointment_inbound =
      appointment_manoeuvre_ && state_ == State::RETURN_NAV;

  // One-hop report of a peer inbound to the appointment, derived from raw
  // first-hand bits only; the wider peer-report predicate would relay an echo
  // nothing can clear. (notes: team-world-inbound-seen-one-hop)
  m.appointment_inbound_seen = peerInboundToAppointment();

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

  // Send my_tour_ only while this robot drives it (PROXIMITY_HOLD judged by
  // prox_resume_state_), else clear it; empty means no route. Sent whatever the
  // local predictor setting. Ids beyond uint16 truncate the tour.
  // (notes: team-world-my-tour-live-only)
  const State tour_state = (state_ == State::PROXIMITY_HOLD)
                               ? prox_resume_state_
                               : state_;
  const bool tour_is_live =
      phase_ == Phase::EXPLORE && !coverage_latched_ &&
      (tour_state == State::PLAN || tour_state == State::NAVIGATE ||
       tour_state == State::INTEGRATE || tour_state == State::LOG_STEP);
  m.my_tour.clear();
  if (!tour_is_live) {
    my_tour_.clear();
  }
  m.my_tour.reserve(my_tour_.size());
  for (int cid : my_tour_) {
    if (cid < 0 || cid > static_cast<int>(
                             std::numeric_limits<uint16_t>::max())) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "TeamWorld: tour cell %d does not fit the uint16 wire field; "
          "broadcasting the route truncated at %zu of %zu cells.",
          cid, m.my_tour.size(), my_tour_.size());
      break;
    }
    m.my_tour.push_back(static_cast<uint16_t>(cid));
  }

  // The pair this robot holds (its own if proposer, else the proposer's
  // echoed). Absent fields are -1, never zero-init, since 0 is a valid cell id.
  // Publishing is not driving; the commit rule decides.
  // (notes: team-world-rendezvous-held-pair)
  m.rendezvous_cell_id     = rendezvous_held_.cell;
  m.rendezvous_interval_ms = -1;
  m.rendezvous_t_meet_ms   = -1;
  // rendezvous_provisional is false whenever the triple is absent; the int32
  // refusal path below must clear it together with the cell.
  // (notes: team-world-rendezvous-provisional)
  m.rendezvous_provisional =
      rendezvous_held_.valid() && rendezvous_held_provisional_;
  if (rendezvous_held_.valid()) {
    // If interval_ms or t_meet_ms exceeds int32, refuse the whole triple
    // (broadcast no proposal) rather than truncate. Check both together; t_meet
    // overflows first. (notes: team-world-rendezvous-int32-guard)
    constexpr long long kMaxMs =
        static_cast<long long>(std::numeric_limits<int32_t>::max());
    if (rendezvous_held_.interval_ms <= kMaxMs &&
        rendezvous_held_.t_meet_ms <= kMaxMs) {
      m.rendezvous_interval_ms =
          static_cast<int32_t>(rendezvous_held_.interval_ms);
      m.rendezvous_t_meet_ms =
          static_cast<int32_t>(rendezvous_held_.t_meet_ms);
    } else {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 30000,
          "TeamWorld: rendezvous interval %lld ms / t_meet %lld ms does not fit "
          "the int32 wire field; broadcasting 'no proposal' rather than a "
          "truncated meeting time the peers would believe.",
          rendezvous_held_.interval_ms, rendezvous_held_.t_meet_ms);
      m.rendezvous_cell_id     = -1;
      m.rendezvous_provisional = false;   // "no proposal" is never provisional
    }
  }

  // finished = coverage_latched_ (early) OR State::DONE (every run ending).
  // Latched in finished_announced_ and never cleared, so relayed copies stay
  // sound; State::DONE alone can be left.
  // (notes: team-world-finished-latch-or-done)
  if (coverage_latched_ || state_ == State::DONE) finished_announced_ = true;
  m.finished = finished_announced_;

  // Latched like finished and never read from state_; DONE outranks HOMING so
  // the level only rises. A finished robot keeping an appointment reports below
  // HOMING until the manoeuvre ends (announcedMode).
  // (notes: team-world-announced-mode-levels)
  using TeamWorldMsg = explo_planner_msgs::msg::TeamWorld;
  static_assert(kModeExploring == TeamWorldMsg::MODE_EXPLORING &&
                    kModeHoming == TeamWorldMsg::MODE_HOMING &&
                    kModeDone == TeamWorldMsg::MODE_DONE,
                "meeting_attendance.hpp's mode levels must match TeamWorld.msg");
  if (state_ == State::RETURN_HOME || mission_return_done_)
    homing_announced_ = true;
  m.mode = announcedMode(finished_announced_, appointment_manoeuvre_,
                         homing_announced_, done_announced_);

  // A robot in DONE never reports team_incomplete. Redundant with finished on
  // purpose, for consumers that read team_incomplete directly.
  // (notes: team-world-done-no-team-break)
  if (state_ == State::DONE) m.team_incomplete = false;

  // --- gossip ---------------------------------------------------------
  //
  // Sized to the fleet and indexed by id ALWAYS, including where nothing is
  // known: the receiver's length check reads a short array as "the sender is
  // running a smaller team definition" and refuses the whole message. Absence
  // is expressed by a negative last-heard, not by a missing entry.
  const size_t n = static_cast<size_t>(fleet_.size());
  m.robot_positions.assign(n, geometry_msgs::msg::Point());
  m.robot_last_heard_sec.assign(n, -1.0f);
  m.robot_finished.assign(n, false);
  // EXPLORING is the "no evidence" fill, not a claim that the robot is
  // exploring; see TeamWorld.msg/robot_mode.
  m.robot_mode.assign(n, TeamWorldMsg::MODE_EXPLORING);
  for (size_t i = 0; i < n; ++i) {
    const int id = static_cast<int>(i);
    if (id == fleet_.self_id) {
      m.robot_finished[i] = m.finished;
      m.robot_mode[i]     = m.mode;
      // Our own entry is our publish time; the receiver reads each peer's age
      // as (our entry - that entry). Without it the whole array is dropped.
      // (notes: team-world-gossip-self-entry)
      m.robot_positions[i]      = m.position;
      m.robot_last_heard_sec[i] = static_cast<float>(t);
      continue;
    }
    const TeamModel::Peer& p = team_model_.peer(id);
    // Relay only first-hand direct contact (last_direct_sec,
    // position_first_hand), bounding relay to one hop. Position and last-heard
    // go as a pair or not at all; a lone last-heard makes (0,0,0) real.
    // (notes: team-world-gossip-first-hand-pair)
    if (p.last_direct_sec >= 0.0 && p.have_position && p.position_first_hand) {
      m.robot_last_heard_sec[i] = static_cast<float>(p.last_direct_sec);
      m.robot_positions[i].x    = p.position_x;
      m.robot_positions[i].y    = p.position_y;
      m.robot_positions[i].z    = p.position_z;
    }
    // robot_finished skips the pairing and freshness rules and is relayed
    // however we hold it: the bit is monotonic and only removes a robot from
    // the waited-for set, so re-relaying is idempotent.
    // (notes: team-world-gossip-finished-relay)
    if (p.finished) m.robot_finished[i] = true;
    // Relayed on exactly the terms of the line above — no pairing rule, no
    // freshness rule, relayed whether held first-hand or by relay — with MAX
    // in place of OR because the fact has three ordered levels. Raising only
    // is what keeps re-relaying idempotent, and so keeps it safe.
    m.robot_mode[i] = std::max(m.robot_mode[i], p.mode);
  }

  team_world_pub_->publish(m);
}

void ExploPlannerNode::drainTeamWorld() {
  if (!team_model_.configured() || !cell_world_.configured()) return;

  const double t = missionElapsed();
  // Before the clock is live nothing can be dated, and TeamModel refuses a
  // non-finite or negative receipt time outright. The pending slots are LEFT
  // rather than dropped: they are full state, so the first tick with a live
  // clock merges them intact and loses only the (zero) elapsed time.
  if (t < 0.0) return;

  std::map<int, PendingTeamWorld> batch;
  batch.swap(team_world_pending_);

  // One row per drained message, in two passes: merge fields here, comms fields
  // after the single team_model_.tick below, so every row reports the same
  // comms picture. (notes: team-drain-two-pass-rows)
  std::vector<TeamExchangeEvent> rows;
  rows.reserve(batch.size());

  // Capture each sender's last-known age in a pre-pass before any observe():
  // observe() gossips third-party times and would shrink later senders' gaps.
  // This is the receiver-side inter-arrival gap.
  // (notes: team-drain-pre-known-age)
  std::map<int, double> pre_known_age;
  for (const auto& kv : batch) {
    pre_known_age[kv.first] = team_model_.lastKnownAgeSec(kv.first, t);
  }

  for (auto& kv : batch) {
    const int sid = kv.first;
    const explo_planner_msgs::msg::TeamWorld& msg = *kv.second.msg;

    TeamExchangeEvent e;
    e.peer_id       = sid;
    e.peer          = fleet_.nameOf(sid);
    e.coalesced     = kv.second.superseded;
    e.queue_age_sec =
        std::max(0.0, this->now().seconds() - kv.second.received.seconds());
    e.cells_in_msg  = static_cast<int>(msg.cells.size());
    e.peer_in_range_mask = msg.in_range_mask;

    // --- config checks, before anything is believed -------------------
    // Hard drops, not partial merges: a mismatched fleet or grid merges cleanly
    // onto the wrong ground or robot. (notes: teamworld-config-checks)
    if (sid < 0 || sid >= fleet_.size()) {
      e.drop_reason = "sender_id_out_of_fleet";
    } else if (msg.team_hash != fleet_.team_hash) {
      e.drop_reason = "team_hash_mismatch";
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 30000,
          "TeamWorld from robot %d: team_hash 0x%08x != ours 0x%08x — that "
          "fleet's robot ids and mask bits address different robots than "
          "ours. Dropping every message from it.",
          sid, msg.team_hash, fleet_.team_hash);
    } else if (msg.grid_hash != cell_world_.grid().configHash()) {
      e.drop_reason = "grid_hash_mismatch";
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 30000,
          "TeamWorld from %s: grid_hash 0x%08x != ours 0x%08x — its cell ids "
          "name different ground than ours (different ROI or cell size). "
          "Dropping; merging would be a confident census of the wrong map.",
          e.peer.c_str(), msg.grid_hash, cell_world_.grid().configHash());
    }

    if (e.drop_reason.empty()) {
      // --- comms model ------------------------------------------------
      TeamModel::Observation o;
      o.sender_id     = sid;
      o.in_range_mask = msg.in_range_mask;
      o.finished      = msg.finished;
      // Copied verbatim and deliberately NOT validated against the MODE_*
      // constants: the scale is open-ended upward by contract, so an
      // unrecognised level is a level above DONE and every threshold read
      // downstream answers correctly for it. See TeamWorld.msg/mode.
      o.mode          = msg.mode;
      // The sender's own first-hand break bit, copied verbatim and never
      // re-broadcast as ours (TeamModel::observe stores it first-hand only).
      o.team_incomplete = msg.team_incomplete;
      // Likewise first-hand only: "I am still driving to the agreed cell".
      o.appointment_inbound = msg.appointment_inbound;
      // And the sender's one-hop report of that bit seen in ITS peers.
      o.appointment_inbound_seen = msg.appointment_inbound_seen;
      o.have_position = true;   // mandatory field; the sender withholds the
                                // whole message rather than send a fake pose
      o.x = msg.position.x;
      o.y = msg.position.y;
      o.z = msg.position.z;
      o.last_heard_sec.reserve(msg.robot_last_heard_sec.size());
      for (float v : msg.robot_last_heard_sec)
        o.last_heard_sec.push_back(static_cast<double>(v));
      // Copied apart from the position loop: sized independently on the wire
      // and merged without its freshness rules. An empty array reads as no
      // relayed evidence. (notes: team-drain-finished-gossip-copy)
      o.finished_gossip.reserve(msg.robot_finished.size());
      for (bool v : msg.robot_finished)
        o.finished_gossip.push_back(v ? 1u : 0u);
      // Same treatment, same reasons; already the right element type, so it
      // copies rather than converting.
      o.mode_gossip = msg.robot_mode;
      const size_t gn = msg.robot_positions.size();
      o.gx.resize(gn); o.gy.resize(gn); o.gz.resize(gn);
      // The wire carries no have_gossip_pos flag; the message defines a
      // position as meaningful exactly when its matching last-heard is
      // non-negative, so that test IS the flag and is reconstructed here.
      o.have_gossip_pos.assign(gn, 0u);
      for (size_t k = 0; k < gn; ++k) {
        o.gx[k] = msg.robot_positions[k].x;
        o.gy[k] = msg.robot_positions[k].y;
        o.gz[k] = msg.robot_positions[k].z;
        o.have_gossip_pos[k] =
            (k < msg.robot_last_heard_sec.size() &&
             msg.robot_last_heard_sec[k] >= 0.0f) ? 1u : 0u;
      }
      const std::string err = team_model_.observe(o, t);
      if (!err.empty()) {
        // The model refused it, so the cell world must not merge it either:
        // the refusals are all statements that this sender's ids do not mean
        // what ours do, and that applies to cell ids as much as robot ids.
        e.drop_reason = "team_model: " + err;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
            "TeamWorld from %s refused by the comms model: %s",
            e.peer.c_str(), err.c_str());
      }
    }

    if (e.drop_reason.empty()) {
      // --- cell world merge -------------------------------------------
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
      // Local priority is anchored on the cell this robot stands in, recomputed
      // each drain. -1 (outside the ROI, or team_merge_local_priority_ off)
      // disables the rule. (notes: team-merge-local-priority-centre)
      const int centre =
          team_merge_local_priority_
              ? cell_world_.grid().idAt(latest_pos_.x(), latest_pos_.y())
              : -1;
      const CellWorld::MergeStats st = cell_world_.mergeWire(sid, wire, centre);
      e.drop_reason    = st.refused;   // "" unless refused wholesale
      e.applied        = st.applied;
      // Accumulated as well as logged per message: the rendezvous release needs
      // "did anything arrive between these two instants", and reconstructing
      // that from the event rows would make a live decision depend on a file.
      team_merge_applied_total_ += st.applied;
      e.known_by_only  = st.known_by_only;
      e.agreed_noop    = st.agreed_noop;
      e.refused_guard  = st.refused_guard;
      e.refused_local  = st.refused_local;
      e.out_of_range   = st.out_of_range;
      e.bad_status     = st.bad_status;
    }

    // --- the peer's route (P6, §3.7) ----------------------------------
    // After every check (grid_hash makes the cell ids comparable); recorded
    // even when our predictor is off. Overwrites unconditionally, empty tour
    // included: absence is information. (notes: teamworld-peer-route)
    if (e.drop_reason.empty()) {
      PeerTour& pt = peer_tours_[sid];
      pt.cells.clear();
      pt.cells.reserve(msg.my_tour.size());
      for (uint16_t c : msg.my_tour) pt.cells.push_back(static_cast<int>(c));
      pt.pos = Eigen::Vector3f(static_cast<float>(msg.position.x),
                               static_cast<float>(msg.position.y),
                               static_cast<float>(msg.position.z));
      // The RECEIPT time, not the queue time: kv.second.received is when the
      // callback saw it, which is what "how stale is this route" means. Using
      // now() would credit the message with a freshness the drain delay
      // already spent, and drainTeamWorld runs on the planning tick.
      pt.stamp = kv.second.received;

      // --- the peer's rendezvous proposal (P5) ------------------------
      // Same checks gate it: the protocol compares cell ids for equality, so a
      // mismatched grid would match integers naming different ground.
      // Overwrites unconditionally, empty pair included.
      // (notes: teamworld-peer-proposal)
      if (sid >= 0 && sid < static_cast<int>(rendezvous_peer_.size()) &&
          sid < static_cast<int>(rendezvous_peer_at_sec_.size()) &&
          sid < static_cast<int>(rendezvous_peer_provisional_.size())) {
        RendezvousProposal& rp = rendezvous_peer_[sid];
        rp.cell        = msg.rendezvous_cell_id;
        rp.interval_ms = msg.rendezvous_interval_ms;
        rp.t_meet_ms   = msg.rendezvous_t_meet_ms;
        // Stored beside the pair, and cleared with it on every path below that
        // drops one. A flag surviving the pair it qualified would say "the peer
        // holds a placeholder" about a peer holding nothing, and the follower
        // adopt rule reads exactly that conjunction.
        rendezvous_peer_provisional_[sid] =
            msg.rendezvous_provisional ? 1 : 0;
        // Bound the cell id against our own grid: valid() only checks cell >= 0
        // and an adopted cell reaches grid().centre(). A grid_hash match does
        // not mean the id is inside our grid.
        // (notes: rendezvous-peer-cell-bounds)
        if (rp.cell >= 0 && !cell_world_.grid().valid(rp.cell)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
              "TeamWorld from %s proposes cell %d, which is outside this "
              "robot's grid — dropping the proposal.", e.peer.c_str(), rp.cell);
          rp = RendezvousProposal{};
        }
        if (!rp.valid()) rp = RendezvousProposal{};   // normalise partial pairs
        if (!rp.valid()) rendezvous_peer_provisional_[sid] = 0;
        // Dated by local receipt on the local mission clock; the producer's
        // clock never enters. Falls back to the drain instant t if the stamp
        // predates the mission baseline.
        // (notes: rendezvous-peer-receipt-dating)
        const double at = missionElapsedAt(kv.second.received);
        rendezvous_peer_at_sec_[sid] = at >= 0.0 ? at : t;
      }
    }

    rows.push_back(std::move(e));
  }

  // Once, for the whole batch, and unconditionally — including when the batch
  // was empty. TTL expiry is a function of time, not of arrivals: a model that
  // only advanced when a message came in could never notice that they stopped,
  // which is the one thing it exists to notice.
  team_model_.tick(t);

  if (!exp_log_ || rows.empty()) return;

  // Second pass. Every row gets the SAME post-tick comms picture, which is the
  // one every other consumer will read this tick.
  const CellWorld::Census cs = cell_world_.census();
  const uint32_t self_mask   = team_model_.directMask();
  const int lost             = team_model_.lostCount();
  for (TeamExchangeEvent& e : rows) {
    if (e.peer_id >= 0 && e.peer_id < team_model_.size()) {
      const TeamModel::Peer& p = team_model_.peer(e.peer_id);
      e.in_comms  = p.status == CommsStatus::IN_COMMS;
      e.direct    = p.direct;
      e.one_way   = p.heard_one_way;
      e.via_relay = p.via_relay;
      e.last_direct_age_sec = team_model_.lastDirectAgeSec(e.peer_id, t);
      // Read with the post-tick comms picture, where transitions are computed;
      // safe because neither transition can occur on a tick without a packet
      // from this peer. (notes: team-drain-acquire-held-read)
      e.acquire_sec = p.acquire_sec;
      e.held_sec    = p.held_sec;
      // NOT re-read here — see the pre-pass. This is the age as it stood
      // BEFORE this message was believed; reading the model now would give
      // 0.0 on every row, which is what v7 did.
      const auto it = pre_known_age.find(e.peer_id);
      if (it != pre_known_age.end()) e.last_known_age_sec = it->second;
    }
    e.peers_lost      = lost;
    e.self_direct_mask = self_mask;
    // The census that this merge produced, on the same line as the merge. A
    // convergence check ("did both robots' worlds agree after the dropout
    // healed?") is then a read of one field per robot, rather than a join of
    // two robots' files on a timestamp neither of them shares.
    e.covered_fraction    = cs.coveredFraction();
    e.covered             = cs.covered;
    e.covered_by_others   = cs.covered_by_others;
    e.exploring           = cs.exploring;
    e.exploring_by_others = cs.exploring_by_others;
    exp_log_->logTeamExchange(expCtx(), e);
  }
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
