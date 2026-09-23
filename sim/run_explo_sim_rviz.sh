#!/usr/bin/env bash
# Host-side orchestrator for a WATCHABLE N-robot exploration + exploitation run
# in the hmr_sim `flatforest` world, lidar-only, with RViz.
#
# The robot count N comes from the scenario yaml, not from this file.
# (notes: harness-team-size-from-scenario)
#
# Everything is NATIVE on the host (ROS humble + the ws overlay). Nothing in
# Docker — hmr_sim is never containerised.
#
# What you see in RViz (config/explo_sim_2robot_lidar.rviz):
#   * both husky meshes, from a robot_state_publisher per robot serving the viz
#     URDF wrapper around hmr_sim's SDF meshes (hmr_sim bridges no
#     /robot_description, so RobotModel has nothing to draw otherwise);
#   * the merged TEAM map (atlas's dscovox_node copy) as height-coloured voxels;
#   * each planner's candidate arrows (red -> green by utility) + selected goal.
#     During EXPLOIT those arrows ARE the vantage ring around the active trunk;
#   * released tree targets as a trunk cylinder + vantage ring + label
#     (sim_target_markers.py — TreeTarget is a custom msg RViz cannot draw);
#   * off by default, one click away: planning maps, raw VLP-16 clouds, nav
#     global/local paths, odom trails, bestla's own copy of the merged map.
#
# The run is exploration + exploitation: the scheduler releases the three
# targets_flatforest.yaml trunks at t = 120 / 420 / 720 sim-seconds; both robots
# activate each one, split the vantage ring through /exploration/intents claims,
# and the team's clear-LoS dwell union closes it. Between targets both revert to
# EIG exploration.
#
# Usage (from anywhere):
#   ./run_explo_sim_rviz.sh                 # runs until Ctrl-C
#   DURATION_S=960 ./run_explo_sim_rviz.sh  # stop after 960 sim-seconds
#   GZ_GUI=1 ./run_explo_sim_rviz.sh        # + the ignition GUI (costs RTF)
#   RECORD=1 ./run_explo_sim_rviz.sh        # + a rosbag under $OUTDIR
#   RVIZ=0   ./run_explo_sim_rviz.sh        # headless; NOTE the planner gates
#                                           # candidate-marker publishing on
#                                           # subscriber count, so no RViz means
#                                           # no marker traffic at all
#   TARGETS=/path/to/targets.yaml           # a different trunk triple, or a
#                                           # never-releasing file for
#                                           # exploration-only
#   DWELL_SYNC=0 ./run_explo_sim_rviz.sh    # A/B: drop the vantage-ring
#                                           # rendezvous barrier (first robot to
#                                           # arrive dwells alone) — see below
#   RECONNECT_MODE=mtare_rendezvous ./run_explo_sim_rviz.sh
#                                           # A/B: reconnection manoeuvre when a
#                                           # teammate goes out of comms. The
#                                           # runnable arm tokens are mtare_off |
#                                           # mtare_pursuit | mtare_rendezvous |
#                                           # mtare_hybrid (+ the two _mdp ones),
#                                           # plus plain `pursuit` and `off`.
#                                           # Plain `rendezvous`/`hybrid` are
#                                           # vocabulary only and cannot run —
#                                           # see the RECONNECT_MODE block below
#   CELL_WORLD=1 ./run_explo_sim_rviz.sh    # + the coarse M-TARE cell layer:
#                                           # cell_census events in the jsonl
#                                           # and a colour-coded grid in RViz.
#                                           # Observation only in P1 — nothing
#                                           # reads it. CELL_SIZE_M /
#                                           # CELL_CENSUS_S tune it;
#                                           # CELL_COVERED_U / CELL_EXPLORING_U
#                                           # / CELL_FRONTIER_FRAC are the
#                                           # world-calibrated status
#                                           # thresholds, overridden here for
#                                           # the same reason DONE_UNKNOWN is
#   FOV_VFOV=0.785 FOV_V_RAYS=12 ./run_explo_sim_rviz.sh
#                                           # A/B: override the EIG evaluator's
#                                           # modelled sensor geometry for this
#                                           # run (FOV_HFOV / FOV_VFOV /
#                                           # FOV_H_RAYS / FOV_V_RAYS /
#                                           # CAND_N_YAW). Unset = whatever
#                                           # shared_params.yaml says — see below
# Ctrl-C tears the whole stack down in reverse start order (SIGINT to each
# process group, then SIGKILL to stragglers), the same teardown the campaign
# driver used.
# Moved comments: docs/sim_notes/run_explo_sim_rviz_notes.md
set -u
# Adds a decimal point to integer values. Use it for every float knob: ros2
# infers the param type from the literal, and an int for a double aborts the
# node. Non-finite input prints FATAL and emits nothing.
# (notes: flt-float-param-literals)
flt() {
  case "$1" in
    *[nN][aA][nN]*|*[iI][nN][fF]*)
      echo "FATAL: non-finite value '$1' for a float parameter" >&2 ;;
    *.*|*e*|*E*) printf '%s' "$1" ;;
    *) printf '%s.0' "$1" ;;
  esac
}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # <repo>/sim
# The repo is expected checked out at …/hmr_explo/ws/src/explo_planner, so the
# ws overlay root is three levels up from this sim/ directory.
WS="$(cd "$HERE/../../.." && pwd)"           # …/hmr_explo/ws
# Overridable. The scenario picks the world, which sets the link budget and the
# coverage floor, so a change invalidates the calibrated done_unknown_fraction;
# recorded in the manifest. (notes: scenario-override-and-calibration)
SCENARIO="${SCENARIO:-flatforest_2robot_lidar.yaml}"
SCENARIO_PATH="$WS/install/hmr_sim/share/hmr_sim/config/scenarios/$SCENARIO"
if [ ! -f "$SCENARIO_PATH" ]; then
  echo "FATAL: scenario '$SCENARIO' is not installed. Rebuild hmr_sim, or pick"\
       "one of:" >&2
  ls "$WS/install/hmr_sim/share/hmr_sim/config/scenarios/" 2>/dev/null >&2
  exit 2
fi
# The roster is read from the scenario, the same file comms_sim.launch.py reads,
# so harness and emulator cannot disagree. Not overridable from the environment;
# to change it, change the scenario. (notes: roster-from-scenario)
ROBOTS="$(python3 - "$SCENARIO_PATH" <<'PY'
import sys, yaml
with open(sys.argv[1]) as f:
    doc = yaml.safe_load(f) or {}
names = [r["name"] for r in (doc.get("robots") or [])]
if len(names) < 2:
    sys.exit("FATAL: scenario names %d robot(s); this harness needs >= 2" % len(names))
if len(set(names)) != len(names):
    sys.exit("FATAL: scenario has duplicate robot names: %s" % names)
print(" ".join(names))
PY
)" || exit 2
N_ROBOTS=$(echo $ROBOTS | wc -w)
DURATION_S="${DURATION_S:-0}"                # 0 = until Ctrl-C
RVIZ="${RVIZ:-1}"
GZ_GUI="${GZ_GUI:-0}"                        # 1 => ignition GUI as well
RECORD="${RECORD:-0}"
MAX_STEPS="${MAX_STEPS:-500}"
# FRONTIER_ONLY=1 (default) sets candidate_enable_polar false: EXPLORE
# candidates are frontier centroids only; exploitation vantages are unaffected.
# Frontier-only can run out of candidates; 0 restores polar.
# (notes: frontier-only-candidates)
FRONTIER_ONLY="${FRONTIER_ONLY:-1}"
# Proximity-yield band (m), sim only: the yaml's 5.0/6.0 (real-robot reaction
# budget) would hold robots on adjacent ring vantages ~3.5 m apart in
# PROXIMITY_HOLD. Doubles, so through flt(). (notes: proximity-yield-sim-band)
PROX_HOLD_M="$(flt "${PROX_HOLD_M:-1.5}")"
PROX_RESUME_M="$(flt "${PROX_RESUME_M:-2.5}")"
# DWELL_SYNC=1 (default): a robot at its vantage holds, dwell clock not started,
# until every peer claiming the same trunk stands on its own angle. 0: the first
# arrival dwells alone. (notes: dwell-sync-barrier)
DWELL_SYNC="${DWELL_SYNC:-1}"
# EXPLOIT=0: pure exploration, exploitation disabled and no target_scheduler.
# The comms/reconnection matrix requires it: ring claims ride the gated intents
# stream and /exploration/targets is not relayed.
# (notes: exploit-off-for-comms-runs)
EXPLOIT="${EXPLOIT:-1}"
# The arm token. off (reconnect_enabled false) is handled here because the
# planner maps unknown modes to rendezvous; mtare_* tokens name whole arm
# stacks. Plain rendezvous and hybrid cannot run, so the default is mtare_.
# (notes: reconnect-mode-arm-tokens)
RECONNECT_MODE="${RECONNECT_MODE:-mtare_hybrid}"
case "$RECONNECT_MODE" in
  rendezvous|pursuit|hybrid|off) ;;
  mtare_off|mtare_pursuit|mtare_rendezvous|mtare_hybrid) ;;
  mtare_pursuit_mdp|mtare_hybrid_mdp) ;;
  *) echo "FATAL: RECONNECT_MODE='$RECONNECT_MODE' is not one of \
rendezvous|pursuit|hybrid|off|mtare_off|mtare_pursuit|mtare_rendezvous|\
mtare_hybrid|mtare_pursuit_mdp|mtare_hybrid_mdp. The planner would silently \
fall back to rendezvous and the run would be mislabelled." >&2; exit 2 ;;
esac
# The token fixes the whole stack. Set here, before the CELL_WORLD / TEAM_WORLD
# / GLOBAL_ALLOC / RECONNECT_GATE blocks read the environment; an explicit
# variable that contradicts it is refused. (notes: arm-token-pins-stack)
_arm_stack=""
case "$RECONNECT_MODE" in
  mtare_off)
    _arm_stack="CELL_WORLD:1 TEAM_WORLD:1 GLOBAL_ALLOC:1 RECONNECT_GATE:silence RENDEZVOUS_SCHEDULE:0 PURSUIT_PREDICTOR:trail" ;;
  mtare_pursuit)
    _arm_stack="CELL_WORLD:1 TEAM_WORLD:1 GLOBAL_ALLOC:1 RECONNECT_GATE:info RENDEZVOUS_SCHEDULE:0 PURSUIT_PREDICTOR:trail" ;;
  mtare_rendezvous)
    _arm_stack="CELL_WORLD:1 TEAM_WORLD:1 GLOBAL_ALLOC:1 RECONNECT_GATE:info RENDEZVOUS_SCHEDULE:1 PURSUIT_PREDICTOR:trail" ;;
  mtare_hybrid)
    _arm_stack="CELL_WORLD:1 TEAM_WORLD:1 GLOBAL_ALLOC:1 RECONNECT_GATE:info RENDEZVOUS_SCHEDULE:1 PURSUIT_PREDICTOR:trail" ;;
  # The predictor pair: byte-identical to the two chasing tokens above except
  # for the last field. Written out in full rather than derived from them so
  # that a future edit to one stack cannot silently desynchronise the other --
  # the diff between a treated arm and its control has to be readable here.
  mtare_pursuit_mdp)
    _arm_stack="CELL_WORLD:1 TEAM_WORLD:1 GLOBAL_ALLOC:1 RECONNECT_GATE:info RENDEZVOUS_SCHEDULE:0 PURSUIT_PREDICTOR:mdp" ;;
  mtare_hybrid_mdp)
    _arm_stack="CELL_WORLD:1 TEAM_WORLD:1 GLOBAL_ALLOC:1 RECONNECT_GATE:info RENDEZVOUS_SCHEDULE:1 PURSUIT_PREDICTOR:mdp" ;;
esac
if [ -n "$_arm_stack" ]; then
  for _kv in $_arm_stack; do
    _k="${_kv%%:*}"; _want="${_kv#*:}"
    eval "_got=\${$_k-}"
    if [ -n "$_got" ] && [ "$_got" != "$_want" ]; then
      echo "FATAL: RECONNECT_MODE=$RECONNECT_MODE implies $_k=$_want, but $_k='$_got'" >&2
      echo "       was set explicitly. The arm token names the whole stack; a" >&2
      echo "       cell that contradicts one part of it would still be recorded," >&2
      echo "       named and analysed as $RECONNECT_MODE." >&2
      exit 2
    fi
    eval "$_k=\$_want"
  done
  unset _kv _k _want _got
fi
unset _arm_stack

# MinPos claim radius override (m); orthogonal to the arm, so declared after
# _arm_stack. Empty passes nothing (yaml pins 10.0). Never set 10 to mean
# default: any value marks a treated cell. Also bounds peer claims.
# (notes: coord-claim-radius-override)
COORD_CLAIM_R="${COORD_CLAIM_R:-}"
if [ -n "$COORD_CLAIM_R" ]; then
  case "$COORD_CLAIM_R" in
    ''|*[!0-9.]*|*.*.*|.|'.'*[!0-9]*)
      echo "FATAL: COORD_CLAIM_R='$COORD_CLAIM_R' is not a plain decimal number." >&2
      echo "       It is the MinPos claim disc in metres and also the clamp on" >&2
      echo "       every peer-advertised radius, so a bad value changes the" >&2
      echo "       experiment silently instead of failing." >&2
      exit 2 ;;
  esac
  # Reject 0 and anything rounding to it: the node reads <= 0 as auto
  # (fov_max_range), not as no override. Unset COORD_CLAIM_R for the yaml
  # default. (notes: coord-claim-radius-zero-is-auto)
  if [ "$(awk -v v="$COORD_CLAIM_R" 'BEGIN{print (v+0 > 0.0) ? 1 : 0}')" != "1" ]; then
    echo "FATAL: COORD_CLAIM_R='$COORD_CLAIM_R' must be > 0. The node reads a" >&2
    echo "       non-positive claim radius as AUTO and would resolve it back to" >&2
    echo "       fov_max_range, so the cell would be named for a radius it did" >&2
    echo "       not run. Unset COORD_CLAIM_R to request the yaml default." >&2
    exit 2
  fi
fi
# Terminal barrier give-up (s). The planner default 0 waits forever, leaving
# time-to-reconnect undefined when a chase is declined; a finite cap makes the
# ending observable. 0 restores the field default. (notes: barrier-max-wait-cap)
RDV_MAX_WAIT="$(flt "${RDV_MAX_WAIT:-600}")"
# The pre-arm confirmation window (planner param reconnect_confirm_sec). The
# claim table can lag intents that were already delivered, so a manoeuvre armed
# on one read of it may be released by the next tick; 8 of 24 recorded firings
# ended within 5 s having moved under a metre. 0 restores arm-on-first-read.
RECONNECT_CONFIRM="$(flt "${RECONNECT_CONFIRM:-3.0}")"
# --- Mid-run trigger + pursuit-gate family (2026-08-17 redesign) -------------
# MIDRUN_SILENCE: seconds of continuous peer absence before a robot interrupts
# exploration to run its arm's reconnect manoeuvre; 0 = terminal-only. Every
# knob in this family is echoed into the manifest. (notes: midrun-silence-clock)
MIDRUN_SILENCE="$(flt "${MIDRUN_SILENCE:-90}")"
# Barrier give-up (s) after a failed mid-run manoeuvre; terminal barriers keep
# RDV_MAX_WAIT. Short on purpose (node default 240): a parked robot can only be
# found. Keep it above RECONNECT_RELEASE_CONFIRM.
# (notes: midrun-barrier-give-up)
MIDRUN_MAX_WAIT="$(flt "${MIDRUN_MAX_WAIT:-30}")"
MIDRUN_MAX_ATTEMPTS="${MIDRUN_MAX_ATTEMPTS:-6}"
# --- Post-latch coast (2026-08-27) -----------------------------------------
# DONE_SEEK=1: a robot that latches coverage mid-chase keeps its current nav
# goal to finish the drive and deliver its map. Metric-neutral; default 0.
# Unreachable under MISSION_RETURN=1. (notes: done-seek-post-latch-coast)
DONE_SEEK="${DONE_SEEK:-0}"
if [ "$DONE_SEEK" = "1" ]; then DONE_SEEK_ARG="true"; else DONE_SEEK_ARG="false"; fi
# Cap on a single coast. 0 disables the cap (the no-progress exit still holds).
DONE_SEEK_MAX="$(flt "${DONE_SEEK_MAX:-600}")"
# --- Mission return (2026-08-27) --------------------------------------------
# MISSION_RETURN=1: at any terminal exploration ending the robot drives back to
# its start pose, so DONE means home or a bounded give-up. Changes the
# exploration-finish endpoint; default 0. (notes: mission-return-home)
MISSION_RETURN="${MISSION_RETURN:-0}"
if [ "$MISSION_RETURN" = "1" ]; then MISSION_RETURN_ARG="true"; else MISSION_RETURN_ARG="false"; fi
# Arrival tolerance. Its own knob (not RECONNECT_ARRIVE_TOL) because home
# arrival is position semantics while the manoeuvre tolerance is sized for
# connectivity — at its historical 4.0 it accepted the partner's home 3 m
# away as an arrival, and nothing ties the two sizes together.
MISSION_HOME_TOL="$(flt "${MISSION_HOME_TOL:-1.0}")"
# Overall cap on the homing leg. The field guarantee that a mission-return run
# still ends: on expiry the robot parks where it is and the run ends with
# mission_complete result=timeout. 0 disables the cap.
MISSION_RETURN_MAX="$(flt "${MISSION_RETURN_MAX:-600}")"
# --- Information gate on the mid-run trigger (2026-08-19) -------------------
# Info gate: fire once the pair has gathered RECONNECT_MIN_SHARE_VOX voxels the
# other lacks (target over gathering rate, clamped to the min/max silence). 0
# (default) keeps the fixed clock. MIDRUN_MAX_SILENCE must stay <=
# MIDRUN_SILENCE. (notes: midrun-info-gate-sizing)
RECONNECT_MIN_SHARE_VOX="$(flt "${RECONNECT_MIN_SHARE_VOX:-0}")"
MIDRUN_MIN_SILENCE="$(flt "${MIDRUN_MIN_SILENCE:-60}")"
MIDRUN_MAX_SILENCE="$(flt "${MIDRUN_MAX_SILENCE:-$MIDRUN_SILENCE}")"
# Release flicker guard (s): the team must read complete this long before a
# manoeuvre releases; 0 = first read. Must exceed coord_claim_ttl_sec (5.0),
# since one packet holds a peer live for the whole TTL.
# (notes: release-confirm-vs-claim-ttl)
RECONNECT_RELEASE_CONFIRM="$(flt "${RECONNECT_RELEASE_CONFIRM:-6}")"
# Manoeuvre arrival tolerance (m) and per-leg drive ceiling (s). At 1.5 m a pair
# lands <= 3 m apart; an obstructed exact point falls to the no-progress window,
# which hands the leg to RETURN_SYNC. (notes: reconnect-arrive-tolerance)
RECONNECT_ARRIVE_TOL="$(flt "${RECONNECT_ARRIVE_TOL:-1.5}")"
RECONNECT_NAV_MAX="$(flt "${RECONNECT_NAV_MAX:-600}")"
# Pursuit staleness gate (s), wide enough to clear this world's outage tail.
# PURSUIT_GOAL_STALE bounds it: past that age the chase drives to the peer's
# last contact pose, not its declared goal. (notes: pursuit-staleness-gate)
PURSUIT_STALENESS="$(flt "${PURSUIT_STALENESS:-900}")"
# INVARIANT: chase budget <= waiter patience. A teammate at the barrier treats
# this as the worst case it may assume about its pursuer, so a budget above
# RDV_MAX_WAIT (600) would let a chase outlive the wait that justified it.
PURSUIT_BUDGET_MAX="$(flt "${PURSUIT_BUDGET_MAX:-600}")"
PURSUIT_GOAL_STALE="$(flt "${PURSUIT_GOAL_STALE:-180}")"
# Pure pursuit's fallback: keep exploring rather than park. Parking is a fixed
# point (two held robots cannot reconnect — p7modes: 5 of 6 holds never did,
# and one mutual hold cost a mission). Bounded, then it reverts to the hold so
# the terminal barrier still guarantees an ending.
PURSUIT_EXPLORE_FALLBACK="${PURSUIT_EXPLORE_FALLBACK:-1}"
PURSUIT_EXPLORE_FALLBACK_ARG=$([ "$PURSUIT_EXPLORE_FALLBACK" = "1" ] \
  && echo true || echo false)
PURSUIT_EXPLORE_MAX="${PURSUIT_EXPLORE_MAX:-6}"
# Terminal-hold escalation: on barrier expiry drive once to the last-connected
# anchor and wait HOLD_ESCALATE_WAIT more before giving up (breaks the
# mutual-hold deadlock that cost the p7modes pursuit_seed2 mission).
HOLD_ESCALATE="${HOLD_ESCALATE:-1}"
HOLD_ESCALATE_ARG=$([ "$HOLD_ESCALATE" = "1" ] && echo true || echo false)
HOLD_ESCALATE_WAIT="$(flt "${HOLD_ESCALATE_WAIT:-300}")"
# ---------------------------------------------------------------------------
# The rendezvous treatment knobs, passed explicitly even at their defaults so
# every manifest records them. RDV_DEPART_DELAY is inert, still passed so the
# param rows keep their schema. (notes: rdv-countdown-knobs)
# RDV_MAX_LATE: the most lateness a robot may accept when picking its lattice
# rung at arming, clamped at zero. RDV_INTERVAL minus this bounds hybrid's chase
# window; 0 = on time or the next rung. (notes: rdv-max-late)
# RDV_INTERVAL: spacing of the meeting timetable, the shortest gap between legal
# meeting instants and the floor on the derived interval. It moves the lattice
# only, not the proposal period. (notes: rdv-interval-lattice)
#   RDV_SETTLE        seconds to hold at the cell AFTER the team is whole again,
#                     so the map merge completes before anyone leaves. Not a
#                     safety margin on the barrier — a separate, later hold.
#   RDV_APPT_WAIT     cap on the barrier wait while keeping an appointment.
#                     0 = WAIT FOREVER, which is the directive ("be there until
#                     all robots are connected"), and is the default. A positive
#                     value turns the rendezvous arm into something else; it
#                     exists so that can be MEASURED, not so it can be tuned in.
# RDV_LATCHED_HOLD: cap on a finished robot's wait at the agreed cell, clocked
# from max(arrival, t_meet). Must cover one rolled rung: RDV_INTERVAL +
# RDV_MAX_LATE + 60 s. Re-derive if either moves; nothing checks it.
# (notes: rdv-latched-hold)
# START_HOLD: pre-mission hold (s) in every arm, so the team agrees its
# rendezvous place while co-located; 0 disables. It does not contain the
# provisional-to-final rendezvous upgrade. (notes: start-hold-pre-mission)
RDV_DEPART_DELAY="$(flt "${RDV_DEPART_DELAY:-100}")"
RDV_MAX_LATE="$(flt "${RDV_MAX_LATE:-60}")"
RDV_INTERVAL="$(flt "${RDV_INTERVAL:-300}")"
RDV_SETTLE="$(flt "${RDV_SETTLE:-30}")"
RDV_APPT_WAIT="$(flt "${RDV_APPT_WAIT:-0}")"
RDV_LATCHED_HOLD="$(flt "${RDV_LATCHED_HOLD:-420}")"
START_HOLD="$(flt "${START_HOLD:-60}")"
# COMMS=1 puts the radio emulator between robots and rewires both the peer map
# stream (scovox_bin) and exploration/intents; gating only one measures nothing.
# COMMS=0 (default): direct topics, perfect comms.
# (notes: comms-emulator-rewiring)
COMMS="${COMMS:-0}"
# --- Coarse cell world (M-TARE evolution, P1) -----------------------------
# CELL_WORLD=1: dices the ROI into CELL_SIZE_M cells with status from this
# robot's own map, logged as cell_census every CELL_CENSUS_S and drawn in RViz.
# Off passes no parameter (the equivalence gate needs compiled defaults).
# (notes: cell-world-layer)
CELL_WORLD="${CELL_WORLD:-0}"
# Validated: downstream tests compare against 1, so a typo such as true would
# silently read as off. (notes: cell-world-flag-validation)
case "$CELL_WORLD" in
  0|1) ;;
  *) echo "FATAL: CELL_WORLD='$CELL_WORLD' is not 0 or 1." >&2; exit 2 ;;
esac
CELL_SIZE_M="$(flt "${CELL_SIZE_M:-10.0}")"
CELL_CENSUS_S="$(flt "${CELL_CENSUS_S:-5.0}")"
# World-calibrated cell status thresholds: this map never saturates, so the
# library defaults promote nothing. Re-derive from the cell_unknown and
# cell_frontier_frac fields of cell_census rather than nudging.
# (notes: cell-status-thresholds)
CELL_COVERED_U="$(flt "${CELL_COVERED_U:-0.55}")"
CELL_EXPLORING_U="$(flt "${CELL_EXPLORING_U:-0.62}")"
CELL_FRONTIER_FRAC="$(flt "${CELL_FRONTIER_FRAC:-0.95}")"
# --- TeamWorld exchange (M-TARE evolution, P2) ----------------------------
# TEAM_WORLD=1: robots exchange their cell census at TEAM_WORLD_HZ and merge
# what they receive; per-peer comms status comes from the handshake. Off passes
# no parameter. Requires CELL_WORLD=1. (notes: team-world-exchange)
TEAM_WORLD="${TEAM_WORLD:-0}"
# Validated for the reason given in the CELL_WORLD block: a typo here reads as
# OFF and is only ever visible after the fact.
case "$TEAM_WORLD" in
  0|1) ;;
  *) echo "FATAL: TEAM_WORLD='$TEAM_WORLD' is not 0 or 1." >&2; exit 2 ;;
esac
# 1 Hz: the rate the message was sized for (a few kB of full state per publish
# at <=400 cells), and comfortably inside the default 5 s comms TTL so a single
# dropped message is not read as a dropout.
TEAM_WORLD_HZ="$(flt "${TEAM_WORLD_HZ:-1.0}")"
# TEAM_WORLD_HZ also switches the exchange: at <= 0 the node builds no publisher
# or timer, so refuse it. Also refuse empty (flt's output for a non-finite
# value) and exponent notation. (notes: team-world-hz-is-switch)
_hz_ok=1
case "$TEAM_WORLD_HZ" in
  ''|*[!0-9.]*|*.*.*) _hz_ok=0 ;;
esac
[ "$_hz_ok" = 1 ] && { awk -v h="$TEAM_WORLD_HZ" 'BEGIN{exit !(h+0 > 0)}' || _hz_ok=0; }
if [ "$_hz_ok" != 1 ]; then
  echo "FATAL: TEAM_WORLD_HZ='${TEAM_WORLD_HZ:-<empty: rejected by flt>}' is not a" >&2
  echo "       positive decimal number. It is the exchange's on/off switch as" >&2
  echo "       well as its rate: at <=0 the node builds no publisher and no" >&2
  echo "       timer, and the run would record team_world=1 having exchanged" >&2
  echo "       nothing. Use 1.0 unless you mean something else by it." >&2
  exit 2
fi
unset _hz_ok
if [ "$TEAM_WORLD" = "1" ] && [ "$CELL_WORLD" != "1" ]; then
  echo "TEAM_WORLD=1 requires CELL_WORLD=1 (the TeamWorld message is the cell" >&2
  echo "census, so there would be nothing to send). Set CELL_WORLD=1." >&2
  exit 2
fi
# --- Team-separation discount (separation.hpp) ------------------------------
# Separation discount: candidates near a teammate's last known position are
# discounted on a linear ramp reaching 1 at SEPARATION_RADIUS_M. Orthogonal to
# the arm, so declared after _arm_stack. Weight 0 (default) = off.
# (notes: separation-discount)
SEPARATION_WEIGHT="${SEPARATION_WEIGHT:-0}"
SEPARATION_RADIUS_M="${SEPARATION_RADIUS_M:-20}"
SEPARATION_MAX_AGE_SEC="${SEPARATION_MAX_AGE_SEC:-10}"
# Validated here too: the node answers a bad value by disabling the term and
# running on, which would file a control run under a treated arm's name.
# (notes: separation-harness-validation)
for _sepkv in "SEPARATION_WEIGHT:$SEPARATION_WEIGHT" \
              "SEPARATION_RADIUS_M:$SEPARATION_RADIUS_M" \
              "SEPARATION_MAX_AGE_SEC:$SEPARATION_MAX_AGE_SEC"; do
  _sepk="${_sepkv%%:*}"; _sepv="${_sepkv#*:}"
  case "$_sepv" in
    ''|*[!0-9.]*|*.*.*)
      echo "FATAL: $_sepk='$_sepv' is not a plain decimal number." >&2
      exit 2 ;;
  esac
  # Second case: the first accepts a bare ".", which awk reads as 0 and which
  # would pass every range check below. "1." and ".5" stay legal.
  # (notes: separation-bare-dot)
  case "$_sepv" in
    *[0-9]*) : ;;
    *)
      echo "FATAL: $_sepk='$_sepv' has no digits in it. awk would read it as 0," >&2
      echo "       which passes every range check below and silently runs the" >&2
      echo "       term at zero weight under whatever arm name this cell has." >&2
      exit 2 ;;
  esac
done
unset _sepkv _sepk _sepv
# Range matches the node's refusals: above 1 the utility goes negative, below 0
# it attracts. Negatives already fail the pattern above, so this catches > 1.
# (notes: separation-weight-range)
if [ "$(awk -v v="$SEPARATION_WEIGHT" 'BEGIN{print (v+0 >= 0.0 && v+0 <= 1.0) ? 1 : 0}')" != "1" ]; then
  echo "FATAL: SEPARATION_WEIGHT='$SEPARATION_WEIGHT' is outside [0, 1]." >&2
  echo "       It is the discount applied on top of a teammate: 0 is off, 1" >&2
  echo "       scores such a candidate at exactly zero." >&2
  exit 2
fi
# Radius and max age are validated even at weight 0: every arm measures
# sep_peer_dist_m and sep_eligible_peers with them.
# (notes: separation-radius-age-always)
for _sepkv in "SEPARATION_RADIUS_M:$SEPARATION_RADIUS_M" \
              "SEPARATION_MAX_AGE_SEC:$SEPARATION_MAX_AGE_SEC"; do
  _sepk="${_sepkv%%:*}"; _sepv="${_sepkv#*:}"
  if [ "$(awk -v v="$_sepv" 'BEGIN{print (v+0 > 0.0) ? 1 : 0}')" != "1" ]; then
    echo "FATAL: $_sepk='$_sepv' must be > 0. The node refuses a non-positive" >&2
    echo "       value by disabling the term, so the cell would run the control" >&2
    echo "       under the treatment's name." >&2
    exit 2
  fi
done
unset _sepkv _sepk _sepv
# The term reads teammate positions out of the TeamModel, which only exists
# when the exchange is on. The node itself refuses this pairing fatally at
# startup; refusing here as well means the campaign loses a cell's setup time
# rather than a cell's worth of wall clock.
if [ "$(awk -v v="$SEPARATION_WEIGHT" 'BEGIN{print (v+0 > 0.0) ? 1 : 0}')" = "1" ] \
   && [ "$TEAM_WORLD" != "1" ]; then
  echo "FATAL: SEPARATION_WEIGHT>0 requires TEAM_WORLD=1 — with no exchange" >&2
  echo "       the planner never learns a teammate position, so the term would" >&2
  echo "       be on in the manifest and inert in the binary." >&2
  exit 2
fi
# --- Global allocator (M-TARE evolution, P3) ------------------------------
# GLOBAL_ALLOC=1: every robot solves the same assignment over the shared cell
# world and takes its own tour; its focus cell re-ranks that tick's candidates.
# Off passes no parameter. Requires TEAM_WORLD=1. (notes: global-allocator)
GLOBAL_ALLOC="${GLOBAL_ALLOC:-0}"
case "$GLOBAL_ALLOC" in
  0|1) ;;
  *) echo "FATAL: GLOBAL_ALLOC='$GLOBAL_ALLOC' is not 0 or 1." >&2; exit 2 ;;
esac
if [ "$GLOBAL_ALLOC" = "1" ] && [ "$TEAM_WORLD" != "1" ]; then
  echo "GLOBAL_ALLOC=1 requires TEAM_WORLD=1 (with no exchange every robot" >&2
  echo "would allocate the whole map to itself and call that agreement)." >&2
  exit 2
fi
# Allocator peer-pose TTL (s, mission time): older poses drop out and free their
# cells; 0 = unbounded and is a legal explicit value, empty passes nothing. Set
# per cell from the _ttl<N> suffix; must follow GLOBAL_ALLOC.
# (notes: alloc-peer-pos-ttl)
ALLOC_POS_TTL="${ALLOC_POS_TTL:-}"
if [ -n "$ALLOC_POS_TTL" ]; then
  case "$ALLOC_POS_TTL" in
    ''|*[!0-9.]*|*.*.*)
      echo "FATAL: ALLOC_POS_TTL='$ALLOC_POS_TTL' is not a plain decimal number." >&2
      echo "       It is the age in seconds past which a latched peer position" >&2
      echo "       stops holding cells in the allocator." >&2
      exit 2 ;;
  esac
  # Second case for the same reason the separation block has one: the pattern
  # above is an OR, and a bare "." satisfies every branch of it. awk then reads
  # "." as 0, which is this knob's OFF value — so "_ttl." would produce a cell
  # named for a TTL that ran unbounded.
  case "$ALLOC_POS_TTL" in
    *[0-9]*) : ;;
    *) echo "FATAL: ALLOC_POS_TTL='$ALLOC_POS_TTL' has no digits in it; awk" >&2
       echo "       would read it as 0, which is this knob's OFF value." >&2
       exit 2 ;;
  esac
  # Refused, not warned: alloc_peer_pos_max_age_sec is read only by the
  # allocator solve in doPlan, so without GLOBAL_ALLOC the treatment would be
  # recorded but inert. (notes: alloc-ttl-needs-allocator)
  if [ "$(awk -v v="$ALLOC_POS_TTL" 'BEGIN{print (v+0 > 0.0) ? 1 : 0}')" = "1" ] \
     && [ "$GLOBAL_ALLOC" != "1" ]; then
    echo "FATAL: ALLOC_POS_TTL>0 requires GLOBAL_ALLOC=1. The TTL is read only" >&2
    echo "       at the allocator solve, so with the allocator off it would be" >&2
    echo "       on in the manifest and inert in the binary." >&2
    exit 2
  fi
fi
# --- Utility-gated reconnection (M-TARE evolution, P4) --------------------
# RECONNECT_GATE=info: when the mid-run clock expires, dispatch only if the
# missing peer lacks map we hold and the leg beats the no-comms plan. silence
# (default) passes no parameter. Requires TEAM_WORLD=1.
# (notes: reconnect-info-gate)
RECONNECT_GATE="${RECONNECT_GATE:-silence}"
case "$RECONNECT_GATE" in
  silence|info) ;;
  *) echo "FATAL: RECONNECT_GATE='$RECONNECT_GATE' is not silence or info." >&2
     exit 2 ;;
esac
if [ "$RECONNECT_GATE" = "info" ] && [ "$TEAM_WORLD" != "1" ]; then
  echo "RECONNECT_GATE=info requires TEAM_WORLD=1 (with no exchange no peer" >&2
  echo "ever enters a known_by mask, so the knowledge gate is vacuously true" >&2
  echo "for the entire run)." >&2
  exit 2
fi
# The gate can only ever SUPPRESS a dispatch the mid-run silence clock already
# allowed, so with the manoeuvre disabled it never evaluates at all. That is a
# legitimate control configuration, but asking for `info` in it is almost
# certainly a mistake in whatever set the arm.
if [ "$RECONNECT_GATE" = "info" ] && [ "$RECONNECT_MODE" = "off" ]; then
  echo "FATAL: RECONNECT_GATE=info with RECONNECT_MODE=off. There is no" >&2
  echo "       mid-run reconnection to gate, so the cell would be labelled as" >&2
  echo "       carrying the P4 treatment and would not carry it." >&2
  exit 2
fi
# --- Scheduled rendezvous (M-TARE evolution, P5) --------------------------
# RENDEZVOUS_SCHEDULE=1: on dispatch every robot derives the same (cell, t_meet)
# from the allocator's tours; with 0 there is no meeting destination. Off passes
# no parameter. Requires TEAM_WORLD=1. (notes: rendezvous-schedule)
RENDEZVOUS_SCHEDULE="${RENDEZVOUS_SCHEDULE:-0}"
case "$RENDEZVOUS_SCHEDULE" in
  0|1) ;;
  *) echo "FATAL: RENDEZVOUS_SCHEDULE='$RENDEZVOUS_SCHEDULE' is not 0 or 1." >&2
     exit 2 ;;
esac
if [ "$RENDEZVOUS_SCHEDULE" = "1" ] && [ "$TEAM_WORLD" != "1" ]; then
  echo "FATAL: RENDEZVOUS_SCHEDULE=1 requires TEAM_WORLD=1 (with no exchange" >&2
  echo "       each robot would schedule a meeting with a fleet of one, at a" >&2
  echo "       cell the peer has never heard of)." >&2
  exit 2
fi
# The appointment is a DESTINATION for a reconnect manoeuvre. With the
# manoeuvre disabled there is nothing to redirect, so the knob is inert and
# asking for it is a mistake in whatever set the arm — the same failure the
# RECONNECT_GATE=info / RECONNECT_MODE=off pairing above refuses.
if [ "$RENDEZVOUS_SCHEDULE" = "1" ] && [ "$RECONNECT_MODE" = "off" ]; then
  echo "FATAL: RENDEZVOUS_SCHEDULE=1 with RECONNECT_MODE=off. There is no" >&2
  echo "       reconnect manoeuvre to schedule, so the cell would be labelled" >&2
  echo "       as carrying the P5 treatment and would not carry it." >&2
  exit 2
fi
# Pursuit arms are the appointment-off cell and the planner never arms an
# appointment there, so the knob would be recorded but inert: refuse it.
# (notes: rendezvous-schedule-not-pursuit)
_rzv_pursuit=0
case "$RECONNECT_MODE" in
  pursuit|mtare_pursuit|mtare_pursuit_mdp) _rzv_pursuit=1 ;;
esac
if [ "$RENDEZVOUS_SCHEDULE" = "1" ] && [ "$_rzv_pursuit" = "1" ]; then
  echo "FATAL: RENDEZVOUS_SCHEDULE=1 with RECONNECT_MODE=$RECONNECT_MODE." >&2
  echo "       The pursuit arm is the appointment-OFF cell of the 2x2 and the" >&2
  echo "       planner will not arm one there. The cell would record the P5" >&2
  echo "       treatment as enabled and would not carry it." >&2
  exit 2
fi
unset _rzv_pursuit
# The CONVERSE of the two checks above — a mode that is nothing without the
# schedule — lives further down, just past the arm-stamp guard, and the reason
# it is down there rather than here is in the comment at its own site.
# --- MDP interception (M-TARE evolution, P6) --------------------------------
# PURSUIT_PREDICTOR=mdp aims the chase at the peer's predicted position on its
# last received tour. Waypoint only: trigger, budget and terminators still use
# the trail. trail (default) passes no parameter. (notes: pursuit-predictor-mdp)
PURSUIT_PREDICTOR="${PURSUIT_PREDICTOR:-trail}"
case "$PURSUIT_PREDICTOR" in
  trail|mdp) ;;
  *) echo "FATAL: PURSUIT_PREDICTOR='$PURSUIT_PREDICTOR' is not trail or mdp." >&2
     exit 2 ;;
esac
# mdp needs GLOBAL_ALLOC: the prediction expands the peer's last received tour,
# which only the allocator produces. The node also refuses this at construction;
# refusing here saves the bring-up. (notes: mdp-needs-allocator)
if [ "$PURSUIT_PREDICTOR" = "mdp" ] && [ "$GLOBAL_ALLOC" != "1" ]; then
  echo "FATAL: PURSUIT_PREDICTOR=mdp requires GLOBAL_ALLOC=1 (the prediction" >&2
  echo "       expands the peer's TOUR, and without the allocator there is no" >&2
  echo "       tour to expand, so every prediction degrades to the trail and" >&2
  echo "       the cell is mislabelled as treated)." >&2
  exit 2
fi
# And a chase to aim. RECONNECT_MODE=off has no manoeuvre; the rendezvous-only
# cell of the 2x2 has an appointment but no chase. In both the knob is inert
# and asking for it is the same name/stamp mistake the RENDEZVOUS_SCHEDULE
# pairings above refuse.
_pp_nochase=0
case "$RECONNECT_MODE" in
  off|mtare_off|rendezvous|mtare_rendezvous) _pp_nochase=1 ;;
esac
if [ "$PURSUIT_PREDICTOR" = "mdp" ] && [ "$_pp_nochase" = "1" ]; then
  echo "FATAL: PURSUIT_PREDICTOR=mdp with RECONNECT_MODE=$RECONNECT_MODE." >&2
  echo "       That arm never arms a chase, so there is no waypoint to aim." >&2
  echo "       The cell would record the P6 treatment as enabled and would not" >&2
  echo "       carry it." >&2
  exit 2
fi
unset _pp_nochase
# Arm name and node stamp must agree. The node prefixes mtare_ when
# GLOBAL_ALLOC, RECONNECT_GATE=info, RENDEZVOUS_SCHEDULE or mdp is on, and
# appends _mdp for mdp. Keep in lockstep with explo_planner_node.cpp.
# (notes: arm-name-stamp-agreement)
_mtare_stamped=0
if [ "$GLOBAL_ALLOC" = "1" ] || [ "$RECONNECT_GATE" = "info" ] \
   || [ "$RENDEZVOUS_SCHEDULE" = "1" ] || [ "$PURSUIT_PREDICTOR" = "mdp" ]; then
  _mtare_stamped=1
fi
# Compares the full name, not just the prefix: a prefix-only check would pass
# mtare_hybrid with the predictor on. (notes: arm-stamp-full-string)
_arm_core="${RECONNECT_MODE#mtare_}"
_arm_core="${_arm_core%_mdp}"
_arm_expect="$_arm_core"
if [ "$_mtare_stamped" = "1" ]; then _arm_expect="mtare_$_arm_expect"; fi
if [ "$PURSUIT_PREDICTOR" = "mdp" ]; then _arm_expect="${_arm_expect}_mdp"; fi
if [ "$_arm_expect" != "$RECONNECT_MODE" ]; then
  echo "FATAL: the arm name and the arm the node will stamp disagree." >&2
  echo "       RECONNECT_MODE=$RECONNECT_MODE names the arm, but" >&2
  echo "       GLOBAL_ALLOC=$GLOBAL_ALLOC RECONNECT_GATE=$RECONNECT_GATE" >&2
  echo "       RENDEZVOUS_SCHEDULE=$RENDEZVOUS_SCHEDULE" >&2
  echo "       PURSUIT_PREDICTOR=$PURSUIT_PREDICTOR means" >&2
  echo "       the node will stamp arm=$_arm_expect." >&2
  echo "       Every directory, index and analysis keys off the name; only" >&2
  echo "       run_start carries the stamp. Use RECONNECT_MODE=$_arm_expect to" >&2
  echo "       request that treatment, and do not set these knobs by hand." >&2
  exit 2
fi
unset _mtare_stamped _arm_core _arm_expect
# Refuses a mode that is nothing without the schedule: rendezvous would run as
# off and hybrid as pursuit. Must stay after the arm-stamp guard, whose report
# wins the tie (the guard calibration asserts it).
# (notes: plain-token-needs-schedule)
_rzv_needed=0
case "$RECONNECT_MODE" in
  rendezvous|hybrid|mtare_rendezvous|mtare_hybrid|mtare_hybrid_mdp) _rzv_needed=1 ;;
esac
if [ "$_rzv_needed" = "1" ] && [ "$RENDEZVOUS_SCHEDULE" != "1" ]; then
  echo "FATAL: RECONNECT_MODE=$RECONNECT_MODE with RENDEZVOUS_SCHEDULE=0." >&2
  echo "       This arm IS the agreed meeting (place AND time) and there is no" >&2
  echo "       agreement to make without the scheduler. The cell would run to" >&2
  echo "       completion looking healthy while behaving as the" >&2
  case "$RECONNECT_MODE" in
    *rendezvous) echo "       'off' arm." >&2 ;;
    *)           echo "       'pursuit' arm." >&2 ;;
  esac
  # NOT "or set RENDEZVOUS_SCHEDULE=1" — that was the advice until 2026-09-16
  # and under a plain token it produces a SECOND exit 2, from the arm-stamp
  # guard above, because the scheduler is one of the four knobs that make the
  # node stamp `mtare_`. The mtare_* token is the only fix.
  case "$RECONNECT_MODE" in
    mtare_*) echo "       The arm stack pins this; something overrode it." >&2 ;;
    *)       echo "       Use RECONNECT_MODE=mtare_$RECONNECT_MODE — the plain" >&2
             echo "       token has no runnable configuration (see the" >&2
             echo "       RECONNECT_MODE default block)." >&2 ;;
  esac
  exit 2
fi
unset _rzv_needed
# Link fading is a pure function of (seed, tick), so this alone selects the run's
# link realisation. Paired-seed designs vary it while holding everything else
# fixed; it is inert with COMMS=0.
SEED="${SEED:-42}"
# Radio TX power (dBm). Default matches comms_sim_params.yaml; exposed so it
# lands in the manifest and can be swept without editing the installed yaml.
# (notes: radio-tx-power)
TX_POWER="$(flt "${TX_POWER:-30.0}")"
# Per-trunk link attenuation (dB), the geometric severity dial: it scales with
# trunks on the link, selecting occlusion outages over distance ones. Default
# matches comms_sim_params.yaml; recorded in the manifest.
# (notes: radio-tree-attenuation)
TREE_ATTEN="$(flt "${TREE_ATTEN:-70.0}")"
# Hard radio horizon (m) for tree-free links; free-space loss alone never drops
# a 30 dBm link inside the ROI. Recorded in the manifest.
# (notes: radio-max-range)
MAX_RANGE="$(flt "${MAX_RANGE:-30.0}")"
# 1 = this arm expects the link to drop, 0 = the control. Affects only the
# outage gate's verdict, never the radio; independent of TX_POWER so a control
# whose link did drop is still reported. (notes: expect-outage-flag)
EXPECT_OUTAGE="${EXPECT_OUTAGE:-1}"
# --- Link-state gate for the mid-run reconnect trigger (§30.11, §30.24) ------
# LINK_GATE=1 (default) vetoes mid-run chases on the emulator's link state, not
# peer record age; the low MIDRUN_SILENCE relies on it. Effective only with
# COMMS=1 (else the topics are empty). Same for every arm.
# (notes: link-gate-default-on)
LINK_GATE="${LINK_GATE:-1}"
# Validated: any value other than 1 disables the veto while looking like a
# request for it. (notes: link-gate-validation)
case "$LINK_GATE" in
  0|1) ;;
  *) echo "FATAL: LINK_GATE='$LINK_GATE' is not 0 or 1. Any other value \
disables the link veto while looking like a request for it, and the manifest \
would record it as such." >&2; exit 2 ;;
esac
if [ "$LINK_GATE" = "1" ] && [ "$COMMS" = "1" ]; then
  LINK_GATE_TOPIC="${LINK_GATE_TOPIC:-/hmr_comms_sim/link_states}"
  LINK_GATE_INDEX="${LINK_GATE_INDEX:-/hmr_comms_sim/robot_index}"
else
  LINK_GATE_TOPIC=""
  LINK_GATE_INDEX=""
fi
# LINK_GATE_EFFECTIVE is what the planner gets, read back from LINK_GATE_TOPIC
# being set. Requested but not effective is unsafe with the low MIDRUN_SILENCE;
# warned at the manifest write, where log() exists. (notes: link-gate-effective)
if [ -n "$LINK_GATE_TOPIC" ]; then
  LINK_GATE_EFFECTIVE=1
else
  LINK_GATE_EFFECTIVE=0
fi
# Newest link sample older than this and the planner stands down to the legacy
# clock rather than acting on a stale belief. The emulator publishes at 5 Hz.
LINK_GATE_STALE="$(flt "${LINK_GATE_STALE:-3.0}")"
# Seconds the radio must be continuously down before the veto lets a mid-run
# chase through; separate from RECONNECT_CONFIRM. At 0 the veto is simply: never
# chase a peer whose radio is up right now. (notes: link-down-confirm-zero)
LINK_DOWN_CONFIRM="$(flt "${LINK_DOWN_CONFIRM:-0}")"
# Reliable-relay backlog cap (bytes). On overflow the emulator drops the oldest
# map delta for good, so size it for a full blackout (1 GiB, ~2.4x margin, up to
# 2 GiB RAM). Overflow stays a hard gate. (notes: relay-queue-cap)
RELAY_QUEUE_BYTES="${RELAY_QUEUE_BYTES:-1073741824}"
# Planner ROI half-extent (m), sim only, square about the world origin. The
# yaml's field-site ROI is largely empty here, so coverage could never
# terminate; 50 fits the ground plane and the planning map below.
# (notes: sim-roi-half-extent)
ROI_HALF="$(flt "${ROI_HALF:-50.0}")"
# Side (m) of the world-fixed global_planning_map the planner reads (dscovox
# copy, fused map). 3x ROI_HALF gives margin: the robot can leave the ROI and
# the flood starts at its cell. Not the 20 m rolling planning_map.
# (notes: global-planning-map-size)
PLAN_MAP_SIZE="$(flt "${PLAN_MAP_SIZE:-$(awk "BEGIN{print 3*$ROI_HALF}")}")"
PLAN_MAP_RES="$(flt "${PLAN_MAP_RES:-0.40}")"
# Reachability flood radius (m); must be set here: the yaml's auto cap, sized
# for polar candidates, rejects distant frontiers. 10x ROI_HALF means unbounded;
# never clamp it to the grid diagonal. 0 is the auto sentinel.
# (notes: cost-grid-flood-cap)
COST_CAP="$(flt "${COST_CAP:-$(awk "BEGIN{print 10*$ROI_HALF}")}")"
# Minimum range (m) to an EXPLORE candidate: without it the planner can
# oscillate between near goals while every health check passes. Must be
# identical across arms. (notes: min-goal-dist-oscillation)
MIN_GOAL_DIST="$(flt "${MIN_GOAL_DIST:-4.0}")"
# Exponent in U = info / (eps + cost)^GAMMA; 1.0 is the original SSMI form,
# which the node short-circuits. A double, so through flt(). A scenario
# correction: never pool runs across a change of it.
# (notes: utility-cost-exponent)
UTIL_GAMMA="$(flt "${UTIL_GAMMA:-0.5}")"
# Frontier search band as insets from the yaml ROI z band [-5.5, 4.0], giving z
# [0.2, 1.5] on flat ground, the slice the VLP-16 sweeps. Needed with
# MIN_GOAL_DIST to stop oscillation. (notes: frontier-z-band)
FRONTIER_Z_LO_OFF="$(flt "${FRONTIER_Z_LO_OFF:-5.7}")"
FRONTIER_Z_HI_OFF="$(flt "${FRONTIER_Z_HI_OFF:-2.5}")"
# EIG sensor-model overrides; unset passes nothing and shared_params.yaml rules.
# FOV_HFOV/FOV_VFOV are doubles (flt); the *_RAYS and CAND_N_YAW are integers
# and must not get a decimal point. CAND_N_YAW needs polar.
# (notes: eig-sensor-overrides)
FOV_HFOV="${FOV_HFOV:-}"
FOV_VFOV="${FOV_VFOV:-}"
FOV_H_RAYS="${FOV_H_RAYS:-}"
FOV_V_RAYS="${FOV_V_RAYS:-}"
CAND_N_YAW="${CAND_N_YAW:-}"
# Recently-visited goal suppression. Radius just above frontier_cluster_radius_m
# (5.0) so a reached goal's cluster is suppressed; the TTL outlasts a
# there-and-back hop and expires so re-frontiered areas can be revisited.
# (notes: visited-goal-suppression)
VISITED_RADIUS="$(flt "${VISITED_RADIUS:-6.0}")"
VISITED_TTL="$(flt "${VISITED_TTL:-180.0}")"
# Failed-goal blacklist TTL (s): must outlast one full nav budget
# (NAV_MAX_TIMEOUT) so a trap cannot expire while the robot fails elsewhere; 240
# = 180 + 60. FAILED_RETIRE blocks a site for the run after that many failures.
# (notes: failed-goal-blacklist)
FAILED_TTL="$(flt "${FAILED_TTL:-240.0}")"
FAILED_RETIRE="${FAILED_RETIRE:-3}"
# Radius (m) of the disc the failed-goal TTL suppresses; too small and the
# planner re-picks a cell beside the trap as a new goal. Recorded with the TTL.
# (notes: failed-goal-radius)
FAILED_RADIUS="$(flt "${FAILED_RADIUS:-2.0}")"
# Nav leg budget and no-progress abort, recorded because the timeout and
# no-progress censoring labels are defined in these units.
# (notes: nav-budget-recorded)
NAV_MIN_TIMEOUT="$(flt "${NAV_MIN_TIMEOUT:-30.0}")"
NAV_MAX_TIMEOUT="$(flt "${NAV_MAX_TIMEOUT:-180.0}")"
PROGRESS_WINDOW="$(flt "${PROGRESS_WINDOW:-15.0}")"
PROGRESS_MIN_DIST="$(flt "${PROGRESS_MIN_DIST:-0.2}")"
# Refuses a TTL below nav_max_timeout_sec + 30, the planner's own threshold (it
# only warns). Guard at +30, not at the 240 default, which would reject working
# configurations. (notes: failed-ttl-guard)
awk -v ttl="$FAILED_TTL" -v navmax="$NAV_MAX_TIMEOUT" 'BEGIN {
  if (ttl < navmax + 30.0) exit 1
}' || {
  echo "FATAL: failed_goal_ttl_sec=$FAILED_TTL is below nav_max_timeout_sec" >&2
  echo "       ($NAV_MAX_TIMEOUT) + 30. A blacklisted trap would expire while" >&2
  echo "       the robot is still burning ONE nav budget elsewhere, and be free" >&2
  echo "       to be re-picked the moment it returns -- the mr1_hybrid_seed18" >&2
  echo "       failure. Raise FAILED_TTL or lower NAV_MAX_TIMEOUT." >&2
  exit 2
}
# Bounds for the homing approach test, which replaces a gross-metres watchdog
# that a robot orbiting a local minimum satisfies forever.
# (notes: homing-approach-watchdog)
RETURN_APPROACH_WINDOW="$(flt "${RETURN_APPROACH_WINDOW:-40.0}")"
RETURN_APPROACH_MIN="$(flt "${RETURN_APPROACH_MIN:-1.0}")"
RETURN_ESCAPE_MAX="${RETURN_ESCAPE_MAX:-3}"
RETURN_ESCAPE_LEG="$(flt "${RETURN_ESCAPE_LEG:-30.0}")"
# scovox voxel edge (m). The launch default 0.10 grows the map until planner
# ingest silently falls behind. Under COMMS=1 it sets the radio's offered load,
# so calibrate link severity at the campaign's resolution.
# (notes: voxel-resolution)
VOXEL_RES="$(flt "${VOXEL_RES:-0.20}")"
# Coverage termination threshold (ROI unknown fraction), set above this world's
# measured floor so runs end naturally. Defines the primary endpoint: identical
# across arms; recorded in the manifest. (notes: done-unknown-threshold)
DONE_UNKNOWN="$(flt "${DONE_UNKNOWN:-0.64}")"
# latch (default): a robot is finished the first tick its own fused ROI unknown
# fraction reaches DONE_UNKNOWN, in any state, and never un-finishes; streak is
# the old rule. Part of the endpoint: same across arms.
# (notes: done-criterion-latch)
DONE_CRITERION="${DONE_CRITERION:-latch}"
case "$DONE_CRITERION" in
  latch|streak) ;;
  *) echo "DONE_CRITERION must be 'latch' or 'streak', got '$DONE_CRITERION'" >&2
     exit 2 ;;
esac
OUTDIR="${OUTDIR:-/tmp/explo_sim_$(date +%Y%m%d_%H%M%S)}"
# Own DDS domain, not 0: other ROS work on this box uses 0, and a second /clock
# there corrupts every use_sim_time node. Children inherit it; export the same
# value to inspect the run by hand. (notes: ros-domain-id)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# RViz mesh per robot from painted variants of the spawned model, assigned by
# roster position so any robot name works. Three variants, so beyond N=3 colours
# repeat (cosmetic). (notes: viz-mesh-palette)
VIZ_PALETTE=(COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_YELLOW
             COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_GREEN
             COSTAR_HUSKY_SENSOR_CONFIG_REDUCED)
declare -A VIZ_MODEL=()
_vi=0
for _r in $ROBOTS; do
  VIZ_MODEL[$_r]=${VIZ_PALETTE[$((_vi % ${#VIZ_PALETTE[@]}))]}
  _vi=$((_vi + 1))
done
unset _vi _r

mkdir -p "$OUTDIR"
log() { echo "[$(date +%H:%M:%S)] $*"; }
# COMMS=1 wiring is per link: peers_of prints every other roster robot, so
# callers building subscription lists get the whole set.
# (notes: peers-of-helper)
peers_of()  { local s=$1 p; for p in $ROBOTS; do [ "$p" = "$s" ] || echo "$p"; done; }
peers_csv() { peers_of "$1" | paste -sd, -; }
# A ROS string-array literal — ["a","b"] — for -p name:=... Empty input yields
# [], which is a valid empty array and the right answer for a lone robot; the
# roster check above makes that unreachable, but a helper that silently emitted
# [""] would be a subscription to the empty topic name.
peers_ros_array() {
  local self=$1 out="" p
  for p in $(peers_of "$self"); do
    [ -z "$out" ] || out="$out,"
    out="$out\"$2$p$3\""
  done
  echo "[$out]"
}

# --- environment: humble + ws overlay, miniconda stripped -------------------
# (miniconda on PATH shadows /usr/bin/python3 and breaks catkin_pkg/ament)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV || true
# Snap scrub, required when launched from a snap-hosted shell such as the VS
# Code snap terminal: its GTK/GDK paths make rviz2 load snap libraries and die
# at startup. Harmless in a plain shell. (notes: snap-env-scrub)
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_IM_MODULE \
      GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GSETTINGS_SCHEMA_DIR \
      LOCPATH GIO_MODULE_DIR XDG_DATA_HOME QT_IM_MODULE QT_PLUGIN_PATH \
      LD_LIBRARY_PATH || true
[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
[ -n "${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-}" ] && export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
for v in $(env | grep -oE '^SNAP[A-Z_]*'); do unset "$v"; done
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
PLANNER_SHARE=$(ros2 pkg prefix explo_planner)/share/explo_planner
TARGETS="${TARGETS:-$PLANNER_SHARE/config/targets_flatforest.yaml}"
[ -f "$TARGETS" ] || { echo "ERROR no such targets file: $TARGETS"; exit 2; }

# RViz layout + viz URDF are read from the SOURCE tree, not the install share:
# both are pure config, so editing them applies on the next run with no rebuild.
RVIZ_CFG="$HERE/../explo_planner/config/explo_sim_2robot_lidar.rviz"
URDF_IN="$HERE/../explo_planner/config/costar_husky_viz.urdf.in"
for f in "$RVIZ_CFG" "$URDF_IN" "$HERE/sim_tf_publisher.py" "$HERE/sim_target_markers.py" \
         "$HERE/comms_gates.py"; do
  [ -f "$f" ] || { echo "ERROR missing $f"; exit 2; }
done

# --- process bookkeeping (same discipline as the campaign driver) -----------
# Every child is started with setsid and killed by explicit PID/group. Never
# reintroduce `pkill -f`: it self-matches the harness/eval wrapper.
PIDS=()
start() { # start <name> <logfile> <cmd...>
  local name=$1 lf=$2; shift 2
  setsid "$@" > "$lf" 2>&1 &
  local pid=$!
  PIDS+=("$name:$pid")
  log "started $name pid=$pid  (log: $lf)"
}
alive() { kill -0 "$1" 2>/dev/null; }
sim_clock() {
  # Write through a temp file, NOT a pipe: a ros2 daemon spawned mid-call
  # inherits a command-substitution pipe's write end and never closes it,
  # blocking the parent read forever.
  local f="$OUTDIR/.clock.$$"
  timeout 8 ros2 topic echo /clock --once > "$f" 2>/dev/null </dev/null
  awk '/sec:/{print $2; exit}' "$f" 2>/dev/null
  rm -f "$f"
}
stack_procs() {
  # Feeds teardown's kill -KILL. With IGN_PARTITION set, keeps only processes
  # whose /proc/<pid>/environ has the same value (inherited, never in argv);
  # unset matches machine-wide, safe for one cell only.
  # (notes: teardown-stack-procs-partition-scope)
  local matched
  matched=$(ps -eo pid,cmd | grep -E "ign gazebo|explo_planner_node|target_scheduler_node|dscovox_node|scovox_node|scovox_mapping_node|dscovox_mapping_node|hmr_comms_sim_node|simple_nav|robot_state_publisher|sim_tf_publisher|sim_target_markers|rosbag2|parameter_bridge|ros_gz|rviz2" \
    | grep -v grep | grep -v claude | grep -v run_explo_sim_rviz)
  [ -z "${IGN_PARTITION:-}" ] && { printf '%s\n' "$matched"; return 0; }
  printf '%s\n' "$matched" | while read -r p rest; do
    [ -n "$p" ] || continue
    # -x so IGN_PARTITION=p1 cannot match IGN_PARTITION=p10; -F so a partition
    # name is never interpreted as a regex. A pid that exits mid-scan yields a
    # read error, swallowed here, and is omitted -- correct, it needs no kill.
    if tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null \
       | grep -qxF "IGN_PARTITION=${IGN_PARTITION}"; then
      printf '%s %s\n' "$p" "$rest"
    fi
  done
}
count_own() {
  # Counts processes matching the pattern that belong to this cell, by the same
  # IGN_PARTITION test as stack_procs, for the bring-up guards. Unset counts
  # machine-wide. (notes: count-own-bringup-guard)
  local pat=$1 n=0 p
  for p in $(ps -eo pid=,cmd= | grep "$pat" | awk '{print $1}'); do
    if [ -z "${IGN_PARTITION:-}" ]; then
      n=$((n+1))
    elif tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null \
         | grep -qxF "IGN_PARTITION=${IGN_PARTITION}"; then
      n=$((n+1))
    fi
  done
  printf '%s\n' "$n"
}
TORN=0
teardown() {
  [ "$TORN" = 1 ] && return 0
  TORN=1
  log "teardown: SIGINT to process groups in reverse order"
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    entry=${PIDS[$i]}; name=${entry%%:*}; pid=${entry##*:}
    if alive "$pid"; then
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
      for _ in $(seq 1 60); do alive "$pid" || break; sleep 1; done   # bag zstd
      alive "$pid" && { log "WARN $name ignored SIGINT, killing group"
                        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null; }
      log "stopped $name"
    fi
  done
  sleep 2
  LEFT=$(stack_procs || true)
  if [ -n "$LEFT" ]; then
    log "WARN leftover processes, force-killing by pid:"; echo "$LEFT"
    echo "$LEFT" | awk '{print $1}' | xargs -r kill -KILL 2>/dev/null
  fi
  # Final gate verdict. The watcher was killed above, so comms_gates.txt is the
  # only record of what it found; the outage gate can only be decided here.
  # (notes: teardown-final-gate-verdict)
  if [ "$COMMS" = "1" ] && [ -f "$OUTDIR/comms_gates.txt" ]; then
    # map_agreement.py is report-only: PASS or INFO, never FAIL or UNRUN (UNRUN
    # scores SUSPECT, which gate_g8 fails). A crash or a missing file is INFO.
    # Test presence, not -x. map_agree matches its --name default.
    # (notes: gate-map-agreement-report-only)
    if [ -f "$HERE/map_agreement.py" ]; then
      python3 "$HERE/map_agreement.py" "$OUTDIR" \
        >> "$OUTDIR/comms_gates.txt" 2>&1
      MAP_RC=$?
      if [ "$MAP_RC" != 0 ]; then
        printf 'INFO\tmap_agree\t%s\n' \
          "map_agreement.py exited $MAP_RC; it is contracted to exit 0 always, so it died before printing — the map-spread reading was NOT taken for this cell (its traceback is above in this file)" \
          >> "$OUTDIR/comms_gates.txt"
      fi
    else
      printf 'INFO\tmap_agree\tmap_agreement.py is missing from %s; the map-spread reading was NOT taken for this cell\n' \
        "$HERE" >> "$OUTDIR/comms_gates.txt"
    fi
    # Validity gate: a FAIL makes the run INVALID; inert unless
    # rendezvous_schedule=1. Must run after the planners stop and before the
    # counters. It must exit 0, so a non-zero exit is UNRUN. Test presence, not
    # -x. (notes: gate-rendezvous-agreement-validity)
    if [ -f "$HERE/rendezvous_agreement.py" ]; then
      python3 "$HERE/rendezvous_agreement.py" "$OUTDIR" \
        >> "$OUTDIR/comms_gates.txt" 2>&1
      RZV_RC=$?
      if [ "$RZV_RC" != 0 ]; then
        printf 'UNRUN\trendezvous_agreed\t%s\n' \
          "rendezvous_agreement.py exited $RZV_RC; it is contracted to exit 0 always, so it died before writing a verdict — the scheduled-arm check DID NOT RUN (its traceback is above in this file)" \
          >> "$OUTDIR/comms_gates.txt"
      fi
    else
      # Use the gate name the checker itself writes, rendezvous_agreed, not the
      # module filename: consumers such as gate_g8's REPORT_ONLY_GATES key on
      # that column. (notes: gate-rendezvous-agreed-name)
      printf 'UNRUN\trendezvous_agreed\t%s\n' \
        "rendezvous_agreement.py is missing from $HERE" \
        >> "$OUTDIR/comms_gates.txt"
    fi
    # teardown is trapped long before GATES_STRICT is assigned, and `set -u` is
    # on, so an early die() would abort IN the trap on an unbound variable.
    GATE_VERDICT=UNKNOWN
    NFAIL=$(grep -c "^FAIL" "$OUTDIR/comms_gates.txt" 2>/dev/null || true)
    NUNRUN=$(grep -c "^UNRUN" "$OUTDIR/comms_gates.txt" 2>/dev/null || true)
    if [ "${NFAIL:-0}" != 0 ]; then
      log "=============================================================="
      log "RUN INVALID: $NFAIL gate failure(s). $OUTDIR/comms_gates.txt:"
      grep "^FAIL" "$OUTDIR/comms_gates.txt" | sed 's/^/    /' || true
      log "=============================================================="
      GATE_VERDICT=INVALID
    elif [ "${NUNRUN:-0}" != 0 ]; then
      log "RUN SUSPECT: $NUNRUN gate(s) could not be evaluated — see $OUTDIR/comms_gates.txt"
      GATE_VERDICT=SUSPECT
    elif ! grep -q "	watch	" "$OUTDIR/comms_gates.txt" 2>/dev/null; then
      # No watch summary means the watcher never adjudicated: the file holds
      # bring-up PASSes only and lacks the outage gate, so absence of failures
      # is not a pass. (notes: gate-missing-watch-summary)
      log "RUN SUSPECT: no run-time gate summary — the watcher never reported"
      GATE_VERDICT=SUSPECT
    else
      log "comms gates: clean for the whole run"
      GATE_VERDICT=CLEAN
    fi
    # The verdict has to outlive this shell. A campaign driver decides "is this
    # cell done?" from the manifest alone, so a verdict that exists only in the
    # console log means an INVALID cell is indistinguishable from a good one on
    # resume and gets skipped forever.
    [ -f "$OUTDIR/run_manifest.txt" ] && \
      echo "run_gates_verdict=$GATE_VERDICT" >> "$OUTDIR/run_manifest.txt"
    # Under GATES_STRICT=1 a run whose run-time gates failed also exits
    # non-zero, so campaign drivers do not record it as OK.
    # (notes: gates-strict-runtime-exit)
    if [ "$GATE_VERDICT" = "INVALID" ] && [ "${GATES_STRICT:-0}" = "1" ]; then
      log "exiting non-zero: GATES_STRICT=1 and the run-time gates failed"
      exit 1
    fi
  elif [ "$COMMS" = "1" ]; then
    # COMMS=1 but no gate report: not one gate ran, so record INVALID, not
    # SUSPECT: run_campaign.sh redoes INVALID cells. COMMS=0 deliberately writes
    # no verdict; gate_g8 reads the absence as MISSING.
    # (notes: gate-report-missing-invalid)
    log "RUN INVALID: COMMS=1 but no $OUTDIR/comms_gates.txt — not one gate ran"
    [ -f "$OUTDIR/run_manifest.txt" ] && \
      echo "run_gates_verdict=INVALID" >> "$OUTDIR/run_manifest.txt"
    if [ "${GATES_STRICT:-0}" = "1" ]; then
      log "exiting non-zero: GATES_STRICT=1 and there is no gate report"
      exit 1
    fi
  fi
  log "teardown complete — outputs in $OUTDIR"
}
# --- resolve the ambient run-control knobs, before the trap and the manifest -
# Resolved before the manifest, which records them, and before trap teardown,
# which reads GATES_STRICT under set -u. STOP_ON_DONE and DONE_GRACE_S define
# what run_end_t_sim means. (notes: run-control-knobs-before-trap)
STOP_ON_DONE="${STOP_ON_DONE:-1}"
DONE_GRACE_S="${DONE_GRACE_S:-30}"
HANG_HB="${HANG_HB:-40}"
GATES_STRICT="${GATES_STRICT:-$COMMS}"
POLL_S="${POLL_S:-2}"
CLOCK_EVERY_S="${CLOCK_EVERY_S:-15}"
CLOCK_DEADMAN_S="${CLOCK_DEADMAN_S:-420}"
CLOCK_FAIL_MAX="${CLOCK_FAIL_MAX:-60}"

trap teardown EXIT INT TERM
die() { log "ERROR $*"; exit 1; }   # trap runs teardown

# wait_for <seconds> <what> -- <cmd...>
# Retry <cmd> until it succeeds or <seconds> of WALL CLOCK have elapsed.
#
# Deadline-based, not attempt-based: ros2 topic echo fails in under a second
# until the topic's type resolves, and ros2 topic list returns promptly, so an
# attempt count would not bound the wait.
# (notes: wait-for-deadline-not-attempts)
wait_for() {
  local budget=$1 what=$2; shift 2
  [ "$1" = "--" ] && shift
  local start_s deadline now
  start_s=$(date +%s); deadline=$(( start_s + budget ))
  until "$@" >/dev/null 2>&1; do
    now=$(date +%s)
    [ "$now" -ge "$deadline" ] && return 1
    # Progress, so a three-minute wait is not indistinguishable from a hang.
    [ $(( (now - start_s) % 30 )) -eq 0 ] && \
      log "  waiting for $what … $((now - start_s))/${budget}s"
    sleep 2
  done
  return 0
}

# --- 0. preflight -----------------------------------------------------------
# The banner reports N_ROBOTS and ROBOTS from the same scenario parse the
# planner census asserts on, so it names the team that actually spawned.
# (notes: preflight-banner-team-size)
log "=== $SCENARIO: ${N_ROBOTS}-robot ($ROBOTS) lidar explore+exploit, RViz=$RVIZ gui=$GZ_GUI ==="
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  (export the same value to inspect by hand)"
# Log which teardown scope is in force: with IGN_PARTITION unset, stack_procs
# and count_own match machine-wide, safe only for one cell. Deliberately not
# fatal; the manifest records it as ign_partition.
# (notes: preflight-log-teardown-scope)
if [ -n "${IGN_PARTITION:-}" ]; then
  log "IGN_PARTITION=$IGN_PARTITION  (teardown scoped to this partition)"
else
  log "IGN_PARTITION unset  (teardown matches stack processes MACHINE-WIDE; safe only if this is the only cell running)"
fi
log "targets: $TARGETS"
log "outputs: $OUTDIR"
PRE=$(stack_procs || true)
[ -n "$PRE" ] && { echo "$PRE"; TORN=1; die "stack processes already running — refusing to start"; }
if [ "$RVIZ" = "1" ]; then
  export DISPLAY="${DISPLAY:-:1}"
  log "RViz will use DISPLAY=$DISPLAY"
fi

# --- 1. simulator (N robots from the scenario, lidar-only models) -----------
GUI_ARG="headless:=true"; [ "$GZ_GUI" = "1" ] && GUI_ARG="headless:=false"
start sim "$OUTDIR/sim.log" \
  ros2 launch hmr_sim robot_sim.launch.py scenario:="$SCENARIO" $GUI_ARG
for r in $ROBOTS; do
  wait_for 180 "$r odom" -- \
    timeout 8 ros2 topic echo /$r/odom_ground_truth --once \
    || die "sim: no /$r/odom_ground_truth after 3 min (see $OUTDIR/sim.log)"
  log "sim publishing $r odom"
done

# --- 2. TF glue + viz model (one each per robot) -----------------------------
# hmr_sim publishes NO TF: sim_tf_publisher.py supplies map->odom (identity),
# odom->base_link (from GT odom) and the static sensor mounts.
for r in $ROBOTS; do
  start tf_$r "$OUTDIR/tf_$r.log" python3 "$HERE/sim_tf_publisher.py" $r
done
# robot_state_publisher exists here ONLY to give RViz a robot to draw: link
# names are pre-prefixed with "<robot>/" so they match the TF frames above with
# no frame_prefix/TF-Prefix indirection, and the wheel joints are fixed (nothing
# publishes /joint_states for a gz-driven robot).
for r in $ROBOTS; do
  urdf="$OUTDIR/${r}_viz.urdf"
  params="$OUTDIR/${r}_viz_params.yaml"
  sed -e "s|@PREFIX@|${r}/|g" -e "s|@MODEL@|${VIZ_MODEL[$r]}|g" -e "s|@ROBOT@|${r}|g" \
    "$URDF_IN" > "$urdf" || die "URDF substitution failed for $r"
  # Pass the URDF in a params file, never as a command-line parameter override:
  # rcl rejects override values containing newlines and robot_state_publisher
  # aborts. A YAML literal block takes the XML verbatim.
  # (notes: rsp-urdf-via-params-file)
  { echo "/**:"
    echo "  ros__parameters:"
    echo "    use_sim_time: true"
    echo "    robot_description: |"
    sed 's/^/      /' "$urdf"
  } > "$params" || die "params-file generation failed for $r"
  start rsp_$r "$OUTDIR/rsp_$r.log" \
    ros2 run robot_state_publisher robot_state_publisher --ros-args \
      -r __ns:=/$r --params-file "$params"
done

# --- 2b. comms emulator (COMMS=1) -------------------------------------------
# Must start before the mappers: relays subscribe via a 1 Hz discovery poll only
# once a source topic exists, so anything published earlier (the first map
# snapshot) is lost uncounted. The /rx/ check waits for the mappers.
# (notes: comms-emulator-before-mappers)
if [ "$COMMS" = "1" ]; then
  start comms "$OUTDIR/comms.log" \
    ros2 launch hmr_sim comms_sim.launch.py \
      scenario:="$SCENARIO" seed:="$SEED" use_sim_time:=true \
      tx_power_dbm:="$TX_POWER" \
      tree_attenuation_db:="$TREE_ATTEN" \
      max_range_m:="$MAX_RANGE" \
      reliable_queue_max_bytes:="$RELAY_QUEUE_BYTES"
  sleep 3
  log "comms emulator started (seed=$SEED tx_power_dbm=$TX_POWER" \
      "tree_attenuation_db=$TREE_ATTEN max_range_m=$MAX_RANGE" \
      "relay_queue=${RELAY_QUEUE_BYTES}B) ahead of the mappers"
  # Per-run connectivity trace: link_states is kept nowhere else and every radio
  # metric derives from link_states.csv. Started here, before the mappers, so it
  # covers the run's start; the logger waits for the topic.
  # (notes: comms-link-logger-early-start)
  start linklog "$OUTDIR/link_logger.log" \
    python3 "$HERE/link_logger.py" --out "$OUTDIR/link_states.csv" \
      --ros-args -p use_sim_time:=true
fi

# --- 3. nav + lidar mapping, mergers cross-wired ----------------------------
# Per robot, a scovox_node on its lidar plus a dscovox_node fusing the team's
# scovox_bin streams, so each planner reads its own copy of the team map. With
# COMMS=1 peer binaries come off the relay; self never does.
# (notes: nav-dscovox-team-map-wiring)
PEER_BIN_PATTERN="/{peer}/scovox_node/scovox_bin"
[ "$COMMS" = "1" ] && PEER_BIN_PATTERN="/{self}/rx/{peer}/scovox_node/scovox_bin"
log "peer_bin_topic_pattern=$PEER_BIN_PATTERN (COMMS=$COMMS)"
# One launch per robot over the roster; peers is a comma-separated list, so each
# dscovox_node fuses every other robot's binary and each planner reads a team
# map, not a pairwise one. (notes: nav-launch-per-robot-roster)
for r in $ROBOTS; do
  start nav_$r "$OUTDIR/nav_$r.log" \
    ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=$r mode:=ugv \
      mapping:=dscovox_lidar peers:="$(peers_csv "$r")" \
      peer_bin_topic_pattern:="$PEER_BIN_PATTERN" \
      voxel_resolution_m:=$VOXEL_RES \
      global_planning_map_size_m:=$PLAN_MAP_SIZE \
      global_planning_map_resolution:=$PLAN_MAP_RES
done
for r in $ROBOTS; do
  wait_for 180 "$r planning_map" -- \
    timeout 8 ros2 topic echo /$r/scovox_node/planning_map --once \
    || die "nav: no /$r planning_map after 3 min (see $OUTDIR/nav_$r.log)"
  log "$r mapping alive (planning_map publishing)"
  # Gate on scovox's global_planning_map: a wrong topic or a map-size arg left
  # at 0.0 (disabled) would fail silently. It is transient_local and emits only
  # when subscribed; this echo pulls the first sample.
  # (notes: nav-gate-scovox-global-map)
  wait_for 120 "$r global_planning_map" -- \
    timeout 10 ros2 topic echo /$r/scovox_node/global_planning_map --once \
    || die "nav: no /$r global_planning_map after 2 min — the \
exploration planner will never leave INIT (see $OUTDIR/nav_$r.log)"
  log "$r scovox global_planning_map alive (${PLAN_MAP_SIZE}m @ ${PLAN_MAP_RES}m/cell)"
  # The dscovox fused map is the one the planner reads. Gated separately with a
  # longer budget: dscovox publishes nothing until a first ScovoxMapBinary has
  # arrived and been fused. (notes: nav-gate-fused-global-map)
  wait_for 180 "$r fused global_planning_map" -- \
    timeout 10 ros2 topic echo /$r/dscovox_node/global_planning_map --once \
    || die "nav: no /$r dscovox global_planning_map after 3 min — the \
exploration planner reads THIS topic and will never leave INIT. Check that the \
merger fused at least one binary (see $OUTDIR/nav_$r.log)"
  log "$r fused global_planning_map alive (planner reads this one)"
done
if [ "$COMMS" = "1" ]; then
  # grep -q inside a function so wait_for can retry the whole pipeline.
  rx_topics_present() { timeout 6 ros2 topic list 2>/dev/null | grep -q "/rx/"; }
  wait_for 120 "/rx/ relay topics" -- rx_topics_present \
    || die "comms: no /rx/ relay topics after ~2 min (see $OUTDIR/comms.log)"
  log "comms relay topics present"
fi
sleep 10   # let the nav pipelines finish coming up

# --- 4. RViz ----------------------------------------------------------------
# Started BEFORE the planners so its candidate-marker subscription exists from
# the first PLAN cycle (publishCandidateViz() returns early on zero subscribers).
if [ "$RVIZ" = "1" ]; then
  start rviz "$OUTDIR/rviz.log" \
    rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
  sleep 5
fi
start targetviz "$OUTDIR/targetviz.log" python3 "$HERE/sim_target_markers.py"

# --- 5. optional bag --------------------------------------------------------
# Under COMMS=1 record the relayed streams and the link states: the pre-relay
# topics show perfect comms whatever the link did.
# (notes: bag-relayed-streams-under-comms)
COMMS_BAG_TOPICS=()
if [ "$COMMS" = "1" ]; then
  for r in $ROBOTS; do
    COMMS_BAG_TOPICS+=( "/$r/exploration/intents" )
    # Record what each robot sent and what arrived from each peer, so
    # convergence is checkable offline. team_world topics only when
    # TEAM_WORLD=1. Topic names grow as N(N-1); the bytes do not.
    # (notes: bag-sent-and-received-intents)
    for p in $(peers_of "$r"); do
      COMMS_BAG_TOPICS+=( "/$r/rx/$p/exploration/intents"
                          "/$r/rx/$p/scovox_node/scovox_bin" )
      if [ "$TEAM_WORLD" = "1" ]; then
        COMMS_BAG_TOPICS+=( "/$r/rx/$p/exploration/team_world" )
      fi
    done
    if [ "$TEAM_WORLD" = "1" ]; then
      COMMS_BAG_TOPICS+=( "/$r/exploration/team_world" )
    fi
  done
  # /hmr_comms_sim/stats carries backlog_bytes and the drop_* counters, the only
  # evidence for the backlog gate and shared-airtime drops; the gate watcher
  # writes samples out only on failure. (notes: bag-comms-stats-topic)
  COMMS_BAG_TOPICS+=( /hmr_comms_sim/link_states /hmr_comms_sim/robot_index
                      /hmr_comms_sim/stats )
fi
if [ "$RECORD" != "0" ]; then
  # RECORD=1 is the full set, enough to reconstruct or re-watch a run; RECORD=2
  # is the lean set of what the offline analysis reads. The OccupancyGrids
  # dominate the full set's size. (notes: bag-record-modes-full-lean)
  BAG_TOPICS=( /clock /tf_static )
  for r in $ROBOTS; do
    BAG_TOPICS+=( /$r/odom_ground_truth /$r/scovox_node/scovox_bin )
  done
  BAG_TOPICS+=( /exploration/intents )
  if [ "$RECORD" = "1" ]; then
    BAG_TOPICS+=( /tf )
    for r in $ROBOTS; do
      BAG_TOPICS+=( /$r/imu/data /$r/cmd_vel
                    /$r/scovox_node/planning_map
                    /$r/scovox_node/global_planning_map
                    /$r/goal_pose /$r/explo_planner/candidates )
    done
    BAG_TOPICS+=( /exploration/targets )
  fi
  log "recording rosbag (RECORD=$RECORD, ${#BAG_TOPICS[@]} base topics + comms)"
  start bag "$OUTDIR/bag.log" \
    ros2 bag record -o "$OUTDIR/rosbag2" \
      --max-bag-size 1000000000 --compression-mode file --compression-format zstd \
      "${BAG_TOPICS[@]}" \
      ${COMMS_BAG_TOPICS[@]+"${COMMS_BAG_TOPICS[@]}"}
  sleep 3
fi

# --- 6. planners (namespaced, one per robot) + one global scheduler ---------
# Same parameter set as exploitation_experiment.launch.py, namespaced per robot.
# terrain_relative_z must stay false on flatforest: terrain mode rejects
# vantages behind trunks as noground. The ground plane is at z=0.
# (notes: planner-launch-terrain-relative-z)
POLAR_ARG="true"; [ "$FRONTIER_ONLY" = "1" ] && POLAR_ARG="false"
DWELL_SYNC_ARG="true"; [ "$DWELL_SYNC" = "0" ] && DWELL_SYNC_ARG="false"
# off and mtare_off disable the manoeuvre via reconnect_enabled false;
# reconnect_mode is then inert, since shouldRendezvous() returns false first.
# Other mtare_* tokens map to their base mode; the planner rebuilds the arm
# label. (notes: arm-token-to-reconnect-params)
RECONNECT_ENABLED="true"; MODE_ARG="$RECONNECT_MODE"
case "$RECONNECT_MODE" in
  off|mtare_off)              RECONNECT_ENABLED="false"; MODE_ARG="hybrid" ;;
  mtare_pursuit|mtare_pursuit_mdp) MODE_ARG="pursuit" ;;
  mtare_rendezvous)           MODE_ARG="rendezvous" ;;
  mtare_hybrid|mtare_hybrid_mdp)   MODE_ARG="hybrid" ;;
esac
log "done_seek_enabled=$DONE_SEEK_ARG done_seek_max_sec=$DONE_SEEK_MAX (DONE_SEEK=$DONE_SEEK)"
log "mission_return_enabled=$MISSION_RETURN_ARG home_tol=${MISSION_HOME_TOL}m max=${MISSION_RETURN_MAX}s (MISSION_RETURN=$MISSION_RETURN)"
log "candidate_enable_polar=$POLAR_ARG (FRONTIER_ONLY=$FRONTIER_ONLY)"
log "proximity_hold/resume_dist_m=$PROX_HOLD_M/$PROX_RESUME_M m (yaml field defaults 5.0/6.0 overridden for sim)"
EXPLOIT_ARG="true"; [ "$EXPLOIT" = "0" ] && EXPLOIT_ARG="false"
# Built once, outside the launch loop, so every robot gets the identical ordered
# team list: an id must mean the same robot on every side, and known_by cannot
# detect a mismatch. (notes: cell-world-team-names-once)
TEAM_NAMES_ARG="["
for r in $ROBOTS; do
  [ "$TEAM_NAMES_ARG" = "[" ] || TEAM_NAMES_ARG="$TEAM_NAMES_ARG,"
  TEAM_NAMES_ARG="$TEAM_NAMES_ARG\"$r\""
done
TEAM_NAMES_ARG="$TEAM_NAMES_ARG]"
# Markers follow RVIZ: the planner gates publication on subscriber count, so
# asking for them headless costs a publisher and nothing else, but it also puts
# a topic on the graph that no run needs.
CELL_MARKERS_ARG="false"; [ "$RVIZ" = "1" ] && CELL_MARKERS_ARG="true"
if [ "$CELL_WORLD" = "1" ]; then
  log "cell world ON: team=$TEAM_NAMES_ARG cell_size=${CELL_SIZE_M}m census_period=${CELL_CENSUS_S}s markers=$CELL_MARKERS_ARG"
  log "  status thresholds: covered<=${CELL_COVERED_U} release>${CELL_EXPLORING_U} frontier_frac<=${CELL_FRONTIER_FRAC} (world-calibrated, cf. done_unknown_fraction=$DONE_UNKNOWN; library defaults 0.15/0.35 assume a saturating map and promote nothing here)"
  log "  ROI [-$ROI_HALF,$ROI_HALF]^2 at ${CELL_SIZE_M}m -> $(awk -v h="$ROI_HALF" -v c="$CELL_SIZE_M" 'BEGIN{n=int((2*h)/c); if (n*c < 2*h) n++; printf "%dx%d", n, n}') cells"
else
  log "cell world OFF (CELL_WORLD=0) — no cell params passed, planner is the pre-M-TARE binary at defaults"
fi
if [ "$TEAM_WORLD" = "1" ]; then
  log "team world exchange ON: ${TEAM_WORLD_HZ} Hz$([ "$COMMS" = 1 ] && echo " through the emulator" || echo " on the shared bus (COMMS=0: no dropout is possible, so this measures the merge, not the healing)")"
else
  log "team world exchange OFF (TEAM_WORLD=0) — no team_world params passed; the cell world is per-robot and never shared"
fi
if [ "$(awk -v v="$SEPARATION_WEIGHT" 'BEGIN{print (v+0 > 0.0) ? 1 : 0}')" = "1" ]; then
  log "separation term ON: candidates within ${SEPARATION_RADIUS_M} m of a teammate position no older than ${SEPARATION_MAX_AGE_SEC} s are discounted, linearly to x$(awk -v w="$SEPARATION_WEIGHT" 'BEGIN{printf "%.3f", 1.0-w}') on top of it"
else
  # At TEAM_WORLD=0 the planner has no peer positions, so sep_peer_dist_m stays
  # -1 and sep_eligible_peers 0: structural zeros, not an untreated
  # counterfactual. (notes: separation-off-structural-zeros)
  if [ "$TEAM_WORLD" = "1" ]; then
    log "separation term OFF (SEPARATION_WEIGHT=0) — no score is changed, but sep_peer_dist_m/sep_eligible_peers are still measured at ${SEPARATION_RADIUS_M} m / ${SEPARATION_MAX_AGE_SEC} s and are this cell's untreated counterfactual"
  else
    log "separation term OFF (SEPARATION_WEIGHT=0) — and with TEAM_WORLD=0 there are no peer positions to measure against, so sep_peer_dist_m stays -1 and sep_eligible_peers 0 for the whole cell. Those are structural zeros, NOT a measurement that the robots stayed apart; this cell is not an untreated counterfactual for a separation campaign"
  fi
fi
if [ "$GLOBAL_ALLOC" = "1" ]; then
  log "global allocator ON: solve-same-take-own over the shared cell world; the won focus cell re-ranks each tick's frontier candidates"
else
  log "global allocator OFF (GLOBAL_ALLOC=0) — no allocation event, candidate ordering is the per-robot utility alone"
fi
if [ -z "$ALLOC_POS_TTL" ]; then
  log "allocator peer-position TTL: not requested (node default 0 = unbounded) — a latched peer pose holds its cells for the rest of the run however old it gets, which is every campaign through sr3"
elif [ "$(awk -v v="$ALLOC_POS_TTL" 'BEGIN{print (v+0 > 0.0) ? 1 : 0}')" = "1" ]; then
  log "allocator peer-position TTL ON: ${ALLOC_POS_TTL} s — a peer whose pose is older than that is dropped from the allocation and the cells it held return to the pool. Applies to the doPlan solve only; the §3.6 gate and the rendezvous snapshot stay unbounded on purpose"
else
  log "allocator peer-position TTL requested as ${ALLOC_POS_TTL} = unbounded — this is the CONTROL level, passed explicitly so it travels the same -p path as the treated arm"
fi
if [ "$RECONNECT_GATE" = "info" ]; then
  log "reconnect gate = info: the mid-run silence clock is a FLOOR; past it, dispatch only if the peer is missing something we hold AND the leg pays for itself (C_re < C_no). Every evaluation is logged, suppressed or not."
else
  log "reconnect gate = silence (planner default) — the mid-run clock decides alone; no reconnect_gate param passed"
fi
if [ "$RENDEZVOUS_SCHEDULE" = "1" ]; then
  log "scheduled rendezvous ON: the team agrees one (cell, interval) while still connected — proposed by robot 0, echoed verbatim, committed only once every peer is confirmed holding the identical pair — and on separation each robot anchors it to its own view of when contact was lost and departs at its own travel-time deadline"
else
  # No arm has a fallback reconnect destination.
  # (notes: rendezvous-off-no-fallback)
  log "scheduled rendezvous OFF (RENDEZVOUS_SCHEDULE=0) — no rendezvous_schedule_enable param passed, so no appointment is ever armed and there is NO fallback destination (the midpoint drives are gone). Only reconnect_mode=pursuit and the off arm are legal here; rendezvous/hybrid are refused above"
fi
if [ "$PURSUIT_PREDICTOR" = "mdp" ]; then
  log "pursuit predictor = mdp: the chase aims at the argmax over the peer's last received tour of P(peer there when we arrive); the budget and every terminator still come from the trail, and an unaffordable intercept downgrades to it"
else
  log "pursuit predictor = trail (planner default) — no pursuit_predictor param passed; the chase drives the peer's last known goal then its last known pose"
fi
log "exploitation_enabled=$EXPLOIT_ARG (EXPLOIT=$EXPLOIT)"
log "exploit_dwell_sync_enabled=$DWELL_SYNC_ARG (DWELL_SYNC=$DWELL_SYNC)"
log "reconnect_mode=$MODE_ARG reconnect_enabled=$RECONNECT_ENABLED (RECONNECT_MODE=$RECONNECT_MODE)"
log "reconnect gates: confirm=${RECONNECT_CONFIRM}s barrier_max_wait=${RDV_MAX_WAIT}s"
log "roi x,y = [-$ROI_HALF, $ROI_HALF] (sim override; yaml carries the field site's ROI)"
# done_coverage_source is pinned to scovox, not auto: auto switches to the 2D
# planning_map when one arrives, changing the termination metric against the
# same done_unknown_fraction. (notes: done-coverage-source-scovox)
log "planning_map = /<r>/dscovox_node/global_planning_map (FUSED team map, ${PLAN_MAP_SIZE}m @ ${PLAN_MAP_RES}m/cell), done_coverage_source=scovox"

# --- 6a. run manifest -------------------------------------------------------
# Everything that distinguishes this run, written into the run directory. Git
# hashes carry a dirty marker, since a bare commit id misleads when there are
# uncommitted changes. (notes: manifest-purpose-and-provenance)
MANIFEST="$OUTDIR/run_manifest.txt"
{
  echo "# hmr_explo comms/reconnection run manifest"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "host=$(hostname)"
  echo "outdir=$OUTDIR"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
  # Both isolation keys, because a parallel campaign is only trustworthy if each
  # cell can be shown after the fact to have been isolated. "unset" here means
  # the cell ran unscoped, which is correct sequentially and a defect in a
  # parallel batch -- so the manifest must be able to say which it was.
  echo "ign_partition=${IGN_PARTITION:-unset}"
  echo
  echo "# --- arm / independent variables ---"
  # Per-robot JSONL event stream: the run's primary record. The manifest names
  # it so a reader knows to look for it (and knows it is missing if it is).
  echo "experiment_log=<robot>.events.jsonl"
  echo "reconnect_mode_requested=$RECONNECT_MODE"
  echo "reconnect_mode_param=$MODE_ARG"
  # none means no override was passed and the node took the yaml value.
  # run_campaign.sh strips the _r<N> arm suffix before RECONNECT_MODE is formed,
  # so this line is the only in-cell record.
  # (notes: manifest-claim-radius-override)
  echo "coord_claim_radius_override=${COORD_CLAIM_R:-none}"
  # The only in-cell record of the level (run_campaign.sh strips the _ttl<N>
  # suffix). none = nothing passed, compiled default 0 (unbounded); 0.0 =
  # unbounded asked for explicitly. Same behaviour, different strings.
  # (notes: manifest-alloc-pos-ttl)
  echo "alloc_peer_pos_max_age_sec=${ALLOC_POS_TTL:-none}"
  # Both spellings, one value. `reconnect_enabled` is the current name; the
  # legacy line stays FOREVER, because the cr3-cr5 manifests carry it and any
  # tool that reads a mixed set of campaigns must not have to know which side
  # of the 2026-09-03 rename a cell fell on.
  echo "reconnect_enabled=$RECONNECT_ENABLED"
  echo "rendezvous_enabled=$RECONNECT_ENABLED"
  echo "rendezvous_max_wait_sec=$RDV_MAX_WAIT"
  echo "reconnect_confirm_sec=$RECONNECT_CONFIRM"
  echo "reconnect_midrun_silence_sec=$MIDRUN_SILENCE"
  echo "reconnect_midrun_max_wait_sec=$MIDRUN_MAX_WAIT"
  echo "reconnect_midrun_max_attempts=$MIDRUN_MAX_ATTEMPTS"
  # Stamped for every cell, both sides. The arm has to be recoverable from the
  # cell itself -- a campaign script can be edited after the fact, a manifest
  # written at launch cannot.
  echo "done_seek_enabled=$DONE_SEEK_ARG"
  echo "done_seek_max_sec=$DONE_SEEK_MAX"
  # Mission return changes what a run's endpoints MEAN (see the MISSION_RETURN
  # block above): a resume must never mix cells across this switch, and an
  # analysis must refuse to pool them.
  echo "mission_return_enabled=$MISSION_RETURN_ARG"
  echo "mission_home_tol_m=$MISSION_HOME_TOL"
  echo "mission_return_max_sec=$MISSION_RETURN_MAX"
  echo "reconnect_min_share_voxels=$RECONNECT_MIN_SHARE_VOX"
  echo "reconnect_midrun_min_silence_sec=$MIDRUN_MIN_SILENCE"
  echo "reconnect_midrun_max_silence_sec=$MIDRUN_MAX_SILENCE"
  # Which clock the mid-run trigger fired on: a behavioural switch, so
  # link_gate=1 and link_gate=0 runs are different experiments.
  # (notes: manifest-link-gate-clock)
  echo "link_gate=$LINK_GATE"
  # link_gate is the request, link_gate_effective what the planner got; they
  # differ only when the gate was asked for with no emulator to feed it.
  # (notes: manifest-link-gate-effective)
  echo "link_gate_effective=$LINK_GATE_EFFECTIVE"
  echo "link_gate_topic=${LINK_GATE_TOPIC:-none}"
  echo "link_gate_index_topic=${LINK_GATE_INDEX:-none}"
  echo "link_gate_stale_sec=$LINK_GATE_STALE"
  echo "reconnect_link_down_confirm_sec=$LINK_DOWN_CONFIRM"
  echo "reconnect_release_confirm_sec=$RECONNECT_RELEASE_CONFIRM"
  echo "reconnect_arrive_tol_m=$RECONNECT_ARRIVE_TOL"
  echo "reconnect_nav_max_sec=$RECONNECT_NAV_MAX"
  echo "pursuit_staleness_max_sec=$PURSUIT_STALENESS"
  echo "pursuit_budget_max_sec=$PURSUIT_BUDGET_MAX"
  echo "pursuit_goal_stale_sec=$PURSUIT_GOAL_STALE"
  echo "pursuit_explore_fallback=$PURSUIT_EXPLORE_FALLBACK_ARG"
  echo "pursuit_explore_max=$PURSUIT_EXPLORE_MAX"
  echo "hold_escalate=$HOLD_ESCALATE_ARG"
  echo "hold_escalate_wait_sec=$HOLD_ESCALATE_WAIT"
  # The generation-19 countdown. Stamped on EVERY cell, including the off and
  # pursuit arms where they are inert: a reader comparing arms needs to see that
  # the untreated cells carried the same numbers, and "absent" cannot say that.
  echo "rendezvous_depart_delay_sec=$RDV_DEPART_DELAY"
  echo "rendezvous_max_lateness_sec=$RDV_MAX_LATE"
  echo "rendezvous_interval_sec=$RDV_INTERVAL"
  echo "rendezvous_settle_sec=$RDV_SETTLE"
  echo "rendezvous_appointment_wait_sec=$RDV_APPT_WAIT"
  echo "rendezvous_latched_hold_sec=$RDV_LATCHED_HOLD"
  echo "mission_start_hold_sec=$START_HOLD"
  echo "comms=$COMMS"
  echo "seed=$SEED"
  echo "tx_power_dbm=$TX_POWER"
  echo "tree_attenuation_db=$TREE_ATTEN"
  echo "max_range_m=$MAX_RANGE"
  # The arm's label for the outage gate, recorded because it is a claim about
  # what this run was FOR, not something recoverable from tx_power_dbm alone.
  echo "expect_outage=$EXPECT_OUTAGE"
  # The M-TARE layer. Recorded even when off, because "this run had the knob
  # and it was 0" and "this run predates the knob" are different facts and the
  # binary hash alone cannot separate them within a generation.
  echo "cell_world=$CELL_WORLD"
  echo "cell_size_m=$CELL_SIZE_M"
  echo "cell_census_period_s=$CELL_CENSUS_S"
  # World-calibrated, so they define what COVERED means on this run and a
  # census is not comparable across two runs that disagree about them.
  echo "cell_covered_max_unknown=$CELL_COVERED_U"
  echo "cell_exploring_min_unknown=$CELL_EXPLORING_U"
  echo "cell_covered_max_frontier_frac=$CELL_FRONTIER_FRAC"
  echo "team_robot_names=$TEAM_NAMES_ARG"
  # Recorded even when off, same rule. team_world=1 with comms=0 is a
  # materially different experiment from team_world=1 with comms=1 — the first
  # cannot produce a dropout to heal — and neither is recoverable from the
  # other fields, so both knobs have to be on the record together.
  echo "team_world=$TEAM_WORLD"
  echo "team_world_hz=$TEAM_WORLD_HZ"
  # The knobs that make the cell world decide something. Recorded even when off,
  # so a control run shows the feature existed and was disabled.
  # (notes: manifest-cell-world-decision-knobs)
  echo "global_alloc=$GLOBAL_ALLOC"
  echo "reconnect_gate=$RECONNECT_GATE"
  echo "rendezvous_schedule=$RENDEZVOUS_SCHEDULE"
  echo "pursuit_predictor=$PURSUIT_PREDICTOR"
  # Recorded even at weight 0: an off cell's sep_peer_dist_m and
  # sep_eligible_peers are still measured on this radius and age bound, and must
  # be comparable with a treated arm. (notes: manifest-separation-knobs)
  echo "separation_weight=$SEPARATION_WEIGHT"
  echo "separation_radius_m=$SEPARATION_RADIUS_M"
  echo "separation_max_age_sec=$SEPARATION_MAX_AGE_SEC"
  echo
  echo "# --- held fixed ---"
  echo "relay_queue_max_bytes=$RELAY_QUEUE_BYTES"
  echo "scenario=$SCENARIO"
  # The roster is derived from the scenario, so write it out: team size and
  # robot identity are then readable from the cell itself.
  # (notes: manifest-roster)
  echo "robots=$(echo $ROBOTS | tr ' ' ',')"
  echo "n_robots=$N_ROBOTS"
  echo "exploitation_enabled=$EXPLOIT_ARG"
  echo "dwell_sync=$DWELL_SYNC_ARG"
  echo "candidate_enable_polar=$POLAR_ARG"
  echo "max_steps=$MAX_STEPS"
  echo "duration_s=$DURATION_S"
  echo "roi_half=$ROI_HALF"
  echo "plan_map_size_m=$PLAN_MAP_SIZE"
  echo "plan_map_res_m=$PLAN_MAP_RES"
  echo "cost_grid_radius_cap_m=$COST_CAP"
  echo "candidate_min_goal_dist_m=$MIN_GOAL_DIST"
  echo "utility_cost_exponent=$UTIL_GAMMA"
  echo "frontier_z_lo_offset_m=$FRONTIER_Z_LO_OFF"
  echo "frontier_z_hi_offset_m=$FRONTIER_Z_HI_OFF"
  echo "visited_goal_radius_m=$VISITED_RADIUS"
  echo "visited_goal_ttl_sec=$VISITED_TTL"
  # (notes: manifest-failed-goal-history)
  echo "failed_goal_ttl_sec=$FAILED_TTL"
  echo "failed_goal_retire_after=$FAILED_RETIRE"
  echo "failed_goal_radius_m=$FAILED_RADIUS"
  # The nav budget and the no-progress abort. The run's end_reason vocabulary is
  # denominated in these, so a cell that omits them cannot be read on its own.
  echo "nav_min_timeout_sec=$NAV_MIN_TIMEOUT"
  echo "nav_max_timeout_sec=$NAV_MAX_TIMEOUT"
  echo "progress_window_sec=$PROGRESS_WINDOW"
  echo "progress_min_distance_m=$PROGRESS_MIN_DIST"
  echo "return_approach_window_sec=$RETURN_APPROACH_WINDOW"
  echo "return_approach_min_m=$RETURN_APPROACH_MIN"
  echo "return_escape_max_attempts=$RETURN_ESCAPE_MAX"
  echo "return_escape_leg_sec=$RETURN_ESCAPE_LEG"
  echo "voxel_resolution_m=$VOXEL_RES"
  echo "done_unknown_fraction=$DONE_UNKNOWN"
  echo "done_criterion=$DONE_CRITERION"
  echo "done_coverage_source=scovox"
  echo "prox_hold_m=$PROX_HOLD_M"
  echo "prox_resume_m=$PROX_RESUME_M"
  echo "targets=$TARGETS"
  echo "record=$RECORD"
  # stop_on_done and done_grace_s define the endpoint, so run_end_t_sim cannot
  # be read without them; the rest are abort and watchdog thresholds that decide
  # whether a slow cell is killed or banked. (notes: manifest-run-control-knobs)
  echo "stop_on_done=$STOP_ON_DONE"
  echo "done_grace_s=$DONE_GRACE_S"
  echo "hang_hb_sim_s=$HANG_HB"
  echo "gates_strict=$GATES_STRICT"
  echo "poll_s=$POLL_S"
  echo "clock_every_s=$CLOCK_EVERY_S"
  echo "clock_deadman_s=$CLOCK_DEADMAN_S"
  echo "clock_fail_max=$CLOCK_FAIL_MAX"
  echo "rviz=$RVIZ"
  echo "gz_gui=$GZ_GUI"
  echo
  # Number of trunks the radio model loaded, parsed from comms.log. A dirty sha
  # cannot show a tree-matcher change, and link_states gives only per-link
  # counts. (notes: manifest-comms-trees-loaded)
  if [ "$COMMS" = "1" ]; then
    echo "comms_trees_loaded=$(sed -n 's/.*Loaded \([0-9]\+\) tree positions.*/\1/p' \
      "$OUTDIR/comms.log" 2>/dev/null | head -1)"
  fi
  echo
  echo "# --- provenance (dirty = uncommitted changes present) ---"
  for repo in "$WS/.." "$WS/src/explo_planner" "$WS/src/hmr_sim" \
              "$WS/src/simple_nav_3d" "$WS/src/scovox"; do
    name=$(basename "$(cd "$repo" && pwd)")
    if git -C "$repo" rev-parse --git-dir >/dev/null 2>&1; then
      sha=$(git -C "$repo" rev-parse --short HEAD 2>/dev/null || echo unknown)
      dirty=""
      if [ -n "$(git -C "$repo" status --porcelain 2>/dev/null)" ]; then
        # A bare "-dirty" flag cannot tell two different uncommitted trees apart,
        # which is exactly the case during a build session. Hash the diff (plus
        # untracked file names) so each working-tree state gets its own id.
        dirty="-dirty.$( { git -C "$repo" diff HEAD; \
          git -C "$repo" ls-files --others --exclude-standard; } 2>/dev/null \
          | sha1sum | cut -c1-8 )"
      fi
      echo "git_$name=$sha$dirty"
    else
      echo "git_$name=not-a-repo"
    fi
  done
  # Commit hashes identify the source; this hash identifies the binary that ran.
  # The install is symlinked, so stat must dereference (-L) to date the real
  # file. (notes: manifest-planner-binary-hash)
  planner_bin="$WS/install/explo_planner/lib/explo_planner/explo_planner_node"
  if [ -e "$planner_bin" ]; then
    echo "sha256_explo_planner_node=$(sha256sum -b "$planner_bin" 2>/dev/null \
      | cut -c1-16)"
    echo "mtime_explo_planner_node=$(stat -Lc '%y' "$planner_bin" 2>/dev/null)"
  else
    echo "sha256_explo_planner_node=missing"
    # Present and saying "missing" beats absent, for the same reason spelled
    # out twice for the params keys below: a consumer that greps for
    # mtime_explo_planner_node and gets no line cannot tell a binary that was
    # not there from a manifest written before the key existed.
    echo "mtime_explo_planner_node=missing"
  fi
  # git_simple_nav_3d moves with the source, not with a rebuild, so hash what
  # ran. All four executables install as a unit; one hash moving alone means a
  # partial or stale install. (notes: manifest-nav-binary-hashes)
  for _navnode in simple_nav_costmap_node simple_nav_planner_node \
                  simple_nav_controller_node simple_nav_navigator_node; do
    _navbin="$WS/install/simple_nav_3d/lib/simple_nav_3d/$_navnode"
    if [ -e "$_navbin" ]; then
      echo "sha256_$_navnode=$(sha256sum -b "$_navbin" 2>/dev/null | cut -c1-16)"
    else
      echo "sha256_$_navnode=missing"
    fi
  done
  # The node loads shared_params.yaml from the install tree, so hash that copy.
  # done_action must be idle for the rendezvous barrier (node default shutdown);
  # a params file that failed to install turns it off.
  # (notes: manifest-params-file-hash)
  planner_params="$WS/install/explo_planner/share/explo_planner/config/shared_params.yaml"
  if [ -e "$planner_params" ]; then
    echo "sha256_shared_params=$(sha256sum -b "$planner_params" 2>/dev/null \
      | cut -c1-16)"
    # Accepts quoted or bare scalars: a probe that reports missing for a
    # formatting choice trains readers to ignore the key.
    # (notes: manifest-params-probe-bare-scalars)
    echo "done_action_in_params=$(sed -n \
      's/^[[:space:]]*done_action:[[:space:]]*"\{0,1\}\([^"#]*\)"\{0,1\}.*/\1/p' \
      "$planner_params" 2>/dev/null | head -1 | sed 's/[[:space:]]*$//')"
    # Two more properties that live only in the params file. With
    # coord_claim_radius_override this pins the claim disc: the override wins
    # when set, else this value, and 0.0 means auto (fov_max_range).
    # (notes: manifest-params-alloc-and-claim)
    echo "global_alloc_comms_mask_in_params=$(sed -n \
      's/^[[:space:]]*global_alloc_comms_mask:[[:space:]]*"\{0,1\}\([^"#]*\)"\{0,1\}.*/\1/p' \
      "$planner_params" 2>/dev/null | head -1 | sed 's/[[:space:]]*$//')"
    echo "coord_claim_radius_m_in_params=$(sed -n \
      's/^[[:space:]]*coord_claim_radius_m:[[:space:]]*"\{0,1\}\([^"#]*\)"\{0,1\}.*/\1/p' \
      "$planner_params" 2>/dev/null | head -1 | sed 's/[[:space:]]*$//')"
  else
    # Write every key as missing, not just the hash: an absent key is
    # indistinguishable from an old manifest, and a params file that failed to
    # install is exactly what these keys expose.
    # (notes: manifest-params-missing-keys)
    echo "sha256_shared_params=missing"
    echo "done_action_in_params=missing"
    # Same argument as done_action_in_params above: present and saying "missing"
    # beats absent, because an absent key is indistinguishable from a manifest
    # written before the key existed.
    echo "global_alloc_comms_mask_in_params=missing"
    echo "coord_claim_radius_m_in_params=missing"
  fi
  # World provenance: the scenario names a world short name, which
  # _world_registry.py maps to an SDF. The SDF's internal world name must equal
  # the registry name, or robot_sim.launch.py spawns fail.
  # (notes: manifest-world-geometry)
  _world_short=$(sed -n 's/^world:[[:space:]]*\([^[:space:]#]*\).*/\1/p' \
    "$SCENARIO_PATH" 2>/dev/null | head -1)
  echo "world=${_world_short:-missing}"
  _world_sdf=$(python3 - "$WS" "${_world_short:-}" <<'PYWORLD' 2>/dev/null
import os, runpy, sys
ws, short = sys.argv[1], sys.argv[2]
reg = os.path.join(ws, 'install/hmr_sim/share/hmr_sim/launch/_world_registry.py')
e = runpy.run_path(reg)['WORLDS'][short]
print(os.path.join(ws, 'install/hmr_sim/share/hmr_sim/worlds',
                   e['sdf_subdir'], e['sdf_file']))
PYWORLD
)
  if [ -n "${_world_sdf:-}" ] && [ -e "$_world_sdf" ]; then
    echo "world_sdf=${_world_sdf#$WS/}"
    echo "sha256_world_sdf=$(sha256sum -b "$_world_sdf" 2>/dev/null | cut -c1-16)"
    # Internal name, plus three counts that make a geometry change legible
    # without diffing a 900 kB file: how many models, how many are collidable,
    # and how many are still dynamic (a physics cost, and for scene decoration
    # always a mistake).
    _world_geom=$(python3 - "$_world_sdf" <<'PYGEOM' 2>/dev/null
import sys, xml.etree.ElementTree as ET
w = ET.parse(sys.argv[1]).getroot().find('world')
models = w.findall('model')
print(f"sdf_world_name={w.get('name')}")
print(f"world_models={len(models)}")
print(f"world_collisions={len(list(w.iter('collision')))}")
print("world_nonstatic_models=%d" % sum(
    1 for m in models if (m.findtext('static') or 'false').strip() == 'false'))
PYGEOM
)
    if [ -n "$_world_geom" ]; then
      printf '%s\n' "$_world_geom"
    else
      # The SDF exists but did not parse. Say so in every key rather than
      # dropping them, so a reader cannot mistake it for an older manifest.
      echo "sdf_world_name=unparseable"
      echo "world_models=unparseable"
      echo "world_collisions=unparseable"
      echo "world_nonstatic_models=unparseable"
    fi
  else
    # Present and saying "missing", never absent: same rule as every key above.
    echo "world_sdf=missing"
    echo "sha256_world_sdf=missing"
    echo "sdf_world_name=missing"
    echo "world_models=missing"
    echo "world_collisions=missing"
    echo "world_nonstatic_models=missing"
  fi
} > "$MANIFEST"
log "run manifest written: $MANIFEST"
# Here because log() does not exist in the defaults block. LINK_GATE=1 with no
# emulator is silently unsafe: MIDRUN_SILENCE defaults below the
# heartbeat-suppression tail and relies on the veto.
# (notes: link-gate-without-emulator-warning)
if [ "$LINK_GATE" = "1" ] && [ "$LINK_GATE_EFFECTIVE" != "1" ]; then
  log "WARNING: LINK_GATE=1 but COMMS=$COMMS -- no link emulator, so the veto" \
      "CANNOT run and this run fires on the bare record-age clock with" \
      "reconnect_midrun_silence_sec=$MIDRUN_SILENCE. Set COMMS=1, or raise" \
      "MIDRUN_SILENCE above 200, or pass LINK_GATE=0 to say you meant it."
fi
# Built once, outside the loop. Empty unless the caller set something, and
# expanded with the ${a[@]+"${a[@]}"} guard because `set -u` treats an empty
# array expansion as an unbound variable on bash < 4.4.
FOV_ARGS=()
if [ -n "$FOV_HFOV" ];   then FOV_ARGS+=( -p fov_hfov:="$(flt "$FOV_HFOV")" ); fi
if [ -n "$FOV_VFOV" ];   then FOV_ARGS+=( -p fov_vfov:="$(flt "$FOV_VFOV")" ); fi
if [ -n "$FOV_H_RAYS" ]; then FOV_ARGS+=( -p fov_h_rays:="$FOV_H_RAYS" ); fi
if [ -n "$FOV_V_RAYS" ]; then FOV_ARGS+=( -p fov_v_rays:="$FOV_V_RAYS" ); fi
if [ -n "$CAND_N_YAW" ]; then FOV_ARGS+=( -p candidate_n_yaw:="$CAND_N_YAW" ); fi
if [ ${#FOV_ARGS[@]} -gt 0 ]; then
  log "EIG sensor-model OVERRIDE: ${FOV_ARGS[*]}"
  echo "fov_override=${FOV_ARGS[*]}" >> "$MANIFEST"
else
  echo "fov_override=none (shared_params.yaml)" >> "$MANIFEST"
fi
for r in $ROBOTS; do
  # COMMS=1 splits the intent stream: publish to a per-robot topic the emulator
  # can see, subscribe to the relayed copy of the peer's. Both defaults are
  # empty, so with COMMS=0 nothing is passed and the planner keeps using the one
  # shared /exploration/intents bus exactly as before.
  EXTRA=()
  if [ "$COMMS" = "1" ]; then
    # One subscription per peer. The param is a string ARRAY on the node side
    # and always has been, so N=2 passes the same single-element list it did.
    EXTRA=( -p coord_intent_pub_topic:=exploration/intents
            -p coord_intent_sub_topics:="$(peers_ros_array "$r" 'rx/' '/exploration/intents')" )
    log "$r intents: pub /$r/exploration/intents  sub $(peers_ros_array "$r" "/$r/rx/" '/exploration/intents')"
  fi
  # Link-state gate. Appended to the same array rather than passed as bare -p
  # flags because the "off" value is the empty string, and `-p name:=` with
  # nothing after it is not a parameter assignment ros2 will accept — an unset
  # gate has to mean "pass no parameter at all", not "pass an empty one".
  if [ -n "$LINK_GATE_TOPIC" ]; then
    EXTRA+=( -p comms_link_states_topic:="$LINK_GATE_TOPIC"
             -p comms_link_robot_index_topic:="$LINK_GATE_INDEX"
             -p comms_link_stale_sec:="$LINK_GATE_STALE" )
    log "$r link gate: $LINK_GATE_TOPIC (index $LINK_GATE_INDEX, stale ${LINK_GATE_STALE}s)"
  fi
  # Coarse cell world. team_robot_names travels with it and is the SAME
  # ordered list on every robot — a robot's id is its index in this array, so
  # two robots given different orderings would disagree about which mask bit
  # means whom while both looking perfectly healthy on their own.
  if [ "$CELL_WORLD" = "1" ]; then
    EXTRA+=( -p team_robot_names:="$TEAM_NAMES_ARG"
             -p cell_world_enable:=true
             -p cell_size_m:="$CELL_SIZE_M"
             -p cell_census_period_s:="$CELL_CENSUS_S"
             -p cell_covered_max_unknown:="$CELL_COVERED_U"
             -p cell_exploring_min_unknown:="$CELL_EXPLORING_U"
             -p cell_covered_max_frontier_frac:="$CELL_FRONTIER_FRAC"
             -p publish_cell_markers:="$CELL_MARKERS_ARG" )
  fi
  # Split pub/sub like the intent stream, so the emulator can relay it per link.
  # With COMMS=0 both ends use the shared /exploration/team_world bus, so such a
  # run says nothing about gating. (notes: team-world-split-topics)
  if [ "$TEAM_WORLD" = "1" ]; then
    EXTRA+=( -p team_world_hz:="$TEAM_WORLD_HZ" )
    if [ "$COMMS" = "1" ]; then
      EXTRA+=( -p team_world_pub_topic:=exploration/team_world
               -p team_world_sub_topics:="$(peers_ros_array "$r" 'rx/' '/exploration/team_world')" )
      log "$r team_world: pub /$r/exploration/team_world  sub $(peers_ros_array "$r" "/$r/rx/" '/exploration/team_world')"
    else
      log "$r team_world: ${TEAM_WORLD_HZ} Hz on the shared /exploration/team_world bus (COMMS=0)"
    fi
  fi
  # Global allocator and the §3.6 reconnect gate. Every other knob of both is
  # left at its compiled default on purpose: the arm under test is the
  # mechanism, not a tuning of it, and each sim-side override is one more thing
  # that has to be held identical across arms and re-justified per campaign.
  if [ "$GLOBAL_ALLOC" = "1" ]; then
    EXTRA+=( -p global_alloc_enable:=true )
  fi
  # MinPos claim disc, passed only when overridden; unset passes nothing and the
  # node keeps the yaml value. Both levels of a radius comparison come through
  # this override path, so it is common to the two arms.
  # (notes: planner-claim-radius-override)
  if [ -n "$COORD_CLAIM_R" ]; then
    EXTRA+=( -p coord_claim_radius_m:="$(flt "$COORD_CLAIM_R")" )
  fi
  # Passed only when set; unset leaves the node on its compiled 0 (unbounded).
  # The control arm is an explicit 0, so both arms share this path. flt()
  # because the node declares a double; a bare int aborts it.
  # (notes: planner-alloc-pos-ttl-param)
  if [ -n "$ALLOC_POS_TTL" ]; then
    EXTRA+=( -p alloc_peer_pos_max_age_sec:="$(flt "$ALLOC_POS_TTL")" )
  fi
  # Passed always, even at weight 0: radius and max age still govern
  # sep_peer_dist_m and sep_eligible_peers in an off arm, so both arms share one
  # path. flt(): doubles, and a bare int aborts the node.
  # (notes: planner-separation-params-always)
  EXTRA+=( -p separation_weight:="$(flt "$SEPARATION_WEIGHT")"
           -p separation_radius_m:="$(flt "$SEPARATION_RADIUS_M")"
           -p separation_max_age_sec:="$(flt "$SEPARATION_MAX_AGE_SEC")" )
  if [ "$RECONNECT_GATE" = "info" ]; then
    EXTRA+=( -p reconnect_gate:=info )
  fi
  if [ "$RENDEZVOUS_SCHEDULE" = "1" ]; then
    EXTRA+=( -p rendezvous_schedule_enable:=true )
  fi
  if [ "$PURSUIT_PREDICTOR" = "mdp" ]; then
    EXTRA+=( -p pursuit_predictor:=mdp )
  fi
  start planner_$r "$OUTDIR/planner_$r.log" \
    ros2 run explo_planner explo_planner_node --ros-args \
      -r __ns:=/$r -r __node:=explo_planner \
      --params-file "$PLANNER_SHARE/config/shared_params.yaml" \
      -p use_sim_time:=true -p robot_name:=$r -p max_steps:=$MAX_STEPS \
      -p terrain_relative_z:=false \
      -p experiment_log_path:="$OUTDIR/$r.events.jsonl" \
      -p candidate_enable_polar:=$POLAR_ARG \
      -p proximity_hold_dist_m:=$PROX_HOLD_M \
      -p proximity_resume_dist_m:=$PROX_RESUME_M \
      -p exploit_dwell_sync_enabled:=$DWELL_SYNC_ARG \
      -p reconnect_mode:=$MODE_ARG \
      -p reconnect_enabled:=$RECONNECT_ENABLED \
      -p rendezvous_max_wait_sec:=$RDV_MAX_WAIT \
      -p reconnect_confirm_sec:=$RECONNECT_CONFIRM \
      -p reconnect_link_down_confirm_sec:=$LINK_DOWN_CONFIRM \
      -p reconnect_midrun_silence_sec:=$MIDRUN_SILENCE \
      -p reconnect_midrun_max_wait_sec:=$MIDRUN_MAX_WAIT \
      -p reconnect_midrun_max_attempts:=$MIDRUN_MAX_ATTEMPTS \
      -p done_seek_enabled:=$DONE_SEEK_ARG \
      -p done_seek_max_sec:=$DONE_SEEK_MAX \
      -p mission_return_enabled:=$MISSION_RETURN_ARG \
      -p mission_home_tol_m:=$MISSION_HOME_TOL \
      -p mission_return_max_sec:=$MISSION_RETURN_MAX \
      -p reconnect_min_share_voxels:=$RECONNECT_MIN_SHARE_VOX \
      -p reconnect_midrun_min_silence_sec:=$MIDRUN_MIN_SILENCE \
      -p reconnect_midrun_max_silence_sec:=$MIDRUN_MAX_SILENCE \
      -p reconnect_release_confirm_sec:=$RECONNECT_RELEASE_CONFIRM \
      -p reconnect_arrive_tol_m:=$RECONNECT_ARRIVE_TOL \
      -p reconnect_nav_max_sec:=$RECONNECT_NAV_MAX \
      -p pursuit_staleness_max_sec:=$PURSUIT_STALENESS \
      -p pursuit_budget_max_sec:=$PURSUIT_BUDGET_MAX \
      -p pursuit_goal_stale_sec:=$PURSUIT_GOAL_STALE \
      -p pursuit_explore_fallback:=$PURSUIT_EXPLORE_FALLBACK_ARG \
      -p pursuit_explore_max:=$PURSUIT_EXPLORE_MAX \
      -p hold_escalate:=$HOLD_ESCALATE_ARG \
      -p hold_escalate_wait_sec:=$HOLD_ESCALATE_WAIT \
      -p rendezvous_depart_delay_sec:=$RDV_DEPART_DELAY \
      -p rendezvous_max_lateness_sec:=$RDV_MAX_LATE \
      -p rendezvous_interval_sec:=$RDV_INTERVAL \
      -p rendezvous_settle_sec:=$RDV_SETTLE \
      -p rendezvous_appointment_wait_sec:=$RDV_APPT_WAIT \
      -p rendezvous_latched_hold_sec:=$RDV_LATCHED_HOLD \
      -p mission_start_hold_sec:=$START_HOLD \
      -p exploitation_enabled:=$EXPLOIT_ARG \
      -p rendezvous_expected_peers:=$((N_ROBOTS - 1)) \
      -p roi_min_x:=-$ROI_HALF -p roi_max_x:=$ROI_HALF \
      -p roi_min_y:=-$ROI_HALF -p roi_max_y:=$ROI_HALF \
      -p use_planning_map:=true \
      -p planning_map_topic:=/$r/dscovox_node/global_planning_map \
      -p done_coverage_source:=scovox \
      -p cost_grid_radius_cap_m:=$COST_CAP \
      -p candidate_min_goal_dist_m:=$MIN_GOAL_DIST \
      -p utility_cost_exponent:=$UTIL_GAMMA \
      -p map_resolution:=$VOXEL_RES \
      -p done_unknown_fraction:=$DONE_UNKNOWN \
      -p done_criterion:=$DONE_CRITERION \
      -p frontier_z_lo_offset_m:=$FRONTIER_Z_LO_OFF \
      -p frontier_z_hi_offset_m:=$FRONTIER_Z_HI_OFF \
      -p visited_goal_radius_m:=$VISITED_RADIUS \
      -p visited_goal_ttl_sec:=$VISITED_TTL \
      -p failed_goal_ttl_sec:=$FAILED_TTL \
      -p failed_goal_retire_after:=$FAILED_RETIRE \
      -p failed_goal_radius_m:=$FAILED_RADIUS \
      -p nav_min_timeout_sec:=$NAV_MIN_TIMEOUT \
      -p nav_max_timeout_sec:=$NAV_MAX_TIMEOUT \
      -p progress_window_sec:=$PROGRESS_WINDOW \
      -p progress_min_distance_m:=$PROGRESS_MIN_DIST \
      -p return_approach_window_sec:=$RETURN_APPROACH_WINDOW \
      -p return_approach_min_m:=$RETURN_APPROACH_MIN \
      -p return_escape_max_attempts:=$RETURN_ESCAPE_MAX \
      -p return_escape_leg_sec:=$RETURN_ESCAPE_LEG \
      ${FOV_ARGS[@]+"${FOV_ARGS[@]}"} \
      ${EXTRA[@]+"${EXTRA[@]}"} \
      -p output_csv:="$OUTDIR/planner_$r.csv"
done
# Node name must stay target_scheduler in the root namespace to match the
# targets yaml's parameter key. Skipped with EXPLOIT=0 so no TreeTarget traffic
# goes on the ungated /exploration/targets bus.
# (notes: scheduler-node-name-and-exploit-off)
if [ "$EXPLOIT" = "0" ]; then
  log "EXPLOIT=0 — pure exploration, target_scheduler NOT started"
else
  start sched "$OUTDIR/sched.log" \
    ros2 run explo_planner target_scheduler_node --ros-args \
      -r __node:=target_scheduler \
      --params-file "$TARGETS" -p use_sim_time:=true
fi
sleep 8
# Count by the absolute install path only the real binary carries; the leading
# [l] keeps the pattern from matching grep's own cmdline. count_own scopes to
# this cell; expect exactly N_ROBOTS. (notes: planner-census-count)
NPLAN=$(count_own "[l]ib/explo_planner/explo_planner_node")
[ "$NPLAN" = "$N_ROBOTS" ] || die "expected exactly $N_ROBOTS explo_planner_node (own cell), found $NPLAN"
log "planners up (exactly $N_ROBOTS explo_planner_node)"

# --- 6b. comms gates (COMMS=1) ----------------------------------------------
# Runs after the planners: two of the four gates need the intent endpoints.
# GATES_STRICT=1, the default when COMMS=1, aborts on a failed gate; 0 reports
# and continues. Resolved with the run-control knobs.
# (notes: comms-gates-bringup-strict)
if [ "$COMMS" = "1" ]; then
  GATE_REPORT="$OUTDIR/comms_gates.txt"
  ROBOT_CSV=$(echo $ROBOTS | tr ' ' ',')
  log "running comms bring-up gates (report: $GATE_REPORT)"
  # Take the gate's own exit status, not a pipeline's: there is no pipefail
  # here. The TeamWorld relay is required only when TEAM_WORLD=1, since
  # team_world_hz defaults to 0.0. (notes: comms-gates-capture-status)
  GATE_EXTRA=()
  if [ "$TEAM_WORLD" = "1" ]; then
    GATE_EXTRA=( --gated-extra exploration/team_world )
    log "gates: also requiring the exploration/team_world relay"
  fi
  python3 "$HERE/comms_gates.py" check --robots "$ROBOT_CSV" \
      --report "$GATE_REPORT" ${GATE_EXTRA[@]+"${GATE_EXTRA[@]}"} \
      >"$OUTDIR/gates_check.out" 2>&1
  GATE_RC=$?
  cat "$OUTDIR/gates_check.out" | tee -a "$OUTDIR/gates.log"
  if [ "$GATE_RC" = 0 ]; then
    log "comms gates: all clear"
  else
    log "WARNING comms gates FAILED (rc=$GATE_RC) — see $GATE_REPORT"
    [ "$GATES_STRICT" = "1" ] && die "comms gates failed rc=$GATE_RC (GATES_STRICT=1)"
  fi
  # EXPECT_OUTAGE=0 is for a control arm run through the emulator at a power
  # where the link stays up. The watcher starts here because it needs the
  # planners' intent endpoints. (notes: comms-gates-expect-outage-watch)
  start gateswatch "$OUTDIR/gates_watch.log" \
    python3 "$HERE/comms_gates.py" watch --robots "$ROBOT_CSV" \
      --report "$GATE_REPORT" ${GATE_EXTRA[@]+"${GATE_EXTRA[@]}"} \
      --expect-outage "$([ "$EXPECT_OUTAGE" = 0 ] && echo no || echo yes)"
fi

T0=$(sim_clock)
[ -n "$T0" ] || die "cannot read /clock"
# SECONDS is never reassigned in this file, so T0_WALL is the bring-up time and
# SECONDS at the end the whole cell; the run-end block banks both.
# (notes: t0-wall-bringup-time)
T0_WALL=$SECONDS
log "sim t0=$T0 — targets release at +120 / +420 / +720 s of the SCHEDULER's clock"
if [ "$DURATION_S" != "0" ]; then
  log "will stop at t_sim=$((T0 + DURATION_S))"
else
  log "no DURATION_S — running until Ctrl-C"
fi

# --- 7. hold, watching component health -------------------------------------
LAST_HB=0
# HANG_HB counts 60-sim-s heartbeats with no step (default 40 = 2400 sim-s),
# sized above the configured manoeuvre budgets. If HANG_HB*60 >= DURATION_S the
# gate cannot fire, so warn. (notes: hang-gate-threshold-sizing)
if [ "$DURATION_S" != "0" ] && [ "$((HANG_HB * 60))" -ge "$DURATION_S" ]; then
  log "WARNING: hang gate is INERT — needs $((HANG_HB * 60)) sim-s of frozen" \
      "steps but DURATION_S=$DURATION_S ends the run first. A hung planner" \
      "will run to the censoring horizon instead of aborting early."
fi
# STOP_ON_DONE=1 ends the run when every planner CSV's state column (found by
# header, never a fixed index) reads DONE; a log line is not terminal.
# DURATION_S still caps it; DONE_GRACE_S is the sim-s drain before teardown.
# (notes: stop-on-done-and-grace)
DONE_SINCE=-1
# Last value of the `state` column in a planner CSV, or empty if the file has no
# data rows yet.
planner_state() {
  awk -F, -v want=state '
    NR==1 { for (i=1; i<=NF; i++) if ($i == want) { col=i } ; next }
    col && NF >= col { last=$col }
    END { print last }
  ' "$OUTDIR/planner_$1.csv" 2>/dev/null
}
# Two wall-second cadences: POLL_S for the cheap liveness and CSV-state checks;
# CLOCK_EVERY_S for sim_clock, which spawns a ros2 process, forced to POLL_S
# only on the all-DONE edge and in the grace window.
# (notes: poll-cadence-two-budgets)
LAST_STEPS=-1; STALL=0
LAST_CLOCK_WALL=0
# Wall-clock deadman on the sim clock: a deadlocked Gazebo keeps every process
# alive and the clock frozen, which no sim-time condition catches.
# CLOCK_DEADMAN_S is deliberately generous. (notes: sim-clock-deadman)
LAST_T_SEEN=-1
LAST_T_WALL=$SECONDS
# A /clock read that returns nothing `continue`s, so it must not be able to spin
# unbounded either: an rmw failure would otherwise look exactly like the frozen
# clock above, minus the log line.
CLOCK_FAIL=0
# CLOCK_FAIL_MAX is resolved with the other run-control knobs above.
# (notes: clock-fail-max-pointer)
while true; do
  sleep "$POLL_S"
  for entry in "${PIDS[@]}"; do
    name=${entry%%:*}; pid=${entry##*:}
    # RViz closed by the user is a normal way to end a watch session.
    if [ "$name" = "rviz" ] && ! alive "$pid"; then
      log "RViz exited — tearing down"; exit 0
    fi
    alive "$pid" || die "$name (pid $pid) died mid-run — see $OUTDIR/$name.log"
  done
  # Cheap probe, hoisted ABOVE the clock read. This is the whole point of the
  # fast poll: it answers "is the run over?" without paying for a ros2 spawn.
  # Kept at 0 when STOP_ON_DONE is off so the clock budget below falls through
  # to the slow cadence, i.e. byte-for-byte the old behaviour.
  all_done=0
  if [ "$STOP_ON_DONE" = "1" ]; then
    all_done=1
    for r in $ROBOTS; do
      [ "$(planner_state "$r")" = "DONE" ] || { all_done=0; break; }
    done
  fi
  # $SECONDS is a bash builtin, so this costs no fork. Read the clock on the
  # slow budget, EXCEPT on the all-DONE edge or with a grace window already
  # open -- there the next few seconds decide when the run ends.
  NOW_WALL=$SECONDS
  if [ "$all_done" = 1 ] || [ "$DONE_SINCE" != -1 ] \
     || [ $((NOW_WALL - LAST_CLOCK_WALL)) -ge "$CLOCK_EVERY_S" ]; then
    LAST_CLOCK_WALL=$NOW_WALL
  else
    continue
  fi
  T=$(sim_clock)
  if [ -z "$T" ]; then
    CLOCK_FAIL=$((CLOCK_FAIL + 1))
    log "WARN /clock read failed ($CLOCK_FAIL/$CLOCK_FAIL_MAX), retrying"
    [ "$CLOCK_FAIL" -lt "$CLOCK_FAIL_MAX" ] \
      || die "/clock unreadable for $CLOCK_FAIL consecutive reads — giving up on this cell"
    continue
  fi
  CLOCK_FAIL=0
  # Deadman: sim time itself must advance. See CLOCK_DEADMAN_S above.
  if [ "$T" != "$LAST_T_SEEN" ]; then
    LAST_T_SEEN=$T
    LAST_T_WALL=$NOW_WALL
  elif [ $((NOW_WALL - LAST_T_WALL)) -ge "$CLOCK_DEADMAN_S" ]; then
    die "sim clock frozen at t_sim=$T for $((NOW_WALL - LAST_T_WALL))s wall — declaring this cell hung"
  fi
  if [ $((T - LAST_HB)) -ge 60 ]; then
    LAST_HB=$T
    # One slash-joined step and completion count per robot, in roster order.
    # Empty counts become 0: grep -c prints nothing for a missing log, which
    # would otherwise reset STALL every heartbeat.
    # (notes: hang-gate-step-counts)
    STEPS_NOW=""; COMPLETE_NOW=""
    for r in $ROBOTS; do
      s=$(grep -c "selected goal" "$OUTDIR/planner_$r.log" 2>/dev/null || true)
      c=$(grep -c "exploitation COMPLETE" "$OUTDIR/planner_$r.log" 2>/dev/null || true)
      STEPS_NOW="$STEPS_NOW${STEPS_NOW:+/}${s:-0}"
      COMPLETE_NOW="$COMPLETE_NOW${COMPLETE_NOW:+/}${c:-0}"
    done
    log "HB t_sim=$T steps($(echo $ROBOTS | tr ' ' '/'))=$STEPS_NOW complete=$COMPLETE_NOW"
    # A planner stuck re-entering PLAN keeps every process alive and the CSV
    # growing, so only the step count stalls. Not fatal once a robot has
    # finished: DONE-idle stops its steps by design.
    # (notes: hang-gate-what-stalls)
    if [ "$STEPS_NOW" = "$LAST_STEPS" ]; then
      STALL=$((STALL + 1))
      # DONE_PAT must match only the completion prefixes the node prints. Empty
      # grep output counts as 0 (missing log); do not append echo 0, which
      # doubles the output. Any robot DONE disarms the gate.
      # (notes: hang-gate-done-pattern)
      DONE_PAT="Exploration complete\|Exploration finished"
      ANY_DONE=0
      for r in $ROBOTS; do
        d=$(grep -c "$DONE_PAT" "$OUTDIR/planner_$r.log" 2>/dev/null || true)
        [ "${d:-0}" = 0 ] || { ANY_DONE=1; break; }
      done
      # The threshold is sized off the configured budgets (see HANG_HB), not
      # observed stalls: aborting a valid cell is worse than a gate that never
      # fires. (notes: hang-gate-false-abort-calibration)
      if [ "$STALL" -ge "$HANG_HB" ] && [ "$ANY_DONE" = 0 ]; then
        die "HUNG: no planner advanced a step in $((STALL * 60)) sim-s \
(steps still $STEPS_NOW) and none reports DONE. Run is invalid — check \
'rejected' counts in $OUTDIR/planner_*.log (cost_grid_radius_cap_m=$COST_CAP)."
      fi
      [ "$STALL" -ge 2 ] && log "WARNING no step progress for $((STALL * 60)) sim-s"
    else
      STALL=0
    fi
    LAST_STEPS=$STEPS_NOW
  fi
  if [ "$STOP_ON_DONE" = "1" ]; then
    # all_done was computed above, before the clock read, so that a fast poll
    # can detect the edge without a ros2 spawn.
    if [ "$all_done" = 1 ]; then
      # Re-armed on any robot leaving DONE, so a target release or a manoeuvre
      # that pulls one back out restarts the grace rather than banking it.
      [ "$DONE_SINCE" = -1 ] && { DONE_SINCE=$T; log "all planners DONE at t_sim=$T — holding ${DONE_GRACE_S}s for backlog drain"; }
      if [ $((T - DONE_SINCE)) -ge "$DONE_GRACE_S" ]; then
        RUN_END_REASON="all_done"
        DONE_DRAIN_COMPLETE=1
        log "run complete: every planner DONE (makespan t_sim=$((DONE_SINCE - T0)) s)"
        break
      fi
    elif [ "$DONE_SINCE" != -1 ]; then
      log "a planner left DONE at t_sim=$T — grace re-armed"
      DONE_SINCE=-1
    fi
  fi
  # Horizon with every planner already DONE: reason stays all_done (DONE_SINCE
  # may be past it). A cut drain goes in done_drain_complete, never a third
  # reason: gate_g8.py, modes_compare.py, reconnect_value.py accept two.
  # (notes: horizon-after-all-done)
  if [ "$DURATION_S" != "0" ] && [ "$T" -ge $((T0 + DURATION_S)) ]; then
    if [ "$DONE_SINCE" != -1 ]; then
      RUN_END_REASON="all_done"
      DONE_DRAIN_ELAPSED=$((T - DONE_SINCE))
      if [ "$DONE_DRAIN_ELAPSED" -ge "$DONE_GRACE_S" ]; then
        DONE_DRAIN_COMPLETE=1
      else
        DONE_DRAIN_COMPLETE=0
      fi
      log "horizon reached at t_sim=+$((T - T0))s, but every planner was already DONE at t_sim=$((DONE_SINCE - T0)) s — the drain grace, not the run, overran (drain ${DONE_DRAIN_ELAPSED}/${DONE_GRACE_S}s, complete=${DONE_DRAIN_COMPLETE})"
    else
      RUN_END_REASON="censored_at_T"
    fi
    break
  fi
done
# Which exit fired is data, not logging: "every robot finished by t" and
# "still unfinished when the horizon cut it off" are different observations and
# the analysis must not average them together.
echo "run_end_reason=${RUN_END_REASON:-censored_at_T}" >> "$OUTDIR/run_manifest.txt"
# T0 is wall-paced, so pre-T0 sim time varies with the real-time factor, and the
# DONE_GRACE_S drain is included. Several scripts read this, but it is not the
# endpoint. (notes: run-end-t-sim-bias)
echo "run_end_t_sim=$((T - T0))" >> "$OUTDIR/run_manifest.txt"
# Endpoint from each planner's run_end event (t_rel_sec from its own t0, no
# bring-up offset), per robot and as the max over robots. Missing or unparseable
# values are written empty, never 0. (notes: run-end-t-rel-endpoint)
_tmax=""
for r in $ROBOTS; do
  _tr=$(sed -n 's/.*"event"[[:space:]]*:[[:space:]]*"run_end".*"t_rel_sec"[[:space:]]*:[[:space:]]*\([0-9.eE+-]*\).*/\1/p' \
        "$OUTDIR/$r.events.jsonl" 2>/dev/null | tail -1)
  case "$_tr" in
    ''|*[!0-9.eE+-]*) _tr="" ;;
  esac
  echo "run_end_t_rel_sec_$r=$_tr" >> "$OUTDIR/run_manifest.txt"
  if [ -n "$_tr" ]; then
    _tmax=$(awk -v a="${_tmax:-}" -v b="$_tr" 'BEGIN{ if (a == "") print b; else print (b+0 > a+0 ? b : a) }')
  fi
done
echo "run_end_t_rel_sec_max=$_tmax" >> "$OUTDIR/run_manifest.txt"
unset _tmax _tr
# 1 = the full DONE_GRACE_S drain elapsed; 0 = the horizon cut it short; empty =
# never all-DONE. Always written, so an absent key means an old manifest.
# (notes: done-drain-complete-key)
echo "done_drain_complete=${DONE_DRAIN_COMPLETE:-}" >> "$OUTDIR/run_manifest.txt"
# run_end_wall_sec is the whole cell including bring-up;
# run_end_wall_sec_since_t0 excludes it (divide by run_end_t_sim for a real-time
# factor). Neither measures planner CPU. Absent keys mean an old manifest.
# (notes: run-end-wall-cost)
echo "run_end_wall_sec=$SECONDS" >> "$OUTDIR/run_manifest.txt"
echo "run_end_wall_sec_since_t0=$((SECONDS - T0_WALL))" >> "$OUTDIR/run_manifest.txt"
echo "finished_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$OUTDIR/run_manifest.txt"
log "run ended at t_sim=+$((T - T0))s (${RUN_END_REASON:-censored_at_T})"
# teardown runs on EXIT
