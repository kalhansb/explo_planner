#!/usr/bin/env bash
# Host-side orchestrator for a WATCHABLE 2-robot exploration + exploitation run
# in the hmr_sim `flatforest` world, lidar-only, with RViz.
#
# This is the 2026-08-02 campaign stack (Bags/2026_08_02__flatforest_2robot_lidar
# _ablation/run_experiment.sh, condition B) with the visualization it never had:
# that campaign ran headless with no RViz and no robot model, so a run was only
# legible after the fact through the CSVs. Here you watch it live.
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
#   RECONNECT_MODE=rendezvous ./run_explo_sim_rviz.sh
#                                           # A/B: reconnection manoeuvre at
#                                           # exploration exhaustion
#                                           # (rendezvous | pursuit | hybrid)
#                                           # — see below
# Ctrl-C tears the whole stack down in reverse start order (SIGINT to each
# process group, then SIGKILL to stragglers), the same teardown the campaign
# driver used.
set -u
# Force a decimal point onto anything destined for a `ros2 ... -p x:=<v>` that
# the node declares as a double. `ros2 param` infers the type from the LITERAL,
# so `-p cost_grid_radius_cap_m:=500` is an integer and the node aborts at
# startup with InvalidParameterTypeException; `:=500.0` is what it wants. This
# bit twice: awk prints 10*50.0 as "500", and every knob below that a user might
# reasonably set as `ROI_HALF=35` feeds -p roi_min_x. Run every float-valued
# shell var through this rather than relying on each one being written with a
# decimal by hand -- the failure is a hard abort seconds into a run, and it is
# only visible in the planner log, not on the console.
flt() { case "$1" in *.*|*e*|*E*) printf '%s' "$1";; *) printf '%s.0' "$1";; esac; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # <repo>/sim
# The repo is expected checked out at …/hmr_explo/ws/src/explo_planner, so the
# ws overlay root is three levels up from this sim/ directory.
WS="$(cd "$HERE/../../.." && pwd)"           # …/hmr_explo/ws
ROBOTS="atlas bestla"
SCENARIO="flatforest_2robot_lidar.yaml"
DURATION_S="${DURATION_S:-0}"                # 0 = until Ctrl-C
RVIZ="${RVIZ:-1}"
GZ_GUI="${GZ_GUI:-0}"                        # 1 => ignition GUI as well
RECORD="${RECORD:-0}"
MAX_STEPS="${MAX_STEPS:-500}"
# FRONTIER_ONLY=1 (default) overrides shared_params.yaml's candidate_enable_polar
# to false, so EXPLORE candidates are ONLY frontier centroids — no polar grid of
# n_radial*n_rings*n_yaw = 96 samples around the robot. Exploitation is
# unaffected either way: vantages come from VantagePlanner::generateVantages(),
# not CandidateGenerator::generate().
#
# CAVEAT, from the shared_params.yaml comment that set the field default to
# polar-on: in frontier-only mode the planner has been seen to exhaust reachable
# candidates (~70) once the robot pushed into an ROI corner, then oscillate until
# the planner timeout fired. Polar exists as the fallback supply. Watch for a
# rising `candidates rejected` / repeated identical goals; FRONTIER_ONLY=0
# restores the field behaviour.
FRONTIER_ONLY="${FRONTIER_ONLY:-1}"
# Proximity-yield band, SIM ONLY. shared_params.yaml ships 5.0/6.0, sized for the
# real-robot reaction budget (peer pose age + tick + nav2 cancel propagation +
# braking at 2x0.8 m/s closing). In flatforest the two huskys work a 3-vantage
# ring whose angles sit ~3.5 m apart, so a 5 m hold disc means the robot driving
# to its angle is braked by the teammate standing on the next one — the yaml
# defaults would spend the run in PROXIMITY_HOLD. These override the params file
# for this script only; the field defaults stay untouched.
# These are doubles in the planner, and `-p x:=2` would make ros2 infer an
# integer and abort the node at startup on the type mismatch. flt() adds the
# decimal point, so `PROX_HOLD_M=2` is now accepted -- but the same trap is live
# for any NEW float knob added below that skips flt().
PROX_HOLD_M="$(flt "${PROX_HOLD_M:-1.5}")"
PROX_RESUME_M="$(flt "${PROX_RESUME_M:-2.5}")"
# DWELL_SYNC=1 (default) keeps the vantage-ring rendezvous barrier on: a robot at
# its vantage holds (dwell clock not started) until every peer claiming the same
# trunk is standing on its own angle, so the team captures one trunk state
# simultaneously. DWELL_SYNC=0 is the A/B against the old behaviour, where the
# first robot to arrive dwells alone and can close the quota before the peer
# lands. See exploit_dwell_sync_* in shared_params.yaml.
DWELL_SYNC="${DWELL_SYNC:-1}"
# EXPLOIT=0 turns the run into pure exploration: exploitation_enabled:=false and
# no target_scheduler at all. The comms/reconnection matrix REQUIRES this (plan
# §3.5) and the yaml default is true, so a matrix run left on the default would
# not be the experiment the plan describes.
#
# Why it contaminates: the scheduler releases three trunks on a sim-time
# schedule, both robots detour to each one, and the vantage ring is split
# through peer CLAIMS on /exploration/intents — the very stream COMMS=1 gates.
# Link severity would then drive exploitation stalls directly, standDownExploitation()
# would fire inside every reconnect manoeuvre, and target detours of arm-dependent
# length would land in the primary coverage endpoint. /exploration/targets is
# also a global bus the emulator does not relay, so the one piece of team
# knowledge that stays perfect under a modelled radio outage would be the target
# list. Default 1 keeps this script's watchable-demo behaviour unchanged.
EXPLOIT="${EXPLOIT:-1}"
# Mesh reconnection manoeuvre (rendezvous | pursuit | hybrid) — which of the
# robot-carried-radio reconnection methods runs when a planner exhausts its
# goals with its teammate out of comms. The pursuit budgets ride along from
# shared_params.yaml (240 s cap / 180 s staleness gate, flatforest-sized).
# The planner invocation below also passes rendezvous_expected_peers:=1
# (this is a 2-robot stack): shared_params.yaml ships 0 — the field value,
# where the launch file computes team-size-1 — and the planner hard-disables
# the whole reconnect feature at startup on expected_peers=0, which would
# leave this knob silently inert.
# NOTE: stock sim DDS is one broadcast domain, so with both planners healthy
# no manoeuvre ever runs: the first finisher parks DONE-idle and keeps
# beaconing, the second finisher counts it and goes DONE in place. The A/B
# that exercises the mode is a dead peer (kill one planner mid-run: the
# survivor's exhaustion finds the claim aged out, chases the corpse's trail,
# then falls back per mode) or, later, a range-gated intent bridge emulating
# finite comms.
#
# `off` is a FOURTH value handled here, not by the planner: it disables the
# manoeuvre outright (rendezvous_enabled:=false) and is the control arm the
# matrix compares the other three against. It cannot be expressed as a
# reconnect_mode — reconnectModeFromString() falls back to RENDEZVOUS on any
# string it does not recognise (planner_util.cpp), setting only a `known` flag
# that the node reports as a single WARN and then ignores. So passing
# `reconnect_mode:=off` would not disable anything; it would silently run the
# rendezvous arm twice and the control-vs-treatment contrast would be a
# comparison of an arm with itself. Unknown values are rejected below rather
# than quietly mapped, for the same reason.
RECONNECT_MODE="${RECONNECT_MODE:-hybrid}"
case "$RECONNECT_MODE" in
  rendezvous|pursuit|hybrid|off) ;;
  *) echo "FATAL: RECONNECT_MODE='$RECONNECT_MODE' is not one of \
rendezvous|pursuit|hybrid|off. The planner would silently fall back to \
rendezvous and the run would be mislabelled." >&2; exit 2 ;;
esac
# COMMS=1 puts the message-level radio emulator (hmr_comms_sim_node) between the
# two robots, which is what turns the NOTE above from a caveat into a runnable
# experiment: with it, "peer out of comms" is produced by distance through trees
# rather than by killing a process. It rewires two streams, and BOTH are needed —
# gating only one produces a run that looks fine and measures nothing:
#   * scovox_bin  — each merger reads its peer's map off /<self>/rx/<peer>/...
#     instead of the peer's own publisher, so map sharing obeys the link.
#   * exploration/intents — planners publish to /<robot>/exploration/intents
#     (per-robot, so the emulator has something to relay) and subscribe to the
#     relayed copies. Left on the shared global bus, no outage can ever make a
#     peer read as missing, and no reconnect manoeuvre would ever fire.
# COMMS=0 (default) leaves both on their direct topics: one broadcast domain,
# perfect comms. That is the only thing COMMS toggles — it is NOT "the 2026-08-02
# campaign unchanged". This script now also enables the 2D planning map
# (use_planning_map, off in the yaml and off in that campaign), which activates
# the candidate free/occupied filter AND the cost-grid reachability filter that
# were both entirely inactive before; overrides the ROI to a ±50 box instead of
# the yaml's field site; and samples the CSV on a timer. rejected_by_unreachable,
# rejected_by_minpos and goal selection therefore do not mean the same thing they
# did in that campaign's CSVs, and the two are not directly comparable.
COMMS="${COMMS:-0}"
# Link fading is a pure function of (seed, tick), so this alone selects the run's
# link realisation. Paired-seed designs vary it while holding everything else
# fixed; it is inert with COMMS=0.
SEED="${SEED:-42}"
# Radio severity. Matches comms_sim_params.yaml's shipped value, so the default
# changes nothing — it exists so the number lands in the run manifest and can be
# swept from the command line during calibration rather than by editing the
# installed yaml, which would silently re-scope every later run and leave no
# record of which severity any given run used.
TX_POWER="$(flt "${TX_POWER:-30.0}")"
# Does this run's arm expect the link to drop? 1 = yes (every treatment arm),
# 0 = the control arm, deliberately run at a tx_power_dbm that keeps the link
# up. Only affects the run-time outage gate's verdict, never the radio itself:
# severity is TX_POWER's job alone, and the two are set independently so that a
# control run whose link DID drop is still reported rather than excused.
EXPECT_OUTAGE="${EXPECT_OUTAGE:-1}"
# Planner ROI half-extent, SIM ONLY (square, centred on the world origin).
# shared_params.yaml carries the real field site's ROI — x ∈ [-51.3, 100.9],
# y ∈ [-38.7, 74.5] — and on flatforest ~42% of that footprint has no geometry
# at all: those columns never leave the prior, so the unknown fraction can never
# reach done_unknown_fraction and EVERY run ends at max_steps instead of at
# coverage. That still fires one reconnect manoeuvre (the step-budget path also
# routes through finishOrRendezvous), which is why the symptom is invisible —
# what it destroys is the repeat reconnect -> re-disperse cycles the experiment
# needs. ±50 fits inside the 110x110 ground plane and covers the oak field.
# ±50 also fits inside the global planning map derived from it just below.
ROI_HALF="$(flt "${ROI_HALF:-50.0}")"
# Side of scovox_node's WORLD-FIXED planning map (~/global_planning_map) — the
# 2D map the EXPLORATION planner consults for free/occupied and reachability.
# Both it and the ROI are centred on the world origin, so side 2*ROI_HALF
# exactly covers the ROI and 3*ROI_HALF leaves ROI_HALF/2 of margin on each
# side. The margin is not cosmetic: candidates are ROI-clipped but the ROBOT is
# not, and the cost-grid flood starts from the robot's own cell — a robot that
# has drifted past the ROI edge onto an out-of-bounds cell floods nothing and
# every candidate is rejected as unreachable. Derived from ROI_HALF so the two
# cannot drift apart.
#
# This is NOT /<r>/scovox_node/planning_map. That one is a 20 m robot-centred
# crop whose extent IS simple_nav_3d's local-planner window, and the exploration
# planner rejects any candidate whose cell is out of bounds (isCellOccupied
# treats out-of-bounds as occupied) — on the rolling map every frontier beyond
# ~10 m is dropped and exploration collapses to a bubble around the robot.
# Widening that topic instead would push the whole world through the local A*
# on the control path. dscovox_node publishes no planning_map at all (only a
# GetOccupancyGrid service), so the planner's own default topic —
# /<r>/dscovox_node/planning_map — has no publisher anywhere in this stack.
PLAN_MAP_SIZE="$(flt "${PLAN_MAP_SIZE:-$(awk "BEGIN{print 3*$ROI_HALF}")}")"
PLAN_MAP_RES="$(flt "${PLAN_MAP_RES:-0.40}")"
# Dijkstra flood radius for the candidate reachability filter. MUST be set here,
# and this is the trap that comes with switching the planning map on.
#
# The filter is dead code with use_planning_map=false: no map means no cost grid
# to flood, so doPlan sets skip_reachability and every candidate passes. Turning
# the map on activates it — with a cap that has therefore never been exercised.
# The yaml ships cost_grid_radius_cap_m: 0.0 = auto = candidate_max_radius + 2 =
# 10 m, a bound sized for POLAR candidates, which are generated within
# candidate_max_radius by construction. Frontier centroids have no range limit
# at all (addFrontierCandidates clips to the ROI and nothing else), and
# FRONTIER_ONLY=1 removes the polar set entirely — so under the auto cap every
# frontier more than 10 m of walked distance away comes back kInfCost and is
# rejected as unreachable. The escape hatch does not save it either: it only
# trips below 10 reached cells, and a 10 m flood at 0.4 m/cell reaches ~1900.
# Exploration would degenerate to 10 m hops, or stall outright once no frontier
# remains inside the disc — and it would look like a legitimate result.
#
# 10x ROI_HALF is deliberately far past the grid diagonal ($PLAN_MAP_SIZE * 1.41
# = 212 m at the defaults): the intent is "unbounded within this grid", and a
# cap must not be clamped to the diagonal because a walked Dijkstra distance
# routinely exceeds the straight line (see the warning in CostGrid::floodFrom).
# Passing 0 to mean unbounded does NOT work — 0 is the auto sentinel. The cost
# is one full flood of a ~140k-cell grid per PLAN tick, which the exploitation
# planner already pays on the same grid with a genuinely unbounded flood.
COST_CAP="$(flt "${COST_CAP:-$(awk "BEGIN{print 10*$ROI_HALF}")}")"
# Minimum range to an acceptable EXPLORE candidate. The yaml ships 0.0 (off,
# preserving field behaviour) and this script overrides it, because on
# flatforest 0.0 does not merely explore inefficiently — it deadlocks.
#
# Measured, not theorised: at 0.0 both robots entered a two-point oscillation
# (atlas ping-ponging between goals 0.87 m apart, bestla 0.63 m) and stayed
# there. Steps kept advancing, every process stayed healthy, the CSV kept
# filling, and the unknown fraction flatlined at 0.87 having moved 0.001 in the
# last 100 sim-seconds. Nothing in the stack reports it: the harness hang gate
# watches the STEP counter, and the steps were fine.
#
# Cause is in the planner's utility, not here — see candidate_min_goal_dist_m
# in shared_params.yaml. 4.0 m sits well inside fov_max_range (10.0), so a hop
# still lands deep in previously-seen space rather than jumping blind.
#
# This is a real scenario correction in the sense of plan §2: it changes what
# every arm does, so it must be identical across arms and no run from before it
# may be pooled with one after.
MIN_GOAL_DIST="$(flt "${MIN_GOAL_DIST:-4.0}")"
# Frontier search band, as an inset from the ROI z band. The yaml ROI band is
# [-5.5, +4.0] (sized for the field site's terrain and canopy), so these insets
# put the frontier search in absolute z [0.2, +1.5] on flatforest's flat ground
# — about the slice a Husky's VLP-16 actually sweeps as it drives.
#
# MIN_GOAL_DIST alone does NOT fix the oscillation, which is why both exist:
# with the full 9.5 m band the near-frontier set is unobservable-by-construction
# (the lidar's free-space wedge has a top and bottom surface at every range and
# those surfaces are frontier forever), so raising the minimum hop just moves
# the ping-pong out to that radius. Measured: at MIN_GOAL_DIST=4.0 with the
# full band, atlas oscillated between two goals 4.3 m apart instead of 0.87 m
# apart. The band is what makes a frontier something driving can consume; the
# minimum hop is what stops the planner picking a goal too close to observe
# from. Verify from the planner log line "Frontier search band inset by ...",
# which prints the ROI band it was applied to.
FRONTIER_Z_LO_OFF="$(flt "${FRONTIER_Z_LO_OFF:-5.7}")"
FRONTIER_Z_HI_OFF="$(flt "${FRONTIER_Z_HI_OFF:-2.5}")"
# Recently-visited goal suppression. The yaml ships 0.0 (off, field behaviour);
# this scenario needs it, and it is the third and last piece of the same defect.
# MIN_GOAL_DIST stops the planner picking a goal too close to observe from, the
# frontier band stops it chasing frontiers no sensor can ever consume, and this
# stops it going back to where it just was -- which it otherwise does forever,
# because a forest's trunk shadows keep regenerating frontier clusters and
# utility reduces to "nearest" once info gain is flat.
#
# 6.0 m is just above frontier_cluster_radius_m (5.0), so suppressing a reached
# goal suppresses the cluster that produced it, not a point inside it. The TTL
# must outlive a there-and-back hop at these distances; 180 s is ~15 steps at
# the observed cadence, and it expires rather than persisting so that an area
# genuinely re-frontiered later in the run can still be revisited.
VISITED_RADIUS="$(flt "${VISITED_RADIUS:-6.0}")"
VISITED_TTL="$(flt "${VISITED_TTL:-180.0}")"
# scovox voxel edge length. The launch default is 0.10, which does not survive a
# run long enough to reach coverage termination: free space is carved along the
# whole ray to max_range 20 m, so the fused map grew to 12.7M voxels by t=550 s
# and the planner's map ingest fell behind for good — one robot then drove for
# three minutes on a frozen map, still stepping, still logging, coverage flat,
# with nothing in the stack reporting it. 0.20 is ~8x cheaper.
#
# NOT a free knob under COMMS=1: the ScovoxMapBinary deltas are exactly what the
# radio model carries, so this sets the offered load the link is stressed with.
# The §4 severity calibration must be run at the resolution the campaign runs
# at; a tx_power_dbm calibrated at another resolution does not transfer.
VOXEL_RES="$(flt "${VOXEL_RES:-0.20}")"
OUTDIR="${OUTDIR:-/tmp/explo_sim_$(date +%Y%m%d_%H%M%S)}"
# Own DDS domain, NOT the default 0. This box runs other ROS work (the scovox
# replay harnesses) on domain 0, and a second /clock publisher appearing there
# mid-run would corrupt every use_sim_time consumer in this stack — sim time is
# not something a run can recover from. Every child inherits this, RViz and the
# script's own `ros2 topic echo` gates included; to poke at the run by hand,
# export the same value first:  export ROS_DOMAIN_ID=42
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Per-robot viz mesh set. Both robots spawn as COSTAR_HUSKY_SENSOR_CONFIG_LIDAR
# in the scenario, but that model is one colour — RViz uses the yellow/green
# variants' meshes (same geometry, painted differently) so the two robots are
# telling apart at a glance.
declare -A VIZ_MODEL=(
  [atlas]=COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_YELLOW
  [bestla]=COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_GREEN
)

mkdir -p "$OUTDIR"
log() { echo "[$(date +%H:%M:%S)] $*"; }
# The other robot in a 2-robot stack. Everything COMMS=1 rewires is per-link, so
# the wiring needs to name the far end; this is deliberately only correct for
# |ROBOTS| == 2, which is what this harness is.
peer_of() { local s=$1 p; for p in $ROBOTS; do [ "$p" = "$s" ] || { echo "$p"; return; }; done; }

# --- environment: humble + ws overlay, miniconda stripped -------------------
# (miniconda on PATH shadows /usr/bin/python3 and breaks catkin_pkg/ament)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV || true
# Snap scrub — REQUIRED when this is launched from the VS Code snap's integrated
# terminal (or any snap-hosted shell). That environment exports GTK/GDK/glib
# paths pointing inside /snap, and a system rviz2 started under it loads the
# snap's GTK stack, which drags in /snap/core20's libpthread and dies at startup
# with "undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE". Verified:
# stripping these makes rviz2 start with hardware GL. Harmless in a plain shell,
# where none of them are set.
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
  ps -eo pid,cmd | grep -E "ign gazebo|explo_planner_node|target_scheduler_node|dscovox_node|scovox_node|scovox_mapping_node|dscovox_mapping_node|hmr_comms_sim_node|simple_nav|robot_state_publisher|sim_tf_publisher|sim_target_markers|rosbag2|parameter_bridge|ros_gz|rviz2" \
    | grep -v grep | grep -v claude | grep -v run_explo_sim_rviz
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
  # Final gate verdict. The watcher is a background child killed by the loop
  # above, so its exit status is unreachable here — the report file it appends
  # to is the only durable record, and until it is read out the run ends looking
  # successful whatever the gates found at minute 40. Includes the outage gate,
  # which can only be decided at the end: a COMMS=1 run whose link never dropped
  # is a control run wearing a treatment label, and nothing before this point
  # can tell.
  if [ "$COMMS" = "1" ] && [ -f "$OUTDIR/comms_gates.txt" ]; then
    NFAIL=$(grep -c "^FAIL" "$OUTDIR/comms_gates.txt" 2>/dev/null || true)
    NUNRUN=$(grep -c "^UNRUN" "$OUTDIR/comms_gates.txt" 2>/dev/null || true)
    if [ "${NFAIL:-0}" != 0 ]; then
      log "=============================================================="
      log "RUN INVALID: $NFAIL gate failure(s). $OUTDIR/comms_gates.txt:"
      grep "^FAIL" "$OUTDIR/comms_gates.txt" | sed 's/^/    /' || true
      log "=============================================================="
    elif [ "${NUNRUN:-0}" != 0 ]; then
      log "RUN SUSPECT: $NUNRUN gate(s) could not be evaluated — see $OUTDIR/comms_gates.txt"
    else
      log "comms gates: clean for the whole run"
    fi
  fi
  log "teardown complete — outputs in $OUTDIR"
}
trap teardown EXIT INT TERM
die() { log "ERROR $*"; exit 1; }   # trap runs teardown

# wait_for <seconds> <what> -- <cmd...>
# Retry <cmd> until it succeeds or <seconds> of WALL CLOCK have elapsed.
#
# Deadline-based, not attempt-based, and that is the whole point. The gates
# below used `for i in $(seq 1 24); do timeout 8 ros2 topic echo … --once; done`
# and described themselves as waiting 3 minutes. They did not. `ros2 topic echo`
# only blocks for its timeout once the topic's TYPE is resolvable — i.e. once a
# publisher already exists. Before that it prints "does not appear to be
# published yet / Could not determine the type" and returns rc=1 in ~0.3 s, so
# all 24 attempts burned in ~8 s and the run died on a sim that was still
# spawning entities. Same trap with `ros2 topic list`, which always returns
# promptly. Any readiness gate built from a fast-failing probe needs an explicit
# sleep or a deadline; counting attempts silently makes the budget a function of
# how the probe fails.
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
log "=== flatforest 2-robot lidar explore+exploit, RViz=$RVIZ gui=$GZ_GUI ==="
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  (export the same value to inspect by hand)"
log "targets: $TARGETS"
log "outputs: $OUTDIR"
PRE=$(stack_procs || true)
[ -n "$PRE" ] && { echo "$PRE"; TORN=1; die "stack processes already running — refusing to start"; }
if [ "$RVIZ" = "1" ]; then
  export DISPLAY="${DISPLAY:-:1}"
  log "RViz will use DISPLAY=$DISPLAY"
fi

# --- 1. simulator (2 robots, lidar-only models) -----------------------------
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
  # The URDF goes in through a params FILE, never `-p robot_description:=<xml>`:
  # rcl rejects a parameter override rule whose value contains newlines
  # ("Couldn't parse parameter override rule", arguments.c:343) and
  # robot_state_publisher aborts before it starts. A YAML literal block scalar
  # takes the XML verbatim, colons and quotes in its comments included.
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
# BEFORE the mappers, and the order is load-bearing. Relay subscriptions are
# created by a 1 Hz topic-discovery poll, and only once the source topic already
# exists — so an emulator started after scovox_node misses everything published
# in between, including the initial full map snapshot. That loss is invisible:
# it happens upstream of the relay, so no drop statistic counts it, and the run
# just quietly begins with each robot missing the other's first map. Started
# here, the subscription is in place ~1 s after each publisher appears, well
# before scovox_node has integrated enough lidar to emit anything.
# Robot names and the tree-bearing world SDF come from the same scenario file
# the sim was launched with; the /rx/ readiness check is deferred until after
# the mappers are up, since that is when the relays can first form.
if [ "$COMMS" = "1" ]; then
  start comms "$OUTDIR/comms.log" \
    ros2 launch hmr_sim comms_sim.launch.py \
      scenario:="$SCENARIO" seed:="$SEED" use_sim_time:=true \
      tx_power_dbm:="$TX_POWER"
  sleep 3
  log "comms emulator started (seed=$SEED tx_power_dbm=$TX_POWER) ahead of the mappers"
fi

# --- 3. nav + lidar mapping, mergers cross-wired ----------------------------
# mapping:=dscovox_lidar => per robot a scovox_node on /<r>/velodyne_points plus
# a dscovox_node merging BOTH robots' scovox_bin streams, so each planner reads
# its own copy of the TEAM map.
# With COMMS=1 each merger takes its PEER's binary off the emulator's relayed
# copy instead of the peer's own publisher. Self is untouched by the pattern —
# a robot's own map never crosses a radio link.
PEER_BIN_PATTERN="/{peer}/scovox_node/scovox_bin"
[ "$COMMS" = "1" ] && PEER_BIN_PATTERN="/{self}/rx/{peer}/scovox_node/scovox_bin"
log "peer_bin_topic_pattern=$PEER_BIN_PATTERN (COMMS=$COMMS)"
start nav_atlas "$OUTDIR/nav_atlas.log" \
  ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv \
    mapping:=dscovox_lidar peers:=bestla \
    peer_bin_topic_pattern:="$PEER_BIN_PATTERN" \
    voxel_resolution_m:=$VOXEL_RES \
    global_planning_map_size_m:=$PLAN_MAP_SIZE \
    global_planning_map_resolution:=$PLAN_MAP_RES
start nav_bestla "$OUTDIR/nav_bestla.log" \
  ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=bestla mode:=ugv \
    mapping:=dscovox_lidar peers:=atlas \
    peer_bin_topic_pattern:="$PEER_BIN_PATTERN" \
    voxel_resolution_m:=$VOXEL_RES \
    global_planning_map_size_m:=$PLAN_MAP_SIZE \
    global_planning_map_resolution:=$PLAN_MAP_RES
for r in $ROBOTS; do
  wait_for 180 "$r planning_map" -- \
    timeout 8 ros2 topic echo /$r/scovox_node/planning_map --once \
    || die "nav: no /$r planning_map after 3 min (see $OUTDIR/nav_$r.log)"
  log "$r mapping alive (planning_map publishing)"
  # The exploration planner's map is a SEPARATE publisher and it is a hard
  # precondition: with use_planning_map=true the planner sits in its start-up
  # wait until one arrives, so a typo in the topic name or a launch arg that
  # silently defaulted to 0.0 (= disabled) would present as two planners that
  # never leave INIT — an hour into a run, with no error anywhere. Gate on it
  # here instead. Both publishers are transient_local and only emit when
  # someone is subscribed, so this echo is also what pulls the first sample.
  wait_for 120 "$r global_planning_map" -- \
    timeout 10 ros2 topic echo /$r/scovox_node/global_planning_map --once \
    || die "nav: no /$r global_planning_map after 2 min — the \
exploration planner will never leave INIT (see $OUTDIR/nav_$r.log)"
  log "$r global_planning_map alive (${PLAN_MAP_SIZE}m @ ${PLAN_MAP_RES}m/cell)"
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
# Under COMMS=1 the interesting streams are the RELAYED ones: /exploration/intents
# is empty (the planners moved to per-robot topics) and the peers' direct
# scovox_bin publishers still carry everything, so a bag of only the pre-relay
# topics would show perfect comms no matter what the link did. Add the link
# states too — they are the ground truth for when each link was up.
COMMS_BAG_TOPICS=()
if [ "$COMMS" = "1" ]; then
  for r in $ROBOTS; do
    p=$(peer_of "$r")
    COMMS_BAG_TOPICS+=( "/$r/exploration/intents"
                        "/$r/rx/$p/exploration/intents"
                        "/$r/rx/$p/scovox_node/scovox_bin" )
  done
  # /hmr_comms_sim/stats carries backlog_bytes and the drop_* counters
  # (drop_airtime, drop_ber, drop_overflow). Those are the ONLY evidence for
  # the backlog gate and for the shared-airtime confound — a single global
  # token bucket means one large map delta can drive the pool negative and drop
  # every best-effort intent on every link, precisely during the post-reconnect
  # drain when peer presence has to be re-detected. Unrecorded, that mechanism
  # is unfalsifiable after the fact: the gate watcher samples it live but only
  # writes a sample out when it fails.
  COMMS_BAG_TOPICS+=( /hmr_comms_sim/link_states /hmr_comms_sim/robot_index
                      /hmr_comms_sim/stats )
fi
if [ "$RECORD" != "0" ]; then
  # RECORD=1 is the full set — everything needed to reconstruct or re-watch a
  # run. RECORD=2 is the LEAN set: only what the plan's offline analysis
  # consumes, which is the §4 calibration replay (both odom + /clock), the §5.1
  # oracle re-merge (sender-side scovox_bin), and the link/airtime diagnostics.
  #
  # The difference is not marginal. The two OccupancyGrids dominate the full
  # set: global_planning_map alone is (size/res)^2 bytes per message — 375x375 =
  # ~140 kB at the ROI_HALF=50 defaults — published per robot at ~1 Hz, so it
  # outweighs everything the analysis actually reads. A full matrix of 30-40
  # runs at full RECORD does not fit on this box; the lean set does. Neither
  # mode records the RELAYED /rx/ copies: the oracle merge is defined on what
  # each robot SENT, and what arrived is already reconstructable from
  # link_states plus the receivers' own CSVs.
  BAG_TOPICS=( /clock /tf_static
               /atlas/odom_ground_truth /atlas/scovox_node/scovox_bin
               /bestla/odom_ground_truth /bestla/scovox_node/scovox_bin
               /exploration/intents )
  if [ "$RECORD" = "1" ]; then
    BAG_TOPICS+=( /tf
                  /atlas/imu/data /atlas/cmd_vel
                  /atlas/scovox_node/planning_map
                  /atlas/scovox_node/global_planning_map
                  /atlas/goal_pose /atlas/explo_planner/candidates
                  /bestla/imu/data /bestla/cmd_vel
                  /bestla/scovox_node/planning_map
                  /bestla/scovox_node/global_planning_map
                  /bestla/goal_pose /bestla/explo_planner/candidates
                  /exploration/targets )
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
# Same parameter set as exploitation_experiment.launch.py, namespaced so the two
# instances don't collide on node name / ~/candidates / proximity_hold_state.
# terrain_relative_z:=false is REQUIRED on flatforest: in terrain mode a vantage
# sits at MAPPED ground + clearance, and the ground on the far side of a trunk is
# exactly what the trunk occludes — those vantages are rejected `noground` and
# targets close PARTIAL with the ring half-driven. The world is a ground_plane at
# z=0, so the fixed absolute height is the same number anyway.
POLAR_ARG="true"; [ "$FRONTIER_ONLY" = "1" ] && POLAR_ARG="false"
DWELL_SYNC_ARG="true"; [ "$DWELL_SYNC" = "0" ] && DWELL_SYNC_ARG="false"
# The control arm. `off` is not a reconnect_mode (see the RECONNECT_MODE block);
# it is rendezvous_enabled:=false, which is the switch the planner actually
# gates the manoeuvre on. reconnect_mode is left at its yaml value in that case
# and is inert, since shouldRendezvous() returns false before the mode is
# consulted.
RDV_ENABLED="true"; MODE_ARG="$RECONNECT_MODE"
if [ "$RECONNECT_MODE" = "off" ]; then RDV_ENABLED="false"; MODE_ARG="hybrid"; fi
log "candidate_enable_polar=$POLAR_ARG (FRONTIER_ONLY=$FRONTIER_ONLY)"
log "proximity_hold/resume_dist_m=$PROX_HOLD_M/$PROX_RESUME_M m (yaml field defaults 5.0/6.0 overridden for sim)"
EXPLOIT_ARG="true"; [ "$EXPLOIT" = "0" ] && EXPLOIT_ARG="false"
log "exploitation_enabled=$EXPLOIT_ARG (EXPLOIT=$EXPLOIT)"
log "exploit_dwell_sync_enabled=$DWELL_SYNC_ARG (DWELL_SYNC=$DWELL_SYNC)"
log "reconnect_mode=$MODE_ARG rendezvous_enabled=$RDV_ENABLED (RECONNECT_MODE=$RECONNECT_MODE)"
log "roi x,y = [-$ROI_HALF, $ROI_HALF] (sim override; yaml carries the field site's ROI)"
# done_coverage_source is pinned to scovox, NOT left on "auto". auto switches to
# the 2D planning_map the instant one is received, so enabling the planning map
# would have silently swapped the termination metric from 2.5D column coverage
# of the ROI to 2D cell coverage — a different number against the same
# done_unknown_fraction threshold, changing when every run ends and invalidating
# any comparison with runs recorded before this change.
log "planning_map = /<r>/scovox_node/global_planning_map (${PLAN_MAP_SIZE}m @ ${PLAN_MAP_RES}m/cell), done_coverage_source=scovox"

# --- 6a. run manifest -------------------------------------------------------
# Everything that distinguishes this run from another one, written INTO the run
# directory. Until now the arm, the seed and the radio severity existed only as
# log() lines on the harness's own stdout — so across a 40-run matrix the label
# for each run lived in the operator's scrollback and nowhere else, and two runs
# that differed only in RECONNECT_MODE were indistinguishable after the fact.
# Git hashes with a dirty marker matter just as much: every repo here has
# uncommitted changes, so a bare commit id would be an actively misleading
# provenance record.
MANIFEST="$OUTDIR/run_manifest.txt"
{
  echo "# hmr_explo comms/reconnection run manifest"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "host=$(hostname)"
  echo "outdir=$OUTDIR"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
  echo
  echo "# --- arm / independent variables ---"
  echo "reconnect_mode_requested=$RECONNECT_MODE"
  echo "reconnect_mode_param=$MODE_ARG"
  echo "rendezvous_enabled=$RDV_ENABLED"
  echo "comms=$COMMS"
  echo "seed=$SEED"
  echo "tx_power_dbm=$TX_POWER"
  # The arm's label for the outage gate, recorded because it is a claim about
  # what this run was FOR, not something recoverable from tx_power_dbm alone.
  echo "expect_outage=$EXPECT_OUTAGE"
  echo
  echo "# --- held fixed ---"
  echo "scenario=$SCENARIO"
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
  echo "frontier_z_lo_offset_m=$FRONTIER_Z_LO_OFF"
  echo "frontier_z_hi_offset_m=$FRONTIER_Z_HI_OFF"
  echo "visited_goal_radius_m=$VISITED_RADIUS"
  echo "visited_goal_ttl_sec=$VISITED_TTL"
  echo "voxel_resolution_m=$VOXEL_RES"
  echo "done_coverage_source=scovox"
  echo "prox_hold_m=$PROX_HOLD_M"
  echo "prox_resume_m=$PROX_RESUME_M"
  echo "targets=$TARGETS"
  echo "record=$RECORD"
  echo
  # Number of trunks the radio model can actually see. Recorded because it is
  # NOT recoverable from the commit sha while a change is uncommitted: two runs
  # either side of a fix to the tree matcher carry byte-identical "<sha>-dirty"
  # provenance but different link statistics. link_states col 3 gives per-link
  # counts, never the world total.
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
} > "$MANIFEST"
log "run manifest written: $MANIFEST"
for r in $ROBOTS; do
  # COMMS=1 splits the intent stream: publish to a per-robot topic the emulator
  # can see, subscribe to the relayed copy of the peer's. Both defaults are
  # empty, so with COMMS=0 nothing is passed and the planner keeps using the one
  # shared /exploration/intents bus exactly as before.
  EXTRA=()
  if [ "$COMMS" = "1" ]; then
    peer=$(peer_of "$r")
    EXTRA=( -p coord_intent_pub_topic:=exploration/intents
            -p coord_intent_sub_topics:="[\"rx/$peer/exploration/intents\"]" )
    log "$r intents: pub /$r/exploration/intents  sub /$r/rx/$peer/exploration/intents"
  fi
  start planner_$r "$OUTDIR/planner_$r.log" \
    ros2 run explo_planner explo_planner_node --ros-args \
      -r __ns:=/$r -r __node:=explo_planner \
      --params-file "$PLANNER_SHARE/config/shared_params.yaml" \
      -p use_sim_time:=true -p robot_name:=$r -p max_steps:=$MAX_STEPS \
      -p terrain_relative_z:=false \
      -p candidate_enable_polar:=$POLAR_ARG \
      -p proximity_hold_dist_m:=$PROX_HOLD_M \
      -p proximity_resume_dist_m:=$PROX_RESUME_M \
      -p exploit_dwell_sync_enabled:=$DWELL_SYNC_ARG \
      -p reconnect_mode:=$MODE_ARG \
      -p rendezvous_enabled:=$RDV_ENABLED \
      -p exploitation_enabled:=$EXPLOIT_ARG \
      -p rendezvous_expected_peers:=1 \
      -p roi_min_x:=-$ROI_HALF -p roi_max_x:=$ROI_HALF \
      -p roi_min_y:=-$ROI_HALF -p roi_max_y:=$ROI_HALF \
      -p use_planning_map:=true \
      -p planning_map_topic:=/$r/scovox_node/global_planning_map \
      -p done_coverage_source:=scovox \
      -p cost_grid_radius_cap_m:=$COST_CAP \
      -p candidate_min_goal_dist_m:=$MIN_GOAL_DIST \
      -p map_resolution:=$VOXEL_RES \
      -p frontier_z_lo_offset_m:=$FRONTIER_Z_LO_OFF \
      -p frontier_z_hi_offset_m:=$FRONTIER_Z_HI_OFF \
      -p visited_goal_radius_m:=$VISITED_RADIUS \
      -p visited_goal_ttl_sec:=$VISITED_TTL \
      ${EXTRA[@]+"${EXTRA[@]}"} \
      -p output_csv:="$OUTDIR/planner_$r.csv"
done
# Scheduler last; its node name must stay target_scheduler in the root namespace
# or the targets yaml's parameter key will not match. Skipped entirely with
# EXPLOIT=0: exploitation_enabled=false already makes the planners ignore
# releases, but leaving the scheduler running would still put TreeTarget traffic
# on the ungated global /exploration/targets bus during a comms run.
if [ "$EXPLOIT" = "0" ]; then
  log "EXPLOIT=0 — pure exploration, target_scheduler NOT started"
else
  start sched "$OUTDIR/sched.log" \
    ros2 run explo_planner target_scheduler_node --ros-args \
      -r __node:=target_scheduler \
      --params-file "$TARGETS" -p use_sim_time:=true
fi
sleep 8
# Count real node binaries only, by the absolute install path that ONLY the
# launched binary carries. Matching the bare node name instead counts anything
# whose cmdline merely mentions it: the `ros2 run` python wrapper (which is why
# "bin/ros2" used to be filtered out), but also any shell watching this run from
# another terminal — a `pgrep -f explo_planner_node` in a sibling process aborts
# the whole stack here, 8 seconds after a healthy start.
# The leading [l] is load-bearing, not a typo: ps and grep run concurrently in
# this pipeline, so ps lists the grep too. A literal pattern matches the grep's
# OWN cmdline and every count comes back one too high. Bracketing one character
# makes the pattern text differ from the text it matches.
NPLAN=$(ps -eo cmd= | grep -c "[l]ib/explo_planner/explo_planner_node")
[ "$NPLAN" = 2 ] || die "expected exactly 2 explo_planner_node, found $NPLAN"
log "planners up (exactly 2 explo_planner_node)"

# --- 6b. comms gates (COMMS=1) ----------------------------------------------
# Run AFTER the planners, because two of the four can only be judged once the
# intent endpoints exist. GATES_STRICT=1 aborts the run on a failed bring-up
# check; the default reports and continues, because a leak found at t=0 is
# still worth seeing the run for. The watcher keeps polling overflow and the
# odom watchdog for the whole run and its verdict lands in the report file —
# an overflow at minute 40 invalidates the run just as surely as one at t=0.
# Strict by default whenever the emulator is in the loop (inert with COMMS=0).
# Every gate here detects a condition the plan treats as run-invalidating —
# intent leakage past the radio model, a QoS mismatch that silently forms no
# relay, reliable-backlog overflow, a dead odom source. Continuing past one does
# not produce a degraded run, it produces a run that measures the wrong thing
# while looking healthy, and an hour of sim time is more expensive than a
# restart. GATES_STRICT=0 still forces the old report-and-continue behaviour.
GATES_STRICT="${GATES_STRICT:-$COMMS}"
if [ "$COMMS" = "1" ]; then
  GATE_REPORT="$OUTDIR/comms_gates.txt"
  ROBOT_CSV=$(echo $ROBOTS | tr ' ' ',')
  log "running comms bring-up gates (report: $GATE_REPORT)"
  # Capture the GATE's status, not the pipeline's. `if cmd | tee ...; then`
  # tests tee's exit status, which is 0 unless the disk fills — so every gate
  # failure announced itself as success and GATES_STRICT was dead code. This
  # script runs `set -u` without `pipefail`, and adding pipefail globally would
  # change the failure semantics of every other pipeline in here (several
  # legitimately end in `|| true` grep counts), so the status is taken directly
  # instead.
  python3 "$HERE/comms_gates.py" check --robots "$ROBOT_CSV" \
      --report "$GATE_REPORT" >"$OUTDIR/gates_check.out" 2>&1
  GATE_RC=$?
  cat "$OUTDIR/gates_check.out" | tee -a "$OUTDIR/gates.log"
  if [ "$GATE_RC" = 0 ]; then
    log "comms gates: all clear"
  else
    log "WARNING comms gates FAILED (rc=$GATE_RC) — see $GATE_REPORT"
    [ "$GATES_STRICT" = "1" ] && die "comms gates failed rc=$GATE_RC (GATES_STRICT=1)"
  fi
  # EXPECT_OUTAGE=0 for the plan's phase-1 control arm: it runs through the
  # emulator (so the relay hop, delay_ms and rx QoS are all present) at a
  # tx_power_dbm where the link is meant to stay up, and the outage gate would
  # otherwise fail every single control run for behaving as designed.
  # Per-run connectivity trace. link_states is published at link_rate_hz and
  # kept nowhere else: the gate watcher reads the aggregate `stats` topic, and
  # recovering it from the bag afterwards means decoding a bag per run. Every
  # §5 metric that mentions the radio -- realised disconnection fraction,
  # outage durations, contact events and their attribution to a planner state
  # -- is derived from this file.
  start linklog "$OUTDIR/link_logger.log" \
    python3 "$HERE/link_logger.py" --out "$OUTDIR/link_states.csv" \
      --ros-args -p use_sim_time:=true
  start gateswatch "$OUTDIR/gates_watch.log" \
    python3 "$HERE/comms_gates.py" watch --robots "$ROBOT_CSV" \
      --report "$GATE_REPORT" \
      --expect-outage "$([ "$EXPECT_OUTAGE" = 0 ] && echo no || echo yes)"
fi

T0=$(sim_clock)
[ -n "$T0" ] || die "cannot read /clock"
log "sim t0=$T0 — targets release at +120 / +420 / +720 s of the SCHEDULER's clock"
if [ "$DURATION_S" != "0" ]; then
  log "will stop at t_sim=$((T0 + DURATION_S))"
else
  log "no DURATION_S — running until Ctrl-C"
fi

# --- 7. hold, watching component health -------------------------------------
LAST_HB=0
# Step-stall hang detector state. HANG_HB counts 60-sim-second heartbeats, so
# the default aborts after 10 minutes of sim time with no step on EITHER robot.
# Sized well above a slow step: a RETURN_NAV or PURSUE manoeuvre legitimately
# spends minutes without completing one.
HANG_HB="${HANG_HB:-10}"
# STOP_ON_DONE=1 (default) ends the run once EVERY planner is in DONE, rather
# than burning the rest of DURATION_S on two parked robots. The experiment's
# primary endpoint is a makespan (§5.2), so the moment the last robot finishes
# is the quantity — and with a matrix of 30+ runs, the difference between
# stopping there and stopping at T is most of the campaign's wall clock.
#
# DURATION_S is still the censoring horizon T and still binds: under a degraded
# radio a robot may never reach DONE, which is a censored observation, not a
# hang. Both exits are needed and they mean different things — record which one
# fired, because "ended at T" and "ended at DONE" are different data points.
#
# Read from the CSV's `state` column, not the log: DONE is a state the planner
# can also LEAVE (a target release pulls it back into the exploit sub-loop, and
# finishOrRendezvous routes through RETURN_NAV/RETURN_SYNC before it), so a
# one-shot "Exploration complete" log line is not terminal and grepping for it
# would stop the run mid-manoeuvre. The column is located by header name because
# it was appended to a schema that other scripts read positionally.
STOP_ON_DONE="${STOP_ON_DONE:-1}"
# Grace, in sim seconds, between all-DONE and teardown: lets the last metrics
# rows land, the reliable backlog drain, and the gate watcher see the final
# state. Without it the run ends inside the very merge the endpoint measures.
DONE_GRACE_S="${DONE_GRACE_S:-30}"
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
LAST_SA=-1; LAST_SB=-1; STALL=0
while true; do
  sleep 15
  for entry in "${PIDS[@]}"; do
    name=${entry%%:*}; pid=${entry##*:}
    # RViz closed by the user is a normal way to end a watch session.
    if [ "$name" = "rviz" ] && ! alive "$pid"; then
      log "RViz exited — tearing down"; exit 0
    fi
    alive "$pid" || die "$name (pid $pid) died mid-run — see $OUTDIR/$name.log"
  done
  T=$(sim_clock)
  [ -n "$T" ] || { log "WARN /clock read failed, retrying"; continue; }
  if [ $((T - LAST_HB)) -ge 60 ]; then
    LAST_HB=$T
    SA=$(grep -c "selected goal" "$OUTDIR/planner_atlas.log" 2>/dev/null || true)
    SB=$(grep -c "selected goal" "$OUTDIR/planner_bestla.log" 2>/dev/null || true)
    CA=$(grep -c "exploitation COMPLETE" "$OUTDIR/planner_atlas.log" 2>/dev/null || true)
    CB=$(grep -c "exploitation COMPLETE" "$OUTDIR/planner_bestla.log" 2>/dev/null || true)
    log "HB t_sim=$T steps(atlas/bestla)=$SA/$SB complete=$CA/$CB"
    # Hang gate. A planner that cannot find an acceptable candidate returns from
    # doPlan and re-enters PLAN forever: no crash, no error, every process
    # alive, and the CSV keeps growing because the metrics timer samples every
    # metrics_period_sec regardless of state. So neither the liveness loop above
    # nor "is the CSV still being written" can see it — the only quantity that
    # actually stalls is the STEP counter, which advances solely on a completed
    # explore step. A silent hang is worth more than a crash to catch: it yields
    # a full-length run whose coverage curve is flat and plausible.
    #
    # Not fatal if a robot has legitimately finished: DONE-idle is a terminal
    # state by design and its step count is supposed to stop.
    if [ "$SA" = "$LAST_SA" ] && [ "$SB" = "$LAST_SB" ]; then
      STALL=$((STALL + 1))
      DONE_A=$(grep -c "Exploration complete\|DONE" "$OUTDIR/planner_atlas.log" 2>/dev/null || true)
      DONE_B=$(grep -c "Exploration complete\|DONE" "$OUTDIR/planner_bestla.log" 2>/dev/null || true)
      if [ "$STALL" -ge "$HANG_HB" ] && [ "$DONE_A" = 0 ] && [ "$DONE_B" = 0 ]; then
        die "HUNG: neither planner advanced a step in $((STALL * 60)) sim-s \
(steps still $SA/$SB) and neither reports DONE. Run is invalid — check \
'rejected' counts in $OUTDIR/planner_*.log (cost_grid_radius_cap_m=$COST_CAP)."
      fi
      [ "$STALL" -ge 2 ] && log "WARNING no step progress for $((STALL * 60)) sim-s"
    else
      STALL=0
    fi
    LAST_SA=$SA; LAST_SB=$SB
  fi
  if [ "$STOP_ON_DONE" = "1" ]; then
    all_done=1
    for r in $ROBOTS; do
      [ "$(planner_state "$r")" = "DONE" ] || { all_done=0; break; }
    done
    if [ "$all_done" = 1 ]; then
      # Re-armed on any robot leaving DONE, so a target release or a manoeuvre
      # that pulls one back out restarts the grace rather than banking it.
      [ "$DONE_SINCE" = -1 ] && { DONE_SINCE=$T; log "all planners DONE at t_sim=$T — holding ${DONE_GRACE_S}s for backlog drain"; }
      if [ $((T - DONE_SINCE)) -ge "$DONE_GRACE_S" ]; then
        RUN_END_REASON="all_done"
        log "run complete: every planner DONE (makespan t_sim=$((DONE_SINCE - T0)) s)"
        break
      fi
    elif [ "$DONE_SINCE" != -1 ]; then
      log "a planner left DONE at t_sim=$T — grace re-armed"
      DONE_SINCE=-1
    fi
  fi
  [ "$DURATION_S" != "0" ] && [ "$T" -ge $((T0 + DURATION_S)) ] && { RUN_END_REASON="censored_at_T"; break; }
done
# Which exit fired is data, not logging: "every robot finished by t" and
# "still unfinished when the horizon cut it off" are different observations and
# the analysis must not average them together.
echo "run_end_reason=${RUN_END_REASON:-censored_at_T}" >> "$OUTDIR/run_manifest.txt"
echo "run_end_t_sim=$((T - T0))" >> "$OUTDIR/run_manifest.txt"
log "run ended at t_sim=+$((T - T0))s (${RUN_END_REASON:-censored_at_T})"
# teardown runs on EXIT
