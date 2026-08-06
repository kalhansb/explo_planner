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
# WRITE THESE WITH A DECIMAL POINT. `-p x:=2` makes ros2 infer an integer and the
# planner declares these as doubles, so the node dies at startup on a type
# mismatch; `-p x:=2.0` is what it wants.
PROX_HOLD_M="${PROX_HOLD_M:-1.5}"
PROX_RESUME_M="${PROX_RESUME_M:-2.5}"
# DWELL_SYNC=1 (default) keeps the vantage-ring rendezvous barrier on: a robot at
# its vantage holds (dwell clock not started) until every peer claiming the same
# trunk is standing on its own angle, so the team captures one trunk state
# simultaneously. DWELL_SYNC=0 is the A/B against the old behaviour, where the
# first robot to arrive dwells alone and can close the quota before the peer
# lands. See exploit_dwell_sync_* in shared_params.yaml.
DWELL_SYNC="${DWELL_SYNC:-1}"
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
RECONNECT_MODE="${RECONNECT_MODE:-hybrid}"
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
for f in "$RVIZ_CFG" "$URDF_IN" "$HERE/sim_tf_publisher.py" "$HERE/sim_target_markers.py"; do
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
  ps -eo pid,cmd | grep -E "ign gazebo|explo_planner_node|target_scheduler_node|dscovox_node|scovox_node|scovox_mapping_node|dscovox_mapping_node|simple_nav|robot_state_publisher|sim_tf_publisher|sim_target_markers|rosbag2|parameter_bridge|ros_gz|rviz2" \
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
  log "teardown complete — outputs in $OUTDIR"
}
trap teardown EXIT INT TERM
die() { log "ERROR $*"; exit 1; }   # trap runs teardown

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
  ok=0
  for i in $(seq 1 24); do
    timeout 8 ros2 topic echo /$r/odom_ground_truth --once >/dev/null 2>&1 && { ok=1; break; }
  done
  [ "$ok" = 1 ] || die "sim: no /$r/odom_ground_truth after 3 min (see $OUTDIR/sim.log)"
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

# --- 3. nav + lidar mapping, mergers cross-wired ----------------------------
# mapping:=dscovox_lidar => per robot a scovox_node on /<r>/velodyne_points plus
# a dscovox_node merging BOTH robots' scovox_bin streams, so each planner reads
# its own copy of the TEAM map.
start nav_atlas "$OUTDIR/nav_atlas.log" \
  ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv \
    mapping:=dscovox_lidar peers:=bestla
start nav_bestla "$OUTDIR/nav_bestla.log" \
  ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=bestla mode:=ugv \
    mapping:=dscovox_lidar peers:=atlas
for r in $ROBOTS; do
  ok=0
  for i in $(seq 1 24); do
    timeout 8 ros2 topic echo /$r/scovox_node/planning_map --once >/dev/null 2>&1 && { ok=1; break; }
  done
  [ "$ok" = 1 ] || die "nav: no /$r planning_map after 3 min (see $OUTDIR/nav_$r.log)"
  log "$r mapping alive (planning_map publishing)"
done
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
if [ "$RECORD" = "1" ]; then
  start bag "$OUTDIR/bag.log" \
    ros2 bag record -o "$OUTDIR/rosbag2" \
      --max-bag-size 1000000000 --compression-mode file --compression-format zstd \
      /clock /tf /tf_static \
      /atlas/odom_ground_truth /atlas/imu/data /atlas/cmd_vel \
      /atlas/scovox_node/scovox_bin /atlas/scovox_node/planning_map \
      /atlas/goal_pose /atlas/explo_planner/candidates \
      /bestla/odom_ground_truth /bestla/imu/data /bestla/cmd_vel \
      /bestla/scovox_node/scovox_bin /bestla/scovox_node/planning_map \
      /bestla/goal_pose /bestla/explo_planner/candidates \
      /exploration/targets /exploration/intents
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
log "candidate_enable_polar=$POLAR_ARG (FRONTIER_ONLY=$FRONTIER_ONLY)"
log "proximity_hold/resume_dist_m=$PROX_HOLD_M/$PROX_RESUME_M m (yaml field defaults 5.0/6.0 overridden for sim)"
log "exploit_dwell_sync_enabled=$DWELL_SYNC_ARG (DWELL_SYNC=$DWELL_SYNC)"
log "reconnect_mode=$RECONNECT_MODE"
for r in $ROBOTS; do
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
      -p reconnect_mode:=$RECONNECT_MODE \
      -p rendezvous_expected_peers:=1 \
      -p output_csv:="$OUTDIR/planner_$r.csv"
done
# Scheduler last; its node name must stay target_scheduler in the root namespace
# or the targets yaml's parameter key will not match.
start sched "$OUTDIR/sched.log" \
  ros2 run explo_planner target_scheduler_node --ros-args \
    -r __node:=target_scheduler \
    --params-file "$TARGETS" -p use_sim_time:=true
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
  fi
  [ "$DURATION_S" != "0" ] && [ "$T" -ge $((T0 + DURATION_S)) ] && break
done
log "duration reached at t_sim=$T"
# teardown runs on EXIT
