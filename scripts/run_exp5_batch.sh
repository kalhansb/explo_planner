#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_WS_ROOT="$(cd "${PKG_DIR}/../../.." && pwd)"

WS_ROOT="${DEFAULT_WS_ROOT}"
RESULTS_DIR="${PKG_DIR}/results"
POSES_FILE="${SCRIPT_DIR}/exp5_start_poses.csv"

ROBOT="atlas"
MODE="ugv"
HEADLESS="true"
MAX_STEPS=200

WORLDS=("cmu_forest")
PLANNERS=("eig:dscovox" "frontier:dscovox" "entropy:dscovox" "entropy:logodds" "random:dscovox")

TRAJECTORY_SCORING="false"
TRAJECTORY_SAMPLE_SPACING_M="1.5"

WAIT_SIM_SEC=18
WAIT_STACK_SEC=15
WAIT_PLANNER_PRIME_SEC=2
PLANNER_TIMEOUT_SEC=1800
CLEANUP_WAIT_SEC=12

RESUME=true
DRY_RUN=false

TRIAL_PREFIX="exp5"
LOG_ROOT=""

SIM_PID=""
NAV_PID=""
TF_PID=""
LOGODDS_PID=""

usage() {
  cat <<'EOF'
Usage:
  run_exp5_batch.sh [options]

Options:
  --workspace <path>       Workspace root (default: auto-detected)
  --results-dir <path>     CSV output directory (default: explo_planner/results)
  --poses-file <path>      CSV with start poses (default: scripts/exp5_start_poses.csv)
  --robot <name>           Robot namespace (default: atlas)
  --mode <ugv|uav>         simple_nav_3d mode (default: ugv)
  --headless <true|false>  Pass headless to sim launch (default: true)
  --max-steps <n>          Planner max steps (default: 200)
  --worlds "w1 w2"         Space-separated world list
  --planners "p:m ..."     Planner/map list (example: "eig:dscovox entropy:logodds")
  --trajectory-scoring <true|false>  Enable trajectory-level scoring (default: false)
  --trajectory-spacing <m> Trajectory sample spacing in metres (default: 1.5)
  --resume <true|false>    Skip trials whose CSV already exists (default: true)
  --dry-run                Print planned commands only
  --trial-prefix <name>    CSV filename prefix (default: exp5)
  --planner-timeout <s>    Hard wall-clock timeout per trial (default: 1800)
  -h, --help               Show this help

Start pose CSV format:
  start_id,x,y,yaw
  s1,0.0,0.0,0.0
  s2,8.0,0.0,1.57
EOF
}

log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"
}

require_cmd() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "ERROR: command not found: ${cmd}" >&2
    exit 1
  fi
}

is_true() {
  [[ "${1,,}" == "1" || "${1,,}" == "true" || "${1,,}" == "yes" || "${1,,}" == "on" ]]
}

kill_pgroup() {
  local pid="$1"
  local sig="$2"
  [[ -z "${pid}" ]] && return 0
  # Send to the process group (negative PID). setsid makes the launched
  # process its own group leader so PGID == PID.
  kill -"${sig}" -- "-${pid}" >/dev/null 2>&1 || true
}

pgroup_alive() {
  local pid="$1"
  [[ -z "${pid}" ]] && return 1
  # Any process still in the group?
  pgrep -g "${pid}" >/dev/null 2>&1
}

cleanup_trial() {
  local pids=("${LOGODDS_PID}" "${TF_PID}" "${NAV_PID}" "${SIM_PID}")

  for pid in "${pids[@]}"; do
    pgroup_alive "${pid}" && kill_pgroup "${pid}" INT
  done
  sleep 3
  for pid in "${pids[@]}"; do
    pgroup_alive "${pid}" && kill_pgroup "${pid}" TERM
  done
  sleep 2
  for pid in "${pids[@]}"; do
    pgroup_alive "${pid}" && kill_pgroup "${pid}" KILL
  done

  # Belt-and-suspenders: nuke any stray ign/gz/ros2 launch processes
  # that escaped the groups (rare, but seen with ros2 launch + gazebo).
  pkill -KILL -f 'ros2 launch hmr_sim' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 launch simple_nav_3d' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 launch odom_to_tf_ros2' >/dev/null 2>&1 || true
  pkill -KILL -f 'ign gazebo' >/dev/null 2>&1 || true
  pkill -KILL -f 'gz sim' >/dev/null 2>&1 || true
  pkill -KILL -f 'ruby.*ign'  >/dev/null 2>&1 || true
  pkill -KILL -f 'log_odds_node' >/dev/null 2>&1 || true
  # Per-trial nodes that occasionally outlive the launch group.
  pkill -KILL -f 'simple_nav_costmap_node\|simple_nav_navigator_node\|simple_nav_planner_node\|simple_nav_controller_node' >/dev/null 2>&1 || true
  pkill -KILL -f 'scovox_mapping_node\|dscovox_mapping_node\|explo_planner_node\|exploration_planner_comp_node\|odom_to_tf' >/dev/null 2>&1 || true
  pkill -KILL -f 'parameter_bridge' >/dev/null 2>&1 || true

  # Clear FastDDS SHM segments left by killed nodes. These survive process
  # death and a stale port file makes the next trial's nodes fail to bind
  # ("RTPS_TRANSPORT_SHM Error] Failed init_port"), which silently breaks
  # the TF tree and starves the planner of map data.
  # Using find -delete because plain `rm /dev/shm/fastrtps_*` glob behaves
  # inconsistently when no files match.
  find /dev/shm -maxdepth 1 -name 'fastrtps*' -delete 2>/dev/null || true
  find /dev/shm -maxdepth 1 -name 'sem.fastrtps*' -delete 2>/dev/null || true

  sleep "${CLEANUP_WAIT_SEC}"

  SIM_PID=""
  NAV_PID=""
  TF_PID=""
  LOGODDS_PID=""
}

on_exit() {
  cleanup_trial
}

on_interrupt() {
  log "Interrupted by user (Ctrl+C). Cleaning up..."
  cleanup_trial
  trap - EXIT
  exit 130
}

wait_for_set_pose_service() {
  local world="$1"
  local timeout_s="$2"
  local start_ts
  start_ts=$(date +%s)
  while true; do
    if ign service -l 2>/dev/null | grep -q "/world/${world}/set_pose"; then
      return 0
    fi
    if (( $(date +%s) - start_ts > timeout_s )); then
      return 1
    fi
    sleep 1
  done
}

teleport_robot() {
  local world="$1"
  local x="$2"
  local y="$3"
  local yaw="$4"

  local qz
  local qw
  qz=$(awk -v yaw="${yaw}" 'BEGIN { printf "%.8f", sin(yaw/2.0) }')
  qw=$(awk -v yaw="${yaw}" 'BEGIN { printf "%.8f", cos(yaw/2.0) }')

  ign service -s "/world/${world}/set_pose" \
    --reqtype ignition.msgs.Pose \
    --reptype ignition.msgs.Boolean \
    --timeout 2000 \
    --req "name: \"${ROBOT}\", position: {x: ${x}, y: ${y}, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}" >/dev/null
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --workspace)
        WS_ROOT="$2"
        shift 2
        ;;
      --results-dir)
        RESULTS_DIR="$2"
        shift 2
        ;;
      --poses-file)
        POSES_FILE="$2"
        shift 2
        ;;
      --robot)
        ROBOT="$2"
        shift 2
        ;;
      --mode)
        MODE="$2"
        shift 2
        ;;
      --headless)
        HEADLESS="$2"
        shift 2
        ;;
      --max-steps)
        MAX_STEPS="$2"
        shift 2
        ;;
      --worlds)
        read -r -a WORLDS <<<"$2"
        shift 2
        ;;
      --planners)
        read -r -a PLANNERS <<<"$2"
        shift 2
        ;;
      --trajectory-scoring)
        TRAJECTORY_SCORING="$2"
        shift 2
        ;;
      --trajectory-spacing)
        TRAJECTORY_SAMPLE_SPACING_M="$2"
        shift 2
        ;;
      --resume)
        RESUME="$2"
        shift 2
        ;;
      --dry-run)
        DRY_RUN=true
        shift
        ;;
      --trial-prefix)
        TRIAL_PREFIX="$2"
        shift 2
        ;;
      --planner-timeout)
        PLANNER_TIMEOUT_SEC="$2"
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "ERROR: unknown arg: $1" >&2
        usage
        exit 1
        ;;
    esac
  done
}

load_start_poses() {
  if [[ ! -f "${POSES_FILE}" ]]; then
    echo "ERROR: poses file not found: ${POSES_FILE}" >&2
    exit 1
  fi

  START_ROWS=()
  while IFS=, read -r start_id x y yaw || [[ -n "${start_id}" ]]; do
    [[ -z "${start_id}" ]] && continue
    [[ "${start_id:0:1}" == "#" ]] && continue
    if [[ "${start_id}" == "start_id" ]]; then
      continue
    fi
    START_ROWS+=("${start_id},${x},${y},${yaw}")
  done <"${POSES_FILE}"

  if [[ ${#START_ROWS[@]} -eq 0 ]]; then
    echo "ERROR: no valid start poses in ${POSES_FILE}" >&2
    exit 1
  fi
}

source_setups() {
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi

  if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${WS_ROOT}/install/setup.bash"
  elif [[ -f "${WS_ROOT}/install/local_setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${WS_ROOT}/install/local_setup.bash"
  else
    echo "ERROR: no install setup found under ${WS_ROOT}/install" >&2
    exit 1
  fi

  if [[ ${had_nounset} -eq 1 ]]; then
    set -u
  fi
}

run_trial() {
  local world="$1"
  local planner="$2"
  local map_type="$3"
  local start_id="$4"
  local x="$5"
  local y="$6"
  local yaw="$7"

  local csv="${RESULTS_DIR}/${TRIAL_PREFIX}_${planner}_${map_type}_${world}_${start_id}.csv"
  local log_dir="${LOG_ROOT}/${TRIAL_PREFIX}_${planner}_${map_type}_${world}_${start_id}"
  mkdir -p "${log_dir}"

  if is_true "${RESUME}" && [[ -f "${csv}" ]]; then
    log "SKIP existing CSV: ${csv}"
    return 0
  fi

  log "TRIAL world=${world} planner=${planner} map=${map_type} start=${start_id} pose=(${x}, ${y}, yaw=${yaw})"

  # Generate a one-off scenario YAML so the robot spawns directly at the
  # experiment start pose — no teleport needed, no ghost voxels at origin.
  local scenario_file="${log_dir}/scenario.yaml"
  cat > "${scenario_file}" <<SCENARIO_EOF
world: ${world}
robots:
  - name: ${ROBOT}
    pose: {x: ${x}, y: ${y}, z: 0.5, yaw: ${yaw}}
SCENARIO_EOF

  local sim_cmd="ros2 launch hmr_sim robot_sim.launch.py scenario:=${scenario_file} headless:=${HEADLESS}"
  # Always use dscovox mapping for the nav stack — it provides the planning
  # map and navigation infrastructure for all planners.  The planner's
  # map_type (dscovox vs logodds) only controls the scoring method.
  local nav_cmd="ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=${ROBOT} mode:=${MODE} mapping:=dscovox"
  local tf_cmd="ros2 launch odom_to_tf_ros2 odom_to_tf_robot.launch.py robot_name:=${ROBOT}"
  local planner_cmd="ros2 launch explo_planner exploration_experiment.launch.py planner:=${planner} map_type:=${map_type} max_steps:=${MAX_STEPS} output_csv:=${csv} trajectory_scoring:=${TRAJECTORY_SCORING} trajectory_sample_spacing_m:=${TRAJECTORY_SAMPLE_SPACING_M}"

  if is_true "${DRY_RUN}"; then
    echo "  ${sim_cmd}"
    echo "  ${nav_cmd}"
    echo "  ${tf_cmd}"
    [[ "${map_type}" == "logodds" ]] && echo "  ros2 run log_odds_mapping log_odds_node (ns=/${ROBOT})"
    echo "  ${planner_cmd}"
    return 0
  fi

  rm -f "${csv}"

  setsid bash -c "${sim_cmd}" >"${log_dir}/sim.log" 2>&1 &
  SIM_PID=$!

  if ! wait_for_set_pose_service "${world}" 90; then
    log "ERROR: set_pose service did not appear for world ${world}"
    cleanup_trial
    return 1
  fi

  sleep "${WAIT_SIM_SEC}"

  setsid bash -c "${nav_cmd}" >"${log_dir}/nav.log" 2>&1 &
  NAV_PID=$!

  setsid bash -c "${tf_cmd}" >"${log_dir}/tf.log" 2>&1 &
  TF_PID=$!

  # For logodds map_type, launch the log_odds_node to provide the 3D
  # pointcloud that the entropy planner scores against.  The nav stack
  # still uses dscovox for its own planning maps.
  if [[ "${map_type}" == "logodds" ]]; then
    # Depth gate aligned with scovox_node defaults (min_depth=0.1) so the
    # entropy A/B (entropy:dscovox vs entropy:logodds) feeds both mappers the
    # same observation set. trace_no_return_rays:=true mirrors scovox_node's
    # default in simple_nav_3d.launch.py — both mappers must carve free space
    # along no-return rays for the ablation to isolate the update rule.
    local logodds_cmd="ros2 run log_odds_mapping log_odds_node \
      --ros-args -r __ns:=/${ROBOT} \
      -p use_sim_time:=true \
      -p base_frame:=${ROBOT}/base_link \
      -p integration_frame:=${ROBOT}/odom \
      -p depth_topic:=rgbd_camera_depth_image \
      -p depth_info_topic:=rgbd_camera_info \
      -p min_depth:=0.1 \
      -p max_depth:=10.0 \
      -p trace_no_return_rays:=true \
      -p planning_map_min_z:=0.05 \
      -p planning_map_max_z:=1.0 \
      -p occupancy_vis_threshold:=0.0"
    setsid bash -c "${logodds_cmd}" >"${log_dir}/logodds.log" 2>&1 &
    LOGODDS_PID=$!
  fi

  sleep "${WAIT_STACK_SEC}"

  set +e
  timeout --foreground --kill-after=30s "${PLANNER_TIMEOUT_SEC}" \
      bash -c "${planner_cmd}" >"${log_dir}/planner.log" 2>&1
  local planner_rc=$?
  set -e

  cleanup_trial

  if [[ ${planner_rc} -eq 124 || ${planner_rc} -eq 137 ]]; then
    log "ERROR: planner timed out after ${PLANNER_TIMEOUT_SEC}s (rc=${planner_rc}, see ${log_dir}/planner.log)"
    return "${planner_rc}"
  fi
  if [[ ${planner_rc} -ne 0 ]]; then
    log "ERROR: planner exited with status ${planner_rc} (see ${log_dir}/planner.log)"
    return "${planner_rc}"
  fi

  if [[ ! -f "${csv}" ]]; then
    log "ERROR: planner finished but CSV missing: ${csv}"
    return 1
  fi

  log "DONE ${csv}"
  return 0
}

main() {
  parse_args "$@"

  require_cmd ros2
  require_cmd ign
  require_cmd awk
  require_cmd setsid
  require_cmd pkill
  require_cmd pgrep
  require_cmd timeout

  source_setups
  load_start_poses

  mkdir -p "${RESULTS_DIR}"
  LOG_ROOT="${RESULTS_DIR}/logs"
  mkdir -p "${LOG_ROOT}"

  log "Workspace: ${WS_ROOT}"
  log "Results: ${RESULTS_DIR}"
  log "Poses: ${POSES_FILE}"
  log "Trials: ${#WORLDS[@]} worlds x ${#PLANNERS[@]} planners x ${#START_ROWS[@]} starts"

  # Save batch configuration to results folder.
  local manifest="${RESULTS_DIR}/batch_$(date '+%Y%m%d_%H%M%S').txt"
  {
    echo "Batch started: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "Workspace: ${WS_ROOT}"
    echo "Trial prefix: ${TRIAL_PREFIX}"
    echo "Max steps: ${MAX_STEPS}"
    echo "Planner timeout: ${PLANNER_TIMEOUT_SEC}s"
    echo "Trajectory scoring: ${TRAJECTORY_SCORING}"
    echo "Trajectory spacing: ${TRAJECTORY_SAMPLE_SPACING_M}m"
    echo "Headless: ${HEADLESS}"
    echo "Resume: ${RESUME}"
    echo ""
    echo "Worlds: ${WORLDS[*]}"
    echo "Planners: ${PLANNERS[*]}"
    echo ""
    echo "Start poses:"
    for row in "${START_ROWS[@]}"; do
      echo "  ${row}"
    done
    echo ""
    echo "Trials:"
    for world in "${WORLDS[@]}"; do
      for planner_map in "${PLANNERS[@]}"; do
        IFS=':' read -r p m <<<"${planner_map}"
        for row in "${START_ROWS[@]}"; do
          IFS=, read -r sid _ _ _ <<<"${row}"
          local csv_name="${TRIAL_PREFIX}_${p}_${m}_${world}_${sid}.csv"
          local status="PENDING"
          [[ -f "${RESULTS_DIR}/${csv_name}" ]] && is_true "${RESUME}" && status="SKIP (exists)"
          echo "  ${csv_name}  ${status}"
        done
      done
    done
  } > "${manifest}"
  log "Batch manifest: ${manifest}"

  local failures=0
  for world in "${WORLDS[@]}"; do
    for planner_map in "${PLANNERS[@]}"; do
      IFS=':' read -r planner map_type <<<"${planner_map}"
      if [[ -z "${planner}" || -z "${map_type}" ]]; then
        echo "ERROR: invalid planner entry '${planner_map}', expected planner:map_type" >&2
        exit 1
      fi
      for row in "${START_ROWS[@]}"; do
        IFS=, read -r start_id x y yaw <<<"${row}"
        if ! run_trial "${world}" "${planner}" "${map_type}" "${start_id}" "${x}" "${y}" "${yaw}"; then
          failures=$((failures + 1))
        fi
      done
    done
  done

  if [[ ${failures} -gt 0 ]]; then
    log "Batch complete with ${failures} failed trials"
    exit 1
  fi

  log "Batch complete with all trials successful"
}

trap on_exit EXIT
trap on_interrupt INT TERM

main "$@"
