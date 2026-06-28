#!/usr/bin/env bash
#
# Exp 6 — two-robot fusion (slim).
# Launches a 2-UGV sim on cmu_forest, two simple_nav_3d stacks (each with
# peers wired to the other so each robot runs its own dscovox_node fused
# map), and per-robot EIG planners. After the planner completes (or times
# out), snapshots 4 maps per trial:
#   - /atlas/scovox_node/pointcloud   (atlas local SCovox)
#   - /atlas/dscovox_node/pointcloud  (atlas fused view: self + bestla via peer_bridge)
#   - /bestla/scovox_node/pointcloud  (bestla local SCovox)
#   - /bestla/dscovox_node/pointcloud (bestla fused view)
# The 4 NPZs per trial feed eval_exp6.py for F-score vs voxelised GT.
#
# Structurally a stripped copy of run_exp7_batch.sh (no bestla peer changes,
# no trajectory scoring). Only eig planner is run; frontier/random are not
# needed for a fusion-works demo.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_WS_ROOT="$(cd "${PKG_DIR}/../../.." && pwd)"

WS_ROOT="${DEFAULT_WS_ROOT}"
RESULTS_DIR="${PKG_DIR}/results"
CONFIGS_FILE="${SCRIPT_DIR}/exp6_start_configs.csv"

MODE="ugv"
HEADLESS="true"
MAX_STEPS=150  # per-robot

WORLDS=("cmu_forest")
PLANNERS=("eig:dscovox")

WAIT_SIM_SEC=14
WAIT_STACK_SEC=10
WAIT_PLANNER_PRIME_SEC=3
PLANNER_TIMEOUT_SEC=1800
CLEANUP_WAIT_SEC=6
SNAPSHOT_TIMEOUT_SEC=20

RESUME=true
DRY_RUN=false
SNAPSHOT_MAPS=true

TRIAL_PREFIX="exp6"
LOG_ROOT=""

SIM_PID=""
NAV_ATLAS_PID=""
NAV_BESTLA_PID=""
TF_ATLAS_PID=""
TF_BESTLA_PID=""

usage() {
  cat <<'EOF'
Usage:
  run_exp6_batch.sh [options]

Options:
  --workspace <path>       Workspace root (auto-detected by default)
  --results-dir <path>     NPZ + CSV output directory
  --configs-file <path>    Start-config CSV (default: exp6_start_configs.csv)
  --mode <ugv|uav>         simple_nav_3d mode (default: ugv)
  --headless <true|false>  Pass headless to sim launch (default: true)
  --max-steps <n>          Per-robot planner step budget (default: 150)
  --worlds "w1 w2"         Space-separated world list (default: cmu_forest)
  --planners "p:m ..."     Planner/map list (default: eig:dscovox)
  --planner-timeout <s>    Hard wall-clock timeout per trial (default: 1800)
  --snapshot-maps <bool>   Snapshot per-robot scovox + dscovox NPZs (default: true)
  --resume <true|false>    Skip trials whose outputs already exist (default: true)
  --dry-run                Print planned commands only
  --trial-prefix <name>    Output filename prefix (default: exp6)
  -h, --help               Show this help

Start config CSV format:
  config_id,atlas_x,atlas_y,atlas_yaw,bestla_x,bestla_y,bestla_yaw
EOF
}

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "ERROR: command not found: $1" >&2; exit 1
  fi
}

is_true() { [[ "${1,,}" == "1" || "${1,,}" == "true" || "${1,,}" == "yes" || "${1,,}" == "on" ]]; }

kill_pgroup() {
  local pid="$1" sig="$2"
  [[ -z "${pid}" ]] && return 0
  kill -"${sig}" -- "-${pid}" >/dev/null 2>&1 || true
}

pgroup_alive() {
  local pid="$1"
  [[ -z "${pid}" ]] && return 1
  pgrep -g "${pid}" >/dev/null 2>&1
}

cleanup_trial() {
  local pids=("${TF_BESTLA_PID}" "${TF_ATLAS_PID}"
              "${NAV_BESTLA_PID}" "${NAV_ATLAS_PID}"
              "${SIM_PID}")
  for pid in "${pids[@]}"; do pgroup_alive "${pid}" && kill_pgroup "${pid}" INT; done
  sleep 3
  for pid in "${pids[@]}"; do pgroup_alive "${pid}" && kill_pgroup "${pid}" TERM; done
  sleep 2
  for pid in "${pids[@]}"; do pgroup_alive "${pid}" && kill_pgroup "${pid}" KILL; done

  pkill -KILL -f 'ros2 launch hmr_sim' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 launch simple_nav_3d' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 launch odom_to_tf_ros2' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 launch explo_planner' >/dev/null 2>&1 || true
  pkill -KILL -f 'pointcloud_to_npz' >/dev/null 2>&1 || true
  pkill -KILL -f 'ign gazebo' >/dev/null 2>&1 || true
  pkill -KILL -f 'gz sim' >/dev/null 2>&1 || true
  pkill -KILL -f 'ruby.*ign'  >/dev/null 2>&1 || true

  sleep "${CLEANUP_WAIT_SEC}"

  SIM_PID=""; NAV_ATLAS_PID=""; NAV_BESTLA_PID=""; TF_ATLAS_PID=""; TF_BESTLA_PID=""
}

on_exit() { cleanup_trial; }

wait_for_set_pose_service() {
  local world="$1" timeout_s="$2" start_ts
  start_ts=$(date +%s)
  while true; do
    if ign service -l 2>/dev/null | grep -q "/world/${world}/set_pose"; then return 0; fi
    if (( $(date +%s) - start_ts > timeout_s )); then return 1; fi
    sleep 1
  done
}

teleport_robot() {
  local world="$1" robot="$2" x="$3" y="$4" yaw="$5"
  local qz qw
  qz=$(awk -v yaw="${yaw}" 'BEGIN { printf "%.8f", sin(yaw/2.0) }')
  qw=$(awk -v yaw="${yaw}" 'BEGIN { printf "%.8f", cos(yaw/2.0) }')
  ign service -s "/world/${world}/set_pose" \
    --reqtype ignition.msgs.Pose \
    --reptype ignition.msgs.Boolean \
    --timeout 2000 \
    --req "name: \"${robot}\", position: {x: ${x}, y: ${y}, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}" >/dev/null
}

# Snapshot one pointcloud topic into an NPZ. Invokes pointcloud_to_npz.py
# directly via python3 (the scovox_eval ament_python package is ignored by
# colcon via its COLCON_IGNORE marker, so `ros2 run scovox_eval ...` is
# unavailable). The script's main() calls rclpy.init(args=args), which
# still consumes --ros-args normally.
SNAPSHOT_PY="${WS_ROOT}/src/robot_sw/distributed_mapping/scovox_eval/scovox_eval/pointcloud_to_npz.py"

snapshot_topic() {
  local topic="$1" out_path="$2" log_path="$3"
  set +e
  timeout --foreground --kill-after=5s "${SNAPSHOT_TIMEOUT_SEC}" \
    python3 "${SNAPSHOT_PY}" \
      --ros-args -p "topic:=${topic}" -p "output:=${out_path}" \
      >"${log_path}" 2>&1
  local rc=$?
  set -e
  if [[ ${rc} -ne 0 ]]; then
    log "WARN: snapshot failed for ${topic} rc=${rc} (see ${log_path})"
    return 1
  fi
  if [[ ! -f "${out_path}" ]]; then
    log "WARN: snapshot produced no file for ${topic}"
    return 1
  fi
  return 0
}

snapshot_all_maps() {
  local stem="$1" log_dir="$2"
  local failures=0
  for robot in atlas bestla; do
    for kind in scovox dscovox; do
      local topic="/${robot}/${kind}_node/pointcloud"
      local out="${stem}_${robot}_${kind}.npz"
      local logp="${log_dir}/snap_${robot}_${kind}.log"
      snapshot_topic "${topic}" "${out}" "${logp}" || failures=$((failures + 1))
    done
  done
  return "${failures}"
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --workspace)        WS_ROOT="$2";          shift 2 ;;
      --results-dir)      RESULTS_DIR="$2";      shift 2 ;;
      --configs-file)     CONFIGS_FILE="$2";     shift 2 ;;
      --mode)             MODE="$2";             shift 2 ;;
      --headless)         HEADLESS="$2";         shift 2 ;;
      --max-steps)        MAX_STEPS="$2";        shift 2 ;;
      --worlds)           read -r -a WORLDS <<<"$2";    shift 2 ;;
      --planners)         read -r -a PLANNERS <<<"$2";  shift 2 ;;
      --resume)           RESUME="$2";           shift 2 ;;
      --dry-run)          DRY_RUN=true;          shift ;;
      --trial-prefix)     TRIAL_PREFIX="$2";     shift 2 ;;
      --planner-timeout)  PLANNER_TIMEOUT_SEC="$2"; shift 2 ;;
      --snapshot-maps)    SNAPSHOT_MAPS="$2";    shift 2 ;;
      -h|--help)          usage; exit 0 ;;
      *) echo "ERROR: unknown arg: $1" >&2; usage; exit 1 ;;
    esac
  done
}

load_start_configs() {
  if [[ ! -f "${CONFIGS_FILE}" ]]; then
    echo "ERROR: configs file not found: ${CONFIGS_FILE}" >&2
    exit 1
  fi
  CONFIG_ROWS=()
  while IFS=, read -r config_id ax ay ayaw bx by byaw || [[ -n "${config_id}" ]]; do
    [[ -z "${config_id}" ]] && continue
    [[ "${config_id:0:1}" == "#" ]] && continue
    [[ "${config_id}" == "config_id" ]] && continue
    CONFIG_ROWS+=("${config_id},${ax},${ay},${ayaw},${bx},${by},${byaw}")
  done <"${CONFIGS_FILE}"
  if [[ ${#CONFIG_ROWS[@]} -eq 0 ]]; then
    echo "ERROR: no valid start configurations in ${CONFIGS_FILE}" >&2
    exit 1
  fi
}

source_setups() {
  local had_nounset=0
  if [[ $- == *u* ]]; then had_nounset=1; set +u; fi
  [[ -f "/opt/ros/humble/setup.bash" ]] && source /opt/ros/humble/setup.bash
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source "${WS_ROOT}/install/setup.bash"
  elif [[ -f "${WS_ROOT}/install/local_setup.bash" ]]; then
    source "${WS_ROOT}/install/local_setup.bash"
  else
    echo "ERROR: no install setup found under ${WS_ROOT}/install" >&2; exit 1
  fi
  [[ ${had_nounset} -eq 1 ]] && set -u
}

run_trial() {
  local world="$1" planner="$2" map_type="$3" config_id="$4"
  local ax="$5" ay="$6" ayaw="$7" bx="$8" by="$9" byaw="${10}"

  local stem="${RESULTS_DIR}/${TRIAL_PREFIX}_${planner}_${world}_${config_id}"
  local atlas_csv="${stem}_atlas.csv"
  local bestla_csv="${stem}_bestla.csv"
  local log_dir="${LOG_ROOT}/${TRIAL_PREFIX}_${planner}_${world}_${config_id}"
  mkdir -p "${log_dir}"

  # Resume check: require CSVs + (optionally) all 4 snapshot NPZs.
  if is_true "${RESUME}" && [[ -f "${atlas_csv}" && -f "${bestla_csv}" ]]; then
    if ! is_true "${SNAPSHOT_MAPS}"; then
      log "SKIP existing outputs: ${stem}_*"
      return 0
    fi
    local all_present=true
    for robot in atlas bestla; do
      for kind in scovox dscovox; do
        [[ -f "${stem}_${robot}_${kind}.npz" ]] || all_present=false
      done
    done
    if ${all_present}; then
      log "SKIP existing outputs: ${stem}_*"
      return 0
    fi
    log "Resume: CSVs exist but snapshot NPZs incomplete, re-running trial"
  fi

  log "TRIAL world=${world} planner=${planner} map=${map_type} config=${config_id}"
  log "  atlas pose=(${ax}, ${ay}, yaw=${ayaw})  bestla pose=(${bx}, ${by}, yaw=${byaw})"

  local sim_cmd="ros2 launch hmr_sim robot_sim.launch.py world:=${world} robots:=atlas,bestla headless:=${HEADLESS}"
  local nav_atlas_cmd="ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas  mode:=${MODE} mapping:=${map_type} peers:=bestla"
  local nav_bestla_cmd="ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=bestla mode:=${MODE} mapping:=${map_type} peers:=atlas"
  local tf_atlas_cmd="ros2 launch odom_to_tf_ros2 odom_to_tf_robot.launch.py robot_name:=atlas"
  local tf_bestla_cmd="ros2 launch odom_to_tf_ros2 odom_to_tf_robot.launch.py robot_name:=bestla"
  local planner_cmd="ros2 launch explo_planner multi_robot_exploration.launch.py robots:=atlas,bestla planner:=${planner} map_type:=${map_type} output_dir:=${RESULTS_DIR} config_id:=${config_id} world:=${world} max_steps:=${MAX_STEPS} coordination_enabled:=true"

  # multi_robot_exploration.launch.py names CSVs "exp7_*" via its config_id
  # semantics. We need exp6_* names — symlink after the run.

  if is_true "${DRY_RUN}"; then
    echo "  ${sim_cmd}"
    echo "  ${nav_atlas_cmd}"; echo "  ${nav_bestla_cmd}"
    echo "  ${tf_atlas_cmd}"; echo "  ${tf_bestla_cmd}"
    echo "  teleport atlas -> x=${ax} y=${ay} yaw=${ayaw}"
    echo "  teleport bestla -> x=${bx} y=${by} yaw=${byaw}"
    echo "  ${planner_cmd}"
    is_true "${SNAPSHOT_MAPS}" && echo "  snapshot: ${stem}_{atlas,bestla}_{scovox,dscovox}.npz"
    return 0
  fi

  rm -f "${atlas_csv}" "${bestla_csv}"

  setsid bash -c "${sim_cmd}" >"${log_dir}/sim.log" 2>&1 &
  SIM_PID=$!

  if ! wait_for_set_pose_service "${world}" 90; then
    log "ERROR: set_pose service did not appear for world ${world}"
    cleanup_trial; return 1
  fi
  sleep "${WAIT_SIM_SEC}"

  setsid bash -c "${nav_atlas_cmd}"  >"${log_dir}/nav_atlas.log"  2>&1 & NAV_ATLAS_PID=$!
  setsid bash -c "${nav_bestla_cmd}" >"${log_dir}/nav_bestla.log" 2>&1 & NAV_BESTLA_PID=$!
  setsid bash -c "${tf_atlas_cmd}"   >"${log_dir}/tf_atlas.log"   2>&1 & TF_ATLAS_PID=$!
  setsid bash -c "${tf_bestla_cmd}"  >"${log_dir}/tf_bestla.log"  2>&1 & TF_BESTLA_PID=$!

  sleep "${WAIT_STACK_SEC}"
  teleport_robot "${world}" "atlas"  "${ax}" "${ay}" "${ayaw}"
  teleport_robot "${world}" "bestla" "${bx}" "${by}" "${byaw}"
  sleep "${WAIT_PLANNER_PRIME_SEC}"

  set +e
  timeout --foreground --kill-after=30s "${PLANNER_TIMEOUT_SEC}" \
      bash -c "${planner_cmd}" >"${log_dir}/planner.log" 2>&1
  local planner_rc=$?
  set -e

  # Planner CSVs are produced under exp7_* names by multi_robot_exploration
  # (its filename is hard-coded to "exp7_..."). Rename to exp6_* here before
  # snapshotting so downstream paths line up.
  local produced_atlas="${RESULTS_DIR}/exp7_${planner}_${world}_${config_id}_atlas.csv"
  local produced_bestla="${RESULTS_DIR}/exp7_${planner}_${world}_${config_id}_bestla.csv"
  [[ -f "${produced_atlas}" ]]  && mv -f "${produced_atlas}"  "${atlas_csv}"
  [[ -f "${produced_bestla}" ]] && mv -f "${produced_bestla}" "${bestla_csv}"

  # Snapshot BEFORE cleanup_trial (topics die with sim/nav stacks).
  local snap_failures=0
  if is_true "${SNAPSHOT_MAPS}"; then
    log "Snapshotting per-robot maps ..."
    snapshot_all_maps "${stem}" "${log_dir}" || snap_failures=$?
  fi

  cleanup_trial

  if [[ ${planner_rc} -eq 124 || ${planner_rc} -eq 137 ]]; then
    log "WARN: planner timed out after ${PLANNER_TIMEOUT_SEC}s (rc=${planner_rc}) — snapshots still attempted"
  elif [[ ${planner_rc} -ne 0 ]]; then
    log "ERROR: planner exited with status ${planner_rc} (see ${log_dir}/planner.log)"
    return "${planner_rc}"
  fi

  if [[ ! -f "${atlas_csv}" || ! -f "${bestla_csv}" ]]; then
    log "ERROR: planner finished but CSV(s) missing"
    return 1
  fi
  if [[ ${snap_failures} -gt 0 ]]; then
    log "WARN: ${snap_failures} snapshot(s) failed; eval_exp6 will report these as missing"
  fi

  log "DONE ${stem}_* (planner_rc=${planner_rc}, snap_failures=${snap_failures})"
  return 0
}

main() {
  parse_args "$@"
  for cmd in ros2 ign awk setsid pkill pgrep timeout; do require_cmd "${cmd}"; done
  source_setups
  load_start_configs

  mkdir -p "${RESULTS_DIR}"
  LOG_ROOT="${RESULTS_DIR}/logs"
  mkdir -p "${LOG_ROOT}"

  log "Workspace: ${WS_ROOT}"
  log "Results:   ${RESULTS_DIR}"
  log "Configs:   ${CONFIGS_FILE}"
  log "Trials:    ${#WORLDS[@]} worlds x ${#PLANNERS[@]} planners x ${#CONFIG_ROWS[@]} configs"

  local failures=0
  for world in "${WORLDS[@]}"; do
    for planner_map in "${PLANNERS[@]}"; do
      IFS=':' read -r planner map_type <<<"${planner_map}"
      [[ -z "${planner}" || -z "${map_type}" ]] && { echo "ERROR: bad planner entry '${planner_map}'" >&2; exit 1; }
      for row in "${CONFIG_ROWS[@]}"; do
        IFS=, read -r config_id ax ay ayaw bx by byaw <<<"${row}"
        if ! run_trial "${world}" "${planner}" "${map_type}" "${config_id}" \
                       "${ax}" "${ay}" "${ayaw}" "${bx}" "${by}" "${byaw}"; then
          failures=$((failures + 1))
        fi
      done
    done
  done

  if [[ ${failures} -gt 0 ]]; then log "Batch complete with ${failures} failed trials"; exit 1; fi
  log "Batch complete with all trials successful"
}

trap on_exit EXIT INT TERM
main "$@"
