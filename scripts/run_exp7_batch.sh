#!/usr/bin/env bash
#
# Multi-robot exploration trial driver for SCovox Experiment 7.
# Mirrors run_exp5_batch.sh but launches a 2-UGV sim, two simple_nav_3d
# stacks (each with peers wired to the other), and a multi_robot_exploration
# planner pair. Two teleport calls per trial -- one per robot.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_WS_ROOT="$(cd "${PKG_DIR}/../../.." && pwd)"

WS_ROOT="${DEFAULT_WS_ROOT}"
RESULTS_DIR="${PKG_DIR}/results"
CONFIGS_FILE="${SCRIPT_DIR}/exp7_start_configs.csv"

ROBOTS=("atlas" "bestla")
MODE="ugv"
HEADLESS="true"
MAX_STEPS=100  # per-robot budget; team total = 2 * MAX_STEPS

WORLDS=("flatforest")
PLANNERS=("eig:dscovox" "frontier:dscovox" "entropy:logodds" "random:dscovox")

WAIT_SIM_SEC=14
WAIT_STACK_SEC=10
WAIT_PLANNER_PRIME_SEC=3
PLANNER_TIMEOUT_SEC=2400
CLEANUP_WAIT_SEC=6
SNAPSHOT_TIMEOUT_SEC=20

RESUME=true
DRY_RUN=false
SNAPSHOT_MAPS=true

TRIAL_PREFIX="exp7"
LOG_ROOT=""

SIM_PID=""
NAV_ATLAS_PID=""
NAV_BESTLA_PID=""
TF_ATLAS_PID=""
TF_BESTLA_PID=""
LOGODDS_ATLAS_PID=""
LOGODDS_BESTLA_PID=""

usage() {
  cat <<'EOF'
Usage:
  run_exp7_batch.sh [options]

Options:
  --workspace <path>       Workspace root (default: auto-detected)
  --results-dir <path>     CSV output directory (default: explo_planner/results)
  --configs-file <path>    CSV with start configurations (default: scripts/exp7_start_configs.csv)
  --mode <ugv|uav>         simple_nav_3d mode (default: ugv)
  --headless <true|false>  Pass headless to sim launch (default: true)
  --max-steps <n>          Per-robot planner step budget (default: 100)
  --worlds "w1 w2"         Space-separated world list
  --planners "p:m ..."     Planner/map list (example: "eig:dscovox entropy:logodds")
  --resume <true|false>    Skip trials whose CSV already exists (default: true)
  --dry-run                Print planned commands only
  --trial-prefix <name>    CSV filename prefix (default: exp7)
  --planner-timeout <s>    Hard wall-clock timeout per trial (default: 2400)
  --snapshot-maps <bool>   Snapshot per-robot scovox + dscovox NPZs (default: true)
  -h, --help               Show this help

Start config CSV format:
  config_id,atlas_x,atlas_y,atlas_yaw,bestla_x,bestla_y,bestla_yaw
  c1,-6,-6,0,6,6,3.14
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
  kill -"${sig}" -- "-${pid}" >/dev/null 2>&1 || true
}

pgroup_alive() {
  local pid="$1"
  [[ -z "${pid}" ]] && return 1
  pgrep -g "${pid}" >/dev/null 2>&1
}

cleanup_trial() {
  local pids=("${LOGODDS_BESTLA_PID}" "${LOGODDS_ATLAS_PID}"
              "${TF_BESTLA_PID}" "${TF_ATLAS_PID}"
              "${NAV_BESTLA_PID}" "${NAV_ATLAS_PID}"
              "${SIM_PID}")

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

  pkill -KILL -f 'ros2 launch hmr_sim' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 launch simple_nav_3d' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 launch odom_to_tf_ros2' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 launch explo_planner' >/dev/null 2>&1 || true
  pkill -KILL -f 'ros2 run log_odds_mapping' >/dev/null 2>&1 || true
  pkill -KILL -f 'ign gazebo' >/dev/null 2>&1 || true
  pkill -KILL -f 'gz sim' >/dev/null 2>&1 || true
  pkill -KILL -f 'ruby.*ign'  >/dev/null 2>&1 || true
  pkill -KILL -f 'pointcloud_to_npz' >/dev/null 2>&1 || true

  sleep "${CLEANUP_WAIT_SEC}"

  SIM_PID=""
  NAV_ATLAS_PID=""
  NAV_BESTLA_PID=""
  TF_ATLAS_PID=""
  TF_BESTLA_PID=""
  LOGODDS_ATLAS_PID=""
  LOGODDS_BESTLA_PID=""
}

on_exit() {
  cleanup_trial
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

# Snapshot helpers — invoke scovox_eval's pointcloud_to_npz.py directly
# because the package is marked COLCON_IGNORE (no ros2 run entry point).
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
  local stem="$1" log_dir="$2" map_type="$3"
  local failures=0
  local kinds
  if [[ "${map_type}" == "logodds" ]]; then
    kinds=(logodds)
  else
    kinds=(scovox dscovox)
  fi
  for robot in atlas bestla; do
    for kind in "${kinds[@]}"; do
      local topic
      if [[ "${kind}" == "logodds" ]]; then
        topic="/${robot}/log_odds_node/pointcloud"
      else
        topic="/${robot}/${kind}_node/pointcloud"
      fi
      local out="${stem}_${robot}_${kind}.npz"
      local logp="${log_dir}/snap_${robot}_${kind}.log"
      snapshot_topic "${topic}" "${out}" "${logp}" || failures=$((failures + 1))
    done
  done
  return "${failures}"
}

teleport_robot() {
  local world="$1"
  local robot="$2"
  local x="$3"
  local y="$4"
  local yaw="$5"

  local qz qw
  qz=$(awk -v yaw="${yaw}" 'BEGIN { printf "%.8f", sin(yaw/2.0) }')
  qw=$(awk -v yaw="${yaw}" 'BEGIN { printf "%.8f", cos(yaw/2.0) }')

  ign service -s "/world/${world}/set_pose" \
    --reqtype ignition.msgs.Pose \
    --reptype ignition.msgs.Boolean \
    --timeout 2000 \
    --req "name: \"${robot}\", position: {x: ${x}, y: ${y}, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}" >/dev/null
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
      *)
        echo "ERROR: unknown arg: $1" >&2
        usage
        exit 1
        ;;
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
    if [[ "${config_id}" == "config_id" ]]; then
      continue
    fi
    CONFIG_ROWS+=("${config_id},${ax},${ay},${ayaw},${bx},${by},${byaw}")
  done <"${CONFIGS_FILE}"

  if [[ ${#CONFIG_ROWS[@]} -eq 0 ]]; then
    echo "ERROR: no valid start configurations in ${CONFIGS_FILE}" >&2
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
  local config_id="$4"
  local ax="$5" ay="$6" ayaw="$7"
  local bx="$8" by="$9" byaw="${10}"

  # Disambiguate trial stem by map_type so entropy:logodds and entropy:dscovox
  # don't overwrite each other. dscovox (historical default) keeps the legacy
  # filename; other map types get a suffix. Must stay in sync with
  # multi_robot_exploration.launch.py:map_suffix.
  local map_suffix=""
  [[ "${map_type}" != "dscovox" ]] && map_suffix="_${map_type}"
  local stem="${RESULTS_DIR}/${TRIAL_PREFIX}_${planner}${map_suffix}_${world}_${config_id}"
  local atlas_csv="${stem}_atlas.csv"
  local bestla_csv="${stem}_bestla.csv"
  local log_dir="${LOG_ROOT}/${TRIAL_PREFIX}_${planner}${map_suffix}_${world}_${config_id}"
  mkdir -p "${log_dir}"

  if is_true "${RESUME}" && [[ -f "${atlas_csv}" && -f "${bestla_csv}" ]]; then
    if ! is_true "${SNAPSHOT_MAPS}"; then
      log "SKIP existing CSVs: ${atlas_csv}, ${bestla_csv}"
      return 0
    fi
    local expected_kinds=(scovox dscovox)
    [[ "${map_type}" == "logodds" ]] && expected_kinds=(logodds)
    local all_present=true
    for robot in atlas bestla; do
      for kind in "${expected_kinds[@]}"; do
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
  log "  atlas pose=(${ax}, ${ay}, yaw=${ayaw})"
  log "  bestla pose=(${bx}, ${by}, yaw=${byaw})"

  local sim_cmd="ros2 launch hmr_sim robot_sim.launch.py world:=${world} robots:=atlas,bestla headless:=${HEADLESS}"
  # Always start the dscovox nav stack — it provides planning map + navigation
  # infrastructure for all planners, regardless of which map the planner scores
  # against. When map_type=logodds, we additionally launch log_odds_node per
  # robot below (mirrors run_exp5_batch.sh).
  local nav_map="dscovox"
  local nav_atlas_cmd="ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=${MODE} mapping:=${nav_map} peers:=bestla"
  local nav_bestla_cmd="ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=bestla  mode:=${MODE} mapping:=${nav_map} peers:=atlas"
  local tf_atlas_cmd="ros2 launch odom_to_tf_ros2 odom_to_tf_robot.launch.py robot_name:=atlas"
  local tf_bestla_cmd="ros2 launch odom_to_tf_ros2 odom_to_tf_robot.launch.py robot_name:=bestla"
  local planner_cmd="ros2 launch explo_planner multi_robot_exploration.launch.py robots:=atlas,bestla planner:=${planner} map_type:=${map_type} output_dir:=${RESULTS_DIR} config_id:=${config_id} world:=${world} max_steps:=${MAX_STEPS} coordination_enabled:=true"

  if is_true "${DRY_RUN}"; then
    echo "  ${sim_cmd}"
    echo "  ${nav_atlas_cmd}"
    echo "  ${nav_bestla_cmd}"
    echo "  ${tf_atlas_cmd}"
    echo "  ${tf_bestla_cmd}"
    echo "  teleport atlas -> /world/${world}/set_pose x=${ax} y=${ay} yaw=${ayaw}"
    echo "  teleport bestla -> /world/${world}/set_pose x=${bx} y=${by} yaw=${byaw}"
    echo "  ${planner_cmd}"
    return 0
  fi

  rm -f "${atlas_csv}" "${bestla_csv}"

  setsid bash -c "${sim_cmd}" >"${log_dir}/sim.log" 2>&1 &
  SIM_PID=$!

  if ! wait_for_set_pose_service "${world}" 90; then
    log "ERROR: set_pose service did not appear for world ${world}"
    cleanup_trial
    return 1
  fi

  sleep "${WAIT_SIM_SEC}"

  setsid bash -c "${nav_atlas_cmd}" >"${log_dir}/nav_atlas.log" 2>&1 &
  NAV_ATLAS_PID=$!
  setsid bash -c "${nav_bestla_cmd}"  >"${log_dir}/nav_bestla.log"  2>&1 &
  NAV_BESTLA_PID=$!

  setsid bash -c "${tf_atlas_cmd}" >"${log_dir}/tf_atlas.log" 2>&1 &
  TF_ATLAS_PID=$!
  setsid bash -c "${tf_bestla_cmd}"  >"${log_dir}/tf_bestla.log"  2>&1 &
  TF_BESTLA_PID=$!

  # For map_type=logodds, launch log_odds_node per robot so the planner has
  # a /<robot>/log_odds_node/pointcloud topic to score against. Params match
  # run_exp5_batch.sh so results are directly comparable.
  if [[ "${map_type}" == "logodds" ]]; then
    for rn in atlas bestla; do
      # trace_no_return_rays:=true mirrors the dscovox/scovox default in
      # simple_nav_3d.launch.py so both mappers carve free space on
      # no-return depth pixels — required for a clean entropy A/B.
      local logodds_cmd="ros2 run log_odds_mapping log_odds_node \
        --ros-args -r __ns:=/${rn} \
        -p use_sim_time:=true \
        -p base_frame:=${rn}/base_link \
        -p integration_frame:=${rn}/odom \
        -p depth_topic:=rgbd_camera_depth_image \
        -p depth_info_topic:=rgbd_camera_info \
        -p min_depth:=0.25 \
        -p max_depth:=10.0 \
        -p trace_no_return_rays:=true \
        -p planning_map_min_z:=0.05 \
        -p planning_map_max_z:=1.0 \
        -p occupancy_vis_threshold:=0.0"
      setsid bash -c "${logodds_cmd}" >"${log_dir}/logodds_${rn}.log" 2>&1 &
      if [[ "${rn}" == "atlas" ]]; then LOGODDS_ATLAS_PID=$!; else LOGODDS_BESTLA_PID=$!; fi
    done
  fi

  sleep "${WAIT_STACK_SEC}"
  teleport_robot "${world}" "atlas" "${ax}" "${ay}" "${ayaw}"
  teleport_robot "${world}" "bestla" "${bx}" "${by}" "${byaw}"
  sleep "${WAIT_PLANNER_PRIME_SEC}"

  set +e
  timeout --foreground --kill-after=30s "${PLANNER_TIMEOUT_SEC}" \
      bash -c "${planner_cmd}" >"${log_dir}/planner.log" 2>&1
  local planner_rc=$?
  set -e

  # multi_robot_exploration.launch.py hard-codes the "exp7_" prefix on the
  # output filenames. Rename to ${TRIAL_PREFIX}_* if the caller chose a
  # different prefix (e.g. exp7_smoke for a smoke test).
  if [[ "${TRIAL_PREFIX}" != "exp7" ]]; then
    local produced_atlas="${RESULTS_DIR}/exp7_${planner}${map_suffix}_${world}_${config_id}_atlas.csv"
    local produced_bestla="${RESULTS_DIR}/exp7_${planner}${map_suffix}_${world}_${config_id}_bestla.csv"
    [[ -f "${produced_atlas}" ]]  && mv -f "${produced_atlas}"  "${atlas_csv}"
    [[ -f "${produced_bestla}" ]] && mv -f "${produced_bestla}" "${bestla_csv}"
  fi

  # Snapshot BEFORE cleanup_trial — topics die with sim/nav stacks.
  local snap_failures=0
  if is_true "${SNAPSHOT_MAPS}"; then
    log "Snapshotting per-robot maps ..."
    snapshot_all_maps "${stem}" "${log_dir}" "${map_type}" || snap_failures=$?
  fi

  cleanup_trial

  if [[ ${planner_rc} -eq 124 || ${planner_rc} -eq 137 ]]; then
    log "WARN: planner timed out after ${PLANNER_TIMEOUT_SEC}s (rc=${planner_rc}) — snapshots still attempted"
  elif [[ ${planner_rc} -ne 0 ]]; then
    log "ERROR: planner exited with status ${planner_rc} (see ${log_dir}/planner.log)"
    return "${planner_rc}"
  fi
  if [[ ${snap_failures} -gt 0 ]]; then
    log "WARN: ${snap_failures} snapshot(s) failed; eval_exp7 will report these as missing"
  fi

  if [[ ! -f "${atlas_csv}" || ! -f "${bestla_csv}" ]]; then
    log "ERROR: planner finished but CSV(s) missing: ${atlas_csv}, ${bestla_csv}"
    return 1
  fi

  log "DONE ${atlas_csv} + ${bestla_csv}"

  # Post-process: merge per-robot CSVs into a joint completeness curve.
  if [[ -x "${SCRIPT_DIR}/merge_exp7_csvs.py" ]]; then
    "${SCRIPT_DIR}/merge_exp7_csvs.py" \
      --atlas "${atlas_csv}" --bestla "${bestla_csv}" \
      --output "${RESULTS_DIR}/${TRIAL_PREFIX}_${planner}${map_suffix}_${world}_${config_id}_joint.csv" \
      >>"${log_dir}/merge.log" 2>&1 || \
      log "WARN: merge_exp7_csvs.py failed (see ${log_dir}/merge.log)"
  fi

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
  load_start_configs

  mkdir -p "${RESULTS_DIR}"
  LOG_ROOT="${RESULTS_DIR}/logs"
  mkdir -p "${LOG_ROOT}"

  log "Workspace: ${WS_ROOT}"
  log "Results: ${RESULTS_DIR}"
  log "Configs: ${CONFIGS_FILE}"
  log "Trials: ${#WORLDS[@]} worlds x ${#PLANNERS[@]} planners x ${#CONFIG_ROWS[@]} configs"

  local failures=0
  for world in "${WORLDS[@]}"; do
    for planner_map in "${PLANNERS[@]}"; do
      IFS=':' read -r planner map_type <<<"${planner_map}"
      if [[ -z "${planner}" || -z "${map_type}" ]]; then
        echo "ERROR: invalid planner entry '${planner_map}'" >&2
        exit 1
      fi
      for row in "${CONFIG_ROWS[@]}"; do
        IFS=, read -r config_id ax ay ayaw bx by byaw <<<"${row}"
        if ! run_trial "${world}" "${planner}" "${map_type}" "${config_id}" \
                       "${ax}" "${ay}" "${ayaw}" "${bx}" "${by}" "${byaw}"; then
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

trap on_exit EXIT INT TERM

main "$@"
