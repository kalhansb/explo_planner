#!/usr/bin/env bash
#
# Quick exp5 validation across all registered worlds.
# Runs one start pose, one planner (eig:dscovox), and a small number of
# steps per world.  Uses the full exp5 pipeline (sim + nav + TF + planner
# + CSV output) via run_exp5_batch.sh.
#
# Usage:
#   ./test_all_worlds.sh                              # all worlds, 5 steps
#   ./test_all_worlds.sh --max-steps 10               # more steps
#   ./test_all_worlds.sh --worlds "flatforest cmu_forest"  # subset
#   ./test_all_worlds.sh --planners "eig:dscovox frontier:dscovox"

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_WS_ROOT="$(cd "${PKG_DIR}/../../.." && pwd)"

WS_ROOT="${DEFAULT_WS_ROOT}"
MAX_STEPS=5
PLANNERS="eig:dscovox"
CUSTOM_WORLDS=""
RESULTS_DIR="${PKG_DIR}/results/all_worlds_test"

source_setups() {
  local had_nounset=0
  [[ $- == *u* ]] && { had_nounset=1; set +u; }
  [[ -f "/opt/ros/humble/setup.bash" ]] && source /opt/ros/humble/setup.bash
  if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    source "${WS_ROOT}/install/setup.bash"
  else
    echo "ERROR: no install setup found under ${WS_ROOT}/install" >&2
    exit 1
  fi
  [[ ${had_nounset} -eq 1 ]] && set -u
}

get_registered_worlds() {
  python3 -c "
import sys, os
sys.path.insert(0, os.path.join('${WS_ROOT}', 'install', 'hmr_sim', 'share', 'hmr_sim', 'launch'))
from _world_registry import WORLDS
print(' '.join(sorted(WORLDS.keys())))
"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)   WS_ROOT="$2";      shift 2 ;;
    --max-steps)   MAX_STEPS="$2";     shift 2 ;;
    --worlds)      CUSTOM_WORLDS="$2"; shift 2 ;;
    --planners)    PLANNERS="$2";      shift 2 ;;
    --results-dir) RESULTS_DIR="$2";   shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage:
  test_all_worlds.sh [options]

Runs run_exp5_batch.sh across all registered worlds (or a subset) with
a single start pose and reduced step count for quick validation.

Options:
  --workspace <path>       Workspace root (default: auto-detected)
  --max-steps <n>          Steps per trial (default: 5)
  --worlds "w1 w2"         Space-separated subset (default: all registered)
  --planners "p:m ..."     Planner/map pairs (default: "eig:dscovox")
  --results-dir <path>     Output dir (default: results/all_worlds_test)
  -h, --help               Show this help
EOF
      exit 0 ;;
    *) echo "ERROR: unknown arg: $1" >&2; exit 1 ;;
  esac
done

source_setups

if [[ -n "${CUSTOM_WORLDS}" ]]; then
  WORLDS_STR="${CUSTOM_WORLDS}"
else
  WORLDS_STR="$(get_registered_worlds)"
fi

POSES_FILE="$(mktemp /tmp/exp5_single_pose_XXXXXX.csv)"
cat > "${POSES_FILE}" <<'CSV'
start_id,x,y,yaw
s1,0.0,0.0,0.0
CSV

echo "============================================="
echo "  Exp5 all-worlds validation"
echo "  Worlds:   ${WORLDS_STR}"
echo "  Planners: ${PLANNERS}"
echo "  Steps:    ${MAX_STEPS}"
echo "  Results:  ${RESULTS_DIR}"
echo "============================================="
echo

exec bash "${SCRIPT_DIR}/run_exp5_batch.sh" \
  --workspace "${WS_ROOT}" \
  --worlds "${WORLDS_STR}" \
  --planners "${PLANNERS}" \
  --poses-file "${POSES_FILE}" \
  --max-steps "${MAX_STEPS}" \
  --results-dir "${RESULTS_DIR}" \
  --trial-prefix "allworlds" \
  --planner-timeout 300 \
  --resume false
