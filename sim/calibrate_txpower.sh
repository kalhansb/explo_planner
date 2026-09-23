#!/usr/bin/env bash
# Phase-2 severity calibration (plan §4): sweep tx_power_dbm over REPLAYED poses
# and report the three acceptance numbers for each value.
#
# Offline: fading is a pure function of (seed, tick) and the link model reads
# only poses and the world SDF, so a bag replaces Gazebo and planners. The
# replayed duty only aims; behaviour changes the realised one.
# (notes: calib-offline-replay)
#
# Acceptance, all three from one tx_power_dbm value (§4):
#   * duty cycle 55-70 % disconnected
#   * median outage > 5 s   (the claim TTL — shorter outages never make a peer
#                            read MISSING, so no manoeuvre could ever arm)
#   * time-since-contact at exhaustion < 180 s in most traces
#
# Usage:
#   ./calibrate_txpower.sh --bags DIR[,DIR...] [--powers "30,26,22,18,14"]
#                          [--seed 42] [--out DIR]
#
# Each DIR is a run OUTDIR containing rosbag2/ (RECORD=1 or 2) — i.e. the phase-1
# control runs. Exhaustion time per bag is read from that run's planner CSVs.
# Moved comments: docs/sim_notes/calibrate_txpower_notes.md
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"

BAGS=""; POWERS="30,26,22,18,14,10"; SEED=42; OUT=""
WORLD_SDF="$WS/install/hmr_sim/share/hmr_sim/worlds/flatforest/flatforestv2.sdf"
ROBOTS="atlas,bestla"
while [ $# -gt 0 ]; do
  case "$1" in
    --bags)   BAGS="$2"; shift 2;;
    --powers) POWERS="$2"; shift 2;;
    --seed)   SEED="$2"; shift 2;;
    --out)    OUT="$2"; shift 2;;
    --world)  WORLD_SDF="$2"; shift 2;;
    --robots) ROBOTS="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done
[ -n "$BAGS" ] || { echo "FATAL: --bags is required" >&2; exit 2; }
OUT="${OUT:-/tmp/hmr_calib_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
# set +u around the sourcing: the ROS setup scripts read unbound variables
# (AMENT_TRACE_SETUP_FILES and friends), which under `set -u` aborts the script
# before it does anything. Same dance as run_explo_sim_rviz.sh.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
# Own domain, and NOT the live-run domain 42: a sweep point publishes /clock from
# the bag, and a second /clock on a domain where a real run is in flight would
# corrupt every use_sim_time consumer in it. Sim time is not recoverable.
export ROS_DOMAIN_ID="${CALIB_DOMAIN_ID:-57}"

log() { echo "[$(date +%H:%M:%S)] $*"; }

# One sweep point: <bag_dir> <tx_power> -> writes $OUT/<tag>.csv
sweep_point() {
  local bagdir=$1 tx=$2 tag=$3
  local logdir="$OUT/$tag"; mkdir -p "$logdir"

  # Emulator first, logger second, bag last. Order matters: the emulator
  # discovers relay topics on a 1 Hz timer and the logger has to be subscribed
  # before the first link_states tick, or the trace silently starts late.
  setsid ros2 launch hmr_sim comms_sim.launch.py \
      robots:="$ROBOTS" world_sdf:="$WORLD_SDF" use_sim_time:=true \
      seed:="$SEED" tx_power_dbm:="$tx" \
      > "$logdir/comms.log" 2>&1 &
  local comms_pid=$!
  setsid python3 "$HERE/link_logger.py" --out "$OUT/$tag.csv" \
      --ros-args -p use_sim_time:=true \
      > "$logdir/logger.log" 2>&1 &
  local logger_pid=$!
  sleep 6

  # Play only /clock and the ground-truth poses: the bag also holds the control
  # run's link_states, and replaying them would mix rows at the control's tx
  # power into the swept trace. (notes: calib-play-only-clock-and-poses)
  #
  # --clock is deliberately NOT passed: the bag carries the live run's /clock
  # and letting `bag play` synthesise a second one would race it.
  local play_topics="/clock"
  local rname
  for rname in ${ROBOTS//,/ }; do
    play_topics="$play_topics /$rname/odom_ground_truth"
  done
  ros2 bag play "$bagdir" --rate "${CALIB_RATE:-5.0}" --topics $play_topics \
      > "$logdir/play.log" 2>&1
  sleep 3

  kill -INT -- "-$logger_pid" 2>/dev/null || kill -INT "$logger_pid" 2>/dev/null
  sleep 2
  kill -INT -- "-$comms_pid" 2>/dev/null || kill -INT "$comms_pid" 2>/dev/null
  sleep 2
  kill -KILL -- "-$logger_pid" 2>/dev/null
  kill -KILL -- "-$comms_pid"  2>/dev/null
  wait 2>/dev/null || true
}

echo "bag,tx_power_dbm,trace_csv" > "$OUT/index.csv"
IFS=',' read -ra PLIST <<< "$POWERS"
IFS=',' read -ra BLIST <<< "$BAGS"
for bag in "${BLIST[@]}"; do
  bagdir="$bag/rosbag2"
  [ -d "$bagdir" ] || { log "SKIP $bag — no rosbag2/"; continue; }
  bagname=$(basename "$bag")
  for tx in "${PLIST[@]}"; do
    tag="${bagname}__tx${tx}"
    log "sweep: $bagname @ tx_power_dbm=$tx"
    sweep_point "$bagdir" "$tx" "$tag"
    n=$(wc -l < "$OUT/$tag.csv" 2>/dev/null || echo 0)
    log "  -> $n rows"
    echo "$bagname,$tx,$OUT/$tag.csv" >> "$OUT/index.csv"
  done
done

log "sweep done — summarising"
python3 "$HERE/calib_summary.py" --index "$OUT/index.csv" \
    --runs "$BAGS" --out "$OUT/summary.txt" | tee "$OUT/summary_stdout.txt"
log "results in $OUT"
