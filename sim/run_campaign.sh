#!/usr/bin/env bash
# Sequential driver for the comms/reconnection campaign (plan §6, §7).
#
# Runs a list of (arm, seed) cells one at a time through run_explo_sim_rviz.sh,
# one OUTDIR each, and appends a row per cell to <root>/campaign_index.csv.
#
# SEQUENTIAL, not parallel, and not negotiable: every run drives one Gazebo, two
# scovox mappers and two planners on shared cores, and the emulator's airtime
# model and the planner's single-threaded executor both turn CPU contention into
# apparent COMMS behaviour. Two runs at once would not be two independent
# samples, they would be two runs of a slower stack -- and the §3.8 heartbeat
# starvation warning exists precisely because executor lag is indistinguishable
# from a radio outage in the output.
#
# RESUMABLE. A cell whose OUTDIR already contains a run_manifest.txt with a
# run_end_reason line is skipped. A 40-run matrix is a many-hour job on one box;
# it will be interrupted, and re-running the whole thing to recover the tail
# would be worse than the interruption.
#
# Usage:
#   ./run_campaign.sh --root DIR --cells "off:1,off:2,pursuit:1" [--duration 3600]
#                     [--tx 22.0] [--record 2] [--tag phase3]
#   ./run_campaign.sh --root DIR --arms "off,rendezvous,pursuit,hybrid" \
#                     --seeds "1,2,3" ...          # cross product
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROOT=""; CELLS=""; ARMS=""; SEEDS=""; TAG="run"
DURATION="${DURATION_S:-3600}"; TX="30.0"; REC="2"; EXPECT_OUT="1"
EXTRA_ENV=""
# Coverage-termination threshold, passed through so every cell in a campaign
# shares one value. It defines the primary endpoint, so a campaign that mixed
# two of them would be comparing different experiments.
DONE_UNKNOWN="${DONE_UNKNOWN:-0.55}"
while [ $# -gt 0 ]; do
  case "$1" in
    --root)     ROOT="$2"; shift 2;;
    --cells)    CELLS="$2"; shift 2;;
    --arms)     ARMS="$2"; shift 2;;
    --seeds)    SEEDS="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --tx)       TX="$2"; shift 2;;
    --record)   REC="$2"; shift 2;;
    --tag)      TAG="$2"; shift 2;;
    --expect-outage) EXPECT_OUT="$2"; shift 2;;
    --done-unknown) DONE_UNKNOWN="$2"; shift 2;;
    --env)      EXTRA_ENV="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done
[ -n "$ROOT" ] || { echo "FATAL: --root is required" >&2; exit 2; }

# Build the cell list. --cells wins; otherwise cross --arms with --seeds.
if [ -z "$CELLS" ]; then
  [ -n "$ARMS" ] && [ -n "$SEEDS" ] || {
    echo "FATAL: give --cells, or both --arms and --seeds" >&2; exit 2; }
  CELLS=""
  IFS=',' read -ra _A <<< "$ARMS"
  IFS=',' read -ra _S <<< "$SEEDS"
  # Seed-major order: every arm at seed 1 before any arm at seed 2. An
  # interrupted campaign then holds a COMPLETE paired block for the seeds it
  # reached, which is analysable; arm-major would leave the last arm missing
  # from every pair and the paired design unusable.
  for s in "${_S[@]}"; do
    for a in "${_A[@]}"; do
      CELLS="${CELLS:+$CELLS,}$a:$s"
    done
  done
fi

mkdir -p "$ROOT"
INDEX="$ROOT/campaign_index.csv"
[ -f "$INDEX" ] || echo "cell,arm,seed,outdir,rc,end_reason,end_t_sim,wall_s,started_utc" > "$INDEX"

log() { echo "[$(date +%H:%M:%S)] [campaign] $*"; }

IFS=',' read -ra CELL_LIST <<< "$CELLS"
log "$TAG: ${#CELL_LIST[@]} cells -> $ROOT (tx=$TX duration=${DURATION}s record=$REC)"

n_ok=0; n_fail=0; n_skip=0; consec_fail=0
for cell in "${CELL_LIST[@]}"; do
  arm="${cell%%:*}"; seed="${cell##*:}"
  name="${TAG}_${arm}_seed${seed}"
  out="$ROOT/$name"

  if [ -f "$out/run_manifest.txt" ] && grep -q '^run_end_reason=' "$out/run_manifest.txt" 2>/dev/null; then
    log "SKIP $name (already complete)"
    n_skip=$((n_skip + 1))
    continue
  fi
  # A half-finished OUTDIR from an interrupted attempt would otherwise mix its
  # CSVs with the retry's.
  [ -d "$out" ] && { log "clearing partial $name"; rm -rf "$out"; }

  log "START $name"
  t0=$(date +%s)
  env OUTDIR="$out" COMMS=1 TX_POWER="$TX" EXPECT_OUTAGE="$EXPECT_OUT" \
      RECONNECT_MODE="$arm" EXPLOIT=0 RVIZ=0 RECORD="$REC" SEED="$seed" \
      DURATION_S="$DURATION" STOP_ON_DONE=1 GATES_STRICT=1 \
      DONE_UNKNOWN="$DONE_UNKNOWN" \
      $EXTRA_ENV \
      "$HERE/run_explo_sim_rviz.sh" > "$ROOT/$name.console.log" 2>&1
  rc=$?
  wall=$(( $(date +%s) - t0 ))

  reason=$(sed -n 's/^run_end_reason=//p' "$out/run_manifest.txt" 2>/dev/null | head -1)
  endt=$(sed -n 's/^run_end_t_sim=//p' "$out/run_manifest.txt" 2>/dev/null | head -1)
  echo "$cell,$arm,$seed,$out,$rc,${reason:-none},${endt:-},$wall,$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$INDEX"

  if [ "$rc" = 0 ]; then
    log "OK $name rc=0 ${reason:-?} t_sim=${endt:-?} wall=${wall}s"
    n_ok=$((n_ok + 1)); consec_fail=0
  else
    log "FAIL $name rc=$rc wall=${wall}s — see $ROOT/$name.console.log"
    n_fail=$((n_fail + 1)); consec_fail=$((consec_fail + 1))
    # Three in a row is a broken stack, not bad luck, and grinding through 30
    # more cells to produce 30 more identical failures wastes hours and buries
    # the first, most diagnosable one under the noise.
    if [ "$consec_fail" -ge 3 ]; then
      log "ABORT: $consec_fail consecutive failures — stopping the campaign"
      break
    fi
  fi

  # Gazebo needs a moment to release the DDS domain before the next bring-up,
  # and a stale participant makes the next run's discovery gates flaky.
  sleep 10
done

log "$TAG done: ok=$n_ok fail=$n_fail skipped=$n_skip"
log "index: $INDEX"
[ "$n_fail" = 0 ]
