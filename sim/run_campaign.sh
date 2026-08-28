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
# --comms 0 runs the IDEAL-COMMS CONTROL: run_explo_sim_rviz.sh leaves both
# robots on their direct topics, one broadcast domain, emulator never launched.
# That is the honest way to express "assume comms never fails" -- the earlier
# control instead ran COMMS=1 at tx_power_dbm=160, which does not describe any
# radio a robot could carry. Transmit power is fixed hardware and identical on
# both robots; it is not an experimental variable. Severity belongs to the
# environment (tree density, separation), never to the radio.
# With --comms 0, --tx and --expect-outage are inert and the gate watcher is
# told not to demand an outage.
COMMS_ON="1"
EXTRA_ENV=""
# Coverage-termination threshold, passed through so every cell in a campaign
# shares one value. It defines the primary endpoint, so a campaign that mixed
# two of them would be comparing different experiments.
DONE_UNKNOWN="${DONE_UNKNOWN:-0.64}"
# WHICH completion rule, passed through for the same reason as the threshold:
# "latch" and "streak" define different endpoints, so a campaign that mixed them
# would be comparing different experiments under one column name. See the long
# note in run_explo_sim_rviz.sh for why "latch" is the default.
DONE_CRITERION="${DONE_CRITERION:-latch}"
# The world. It sets BOTH the link budget (stems in the Fresnel corridor) and
# the coverage floor, so it is not a free knob: a campaign that changes it is a
# different experiment and needs its own DONE_UNKNOWN. Passed explicitly rather
# than inherited from the environment so a stale exported SCENARIO cannot
# silently relabel a campaign.
SCENARIO="${SCENARIO:-flatforest_2robot_lidar.yaml}"
# Mission return (see run_explo_sim_rviz.sh). An explicit first-class flag, NOT
# an --env passenger, because it applies to EVERY cell identically in BOTH arms
# — it is part of the mission definition, not an arm. Default 1: from 2026-08-27
# campaigns measure mission end time as the primary endpoint, and their
# exploration-finish numbers are a NEW endpoint never pooled with banked runs.
# Pass --mission-return 0 only to reproduce the legacy park-in-place design.
MISSION_RETURN_FLAG="1"
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
    --scenario) SCENARIO="$2"; shift 2;;
    --expect-outage) EXPECT_OUT="$2"; shift 2;;
    --comms)    COMMS_ON="$2"; shift 2;;
    --done-unknown) DONE_UNKNOWN="$2"; shift 2;;
    --done-criterion) DONE_CRITERION="$2"; shift 2;;
    --mission-return) MISSION_RETURN_FLAG="$2"; shift 2;;
    --env)      EXTRA_ENV="$2"; shift 2;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done
[ -n "$ROOT" ] || { echo "FATAL: --root is required" >&2; exit 2; }
# EXTRA_ENV is expanded AFTER the per-cell assignments below, so a DONE_SEEK in
# it would win over the arm-name suffix and flip every cell to the same side
# while the OUTDIR names still claimed an A/B. That failure is invisible in the
# campaign index and only recoverable from the manifests, so refuse it here.
case " $EXTRA_ENV " in
  *" DONE_SEEK="*|*"DONE_SEEK="*)
    echo "FATAL: set the post-latch coast with the arm suffix (e.g. hybrid_seek)," >&2
    echo "       not with --env DONE_SEEK=... -- --env applies to EVERY cell and" >&2
    echo "       would silently collapse the A/B into one arm." >&2
    exit 2;;
esac
# Same shape of failure, different mechanism: MISSION_RETURN in --env would
# win over the explicit flag below, so the campaign index and the operator's
# intent could disagree while every OUTDIR name looked right.
case " $EXTRA_ENV " in
  *"MISSION_RETURN="*)
    echo "FATAL: set mission return with --mission-return 0|1, not --env" >&2
    echo "       MISSION_RETURN=... -- the flag is recorded per cell and guarded" >&2
    echo "       on resume; an --env passenger is neither." >&2
    exit 2;;
esac
case "$MISSION_RETURN_FLAG" in
  0|1) ;;
  *) echo "FATAL: --mission-return must be 0 or 1 (got '$MISSION_RETURN_FLAG')" >&2; exit 2;;
esac

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
if [ "$COMMS_ON" = "0" ]; then
  log "$TAG: ${#CELL_LIST[@]} cells -> $ROOT (IDEAL COMMS, no emulator;" \
      "duration=${DURATION}s record=$REC)"
else
  log "$TAG: ${#CELL_LIST[@]} cells -> $ROOT (tx=$TX duration=${DURATION}s record=$REC scenario=$SCENARIO)"
fi

n_ok=0; n_fail=0; n_skip=0; consec_fail=0
for cell in "${CELL_LIST[@]}"; do
  arm="${cell%%:*}"; seed="${cell##*:}"
  name="${TAG}_${arm}_seed${seed}"
  out="$ROOT/$name"

  # Arm-name suffix "_seek" = same RECONNECT_MODE, post-latch coast on.
  #
  # --env sets ONE environment for every cell, so it cannot express a
  # within-campaign A/B; without this the treated and control arms would need
  # two run_campaign.sh calls, and 30.27 measured that a session boundary moves
  # the geomean ~1.08x -- the same size as the effect under test. Encoding the
  # switch in the arm token keeps both sides inside one invocation, interleaved
  # by the seed-major loop above, and keeps them in separate OUTDIRs with
  # distinct names so no analysis script has to know about the feature.
  cell_mode="$arm"; cell_seek="0"
  case "$arm" in
    *_seek) cell_mode="${arm%_seek}"; cell_seek="1";;
  esac

  # "Complete" means reached an end reason AND passed its run-time gates. A run
  # that dropped relay traffic reaches all_done exactly like a good one, so
  # resuming on run_end_reason alone would skip every invalid cell forever and
  # quietly hand the analysis a matrix of corrupted maps.
  if [ -f "$out/run_manifest.txt" ] && grep -q '^run_end_reason=' "$out/run_manifest.txt" 2>/dev/null; then
    # A cell may only satisfy a resume if it ran the same MISSION DEFINITION.
    # mission_return changes what both endpoints mean, so a completed cell from
    # the other side of that switch (or from before it existed — no line at
    # all) is not "already complete", it is a different experiment sharing the
    # directory name. That is an operator error to stop on, not to paper over
    # with a silent REDO that would overwrite banked data.
    want_mr="false"; [ "$MISSION_RETURN_FLAG" = "1" ] && want_mr="true"
    have_mr=$(sed -n 's/^mission_return_enabled=//p' "$out/run_manifest.txt" 2>/dev/null | head -1)
    if [ "${have_mr:-missing}" != "$want_mr" ]; then
      log "ABORT: $name is complete but its manifest says mission_return_enabled=${have_mr:-<absent>},"
      log "       while this campaign runs --mission-return $MISSION_RETURN_FLAG. Same name, different"
      log "       experiment — refusing to skip OR overwrite. Use a fresh --root/--tag."
      exit 2
    fi
    if grep -q '^run_gates_verdict=INVALID' "$out/run_manifest.txt" 2>/dev/null; then
      # Keep the evidence. A gate can fail *because the link never dropped*, so
      # re-rolling preferentially discards mild-outage realisations; deleting the
      # attempt makes a cell that needed four tries indistinguishable from one
      # that passed first time, and the retained sample ends up silently
      # conditioned on outage severity. Copy the two small files that record what
      # the discarded attempt saw before the retry overwrites it.
      att="$ROOT/${name}.attempts"; mkdir -p "$att"
      k=$(( $(ls -1 "$att" 2>/dev/null | grep -c '_manifest.txt$') + 1 ))
      cp "$out/run_manifest.txt" "$att/attempt${k}_manifest.txt" 2>/dev/null || true
      cp "$out/comms_gates.txt"  "$att/attempt${k}_gates.txt"    2>/dev/null || true
      log "REDO $name (attempt $k failed its gates; evidence kept in ${name}.attempts/)"
    else
      log "SKIP $name (already complete)"
      n_skip=$((n_skip + 1))
      continue
    fi
  fi
  # A half-finished OUTDIR from an interrupted attempt would otherwise mix its
  # CSVs with the retry's.
  [ -d "$out" ] && { log "clearing partial $name"; rm -rf "$out"; }

  # Disk guard. A lean-record cell is ~340 MB and a 90-cell matrix is ~31 GB on
  # a box that is already 91% full, so this campaign can plausibly fill the
  # disk overnight. Out of space does not fail cleanly: the bag writer, the
  # event log and the manifest all fail independently and produce cells that
  # look complete and are silently truncated. Stopping with a whole cell's
  # margin left is much cheaper than finding that out afterwards.
  free_mb=$(df -Pm "$ROOT" | awk 'NR==2 {print $4}')
  if [ "${free_mb:-0}" -lt "${MIN_FREE_MB:-8192}" ]; then
    log "ABORT: only ${free_mb} MB free under $ROOT (need ${MIN_FREE_MB:-8192})"
    log "       stopping before $name rather than writing a truncated cell."
    break
  fi

  log "START $name (reconnect_mode=$cell_mode done_seek=$cell_seek mission_return=$MISSION_RETURN_FLAG free=${free_mb}MB)"
  t0=$(date +%s)
  # An ideal-comms cell has no link to drop, so demanding an outage would fail
  # every gate; force expect_outage off rather than trusting the caller.
  cell_expect="$EXPECT_OUT"
  [ "$COMMS_ON" = "0" ] && cell_expect=0

  env OUTDIR="$out" COMMS="$COMMS_ON" TX_POWER="$TX" EXPECT_OUTAGE="$cell_expect" \
      RECONNECT_MODE="$cell_mode" DONE_SEEK="$cell_seek" \
      MISSION_RETURN="$MISSION_RETURN_FLAG" \
      EXPLOIT=0 RVIZ=0 RECORD="$REC" SEED="$seed" \
      DURATION_S="$DURATION" STOP_ON_DONE=1 GATES_STRICT=1 \
      DONE_UNKNOWN="$DONE_UNKNOWN" DONE_CRITERION="$DONE_CRITERION" \
      SCENARIO="$SCENARIO" \
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
