#!/usr/bin/env bash
# Sequential driver for the comms/reconnection campaign (plan §6, §7).
#
# Runs a list of (arm, seed) cells one at a time through run_explo_sim_rviz.sh,
# one OUTDIR each, and appends a row per cell to <root>/campaign_index.csv.
#
# Cells run strictly one at a time: parallel runs contend for CPU, which shows
# up as apparent comms behaviour. Resumable: a re-run skips the cells that are
# already complete rather than repeating them.
# (notes: campaign-sequential-resumable)
#
# Usage (--root, --duration and --scenario are REQUIRED; see their declarations
# for why the last two lost their defaults):
#   ./run_campaign.sh --root DIR --duration 3000 \
#                     --scenario flatforest_dense_2robot_lidar.yaml \
#                     --cells "off:1,off:2,pursuit:1" \
#                     [--tx 30.0] [--record 0] [--tag phase3]
#   ./run_campaign.sh --root DIR --duration 3000 --scenario ... \
#                     --arms "off,hybrid" --seeds "1,2,3"   # cross product
#   The arms are the planner's four ARM names (off, pursuit, rendezvous,
#   hybrid), no suffixes, and --mission-return must stay 1 (DESIGN_gen34.md).
# Moved comments: docs/sim_notes/run_campaign_notes.md
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROOT=""; CELLS=""; ARMS=""; SEEDS=""; TAG="run"
# No default: DURATION is the censoring horizon and defines the primary
# endpoint, so every campaign must pass --duration explicitly.
# (notes: campaign-duration-no-default)
DURATION=""
# Transmit power. Fixed hardware, identical on both robots, never an
# experimental variable (see the --comms note below) — so unlike DURATION this
# one genuinely has a correct default and keeps it.
TX="30.0"
# Bag recording off by default: the analyses read the event JSONL and the
# planner CSV, never the bags. (notes: campaign-record-off-default)
REC="0"
EXPECT_OUT="1"
# --comms 0 is the ideal-comms control: both robots on direct topics, emulator
# never launched. With --comms 0, --tx and --expect-outage are inert and the
# gate watcher does not demand an outage. (notes: campaign-ideal-comms-control)
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
# The world sets both the link budget and the coverage floor, so changing it is
# a different experiment needing its own DONE_UNKNOWN. Passed explicitly, never
# inherited from the environment; no default.
# (notes: campaign-scenario-no-default)
SCENARIO=""
# Mission return is a first-class flag, not an --env passenger, because it
# applies identically to every cell in both arms. Default 1; 0 reproduces the
# legacy park-in-place design. (notes: campaign-mission-return-flag)
MISSION_RETURN_FLAG="1"
DRY_RUN=0
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
    # Run every validation, print the cell list, launch nothing.
    # (notes: campaign-dry-run)
    --dry-run)  DRY_RUN=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done
[ -n "$ROOT" ] || { echo "FATAL: --root is required" >&2; exit 2; }
# The two experiment-defining knobs that used to carry defaults. See their
# declarations for why a default is worse than nothing here.
[ -n "$DURATION" ] || {
  echo "FATAL: --duration is required (it is the censoring horizon, so it" >&2
  echo "       defines the endpoint; campaigns since 2026-08-27 use 3000)" >&2
  exit 2; }
[ -n "$SCENARIO" ] || {
  echo "FATAL: --scenario is required (it sets both the link budget and the" >&2
  echo "       coverage floor; the current standard is" >&2
  echo "       flatforest_dense_2robot_lidar.yaml)" >&2
  exit 2; }
case "$DURATION" in
  ''|*[!0-9]*)
    echo "FATAL: --duration must be whole seconds (got '$DURATION')" >&2
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
# Both helpers split EXTRA_ENV on whitespace and match the key up to the first
# '=' exactly. Do not substring-match the whole string: GATE_MIDRUN_SILENCE
# would then match MIDRUN_SILENCE. (notes: env-helpers-exact-key-match)
env_has() {
  for _tok in $EXTRA_ENV; do
    case "$_tok" in "$1"=*) return 0;; esac
  done
  return 1
}

# The value --env carries for key $1, else the optional default $2 (empty when
# omitted; keep the set -u safe read). Predicts a cell's manifest for the resume
# guard; arm tokens override it per cell. (notes: env-val-optional-default)
env_val() {
  for _tok in $EXTRA_ENV; do
    case "$_tok" in "$1"=*) printf '%s' "${_tok#*=}"; return 0;; esac
  done
  printf '%s' "${2-}"
}

case "$COMMS_ON" in
  0|1) ;;
  *) echo "FATAL: --comms must be 0 or 1 (got '$COMMS_ON'). Anything else is" >&2
     echo "       neither the emulated arm nor the ideal-comms control, and" >&2
     echo "       run_explo_sim_rviz.sh treats every non-1 value as 0." >&2
     exit 2;;
esac
# --env is expanded after the per-cell COMMS assignment and the last assignment
# wins, so COMMS in --env would override --comms while every record names the
# wrong arm. (notes: guard-comms-in-env)
if env_has COMMS; then
  echo "FATAL: set the emulator with --comms 0|1, not --env COMMS=... --" >&2
  echo "       --env is expanded last, so the passenger wins over the flag and" >&2
  echo "       every record of the campaign would name the wrong arm." >&2
  exit 2
fi
# Each is set per cell or defines the arm, and --env is expanded last, so any of
# them in --env would override every cell while OUTDIR names, index and manifest
# claim otherwise. CELL_WORLD and TEAM_WORLD stay allowed.
# (notes: guard-blocked-env-knobs)
for _blocked in RECONNECT_MODE SEED GLOBAL_ALLOC RECONNECT_GATE \
                RENDEZVOUS_SCHEDULE PURSUIT_PREDICTOR COORD_CLAIM_R \
                ALLOC_POS_TTL ARM; do
  if env_has "$_blocked"; then
    echo "FATAL: $_blocked is set per cell (see the env line at the bottom of" >&2
    echo "       this script) and --env is expanded after it, so --env" >&2
    echo "       $_blocked=... would override EVERY cell while the directory" >&2
    echo "       names, the index and the gate all still claim otherwise." >&2
    echo "       Use --arms/--cells for the arm and --seeds for the seed." >&2
    exit 2
  fi
done
unset _blocked

case "$MISSION_RETURN_FLAG" in
  0|1) ;;
  *) echo "FATAL: --mission-return must be 0 or 1 (got '$MISSION_RETURN_FLAG')" >&2; exit 2;;
esac
if [ "$MISSION_RETURN_FLAG" != "1" ]; then
  echo "FATAL: the planner always homes; --mission-return 0 would be" >&2
  echo "       recorded and never run." >&2
  exit 2
fi
# Gen-33 knobs the planner never reads; the runner refuses them too, but a
# cell that dies at launch costs a bring-up each.
for _g33 in LINK_GATE MIDRUN_SILENCE DONE_SEEK EXPLOIT; do
  if env_has "$_g33"; then
    echo "FATAL: --env $_g33=...: the planner has no such knob (it was gen" >&2
    echo "       33's), and the runner refuses it." >&2
    exit 2
  fi
done
unset _g33

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

# The arms are the four ARM names, whole: gen 33's suffixes (_seek, _ttl, _r)
# and mtare_ tokens mean nothing to the planner.
for _a in $(printf '%s' "$CELLS" | tr ',' ' '); do
  case "${_a%%:*}" in
    off|pursuit|rendezvous|hybrid) ;;
    *) echo "FATAL: arm '${_a%%:*}' is not an arm" >&2
       echo "       (off|pursuit|rendezvous|hybrid, no suffixes)." >&2
       exit 2;;
  esac
done
unset _a
# What the resume guard expects of the gen-33 lines the runner still writes:
# it pins LINK_GATE=0 and leaves MIDRUN_SILENCE at its default (90), and
# TEAM_WORLD_HZ is compared numerically since the launcher flt()s it.
# (notes: resume-req-values-outlive-guard)
LINK_GATE_REQ=0
LINK_GATE_EFFECTIVE_REQ=0
MIDRUN_SILENCE_REQ=90
TEAM_WORLD_HZ_REQ="$(env_val TEAM_WORLD_HZ 1.0)"

# Guards read only EXTRA_ENV and the launch line strips these names, so warn
# when one is exported: only --env reaches the cells. MAP_AGREE_MAX_PCT is read
# by nothing but stays in both lists on purpose. (notes: ambient-knob-warning)
for _amb in LINK_GATE MIDRUN_SILENCE \
            CELL_WORLD TEAM_WORLD TEAM_WORLD_HZ GLOBAL_ALLOC RECONNECT_GATE \
            RENDEZVOUS_SCHEDULE PURSUIT_PREDICTOR \
            STOP_ON_DONE DONE_GRACE_S HANG_HB GATES_STRICT POLL_S \
            CLOCK_EVERY_S CLOCK_DEADMAN_S CLOCK_FAIL_MAX MAP_AGREE_MAX_PCT \
            GZ_GUI; do
  if [ -n "${!_amb+x}" ]; then
    echo "NOTE: $_amb=${!_amb} is exported in this shell and will be IGNORED --" >&2
    echo "      the per-cell launch strips it so the campaign's guards cannot be" >&2
    echo "      bypassed by the ambient environment. Pass --env $_amb=... if you" >&2
    echo "      meant it; the guards read that." >&2
  fi
done
unset _amb

# Strips every defaulted knob the runner reads, derived from
# run_explo_sim_rviz.sh so it cannot go stale; --env still wins. RUNNER_ENV_KEEP
# names inherited infrastructure: DDS domain, display, Gazebo partition, snap
# paths. (notes: runner-strip-derived)
# One line on purpose: the awk below matches " NAME " inside it, so a name
# wrapped onto a continuation would be bounded by a newline instead of a space
# and would silently stop being kept.
RUNNER_ENV_KEEP="ROS_DOMAIN_ID DISPLAY IGN_PARTITION XDG_DATA_DIRS_VSCODE_SNAP_ORIG XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
RUNNER_STRIP=$(awk -v keep=" $RUNNER_ENV_KEEP " '
  { line = $0
    sub(/^[ \t]*#.*/, "", line)
    while (match(line, /\$\{[A-Z][A-Z0-9_]+:?[-=]/)) {
      tok  = substr(line, RSTART, RLENGTH)
      line = substr(line, RSTART + RLENGTH)
      sub(/^\$\{/, "", tok); sub(/:?[-=]$/, "", tok)
      if (length(tok) >= 3 && index(keep, " " tok " ") == 0) seen[tok] = 1
    } }
  END { for (n in seen) printf " -u %s", n }
' "$HERE/run_explo_sim_rviz.sh")
# A scan that silently returns nothing would silently drop the strip list; fewer
# than ten derived knobs means the scan broke, not that the knobs went away.
# (notes: runner-strip-sanity-floor)
_nstrip=$(printf '%s\n' "$RUNNER_STRIP" | tr ' ' '\n' | grep -c '^-u$' || true)
if [ "${_nstrip:-0}" -lt 10 ]; then
  echo "FATAL: derived only ${_nstrip:-0} ambient knob(s) from" >&2
  echo "       $HERE/run_explo_sim_rviz.sh -- the scan is broken, not the" >&2
  echo "       runner. Every \${NAME:-default} knob in that file would reach" >&2
  echo "       each cell from the launching shell unchecked. Refusing to run." >&2
  exit 2
fi
unset _nstrip

if [ "$DRY_RUN" = "1" ]; then
  echo "DRY RUN: every validation passed, nothing launched."
  echo "  tag=$TAG comms=$COMMS_ON tx=$TX duration=${DURATION}s record=$REC"
  echo "  scenario=$SCENARIO mission_return=$MISSION_RETURN_FLAG"
  echo "  done_criterion=$DONE_CRITERION done_unknown=$DONE_UNKNOWN"
  echo "  env='$EXTRA_ENV'"
  echo "  ambient knobs stripped from every cell:$RUNNER_STRIP"
  echo "  cells=$CELLS"
  exit 0
fi

mkdir -p "$ROOT"
INDEX="$ROOT/campaign_index.csv"
[ -f "$INDEX" ] || echo "cell,arm,seed,outdir,rc,end_reason,end_t_sim,wall_s,started_utc" > "$INDEX"

log() { echo "[$(date +%H:%M:%S)] [campaign] $*"; }

# Campaign-wide expectations for three knobs the resume guard compares; only
# --env sets them. Defaults duplicate the runner's, and campaign_guard_calib.sh
# fails if they drift. (notes: resume-radio-knob-defaults)
TREE_ATTEN_REQ=$(env_val TREE_ATTEN 70.0)
MAX_RANGE_REQ=$(env_val MAX_RANGE 30.0)
CELL_SIZE_REQ=$(env_val CELL_SIZE_M 10.0)
# The separation term's three knobs, defaults duplicated from the runner. All
# three are guarded: the radius and freshness bound define sep_peer_dist_m and
# sep_eligible_peers in every arm, even at weight 0.
# (notes: resume-separation-knobs)
SEPARATION_WEIGHT_REQ=$(env_val SEPARATION_WEIGHT 0)
SEPARATION_RADIUS_REQ=$(env_val SEPARATION_RADIUS_M 20)
SEPARATION_MAX_AGE_REQ=$(env_val SEPARATION_MAX_AGE_SEC 10)

IFS=',' read -ra CELL_LIST <<< "$CELLS"
if [ "$COMMS_ON" = "0" ]; then
  log "$TAG: ${#CELL_LIST[@]} cells -> $ROOT (IDEAL COMMS, no emulator;" \
      "duration=${DURATION}s record=$REC)"
else
  log "$TAG: ${#CELL_LIST[@]} cells -> $ROOT (tx=$TX duration=${DURATION}s record=$REC scenario=$SCENARIO)"
fi

n_ok=0; n_fail=0; n_skip=0; consec_fail=0
# Cells that are COMPLETE but not CLEAN. Tracked separately from n_skip because
# they are the ones the summary line hid: `skipped=N` is the same number whether
# every banked cell passed its gates or none of them did, and gate_g8 -- the only
# thing that reads the verdict -- does not run until the campaign is over.
n_susp=0; SUSP_CELLS=""
for cell in "${CELL_LIST[@]}"; do
  arm="${cell%%:*}"; seed="${cell##*:}"
  name="${TAG}_${arm}_seed${seed}"
  out="$ROOT/$name"

  # Every arm runs mtare_off's stack (the runner pins it), so the resume
  # expectations below are mtare_off's; the arm itself is checked through the
  # manifest's arm= line. The gen-33 knobs keep their manifest lines at these
  # values. (notes: resume-mtare-knob-expectations)
  CELL_WORLD_REQ=1; TEAM_WORLD_REQ=1
  GLOBAL_ALLOC_REQ=1; RECONNECT_GATE_REQ=silence; RENDEZVOUS_SCHEDULE_REQ=0
  PURSUIT_PREDICTOR_REQ=trail

  # "Complete" means reached an end reason AND passed its run-time gates. A run
  # that dropped relay traffic reaches all_done exactly like a good one, so
  # resuming on run_end_reason alone would skip every invalid cell forever and
  # quietly hand the analysis a matrix of corrupted maps.
  if [ -f "$out/run_manifest.txt" ] && grep -q '^run_end_reason=' "$out/run_manifest.txt" 2>/dev/null; then
    # A banked cell satisfies a resume only if it ran the same mission
    # definition; a mission_return mismatch or missing line aborts rather than
    # skip or overwrite. (notes: resume-mission-definition)
    want_mr="false"; [ "$MISSION_RETURN_FLAG" = "1" ] && want_mr="true"
    # Every key here changes what the endpoints mean; any mismatch aborts the
    # resume. An absent key counts as a mismatch, since an older manifest cannot
    # be shown to agree. (notes: resume-string-key-guard)
    # node and arm: the cell must be a gen-34 cell of this arm. A gen-33 cell
    # predates the key or says gen33. (notes: resume-string-key-guard)
    have_node=$(sed -n "s/^node=//p" "$out/run_manifest.txt" 2>/dev/null | head -1)
    have_arm=$(sed -n "s/^arm=//p" "$out/run_manifest.txt" 2>/dev/null | head -1)
    if [ "$have_node" != "gen34" ] || [ "$have_arm" != "$arm" ]; then
      log "ABORT: $name is complete but its manifest says node=${have_node:-<absent>}"
      log "       arm=${have_arm:-<absent>}, while this campaign runs"
      log "       node=gen34 arm=$arm. Same name, different"
      log "       experiment — refusing to skip OR overwrite. Use a fresh --root/--tag."
      exit 2
    fi
    for kv in \
      "mission_return_enabled=$want_mr" \
      "scenario=$SCENARIO" \
      "duration_s=$DURATION" \
      "done_criterion=$DONE_CRITERION" \
      "cell_world=$CELL_WORLD_REQ" \
      "team_world=$TEAM_WORLD_REQ" \
      "global_alloc=$GLOBAL_ALLOC_REQ" \
      "reconnect_gate=$RECONNECT_GATE_REQ" \
      "rendezvous_schedule=$RENDEZVOUS_SCHEDULE_REQ" \
      "pursuit_predictor=$PURSUIT_PREDICTOR_REQ" \
      "link_gate=$LINK_GATE_REQ" \
      "link_gate_effective=$LINK_GATE_EFFECTIVE_REQ" \
      "done_seek_enabled=false"
    do
      k="${kv%%=*}"; want="${kv#*=}"
      have=$(sed -n "s/^$k=//p" "$out/run_manifest.txt" 2>/dev/null | head -1)
      if [ "${have:-<absent>}" != "$want" ]; then
        log "ABORT: $name is complete but its manifest says $k=${have:-<absent>},"
        log "       while this campaign runs $k=$want. Same name, different"
        log "       experiment — refusing to skip OR overwrite. Use a fresh --root/--tag."
        exit 2
      fi
    done
    # Numeric keys compare by value, not spelling. none (no override passed) is
    # matched as a string. Both sides are trimmed and must match a number regex:
    # awk reads junk as 0 and nan as equal to anything.
    # (notes: resume-numeric-key-guard)
    for kv in \
      "done_unknown_fraction=$DONE_UNKNOWN" \
      "tx_power_dbm=$TX" \
      "tree_attenuation_db=$TREE_ATTEN_REQ" \
      "max_range_m=$MAX_RANGE_REQ" \
      "cell_size_m=$CELL_SIZE_REQ" \
      "coord_claim_radius_override=none" \
      "alloc_peer_pos_max_age_sec=none" \
      "separation_weight=$SEPARATION_WEIGHT_REQ" \
      "separation_radius_m=$SEPARATION_RADIUS_REQ" \
      "separation_max_age_sec=$SEPARATION_MAX_AGE_REQ" \
      "reconnect_midrun_silence_sec=$MIDRUN_SILENCE_REQ" \
      "team_world_hz=$TEAM_WORLD_HZ_REQ"
    do
      k="${kv%%=*}"; want="${kv#*=}"
      have=$(sed -n "s/^$k=//p" "$out/run_manifest.txt" 2>/dev/null | head -1)
      same=0
      if [ "$want" = "none" ] || [ "${have:-<absent>}" = "none" ]; then
        if [ "${have:-<absent>}" = "$want" ]; then same=1; fi
      elif awk -v a="${have:-}" -v b="$want" \
             'BEGIN {
                gsub(/^[ \t]+|[ \t]+$/, "", a)
                gsub(/^[ \t]+|[ \t]+$/, "", b)
                num = "^[+-]?([0-9]+\\.?[0-9]*|\\.[0-9]+)([eE][+-]?[0-9]+)?$"
                exit !(a ~ num && b ~ num && a + 0 == b + 0) }'; then
        same=1
      fi
      if [ "$same" != "1" ]; then
        case "$k" in
          done_unknown_fraction) why="that is the primary endpoint's threshold";;
          tx_power_dbm|tree_attenuation_db|max_range_m)
                                 why="that is the radio regime, and this project does not pool across it";;
          cell_size_m)           why="that is the unit the coverage census counts";;
          *)                     why="that is an independent variable";;
        esac
        log "ABORT: $name is complete but its manifest says $k=${have:-<absent>},"
        log "       while this campaign runs $k=$want — $why."
        log "       Same name, different experiment — refusing to skip OR"
        log "       overwrite. Use a fresh --root/--tag."
        exit 2
      fi
    done
    # n_robots is checked for consistency across the tag's banked cells, not
    # against a prediction. Detective only: cells this invocation runs are not
    # compared, so a mixed tag aborts one invocation late.
    # (notes: resume-team-size-consistency)
    have_n=$(sed -n 's/^n_robots=//p' "$out/run_manifest.txt" 2>/dev/null | head -1)
    if [ -n "${have_n:-}" ]; then
      if [ -z "${BANKED_N_ROBOTS:-}" ]; then
        BANKED_N_ROBOTS="$have_n"
      elif [ "$have_n" != "$BANKED_N_ROBOTS" ]; then
        log "ABORT: $name is complete with n_robots=$have_n, but another banked"
        log "       cell of this tag has n_robots=$BANKED_N_ROBOTS. One tag,"
        log "       two team sizes — endpoints computed over robots are not"
        log "       comparable across N. Use a fresh --root/--tag."
        exit 2
      fi
    fi
    # Read the verdict itself (CLEAN, SUSPECT, INVALID or absent), not just a
    # test for INVALID. tail -1, not head -1: the teardown appends the key, so
    # the last one is the verdict reached. (notes: resume-verdict-read)
    banked_verdict=$(sed -n 's/^run_gates_verdict=//p' \
                       "$out/run_manifest.txt" 2>/dev/null | tail -1)
    # Only INVALID cells are redone by default; REDO_SUSPECT=1 opts in to
    # re-running SUSPECT ones. Re-rolling conditions the retained sample on
    # whatever made the cell fail. (notes: resume-redo-suspect-opt-in)
    if [ "$banked_verdict" = "INVALID" ] || \
       { [ "${REDO_SUSPECT:-0}" = "1" ] && [ -n "$banked_verdict" ] \
         && [ "$banked_verdict" != "CLEAN" ]; }; then
      # Keep the discarded attempt's manifest and gate report before the retry
      # overwrites them, so the retained sample is not silently conditioned on
      # outage severity. (notes: resume-keep-attempt-evidence)
      att="$ROOT/${name}.attempts"; mkdir -p "$att"
      k=$(( $(ls -1 "$att" 2>/dev/null | grep -c '_manifest.txt$') + 1 ))
      cp "$out/run_manifest.txt" "$att/attempt${k}_manifest.txt" 2>/dev/null || true
      cp "$out/comms_gates.txt"  "$att/attempt${k}_gates.txt"    2>/dev/null || true
      log "REDO $name (attempt $k banked ${banked_verdict:-<absent>}; evidence kept in ${name}.attempts/)"
    else
      if [ "${banked_verdict:-}" = "CLEAN" ]; then
        log "SKIP $name (already complete)"
      else
        # Named, counted, and repeated in the summary, because this is the one
        # state that costs nothing to notice now and a whole campaign to notice
        # later.
        log "SKIP $name (complete, but run_gates_verdict=${banked_verdict:-<absent>}"
        log "     — gate_g8 hard-fails this cell. REDO_SUSPECT=1 re-runs it instead;"
        log "       read $out/comms_gates.txt before deciding.)"
        n_susp=$((n_susp + 1))
        SUSP_CELLS="${SUSP_CELLS:+$SUSP_CELLS }$name"
      fi
      n_skip=$((n_skip + 1))
      continue
    fi
  fi
  # A half-finished OUTDIR from an interrupted attempt would otherwise mix its
  # CSVs with the retry's.
  [ -d "$out" ] && { log "clearing partial $name"; rm -rf "$out"; }

  # Disk guard: stop with a whole cell's margin left, because running out of
  # space yields cells that look complete but are silently truncated.
  # (notes: campaign-disk-guard)
  free_mb=$(df -Pm "$ROOT" | awk 'NR==2 {print $4}')
  if [ "${free_mb:-0}" -lt "${MIN_FREE_MB:-8192}" ]; then
    log "ABORT: only ${free_mb} MB free under $ROOT (need ${MIN_FREE_MB:-8192})"
    log "       stopping before $name rather than writing a truncated cell."
    break
  fi

  log "START $name (arm=$arm mission_return=$MISSION_RETURN_FLAG free=${free_mb}MB)"
  t0=$(date +%s)
  # An ideal-comms cell has no link to drop, so demanding an outage would fail
  # every gate; force expect_outage off rather than trusting the caller.
  cell_expect="$EXPECT_OUT"
  [ "$COMMS_ON" = "0" ] && cell_expect=0

  # Invariant: every knob read through env_val is either stripped here or
  # assigned on this line; campaign_guard_calib.sh checks it. RUNNER_STRIP is
  # the superset; keep the literal -u names, the calib reads them.
  # (notes: launch-strip-list-invariant)
  env $RUNNER_STRIP \
      -u LINK_GATE -u MIDRUN_SILENCE \
      -u CELL_WORLD -u TEAM_WORLD -u TEAM_WORLD_HZ \
      -u GLOBAL_ALLOC -u RECONNECT_GATE -u RENDEZVOUS_SCHEDULE \
      -u PURSUIT_PREDICTOR \
      -u TREE_ATTEN -u MAX_RANGE -u CELL_SIZE_M \
      -u SEPARATION_WEIGHT -u SEPARATION_RADIUS_M -u SEPARATION_MAX_AGE_SEC \
      -u DONE_GRACE_S -u HANG_HB -u POLL_S \
      -u CLOCK_EVERY_S -u CLOCK_DEADMAN_S -u CLOCK_FAIL_MAX \
      -u MAP_AGREE_MAX_PCT -u GZ_GUI \
      OUTDIR="$out" COMMS="$COMMS_ON" TX_POWER="$TX" EXPECT_OUTAGE="$cell_expect" \
      ARM="$arm" \
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
    # rc=0 does not mean the gates certified the cell (a SUSPECT cell exits 0),
    # so the verdict is logged on the OK line and non-CLEAN cells are counted.
    # (notes: ok-line-carries-verdict)
    fresh_verdict=$(sed -n 's/^run_gates_verdict=//p' \
                      "$out/run_manifest.txt" 2>/dev/null | tail -1)
    log "OK $name rc=0 ${reason:-?} t_sim=${endt:-?} wall=${wall}s gates=${fresh_verdict:-<absent>}"
    if [ "${fresh_verdict:-}" != "CLEAN" ]; then
      log "     ^ NOT CLEAN — this cell banks but gate_g8 hard-fails it; see $out/comms_gates.txt"
      n_susp=$((n_susp + 1))
      SUSP_CELLS="${SUSP_CELLS:+$SUSP_CELLS }$name"
    fi
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

log "$TAG done: ok=$n_ok fail=$n_fail skipped=$n_skip not_clean=$n_susp"
# NOT_CLEAN IS NAMED, NOT JUST COUNTED. A count tells the operator to go
# looking; the names tell them where, and this is the last line of output the
# campaign produces, so anything not said here has to be reconstructed from a
# few hundred console logs later.
if [ "$n_susp" != 0 ]; then
  log "  $n_susp cell(s) are COMPLETE but not run_gates_verdict=CLEAN. They are"
  log "  banked and will be skipped by every later resume, and gate_g8 hard-fails"
  log "  each of them, so the campaign is short by that many cells until they are"
  log "  dealt with (read their comms_gates.txt; REDO_SUSPECT=1 re-runs them):"
  for s in $SUSP_CELLS; do log "    $s"; done
fi
log "index: $INDEX"
# The exit status still tracks FAILURES ONLY. not_clean cells exited 0 by the
# runner's own contract and re-rolling them is the operator's call, so turning
# them into a non-zero status here would abort the ts4 chain on a condition the
# chain cannot resolve. They are reported, loudly, and that is the remedy.
[ "$n_fail" = 0 ]
