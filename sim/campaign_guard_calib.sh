#!/usr/bin/env bash
# Known-answer cases for run_campaign.sh's guards.
#
# Main sections: the campaign's arm and knob refusals (the planner is gen 34:
# four ARM names, no gen-33 knobs); launcher validation; the resume guard.
# Gen 33's link-veto cases went with its node (backup/gen33/).
#
# No case may launch a simulation; the process-census section checks for started
# processes. (notes: guardcal-nothing-launches)
#
# Run: ./campaign_guard_calib.sh   -- exits non-zero on any FAIL.
# Moved comments: docs/sim_notes/campaign_guard_calib_notes.md
set -u
CS="$(cd "$(dirname "$0")" && pwd)/run_campaign.sh"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
fails=0
# Every assertion increments this, so the count quoted in
# comms_reconnection_experiment.md §32.15.5 is emitted by the harness rather
# than counted by hand -- it drifted twice while being maintained that way.
cases=0
# Taken BEFORE any case runs. The final census is a delta against this, so a sim
# already on the box is not blamed on this harness and does not fail the run.
CENSUS_BEFORE=$(ps -e -o pid=,comm= \
  | grep -c -E 'ruby|ign|gz|parameter_br|explo_planner|scovox|dscovox|robot_state|rviz')

# want=ALLOW   -> the campaign must be accepted, i.e. exit 0
# want=<text>  -> the run must refuse (non-zero exit) AND print <text>
#
# ALLOW requires exit 0, not merely the absence of a FATAL, so an unrelated
# refusal is not a pass; a refusal requires its own text, so a campaign that
# died of something else is not one either.
# (notes: guardcal-allow-needs-exit-zero)
g() {
  want="$1"; label="$2"; shift 2
  cases=$((cases+1))
  out=$(timeout 20 "$CS" --root "$TMP" --duration 3000 \
        --scenario flatforest_dense_2robot_lidar.yaml --dry-run "$@" 2>&1)
  rc=$?
  ok=0
  if [ "$want" = ALLOW ]; then
    [ "$rc" = 0 ] && ok=1
  else
    [ "$rc" != 0 ] && printf '%s\n' "$out" | grep -qF -- "$want" && ok=1
  fi
  if [ "$ok" = 1 ]; then
    echo "  PASS  $label"
  else
    echo "  FAIL  $label: rc=$rc, wanted ${want}"
    printf '%s\n' "$out" | grep -E "FATAL|unknown arg" | sed 's/^/          | /'
    fails=$((fails+1))
  fi
}

echo "=== the arms are the planner's four names ==="
g ALLOW "the reference campaign"                   --arms hybrid,off --seeds 1
g ALLOW "all four arms"                            --arms off,pursuit,rendezvous,hybrid --seeds 1
g ALLOW "a --cells list"                           --cells "off:1,hybrid:2"
g ALLOW "the ideal-comms control"                  --arms hybrid,off --seeds 1 --comms 0
g "is not an arm"  "a gen-33 mtare_ token is refused"        --arms mtare_hybrid,off --seeds 1
g "is not an arm"  "a gen-33 _seek suffix is refused"        --arms hybrid_seek,hybrid --seeds 1
g "is not an arm"  "a _r suffix inside --cells is refused"   --cells "off:1,hybrid_r40:2"
g "is not an arm"  "a _ttl suffix is refused"                --arms hybrid_ttl120 --seeds 1
g "is not an arm"  "a mis-cased arm is refused"              --arms Hybrid --seeds 1

echo "=== gen 33's knobs and flags are refused, not carried ==="
g "unknown arg: --node"    "--node is gone with the second planner"   --arms hybrid --seeds 1 --node gen33
g "always homes"           "--mission-return 0 is refused"            --arms hybrid --seeds 1 --mission-return 0
g "--env LINK_GATE=..."    "--env LINK_GATE is refused"               --arms hybrid --seeds 1 --env "LINK_GATE=0"
g "--env MIDRUN_SILENCE=..." "--env MIDRUN_SILENCE is refused"        --arms hybrid --seeds 1 --env "MIDRUN_SILENCE=240"
g "--env EXPLOIT=..."      "--env EXPLOIT is refused"                 --arms hybrid --seeds 1 --env "EXPLOIT=1"
# The key matcher splits on whitespace and compares up to '=': a name that
# merely ends in a refused key is not that key.
g ALLOW "MY_LINK_GATE=0 is not LINK_GATE"          --arms hybrid --seeds 1 --env "MY_LINK_GATE=0"
g ALLOW "GATE_MIDRUN_SILENCE=240 is not MIDRUN_SILENCE" \
                                                   --arms hybrid --seeds 1 --env "GATE_MIDRUN_SILENCE=240"
# The self-check for g()'s refusal verdict: the text has to be the guard's own.
g "unknown arg: --bogus"   "an unrelated refusal names itself"        --arms hybrid,off --seeds 1 --bogus

echo
echo "=== the other refusals ==="
g "not --env COMMS="       "--env COMMS=0 is refused"     --arms hybrid,off --seeds 1 --env "COMMS=0"
g "--comms must be 0 or 1" "--comms 2 is refused"         --arms hybrid,off --seeds 1 --comms 2

echo
echo "=== the ambient environment must not reach the cells ==="
# env without -i inherits the caller's environment, so the per-cell launch must
# strip LINK_GATE and MIDRUN_SILENCE, which the runner would otherwise refuse
# on every cell. Asserted on the launch line.
# (notes: guardcal-ambient-link-gate-strip)
cases=$((cases+1))
if grep -q -- '-u LINK_GATE -u MIDRUN_SILENCE' "$CS"; then
  echo "  PASS  the per-cell launch strips both from the inherited environment"
else
  echo "  FAIL  the per-cell launch does not strip LINK_GATE/MIDRUN_SILENCE, so" \
       "an exported value silently beats the guard"
  fails=$((fails+1))
fi
# And the strip must not break the channel the guards DO read: env applies
# assignments after unsets, so --env has to keep winning, or no cell would
# ever receive an --env setting.
cases=$((cases+1))
if env -u LINK_GATE LINK_GATE=7 sh -c '[ "${LINK_GATE-}" = 7 ]'; then
  echo "  PASS  env applies assignments after -u, so --env still reaches the cell"
else
  echo "  FAIL  env -u swallowed the later assignment: --env would be inert"
  fails=$((fails+1))
fi
# The operator has to be told, not silently overridden.
out=$(LINK_GATE=0 timeout 20 "$CS" --root "$TMP" --duration 3000 \
      --scenario flatforest_dense_2robot_lidar.yaml --dry-run \
      --arms hybrid,off --seeds 1 2>&1)
rc=$?
cases=$((cases+1))
if [ "$rc" = 0 ] && echo "$out" | grep -q "LINK_GATE=0 is exported in this shell and will be IGNORED"; then
  echo "  PASS  an exported LINK_GATE is reported and does not stop the campaign"
else
  echo "  FAIL  an exported LINK_GATE was neither reported nor refused (rc=$rc)"
  fails=$((fails+1))
fi

echo
echo "=== the resume guard's copies of the runner's gen-33 pins must still be true ==="
# The runner pins LINK_GATE=0 in its planner block and leaves MIDRUN_SILENCE
# at its default; the resume guard predicts both manifest lines from its own
# copies. Nothing else links the two files, so read the literals back.
# (notes: guardcal-link-gate-default-readback)
LAUNCHER="$(dirname "$CS")/run_explo_sim_rviz.sh"
_pin=$(sed -n '/^# --- Planner generation/,/^BEST_EFFORT_PRIORITY=/p' "$LAUNCHER" \
       | sed -n 's/^LINK_GATE=\([0-9]*\)$/\1/p' | head -1)
_req=$(sed -n 's/^LINK_GATE_REQ=\([0-9]*\)$/\1/p' "$CS" | head -1)
cases=$((cases+1))
if [ -n "$_pin" ] && [ "$_pin" = "$_req" ]; then
  echo "  PASS  the runner's LINK_GATE pin ($_pin) matches the resume guard's copy"
else
  echo "  FAIL  the runner pins LINK_GATE='${_pin:-UNREADABLE}' but the resume" \
       "guard expects '${_req:-UNREADABLE}'"
  fails=$((fails+1))
fi
_ld=$(sed -n 's/^MIDRUN_SILENCE="$(flt "${MIDRUN_SILENCE:-\([0-9.]*\)}")"$/\1/p' "$LAUNCHER" | head -1)
_req=$(sed -n 's/^MIDRUN_SILENCE_REQ=\([0-9.]*\)$/\1/p' "$CS" | head -1)
cases=$((cases+1))
if [ -n "$_ld" ] && [ "$_ld" = "$_req" ]; then
  echo "  PASS  the runner's MIDRUN_SILENCE default ($_ld) matches the resume guard's copy"
else
  echo "  FAIL  the runner's MIDRUN_SILENCE default is '${_ld:-UNREADABLE}' but the" \
       "resume guard expects '${_req:-UNREADABLE}'"
  fails=$((fails+1))
fi
unset _pin _req _ld

echo
echo "=== --env passengers that would silently collapse the design ==="
# Each of these is set per cell on the env line at the bottom of run_campaign.sh
# and --env is expanded after it, so a passenger wins over the real value while
# every name, index row and manifest still claims the intended one.
g "RECONNECT_MODE is set per cell" "--env RECONNECT_MODE is refused" \
    --arms hybrid,off --seeds 1 --env "RECONNECT_MODE=off"
g "SEED is set per cell"           "--env SEED is refused" \
    --arms hybrid,off --seeds 1,2 --env "SEED=7"
g "ARM is set per cell"            "--env ARM is refused" \
    --arms hybrid,off --seeds 1 --env "ARM=off"
g "--env DONE_SEEK=..."            "--env DONE_SEEK is refused" \
    --arms hybrid,off --seeds 1 --env "DONE_SEEK=1"
g "MISSION_RETURN"                 "--env MISSION_RETURN is refused" \
    --arms hybrid,off --seeds 1 --env "MISSION_RETURN=0"
g "COORD_CLAIM_R is set per cell"  "--env COORD_CLAIM_R is refused" \
    --arms hybrid,off --seeds 1 --env "COORD_CLAIM_R=40"
g ALLOW "MY_SEED=7 must not be read as SEED" --arms hybrid,off --seeds 1 --env "MY_SEED=7"

echo
echo "=== the harness itself must launch nothing ==="
# Asserts the harness launched nothing: the census must equal CENSUS_BEFORE (a
# delta, so a sim already on the box is not blamed). The planted case below
# proves the census can read non-zero. (notes: guardcal-process-census-delta)
census() {
  ps -e -o pid=,comm= \
    | grep -c -E 'ruby|ign|gz|parameter_br|explo_planner|scovox|dscovox|robot_state|rviz'
  return 0            # grep -c exits 1 on no match, and 0 matches is the normal case
}
census_after=$(census)
cases=$((cases+1))
if [ "$census_after" = "$CENSUS_BEFORE" ]; then
  echo "  PASS  process census unchanged across the run ($CENSUS_BEFORE -> $census_after)"
else
  echo "  FAIL  process census moved $CENSUS_BEFORE -> $census_after: this harness" \
       "started or killed a simulation"
  fails=$((fails+1))
fi
# Known-answer case for the census itself. The census reads `comm=`, which is the
# EXECUTABLE's basename and not argv[0], so `exec -a` would not plant anything --
# copy a real binary under a matching name instead.
cp /bin/sleep "$TMP/scovox_probe"
"$TMP/scovox_probe" 30 &
probe=$!
sleep 0.3
cases=$((cases+1))
if [ "$(census)" -gt "$census_after" ]; then
  echo "  PASS  the census can read non-zero (planted process seen)"
else
  echo "  FAIL  the census did not see a planted process -- it has stopped counting"
  fails=$((fails+1))
fi
kill "$probe" 2>/dev/null
wait "$probe" 2>/dev/null
# And --dry-run must be doing that by REACHING the end, not by dying early:
# a --dry-run that exited 2 on everything would also leave a census of 0.
cases=$((cases+1))
if timeout 20 "$CS" --root "$TMP" --duration 3000 \
     --scenario flatforest_dense_2robot_lidar.yaml --dry-run \
     --arms hybrid,off --seeds 1,2 2>&1 \
   | grep -q "cells=hybrid:1,off:1,hybrid:2,off:2"; then
  echo "  PASS  --dry-run reaches the cell list (seed-major, both arms)"
else
  echo "  FAIL  --dry-run did not reach the cell list"
  fails=$((fails+1))
fi

echo
echo "=== the launcher's own validation blocks (the planner block, the M-TARE knobs) ==="
# The launcher has no --dry-run, so its inert prelude, through unset
# _rzv_needed, is cut from the shipped file and run alone. Write the cut into
# sim/, not a tmpdir: the launcher derives paths from BASH_SOURCE[0].
# (notes: guardcal-prelude-cut)
PRELUDE_ANCHOR='^unset _rzv_needed$'
PRELUDE_END=$(grep -n "$PRELUDE_ANCHOR" "$LAUNCHER" | head -1 | cut -d: -f1)
PROBE="$(dirname "$LAUNCHER")/.prelude_probe.$$.sh"
trap 'rm -rf "$TMP"; rm -f "$PROBE"' EXIT

cases=$((cases+1))
if [ -n "$PRELUDE_END" ]; then
  echo "  PASS  the prelude anchor is present (launcher line $PRELUDE_END)"
else
  echo "  FAIL  '$PRELUDE_ANCHOR' is not in the launcher -- the schedule-needed"
  echo "        was renamed or removed, and every case below would be vacuous"
  fails=$((fails+1))
fi

if [ -n "$PRELUDE_END" ]; then
  sed -n "1,${PRELUDE_END}p" "$LAUNCHER" > "$PROBE"
  # The sentinel is only reachable at the END of the cut, and it carries the
  # knobs as the launcher resolved them. That is what makes an ALLOW case say
  # something: absence of our FATAL text is also true of a prelude that died at
  # the scenario check, and would certify a launcher that refuses everything.
  printf '%s\n' 'echo "__PRELUDE_OK__ cw=$CELL_WORLD tw=$TEAM_WORLD hz=$TEAM_WORLD_HZ ga=$GLOBAL_ALLOC rg=$RECONNECT_GATE rs=$RENDEZVOUS_SCHEDULE pp=$PURSUIT_PREDICTOR rm=$RECONNECT_MODE"' >> "$PROBE"
  # A SECOND line rather than more fields on the first, so the roster can be
  # asserted without rewriting the expect string of every case above -- lg
  # matches a fixed substring of the whole output, not the whole output.
  printf '%s\n' 'echo "__ROSTER_OK__ n=$N_ROBOTS robots=[$ROBOTS]"' >> "$PROBE"
  # And the planner block's resolution: the executable, the arm and the pins.
  printf '%s\n' 'echo "__PLANNER_OK__ node=$NODE exe=$PLANNER_EXE arm=$ARM topic=$TEAM_XCHG_TOPIC bep=$BEST_EFFORT_PRIORITY ex=$EXPLOIT ds=$DONE_SEEK mr=$MISSION_RETURN"' >> "$PROBE"

  # And the cut has to CONTAIN the guards. A range that stopped short would
  # fail every BLOCK case for the wrong reason and pass every ALLOW one.
  for _need in "FATAL: CELL_WORLD" "FATAL: TEAM_WORLD=" "FATAL: TEAM_WORLD_HZ" \
               "FATAL: RENDEZVOUS_SCHEDULE" "FATAL: PURSUIT_PREDICTOR" \
               "the arm name and the arm" "has no runnable configuration" \
               "FATAL: the planner refuses:" \
               "is not off|pursuit|rendezvous|hybrid"; do
    cases=$((cases+1))
    if grep -qF "$_need" "$PROBE"; then
      echo "  PASS  the cut carries the guard: $_need"
    else
      echo "  FAIL  the cut does NOT carry: $_need -- the line range is wrong"
      fails=$((fails+1))
    fi
  done

  # env -i: an ambient export must not reach a cell, so it must not reach the
  # probe either. SCEN picks the scenario, so the roster can vary; callers that
  # care set it. (notes: guardcal-lg-env-i-and-scen)
  SCEN=flatforest_dense_2robot_lidar.yaml
  lg() {
    local want="$1" expect="$2" label="$3"; shift 3
    local out rc
    cases=$((cases+1))
    out=$(env -i PATH="$PATH" HOME="$HOME" USER="${USER:-nobody}" \
              SCENARIO="$SCEN" "$@" \
              timeout 30 bash -c \
              "source /opt/ros/humble/setup.bash >/dev/null 2>&1; bash '$PROBE'" 2>&1)
    rc=$?
    # A __ sentinel is matched as a whole line (-qxF), a FATAL as a substring:
    # arm names are prefixes of each other, e.g. rm=mtare_hybrid and
    # rm=mtare_hybrid_mdp. (notes: guardcal-sentinel-whole-line)
    local ok=0 _mx=-qF
    case "$expect" in __*) _mx=-qxF ;; esac
    if [ "$want" = BLOCK ]; then
      [ "$rc" != 0 ] && printf '%s\n' "$out" | grep "$_mx" "$expect" && ok=1
    else
      [ "$rc" = 0 ] && printf '%s\n' "$out" | grep "$_mx" "$expect" && ok=1
    fi
    if [ "$ok" = 1 ]; then
      echo "  PASS  $label"
    else
      echo "  FAIL  $label"
      echo "          rc=$rc, wanted $want with '$expect'"
      printf '%s\n' "$out" | tail -3 | sed 's/^/          | /'
      fails=$((fails+1))
    fi
  }

  # The control that gives every refusal below its meaning: a bare invocation
  # resolves. The runner pins mtare_off's stack for every arm, so the arm is
  # the only difference between cells.
  # (notes: guardcal-default-is-mtare-hybrid)
  STACK='__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=silence rs=0 pp=trail rm=mtare_off'
  lg ALLOW "$STACK" "shipped defaults resolve to the mtare_off stack"
  # The resume guard predicts the same stack from its own copy; read it back.
  cases=$((cases+1))
  _req=$( eval "$(grep -E '^  (CELL_WORLD|GLOBAL_ALLOC|PURSUIT_PREDICTOR)_REQ=' "$CS")"
          echo "cw=${CELL_WORLD_REQ-} tw=${TEAM_WORLD_REQ-} ga=${GLOBAL_ALLOC_REQ-} rg=${RECONNECT_GATE_REQ-} rs=${RENDEZVOUS_SCHEDULE_REQ-} pp=${PURSUIT_PREDICTOR_REQ-}" )
  _res=$(printf '%s\n' "$STACK" | sed 's/^__PRELUDE_OK__ //; s/ hz=[^ ]*//; s/ rm=.*//')
  if [ "$_req" = "$_res" ]; then
    echo "  PASS  the resume guard's stack matches the runner's ($_req)"
  else
    echo "  FAIL  the resume guard expects '$_req' but the runner resolves '$_res'"
    fails=$((fails+1))
  fi
  unset _req _res
  lg ALLOW '__PLANNER_OK__ node=gen34 exe=explo_planner_node arm=hybrid topic=exploration/team_beacon bep=true ex=0 ds=0 mr=1' \
     "shipped defaults run explo_planner_node at arm hybrid with the gen-34 pins"
  for _arm in off pursuit rendezvous; do
    lg ALLOW "$STACK" "ARM=$_arm runs the same stack" ARM=$_arm
    lg ALLOW "__PLANNER_OK__ node=gen34 exe=explo_planner_node arm=$_arm topic=exploration/team_beacon bep=true ex=0 ds=0 mr=1" \
       "ARM=$_arm reaches the planner" ARM=$_arm
  done
  unset _arm
  # NODE is a label, not a knob: npm exports NODE as the path of its binary,
  # and neither that nor a stale NODE=gen33 may change what runs.
  lg ALLOW '__PLANNER_OK__ node=gen34 exe=explo_planner_node arm=hybrid topic=exploration/team_beacon bep=true ex=0 ds=0 mr=1' \
     "an exported NODE=/usr/bin/node is not read" NODE=/usr/bin/node
  lg ALLOW '__PLANNER_OK__ node=gen34 exe=explo_planner_node arm=hybrid topic=exploration/team_beacon bep=true ex=0 ds=0 mr=1' \
     "a stale NODE=gen33 is not read" NODE=gen33

  # The arm is one of four names, exactly.
  lg BLOCK "FATAL: ARM='mtare_hybrid' is not off|pursuit|rendezvous|hybrid." \
     "a gen-33 token as ARM is refused" ARM=mtare_hybrid
  lg BLOCK "FATAL: ARM='Hybrid' is not off|pursuit|rendezvous|hybrid." \
     "a mis-cased ARM is refused" ARM=Hybrid

  # The gen-33 knobs are refused, not overridden: a cell recorded with a knob
  # the binary never read is a mislabelled cell. The values the runner pins
  # are accepted when restated.
  lg BLOCK "FATAL: the planner refuses: RECONNECT_MODE=mtare_hybrid." \
     "RECONNECT_MODE is refused" RECONNECT_MODE=mtare_hybrid
  lg BLOCK "FATAL: the planner refuses: RECONNECT_MODE=mtare_off." \
     "RECONNECT_MODE is refused even at the pinned stack" RECONNECT_MODE=mtare_off
  lg BLOCK "FATAL: the planner refuses: EXPLOIT=1." \
     "EXPLOIT=1 is refused (exploit is phase 2)" EXPLOIT=1
  lg BLOCK "FATAL: the planner refuses: DONE_SEEK=1." \
     "DONE_SEEK=1 is refused" DONE_SEEK=1
  lg BLOCK "FATAL: the planner refuses: MISSION_RETURN=0." \
     "MISSION_RETURN=0 is refused (the planner always homes)" MISSION_RETURN=0
  lg BLOCK "FATAL: the planner refuses: LINK_GATE=1." \
     "LINK_GATE=1 is refused (no link veto)" LINK_GATE=1
  lg BLOCK "FATAL: the planner refuses: EXPLOIT=1 LINK_GATE=1." \
     "every refused knob is named, not just the first" EXPLOIT=1 LINK_GATE=1
  lg ALLOW "$STACK" "the pinned values restated are accepted" \
     EXPLOIT=0 DONE_SEEK=0 MISSION_RETURN=1 LINK_GATE=0

  # The pinned stack refuses a contradiction of any of its knobs, including a
  # typo that would read as off. (notes: guardcal-arm-stamp-per-knob)
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_off implies CELL_WORLD=1, but CELL_WORLD='true'" \
     "CELL_WORLD=true is refused, not silently read as off" CELL_WORLD=true
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_off implies TEAM_WORLD=1, but TEAM_WORLD='0'" \
     "TEAM_WORLD=0 contradicts the stack" TEAM_WORLD=0
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_off implies GLOBAL_ALLOC=1, but GLOBAL_ALLOC='0'" \
     "GLOBAL_ALLOC=0 contradicts the stack" GLOBAL_ALLOC=0
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_off implies RECONNECT_GATE=silence, but RECONNECT_GATE='info'" \
     "RECONNECT_GATE=info contradicts the stack" RECONNECT_GATE=info
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_off implies RENDEZVOUS_SCHEDULE=0, but RENDEZVOUS_SCHEDULE='1'" \
     "RENDEZVOUS_SCHEDULE=1 contradicts the stack" RENDEZVOUS_SCHEDULE=1
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_off implies PURSUIT_PREDICTOR=trail, but PURSUIT_PREDICTOR='mdp'" \
     "PURSUIT_PREDICTOR=mdp contradicts the stack" PURSUIT_PREDICTOR=mdp
  lg BLOCK "FATAL: TEAM_WORLD_HZ='0.0'" \
     "TEAM_WORLD_HZ=0 is refused" TEAM_WORLD_HZ=0
  lg BLOCK "FATAL: TEAM_WORLD_HZ='<empty: rejected by flt>'" \
     "flt's empty return for nan is refused, not passed to ros2 as a bare -p" \
     TEAM_WORLD_HZ=nan

  # --- the roster axis (P7) -------------------------------------------------
  # The roster is read from the scenario and cannot be overridden, so SCEN
  # selects it. The N == 2 case is the control: an assertion matching a constant
  # string would pass at both sizes. (notes: guardcal-roster-axis)
  lg ALLOW '__ROSTER_OK__ n=2 robots=[atlas bestla]' \
     "the roster resolves to the 2-robot scenario's own names"
  SCEN=flatforest_3robot_lidar.yaml
  lg ALLOW '__ROSTER_OK__ n=3 robots=[atlas bestla husky]' \
     "the roster resolves to THREE at the 3-robot scenario"
  # And the planner block is not quietly conditioned on the pair.
  lg ALLOW '__PLANNER_OK__ node=gen34 exe=explo_planner_node arm=rendezvous topic=exploration/team_beacon bep=true ex=0 ds=0 mr=1' \
     "the planner block resolves the same at N == 3" ARM=rendezvous
  lg BLOCK "FATAL: the planner refuses: RECONNECT_MODE=mtare_hybrid." \
     "the planner block still refuses at N == 3" RECONNECT_MODE=mtare_hybrid
  SCEN=flatforest_dense_2robot_lidar.yaml
fi

echo
echo "=== the launcher's per-robot derivations (the region BELOW the prelude) ==="
# The per-robot derivations (peers_of, peers_csv, peers_ros_array, the viz
# palette) are inert shell over ROBOTS, so they are cut and run after the
# prelude. The one skipped variable they read, OUTDIR, the shim supplies.
# (notes: guardcal-derivation-cut)
DERIV_START_ANCHOR='^VIZ_PALETTE=(COSTAR_'
DERIV_END_ANCHOR='^# --- environment: humble'
DERIV_START=$(grep -n "$DERIV_START_ANCHOR" "$LAUNCHER" | head -1 | cut -d: -f1)
DERIV_END=$(grep -n "$DERIV_END_ANCHOR" "$LAUNCHER" | head -1 | cut -d: -f1)
DPROBE="$(dirname "$LAUNCHER")/.deriv_probe.$$.sh"
trap 'rm -rf "$TMP"; rm -f "$PROBE" "$DPROBE"' EXIT

cases=$((cases+1))
if [ -n "$PRELUDE_END" ] && [ -n "$DERIV_START" ] && [ -n "$DERIV_END" ] \
   && [ "$DERIV_START" -lt "$DERIV_END" ]; then
  echo "  PASS  the derivation anchors are present (launcher $DERIV_START-$DERIV_END)"
else
  echo "  FAIL  the derivation anchors are missing or inverted -- every case"
  echo "        below would be vacuous"
  fails=$((fails+1))
fi

if [ -n "$PRELUDE_END" ] && [ -n "$DERIV_START" ] && [ -n "$DERIV_END" ] \
   && [ "$DERIV_START" -lt "$DERIV_END" ]; then
  sed -n "1,${PRELUDE_END}p" "$LAUNCHER" > "$DPROBE"
  printf '%s\n' "OUTDIR=\"$TMP/deriv\"" >> "$DPROBE"
  sed -n "${DERIV_START},$((DERIV_END - 1))p" "$LAUNCHER" >> "$DPROBE"

  # The cut has to CONTAIN the helpers, or every case below matches nothing for
  # a reason that has nothing to do with the roster.
  for _need in "peers_of()" "peers_csv()" "peers_ros_array()" "VIZ_PALETTE=("; do
    cases=$((cases+1))
    if grep -qF "$_need" "$DPROBE"; then
      echo "  PASS  the cut carries: $_need"
    else
      echo "  FAIL  the cut does NOT carry: $_need -- the line range is wrong"
      fails=$((fails+1))
    fi
  done

  cat >> "$DPROBE" <<'PY'
for _r in $ROBOTS; do echo "__PEERS__ $_r -> $(peers_csv "$_r")"; done
echo "__ARRAY__ $(peers_ros_array "${ROBOTS%% *}" /rx/ /exploration/intents)"
for _r in $ROBOTS; do echo "__VIZ__ $_r=${VIZ_MODEL[$_r]}"; done
PY

  dv() {
    local scen="$1" expect="$2" label="$3"
    local out rc
    cases=$((cases+1))
    out=$(env -i PATH="$PATH" HOME="$HOME" USER="${USER:-nobody}" \
              SCENARIO="$scen" timeout 30 bash -c \
              "source /opt/ros/humble/setup.bash >/dev/null 2>&1; bash '$DPROBE'" 2>&1)
    rc=$?
    # -qxF, whole line: see the note in lg() above. Every expectation here is a
    # complete sentinel line, and several of the values are prefixes of each
    # other.
    if [ "$rc" = 0 ] && printf '%s\n' "$out" | grep -qxF "$expect"; then
      echo "  PASS  $label"
    else
      echo "  FAIL  $label"
      echo "          rc=$rc, wanted '$expect'"
      printf '%s\n' "$out" | tail -4 | sed 's/^/          | /'
      fails=$((fails+1))
    fi
  }

  _S2=flatforest_dense_2robot_lidar.yaml
  _S3=flatforest_3robot_lidar.yaml

  # At N == 2 peers_csv must emit the single name its pairwise predecessor did.
  # This is the equivalence half: every existing 2-robot call site is unchanged.
  dv "$_S2" '__PEERS__ atlas -> bestla' "peers_csv at N == 2 is still the one peer"
  dv "$_S2" '__PEERS__ bestla -> atlas' "peers_csv is not self-referential"
  # At N == 3 it must emit BOTH far ends. A helper that returned one arbitrary
  # member would wire each robot to a single peer and the third radio would be
  # modelled by the emulator and subscribed by nobody.
  dv "$_S3" '__PEERS__ atlas -> bestla,husky' "peers_csv at N == 3 lists both peers"
  dv "$_S3" '__PEERS__ husky -> atlas,bestla' "peers_csv excludes self, not position 0"
  # The ROS array literal, which is what actually reaches -p on the launch line.
  dv "$_S3" '__ARRAY__ ["/rx/bestla/exploration/intents","/rx/husky/exploration/intents"]' \
     "peers_ros_array quotes and comma-joins every peer topic"
  dv "$_S2" '__ARRAY__ ["/rx/bestla/exploration/intents"]' \
     "peers_ros_array at N == 2 is a one-element array, not a bare string"
  # Palette by roster POSITION. A name-keyed table would hand husky an empty
  # model string and a URDF that fails to parse; three distinct models is the
  # assertion, and the third robot getting the unpainted base model is correct.
  dv "$_S3" '__VIZ__ husky=COSTAR_HUSKY_SENSOR_CONFIG_REDUCED' \
     "the third roster slot gets a real model, not an empty string"
  dv "$_S3" '__VIZ__ atlas=COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_YELLOW' \
     "the first roster slot keeps the model it had at N == 2"
fi

echo
echo "=== the launch-time planner count is roster-derived, not a literal ==="
# The planner-count guard needs a real bring-up, so it is checked textually on
# three inputs: the shipped launcher, a copy mutated to a literal 2, and a copy
# with the guard deleted, which must not PASS.
# (notes: guardcal-planner-count-textual)
nplan_guard() {
  local f=$1 ln
  ln=$(grep -n 'NPLAN=$(count_own' "$f" | head -1 | cut -d: -f1)
  [ -n "$ln" ] || return 2                      # the guard is gone entirely
  sed -n "$((ln + 1))p" "$f" | grep -qF '[ "$NPLAN" = "$N_ROBOTS" ]'
}
_MUT="$TMP/launcher_literal.sh"
sed 's|\[ "\$NPLAN" = "\$N_ROBOTS" \]|[ "$NPLAN" = 2 ]|' "$LAUNCHER" > "$_MUT"
_GONE="$TMP/launcher_noguard.sh"
grep -v 'NPLAN=$(count_own' "$LAUNCHER" > "$_GONE"
for _spec in "$LAUNCHER:0:the shipped launcher counts \$N_ROBOTS planners" \
             "$_MUT:1:a launcher mutated back to the literal 2 is caught" \
             "$_GONE:2:a launcher with no planner-count guard is not a PASS"; do
  _f=${_spec%%:*}; _rest=${_spec#*:}; _want=${_rest%%:*}; _lbl=${_rest#*:}
  cases=$((cases+1))
  nplan_guard "$_f"; _rc=$?
  if [ "$_rc" = "$_want" ]; then
    echo "  PASS  $_lbl"
  else
    echo "  FAIL  $_lbl: want rc=$_want got rc=$_rc"
    fails=$((fails+1))
  fi
done
unset _spec _f _rest _want _lbl _rc _MUT _GONE

echo
# A clean --dry-run must write nothing to stderr, so a real error is not skimmed
# past. Asserts the absence of output, not a specific message.
# (notes: guardcal-silent-stderr)
cases=$((cases+1))
_stderr=$(timeout 20 "$CS" --root "$TMP" --duration 3000 \
            --scenario flatforest_dense_2robot_lidar.yaml --dry-run \
            --arms hybrid,off --seeds 1 2>&1 >/dev/null)
if [ -z "$_stderr" ]; then
  echo "  PASS  a clean --dry-run writes nothing to stderr"
else
  echo "  FAIL  a clean --dry-run wrote to stderr:"
  echo "$_stderr" | sed 's/^/          /'
  fails=$((fails+1))
fi
unset _stderr

# Each knob run_campaign.sh reads through env_val must be stripped (-u) or
# assigned on the per-cell launch line, or the cell runs the ambient value. The
# knob list is derived; an empty extraction is a FAIL.
# (notes: guardcal-launch-line-strips-knobs)
_launch_line=$(sed -n '/^  env /,/run_explo_sim_rviz.sh"/p' "$CS")
_knobs=$(grep -oE 'env_val [A-Z_][A-Z0-9_]*' "$CS" | awk '{print $2}' | sort -u)
if [ -z "$_knobs" ]; then
  echo "  FAIL  found no env_val knobs to check -- this loop has stopped checking"
  fails=$((fails+1)); cases=$((cases+1))
fi
cases=$((cases+1))
if [ -z "$_launch_line" ]; then
  echo "  FAIL  the per-cell launch line could not be extracted from run_campaign.sh;"
  echo "        every knob below would read as unprotected whatever the launcher does"
  fails=$((fails+1))
else
  echo "  PASS  the per-cell launch line was located ($(printf '%s\n' "$_launch_line" | wc -l) lines)"
fi
for _u in $_knobs; do
  cases=$((cases+1))
  if printf '%s\n' "$_launch_line" | grep -qE -- "-u $_u( |\\\\|$)"; then
    echo "  PASS  the launch line strips ambient $_u"
  elif printf '%s\n' "$_launch_line" | grep -qE "(^|[[:space:]])$_u="; then
    echo "  PASS  the launch line assigns $_u explicitly (strip not needed)"
  else
    echo "  FAIL  ambient $_u reaches every cell -- neither stripped nor assigned"
    fails=$((fails+1))
  fi
done
unset _u _knobs _launch_line

echo
echo "=== the ambient-knob sweep derived from the runner ==="
# The runner reads about 110 defaulted knobs, not just the ~20 run_campaign.sh
# models; unstripped, an ambient export of any would retune every cell. The
# sweep is derived from the runner; these cases check the derivation.
# (notes: guardcal-runner-knob-sweep)
_keep=$(sed -n 's/^RUNNER_ENV_KEEP="\(.*\)"$/\1/p' "$CS" | head -1)
# The scanner is extracted from run_campaign.sh, never pasted here: a pasted
# copy would keep passing while the real scanner is broken.
# (notes: guardcal-scanner-extracted)
_awkprog=$(sed -n '/^RUNNER_STRIP=\$(awk -v keep=/,/^'"'"' "\$HERE\/run_explo_sim_rviz.sh")$/p' "$CS" \
           | sed '1d;$d')
_scan() {  # $1 = file to scan; prints one name per line
  # The launcher's program emits a single ` -u NAME -u NAME` line for the env
  # command; split it back apart rather than keeping a print-one-per-line
  # variant here, so the FORMAT the launch line consumes is what gets checked.
  awk -v keep=" $_keep " "$_awkprog" "$1" \
    | tr ' ' '\n' | grep -v '^-u$' | grep -v '^$'
}
_runner="$(dirname "$CS")/run_explo_sim_rviz.sh"

cases=$((cases+1))
if printf '%s\n' "$_awkprog" | grep -q 'match(line'; then
  echo "  PASS  the launcher's own scanner was extracted ($(printf '%s\n' "$_awkprog" | wc -l) lines)"
else
  echo "  FAIL  could not extract the scanner from run_campaign.sh -- every case"
  echo "        below would be running an empty awk program and calling it a PASS"
  fails=$((fails+1))
fi
_derived=$(_scan "$_runner" | sort)
_nd=$(printf '%s\n' "$_derived" | grep -c '[A-Z]' || true)

cases=$((cases+1))
if [ -z "$_keep" ]; then
  echo "  FAIL  RUNNER_ENV_KEEP could not be read out of run_campaign.sh -- these"
  echo "        cases are scanning with an empty keep list and prove nothing"
  fails=$((fails+1))
elif [ "$_nd" -ge 60 ]; then
  echo "  PASS  the sweep derives $_nd ambient knobs from the runner"
else
  echo "  FAIL  the sweep derives only $_nd knob(s); the runner reads ~110, so the"
  echo "        scan is broken and the launcher's own tripwire (<10) would not"
  echo "        have caught a partial break either"
  fails=$((fails+1))
fi

# The six that finding #24 was about, named individually. A count alone passes
# while the one knob that matters is the one that got away.
for _k in RDV_MAX_WAIT RDV_DEPART_DELAY RDV_SETTLE RDV_APPT_WAIT \
          RDV_LATCHED_HOLD START_HOLD; do
  cases=$((cases+1))
  if printf '%s\n' "$_derived" | grep -qx -- "$_k"; then
    echo "  PASS  ambient $_k is stripped from every cell"
  else
    echo "  FAIL  ambient $_k still reaches every cell -- it is a rendezvous knob"
    echo "        and two of ts4's four arms ARE the rendezvous"
    fails=$((fails+1))
  fi
done

# The KEEP list, both directions. A name in KEEP must be exempt (or the exemption
# is decorative) AND must actually be read by the runner (or it is a typo, and a
# typo in KEEP does not fail loudly -- it strips the variable it was written to
# protect).
for _k in $_keep; do
  cases=$((cases+1))
  if printf '%s\n' "$_derived" | grep -qx -- "$_k"; then
    echo "  FAIL  $_k is in RUNNER_ENV_KEEP and is stripped anyway"
    fails=$((fails+1))
  elif ! grep -q "\${$_k[:]\?[-=]" "$_runner"; then
    echo "  FAIL  $_k is exempted but the runner never reads it -- a dead or"
    echo "        misspelled KEEP entry, and the misspelling is the dangerous one"
    fails=$((fails+1))
  else
    echo "  PASS  infrastructure knob $_k is read by the runner and exempt"
  fi
done

# The scanner itself, against a fixture whose answer is known by construction --
# because every case above is derived from the same file the launcher derives
# from, so a scanner that matched the WRONG thing consistently would agree with
# itself everywhere.
cat > "$TMP/fixture.sh" <<'FIXEOF'
NEW_KNOB="${NEW_KNOB:-7}"
ASSIGN_FORM="${ASSIGN_FORM:=7}"
BARE_FORM="${BARE_FORM-7}"
# COMMENTED_KNOB="${COMMENTED_KNOB:-7}"
PLAIN_READ="$NOT_A_DEFAULTED_READ"
lower_case="${lower_case:-7}"
FIXEOF
_fx=$(_scan "$TMP/fixture.sh" | sort | tr '\n' ' ')
cases=$((cases+1))
if [ "$_fx" = "ASSIGN_FORM BARE_FORM NEW_KNOB " ]; then
  echo "  PASS  the scanner finds all three default forms, and skips comments,"
  echo "        undefaulted reads and lower-case names"
else
  echo "  FAIL  the scanner returned [$_fx]; expected exactly the three defaulted"
  echo "        upper-case names. A comment or a plain \$VAR read leaking in means"
  echo "        the strip list carries names nothing configures; a missing form"
  echo "        means knobs of that shape still reach every cell"
  fails=$((fails+1))
fi

cases=$((cases+1))
: > "$TMP/empty.sh"
if [ -z "$(_scan "$TMP/empty.sh")" ]; then
  echo "  PASS  a source with no knobs derives nothing, so the launcher's"
  echo "        tripwire is reachable rather than decorative"
else
  echo "  FAIL  the scanner invents names from an empty file"
  fails=$((fails+1))
fi

# Reads the launcher's derived-knob tripwire threshold back and requires it
# between 0 and the derived count: at 0 it never fires, at or above the count it
# refuses every real campaign. (notes: guardcal-tripwire-threshold)
cases=$((cases+1))
_thr=$(sed -n 's/^if \[ "${_nstrip:-0}" -lt \([0-9]*\) \]; then$/\1/p' "$CS" | head -1)
if [ -z "$_thr" ]; then
  echo "  FAIL  the launcher's derived-knob tripwire could not be found at all"
  fails=$((fails+1))
elif [ "$_thr" -gt 0 ] && [ "$_thr" -lt "$_nd" ]; then
  echo "  PASS  the tripwire refuses below $_thr, between zero and the $_nd real knobs"
else
  echo "  FAIL  the tripwire threshold is $_thr against $_nd real knobs: at 0 it can"
  echo "        never fire, at or above $_nd it refuses every legitimate campaign"
  fails=$((fails+1))
fi

# The derivation must emit -u flags, the form env and the launcher's tripwire
# count (bare names would make env run the first as the command), and the launch
# line must expand RUNNER_STRIP. (notes: guardcal-sweep-output-and-wiring)
cases=$((cases+1))
_nflag=$(awk -v keep=" $_keep " "$_awkprog" "$_runner" | tr ' ' '\n' | grep -c '^-u$' || true)
if [ "${_nflag:-0}" -ge 10 ]; then
  echo "  PASS  the derivation emits $_nflag -u flags, the form env and the"
  echo "        launcher's own tripwire both count"
else
  echo "  FAIL  the derivation emits ${_nflag:-0} -u flag(s); the launch line would"
  echo "        read the first derived name as the command to run"
  fails=$((fails+1))
fi

cases=$((cases+1))
if sed -n '/^  env /,/run_explo_sim_rviz.sh"/p' "$CS" | grep -q '\$RUNNER_STRIP'; then
  echo "  PASS  the launch line expands \$RUNNER_STRIP"
else
  echo "  FAIL  \$RUNNER_STRIP is derived and never used on the launch line"
  fails=$((fails+1))
fi

# And the half that makes over-stripping safe: --env must still beat the strip.
# If it did not, this whole sweep would be a hundred silently-ignored knobs
# rather than a hundred neutralised ones.
cases=$((cases+1))
_got=$(RDV_DEPART_DELAY=ambient env -u RDV_DEPART_DELAY RDV_DEPART_DELAY=explicit \
       sh -c 'echo "${RDV_DEPART_DELAY:-DEFAULT}"')
_amb=$(RDV_DEPART_DELAY=ambient env -u RDV_DEPART_DELAY \
       sh -c 'echo "${RDV_DEPART_DELAY:-DEFAULT}"')
if [ "$_got" = "explicit" ] && [ "$_amb" = "DEFAULT" ]; then
  echo "  PASS  env applies -u before assignments: --env still wins, ambient does not"
else
  echo "  FAIL  env precedence is not what the sweep assumes (assigned=[$_got]"
  echo "        stripped=[$_amb]) -- --env may no longer reach the cell"
  fails=$((fails+1))
fi
unset _keep _runner _derived _nd _k _fx _got _amb _awkprog _nflag _thr
unset -f _scan

echo
echo "=== the resume guard: a banked cell must have run THIS experiment ==="
# Asserts the resume guard both ways, already complete versus a different
# experiment; floats compare numerically. No --dry-run (the guard sits below
# it): a huge MIN_FREE_MB stops a let-through, which reads LAUNCHED.
# (notes: guardcal-resume-guard-cases)
RG_ARM=hybrid
# Agrees with the reference campaign below on every key the guard reads. Each
# case overwrites, or deletes, exactly one line.
#
# run_gates_verdict is CLEAN, a token the harness can write: the teardown writes
# exactly one of CLEAN, SUSPECT or INVALID.
# (notes: guardcal-fixture-verdict-clean)
rg_manifest() {  # $1 = the arm the fixture cell ran
  cat <<'EOF'
run_end_reason=all_done
run_gates_verdict=CLEAN
node=gen34
mission_return_enabled=true
scenario=flatforest_dense_2robot_lidar.yaml
duration_s=3000
done_criterion=latch
cell_world=1
team_world=1
global_alloc=1
reconnect_gate=silence
rendezvous_schedule=0
pursuit_predictor=trail
done_unknown_fraction=0.64
tx_power_dbm=30.0
tree_attenuation_db=70.0
max_range_m=30.0
cell_size_m=10.0
coord_claim_radius_override=none
alloc_peer_pos_max_age_sec=none
separation_weight=0
separation_radius_m=20
separation_max_age_sec=10
share_full_period_s=0.0
link_gate=0
link_gate_effective=0
done_seek_enabled=false
reconnect_midrun_silence_sec=90
team_world_hz=1.0
EOF
  echo "arm=$1"
}
# Every key the guard reads must be here at the reference campaign's value
# (launcher defaults, no --env), or every SKIP case aborts on the missing key.
# n_robots is deliberately absent. (notes: guardcal-fixture-keys)
# rg WANT "label" ARM KEY VALUE
#   KEY=""          -> the manifest is left agreeing
#   VALUE="<none>"  -> the key is DELETED, i.e. a manifest predating it
rg() {
  _want="$1"; _lbl="$2"; _arm="$3"; _key="${4-}"; _val="${5-}"
  cases=$((cases+1))
  _root="$TMP/resume_$cases"; _cell="$_root/rg_${_arm}_seed1"
  mkdir -p "$_cell"
  if [ -n "$_key" ]; then
    rg_manifest "$_arm" | grep -v "^$_key=" > "$_cell/run_manifest.txt"
    [ "$_val" = "<none>" ] || echo "$_key=$_val" >> "$_cell/run_manifest.txt"
  else
    rg_manifest "$_arm" > "$_cell/run_manifest.txt"
  fi
  # RG_EXTRA, when set, is one more manifest line (a key the fixture lacks).
  [ -n "${RG_EXTRA:-}" ] && echo "$RG_EXTRA" >> "$_cell/run_manifest.txt"
  # RG_ENV, when set, is the campaign's one --env token (the full-map cases).
  _out=$(MIN_FREE_MB=999999999999 timeout 60 "$CS" --root "$_root" --tag rg \
           --duration 3000 --scenario flatforest_dense_2robot_lidar.yaml \
           ${RG_ENV:+--env "$RG_ENV"} \
           --arms "$_arm" --seeds 1 2>&1)
  _rc=$?
  if echo "$_out" | grep -q "ABORT: rg_${_arm}_seed1 is complete but its manifest says"; then
    # The text alone is not the verdict: a guard
    # that printed the refusal and then ran the cell anyway would be worse than
    # no guard at all.
    [ "$_rc" = 0 ] && _got=PRINTED-BUT-RAN || _got=ABORT
  elif echo "$_out" | grep -q "SKIP rg_${_arm}_seed1 (already complete)"; then
    _got=SKIP
  elif echo "$_out" | grep -q "SKIP rg_${_arm}_seed1 (complete, but run_gates_verdict="; then
    # SKIP-DIRTY: the cell is skipped but carries a verdict gate_g8 will
    # hard-fail; kept apart from SKIP so the loud and silent paths stay
    # distinguishable. (notes: guardcal-skip-dirty)
    _got=SKIP-DIRTY
  elif echo "$_out" | grep -q "MB free under"; then
    _got=LAUNCHED          # the guard passed the cell through to be re-run
  else
    _got="REFUSED(rc=$_rc)" # something else stopped it before the guard
  fi
  if [ "$_got" = "$_want" ]; then
    echo "  PASS  $_lbl ($_got)"
  else
    echo "  FAIL  $_lbl: want $_want got $_got"
    echo "$_out" | grep -E "FATAL|ABORT|SKIP|free under" | sed 's/^/          | /'
    fails=$((fails+1))
  fi
}

rg SKIP  "an agreeing manifest is skipped, not re-run"        "$RG_ARM"
rg SKIP  "an agreeing off cell is skipped too"                off
# node and arm: a gen-33 cell, or a gen-34 cell of another arm, banked under
# this name is a different experiment. (notes: resume-string-key-guard)
rg ABORT "a gen-33 cell (no node= line) is not this experiment" "$RG_ARM" node "<none>"
rg ABORT "a cell that says node=gen33 is not this experiment"   "$RG_ARM" node gen33
rg ABORT "a hybrid-named cell that ran pursuit"               "$RG_ARM" arm pursuit
rg ABORT "a cell with no arm= line"                           "$RG_ARM" arm "<none>"
rg ABORT "a gen-33 mtare_ arm token on the arm= line"         "$RG_ARM" arm mtare_hybrid
# The pinned gen-33 lines: the runner writes them at fixed values, so any
# other value is a cell from another runner.
rg ABORT "a cell banked with the link gate on"                "$RG_ARM" link_gate 1
rg ABORT "a cell banked with the schedule on (gen-33 stack)"  "$RG_ARM" rendezvous_schedule 1
rg ABORT "a cell banked with the post-latch coast"            "$RG_ARM" done_seek_enabled true
# Anti-vacuity for the numeric compare, in the direction that costs wall time
# rather than data: these are the SAME configuration, spelled the way a hand-
# written or older manifest spells it.
rg SKIP  "cell_size_m=10 agrees with 10.0"                    "$RG_ARM" cell_size_m 10
rg SKIP  "tree_attenuation_db=70 agrees with 70.0"            "$RG_ARM" tree_attenuation_db 70
rg SKIP  "done_unknown_fraction=.64 agrees with 0.64"         "$RG_ARM" done_unknown_fraction .64
# The radio regime: a cell banked under a different tree attenuation, max range
# or tx power must abort. (notes: guardcal-radio-regime-keys)
rg ABORT "tree_attenuation_db=11.98 is the old radio"         "$RG_ARM" tree_attenuation_db 11.98
rg ABORT "max_range_m absent predates the horizon"            "$RG_ARM" max_range_m "<none>"
rg ABORT "max_range_m=60.0 is a different horizon"            "$RG_ARM" max_range_m 60.0
rg ABORT "tx_power_dbm=40.0 is a different link budget"       "$RG_ARM" tx_power_dbm 40.0
# The census unit, and the one the original todo named: cell_size_m decides what
# a covered cell IS, so two runs that disagree about it have incomparable
# coverage curves and a pooled completion time that means nothing.
rg ABORT "cell_size_m=20.0 is a different census"             "$RG_ARM" cell_size_m 20.0
rg ABORT "cell_size_m absent cannot be shown to agree"        "$RG_ARM" cell_size_m "<none>"
# Regression cover for the key that WAS compared before this loop existed --
# folding it in must not have dropped it.
rg ABORT "done_unknown_fraction=0.50 still aborts"            "$RG_ARM" done_unknown_fraction 0.50
# The claim radius and the allocator TTL were gen-33 suffix levels; the runner
# now passes neither, so "none" is the only agreeing value. The sentinel branch
# still has to refuse a non-number: compared numerically, awk reads every
# non-number as 0.
rg ABORT "a cell banked at a pinned claim radius 10.0"        "$RG_ARM" coord_claim_radius_override 10.0
rg ABORT "a non-numeric value is not the 'none' sentinel"     "$RG_ARM" coord_claim_radius_override unset
rg ABORT "an empty value cannot be shown to agree"            "$RG_ARM" coord_claim_radius_override ""
rg ABORT "a cell banked at a pinned allocator TTL 120.0"      "$RG_ARM" alloc_peer_pos_max_age_sec 120.0
rg ABORT "a cell predating the allocator TTL key"             "$RG_ARM" alloc_peer_pos_max_age_sec "<none>"
# The separation term's three knobs. The weight is the obvious one; the other
# two are guarded because they are NOT inert at weight 0 -- sep_peer_dist_m and
# sep_eligible_peers are measured on them in every arm, so a resume that moved
# either would bank half a campaign's counterfactual on a different bound.
rg SKIP  "separation_weight=0 matches the campaign default"   "$RG_ARM" separation_weight 0
rg ABORT "a cell banked at separation_weight=0.4 differs"     "$RG_ARM" separation_weight 0.4
rg ABORT "separation_weight absent cannot be shown to agree"  "$RG_ARM" separation_weight "<none>"
# Compared numerically: the manifest carries the node's resolved value and the
# guard the operator's spelling, so 20.0 must match 20. The differing-value
# cases stop that from becoming any value agrees.
# (notes: guardcal-numeric-compare)
rg SKIP  "separation_radius_m=20.0 matches a requested 20"    "$RG_ARM" separation_radius_m 20.0
rg ABORT "a cell banked at separation_radius_m=25.0 differs"  "$RG_ARM" separation_radius_m 25.0
rg ABORT "separation_radius_m absent cannot be shown to agree" "$RG_ARM" separation_radius_m "<none>"
rg SKIP  "separation_max_age_sec=10.0 matches a requested 10" "$RG_ARM" separation_max_age_sec 10.0
rg ABORT "a cell banked at separation_max_age_sec=3.0 differs" "$RG_ARM" separation_max_age_sec 3.0
rg ABORT "separation_max_age_sec absent cannot be shown to agree" \
                                                              "$RG_ARM" separation_max_age_sec "<none>"
# A manifest value that is not a number must abort: awk reads it as 0, which
# agrees with the requested weight of 0. The 0.0 case is the control, so
# abort-on-everything fails too. (notes: guardcal-non-number-aborts)
rg ABORT "separation_weight=off is not a number"              "$RG_ARM" separation_weight off
rg ABORT "separation_weight=unset is not a number"            "$RG_ARM" separation_weight unset
rg ABORT "separation_weight=0x0 is not a number"              "$RG_ARM" separation_weight 0x0
rg ABORT "separation_weight= (truncated line) is not a number" "$RG_ARM" separation_weight ""
rg SKIP  "separation_weight=0.0 IS a number and agrees with 0" "$RG_ARM" separation_weight 0.0
rg ABORT "separation_radius_m=default is not a number"        "$RG_ARM" separation_radius_m default
# nan is tested at keys whose requested value is not 0: this awk compares nan
# equal to anything under a bare +0 compare, which the off and unset cases
# cannot catch. (notes: guardcal-nan-compare)
rg ABORT "separation_max_age_sec=nan is not a number"         "$RG_ARM" separation_max_age_sec nan
rg ABORT "separation_radius_m=nan agrees with 20 under a bare +0" \
                                                              "$RG_ARM" separation_radius_m nan
# The other direction, and the reason the numeric branch trims before matching:
# the regex is STRICTER than `+ 0` was, so a value the old compare accepted
# must not start aborting a correct resume. Whitespace is the realistic way to
# acquire one, and this is the case that would fail if the trim were dropped.
rg SKIP  "separation_radius_m= 20.0 (padded) still resumes"   "$RG_ARM" separation_radius_m " 20.0 "

# The map stream (DESIGN_gen34 section 12). Absent is the one key read as a
# value, 0, since every manifest before the full-map arm ran deltas; a present
# value compares as usual, and a full-map campaign still aborts on an old cell.
# (notes: guardcal-share-full-absent)
rg SKIP  "share_full_period_s absent predates the arm: deltas" "$RG_ARM" share_full_period_s "<none>"
rg SKIP  "share_full_period_s=0 agrees with the delta default" "$RG_ARM" share_full_period_s 0
rg ABORT "a full-map cell (0.5) under a delta campaign"       "$RG_ARM" share_full_period_s 0.5
rg ABORT "share_full_period_s= (truncated line) is not absent" "$RG_ARM" share_full_period_s ""
rg ABORT "share_full_period_s=off is not a number"            "$RG_ARM" share_full_period_s off
RG_ENV="SHARE_FULL_PERIOD_S=0.5"
rg SKIP  "a full-map cell at 0.5 resumes a 0.5 campaign"      "$RG_ARM" share_full_period_s 0.5
rg SKIP  "0.50 agrees with a requested 0.5"                   "$RG_ARM" share_full_period_s 0.50
rg ABORT "an old cell (absent) under a full-map campaign"     "$RG_ARM" share_full_period_s "<none>"
rg ABORT "a delta cell (0.0) under a full-map campaign"       "$RG_ARM" share_full_period_s 0.0
rg ABORT "a full-map cell at 1.0 under a 0.5 campaign"        "$RG_ARM" share_full_period_s 1.0
unset RG_ENV
# Absent reads as 0 only for a manifest from before the arm: the new runner
# writes map_stream beside it, so absence there is a damaged manifest.
RG_EXTRA="map_stream=delta"
rg ABORT "share_full_period_s absent beside map_stream"       "$RG_ARM" share_full_period_s "<none>"
unset RG_EXTRA
# transmission_model (section 12.12): absent is admission, same rule.
rg SKIP  "transmission_model absent predates it: admission"   "$RG_ARM" transmission_model "<none>"
rg SKIP  "transmission_model=admission agrees with default"   "$RG_ARM" transmission_model admission
rg ABORT "a progressive cell under an admission campaign"     "$RG_ARM" transmission_model progressive
RG_EXTRA="map_stream=delta"
rg ABORT "transmission_model absent beside map_stream"        "$RG_ARM" transmission_model "<none>"
unset RG_EXTRA
RG_ENV="TX_MODEL=progressive"
rg SKIP  "a progressive cell resumes a progressive campaign"  "$RG_ARM" transmission_model progressive
rg ABORT "an old cell (absent) under a progressive campaign"  "$RG_ARM" transmission_model "<none>"
rg ABORT "an admission cell under a progressive campaign"     "$RG_ARM" transmission_model admission
RG_ENV="TX_MODEL=bogus"
rg "REFUSED(rc=2)" "TX_MODEL=bogus fails up front"            "$RG_ARM"
RG_ENV="SHARE_FULL_PERIOD_S=junk"
rg "REFUSED(rc=2)" "SHARE_FULL_PERIOD_S=junk fails up front"  "$RG_ARM"
unset RG_ENV

# --- the verdict itself, which the resume guard used to read one bit of ------
# A banked SUSPECT, verdict-less or unknown-verdict cell still skips but says so
# (SKIP-DIRTY), not with the certified SKIP line. Still skipping and saying why
# are pinned separately. (notes: guardcal-verdict-directions)
rg SKIP-DIRTY "a banked SUSPECT cell still skips, but says so"  "$RG_ARM" run_gates_verdict SUSPECT
rg SKIP-DIRTY "a banked cell with no verdict at all says so"    "$RG_ARM" run_gates_verdict "<none>"
rg SKIP-DIRTY "a verdict token nothing emits is not read as CLEAN" "$RG_ARM" run_gates_verdict VALID
# INVALID still falls through to a re-run (stopped here by the disk guard, hence
# LAUNCHED), not SKIP-DIRTY: it is the only verdict this driver remedies
# automatically. (notes: guardcal-invalid-still-redoes)
rg LAUNCHED   "INVALID still redoes the cell, not just reports it" "$RG_ARM" run_gates_verdict INVALID

# REDO_SUSPECT=1 re-rolls SUSPECT cells: tested at 1 (LAUNCHED, stopped by the
# huge MIN_FREE_MB) and 0 (SKIP-DIRTY). Run inline because rg() has no env hook.
# (notes: guardcal-redo-suspect)
for _rs in 1 0; do
  cases=$((cases+1))
  _root="$TMP/resume_redo_$_rs"; _cell="$_root/rg_${RG_ARM}_seed1"
  mkdir -p "$_cell"
  rg_manifest "$RG_ARM" | sed 's/^run_gates_verdict=.*/run_gates_verdict=SUSPECT/' \
    > "$_cell/run_manifest.txt"
  _out=$(REDO_SUSPECT=$_rs MIN_FREE_MB=999999999999 timeout 60 "$CS" \
           --root "$_root" --tag rg --duration 3000 \
           --scenario flatforest_dense_2robot_lidar.yaml \
           --arms "$RG_ARM" --seeds 1 2>&1)
  if echo "$_out" | grep -q "MB free under"; then _got=LAUNCHED
  elif echo "$_out" | grep -q "SKIP rg_${RG_ARM}_seed1 (complete, but"; then _got=SKIP-DIRTY
  else _got=OTHER; fi
  [ "$_rs" = 1 ] && _want=LAUNCHED || _want=SKIP-DIRTY
  if [ "$_got" = "$_want" ]; then
    echo "  PASS  REDO_SUSPECT=$_rs on a SUSPECT bank ($_got)"
  else
    echo "  FAIL  REDO_SUSPECT=$_rs on a SUSPECT bank: want $_want got $_got"
    echo "$_out" | grep -E "FATAL|ABORT|SKIP|REDO|free under" | sed 's/^/          | /'
    fails=$((fails+1))
  fi
done
# REDO_SUSPECT must NOT re-roll a CLEAN cell. Without this the knob reads as
# "re-run everything", which would silently discard a finished campaign.
cases=$((cases+1))
_root="$TMP/resume_redo_clean"; _cell="$_root/rg_${RG_ARM}_seed1"
mkdir -p "$_cell"; rg_manifest "$RG_ARM" > "$_cell/run_manifest.txt"
_out=$(REDO_SUSPECT=1 MIN_FREE_MB=999999999999 timeout 60 "$CS" \
         --root "$_root" --tag rg --duration 3000 \
         --scenario flatforest_dense_2robot_lidar.yaml \
         --arms "$RG_ARM" --seeds 1 2>&1)
if echo "$_out" | grep -q "SKIP rg_${RG_ARM}_seed1 (already complete)"; then
  echo "  PASS  REDO_SUSPECT=1 leaves a CLEAN cell banked (SKIP)"
else
  echo "  FAIL  REDO_SUSPECT=1 re-rolled a CLEAN cell"
  echo "$_out" | grep -E "FATAL|ABORT|SKIP|REDO|free under" | sed 's/^/          | /'
  fails=$((fails+1))
fi
unset _want _lbl _arm _key _val _root _cell _out _rc _got _rs RG_ARM

echo
echo "=== the resume guard's copies of the launcher defaults must still be true ==="
# The resume guard keeps copies of launcher defaults to predict each manifest; a
# stale copy aborts correct resumes or skips cells from the wrong regime. Read
# TREE_ATTEN, MAX_RANGE and CELL_SIZE_M back.
# (notes: guardcal-resume-default-readback)
for _spec in TREE_ATTEN:TREE_ATTEN_REQ MAX_RANGE:MAX_RANGE_REQ \
             CELL_SIZE_M:CELL_SIZE_REQ; do
  _ek=${_spec%%:*}; _vn=${_spec#*:}
  cases=$((cases+1))
  _ld=$(sed -n "s/^$_ek=\"\$(flt \"\${$_ek:-\(.*\)}\")\"\$/\1/p" "$LAUNCHER" | head -1)
  _gd=$(sed -n "s/^$_vn=\$(env_val $_ek \(.*\))\$/\1/p" "$CS" | head -1)
  if [ -n "$_ld" ] && [ "$_ld" = "$_gd" ]; then
    echo "  PASS  launcher default $_ek=$_ld matches the resume guard's copy"
  else
    echo "  FAIL  launcher default $_ek is '${_ld:-UNREADABLE}' but the resume" \
         "guard assumes '${_gd:-UNREADABLE}'"
    fails=$((fails+1))
  fi
done
unset _spec _ek _vn _ld _gd
# The separation knobs are declared in the launcher WITHOUT flt() -- they are
# validated in place instead -- so they need their own readback pattern. Same
# hazard, same consequence: a stale copy aborts every resume of a campaign that
# is running exactly as intended.
for _spec in SEPARATION_WEIGHT:SEPARATION_WEIGHT_REQ \
             SEPARATION_RADIUS_M:SEPARATION_RADIUS_REQ \
             SEPARATION_MAX_AGE_SEC:SEPARATION_MAX_AGE_REQ \
             SHARE_FULL_PERIOD_S:SHARE_FULL_PERIOD_REQ \
             TX_MODEL:TX_MODEL_REQ; do
  _ek=${_spec%%:*}; _vn=${_spec#*:}
  cases=$((cases+1))
  _ld=$(sed -n "s/^$_ek=\"\${$_ek:-\(.*\)}\"$/\1/p" "$LAUNCHER" | head -1)
  _gd=$(sed -n "s/^$_vn=\$(env_val $_ek \(.*\))$/\1/p" "$CS" | head -1)
  if [ -n "$_ld" ] && [ "$_ld" = "$_gd" ]; then
    echo "  PASS  launcher default $_ek=$_ld matches the resume guard's copy"
  else
    echo "  FAIL  launcher default $_ek is '${_ld:-UNREADABLE}' but the resume" \
         "guard assumes '${_gd:-UNREADABLE}'"
    fails=$((fails+1))
  fi
done
unset _spec _ek _vn _ld _gd

echo
[ "$fails" = 0 ] && echo "ALL PASS ($cases known-answer cases)" \
                 || echo "$fails FAILURE(S) of $cases known-answer cases"
exit $((fails > 0))
