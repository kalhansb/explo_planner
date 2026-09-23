#!/usr/bin/env bash
# Known-answer cases for run_campaign.sh's guards.
#
# Main sections: the link veto, which refuses a campaign arming the mid-run
# reconnect trigger at the 90 s clock (below the ~180 s heartbeat-suppression
# tail) with no link veto; launcher validation; the resume guard.
# (notes: guardcal-link-veto-scope)
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

# want=BLOCK  -> the guard must fire: its FATAL text appears AND the run refuses
# want=ALLOW  -> the guard must not fire AND the campaign must actually be
#                accepted, i.e. exit 0
#
# ALLOW requires exit 0, not merely the absence of the guard's FATAL text, so an
# unrelated refusal is not a pass; the planted --bogus case pins it.
# (notes: guardcal-allow-needs-exit-zero)
t() {
  want="$1"; label="$2"; shift 2
  cases=$((cases+1))
  out=$(timeout 20 "$CS" --root "$TMP" --duration 3000 \
        --scenario flatforest_dense_2robot_lidar.yaml --dry-run "$@" 2>&1)
  rc=$?
  if echo "$out" | grep -q "would run the mid-run trigger with NO link veto"; then
    # The text alone is not the verdict: a guard that printed the FATAL and
    # then launched anyway would be worse than no guard.
    [ "$rc" = 0 ] && got="PRINTED-BUT-RAN" || got=BLOCK
  elif [ "$rc" = 0 ]; then
    got=ALLOW
  else
    got="REFUSED(rc=$rc)"   # neither: something else stopped it first
  fi
  if [ "$got" = "$want" ]; then
    echo "  PASS  $label ($got)"
  else
    echo "  FAIL  $label: want $want got $got"
    echo "$out" | grep -E "FATAL|MIDRUN|LINK" | sed 's/^/          | /'
    fails=$((fails+1))
  fi
}

echo "=== the hazard itself ==="
t BLOCK "LINK_GATE=0 at the default clock"        --arms hybrid,off --seeds 1 --env "LINK_GATE=0"
t BLOCK "--comms 0 at the default clock"          --arms hybrid,off --seeds 1 --comms 0
t BLOCK "LINK_GATE=false (not a 1, so off)"       --arms hybrid,off --seeds 1 --env "LINK_GATE=false"
t BLOCK "LINK_GATE=2"                             --arms hybrid,off --seeds 1 --env "LINK_GATE=2"
t BLOCK "LINK_GATE= (empty)"                      --arms hybrid,off --seeds 1 --env "LINK_GATE="

echo "=== the escape hatch must clear the tail, not merely be mentioned ==="
t ALLOW "MIDRUN_SILENCE=240 with the gate off"    --arms hybrid,off --seeds 1 --env "LINK_GATE=0 MIDRUN_SILENCE=240"
t ALLOW "MIDRUN_SILENCE=200, exactly the floor"   --arms hybrid,off --seeds 1 --env "LINK_GATE=0 MIDRUN_SILENCE=200"
t BLOCK "MIDRUN_SILENCE=5, worse than default"    --arms hybrid,off --seeds 1 --env "LINK_GATE=0 MIDRUN_SILENCE=5"
t BLOCK "MIDRUN_SILENCE=199, just under"          --arms hybrid,off --seeds 1 --env "LINK_GATE=0 MIDRUN_SILENCE=199"

echo "=== substring collisions, both directions ==="
t BLOCK "GATE_MIDRUN_SILENCE=240 must NOT disarm" --arms hybrid,off --seeds 1 --env "LINK_GATE=0 GATE_MIDRUN_SILENCE=240"
t ALLOW "MY_LINK_GATE=0 must NOT arm"             --arms hybrid,off --seeds 1 --env "MY_LINK_GATE=0"

echo "=== arm awareness ==="
t ALLOW "off-only campaign has no treatment"      --arms off --seeds 1 --comms 0
t BLOCK "a --cells list containing a treated arm" --cells "off:1,hybrid:2" --comms 0
t ALLOW "--cells with only off"                   --cells "off:1,off:2" --comms 0
t BLOCK "a future arm is inside the guard"        --arms pursuit,off --seeds 1 --comms 0
# The self-check for t()'s ALLOW verdict: an unrelated refusal must not be
# reported as a permitted campaign. Before the exit code was checked, this line
# printed "PASS ... (ALLOW)".
t "REFUSED(rc=2)" "an unrelated refusal is not an ALLOW" --arms hybrid,off --seeds 1 --bogus

echo "=== the happy path ==="
t ALLOW "the real generation-9 campaign"          --arms hybrid,off --seeds 1

echo
echo "=== the other refusals (checked separately, they exit before the guard) ==="
chk() {
  want="$1"; label="$2"; shift 2
  cases=$((cases+1))
  out=$(timeout 20 "$CS" --root "$TMP" --duration 3000 \
        --scenario flatforest_dense_2robot_lidar.yaml --dry-run "$@" 2>&1)
  if echo "$out" | grep -q -e "$want"; then
    echo "  PASS  $label"
  else
    echo "  FAIL  $label: /$want/ not in output"
    echo "$out" | grep -E "FATAL" | sed 's/^/          | /'
    fails=$((fails+1))
  fi
}
chk "not --env COMMS="        "--env COMMS=0 is refused"     --arms hybrid,off --seeds 1 --env "COMMS=0"
chk "--comms must be 0 or 1"  "--comms 2 is refused"         --arms hybrid,off --seeds 1 --comms 2
chk "is not a number"         "a non-numeric clock refused"  --arms hybrid,off --seeds 1 --env "LINK_GATE=0 MIDRUN_SILENCE=lots"

echo
echo "=== the ambient environment must not reach the cells ==="
# env without -i inherits the caller's environment, so the per-cell launch must
# strip LINK_GATE and MIDRUN_SILENCE. Asserted on the launch line, since the
# guard's verdict is the same either way.
# (notes: guardcal-ambient-link-gate-strip)
cases=$((cases+1))
if grep -q -- '-u LINK_GATE -u MIDRUN_SILENCE' "$CS"; then
  echo "  PASS  the per-cell launch strips both from the inherited environment"
else
  echo "  FAIL  the per-cell launch does not strip LINK_GATE/MIDRUN_SILENCE, so" \
       "an exported value silently beats the guard"
  fails=$((fails+1))
fi
# And the strip must not break the channel the guard DOES read: env applies
# assignments after unsets, so --env has to keep winning. If that ordering were
# wrong, every escape-hatch case above would still pass (they check the guard,
# which runs before the launch) while no cell ever received the setting.
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
  echo "  PASS  an exported LINK_GATE is reported and does not trip the guard"
else
  echo "  FAIL  an exported LINK_GATE was neither reported nor refused (rc=$rc)"
  fails=$((fails+1))
fi

echo
echo "=== the guard's copy of the launcher default must still be true ==="
# run_campaign.sh hard-codes the launcher's LINK_GATE default to decide before
# launch whether the veto is live; nothing else links the two files, so read the
# literal back and compare. (notes: guardcal-link-gate-default-readback)
LAUNCHER="$(dirname "$CS")/run_explo_sim_rviz.sh"
lg_default=$(sed -n 's/^LINK_GATE="\${LINK_GATE:-\([^}]*\)}"$/\1/p' "$LAUNCHER" | head -1)
guard_assumes=$(sed -n 's/^env_has LINK_GATE || _link_gate_req=\([0-9]*\).*$/\1/p' "$CS" | head -1)
cases=$((cases+1))
if [ -n "$lg_default" ] && [ "$lg_default" = "$guard_assumes" ]; then
  echo "  PASS  launcher default LINK_GATE=$lg_default matches the guard's assumption"
else
  echo "  FAIL  launcher default is '${lg_default:-UNREADABLE}' but the guard" \
       "assumes '${guard_assumes:-UNREADABLE}'"
  fails=$((fails+1))
fi

echo
echo "=== --env passengers that would silently collapse the design ==="
# Each of these is set per cell on the env line at the bottom of run_campaign.sh
# and --env is expanded after it, so a passenger wins over the real value while
# every name, index row and manifest still claims the intended one.
chk "RECONNECT_MODE is set per cell" "--env RECONNECT_MODE is refused" \
    --arms hybrid,off --seeds 1 --env "RECONNECT_MODE=off"
chk "SEED is set per cell"           "--env SEED is refused" \
    --arms hybrid,off --seeds 1,2 --env "SEED=7"
chk "DONE_SEEK"                      "--env DONE_SEEK is refused" \
    --arms hybrid,off --seeds 1 --env "DONE_SEEK=1"
chk "MISSION_RETURN"                 "--env MISSION_RETURN is refused" \
    --arms hybrid,off --seeds 1 --env "MISSION_RETURN=0"
# The quietest of the set: the claim radius comes from the _r<N> arm suffix, so
# a passenger would run every cell of cr2 at one radius while half the
# directories still said the other. Gate check 3i catches it from the manifest,
# a campaign too late.
chk "COORD_CLAIM_R is set per cell"  "--env COORD_CLAIM_R is refused" \
    --arms mtare_hybrid_r10,mtare_hybrid_r40 --seeds 1 --env "COORD_CLAIM_R=40"
# And the key matcher must not fire on a name that merely ends in the key --
# env_has splits on whitespace and compares up to '=', so this must run.
t ALLOW "MY_SEED=7 must not be read as SEED" --arms hybrid,off --seeds 1 --env "MY_SEED=7"

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
echo "=== the launcher's own validation blocks (the M-TARE knobs) ==="
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

  # And the cut has to CONTAIN the guards. A range that stopped short would
  # fail every BLOCK case for the wrong reason and pass every ALLOW one.
  for _need in "FATAL: CELL_WORLD" "FATAL: TEAM_WORLD=" "FATAL: TEAM_WORLD_HZ" \
               "FATAL: RENDEZVOUS_SCHEDULE" "FATAL: PURSUIT_PREDICTOR" \
               "the arm name and the arm" "has no runnable configuration"; do
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

  # The two controls that give every refusal below its meaning. If the first
  # ever fails, the launcher is refusing a bare invocation; if the second fails,
  # the mtare_hybrid arm does not exist and the campaign has no treatment.
  #
  # Both cases assert the same vector by two routes: the shipped default and the
  # mtare_hybrid token must not drift apart.
  # (notes: guardcal-default-is-mtare-hybrid)
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=1 pp=trail rm=mtare_hybrid' \
     "shipped defaults resolve to the mtare_hybrid stack"
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=1 pp=trail rm=mtare_hybrid' \
     "RECONNECT_MODE=mtare_hybrid expands to the whole P1-P5 stack" \
     RECONNECT_MODE=mtare_hybrid
  # An all-knobs-off vector still has to resolve, or the launcher is refusing
  # off-arm cells -- that is what the old bare-default case was really buying.
  # `off` is one of the two plain tokens that survive, so it carries the job.
  lg ALLOW '__PRELUDE_OK__ cw=0 tw=0 hz=1.0 ga=0 rg=silence rs=0 pp=trail rm=off' \
     "an all-knobs-off vector still resolves" \
     RECONNECT_MODE=off

  # The two plain tokens that CANNOT run, asserted in both directions so the
  # dead end is a property under test rather than a comment. Either refusal
  # going quiet means a cell can be filed under a name the node will not stamp.
  lg BLOCK "has no runnable configuration" \
     "plain hybrid at its default is refused (it would behave as pursuit)" \
     RECONNECT_MODE=hybrid
  lg BLOCK "has no runnable configuration" \
     "plain rendezvous at its default is refused (it would behave as off)" \
     CELL_WORLD=1 TEAM_WORLD=1 RECONNECT_MODE=rendezvous
  lg BLOCK "FATAL: the arm name and the arm" \
     "and turning the schedule ON under a plain token is the other refusal" \
     CELL_WORLD=1 TEAM_WORLD=1 RENDEZVOUS_SCHEDULE=1 RECONNECT_MODE=rendezvous

  # The other three factorial cells: mtare_pursuit is the chase without an
  # appointment, mtare_rendezvous the appointment without a chase, mtare_off
  # neither. Each full vector is asserted. (notes: guardcal-factorial-tokens)
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=silence rs=0 pp=trail rm=mtare_off' \
     "RECONNECT_MODE=mtare_off expands to P1-P3 and NEITHER mechanism" \
     RECONNECT_MODE=mtare_off
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=0 pp=trail rm=mtare_pursuit' \
     "RECONNECT_MODE=mtare_pursuit expands to the chase WITHOUT an appointment" \
     RECONNECT_MODE=mtare_pursuit
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=1 pp=trail rm=mtare_rendezvous' \
     "RECONNECT_MODE=mtare_rendezvous expands to the appointment WITHOUT a chase" \
     RECONNECT_MODE=mtare_rendezvous

  # A typo in a 0/1 knob would read as OFF and go unnoticed; only the launcher
  # can catch it. RECONNECT_MODE is pursuit so the arm stack is empty and the
  # 0/1 check, not the mtare_* contradiction check, fires.
  # (notes: guardcal-01-knob-typos)
  lg BLOCK "FATAL: CELL_WORLD='true'" \
     "CELL_WORLD=true is refused, not silently read as off" \
     CELL_WORLD=true RECONNECT_MODE=pursuit
  lg BLOCK "FATAL: TEAM_WORLD='yes'" \
     "TEAM_WORLD=yes is refused, not silently read as off" \
     CELL_WORLD=1 TEAM_WORLD=yes RECONNECT_MODE=pursuit

  # TEAM_WORLD_HZ is the exchange's on/off switch as well as its rate: the node
  # builds no publisher and no timer at <=0, so these three would each have
  # recorded team_world=1 on a cell that exchanged nothing.
  lg BLOCK "FATAL: TEAM_WORLD_HZ='0.0'" \
     "TEAM_WORLD_HZ=0 is refused (it disables the exchange it records)" \
     CELL_WORLD=1 TEAM_WORLD=1 TEAM_WORLD_HZ=0
  lg BLOCK "FATAL: TEAM_WORLD_HZ='-1.0'" \
     "a negative TEAM_WORLD_HZ is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 TEAM_WORLD_HZ=-1
  lg BLOCK "FATAL: TEAM_WORLD_HZ='<empty: rejected by flt>'" \
     "flt's empty return for nan is refused, not passed to ros2 as a bare -p" \
     CELL_WORLD=1 TEAM_WORLD=1 TEAM_WORLD_HZ=nan
  # RECONNECT_MODE=pursuit, explicitly: this case is about TEAM_WORLD_HZ, and
  # leaving the mode at its default would make it also assert the default's
  # whole stack and fail the day that changes for an unrelated reason.
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=2.0 ga=0 rg=silence rs=0 pp=trail rm=pursuit' \
     "a legitimate TEAM_WORLD_HZ still passes -- the guard is not a blanket no" \
     CELL_WORLD=1 TEAM_WORLD=1 TEAM_WORLD_HZ=2 RECONNECT_MODE=pursuit

  # The node stamps an mtare_ prefix from its knobs, but directories and
  # analysis key off the name, so a knob set by hand under a plain name files a
  # treated cell as control. Each knob gets its own case.
  # (notes: guardcal-arm-stamp-per-knob)
  lg BLOCK "FATAL: the arm name and the arm" \
     "GLOBAL_ALLOC=1 under the plain hybrid name is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 GLOBAL_ALLOC=1 RECONNECT_MODE=hybrid
  lg BLOCK "FATAL: the arm name and the arm" \
     "RECONNECT_GATE=info under the plain hybrid name is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 GLOBAL_ALLOC=1 RECONNECT_GATE=info RECONNECT_MODE=hybrid
  lg BLOCK "FATAL: the arm name and the arm" \
     "RENDEZVOUS_SCHEDULE=1 under the plain hybrid name is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 RENDEZVOUS_SCHEDULE=1 RECONNECT_MODE=hybrid

  # The P5 knob's own preconditions, mirroring the node's fatals. Each names a
  # configuration in which the cell would RECORD the treatment and not carry
  # it, which is the failure mode the manifest cannot self-diagnose.
  lg BLOCK "FATAL: RENDEZVOUS_SCHEDULE=1 requires TEAM_WORLD=1" \
     "the appointment without the exchange is refused" \
     CELL_WORLD=1 RENDEZVOUS_SCHEDULE=1 RECONNECT_MODE=hybrid
  lg BLOCK "FATAL: RENDEZVOUS_SCHEDULE=1 with RECONNECT_MODE=off" \
     "an appointment with no manoeuvre to schedule is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 RENDEZVOUS_SCHEDULE=1 RECONNECT_MODE=off
  lg BLOCK "FATAL: RENDEZVOUS_SCHEDULE=1 with RECONNECT_MODE=pursuit" \
     "an appointment in the appointment-OFF cell of the 2x2 is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 RENDEZVOUS_SCHEDULE=1 RECONNECT_MODE=pursuit
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_pursuit implies RENDEZVOUS_SCHEDULE=0" \
     "contradicting the mtare_pursuit token's own stack is refused" \
     RENDEZVOUS_SCHEDULE=1 RECONNECT_MODE=mtare_pursuit
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_rendezvous implies RECONNECT_GATE=info" \
     "contradicting the mtare_rendezvous token's own stack is refused" \
     RECONNECT_GATE=silence RECONNECT_MODE=mtare_rendezvous

  # PURSUIT_PREDICTOR also flips the node's mtare_ prefix: typo, arm-stamp and
  # precondition cases. RECONNECT_MODE is pursuit because every mtare_* token
  # pins the predictor and would refuse MDP first.
  # (notes: guardcal-pursuit-predictor-cases)
  lg BLOCK "FATAL: PURSUIT_PREDICTOR='MDP'" \
     "a mis-cased PURSUIT_PREDICTOR is refused, not read as trail" \
     PURSUIT_PREDICTOR=MDP RECONNECT_MODE=pursuit
  lg BLOCK "FATAL: the arm name and the arm" \
     "PURSUIT_PREDICTOR=mdp under the plain hybrid name is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 GLOBAL_ALLOC=1 PURSUIT_PREDICTOR=mdp \
     RECONNECT_MODE=hybrid
  # Under a plain arm, because the mtare_* tokens all set GLOBAL_ALLOC=1
  # themselves and this precondition is unreachable beneath any of them. The
  # PURSUIT_PREDICTOR block runs BEFORE the arm-stamp guard, so a plain token
  # reaches this refusal rather than the name/stamp one.
  lg BLOCK "FATAL: PURSUIT_PREDICTOR=mdp requires GLOBAL_ALLOC=1" \
     "the predictor without the allocator that makes tours is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 PURSUIT_PREDICTOR=mdp RECONNECT_MODE=hybrid
  lg BLOCK "FATAL: PURSUIT_PREDICTOR=mdp with RECONNECT_MODE=rendezvous" \
     "aiming a chase in the chase-OFF cell of the 2x2 is refused" \
     CELL_WORLD=1 TEAM_WORLD=1 GLOBAL_ALLOC=1 PURSUIT_PREDICTOR=mdp \
     RECONNECT_MODE=rendezvous
  # The pin, in both directions. The four factorial tokens fix the predictor at
  # `trail` (an mdp cell would be indexed as the arm it is not), and the ALLOW
  # cases above already assert pp=trail comes out of each; this asserts the
  # token REFUSES the override rather than quietly winning over it.
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_hybrid implies PURSUIT_PREDICTOR=trail" \
     "contradicting the mtare_hybrid token's predictor pin is refused" \
     PURSUIT_PREDICTOR=mdp RECONNECT_MODE=mtare_hybrid
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_pursuit implies PURSUIT_PREDICTOR=trail" \
     "contradicting the mtare_pursuit token's predictor pin is refused" \
     PURSUIT_PREDICTOR=mdp RECONNECT_MODE=mtare_pursuit

  # The two _mdp tokens are the only way to request the predictor; the stack
  # comes from the name, which the node stamps back. Asserted: the expansion,
  # and refusal of an override that splits name from stamp.
  # (notes: guardcal-mdp-tokens)
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=0 pp=mdp rm=mtare_pursuit_mdp' \
     "mtare_pursuit_mdp expands to the mtare_pursuit stack with the predictor on" \
     RECONNECT_MODE=mtare_pursuit_mdp
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=1 pp=mdp rm=mtare_hybrid_mdp' \
     "mtare_hybrid_mdp expands to the mtare_hybrid stack with the predictor on" \
     RECONNECT_MODE=mtare_hybrid_mdp
  lg BLOCK "FATAL: RECONNECT_MODE=mtare_hybrid_mdp implies PURSUIT_PREDICTOR=mdp" \
     "turning the predictor back off under an _mdp token is refused" \
     PURSUIT_PREDICTOR=trail RECONNECT_MODE=mtare_hybrid_mdp
  # There is no _mdp counterpart for the two arms that never chase, and the
  # name has to be refused as unknown rather than mapped to the nearest thing
  # that parses -- a token that silently degrades is a mislabelled cell.
  lg BLOCK "FATAL: RECONNECT_MODE='mtare_rendezvous_mdp' is not one of" \
     "an _mdp name for a chase-OFF arm is not a token" \
     RECONNECT_MODE=mtare_rendezvous_mdp

  # --- the roster axis (P7) -------------------------------------------------
  # The roster is read from the scenario and cannot be overridden, so SCEN
  # selects it. The N == 2 case is the control: an assertion matching a constant
  # string would pass at both sizes. (notes: guardcal-roster-axis)
  lg ALLOW '__ROSTER_OK__ n=2 robots=[atlas bestla]' \
     "the roster resolves to the 2-robot scenario's own names"
  SCEN=flatforest_3robot_lidar.yaml
  lg ALLOW '__ROSTER_OK__ n=3 robots=[atlas bestla husky]' \
     "the roster resolves to THREE at the 3-robot scenario"
  # And the guards are not quietly conditioned on the pair: one refusal and one
  # token expansion, re-run at N == 3. If either changed with the roster size the
  # M-TARE knobs would mean something different in an N-robot campaign.
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=1 pp=mdp rm=mtare_hybrid_mdp' \
     "mtare_hybrid_mdp expands to the same stack at N == 3" \
     RECONNECT_MODE=mtare_hybrid_mdp
  lg BLOCK "FATAL: the arm name and the arm" \
     "the arm-stamp guard still fires at N == 3" \
     CELL_WORLD=1 TEAM_WORLD=1 GLOBAL_ALLOC=1 RECONNECT_MODE=hybrid
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
RG_ARM=mtare_hybrid
# Agrees with the reference campaign below on every key the guard reads. Each
# case overwrites, or deletes, exactly one line.
#
# run_gates_verdict is CLEAN, a token the harness can write: the teardown writes
# exactly one of CLEAN, SUSPECT or INVALID.
# (notes: guardcal-fixture-verdict-clean)
rg_manifest() {
  cat <<'EOF'
run_end_reason=all_done
run_gates_verdict=CLEAN
mission_return_enabled=true
scenario=flatforest_dense_2robot_lidar.yaml
duration_s=3000
done_criterion=latch
cell_world=1
team_world=1
global_alloc=1
reconnect_gate=info
rendezvous_schedule=1
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
link_gate=1
link_gate_effective=1
done_seek_enabled=false
reconnect_midrun_silence_sec=90
team_world_hz=1.0
EOF
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
    rg_manifest | grep -v "^$_key=" > "$_cell/run_manifest.txt"
    [ "$_val" = "<none>" ] || echo "$_key=$_val" >> "$_cell/run_manifest.txt"
  else
    rg_manifest > "$_cell/run_manifest.txt"
  fi
  _out=$(MIN_FREE_MB=999999999999 timeout 60 "$CS" --root "$_root" --tag rg \
           --duration 3000 --scenario flatforest_dense_2robot_lidar.yaml \
           --arms "$_arm" --seeds 1 2>&1)
  _rc=$?
  if echo "$_out" | grep -q "ABORT: rg_${_arm}_seed1 is complete but its manifest says"; then
    # As with the link veto above, the text alone is not the verdict: a guard
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
# cr2's independent variable, and the only key with a sentinel. "none" and
# "10.0" are different cells even though the yaml default is 10: one pinned the
# radius, one took whatever the config said that day.
rg SKIP  "_r40 with a matching 40.0 override is skipped"      mtare_hybrid_r40 coord_claim_radius_override 40.0
rg ABORT "_r40 banked at 10.0 is a different level"           mtare_hybrid_r40 coord_claim_radius_override 10.0
rg ABORT "_r40 banked with no override at all"                mtare_hybrid_r40 coord_claim_radius_override none
rg ABORT "an unsuffixed arm banked at a pinned 10.0"          "$RG_ARM" coord_claim_radius_override 10.0
# The sentinel branch itself. Compared numerically, awk reads EVERY non-number
# as 0, so "unset" would equal "none" and a cell whose radius can no longer be
# determined would score as agreeing. This is the case that separates the two
# spellings; the four above pass either way.
rg ABORT "a non-numeric value is not the 'none' sentinel"     "$RG_ARM" coord_claim_radius_override unset
rg ABORT "an empty value cannot be shown to agree"            "$RG_ARM" coord_claim_radius_override ""
# alloc_peer_pos_max_age_sec comes from the _ttl<N> suffix; 0 is a real level
# (unbounded, the control), so 0.0 and none are different cells. The two SKIP
# cases are the only test of the suffix parser. (notes: guardcal-ttl-suffix)
rg SKIP  "_ttl120 with a matching 120.0 is skipped"           mtare_hybrid_ttl120 alloc_peer_pos_max_age_sec 120.0
rg SKIP  "_ttl0 (the control level) matches an explicit 0.0"  mtare_hybrid_ttl0 alloc_peer_pos_max_age_sec 0.0
# The case this key exists for: a cell named for the treatment, banked with the
# control's behaviour. Nothing else in the manifest differs -- the arm token, the
# six M-TARE knobs and the radio regime are all identical between the two levels.
rg ABORT "_ttl120 banked at 0.0 ran unbounded under the treated name" \
                                                              mtare_hybrid_ttl120 alloc_peer_pos_max_age_sec 0.0
rg ABORT "_ttl120 banked before the knob existed"             mtare_hybrid_ttl120 alloc_peer_pos_max_age_sec "<none>"
rg ABORT "_ttl0 banked with no -p at all is a different route" mtare_hybrid_ttl0 alloc_peer_pos_max_age_sec none
rg ABORT "an unsuffixed arm banked at a pinned 120.0"         "$RG_ARM" alloc_peer_pos_max_age_sec 120.0
rg ABORT "an unsuffixed arm predating the knob"               "$RG_ARM" alloc_peer_pos_max_age_sec "<none>"
# The numeric branch at this key, exercised at a level that is NOT 0 so that the
# `a + 0` reading of an unparseable string cannot accidentally agree.
rg ABORT "_ttl120 banked as 'unset' is not a number"          mtare_hybrid_ttl120 alloc_peer_pos_max_age_sec unset
rg ABORT "_ttl120 banked as nan agrees with 120 under a bare +0" \
                                                              mtare_hybrid_ttl120 alloc_peer_pos_max_age_sec nan
rg SKIP  "_ttl120 banked as ' 120.0 ' (padded) still resumes" mtare_hybrid_ttl120 alloc_peer_pos_max_age_sec " 120.0 "
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
  rg_manifest | sed 's/^run_gates_verdict=.*/run_gates_verdict=SUSPECT/' \
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
mkdir -p "$_cell"; rg_manifest > "$_cell/run_manifest.txt"
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
             SEPARATION_MAX_AGE_SEC:SEPARATION_MAX_AGE_REQ; do
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
