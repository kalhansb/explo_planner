#!/usr/bin/env bash
# Known-answer cases for the run_campaign.sh link-veto guard.
#
# The guard refuses a campaign that would arm the mid-run reconnect trigger at
# the generation-9 clock (90 s, below the ~180 s heartbeat-suppression tail)
# with no link veto to tell a quiet teammate from an absent one. Its first draft
# was wrong in four ways at once, none of which a reading would have caught:
# it disarmed on the mere PRESENCE of the string "MIDRUN_SILENCE=" (so
# GATE_MIDRUN_SILENCE=240 turned it off, and MIDRUN_SILENCE=5 passed), it armed
# on "LINK_GATE=0" as a substring (so MY_LINK_GATE=0 tripped it), it matched
# only the literal value 0 while the launcher treats every non-1 value as off,
# it never saw COMMS= arriving through --env where it beats the flag, and it
# blocked control-only campaigns that have no treatment to protect.
#
# Every case runs --dry-run, so nothing launches. Asserting the ALLOW cases by
# letting them start and killing the driver orphaned six gazebo/scovox/
# robot_state_publisher processes the first time it was tried; the last section
# checks the process census to make sure that cannot come back.
#
# Run: ./campaign_guard_calib.sh   -- exits non-zero on any FAIL.
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
# ALLOW checks the EXIT CODE and not merely the absence of the guard's FATAL
# text. An earlier version asserted absence alone and discarded the status, so
# any unrelated refusal scored as ALLOW -- `--bogus` exits 2 on an unknown
# argument, never reaches the guard, and was counted a pass. That is a check
# that agrees with itself: the campaign it certifies as permitted is one that
# cannot run at all. The planted case at the end of the arm section pins it.
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
# The hole every case above was blind to, because every case passes its setting
# through --env, which is the one channel the guard reads. `env` without -i
# inherits the caller's environment and the launch line did not assign LINK_GATE
# or MIDRUN_SILENCE, so `export LINK_GATE=0` before a normal launch ran the whole
# treated matrix on the 90 s clock with no veto and no complaint.
#
# Asserted on the LAUNCH LINE rather than on the guard's verdict, because the
# guard would (correctly) stay silent either way -- the bug was never in what it
# concluded, it was in what reached the cells afterwards.
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
# run_campaign.sh decides whether the veto will be live BEFORE anything is
# launched, so it has to know what LINK_GATE defaults to inside
# run_explo_sim_rviz.sh, and it hard-codes 1. Nothing else links the two files:
# flip the launcher default to 0 and the guard would happily pass a campaign
# that runs the 90 s clock bare, which is the entire hazard it exists to stop.
# Read the literal back out and compare.
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
# And the key matcher must not fire on a name that merely ends in the key --
# env_has splits on whitespace and compares up to '=', so this must run.
t ALLOW "MY_SEED=7 must not be read as SEED" --arms hybrid,off --seeds 1 --env "MY_SEED=7"

echo
echo "=== the harness itself must launch nothing ==="
# The first version of this file had no --dry-run and asserted the ALLOW cases
# by letting them start and killing the driver after 20 s. That orphaned six
# gazebo/scovox/robot_state_publisher processes, which is a worse bug than any
# it was testing for. Assert the absence directly rather than assuming it.
# A DELTA, not an absolute. The first version took one census after the cases and
# demanded zero, which is wrong in both directions: a stray sim already on the box
# is reported as "this harness started a simulation", and on a busy box the check
# fails for someone else's work. What this harness is responsible for is the
# CHANGE it caused.
#
# It also had no known-answer case -- every case runs --dry-run, so nothing could
# ever start a process and the check could not fail however broken it was. That is
# the shape §32.14 is about, in the harness whose whole purpose is to prevent it.
# The planted case below starts a process the census pattern matches and requires
# the census to see it, so a census that has stopped counting says so.
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
# run_explo_sim_rviz.sh has no --dry-run: invoking it launches gazebo for real.
# Probing it that way once burned two minutes and left processes on the box --
# the same failure the census section above exists to stop. Every guard tested
# here lives in the launcher's PRELUDE: the defaults, the validation and the
# refusals, ending at the arm-stamp check. That region runs no ROS, spawns
# nothing and writes nothing, so it is cut out of the SHIPPED file by line range
# and run on its own. It is the real text, not a transcription of it: edit a
# guard and these cases move with it.
#
# THE CUT MUST BE WRITTEN INTO sim/, not a tmpdir. The launcher computes HERE
# from BASH_SOURCE[0] and the workspace root three levels above it, so a copy
# anywhere else dies at the scenario-installed check for a reason that has
# nothing to do with the guard under test. Every BLOCK case would still see a
# non-zero exit, and this whole section would pass while testing nothing.
PRELUDE_ANCHOR='^unset _mtare_stamped _arm_core _arm_expect$'
PRELUDE_END=$(grep -n "$PRELUDE_ANCHOR" "$LAUNCHER" | head -1 | cut -d: -f1)
PROBE="$(dirname "$LAUNCHER")/.prelude_probe.$$.sh"
trap 'rm -rf "$TMP"; rm -f "$PROBE"' EXIT

cases=$((cases+1))
if [ -n "$PRELUDE_END" ]; then
  echo "  PASS  the prelude anchor is present (launcher line $PRELUDE_END)"
else
  echo "  FAIL  '$PRELUDE_ANCHOR' is not in the launcher -- the arm-stamp guard"
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

  # And the cut has to CONTAIN the guards. A range that stopped short would
  # fail every BLOCK case for the wrong reason and pass every ALLOW one.
  for _need in "FATAL: CELL_WORLD" "FATAL: TEAM_WORLD=" "FATAL: TEAM_WORLD_HZ" \
               "FATAL: RENDEZVOUS_SCHEDULE" "FATAL: PURSUIT_PREDICTOR" \
               "the arm name and the arm"; do
    cases=$((cases+1))
    if grep -qF "$_need" "$PROBE"; then
      echo "  PASS  the cut carries the guard: $_need"
    else
      echo "  FAIL  the cut does NOT carry: $_need -- the line range is wrong"
      fails=$((fails+1))
    fi
  done

  # env -i: the point of several of these guards is that an AMBIENT export must
  # not reach a cell, so the probe must not inherit one either.
  lg() {
    local want="$1" expect="$2" label="$3"; shift 3
    local out rc
    cases=$((cases+1))
    out=$(env -i PATH="$PATH" HOME="$HOME" USER="${USER:-nobody}" \
              SCENARIO=flatforest_dense_2robot_lidar.yaml "$@" \
              timeout 30 bash -c \
              "source /opt/ros/humble/setup.bash >/dev/null 2>&1; bash '$PROBE'" 2>&1)
    rc=$?
    local ok=0
    if [ "$want" = BLOCK ]; then
      [ "$rc" != 0 ] && printf '%s' "$out" | grep -qF "$expect" && ok=1
    else
      [ "$rc" = 0 ] && printf '%s' "$out" | grep -qF "$expect" && ok=1
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
  # ever fails, the launcher is refusing off-arm cells; if the second fails,
  # the mtare_hybrid arm does not exist and the campaign has no treatment.
  lg ALLOW '__PRELUDE_OK__ cw=0 tw=0 hz=1.0 ga=0 rg=silence rs=0 pp=trail rm=hybrid' \
     "shipped defaults resolve with every M-TARE knob off"
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=1 pp=trail rm=mtare_hybrid' \
     "RECONNECT_MODE=mtare_hybrid expands to the whole P1-P5 stack" \
     RECONNECT_MODE=mtare_hybrid

  # The other three cells of the doc §3.6.1 factorial. Each expands to a
  # DIFFERENT vector, and the two that differ from mtare_hybrid are the point
  # of the design: mtare_pursuit is chase without an appointment,
  # mtare_rendezvous is an appointment without a chase, mtare_off is neither.
  # Asserted one knob at a time because a token that expanded to the hybrid
  # stack under a different name would run the campaign as one arm four times
  # and every gate downstream would agree with it.
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=silence rs=0 pp=trail rm=mtare_off' \
     "RECONNECT_MODE=mtare_off expands to P1-P3 and NEITHER mechanism" \
     RECONNECT_MODE=mtare_off
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=0 pp=trail rm=mtare_pursuit' \
     "RECONNECT_MODE=mtare_pursuit expands to the chase WITHOUT an appointment" \
     RECONNECT_MODE=mtare_pursuit
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=1 pp=trail rm=mtare_rendezvous' \
     "RECONNECT_MODE=mtare_rendezvous expands to the appointment WITHOUT a chase" \
     RECONNECT_MODE=mtare_rendezvous

  # A typo in a 0/1 knob reads as OFF, and an off treatment knob is invisible:
  # the run completes, the manifest records the value it was handed, and the
  # cell is analysed as treated. Only the launcher can catch this.
  lg BLOCK "FATAL: CELL_WORLD='true'" \
     "CELL_WORLD=true is refused, not silently read as off" \
     CELL_WORLD=true
  lg BLOCK "FATAL: TEAM_WORLD='yes'" \
     "TEAM_WORLD=yes is refused, not silently read as off" \
     CELL_WORLD=1 TEAM_WORLD=yes

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
  lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=2.0 ga=0 rg=silence rs=0 pp=trail rm=hybrid' \
     "a legitimate TEAM_WORLD_HZ still passes -- the guard is not a blanket no" \
     CELL_WORLD=1 TEAM_WORLD=1 TEAM_WORLD_HZ=2

  # The node reconstitutes the arm itself, prefixing `mtare_` when
  # global_alloc_enable_ || reconnect_gate_info_ || rendezvous_schedule_enable_.
  # Every directory, index row and analysis keys off the NAME; only run_start
  # carries the stamp. Setting a knob by hand under the plain name puts a
  # treated cell in the control column.
  #
  # One case per disjunct, and that is the whole reason this block is a list
  # rather than one case: a guard that tracked only two of the three would keep
  # printing two PASSes while the third knob walked straight through it.
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

  # The P6 knob. It is the SIXTH thing that flips the node's `mtare_` prefix, so
  # it needs the same three-way coverage the fifth got: the typo, the arm-stamp
  # direction, and the preconditions.
  lg BLOCK "FATAL: PURSUIT_PREDICTOR='MDP'" \
     "a mis-cased PURSUIT_PREDICTOR is refused, not read as trail" \
     PURSUIT_PREDICTOR=MDP
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

  # The two _mdp tokens are the only way to REQUEST the predictor, and they are
  # ordinary arm tokens: the whole stack comes from the name, and the name is
  # what the node will stamp back (its arm rule gained the matching `_mdp`
  # suffix in the same commit). These four assert both halves -- the stack the
  # token expands to, and the refusal of an override that would make the
  # directory name and the run_start stamp disagree.
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
fi

# A clean launch must be SILENT on stderr. `env_val` took its default from a
# bare "$2" while two callers legitimately omit it, so under `set -u` every
# campaign launch opened with two `line 190: $2: unbound variable` lines. They
# were harmless -- the value returned was the empty string the callers wanted
# -- and that is the problem: an operator who learns the launcher always prints
# two errors has learned to skim past the third one that means something. This
# asserts the absence, not a message, because the next such regression will
# have a different line number and a different variable.
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

# The ambient-export strip on the launch line. This is the one leak the
# arm-stamp guard above CANNOT see: CELL_WORLD and TEAM_WORLD rename nothing, so
# `export CELL_WORLD=1 TEAM_WORLD=1` in the launching shell would run the census
# and the exchange in every cell of a plain hybrid-vs-off campaign, stamp the
# same arm on both sides, and score CLEAN. Read the -u list back out rather than
# trusting that it was kept in step with the knobs the launcher grew.
for _u in LINK_GATE MIDRUN_SILENCE CELL_WORLD TEAM_WORLD TEAM_WORLD_HZ \
          GLOBAL_ALLOC RECONNECT_GATE RENDEZVOUS_SCHEDULE PURSUIT_PREDICTOR; do
  cases=$((cases+1))
  if sed -n '/^  env -u LINK_GATE/,/run_explo_sim_rviz.sh"/p' "$CS" \
     | grep -qE -- "-u $_u( |\\\\|$)"; then
    echo "  PASS  the launch line strips ambient $_u"
  else
    echo "  FAIL  ambient $_u reaches every cell -- it is not in the -u list"
    fails=$((fails+1))
  fi
done
unset _u

echo
[ "$fails" = 0 ] && echo "ALL PASS ($cases known-answer cases)" \
                 || echo "$fails FAILURE(S) of $cases known-answer cases"
exit $((fails > 0))
