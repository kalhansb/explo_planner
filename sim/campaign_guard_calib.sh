#!/usr/bin/env bash
# Known-answer cases for run_campaign.sh's guards.
#
# The link veto below is the one this file was written for and still the
# longest section. Two others have joined it: the launcher's own validation
# blocks, and the RESUME guard, which decides whether a directory already
# holding a finished cell is "already complete" or a different experiment
# wearing the same name (last section).
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
# Moved past the arm-stamp guard 2026-09-16, to the end of the _rzv_needed
# block that now follows it. Cutting at the arm-stamp unset would have left the
# `a mode that is nothing without the schedule` refusal outside the probe
# entirely -- so the bare-default ALLOW case below would have certified a
# default that the real launcher refuses one line later, which is the exact
# shape of a check that has stopped checking.
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

  # env -i: the point of several of these guards is that an AMBIENT export must
  # not reach a cell, so the probe must not inherit one either.
  # $SCEN, not a literal: every case here used to pin the 2-robot scenario, so
  # the whole section only ever saw N == 2 and nothing that varies with the
  # roster was under test in either direction. Callers that care set SCEN.
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
    # A sentinel is matched as a WHOLE LINE, a FATAL as a substring. The
    # distinction is not cosmetic: the sentinel's last field is the arm name, so
    # a substring match for `rm=mtare_hybrid` is also satisfied by an output
    # reading `rm=mtare_hybrid_mdp`. The palette case in the derivations section
    # below was written with a substring match and could not fail for exactly
    # this reason -- COSTAR_..._REDUCED is a prefix of COSTAR_..._REDUCED_YELLOW
    # -- and survived the mutation that was meant to kill it.
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
  # The first used to assert `every M-TARE knob off`, because RECONNECT_MODE
  # defaulted to plain `hybrid`. That default was UNRUNNABLE from the day the
  # schedule-needed refusal landed -- plain hybrid with rs=0 is refused there,
  # and with rs=1 it is refused by the arm-stamp guard -- so the default moved
  # to mtare_hybrid and this case moved with it. The two now assert the same
  # vector by two routes, which is the point: the default and the token that
  # names it must not drift apart.
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
  #
  # RECONNECT_MODE=pursuit on both, so the arm stack is EMPTY and the 0/1
  # validity check is what the case actually reaches. Under an mtare_* token
  # (which the default now is) the stack's own contradiction check fires first
  # and reports the same input as a different mistake -- still a refusal, but
  # not the one this case is calibrating.
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
  # RECONNECT_MODE=pursuit for the same reason as the two 0/1 typo cases above:
  # every mtare_* token pins PURSUIT_PREDICTOR, so under the default the arm
  # stack refuses 'MDP' as a contradiction before the spelling check sees it.
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

  # --- the roster axis (P7) -------------------------------------------------
  # Everything above this line ran at N == 2 and could not have told a harness
  # that generalised from one that merely still works for a pair. That is not a
  # hypothetical gap: the launch-time planner-count guard kept comparing against
  # a literal 2 through a whole "78/78 ALL PASS" run, because no case ever asked
  # for a third robot.
  #
  # The roster is read from the SCENARIO and is deliberately not overridable, so
  # the only way to vary it is to point at a different scenario file. The N == 2
  # case below is the control: without it, an assertion that matched a constant
  # string would pass at both sizes and prove nothing.
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
# The prelude section above stops at the arm-stamp guard, which is where the
# launcher stops being inert. Everything the P7 change actually rewired --
# peers_of / peers_csv / peers_ros_array, and the roster-position viz palette --
# lives below that line and had no coverage at all, in either direction.
#
# They are still inert: pure shell over $ROBOTS, no ROS, no processes. So they
# get their own cut, assembled from the prelude (which is what resolves $ROBOTS
# from the scenario) plus this block. The two are not adjacent, and the region
# skipped between them sets exactly one variable this block reads -- $OUTDIR --
# which the shim supplies. If that ever stops being true the probe fails loudly
# under `set -u` rather than testing a stub.
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
# This one guard cannot be executed without launching gazebo -- it counts real
# processes 8 s after a real bring-up -- so it is asserted textually. That is a
# weaker check than running it, and it is here because the ALTERNATIVE was no
# check: it compared against a literal 2 while everything around it was
# generalised, so a 3-robot scenario started the entire stack and then died on
# "expected exactly 2 explo_planner_node, found 3".
#
# A textual assertion is worth nothing unless it can fail, so it is run three
# times: on the shipped launcher, on a copy mutated back to the literal, and on
# a copy with the guard deleted. The last one matters most -- a check that
# reports PASS when its subject is absent has stopped checking.
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
#
# DERIVED, not re-listed. This loop used to enumerate nine names by hand, and a
# hand-maintained copy of a list is a check that goes stale silently: by the
# time anything looked, six knobs the guard reasons about (TREE_ATTEN,
# MAX_RANGE, CELL_SIZE_M and the three SEPARATION_*) had been added to
# run_campaign.sh and to none of them, and this section printed nine PASSes
# without noticing. So take the knob list from the script's own env_val() calls
# -- that IS the set of things the guard models -- and assert the invariant
# stated at the -u list: each one is either stripped there, or assigned
# explicitly on the same launch line. A knob that is neither is one the guard
# scores at its default while the cell runs on the ambient shell's value.
# Anchored on `env ` and not on `env -u LINK_GATE`, because the launch line
# grew a derived sweep in front of the literal strips on 2026-09-18 and the
# tighter anchor then matched nothing: sed returned an empty range, every knob
# below was "neither stripped nor assigned", and fifteen guards that were in
# fact intact reported FAIL. An extraction that can silently select nothing is
# the same defect as a gate that can silently examine nothing, so the emptiness
# of BOTH halves is now an explicit case rather than a fifteen-way symptom.
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
# The section above asserts an invariant over the knobs run_campaign.sh MODELS.
# That is roughly twenty names. run_explo_sim_rviz.sh reads about a hundred and
# ten, all as `${NAME:-default}`, and until 2026-09-18 the other ninety reached
# every cell straight from the launching shell: `env` without -i inherits, the
# strip list had never heard of them, the resume guard does not compare them,
# and for most of them run_manifest.txt does not record them either. So an
# `export RDV_DEPART_DELAY=30` left in a terminal retuned every cell of a
# multi-day matrix with nothing anywhere able to say it had.
#
# RDV_DEPART_DELAY is not a hypothetical: through generation 24 it was the
# 100 s in "after 100s after one robot disconnect, all robots go to the
# rendezvous point" — the treatment itself in two of ts4's four arms. Since
# generation 25 it is inert in the binary but still logged, so a leak now
# forges the param rows rather than the behaviour.
#
# The sweep is derived from the runner rather than listed, so these cases check
# the DERIVATION -- reproduced here from the same source, which is the only way
# to notice the scan silently matching nothing.
_keep=$(sed -n 's/^RUNNER_ENV_KEEP="\(.*\)"$/\1/p' "$CS" | head -1)
# THE SCANNER IS EXTRACTED, NOT REPRODUCED, and that distinction was found the
# hard way. The first draft of this section pasted a copy of the awk program
# here; two mutations of the launcher's real scanner -- dropping the bare
# `${X-default}` form, and letting commented-out knobs through -- then passed
# every case below, because the thing under test was a second copy that had not
# been mutated. A calibrator holding its own copy of the code it certifies is
# the same defect as the hand-maintained knob list this whole section replaced,
# and it fails the same way: silently, while printing PASS.
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

# And the tripwire's own threshold, read back rather than assumed. The case
# above establishes that a broken scan reaches zero; this one establishes that
# zero is refused. `-lt 0` can never fire and `-lt 200` would refuse every real
# campaign, and both edits look equally innocuous in a diff -- which is how a
# threshold is the quietest place for a guard to die.
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

# The sweep must be WIRED, not merely computed. A derivation that is never
# referenced on the launch line is the same nothing as no derivation, and it
# reads as a fix in every diff.
# The OUTPUT FORM, which every other case here is deliberately blind to: _scan
# splits the `-u` flags off before comparing names, so a scanner that emitted
# bare names would satisfy all of them. It would also turn the launch line into
# `env RDV_APPT_WAIT OUTDIR=... runner`, where env runs the first name as the
# command. That fails loudly at the first cell rather than silently -- but it is
# the launcher's own `<10` tripwire that makes it loud, by counting exactly the
# `-u` tokens asserted here, so this is the case that keeps that tripwire honest.
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
# The resume guard decides whether a directory that already holds a finished
# cell counts as "already complete" or as a different experiment wearing the
# same name. It had no known-answer case of any kind until this section, while
# growing from one compared key to seventeen -- and the failure it exists to
# stop is silent by construction: the campaign prints SKIP, the matrix fills
# up, and two configurations end up pooled under one tag with nothing in the
# analysis able to tell.
#
# Both directions are asserted, because each has its own way of going wrong. A
# guard that never aborts is a rubber stamp; a guard that always aborts is
# worse than none, since the operator learns to reach for a fresh --tag every
# time and the guard stops being read. The float keys make the second failure
# easy to write by accident: the two sides of the comparison reach it by
# different routes -- the manifest carries the value as the run recorded it,
# the guard the value the operator typed -- so a cell banked at "20.0" can be
# checked against a request for "20" and a string compare would abort a correct
# resume on a formatting difference alone. Cases feeding both spellings pin
# that, and the garbage-value cases below pin the other end of it: awk reads an
# unparseable string as 0, so the numeric branch has to check that what it was
# handed is a number before believing they agree.
#
# Nothing launches. The cases run without --dry-run -- the guard sits below the
# point where --dry-run exits -- so MIN_FREE_MB is set absurdly high, which
# trips the disk guard immediately AFTER the resume guard and before the cell
# is started. That also gives the "guard wrongly let it through" outcome its
# own distinguishable name (LAUNCHED) instead of a 3000 s gazebo run.
RG_ARM=mtare_hybrid
# Agrees with the reference campaign below on every key the guard reads. Each
# case overwrites, or deletes, exactly one line.
#
# run_gates_verdict IS `CLEAN`, WHICH IS A TOKEN THE HARNESS CAN ACTUALLY WRITE.
# It said `VALID` until 2026-09-18, and nothing in run_explo_sim_rviz.sh has ever
# emitted that word: the teardown writes exactly one of CLEAN, SUSPECT or
# INVALID. The reference manifest was therefore a manifest no cell could have,
# which cost nothing while the resume guard asked only "is this literally
# =INVALID", and became thirteen simultaneous failures the moment it started
# distinguishing CLEAN from everything else. A fixture that cannot be produced
# by the thing it stands in for is a fixture that will one day disagree with it
# for a reason that has nothing to do with the case being tested.
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
# The five keys above the EOF joined the guard in the C2/C3 pass and were not
# added here at the time, which put SIXTEEN of the cases below into permanent
# ABORT: every SKIP case failed on `link_gate=<absent>` long before reaching the
# key it was written to exercise. The suite exited non-zero either way, so the
# 16 reds read as one known breakage rather than as sixteen assertions that had
# stopped asserting anything -- the cases pinning the FLOAT-formatting branch
# (20 vs 20.0, padded values, the nan/awk trap) were the expensive ones to lose,
# because that branch is the one that aborts a CORRECT resume.
#
# Values are the reference campaign's, i.e. what a no---env `--arms mtare_hybrid
# --seeds 1` invocation predicts: LINK_GATE defaults to 1 in the launcher and
# run_campaign mirrors that when --env carries no LINK_GATE, --comms defaults to
# 1 so the effective veto is live too, MIDRUN_SILENCE and TEAM_WORLD_HZ mirror
# the launcher's 90 and 1.0, and done_seek is off (the campaign spells it 0, the
# runner writes the ROS bool).
#
# n_robots is deliberately NOT here. Its check is a consistency test across the
# banked cells of a tag, and it skips a manifest that does not carry the key --
# so an absent one is a case in its own right rather than a hole, and every rg()
# case banks exactly one cell anyway.
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
    # BANKED BUT NOT CERTIFIED. Its own outcome name, because it is neither of
    # the two the guard used to have: the cell IS skipped (so it is not ABORT
    # and not LAUNCHED) but it is skipped with a verdict gate_g8 will hard-fail,
    # and folding it into SKIP would make the loud path and the silent path
    # indistinguishable here — which is the exact defect being calibrated.
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
# The radio regime. The first of these is the change of 2026-09-03 itself: a
# cell banked under the 11.98 dB trunks, resumed by a campaign running the
# 70 dB ones. Before this key was compared it scored SKIP.
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
# The allocator peer-position TTL, set from the "_ttl<N>" suffix. Same sentinel
# shape as the radius above, with one difference that matters: 0 is a REAL level
# here (unbounded, the control arm) rather than a spelling of "unset", so "0.0"
# and "none" are the same behaviour and must still be different cells.
#
# The two SKIP cases below are also the only test of the SUFFIX PARSER itself,
# and they test both halves of it at once. A parser that failed to strip "_ttl120"
# would leave cell_pos_ttl empty (want "none" against a manifest saying 120.0 ->
# ABORT, not SKIP) AND leave cell_mode as "mtare_hybrid_ttl120", which matches no
# branch of the arm-stack case, so cell_world would fall back to 0 against a
# manifest saying 1 and abort there instead. Either way the SKIP does not happen.
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
# Numeric, not string: a cell can bank "20.0" against a campaign requesting
# "20" -- the two spellings travel by different routes, the manifest taking the
# value the node resolved and the guard the value the operator typed -- so a
# string compare would abort a correct resume. This is the case that would catch
# that regression, and the 25.0 one below is what stops the fix from
# degenerating into "any radius agrees".
rg SKIP  "separation_radius_m=20.0 matches a requested 20"    "$RG_ARM" separation_radius_m 20.0
rg ABORT "a cell banked at separation_radius_m=25.0 differs"  "$RG_ARM" separation_radius_m 25.0
rg ABORT "separation_radius_m absent cannot be shown to agree" "$RG_ARM" separation_radius_m "<none>"
rg SKIP  "separation_max_age_sec=10.0 matches a requested 10" "$RG_ARM" separation_max_age_sec 10.0
rg ABORT "a cell banked at separation_max_age_sec=3.0 differs" "$RG_ARM" separation_max_age_sec 3.0
rg ABORT "separation_max_age_sec absent cannot be shown to agree" \
                                                              "$RG_ARM" separation_max_age_sec "<none>"
# A manifest value that is not a NUMBER must abort, and separation_weight is the
# key where getting this wrong is invisible: awk reads any unparseable string as
# 0, the requested weight is 0 in every campaign run so far, so before the
# numeric branch was taught to check its inputs each of these compared EQUAL and
# banked a cell whose weight could no longer be determined. "0.0" is the control
# alongside them -- a real number that really does agree -- so a fix that
# degenerated into "abort on everything" would not pass this block either.
rg ABORT "separation_weight=off is not a number"              "$RG_ARM" separation_weight off
rg ABORT "separation_weight=unset is not a number"            "$RG_ARM" separation_weight unset
rg ABORT "separation_weight=0x0 is not a number"              "$RG_ARM" separation_weight 0x0
rg ABORT "separation_weight= (truncated line) is not a number" "$RG_ARM" separation_weight ""
rg SKIP  "separation_weight=0.0 IS a number and agrees with 0" "$RG_ARM" separation_weight 0.0
rg ABORT "separation_radius_m=default is not a number"        "$RG_ARM" separation_radius_m default
# "nan" gets its own case at a key whose requested value is NOT 0, because it
# fails differently from the strings above. This awk parses it as a real NaN
# and then reports nan == <anything> as TRUE, so under a bare `a + 0 == b + 0`
# a single corrupted value agrees with every key at every level -- 25.0 as
# readily as 0. The "off"/"unset" cases cannot catch that: they only compare
# equal where the request happens to be 0.
rg ABORT "separation_max_age_sec=nan is not a number"         "$RG_ARM" separation_max_age_sec nan
rg ABORT "separation_radius_m=nan agrees with 20 under a bare +0" \
                                                              "$RG_ARM" separation_radius_m nan
# The other direction, and the reason the numeric branch trims before matching:
# the regex is STRICTER than `+ 0` was, so a value the old compare accepted
# must not start aborting a correct resume. Whitespace is the realistic way to
# acquire one, and this is the case that would fail if the trim were dropped.
rg SKIP  "separation_radius_m= 20.0 (padded) still resumes"   "$RG_ARM" separation_radius_m " 20.0 "

# --- the verdict itself, which the resume guard used to read one bit of ------
# The guard's test was `grep -q '^run_gates_verdict=INVALID'`, so of the four
# states a banked cell can be in it distinguished exactly one. SUSPECT and a
# missing key both landed in the `else` and printed `SKIP (already complete)` --
# the same line a certified cell gets -- and gate_g8, which is the only thing
# that reads the verdict, is run by hand after the campaign. A bank full of
# uncertifiable cells therefore announced itself for the first time at analysis
# time, with every hour of sim already spent.
#
# These cases pin the three directions separately, because "it still skips" and
# "it says why" are different properties and only one of them was broken.
rg SKIP-DIRTY "a banked SUSPECT cell still skips, but says so"  "$RG_ARM" run_gates_verdict SUSPECT
rg SKIP-DIRTY "a banked cell with no verdict at all says so"    "$RG_ARM" run_gates_verdict "<none>"
rg SKIP-DIRTY "a verdict token nothing emits is not read as CLEAN" "$RG_ARM" run_gates_verdict VALID
# LAUNCHED, not SKIP-DIRTY: INVALID keeps its old behaviour of falling through
# to a re-run (the disk guard then stops it, which is what LAUNCHED names here).
# The new branch must not have swallowed the one verdict that was already acted
# on — an INVALID cell that started merely being reported instead of redone
# would be a silent loss of the only automatic remedy this driver has.
rg LAUNCHED   "INVALID still redoes the cell, not just reports it" "$RG_ARM" run_gates_verdict INVALID

# REDO_SUSPECT=1 IS THE OPT-IN, AND IT HAS TO BE TESTED IN BOTH DIRECTIONS.
# The env var is the operator's way of saying "re-roll the uncertified cells",
# and a knob that silently does nothing is worse than no knob: it converts a
# deliberate decision into a no-op that looks like it was honoured. Run inline
# rather than through rg(), which has no env hook.
#
# LAUNCHED is the wanted outcome — the cell is passed through to be re-run and
# stopped immediately afterwards by the absurd MIN_FREE_MB, which is the same
# trick every case above uses to avoid starting a 3000 s gazebo run.
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
# Same hazard as the LINK_GATE readback above, and the reason that one exists is
# on display here: TREE_ATTEN's shipped default MOVED, from 11.98 to 70.0. The
# resume guard predicts what a cell's manifest will say, so a stale copy of a
# default does not fail loudly -- it aborts every resume of a campaign that is
# running exactly as intended, or, in the other direction, skips a cell from
# the wrong regime. Read all three literals back out of the launcher.
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
