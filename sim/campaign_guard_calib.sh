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
[ "$fails" = 0 ] && echo "ALL PASS ($cases known-answer cases)" \
                 || echo "$fails FAILURE(S) of $cases known-answer cases"
exit $((fails > 0))
