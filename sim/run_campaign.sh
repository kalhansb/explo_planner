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
# Usage (--root, --duration and --scenario are REQUIRED; see their declarations
# for why the last two lost their defaults):
#   ./run_campaign.sh --root DIR --duration 3000 \
#                     --scenario flatforest_dense_2robot_lidar.yaml \
#                     --cells "off:1,off:2,pursuit:1" \
#                     [--tx 30.0] [--record 0] [--tag phase3]
#   ./run_campaign.sh --root DIR --duration 3000 --scenario ... \
#                     --arms "off,hybrid" --seeds "1,2,3"   # cross product
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROOT=""; CELLS=""; ARMS=""; SEEDS=""; TAG="run"
# DURATION has NO default. It is the censoring horizon, so it defines the
# primary endpoint every bit as much as DONE_UNKNOWN does: a cell that ran to
# 3600 s and one that ran to 3000 s do not have comparable completion times or
# comparable censoring rates, and nothing downstream can tell them apart from
# the numbers alone. The old default was 3600 while every campaign since
# 2026-08-27 has passed 3000, so the default's only reachable effect was to
# silently relabel a campaign that forgot the flag. Required is better than
# right: there is no value here that is correct for an unknown experiment.
DURATION=""
# Transmit power. Fixed hardware, identical on both robots, never an
# experimental variable (see the --comms note below) — so unlike DURATION this
# one genuinely has a correct default and keeps it.
TX="30.0"
# Bag recording off by default. A lean-record cell is ~340 MB, a 60-cell matrix
# is ~20 GB, and the analyses this project actually runs read the event JSONL
# and the planner CSV, never the bags. The old default of 2 meant forgetting
# --record spent a fifth of the disk on data nothing reads.
REC="0"
EXPECT_OUT="1"
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
#
# No default, for the reason the paragraph above gives: the last default named
# a world no campaign has used since the dense forest became the standard, so a
# forgotten --scenario produced a sparser map, a different link budget and a
# different coverage floor under the campaign's own tag. Nothing in the cell
# would have said so except this key in the manifest.
SCENARIO=""
# Mission return (see run_explo_sim_rviz.sh). An explicit first-class flag, NOT
# an --env passenger, because it applies to EVERY cell identically in BOTH arms
# — it is part of the mission definition, not an arm. Default 1: from 2026-08-27
# campaigns measure mission end time as the primary endpoint, and their
# exploration-finish numbers are a NEW endpoint never pooled with banked runs.
# Pass --mission-return 0 only to reproduce the legacy park-in-place design.
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
    # Run every validation, print the cell list, launch nothing. Added because
    # the refusals above could not be calibrated any other way: checking that a
    # legitimate campaign is NOT refused meant letting it start, and a harness
    # that then killed the driver left the cell's gazebo, scovox and
    # robot_state_publisher processes orphaned -- six of them, on the first
    # attempt. A guard whose happy path cannot be tested without side effects is
    # a guard whose happy path does not get tested.
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
# BOTH helpers below read EXTRA_ENV one KEY=VALUE token at a time, exactly.
# Substring matching on the whole string is what the first draft of the guard
# did, and it is wrong in both directions at once: `GATE_MIDRUN_SILENCE=240` (a
# variable for the gate script, not the launcher) contains "MIDRUN_SILENCE=" and
# silently DISARMED the guard, while `MY_LINK_GATE=0` contains "LINK_GATE=0" and
# would have armed it on a campaign that never touched the veto. Splitting on
# whitespace and comparing the key up to the first '=' removes both.
env_has() {
  for _tok in $EXTRA_ENV; do
    case "$_tok" in "$1"=*) return 0;; esac
  done
  return 1
}

# The value --env carries for a key, or $2 if it does not carry one. (A second,
# defaultless copy of this function used to sit above env_has and was shadowed
# by this one at parse time -- dead code that read as if it were the live
# definition. Deleted; its rationale is folded into the comment above env_has.)
# Used to
# predict what a cell's manifest WILL say, so the resume guard can compare a
# banked cell against this campaign's configuration rather than assuming they
# agree. Mirrors run_explo_sim_rviz.sh's own defaults, and the mtare_hybrid arm
# token overrides the answer per cell below -- so this is the un-treated
# baseline, not the final word.
# The default is optional, and omitting it is a real call shape, not a mistake:
# the LINK_GATE and MIDRUN_SILENCE readers below want "the value the operator
# set, or nothing", because each decides for itself what absence means (one
# re-derives the launcher's default a few lines down, the other only reads the
# result inside an `env_has` branch). Under `set -u` a bare "$2" made those two
# calls print `$2: unbound variable` on stderr on every campaign launch while
# still returning the empty string the caller wanted — noise that reads like a
# broken guard and would bury a real one.
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
# COMMS in --env would beat --comms: the per-cell `env COMMS="$COMMS_ON" ...`
# assignment below is expanded BEFORE $EXTRA_ENV, and last assignment wins. So
# `--comms 1 --env COMMS=0` runs an ideal-comms campaign whose manifest, index
# and directory names all say the emulator was up. Same shape of failure as
# MISSION_RETURN above; refuse it the same way.
if env_has COMMS; then
  echo "FATAL: set the emulator with --comms 0|1, not --env COMMS=... --" >&2
  echo "       --env is expanded last, so the passenger wins over the flag and" >&2
  echo "       every record of the campaign would name the wrong arm." >&2
  exit 2
fi
# The last two per-cell assignments on the env line at the bottom of this file,
# and the two worst to lose. RECONNECT_MODE is the treatment itself: an --env
# passenger sets every cell to one arm while the OUTDIR names, the index and the
# gate's arm parser all still read hybrid-vs-off, so the campaign looks like a
# controlled comparison and is a single arm run twice. SEED is the world: one
# value for all cells turns 30 replicates into 30 repeats of one map, and the
# sim is nondeterministic run-to-run ([[sim-run-to-run-nondeterminism]]), so the
# spread would look like real between-cell variation.
#
# GLOBAL_ALLOC, RECONNECT_GATE and RENDEZVOUS_SCHEDULE join them because they
# are arm-DEFINING: the node's `arm` string is "mtare_" + the mode whenever any
# of the three is live, so an
# --env passenger turning one on renames every cell's arm from inside the
# binary while the directory, the index and the manifest's
# reconnect_mode_requested all still say hybrid and off. analyze_runs.py,
# manoeuvre_events.py and progress_signature.py all key the arm off the
# manifest, so a fully-treated P3 campaign would be summarised and
# permutation-tested as plain hybrid-vs-off — and it would not even be an A/B,
# since --env applies to the control arm too.
#
# The arm token in run_explo_sim_rviz.sh refuses the contradiction in one
# direction only (mtare_hybrid with the allocator off). This is the other
# direction, and it belongs here because --env is this script's channel.
#
# CELL_WORLD and TEAM_WORLD deliberately stay ALLOWED: P1 and P2 change no
# decision and do not rename the arm, so they are legitimately campaign-wide
# settings rather than treatments.
#
# COORD_CLAIM_R is here for the same reason and is the quietest of the set: it
# is assigned per cell from the _r<N> arm suffix, so `--env COORD_CLAIM_R=40`
# would run every cell at 40 m while half of them sat in directories named
# _r10. Gate check 3i reads the manifest and would catch it afterwards, which
# is a campaign too late.
# ALLOC_POS_TTL joins for exactly the COORD_CLAIM_R reason, and it is the one
# where getting it wrong is hardest to see afterwards: the TTL's control level
# is 0 = unbounded = the pre-TTL planner, so `--env ALLOC_POS_TTL=120` would put
# the treatment in EVERY cell, including the ones sitting in directories named
# _ttl0, and the campaign would be a treated-vs-treated comparison that returns
# a clean null. Set it with the _ttl<N> suffix, which is per cell.
for _blocked in RECONNECT_MODE SEED GLOBAL_ALLOC RECONNECT_GATE \
                RENDEZVOUS_SCHEDULE PURSUIT_PREDICTOR COORD_CLAIM_R \
                ALLOC_POS_TTL; do
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

# Unlike DONE_SEEK and MISSION_RETURN, LINK_GATE in --env is LEGITIMATE: it is
# how you reproduce a record-age campaign, and it applies to every cell on
# purpose because the arms must differ in RECONNECT_MODE alone. What is not
# legitimate is running it against the generation-9 threshold.
# reconnect_midrun_silence_sec defaults to 90, which is below the ~180 s
# heartbeat-suppression tail, and that is only safe because the veto can tell a
# quiet teammate from an absent one: the record-age clock ages whenever a peer is
# not SENDING, so a healthy partner two metres away in a long PLAN loop reads as
# missing at 90 s and the campaign spends its treatment arm chasing robots that
# were never lost.
#
# --comms 0 is the same hazard by a different route. It is a documented arm (the
# ideal-comms control), but with no emulator there is no link topic, so the veto
# cannot run there either -- and heartbeat suppression is a property of the
# beacon, not of the radio, so it does not go away just because the radio is
# perfect.
#
# THE ESCAPE HATCH IS NOT "MENTION MIDRUN_SILENCE". The first draft disarmed on
# the mere PRESENCE of the name, so `--env MIDRUN_SILENCE=5` -- which is worse
# than the default in exactly the way the guard exists to prevent -- sailed
# through, and so did a campaign that only ever mentioned GATE_MIDRUN_SILENCE.
# The value is what matters, and it has to clear the suppression tail.
#
# AND THE ADVICE MATTERS TOO. Telling a --comms 0 operator to raise
# MIDRUN_SILENCE would make the ideal-comms control differ from the treated arm
# in TWO variables -- the radio and the trigger clock -- which is precisely the
# compounded design this project has repeatedly been burned by
# [[no-compound-experiments]]. Under --comms 0 the right answer is a different
# one, so the guard says a different thing.
#
# IT RUNS HERE, BELOW THE CELL LIST, AND THAT PLACEMENT IS LOAD-BEARING. It used
# to run above every other validation, so it read $ARMS and $CELLS raw and had
# to invent an answer when neither was given -- and a campaign launched with no
# arm selector at all, or with a bad --mission-return, was refused by THIS guard
# with THIS message, which describes a hazard that campaign does not have. A
# guard that answers a question nobody asked teaches the operator to route
# around it. Below the construction, $CELLS is populated, validated and always
# in "arm:seed" form, so the treated test is a plain read of the real cell list.
MIDRUN_SILENCE_FLOOR=200
_link_gate_req="$(env_val LINK_GATE)"
# The launcher's own default, duplicated here because the guard has to decide
# before anything is launched. run_explo_sim_rviz.sh spells it
# `LINK_GATE="${LINK_GATE:-1}"`. If that ever changes, this guard would reason
# about a run that does not happen -- so campaign_guard_calib.sh reads the
# literal back out of the launcher and fails if the two disagree, which is the
# only thing keeping the duplication honest.
env_has LINK_GATE || _link_gate_req=1
# Mirrors run_explo_sim_rviz.sh: the veto is live only when the request is
# exactly "1" AND the emulator is up. Every other value -- "0", "false", "2",
# empty -- is off there, so it must be off here.
_veto_live=0
[ "$_link_gate_req" = "1" ] && [ "$COMMS_ON" = "1" ] && _veto_live=1
# Only the TREATED arms can chase a peer, so an off-only campaign is not exposed
# to this at all and must not be blocked by it. Naming "off" rather than
# "hybrid" keeps a future arm inside the guard by default.
#
# The `mtare_` prefix is stripped before that comparison, because what decides
# whether this hazard exists is reconnect_enabled, and the runner sets that
# from the arm's SUFFIX alone: mtare_off is reconnect_enabled=false exactly
# like off, and dispatches nothing to chase with. Leaving the prefix on would
# have classed the P7 factorial's own control as treated and blocked a --comms
# 0 run of it for a trigger that arm never arms.
#
# $CELLS is the single source: it is what actually runs, whether it came from
# --cells or from crossing --arms with --seeds, so there is no second spelling
# for the guard to go silent on. Cells are "arm:seed", and the trailing _seek is
# a runtime switch stripped further down (cell_mode/cell_seek), not an arm -- so
# "off_seek" is an untreated control and must not be classed as treated on the
# strength of a suffix.
_treated=0
for _a in $(printf '%s' "$CELLS" | tr ',' ' '); do
  _a="${_a%%:*}"
  _a="${_a%_seek}"
  # ... and the same for the allocator peer-position TTL suffix. Position in this
  # chain is not free: suffixes come off in reverse order of how they were put
  # on, so _ttl<N> must be stripped BEFORE _r<N> and AFTER _seek. The full
  # append order is <mode>_r<N>_ttl<N>_seek, and "mtare_off_r40_ttl120" does not
  # match the _r patterns below until the _ttl part is gone.
  case "$_a" in
    *_ttl[0-9]|*_ttl[0-9][0-9]|*_ttl[0-9][0-9][0-9]|*_ttl[0-9][0-9][0-9][0-9])
      _a="${_a%_ttl*}";;
  esac
  # ... and the same for the cr2 claim-radius suffix, for the same reason: it is
  # a runtime switch, not an arm, so "mtare_off_r40" is still an untreated cell
  # and must not be classed as treated on the strength of a suffix.
  # THREE DIGITS, not two (2026-09-18): gate_g8.py's DESIGN_SUFFIXES has always
  # read `_r([0-9]{1,3})`, so the gate and the launcher disagreed about which
  # cell names are well-formed. Widening HERE rather than narrowing there is
  # deliberate — narrowing the gate to two digits would make its greedy `(.*)`
  # match `mtare_hybrid_r1` out of `mtare_hybrid_r100` and report a 0 m claim
  # radius, trading a loud disagreement for a silent misparse.
  case "$_a" in *_r[0-9]|*_r[0-9][0-9]|*_r[0-9][0-9][0-9]) _a="${_a%_r*}";; esac
  _a="${_a#mtare_}"
  [ "$_a" = "off" ] || _treated=1
done
_sil="$(env_val MIDRUN_SILENCE)"
_sil_ok=0
if env_has MIDRUN_SILENCE; then
  case "$_sil" in
    ''|*[!0-9.]*|*.*.*)
      echo "FATAL: --env MIDRUN_SILENCE='$_sil' is not a number" >&2; exit 2;;
  esac
  # Integer compare on the whole-second part; the clock is never set in
  # fractions and awk is not guaranteed to be on the path this early.
  [ "${_sil%%.*}" -ge "$MIDRUN_SILENCE_FLOOR" ] 2>/dev/null && _sil_ok=1
  # 0 DISABLES THE MID-RUN TRIGGER ENTIRELY -- it is the documented switch for
  # reproducing pre-2026-08-17 behaviour bit-for-bit, and the node's own trigger
  # requires reconnect_midrun_silence_sec_ > 0.0. Refusing it made the guard
  # block the one configuration in which the hazard it names cannot occur.
  case "${_sil%%.*}" in 0) _sil_ok=1;; esac
fi
if [ "$_treated" = "1" ] && [ "$_veto_live" = "0" ] && [ "$_sil_ok" = "0" ]; then
  echo "FATAL: this campaign would run the mid-run trigger with NO link veto" >&2
  echo "       (LINK_GATE=$_link_gate_req, --comms $COMMS_ON) while" >&2
  echo "       reconnect_midrun_silence_sec is at ${_sil:-the generation-9 default of 90}s," >&2
  echo "       below the ~180 s heartbeat-suppression tail. Without the veto the" >&2
  echo "       trigger cannot tell a silent teammate from an absent one and the" >&2
  echo "       treated arm will chase peers that are in range and fine." >&2
  if [ "$COMMS_ON" = "0" ]; then
    echo "       Under --comms 0 do NOT just raise MIDRUN_SILENCE: that makes the" >&2
    echo "       ideal-comms control differ from the treated arm in two variables" >&2
    echo "       at once. Either run --comms 0 with --arms off (the control needs" >&2
    echo "       no reconnect trigger), or run the whole matrix at --comms 1." >&2
  else
    echo "       Add --env MIDRUN_SILENCE=$MIDRUN_SILENCE_FLOOR or more to" >&2
    echo "       reproduce a record-age campaign, or drop LINK_GATE=0 to keep" >&2
    echo "       the veto." >&2
  fi
  exit 2
fi
# C3 (2026-09-14): the resume guard below compares these, so they have to
# outlive the guard that derived them. LINK_GATE_REQ / LINK_GATE_EFFECTIVE_REQ
# are exactly _link_gate_req / _veto_live -- the request, and the request AND an
# emulator -- and re-deriving them at the comparison site would be a second copy
# of the launcher's rule to keep in step. MIDRUN_SILENCE_REQ and
# TEAM_WORLD_HZ_REQ mirror run_explo_sim_rviz.sh's own defaults (90 and 1.0) the
# same way _link_gate_req mirrors LINK_GATE's, and are compared numerically
# because the launcher flt()s both.
LINK_GATE_REQ="$_link_gate_req"
LINK_GATE_EFFECTIVE_REQ="$_veto_live"
MIDRUN_SILENCE_REQ="$(env_val MIDRUN_SILENCE 90)"
TEAM_WORLD_HZ_REQ="$(env_val TEAM_WORLD_HZ 1.0)"
unset _link_gate_req _veto_live _treated _sil _sil_ok _a

# Every guard here scans only $EXTRA_ENV, and the launch line strips these from
# the inherited environment so that assumption holds. Say so when the caller has
# one exported, rather than ignoring it in silence: whoever typed
# `export LINK_GATE=0` meant something by it, and the useful answer is which
# channel actually reaches the cells.
#
# The M-TARE knobs are here for a sharper reason than tidiness, and it is the
# one failure the arm-stamp guard cannot see. The node prefixes the arm with
# `mtare_` on `global_alloc_enable_ || reconnect_gate_info_ ||
# rendezvous_schedule_enable_ || pursuit_predictor_mdp_` -- so an ambient
# GLOBAL_ALLOC, RECONNECT_GATE, RENDEZVOUS_SCHEDULE or PURSUIT_PREDICTOR at
# least renames the arm and trips check 3e.
# CELL_WORLD and TEAM_WORLD rename NOTHING. `export CELL_WORLD=1 TEAM_WORLD=1`
# left over from a P2 debugging session would run the census and the 1 Hz
# exchange in every cell of a plain hybrid-vs-off campaign, stamp the same arm
# on both sides, pass 3e, pass check 21, and score CLEAN -- delivering sixty
# cells of a P2-treated binary to a permutation test that believes it is
# comparing the shipped default. Stripping them makes the guards' model true by
# construction. --env is expanded after these flags and env applies assignments
# after unsets, so `--env CELL_WORLD=1` still works and is still the one channel
# the guards read.
#
# C2 (2026-09-14) adds the run-control knobs on the second line. They are not
# guard inputs, they are ENDPOINT inputs: STOP_ON_DONE decides whether a cell
# stops when the robots finish or grinds to the duration cap, DONE_GRACE_S is
# the sim-second drain counted into every run_end_t_sim, and the rest are the
# abort/watchdog thresholds that decide whether a slow cell is killed or banked
# -- i.e. a selection rule on the sample. An `export DONE_GRACE_S=0` left in a
# shell would shift every cell of a campaign with nothing anywhere to show it.
# They are now recorded in run_manifest.txt as well, so the strip list and the
# record agree.
#
# EIGHTEEN OF THE NINETEEN DO. The exception is MAP_AGREE_MAX_PCT, and the
# sentence above read as though it covered the whole list until 2026-09-18.
# That name is not recorded because it is not READ: grep the tree and it occurs
# exactly twice, both of them here (this list and the `-u` on the launch line),
# and nowhere in run_explo_sim_rviz.sh or map_agreement.py. It is the vestige of
# the era when map agreement was a PASS/FAIL gate with a threshold; the gate is
# report-only now (see the "THAT THEORY DID NOT SURVIVE" note in
# run_explo_sim_rviz.sh) and the threshold went with it.
#
# Kept in both lists anyway, deliberately. Stripping a name nothing reads costs
# nothing, and the failure directions are not symmetric: leaving it means one
# spurious NOTE if someone has it exported, while dropping it means that if the
# gate is ever re-armed on this name it silently inherits whatever the launching
# shell happened to hold. What is NOT kept is the claim that it is recorded.
# Nothing records it, and an unrecorded knob that no longer exists is fine
# precisely because it cannot affect a cell.
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

# THE LIST ABOVE IS THE KNOBS *THIS SCRIPT* REASONS ABOUT. IT IS NOT THE KNOBS
# THAT CHANGE A CELL, and the gap between the two is about seventy names.
#
# run_explo_sim_rviz.sh reads its whole configuration as `${NAME:-default}` --
# RDV_DEPART_DELAY, RDV_SETTLE, RDV_APPT_WAIT, RDV_MAX_WAIT, RDV_LATCHED_HOLD,
# START_HOLD, PROX_HOLD_M, VOXEL_RES, MAX_STEPS, ROI_HALF, UTIL_GAMMA, every
# PURSUIT_*, every RECONNECT_*, every RETURN_* -- and this script had never
# heard of any of them. `env` without -i inherits the launching shell, so a
# single leftover `export RDV_DEPART_DELAY=30` retunes EVERY cell of a
# multi-day matrix, and for most of these names nothing downstream can say so:
# they do not rename the arm, they are not compared by the resume guard, and
# the ones that do reach run_manifest.txt are read by no analysis script.
#
# Two of ts4's four arms ARE the rendezvous. RDV_APPT_WAIT=0 is the unbounded
# appointment wait that makes the meeting exact, and an ambient export of it
# does not corrupt the campaign loudly; it silently replaces the treatment
# with a different one. RDV_DEPART_DELAY has been inert in the binary since
# generation 25, but it still reaches the manifest and the logged param rows,
# so a leak of it forges a provenance difference between cells that ran
# identical treatments — quieter than retuning, still a corruption.
#
# DERIVED, NOT LISTED, for the reason written at the -u list itself: the last
# hand-kept copy of a knob list in this file was three names out of date before
# anything looked at it, and the runner grows knobs far faster than this script
# does. So the names are read out of the runner, and the only thing maintained
# by hand is the short KEEP list below of things deliberately inherited.
#
# Over-stripping is safe by construction and that is what makes this tractable:
# `env` applies -u before assignments, so a name that is also assigned on the
# launch line is unaffected, and a name that is not falls back to the runner's
# own documented default -- which is precisely the value every guard here
# already models. `--env NAME=...` still wins, and is still the one channel the
# guards read.
#
# KEEP is not "knobs we like". It is names whose ambient value is INFRASTRUCTURE
# rather than configuration -- where stripping would not neutralise a treatment,
# it would move the run onto different plumbing:
#   ROS_DOMAIN_ID   the DDS domain. Forcing it back to the runner's 42 would put
#                   a cell deliberately isolated onto another domain straight
#                   back on top of whatever is already running there.
#   DISPLAY         where rendering goes; the runner falls back to :1.
#   IGN_PARTITION   the Gazebo transport namespace. Recorded in
#                   run_manifest.txt, and stripping it moves discovery out from
#                   under a host that needs one.
#   XDG_*_SNAP_ORIG the snap-escape probe the runner uses to recover the real
#                   system paths -- deleting these defeats the escape.
#
# Everything else goes, including the runner's own internals (NFAIL, NUNRUN,
# RUN_END_REASON, DONE_DRAIN_COMPLETE). Those are not knobs, and stripping them
# is not neutral either -- it is protective. Each is assigned on some paths and
# read with `${NAME:-...}` on all of them, so an ambient
# `export RUN_END_REASON=all_done` would be written into run_manifest.txt by any
# path that never reached the assignment, and the resume guard reads that key.
#
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
# A derivation that silently returns nothing is a strip list that silently
# reverts to the hand-written one -- the exact failure `checks-that-stopped-
# checking` is about, arriving through the mechanism that was supposed to end
# it. The runner reads seventy-odd of these; under ten means the scan broke or
# the file moved, not that the knobs went away.
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

# What this campaign's cells will record for three knobs the resume guard below
# compares. Campaign-wide, unlike the M-TARE stack, because no arm token sets
# them: they arrive through --env or not at all.
#
# The defaults are run_explo_sim_rviz.sh's, duplicated here for the same reason
# LINK_GATE's is duplicated above and kept honest the same way -- campaign_guard
# _calib.sh reads all three literals back out of the launcher and fails if they
# have drifted. That is not hypothetical maintenance: TREE_ATTEN's default moved
# from 11.98 to 70.0 on 2026-09-03, and a guard still assuming 11.98 would abort
# every resume of a campaign that is running exactly as intended.
TREE_ATTEN_REQ=$(env_val TREE_ATTEN 70.0)
MAX_RANGE_REQ=$(env_val MAX_RANGE 30.0)
CELL_SIZE_REQ=$(env_val CELL_SIZE_M 10.0)
# The separation term's three knobs, same shape of fact and same defaults rule.
# All three are guarded and not only the weight: the radius and the freshness
# bound are what sep_peer_dist_m and sep_eligible_peers are MEASURED on, in
# every arm including the ones running at weight 0, so a resume that changed
# either would bank half a campaign's counterfactual on one bound and half on
# another -- with both halves recording their own value in their own manifest
# and nothing comparing them. That is the identical defect the radio pair was
# added here for.
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

  # Arm-name suffix "_ttl<N>" = same RECONNECT_MODE, the allocator's peer-position
  # TTL forced to N seconds. Third instance of the _seek/_r<N> mechanism and the
  # same reason: alloc_peer_pos_max_age_sec is a node parameter, --env sets it
  # campaign-wide, and the whole design of this treatment is that ONE binary runs
  # both levels as interleaved arms of one invocation.
  #
  # Stripped BEFORE _r<N> below and AFTER _seek above, because suffixes come off
  # in reverse order of how they went on and the append order is
  # <mode>_r<N>_ttl<N>_seek: "mtare_off_r40_ttl120" does not match the _r
  # patterns until the _ttl part is gone, and "mtare_off_ttl120_seek" does not
  # match the _ttl patterns until _seek is gone. Moving this block either way
  # breaks one of the two combinations — loudly, which is the one piece of luck
  # here: an unstripped suffix is still attached to cell_mode when it is handed
  # over as RECONNECT_MODE, and the runner whitelists that against the known
  # modes and refuses. So a mis-ordered strip fails the cell rather than running
  # the unsuffixed arm under the suffixed name. Four digits, so a TTL up
  # to 9999 s is expressible -- longer than any cell runs, which is the point,
  # since a TTL above the run length is a second spelling of "off".
  #
  # 0 IS LEGAL HERE, unlike _r0. The claim radius reads 0 as AUTO and resolves it
  # back to fov_max_range, so _r0 would name a cell for a radius it did not run;
  # the TTL reads 0 as unbounded, which is a real and wanted level -- it is the
  # control arm. So "_ttl0" is written out explicitly rather than left to the
  # unsuffixed default, for the reason the _r block gives: both levels then travel
  # the identical -p code path, and the passthrough itself is not confounded with
  # the arm.
  cell_pos_ttl=""
  case "$cell_mode" in
    *_ttl[0-9]|*_ttl[0-9][0-9]|*_ttl[0-9][0-9][0-9]|*_ttl[0-9][0-9][0-9][0-9])
      cell_pos_ttl="${cell_mode##*_ttl}.0"; cell_mode="${cell_mode%_ttl*}";;
  esac

  # Arm-name suffix "_r<N>" = same RECONNECT_MODE, MinPos claim radius forced to
  # N metres. Same mechanism and same reason as _seek above: the claim radius is
  # a yaml parameter, so --env can only set it campaign-wide, and campaign cr2
  # needs 10 m and 40 m interleaved inside ONE invocation. Stripped here, so the
  # runner is handed a clean token and every arm-token guard it owns (the
  # RECONNECT_MODE vocabulary, _arm_stack, the arm-name stamp cross-check) sees
  # exactly what it saw before this existed. The suffix survives only in the
  # OUTDIR name and the campaign index, which is where the analysis reads it.
  #
  # Empty = pass nothing, which leaves the node on the yaml value (0 = auto =
  # fov_max_range = 10 m). An UNSUFFIXED arm is therefore unchanged by this
  # block, and _r10 is written out explicitly rather than left to the default so
  # that both cr2 levels travel through the identical -p code path -- otherwise
  # the passthrough itself is confounded with the arm.
  #
  # Three digits, matching the strip in _arm_stack above and gate_g8.py's
  # `_r([0-9]{1,3})`. All three have to accept the same set of names or a cell
  # is stamped one way and read another: before this, `_r100` fell past every
  # arm of this case, so cell_claim_r stayed empty (the node kept the yaml
  # default) while the OUTDIR name still said r100 and the gate still parsed
  # 100 out of it. Nothing would have failed; the radius would simply not have
  # been applied, under a directory named for it.
  cell_claim_r=""
  case "$cell_mode" in
    *_r[0-9]|*_r[0-9][0-9]|*_r[0-9][0-9][0-9])
      cell_claim_r="${cell_mode##*_r}.0"; cell_mode="${cell_mode%_r*}";;
  esac

  # What this cell's manifest WILL say for done_seek_enabled. The runner spells
  # the flag as a ROS bool ("true"/"false") while the campaign carries it as
  # 0/1, and the resume guard compares manifest text, so the translation has to
  # happen on this side. C3: done-seek changes the endpoint -- it keeps a robot
  # exploring past its own done criterion -- so a tag with _seek cells banked
  # beside plain ones is two experiments, and until now nothing said so.
  if [ "$cell_seek" = "1" ]; then cell_seek_arg="true"; else cell_seek_arg="false"; fi

  # What this cell's manifest WILL say for the six M-TARE knobs, so the resume
  # guard below can compare rather than assume. Per cell and not per campaign,
  # because the arm token is what sets them: each mtare_* token expands to its
  # own full stack inside run_explo_sim_rviz.sh, every other arm takes the
  # --env value or the
  # runner's default. Kept in step with that expansion by hand -- if the two
  # ever disagree, the resume guard aborts a correct resume, which is the safe
  # direction to be wrong in.
  CELL_WORLD_REQ=$(env_val CELL_WORLD 0)
  TEAM_WORLD_REQ=$(env_val TEAM_WORLD 0)
  GLOBAL_ALLOC_REQ=$(env_val GLOBAL_ALLOC 0)
  RECONNECT_GATE_REQ=$(env_val RECONNECT_GATE silence)
  RENDEZVOUS_SCHEDULE_REQ=$(env_val RENDEZVOUS_SCHEDULE 0)
  PURSUIT_PREDICTOR_REQ=$(env_val PURSUIT_PREDICTOR trail)
  case "$cell_mode" in
    mtare_off)
      CELL_WORLD_REQ=1; TEAM_WORLD_REQ=1
      GLOBAL_ALLOC_REQ=1; RECONNECT_GATE_REQ=silence; RENDEZVOUS_SCHEDULE_REQ=0
      PURSUIT_PREDICTOR_REQ=trail ;;
    mtare_pursuit)
      CELL_WORLD_REQ=1; TEAM_WORLD_REQ=1
      GLOBAL_ALLOC_REQ=1; RECONNECT_GATE_REQ=info;    RENDEZVOUS_SCHEDULE_REQ=0
      PURSUIT_PREDICTOR_REQ=trail ;;
    mtare_rendezvous|mtare_hybrid)
      CELL_WORLD_REQ=1; TEAM_WORLD_REQ=1
      GLOBAL_ALLOC_REQ=1; RECONNECT_GATE_REQ=info;    RENDEZVOUS_SCHEDULE_REQ=1
      PURSUIT_PREDICTOR_REQ=trail ;;
    # The P6 pair: the same stacks as mtare_pursuit / mtare_hybrid with the
    # predictor swapped. Listed separately rather than folded into those two
    # branches so the one field that differs is visible at the point of use.
    mtare_pursuit_mdp)
      CELL_WORLD_REQ=1; TEAM_WORLD_REQ=1
      GLOBAL_ALLOC_REQ=1; RECONNECT_GATE_REQ=info;    RENDEZVOUS_SCHEDULE_REQ=0
      PURSUIT_PREDICTOR_REQ=mdp ;;
    mtare_hybrid_mdp)
      CELL_WORLD_REQ=1; TEAM_WORLD_REQ=1
      GLOBAL_ALLOC_REQ=1; RECONNECT_GATE_REQ=info;    RENDEZVOUS_SCHEDULE_REQ=1
      PURSUIT_PREDICTOR_REQ=mdp ;;
    # NO SILENT DEFAULT (2026-09-18). This case had no `*)` arm, so an arm token
    # that reached here unrecognised kept the six --env-derived defaults above —
    # CELL_WORLD/TEAM_WORLD/GLOBAL_ALLOC at 0, RECONNECT_GATE at silence,
    # RENDEZVOUS_SCHEDULE at 0 — i.e. the expectation set of an UNTREATED cell,
    # and then wrote them into the manifest under a treated arm's name. Nothing
    # downstream could recover the discrepancy, because the manifest is the
    # record of what was expected. run_explo_sim_rviz.sh's RECONNECT_MODE
    # vocabulary check does refuse the run a few seconds later, but only after
    # this has already built and recorded the wrong expectations.
    #
    # The reachable route in is a suffix the strip above cannot express: before
    # today the claim-radius strip took 1-2 digits, so `..._r100` never became
    # `mtare_hybrid` and landed here. That strip now takes three (see the two
    # `*_r[0-9]...` cases), which closes the known path — this arm is for the
    # next one.
    *)
      echo "FATAL: unrecognised arm stack '$cell_mode' (from arm '$arm')." >&2
      echo "       Refusing rather than recording an untreated cell's" >&2
      echo "       expectations under a treated arm's name. Add a case above" >&2
      echo "       if this is a real stack; check the _r/_ttl/_seek suffix" >&2
      echo "       strips if it is a token they failed to remove." >&2
      exit 2;;
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
    # Every key here changes what the run's endpoints MEAN, so a completed cell
    # that disagrees on any of them is not "already complete" — it is a
    # different experiment sharing a directory name. Checking only
    # mission_return covered one of five: a resume that changed --scenario or
    # --duration silently kept the old cells and pooled two horizons under one
    # tag, which is the same defect the mission_return guard was written for.
    # An absent key counts as a mismatch (`<absent>`): a manifest predating the
    # key cannot be shown to agree, and "cannot be shown to agree" is exactly
    # what this guard is for.
    #
    # pursuit_predictor is the newest key and the one where that rule bites a
    # reader who knows better: a manifest without it was written by a binary
    # that had no such parameter, so the cell provably ran `trail` and the
    # abort looks pedantic. It is kept strict anyway, because resuming across
    # that boundary means resuming with a DIFFERENT BINARY, which check 3b
    # hard-fails downstream regardless -- aborting here just says so before
    # another cell's worth of wall time is spent.
    #
    # The six M-TARE knobs are here for the same reason, and one of them is
    # the only member of this list whose absence is COMPLETELY silent. Resume a
    # half-finished campaign with `--env "CELL_WORLD=1 TEAM_WORLD=1"` added and
    # the banked cells satisfy every other key, so seeds 1-15 carry no cell
    # world and seeds 16-30 carry one, under a single arm name. P1 and P2
    # deliberately do not rename the arm — their claim is that they change no
    # decision — so nothing downstream can tell: not check 3e, which compares a
    # stamp that is identical either way, not check 21, not any analysis
    # script. The manifest records it per cell and, until this loop, nothing
    # ever compared it. global_alloc and reconnect_gate would at least be
    # caught by 3e, half the cells having stamped a different arm; that is a
    # louder failure, not a different one.
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
      "done_seek_enabled=$cell_seek_arg"
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
    # The same rule for the keys whose manifest value is a NUMBER, which cannot
    # go in the loop above, because the two sides can spell the same value
    # differently and a string compare would abort a correct resume over that
    # alone -- teaching the operator to distrust the guard, which is the worse
    # of the two failures. Compared with awk instead.
    #
    # For most of these the launcher itself introduces the difference: it runs
    # the value through flt() at assignment, so `--tx 30` is banked as "30.0"
    # while the campaign still holds the "30" it was handed. The three
    # separation keys are NOT flt()'d on that path -- the launcher writes them
    # into the manifest exactly as typed and normalises only at the -p site --
    # so for those the difference is between two invocations rather than
    # between the two sides. They are compared the same way regardless: a
    # numeric key is a number, and which spelling reached the file is not
    # something a resume decision should turn on.
    #
    # done_unknown_fraction was the only member for a long time. The radio pair
    # joined it on 2026-09-03, when the shipped tree_attenuation_db moved from
    # 11.98 to 70.0 and max_range_m appeared: resuming a half-finished campaign
    # across that change would have banked seeds 1-15 on a radio where one
    # trunk costs 12 dB and seeds 16-30 on one where a trunk is fatal and no
    # link reaches past 30 m, under a single tag, with the manifests recording
    # the difference and nothing reading them. cell_size_m and tx_power_dbm are
    # here because they are the same shape of fact -- the unit the coverage
    # census counts, and the power every link is scaled by.
    #
    # coord_claim_radius_override is compared too, and it is the one key with a
    # sentinel: "none" means no -p was passed and the node took the yaml value.
    # That is a positive statement about the cell, not a gap, and it is NOT the
    # same cell as a pinned 10.0 even though the yaml default happens to be 10
    # -- one fixed the radius, the other took whatever the config said that day.
    #
    # Matched as a string when either side is "none", numerically otherwise.
    # The string branch is what stops awk from reading a non-number as 0: under
    # a numeric-only compare every unparseable value -- "unset", "default", a
    # truncated line -- would equal "none" and a cell whose radius can no longer
    # be determined would score as agreeing.
    #
    # The numeric branch has to refuse non-numbers for the same reason, and it
    # did not until a review found it. `a + 0` reads ANY unparseable string as
    # 0, so for every key whose requested value is 0 -- separation_weight in a
    # control campaign, which is to say all of them so far -- a manifest reading
    # "off", "unset" or a line truncated by the out-of-disk failure the guard
    # above exists to catch would compare EQUAL and the cell would be banked as
    # agreeing when its value can no longer be read at all.
    #
    # "nan" is worse than that and is why the test is a regex rather than a
    # comparison against 0. This awk parses it as a real NaN and then reports
    # `nan == <anything>` as TRUE, so a single corrupted value would agree with
    # every key at every level, not just the ones requesting 0. Checked here:
    #   awk -v a=nan -v b=10 'BEGIN{print (a+0 == b+0)}'   ->   1
    #
    # So both sides must LOOK like a number before their values are compared;
    # anything else falls through to the abort, which is the safe direction.
    # Surrounding whitespace is stripped first, because the regex is stricter
    # than `+ 0` was -- " 20" used to compare equal to 20 and now would not --
    # and aborting a correct resume over a stray space is the failure this
    # whole numeric branch exists to avoid.
    for kv in \
      "done_unknown_fraction=$DONE_UNKNOWN" \
      "tx_power_dbm=$TX" \
      "tree_attenuation_db=$TREE_ATTEN_REQ" \
      "max_range_m=$MAX_RANGE_REQ" \
      "cell_size_m=$CELL_SIZE_REQ" \
      "coord_claim_radius_override=${cell_claim_r:-none}" \
      "alloc_peer_pos_max_age_sec=${cell_pos_ttl:-none}" \
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
    # C3: team size, checked for CONSISTENCY rather than against a prediction.
    # n_robots is derived inside the runner from the scenario roster, and this
    # script does not carry that mapping -- so the honest check is not "is it
    # the number I expected" but "do all the banked cells of this tag agree".
    # That is the hazard that matters: scenario IS compared above, but the
    # scenario -> roster expansion lives in an install tree that can be rebuilt
    # between two halves of a campaign, and a tag holding 40 two-robot cells and
    # 40 three-robot cells under one name would pass every other check here.
    # Order statistics over robots move with N under a pure null, so pooling
    # across team sizes is not a small error.
    #
    # IT IS DETECTIVE, NOT PREVENTIVE, AND IT FIRES ONE INVOCATION LATE. This
    # whole block sits in the COMPLETE branch of the single `for cell` loop
    # above, so it only ever reads cells that were already banked when the loop
    # reached them. Cells THIS invocation runs are never compared against
    # BANKED_N_ROBOTS — nothing reads their manifest until some later run of
    # this script walks past them as complete. So the rebuilt-roster case plays
    # out as: invocation 2 banks its differently-sized cells without a word, and
    # invocation 3 aborts on a tag that is already mixed. The abort is still
    # worth having (it stops the mixed tag being EXTENDED, and it names the
    # problem in a place the operator will look), but do not read it as a
    # promise that a tag on disk is homogeneous just because the last run did
    # not abort. It only proves the cells that were complete BEFORE that run
    # agreed with each other. To check a tag as it stands, compare n_robots
    # across the banked manifests directly.
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
    # THE VERDICT IS READ, NOT MERELY TESTED FOR `=INVALID`. A banked cell
    # carries exactly one of CLEAN / SUSPECT / INVALID, or no key at all, and
    # until 2026-09-18 this site asked only the third of those four questions.
    # Everything that was not literally INVALID -- SUSPECT, and the absent key --
    # fell through to the `else` and was logged `SKIP (already complete)`: the
    # same line, character for character, that a CLEAN cell gets. gate_g8
    # hard-fails both, but gate_g8 is run by hand after the campaign, so the
    # first anyone heard of a bank full of uncertifiable cells was at analysis
    # time with every hour of sim already spent.
    #
    # `tail -1` and not `head -1`: the teardown APPENDS this key, so if a
    # manifest ever carries two the last one is the verdict that was reached.
    # (It should not -- the OUTDIR is rm -rf'd before a retry -- but reading the
    # first would silently prefer a stale verdict over the live one, and the
    # cheap read is the one that cannot be wrong.)
    banked_verdict=$(sed -n 's/^run_gates_verdict=//p' \
                       "$out/run_manifest.txt" 2>/dev/null | tail -1)
    # REDO_SUSPECT IS OPT-IN, AND DELIBERATELY SO. Re-rolling a cell conditions
    # the retained sample on whatever made it fail, which is the argument
    # recorded just below for keeping the INVALID attempts; SUSPECT's causes are
    # mostly harness defects rather than outage severity, but "mostly" is not a
    # basis for silently re-rolling somebody's arm. The default therefore keeps
    # the evidence and makes the operator look; the env var exists for when the
    # operator has looked and decided.
    if [ "$banked_verdict" = "INVALID" ] || \
       { [ "${REDO_SUSPECT:-0}" = "1" ] && [ -n "$banked_verdict" ] \
         && [ "$banked_verdict" != "CLEAN" ]; }; then
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

  log "START $name (reconnect_mode=$cell_mode done_seek=$cell_seek claim_r=${cell_claim_r:-yaml} pos_ttl=${cell_pos_ttl:-yaml} mission_return=$MISSION_RETURN_FLAG free=${free_mb}MB)"
  t0=$(date +%s)
  # An ideal-comms cell has no link to drop, so demanding an outage would fail
  # every gate; force expect_outage off rather than trusting the caller.
  cell_expect="$EXPECT_OUT"
  [ "$COMMS_ON" = "0" ] && cell_expect=0

  # The -u list, and this is a correctness fix, not tidiness.
  #
  # `env` without -i inherits the caller's environment, and these are the ONLY
  # settings the guards above reason about that are not also assigned explicitly
  # on this line. run_explo_sim_rviz.sh reads them as
  # "${LINK_GATE:-1}" and "${MIDRUN_SILENCE:-90}", so an exported value from the
  # launching shell -- a leftover debugging export, a line in a wrapper -- beat
  # the launcher default while the guard, which scans only $EXTRA_ENV, computed
  # the default and stayed silent. `export LINK_GATE=0` followed by a normal
  # campaign launch would have run every treated cell of a multi-day matrix on
  # the 90 s clock with no veto: exactly the configuration the guard exists to
  # refuse, arriving through the one channel it could not see.
  #
  # THE INVARIANT, stated once so it can be checked mechanically rather than
  # remembered: every knob this script reads through env_val() must be either
  # stripped here or assigned explicitly further down this same line. env_val
  # scans $EXTRA_ENV alone, so any knob that is neither is one the guard models
  # as its default while the cell runs on whatever the ambient shell held.
  # campaign_guard_calib.sh derives the env_val list from this file and checks
  # that invariant, rather than re-listing the names — the list was already
  # three knobs out of date (TREE_ATTEN, MAX_RANGE, CELL_SIZE_M) by the time
  # anything looked, which is what a hand-maintained copy is worth.
  #
  # Stripping them makes the guard's model true by construction rather than by
  # assumption, and costs nothing: --env is expanded after these flags and env
  # applies assignments after unsets, so `--env LINK_GATE=0` still works and is
  # still the channel the guard reads.
  #
  # $RUNNER_STRIP is the SUPERSET, derived from the runner's own
  # `${NAME:-default}` reads, where this literal list is derived from what THIS
  # script models -- see its construction near the dry-run exit. The two
  # overlap heavily and that is fine; repeating a -u is a no-op. The literal
  # names stay because each carries a reason a derivation cannot, and because
  # campaign_guard_calib.sh reads them back out of this line.
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
      RECONNECT_MODE="$cell_mode" DONE_SEEK="$cell_seek" \
      COORD_CLAIM_R="$cell_claim_r" \
      ALLOC_POS_TTL="$cell_pos_ttl" \
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
    # THE VERDICT TRAVELS ON THE OK LINE. rc=0 says the stack came up and the
    # run reached its end condition; it says nothing about whether the gates
    # certified it, and under the default GATES_STRICT a SUSPECT cell exits 0.
    # So `OK ... rc=0` was the operator-facing report for a cell that gate_g8
    # will hard-fail, and the campaign log gave no way to tell the two apart
    # while there was still time to stop.
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
