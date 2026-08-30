#!/usr/bin/env bash
# Host-side orchestrator for a WATCHABLE 2-robot exploration + exploitation run
# in the hmr_sim `flatforest` world, lidar-only, with RViz.
#
# This is the 2026-08-02 campaign stack (Bags/2026_08_02__flatforest_2robot_lidar
# _ablation/run_experiment.sh, condition B) with the visualization it never had:
# that campaign ran headless with no RViz and no robot model, so a run was only
# legible after the fact through the CSVs. Here you watch it live.
#
# Everything is NATIVE on the host (ROS humble + the ws overlay). Nothing in
# Docker — hmr_sim is never containerised.
#
# What you see in RViz (config/explo_sim_2robot_lidar.rviz):
#   * both husky meshes, from a robot_state_publisher per robot serving the viz
#     URDF wrapper around hmr_sim's SDF meshes (hmr_sim bridges no
#     /robot_description, so RobotModel has nothing to draw otherwise);
#   * the merged TEAM map (atlas's dscovox_node copy) as height-coloured voxels;
#   * each planner's candidate arrows (red -> green by utility) + selected goal.
#     During EXPLOIT those arrows ARE the vantage ring around the active trunk;
#   * released tree targets as a trunk cylinder + vantage ring + label
#     (sim_target_markers.py — TreeTarget is a custom msg RViz cannot draw);
#   * off by default, one click away: planning maps, raw VLP-16 clouds, nav
#     global/local paths, odom trails, bestla's own copy of the merged map.
#
# The run is exploration + exploitation: the scheduler releases the three
# targets_flatforest.yaml trunks at t = 120 / 420 / 720 sim-seconds; both robots
# activate each one, split the vantage ring through /exploration/intents claims,
# and the team's clear-LoS dwell union closes it. Between targets both revert to
# EIG exploration.
#
# Usage (from anywhere):
#   ./run_explo_sim_rviz.sh                 # runs until Ctrl-C
#   DURATION_S=960 ./run_explo_sim_rviz.sh  # stop after 960 sim-seconds
#   GZ_GUI=1 ./run_explo_sim_rviz.sh        # + the ignition GUI (costs RTF)
#   RECORD=1 ./run_explo_sim_rviz.sh        # + a rosbag under $OUTDIR
#   RVIZ=0   ./run_explo_sim_rviz.sh        # headless; NOTE the planner gates
#                                           # candidate-marker publishing on
#                                           # subscriber count, so no RViz means
#                                           # no marker traffic at all
#   TARGETS=/path/to/targets.yaml           # a different trunk triple, or a
#                                           # never-releasing file for
#                                           # exploration-only
#   DWELL_SYNC=0 ./run_explo_sim_rviz.sh    # A/B: drop the vantage-ring
#                                           # rendezvous barrier (first robot to
#                                           # arrive dwells alone) — see below
#   RECONNECT_MODE=rendezvous ./run_explo_sim_rviz.sh
#                                           # A/B: reconnection manoeuvre at
#                                           # exploration exhaustion
#                                           # (rendezvous | pursuit | hybrid)
#                                           # — see below
#   CELL_WORLD=1 ./run_explo_sim_rviz.sh    # + the coarse M-TARE cell layer:
#                                           # cell_census events in the jsonl
#                                           # and a colour-coded grid in RViz.
#                                           # Observation only in P1 — nothing
#                                           # reads it. CELL_SIZE_M /
#                                           # CELL_CENSUS_S tune it;
#                                           # CELL_COVERED_U / CELL_EXPLORING_U
#                                           # / CELL_FRONTIER_FRAC are the
#                                           # world-calibrated status
#                                           # thresholds, overridden here for
#                                           # the same reason DONE_UNKNOWN is
#   FOV_VFOV=0.785 FOV_V_RAYS=12 ./run_explo_sim_rviz.sh
#                                           # A/B: override the EIG evaluator's
#                                           # modelled sensor geometry for this
#                                           # run (FOV_HFOV / FOV_VFOV /
#                                           # FOV_H_RAYS / FOV_V_RAYS /
#                                           # CAND_N_YAW). Unset = whatever
#                                           # shared_params.yaml says — see below
# Ctrl-C tears the whole stack down in reverse start order (SIGINT to each
# process group, then SIGKILL to stragglers), the same teardown the campaign
# driver used.
set -u
# Force a decimal point onto anything destined for a `ros2 ... -p x:=<v>` that
# the node declares as a double. `ros2 param` infers the type from the LITERAL,
# so `-p cost_grid_radius_cap_m:=500` is an integer and the node aborts at
# startup with InvalidParameterTypeException; `:=500.0` is what it wants. This
# bit twice: awk prints 10*50.0 as "500", and every knob below that a user might
# reasonably set as `ROI_HALF=35` feeds -p roi_min_x. Run every float-valued
# shell var through this rather than relying on each one being written with a
# decimal by hand -- the failure is a hard abort seconds into a run, and it is
# only visible in the planner log, not on the console.
# Reject non-finite spellings outright: YAML/ROS accept both `nan` and `.nan`
# as a double, and `.nan` matches the *.* passthrough below -- it would sail
# into the planner and silently disarm every comparison against the value.
# Emitting nothing (plus the stderr line) makes the ros2 invocation fail
# loudly instead; `exit` here would only kill the $(...) subshell.
flt() {
  case "$1" in
    *[nN][aA][nN]*|*[iI][nN][fF]*)
      echo "FATAL: non-finite value '$1' for a float parameter" >&2 ;;
    *.*|*e*|*E*) printf '%s' "$1" ;;
    *) printf '%s.0' "$1" ;;
  esac
}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # <repo>/sim
# The repo is expected checked out at …/hmr_explo/ws/src/explo_planner, so the
# ws overlay root is three levels up from this sim/ directory.
WS="$(cd "$HERE/../../.." && pwd)"           # …/hmr_explo/ws
ROBOTS="atlas bestla"
# Overridable so a denser stand can be run without editing the harness. The
# scenario picks the WORLD, and the world sets both the link budget (stems in
# the Fresnel corridor) and the coverage floor — so a scenario change
# invalidates the calibrated done_unknown_fraction and is recorded in the
# manifest for exactly that reason. Any replacement must keep two robots named
# atlas and bestla, since ROBOTS above and the topic wiring below assume them.
SCENARIO="${SCENARIO:-flatforest_2robot_lidar.yaml}"
if [ ! -f "$WS/install/hmr_sim/share/hmr_sim/config/scenarios/$SCENARIO" ]; then
  echo "FATAL: scenario '$SCENARIO' is not installed. Rebuild hmr_sim, or pick"\
       "one of:" >&2
  ls "$WS/install/hmr_sim/share/hmr_sim/config/scenarios/" 2>/dev/null >&2
  exit 2
fi
DURATION_S="${DURATION_S:-0}"                # 0 = until Ctrl-C
RVIZ="${RVIZ:-1}"
GZ_GUI="${GZ_GUI:-0}"                        # 1 => ignition GUI as well
RECORD="${RECORD:-0}"
MAX_STEPS="${MAX_STEPS:-500}"
# FRONTIER_ONLY=1 (default) overrides shared_params.yaml's candidate_enable_polar
# to false, so EXPLORE candidates are ONLY frontier centroids — no polar grid of
# n_radial*n_rings*n_yaw = 96 samples around the robot. Exploitation is
# unaffected either way: vantages come from VantagePlanner::generateVantages(),
# not CandidateGenerator::generate().
#
# CAVEAT, from the shared_params.yaml comment that set the field default to
# polar-on: in frontier-only mode the planner has been seen to exhaust reachable
# candidates (~70) once the robot pushed into an ROI corner, then oscillate until
# the planner timeout fired. Polar exists as the fallback supply. Watch for a
# rising `candidates rejected` / repeated identical goals; FRONTIER_ONLY=0
# restores the field behaviour.
FRONTIER_ONLY="${FRONTIER_ONLY:-1}"
# Proximity-yield band, SIM ONLY. shared_params.yaml ships 5.0/6.0, sized for the
# real-robot reaction budget (peer pose age + tick + nav2 cancel propagation +
# braking at 2x0.8 m/s closing). In flatforest the two huskys work a 3-vantage
# ring whose angles sit ~3.5 m apart, so a 5 m hold disc means the robot driving
# to its angle is braked by the teammate standing on the next one — the yaml
# defaults would spend the run in PROXIMITY_HOLD. These override the params file
# for this script only; the field defaults stay untouched.
# These are doubles in the planner, and `-p x:=2` would make ros2 infer an
# integer and abort the node at startup on the type mismatch. flt() adds the
# decimal point, so `PROX_HOLD_M=2` is now accepted -- but the same trap is live
# for any NEW float knob added below that skips flt().
PROX_HOLD_M="$(flt "${PROX_HOLD_M:-1.5}")"
PROX_RESUME_M="$(flt "${PROX_RESUME_M:-2.5}")"
# DWELL_SYNC=1 (default) keeps the vantage-ring rendezvous barrier on: a robot at
# its vantage holds (dwell clock not started) until every peer claiming the same
# trunk is standing on its own angle, so the team captures one trunk state
# simultaneously. DWELL_SYNC=0 is the A/B against the old behaviour, where the
# first robot to arrive dwells alone and can close the quota before the peer
# lands. See exploit_dwell_sync_* in shared_params.yaml.
DWELL_SYNC="${DWELL_SYNC:-1}"
# EXPLOIT=0 turns the run into pure exploration: exploitation_enabled:=false and
# no target_scheduler at all. The comms/reconnection matrix REQUIRES this (plan
# §3.5) and the yaml default is true, so a matrix run left on the default would
# not be the experiment the plan describes.
#
# Why it contaminates: the scheduler releases three trunks on a sim-time
# schedule, both robots detour to each one, and the vantage ring is split
# through peer CLAIMS on /exploration/intents — the very stream COMMS=1 gates.
# Link severity would then drive exploitation stalls directly, standDownExploitation()
# would fire inside every reconnect manoeuvre, and target detours of arm-dependent
# length would land in the primary coverage endpoint. /exploration/targets is
# also a global bus the emulator does not relay, so the one piece of team
# knowledge that stays perfect under a modelled radio outage would be the target
# list. Default 1 keeps this script's watchable-demo behaviour unchanged.
EXPLOIT="${EXPLOIT:-1}"
# Mesh reconnection manoeuvre (rendezvous | pursuit | hybrid) — which of the
# robot-carried-radio reconnection methods runs when a planner exhausts its
# goals with its teammate out of comms. The pursuit budgets ride along from
# shared_params.yaml (240 s cap / 180 s staleness gate, flatforest-sized).
# The planner invocation below also passes rendezvous_expected_peers:=1
# (this is a 2-robot stack): shared_params.yaml ships 0 — the field value,
# where the launch file computes team-size-1 — and the planner hard-disables
# the whole reconnect feature at startup on expected_peers=0, which would
# leave this knob silently inert.
# NOTE: stock sim DDS is one broadcast domain, so with both planners healthy
# no manoeuvre ever runs: the first finisher parks DONE-idle and keeps
# beaconing, the second finisher counts it and goes DONE in place. The A/B
# that exercises the mode is a dead peer (kill one planner mid-run: the
# survivor's exhaustion finds the claim aged out, chases the corpse's trail,
# then falls back per mode) or, later, a range-gated intent bridge emulating
# finite comms.
#
# `off` is a FOURTH value handled here, not by the planner: it disables the
# manoeuvre outright (rendezvous_enabled:=false) and is the control arm the
# matrix compares the other three against. It cannot be expressed as a
# reconnect_mode — reconnectModeFromString() falls back to RENDEZVOUS on any
# string it does not recognise (planner_util.cpp), setting only a `known` flag
# that the node reports as a single WARN and then ignores. So passing
# `reconnect_mode:=off` would not disable anything; it would silently run the
# rendezvous arm twice and the control-vs-treatment contrast would be a
# comparison of an arm with itself. Unknown values are rejected below rather
# than quietly mapped, for the same reason.
RECONNECT_MODE="${RECONNECT_MODE:-hybrid}"
case "$RECONNECT_MODE" in
  rendezvous|pursuit|hybrid|off) ;;
  *) echo "FATAL: RECONNECT_MODE='$RECONNECT_MODE' is not one of \
rendezvous|pursuit|hybrid|off. The planner would silently fall back to \
rendezvous and the run would be mislabelled." >&2; exit 2 ;;
esac
# Barrier cap. The planner's code default is 0 = wait forever, which is the
# right field behaviour and the wrong experiment: a robot that gives up on the
# chase raises the barrier at its current pose and never lowers it, so the run
# ends at the horizon and its time-to-reconnect is undefined — in exactly the
# arm where the modes differ most. Observed in p4mild_pursuit_seed1: both
# robots declined a stale chase, held, and censored the run at 5807 s.
# A finite cap makes the ending observable ("gave up after N s") instead of
# indistinguishable from "still waiting". Set 0 to restore the field default.
# flt() per the warning above: these are doubles in the planner and a bare
# integer makes ros2 infer int and abort the node at startup.
RDV_MAX_WAIT="$(flt "${RDV_MAX_WAIT:-600}")"
# The pre-arm confirmation window (planner param reconnect_confirm_sec). The
# claim table can lag intents that were already delivered, so a manoeuvre armed
# on one read of it may be released by the next tick; 8 of 24 recorded firings
# ended within 5 s having moved under a metre. 0 restores arm-on-first-read.
RECONNECT_CONFIRM="$(flt "${RECONNECT_CONFIRM:-3.0}")"
# --- Mid-run trigger + pursuit-gate family (2026-08-17 redesign) -------------
# These now default ON and match the planner's own code defaults, so a bare
# invocation of this script runs the same policy the robots run. To reproduce
# the p7modes (pre-redesign) behaviour bit-for-bit, export MIDRUN_SILENCE=0
# RECONNECT_RELEASE_CONFIRM=0 PURSUIT_STALENESS=180 HOLD_ESCALATE=0.
# Every one of these is echoed into the manifest below: the p7modes campaign
# taught that an un-recorded planner param makes runs post-hoc
# indistinguishable, which the mode comparison then has to treat as a confound.
# Silence (s) of continuous peer absence before a robot interrupts exploration
# to run its arm's reconnect manoeuvre. 0 = terminal-only (legacy).
#
# 90 since generation 9; it was 240 through generation 8. The old value had to
# clear the ~180 s heartbeat-suppression tail by pure waiting, because the
# record-age clock alone cannot tell a silent teammate from an absent one and
# firing at a healthy, silently-planning peer was the failure it had to avoid.
# LINK_GATE=1 (now the default, below) makes that distinction directly from the
# radio, so the threshold is no longer set by the suppression tail.
#
# Lowering it was forced by measurement, not preference: in g8r1's 23 hybrid
# cells the 240 s clock expired in 3, so 87 % of the treated arm ran
# behaviourally identical to the control and the campaign could not measure its
# own treatment.
#
# TWO DRAFTS OF THIS NUMBER WERE WRONG BEFORE THIS ONE. The first, 120, came
# from a circular sweep: it filtered episodes on the presence clock and then
# scored them on the same clock, so "zero waste" merely restated the filter. The
# second, 90, was re-scored against g8r1's banked link_states.csv with the filter
# on the past (presence gap) and the score on the future (radio) -- but on the 20
# hybrid cells that did NOT dispatch, which is a sample selected on the outcome
# being swept.
#
# The number that stands is scored on the `off` arm, which never ran the trigger
# at all and is therefore untreated by construction. There, at LINK_DOWN_CONFIRM
# as it now ships (0, i.e. "not up right now"), 90 arms 18 of 23 cells, 20 % of
# fires land in an outage that would have closed on its own anyway, and 85 %
# beat natural recovery -- the argmin of waste across the UNWALKED grid, where
# 120, 90 and 75 all arm the same 18 of 23.
#
# NEITHER FACT SURVIVES THE FORWARD WALK, so 90 is not chosen by the grid. Scored
# at T plus the measured 14.0-18.8 s dispatch overshoot the waste argmin moves to
# 75 at every offset and the plateau breaks; a ranking that flips that easily is
# one 23 cells cannot resolve, and re-tuning to 75 on the same cells would repeat
# the error. What survives is the coarse verdict (240 far too high, 60 past where
# waste turns back up); inside 120-75 the choice is off-grid, and 90 clears the
# p90 radio outage (52.0-111.0 s, nearest-rank) while sitting far below 240. See
# §32.15 and the member-default comment in explo_planner_node.cpp, which carries
# the full table and the walked numbers.
MIDRUN_SILENCE="$(flt "${MIDRUN_SILENCE:-90}")"
# Barrier give-up for mid-run attempts (terminal barriers keep RDV_MAX_WAIT).
MIDRUN_MAX_WAIT="$(flt "${MIDRUN_MAX_WAIT:-240}")"
MIDRUN_MAX_ATTEMPTS="${MIDRUN_MAX_ATTEMPTS:-6}"
# --- Post-latch coast (2026-08-27) -----------------------------------------
# 56 of 88 banked reconnect_end events are a robot that crossed the coverage
# latch mid-chase: it brakes, drops the contact, and the partner keeps waiting
# at a barrier for someone who is no longer coming. DONE_SEEK=1 keeps the nav
# goal the robot already had so it finishes that drive and delivers its map.
#
# It is metric-neutral by construction -- state_ reads DONE from the same tick,
# so the all_done check below is untouched -- which is exactly why it has to be
# a RUNTIME switch: treated and control cells must sit inside ONE run_campaign
# invocation or the arm is confounded with the session (30.27, ~1.08x floor).
# Default 0 so an un-set campaign reproduces every banked run.
#
# SUPERSEDED under MISSION_RETURN=1: the mission-return branch in the planner
# pre-empts the coast gate at every terminal ending, so the coast is
# unreachable there (the homing traverse is itself the go-reconnect behaviour
# the coast approximated). DONE_SEEK matters only in MISSION_RETURN=0 runs.
DONE_SEEK="${DONE_SEEK:-0}"
if [ "$DONE_SEEK" = "1" ]; then DONE_SEEK_ARG="true"; else DONE_SEEK_ARG="false"; fi
# Cap on a single coast. 0 disables the cap (the no-progress exit still holds).
DONE_SEEK_MAX="$(flt "${DONE_SEEK_MAX:-600}")"
# --- Mission return (2026-08-27) --------------------------------------------
# MISSION_RETURN=1 adds an arm-invariant requirement to the mission itself: at
# ANY terminal exploration ending (coverage latch, step budget, barrier
# give-up) the robot drives back to its recorded start pose. Both arms then
# end in the same connected configuration (spawns are 3 m apart), which is
# what makes "mission end time" a well-defined primary endpoint in the off
# arm too — the run still ends when every planner reads DONE, so the all_done
# check below is untouched, but DONE now means "home (or bounded give-up)",
# not "parked wherever exploration ended".
#
# NOT metric-neutral for exploration finish: a robot that finishes first now
# drives home through the world and can deliver its map to the still-exploring
# partner en route, in BOTH arms — that is part of the shared mission
# definition, so exploration finish under MISSION_RETURN=1 is a NEW endpoint,
# never poolable with banked numbers. Default 0 so an un-set invocation
# reproduces every banked run bit-for-bit.
MISSION_RETURN="${MISSION_RETURN:-0}"
if [ "$MISSION_RETURN" = "1" ]; then MISSION_RETURN_ARG="true"; else MISSION_RETURN_ARG="false"; fi
# Arrival tolerance. Its own knob (not RECONNECT_ARRIVE_TOL=4.0) because the
# two homes are only 3 m apart — the manoeuvre tolerance would accept the
# partner's home as an arrival.
MISSION_HOME_TOL="$(flt "${MISSION_HOME_TOL:-1.0}")"
# Overall cap on the homing leg. The field guarantee that a mission-return run
# still ends: on expiry the robot parks where it is and the run ends with
# mission_complete result=timeout. 0 disables the cap.
MISSION_RETURN_MAX="$(flt "${MISSION_RETURN_MAX:-600}")"
# --- Information gate on the mid-run trigger (2026-08-19) -------------------
# Replaces the fixed MIDRUN_SILENCE clock with "reconnect once the pair has
# gathered RECONNECT_MIN_SHARE_VOX of map the other side has not seen", by
# dividing that target by the pair's beaconed gathering rate and clamping the
# quotient into [MIDRUN_MIN_SILENCE, MIDRUN_MAX_SILENCE].
#
# 0 (the default) keeps the legacy fixed clock EXACTLY -- midrunGateSec returns
# reconnect_midrun_silence_sec unevaluated -- so every campaign already on disk
# reproduces bit-for-bit and this block is inert unless a run asks for it.
#
# Sizing, measured on cg050's 8 cells (gate_forecast.py / gate_verdict.py):
# the pair gathers ~9,100 vox/s in the first 200 s and ~2,900 vox/s after, so
#   550k / 9,100 = 60 s  -> the floor is reached but never BINDS, which is the
#                           point: every firing time is set by the information,
#                           not by the clamp (at 400k the floor binds and the
#                           arm would be a fixed 60 s clock wearing the gate's
#                           name; at 700k a whole cell drops to zero triggers).
#   550k / 2,900 = 190 s -> still inside the ceiling, so late outages still fire
#                           EARLIER than the 240 s legacy clock.
#   550k / 319   = ceiling -> a saturated pair drifting apart slowly is declined,
#                           which is the gate doing its job: 200 s of silence at
#                           that rate is only ~64k voxels, four chance merges.
# MIDRUN_MAX_SILENCE must stay <= MIDRUN_SILENCE: a ceiling above it would let
# the gated arm fire LATER than the control and confound "gated vs not" with
# "waited longer". The planner warns at startup if it does.
#
# It TRACKS MIDRUN_SILENCE rather than carrying a copied constant. It used to be
# a literal 90, which was correct only by coincidence: it matched generation 9's
# clock and would have silently violated the invariant the moment either number
# moved, in exactly the direction the warning exists to catch.
#
# THE FLOOR NOW BINDS OVER MOST OF THE USABLE RANGE, which the previous wording
# denied. WHICH RATE YOU DIVIDE BY DECIDES THE ANSWER, and the previous wording
# mixed two in one sentence: it wrote "550k/9,100 = 60 s" (cg050's early-run
# rate, above) and then quoted a window derived from a different rate entirely.
# The window [138k, 207k] is the ~2,300 vox/s figure the ESTIMATOR was
# calibrated on in p14 (explo_planner_node.cpp, reconnect_min_share_voxels_):
# 138k/2300 = 60 s = the floor, 207k/2300 = 90 s = the ceiling. Under cg050's
# rates the same clamps give [546k, 819k] early and [174k, 261k] late. Both are
# legitimate; they answer for different phases of a run and differ by ~4x, which
# is precisely why the target has to be RE-DERIVED against whichever rate the
# next campaign's world actually produces rather than copied from either. Do not
# reuse 550k. Both clamps are inert while
# RECONNECT_MIN_SHARE_VOX=0, which is the default and what the current campaign
# runs -- keeping the invariant true matters for the day the gate is switched on
# again, not for these runs.
RECONNECT_MIN_SHARE_VOX="$(flt "${RECONNECT_MIN_SHARE_VOX:-0}")"
MIDRUN_MIN_SILENCE="$(flt "${MIDRUN_MIN_SILENCE:-60}")"
MIDRUN_MAX_SILENCE="$(flt "${MIDRUN_MAX_SILENCE:-$MIDRUN_SILENCE}")"
# Release flicker guard: the team must read complete this long before a
# manoeuvre releases (and the silence clock resets). 0 = first-read (legacy).
# MUST exceed coord_claim_ttl_sec (5.0), which the old default of 3 did not:
# one packet holds a peer live for the whole TTL, so a 3 s window was satisfied
# by that single packet and the guard admitted the very flicker it was written
# to reject. The planner warns at startup if this is set at or below the TTL.
RECONNECT_RELEASE_CONFIRM="$(flt "${RECONNECT_RELEASE_CONFIRM:-6}")"
# Arrival tolerance for a manoeuvre destination (m), and the ceiling on one
# manoeuvre drive leg (s). See shared_params.yaml for the sizing argument.
RECONNECT_ARRIVE_TOL="$(flt "${RECONNECT_ARRIVE_TOL:-4.0}")"
RECONNECT_NAV_MAX="$(flt "${RECONNECT_NAV_MAX:-600}")"
# Pursuit gates. The old 180 s staleness vetoed every chase in the dense world
# (outage tail 861 s ~ staleness at a terminal trigger), so the chase was dead
# code here; 900 clears that tail. What keeps the wider window honest is
# PURSUIT_GOAL_STALE: past 180 s the chase drops the peer's declared goal and
# drives to its last contact pose instead.
PURSUIT_STALENESS="$(flt "${PURSUIT_STALENESS:-900}")"
# INVARIANT: chase budget <= waiter patience. A teammate at the barrier treats
# this as the worst case it may assume about its pursuer, so a budget above
# RDV_MAX_WAIT (600) would let a chase outlive the wait that justified it.
PURSUIT_BUDGET_MAX="$(flt "${PURSUIT_BUDGET_MAX:-600}")"
PURSUIT_GOAL_STALE="$(flt "${PURSUIT_GOAL_STALE:-180}")"
# Pure pursuit's fallback: keep exploring rather than park. Parking is a fixed
# point (two held robots cannot reconnect — p7modes: 5 of 6 holds never did,
# and one mutual hold cost a mission). Bounded, then it reverts to the hold so
# the terminal barrier still guarantees an ending.
PURSUIT_EXPLORE_FALLBACK="${PURSUIT_EXPLORE_FALLBACK:-1}"
PURSUIT_EXPLORE_FALLBACK_ARG=$([ "$PURSUIT_EXPLORE_FALLBACK" = "1" ] \
  && echo true || echo false)
PURSUIT_EXPLORE_MAX="${PURSUIT_EXPLORE_MAX:-6}"
# Terminal-hold escalation: on barrier expiry drive once to the last-connected
# anchor and wait HOLD_ESCALATE_WAIT more before giving up (breaks the
# mutual-hold deadlock that cost the p7modes pursuit_seed2 mission).
HOLD_ESCALATE="${HOLD_ESCALATE:-1}"
HOLD_ESCALATE_ARG=$([ "$HOLD_ESCALATE" = "1" ] && echo true || echo false)
HOLD_ESCALATE_WAIT="$(flt "${HOLD_ESCALATE_WAIT:-300}")"
# COMMS=1 puts the message-level radio emulator (hmr_comms_sim_node) between the
# two robots, which is what turns the NOTE above from a caveat into a runnable
# experiment: with it, "peer out of comms" is produced by distance through trees
# rather than by killing a process. It rewires two streams, and BOTH are needed —
# gating only one produces a run that looks fine and measures nothing:
#   * scovox_bin  — each merger reads its peer's map off /<self>/rx/<peer>/...
#     instead of the peer's own publisher, so map sharing obeys the link.
#   * exploration/intents — planners publish to /<robot>/exploration/intents
#     (per-robot, so the emulator has something to relay) and subscribe to the
#     relayed copies. Left on the shared global bus, no outage can ever make a
#     peer read as missing, and no reconnect manoeuvre would ever fire.
# COMMS=0 (default) leaves both on their direct topics: one broadcast domain,
# perfect comms. That is the only thing COMMS toggles — it is NOT "the 2026-08-02
# campaign unchanged". This script now also enables the 2D planning map
# (use_planning_map, off in the yaml and off in that campaign), which activates
# the candidate free/occupied filter AND the cost-grid reachability filter that
# were both entirely inactive before; overrides the ROI to a ±50 box instead of
# the yaml's field site; and samples the CSV on a timer. rejected_by_unreachable,
# rejected_by_minpos and goal selection therefore do not mean the same thing they
# did in that campaign's CSVs, and the two are not directly comparable.
COMMS="${COMMS:-0}"
# --- Coarse cell world (M-TARE evolution, P1) -----------------------------
# CELL_WORLD=1 turns on the coarse global layer: the ROI diced into
# CELL_SIZE_M cells, each carrying a status derived from this robot's own map,
# sampled into `cell_census` events every CELL_CENSUS_S sim-seconds and drawn
# on /<r>/explo_planner/cell_world.
#
# OFF by default, and off means NO PARAMETER IS PASSED — not `cell_world_enable
# :=false`. The per-phase equivalence gate reads the run_start param dump and
# requires every param new in the child to sit at its compiled default; passing
# the knob explicitly would still satisfy that, but passing `team_robot_names`
# would NOT, and the two have to travel together because the cell world refuses
# to configure without a fleet identity. So the whole block is gated.
#
# In this phase the layer is pure observation: nothing reads it to make a
# decision. What a CELL_WORLD=1 run therefore costs is one extra sweep of the
# voxel grid per census tick and one JSONL row; what it buys is the P1 gate,
# which reads `covered_fraction` against `roi_unknown_fraction` on those rows.
CELL_WORLD="${CELL_WORLD:-0}"
CELL_SIZE_M="$(flt "${CELL_SIZE_M:-10.0}")"
CELL_CENSUS_S="$(flt "${CELL_CENSUS_S:-5.0}")"
# The cell status thresholds, overridden here for the same reason DONE_UNKNOWN
# is overridden to 0.64 below: this world has a permanent coverage floor.
#
# A cell's unknown fraction counts x/y columns holding no observed voxel. The
# map stores lidar returns plus a thin free shell, not dense ray-traced free
# space, so even ground a robot drove straight over keeps a large fraction of
# its 0.1 m columns empty forever. Measured over two runs, the per-cell floor
# (`cell_unknown_min`) plateaus at 0.36-0.42 and the mature median at
# 0.50-0.55, against an ROI-wide 0.63 — which is exactly why DONE_UNKNOWN is
# 0.64 and not the shipped 0.05. The library defaults (0.15/0.35) assume a map
# that saturates and are unreachable here: the first P1 run at those values
# promoted zero cells over its entire length.
#
# Placed by the same rule as DONE_UNKNOWN. Release just under the level the
# mission itself accepts for the whole ROI, because a cell that has degraded to
# the mission's own accept level is not distinguishably explored; promote a
# band's width below that. The band must clear the run-to-run spread of the
# floor, not just one run's: 0.42 was set from a single long run's p10 and a
# later, shorter run floored at 0.4232 and promoted nothing.
#
# Re-derive rather than nudge: cell_census carries
# cell_unknown_{min,p10,median} and cell_frontier_frac_{min,median} on every
# row. 0.95 for the frontier veto because in this sparse map 79-89% of a
# cell's voxels are boundary voxels, so a threshold inside that band splits the
# population near its median and reports mostly noise; at 0.95 it guards
# outliers and the column measure discriminates.
CELL_COVERED_U="$(flt "${CELL_COVERED_U:-0.55}")"
CELL_EXPLORING_U="$(flt "${CELL_EXPLORING_U:-0.62}")"
CELL_FRONTIER_FRAC="$(flt "${CELL_FRONTIER_FRAC:-0.95}")"
# Link fading is a pure function of (seed, tick), so this alone selects the run's
# link realisation. Paired-seed designs vary it while holding everything else
# fixed; it is inert with COMMS=0.
SEED="${SEED:-42}"
# Radio severity. Matches comms_sim_params.yaml's shipped value, so the default
# changes nothing — it exists so the number lands in the run manifest and can be
# swept from the command line during calibration rather than by editing the
# installed yaml, which would silently re-scope every later run and leave no
# record of which severity any given run used.
TX_POWER="$(flt "${TX_POWER:-30.0}")"
# The SECOND severity dial, and the one that acts on geometry rather than
# margin. tx_power_dbm shifts every link equally; tree_attenuation_db shifts a
# link in proportion to how many trunks are on it, so it is what selects
# occlusion-driven outages over distance-driven ones. Exposed here for exactly
# the reason TX_POWER is, and it was missing: the sw-campaign design (§30) needs
# to move it, and until now the only way to do that was to edit the installed
# comms_sim_params.yaml — which re-scopes every later run on the machine and
# leaves no field in any run directory saying which severity that run used.
# Default matches the shipped yaml, so a bare invocation changes nothing.
TREE_ATTEN="$(flt "${TREE_ATTEN:-11.98}")"
# Does this run's arm expect the link to drop? 1 = yes (every treatment arm),
# 0 = the control arm, deliberately run at a tx_power_dbm that keeps the link
# up. Only affects the run-time outage gate's verdict, never the radio itself:
# severity is TX_POWER's job alone, and the two are set independently so that a
# control run whose link DID drop is still reported rather than excused.
EXPECT_OUTAGE="${EXPECT_OUTAGE:-1}"
# --- Link-state gate for the mid-run reconnect trigger (§30.11, §30.24) ------
# The trigger used to fire on peer RECORD AGE, which ages whenever a teammate is
# not sending -- link up or down. Measured against the emulator's own trace it
# ran +49.1 s ahead of real link-down and 41-42 % of all mid-run fires bought
# nothing, 16-21 % of them firing while the radio was UP. LINK_GATE=1 points the
# planner at the emulator's connected bit instead.
#
# ON BY DEFAULT since generation 9. It was off through generation 8, on the
# argument that every banked campaign (pb3g2, tr1, tl1, tl2, td1) ran the
# record-age clock and a default flip would silently make the next run
# incomparable with all of them. That argument no longer holds and a stronger
# one now points the other way:
#
#   - Generation 9 already breaks comparability by design. MIDRUN_SILENCE moved
#     240 -> 90 in the same change, so these runs are a new generation and are
#     not poolable with the record-age campaigns whatever this flag says.
#   - The two settings are COUPLED. 90 s sits below the ~180 s
#     heartbeat-suppression tail, so it is only safe because the veto can tell a
#     silent teammate from an absent one. Defaulting the threshold low and the
#     veto off leaves the low clock running bare -- firing at a healthy peer
#     that is merely deep in a PLAN loop.
#
#     And a BARE INVOCATION IS that combination, which an earlier draft of this
#     comment denied. COMMS defaults to 0, so LINK_GATE=1 resolves to
#     LINK_GATE_EFFECTIVE=0 and the 90 s clock runs with no veto. That is
#     tolerable for a smoke test and intolerable for a campaign, which is why
#     run_campaign.sh refuses it outright (see its guard, and
#     campaign_guard_calib.sh for the known-answer cases). Here it is a loud
#     WARNING at the manifest write, not a refusal.
#   - g8r1 is what an unset gate costs in practice: it ran with the veto off and
#     the only trace at the firing site was a "radio down -1s" sentinel, which
#     reads like a measurement rather than "the check did not run".
#
# Set LINK_GATE=0 explicitly to reproduce a record-age campaign. The manifest
# line below is still the record either way, and it is now the thing to check
# first when comparing two runs.
#
# Only meaningful with COMMS=1: with no emulator there is no link topic, the
# planner finds no samples, and it would spend the run warning about a gate it
# cannot use. So the topics resolve to "" unless the radio is actually in
# circuit, which also keeps a COMMS=0 smoke test quiet.
#
# Set identically for EVERY arm. The arms must differ in RECONNECT_MODE alone;
# `off` cannot reach the gated code at all (it needs rendezvous_enabled, which
# is false there), so passing it the same parameters costs nothing and removes
# a whole class of "did the arms differ in something else too" question.
LINK_GATE="${LINK_GATE:-1}"
# Validated like COMMS and RECONNECT_MODE, and for the same reason: every value
# that is not exactly "1" turns the veto off in the topic block below, so "true"
# or "2" reads as a request and behaves as a refusal. Worse, it used to be
# copied verbatim into LINK_GATE_EFFECTIVE, so the manifest recorded
# link_gate_effective=2 for a run with no veto at all, and the safety WARNING is
# conditioned on LINK_GATE="1" and so could not fire. Refuse the value instead
# of recording a provenance line that disagrees with the run.
case "$LINK_GATE" in
  0|1) ;;
  *) echo "FATAL: LINK_GATE='$LINK_GATE' is not 0 or 1. Any other value \
disables the link veto while looking like a request for it, and the manifest \
would record it as such." >&2; exit 2 ;;
esac
if [ "$LINK_GATE" = "1" ] && [ "$COMMS" = "1" ]; then
  LINK_GATE_TOPIC="${LINK_GATE_TOPIC:-/hmr_comms_sim/link_states}"
  LINK_GATE_INDEX="${LINK_GATE_INDEX:-/hmr_comms_sim/robot_index}"
else
  LINK_GATE_TOPIC=""
  LINK_GATE_INDEX=""
fi
# LINK_GATE is the REQUEST; LINK_GATE_EFFECTIVE is what the planner will
# actually get, and only the second one describes the run. They diverge on
# exactly one pairing -- gate asked for, emulator absent -- and that pairing is
# silently unsafe rather than merely inert: MIDRUN_SILENCE now defaults BELOW
# the ~180 s heartbeat-suppression tail on the understanding that the veto can
# tell a quiet teammate from an absent one, so losing the veto leaves the low
# threshold running bare. Say so at the top of the log instead of letting the
# reader reconstruct it from two manifest lines.
#
# Only the flag is computed here: this is the defaults block and log() is not
# defined until much later in the file, so the warning itself is emitted at the
# manifest write where the rest of the run's configuration is reported.
#
# Derived from the OBSERVABLE, not re-derived from the inputs. LINK_GATE_TOPIC
# being non-empty is the single fact that decides whether the node is handed
# comms_link_states_topic at all (see the EXTRA block below), so reading it back
# here means link_gate_effective cannot drift from what the run did if the
# condition above is ever edited. An earlier version recomputed the same
# conjunction a second time and assigned "$LINK_GATE" in the else branch, which
# is only 0-or-1 because the validator above now makes it so.
if [ -n "$LINK_GATE_TOPIC" ]; then
  LINK_GATE_EFFECTIVE=1
else
  LINK_GATE_EFFECTIVE=0
fi
# Newest link sample older than this and the planner stands down to the legacy
# clock rather than acting on a stale belief. The emulator publishes at 5 Hz.
LINK_GATE_STALE="$(flt "${LINK_GATE_STALE:-3.0}")"
# How long the radio must have been CONTINUOUSLY down before the veto lets a
# mid-run chase through. Its own knob since generation 9; both veto sites used
# to borrow RECONNECT_CONFIRM (3.0), which is the team-PRESENCE release confirm
# and answers a different question. Decoupling them is right at any value.
#
# THE VALUE IS 0, AND THE 30 s DEBOUNCE THAT WAS HERE IS WITHDRAWN. At 0 the
# veto is exactly its disjunct -- "never chase a peer whose radio is up right
# now" -- which is the whole test. 30 was credited in an earlier draft of this
# comment with moving wasted fires "from 50 % to 33 %", and that was a
# misattribution: the table it pointed at held this conjunct FIXED at 30 and
# varied the presence clock, so 50 -> 33 is the 150 -> 90 move, not this
# parameter's effect. Measured properly on the untreated `off` arm, holding
# MIDRUN_SILENCE at 90:
#
#     confirm   cells arm (of 23)   fires   wasted   beats natural recovery
#        0            18             40      20 %            85 %
#       30            12             26      15 %            92 %
#
# It buys 5 points of purity for A THIRD of the arm's activation -- 18 arming
# cells down to 12, i.e. 6 of the 18 that armed. ("A quarter", in an earlier
# draft, is 6/23: the share of the arm, not of the activation being traded.)
# Dilution is the defect this generation exists to fix, so the trade goes the
# other way.
LINK_DOWN_CONFIRM="$(flt "${LINK_DOWN_CONFIRM:-0}")"
# Reliable-relay backlog cap, in bytes. The emulator's shipped default is 64 MiB
# and on overflow it drops the OLDEST queued map delta and never retransmits, so
# the receiver's merged map loses those voxels for the rest of the run. That
# silently breaks the experiment's central premise — the backlog is supposed to
# DRAIN at each contact (B0a), which requires the link to be a delay, not a
# lossy channel. At 64 MiB it did not hold: measured against tx_power_dbm=-14
# and 0.20 m voxels, atlas published 3345 scovox_bin deltas and bestla received
# 3190, i.e. 153 lost exactly as drop_overflow reported, and the `off` arm lost
# 1180 in BOTH directions. Sized here for the worst case instead: ~120 kB/s of
# deltas per direction (2 Hz, ~60 kB each) against a full T=3600 s blackout is
# ~430 MB, so 1 GiB carries a 2.4x margin and costs at most 2 GiB of RAM across
# both directions. The overflow counter stays a hard gate — this raises the cap
# so the gate stops firing for real, it does not silence it.
RELAY_QUEUE_BYTES="${RELAY_QUEUE_BYTES:-1073741824}"
# Planner ROI half-extent, SIM ONLY (square, centred on the world origin).
# shared_params.yaml carries the real field site's ROI — x ∈ [-51.3, 100.9],
# y ∈ [-38.7, 74.5] — and on flatforest ~42% of that footprint has no geometry
# at all: those columns never leave the prior, so the unknown fraction can never
# reach done_unknown_fraction and EVERY run ends at max_steps instead of at
# coverage. That still fires one reconnect manoeuvre (the step-budget path also
# routes through finishOrRendezvous), which is why the symptom is invisible —
# what it destroys is the repeat reconnect -> re-disperse cycles the experiment
# needs. ±50 fits inside the 110x110 ground plane and covers the oak field.
# ±50 also fits inside the global planning map derived from it just below.
ROI_HALF="$(flt "${ROI_HALF:-50.0}")"
# Side of the WORLD-FIXED planning map (~/global_planning_map) — the
# 2D map the EXPLORATION planner consults for free/occupied and reachability.
# Published by BOTH scovox_node (this robot's own measurements) and
# dscovox_node (the fused team map); same envelope, same resolution, so the
# two are comparable cell-for-cell. The planner reads the DSCOVOX one — see
# the map-domain note below.
# Both it and the ROI are centred on the world origin, so side 2*ROI_HALF
# exactly covers the ROI and 3*ROI_HALF leaves ROI_HALF/2 of margin on each
# side. The margin is not cosmetic: candidates are ROI-clipped but the ROBOT is
# not, and the cost-grid flood starts from the robot's own cell — a robot that
# has drifted past the ROI edge onto an out-of-bounds cell floods nothing and
# every candidate is rejected as unreachable. Derived from ROI_HALF so the two
# cannot drift apart.
#
# This is NOT /<r>/scovox_node/planning_map. That one is a 20 m robot-centred
# crop whose extent IS simple_nav_3d's local-planner window, and the exploration
# planner rejects any candidate whose cell is out of bounds (isCellOccupied
# treats out-of-bounds as occupied) — on the rolling map every frontier beyond
# ~10 m is dropped and exploration collapses to a bubble around the robot.
# Widening that topic instead would push the whole world through the local A*
# on the control path.
#
# THE MAP-DOMAIN FIX (2026-08-21). The planner used to read scovox_node's copy,
# which contains only what THIS robot measured — while planning over the FUSED
# team map for everything else. Ground the partner surveyed was therefore
# unknown on the reachability map, so candidates in it were rejected as
# unreachable and the planner starved. dscovox_node now publishes the same
# world-fixed envelope over the fused grid and the planner reads that instead,
# putting reachability in the same domain as the candidates it filters.
#
# Still NOT /<r>/dscovox_node/planning_map (no "global_"). That name means
# scovox_node's 20 m rolling crop, which the LOCAL nav planner consumes.
# simple_nav_3d's nav global planner used to subscribe to it — transient_local,
# with no publisher — and so never planned for the whole campaign history; in
# generation 5 it was repointed at this same global_planning_map instead of
# that name being made real. Both planners now read this topic; neither reads
# the other's.
PLAN_MAP_SIZE="$(flt "${PLAN_MAP_SIZE:-$(awk "BEGIN{print 3*$ROI_HALF}")}")"
PLAN_MAP_RES="$(flt "${PLAN_MAP_RES:-0.40}")"
# Dijkstra flood radius for the candidate reachability filter. MUST be set here,
# and this is the trap that comes with switching the planning map on.
#
# The filter is dead code with use_planning_map=false: no map means no cost grid
# to flood, so doPlan sets skip_reachability and every candidate passes. Turning
# the map on activates it — with a cap that has therefore never been exercised.
# The yaml ships cost_grid_radius_cap_m: 0.0 = auto = candidate_max_radius + 2 =
# 10 m, a bound sized for POLAR candidates, which are generated within
# candidate_max_radius by construction. Frontier centroids have no range limit
# at all (addFrontierCandidates clips to the ROI and nothing else), and
# FRONTIER_ONLY=1 removes the polar set entirely — so under the auto cap every
# frontier more than 10 m of walked distance away comes back kInfCost and is
# rejected as unreachable. The escape hatch does not save it either: it only
# trips below 10 reached cells, and a 10 m flood at 0.4 m/cell reaches ~1900.
# Exploration would degenerate to 10 m hops, or stall outright once no frontier
# remains inside the disc — and it would look like a legitimate result.
#
# 10x ROI_HALF is deliberately far past the grid diagonal ($PLAN_MAP_SIZE * 1.41
# = 212 m at the defaults): the intent is "unbounded within this grid", and a
# cap must not be clamped to the diagonal because a walked Dijkstra distance
# routinely exceeds the straight line (see the warning in CostGrid::floodFrom).
# Passing 0 to mean unbounded does NOT work — 0 is the auto sentinel. The cost
# is one full flood of a ~140k-cell grid per PLAN tick, which the exploitation
# planner already pays on the same grid with a genuinely unbounded flood.
COST_CAP="$(flt "${COST_CAP:-$(awk "BEGIN{print 10*$ROI_HALF}")}")"
# Minimum range to an acceptable EXPLORE candidate. The yaml ships 0.0 (off,
# preserving field behaviour) and this script overrides it, because on
# flatforest 0.0 does not merely explore inefficiently — it deadlocks.
#
# Measured, not theorised: at 0.0 both robots entered a two-point oscillation
# (atlas ping-ponging between goals 0.87 m apart, bestla 0.63 m) and stayed
# there. Steps kept advancing, every process stayed healthy, the CSV kept
# filling, and the unknown fraction flatlined at 0.87 having moved 0.001 in the
# last 100 sim-seconds. Nothing in the stack reports it: the harness hang gate
# watches the STEP counter, and the steps were fine.
#
# Cause is in the planner's utility, not here — see candidate_min_goal_dist_m
# in shared_params.yaml. 4.0 m sits well inside fov_max_range (10.0), so a hop
# still lands deep in previously-seen space rather than jumping blind.
#
# This is a real scenario correction in the sense of plan §2: it changes what
# every arm does, so it must be identical across arms and no run from before it
# may be pooled with one after.
MIN_GOAL_DIST="$(flt "${MIN_GOAL_DIST:-4.0}")"
# Distance discount on the utility denominator: U = info / (eps + cost)^GAMMA.
# 1.0 is the original SSMI form and the node short-circuits it to be
# bit-identical. Goes through flt() because the node declares it as a double
# and an integer literal would abort it at startup on the type mismatch (see
# the flt() note at the top).
#
# The default moved 1.0 -> 0.5 on 2026-08-19 on a 6-replicate sweep: 18.9%
# faster to the 0.60 rung (487 +/- 46 s vs 600 +/- 90 s, exact permutation
# p = 0.0152), via 25% fewer metres driven at unchanged speed. The evidence and
# its limits are written out at utility_cost_exponent in shared_params.yaml —
# read that before trusting the number, because it was measured on ONE world
# with PERFECT COMMS.
#
# CAMPAIGNS THAT STRADDLE THE CHANGE MAY NOT BE POOLED. This is a SCENARIO
# CORRECTION in the sense of plan §2 exactly like MIN_GOAL_DIST: it changes
# what every arm does. Anything compared against p14 or against any run made
# before 2026-08-19 must set UTIL_GAMMA=1.0 explicitly rather than relying on
# the default, which no longer reproduces those runs. It lands in the run
# manifest below so a cell can always be attributed after the fact.
UTIL_GAMMA="$(flt "${UTIL_GAMMA:-0.5}")"
# Frontier search band, as an inset from the ROI z band. The yaml ROI band is
# [-5.5, +4.0] (sized for the field site's terrain and canopy), so these insets
# put the frontier search in absolute z [0.2, +1.5] on flatforest's flat ground
# — about the slice a Husky's VLP-16 actually sweeps as it drives.
#
# MIN_GOAL_DIST alone does NOT fix the oscillation, which is why both exist:
# with the full 9.5 m band the near-frontier set is unobservable-by-construction
# (the lidar's free-space wedge has a top and bottom surface at every range and
# those surfaces are frontier forever), so raising the minimum hop just moves
# the ping-pong out to that radius. Measured: at MIN_GOAL_DIST=4.0 with the
# full band, atlas oscillated between two goals 4.3 m apart instead of 0.87 m
# apart. The band is what makes a frontier something driving can consume; the
# minimum hop is what stops the planner picking a goal too close to observe
# from. Verify from the planner log line "Frontier search band inset by ...",
# which prints the ROI band it was applied to.
FRONTIER_Z_LO_OFF="$(flt "${FRONTIER_Z_LO_OFF:-5.7}")"
FRONTIER_Z_HI_OFF="$(flt "${FRONTIER_Z_HI_OFF:-2.5}")"
# EIG sensor-model overrides. ALL UNSET BY DEFAULT — with none of them set the
# harness passes nothing and shared_params.yaml remains the single point of
# control for the evaluator's geometry, which is what it was before these
# existed. They exist so an A/B on that geometry does not have to edit an
# installed build artifact (install/.../shared_params.yaml) and then remember to
# rebuild, which is unreproducible and invisible in the run manifest.
#
# fov_hfov/fov_vfov are doubles in the node and go through flt(); the *_RAYS and
# CAND_N_YAW are declared as integers and must NOT get a decimal point, or the
# node aborts at startup on the type mismatch (see the flt() note at the top —
# the trap runs in both directions).
#
# CAND_N_YAW only bites with POLAR on: FRONTIER_ONLY=1 (the default here) turns
# the polar grid off entirely, and frontier candidates take their yaw from the
# centroid direction, not from this count.
FOV_HFOV="${FOV_HFOV:-}"
FOV_VFOV="${FOV_VFOV:-}"
FOV_H_RAYS="${FOV_H_RAYS:-}"
FOV_V_RAYS="${FOV_V_RAYS:-}"
CAND_N_YAW="${CAND_N_YAW:-}"
# Recently-visited goal suppression. The yaml ships 0.0 (off, field behaviour);
# this scenario needs it, and it is the third and last piece of the same defect.
# MIN_GOAL_DIST stops the planner picking a goal too close to observe from, the
# frontier band stops it chasing frontiers no sensor can ever consume, and this
# stops it going back to where it just was -- which it otherwise does forever,
# because a forest's trunk shadows keep regenerating frontier clusters and
# utility reduces to "nearest" once info gain is flat.
#
# 6.0 m is just above frontier_cluster_radius_m (5.0), so suppressing a reached
# goal suppresses the cluster that produced it, not a point inside it. The TTL
# must outlive a there-and-back hop at these distances; 180 s is ~15 steps at
# the observed cadence, and it expires rather than persisting so that an area
# genuinely re-frontiered later in the run can still be revisited.
VISITED_RADIUS="$(flt "${VISITED_RADIUS:-6.0}")"
VISITED_TTL="$(flt "${VISITED_TTL:-180.0}")"
# Failed-goal blacklist (generation 5). The mismatch that made this a defect:
# the TTL was 60 s while ONE nav attempt is budgeted up to nav_max_timeout_sec
# = 180 s, so a blacklisted trap reliably expired while the robot was still
# burning a single budget somewhere else and was then free to be re-picked.
# mr1_hybrid_seed18 atlas did exactly that: eight 180 s failures at the same
# two sites, 1441 s of the run spent on ground it had already proved
# unreachable, ending 0.021 above the 0.64 coverage threshold -- i.e. censored
# by this alone. 240 = 180 + 60 keeps a site suppressed across a full failure
# elsewhere; the planner WARNs at startup if this drops below that sum.
# RETIRE_AFTER holds a site for the rest of the run once it has failed that
# many times, because TTL expiry alone cannot close a permanent terrain trap.
# Both are released immediately by actually reaching the site.
FAILED_TTL="$(flt "${FAILED_TTL:-240.0}")"
FAILED_RETIRE="${FAILED_RETIRE:-3}"
# The disc the TTL above applies over. Passed and recorded because "suppressed"
# is a claim about an AREA, not a point: at 2.0 m one entry covers a goal and
# its neighbours, and at 0.2 m the planner would re-pick a cell 30 cm from the
# trap and call it a new goal. A manifest that names the TTL but not the radius
# describes half of the mechanism.
FAILED_RADIUS="$(flt "${FAILED_RADIUS:-2.0}")"
# The nav budget the TTL above is sized against, and the no-progress abort that
# ends a leg early. All four are recorded for one reason: generation 5's
# censoring vocabulary is written in these units. A cell ending in `timeout`
# means a leg hit NAV_MAX_TIMEOUT, `no-progress` means it moved less than
# PROGRESS_MIN_DIST in PROGRESS_WINDOW, and neither label can be read at all
# without the numbers that produced it. They were fixed in the YAML through four
# binary generations and invisible in every cell, so a reader comparing a gen-4
# cell to a gen-5 one had no way to tell whether the budget had moved under them.
NAV_MIN_TIMEOUT="$(flt "${NAV_MIN_TIMEOUT:-30.0}")"
NAV_MAX_TIMEOUT="$(flt "${NAV_MAX_TIMEOUT:-180.0}")"
PROGRESS_WINDOW="$(flt "${PROGRESS_WINDOW:-15.0}")"
PROGRESS_MIN_DIST="$(flt "${PROGRESS_MIN_DIST:-0.2}")"
# Refuse the exact defect generation 5 exists to fix, rather than warning about
# it. The planner has its own startup check on this inequality, but it is a
# WARN: it scrolls past in a log nobody reads until the campaign is over, which
# is precisely what happened for the whole of mr1. Here it costs a cell nothing
# to stop.
#
# The threshold is the planner's (+30), NOT the 240 = 180 + 60 the default
# picks. Two numbers for two jobs: +30 is the floor below which the TTL is
# certainly broken, and the extra 30 s in the default is deliberate slack on
# top. A guard set at the default would reject working configurations, and one
# set below +30 would pass the broken ones.
awk -v ttl="$FAILED_TTL" -v navmax="$NAV_MAX_TIMEOUT" 'BEGIN {
  if (ttl < navmax + 30.0) exit 1
}' || {
  echo "FATAL: failed_goal_ttl_sec=$FAILED_TTL is below nav_max_timeout_sec" >&2
  echo "       ($NAV_MAX_TIMEOUT) + 30. A blacklisted trap would expire while" >&2
  echo "       the robot is still burning ONE nav budget elsewhere, and be free" >&2
  echo "       to be re-picked the moment it returns -- the mr1_hybrid_seed18" >&2
  echo "       failure. Raise FAILED_TTL or lower NAV_MAX_TIMEOUT." >&2
  exit 2
}
# Homing approach watchdog (generation 5). See §32.10: the original homing
# watchdog measured GROSS metres travelled, which a robot orbiting a local
# minimum satisfies forever. These bound the approach test that replaces it.
RETURN_APPROACH_WINDOW="$(flt "${RETURN_APPROACH_WINDOW:-40.0}")"
RETURN_APPROACH_MIN="$(flt "${RETURN_APPROACH_MIN:-1.0}")"
RETURN_ESCAPE_MAX="${RETURN_ESCAPE_MAX:-3}"
RETURN_ESCAPE_LEG="$(flt "${RETURN_ESCAPE_LEG:-30.0}")"
# scovox voxel edge length. The launch default is 0.10, which does not survive a
# run long enough to reach coverage termination: free space is carved along the
# whole ray to max_range 20 m, so the fused map grew to 12.7M voxels by t=550 s
# and the planner's map ingest fell behind for good — one robot then drove for
# three minutes on a frozen map, still stepping, still logging, coverage flat,
# with nothing in the stack reporting it. 0.20 is ~8x cheaper.
#
# NOT a free knob under COMMS=1: the ScovoxMapBinary deltas are exactly what the
# radio model carries, so this sets the offered load the link is stressed with.
# The §4 severity calibration must be run at the resolution the campaign runs
# at; a tx_power_dbm calibrated at another resolution does not transfer.
VOXEL_RES="$(flt "${VOXEL_RES:-0.20}")"
# Coverage-termination threshold. The yaml ships 0.05, which this scenario
# cannot reach: measured on a full-length control run, the ROI unknown fraction
# fell to 0.4922 by t=2900 and then did not move for the remaining 2600 sim-s
# while both robots drove a further ~950 m each. That is a floor, not a plateau
# on the way somewhere — the frontier utility is info/(eps+cost) with info
# near-constant, so the planner is a diffusive nearest-frontier crawler: trunk
# shadows keep regenerating frontiers inside the region it has already covered,
# and a distant unexplored corner never wins on cost. It saturates at roughly
# half of a +-50 ROI and then cycles there indefinitely.
#
# So this follows plan §2.1's own prescription — measure the floor, set the
# criterion above it — rather than chasing a threshold that cannot be hit. The
# criterion lands in the fast early phase, where map sharing is what separates
# the arms, and the runs terminate naturally instead of every arm censoring at T
# and reporting the same non-answer. The default was 0.55 through the sw/pb
# campaigns; it is 0.64 from tr1 onward, which is the value tr1's 40 banked
# cells were actually run at and the value the current criterion is defined at.
#
# It MUST be identical across arms: it is the definition of the primary
# endpoint, so a run at a different value is not comparable to one at this one.
# Recorded in the manifest for exactly that reason.
DONE_UNKNOWN="$(flt "${DONE_UNKNOWN:-0.64}")"
# WHICH RULE decides a robot has finished. "latch" (default): each robot is
# finished the first tick its OWN dscovox-fused ROI unknown fraction reaches
# DONE_UNKNOWN — any state, no confirmation streak, no rendezvous gate, and it
# never un-finishes. The run then ends when BOTH robots have latched, which is
# the STOP_ON_DONE rule below unchanged (it already waits for every planner's
# state column to read DONE).
#
# Why the default moved off "streak": the old rule was only tested at the top of
# doPlan, so a robot inside a reconnect manoeuvre could not declare itself
# finished however saturated its map was. On tr1 that put ~60 s of detection
# latency on the hybrid arm and 0 s on off — the endpoint moved with the
# treatment, which makes it part of the treatment rather than a measure of it.
# "latch" measures the same thing in both arms. Robots still BEHAVE differently
# between arms; the chase simply no longer decides when the clock stops.
#
# Like DONE_UNKNOWN this is the definition of the primary endpoint, so it MUST
# be identical across arms and runs at a different value are not comparable.
# Recorded in the manifest for that reason.
DONE_CRITERION="${DONE_CRITERION:-latch}"
case "$DONE_CRITERION" in
  latch|streak) ;;
  *) echo "DONE_CRITERION must be 'latch' or 'streak', got '$DONE_CRITERION'" >&2
     exit 2 ;;
esac
OUTDIR="${OUTDIR:-/tmp/explo_sim_$(date +%Y%m%d_%H%M%S)}"
# Own DDS domain, NOT the default 0. This box runs other ROS work (the scovox
# replay harnesses) on domain 0, and a second /clock publisher appearing there
# mid-run would corrupt every use_sim_time consumer in this stack — sim time is
# not something a run can recover from. Every child inherits this, RViz and the
# script's own `ros2 topic echo` gates included; to poke at the run by hand,
# export the same value first:  export ROS_DOMAIN_ID=42
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Per-robot viz mesh set. Both robots spawn as COSTAR_HUSKY_SENSOR_CONFIG_LIDAR
# in the scenario, but that model is one colour — RViz uses the yellow/green
# variants' meshes (same geometry, painted differently) so the two robots are
# telling apart at a glance.
declare -A VIZ_MODEL=(
  [atlas]=COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_YELLOW
  [bestla]=COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_GREEN
)

mkdir -p "$OUTDIR"
log() { echo "[$(date +%H:%M:%S)] $*"; }
# The other robot in a 2-robot stack. Everything COMMS=1 rewires is per-link, so
# the wiring needs to name the far end; this is deliberately only correct for
# |ROBOTS| == 2, which is what this harness is.
peer_of() { local s=$1 p; for p in $ROBOTS; do [ "$p" = "$s" ] || { echo "$p"; return; }; done; }

# --- environment: humble + ws overlay, miniconda stripped -------------------
# (miniconda on PATH shadows /usr/bin/python3 and breaks catkin_pkg/ament)
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)
unset PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV || true
# Snap scrub — REQUIRED when this is launched from the VS Code snap's integrated
# terminal (or any snap-hosted shell). That environment exports GTK/GDK/glib
# paths pointing inside /snap, and a system rviz2 started under it loads the
# snap's GTK stack, which drags in /snap/core20's libpthread and dies at startup
# with "undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE". Verified:
# stripping these makes rviz2 start with hardware GL. Harmless in a plain shell,
# where none of them are set.
unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_IM_MODULE \
      GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR GSETTINGS_SCHEMA_DIR \
      LOCPATH GIO_MODULE_DIR XDG_DATA_HOME QT_IM_MODULE QT_PLUGIN_PATH \
      LD_LIBRARY_PATH || true
[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ] && export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
[ -n "${XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG:-}" ] && export XDG_CONFIG_DIRS="$XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG"
for v in $(env | grep -oE '^SNAP[A-Z_]*'); do unset "$v"; done
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
PLANNER_SHARE=$(ros2 pkg prefix explo_planner)/share/explo_planner
TARGETS="${TARGETS:-$PLANNER_SHARE/config/targets_flatforest.yaml}"
[ -f "$TARGETS" ] || { echo "ERROR no such targets file: $TARGETS"; exit 2; }

# RViz layout + viz URDF are read from the SOURCE tree, not the install share:
# both are pure config, so editing them applies on the next run with no rebuild.
RVIZ_CFG="$HERE/../explo_planner/config/explo_sim_2robot_lidar.rviz"
URDF_IN="$HERE/../explo_planner/config/costar_husky_viz.urdf.in"
for f in "$RVIZ_CFG" "$URDF_IN" "$HERE/sim_tf_publisher.py" "$HERE/sim_target_markers.py" \
         "$HERE/comms_gates.py"; do
  [ -f "$f" ] || { echo "ERROR missing $f"; exit 2; }
done

# --- process bookkeeping (same discipline as the campaign driver) -----------
# Every child is started with setsid and killed by explicit PID/group. Never
# reintroduce `pkill -f`: it self-matches the harness/eval wrapper.
PIDS=()
start() { # start <name> <logfile> <cmd...>
  local name=$1 lf=$2; shift 2
  setsid "$@" > "$lf" 2>&1 &
  local pid=$!
  PIDS+=("$name:$pid")
  log "started $name pid=$pid  (log: $lf)"
}
alive() { kill -0 "$1" 2>/dev/null; }
sim_clock() {
  # Write through a temp file, NOT a pipe: a ros2 daemon spawned mid-call
  # inherits a command-substitution pipe's write end and never closes it,
  # blocking the parent read forever.
  local f="$OUTDIR/.clock.$$"
  timeout 8 ros2 topic echo /clock --once > "$f" 2>/dev/null </dev/null
  awk '/sec:/{print $2; exit}' "$f" 2>/dev/null
  rm -f "$f"
}
stack_procs() {
  # Matching machine-wide by command pattern is correct for ONE cell and a
  # cross-kill for two: this list feeds teardown()'s `kill -KILL`, so without
  # scoping, cell A's teardown SIGKILLs cell B's gazebo and planner mid-run.
  # That surfaces as a random mid-run death in an unrelated cell -- the most
  # expensive kind of bug to chase. When IGN_PARTITION is set we keep only
  # processes whose OWN environment carries the same value, read from
  # /proc/<pid>/environ rather than the command line: the partition is
  # inherited by gazebo and the ros_gz bridge and never appears in argv.
  # Unset (the sequential default) falls through to the original behaviour
  # byte for byte, so this cannot regress a normal single-cell run.
  local matched
  matched=$(ps -eo pid,cmd | grep -E "ign gazebo|explo_planner_node|target_scheduler_node|dscovox_node|scovox_node|scovox_mapping_node|dscovox_mapping_node|hmr_comms_sim_node|simple_nav|robot_state_publisher|sim_tf_publisher|sim_target_markers|rosbag2|parameter_bridge|ros_gz|rviz2" \
    | grep -v grep | grep -v claude | grep -v run_explo_sim_rviz)
  [ -z "${IGN_PARTITION:-}" ] && { printf '%s\n' "$matched"; return 0; }
  printf '%s\n' "$matched" | while read -r p rest; do
    [ -n "$p" ] || continue
    # -x so IGN_PARTITION=p1 cannot match IGN_PARTITION=p10; -F so a partition
    # name is never interpreted as a regex. A pid that exits mid-scan yields a
    # read error, swallowed here, and is omitted -- correct, it needs no kill.
    if tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null \
       | grep -qxF "IGN_PARTITION=${IGN_PARTITION}"; then
      printf '%s %s\n' "$p" "$rest"
    fi
  done
}
count_own() {
  # How many processes matching $1 belong to THIS cell? Same ownership test as
  # stack_procs, for the bring-up guards rather than for teardown. A guard that
  # counts machine-wide is correct for one cell and a RACE for two: with two
  # concurrent cells there are four planner binaries, and whichever cell checks
  # first aborts on the other cell's existence. That is not hypothetical -- it
  # was observed directly, a cell dying "expected exactly 2, found 4" eight
  # seconds after an otherwise healthy start, while the other survived purely
  # because it happened to check after the first had been torn down.
  # IGN_PARTITION unset falls through to the machine-wide count, so sequential
  # runs keep exactly today's behaviour.
  local pat=$1 n=0 p
  for p in $(ps -eo pid=,cmd= | grep "$pat" | awk '{print $1}'); do
    if [ -z "${IGN_PARTITION:-}" ]; then
      n=$((n+1))
    elif tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null \
         | grep -qxF "IGN_PARTITION=${IGN_PARTITION}"; then
      n=$((n+1))
    fi
  done
  printf '%s\n' "$n"
}
TORN=0
teardown() {
  [ "$TORN" = 1 ] && return 0
  TORN=1
  log "teardown: SIGINT to process groups in reverse order"
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    entry=${PIDS[$i]}; name=${entry%%:*}; pid=${entry##*:}
    if alive "$pid"; then
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
      for _ in $(seq 1 60); do alive "$pid" || break; sleep 1; done   # bag zstd
      alive "$pid" && { log "WARN $name ignored SIGINT, killing group"
                        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null; }
      log "stopped $name"
    fi
  done
  sleep 2
  LEFT=$(stack_procs || true)
  if [ -n "$LEFT" ]; then
    log "WARN leftover processes, force-killing by pid:"; echo "$LEFT"
    echo "$LEFT" | awk '{print $1}' | xargs -r kill -KILL 2>/dev/null
  fi
  # Final gate verdict. The watcher is a background child killed by the loop
  # above, so its exit status is unreachable here — the report file it appends
  # to is the only durable record, and until it is read out the run ends looking
  # successful whatever the gates found at minute 40. Includes the outage gate,
  # which can only be decided at the end: a COMMS=1 run whose link never dropped
  # is a control run wearing a treatment label, and nothing before this point
  # can tell.
  if [ "$COMMS" = "1" ] && [ -f "$OUTDIR/comms_gates.txt" ]; then
    # Merged-map convergence, decided only now because it reads the finished
    # CSVs. Both robots merge both scovox_bin streams, so with the deltas in
    # reliable_topics their two copies must agree at the end. Three dense-world
    # runs finished 1.5-1.8% apart with every other gate green: a KeepLast
    # reader overflowing on the reconnect burst discards the excess with no
    # error and no counter, and scovox_node's new-subscriber resnapshot cannot
    # heal it because the emulator's DDS subscription never drops. Appended
    # before the verdict is computed so a holed map counts as a FAIL like any
    # other. `|| true` so a broken checker cannot abort the trap; it emits UNRUN
    # on its own failure paths, which scores as SUSPECT rather than silent PASS.
    if [ -x "$HERE/map_agreement.py" ]; then
      python3 "$HERE/map_agreement.py" "$OUTDIR" \
        --max-pct "${MAP_AGREE_MAX_PCT:-0.5}" >> "$OUTDIR/comms_gates.txt" 2>&1 || true
    fi
    # teardown is trapped long before GATES_STRICT is assigned, and `set -u` is
    # on, so an early die() would abort IN the trap on an unbound variable.
    GATE_VERDICT=UNKNOWN
    NFAIL=$(grep -c "^FAIL" "$OUTDIR/comms_gates.txt" 2>/dev/null || true)
    NUNRUN=$(grep -c "^UNRUN" "$OUTDIR/comms_gates.txt" 2>/dev/null || true)
    if [ "${NFAIL:-0}" != 0 ]; then
      log "=============================================================="
      log "RUN INVALID: $NFAIL gate failure(s). $OUTDIR/comms_gates.txt:"
      grep "^FAIL" "$OUTDIR/comms_gates.txt" | sed 's/^/    /' || true
      log "=============================================================="
      GATE_VERDICT=INVALID
    elif [ "${NUNRUN:-0}" != 0 ]; then
      log "RUN SUSPECT: $NUNRUN gate(s) could not be evaluated — see $OUTDIR/comms_gates.txt"
      GATE_VERDICT=SUSPECT
    elif ! grep -q "	watch	" "$OUTDIR/comms_gates.txt" 2>/dev/null; then
      # No watch summary => the run-time watcher never adjudicated, and the file
      # holds bring-up PASSes only. Absence of failures is NOT a pass: the
      # missing line is exactly the one carrying the outage gate, i.e. the proof
      # that the treatment happened. Every run before 2026-08-15 landed here and
      # was scored CLEAN on bring-up alone, because the watcher was killed with a
      # signal it did not handle (see comms_gates.py's handler registration).
      log "RUN SUSPECT: no run-time gate summary — the watcher never reported"
      GATE_VERDICT=SUSPECT
    else
      log "comms gates: clean for the whole run"
      GATE_VERDICT=CLEAN
    fi
    # The verdict has to outlive this shell. A campaign driver decides "is this
    # cell done?" from the manifest alone, so a verdict that exists only in the
    # console log means an INVALID cell is indistinguishable from a good one on
    # resume and gets skipped forever.
    [ -f "$OUTDIR/run_manifest.txt" ] && \
      echo "run_gates_verdict=$GATE_VERDICT" >> "$OUTDIR/run_manifest.txt"
    # GATES_STRICT used to cover only the BRING-UP gates, so a run whose link
    # gates failed at minute 40 still exited 0 and every campaign driver recorded
    # it as OK. The three phase-3 mode runs were all logged "OK rc=0" while the
    # same teardown printed RUN INVALID directly above it.
    if [ "$GATE_VERDICT" = "INVALID" ] && [ "${GATES_STRICT:-0}" = "1" ]; then
      log "exiting non-zero: GATES_STRICT=1 and the run-time gates failed"
      exit 1
    fi
  fi
  log "teardown complete — outputs in $OUTDIR"
}
trap teardown EXIT INT TERM
die() { log "ERROR $*"; exit 1; }   # trap runs teardown

# wait_for <seconds> <what> -- <cmd...>
# Retry <cmd> until it succeeds or <seconds> of WALL CLOCK have elapsed.
#
# Deadline-based, not attempt-based, and that is the whole point. The gates
# below used `for i in $(seq 1 24); do timeout 8 ros2 topic echo … --once; done`
# and described themselves as waiting 3 minutes. They did not. `ros2 topic echo`
# only blocks for its timeout once the topic's TYPE is resolvable — i.e. once a
# publisher already exists. Before that it prints "does not appear to be
# published yet / Could not determine the type" and returns rc=1 in ~0.3 s, so
# all 24 attempts burned in ~8 s and the run died on a sim that was still
# spawning entities. Same trap with `ros2 topic list`, which always returns
# promptly. Any readiness gate built from a fast-failing probe needs an explicit
# sleep or a deadline; counting attempts silently makes the budget a function of
# how the probe fails.
wait_for() {
  local budget=$1 what=$2; shift 2
  [ "$1" = "--" ] && shift
  local start_s deadline now
  start_s=$(date +%s); deadline=$(( start_s + budget ))
  until "$@" >/dev/null 2>&1; do
    now=$(date +%s)
    [ "$now" -ge "$deadline" ] && return 1
    # Progress, so a three-minute wait is not indistinguishable from a hang.
    [ $(( (now - start_s) % 30 )) -eq 0 ] && \
      log "  waiting for $what … $((now - start_s))/${budget}s"
    sleep 2
  done
  return 0
}

# --- 0. preflight -----------------------------------------------------------
log "=== flatforest 2-robot lidar explore+exploit, RViz=$RVIZ gui=$GZ_GUI ==="
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  (export the same value to inspect by hand)"
log "targets: $TARGETS"
log "outputs: $OUTDIR"
PRE=$(stack_procs || true)
[ -n "$PRE" ] && { echo "$PRE"; TORN=1; die "stack processes already running — refusing to start"; }
if [ "$RVIZ" = "1" ]; then
  export DISPLAY="${DISPLAY:-:1}"
  log "RViz will use DISPLAY=$DISPLAY"
fi

# --- 1. simulator (2 robots, lidar-only models) -----------------------------
GUI_ARG="headless:=true"; [ "$GZ_GUI" = "1" ] && GUI_ARG="headless:=false"
start sim "$OUTDIR/sim.log" \
  ros2 launch hmr_sim robot_sim.launch.py scenario:="$SCENARIO" $GUI_ARG
for r in $ROBOTS; do
  wait_for 180 "$r odom" -- \
    timeout 8 ros2 topic echo /$r/odom_ground_truth --once \
    || die "sim: no /$r/odom_ground_truth after 3 min (see $OUTDIR/sim.log)"
  log "sim publishing $r odom"
done

# --- 2. TF glue + viz model (one each per robot) -----------------------------
# hmr_sim publishes NO TF: sim_tf_publisher.py supplies map->odom (identity),
# odom->base_link (from GT odom) and the static sensor mounts.
for r in $ROBOTS; do
  start tf_$r "$OUTDIR/tf_$r.log" python3 "$HERE/sim_tf_publisher.py" $r
done
# robot_state_publisher exists here ONLY to give RViz a robot to draw: link
# names are pre-prefixed with "<robot>/" so they match the TF frames above with
# no frame_prefix/TF-Prefix indirection, and the wheel joints are fixed (nothing
# publishes /joint_states for a gz-driven robot).
for r in $ROBOTS; do
  urdf="$OUTDIR/${r}_viz.urdf"
  params="$OUTDIR/${r}_viz_params.yaml"
  sed -e "s|@PREFIX@|${r}/|g" -e "s|@MODEL@|${VIZ_MODEL[$r]}|g" -e "s|@ROBOT@|${r}|g" \
    "$URDF_IN" > "$urdf" || die "URDF substitution failed for $r"
  # The URDF goes in through a params FILE, never `-p robot_description:=<xml>`:
  # rcl rejects a parameter override rule whose value contains newlines
  # ("Couldn't parse parameter override rule", arguments.c:343) and
  # robot_state_publisher aborts before it starts. A YAML literal block scalar
  # takes the XML verbatim, colons and quotes in its comments included.
  { echo "/**:"
    echo "  ros__parameters:"
    echo "    use_sim_time: true"
    echo "    robot_description: |"
    sed 's/^/      /' "$urdf"
  } > "$params" || die "params-file generation failed for $r"
  start rsp_$r "$OUTDIR/rsp_$r.log" \
    ros2 run robot_state_publisher robot_state_publisher --ros-args \
      -r __ns:=/$r --params-file "$params"
done

# --- 2b. comms emulator (COMMS=1) -------------------------------------------
# BEFORE the mappers, and the order is load-bearing. Relay subscriptions are
# created by a 1 Hz topic-discovery poll, and only once the source topic already
# exists — so an emulator started after scovox_node misses everything published
# in between, including the initial full map snapshot. That loss is invisible:
# it happens upstream of the relay, so no drop statistic counts it, and the run
# just quietly begins with each robot missing the other's first map. Started
# here, the subscription is in place ~1 s after each publisher appears, well
# before scovox_node has integrated enough lidar to emit anything.
# Robot names and the tree-bearing world SDF come from the same scenario file
# the sim was launched with; the /rx/ readiness check is deferred until after
# the mappers are up, since that is when the relays can first form.
if [ "$COMMS" = "1" ]; then
  start comms "$OUTDIR/comms.log" \
    ros2 launch hmr_sim comms_sim.launch.py \
      scenario:="$SCENARIO" seed:="$SEED" use_sim_time:=true \
      tx_power_dbm:="$TX_POWER" \
      tree_attenuation_db:="$TREE_ATTEN" \
      reliable_queue_max_bytes:="$RELAY_QUEUE_BYTES"
  sleep 3
  log "comms emulator started (seed=$SEED tx_power_dbm=$TX_POWER" \
      "tree_attenuation_db=$TREE_ATTEN" \
      "relay_queue=${RELAY_QUEUE_BYTES}B) ahead of the mappers"
  # Per-run connectivity trace. link_states is published at link_rate_hz and
  # kept nowhere else: the gate watcher reads the aggregate `stats` topic, and
  # recovering it from the bag afterwards means decoding a bag per run. Every
  # §5 metric that mentions the radio -- realised disconnection fraction,
  # outage durations, contact events and their attribution to a planner state
  # -- is derived from this file.
  #
  # Started HERE, one publisher-startup behind the emulator, rather than down in
  # §6b with the gate watcher. From §6b the trace began at t_sim~48 while the
  # planners had been running since ~27, so the first ~21 s of every run had no
  # connectivity record at all — and that is the window in which the robots are
  # closest together and the link is certain to be up, i.e. the part a
  # disconnection fraction most needs in its denominator. The logger simply
  # waits for the topic, so starting it before the mappers costs nothing.
  start linklog "$OUTDIR/link_logger.log" \
    python3 "$HERE/link_logger.py" --out "$OUTDIR/link_states.csv" \
      --ros-args -p use_sim_time:=true
fi

# --- 3. nav + lidar mapping, mergers cross-wired ----------------------------
# mapping:=dscovox_lidar => per robot a scovox_node on /<r>/velodyne_points plus
# a dscovox_node merging BOTH robots' scovox_bin streams, so each planner reads
# its own copy of the TEAM map.
# With COMMS=1 each merger takes its PEER's binary off the emulator's relayed
# copy instead of the peer's own publisher. Self is untouched by the pattern —
# a robot's own map never crosses a radio link.
PEER_BIN_PATTERN="/{peer}/scovox_node/scovox_bin"
[ "$COMMS" = "1" ] && PEER_BIN_PATTERN="/{self}/rx/{peer}/scovox_node/scovox_bin"
log "peer_bin_topic_pattern=$PEER_BIN_PATTERN (COMMS=$COMMS)"
start nav_atlas "$OUTDIR/nav_atlas.log" \
  ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=atlas mode:=ugv \
    mapping:=dscovox_lidar peers:=bestla \
    peer_bin_topic_pattern:="$PEER_BIN_PATTERN" \
    voxel_resolution_m:=$VOXEL_RES \
    global_planning_map_size_m:=$PLAN_MAP_SIZE \
    global_planning_map_resolution:=$PLAN_MAP_RES
start nav_bestla "$OUTDIR/nav_bestla.log" \
  ros2 launch simple_nav_3d simple_nav_3d.launch.py robot:=bestla mode:=ugv \
    mapping:=dscovox_lidar peers:=atlas \
    peer_bin_topic_pattern:="$PEER_BIN_PATTERN" \
    voxel_resolution_m:=$VOXEL_RES \
    global_planning_map_size_m:=$PLAN_MAP_SIZE \
    global_planning_map_resolution:=$PLAN_MAP_RES
for r in $ROBOTS; do
  wait_for 180 "$r planning_map" -- \
    timeout 8 ros2 topic echo /$r/scovox_node/planning_map --once \
    || die "nav: no /$r planning_map after 3 min (see $OUTDIR/nav_$r.log)"
  log "$r mapping alive (planning_map publishing)"
  # The exploration planner's map is a SEPARATE publisher and it is a hard
  # precondition: with use_planning_map=true the planner sits in its start-up
  # wait until one arrives, so a typo in the topic name or a launch arg that
  # silently defaulted to 0.0 (= disabled) would present as two planners that
  # never leave INIT — an hour into a run, with no error anywhere. Gate on it
  # here instead. Both publishers are transient_local and only emit when
  # someone is subscribed, so this echo is also what pulls the first sample.
  wait_for 120 "$r global_planning_map" -- \
    timeout 10 ros2 topic echo /$r/scovox_node/global_planning_map --once \
    || die "nav: no /$r global_planning_map after 2 min — the \
exploration planner will never leave INIT (see $OUTDIR/nav_$r.log)"
  log "$r scovox global_planning_map alive (${PLAN_MAP_SIZE}m @ ${PLAN_MAP_RES}m/cell)"
  # THE ONE THE PLANNER ACTUALLY READS (2026-08-21). Gated separately from the
  # scovox copy above because they fail for different reasons: the scovox one
  # needs only this robot's own integration, while this one additionally needs
  # a ScovoxMapBinary to have arrived and been fused (dscovox allocates its
  # fused grid lazily on the first wire frame, and publishes nothing until
  # then). A longer budget for that reason.
  wait_for 180 "$r fused global_planning_map" -- \
    timeout 10 ros2 topic echo /$r/dscovox_node/global_planning_map --once \
    || die "nav: no /$r dscovox global_planning_map after 3 min — the \
exploration planner reads THIS topic and will never leave INIT. Check that the \
merger fused at least one binary (see $OUTDIR/nav_$r.log)"
  log "$r fused global_planning_map alive (planner reads this one)"
done
if [ "$COMMS" = "1" ]; then
  # grep -q inside a function so wait_for can retry the whole pipeline.
  rx_topics_present() { timeout 6 ros2 topic list 2>/dev/null | grep -q "/rx/"; }
  wait_for 120 "/rx/ relay topics" -- rx_topics_present \
    || die "comms: no /rx/ relay topics after ~2 min (see $OUTDIR/comms.log)"
  log "comms relay topics present"
fi
sleep 10   # let the nav pipelines finish coming up

# --- 4. RViz ----------------------------------------------------------------
# Started BEFORE the planners so its candidate-marker subscription exists from
# the first PLAN cycle (publishCandidateViz() returns early on zero subscribers).
if [ "$RVIZ" = "1" ]; then
  start rviz "$OUTDIR/rviz.log" \
    rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
  sleep 5
fi
start targetviz "$OUTDIR/targetviz.log" python3 "$HERE/sim_target_markers.py"

# --- 5. optional bag --------------------------------------------------------
# Under COMMS=1 the interesting streams are the RELAYED ones: /exploration/intents
# is empty (the planners moved to per-robot topics) and the peers' direct
# scovox_bin publishers still carry everything, so a bag of only the pre-relay
# topics would show perfect comms no matter what the link did. Add the link
# states too — they are the ground truth for when each link was up.
COMMS_BAG_TOPICS=()
if [ "$COMMS" = "1" ]; then
  for r in $ROBOTS; do
    p=$(peer_of "$r")
    COMMS_BAG_TOPICS+=( "/$r/exploration/intents"
                        "/$r/rx/$p/exploration/intents"
                        "/$r/rx/$p/scovox_node/scovox_bin" )
  done
  # /hmr_comms_sim/stats carries backlog_bytes and the drop_* counters
  # (drop_airtime, drop_ber, drop_overflow). Those are the ONLY evidence for
  # the backlog gate and for the shared-airtime confound — a single global
  # token bucket means one large map delta can drive the pool negative and drop
  # every best-effort intent on every link, precisely during the post-reconnect
  # drain when peer presence has to be re-detected. Unrecorded, that mechanism
  # is unfalsifiable after the fact: the gate watcher samples it live but only
  # writes a sample out when it fails.
  COMMS_BAG_TOPICS+=( /hmr_comms_sim/link_states /hmr_comms_sim/robot_index
                      /hmr_comms_sim/stats )
fi
if [ "$RECORD" != "0" ]; then
  # RECORD=1 is the full set — everything needed to reconstruct or re-watch a
  # run. RECORD=2 is the LEAN set: only what the plan's offline analysis
  # consumes, which is the §4 calibration replay (both odom + /clock), the §5.1
  # oracle re-merge (sender-side scovox_bin), and the link/airtime diagnostics.
  #
  # The difference is not marginal. The two OccupancyGrids dominate the full
  # set: global_planning_map alone is (size/res)^2 bytes per message — 375x375 =
  # ~140 kB at the ROI_HALF=50 defaults — published per robot at ~1 Hz, so it
  # outweighs everything the analysis actually reads. A full matrix of 30-40
  # runs at full RECORD does not fit on this box; the lean set does. Neither
  # mode records the RELAYED /rx/ copies: the oracle merge is defined on what
  # each robot SENT, and what arrived is already reconstructable from
  # link_states plus the receivers' own CSVs.
  BAG_TOPICS=( /clock /tf_static
               /atlas/odom_ground_truth /atlas/scovox_node/scovox_bin
               /bestla/odom_ground_truth /bestla/scovox_node/scovox_bin
               /exploration/intents )
  if [ "$RECORD" = "1" ]; then
    BAG_TOPICS+=( /tf
                  /atlas/imu/data /atlas/cmd_vel
                  /atlas/scovox_node/planning_map
                  /atlas/scovox_node/global_planning_map
                  /atlas/goal_pose /atlas/explo_planner/candidates
                  /bestla/imu/data /bestla/cmd_vel
                  /bestla/scovox_node/planning_map
                  /bestla/scovox_node/global_planning_map
                  /bestla/goal_pose /bestla/explo_planner/candidates
                  /exploration/targets )
  fi
  log "recording rosbag (RECORD=$RECORD, ${#BAG_TOPICS[@]} base topics + comms)"
  start bag "$OUTDIR/bag.log" \
    ros2 bag record -o "$OUTDIR/rosbag2" \
      --max-bag-size 1000000000 --compression-mode file --compression-format zstd \
      "${BAG_TOPICS[@]}" \
      ${COMMS_BAG_TOPICS[@]+"${COMMS_BAG_TOPICS[@]}"}
  sleep 3
fi

# --- 6. planners (namespaced, one per robot) + one global scheduler ---------
# Same parameter set as exploitation_experiment.launch.py, namespaced so the two
# instances don't collide on node name / ~/candidates / proximity_hold_state.
# terrain_relative_z:=false is REQUIRED on flatforest: in terrain mode a vantage
# sits at MAPPED ground + clearance, and the ground on the far side of a trunk is
# exactly what the trunk occludes — those vantages are rejected `noground` and
# targets close PARTIAL with the ring half-driven. The world is a ground_plane at
# z=0, so the fixed absolute height is the same number anyway.
POLAR_ARG="true"; [ "$FRONTIER_ONLY" = "1" ] && POLAR_ARG="false"
DWELL_SYNC_ARG="true"; [ "$DWELL_SYNC" = "0" ] && DWELL_SYNC_ARG="false"
# The control arm. `off` is not a reconnect_mode (see the RECONNECT_MODE block);
# it is rendezvous_enabled:=false, which is the switch the planner actually
# gates the manoeuvre on. reconnect_mode is left at its yaml value in that case
# and is inert, since shouldRendezvous() returns false before the mode is
# consulted.
RDV_ENABLED="true"; MODE_ARG="$RECONNECT_MODE"
if [ "$RECONNECT_MODE" = "off" ]; then RDV_ENABLED="false"; MODE_ARG="hybrid"; fi
log "done_seek_enabled=$DONE_SEEK_ARG done_seek_max_sec=$DONE_SEEK_MAX (DONE_SEEK=$DONE_SEEK)"
log "mission_return_enabled=$MISSION_RETURN_ARG home_tol=${MISSION_HOME_TOL}m max=${MISSION_RETURN_MAX}s (MISSION_RETURN=$MISSION_RETURN)"
log "candidate_enable_polar=$POLAR_ARG (FRONTIER_ONLY=$FRONTIER_ONLY)"
log "proximity_hold/resume_dist_m=$PROX_HOLD_M/$PROX_RESUME_M m (yaml field defaults 5.0/6.0 overridden for sim)"
EXPLOIT_ARG="true"; [ "$EXPLOIT" = "0" ] && EXPLOIT_ARG="false"
# Cell-world args. Built here rather than in the launch loop so the ordered
# team array is constructed ONCE and every robot is handed the identical
# literal — building it per robot invites a future edit that reorders it for
# one of them, and an id that means a different robot on each side is exactly
# the failure the whole known_by scheme cannot detect from inside.
TEAM_NAMES_ARG="["
for r in $ROBOTS; do
  [ "$TEAM_NAMES_ARG" = "[" ] || TEAM_NAMES_ARG="$TEAM_NAMES_ARG,"
  TEAM_NAMES_ARG="$TEAM_NAMES_ARG\"$r\""
done
TEAM_NAMES_ARG="$TEAM_NAMES_ARG]"
# Markers follow RVIZ: the planner gates publication on subscriber count, so
# asking for them headless costs a publisher and nothing else, but it also puts
# a topic on the graph that no run needs.
CELL_MARKERS_ARG="false"; [ "$RVIZ" = "1" ] && CELL_MARKERS_ARG="true"
if [ "$CELL_WORLD" = "1" ]; then
  log "cell world ON: team=$TEAM_NAMES_ARG cell_size=${CELL_SIZE_M}m census_period=${CELL_CENSUS_S}s markers=$CELL_MARKERS_ARG"
  log "  status thresholds: covered<=${CELL_COVERED_U} release>${CELL_EXPLORING_U} frontier_frac<=${CELL_FRONTIER_FRAC} (world-calibrated, cf. done_unknown_fraction=$DONE_UNKNOWN; library defaults 0.15/0.35 assume a saturating map and promote nothing here)"
  log "  ROI [-$ROI_HALF,$ROI_HALF]^2 at ${CELL_SIZE_M}m -> $(awk -v h="$ROI_HALF" -v c="$CELL_SIZE_M" 'BEGIN{n=int((2*h)/c); if (n*c < 2*h) n++; printf "%dx%d", n, n}') cells"
else
  log "cell world OFF (CELL_WORLD=0) — no cell params passed, planner is the pre-M-TARE binary at defaults"
fi
log "exploitation_enabled=$EXPLOIT_ARG (EXPLOIT=$EXPLOIT)"
log "exploit_dwell_sync_enabled=$DWELL_SYNC_ARG (DWELL_SYNC=$DWELL_SYNC)"
log "reconnect_mode=$MODE_ARG rendezvous_enabled=$RDV_ENABLED (RECONNECT_MODE=$RECONNECT_MODE)"
log "reconnect gates: confirm=${RECONNECT_CONFIRM}s barrier_max_wait=${RDV_MAX_WAIT}s"
log "roi x,y = [-$ROI_HALF, $ROI_HALF] (sim override; yaml carries the field site's ROI)"
# done_coverage_source is pinned to scovox, NOT left on "auto". auto switches to
# the 2D planning_map the instant one is received, so enabling the planning map
# would have silently swapped the termination metric from 2.5D column coverage
# of the ROI to 2D cell coverage — a different number against the same
# done_unknown_fraction threshold, changing when every run ends and invalidating
# any comparison with runs recorded before this change.
log "planning_map = /<r>/dscovox_node/global_planning_map (FUSED team map, ${PLAN_MAP_SIZE}m @ ${PLAN_MAP_RES}m/cell), done_coverage_source=scovox"

# --- 6a. run manifest -------------------------------------------------------
# Everything that distinguishes this run from another one, written INTO the run
# directory. Until now the arm, the seed and the radio severity existed only as
# log() lines on the harness's own stdout — so across a 40-run matrix the label
# for each run lived in the operator's scrollback and nowhere else, and two runs
# that differed only in RECONNECT_MODE were indistinguishable after the fact.
# Git hashes with a dirty marker matter just as much: every repo here has
# uncommitted changes, so a bare commit id would be an actively misleading
# provenance record.
MANIFEST="$OUTDIR/run_manifest.txt"
{
  echo "# hmr_explo comms/reconnection run manifest"
  echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "host=$(hostname)"
  echo "outdir=$OUTDIR"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
  # Both isolation keys, because a parallel campaign is only trustworthy if each
  # cell can be shown after the fact to have been isolated. "unset" here means
  # the cell ran unscoped, which is correct sequentially and a defect in a
  # parallel batch -- so the manifest must be able to say which it was.
  echo "ign_partition=${IGN_PARTITION:-unset}"
  echo
  echo "# --- arm / independent variables ---"
  # Per-robot JSONL event stream: the run's primary record. The manifest names
  # it so a reader knows to look for it (and knows it is missing if it is).
  echo "experiment_log=<robot>.events.jsonl"
  echo "reconnect_mode_requested=$RECONNECT_MODE"
  echo "reconnect_mode_param=$MODE_ARG"
  echo "rendezvous_enabled=$RDV_ENABLED"
  echo "rendezvous_max_wait_sec=$RDV_MAX_WAIT"
  echo "reconnect_confirm_sec=$RECONNECT_CONFIRM"
  echo "reconnect_midrun_silence_sec=$MIDRUN_SILENCE"
  echo "reconnect_midrun_max_wait_sec=$MIDRUN_MAX_WAIT"
  echo "reconnect_midrun_max_attempts=$MIDRUN_MAX_ATTEMPTS"
  # Stamped for every cell, both sides. The arm has to be recoverable from the
  # cell itself -- a campaign script can be edited after the fact, a manifest
  # written at launch cannot.
  echo "done_seek_enabled=$DONE_SEEK_ARG"
  echo "done_seek_max_sec=$DONE_SEEK_MAX"
  # Mission return changes what a run's endpoints MEAN (see the MISSION_RETURN
  # block above): a resume must never mix cells across this switch, and an
  # analysis must refuse to pool them.
  echo "mission_return_enabled=$MISSION_RETURN_ARG"
  echo "mission_home_tol_m=$MISSION_HOME_TOL"
  echo "mission_return_max_sec=$MISSION_RETURN_MAX"
  echo "reconnect_min_share_voxels=$RECONNECT_MIN_SHARE_VOX"
  echo "reconnect_midrun_min_silence_sec=$MIDRUN_MIN_SILENCE"
  echo "reconnect_midrun_max_silence_sec=$MIDRUN_MAX_SILENCE"
  # Which CLOCK the mid-run trigger fired on. This is a behavioural switch, not
  # a tuning knob: a link_gate=1 run and a link_gate=0 run are different
  # experiments and their fire times are not comparable. Recorded per run so no
  # future readout has to infer it from a git hash — two of the hashes on the
  # last campaign were '-dirty' and could not have answered this.
  echo "link_gate=$LINK_GATE"
  # link_gate is the REQUEST, link_gate_effective is what the planner got. They
  # differ only when the gate was asked for with no emulator to feed it, and a
  # reader comparing two campaigns needs the second one. g8r1 is why: its
  # manifest honestly said link_gate=0 and the only trace at the firing site was
  # a "radio down -1s" sentinel that reads like a measurement rather than like a
  # check that never ran.
  echo "link_gate_effective=$LINK_GATE_EFFECTIVE"
  echo "link_gate_topic=${LINK_GATE_TOPIC:-none}"
  echo "link_gate_index_topic=${LINK_GATE_INDEX:-none}"
  echo "link_gate_stale_sec=$LINK_GATE_STALE"
  echo "reconnect_link_down_confirm_sec=$LINK_DOWN_CONFIRM"
  echo "reconnect_release_confirm_sec=$RECONNECT_RELEASE_CONFIRM"
  echo "reconnect_arrive_tol_m=$RECONNECT_ARRIVE_TOL"
  echo "reconnect_nav_max_sec=$RECONNECT_NAV_MAX"
  echo "pursuit_staleness_max_sec=$PURSUIT_STALENESS"
  echo "pursuit_budget_max_sec=$PURSUIT_BUDGET_MAX"
  echo "pursuit_goal_stale_sec=$PURSUIT_GOAL_STALE"
  echo "pursuit_explore_fallback=$PURSUIT_EXPLORE_FALLBACK_ARG"
  echo "pursuit_explore_max=$PURSUIT_EXPLORE_MAX"
  echo "hold_escalate=$HOLD_ESCALATE_ARG"
  echo "hold_escalate_wait_sec=$HOLD_ESCALATE_WAIT"
  echo "comms=$COMMS"
  echo "seed=$SEED"
  echo "tx_power_dbm=$TX_POWER"
  echo "tree_attenuation_db=$TREE_ATTEN"
  # The arm's label for the outage gate, recorded because it is a claim about
  # what this run was FOR, not something recoverable from tx_power_dbm alone.
  echo "expect_outage=$EXPECT_OUTAGE"
  # The M-TARE layer. Recorded even when off, because "this run had the knob
  # and it was 0" and "this run predates the knob" are different facts and the
  # binary hash alone cannot separate them within a generation.
  echo "cell_world=$CELL_WORLD"
  echo "cell_size_m=$CELL_SIZE_M"
  echo "cell_census_period_s=$CELL_CENSUS_S"
  # World-calibrated, so they define what COVERED means on this run and a
  # census is not comparable across two runs that disagree about them.
  echo "cell_covered_max_unknown=$CELL_COVERED_U"
  echo "cell_exploring_min_unknown=$CELL_EXPLORING_U"
  echo "cell_covered_max_frontier_frac=$CELL_FRONTIER_FRAC"
  echo "team_robot_names=$TEAM_NAMES_ARG"
  echo
  echo "# --- held fixed ---"
  echo "relay_queue_max_bytes=$RELAY_QUEUE_BYTES"
  echo "scenario=$SCENARIO"
  echo "exploitation_enabled=$EXPLOIT_ARG"
  echo "dwell_sync=$DWELL_SYNC_ARG"
  echo "candidate_enable_polar=$POLAR_ARG"
  echo "max_steps=$MAX_STEPS"
  echo "duration_s=$DURATION_S"
  echo "roi_half=$ROI_HALF"
  echo "plan_map_size_m=$PLAN_MAP_SIZE"
  echo "plan_map_res_m=$PLAN_MAP_RES"
  echo "cost_grid_radius_cap_m=$COST_CAP"
  echo "candidate_min_goal_dist_m=$MIN_GOAL_DIST"
  echo "utility_cost_exponent=$UTIL_GAMMA"
  echo "frontier_z_lo_offset_m=$FRONTIER_Z_LO_OFF"
  echo "frontier_z_hi_offset_m=$FRONTIER_Z_HI_OFF"
  echo "visited_goal_radius_m=$VISITED_RADIUS"
  echo "visited_goal_ttl_sec=$VISITED_TTL"
  # The failed-goal pair was NOT in this manifest before generation 5, which is
  # precisely how a 60 s TTL against a 180 s nav budget survived a whole
  # campaign without anyone being able to see it in a cell's own record.
  echo "failed_goal_ttl_sec=$FAILED_TTL"
  echo "failed_goal_retire_after=$FAILED_RETIRE"
  echo "failed_goal_radius_m=$FAILED_RADIUS"
  # The nav budget and the no-progress abort. The run's end_reason vocabulary is
  # denominated in these, so a cell that omits them cannot be read on its own.
  echo "nav_min_timeout_sec=$NAV_MIN_TIMEOUT"
  echo "nav_max_timeout_sec=$NAV_MAX_TIMEOUT"
  echo "progress_window_sec=$PROGRESS_WINDOW"
  echo "progress_min_distance_m=$PROGRESS_MIN_DIST"
  echo "return_approach_window_sec=$RETURN_APPROACH_WINDOW"
  echo "return_approach_min_m=$RETURN_APPROACH_MIN"
  echo "return_escape_max_attempts=$RETURN_ESCAPE_MAX"
  echo "return_escape_leg_sec=$RETURN_ESCAPE_LEG"
  echo "voxel_resolution_m=$VOXEL_RES"
  echo "done_unknown_fraction=$DONE_UNKNOWN"
  echo "done_criterion=$DONE_CRITERION"
  echo "done_coverage_source=scovox"
  echo "prox_hold_m=$PROX_HOLD_M"
  echo "prox_resume_m=$PROX_RESUME_M"
  echo "targets=$TARGETS"
  echo "record=$RECORD"
  echo
  # Number of trunks the radio model can actually see. Recorded because it is
  # NOT recoverable from the commit sha while a change is uncommitted: two runs
  # either side of a fix to the tree matcher carry byte-identical "<sha>-dirty"
  # provenance but different link statistics. link_states col 3 gives per-link
  # counts, never the world total.
  if [ "$COMMS" = "1" ]; then
    echo "comms_trees_loaded=$(sed -n 's/.*Loaded \([0-9]\+\) tree positions.*/\1/p' \
      "$OUTDIR/comms.log" 2>/dev/null | head -1)"
  fi
  echo
  echo "# --- provenance (dirty = uncommitted changes present) ---"
  for repo in "$WS/.." "$WS/src/explo_planner" "$WS/src/hmr_sim" \
              "$WS/src/simple_nav_3d" "$WS/src/scovox"; do
    name=$(basename "$(cd "$repo" && pwd)")
    if git -C "$repo" rev-parse --git-dir >/dev/null 2>&1; then
      sha=$(git -C "$repo" rev-parse --short HEAD 2>/dev/null || echo unknown)
      dirty=""
      if [ -n "$(git -C "$repo" status --porcelain 2>/dev/null)" ]; then
        # A bare "-dirty" flag cannot tell two different uncommitted trees apart,
        # which is exactly the case during a build session. Hash the diff (plus
        # untracked file names) so each working-tree state gets its own id.
        dirty="-dirty.$( { git -C "$repo" diff HEAD; \
          git -C "$repo" ls-files --others --exclude-standard; } 2>/dev/null \
          | sha1sum | cut -c1-8 )"
      fi
      echo "git_$name=$sha$dirty"
    else
      echo "git_$name=not-a-repo"
    fi
  done
  # The commit hashes above identify the SOURCE; this identifies what actually
  # ran. p7modes recorded four planner commits for one binary, and settling
  # that after the fact took six pairwise diffs plus an mtime that turned out
  # to be a symlink's (this is a symlink-install workspace: the executable
  # lives in build/, and `stat` does not dereference by default). Hash the
  # binary and the question becomes a lookup.
  planner_bin="$WS/install/explo_planner/lib/explo_planner/explo_planner_node"
  if [ -e "$planner_bin" ]; then
    echo "sha256_explo_planner_node=$(sha256sum -b "$planner_bin" 2>/dev/null \
      | cut -c1-16)"
    echo "mtime_explo_planner_node=$(stat -Lc '%y' "$planner_bin" 2>/dev/null)"
  else
    echo "sha256_explo_planner_node=missing"
  fi
  # And the params file, for the same reason — it is the OTHER half of what
  # the node actually ran, and it was invisible in the run record.
  #
  # The node loads it from the INSTALL tree, not from source, so a stale or
  # hand-edited installed copy silently changes behaviour with the binary hash
  # unmoved. done_action lives only here: it must be "idle" for the rendezvous
  # barrier, and the node's own default is "shutdown", so a params file that
  # failed to install turns the barrier off while every other provenance field
  # in this manifest still matches.
  planner_params="$WS/install/explo_planner/share/explo_planner/config/shared_params.yaml"
  if [ -e "$planner_params" ]; then
    echo "sha256_shared_params=$(sha256sum -b "$planner_params" 2>/dev/null \
      | cut -c1-16)"
    # Accept quoted OR bare scalars. The probe used to require double quotes,
    # so `done_action: idle` — valid YAML, and what a hand-edited params file
    # usually looks like — recorded "missing" while the parameter was present
    # and load-bearing. A provenance probe that reports absence for a formatting
    # choice is worse than no probe: it trains the reader to ignore the key.
    echo "done_action_in_params=$(sed -n \
      's/^[[:space:]]*done_action:[[:space:]]*"\{0,1\}\([^"#]*\)"\{0,1\}.*/\1/p' \
      "$planner_params" 2>/dev/null | head -1 | sed 's/[[:space:]]*$//')"
  else
    # BOTH keys, not just the hash. Dropping done_action_in_params here made a
    # missing params file the one case where the field vanishes from the
    # manifest entirely — and any consumer that greps for the key gets no line,
    # which is indistinguishable from an OLD manifest written before the key
    # existed. The failure it is meant to expose (barrier silently off because
    # the params did not install) is exactly this failure, so the key has to be
    # present and say so.
    echo "sha256_shared_params=missing"
    echo "done_action_in_params=missing"
  fi
} > "$MANIFEST"
log "run manifest written: $MANIFEST"
# Deferred from the defaults block, where log() does not exist yet. The pairing
# below is the one combination that is silently unsafe rather than merely inert:
# MIDRUN_SILENCE now defaults BELOW the ~180 s heartbeat-suppression tail on the
# understanding that the veto can tell a quiet teammate from an absent one, so
# with no emulator the low threshold runs bare and a healthy in-range partner
# deep in a PLAN loop reads as missing. The planner warns too; this says it
# before the run rather than in a per-node log nobody opens until afterwards.
if [ "$LINK_GATE" = "1" ] && [ "$LINK_GATE_EFFECTIVE" != "1" ]; then
  log "WARNING: LINK_GATE=1 but COMMS=$COMMS -- no link emulator, so the veto" \
      "CANNOT run and this run fires on the bare record-age clock with" \
      "reconnect_midrun_silence_sec=$MIDRUN_SILENCE. Set COMMS=1, or raise" \
      "MIDRUN_SILENCE above 200, or pass LINK_GATE=0 to say you meant it."
fi
# Built once, outside the loop. Empty unless the caller set something, and
# expanded with the ${a[@]+"${a[@]}"} guard because `set -u` treats an empty
# array expansion as an unbound variable on bash < 4.4.
FOV_ARGS=()
if [ -n "$FOV_HFOV" ];   then FOV_ARGS+=( -p fov_hfov:="$(flt "$FOV_HFOV")" ); fi
if [ -n "$FOV_VFOV" ];   then FOV_ARGS+=( -p fov_vfov:="$(flt "$FOV_VFOV")" ); fi
if [ -n "$FOV_H_RAYS" ]; then FOV_ARGS+=( -p fov_h_rays:="$FOV_H_RAYS" ); fi
if [ -n "$FOV_V_RAYS" ]; then FOV_ARGS+=( -p fov_v_rays:="$FOV_V_RAYS" ); fi
if [ -n "$CAND_N_YAW" ]; then FOV_ARGS+=( -p candidate_n_yaw:="$CAND_N_YAW" ); fi
if [ ${#FOV_ARGS[@]} -gt 0 ]; then
  log "EIG sensor-model OVERRIDE: ${FOV_ARGS[*]}"
  echo "fov_override=${FOV_ARGS[*]}" >> "$MANIFEST"
else
  echo "fov_override=none (shared_params.yaml)" >> "$MANIFEST"
fi
for r in $ROBOTS; do
  # COMMS=1 splits the intent stream: publish to a per-robot topic the emulator
  # can see, subscribe to the relayed copy of the peer's. Both defaults are
  # empty, so with COMMS=0 nothing is passed and the planner keeps using the one
  # shared /exploration/intents bus exactly as before.
  EXTRA=()
  if [ "$COMMS" = "1" ]; then
    peer=$(peer_of "$r")
    EXTRA=( -p coord_intent_pub_topic:=exploration/intents
            -p coord_intent_sub_topics:="[\"rx/$peer/exploration/intents\"]" )
    log "$r intents: pub /$r/exploration/intents  sub /$r/rx/$peer/exploration/intents"
  fi
  # Link-state gate. Appended to the same array rather than passed as bare -p
  # flags because the "off" value is the empty string, and `-p name:=` with
  # nothing after it is not a parameter assignment ros2 will accept — an unset
  # gate has to mean "pass no parameter at all", not "pass an empty one".
  if [ -n "$LINK_GATE_TOPIC" ]; then
    EXTRA+=( -p comms_link_states_topic:="$LINK_GATE_TOPIC"
             -p comms_link_robot_index_topic:="$LINK_GATE_INDEX"
             -p comms_link_stale_sec:="$LINK_GATE_STALE" )
    log "$r link gate: $LINK_GATE_TOPIC (index $LINK_GATE_INDEX, stale ${LINK_GATE_STALE}s)"
  fi
  # Coarse cell world. team_robot_names travels with it and is the SAME
  # ordered list on every robot — a robot's id is its index in this array, so
  # two robots given different orderings would disagree about which mask bit
  # means whom while both looking perfectly healthy on their own.
  if [ "$CELL_WORLD" = "1" ]; then
    EXTRA+=( -p team_robot_names:="$TEAM_NAMES_ARG"
             -p cell_world_enable:=true
             -p cell_size_m:="$CELL_SIZE_M"
             -p cell_census_period_s:="$CELL_CENSUS_S"
             -p cell_covered_max_unknown:="$CELL_COVERED_U"
             -p cell_exploring_min_unknown:="$CELL_EXPLORING_U"
             -p cell_covered_max_frontier_frac:="$CELL_FRONTIER_FRAC"
             -p publish_cell_markers:="$CELL_MARKERS_ARG" )
  fi
  start planner_$r "$OUTDIR/planner_$r.log" \
    ros2 run explo_planner explo_planner_node --ros-args \
      -r __ns:=/$r -r __node:=explo_planner \
      --params-file "$PLANNER_SHARE/config/shared_params.yaml" \
      -p use_sim_time:=true -p robot_name:=$r -p max_steps:=$MAX_STEPS \
      -p terrain_relative_z:=false \
      -p experiment_log_path:="$OUTDIR/$r.events.jsonl" \
      -p candidate_enable_polar:=$POLAR_ARG \
      -p proximity_hold_dist_m:=$PROX_HOLD_M \
      -p proximity_resume_dist_m:=$PROX_RESUME_M \
      -p exploit_dwell_sync_enabled:=$DWELL_SYNC_ARG \
      -p reconnect_mode:=$MODE_ARG \
      -p rendezvous_enabled:=$RDV_ENABLED \
      -p rendezvous_max_wait_sec:=$RDV_MAX_WAIT \
      -p reconnect_confirm_sec:=$RECONNECT_CONFIRM \
      -p reconnect_link_down_confirm_sec:=$LINK_DOWN_CONFIRM \
      -p reconnect_midrun_silence_sec:=$MIDRUN_SILENCE \
      -p reconnect_midrun_max_wait_sec:=$MIDRUN_MAX_WAIT \
      -p reconnect_midrun_max_attempts:=$MIDRUN_MAX_ATTEMPTS \
      -p done_seek_enabled:=$DONE_SEEK_ARG \
      -p done_seek_max_sec:=$DONE_SEEK_MAX \
      -p mission_return_enabled:=$MISSION_RETURN_ARG \
      -p mission_home_tol_m:=$MISSION_HOME_TOL \
      -p mission_return_max_sec:=$MISSION_RETURN_MAX \
      -p reconnect_min_share_voxels:=$RECONNECT_MIN_SHARE_VOX \
      -p reconnect_midrun_min_silence_sec:=$MIDRUN_MIN_SILENCE \
      -p reconnect_midrun_max_silence_sec:=$MIDRUN_MAX_SILENCE \
      -p reconnect_release_confirm_sec:=$RECONNECT_RELEASE_CONFIRM \
      -p reconnect_arrive_tol_m:=$RECONNECT_ARRIVE_TOL \
      -p reconnect_nav_max_sec:=$RECONNECT_NAV_MAX \
      -p pursuit_staleness_max_sec:=$PURSUIT_STALENESS \
      -p pursuit_budget_max_sec:=$PURSUIT_BUDGET_MAX \
      -p pursuit_goal_stale_sec:=$PURSUIT_GOAL_STALE \
      -p pursuit_explore_fallback:=$PURSUIT_EXPLORE_FALLBACK_ARG \
      -p pursuit_explore_max:=$PURSUIT_EXPLORE_MAX \
      -p hold_escalate:=$HOLD_ESCALATE_ARG \
      -p hold_escalate_wait_sec:=$HOLD_ESCALATE_WAIT \
      -p exploitation_enabled:=$EXPLOIT_ARG \
      -p rendezvous_expected_peers:=1 \
      -p roi_min_x:=-$ROI_HALF -p roi_max_x:=$ROI_HALF \
      -p roi_min_y:=-$ROI_HALF -p roi_max_y:=$ROI_HALF \
      -p use_planning_map:=true \
      -p planning_map_topic:=/$r/dscovox_node/global_planning_map \
      -p done_coverage_source:=scovox \
      -p cost_grid_radius_cap_m:=$COST_CAP \
      -p candidate_min_goal_dist_m:=$MIN_GOAL_DIST \
      -p utility_cost_exponent:=$UTIL_GAMMA \
      -p map_resolution:=$VOXEL_RES \
      -p done_unknown_fraction:=$DONE_UNKNOWN \
      -p done_criterion:=$DONE_CRITERION \
      -p frontier_z_lo_offset_m:=$FRONTIER_Z_LO_OFF \
      -p frontier_z_hi_offset_m:=$FRONTIER_Z_HI_OFF \
      -p visited_goal_radius_m:=$VISITED_RADIUS \
      -p visited_goal_ttl_sec:=$VISITED_TTL \
      -p failed_goal_ttl_sec:=$FAILED_TTL \
      -p failed_goal_retire_after:=$FAILED_RETIRE \
      -p failed_goal_radius_m:=$FAILED_RADIUS \
      -p nav_min_timeout_sec:=$NAV_MIN_TIMEOUT \
      -p nav_max_timeout_sec:=$NAV_MAX_TIMEOUT \
      -p progress_window_sec:=$PROGRESS_WINDOW \
      -p progress_min_distance_m:=$PROGRESS_MIN_DIST \
      -p return_approach_window_sec:=$RETURN_APPROACH_WINDOW \
      -p return_approach_min_m:=$RETURN_APPROACH_MIN \
      -p return_escape_max_attempts:=$RETURN_ESCAPE_MAX \
      -p return_escape_leg_sec:=$RETURN_ESCAPE_LEG \
      ${FOV_ARGS[@]+"${FOV_ARGS[@]}"} \
      ${EXTRA[@]+"${EXTRA[@]}"} \
      -p output_csv:="$OUTDIR/planner_$r.csv"
done
# Scheduler last; its node name must stay target_scheduler in the root namespace
# or the targets yaml's parameter key will not match. Skipped entirely with
# EXPLOIT=0: exploitation_enabled=false already makes the planners ignore
# releases, but leaving the scheduler running would still put TreeTarget traffic
# on the ungated global /exploration/targets bus during a comms run.
if [ "$EXPLOIT" = "0" ]; then
  log "EXPLOIT=0 — pure exploration, target_scheduler NOT started"
else
  start sched "$OUTDIR/sched.log" \
    ros2 run explo_planner target_scheduler_node --ros-args \
      -r __node:=target_scheduler \
      --params-file "$TARGETS" -p use_sim_time:=true
fi
sleep 8
# Count real node binaries only, by the absolute install path that ONLY the
# launched binary carries. Matching the bare node name instead counts anything
# whose cmdline merely mentions it: the `ros2 run` python wrapper (which is why
# "bin/ros2" used to be filtered out), but also any shell watching this run from
# another terminal — a `pgrep -f explo_planner_node` in a sibling process aborts
# the whole stack here, 8 seconds after a healthy start.
# The leading [l] is load-bearing, not a typo: ps and grep run concurrently in
# this pipeline, so ps lists the grep too. A literal pattern matches the grep's
# OWN cmdline and every count comes back one too high. Bracketing one character
# makes the pattern text differ from the text it matches.
# count_own, not a machine-wide `grep -c`: see its definition. The [l] bracket
# stays load-bearing there for the same reason it was here.
NPLAN=$(count_own "[l]ib/explo_planner/explo_planner_node")
[ "$NPLAN" = 2 ] || die "expected exactly 2 explo_planner_node (own cell), found $NPLAN"
log "planners up (exactly 2 explo_planner_node)"

# --- 6b. comms gates (COMMS=1) ----------------------------------------------
# Run AFTER the planners, because two of the four can only be judged once the
# intent endpoints exist. GATES_STRICT=1 aborts the run on a failed bring-up
# check; the default reports and continues, because a leak found at t=0 is
# still worth seeing the run for. The watcher keeps polling overflow and the
# odom watchdog for the whole run and its verdict lands in the report file —
# an overflow at minute 40 invalidates the run just as surely as one at t=0.
# Strict by default whenever the emulator is in the loop (inert with COMMS=0).
# Every gate here detects a condition the plan treats as run-invalidating —
# intent leakage past the radio model, a QoS mismatch that silently forms no
# relay, reliable-backlog overflow, a dead odom source. Continuing past one does
# not produce a degraded run, it produces a run that measures the wrong thing
# while looking healthy, and an hour of sim time is more expensive than a
# restart. GATES_STRICT=0 still forces the old report-and-continue behaviour.
GATES_STRICT="${GATES_STRICT:-$COMMS}"
if [ "$COMMS" = "1" ]; then
  GATE_REPORT="$OUTDIR/comms_gates.txt"
  ROBOT_CSV=$(echo $ROBOTS | tr ' ' ',')
  log "running comms bring-up gates (report: $GATE_REPORT)"
  # Capture the GATE's status, not the pipeline's. `if cmd | tee ...; then`
  # tests tee's exit status, which is 0 unless the disk fills — so every gate
  # failure announced itself as success and GATES_STRICT was dead code. This
  # script runs `set -u` without `pipefail`, and adding pipefail globally would
  # change the failure semantics of every other pipeline in here (several
  # legitimately end in `|| true` grep counts), so the status is taken directly
  # instead.
  python3 "$HERE/comms_gates.py" check --robots "$ROBOT_CSV" \
      --report "$GATE_REPORT" >"$OUTDIR/gates_check.out" 2>&1
  GATE_RC=$?
  cat "$OUTDIR/gates_check.out" | tee -a "$OUTDIR/gates.log"
  if [ "$GATE_RC" = 0 ]; then
    log "comms gates: all clear"
  else
    log "WARNING comms gates FAILED (rc=$GATE_RC) — see $GATE_REPORT"
    [ "$GATES_STRICT" = "1" ] && die "comms gates failed rc=$GATE_RC (GATES_STRICT=1)"
  fi
  # EXPECT_OUTAGE=0 for the plan's phase-1 control arm: it runs through the
  # emulator (so the relay hop, delay_ms and rx QoS are all present) at a
  # tx_power_dbm where the link is meant to stay up, and the outage gate would
  # otherwise fail every single control run for behaving as designed.
  # The link trace itself now starts with the emulator, up in §2 — see the note
  # there. Only the gate watcher, which needs the planners' intent endpoints to
  # exist before it can judge anything, still starts here.
  start gateswatch "$OUTDIR/gates_watch.log" \
    python3 "$HERE/comms_gates.py" watch --robots "$ROBOT_CSV" \
      --report "$GATE_REPORT" \
      --expect-outage "$([ "$EXPECT_OUTAGE" = 0 ] && echo no || echo yes)"
fi

T0=$(sim_clock)
[ -n "$T0" ] || die "cannot read /clock"
log "sim t0=$T0 — targets release at +120 / +420 / +720 s of the SCHEDULER's clock"
if [ "$DURATION_S" != "0" ]; then
  log "will stop at t_sim=$((T0 + DURATION_S))"
else
  log "no DURATION_S — running until Ctrl-C"
fi

# --- 7. hold, watching component health -------------------------------------
LAST_HB=0
# Step-stall hang detector state. HANG_HB counts 60-sim-second heartbeats, so
# the default aborts after 10 minutes of sim time with no step on EITHER robot.
# Sized well above a slow step: a RETURN_NAV or PURSUE manoeuvre legitimately
# spends minutes without completing one.
# 40 heartbeats x 60 sim-s = 2400 sim-s, NOT the 10 (600 s) this shipped with.
# 600 was set against an observed stall statistic and it collided exactly with a
# CONFIGURED budget. Sized off those budgets rather than off observation,
# because the corpus cannot bound this: it holds no manoeuvre that ran to its
# own limit.
#
# THE FULL LEG, in order. dispatchReconnect starts the CHASE first under
# hybrid, and none of these legs emits "selected goal" (that string has exactly
# one source, doPlan), so none of them advances the counter this gate watches:
#
#   PURSUE       <= PURSUIT_BUDGET_MAX   600 s   (doPursue gives up at budget)
#   RETURN_NAV   <= RECONNECT_NAV_MAX    600 s
#   RETURN_SYNC  <= MIDRUN_MAX_WAIT      240 s
#   pre-dispatch quiet                   ~40 s   (observed)
#                                       ------
#                                       ~1480 s
#
# plus an unbounded-in-principle correction: doProximityHold REFUNDS held time
# to pursue_start_time_, so PURSUE's wall duration can exceed its budget by the
# accumulated hold, up to proximity_max_hold_sec (120 s, never overridden here)
# — call it ~1600 s worst case.
#
# The first version of this comment said 840 s, having simply omitted PURSUE.
# That figure was already falsified by the bank: the longest banked manoeuvre
# is 840.2 s, and the longest interval with NEITHER robot stepping is 1024.8
# sim-s whole-run, both in hybrid cells.
#
# 1024.8 is NOT the figure this gate has to clear, though, and an earlier
# version of this comment quoted 921.6, which reproduces neither number. The
# gate below is disarmed once either robot is DONE (`[ "${DONE_A:-0}" = 0 ]`),
# so what bounds a FALSE ABORT is the longest freeze inside the armed window,
# which is 420.0 sim-s. The whole-run maximum is the larger number and the
# irrelevant one: most of it accrues after a robot has declared, when the gate
# is already asleep and cannot fire. 1800 would have cleared even the
# whole-run maximum, but left only 1.15x over the configured ceiling.
#
# Why the margin has to be generous in this direction specifically: only the
# hybrid arm manoeuvres at all. Across the banked hybrid cells the mid-run
# reconnect logic produced 302 actual dispatches, against 0 across 157 banked
# off cells. (This comment once claimed "422 dispatches", which was the sum of
# three different line counts, and was then "corrected" to a 302 + 113 + 7
# decomposition of that same 422. The decomposition does not hold either: the
# link-gate veto line is RCLCPP_INFO_THROTTLE'd at 30 s, so 113 counts PRINTED
# lines and is a lower bound on vetoes, and 422 is therefore not a total of
# anything. Only the 302 is a count of events, and it is the only number the
# argument needs.) So a false abort is drawn from ONE arm of a
# 30x2 comparison, on the
# primary endpoint. A late abort merely wastes wall time. The base rate of a
# real hang is 0 in 500 banked cells, so the expected cost of the extra 600 s
# is ~zero and it buys 1.5x over the configured ceiling. Still inside
# DURATION=3000, so the gate stays live — the check below enforces that.
HANG_HB="${HANG_HB:-40}"
# ...and say so out loud when it is NOT, because "well inside DURATION" is a
# claim about two numbers that are set independently and never compared. At
# HANG_HB=30 the gate needs 1800 sim-s of frozen steps, so any cell shorter
# than that ships with the hang detector unable to fire even once — inert while
# still printing its armed message, which is the same silent-non-enforcement
# shape the threshold change above exists to fix. Warn rather than abort: a
# short debug run with no hang gate is legitimate, a CAMPAIGN with one is not,
# and this line is what tells the two apart in the console log.
if [ "$DURATION_S" != "0" ] && [ "$((HANG_HB * 60))" -ge "$DURATION_S" ]; then
  log "WARNING: hang gate is INERT — needs $((HANG_HB * 60)) sim-s of frozen" \
      "steps but DURATION_S=$DURATION_S ends the run first. A hung planner" \
      "will run to the censoring horizon instead of aborting early."
fi
# STOP_ON_DONE=1 (default) ends the run once EVERY planner is in DONE, rather
# than burning the rest of DURATION_S on two parked robots. The experiment's
# primary endpoint is a makespan (§5.2), so the moment the last robot finishes
# is the quantity — and with a matrix of 30+ runs, the difference between
# stopping there and stopping at T is most of the campaign's wall clock.
#
# DURATION_S is still the censoring horizon T and still binds: under a degraded
# radio a robot may never reach DONE, which is a censored observation, not a
# hang. Both exits are needed and they mean different things — record which one
# fired, because "ended at T" and "ended at DONE" are different data points.
#
# Read from the CSV's `state` column, not the log: DONE is a state the planner
# can also LEAVE (a target release pulls it back into the exploit sub-loop, and
# finishOrRendezvous routes through RETURN_NAV/RETURN_SYNC before it), so a
# one-shot "Exploration complete" log line is not terminal and grepping for it
# would stop the run mid-manoeuvre. The column is located by header name because
# it was appended to a schema that other scripts read positionally.
STOP_ON_DONE="${STOP_ON_DONE:-1}"
# Grace, in sim seconds, between all-DONE and teardown: lets the last metrics
# rows land, the reliable backlog drain, and the gate watcher see the final
# state. Without it the run ends inside the very merge the endpoint measures.
DONE_GRACE_S="${DONE_GRACE_S:-30}"
DONE_SINCE=-1
# Last value of the `state` column in a planner CSV, or empty if the file has no
# data rows yet.
planner_state() {
  awk -F, -v want=state '
    NR==1 { for (i=1; i<=NF; i++) if ($i == want) { col=i } ; next }
    col && NF >= col { last=$col }
    END { print last }
  ' "$OUTDIR/planner_$1.csv" 2>/dev/null
}
# Poll cadence, in WALL seconds. Deliberately TWO numbers, not one.
#
# The loop's cheap work -- process liveness, and the planner's own `state`
# column via an awk over a local CSV -- costs effectively nothing, so it runs
# often. That is what tightens the two end-of-run boundaries: the instant every
# robot reaches DONE, and the instant the grace window expires. Both were
# previously detected up to one poll late, and the poll was 15 s, so a run could
# be held open ~15 wall-s at each boundary for nothing.
#
# The expensive work is sim_clock, which spawns `ros2 topic echo /clock --once`
# and costs ~0.3 s of CPU per call (measured, idle machine). Polling THAT every
# POLL_S would be a ~16 % continuous duty cycle per cell, and with concurrent
# shards it would contend with the single render thread that gates sim time --
# trading one bottleneck for another. So the clock keeps its own slower budget
# and is only forced to POLL_S resolution when the answer is about to change:
# on the all-DONE edge, and inside the grace window, which is exactly where the
# resolution is the thing being bought.
POLL_S="${POLL_S:-2}"
CLOCK_EVERY_S="${CLOCK_EVERY_S:-15}"
LAST_SA=-1; LAST_SB=-1; STALL=0
LAST_CLOCK_WALL=0
# Wall-clock deadman on the sim clock itself.
#
# Every other end condition in this loop -- the duration cap, the step-stall
# hang gate, the all-DONE grace -- is keyed on sim time, and the only wall-clock
# test is process liveness. A deadlocked Gazebo passes all of them: the
# processes stay alive, sim_clock keeps returning the same non-empty number, so
# the duration cap is never reached, and the 60-sim-second heartbeat that drives
# the hang gate never ticks either. The loop then polls silently forever. In a
# sequential campaign that does not cost one cell, it costs the night: the
# driver is still inside cell 7 at breakfast and cells 8..60 never started.
#
# Deliberately generous. Sim time can legitimately stall for tens of seconds
# during a heavy lidar frame or a costmap rebuild, and killing a healthy slow
# cell is a worse failure than the one being prevented.
CLOCK_DEADMAN_S="${CLOCK_DEADMAN_S:-420}"
LAST_T_SEEN=-1
LAST_T_WALL=$SECONDS
# A /clock read that returns nothing `continue`s, so it must not be able to spin
# unbounded either: an rmw failure would otherwise look exactly like the frozen
# clock above, minus the log line.
CLOCK_FAIL=0
CLOCK_FAIL_MAX="${CLOCK_FAIL_MAX:-60}"
while true; do
  sleep "$POLL_S"
  for entry in "${PIDS[@]}"; do
    name=${entry%%:*}; pid=${entry##*:}
    # RViz closed by the user is a normal way to end a watch session.
    if [ "$name" = "rviz" ] && ! alive "$pid"; then
      log "RViz exited — tearing down"; exit 0
    fi
    alive "$pid" || die "$name (pid $pid) died mid-run — see $OUTDIR/$name.log"
  done
  # Cheap probe, hoisted ABOVE the clock read. This is the whole point of the
  # fast poll: it answers "is the run over?" without paying for a ros2 spawn.
  # Kept at 0 when STOP_ON_DONE is off so the clock budget below falls through
  # to the slow cadence, i.e. byte-for-byte the old behaviour.
  all_done=0
  if [ "$STOP_ON_DONE" = "1" ]; then
    all_done=1
    for r in $ROBOTS; do
      [ "$(planner_state "$r")" = "DONE" ] || { all_done=0; break; }
    done
  fi
  # $SECONDS is a bash builtin, so this costs no fork. Read the clock on the
  # slow budget, EXCEPT on the all-DONE edge or with a grace window already
  # open -- there the next few seconds decide when the run ends.
  NOW_WALL=$SECONDS
  if [ "$all_done" = 1 ] || [ "$DONE_SINCE" != -1 ] \
     || [ $((NOW_WALL - LAST_CLOCK_WALL)) -ge "$CLOCK_EVERY_S" ]; then
    LAST_CLOCK_WALL=$NOW_WALL
  else
    continue
  fi
  T=$(sim_clock)
  if [ -z "$T" ]; then
    CLOCK_FAIL=$((CLOCK_FAIL + 1))
    log "WARN /clock read failed ($CLOCK_FAIL/$CLOCK_FAIL_MAX), retrying"
    [ "$CLOCK_FAIL" -lt "$CLOCK_FAIL_MAX" ] \
      || die "/clock unreadable for $CLOCK_FAIL consecutive reads — giving up on this cell"
    continue
  fi
  CLOCK_FAIL=0
  # Deadman: sim time itself must advance. See CLOCK_DEADMAN_S above.
  if [ "$T" != "$LAST_T_SEEN" ]; then
    LAST_T_SEEN=$T
    LAST_T_WALL=$NOW_WALL
  elif [ $((NOW_WALL - LAST_T_WALL)) -ge "$CLOCK_DEADMAN_S" ]; then
    die "sim clock frozen at t_sim=$T for $((NOW_WALL - LAST_T_WALL))s wall — declaring this cell hung"
  fi
  if [ $((T - LAST_HB)) -ge 60 ]; then
    LAST_HB=$T
    SA=$(grep -c "selected goal" "$OUTDIR/planner_atlas.log" 2>/dev/null || true)
    SB=$(grep -c "selected goal" "$OUTDIR/planner_bestla.log" 2>/dev/null || true)
    CA=$(grep -c "exploitation COMPLETE" "$OUTDIR/planner_atlas.log" 2>/dev/null || true)
    CB=$(grep -c "exploitation COMPLETE" "$OUTDIR/planner_bestla.log" 2>/dev/null || true)
    log "HB t_sim=$T steps(atlas/bestla)=$SA/$SB complete=$CA/$CB"
    # Hang gate. A planner that cannot find an acceptable candidate returns from
    # doPlan and re-enters PLAN forever: no crash, no error, every process
    # alive, and the CSV keeps growing because the metrics timer samples every
    # metrics_period_sec regardless of state. So neither the liveness loop above
    # nor "is the CSV still being written" can see it — the only quantity that
    # actually stalls is the STEP counter, which advances solely on a completed
    # explore step. A silent hang is worth more than a crash to catch: it yields
    # a full-length run whose coverage curve is flat and plausible.
    #
    # Not fatal if a robot has legitimately finished: DONE-idle is a terminal
    # state by design and its step count is supposed to stop.
    if [ "$SA" = "$LAST_SA" ] && [ "$SB" = "$LAST_SB" ]; then
      STALL=$((STALL + 1))
      # The pattern must match ONLY genuine completion. Through generation 7 the
      # alternate was a bare `DONE`, and every planner log's line 2 reads
      # "DONE-SEEK DISABLED (done_seek_enabled=false, ...)" — a banner announcing
      # a feature is OFF. All 24 g6pilot logs matched it 3 times, so DONE_A was
      # never 0 and this gate could not fire in any run of any campaign that used
      # it. Anchored now on the two prefixes the node emits on a completion
      # path: "Exploration complete" and "Exploration finished". Line numbers
      # deliberately NOT cited — the ones that used to be here had already gone
      # stale by five commits, and a stale pointer in a comment about a silent
      # disarm is worse than no pointer.
      #
      # The first of those is emitted from recordExplorationComplete, which is
      # the single funnel BOTH endings route through (see the comment there).
      # That matters: through generation 7 the step-budget ending printed
      # neither prefix, so the gate stayed armed across the whole homing leg
      # and killed cells at exactly mission_return_max_sec. Generation 8 is the
      # one that FIXED it, by routing both endings through the single funnel —
      # saying "through generation 8" would credit the fix to the generations
      # that still had the bug.
      #
      # The empty-vs-zero handling below is the OTHER silent disarm, and it is
      # subtle in both directions. `grep -c` on an existing file with no match
      # prints "0" and EXITS 1; on a missing file it prints NOTHING and exits 2.
      # So `|| true` alone leaves DONE_A empty for a missing log and `[ "" = 0 ]`
      # is false — a planner that died before creating its log disarmed the gate
      # on the very path it is most needed. But `|| echo 0` is not the fix: on
      # the no-match-but-file-exists case grep's own "0" and the echoed "0" both
      # land, giving "0\n0", and `[ "0 0" = 0 ]` is false too — which disarms it
      # on the PRIMARY hung-run case. Calibrated against all three inputs.
      DONE_PAT="Exploration complete\|Exploration finished"
      DONE_A=$(grep -c "$DONE_PAT" "$OUTDIR/planner_atlas.log" 2>/dev/null || true)
      DONE_B=$(grep -c "$DONE_PAT" "$OUTDIR/planner_bestla.log" 2>/dev/null || true)
      # A gate that aborts valid cells is worse than one that never fires, and
      # the calibration that used to justify 600 s here was WRONG in a way worth
      # recording, because it is the reason a one-arm dropout mechanism shipped:
      #
      #   "the longest interval in which NEITHER robot advanced a step was
      #    59.5 s across the 16 banked cells … a full-length reconnect
      #    manoeuvre never came close, since the two robots do not stall in
      #    lockstep."
      #
      # Three things were wrong with it. (1) 59.5 s is the max gap BETWEEN
      # CONSECUTIVE steps, which cannot see a stall that is never followed by
      # another step — and a manoeuvre that ends in a latch produces exactly
      # that. Recomputed over the window this gate is actually armed in (start
      # -> first DONE prefix), the max is 116.7 s wall / ~101 sim-s. (2) The two
      # largest are g6pilot_hybrid_seed105 and _seed106, in BOTH of which both
      # robots dispatched 8-18 s apart — they stall in lockstep by construction,
      # because they cross the same silence gate on the same shared outage.
      # (3) Every banked manoeuvre was truncated by the coverage latch at
      # 28.5-82.9 s, so "never came close" was measured on a corpus containing
      # no full-length manoeuvre at all.
      #
      # The threshold is therefore sized off the configured budgets (see
      # HANG_HB) rather than off this corpus, which cannot bound it.
      #
      # Nor did it bound the HOMING leg, for a fourth reason: all 24 banked
      # ROBOT-RUNS — 12 cells, two robots each, not 24 cells — ended by LATCH,
      # which prints a matching prefix and disarms this gate before homing
      # starts. Half the corpus this line used to claim. That bound is
      # structural instead, which is why halving it changes nothing — the
      # funnel line is printed on every ending, before startReturnHome.
      if [ "$STALL" -ge "$HANG_HB" ] && [ "${DONE_A:-0}" = 0 ] && [ "${DONE_B:-0}" = 0 ]; then
        die "HUNG: neither planner advanced a step in $((STALL * 60)) sim-s \
(steps still $SA/$SB) and neither reports DONE. Run is invalid — check \
'rejected' counts in $OUTDIR/planner_*.log (cost_grid_radius_cap_m=$COST_CAP)."
      fi
      [ "$STALL" -ge 2 ] && log "WARNING no step progress for $((STALL * 60)) sim-s"
    else
      STALL=0
    fi
    LAST_SA=$SA; LAST_SB=$SB
  fi
  if [ "$STOP_ON_DONE" = "1" ]; then
    # all_done was computed above, before the clock read, so that a fast poll
    # can detect the edge without a ros2 spawn.
    if [ "$all_done" = 1 ]; then
      # Re-armed on any robot leaving DONE, so a target release or a manoeuvre
      # that pulls one back out restarts the grace rather than banking it.
      [ "$DONE_SINCE" = -1 ] && { DONE_SINCE=$T; log "all planners DONE at t_sim=$T — holding ${DONE_GRACE_S}s for backlog drain"; }
      if [ $((T - DONE_SINCE)) -ge "$DONE_GRACE_S" ]; then
        RUN_END_REASON="all_done"
        DONE_DRAIN_COMPLETE=1
        log "run complete: every planner DONE (makespan t_sim=$((DONE_SINCE - T0)) s)"
        break
      fi
    elif [ "$DONE_SINCE" != -1 ]; then
      log "a planner left DONE at t_sim=$T — grace re-armed"
      DONE_SINCE=-1
    fi
  fi
  # The horizon. A run that reaches it while the DONE drain grace is still
  # counting down did NOT get cut off unfinished: every planner had already
  # declared, at DONE_SINCE. Only the ${DONE_GRACE_S}s drain hold spilled past
  # it. Calling that "censored_at_T" labels a completed run as an incomplete
  # one, and it does so ASYMMETRICALLY — the slower arm finishes nearer the
  # horizon, so it collects more of these — which turns a labelling bug into an
  # apparent arm effect.
  #
  # DONE_SINCE is NOT guaranteed to precede the horizon. It is stamped from $T,
  # and $T is only resampled every CLOCK_EVERY_S=${CLOCK_EVERY_S}s of wall time
  # outside the grace path, so the reading that first satisfies the all-DONE
  # condition can already be past T0+DURATION_S. The declaration is still real
  # — every planner is DONE — but "inside the horizon by construction" is not a
  # property this loop maintains, and an earlier version of this comment said
  # it was.
  #
  # The drain is what guarantees the trailing rows are flushed, so a relabel
  # that fires BEFORE it elapsed is promising a clean end the harness did not
  # actually wait for. That is recorded as its own manifest key rather than as
  # a third run_end_reason: gate_g8.py, modes_compare.py and reconnect_value.py
  # all enumerate the two legal reasons, and a new string would silently drop
  # those cells out of every one of them.
  #
  # The primary endpoint reader is immune (event_log.py decides censoring from
  # the planner's own events, never from this string), so this is a fix to the
  # manifest and to everything that reads it, not to the headline result.
  if [ "$DURATION_S" != "0" ] && [ "$T" -ge $((T0 + DURATION_S)) ]; then
    if [ "$DONE_SINCE" != -1 ]; then
      RUN_END_REASON="all_done"
      DONE_DRAIN_ELAPSED=$((T - DONE_SINCE))
      if [ "$DONE_DRAIN_ELAPSED" -ge "$DONE_GRACE_S" ]; then
        DONE_DRAIN_COMPLETE=1
      else
        DONE_DRAIN_COMPLETE=0
      fi
      log "horizon reached at t_sim=+$((T - T0))s, but every planner was already DONE at t_sim=$((DONE_SINCE - T0)) s — the drain grace, not the run, overran (drain ${DONE_DRAIN_ELAPSED}/${DONE_GRACE_S}s, complete=${DONE_DRAIN_COMPLETE})"
    else
      RUN_END_REASON="censored_at_T"
    fi
    break
  fi
done
# Which exit fired is data, not logging: "every robot finished by t" and
# "still unfinished when the horizon cut it off" are different observations and
# the analysis must not average them together.
echo "run_end_reason=${RUN_END_REASON:-censored_at_T}" >> "$OUTDIR/run_manifest.txt"
echo "run_end_t_sim=$((T - T0))" >> "$OUTDIR/run_manifest.txt"
# 1 = the full ${DONE_GRACE_S}s drain elapsed after the last planner declared,
# 0 = all_done was reached but the horizon cut the drain short, empty = the run
# never reached the all-DONE state at all. Emitted unconditionally, including
# the empty case, so that an absent key means an OLD manifest and never "this
# run happened not to drain" — the distinction that made done_action_in_params
# worth fixing in the same file.
echo "done_drain_complete=${DONE_DRAIN_COMPLETE:-}" >> "$OUTDIR/run_manifest.txt"
log "run ended at t_sim=+$((T - T0))s (${RUN_END_REASON:-censored_at_T})"
# teardown runs on EXIT
