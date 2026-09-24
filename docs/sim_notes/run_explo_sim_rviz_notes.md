# run_explo_sim_rviz.sh — design notes and history

The long comments of `sim/run_explo_sim_rviz.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 1
- [flt()](#flt) — 1
- [Top level (part 2)](#top-level-part-2) — 11
- [Mid-run trigger + pursuit-gate family (2026-08-17 redesign)](#mid-run-trigger--pursuit-gate-family-2026-08-17-redesign) — 2
- [Post-latch coast (2026-08-27)](#post-latch-coast-2026-08-27) — 1
- [Mission return (2026-08-27)](#mission-return-2026-08-27) — 1
- [Information gate on the mid-run trigger (2026-08-19)](#information-gate-on-the-mid-run-trigger-2026-08-19) — 10
- [Coarse cell world (M-TARE evolution, P1)](#coarse-cell-world-m-tare-evolution-p1) — 3
- [TeamWorld exchange (M-TARE evolution, P2)](#teamworld-exchange-m-tare-evolution-p2) — 2
- [Team-separation discount (separation.hpp)](#team-separation-discount-separationhpp) — 5
- [Global allocator (M-TARE evolution, P3)](#global-allocator-m-tare-evolution-p3) — 3
- [Utility-gated reconnection (M-TARE evolution, P4)](#utility-gated-reconnection-m-tare-evolution-p4) — 1
- [Scheduled rendezvous (M-TARE evolution, P5)](#scheduled-rendezvous-m-tare-evolution-p5) — 2
- [MDP interception (M-TARE evolution, P6)](#mdp-interception-m-tare-evolution-p6) — 9
- [Link-state gate for the mid-run reconnect trigger (§30.11, §30.24)](#link-state-gate-for-the-mid-run-reconnect-trigger-3011-3024) — 23
- [peers_of()](#peers_of) — 1
- [environment: humble + ws overlay, miniconda stripped](#environment-humble--ws-overlay-miniconda-stripped) — 1
- [stack_procs()](#stack_procs) — 1
- [count_own()](#count_own) — 1
- [teardown()](#teardown) — 7
- [resolve the ambient run-control knobs, before the trap and the manifest](#resolve-the-ambient-run-control-knobs-before-the-trap-and-the-manifest) — 1
- [wait_for()](#wait_for) — 1
- [0. preflight](#0-preflight) — 2
- [2. TF glue + viz model (one each per robot)](#2-tf-glue--viz-model-one-each-per-robot) — 1
- [2b. comms emulator (COMMS=1)](#2b-comms-emulator-comms1) — 2
- [3. nav + lidar mapping, mergers cross-wired](#3-nav--lidar-mapping-mergers-cross-wired) — 4
- [5. optional bag](#5-optional-bag) — 4
- [6. planners (namespaced, one per robot) + one global scheduler](#6-planners-namespaced-one-per-robot--one-global-scheduler) — 6
- [6a. run manifest](#6a-run-manifest) — 25
- [6b. comms gates (COMMS=1)](#6b-comms-gates-comms1) — 4
- [7. hold, watching component health](#7-hold-watching-component-health) — 14

## Top level

### harness-team-size-from-scenario

**Team size and the stack's origin** — attached to `set -u` (line 5)

```text
N comes from the scenario yaml, not from this file. It was 2 for every campaign
up to 2026-09-05 and the script said so in several places; the team-size series
runs 2, 3 and 4, so those literals have been replaced by $ROBOTS/$N_ROBOTS.

This is the 2026-08-02 campaign stack (Bags/2026_08_02__flatforest_2robot_lidar
_ablation/run_experiment.sh, condition B) with the visualization it never had:
that campaign ran headless with no RViz and no robot model, so a run was only
legible after the fact through the CSVs. Here you watch it live.
```

## flt()

### flt-float-param-literals

**Why float knobs go through flt** — attached to `flt() {` (line 82)

```text
Force a decimal point onto anything destined for a `ros2 ... -p x:=<v>` that
the node declares as a double. `ros2 param` infers the type from the LITERAL,
so `-p cost_grid_radius_cap_m:=500` is an integer and the node aborts at
startup with InvalidParameterTypeException; `:=500.0` is what it wants. This
bit twice: awk prints 10*50.0 as "500", and every knob below that a user might
reasonably set as `ROI_HALF=35` feeds -p roi_min_x. Run every float-valued
shell var through this rather than relying on each one being written with a
decimal by hand -- the failure is a hard abort seconds into a run, and it is
only visible in the planner log, not on the console.
Reject non-finite spellings outright: YAML/ROS accept both `nan` and `.nan`
as a double, and `.nan` matches the *.* passthrough below -- it would sail
into the planner and silently disarm every comparison against the value.
Emitting nothing (plus the stderr line) makes the ros2 invocation fail
loudly instead; `exit` here would only kill the $(...) subshell.
```

## Top level (part 2)

### scenario-override-and-calibration

**Scenario choice invalidates calibration** — attached to `SCENARIO="${SCENARIO:-flatforest_2robot_lidar.yaml}"` (line 108)

```text
Overridable so a denser stand can be run without editing the harness. The
scenario picks the WORLD, and the world sets both the link budget (stems in
the Fresnel corridor) and the coverage floor — so a scenario change
invalidates the calibrated done_unknown_fraction and is recorded in the
manifest for exactly that reason.
```

### roster-from-scenario

**Robot roster read from the scenario** — attached to `ROBOTS="$(python3 - "$SCENARIO_PATH" <<'PY'` (line 121)

```text
The roster comes from the SCENARIO, not from a literal here (P7). It used to
be `ROBOTS="atlas bestla"` with a comment telling the reader not to change the
scenario without changing this line, which is a consistency requirement kept
by hand — and the failure mode is bad: comms_sim.launch.py already reads its
own robot list from this same file, so a mismatch produces a run in which the
emulator models three radios while the harness starts two planners and the
third robot sits in the world as an unexplained obstacle. Reading the one
file makes that disagreement unrepresentable.

NOT overridable from the environment, for the same reason: an override could
only ever disagree with the emulator. To change the roster, change (or add) a
scenario.
```

### frontier-only-candidates

**Frontier-only exploration candidates** — attached to `FRONTIER_ONLY="${FRONTIER_ONLY:-1}"` (line 151)

```text
FRONTIER_ONLY=1 (default) overrides shared_params.yaml's candidate_enable_polar
to false, so EXPLORE candidates are ONLY frontier centroids — no polar grid of
n_radial*n_rings*n_yaw = 96 samples around the robot. Exploitation is
unaffected either way: vantages come from VantagePlanner::generateVantages(),
not CandidateGenerator::generate().

CAVEAT, from the shared_params.yaml comment that set the field default to
polar-on: in frontier-only mode the planner has been seen to exhaust reachable
candidates (~70) once the robot pushed into an ROI corner, then oscillate until
the planner timeout fired. Polar exists as the fallback supply. Watch for a
rising `candidates rejected` / repeated identical goals; FRONTIER_ONLY=0
restores the field behaviour.
```

### proximity-yield-sim-band

**Sim-only proximity-yield band** — attached to `PROX_HOLD_M="$(flt "${PROX_HOLD_M:-1.5}")"` (line 164)

```text
Proximity-yield band, SIM ONLY. shared_params.yaml ships 5.0/6.0, sized for the
real-robot reaction budget (peer pose age + tick + nav2 cancel propagation +
braking at 2x0.8 m/s closing). In flatforest the two huskys work a 3-vantage
ring whose angles sit ~3.5 m apart, so a 5 m hold disc means the robot driving
to its angle is braked by the teammate standing on the next one — the yaml
defaults would spend the run in PROXIMITY_HOLD. These override the params file
for this script only; the field defaults stay untouched.
These are doubles in the planner, and `-p x:=2` would make ros2 infer an
integer and abort the node at startup on the type mismatch. flt() adds the
decimal point, so `PROX_HOLD_M=2` is now accepted -- but the same trap is live
for any NEW float knob added below that skips flt().
```

### dwell-sync-barrier

**Vantage-ring dwell sync barrier** — attached to `DWELL_SYNC="${DWELL_SYNC:-1}"` (line 177)

```text
DWELL_SYNC=1 (default) keeps the vantage-ring rendezvous barrier on: a robot at
its vantage holds (dwell clock not started) until every peer claiming the same
trunk is standing on its own angle, so the team captures one trunk state
simultaneously. DWELL_SYNC=0 is the A/B against the old behaviour, where the
first robot to arrive dwells alone and can close the quota before the peer
lands. See exploit_dwell_sync_* in shared_params.yaml.
```

### exploit-off-for-comms-runs

**Why comms runs disable exploitation** — attached to `EXPLOIT="${EXPLOIT:-1}"` (line 184)

```text
EXPLOIT=0 turns the run into pure exploration: exploitation_enabled:=false and
no target_scheduler at all. The comms/reconnection matrix REQUIRES this (plan
§3.5) and the yaml default is true, so a matrix run left on the default would
not be the experiment the plan describes.

Why it contaminates: the scheduler releases three trunks on a sim-time
schedule, both robots detour to each one, and the vantage ring is split
through peer CLAIMS on /exploration/intents — the very stream COMMS=1 gates.
Link severity would then drive exploitation stalls directly, standDownExploitation()
would fire inside every reconnect manoeuvre, and target detours of arm-dependent
length would land in the primary coverage endpoint. /exploration/targets is
also a global bus the emulator does not relay, so the one piece of team
knowledge that stays perfect under a modelled radio outage would be the target
list. Default 1 keeps this script's watchable-demo behaviour unchanged.
```

### reconnect-mode-arm-tokens

**Reconnect mode and arm tokens** — attached to `RECONNECT_MODE="${RECONNECT_MODE:-mtare_hybrid}"` (line 199)

```text
Mesh reconnection manoeuvre (rendezvous | pursuit | hybrid) — which of the
robot-carried-radio reconnection methods runs when a planner exhausts its
goals with its teammate out of comms. The pursuit budgets ride along from
shared_params.yaml (240 s cap / 180 s staleness gate, flatforest-sized).
The planner invocation below also passes rendezvous_expected_peers:=N-1,
computed from the scenario roster: shared_params.yaml ships 0 — the field
value, where the launch file computes team-size-1 — and the planner
hard-disables the whole reconnect feature at startup on expected_peers=0,
which would leave this knob silently inert.
NOTE: stock sim DDS is one broadcast domain, so with both planners healthy
no manoeuvre ever runs: the first finisher parks DONE-idle and keeps
beaconing, the second finisher counts it and goes DONE in place. The A/B
that exercises the mode is a dead peer (kill one planner mid-run: the
survivor's exhaustion finds the claim aged out, chases the corpse's trail,
then falls back per mode) or, later, a range-gated intent bridge emulating
finite comms.

`off` is a FOURTH value handled here, not by the planner: it disables the
manoeuvre outright (reconnect_enabled:=false) and is the control arm the
matrix compares the other three against. It cannot be expressed as a
reconnect_mode — reconnectModeFromString() falls back to RENDEZVOUS on any
string it does not recognise (planner_util.cpp), setting only a `known` flag
that the node reports as a single WARN and then ignores. So passing
`reconnect_mode:=off` would not disable anything; it would silently run the
rendezvous arm twice and the control-vs-treatment contrast would be a
comparison of an arm with itself. Unknown values are rejected below rather
than quietly mapped, for the same reason.

`mtare_hybrid` is a FIFTH value and the same kind of thing as `off`: an ARM,
not a mode. It is `hybrid` with the M-TARE decision stack engaged — the cell
world, the TeamWorld exchange, the global allocator and the §3.6 reconnect
gate — and it is one token rather than four environment variables because the
arm token is the only thing that survives into the directory name, the index
row and every gate's arm parser. Four independent knobs would let a campaign
run cells labelled mtare_hybrid with, say, the allocator off, and nothing
downstream could tell.

P5/P7 add five MORE arm tokens. Four of them — the ones without an `_mdp`
suffix — are together the 2x2 factorial of doc §3.6.1, chase on/off x
appointment on/off:

  token              chase   appointment   reconnect_mode  reconnect_enabled
  mtare_off            -          -          (hybrid)          false
  mtare_pursuit       yes         -           pursuit           true
  mtare_rendezvous     -         yes         rendezvous         true
  mtare_hybrid        yes        yes           hybrid           true

All four run the SAME P1-P3 decision stack (cell world, TeamWorld exchange,
global allocator), so the only levers between them are the two mechanisms.
`mtare_off` carries RECONNECT_GATE=silence rather than `info`, because the
gate can only suppress a dispatch and that arm makes none: pinning it there
would be pinning an inert knob, and the harness refuses the pairing outright
a few blocks below. GLOBAL_ALLOC is emphatically NOT inert in that arm, which
is why the control is `mtare_off` and not plain `off` — a plain-`off` control
would confound the two mechanisms with the allocator.

All four of those pin PURSUIT_PREDICTOR=trail. P6's MDP interception is a THIRD
mechanism, not a third level of either lever, and the factorial has no cell
for it: an mdp run of `mtare_hybrid` would land in a directory named
mtare_hybrid, be indexed as that arm, and be pooled with the trail cells by
every analysis script.

So it gets NAMES instead of an override — `mtare_pursuit_mdp` and
`mtare_hybrid_mdp`, the two arms that actually chase, each the exact stack of
its trail-named counterpart with the predictor swapped:

  token                chase   appointment   predictor
  mtare_pursuit         yes         -          trail
  mtare_pursuit_mdp     yes         -          mdp
  mtare_hybrid          yes        yes         trail
  mtare_hybrid_mdp      yes        yes         mdp

There is deliberately no mdp counterpart for mtare_off or mtare_rendezvous:
neither arms a chase, so the predictor has nothing to aim and the cell would
record a treatment it cannot carry. A P6 comparison is therefore one of two
paired arms against its own trail control, in one invocation, and never a
third level inside the 2x2 — the factorial and the predictor are separate
questions and this keeps them in separate directories.

Six tokens, not eight, and NOT a 2x2x2: chase-on is a precondition of the
predictor, so the third factor is only defined on half the design.

THE DEFAULT IS AN mtare_ TOKEN, and it has to be (2026-09-16). The plain
`rendezvous` and `hybrid` tokens cannot produce a run at all any more, in
either direction:

  RENDEZVOUS_SCHEDULE=0 — the node throws at construction. Both arms ARE the
    agreed meeting, so without the scheduler rendezvous degrades to `off` and
    hybrid to `pursuit`; see explo_planner_node.cpp, the reconnect_mode /
    rendezvous_schedule_enable interlock. The `_rzv_needed` block below is the
    copy that fires before Gazebo starts.
  RENDEZVOUS_SCHEDULE=1 — the arm-stamp guard below refuses. The node builds
    its own arm name and prefixes `mtare_` when ANY of four knobs is on, and
    the scheduler is one of the four. So a scheduled run under a plain name
    stamps `mtare_hybrid` into run_start while every directory and index says
    `hybrid`.

The two together leave the plain tokens with an empty configuration space.
They stay in the vocabulary because they are still the RECONNECT_MODE the
node is passed (MODE_ARG strips the prefix) and because the guard calibration
uses them as its plain-token vehicle — but nothing can run one, so nothing may
default to one. `hybrid` was the default until today, which meant a bare
`./run_explo_sim_rviz.sh` exited 2.
```

### arm-token-pins-stack

**The arm token pins the stack** — attached to `_arm_stack=""` (line 312)

```text
The arm token IS the configuration. Set here, before the CELL_WORLD /
TEAM_WORLD / GLOBAL_ALLOC / RECONNECT_GATE default blocks below read their
environment, so the arm decides and those blocks only validate.

An explicit environment variable that CONTRADICTS the token is refused, not
honoured. `RECONNECT_MODE=mtare_hybrid GLOBAL_ALLOC=0` would otherwise
produce a run in a directory named mtare_hybrid, stamped as a treated cell in
the index, that is missing a quarter of the treatment — and no gate reads
four separate knobs to notice. Refusing costs an operator one re-run; the
silent version costs the campaign.
```

### coord-claim-radius-override

**MinPos claim radius override** — attached to `COORD_CLAIM_R="${COORD_CLAIM_R:-}"` (line 358)

```text
MinPos claim radius override (campaign cr2). Empty = pass no parameter at all,
which leaves the node on the yaml value -- and that value is now PINNED.
shared_params.yaml ships `coord_claim_radius_m: 10.0` (search that key; it is
near the coord_claim_ttl_sec line), so the node's "0 means auto" branch --
`if (coord_claim_radius_m_ <= 0.0) { coord_claim_radius_m_ = fcfg.max_range; }`
in explo_planner_node.cpp, in the auto-resolve block just after the FOV config
is built -- does NOT run. An unset COORD_CLAIM_R is a 10 m disc, which is the
same 10 m every campaign up to and including mt2 got back when auto was still
resolving it. Generation 9 is comparable to those cells on claim radius, and
the pin is exactly what makes that true: D1 moved fov_max_range 10.0 -> 20.0,
and had the yaml still said 0.0 the disc would have silently doubled with the
sensor model. AUTO is a POINTER, not a number -- that is why it is gone.

Two ways to break this, both of which look like housekeeping:
  - "restoring" the 0.0/auto spelling in the yaml, which re-couples the claim
    disc to fov_max_range and hands generation 9 a 20 m disc nobody asked for;
  - setting COORD_CLAIM_R=10 so the manifest "says" 10. That writes
    coord_claim_radius_override=10, which is how a cr2 TREATMENT cell is
    identified, so an untreated cell would be filed as a treated one.
For what the node actually used, read coord_claim_radius_m_in_params (the
resolved yaml value, echoed below); coord_claim_radius_override only records
whether this harness passed -p at all.

Declared HERE, below the _arm_stack block, so it is not one of the knobs an
arm token pins -- the radius is deliberately orthogonal to the arm. It is set
per cell by run_campaign.sh from the "_r<N>" arm-name suffix, which that script
strips before it hands over RECONNECT_MODE, so nothing in this file sees the
suffix and no arm-token guard below has to learn about it.

Validated rather than passed through, and NOT merely for tidiness: this value
is the bound the receiver clamps every peer-advertised claim to. The clamp
itself is Coordination::onIntent (coordination.cpp:56-57); the bound it clamps
against is our own resolved radius, handed to the Coordination ctor at
explo_planner_node.cpp:3738-3741 as coord_claim_radius_m_, which :3440-3441
has already resolved from the 0 = auto spelling to fov_max_range. So a
garbage value does not fail loudly -- it silently changes how much of the
candidate set a peer can veto. A non-finite spelling would reach the node as a
double and disarm MinPos in one direction or veto everything in the other.
```

### coord-claim-radius-zero-is-auto

**Why a zero claim radius is refused** — attached to `if [ "$(awk -v v="$COORD_CLAIM_R" 'BEGIN{print (v+0 > 0.0) ? 1 : 0}')" != "1" ]; then` (line 406)

```text
Reject 0 and anything that rounds to it. Zero is not "no override" here --
the node reads <= 0 as AUTO, so `-p coord_claim_radius_m:=0.0` would quietly
resolve back to fov_max_range and produce a cell named _r0 that ran at the
sensor range (20 m since D1) instead.
Leave COORD_CLAIM_R unset for the yaml default; that path passes no -p.
```

### barrier-max-wait-cap

**Finite terminal barrier cap** — attached to `RDV_MAX_WAIT="$(flt "${RDV_MAX_WAIT:-600}")"` (line 419)

```text
Barrier cap. The planner's code default is 0 = wait forever, which is the
right field behaviour and the wrong experiment: a robot that gives up on the
chase raises the barrier at its current pose and never lowers it, so the run
ends at the horizon and its time-to-reconnect is undefined — in exactly the
arm where the modes differ most. Observed in p4mild_pursuit_seed1: both
robots declined a stale chase, held, and censored the run at 5807 s.
A finite cap makes the ending observable ("gave up after N s") instead of
indistinguishable from "still waiting". Set 0 to restore the field default.
flt() per the warning above: these are doubles in the planner and a bare
integer makes ros2 infer int and abort the node at startup.
```

## Mid-run trigger + pursuit-gate family (2026-08-17 redesign)

### midrun-silence-clock

**Mid-run silence clock and its value** — attached to `MIDRUN_SILENCE="$(flt "${MIDRUN_SILENCE:-90}")"` (line 436)

```text
These now default ON and match the planner's own code defaults, so a bare
invocation of this script runs the same policy the robots run. To reproduce
the p7modes (pre-redesign) behaviour bit-for-bit, export MIDRUN_SILENCE=0
RECONNECT_RELEASE_CONFIRM=0 PURSUIT_STALENESS=180 HOLD_ESCALATE=0.
Every one of these is echoed into the manifest below: the p7modes campaign
taught that an un-recorded planner param makes runs post-hoc
indistinguishable, which the mode comparison then has to treat as a confound.
Silence (s) of continuous peer absence before a robot interrupts exploration
to run its arm's reconnect manoeuvre. 0 = terminal-only (legacy).

90 since generation 9; it was 240 through generation 8. The old value had to
clear the ~180 s heartbeat-suppression tail by pure waiting, because the
record-age clock alone cannot tell a silent teammate from an absent one and
firing at a healthy, silently-planning peer was the failure it had to avoid.
LINK_GATE=1 (now the default, below) makes that distinction directly from the
radio, so the threshold is no longer set by the suppression tail.

Lowering it was forced by measurement, not preference: in g8r1's 23 hybrid
cells the 240 s clock expired in 3, so 87 % of the treated arm ran
behaviourally identical to the control and the campaign could not measure its
own treatment.

TWO DRAFTS OF THIS NUMBER WERE WRONG BEFORE THIS ONE. The first, 120, came
from a circular sweep: it filtered episodes on the presence clock and then
scored them on the same clock, so "zero waste" merely restated the filter. The
second, 90, was re-scored against g8r1's banked link_states.csv with the filter
on the past (presence gap) and the score on the future (radio) -- but on the 20
hybrid cells that did NOT dispatch, which is a sample selected on the outcome
being swept.

The number that stands is scored on the `off` arm, which never ran the trigger
at all and is therefore untreated by construction. There, at LINK_DOWN_CONFIRM
as it now ships (0, i.e. "not up right now"), 90 arms 18 of 23 cells, 20 % of
fires land in an outage that would have closed on its own anyway, and 85 %
beat natural recovery -- the argmin of waste across the UNWALKED grid, where
120, 90 and 75 all arm the same 18 of 23.

NEITHER FACT SURVIVES THE FORWARD WALK, so 90 is not chosen by the grid. Scored
at T plus the measured 14.0-18.8 s dispatch overshoot the waste argmin moves to
75 at every offset and the plateau breaks; a ranking that flips that easily is
one 23 cells cannot resolve, and re-tuning to 75 on the same cells would repeat
the error. What survives is the coarse verdict (240 far too high, 60 past where
waste turns back up); inside 120-75 the choice is off-grid, and 90 clears the
p90 radio outage (52.0-111.0 s, nearest-rank) while sitting far below 240. See
§32.15 and the member-default comment in explo_planner_node.cpp, which carries
the full table and the walked numbers.
```

### midrun-barrier-give-up

**Mid-run barrier give-up** — attached to `MIDRUN_MAX_WAIT="$(flt "${MIDRUN_MAX_WAIT:-30}")"` (line 483)

```text
Barrier give-up for mid-run attempts (terminal barriers keep RDV_MAX_WAIT).

30, NOT THE NODE DEFAULT'S 240 (2026-09-20, Kalhan). What this bounds is a
robot STANDING STILL at the end of a failed mid-run manoeuvre — at a chase's
predicted intercept, or wherever holdForTeam stopped it — waiting for a peer
to walk into range. The node's own explore-fallback comment calls that
strictly dominated ("a moving robot can still regain the link; a parked one
can only be found"), and at 240 it was a twelfth of a cell spent on the
dominated option. Comfortably above RECONNECT_RELEASE_CONFIRM (3 s), so a
peer that does arrive still registers.

WHAT IT COSTS: MIDRUN_MAX_ATTEMPTS is consumed faster. The re-arm cycle is
this cap plus MIDRUN_SILENCE, so 120 s rather than 330, and a robot alone for
a long stretch can spend all 6 attempts by roughly t+720 s and explore
unassisted for the rest of the cell. That is the same trade in the same
direction, which is why it is priced here rather than compensated for.

Terminal barriers keep RDV_MAX_WAIT untouched: a robot there has no
exploration left to go back to, so the dominance argument does not apply.
```

## Post-latch coast (2026-08-27)

### done-seek-post-latch-coast

**Post-latch coast (DONE_SEEK)** — attached to `DONE_SEEK="${DONE_SEEK:-0}"` (line 505)

```text
56 of 88 banked reconnect_end events are a robot that crossed the coverage
latch mid-chase: it brakes, drops the contact, and the partner keeps waiting
at a barrier for someone who is no longer coming. DONE_SEEK=1 keeps the nav
goal the robot already had so it finishes that drive and delivers its map.

It is metric-neutral by construction -- state_ reads DONE from the same tick,
so the all_done check below is untouched -- which is exactly why it has to be
a RUNTIME switch: treated and control cells must sit inside ONE run_campaign
invocation or the arm is confounded with the session (30.27, ~1.08x floor).
Default 0 so an un-set campaign reproduces every banked run.

SUPERSEDED under MISSION_RETURN=1: the mission-return branch in the planner
pre-empts the coast gate at every terminal ending, so the coast is
unreachable there (the homing traverse is itself the go-reconnect behaviour
the coast approximated). DONE_SEEK matters only in MISSION_RETURN=0 runs.
```

## Mission return (2026-08-27)

### mission-return-home

**Mission return to the start pose** — attached to `MISSION_RETURN="${MISSION_RETURN:-0}"` (line 525)

```text
MISSION_RETURN=1 adds an arm-invariant requirement to the mission itself: at
ANY terminal exploration ending (coverage latch, step budget, barrier
give-up) the robot drives back to its recorded start pose. Both arms then
end in the same connected configuration (spawns are 3 m apart), which is
what makes "mission end time" a well-defined primary endpoint in the off
arm too — the run still ends when every planner reads DONE, so the all_done
check below is untouched, but DONE now means "home (or bounded give-up)",
not "parked wherever exploration ended".

NOT metric-neutral for exploration finish: a robot that finishes first now
drives home through the world and can deliver its map to the still-exploring
partner en route, in BOTH arms — that is part of the shared mission
definition, so exploration finish under MISSION_RETURN=1 is a NEW endpoint,
never poolable with banked numbers. Default 0 so an un-set invocation
reproduces every banked run bit-for-bit.
```

## Information gate on the mid-run trigger (2026-08-19)

### midrun-info-gate-sizing

**Information gate on the mid-run trigger** — attached to `RECONNECT_MIN_SHARE_VOX="$(flt "${RECONNECT_MIN_SHARE_VOX:-0}")"` (line 552)

```text
Replaces the fixed MIDRUN_SILENCE clock with "reconnect once the pair has
gathered RECONNECT_MIN_SHARE_VOX of map the other side has not seen", by
dividing that target by the pair's beaconed gathering rate and clamping the
quotient into [MIDRUN_MIN_SILENCE, MIDRUN_MAX_SILENCE].

0 (the default) keeps the legacy fixed clock EXACTLY -- midrunGateSec returns
reconnect_midrun_silence_sec unevaluated -- so every campaign already on disk
reproduces bit-for-bit and this block is inert unless a run asks for it.

Sizing, measured on cg050's 8 cells (gate_forecast.py / gate_verdict.py):
the pair gathers ~9,100 vox/s in the first 200 s and ~2,900 vox/s after, so
  550k / 9,100 = 60 s  -> the floor is reached but never BINDS, which is the
                          point: every firing time is set by the information,
                          not by the clamp (at 400k the floor binds and the
                          arm would be a fixed 60 s clock wearing the gate's
                          name; at 700k a whole cell drops to zero triggers).
  550k / 2,900 = 190 s -> still inside the ceiling, so late outages still fire
                          EARLIER than the 240 s legacy clock.
  550k / 319   = ceiling -> a saturated pair drifting apart slowly is declined,
                          which is the gate doing its job: 200 s of silence at
                          that rate is only ~64k voxels, four chance merges.
MIDRUN_MAX_SILENCE must stay <= MIDRUN_SILENCE: a ceiling above it would let
the gated arm fire LATER than the control and confound "gated vs not" with
"waited longer". The planner warns at startup if it does.

It TRACKS MIDRUN_SILENCE rather than carrying a copied constant. It used to be
a literal 90, which was correct only by coincidence: it matched generation 9's
clock and would have silently violated the invariant the moment either number
moved, in exactly the direction the warning exists to catch.

THE FLOOR NOW BINDS OVER MOST OF THE USABLE RANGE, which the previous wording
denied. WHICH RATE YOU DIVIDE BY DECIDES THE ANSWER, and the previous wording
mixed two in one sentence: it wrote "550k/9,100 = 60 s" (cg050's early-run
rate, above) and then quoted a window derived from a different rate entirely.
The window [138k, 207k] is the ~2,300 vox/s figure the ESTIMATOR was
calibrated on in p14 (explo_planner_node.cpp, reconnect_min_share_voxels_):
138k/2300 = 60 s = the floor, 207k/2300 = 90 s = the ceiling. Under cg050's
rates the same clamps give [546k, 819k] early and [174k, 261k] late. Both are
legitimate; they answer for different phases of a run and differ by ~4x, which
is precisely why the target has to be RE-DERIVED against whichever rate the
next campaign's world actually produces rather than copied from either. Do not
reuse 550k. Both clamps are inert while
RECONNECT_MIN_SHARE_VOX=0, which is the default and what the current campaign
runs -- keeping the invariant true matters for the day the gate is switched on
again, not for these runs.
```

### release-confirm-vs-claim-ttl

**Release confirm must exceed claim TTL** — attached to `RECONNECT_RELEASE_CONFIRM="$(flt "${RECONNECT_RELEASE_CONFIRM:-6}")"` (line 600)

```text
Release flicker guard: the team must read complete this long before a
manoeuvre releases (and the silence clock resets). 0 = first-read (legacy).
MUST exceed coord_claim_ttl_sec (5.0), which the old default of 3 did not:
one packet holds a peer live for the whole TTL, so a 3 s window was satisfied
by that single packet and the guard admitted the very flicker it was written
to reject. The planner warns at startup if this is set at or below the TTL.
```

### reconnect-arrive-tolerance

**Manoeuvre arrival tolerance** — attached to `RECONNECT_ARRIVE_TOL="$(flt "${RECONNECT_ARRIVE_TOL:-1.5}")"` (line 607)

```text
Arrival tolerance for a manoeuvre destination (m), and the ceiling on one
manoeuvre drive leg (s). shared_params.yaml carries the original 4.0 sizing
argument and why it no longer holds here.

1.5 since 2026-09-18: the 4.0 argument assumed a link that carried 8 m with
room to spare, which the pre-2026-09 radio did. Under 70 dB trunks + 30 m
horizon it does not: in ts4_smoke24_n2/rendezvous both robots stopped on
the 4 m ring on opposite sides (5.9 m apart, one trunk on the chord) and
the link's longest up-streak over the final 346 s was 2.0 s against a 6 s
release confirm — an appointment hold that is unbounded BY DESIGN parked
both robots from t~350 to the censor. At 1.5 the pair lands <= 3 m apart.
An obstructed exact point is not a budget burn: the 15 s / 0.2 m
no-progress window hands the leg to RETURN_SYNC from beside it.
```

### pursuit-staleness-gate

**Pursuit staleness and goal-stale gates** — attached to `PURSUIT_STALENESS="$(flt "${PURSUIT_STALENESS:-900}")"` (line 622)

```text
Pursuit gates. The old 180 s staleness vetoed every chase in the dense world
(outage tail 861 s ~ staleness at a terminal trigger), so the chase was dead
code here; 900 clears that tail. What keeps the wider window honest is
PURSUIT_GOAL_STALE: past 180 s the chase drops the peer's declared goal and
drives to its last contact pose instead.
```

### rdv-countdown-knobs

**The countdown knobs and depart delay** — attached to `RDV_DEPART_DELAY="$(flt "${RDV_DEPART_DELAY:-100}")"` (line 648)

```text
THE COUNTDOWN (generation 19). These knobs ARE the rendezvous treatment
(minus RDV_DEPART_DELAY, inert since generation 25 — see its entry), and
until now they existed only as C++ defaults: not passed, not overridable, and
— the part that matters — not written into any manifest. A campaign's own
record could not say what treatment it had run, and `cell-provenance-in-
manifest` is the rule that says a cell must be readable from itself.

They are passed EXPLICITLY even at their default values. A -p that is absent
and a -p that happens to match the header are indistinguishable in a banked
run, which is exactly the ambiguity the `ttl0-is-the-default` lesson was about.

  RDV_DEPART_DELAY  INERT since generation 25: the arming floor is bare
                    t_now, because adding this per robot forked the ts4 N=3
                    cell across two occurrences (see the arming block in
                    explo_planner_node.cpp). Still passed and still in the
                    manifest so the param rows keep their schema across
                    generations; it decides nothing.
```

### rdv-max-late

**RDV_MAX_LATE and the chase window** — attached to `RDV_DEPART_DELAY="$(flt "${RDV_DEPART_DELAY:-100}")"` (line 665)

```text
  RDV_MAX_LATE      generation 29. How late a robot may agree to BE. At
                    arming it picks the first rung of the agreed lattice it
                    can reach with at most this much lateness, then leaves
                    early enough to arrive. IT IS NOT A NOTICE PERIOD and it
                    is not RDV_DEPART_DELAY returning: it is subtracted from
                    the robot's own drive and CLAMPED AT ZERO, so a robot
                    that can make the nearest rung contributes nothing and
                    the team cannot fork across occurrences.
                    IT ALSO SIZES HYBRID'S CHASE WINDOW, and that is not a
                    side effect to discover later. A robot signed up to a
                    rung it is already N seconds late for must depart at
                    once, and hybrid only chases while the appointment is not
                    yet due — so RDV_INTERVAL minus this value is the WIDEST
                    the window can be, and the two knobs have to be read
                    together. It is a supremum and not a floor: the window is
                    the gap from the arming instant to the next rung, less
                    min(lead, this), and an outage that happens to arm just
                    before a rung gets no window at all. See
                    rendezvous_interval_sec_ in explo_planner_node.cpp for
                    the derivation.
                    Measured on the banked generation-28 armings, the lattice
                    the schedule actually derived was 30 s on 10 of the 16
                    distinct intervals seen, and against those a 60 s budget
                    left NO window at all on 180 of 211 armings: hybrid armed,
                    never chased, and was rendezvous wearing a different arm
                    label. RDV_INTERVAL=300 is what makes 60 affordable — it
                    takes that no-window share from roughly five armings in
                    six down to roughly one in five.
                    0 = be exactly on time or take the next rung, which
                    leaves hybrid's chase dose identical to generation 28.
```

### rdv-interval-lattice

**RDV_INTERVAL as timetable spacing** — attached to `RDV_DEPART_DELAY="$(flt "${RDV_DEPART_DELAY:-100}")"` (line 695)

```text
  RDV_INTERVAL      generation 29. The SPACING OF THE TIMETABLE: the shortest
                    gap between two legal meeting instants, and so the floor
                    on the interval the scheduler derives. Meetings are not
                    periodic — they happen when the team comes apart — so this
                    is not a meeting rate; it is how far apart the rungs are
                    that a robot can sign up to, and therefore how much room a
                    late robot has to roll to the next one. Before generation
                    29 this was silently the same knob as the proposal period
                    (30 s), which also bounds how stale the snapshot the
                    punctuality estimate is costed against may be — so raising
                    the spacing used to age every drive estimate tenfold as a
                    side effect. They are separate knobs now; this one moves
                    the lattice and nothing else.
```

### rdv-latched-hold

**RDV_LATCHED_HOLD sizing and clock start** — attached to `RDV_DEPART_DELAY="$(flt "${RDV_DEPART_DELAY:-100}")"` (line 716)

```text
  RDV_LATCHED_HOLD  cap on the AT-THE-RENDEZVOUS HOLD only: how long a robot
                    that has FINISHED EXPLORING waits at the agreed cell for
                    the rest of the team. Separate from RDV_APPT_WAIT on
                    purpose. RDV_APPT_WAIT=0 (wait forever) is right for a
                    robot that still has exploring to trade against the wait;
                    a finished robot has none, so leaving that one unbounded
                    is a cell that runs to the wall clock with a robot
                    standing still and a censored completion time.

                    IT NOW CAPS EVERY FINISHED KEEPER, NOT JUST THE ONE THAT
                    HAPPENED TO SATURATE ON THE CELL (2026-09-21). Until this
                    generation a robot that finished anywhere else abandoned
                    the meeting and drove home, so this knob only ever saw
                    robots already standing on the agreed cell. They now keep
                    the appointment and travel to it, which widens WHO the cap
                    covers; it does not change how long for. The value below
                    is unchanged for that reason.

                    THE CLOCK STARTS AT max(ARRIVAL, T_MEET) (2026-09-22). It
                    used to start wherever the robot came to a stop, which
                    this comment described and which was wrong for every
                    early arrival — and early arrival is the DESIGNED case,
                    not an edge one. The deadline departure aims the arrival
                    at t_meet and departs a marked-up travel estimate before
                    it, so a robot whose estimate holds is early by a fifth
                    of its drive; a keeper that saturates its map first sets
                    off immediately and can stand for most of the countdown.
                    Either way an unclamped clock spends the margin derived
                    below on a team that is not yet due. The floor is applied
                    at both stamp sites in the node.

                    IT MUST OUTLAST ONE ROLLED RUNG (2026-09-20). The hold
                    runs from t_meet, so the earliest teardown is
                    t_meet+cap. A
                    teammate that could not make this rung rolls to the next
                    one and arrives RDV_INTERVAL later, plus up to
                    RDV_MAX_LATE of its own permitted lateness. At the old
                    300 those two quantities were equal and the teardown
                    landed on the rolled robot's arrival instant — the
                    finished robot walking off the cell in the same moment
                    the robot it was waiting for drove onto it, which is the
                    one outcome "wait until all are there" exists to forbid.
                    420 = RDV_INTERVAL + RDV_MAX_LATE + 60 s of margin, so
                    one roll is always covered. It is deliberately NOT two
                    rolls: a robot that misses twice is not coming, and the
                    cell is better spent than held.

                    RE-DERIVE THIS IF EITHER INPUT MOVES. The three knobs are
                    coupled and nothing in the node checks the relation.
```

### start-hold-pre-mission

**START_HOLD pre-mission hold** — attached to `RDV_DEPART_DELAY="$(flt "${RDV_DEPART_DELAY:-100}")"` (line 765)

```text
  START_HOLD        seconds of PRE-MISSION HOLD, applied in EVERY ARM: no
                    robot plans or navigates until it has elapsed. The team
                    agrees its rendezvous place while it is still co-located,
                    because a fleet that disperses mid-agreement meets in two
                    places (ts4 smoke20 N=3 hybrid: the upgrade was authored
                    0.6 s after a peer went silent, and that peer drove to a
                    different cell and waited out the run). It is set for off
                    and pursuit too, which never run the protocol, so the dead
                    time is a constant that cancels in every between-arm
                    contrast instead of a handicap on two arms out of four.
                    0 disables it.
                    WHAT IT DOES NOT DO: it does not contain the provisional
                    -> final rendezvous UPGRADE. An edit confining the upgrade
                    to this window shipped on 2026-09-17 and was withdrawn the
                    same day -- the upgrade needs candidate cells, candidates
                    come from the allocator's tours, tours come from completed
                    exploration steps, and a held robot completes none, so the
                    confinement deleted the upgrade rather than scheduling it.
                    The 41-77 s sweep that argued for a 120 s window measured
                    that upgrade and no longer constrains this knob. 60 s is
                    about twelve retries of margin on a handshake that
                    normally completes on the first. See
                    mission_start_hold_sec_ in explo_planner_node.cpp.
```

### comms-emulator-rewiring

**What COMMS=1 rewires** — attached to `COMMS="${COMMS:-0}"` (line 795)

```text
COMMS=1 puts the message-level radio emulator (hmr_comms_sim_node) between the
two robots, which is what turns the NOTE above from a caveat into a runnable
experiment: with it, "peer out of comms" is produced by distance through trees
rather than by killing a process. It rewires two streams, and BOTH are needed —
gating only one produces a run that looks fine and measures nothing:
  * scovox_bin  — each merger reads its peer's map off /<self>/rx/<peer>/...
    instead of the peer's own publisher, so map sharing obeys the link.
  * exploration/intents — planners publish to /<robot>/exploration/intents
    (per-robot, so the emulator has something to relay) and subscribe to the
    relayed copies. Left on the shared global bus, no outage can ever make a
    peer read as missing, and no reconnect manoeuvre would ever fire.
COMMS=0 (default) leaves both on their direct topics: one broadcast domain,
perfect comms. That is the only thing COMMS toggles — it is NOT "the 2026-08-02
campaign unchanged". This script now also enables the 2D planning map
(use_planning_map, off in the yaml and off in that campaign), which activates
the candidate free/occupied filter AND the cost-grid reachability filter that
were both entirely inactive before; overrides the ROI to a ±50 box instead of
the yaml's field site; and samples the CSV on a timer. rejected_by_unreachable,
rejected_by_minpos and goal selection therefore do not mean the same thing they
did in that campaign's CSVs, and the two are not directly comparable.
```

## Coarse cell world (M-TARE evolution, P1)

### cell-world-layer

**Coarse cell world (CELL_WORLD)** — attached to `CELL_WORLD="${CELL_WORLD:-0}"` (line 817)

```text
CELL_WORLD=1 turns on the coarse global layer: the ROI diced into
CELL_SIZE_M cells, each carrying a status derived from this robot's own map,
sampled into `cell_census` events every CELL_CENSUS_S sim-seconds and drawn
on /<r>/explo_planner/cell_world.

OFF by default, and off means NO PARAMETER IS PASSED — not `cell_world_enable
:=false`. The per-phase equivalence gate reads the run_start param dump and
requires every param new in the child to sit at its compiled default; passing
the knob explicitly would still satisfy that, but passing `team_robot_names`
would NOT, and the two have to travel together because the cell world refuses
to configure without a fleet identity. So the whole block is gated.

In this phase the layer is pure observation: nothing reads it to make a
decision. What a CELL_WORLD=1 run therefore costs is one extra sweep of the
voxel grid per census tick and one JSONL row; what it buys is the P1 gate,
which reads `covered_fraction` against `roi_unknown_fraction` on those rows.
```

### cell-world-flag-validation

**Why the cell-world flag is validated** — attached to `case "$CELL_WORLD" in` (line 834)

```text
Validated like COMMS and RECONNECT_MODE. An unvalidated flag is worse here
than elsewhere: every downstream test is `= "1"`, so CELL_WORLD=true or
CELL_WORLD=yes reads as OFF, the run completes, and the manifest records the
typo rather than the intent. The only way that surfaces is as a missing arm
in the gate's check 21, a whole campaign later.
```

### cell-status-thresholds

**Cell status thresholds for flatforest** — attached to `CELL_COVERED_U="$(flt "${CELL_COVERED_U:-0.55}")"` (line 845)

```text
The cell status thresholds, overridden here for the same reason DONE_UNKNOWN
is overridden to 0.64 below: this world has a permanent coverage floor.

A cell's unknown fraction counts x/y columns holding no observed voxel. The
map stores lidar returns plus a thin free shell, not dense ray-traced free
space, so even ground a robot drove straight over keeps a large fraction of
its 0.1 m columns empty forever. Measured over two runs, the per-cell floor
(`cell_unknown_min`) plateaus at 0.36-0.42 and the mature median at
0.50-0.55, against an ROI-wide 0.63 — which is exactly why DONE_UNKNOWN is
0.64 and not the shipped 0.05. The library defaults (0.15/0.35) assume a map
that saturates and are unreachable here: the first P1 run at those values
promoted zero cells over its entire length.

Placed by the same rule as DONE_UNKNOWN. Release just under the level the
mission itself accepts for the whole ROI, because a cell that has degraded to
the mission's own accept level is not distinguishably explored; promote a
band's width below that. The band must clear the run-to-run spread of the
floor, not just one run's: 0.42 was set from a single long run's p10 and a
later, shorter run floored at 0.4232 and promoted nothing.

Re-derive rather than nudge: cell_census carries
cell_unknown_{min,p10,median} and cell_frontier_frac_{min,median} on every
row. 0.95 for the frontier veto because in this sparse map 79-89% of a
cell's voxels are boundary voxels, so a threshold inside that band splits the
population near its median and reports mostly noise; at 0.95 it guards
outliers and the column measure discriminates.
```

## TeamWorld exchange (M-TARE evolution, P2)

### team-world-exchange

**TeamWorld exchange (TEAM_WORLD)** — attached to `TEAM_WORLD="${TEAM_WORLD:-0}"` (line 875)

```text
TEAM_WORLD=1 makes the robots actually SHARE the cell world: each publishes
its whole census at TEAM_WORLD_HZ and merges what it receives through the
guard table, and a comms model derives per-peer status from the mutual
handshake in those messages.

OFF by default and off means NO PARAMETER IS PASSED, for the same reason
CELL_WORLD is gated the same way — see that block.

Requires CELL_WORLD=1: the message IS the cell census, so the planner treats
the combination as fatal and refuses to start. Checked here as well, because
a planner that throws at construction takes both robots down several seconds
into a bring-up that otherwise looks normal, and the reason scrolls past in
two separate log files.

Worth being explicit about what TEAM_WORLD=1 COMMS=0 measures: the exchange
over a perfect shared bus, i.e. both robots converging immediately and
staying converged. That is the right control for the merge itself, and it is
NOT the P2 gate — the gate needs a dropout to heal, which needs COMMS=1.
```

### team-world-hz-is-switch

**TEAM_WORLD_HZ is the on/off switch** — attached to `_hz_ok=1` (line 904)

```text
TEAM_WORLD_HZ IS THE SWITCH, NOT JUST A RATE. The node builds the publisher
and the timer under `if (team_world_hz_ > 0.0)`, so TEAM_WORLD=1 with
TEAM_WORLD_HZ=0 is not a slow exchange, it is NO exchange — inside a run
whose manifest records `team_world=1`. Nothing downstream can tell that cell
from a correctly configured one: team_convergence.py just finds zero
`team_exchange` events, and run_campaign.sh's resume guard compares
`team_world=1` on both sides and certifies them alike. A negative value does
the same. The pairing is a contradiction, so refuse it here.

Three spellings have to fail, not one. flt emits NOTHING for a non-finite
value (it prints its own FATAL to stderr and returns empty, because `exit`
inside $(...) kills only the subshell), so the empty string is a real input
here and would otherwise reach ros2 as a bare `-p team_world_hz:=` and land
in the manifest as ''. Exponent notation is rejected too: flt passes `1e-3`
through untouched, and a rate nobody writes that way is not worth the
ambiguity of parsing it.
```

## Team-separation discount (separation.hpp)

### separation-discount

**Team-separation discount** — attached to `SEPARATION_WEIGHT="${SEPARATION_WEIGHT:-0}"` (line 940)

```text
A continuous dispersion term on the exploration utility: every candidate is
discounted for being close to a teammate's last known position, on a linear
ramp that reaches 1 at SEPARATION_RADIUS_M.

Why it exists: across cr3/cr4/cr5 the fraction of a run spent within 20 m of
the partner predicted finish time at rho = 0.677 and was significant at
n = 46 cells, while no arm mean was. Nothing in the planner acted on it —
MinPos vetoes a disc around the peer's GOAL and fires about once a campaign,
and the global allocator excludes without dispersing.

DECLARED HERE, BELOW THE _arm_stack BLOCK, deliberately, for the same reason
COORD_CLAIM_R is: the weight is orthogonal to the reconnect arm token, so no
arm pins it and a separation campaign can cross the two. That also means the
four-arm reconnect design does NOT become a compound experiment by this
file's existence — with the weight at its default 0 the planner is
bit-for-bit the pre-separation one, and the only thing the term contributes
to those runs is the sep_* diagnostic columns, which are measured with the
term off and are exactly the untreated counterfactual a later separation
campaign has to be read against.

The parity with COORD_CLAIM_R stops at the declaration, though. That knob has
a second route in: run_campaign.sh parses an "_r<N>" suffix off the arm name
and sets it PER CELL, so one invocation can run several radii. Separation has
no such suffix, so within one run_campaign.sh invocation the weight is the
same in every cell — which is all the four-arm redo needs (weight 0
throughout) and not enough for a separation campaign, where varying the
weight across arms would otherwise take two invocations and confound the
treatment with the session. Add the suffix then, and give the resume guard
the per-cell treatment cell_claim_r already gets; doing it now would ship an
untested parser to serve a campaign that is not being run.

0 = OFF and is the default, so an unset SEPARATION_WEIGHT reproduces every
campaign up to and including cr5.
```

### separation-harness-validation

**Why separation is validated here too** — attached to `for _sepkv in "SEPARATION_WEIGHT:$SEPARATION_WEIGHT" \` (line 976)

```text
Validated HERE as well as in the node, and the duplication is the point. The
node's SeparationTerm::configure() refuses a bad value and logs why, but it
refuses by DISABLING the term and letting the run continue — which is the
right thing for a field robot and the wrong thing for a campaign cell, where
it would produce a directory named for a treated arm whose planner ran the
control. Catching it here turns that into a cell that never starts.
```

### separation-bare-dot

**Rejecting a bare dot** — attached to `case "$_sepv" in` (line 991)

```text
A second case, because the first cannot express "must contain a digit" —
its patterns are ORed, and a bare "." satisfies all three of them: it is
not empty, every character is in [0-9.], and there is only one dot. That
matters more than it looks. awk reads "." as 0, so SEPARATION_WEIGHT="."
passes the [0, 1] range check below, reaches the node as 0, and the term is
OFF in a cell named for a treated arm — the exact outcome the range checks
here exist to prevent, arrived at through the one spelling they all agree
is fine. ("1." and ".5" are real numbers and stay legal.)
```

### separation-weight-range

**Separation weight range** — attached to `if [ "$(awk -v v="$SEPARATION_WEIGHT" 'BEGIN{print (v+0 >= 0.0 && v+0 <= 1.0) ? 1 : 0}')" != "1" ]; then` (line 1009)

```text
Range, matching the node's refusals so the two cannot disagree about what is
legal. Weight is a multiplier on the utility: above 1 it drives the utility
negative and below 0 it is an ATTRACTION to the teammate, neither of which is
a separation term. (Negative spellings are already rejected above — the case
pattern has no minus sign — so this catches the > 1 half.)
```

### separation-radius-age-always

**Radius and age validated at weight 0** — attached to `for _sepkv in "SEPARATION_RADIUS_M:$SEPARATION_RADIUS_M" \` (line 1020)

```text
Radius and max-age are validated even when the weight is 0, because they are
NOT inert then: the planner measures sep_peer_dist_m and sep_eligible_peers
on this radius and this freshness bound in every arm, treated or not. A
control arm silently measuring on a different bound from its treated arm is
the kind of mismatch that survives every downstream check, since both cells
print the requested numbers in their manifests.
```

## Global allocator (M-TARE evolution, P3)

### global-allocator

**Global allocator (GLOBAL_ALLOC)** — attached to `GLOBAL_ALLOC="${GLOBAL_ALLOC:-0}"` (line 1049)

```text
GLOBAL_ALLOC=1 makes the shared cell world DECIDE: each robot solves the same
assignment over the same converged world and takes its own tour, and the
focus cell it wins re-ranks that tick's candidates.

OFF by default and off means NO PARAMETER IS PASSED, for the reason the two
blocks above are gated the same way — the per-phase equivalence gate requires
every param new in the child to sit at its compiled default.

Requires TEAM_WORLD=1, and the planner refuses the pairing outright. Without
the exchange there is no shared world, so "solve the same problem" is vacuous:
every robot divides the whole map among a fleet of one and emits an
`allocation` event whose every field looks healthy. Checked here as well
because a planner that throws at construction takes both robots down several
seconds into an otherwise-normal bring-up, and the reason scrolls past in two
separate log files.
```

### alloc-peer-pos-ttl

**Allocator peer-position TTL** — attached to `ALLOC_POS_TTL="${ALLOC_POS_TTL:-}"` (line 1074)

```text
Allocator peer-position TTL, in seconds of mission-elapsed time. A peer whose
latched pose is older than this is dropped from the allocation problem and the
cells it was holding go back into the pool; 0 = unbounded = every campaign up
to and including sr3, bit-for-bit.

Set per cell by run_campaign.sh from the "_ttl<N>" arm-name suffix, which that
script strips before handing over RECONNECT_MODE — so nothing in this file sees
the suffix and no arm-token guard has to learn about it. Third knob to use that
mechanism, after DONE_SEEK and COORD_CLAIM_R.

Placed HERE and not up beside COORD_CLAIM_R because the pairing check below
needs GLOBAL_ALLOC to have been resolved, and that happens ten lines up.

Empty = pass no -p at all, leaving the node on its compiled default. Unlike
COORD_CLAIM_R, "0" is a legal explicit value rather than a spelling of "unset":
the node reads <= 0 as unbounded, which is the control level of this treatment
and needs to travel the same -p path as the treated level.
```

### alloc-ttl-needs-allocator

**TTL refused without the allocator** — attached to `if [ "$(awk -v v="$ALLOC_POS_TTL" 'BEGIN{print (v+0 > 0.0) ? 1 : 0}')" = "1" ] \` (line 1110)

```text
A TTL with no allocator is a manifest that records the treatment and a binary
that cannot express it: alloc_peer_pos_max_age_sec is read at exactly one
place, the doPlan allocator solve, which does not run when GLOBAL_ALLOC=0.
Refused rather than warned, because the cell would otherwise be pooled into
the treated arm carrying the control's behaviour — the same failure the
SEPARATION_WEIGHT/TEAM_WORLD pairing above exists to prevent.
```

## Utility-gated reconnection (M-TARE evolution, P4)

### reconnect-info-gate

**Utility-gated reconnection** — attached to `RECONNECT_GATE="${RECONNECT_GATE:-silence}"` (line 1125)

```text
RECONNECT_GATE=info arms the §3.6 knowledge + value gate: when the mid-run
silence clock expires, dispatch only if the missing peer is actually missing
something we know AND the reconnect leg pays for itself against the no-comms
plan. `silence` is the planner's compiled default and today's behaviour.

NOT the same thing as the adaptive silence clock the planner already calls an
"info gate" (reconnect_min_share_voxels / midrunGateSec). That one lengthens
the WAIT while our own backlog is small; this one is a value comparison at
the moment the wait expires. Both are on in an mtare_hybrid cell and they
compose in that order.

`silence` is the default and means NO PARAMETER IS PASSED, same rule as the
blocks above. Requires TEAM_WORLD=1: without the exchange no peer ever enters
a `known_by` mask, so the knowledge half would answer "they know nothing" on
every cell for the whole run while logging a perfectly healthy verdict. The
planner treats that pairing as fatal; refused here for the bring-up reason.
```

## Scheduled rendezvous (M-TARE evolution, P5)

### rendezvous-schedule

**Scheduled rendezvous** — attached to `RENDEZVOUS_SCHEDULE="${RENDEZVOUS_SCHEDULE:-0}"` (line 1164)

```text
RENDEZVOUS_SCHEDULE=1 arms the §3.5 appointment: when a reconnect is
dispatched, both robots derive the SAME (cell, t_meet) from the allocator's
own tours — the cheapest cell to insert into the tours they are already
driving. It used to say "instead of steering to the geometric midpoint of the
last contact"; the midpoint drives were deleted from every arm on 2026-09-16,
so with RENDEZVOUS_SCHEDULE=0 there is no meeting destination at all, not a
different one. Same correction as the sibling block below, which notes that
while stale it actively CONCEALED the rendezvous/hybrid silent null.
Agreement is by construction, not by protocol: the allocator is bit-identical
across processes, so anything derived from its output is too.

OFF by default and off means NO PARAMETER IS PASSED, same rule as the three
blocks above: the per-phase equivalence gate requires every param new in the
child to sit at its compiled default.

Requires TEAM_WORLD=1 for the same reason GLOBAL_ALLOC does — without the
exchange each robot schedules a meeting with a fleet of one, at a cell the
peer has never heard of, and logs a healthy-looking `rendezvous_agreed`
event for it. The planner treats the pairing as fatal; refused here too,
because a planner that throws at construction takes both robots down several
seconds into an otherwise-normal bring-up and the reason scrolls past in two
separate log files.
```

### rendezvous-schedule-not-pursuit

**No appointment in pursuit arms** — attached to `_rzv_pursuit=0` (line 1208)

```text
PURSUIT is the chase-only cell of the §3.6.1 factorial, and the planner
refuses to arm an appointment in it by design — that absence IS the A/B.
Setting the knob there is therefore not merely inert, it is a request for
the arm the run will not be. Fatal rather than a warning: the planner would
stamp `arm=mtare_pursuit` and the run_start param dump would say the P5
treatment was on, which is exactly the name/stamp disagreement the check
below exists to prevent, one level down.
```

## MDP interception (M-TARE evolution, P6)

### pursuit-predictor-mdp

**MDP interception (PURSUIT_PREDICTOR)** — attached to `PURSUIT_PREDICTOR="${PURSUIT_PREDICTOR:-trail}"` (line 1231)

```text
PURSUIT_PREDICTOR=mdp aims the chase at where the peer is PREDICTED to be
when this robot can get there — the argmax over the peer's last directly
received tour of P(peer at cell c at now + travel_time(c)) — instead of at
the stale last-known goal. `trail` is today's behaviour and the default.

It changes the chase WAYPOINT only. The dispatch trigger, the chase budget
and every terminator are computed from the legacy trail on both settings, so
the mdp arm cannot buy itself more chase time than the control gets, and an
intercept that does not fit the budget downgrades to the trail rather than
suppressing a chase the control would have performed.

OFF means NO PARAMETER IS PASSED, same rule as the four blocks above.
```

### mdp-needs-allocator

**MDP predictor needs the allocator** — attached to `if [ "$PURSUIT_PREDICTOR" = "mdp" ] && [ "$GLOBAL_ALLOC" != "1" ]; then` (line 1249)

```text
Needs the allocator, which in turn needs the exchange. The prediction runs on
the peer's last directly received TOUR, and a tour only exists because the
global allocator produced one; without it predictIntercept has nothing to
expand, returns invalid on every dispatch, and the cell records the P6
treatment as enabled while behaving exactly like the control.

The node refuses this pairing at construction (mdp without global_alloc). It
is caught here as well because a planner that throws at construction takes
every robot down several seconds into an otherwise-normal bring-up, and the
reason scrolls past in N separate log files.
```

### arm-name-stamp-agreement

**Arm name and node stamp must agree** — attached to `_mtare_stamped=0` (line 1282)

```text
THE ARM NAME AND THE ARM STAMP MUST AGREE, IN BOTH DIRECTIONS.

The node names the arm itself, and its rule is an OR plus a suffix:
  arm = (global_alloc_enable || reconnect_gate==info ||
         rendezvous_schedule_enable || pursuit_predictor==mdp
           ? "mtare_" : "") +
        (reconnect_enabled ? reconnect_mode : "off") +
        (pursuit_predictor==mdp ? "_mdp" : "")
Everything downstream — the cell directory, campaign_index.csv, every
analysis script — takes the arm from RECONNECT_MODE instead. So any
configuration where those two disagree produces a cell whose directory says
one arm and whose run_start says another.

The mtare_hybrid token guards one direction (it refuses a flag turned back
off). This is the other: flags turned ON under a plain token. It was
reachable through `--env "GLOBAL_ALLOC=1"` and through a direct invocation,
and the cost of not catching it here is a full cell budget (~79 s + 1.149x
t_sim) burned before gate_g8's check 3e reports the mismatch — or, if nobody
scores that root, a silently mislabelled arm that analyze_runs.py pools with
the untreated cells.

Written as the node's own predicate rather than a list of bad pairs, so a
further knob that sets the prefix cannot slip past it. P5's
RENDEZVOUS_SCHEDULE and P6's PURSUIT_PREDICTOR are the fifth and sixth, and
each was added to BOTH sides in the same commit: a predicate that stops
tracking the node's is a check that has stopped checking while still
printing a pass. Keep this list in lockstep with the `mtare` OR in
explo_planner_node.cpp's arm stamp — grep `mtare_` there.
```

### arm-stamp-full-string

**Comparing the full arm name** — attached to `_arm_core="${RECONNECT_MODE#mtare_}"` (line 1315)

```text
Reconstructed as the WHOLE STRING rather than as a prefix-present flag. The
core is whatever the token is once its decorations are stripped, and each
decoration is then put back from the knob that actually drives it in the node
— so the comparison is the full name the run_start dump will carry against
the full name every directory and index will carry. A prefix-only check would
pass `mtare_hybrid` with the predictor on, which is a different treatment
under the same name and precisely the pooling this exists to stop.
```

### plain-token-needs-schedule

**Modes that need the schedule** — attached to `_rzv_needed=0` (line 1340)

```text
The converse of the two RENDEZVOUS_SCHEDULE checks above, added 2026-09-16 —
the one that was actually costing us. Those refuse the schedule where it
cannot fire; this refuses a MODE THAT IS NOTHING WITHOUT IT. `rendezvous` and
`hybrid` are defined by the agreed (place, time), so with
RENDEZVOUS_SCHEDULE=0 the planner arms no appointment and:

  rendezvous -> dispatch returns false every time. The run behaves as the
                `off` arm while sitting in a directory named rendezvous.
  hybrid     -> only the chase half survives. The run behaves as the
                `pursuit` arm while sitting in a directory named hybrid.

Both are worse than an inert knob: `off` and `pursuit` are OTHER ARMS OF THE
SAME EXPERIMENT, so the cell is not voided, it is silently relabelled, and the
contrast it lands in is the one it was supposed to be measured against. The
planner refuses the same pairing at construction; this is the copy that fires
before Gazebo starts.

WHY IT SITS AFTER THE ARM-STAMP GUARD AND NOT WITH ITS TWO SIBLINGS. Up there
it SHADOWED that guard. `GLOBAL_ALLOC=1 RECONNECT_MODE=hybrid` is a
name/stamp disagreement — a treated cell about to be filed in the control
column — and it also happens to have RENDEZVOUS_SCHEDULE=0, so whichever
check runs first is the one the operator sees. The name/stamp report is the
more useful of the two (it names the knob that did it and the token that
would be honest), and it is the one the guard calibration asserts, so it wins
the tie by running first. Nothing is lost by deferring: both are fatal, both
fire before anything is launched, and no configuration escapes by reaching
only one of them.

The mtare_* tokens pin RENDEZVOUS_SCHEDULE through the arm stack and their
own contradiction check fires ~700 lines earlier, so what actually lands here
is the plain `rendezvous` / `hybrid` token at its default — which, with the
arm-stamp guard taking the RENDEZVOUS_SCHEDULE=1 side, is the whole reason
those two tokens cannot produce a run. See the RECONNECT_MODE default block.
```

### radio-tx-power

**Radio TX power knob** — attached to `TX_POWER="$(flt "${TX_POWER:-30.0}")"` (line 1403)

```text
Radio severity. Matches comms_sim_params.yaml's shipped value, so the default
changes nothing — it exists so the number lands in the run manifest and can be
swept from the command line during calibration rather than by editing the
installed yaml, which would silently re-scope every later run and leave no
record of which severity any given run used.
```

### radio-tree-attenuation

**Per-trunk attenuation severity dial** — attached to `TREE_ATTEN="$(flt "${TREE_ATTEN:-70.0}")"` (line 1409)

```text
The SECOND severity dial, and the one that acts on geometry rather than
margin. tx_power_dbm shifts every link equally; tree_attenuation_db shifts a
link in proportion to how many trunks are on it, so it is what selects
occlusion-driven outages over distance-driven ones. Exposed here for exactly
the reason TX_POWER is, and it was missing: the sw-campaign design (§30) needs
to move it, and until now the only way to do that was to edit the installed
comms_sim_params.yaml — which re-scopes every later run on the machine and
leaves no field in any run directory saying which severity that run used.
Default matches the shipped yaml, so a bare invocation changes nothing. The
shipped value moved 2026-09-03 from 11.98 (the IEEE 9260568 per-trunk fit,
under which one trunk almost never dropped a link) to 70.0 (any trunk in the
Fresnel zone is fatal). cr3/cr4/cr5 and everything earlier ran 11.98 — a
different radio regime; never pool across the change.
```

### radio-max-range

**Hard radio horizon** — attached to `MAX_RANGE="$(flt "${MAX_RANGE:-30.0}")"` (line 1423)

```text
The THIRD severity dial, added with the 70 dB trunks: a hard radio horizon.
The other two shape where a link fails; this one bounds how far a TREE-FREE
link reaches at all — free-space loss alone never drops a 30 dBm link inside
the ROI, so with trunks fatal, clear lanes would otherwise carry for ever.
Recorded in the manifest for the same reason as the other two.
```

### expect-outage-flag

**EXPECT_OUTAGE and the outage gate** — attached to `EXPECT_OUTAGE="${EXPECT_OUTAGE:-1}"` (line 1429)

```text
Does this run's arm expect the link to drop? 1 = yes (every treatment arm),
0 = the control arm, deliberately run at a tx_power_dbm that keeps the link
up. Only affects the run-time outage gate's verdict, never the radio itself:
severity is TX_POWER's job alone, and the two are set independently so that a
control run whose link DID drop is still reported rather than excused.
```

## Link-state gate for the mid-run reconnect trigger (§30.11, §30.24)

### link-gate-default-on

**Link-state gate on the mid-run trigger** — attached to `LINK_GATE="${LINK_GATE:-1}"` (line 1436)

```text
The trigger used to fire on peer RECORD AGE, which ages whenever a teammate is
not sending -- link up or down. Measured against the emulator's own trace it
ran +49.1 s ahead of real link-down and 41-42 % of all mid-run fires bought
nothing, 16-21 % of them firing while the radio was UP. LINK_GATE=1 points the
planner at the emulator's connected bit instead.

ON BY DEFAULT since generation 9. It was off through generation 8, on the
argument that every banked campaign (pb3g2, tr1, tl1, tl2, td1) ran the
record-age clock and a default flip would silently make the next run
incomparable with all of them. That argument no longer holds and a stronger
one now points the other way:

  - Generation 9 already breaks comparability by design. MIDRUN_SILENCE moved
    240 -> 90 in the same change, so these runs are a new generation and are
    not poolable with the record-age campaigns whatever this flag says.
  - The two settings are COUPLED. 90 s sits below the ~180 s
    heartbeat-suppression tail, so it is only safe because the veto can tell a
    silent teammate from an absent one. Defaulting the threshold low and the
    veto off leaves the low clock running bare -- firing at a healthy peer
    that is merely deep in a PLAN loop.

    And a BARE INVOCATION IS that combination, which an earlier draft of this
    comment denied. COMMS defaults to 0, so LINK_GATE=1 resolves to
    LINK_GATE_EFFECTIVE=0 and the 90 s clock runs with no veto. That is
    tolerable for a smoke test and intolerable for a campaign, which is why
    run_campaign.sh refuses it outright (see its guard, and
    campaign_guard_calib.sh for the known-answer cases). Here it is a loud
    WARNING at the manifest write, not a refusal.
  - g8r1 is what an unset gate costs in practice: it ran with the veto off and
    the only trace at the firing site was a "radio down -1s" sentinel, which
    reads like a measurement rather than "the check did not run".

Set LINK_GATE=0 explicitly to reproduce a record-age campaign. The manifest
line below is still the record either way, and it is now the thing to check
first when comparing two runs.

Only meaningful with COMMS=1: with no emulator there is no link topic, the
planner finds no samples, and it would spend the run warning about a gate it
cannot use. So the topics resolve to "" unless the radio is actually in
circuit, which also keeps a COMMS=0 smoke test quiet.

Set identically for EVERY arm. The arms must differ in RECONNECT_MODE alone;
`off` cannot reach the gated code at all (it needs reconnect_enabled, which
is false there), so passing it the same parameters costs nothing and removes
a whole class of "did the arms differ in something else too" question.
```

### link-gate-validation

**Why LINK_GATE is validated** — attached to `case "$LINK_GATE" in` (line 1482)

```text
Validated like COMMS and RECONNECT_MODE, and for the same reason: every value
that is not exactly "1" turns the veto off in the topic block below, so "true"
or "2" reads as a request and behaves as a refusal. Worse, it used to be
copied verbatim into LINK_GATE_EFFECTIVE, so the manifest recorded
link_gate_effective=2 for a run with no veto at all, and the safety WARNING is
conditioned on LINK_GATE="1" and so could not fire. Refuse the value instead
of recording a provenance line that disagrees with the run.
```

### link-gate-effective

**Requested versus effective link gate** — attached to `if [ -n "$LINK_GATE_TOPIC" ]; then` (line 1502)

```text
LINK_GATE is the REQUEST; LINK_GATE_EFFECTIVE is what the planner will
actually get, and only the second one describes the run. They diverge on
exactly one pairing -- gate asked for, emulator absent -- and that pairing is
silently unsafe rather than merely inert: MIDRUN_SILENCE now defaults BELOW
the ~180 s heartbeat-suppression tail on the understanding that the veto can
tell a quiet teammate from an absent one, so losing the veto leaves the low
threshold running bare. Say so at the top of the log instead of letting the
reader reconstruct it from two manifest lines.

Only the flag is computed here: this is the defaults block and log() is not
defined until much later in the file, so the warning itself is emitted at the
manifest write where the rest of the run's configuration is reported.

Derived from the OBSERVABLE, not re-derived from the inputs. LINK_GATE_TOPIC
being non-empty is the single fact that decides whether the node is handed
comms_link_states_topic at all (see the EXTRA block below), so reading it back
here means link_gate_effective cannot drift from what the run did if the
condition above is ever edited. An earlier version recomputed the same
conjunction a second time and assigned "$LINK_GATE" in the else branch, which
is only 0-or-1 because the validator above now makes it so.
```

### link-down-confirm-zero

**Link-down confirm set to 0** — attached to `LINK_DOWN_CONFIRM="$(flt "${LINK_DOWN_CONFIRM:-0}")"` (line 1530)

```text
How long the radio must have been CONTINUOUSLY down before the veto lets a
mid-run chase through. Its own knob since generation 9; both veto sites used
to borrow RECONNECT_CONFIRM (3.0), which is the team-PRESENCE release confirm
and answers a different question. Decoupling them is right at any value.

THE VALUE IS 0, AND THE 30 s DEBOUNCE THAT WAS HERE IS WITHDRAWN. At 0 the
veto is exactly its disjunct -- "never chase a peer whose radio is up right
now" -- which is the whole test. 30 was credited in an earlier draft of this
comment with moving wasted fires "from 50 % to 33 %", and that was a
misattribution: the table it pointed at held this conjunct FIXED at 30 and
varied the presence clock, so 50 -> 33 is the 150 -> 90 move, not this
parameter's effect. Measured properly on the untreated `off` arm, holding
MIDRUN_SILENCE at 90:

    confirm   cells arm (of 23)   fires   wasted   beats natural recovery
       0            18             40      20 %            85 %
      30            12             26      15 %            92 %

It buys 5 points of purity for A THIRD of the arm's activation -- 18 arming
cells down to 12, i.e. 6 of the 18 that armed. ("A quarter", in an earlier
draft, is 6/23: the share of the arm, not of the activation being traded.)
Dilution is the defect this generation exists to fix, so the trade goes the
other way.
```

### relay-queue-cap

**Reliable-relay backlog cap** — attached to `RELAY_QUEUE_BYTES="${RELAY_QUEUE_BYTES:-1073741824}"` (line 1554)

```text
Reliable-relay backlog cap, in bytes. The emulator's shipped default is 64 MiB
and on overflow it drops the OLDEST queued map delta and never retransmits, so
the receiver's merged map loses those voxels for the rest of the run. That
silently breaks the experiment's central premise — the backlog is supposed to
DRAIN at each contact (B0a), which requires the link to be a delay, not a
lossy channel. At 64 MiB it did not hold: measured against tx_power_dbm=-14
and 0.20 m voxels, atlas published 3345 scovox_bin deltas and bestla received
3190, i.e. 153 lost exactly as drop_overflow reported, and the `off` arm lost
1180 in BOTH directions. Sized here for the worst case instead: ~120 kB/s of
deltas per direction (2 Hz, ~60 kB each) against a full T=3600 s blackout is
~430 MB, so 1 GiB carries a 2.4x margin and costs at most 2 GiB of RAM across
both directions. The overflow counter stays a hard gate — this raises the cap
so the gate stops firing for real, it does not silence it.
```

### sim-roi-half-extent

**Sim-only planner ROI** — attached to `ROI_HALF="$(flt "${ROI_HALF:-50.0}")"` (line 1568)

```text
Planner ROI half-extent, SIM ONLY (square, centred on the world origin).
shared_params.yaml carries the real field site's ROI — x ∈ [-51.3, 100.9],
y ∈ [-38.7, 74.5] — and on flatforest ~42% of that footprint has no geometry
at all: those columns never leave the prior, so the unknown fraction can never
reach done_unknown_fraction and EVERY run ends at max_steps instead of at
coverage. That still fires one reconnect manoeuvre (the step-budget path also
routes through finishOrRendezvous), which is why the symptom is invisible —
what it destroys is the repeat reconnect -> re-disperse cycles the experiment
needs. ±50 fits inside the 110x110 ground plane and covers the oak field.
±50 also fits inside the global planning map derived from it just below.
```

### global-planning-map-size

**World-fixed planning map domain** — attached to `PLAN_MAP_SIZE="$(flt "${PLAN_MAP_SIZE:-$(awk "BEGIN{print 3*$ROI_HALF}")}")"` (line 1579)

```text
Side of the WORLD-FIXED planning map (~/global_planning_map) — the
2D map the EXPLORATION planner consults for free/occupied and reachability.
Published by BOTH scovox_node (this robot's own measurements) and
dscovox_node (the fused team map); same envelope, same resolution, so the
two are comparable cell-for-cell. The planner reads the DSCOVOX one — see
the map-domain note below.
Both it and the ROI are centred on the world origin, so side 2*ROI_HALF
exactly covers the ROI and 3*ROI_HALF leaves ROI_HALF/2 of margin on each
side. The margin is not cosmetic: candidates are ROI-clipped but the ROBOT is
not, and the cost-grid flood starts from the robot's own cell — a robot that
has drifted past the ROI edge onto an out-of-bounds cell floods nothing and
every candidate is rejected as unreachable. Derived from ROI_HALF so the two
cannot drift apart.

This is NOT /<r>/scovox_node/planning_map. That one is a 20 m robot-centred
crop whose extent IS simple_nav_3d's local-planner window, and the exploration
planner rejects any candidate whose cell is out of bounds (isCellOccupied
treats out-of-bounds as occupied) — on the rolling map every frontier beyond
~10 m is dropped and exploration collapses to a bubble around the robot.
Widening that topic instead would push the whole world through the local A*
on the control path.

THE MAP-DOMAIN FIX (2026-08-21). The planner used to read scovox_node's copy,
which contains only what THIS robot measured — while planning over the FUSED
team map for everything else. Ground the partner surveyed was therefore
unknown on the reachability map, so candidates in it were rejected as
unreachable and the planner starved. dscovox_node now publishes the same
world-fixed envelope over the fused grid and the planner reads that instead,
putting reachability in the same domain as the candidates it filters.

Still NOT /<r>/dscovox_node/planning_map (no "global_"). That name means
scovox_node's 20 m rolling crop, which the LOCAL nav planner consumes.
simple_nav_3d's nav global planner used to subscribe to it — transient_local,
with no publisher — and so never planned for the whole campaign history; in
generation 5 it was repointed at this same global_planning_map instead of
that name being made real. Both planners now read this topic; neither reads
the other's.
```

### cost-grid-flood-cap

**Reachability flood radius cap** — attached to `COST_CAP="$(flt "${COST_CAP:-$(awk "BEGIN{print 10*$ROI_HALF}")}")"` (line 1618)

```text
Dijkstra flood radius for the candidate reachability filter. MUST be set here,
and this is the trap that comes with switching the planning map on.

The filter is dead code with use_planning_map=false: no map means no cost grid
to flood, so doPlan sets skip_reachability and every candidate passes. Turning
the map on activates it — with a cap that has therefore never been exercised.
The yaml ships cost_grid_radius_cap_m: 0.0 = auto = candidate_max_radius + 2 =
10 m, a bound sized for POLAR candidates, which are generated within
candidate_max_radius by construction. Frontier centroids have no range limit
at all (addFrontierCandidates clips to the ROI and nothing else), and
FRONTIER_ONLY=1 removes the polar set entirely — so under the auto cap every
frontier more than 10 m of walked distance away comes back kInfCost and is
rejected as unreachable. The escape hatch does not save it either: it only
trips below 10 reached cells, and a 10 m flood at 0.4 m/cell reaches ~1900.
Exploration would degenerate to 10 m hops, or stall outright once no frontier
remains inside the disc — and it would look like a legitimate result.

10x ROI_HALF is deliberately far past the grid diagonal ($PLAN_MAP_SIZE * 1.41
= 212 m at the defaults): the intent is "unbounded within this grid", and a
cap must not be clamped to the diagonal because a walked Dijkstra distance
routinely exceeds the straight line (see the warning in CostGrid::floodFrom).
Passing 0 to mean unbounded does NOT work — 0 is the auto sentinel. The cost
is one full flood of a ~140k-cell grid per PLAN tick, which the exploitation
planner already pays on the same grid with a genuinely unbounded flood.
```

### min-goal-dist-oscillation

**Minimum goal distance vs oscillation** — attached to `MIN_GOAL_DIST="$(flt "${MIN_GOAL_DIST:-4.0}")"` (line 1643)

```text
Minimum range to an acceptable EXPLORE candidate. The yaml ships 0.0 (off,
preserving field behaviour) and this script overrides it, because on
flatforest 0.0 does not merely explore inefficiently — it deadlocks.

Measured, not theorised: at 0.0 both robots entered a two-point oscillation
(atlas ping-ponging between goals 0.87 m apart, bestla 0.63 m) and stayed
there. Steps kept advancing, every process stayed healthy, the CSV kept
filling, and the unknown fraction flatlined at 0.87 having moved 0.001 in the
last 100 sim-seconds. Nothing in the stack reports it: the harness hang gate
watches the STEP counter, and the steps were fine.

Cause is in the planner's utility, not here — see candidate_min_goal_dist_m
in shared_params.yaml. 4.0 m sits well inside fov_max_range (20.0 since D1,
10.0 when this was measured), so a hop still lands deep in previously-seen
space rather than jumping blind. The margin only grew, so the correction
still holds -- but it was measured at the smaller range and has not been
re-measured at the larger one.

This is a real scenario correction in the sense of plan §2: it changes what
every arm does, so it must be identical across arms and no run from before it
may be pooled with one after.
```

### utility-cost-exponent

**Utility distance-discount exponent** — attached to `UTIL_GAMMA="$(flt "${UTIL_GAMMA:-0.5}")"` (line 1665)

```text
Distance discount on the utility denominator: U = info / (eps + cost)^GAMMA.
1.0 is the original SSMI form and the node short-circuits it to be
bit-identical. Goes through flt() because the node declares it as a double
and an integer literal would abort it at startup on the type mismatch (see
the flt() note at the top).

The default moved 1.0 -> 0.5 on 2026-08-19 on a 6-replicate sweep: 18.9%
faster to the 0.60 rung (487 +/- 46 s vs 600 +/- 90 s, exact permutation
p = 0.0152), via 25% fewer metres driven at unchanged speed. The evidence and
its limits are written out at utility_cost_exponent in shared_params.yaml —
read that before trusting the number, because it was measured on ONE world
with PERFECT COMMS.

CAMPAIGNS THAT STRADDLE THE CHANGE MAY NOT BE POOLED. This is a SCENARIO
CORRECTION in the sense of plan §2 exactly like MIN_GOAL_DIST: it changes
what every arm does. Anything compared against p14 or against any run made
before 2026-08-19 must set UTIL_GAMMA=1.0 explicitly rather than relying on
the default, which no longer reproduces those runs. It lands in the run
manifest below so a cell can always be attributed after the fact.
```

### frontier-z-band

**Frontier search z band** — attached to `FRONTIER_Z_LO_OFF="$(flt "${FRONTIER_Z_LO_OFF:-5.7}")"` (line 1685)

```text
Frontier search band, as an inset from the ROI z band. The yaml ROI band is
[-5.5, +4.0] (sized for the field site's terrain and canopy), so these insets
put the frontier search in absolute z [0.2, +1.5] on flatforest's flat ground
— about the slice a Husky's VLP-16 actually sweeps as it drives.

MIN_GOAL_DIST alone does NOT fix the oscillation, which is why both exist:
with the full 9.5 m band the near-frontier set is unobservable-by-construction
(the lidar's free-space wedge has a top and bottom surface at every range and
those surfaces are frontier forever), so raising the minimum hop just moves
the ping-pong out to that radius. Measured: at MIN_GOAL_DIST=4.0 with the
full band, atlas oscillated between two goals 4.3 m apart instead of 0.87 m
apart. The band is what makes a frontier something driving can consume; the
minimum hop is what stops the planner picking a goal too close to observe
from. Verify from the planner log line "Frontier search band inset by ...",
which prints the ROI band it was applied to.
```

### eig-sensor-overrides

**EIG sensor-model overrides** — attached to `FOV_HFOV="${FOV_HFOV:-}"` (line 1702)

```text
EIG sensor-model overrides. ALL UNSET BY DEFAULT — with none of them set the
harness passes nothing and shared_params.yaml remains the single point of
control for the evaluator's geometry, which is what it was before these
existed. They exist so an A/B on that geometry does not have to edit an
installed build artifact (install/.../shared_params.yaml) and then remember to
rebuild, which is unreproducible and invisible in the run manifest.

fov_hfov/fov_vfov are doubles in the node and go through flt(); the *_RAYS and
CAND_N_YAW are declared as integers and must NOT get a decimal point, or the
node aborts at startup on the type mismatch (see the flt() note at the top —
the trap runs in both directions).

CAND_N_YAW only bites with POLAR on: FRONTIER_ONLY=1 (the default here) turns
the polar grid off entirely, and frontier candidates take their yaw from the
centroid direction, not from this count.
```

### visited-goal-suppression

**Recently-visited goal suppression** — attached to `VISITED_RADIUS="$(flt "${VISITED_RADIUS:-6.0}")"` (line 1722)

```text
Recently-visited goal suppression. The yaml ships 0.0 (off, field behaviour);
this scenario needs it, and it is the third and last piece of the same defect.
MIN_GOAL_DIST stops the planner picking a goal too close to observe from, the
frontier band stops it chasing frontiers no sensor can ever consume, and this
stops it going back to where it just was -- which it otherwise does forever,
because a forest's trunk shadows keep regenerating frontier clusters and
utility reduces to "nearest" once info gain is flat.

6.0 m is just above frontier_cluster_radius_m (5.0), so suppressing a reached
goal suppresses the cluster that produced it, not a point inside it. The TTL
must outlive a there-and-back hop at these distances; 180 s is ~15 steps at
the observed cadence, and it expires rather than persisting so that an area
genuinely re-frontiered later in the run can still be revisited.
```

### failed-goal-blacklist

**Failed-goal blacklist TTL and retire** — attached to `FAILED_TTL="$(flt "${FAILED_TTL:-240.0}")"` (line 1737)

```text
Failed-goal blacklist (generation 5). The mismatch that made this a defect:
the TTL was 60 s while ONE nav attempt is budgeted up to nav_max_timeout_sec
= 180 s, so a blacklisted trap reliably expired while the robot was still
burning a single budget somewhere else and was then free to be re-picked.
mr1_hybrid_seed18 atlas did exactly that: eight 180 s failures at the same
two sites, 1441 s of the run spent on ground it had already proved
unreachable, ending 0.021 above the 0.64 coverage threshold -- i.e. censored
by this alone. 240 = 180 + 60 keeps a site suppressed across a full failure
elsewhere; the planner WARNs at startup if this drops below that sum.
RETIRE_AFTER holds a site for the rest of the run once it has failed that
many times, because TTL expiry alone cannot close a permanent terrain trap.
Both are released immediately by actually reaching the site.
```

### failed-goal-radius

**Failed-goal suppression radius** — attached to `FAILED_RADIUS="$(flt "${FAILED_RADIUS:-2.0}")"` (line 1751)

```text
The disc the TTL above applies over. Passed and recorded because "suppressed"
is a claim about an AREA, not a point: at 2.0 m one entry covers a goal and
its neighbours, and at 0.2 m the planner would re-pick a cell 30 cm from the
trap and call it a new goal. A manifest that names the TTL but not the radius
describes half of the mechanism.
```

### nav-budget-recorded

**Nav budget knobs in the manifest** — attached to `NAV_MIN_TIMEOUT="$(flt "${NAV_MIN_TIMEOUT:-30.0}")"` (line 1757)

```text
The nav budget the TTL above is sized against, and the no-progress abort that
ends a leg early. All four are recorded for one reason: generation 5's
censoring vocabulary is written in these units. A cell ending in `timeout`
means a leg hit NAV_MAX_TIMEOUT, `no-progress` means it moved less than
PROGRESS_MIN_DIST in PROGRESS_WINDOW, and neither label can be read at all
without the numbers that produced it. They were fixed in the YAML through four
binary generations and invisible in every cell, so a reader comparing a gen-4
cell to a gen-5 one had no way to tell whether the budget had moved under them.
```

### failed-ttl-guard

**Refusing a TTL below one nav budget** — attached to `awk -v ttl="$FAILED_TTL" -v navmax="$NAV_MAX_TIMEOUT" 'BEGIN {` (line 1769)

```text
Refuse the exact defect generation 5 exists to fix, rather than warning about
it. The planner has its own startup check on this inequality, but it is a
WARN: it scrolls past in a log nobody reads until the campaign is over, which
is precisely what happened for the whole of mr1. Here it costs a cell nothing
to stop.

The threshold is the planner's (+30), NOT the 240 = 180 + 60 the default
picks. Two numbers for two jobs: +30 is the floor below which the TTL is
certainly broken, and the extra 30 s in the default is deliberate slack on
top. A guard set at the default would reject working configurations, and one
set below +30 would pass the broken ones.
```

### homing-approach-watchdog

**Homing approach watchdog** — attached to `RETURN_APPROACH_WINDOW="$(flt "${RETURN_APPROACH_WINDOW:-40.0}")"` (line 1790)

```text
Homing approach watchdog (generation 5). See §32.10: the original homing
watchdog measured GROSS metres travelled, which a robot orbiting a local
minimum satisfies forever. These bound the approach test that replaces it.
```

### voxel-resolution

**scovox voxel resolution** — attached to `VOXEL_RES="$(flt "${VOXEL_RES:-0.20}")"` (line 1797)

```text
scovox voxel edge length. The launch default is 0.10, which does not survive a
run long enough to reach coverage termination: free space is carved along the
whole ray to max_range 20 m, so the fused map grew to 12.7M voxels by t=550 s
and the planner's map ingest fell behind for good — one robot then drove for
three minutes on a frozen map, still stepping, still logging, coverage flat,
with nothing in the stack reporting it. 0.20 is ~8x cheaper.

NOT a free knob under COMMS=1: the ScovoxMapBinary deltas are exactly what the
radio model carries, so this sets the offered load the link is stressed with.
The §4 severity calibration must be run at the resolution the campaign runs
at; a tx_power_dbm calibrated at another resolution does not transfer.
```

### done-unknown-threshold

**Coverage termination threshold** — attached to `DONE_UNKNOWN="$(flt "${DONE_UNKNOWN:-0.64}")"` (line 1809)

```text
Coverage-termination threshold. The yaml ships 0.05, which this scenario
cannot reach: measured on a full-length control run, the ROI unknown fraction
fell to 0.4922 by t=2900 and then did not move for the remaining 2600 sim-s
while both robots drove a further ~950 m each. That is a floor, not a plateau
on the way somewhere — the frontier utility is info/(eps+cost) with info
near-constant, so the planner is a diffusive nearest-frontier crawler: trunk
shadows keep regenerating frontiers inside the region it has already covered,
and a distant unexplored corner never wins on cost. It saturates at roughly
half of a +-50 ROI and then cycles there indefinitely.

So this follows plan §2.1's own prescription — measure the floor, set the
criterion above it — rather than chasing a threshold that cannot be hit. The
criterion lands in the fast early phase, where map sharing is what separates
the arms, and the runs terminate naturally instead of every arm censoring at T
and reporting the same non-answer. The default was 0.55 through the sw/pb
campaigns; it is 0.64 from tr1 onward, which is the value tr1's 40 banked
cells were actually run at and the value the current criterion is defined at.

It MUST be identical across arms: it is the definition of the primary
endpoint, so a run at a different value is not comparable to one at this one.
Recorded in the manifest for exactly that reason.
```

### done-criterion-latch

**The latch done criterion** — attached to `DONE_CRITERION="${DONE_CRITERION:-latch}"` (line 1831)

```text
WHICH RULE decides a robot has finished. "latch" (default): each robot is
finished the first tick its OWN dscovox-fused ROI unknown fraction reaches
DONE_UNKNOWN — any state, no confirmation streak, no rendezvous gate, and it
never un-finishes. The run then ends when BOTH robots have latched, which is
the STOP_ON_DONE rule below unchanged (it already waits for every planner's
state column to read DONE).

Why the default moved off "streak": the old rule was only tested at the top of
doPlan, so a robot inside a reconnect manoeuvre could not declare itself
finished however saturated its map was. On tr1 that put ~60 s of detection
latency on the hybrid arm and 0 s on off — the endpoint moved with the
treatment, which makes it part of the treatment rather than a measure of it.
"latch" measures the same thing in both arms. Robots still BEHAVE differently
between arms; the chase simply no longer decides when the clock stops.

Like DONE_UNKNOWN this is the definition of the primary endpoint, so it MUST
be identical across arms and runs at a different value are not comparable.
Recorded in the manifest for that reason.
```

### ros-domain-id

**Own DDS domain for the run** — attached to `export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"` (line 1856)

```text
Own DDS domain, NOT the default 0. This box runs other ROS work (the scovox
replay harnesses) on domain 0, and a second /clock publisher appearing there
mid-run would corrupt every use_sim_time consumer in this stack — sim time is
not something a run can recover from. Every child inherits this, RViz and the
script's own `ros2 topic echo` gates included; to poke at the run by hand,
export the same value first:  export ROS_DOMAIN_ID=42
```

### viz-mesh-palette

**Per-robot RViz mesh palette** — attached to `VIZ_PALETTE=(COSTAR_HUSKY_SENSOR_CONFIG_REDUCED_YELLOW` (line 1864)

```text
Per-robot viz mesh set. Every robot spawns as COSTAR_HUSKY_SENSOR_CONFIG_LIDAR
in the scenario, but that model is one colour — RViz uses the painted variants'
meshes (same geometry, different texture) so the robots can be told apart at a
glance. Assigned by ROSTER POSITION rather than by name (P7): a scenario is
free to name its robots anything, and a name-keyed table would hand the third
robot an empty model string and a URDF that fails to parse. Only three painted
variants exist, so beyond N=3 the palette repeats — cosmetic only, and the
banner says so rather than letting the reader wonder why two robots match.
```

## peers_of()

### peers-of-helper

**Per-link peer helpers** — attached to `peers_of()  { local s=$1 p; for p in $ROBOTS; do [ "$p" = "$s" ] || echo "$p"; done; }` (line 1885)

```text
Everything COMMS=1 rewires is per-link, so the wiring has to name each far
end. peers_of returns ALL of them (P7): at |ROBOTS| == 2 it emits the single
name its predecessor peer_of did, so every pairwise call site is unchanged,
and at N >= 3 the callers that build subscription LISTS get the whole set
instead of one arbitrary member of it.
```

## environment: humble + ws overlay, miniconda stripped

### snap-env-scrub

**Snap environment scrub** — attached to `unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_IM_MODULE \` (line 1909)

```text
Snap scrub — REQUIRED when this is launched from the VS Code snap's integrated
terminal (or any snap-hosted shell). That environment exports GTK/GDK/glib
paths pointing inside /snap, and a system rviz2 started under it loads the
snap's GTK stack, which drags in /snap/core20's libpthread and dies at startup
with "undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE". Verified:
stripping these makes rviz2 start with hardware GL. Harmless in a plain shell,
where none of them are set.
```

## stack_procs()

### teardown-stack-procs-partition-scope

**Scoping stack_procs to one partition** — attached to `local matched` (line 1962)

```text
Matching machine-wide by command pattern is correct for ONE cell and a
cross-kill for two: this list feeds teardown()'s `kill -KILL`, so without
scoping, cell A's teardown SIGKILLs cell B's gazebo and planner mid-run.
That surfaces as a random mid-run death in an unrelated cell -- the most
expensive kind of bug to chase. When IGN_PARTITION is set we keep only
processes whose OWN environment carries the same value, read from
/proc/<pid>/environ rather than the command line: the partition is
inherited by gazebo and the ros_gz bridge and never appears in argv.
Unset (the sequential default) falls through to the original behaviour
byte for byte, so this cannot regress a normal single-cell run.
```

## count_own()

### count-own-bringup-guard

**Per-cell process counts for guards** — attached to `local pat=$1 n=0 p` (line 1988)

```text
How many processes matching $1 belong to THIS cell? Same ownership test as
stack_procs, for the bring-up guards rather than for teardown. A guard that
counts machine-wide is correct for one cell and a RACE for two: with two
concurrent cells there are four planner binaries, and whichever cell checks
first aborts on the other cell's existence. That is not hypothetical -- it
was observed directly, a cell dying "expected exactly 2, found 4" eight
seconds after an otherwise healthy start, while the other survived purely
because it happened to check after the first had been torn down.
IGN_PARTITION unset falls through to the machine-wide count, so sequential
runs keep exactly today's behaviour.
```

## teardown()

### teardown-final-gate-verdict

**Final gate verdict at teardown** — attached to `if [ "$COMMS" = "1" ] && [ -f "$OUTDIR/comms_gates.txt" ]; then` (line 2030)

```text
Final gate verdict. The watcher is a background child killed by the loop
above, so its exit status is unreachable here — the report file it appends
to is the only durable record, and until it is read out the run ends looking
successful whatever the gates found at minute 40. Includes the outage gate,
which can only be decided at the end: a COMMS=1 run whose link never dropped
is a control run wearing a treatment label, and nothing before this point
can tell.
```

### gate-map-agreement-report-only

**map_agreement is report-only** — attached to `if [ -f "$HERE/map_agreement.py" ]; then` (line 2038)

```text
Merged-map convergence, decided only now because it reads the finished
CSVs. Both robots merge both scovox_bin streams, so with the deltas in
reliable_topics their two copies must agree at the end. Three dense-world
runs finished 1.5-1.8% apart with every other gate green: a KeepLast
reader overflowing on the reconnect burst discards the excess with no
error and no counter, and scovox_node's new-subscriber resnapshot cannot
heal it because the emulator's DDS subscription never drops.

THAT THEORY DID NOT SURVIVE and map_agreement.py is now REPORT ONLY — see
its module docstring; the end-of-run gap turned out to measure undrained
backlog, which scales with outage severity, so failing on it would discard
exactly the cells where the comms treatment bit hardest. It therefore
emits PASS or INFO and never FAIL or UNRUN, and it cannot move the verdict
below. This comment used to say the opposite ("it emits UNRUN on its own
failure paths, which scores as SUSPECT rather than silent PASS") and that
was not merely stale: at N>=3 the checker's own two-robot assumption made
it emit UNRUN on every cell, which scored every cell SUSPECT, which
hard-failed every cell in gate_g8 — a report-only check failing a whole
campaign through a counter it was never meant to reach.

THE EXIT STATUS IS READ. This ended in a bare `|| true`, which was never a
backstop: the script runs `set -u` without `set -e`, so nothing here could
abort the trap, and `|| true` only destroyed the status. The checker exits
0 on its own INFO paths deliberately and says so in its own comment ("that
`|| true` should not be what keeps the contract"), so a NON-ZERO exit here
means it crashed before printing anything and the map-spread reading was
not taken — indistinguishable, in the file, from a healthy run that had
nothing to say. INFO and not UNRUN for the reason given three paragraphs
down: this gate is report-only and UNRUN would hand it a verdict it is not
entitled to cast.
PRESENCE, NOT THE EXECUTE BIT. Same defect, same reasoning as the
rendezvous block below: this tested -x and then invoked through `python3`
regardless, so the bit could not affect whether the gate would WORK -- it
could only skip a gate that would have succeeded. A lost execute bit (a
fresh clone under a permissive umask, a copy through a filesystem that
drops modes) silently disarmed the map-spread reading on every cell of a
campaign, with nothing in comms_gates.txt to say it had not been taken.

THE ABSENCE LINE IS INFO AND NOT UNRUN, DELIBERATELY. Everything argued at
the top of this block applies to the missing-file path too: this gate is
REPORT-ONLY, and UNRUN is counted a few lines below into NUNRUN, which
scores the cell SUSPECT, which gate_g8 hard-fails. Emitting UNRUN here
would reproduce exactly the failure this block already suffered once -- a
report-only check failing a whole campaign through a counter it was never
meant to reach -- and it would do it on the cells where the file is
missing, i.e. all of them. Absence must be RECORDED without being handed a
verdict the gate is not entitled to cast. The name matches map_agreement's
own --name default so both paths occupy one gate name, not two.
```

### gate-rendezvous-agreement-validity

**The scheduled-arm rendezvous validity gate** — attached to `if [ -f "$HERE/rendezvous_agreement.py" ]; then` (line 2099)

```text
Did the SCHEDULED arm schedule anything? Unlike map_agreement.py this one
IS a validity gate: it emits FAIL, and the count below turns that into
run_gates_verdict=INVALID and a non-zero exit under GATES_STRICT.

It exists because a rendezvous or hybrid cell that never commits a triple
is indistinguishable, everywhere else in the pipeline, from one that did.
The ts4 generation-15 smoke produced three such cells out of six: 600 s
each, rc=0, every comms gate green, `run_gates_verdict=CLEAN`, and no
agreement anywhere in the run. The arm suffix, the manifest and the
campaign index all said "rendezvous" because all three record the REQUEST.
Nothing recorded whether the treatment happened, so a campaign of untreated
cells would have banked and read as a null result.

Inert on the off and pursuit arms: the checker returns silently unless the
manifest says rendezvous_schedule=1, so it adds no line and cannot move the
verdict on an arm that is not supposed to schedule anything.

Placed AFTER map_agreement.py and BEFORE the counters for the same reason
that one is: the planner logs and the event files are only complete once
the planners have been stopped, which the loop above has just done.
PRESENCE, NOT THE EXECUTE BIT. This tested -x and then invoked through
`python3` anyway, so the bit it tested could not affect whether the gate
would run -- it could only skip a gate that would have worked. A lost
execute bit (a fresh clone with a permissive umask, a copy through a
filesystem that drops modes) would then have silently disarmed the
rendezvous check on every cell, with nothing in comms_gates.txt to say so.
An absent FILE is still worth an UNRUN line rather than silence, for the
same reason the gate itself now emits one when its manifest key is gone.

THE EXIT STATUS IS READ, AND THAT IS THE WHOLE POINT OF THIS BLOCK. It
used to end in a bare `|| true`, which was not a backstop at all: this
script runs `set -u` WITHOUT `set -e`, so nothing here could have aborted
the trap in the first place, and the only thing `|| true` accomplished was
to destroy the status. The checker's contract (see its docstring, "EXIT
STATUS AND THE VERDICT") is that it exits 0 ALWAYS and writes exactly one
`VERDICT\trendezvous_agreed\t...` line, so a NON-ZERO exit means it died
before reaching emit() and wrote no verdict at all. Its traceback lands in
comms_gates.txt matching neither ^FAIL nor ^UNRUN, so the counters below
read a clean file and the cell banked CLEAN with no rendezvous evidence in
it — which is precisely the outcome this gate exists to prevent, reached
through the gate itself. A crashed validity gate is the definition of
UNRUN, and the trap continues either way because there is no `set -e`.
```

### gate-rendezvous-agreed-name

**One gate name for the rendezvous check** — attached to `printf 'UNRUN\trendezvous_agreed\t%s\n' \` (line 2151)

```text
THE GATE NAME IS THE ONE THE CHECKER ITSELF WRITES -- `rendezvous_agreed`
(rendezvous_agreement.py's single emit()), not the module's filename.
This said `rendezvous_agreement` until 2026-09-18, which gave one check
two names in comms_gates.txt: any consumer keyed on the second column
(gate_g8's REPORT_ONLY_GATES membership test, or any tally of which
gates ran across a campaign) saw one gate that never passes and another
that never goes missing. The divergence landed on the ONE path where the
name is the only evidence the gate was ever supposed to exist, since
there is no output from the checker to corroborate it.
```

### gate-missing-watch-summary

**No watch summary means SUSPECT** — attached to `log "RUN SUSPECT: no run-time gate summary — the watcher never reported"` (line 2179)

```text
No watch summary => the run-time watcher never adjudicated, and the file
holds bring-up PASSes only. Absence of failures is NOT a pass: the
missing line is exactly the one carrying the outage gate, i.e. the proof
that the treatment happened. Every run before 2026-08-15 landed here and
was scored CLEAN on bring-up alone, because the watcher was killed with a
signal it did not handle (see comms_gates.py's handler registration).
```

### gates-strict-runtime-exit

**GATES_STRICT covers run-time gates** — attached to `if [ "$GATE_VERDICT" = "INVALID" ] && [ "${GATES_STRICT:-0}" = "1" ]; then` (line 2197)

```text
GATES_STRICT used to cover only the BRING-UP gates, so a run whose link
gates failed at minute 40 still exited 0 and every campaign driver recorded
it as OK. The three phase-3 mode runs were all logged "OK rc=0" while the
same teardown printed RUN INVALID directly above it.
```

### gate-report-missing-invalid

**COMMS=1 with no gate report** — attached to `log "RUN INVALID: COMMS=1 but no $OUTDIR/comms_gates.txt — not one gate ran"` (line 2206)

```text
COMMS=1 AND NO REPORT FILE AT ALL. The condition above is a conjunction,
and until 2026-09-18 it had no else: a treated run whose gate report never
materialised fell straight out of the block, wrote no run_gates_verdict,
and banked. Every consumer then read the key's ABSENCE, and only one of
them treats absence as a problem — run_campaign.sh greps for the literal
`=INVALID`, so the cell was skipped as complete on every later resume, and
ts4_smoke21_check reports "<absent>" long after the campaign is over.

This is strictly worse than any SUSPECT the block above can produce: not
one gate ran, so nothing certifies that the emulator was in the path, that
the link ever dropped, or that the arm was treated at all. That is exactly
the "control wearing a treatment label" hazard, and it is the one failure
that a cell must never be allowed to bank through. INVALID, therefore, and
not SUSPECT — INVALID is the only verdict run_campaign.sh will redo, and a
cell with no gate evidence is cheap to redo and worthless to keep.

The COMMS=0 path deliberately still writes nothing. An ideal-comms run has
no link to gate, so it has no verdict to record; gate_g8 reads the absence
as MISSING and hard-fails it already, which is correct for a gate written
for comms campaigns, and inventing a token here would change what those
runs report without telling anyone anything new.
```

## resolve the ambient run-control knobs, before the trap and the manifest

### run-control-knobs-before-trap

**Resolving run-control knobs before the trap** — attached to `STOP_ON_DONE="${STOP_ON_DONE:-1}"` (line 2238)

```text
C2 (2026-09-14). Every knob below used to be resolved further down, AFTER the
manifest was written, so none of them could be recorded -- and several of them
define what the manifest's own run_end_t_sim MEANS. STOP_ON_DONE decides
whether the run ends on the endpoint at all or grinds to the duration cap, and
DONE_GRACE_S is the sim-second drain added after the last robot declares done;
an ambient `export DONE_GRACE_S=0` in the launching shell would shorten every
cell of a campaign, change the endpoint, and leave no trace anywhere.

Resolving them here is a no-op for the run: each is the same `${X:-default}`
expansion it always was, the later lines that still carry it are idempotent
once the variable is set, and nothing reads any of them before this point.
What changes is that the cell can now say what it was run under.

Placed HERE, immediately before `trap teardown`, for a second reason: teardown
reads GATES_STRICT, `set -u` is on, and the trap can
fire at any point after it is installed. Resolving them below the trap left an
early die() aborting inside the trap on an unbound variable -- the failure the
GATES_STRICT comment further down already warns about. Every name in this
block is now bound before the trap exists.
```

## wait_for()

### wait-for-deadline-not-attempts

**Why wait_for is deadline-based** — attached to `wait_for() {` (line 2272)

```text
Deadline-based, not attempt-based, and that is the whole point. The gates
below used `for i in $(seq 1 24); do timeout 8 ros2 topic echo … --once; done`
and described themselves as waiting 3 minutes. They did not. `ros2 topic echo`
only blocks for its timeout once the topic's TYPE is resolvable — i.e. once a
publisher already exists. Before that it prints "does not appear to be
published yet / Could not determine the type" and returns rc=1 in ~0.3 s, so
all 24 attempts burned in ~8 s and the run died on a sim that was still
spawning entities. Same trap with `ros2 topic list`, which always returns
promptly. Any readiness gate built from a fast-failing probe needs an explicit
sleep or a deadline; counting attempts silently makes the budget a function of
how the probe fails.
```

## 0. preflight

### preflight-banner-team-size

**Banner team size from the scenario** — attached to `log "=== $SCENARIO: ${N_ROBOTS}-robot ($ROBOTS) lidar explore+exploit, RViz=$RVIZ gui=$GZ_GUI ==="` (line 2300)

```text
The banner reports the team from $ROBOTS, which is parsed from the scenario,
rather than the "2-robot" literal it used to hardcode. That literal was
harmless while every campaign ran two robots and actively misleading the
moment one did not: the team-size series makes N the treatment, so a log whose
first line names the wrong N misidentifies the arm it belongs to. $N_ROBOTS is
set at the top from the same parse the planner census asserts on, so this line
cannot drift from what actually spawned.
```

### preflight-log-teardown-scope

**Logging the teardown scope** — attached to `if [ -n "${IGN_PARTITION:-}" ]; then` (line 2309)

```text
A11 (2026-09-15): say which teardown scope is in force, in the log, because
the two differ in what they are ALLOWED to kill and the difference is
invisible otherwise. Unset means stack_procs() and count_own() match
machine-wide, which is correct and intended for a sequential campaign and a
cross-kill the moment two cells overlap. Not made fatal: every campaign to
date runs sequentially with it unset, so refusing to start would break the
working path to guard a path nobody uses yet. It is already in the manifest
as ign_partition, so this line is for whoever is reading a cell log while the
cell is still running.
```

## 2. TF glue + viz model (one each per robot)

### rsp-urdf-via-params-file

**URDF through a params file** — attached to `{ echo "/**:"` (line 2358)

```text
The URDF goes in through a params FILE, never `-p robot_description:=<xml>`:
rcl rejects a parameter override rule whose value contains newlines
("Couldn't parse parameter override rule", arguments.c:343) and
robot_state_publisher aborts before it starts. A YAML literal block scalar
takes the XML verbatim, colons and quotes in its comments included.
```

## 2b. comms emulator (COMMS=1)

### comms-emulator-before-mappers

**Emulator starts before the mappers** — attached to `if [ "$COMMS" = "1" ]; then` (line 2375)

```text
BEFORE the mappers, and the order is load-bearing. Relay subscriptions are
created by a 1 Hz topic-discovery poll, and only once the source topic already
exists — so an emulator started after scovox_node misses everything published
in between, including the initial full map snapshot. That loss is invisible:
it happens upstream of the relay, so no drop statistic counts it, and the run
just quietly begins with each robot missing the other's first map. Started
here, the subscription is in place ~1 s after each publisher appears, well
before scovox_node has integrated enough lidar to emit anything.
Robot names and the tree-bearing world SDF come from the same scenario file
the sim was launched with; the /rx/ readiness check is deferred until after
the mappers are up, since that is when the relays can first form.
```

### comms-link-logger-early-start

**Link-state logger starts with the emulator** — attached to `start linklog "$OUTDIR/link_logger.log" \` (line 2398)

```text
Per-run connectivity trace. link_states is published at link_rate_hz and
kept nowhere else: the gate watcher reads the aggregate `stats` topic, and
recovering it from the bag afterwards means decoding a bag per run. Every
§5 metric that mentions the radio -- realised disconnection fraction,
outage durations, contact events and their attribution to a planner state
-- is derived from this file.

Started HERE, one publisher-startup behind the emulator, rather than down in
§6b with the gate watcher. From §6b the trace began at t_sim~48 while the
planners had been running since ~27, so the first ~21 s of every run had no
connectivity record at all — and that is the window in which the robots are
closest together and the link is certain to be up, i.e. the part a
disconnection fraction most needs in its denominator. The logger simply
waits for the topic, so starting it before the mappers costs nothing.
```

## 3. nav + lidar mapping, mergers cross-wired

### nav-dscovox-team-map-wiring

**Merger wiring for the team map** — attached to `PEER_BIN_PATTERN="/{peer}/scovox_node/scovox_bin"` (line 2418)

```text
mapping:=dscovox_lidar => per robot a scovox_node on /<r>/velodyne_points plus
a dscovox_node merging BOTH robots' scovox_bin streams, so each planner reads
its own copy of the TEAM map.
With COMMS=1 each merger takes its PEER's binary off the emulator's relayed
copy instead of the peer's own publisher. Self is untouched by the pattern —
a robot's own map never crosses a radio link.
```

### nav-launch-per-robot-roster

**One nav launch per robot** — attached to `for r in $ROBOTS; do` (line 2427)

```text
One launch per robot, over the roster (P7). This was two hand-written blocks
naming atlas and bestla; `peers:` is a comma-separated list on the simple_nav
side, so an N-robot merger is the same call with a longer list — each robot's
dscovox_node fuses every OTHER robot's binary, which is what makes each
planner read a team map rather than a pairwise one.
```

### nav-gate-scovox-global-map

**Gating on the scovox global map** — attached to `wait_for 120 "$r global_planning_map" -- \` (line 2446)

```text
The exploration planner's map is a SEPARATE publisher and it is a hard
precondition: with use_planning_map=true the planner sits in its start-up
wait until one arrives, so a typo in the topic name or a launch arg that
silently defaulted to 0.0 (= disabled) would present as two planners that
never leave INIT — an hour into a run, with no error anywhere. Gate on it
here instead. Both publishers are transient_local and only emit when
someone is subscribed, so this echo is also what pulls the first sample.
```

### nav-gate-fused-global-map

**Gating on the fused map the planner reads** — attached to `wait_for 180 "$r fused global_planning_map" -- \` (line 2458)

```text
THE ONE THE PLANNER ACTUALLY READS (2026-08-21). Gated separately from the
scovox copy above because they fail for different reasons: the scovox one
needs only this robot's own integration, while this one additionally needs
a ScovoxMapBinary to have arrived and been fused (dscovox allocates its
fused grid lazily on the first wire frame, and publishes nothing until
then). A longer budget for that reason.
```

## 5. optional bag

### bag-relayed-streams-under-comms

**Bagging the relayed streams** — attached to `COMMS_BAG_TOPICS=()` (line 2491)

```text
Under COMMS=1 the interesting streams are the RELAYED ones: /exploration/intents
is empty (the planners moved to per-robot topics) and the peers' direct
scovox_bin publishers still carry everything, so a bag of only the pre-relay
topics would show perfect comms no matter what the link did. Add the link
states too — they are the ground truth for when each link was up.
```

### bag-sent-and-received-intents

**Bagging both ends of the exchange** — attached to `for p in $(peers_of "$r"); do` (line 2500)

```text
Both ends of the exchange: what this robot SENT (once, above) and what
actually arrived from each peer. The pair is what makes the P2
convergence claim checkable offline — a bag of only the sent copies shows
robots that all published diligently through a blackout none of them
received across. Only when the stream exists; a topic nobody publishes
just makes `ros2 bag record` warn once per run about a name it cannot
find. At N robots the received side is N-1 topics per robot, so this list
grows quadratically — it is names only, and the BYTES are unchanged
(every relayed copy exists whether or not it is recorded).
```

### bag-comms-stats-topic

**Why the emulator stats are bagged** — attached to `COMMS_BAG_TOPICS+=( /hmr_comms_sim/link_states /hmr_comms_sim/robot_index` (line 2520)

```text
/hmr_comms_sim/stats carries backlog_bytes and the drop_* counters
(drop_airtime, drop_ber, drop_overflow). Those are the ONLY evidence for
the backlog gate and for the shared-airtime confound — a single global
token bucket means one large map delta can drive the pool negative and drop
every best-effort intent on every link, precisely during the post-reconnect
drain when peer presence has to be re-detected. Unrecorded, that mechanism
is unfalsifiable after the fact: the gate watcher samples it live but only
writes a sample out when it fails.
```

### bag-record-modes-full-lean

**RECORD full versus lean bag sets** — attached to `BAG_TOPICS=( /clock /tf_static )` (line 2532)

```text
RECORD=1 is the full set — everything needed to reconstruct or re-watch a
run. RECORD=2 is the LEAN set: only what the plan's offline analysis
consumes, which is the §4 calibration replay (both odom + /clock), the §5.1
oracle re-merge (sender-side scovox_bin), and the link/airtime diagnostics.

The difference is not marginal. The two OccupancyGrids dominate the full
set: global_planning_map alone is (size/res)^2 bytes per message — 375x375 =
~140 kB at the ROI_HALF=50 defaults — published per robot at ~1 Hz, so it
outweighs everything the analysis actually reads. A full matrix of 30-40
runs at full RECORD does not fit on this box; the lean set does. Neither
mode records the RELAYED /rx/ copies: the oracle merge is defined on what
each robot SENT, and what arrived is already reconstructable from
link_states plus the receivers' own CSVs.

Built by looping the roster rather than naming atlas/bestla (P7). At N == 2
this emits the same topics in the same order the literal lists did.
```

## 6. planners (namespaced, one per robot) + one global scheduler

### planner-launch-terrain-relative-z

**Planner launch and flat terrain height** — attached to `POLAR_ARG="true"; [ "$FRONTIER_ONLY" = "1" ] && POLAR_ARG="false"` (line 2573)

```text
Same parameter set as exploitation_experiment.launch.py, namespaced so the two
instances don't collide on node name / ~/candidates / proximity_hold_state.
terrain_relative_z:=false is REQUIRED on flatforest: in terrain mode a vantage
sits at MAPPED ground + clearance, and the ground on the far side of a trunk is
exactly what the trunk occludes — those vantages are rejected `noground` and
targets close PARTIAL with the ring half-driven. The world is a ground_plane at
z=0, so the fixed absolute height is the same number anyway.
```

### arm-token-to-reconnect-params

**Mapping arm tokens to reconnect params** — attached to `RECONNECT_ENABLED="true"; MODE_ARG="$RECONNECT_MODE"` (line 2582)

```text
The control arm. `off` is not a reconnect_mode (see the RECONNECT_MODE block);
it is reconnect_enabled:=false, which is the switch the planner actually
gates the manoeuvre on. reconnect_mode is left at its yaml value in that case
and is inert, since shouldRendezvous() returns false before the mode is
consulted.

The six `mtare_*` tokens map the same way and for the same reason: none of
them is a reconnect_mode either, each is one of the four §3.6.1 cells (or, for
the two `_mdp` names, a chasing cell with the P6 predictor) plus the P1-P6
stack the blocks above have already switched on. `mtare_off` is
`off` with that stack, so it drops reconnect_enabled exactly as plain `off`
does. The planner reconstitutes the arm label itself from what it was handed
(see its addParamStr("arm", ...)), so nothing here has to carry the name
forward — the token strips to its suffix and the prefix comes back from the
knobs. That round trip is what the name/stamp check above verifies.
```

### cell-world-team-names-once

**Team name list built once** — attached to `TEAM_NAMES_ARG="["` (line 2609)

```text
Cell-world args. Built here rather than in the launch loop so the ordered
team array is constructed ONCE and every robot is handed the identical
literal — building it per robot invites a future edit that reorders it for
one of them, and an id that means a different robot on each side is exactly
the failure the whole known_by scheme cannot detect from inside.
```

### separation-off-structural-zeros

**Separation-off structural zeros** — attached to `if [ "$TEAM_WORLD" = "1" ]; then` (line 2639)

```text
The counterfactual half of this line is only true when there IS a peer
position to measure against. The planner builds its anchor list from the
team model, so at TEAM_WORLD=0 that list is empty on every tick and
sep_peer_dist_m is -1 and sep_eligible_peers 0 for the whole cell — a
structural zero, not an observation that the robots stayed apart. Saying
"untreated counterfactual" there would invite exactly the pooling it is
meant to prevent.
```

### rendezvous-off-no-fallback

**No fallback reconnect destination** — attached to `log "scheduled rendezvous OFF (RENDEZVOUS_SCHEDULE=0) — no rendezvous_schedule_enable param passed, so no appo` (line 2672)

```text
Reworded 2026-09-16. This used to say "the reconnect destination is the
last-contact midpoint", which stopped being true the day the three midpoint
drives were removed, and while it was stale it actively CONCEALED the
rendezvous/hybrid silent null — it read as a description of a working
fallback. There is no fallback destination any more, in any arm.
```

### done-coverage-source-scovox

**Coverage source pinned to scovox** — attached to `log "planning_map = /<r>/dscovox_node/global_planning_map (FUSED team map, ${PLAN_MAP_SIZE}m @ ${PLAN_MAP_RES}` (line 2689)

```text
done_coverage_source is pinned to scovox, NOT left on "auto". auto switches to
the 2D planning_map the instant one is received, so enabling the planning map
would have silently swapped the termination metric from 2.5D column coverage
of the ROI to 2D cell coverage — a different number against the same
done_unknown_fraction threshold, changing when every run ends and invalidating
any comparison with runs recorded before this change.
```

## 6a. run manifest

### manifest-purpose-and-provenance

**What the run manifest is for** — attached to `MANIFEST="$OUTDIR/run_manifest.txt"` (line 2698)

```text
Everything that distinguishes this run from another one, written INTO the run
directory. Until now the arm, the seed and the radio severity existed only as
log() lines on the harness's own stdout — so across a 40-run matrix the label
for each run lived in the operator's scrollback and nowhere else, and two runs
that differed only in RECONNECT_MODE were indistinguishable after the fact.
Git hashes with a dirty marker matter just as much: every repo here has
uncommitted changes, so a bare commit id would be an actively misleading
provenance record.
```

### manifest-claim-radius-override

**Recording the claim-radius override** — attached to `echo "coord_claim_radius_override=${COORD_CLAIM_R:-none}"` (line 2725)

```text
cr2's independent variable. "none" means no -p was passed and the node took
the yaml value, which is what every campaign through mt2 did -- so this key
reading "none" is a positive statement about those cells, not a gap. The
arm token cannot carry it: run_campaign.sh strips the _r<N> suffix before
RECONNECT_MODE is formed, so without this line the radius would be
recoverable only from the directory NAME, and a directory can be renamed
while a manifest written at launch cannot.
```

### manifest-alloc-pos-ttl

**Recording the allocator position TTL** — attached to `echo "alloc_peer_pos_max_age_sec=${ALLOC_POS_TTL:-none}"` (line 2733)

```text
Same contract as the line above, and for the same reason: run_campaign.sh
strips the _ttl<N> suffix before RECONNECT_MODE is formed, so this is the
only record of the level that is not the directory name. "none" = no -p was
passed and the node took its compiled default (0 = unbounded), which is a
positive statement about every campaign through sr3 rather than a gap.

Note "0.0" and "none" are the same BEHAVIOUR and deliberately different
STRINGS: 0.0 means the campaign asked for unbounded explicitly, through the
same -p path as the treated arm, and none means the question was never asked.
An analysis that pools them is fine; one that uses "none" to identify the
control arm of a TTL campaign is reading a cell that ran no -p at all.
```

### manifest-link-gate-clock

**Recording which clock the trigger used** — attached to `echo "link_gate=$LINK_GATE"` (line 2770)

```text
Which CLOCK the mid-run trigger fired on. This is a behavioural switch, not
a tuning knob: a link_gate=1 run and a link_gate=0 run are different
experiments and their fire times are not comparable. Recorded per run so no
future readout has to infer it from a git hash — two of the hashes on the
last campaign were '-dirty' and could not have answered this.
```

### manifest-link-gate-effective

**Requested versus effective link gate** — attached to `echo "link_gate_effective=$LINK_GATE_EFFECTIVE"` (line 2776)

```text
link_gate is the REQUEST, link_gate_effective is what the planner got. They
differ only when the gate was asked for with no emulator to feed it, and a
reader comparing two campaigns needs the second one. g8r1 is why: its
manifest honestly said link_gate=0 and the only trace at the firing site was
a "radio down -1s" sentinel that reads like a measurement rather than like a
check that never ran.
```

### manifest-cell-world-decision-knobs

**Recording the cell-world decision knobs** — attached to `echo "global_alloc=$GLOBAL_ALLOC"` (line 2833)

```text
The two knobs that make the cell world DECIDE something, and so the two
that separate an mtare_hybrid cell from a hybrid one. Recorded even when
off, same rule as every knob above: a control run must say the feature
existed and was disabled, or it is indistinguishable from a run of a binary
that never had it.
```

### manifest-separation-knobs

**Recording separation knobs at weight 0** — attached to `echo "separation_weight=$SEPARATION_WEIGHT"` (line 2842)

```text
Recorded even when the weight is 0, same rule as every knob above, and with
one extra reason: an off cell's sep_peer_dist_m and sep_eligible_peers
columns ARE measurements, taken on the radius and freshness bound below,
and a reader comparing them against a treated arm has to be able to see
from the cell itself that the two were measured the same way.
```

### manifest-roster

**Recording the roster** — attached to `echo "robots=$(echo $ROBOTS | tr ' ' ',')"` (line 2854)

```text
The roster is DERIVED from the scenario (P7), so recording the scenario
name alone leaves a reader to go and look it up in an install tree that may
have been rebuilt since. Written out so team size and robot identity can be
read off the cell itself — every analysis script currently hardcodes
("atlas","bestla") and this is what lets them stop.
```

### manifest-failed-goal-history

**Failed-goal keys in the manifest** — attached to `echo "failed_goal_ttl_sec=$FAILED_TTL"` (line 2876)

```text
The failed-goal pair was NOT in this manifest before generation 5, which is
precisely how a 60 s TTL against a 180 s nav budget survived a whole
campaign without anyone being able to see it in a cell's own record.
```

### manifest-run-control-knobs

**Recording the run-control knobs** — attached to `echo "stop_on_done=$STOP_ON_DONE"` (line 2900)

```text
C2: the run-control knobs. The first two DEFINE the endpoint -- whether the
run stops when the robots are done, and how much sim time drains after they
are -- so a cell that omits them cannot have its run_end_t_sim interpreted.
The rest are abort/watchdog thresholds: they decide whether a slow cell is
killed or banked, which is a selection rule on the data. All were ambient
and untraceable before 2026-09-14.
```

### manifest-comms-trees-loaded

**Recording trunks the radio sees** — attached to `if [ "$COMMS" = "1" ]; then` (line 2917)

```text
Number of trunks the radio model can actually see. Recorded because it is
NOT recoverable from the commit sha while a change is uncommitted: two runs
either side of a fix to the tree matcher carry byte-identical "<sha>-dirty"
provenance but different link statistics. link_states col 3 gives per-link
counts, never the world total.
```

### manifest-planner-binary-hash

**Hashing the installed planner binary** — attached to `planner_bin="$WS/install/explo_planner/lib/explo_planner/explo_planner_node"` (line 2947)

```text
The commit hashes above identify the SOURCE; this identifies what actually
ran. p7modes recorded four planner commits for one binary, and settling
that after the fact took six pairwise diffs plus an mtime that turned out
to be a symlink's (this is a symlink-install workspace: the executable
lives in build/, and `stat` does not dereference by default). Hash the
binary and the question becomes a lookup.
```

### manifest-nav-binary-hashes

**Hashing the simple_nav_3d binaries** — attached to `for _navnode in simple_nav_costmap_node simple_nav_planner_node \` (line 2966)

```text
The same argument, for the OTHER binary that carries behaviour. D2 (the
goal-snap append) lives in simple_nav_planner_node, and generation 9 changes
it. git_simple_nav_3d above moves with the SOURCE tree -- it moves whether or
not colcon ran -- so it cannot witness a rebuild. That makes the one failure
mode D2 has both specific and silent: edit simple_nav_3d, forget to build,
and every provenance field in this manifest still reports generation 9 while
the node on the wire is generation 8. Hash what ran.

All four executables, not just the planner. They are built and installed as
a unit, so a hash that moves for one and not the others is a partial or
stale install -- which this symlink-install workspace can actually produce,
and which is otherwise indistinguishable from a clean one.
```

### manifest-params-file-hash

**Hashing the installed params file** — attached to `planner_params="$WS/install/explo_planner/share/explo_planner/config/shared_params.yaml"` (line 2987)

```text
And the params file, for the same reason — it is the OTHER half of what
the node actually ran, and it was invisible in the run record.

The node loads it from the INSTALL tree, not from source, so a stale or
hand-edited installed copy silently changes behaviour with the binary hash
unmoved. done_action lives only here: it must be "idle" for the rendezvous
barrier, and the node's own default is "shutdown", so a params file that
failed to install turns the barrier off while every other provenance field
in this manifest still matches.
```

### manifest-params-probe-bare-scalars

**Params probe accepts bare scalars** — attached to `echo "done_action_in_params=$(sed -n \` (line 3000)

```text
Accept quoted OR bare scalars. The probe used to require double quotes,
so `done_action: idle` — valid YAML, and what a hand-edited params file
usually looks like — recorded "missing" while the parameter was present
and load-bearing. A provenance probe that reports absence for a formatting
choice is worse than no probe: it trains the reader to ignore the key.
```

### manifest-params-alloc-and-claim

**Two more params-only world properties** — attached to `echo "global_alloc_comms_mask_in_params=$(sed -n \` (line 3008)

```text
Two more world properties that live ONLY in the params file, and that a
campaign can therefore be run under without any record of but the hash.

global_alloc_comms_mask was flipped true for the cm1 pilot and back to
false for cr2. It is invisible in every mt2-era manifest: the hash moves,
but nothing says WHICH way, so a reader holding two campaigns could see
that the params differed and not what differed. That is exactly the shape
of a check that has stopped checking.

coord_claim_radius_m is the yaml side of cr2's independent variable, so
the pair (this, coord_claim_radius_override above) pins the effective disc
without reading the source tree: override wins when set, otherwise this
value applies, and 0.0 here means auto = fov_max_range.
```

### manifest-params-missing-keys

**Missing params file keeps its keys** — attached to `echo "sha256_shared_params=missing"` (line 3028)

```text
BOTH keys, not just the hash. Dropping done_action_in_params here made a
missing params file the one case where the field vanishes from the
manifest entirely — and any consumer that greps for the key gets no line,
which is indistinguishable from an OLD manifest written before the key
existed. The failure it is meant to expose (barrier silently off because
the params did not install) is exactly this failure, so the key has to be
present and say so.
```

### manifest-world-geometry

**Recording the world geometry** — attached to `_world_short=$(sed -n 's/^world:[[:space:]]*\([^[:space:]#]*\).*/\1/p' \` (line 3043)

```text
A10a (2026-09-15): the world GEOMETRY. Until now it was the one load-bearing
input with no record in the cell at all. `scenario=` above names a yaml; that
yaml names a world SHORT name; _world_registry.py maps that to an SDF file.
Three hops, none of them written down, and the install tree is symlinked to
source, so the SDF can change under a workspace that looks rebuilt without
any hash already in this manifest moving.

The gap is not hypothetical. On 2026-09-15 a sim review was written, and
world edits applied, against flatforestv2.sdf (registry short name
`flatforest`) while every ts1b cell had in fact run flatforest_dense.sdf.
Nothing in a finished cell could have caught it, because no field named the
file. These do.

sdf_world_name is here because robot_sim.launch.py composes
/world/<short_name>/create from the REGISTRY name, so the SDF's internal
<world name> has to equal it or every spawn call goes to a service that does
not exist. It matches today, verified for all six flatforest worlds; this
records it per cell so a future rename reads as a diff instead of as a
bring-up timeout.
```

### link-gate-without-emulator-warning

**Warning on a link gate with no emulator** — attached to `if [ "$LINK_GATE" = "1" ] && [ "$LINK_GATE_EFFECTIVE" != "1" ]; then` (line 3113)

```text
Deferred from the defaults block, where log() does not exist yet. The pairing
below is the one combination that is silently unsafe rather than merely inert:
MIDRUN_SILENCE now defaults BELOW the ~180 s heartbeat-suppression tail on the
understanding that the veto can tell a quiet teammate from an absent one, so
with no emulator the low threshold runs bare and a healthy in-range partner
deep in a PLAN loop reads as missing. The planner warns too; this says it
before the run rather than in a per-node log nobody opens until afterwards.
```

### team-world-split-topics

**Split topics for the TeamWorld stream** — attached to `if [ "$TEAM_WORLD" = "1" ]; then` (line 3178)

```text
TeamWorld exchange. Split pub/sub exactly like the intent stream and for
exactly the same reason: the emulator gates a stream by relaying it
per-link, which it can only do when publisher and subscriber sit on
different topics. With COMMS=0 no topic params are passed, so both ends
default to the shared /exploration/team_world bus — correct for a
perfect-comms run, and the reason a COMMS=0 TeamWorld run cannot be read as
evidence about gating.
```

### planner-claim-radius-override

**Passing the claim-radius override** — attached to `if [ -n "$COORD_CLAIM_R" ]; then` (line 3202)

```text
MinPos claim disc. Passed only when overridden, so an unset value leaves the
node on the yaml "0 = auto" path and reproduces every campaign before cr2
byte for byte. Both cr2 levels (10 and 40) come through here, so the -p code
path itself is common to the two arms and cannot be confounded with them.
```

### planner-alloc-pos-ttl-param

**Passing the allocator position TTL** — attached to `if [ -n "$ALLOC_POS_TTL" ]; then` (line 3209)

```text
Allocator peer-position TTL. Passed only when set, so an unset value leaves
the node on its compiled 0 and reproduces every campaign through sr3 byte for
byte. BOTH levels of a TTL campaign come through here — the control arm is
_ttl0, not the absence of a suffix — so the -p path is common to the two arms
and cannot be confounded with them. flt() because the node declares a double
and ros2 infers int from a bare "0", which aborts the node at startup.
```

### planner-separation-params-always

**Separation params passed at weight 0** — attached to `EXTRA+=( -p separation_weight:="$(flt "$SEPARATION_WEIGHT")"` (line 3218)

```text
Team-separation discount. Passed ALWAYS, including at weight 0, which is
the opposite of the COORD_CLAIM_R rule above and deliberately so. The
radius and the freshness bound are live in an off arm — they are what
sep_peer_dist_m and sep_eligible_peers are measured on — so the -p code
path has to be identical in the treated and untreated arms or the
counterfactual is measured by a different route from the treatment. flt()
on all three: they are doubles in the planner, and a bare integer makes
ros2 infer int and abort the node at startup.
```

### scheduler-node-name-and-exploit-off

**Target scheduler name and EXPLOIT=0** — attached to `if [ "$EXPLOIT" = "0" ]; then` (line 3314)

```text
Scheduler last; its node name must stay target_scheduler in the root namespace
or the targets yaml's parameter key will not match. Skipped entirely with
EXPLOIT=0: exploitation_enabled=false already makes the planners ignore
releases, but leaving the scheduler running would still put TreeTarget traffic
on the ungated global /exploration/targets bus during a comms run.
```

### planner-census-count

**Counting planner binaries** — attached to `NPLAN=$(count_own "[l]ib/explo_planner/explo_planner_node")` (line 3328)

```text
Count real node binaries only, by the absolute install path that ONLY the
launched binary carries. Matching the bare node name instead counts anything
whose cmdline merely mentions it: the `ros2 run` python wrapper (which is why
"bin/ros2" used to be filtered out), but also any shell watching this run from
another terminal — a `pgrep -f explo_planner_node` in a sibling process aborts
the whole stack here, 8 seconds after a healthy start.
The leading [l] is load-bearing, not a typo: ps and grep run concurrently in
this pipeline, so ps lists the grep too. A literal pattern matches the grep's
OWN cmdline and every count comes back one too high. Bracketing one character
makes the pattern text differ from the text it matches.
count_own, not a machine-wide `grep -c`: see its definition. The [l] bracket
stays load-bearing there for the same reason it was here.
$N_ROBOTS, not a literal 2: the P7 harness launches one planner per robot in
$ROBOTS, so a three-robot scenario brought the whole stack up and then died
here, 8 seconds later, on "expected exactly 2 ... found 3". The count is still
exact — one planner per robot, no more and no fewer — it just reads the
expected number from the same roster the launch loop iterates.
```

## 6b. comms gates (COMMS=1)

### comms-gates-bringup-strict

**Comms bring-up gates and GATES_STRICT** — attached to `if [ "$COMMS" = "1" ]; then` (line 3350)

```text
Run AFTER the planners, because two of the four can only be judged once the
intent endpoints exist. GATES_STRICT=1 aborts the run on a failed bring-up
check; the default reports and continues, because a leak found at t=0 is
still worth seeing the run for. The watcher keeps polling overflow and the
odom watchdog for the whole run and its verdict lands in the report file —
an overflow at minute 40 invalidates the run just as surely as one at t=0.
Strict by default whenever the emulator is in the loop (inert with COMMS=0).
Every gate here detects a condition the plan treats as run-invalidating —
intent leakage past the radio model, a QoS mismatch that silently forms no
relay, reliable-backlog overflow, a dead odom source. Continuing past one does
not produce a degraded run, it produces a run that measures the wrong thing
while looking healthy, and an hour of sim time is more expensive than a
restart. GATES_STRICT=0 still forces the old report-and-continue behaviour.
GATES_STRICT: resolved with the other run-control knobs above (the
`C2 (2026-09-14)` block, ~:2119).

All eight of these back-references read "~:1801" until 2026-09-18, which is
the snap-scrub/ROS-sourcing block and has never resolved a knob. The grep
string is the durable half of the reference; the number is a convenience that
goes stale on the next insertion above it, as it already has once. If they
disagree, believe the grep.
```

### comms-gates-capture-status

**Capturing the gate check's status** — attached to `GATE_EXTRA=()` (line 3375)

```text
Capture the GATE's status, not the pipeline's. `if cmd | tee ...; then`
tests tee's exit status, which is 0 unless the disk fills — so every gate
failure announced itself as success and GATES_STRICT was dead code. This
script runs `set -u` without `pipefail`, and adding pipefail globally would
change the failure semantics of every other pipeline in here (several
legitimately end in `|| true` grep counts), so the status is taken directly
instead.
The TeamWorld relay is required to exist only when the stream is on;
team_world_hz defaults to 0.0, so demanding the relay unconditionally would
fail every COMMS=1 run without the exchange for being configured correctly.
```

### comms-gates-expect-outage-watch

**Outage expectation and the gate watcher** — attached to `start gateswatch "$OUTDIR/gates_watch.log" \` (line 3401)

```text
EXPECT_OUTAGE=0 for the plan's phase-1 control arm: it runs through the
emulator (so the relay hop, delay_ms and rx QoS are all present) at a
tx_power_dbm where the link is meant to stay up, and the outage gate would
otherwise fail every single control run for behaving as designed.
The link trace itself now starts with the emulator, up in §2 — see the note
there. Only the gate watcher, which needs the planners' intent endpoints to
exist before it can judge anything, still starts here.
```

### t0-wall-bringup-time

**Wall clock at T0** — attached to `T0_WALL=$SECONDS` (line 3416)

```text
Wall clock at T0, so the run-end block can bank BOTH a total cell wall time
and a bring-up-free one. $SECONDS is a bash builtin counting from shell start
and is never reassigned in this file (grep '^SECONDS='), so T0_WALL is
literally "seconds of bring-up" and $SECONDS at the end is the whole cell.
The decision this exists for: generation 9's FOV bundle is 6x the rays of
generation 8 at up to 2x the length with NO candidate-count offset (the
n_yaw 4->1 cut is inert on this path — FRONTIER_ONLY=1), and nothing measures
plan CPU. The agreed gate is to read the wall/sim ratio off the FIRST cell
against the ~1.149 generation-8 baseline rather than guess, and that read was
not possible from a manifest that banked started_utc and no finish stamp.
```

## 7. hold, watching component health

### hang-gate-threshold-sizing

**Sizing the hang-gate threshold** — attached to `if [ "$DURATION_S" != "0" ] && [ "$((HANG_HB * 60))" -ge "$DURATION_S" ]; then` (line 3436)

```text
Step-stall hang detector state. HANG_HB counts 60-sim-second heartbeats, so
the default aborts after 10 minutes of sim time with no step on EITHER robot.
Sized well above a slow step: a RETURN_NAV or PURSUE manoeuvre legitimately
spends minutes without completing one.
40 heartbeats x 60 sim-s = 2400 sim-s, NOT the 10 (600 s) this shipped with.
600 was set against an observed stall statistic and it collided exactly with a
CONFIGURED budget. Sized off those budgets rather than off observation,
because the corpus cannot bound this: it holds no manoeuvre that ran to its
own limit.

THE FULL LEG, in order. dispatchReconnect starts the CHASE first under
hybrid, and none of these legs emits "selected goal" (that string has exactly
one source, doPlan), so none of them advances the counter this gate watches:

  PURSUE       <= PURSUIT_BUDGET_MAX   600 s   (doPursue gives up at budget)
  RETURN_NAV   <= RECONNECT_NAV_MAX    600 s
  RETURN_SYNC  <= MIDRUN_MAX_WAIT       30 s
  pre-dispatch quiet                   ~40 s   (observed)
                                      ------
                                      ~1270 s

plus an unbounded-in-principle correction: doProximityHold REFUNDS held time
to pursue_start_time_, so PURSUE's wall duration can exceed its budget by the
accumulated hold, up to proximity_max_hold_sec (120 s, never overridden here)
— call it ~1390 s worst case.

The first version of this comment said 840 s, having simply omitted PURSUE.
That figure was already falsified by the bank: the longest banked manoeuvre
is 840.2 s, and the longest interval with NEITHER robot stepping is 1024.8
sim-s whole-run, both in hybrid cells.

1024.8 is NOT the figure this gate has to clear, though, and an earlier
version of this comment quoted 921.6, which reproduces neither number. The
gate below is disarmed once either robot is DONE (`[ "${DONE_A:-0}" = 0 ]`),
so what bounds a FALSE ABORT is the longest freeze inside the armed window,
which is 420.0 sim-s. The whole-run maximum is the larger number and the
irrelevant one: most of it accrues after a robot has declared, when the gate
is already asleep and cannot fire. 1800 would have cleared even the
whole-run maximum, but left only 1.15x over the configured ceiling.

Why the margin has to be generous in this direction specifically: only the
hybrid arm manoeuvres at all. Across the banked hybrid cells the mid-run
reconnect logic produced 302 actual dispatches, against 0 across 157 banked
off cells. (This comment once claimed "422 dispatches", which was the sum of
three different line counts, and was then "corrected" to a 302 + 113 + 7
decomposition of that same 422. The decomposition does not hold either: the
link-gate veto line is RCLCPP_INFO_THROTTLE'd at 30 s, so 113 counts PRINTED
lines and is a lower bound on vetoes, and 422 is therefore not a total of
anything. Only the 302 is a count of events, and it is the only number the
argument needs.) So a false abort is drawn from ONE arm of a
30x2 comparison, on the
primary endpoint. A late abort merely wastes wall time. The base rate of a
real hang is 0 in 500 banked cells, so the expected cost of the extra 600 s
is ~zero and it buys 1.5x over the configured ceiling. Still inside
DURATION=3000, so the gate stays live — the check below enforces that.
HANG_HB: resolved with the other run-control knobs above (the `C2 (2026-09-14)` block, ~:2119).
...and say so out loud when it is NOT, because "well inside DURATION" is a
claim about two numbers that are set independently and never compared. At
HANG_HB=30 the gate needs 1800 sim-s of frozen steps, so any cell shorter
than that ships with the hang detector unable to fire even once — inert while
still printing its armed message, which is the same silent-non-enforcement
shape the threshold change above exists to fix. Warn rather than abort: a
short debug run with no hang gate is legitimate, a CAMPAIGN with one is not,
and this line is what tells the two apart in the console log.
```

### stop-on-done-and-grace

**Stopping on DONE and the drain grace** — attached to `DONE_SINCE=-1` (line 3505)

```text
STOP_ON_DONE=1 (default) ends the run once EVERY planner is in DONE, rather
than burning the rest of DURATION_S on two parked robots. The experiment's
primary endpoint is a makespan (§5.2), so the moment the last robot finishes
is the quantity — and with a matrix of 30+ runs, the difference between
stopping there and stopping at T is most of the campaign's wall clock.

DURATION_S is still the censoring horizon T and still binds: under a degraded
radio a robot may never reach DONE, which is a censored observation, not a
hang. Both exits are needed and they mean different things — record which one
fired, because "ended at T" and "ended at DONE" are different data points.

Read from the CSV's `state` column, not the log: DONE is a state the planner
can also LEAVE (a target release pulls it back into the exploit sub-loop, and
finishOrRendezvous routes through RETURN_NAV/RETURN_SYNC before it), so a
one-shot "Exploration complete" log line is not terminal and grepping for it
would stop the run mid-manoeuvre. The column is located by scanning the header
row rather than by a fixed field number: `state` sits partway along a schema
that only ever grows at the right-hand end, so its index has changed once
already and would change again the next time a column is appended before it.
STOP_ON_DONE: resolved with the other run-control knobs above (the `C2 (2026-09-14)` block, ~:2119).
Grace, in sim seconds, between all-DONE and teardown: lets the last metrics
rows land, the reliable backlog drain, and the gate watcher see the final
state. Without it the run ends inside the very merge the endpoint measures.
DONE_GRACE_S: resolved with the other run-control knobs above (the `C2 (2026-09-14)` block, ~:2119).
```

### poll-cadence-two-budgets

**Two poll cadences** — attached to `LAST_STEPS=-1; STALL=0` (line 3539)

```text
Poll cadence, in WALL seconds. Deliberately TWO numbers, not one.

The loop's cheap work -- process liveness, and the planner's own `state`
column via an awk over a local CSV -- costs effectively nothing, so it runs
often. That is what tightens the two end-of-run boundaries: the instant every
robot reaches DONE, and the instant the grace window expires. Both were
previously detected up to one poll late, and the poll was 15 s, so a run could
be held open ~15 wall-s at each boundary for nothing.

The expensive work is sim_clock, which spawns `ros2 topic echo /clock --once`
and costs ~0.3 s of CPU per call (measured, idle machine). Polling THAT every
POLL_S would be a ~16 % continuous duty cycle per cell, and with concurrent
shards it would contend with the single render thread that gates sim time --
trading one bottleneck for another. So the clock keeps its own slower budget
and is only forced to POLL_S resolution when the answer is about to change:
on the all-DONE edge, and inside the grace window, which is exactly where the
resolution is the thing being bought.
POLL_S: resolved with the other run-control knobs above (the `C2 (2026-09-14)` block, ~:2119).
CLOCK_EVERY_S: resolved with the other run-control knobs above (the `C2 (2026-09-14)` block, ~:2119).
```

### sim-clock-deadman

**Wall-clock deadman on the sim clock** — attached to `LAST_T_SEEN=-1` (line 3560)

```text
Wall-clock deadman on the sim clock itself.

Every other end condition in this loop -- the duration cap, the step-stall
hang gate, the all-DONE grace -- is keyed on sim time, and the only wall-clock
test is process liveness. A deadlocked Gazebo passes all of them: the
processes stay alive, sim_clock keeps returning the same non-empty number, so
the duration cap is never reached, and the 60-sim-second heartbeat that drives
the hang gate never ticks either. The loop then polls silently forever. In a
sequential campaign that does not cost one cell, it costs the night: the
driver is still inside cell 7 at breakfast and cells 8..60 never started.

Deliberately generous. Sim time can legitimately stall for tens of seconds
during a heavy lidar frame or a costmap rebuild, and killing a healthy slow
cell is a worse failure than the one being prevented.
CLOCK_DEADMAN_S: resolved with the other run-control knobs above (the `C2 (2026-09-14)` block, ~:2119).
```

### clock-fail-max-pointer

**Where CLOCK_FAIL_MAX is resolved** — attached to `while true; do` (line 3581)

```text
CLOCK_FAIL_MAX: resolved with the other run-control knobs above (the `C2 (2026-09-14)` block, ~:2119).
```

### hang-gate-step-counts

**Per-robot step counts** — attached to `STEPS_NOW=""; COMPLETE_NOW=""` (line 3631)

```text
One slash-joined field per robot in roster order, instead of the SA/SB
pair this used to keep (P7). The counts are normalised to 0 here rather
than compared raw, which closes the same empty-vs-zero disarm the DONE
block below documents: `grep -c` prints NOTHING for a missing log, so a
planner that died before creating its log used to alternate ""/"0" against
whatever the other robot reported and reset STALL every heartbeat — on the
path the gate is most needed. Normalising cannot cause a false HUNG,
because a missing log also reports no DONE prefix, which is exactly the
broken run this is supposed to kill.
```

### hang-gate-what-stalls

**What the hang gate watches** — attached to `if [ "$STEPS_NOW" = "$LAST_STEPS" ]; then` (line 3648)

```text
Hang gate. A planner that cannot find an acceptable candidate returns from
doPlan and re-enters PLAN forever: no crash, no error, every process
alive, and the CSV keeps growing because the metrics timer samples every
metrics_period_sec regardless of state. So neither the liveness loop above
nor "is the CSV still being written" can see it — the only quantity that
actually stalls is the STEP counter, which advances solely on a completed
explore step. A silent hang is worth more than a crash to catch: it yields
a full-length run whose coverage curve is flat and plausible.

Not fatal if a robot has legitimately finished: DONE-idle is a terminal
state by design and its step count is supposed to stop.
```

Gen 34 (DESIGN_gen34.md §11.6, 2026-09-24): the gate also counts the node's
"Team activity:" lines, printed where the state machine follows a new team
activity, so a meeting or a chase is progress. Only robots still exploring
(no completion line) feed it, and the default HANG_HB is derived from
ROI_HALF: max(25, ceil((12*ROI_HALF + 600) / 60)).

### hang-gate-done-pattern

**The DONE pattern and empty counts** — attached to `DONE_PAT="Exploration complete\|Exploration finished"` (line 3661)

```text
The pattern must match ONLY genuine completion. Through generation 7 the
alternate was a bare `DONE`, and every planner log's line 2 reads
"DONE-SEEK DISABLED (done_seek_enabled=false, ...)" — a banner announcing
a feature is OFF. All 24 g6pilot logs matched it 3 times, so DONE_A was
never 0 and this gate could not fire in any run of any campaign that used
it. Anchored now on the two prefixes the node emits on a completion
path: "Exploration complete" and "Exploration finished". Line numbers
deliberately NOT cited — the ones that used to be here had already gone
stale by five commits, and a stale pointer in a comment about a silent
disarm is worse than no pointer.

The first of those is emitted from recordExplorationComplete, which is
the single funnel BOTH endings route through (see the comment there).
That matters: through generation 7 the step-budget ending printed
neither prefix, so the gate stayed armed across the whole homing leg
and killed cells at exactly mission_return_max_sec. Generation 8 is the
one that FIXED it, by routing both endings through the single funnel —
saying "through generation 8" would credit the fix to the generations
that still had the bug.

The empty-vs-zero handling below is the OTHER silent disarm, and it is
subtle in both directions. `grep -c` on an existing file with no match
prints "0" and EXITS 1; on a missing file it prints NOTHING and exits 2.
So `|| true` alone leaves DONE_A empty for a missing log and `[ "" = 0 ]`
is false — a planner that died before creating its log disarmed the gate
on the very path it is most needed. But `|| echo 0` is not the fix: on
the no-match-but-file-exists case grep's own "0" and the echoed "0" both
land, giving "0\n0", and `[ "0 0" = 0 ]` is false too — which disarms it
on the PRIMARY hung-run case. Calibrated against all three inputs.

ANY robot reporting DONE disarms the gate, which is what the two-term
`DONE_A = 0 && DONE_B = 0` said and is the conservative direction: with
one robot legitimately finished the team's step counter can sit still
for reasons that are not a hang.
```

Gen 34 (DESIGN_gen34.md §11.6): no longer any robot. A finished robot leaves
the gate and the rest still feed it; the gate stops only when every robot
has finished.

### hang-gate-false-abort-calibration

**Why the hang threshold is not corpus-sized** — attached to `if [ "$STALL" -ge "$HANG_HB" ] && [ "$ANY_DONE" = 0 ]; then` (line 3701)

```text
A gate that aborts valid cells is worse than one that never fires, and
the calibration that used to justify 600 s here was WRONG in a way worth
recording, because it is the reason a one-arm dropout mechanism shipped:

  "the longest interval in which NEITHER robot advanced a step was
   59.5 s across the 16 banked cells … a full-length reconnect
   manoeuvre never came close, since the two robots do not stall in
   lockstep."

Three things were wrong with it. (1) 59.5 s is the max gap BETWEEN
CONSECUTIVE steps, which cannot see a stall that is never followed by
another step — and a manoeuvre that ends in a latch produces exactly
that. Recomputed over the window this gate is actually armed in (start
-> first DONE prefix), the max is 116.7 s wall / ~101 sim-s. (2) The two
largest are g6pilot_hybrid_seed105 and _seed106, in BOTH of which both
robots dispatched 8-18 s apart — they stall in lockstep by construction,
because they cross the same silence gate on the same shared outage.
(3) Every banked manoeuvre was truncated by the coverage latch at
28.5-82.9 s, so "never came close" was measured on a corpus containing
no full-length manoeuvre at all.

The threshold is therefore sized off the configured budgets (see
HANG_HB) rather than off this corpus, which cannot bound it.

Nor did it bound the HOMING leg, for a fourth reason: all 24 banked
ROBOT-RUNS — 12 cells, two robots each, not 24 cells — ended by LATCH,
which prints a matching prefix and disarms this gate before homing
starts. Half the corpus this line used to claim. That bound is
structural instead, which is why halving it changes nothing — the
funnel line is printed on every ending, before startReturnHome.
```

### horizon-after-all-done

**Horizon reached after all-DONE** — attached to `if [ "$DURATION_S" != "0" ] && [ "$T" -ge $((T0 + DURATION_S)) ]; then` (line 3760)

```text
The horizon. A run that reaches it while the DONE drain grace is still
counting down did NOT get cut off unfinished: every planner had already
declared, at DONE_SINCE. Only the ${DONE_GRACE_S}s drain hold spilled past
it. Calling that "censored_at_T" labels a completed run as an incomplete
one, and it does so ASYMMETRICALLY — the slower arm finishes nearer the
horizon, so it collects more of these — which turns a labelling bug into an
apparent arm effect.

DONE_SINCE is NOT guaranteed to precede the horizon. It is stamped from $T,
and $T is only resampled every CLOCK_EVERY_S=${CLOCK_EVERY_S}s of wall time
outside the grace path, so the reading that first satisfies the all-DONE
condition can already be past T0+DURATION_S. The declaration is still real
— every planner is DONE — but "inside the horizon by construction" is not a
property this loop maintains, and an earlier version of this comment said
it was.

The drain is what guarantees the trailing rows are flushed, so a relabel
that fires BEFORE it elapsed is promising a clean end the harness did not
actually wait for. That is recorded as its own manifest key rather than as
a third run_end_reason: gate_g8.py, modes_compare.py and reconnect_value.py
all enumerate the two legal reasons, and a new string would silently drop
those cells out of every one of them.

The primary endpoint reader is immune (event_log.py decides censoring from
the planner's own events, never from this string), so this is a fix to the
manifest and to everything that reads it, not to the headline result.
```

### run-end-t-sim-bias

**run_end_t_sim is not the endpoint** — attached to `echo "run_end_t_sim=$((T - T0))" >> "$OUTDIR/run_manifest.txt"` (line 3806)

```text
T0 IS WALL-PACED, SO THIS IS NOT A CLEAN DURATION. Bring-up deadlines are
wall-clock, so how much SIM time elapses before T0 depends on the real-time
factor, which depends on team size: RTF 0.9063 at N=2 vs 0.4217 at N=4 gave a
measured pre-T0 offset of 28.80 +- 2.02 s and 24.71 +- 0.99 s respectively --
a systematic ~4.1 sim-s bias that is ALIGNED WITH N, on top of the
DONE_GRACE_S drain that is inside this number by construction. Kept because
several scripts read it and because it is the only figure that describes the
harness's own horizon, but it is not the endpoint.
```

### run-end-t-rel-endpoint

**Endpoint from the planners' run_end** — attached to `_tmax=""` (line 3815)

```text
C2: the endpoint, taken from the planners themselves. Each robot's `run_end`
event carries t_rel_sec, measured from that planner's own t0, so it has no
wall-paced zero point and no bring-up offset. Written per robot AND as the
max, because "when did the team finish" is the max over robots while "how
long did this robot run" is the per-robot value, and conflating them is how
an order statistic got reported as a team mean. Absent/unparseable robots are
recorded as empty rather than 0 -- a missing endpoint must not read as a fast
one.
```

### done-drain-complete-key

**The done_drain_complete key** — attached to `echo "done_drain_complete=${DONE_DRAIN_COMPLETE:-}" >> "$OUTDIR/run_manifest.txt"` (line 3837)

```text
1 = the full ${DONE_GRACE_S}s drain elapsed after the last planner declared,
0 = all_done was reached but the horizon cut the drain short, empty = the run
never reached the all-DONE state at all. Emitted unconditionally, including
the empty case, so that an absent key means an OLD manifest and never "this
run happened not to drain" — the distinction that made done_action_in_params
worth fixing in the same file.
```

### run-end-wall-cost

**Banking cell wall-clock cost** — attached to `echo "run_end_wall_sec=$SECONDS" >> "$OUTDIR/run_manifest.txt"` (line 3844)

```text
Cell cost, banked so it can be read rather than reconstructed from directory
mtimes. Two numbers because they answer two questions and the ratio of the
wrong one is meaningless:
  run_end_wall_sec          — the WHOLE cell, bring-up included. This is the
                              quantity the ~79 s + 1.149x t_sim budget model
                              predicts, so it is the one to compare against it.
  run_end_wall_sec_since_t0 — wall seconds after T0 only. Divide this by
                              run_end_t_sim for a real-time factor that does
                              not have bring-up folded into it.
Neither is a planner-CPU measurement and neither should be quoted as one: a
cell is Gazebo plus scovox plus the planners, and at ~1 core per sim-second
scovox dominates. What they CAN do is answer "did generation 9's raycast
change the cell budget", which is the whole point — if this ratio matches
generation 8's, the 12x upper bound on the FOV cost did not matter, and if it
does not, it shows up on cell 1 and costs one cell to learn.
Absent keys mean an OLD manifest, never a zero-cost cell — same rule as
done_drain_complete above.
```
