# explo_planner

Next-Best-View (NBV) exploration planner for ROS 2. It selects viewpoints by
**Expected Information Gain (EIG)** computed from
[SCovox](https://github.com/kalhansb) Beta-conjugate occupancy maps, drives a
robot toward the chosen goal through the navigation stack, and logs per-step
metrics for experiments.

It runs single- or multi-robot: each robot plans against its own fused SCovox
view and (optionally) deconflicts viewpoints with teammates through a MinPos
intent table. The multi-planner comparison harness (entropy / frontier / random
/ ssmi baselines, used for the ablation studies) lives on the `experiments`
branch; `main` ships the EIG planner only.

## How it works

Each PLAN cycle the node:

1. **Generates candidates** — a polar grid of viewpoints (radius × ring × yaw)
   around the current robot pose, **plus one candidate per frontier-cluster
   centroid** anywhere in the ROI (the long-range escape from a locally
   exhausted neighbourhood). Both are filtered against a region-of-interest box
   and, when `use_planning_map: true`, the 2D inflated `planning_map` for
   free/occupied.
2. **Scores each candidate** — simulated FOV ray-casting over the SCovox map
   yields an expected information gain (EIG) from the Beta-conjugate occupancy.
3. **Costs each candidate** — bounded grid Dijkstra over the `planning_map`
   gives a reachable path cost. With the planning_map disabled (the default,
   `use_planning_map: false`) the cost is straight-line distance and there is no
   2D reachability/obstacle filtering — avoidance is left to the nav stack.
4. **Picks the best** — SSMI-style information-per-distance utility
   (Asgharivaskasi & Atanasov, TRO 2023):

   ```
   U(c) = info_gain(c) / (ε + path_cost(c))
   ```

   Longer paths dilute a candidate's score; unreachable candidates get
   `U = −∞` and sort last.
5. **Navigates** to the goal with a distance-scaled timeout, a no-progress
   watchdog, and a TTL/radius blacklist of recently-failed goals to avoid
   re-picking unreachable targets.
6. **Terminates** when the unknown fraction in the ROI stays below a threshold
   for N consecutive cycles, or `max_steps` is reached.

Multi-robot coordination (optional) uses a MinPos intent table so teammates
deconflict their selected viewpoints. Each robot plans against its own fused
SCovox view — there is no central merger.

### Reconnection: rendezvous / pursuit / hybrid (multi-robot, default on)

A robot that loses comms keeps exploring on its own rather than halting. When it
exhausts its exploration goals (the coverage termination above) it does **not**
stop while a teammate is still out of range: it runs the manoeuvre selected by
`reconnect_mode` and **holds at its endpoint until the whole team is back in
comms**, then re-plans against the now-merged map. `rendezvous` (the legacy
behaviour, and the code default) drives back to the **last-connected anchor** —
the pose where it last heard a teammate. For a robot-carried radio mesh the
planner also keeps a per-peer **last-contact record** (its own pose, the peer's
advertised pose, and the peer's declared goal at the last received intent):
`pursuit` chases the missing peer's declared goal and then its last heard pose
on a staleness-scaled time budget, and `hybrid` (the shipped yaml default)
chases on the budget and then falls back to the deterministic meeting point —
the midpoint of the last-contact pose pair, computed independently by both
sides. Once the team reconnects, if new frontiers appeared it disperses again
(MinPos splits them); if not, the whole team reaches `DONE` together. A robot
can therefore only finish when the full team is present and the merged map is
saturated, so nobody quits while a teammate is still exploring — and a robot
that *has* finished keeps beaconing its presence while idling in `DONE`
(`done_action: idle`), so a later finisher can count it. See
`docs/planner_method.md` §10.2 for the guarantee arguments and
`doc/limitations.md` §10–11 for the accepted failure modes.

This is `reconnect_enabled` (called `rendezvous_enabled` before 2026-09-03,
still accepted) and it defaults **on**, but it only *activates*
where it is meaningful: `coordination_enabled` must be on (the barrier waits on
peer claims) and `rendezvous_expected_peers` must be positive. Single-robot runs
and one-robot teams are therefore unaffected — it stays inert, and behaviour is
bit-for-bit the finish-and-stop of before. Set `reconnect_enabled:=false` to
force that independent finish even in a multi-robot run.

The anchor needs no configuration — it is recorded automatically from incoming
peer intents. The barrier waits for `rendezvous_expected_peers` teammates (the
multi-robot launch sets this from the team size). By default the wait is
unbounded (STAY until all connected); set `rendezvous_max_wait_sec > 0` as a
field escape hatch so a robot whose teammate died doesn't hold the anchor
forever. `max_steps` still ends a run directly, independent of the barrier.

### Coordinated proximity stop (multi-robot, default on)

MinPos deconflicts *goals*, not *paths*: two robots' commanded routes can still
cross (the forest trial's routes cross with 0.0 m closest approach). While a nav
goal is in flight, the planner therefore watches its teammates' live poses and
**yields when a higher-priority teammate is moving nearby**: it publishes a
brake goal at its own pose — which the navigator reads as an immediate arrival,
emptying the path and zeroing `cmd_vel` within ~100–150 ms — then parks in a
`PROXIMITY_HOLD` state and resumes the same goal once the peer has cleared off or
parked. Simply ceasing to publish `goal_pose` would *not* stop the robot: the
navigator latches the last goal it accepted and drives it to completion.

The planner also attempts a `NavigateToPose` cancel alongside the brake, but
**that path is dead on this stack and has never fired**: the navigator is
`simple_nav_3d`, which serves no action server, so the cancel client never
becomes ready and every call site skips it. The brake goal is the *only* stop
mechanism — read it as one mechanism, not two. See the block at
`nav_cancel_client_`'s construction in `explo_planner_node.cpp` for why the
client is kept anyway.

Right of way is the **lexicographically smaller `robot_name`** — the same total
order as the MinPos tiebreak. It is computed from ids alone, so both robots of a
pair always agree on who yields: exactly one stops, never both (standoff) and
never neither (race). The hold enters below `proximity_hold_dist_m` (5 m) and
releases beyond `proximity_resume_dist_m` (6 m, hysteresis). The 5 m is sized
against the **reaction budget**, not just the documented 1.5 m panic line: pose
age + tick + brake-goal propagation + braking exceeds a second while the pair keeps
closing at the peer's speed, which consumed the whole margin of the earlier 3 m
default at field closing speeds. A peer that has stopped moving for
`proximity_peer_static_sec` is treated as **parked** and released — a stationary
robot is an ordinary mapped obstacle for the navigator, and holding against one
would deadlock (e.g. a teammate waiting at its rendezvous anchor) — **unless it
sits inside `proximity_parked_keep_dist_m`** (1.5 m): a peer parked closer than
the panic line keeps the hold until it moves off or `proximity_max_hold_sec`
(120 s) forces a loud resume. Stationary planner states (dwell, integrate, plan)
are exempt from holding for the same reason.

Peer poses come from the 1 Hz intent heartbeat automatically, plus any
`proximity_peer_pose_topics` entries (`"<robot>:<topic>"`). **On hardware, point
those at the peers' localiser robot pose** (map frame, ~10 Hz, whatever that
topic is called on the platform): the heartbeat alone leaves metre-scale pose
lag at field closing speeds. Misconfiguration does not stay silent: the planner
warns at startup when no pose topics are set, warns repeatedly while a
configured topic has delivered nothing, and warns when
`coordination_enabled=false` leaves the guard with no input at all (no heartbeat
and no pose topics). The guard is inert in single-robot runs (no peer is ever
tracked).

For the operator and post-hoc analysis: the latched topic
`/<robot>/proximity_hold_state` carries the current state
(`hold peer=... dist=...` / `clear reason=... held=...`, reasons
`clear|parked|stale|max-hold`),
and the per-step CSV gains cumulative `prox_hold_count` / `prox_hold_total_sec`
columns so holds can be correlated with the trajectory. Held time does not
refund the nav budget: the resume continues the goal's drive-time clock where
the hold interrupted it, so repeated holds cannot grant one goal unbounded time.

This is best-effort **coordination, not a certified safety stop**: it needs live
peer data and both planners alive, and it rests on the brake goal alone (the
cancel never lands — see above); the right-of-way robot keeps driving and relies
on its own local map to skirt the held robot. The crewed 1.5 m panic-stop
procedure remains the hard backstop in the field.

## Perceptive exploitation

On top of exploration the planner runs an **exploitation** overlay (forest
inspection, Scenario 2): when a tree target arrives it switches `EXPLORE →
EXPLOIT`, circles the trunk at occlusion-free **vantage points** and dwells at
each so the rosbag captures overlapping RGB-D/LiDAR views, then reverts to
exploration when the target's vantages are covered. Sensor fusion of the
captured data is **offline** — the planner only positions and dwells.

Targets arrive on a shared topic (`targets_topic`, default
`/exploration/targets`) as `explo_planner_msgs/TreeTarget` messages. Two
producers ship with the package: the live `tree_detector_node` (segments trunks
out of the fused SCovox map and emits each once, with position-derived stable
ids so every robot names a tree identically), and the time-based
`target_scheduler_node` (publishes a preselected list on a schedule — the
repeatable-experiment stand-in). The exploitation launch picks with
`use_detector:=true|false`; the planner is agnostic to which one feeds it.

**Vantage-point selection** (per active target, recomputed each `EXPLOIT_PLAN`
tick so the map can improve a previously-blocked angle):

1. **Generate** `n_vantages` viewpoints evenly spaced on a standoff circle
   around the trunk (3 ⇒ ~120° apart). Standoff `= radius + vantage_standoff_m`,
   clamped to `[fov_min_range + radius, fov_max_range]`; each viewpoint faces the
   trunk.
2. **Validate** each: inside the ROI, a free cell on the inflated
   `planning_map`, reachable via the cost grid, and **line-of-sight clear** —
   a ray-march to the trunk hits no known-occupied voxel before the trunk
   surface (voxels at/inside the trunk are the trunk itself, not an occluder).
3. **Select** the nearest (cost-grid path cost) valid, unvisited, non-blacklisted
   vantage; navigate → dwell → mark visited. The target is **complete** once
   `min_vantages_required` clear-LoS vantages are dwelled, or **partial** once no
   selectable vantage remains.

Vantages flow through the same candidate → cost → `RobotIntent`/MinPos pipeline
as exploration, and multi-robot vantage deconfliction is live: each robot
claims its selected vantage through the intent table (per-vantage MinPos, with
yield rules for in-flight, mid-dwell and parked peers), and every intent
carries a **team dwell-credit mask**, so with coordination on
`min_vantages_required` is a team quota — robots take different angles on the
same tree and their clear-LoS dwells count together.

## Architecture

One planner node per robot. Each planner consumes its **own robot's** fused
map and talks to peers only through the shared intent topic:

```
                         (per robot)
  dscovox merger ──ScovoxMap──▶ ┌───────────────────┐ ──PoseStamped──▶ navigator
  target producer ──TreeTarget▶ │ explo_planner_node │ ──MarkerArray──▶ RViz
  tf: map → base_link ────────▶ │  (10 Hz tick)      │ ──CSV──▶ output_csv
                                └─────────┬─────────┘
                                          │ RobotIntent (pub + sub)
                             /exploration/intents  ◀── every peer planner
```

The node logic is split into small, unit-tested modules:

| Module | Responsibility |
| --- | --- |
| `explo_planner_node` | State machine (PLAN → NAVIGATE → INTEGRATE → LOG_STEP, plus EXPLOIT_PLAN/EXPLOIT_DWELL) and ROS wiring |
| `candidate_generator` | Polar-grid viewpoint candidate generation |
| `fov_evaluator` | Simulated FOV ray-casting for viewpoint evaluation |
| `scoring` | Viewpoint scoring (EIG / entropy / frontier) |
| `cost_grid` | Bounded 8-connected grid Dijkstra path cost over `planning_map` |
| `plan_map_query` | Pure 2D occupancy-grid cell queries |
| `map_cache` | Read-only Bonxai grid rebuilt from ROS map messages |
| `failed_goal_blacklist` | TTL + radius blacklist of recently-failed goals |
| `coordination` | Multi-robot intent table + MinPos deconfliction |
| `proximity_guard` | Coordinated proximity-stop arbiter (yield/hold/resume decisions) |
| `target_queue` | Tree-target queue (ingest/dedup/lifecycle) for exploitation |
| `vantage_planner` | Vantage-point generation + line-of-sight occlusion test |
| `tree_detector` | Trunk segmentation over the fused map (stable position-derived ids) |
| `metrics_logger` | Per-step CSV metric logging |
| `planner_util` | Small shared pure helpers |

Two further executables drive the exploitation targets topic: the live
`tree_detector_node` (wraps the `tree_detector` module) and the time-based
`target_scheduler_node` (publishes a preselected list on a schedule).

## Build

ROS 2 (`ament_cmake`) package. It depends on the SCovox packages
(`scovox_core`, `scovox_msgs`), so build it in a workspace that overlays
SCovox:

```bash
cd <ws>
colcon build --packages-up-to explo_planner
source install/setup.bash
```

## Run

Single-robot experiment:

```bash
ros2 launch explo_planner exploration_experiment.launch.py \
  robot:=atlas max_steps:=200 \
  output_csv:=/tmp/exploration_eig.csv
```

Multi-robot (one planner per robot; expects Gazebo + per-robot mapping/nav
already running):

```bash
ros2 launch explo_planner multi_robot_exploration.launch.py \
  robots:=atlas,boreas coordination_enabled:=true
```

Exploration **+ exploitation** (planner + the time-based target scheduler;
expects the fused map + nav stack already running):

```bash
ros2 launch explo_planner exploitation_experiment.launch.py \
  robot:=atlas max_steps:=200 \
  output_csv:=/tmp/exploitation.csv \
  targets_file:=<pkg>/config/targets.yaml
```

### Real-robot trials (Docker)

For a full two-robot field trial — bring-up order, per-robot configuration,
what to watch, panic stops and data offload — see
[`../docs/user_manual.md`](../docs/user_manual.md). It spans the localiser,
mapping, nav and planner, so it sits at the repository root rather than in this
package. The build and launch essentials are below.

The planner has no container of its own — build and run it in an overlay
workspace inside the running `scovox` container (image `scovox:jazzy`, the
sibling repo's `compose.yaml`; it provides ROS 2 Jazzy plus the
`scovox_core`/`scovox_msgs` dependencies at `/scovox/install`):

```bash
cd ../scovox && docker compose up -d && cd -
docker exec scovox bash -c 'rm -rf /tmp/explo_ws/src/explo_planner && mkdir -p /tmp/explo_ws/src'
docker cp . scovox:/tmp/explo_ws/src/explo_planner   # always onto a fresh dir, never an existing one
docker exec scovox bash -c 'source /opt/ros/jazzy/setup.bash && source /scovox/install/setup.bash \
  && cd /tmp/explo_ws && colcon build --packages-select explo_planner --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

Prerequisites on the robot: the `map → odom → base_link` TF tree from the
sibling `hmr_localisation` NDT localizer, per-robot SCovox + DSCovox mapping
(scovox README, "Multi-robot mapping"), and a nav stack listening on
`goal_topic`. Then, per robot:

```bash
docker exec scovox bash -c 'source /opt/ros/jazzy/setup.bash && source /scovox/install/setup.bash \
  && source /tmp/explo_ws/install/setup.bash \
  && ros2 launch explo_planner exploration_experiment.launch.py robot:=atlas'
```

All tuning lives in [`config/shared_params.yaml`](config/shared_params.yaml)
(the shared parameter file, loaded by every launch file). It now ships **field defaults**: `use_sim_time:
false`, `done_action: idle`, `terrain_relative_z: true`, `coordination_enabled:
true`, and the forest-AO ROI. Sim and bag runs must pass `use_sim_time:=true`
(every launch file declares the argument, and it overrides the yaml) — with
`true` and no `/clock` publisher the planner's 10 Hz tick timer never fires and
the node sits silently idle. Keep `roi_min_z`/`roi_max_z` inside the multi-robot
share z-band; note that with `terrain_relative_z: true` they are **relative to
the robot's z**, so the absolute share band must be a superset of everywhere
that window can sit (see the KEEP-IN-SYNC comment in the yaml).

Two settings must be matched to the navigator on each platform:

- `goal_xy_tolerance` / `goal_yaw_tolerance` must be strictly **looser** than the
  navigator's own stop condition. If they are tighter the navigator stops inside
  its tolerance but outside the planner's, the planner never registers arrival,
  and it blacklists a goal the robot is standing on. The node warns at startup if
  either is below 0.3. The navigator here is `simple_nav_3d`, whose UGV stop
  condition is `ugv.goal_xy_tol_m` / `ugv.goal_yaw_tol_rad` (both 0.2 as
  shipped) — *not* nav2's `general_goal_checker`, which has never been in the
  loop for any campaign in this repo. The shipped 0.4 is not sized off either
  tolerance: four stacked grid roundings put the median ts1b arrival 0.269 m from
  the commanded point, so the gate has to clear the measured park *distribution*.
- `goal_republish_sec` throttles the keep-alive re-send of an unchanged goal. The
  throttle was added against a nav2 failure mode (`GoalUpdated` firing
  continuously and burning `RecoveryNode`'s retries) that **cannot occur on this
  stack**, and the old advice to set `0` inverts here: on `simple_nav_3d` an
  unchanged re-send is a genuine no-op, while a goal published while the
  navigator is *down* is lost in silence with no action feedback to notice it
  with. Keep it non-zero; 5.0 s is cheap insurance against a navigator that
  restarts mid-run.

### Key topics

| Direction | Topic | Purpose |
| --- | --- | --- |
| sub | `/<robot>/dscovox_node/scovox` | SCovox Beta-conjugate occupancy map |
| sub | `/<robot>/dscovox_node/planning_map` | 2D inflated grid for filtering / cost (only when `use_planning_map: true`) |
| sub | `/exploration/targets` | Tree targets to exploit (`TreeTarget`; shared) |
| pub | `goal_topic` | Selected NBV / vantage goal pose for the nav stack |
| pub | `~/candidates` | Candidate / vantage markers (RViz) |
| pub | `/exploration/intents` | MinPos intents (always published; consumed by peers when coordination is on) |

## Configuration

All parameters live in [`config/shared_params.yaml`](config/shared_params.yaml),
which is heavily commented. Common overrides:

- `use_planning_map` — enable the 2D `planning_map` (free/occupied filter +
  cost-grid reachability) in both exploration and exploitation. **Off by default**;
  when off the planner never subscribes to it and runs on straight-line costs.
- `roi_min/max_x/y` — exploration region. Also the box the coverage-done
  measure is taken over, so unreachable columns inside it put a permanent floor
  under `done_unknown_fraction`. Only needs keeping in sync with the
  `planning_map` when `use_planning_map` is on — it is off by default, and the
  dscovox merger publishes no `planning_map`. The tighter phase-1 AO box is
  commented next to these values in the yaml — set it there when phase 2 is
  not run the same day.
- `roi_min/max_z` — vertical band, **robot-relative** when
  `terrain_relative_z` is on. It must stay wide enough to contain the whole
  ground-search window plus the re-band hysteresis; the node warns at startup
  if it does not (see the invariant in the yaml).
- `candidate_*` — polar candidate-grid density and radii
- `fov_*` — FOV geometry for information-gain ray-casting
- `coordination_enabled` — turn multi-robot MinPos on/off
- `reconnect_enabled` / `rendezvous_expected_peers` / `rendezvous_max_wait_sec`
  / `reconnect_mode` / `pursuit_budget_max_sec` / `pursuit_staleness_max_sec`
  — reconnection barrier and manoeuvre (see "Reconnection: rendezvous /
  pursuit / hybrid")
- `exploitation_enabled` — turn the exploitation overlay on/off
- `n_vantages` / `min_vantages_required` / `vantage_standoff_m` /
  `exploit_dwell_sec` — vantage geometry and dwell behaviour

The preselected target schedule for `target_scheduler_node` lives in
[`config/targets.yaml`](config/targets.yaml).

## Tests

```bash
colcon test --packages-select explo_planner
colcon test-result --verbose
```

GTest suites cover scoring, candidate generation, FOV evaluation, cost grid,
coordination, plan-map queries, the failed-goal blacklist, planner utils, the
map cache, the proximity guard, the tree detector, the tree-target queue
(dedup/lifecycle), and the vantage planner (spacing/standoff/line-of-sight).

## License

BSD-3-Clause.
