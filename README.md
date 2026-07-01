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
   around the current robot pose, filtered against the 2D inflated
   `planning_map` for free/occupied and a region-of-interest box.
2. **Scores each candidate** — simulated FOV ray-casting over the SCovox map
   yields an expected information gain (EIG) from the Beta-conjugate occupancy.
3. **Costs each candidate** — bounded grid Dijkstra over the `planning_map`
   gives a reachable path cost.
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

## Perceptive exploitation

On top of exploration the planner runs an **exploitation** overlay (forest
inspection, Scenario 2): when a tree target arrives it switches `EXPLORE →
EXPLOIT`, circles the trunk at occlusion-free **vantage points** and dwells at
each so the rosbag captures overlapping RGB-D/LiDAR views, then reverts to
exploration when the target's vantages are covered. Sensor fusion of the
captured data is **offline** — the planner only positions and dwells.

Targets arrive on a shared topic (`targets_topic`, default
`/exploration/targets`) as `scovox_msgs/TreeTarget` messages. There is **no live
tree detection**: today a small time-based `target_scheduler_node` publishes a
preselected list on a schedule, but a real detector can publish the identical
message on the same topic later with no planner change.

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
as exploration, so multi-robot vantage deconfliction (different robots taking
different angles on one tree) drops in later with no new selection code.

## Architecture

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
| `target_queue` | Tree-target queue (ingest/dedup/lifecycle) for exploitation |
| `vantage_planner` | Vantage-point generation + line-of-sight occlusion test |
| `metrics_logger` | Per-step CSV metric logging |
| `planner_util` | Small shared pure helpers |

The `target_scheduler_node` executable is the time-based tree-target publisher
(stand-in for a detector) that drives the exploitation targets topic.

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

### Key topics

| Direction | Topic | Purpose |
| --- | --- | --- |
| sub | `/<robot>/dscovox_node/scovox` | SCovox Beta-conjugate occupancy map |
| sub | `/<robot>/dscovox_node/planning_map` | 2D inflated grid for filtering / cost |
| sub | `/exploration/targets` | Tree targets to exploit (`TreeTarget`; shared) |
| pub | `goal_topic` | Selected NBV / vantage goal pose for the nav stack |
| pub | `~/candidates` | Candidate / vantage markers (RViz) |
| pub | `/exploration/intents` | MinPos intents (multi-robot only) |

## Configuration

All parameters live in [`config/exploration_params.yaml`](config/exploration_params.yaml),
which is heavily commented. Common overrides:

- `roi_min/max_x/y` — exploration region (keep in sync with the `planning_map`)
- `candidate_*` — polar candidate-grid density and radii
- `fov_*` — FOV geometry for information-gain ray-casting
- `coordination_enabled` — turn multi-robot MinPos on/off
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
tree-target queue (dedup/lifecycle), and the vantage planner (spacing/standoff/
line-of-sight).

## License

BSD-3-Clause.
