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

## Architecture

The node logic is split into small, unit-tested modules:

| Module | Responsibility |
| --- | --- |
| `explo_planner_node` | State machine (PLAN → NAVIGATE → DONE) and ROS wiring |
| `candidate_generator` | Polar-grid viewpoint candidate generation |
| `fov_evaluator` | Simulated FOV ray-casting for viewpoint evaluation |
| `scoring` | Viewpoint scoring (EIG / entropy / frontier) |
| `cost_grid` | Bounded 8-connected grid Dijkstra path cost over `planning_map` |
| `plan_map_query` | Pure 2D occupancy-grid cell queries |
| `map_cache` | Read-only Bonxai grid rebuilt from ROS map messages |
| `failed_goal_blacklist` | TTL + radius blacklist of recently-failed goals |
| `coordination` | Multi-robot intent table + MinPos deconfliction |
| `metrics_logger` | Per-step CSV metric logging |
| `planner_util` | Small shared pure helpers |

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

### Key topics

| Direction | Topic | Purpose |
| --- | --- | --- |
| sub | `/<robot>/dscovox_node/scovox` | SCovox Beta-conjugate occupancy map |
| sub | `/<robot>/dscovox_node/planning_map` | 2D inflated grid for filtering / cost |
| pub | `goal_topic` | Selected NBV goal pose for the nav stack |
| pub | `~/candidates` | Candidate viewpoint markers (RViz) |
| pub | `/exploration/intents` | MinPos intents (multi-robot only) |

## Configuration

All parameters live in [`config/exploration_params.yaml`](config/exploration_params.yaml),
which is heavily commented. Common overrides:

- `roi_min/max_x/y` — exploration region (keep in sync with the `planning_map`)
- `candidate_*` — polar candidate-grid density and radii
- `fov_*` — FOV geometry for information-gain ray-casting
- `coordination_enabled` — turn multi-robot MinPos on/off

## Tests

```bash
colcon test --packages-select explo_planner
colcon test-result --verbose
```

GTest suites cover scoring, candidate generation, FOV evaluation, cost grid,
coordination, plan-map queries, the failed-goal blacklist, and planner utils.

## License

BSD-3-Clause.
