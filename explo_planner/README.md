# explo_planner

Next-Best-View (NBV) exploration and perceptive exploitation planner for ROS 2.
It picks viewpoints by **expected information gain (EIG)** over a fused
[SCovox](https://github.com/kalhansb/scovox) Beta-conjugate occupancy map,
sends each one to the navigation stack as a goal pose, and logs per-step
metrics to CSV. When a tree target arrives it circles the trunk and dwells at
vantage points so the rosbag captures close-range views.

It runs alone or in a team. Each robot plans against its own fused map and
talks to teammates only through a shared intent topic. There a MinPos intent
table deconflicts their goals: the robot closer to a contested viewpoint keeps
it. This package ships the EIG planner only. The
baselines used for the ablations (entropy, frontier, random, SSMI) are on the
`experiments` branch.

To build, see the [root README](../README.md#build).

## How it works

Each planning cycle the node:

1. **Generates candidates.** One candidate per frontier-cluster centroid in
   the region of interest (ROI). An optional polar grid of 96 viewpoints
   around the robot (`candidate_enable_polar`, off in `shared_params.yaml`)
   adds local options. Candidates outside the ROI are dropped. With
   `use_planning_map: true`, a candidate on a blocked or unreachable cell of
   the 2D planning map first moves to the nearest good cell within
   `candidate_snap_radius_m`, and is dropped if there is none. Frontier
   candidates may land on unknown cells; the rest need known free ones.
2. **Scores each candidate.** Simulated field-of-view ray casting over the
   SCovox map gives its EIG.
3. **Costs each candidate.** With the planning map, a bounded grid Dijkstra
   gives the path cost and doubles as a reachability test. Without it (the
   `shared_params.yaml` default) the cost is straight-line distance and
   obstacle avoidance is left to the navigator.
4. **Picks the best** by information per distance, SSMI-style (Asgharivaskasi
   & Atanasov, T-RO 2023):

   ```
   U(c) = info_gain(c) / (ε + path_cost(c)),   ε = 0.1
   ```

   Unreachable candidates get `U = −∞` and sort last.
5. **Navigates** under a distance-scaled time budget and a no-progress
   watchdog. It gives up a goal Nav2 aborts twice from the same spot, or once
   if Nav2 had already run its recoveries there, and blacklists failed goals
   for a while so it does not re-pick them. Each goal
   ends with one `Goal end` log line and the same fields in the per-step CSV.
6. **Terminates** when the ROI's unknown fraction stays below a threshold for
   N consecutive cycles, or at `max_steps`.

The full method, with every number, is in
[docs/planner_method.md](../docs/planner_method.md).

### Rendezvous reconnection (multi-robot, default on)

A robot that loses comms keeps exploring. When it runs out of goals it drives
back to its start point and waits there until the whole team is back in comms,
then re-plans on the merged map. With the team already present it still drives
home before finishing. Open tree targets are stood down for the return and
resume once the team reconnects. It is active only with `coordination_enabled` and
`rendezvous_expected_peers` > 0. `shared_params.yaml` ships `0`, which leaves
it inert. `multi_robot_exploration.launch.py` and the per-robot blocks of the
real-robot overlay set it; any other team launch must set it by hand. Keep
`done_action: "idle"`: with `"shutdown"` the first robot to finish exits, and
its teammate waits at home for a robot that is gone.
Method: [planner_method.md §10.2](../docs/planner_method.md#102-rendezvous-reconnection).
Field settings: [user_manual.md §3.3](../docs/user_manual.md#33-rendezvous-reconnection).

### Coordinated proximity stop (multi-robot, default on)

MinPos deconflicts goals, not paths. While driving, a robot yields to a
higher-priority teammate (the lexicographically smaller `robot_name`) that is
moving within 5 m. It cancels its Nav2 goal, publishes a brake goal at its own
pose, holds, and resumes the same goal once the peer is beyond 6 m or has
parked. On hardware, set `proximity_peer_pose_topics` to the peers' localiser
poses: the 1 Hz intent heartbeat alone lags by metres at field speeds. The node
warns at startup when none are set, and warns that the guard will never hold if
`coordination_enabled` is also false. This is
coordination, not a certified safety stop. The crewed 1.5 m panic stop stays
the backstop.

The latched topic `proximity_hold_state`, under the node's namespace (e.g.
`/bunker/proximity_hold_state`), shows `hold peer=<id> dist=<m> n=<count>` or
`clear reason=<clear|parked|stale|no-peer|max-hold> held=<s>`.
Method: [planner_method.md §10.3](../docs/planner_method.md#103-coordinated-proximity-stop).
Field settings: [user_manual.md §3.2](../docs/user_manual.md#32-coordinated-proximity-stop).

## Perceptive exploitation

When a `TreeTarget` arrives, the planner switches from EXPLORE to EXPLOIT. It
places `n_vantages` (3) viewpoints on a circle around the trunk, keeps those
with a clear line of sight, and drives to each and dwells so the rosbag
captures overlapping views. A target completes after `min_vantages_required`
clear-sightline dwells, counted across the team, or closes as partial when
`exploit_target_timeout_sec` runs out. When the target queue drains, the
planner returns to exploration. Teammates split a ring's angles through the
intent table. The planner only positions and dwells; sensor fusion is offline.

Targets come from the time-based `target_scheduler_node` (a preselected list)
or the live `tree_detector_node`. `exploitation_experiment.launch.py` picks
one with `use_detector:=true|false`. Method:
[planner_method.md §11](../docs/planner_method.md#11-perceptive-exploitation).

## Architecture

One planner node per robot. Each planner consumes its **own robot's** fused
map and talks to peers only through the shared intent topic:

```
                         (per robot)
  dscovox merger ──ScovoxMap──▶ ┌───────────────────┐ ──PoseStamped──▶ Nav2
  target producer ──TreeTarget▶ │ explo_planner_node │ ──MarkerArray──▶ RViz
  tf: map → base_link ────────▶ │  (10 Hz tick)      │ ──CSV──▶ output_csv
                                └─────────┬─────────┘
                                          │ RobotIntent (pub + sub)
                             /exploration/intents  ◀── every peer planner
```

By default the map arrives on `/<robot>/dscovox_node/scovox`, goals go out on
`/<robot>/goal_pose` and targets arrive on `/exploration/targets`. Every topic,
action, tf and parameter is in
[docs/ros_api.md §4.1](../docs/ros_api.md#41-explo_planner_node).

The node logic is split into small, unit-tested modules:

| Module | Responsibility |
| --- | --- |
| `explo_planner_node` | The 10 Hz state machine and all ROS wiring |
| `candidate_generator` | Viewpoint candidates: frontier-cluster centroids and the polar grid |
| `candidate_snap` | Moves a frontier candidate off a bad planning-map cell onto the nearest good one |
| `fov_evaluator` | Simulated FOV ray casting for viewpoint evaluation |
| `scoring` | Per-voxel scores (EIG, entropy, frontier, random) |
| `cost_grid` | Bounded 8-connected grid Dijkstra path cost over the planning map |
| `plan_map_query` | Pure 2D occupancy-grid cell queries |
| `traversability_accumulator` | Stitches a robot-centred rolling grid window into one ROI-sized planning map |
| `roi` | The ROI rectangle, optionally rotated and offset from the map frame |
| `map_cache` | Read-only Bonxai grid rebuilt from ROS map messages |
| `failed_goal_blacklist` | TTL + radius blacklist of recently failed goals |
| `coordination` | Multi-robot intent table + MinPos deconfliction |
| `proximity_guard` | Coordinated proximity-stop arbiter (yield/hold/resume decisions) |
| `target_queue` | Tree-target queue (ingest, dedup, lifecycle) for exploitation |
| `vantage_planner` | Vantage-point generation + line-of-sight occlusion test |
| `tree_detector` | Trunk segmentation over the fused map, and the gate that decides when a tree is nominated |
| `metrics_logger` | Per-step CSV metric logging |
| `planner_util` | Small shared pure helpers |

Two further executables publish tree targets: `tree_detector_node` (wraps the
`tree_detector` module and gives each tree a stable id derived from its
position) and `target_scheduler_node`.

## Run

`shared_params.yaml` ships `use_sim_time: false`. Sim and bag runs must pass
`use_sim_time:=true`. With `true` and no `/clock` publisher the planner
never ticks; it warns once after 10 s.

| To | Read |
| --- | --- |
| Launch in sim or on a bag (single robot, exploitation, two robots on one host) | [docs/ros_api.md §3](../docs/ros_api.md#3-quick-start), launch files in [§6](../docs/ros_api.md#6-launch-files) |
| Build and run inside the `scovox` container | [docs/user_manual.md §4](../docs/user_manual.md#4-before-field-day) |
| Run on one robot, and pick the config for each run | [docs/field_setup.md](../docs/field_setup.md#run) |
| Run the two-robot field trial | [docs/user_manual.md §6](../docs/user_manual.md#6-bring-up-order) |
| Dry-run on a bag | [doc/dscovox_exploration_run.md](doc/dscovox_exploration_run.md), [doc/dscovox_exploitation_run.md](doc/dscovox_exploitation_run.md) |

### Real-robot runs (Bunker Mini + CURT Mini)

Layer [`config/exploration_real_robot.yaml`](config/exploration_real_robot.yaml)
over the shared file with `params_file:=`. It carries each platform's
`base_frame` and timeouts measured from the robots' own bags; how each value
was derived is in [docs/real_robot_tuning.md](../docs/real_robot_tuning.md).
Before the first run at a new site, set the ROI box and
`proximity_peer_pose_topics`
([real_robot_tuning.md §4](../docs/real_robot_tuning.md#4-required-to-update-before-real-runs)),
and fix bunker's doubled `odom → base_link` TF
([field_setup.md, Requirements](../docs/field_setup.md#requirements)). All
three fail quietly if skipped.

## Configuration

Every parameter lives in [`config/shared_params.yaml`](config/shared_params.yaml),
loaded by every launch file and commented with the reasoning behind each
default. [docs/ros_api.md](../docs/ros_api.md#parameters) indexes them. The
per-robot settings an operator must set, such as `robot_name`, the goal
tolerances against Nav2's and the z-band, are in
[user_manual.md §5](../docs/user_manual.md#5-per-robot-configuration). With
`terrain_relative_z: true` the node warns at startup if `roi_min_z`/`roi_max_z`
are too tight for the ground search (the invariant is in the yaml). The
preselected target schedule for `target_scheduler_node` is
[`config/targets.yaml`](config/targets.yaml).

## Tests

```bash
colcon test --packages-select explo_planner
colcon test-result --verbose
```

Every module in the table except `explo_planner_node` has a GTest suite,
`test/test_<module>.cpp`.

## License

BSD-3-Clause.
