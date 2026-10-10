# explo_planner — field setup

How to set up and run the planner on one robot's hardware: requirements,
build, run commands, the ROI and the planning map. The two-robot field-day
procedure is the [user manual](user_manual.md). How the planner works is in
the [package README](../explo_planner/README.md).

Each robot runs this chain: localizer (TF `map` → `base_link`) →
`scovox_node` (`mode: "rolling"`) → `dscovox_node` (merger) →
`explo_planner` → nav2. The planner reads the merger's fused map,
`/<robot>/dscovox_node/scovox`, never the mapper's own `~/scovox`. The
planner subscribes transient-local and the mapper publishes volatile, so
that subscription never connects. The mapping side is in
`scovox/docs/field_setup.md`.

## Requirements

- **ROS 2 Jazzy**, the same as the robot's nav2. On the robots, build and
  run inside the Jazzy `scovox` container ([user manual](user_manual.md)
  §4). Workspace `/home/jetsondevkit/hmr_explo/ws`, with `explo_planner`
  (depends on `scovox_core`, `scovox_msgs`, `nav2_msgs`) and
  `explo_planner_msgs` (`RobotIntent`, `TreeTarget`; depends only on
  `std_msgs` and `geometry_msgs`).
- This robot's **scovox mapper (`mode: "rolling"`) and dscovox merger**,
  publishing the fused map on `/<robot>/dscovox_node/scovox`.
- **TF** `map_frame` → `base_frame`. It is the planner's only pose source.
  Read the `TF BLOCKER` banner at the top of
  `config/exploration_real_robot.yaml` before any hardware run. bunker has
  been recorded with two publishers on `odom` → `base_link` that disagree by
  ~6.4 m, and tf2 cannot choose between them, so every pose the planner reads
  jumps between the two. Check there is exactly one broadcaster per edge:
  `ros2 run tf2_ros tf2_monitor odom base_link`.
- **nav2** (Jazzy or later) in the robot's namespace, reading the namespaced
  traversability topics. Start it with
  `ros2 launch explo_planner nav2_namespaced.launch.py namespace:=<robot>
  params_file:=<the robot's nav2 params>`. The params file is used
  unchanged; the launch file's header lists what it changes.
  [nav2_config.md](nav2_config.md) lists what the planner needs from nav2.

## Setup (once)

```bash
cd /home/jetsondevkit/hmr_explo/ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to explo_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run

**One robot, by hand** (`config/shared_params.yaml` is the documented
parameter set):

```bash
source /home/jetsondevkit/hmr_explo/ws/install/setup.bash
CFG=$(ros2 pkg prefix explo_planner)/share/explo_planner/config

ros2 run explo_planner explo_planner_node --ros-args \
  --params-file $CFG/shared_params.yaml \
  -p robot_name:=robot1 \
  -p base_frame:=base_link \
  -p map_frame:=map \
  -p dscovox_topic:=/robot1/dscovox_node/scovox \
  -p map_resolution:=0.20
```

Set `map_resolution` to the mapper's `resolution`. It only seeds the grid;
after the first message the planner uses the resolution carried on each fused
map.

Until the fused map, the pose and (if enabled) the planning map have all
arrived, the planner waits in WAIT_FOR_MAP and logs what is missing every
5 s. `map=0` there means no voxels inside the ROI yet, not necessarily no
message: an ROI that misses the robot looks the same as a mapper that never
published. The `Loaded fused map ... voxels in ROI (N in msg)` line tells
them apart.

For bag replay pass `use_sim_time:=true` (every launch file declares it).
`shared_params.yaml` ships `false` on purpose: with `true` and no `/clock`
publisher the 10 Hz tick timer never fires and the planner does nothing; it
logs `use_sim_time=true but no /clock received` after 10 s.

**Two robots**, one planner per namespace, with the hardware overlay that
carries each platform's frames and timeouts:

```bash
source /home/jetsondevkit/hmr_explo/ws/install/setup.bash
CFG=$(ros2 pkg prefix explo_planner)/share/explo_planner/config

ros2 launch explo_planner multi_robot_exploration.launch.py \
  robots:=bunker,curt params_file:=$CFG/exploration_real_robot.yaml
```

Both robots share the `map` frame. Each needs its own `odom` frame (bunker
`odom`, curt `odom_curt`). See the second TF constraint in the overlay's
header.

**Which config for which run.** There is no separate single-robot YAML. The
single-robot launches (`exploration_experiment.launch.py`, or
`exploitation_experiment.launch.py` with targets) start an un-namespaced
`/explo_planner` node:

| Run | Launch | Params |
| --- | --- | --- |
| One robot, sim/defaults | `exploration_experiment.launch.py robot:=<name>` | `shared_params.yaml` |
| One robot, hardware | same, plus `base_frame:=<frame> params_file:=$CFG/exploration_real_robot.yaml` | shared + the overlay's `/**` block (its per-robot blocks do not match an un-namespaced node) |
| Two robots, hardware | `multi_robot_exploration.launch.py robots:=bunker,curt params_file:=$CFG/exploration_real_robot.yaml` | shared + `/**` + the `/bunker/…` and `/curt/…` blocks |
| Bag replay (map-test-2) | `ws/src/run_explo_experiment.sh` (hmr_explo) | shared + `exploration_fused_bag.yaml` |
| Bag replay (CURT Mini) | `ws/src/run_explo_curtmini.sh` (hmr_explo) | see the script header |

On the single-robot launch the frames, `output_csv` and `max_steps` come
from launch arguments: its per-node dict is applied after the YAML files and
wins.

## Area of interest from the ground-truth map

The planner works only inside its ROI rectangle. Candidates, frontier
targets, the fused-map clip, the FOV rays and the coverage-done measure all
stop at its edge. The rectangle may be rotated to follow the site rather than
the map axes.

**Choosing the box.** Plot the hmr_localisation ground-truth map
(`gt_map_us050.pcd`, the map NDT localises against, so its frame is the
planner's `map`) top-down, rotated until its strongest straight edges (the
road) run left to right:

```bash
# hmr_localisation package, inside its container (package mounted at /ws)
python3 scripts/analysis/gt_map_aligned_topdown.py
#   -> docs/gt_map_us050_topdown_aligned.png
#   --angle-deg A   fix the rotation (default: auto-fit near 24 deg -> 25.75)
#   --x-zero / --y-zero   where x' = 0 / y' = 0 sit, in the rotated frame
```

That gives a "road frame" x′/y′ in metres: x′ along the road at 25.75° to
map x, y′ = 0 on the far road edge, x′ = 0 50 m west of the map origin. Read
the box off that plot. The shipped ROI is x′ ∈ [0, 100], y′ ∈ [0, 25], the
tree block between the road and the building line
(`hmr_localisation/docs/explo_roi_box.png` shows it in both frames).

**Setting the parameters** (shipped in `shared_params.yaml`). The four
`roi_min/max_x/y` are the box in the rotated frame. Three more keys place
that frame in `map`:

```yaml
roi_yaw_deg: 25.75      # rotation of the ROI frame's x axis from map x
roi_origin_x: -40.30    # map-frame point where the ROI frame's (0, 0) is
roi_origin_y: -31.54
roi_min_x: 0.0          # box, in the ROI frame (m)
roi_max_x: 100.0
roi_min_y: 0.0
roi_max_y: 25.0
```

For a plot origin (x′₀, y′₀), the script's `--x-zero`/`--y-zero` (here
(−50, −10.9)), the map-frame origin is
`(x′₀ cos θ − y′₀ sin θ, x′₀ sin θ + y′₀ cos θ)` with θ = `roi_yaw_deg`. To
move or resize the box, change only the four bounds. To change the frame
itself, recompute the origin. `roi_yaw_deg: 0` with origin (0, 0) is a plain
map-frame box.

**Check it.** The node prints the box and its map-frame extent at startup:

```
ROI: [0.00, 100.00] x [0.00, 25.00] m in a frame yawed 25.75 deg about map (-40.30, -31.54); map-frame extent x [-51.16, 49.77] y [-31.54, 34.42]
```

The robot must start inside the box: the map precondition clears only once
the fused map has voxels inside it (see `map=0` above). The map origin, the
usual deployment point, is inside the shipped box at (x′ 50, y′ 10.9).

## The planning map: path cost and goal placement

Each candidate is ranked by `U = info_gain / (0.1 + path_cost)`. The 2D
planning map sets what `path_cost` measures and which goals are allowed:

- **Off** (`use_planning_map: false`, the `shared_params.yaml` default).
  `path_cost` is the 2D straight line to the candidate. Walls, trees, bushes
  and height are ignored, and nav2 alone decides whether a goal is
  reachable.
- **On.** The planner runs a Dijkstra search over the 2D grid from the robot
  (`CostGrid`, 8-connected). `path_cost` becomes the shortest path around
  obstacles, candidates the search cannot reach are dropped, and
  non-frontier candidates on unknown cells are rejected. The grid is rebuilt
  only when a new map arrives, and the search reruns only on a new map or
  after 0.25 m of motion.

When on, the planning map is a hard startup precondition: the planner waits
in WAIT_FOR_MAP until the first grid arrives (with the real-robot overlay,
until `traversability_map.launch.py` is running). The default
`planning_map_topic`, `/<robot_name>/dscovox_node/planning_map`, has no
publisher.

`config/exploration_real_robot.yaml` turns it on, fed from
traversability_mapping (next section). nav2's ground-truth costmap is the
alternative source (the section after).

**What to watch**, with either source. The `No planning_map available`
warning should never appear. In the metrics CSV (`output_csv`),
`selected_path_cost` and `mean_path_cost` should exceed the straight-line
distance wherever the route bends around something. They are costs, not
metres: roughness, unseen cells and the clearance zone scale them up. The
route's length in metres is `path=` in the selection log line and
`goal_path_length_m` in the CSV, and the nav budget is built from it.

## Traversability map (keeps goals out of bushes)

The fused scovox map says what is occupied, not what the robot can drive on.
Frontier centroids sit at the edge of unexplored space, which in vegetation
is usually inside or against a bush. traversability_mapping
(github.com/errorcodecritical/traversability_mapping, cloned into
`ws/src/traversability_mapping`) scores the ground from the LiDAR, and the
real-robot overlay plans on it:

```
/hesai/points ─> traversability_filter ─> traversability_map ─/occupancy_map_local─> explo_planner
                                          (20 x 20 m window at 0.1 m       │  planning_map_mode: accumulate
                                           around the robot, ~5 Hz)        └─> ~/traversability_map (ROI grid, latched)
```

**Run it** on each robot, next to its mapper, merger and planner, in the
robot's namespace (cloud topics and frames as recorded on 2026-07-31):

```bash
# bunker: Hesai QT128; the shipped config's frames are bunker's
ros2 launch explo_planner traversability_map.launch.py namespace:=bunker \
  input_cloud_topic:=/hesai/points
# curt: Ouster, 128 x 1024
ros2 launch explo_planner traversability_map.launch.py namespace:=curt \
  base_frame:=base_link_curt lidar_frame:=os_lidar \
  input_cloud_topic:=/ouster/points
```

Each map lands on `/<robot>/occupancy_map_local`, which the overlay's
per-robot blocks read. The nodes' other topics move under the namespace too,
including `/<robot>/pointcloud_2_laserscan`, which nav2's collision monitor
reads. Frames do not, as both robots share one TF tree. Without `namespace`
the topics stay un-namespaced, for a single-robot `/explo_planner`.

The launch starts only `traversability_filter` and `traversability_map`, with
their shipped config. Do not use the package's own
`traversability_mapping.launch.py`: it also starts `traversability_prm` and
`traversability_path`, a planner of their own that runs on every map message,
goal or not, and grows without bound over a run (see CPU and memory below).
Launch arguments: `namespace`, `input_cloud_topic`, `base_frame`,
`lidar_frame`, `use_sim_time`, `params_file`, `sensor_range_limit`,
`prediction_enable`. Float overrides keep their decimal point (`12.0`): the
nodes read them with `as_double()` and die on an integer.

**What the planner does with it** (`planning_map_mode: "accumulate"`):

1. **Accumulates.** The window covers only 10 m around the robot, and every
   cell outside the planning map reads as blocked. So the planner stitches
   the windows into one fixed grid over the ROI box plus
   `planning_map_accum_margin_m` (555 × 380 cells at 0.2 m for the shipped
   ROI, 0.2 MB). A window's cells replace what was there. Cells it has not
   seen (−1) keep their earlier value. Window cells finer than the grid are
   max-pooled, so a thin obstacle survives. The grid is copied into the
   planning map at most every `planning_map_accum_refresh_sec` and
   republished latched on `~/traversability_map` for RViz.
2. **Blocks.** Cells ≥ `planning_map_obstacle_threshold` (50) are obstacles.
   On traversability_mapping's 0–100 score (slope 0.5, step 0.3, roughness
   0.2; hard obstacles forced to 100) that means steep, stepped or rough.
3. **Keeps clear.** Passable cells within `planning_map_clearance_m` (0.6 m)
   of an obstacle are never a goal, and cost `planning_map_clearance_penalty`
   (3×) to drive through.
4. **Snaps.** A frontier candidate on an obstacle, in the clearance zone or
   unreachable moves to the nearest good cell within
   `candidate_snap_radius_m` (2 m) inside the ROI. Its yaw is re-aimed at its
   old spot and its z re-seated on the terrain. With no good cell it stays
   put and the filters drop it.
5. **Costs the path.** A passable cell with score v costs
   `step × (1 + planning_map_trav_weight × v / 100)`. With weight 1, rough
   passable ground (v < 50) costs up to ~1.5×. An unseen cell counts as
   `planning_map_unknown_value` (25, so 1.25×). The utility therefore prefers
   frontiers reached over open, already-seen ground. The nav time budget uses
   the same path's length in metres.
6. **Lets the robot out.** Blocked cells within `planning_map_robot_clear_m`
   (1 m) of the robot do not wall in its own path search; they stay obstacles
   for goals. A parked robot can sit in a ring of 99-cells 0.4–0.8 m out,
   most likely its own body: traversability_mapping keeps returns from 0.5 m
   out, inside a Bunker's footprint.
7. **Re-checks.** While driving, a goal whose cell turns into an obstacle in
   a newer map aborts the drive and replans (`goal (x, y) is now inside an
   obstacle — aborting navigation and re-planning`). Expect this now and
   then: cells are scored only once seen often enough and from close enough.

**Check it.**

- Startup: `planning_map accumulate: 555x380 cells at 0.20 m over x [-56.2,
  54.8] y [-36.5, 39.5] (grid 0.2 MB; …)` and `planning_map: mode=accumulate
  obstacle>=50 … robot_clear=1.00 m flood_cap=500.0 m`.
- While waiting: `accumulate: N windows integrated, M rejected (last: …), K
  known cells`. M stays 0. `outside the accumulator box` means the robot is
  far outside the ROI; `rotated origin` means a different publisher.
- Per step: `[… clear=… unreach=… blk=…, snapped=… nospot=…]`. On the bag
  ~12 % of candidates snapped and ~1 % found no spot.
- `planning_map: no window for N s — planning on the stale accumulated
  map.` (after 5 s): traversability_map stopped, or its TF `map → base_link`
  is missing.
- `CostGrid: flood reached only N cells … Skipping reachability filter this
  tick.` should be rare. Repeated at one spot, the robot is walled in beyond
  `planning_map_robot_clear_m`.
- RViz: `/<robot>/explo_planner/traversability_map` over the fused map.

**Bag test** (2026-07-31 bunker bag replay, x86 desktop). With the map, 0
of 41 goals were picked on an obstacle cell, against 8 of 42 without it. The
map added 25–75 ms to the median PLAN and under one point of CPU; the
accumulator takes 0.2 MB and the CostGrid ~3 MB.

**CPU and memory on traversability_mapping's side.** Desktop x86, one core =
100 %:

| Input | traversability_filter CPU, mean / p95 | traversability_map CPU, mean / p95 |
| --- | --- | --- |
| `/hesai/points`, 10 Hz | 65–82 % / 98–103 % | 59–74 % / 97–100 % |
| throttled to 5 Hz | 29 % / 46 % | 29 % / 44 % |

At 10 Hz both nodes need a full core, so on the Jetson they will fall
behind and the windows will lag the robot. Ways to cut the load, none needing
a code change:

- **Feed 5 Hz.** It more than halves both nodes' CPU, with as good a map.
  Throttle at the driver if you can. Otherwise run `ros2 run topic_tools
  throttle messages /hesai/points 5.0 /hesai/points_5hz` (~16 % of a core)
  and pass `input_cloud_topic:=/hesai/points_5hz`.
- `sensor_range_limit:=12.0`: less work per scan (not measured).
- `prediction_enable:=false`: filter 43 % / 51 %, map 23 % / 30 %, but twice
  as many cells read 100 and three times as many drives abort on goal cells
  turning into obstacles. A last resort.

Memory:

- traversability_map's global map takes ~12.6 KB per m² seen and never frees
  it (~1.8 GB per hour at 1 m/s through new ground). If the node falls
  behind, its queue grows too (~1 MB/s at 10 Hz): watch its RSS while the
  robot is parked.
- `global_map_length: 500.0`, in a copy of its yaml passed as `params_file`,
  cuts its 32 MB up-front reservation to 2 MB while the robot stays within
  ~230 m of the map origin.
- Never subscribe to `/<robot>/elevation_pointcloud` (RViz included): up to
  ~25 MB per message at 5 Hz. `visualization_radius: 15.0` shrinks it.

**Caveats.**

- **Blocking threshold.** On the bunker bag, 5 % of the cells the robot
  drove over read ≥ 50 (2 % ≥ 70), against 23 % of all mapped ground. 50 is
  the conservative choice. If the robot refuses ground it can drive, raise
  `planning_map_obstacle_threshold` to 70.
- **CURT's map is noisier.** On the curt bag (416 s, the command above with
  `use_sim_time:=true`), 12.5 % of the cells CURT drove over read ≥ 50
  (3.2 % ≥ 70), and 37 % of mapped ground. At 50, CURT's planner treats about
  one in eight drivable cells as blocked. If it refuses open ground, set
  `planning_map_obstacle_threshold: 70` in the `/curt/explo_planner` block
  only. The shipped config's slope check assumes ring 0 is the bottom beam
  (bunker's Hesai); on the Ouster it is the top one (+21°). Flipping the
  rings changed little on the bag (9.8 % against 12.5 %), so they are left
  as they are. nav2 has not yet driven CURT on this map.
- **Unscored cells read 0 (free).** traversability_mapping writes 0 for
  ground it has seen but not scored (fewer than 10 observations, beyond
  15 m, or no slope). Goals can land on such a cell; step 7 catches it once
  scored.
- **TF.** traversability_map projects each cloud with the latest `map →
  base_link`. With bunker's doubled `odom → base_link` (Requirements),
  consecutive clouds land ~0.12 m apart in height, which the scoring reads
  as steps.
- **nav2 must agree.** Notes on the robots' nav2, for whoever runs it:
  - A static layer on `occupancy_map_local` with the default
    `trinary_costmap: true` treats only 100 as lethal, so nav2 plans through
    the 50–99 ground the planner avoids. traversability_mapping's
    `config/nav2_costmap.yaml` does this.
  - `lethal_cost_threshold: 50` matches the planner, but turns the ring
    around a parked robot (step 6) into a wall around it.
  - `trinary_costmap: false` keeps only 100 lethal and costs a cell scored v
    at v × 2.54 (50 → 127, 99 → 251), so nav2 prefers the planner's ground
    and walls nothing in. Try it on a bag first.
  - bunker's params (2026-10-09) set no `lethal_cost_threshold`, so only 100
    is an obstacle. `robot_radius: 0.2` is a 0.4 m disc for a
    ~1.0 × 0.78 m Bunker, so only the 0.8 m inflation keeps its sides and
    corners off obstacles. Its `::` plugin names need nav2 on Jazzy or later
    (Humble's `planner_server` fails with `Failed to create global planner`).
- **Goals outside nav2's global costmap.** nav2 (Jazzy, default tree)
  rejects them at once with `GOAL_OUTSIDE_MAP` (error 204), with no
  recovery. The planner re-sends the goal once, then fails it as
  `[nav2-aborted]` (`nav2_abort_limit` 2) about 5 s in, blacklists it and
  picks another. The log shows `nav2 abort reason: error_code 204
  GOAL_OUTSIDE_MAP`. Each such goal still costs a PLAN, so nav2's global
  costmap should reach as far as the goals.
  - The 20 m rolling global costmap in traversability_mapping's
    `config/nav2_costmap.yaml` cannot hold a goal more than 10 m away: 3–4 of
    ~45 goals per bag replay.
  - bunker's params (2026-10-09) set a 200 × 200 m rolling window at 0.1 m.
    It holds the whole shipped ROI (101 × 66 m) from almost anywhere in it,
    and keeps the cells of earlier map windows until it rolls past them.
  - Check a robot's with
    `ros2 param dump /<robot>/global_costmap/global_costmap`.
  - The planner's latched `~/traversability_map` (ROI + 5 m, fixed origin)
    can replace it: a non-rolling global costmap (`rolling_window: false`)
    with a static layer on that topic (`map_subscribe_transient_local:
    true`) and `lethal_cost_threshold: 50`.

## nav2's ground-truth costmap (alternative planning map)

nav2's global costmap built from the ground-truth map is a grid the robot
drives on, published as the latched `nav_msgs/OccupancyGrid` the planner
expects. It needs no extra node, only parameters. In place of the
traversability block in `config/exploration_real_robot.yaml`:

```yaml
/**:
  ros__parameters:
    use_planning_map: true
    planning_map_mode: "direct"
    cost_grid_radius_cap_m: 150.0
    planning_map_topic: "/global_costmap/costmap"          # nav2 without a namespace

/bunker/explo_planner:
  ros__parameters:
    planning_map_topic: "/bunker/global_costmap/costmap"   # nav2 in /bunker
```

`cost_grid_radius_cap_m` is required. Its default (0 = candidate radius +
2 m = 10 m) stops the search at 10 m, so every candidate beyond reads
unreachable and long-range frontier targets are never picked. Size it to
cover the ROI. For a *rolling* costmap (nav2's usual global costmap without a
prior map) use `"accumulate"`, as for traversability_mapping, or every goal
beyond the window is rejected.

**Check the costmap before the first run.**

1. **It exists and matches.** `ros2 topic info -v <topic>`: type
   `nav_msgs/msg/OccupancyGrid`, durability `TRANSIENT_LOCAL`,
   `header.frame_id` `map`.
2. **It covers the ROI.** The shipped box spans map x −51.2 … 49.8,
   y −31.5 … 34.4. Cells outside the costmap read as blocked.
3. **Resolution.** Each message is copied and the CostGrid rebuilt from it.
   At 0.05 m over the full 230 × 210 m map that is ~19 M cells (tens of ms
   and ~150 MB each). 0.1–0.2 m, or a costmap sized to the ROI, is cheap.
   This matters most on the Jetson.
4. **Blocking threshold.** nav2 publishes lethal 100, inscribed 99, unknown
   −1 and inflation 1–98. The planner blocks ≥ 50, about nav2 cost 128, so
   paths keep a little more clearance than nav2 needs.
5. **Unknown cells.** The search passes through unknown (−1) cells; only the
   candidate filter rejects them (frontier candidates are exempt). A static
   layer from the full ground-truth cloud leaves little unknown inside the
   ROI. Confirm it in RViz.
6. **Updates.** nav2 sends the full grid on `…/costmap` once, then only
   diffs on `…/costmap_updates`, which the planner does not read. That is
   fine for a static ground-truth layer. If live obstacle layers matter to
   path cost, set `always_send_full_costmap: true` on the global costmap.

Painting out-of-ROI cells lethal in the costmap (as the forest-inspection
script does) also makes them walls for the path search, so path cost and the
ROI agree.

## Notes

- **Stopping needs the action client.** If the planner only stops
  publishing, nav2 still drives its last accepted goal to the end. So the
  planner cancels the in-flight `NavigateToPose` goal through a cancel-only
  client on `proximity_nav_cancel_action`, and publishes a brake goal at its
  own pose. That client exists only while `proximity_stop_enabled` is true
  (the default); leave it on with nav2.
- **Robot geometry** varies by platform: `candidate_robot_z` (sensor
  height), `candidate_occ_thresh`, `candidate_z_clearance` and the proximity
  hold/resume distances.
- **Elsewhere.** The navigator contract: [ros_api.md](ros_api.md)
  (Integration notes). Per-robot settings the operator must set, including
  goal tolerances against nav2's: [user_manual.md
  §5](user_manual.md#5-per-robot-configuration) and
  [real_robot_tuning.md](real_robot_tuning.md) §4. Coordination, rendezvous
  and termination: [planner_method.md](planner_method.md) §9–10. Every
  parameter: [ros_api.md](ros_api.md).
