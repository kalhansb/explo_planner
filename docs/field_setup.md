# explo_planner — frontier exploration on the fused map (field setup)

`explo_planner_node` is a next-best-view exploration planner. It ingests the
dscovox merger's fused `ScovoxMap`, reads its own pose from TF
(`map_frame` → `base_frame`), generates candidate viewpoints around the robot,
scores each by expected information gain over an FOV raycast, and drives the
robot by publishing a `geometry_msgs/PoseStamped` goal. A 10 Hz state machine
handles navigation budgets, a no-progress watchdog, failed-goal blacklisting,
multi-robot goal claiming, and coverage-based termination.

```
localizer ──TF map→base_link──────────────────────────────────┐
scovox_node ──deltas──> dscovox_node ──/robot1/dscovox_node/scovox──> explo_planner ──/robot1/goal_pose──> Nav2 (bt_navigator)
             (mode: rolling)           (fused ScovoxMap, latched)          │ ▲
             peer planners <──/exploration/intents (claims, 1 Hz)──────────┘ └─ NavigateToPose action: cancel only
```

The full per-robot chain is: localizer (TF pose) → `scovox_node`
(`mode: "rolling"`) → `dscovox_node` (merger) → `explo_planner` → Nav2. The
planner's map input is the **merger's** topic, never the mapper's own
`~/scovox`: the planner subscribes transient-local, the mapper publishes
volatile, and incompatible durability means the subscription simply never
matches. See `scovox/docs/field_setup.md` for the mapping side.

## Requirements

- **ROS 2 Humble**, colcon workspace at `/home/jetsondevkit/hmr_explo/ws`.
  Packages: `explo_planner` — which depends on `scovox_core`, `scovox_msgs`
  and `nav2_msgs` — and `explo_planner_msgs`, which owns the planner's own
  `RobotIntent`/`TreeTarget` types and depends only on
  `std_msgs`/`geometry_msgs`.
- A running **scovox mapper (`mode: "rolling"`) + dscovox merger** for this
  robot, publishing the fused map on `/<robot>/dscovox_node/scovox`.
- **TF**: `map_frame` → `base_frame` must resolve (the localizer's tree). The
  planner has no odom subscription; TF is its only pose source. **Read the
  `TF BLOCKER` banner at the top of `config/exploration_real_robot.yaml`
  before any hardware run**: bunker has been recorded with two publishers on
  `odom`→`base_link` disagreeing by ~6.4 m, which tf2 cannot arbitrate — every
  pose the planner reads then oscillates between them. Exactly one broadcaster
  per edge, checked with `ros2 run tf2_ros tf2_monitor odom base_link`, is a
  precondition for anything downstream meaning anything.
- **A navigator** subscribed to the goal topic. Nav2 is the supported path:
  `bt_navigator` subscribes `goal_pose` and wraps each received pose in a
  `NavigateToPose` action goal it sends itself.

## Setup (once)

```bash
cd /home/jetsondevkit/hmr_explo/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to explo_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run

**One robot** (`config/shared_params.yaml` is the documented parameter set):

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

Set `map_resolution` to the mapper's `resolution`: it seeds the grid before
the first message arrives, after which the planner adopts the resolution
carried on each fused map, so a mismatch self-corrects rather than corrupting
the grid. Until the fused map, pose (and planning_map, if enabled) have all
arrived, the planner sits in WAIT_FOR_MAP and logs which precondition is
missing every 5 s.

`map=0` on that line means *no in-ROI voxels yet*, not necessarily no message:
the map precondition clears only once a fused map leaves more than zero voxels
after the ROI clip, so an ROI box that misses the robot looks identical to a
mapper that never published. The `Loaded fused map ... voxels in ROI (N in
msg)` line separates the two.

For bag replay pass `use_sim_time:=true` (all three launch files declare it as
an argument). `shared_params.yaml` ships `false` on purpose: the 10 Hz tick
timer runs off the node clock, so `use_sim_time: true` with no `/clock`
publisher means the timer never fires and the planner does nothing at all,
without a warning.

**Two robots** — one planner per namespace, with the hardware overlay that
carries each platform's frames and timeouts:

```bash
source /home/jetsondevkit/hmr_explo/ws/install/setup.bash
CFG=$(ros2 pkg prefix explo_planner)/share/explo_planner/config

ros2 launch explo_planner multi_robot_exploration.launch.py \
  robots:=bunker,curt params_file:=$CFG/exploration_real_robot.yaml
```

Give each robot its own `map`/`odom` frame names and make that overlay's
per-robot blocks match — see the TF banner in its header.

**Which config for which run.** There is no separate single-robot YAML; the
single-robot path is `exploration_experiment.launch.py` (or
`exploitation_experiment.launch.py` with targets), which launches an
un-namespaced `/explo_planner` node:

| Run | Launch | Params |
| --- | --- | --- |
| One robot, sim/defaults | `exploration_experiment.launch.py robot:=<name>` | `shared_params.yaml` |
| One robot, hardware | same, plus `base_frame:=<frame> params_file:=$CFG/exploration_real_robot.yaml` | shared + the overlay's `/**` block (its per-robot blocks do not match an un-namespaced node) |
| Two robots, hardware | `multi_robot_exploration.launch.py robots:=bunker,curt params_file:=$CFG/exploration_real_robot.yaml` | shared + `/**` + the `/bunker/…` and `/curt/…` blocks |
| Bag replay (map-test-2) | `run_explo_experiment.sh` (repo root) | shared + `exploration_fused_bag.yaml` |
| Bag replay (CURT Mini) | `run_explo_curtmini.sh` (repo root) | see the script header |

On the single-robot launch the frames, `output_csv` and `max_steps` come from
launch arguments — its per-node dict is applied after the YAML files and wins.

## Area of interest from the ground-truth map

The planner only works inside its ROI rectangle: candidates, frontier targets,
the fused-map ingest clip, the FOV ray clip and the coverage-done measure all
stop at its edge. The rectangle may be rotated, so it can follow the site
instead of the map axes.

**How the shipped box was chosen.** The hmr_localisation ground-truth map
(`gt_map_us050.pcd`, the map NDT localises against, so its frame *is* the
planner's `map`) is plotted top-down and rotated until its strongest straight
edges — the road — run left to right:

```bash
# hmr_localisation package, inside its container (package mounted at /ws)
python3 scripts/analysis/gt_map_aligned_topdown.py
#   -> docs/gt_map_us050_topdown_aligned.png
#   --angle-deg A   fix the rotation (default: auto-fit near 24 deg -> 25.75)
#   --x-zero / --y-zero   where x' = 0 / y' = 0 sit, in the rotated frame
```

That gives a "road frame" x′/y′ in metres: x′ along the road at 25.75° to map
x, y′ = 0 on the far road edge, x′ = 0 50 m west of the map origin. Reading
the box off that plot is the whole job — the shipped ROI is x′ ∈ [0, 100],
y′ ∈ [0, 25], the tree block between the road and the building line
(`hmr_localisation/docs/explo_roi_box.png` shows it in both frames).

**Turning a box on the plot into parameters.** The four `roi_min/max_x/y`
are the box in the rotated frame; three more keys place that frame in `map`:

```yaml
roi_yaw_deg: 25.75      # rotation of the ROI frame's x axis from map x
roi_origin_x: -40.30    # map-frame point where the ROI frame's (0, 0) is
roi_origin_y: -31.54
roi_min_x: 0.0          # box, in the ROI frame (m)
roi_max_x: 100.0
roi_min_y: 0.0
roi_max_y: 25.0
```

For a plot origin (x′₀, y′₀) — the aligned script's `--x-zero`/`--y-zero`,
here (−50, −10.9) — the map-frame origin is
`(x′₀ cos θ − y′₀ sin θ, x′₀ sin θ + y′₀ cos θ)`. To move or resize the box,
change only the four bounds; to change the frame itself, recompute the
origin. `roi_yaw_deg: 0` with origin (0, 0) is a plain map-frame box.

**Check it.** The node prints the box and its map-frame extent at startup:

```
ROI: [0.00, 100.00] x [0.00, 25.00] m in a frame yawed 25.75 deg about map (-40.30, -31.54); map-frame extent x [-51.16, 49.77] y [-31.54, 34.42]
```

The robot must start inside the box — the map precondition only clears once
the fused map has voxels inside it (see `map=0` above). The map origin, the
usual deployment point, is inside the shipped box at (x′ 50, y′ 10.9).

## The planning map: path cost and goal placement

Each candidate is ranked by `U = info_gain / (0.1 + path_cost)`. What
`path_cost` measures, and which goals are allowed, depends on the 2D planning
map:

- **Off (`use_planning_map: false`, the `shared_params.yaml` default).**
  `path_cost` is the 2D straight line from the robot to the candidate. Walls,
  trees, bushes and height are all ignored; a candidate behind a building
  costs the same as one in the open, and nav2 alone decides whether it is
  reachable.
- **On.** The planner runs a Dijkstra search over the 2D grid from the robot
  (`CostGrid`, 8-connected). `path_cost` becomes the shortest path *around*
  obstacles, candidates the search cannot reach are dropped, and non-frontier
  candidates standing on unknown cells are rejected. The grid's layers are
  rebuilt only when a new map arrives, and the search reruns only on a new
  map or after 0.25 m of motion.

`config/exploration_real_robot.yaml` turns it on, fed from
traversability_mapping (next section). nav2's ground-truth costmap is the
alternative source (the section after).

## Traversability map (keeps goals out of bushes)

The fused scovox map says what is occupied, not what the robot can drive on.
Frontier centroids sit at the edge of unexplored space, which in vegetation is
usually inside or against a bush — so without a ground model the planner
sends the robot into them. traversability_mapping
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
per-robot blocks read. The nodes' other topics move under the namespace too;
frames do not (one TF tree). Without `namespace` the topics stay
un-namespaced, for a single-robot `/explo_planner`.

This starts only `traversability_filter` and `traversability_map`, with their
shipped config. Do not use the package's own `traversability_mapping.launch.py`:
it also starts `traversability_prm` and `traversability_path`, a planner of
their own that runs on every map message whether or not it has a goal and
grows without bound over a run (see CPU and memory below). Launch
arguments: `namespace`, `input_cloud_topic`, `base_frame`, `lidar_frame`,
`use_sim_time`, `params_file`, `sensor_range_limit`, `prediction_enable`. Float overrides keep their decimal point (`12.0`): the
nodes read them with `as_double()` and die on an integer.

**What the planner does with it** (`planning_map_mode: "accumulate"`):

1. **Accumulates.** The window only covers 10 m around the robot, and every
   cell outside the planning map reads as blocked — fed the raw window, every
   candidate beyond 10 m would be rejected. The planner stitches the windows
   into one fixed grid over the ROI box plus `planning_map_accum_margin_m`
   (555 × 380 cells at 0.2 m for the shipped ROI, 0.2 MB). A window's cells
   replace what was there; cells it has not seen (−1) keep their earlier
   value; window cells finer than the grid are max-pooled, so a thin obstacle
   survives. The grid is snapshotted into the planning map at most every
   `planning_map_accum_refresh_sec` and republished latched on
   `~/traversability_map` for RViz.
2. **Blocks.** Cells ≥ `planning_map_obstacle_threshold` (50) are obstacles.
   On traversability_mapping's 0–100 score (slope 0.5, step 0.3, roughness
   0.2; hard obstacles forced to 100) that is "steep, stepped or rough".
3. **Keeps clear.** Passable cells within `planning_map_clearance_m` (0.6) of
   an obstacle are never a goal, and cost `planning_map_clearance_penalty`
   (3×) to drive through.
4. **Snaps.** A frontier candidate on an obstacle, in the clearance zone, or
   unreachable moves to the nearest good cell within
   `candidate_snap_radius_m` (2 m) inside the ROI, its yaw re-aimed at where
   it was and its z re-seated on the terrain. With no good cell it is left
   alone and the filter drops it. Polar candidates stay off.
5. **Costs the path.** A passable cell with score v costs
   `step × (1 + planning_map_trav_weight × v / 100)` (weight 1: rough ground
   counts up to double), and an unseen cell counts as
   `planning_map_unknown_value` (25: 1.25×), so the utility prefers
   frontiers reached over open, already-seen ground. The nav time budget
   uses the same path's length in metres (logged as `path=… m` with the
   selection).
6. **Lets the robot out.** Blocked cells within `planning_map_robot_clear_m`
   (1 m) of the robot do not wall in its own search (they stay obstacles for
   goals). On the bag a parked robot sat in a ring of 99-cells 0.4–0.8 m out
   — most likely its own body: traversability_mapping keeps returns from
   0.5 m out, inside a Bunker's footprint.
7. **Re-checks.** While driving, a goal whose cell turns into an obstacle in
   a newer map aborts the drive and replans (`goal (x, y) is now inside an
   obstacle — aborting navigation and re-planning`). Expected now and then:
   cells are only scored once seen often enough and from close enough.

**Check it.**

- Startup: `planning_map accumulate: 555x380 cells at 0.20 m over x [-56.2,
  54.8] y [-36.5, 39.5] (grid 0.2 MB; …)` and `planning_map: mode=accumulate
  obstacle>=50 … robot_clear=1.00 m flood_cap=500.0 m`.
- While waiting: `accumulate: N windows integrated, M rejected (last: …), K
  known cells`. M stays 0; `outside the accumulator box` means the robot is
  far outside the ROI, `rotated origin` a different publisher.
- Per step: `[… clear=… unreach=… blk=…, snapped=… nospot=…]`. On the bag
  ~12 % of candidates snapped and ~1 % found no spot.
- `planning_map: no window for N s — planning on the stale accumulated
  map.` (after 5 s): traversability_map stopped, or its TF `map → base_link`
  is missing.
- `CostGrid: flood reached only N cells … Skipping reachability filter this
  tick.` should be rare; repeated at one spot means the robot is walled in
  beyond `planning_map_robot_clear_m`.
- RViz: `/<robot>/explo_planner/traversability_map` over the fused map.

**Bag test** (2026-07-31 bunker bag, 427 s, 81 m driven; x86 desktop, 8
cores). Two planners on the same replay, one with the map (the shipped
real-robot settings) and one without; both open loop — the robot drives the
recorded path, so this measures where the goals land, not whether nav2 got
there. Goals are scored against the accumulated map when picked and at the
end:

| | with the map | without |
| --- | --- | --- |
| goals on an obstacle cell, when picked | 0 / 41 | 8 / 42 |
| goals within 0.4 m of an obstacle, when picked | 0 / 41 | 9 / 42 |
| goals on an obstacle cell, final map | 0 / 45 | 3 / 44 |
| obstacle share of the 1 m disc around a goal, final map (mean) | 1.1 % | 4.6 % |
| PLAN time, median | 690 ms | 615 ms |
| planner CPU, mean (one core = 100 %) | 20.9 % | 20.1 % |
| planner RSS at the end | 762 MB | 834 MB |

Over five replays the map added 25–75 ms to the median PLAN (the rest is
the information-gain raycast) and under one point of CPU. The planner's RSS
is the fused map (5 M voxels by the end), with or without the traversability
map, and swings with it; the accumulator adds 0.2 MB and the CostGrid ~3 MB.
Two settings came out of the replays: at 0.4 m clearance half the goals had
an obstacle 0.4–1 m away (a ~0.78 m wide Bunker's sides at the bush edge),
hence 0.6 m; and without `planning_map_robot_clear_m` the search was walled
in at 6–7 PLANs per run, nearly all where the robot stood parked.

**CPU and memory on traversability_mapping's side** — all controllable
without changing its code:

| input | node | CPU mean / p95 | RSS start → end |
| --- | --- | --- | --- |
| `/hesai/points`, 10 Hz | traversability_filter | 65–82 % / 98–103 % | 48 → 91 MB |
| | traversability_map | 59–74 % / 97–100 % | 69 → 111 MB |
| throttled to 5 Hz | traversability_filter | 29 % / 46 % | 48 → 80 MB |
| | traversability_map | 29 % / 44 % | 69 → 109 MB |

(desktop x86; one core = 100 %.) At 10 Hz both sit at a full core at p95 —
on the Jetson expect them to fall behind, and the windows to lag the robot.
Levers:

- **Feed 5 Hz.** Throttling the cloud to 5 Hz cut both nodes to under half,
  with the map as good as at 10 Hz (4.7 % of driven-over cells ≥ 50, goals
  0 / 46 on an obstacle). Best at the source (driver or decimator); otherwise
  `ros2 run topic_tools throttle messages /hesai/points 5.0
  /hesai/points_5hz` and `input_cloud_topic:=/hesai/points_5hz` — the relay
  itself took ~16 % of a core. The map then publishes a window every 2nd
  cloud, ~2.5 Hz, which is plenty for the planner (it snapshots at 1 Hz).
- `prediction_enable:=false` cut the filter to 43 % / 51 % and the map to
  23 % / 30 % (mean / p95) on the same replay, but the map got noisier
  without the filter's ground prediction: twice the cells read 100, 6.7 % of
  the driven-over cells read ≥ 50 (p95 of their scores 99, against 37–52),
  and 6 drives were aborted as their goal cell turned into an obstacle (2
  with prediction on). A last resort, not the first lever.
- `sensor_range_limit:=12.0` — less work per scan and slower map growth
  (not measured here).
- If traversability_map cannot keep up, its internal observation queue grows
  (~1 MB/s at 10 Hz) and the windows lag the robot. Watch for its RSS
  climbing while the robot is parked.
- Its global map allocates ~12.6 KB per m² the robot has seen and never frees
  it (~40 MB on this bag, ~1.8 GB per hour at 1 m/s through new ground);
  `global_map_length: 2000.0` also reserves 32 MB of index arrays up front.
  With `params_file` pointing at a copy of its yaml, `global_map_length:
  500.0` cuts that to 2 MB as long as the robot stays within ~230 m of the
  map origin.
- Never subscribe to `/elevation_pointcloud` (RViz included): it is rebuilt
  under the map mutex for every window, up to ~25 MB per message at 5 Hz
  (`visualization_radius: 15.0` shrinks it).
- `traversability_prm` / `traversability_path`: not needed and not launched
  (above). The PRM busy-loops and its edge set grows O(k²); subscribing to
  `/prm_cloud_graph` at a few thousand nodes is ~400 MB per message.

**Caveats.**

- **Blocking threshold.** On the bunker bag, 5 % of the cells the robot
  actually drove over read ≥ 50 (2 % ≥ 70, none = 100), while 23 % of the
  mapped ground reads ≥ 50. 50 is the conservative choice; if the robot
  refuses ground it can drive, raise `planning_map_obstacle_threshold` to 70.
- **CURT's map is noisier.** On the curt bag (416 s, the command above with
  `use_sim_time:=true`), 12.5 % of the cells CURT drove over read ≥ 50
  (3.2 % ≥ 70, p95 63) and 37 % of the mapped ground; on bunker, 5 % and
  23 %. At 50 CURT's planner will treat about one in eight cells it can
  drive as blocked: if it refuses open ground, set
  `planning_map_obstacle_threshold: 70` in the `/curt/explo_planner` block
  alone. The shipped config was written for bunker's Hesai, whose ring 0 is
  the bottom beam; the Ouster's ring 0 is the top one (+21°), so the
  filter's slope check reads the wrong beams on CURT. Flipping the rings in
  a relay changed little on the bag (9.8 % of driven cells ≥ 50 against
  12.5 %), so it is left as is: the map node's own slope, step and
  roughness scoring does the work. Nav2 has not driven CURT on this map yet.
- **Unscored cells read 0 (free).** traversability_mapping writes 0 for
  ground it has seen but not scored (fewer than 10 observations, beyond 15 m,
  no slope). Goals can land on such a cell; step 7 above catches it once
  scored.
- **nav2 must agree.** If nav2's costmaps take `/occupancy_map_local` through
  a StaticLayer as in the package's `config/nav2_costmap.yaml`, the default
  `trinary_costmap: true` makes only 100 lethal — 50–99 is free space to
  nav2, so it plans straight through the ground the planner avoids — and that
  file sets no `footprint`/`robot_radius` (nav2 default: a 0.1 m disc) with
  0.2 m inflation. Set, in the robot's own nav2 params (not in the package):
  `lethal_cost_threshold: 50` on each costmap, the robot's real `footprint`,
  and inflation to match. With 50 lethal, the ring a parked robot sees around
  itself (step 6) becomes lethal to nav2 too. `footprint_clearing_enabled:
  true` on the static layer clears only the footprint, and the ring reaches
  0.8 m, past a Bunker's ~0.39 m half-width.
- **Goals outside nav2's global costmap.** The package's 20 m rolling global
  costmap cannot hold a goal more than 10 m away: 3–4 of ~45 goals per bag
  replay. nav2 (Jazzy, default tree) rejects such a goal at once with
  `GOAL_OUTSIDE_MAP` (error 204) and tries no recovery. The planner watches
  nav2's action status: the keep-alive re-sends the goal once, nav2 rejects
  it again from the same spot, and `nav2_abort_limit` (2) fails it as
  `[nav2-aborted]` about 5 s in; it is blacklisted and the planner picks
  another. On Jazzy the log names the reason (`nav2 abort reason:
  error_code 204 GOAL_OUTSIDE_MAP`). Before this the robot stood still until
  the nav budget ran out (136–180 s for a goal over 10 m away), because the
  no-progress watchdog counted pose jitter on a parked Bunker (~1.5 m per
  30 s) as travel; it now measures net displacement and fires too. Many
  such goals still cost a PLAN each, so size nav2's global costmap to the
  goals' reach. The robot's own nav2 on 2026-07-31 rejected two hand-sent goals
  only 3.1 m and 4.8 m away the same way, so its global costmap is not the
  package's 20 m window; check it with `ros2 param dump
  /global_costmap/global_costmap`. The planner's latched
  `~/traversability_map` covers the ROI + 5 m at a fixed origin and can
  replace it: a non-rolling global costmap (`rolling_window: false`) with a
  static layer on that topic (`map_subscribe_transient_local: true`) and
  `lethal_cost_threshold: 50`.
- **TF.** traversability_map projects each cloud with the latest `map →
  base_link`. With the doubled `odom → base_link` on bunker (Requirements)
  consecutive clouds land ~0.12 m apart in height, which the scoring can
  only read as steps; the bag test dropped the second (planar) publisher.
- **Several robots.** traversability_mapping's topics are absolute
  parameters, so a ROS namespace alone does not move them.
  `traversability_map.launch.py namespace:=<robot>` prefixes every topic
  but the input cloud (Run it, above); the overlay's per-robot blocks
  already read `/<robot>/occupancy_map_local`.

## nav2's ground-truth costmap (alternative planning map)

nav2's global costmap built from the ground-truth map is a grid the robot
drives on, published as the latched `nav_msgs/OccupancyGrid` the planner
expects — no extra node, only parameters. In place of the traversability
block in `config/exploration_real_robot.yaml`:

```yaml
/**:
  ros__parameters:
    use_planning_map: true
    planning_map_mode: "direct"
    cost_grid_radius_cap_m: 150.0
    planning_map_topic: "/global_costmap/costmap"          # single robot only

/bunker/explo_planner:
  ros__parameters:
    planning_map_topic: "/bunker/global_costmap/costmap"   # two robots
```

`cost_grid_radius_cap_m` is not optional. Its default (0 = candidate radius
+ 2 m = 10 m) stops the search at 10 m, and every candidate beyond that reads
unreachable — long-range frontier targets would never be picked. Size it to
cover the ROI. A *rolling* costmap (nav2's usual global costmap on a robot
without a prior map) is a window around the robot: use `"accumulate"` for it,
as for traversability_mapping, or every goal beyond the window is rejected.

**Check the costmap before the first run.**

1. **It exists and matches.** `ros2 topic info -v <topic>`: type
   `nav_msgs/msg/OccupancyGrid`, durability `TRANSIENT_LOCAL`, and
   `header.frame_id` `map`. The planner waits in WAIT_FOR_MAP (logging the
   missing planning_map every 5 s) until the first message arrives.
2. **It covers the ROI.** The shipped box spans map x −51.2 … 49.8,
   y −31.5 … 34.4. Cells outside the costmap read as blocked.
3. **Resolution.** Every new message is copied and the CostGrid rebuilt from
   it. At 0.05 m over the full 230 × 210 m map that is ~19 M cells (tens of
   ms and ~150 MB each); 0.1–0.2 m, or a costmap sized to the ROI, is cheap.
   Matters most on the Jetson.
4. **Blocking threshold.** nav2 publishes lethal 100, inscribed 99, unknown
   −1 and inflation 1–98; the planner blocks ≥ 50
   (`planning_map_obstacle_threshold`) — about nav2 cost 128, the default
   circumscribed cost. Paths keep a little more clearance than nav2 needs.
5. **Unknown cells.** The Dijkstra passes through unknown (−1) cells; only the
   candidate filter rejects them (frontier candidates are exempt). A static
   layer from the full ground-truth cloud leaves little unknown inside the
   ROI — confirm by looking at the costmap in RViz.
6. **Updates.** nav2 sends the full grid on `…/costmap` once and then only
   diffs on `…/costmap_updates`, which the planner does not read. With a
   static ground-truth layer that is fine. If live obstacle layers matter to
   path cost, set `always_send_full_costmap: true` on the global costmap.

Painting out-of-ROI cells lethal in the costmap (as the forest-inspection
script does) also makes them walls for the path search, so path cost and the
ROI agree.

**What to watch.** The `No planning_map available` warning should never
appear. The metrics CSV (`output_csv`) records `selected_path_cost` and
`mean_path_cost`; with the grid on they should exceed the straight-line
distance wherever the route bends around something. They are costs, not
metres: roughness, unseen cells and the clearance zone scale them up. The
route's length in metres is `path=` in the selection log line and
`goal_path_length_m` in the CSV, and the nav budget is built from it.

## Notes

- **Navigation is pluggable; Nav2 is what you will use.** The only command
  path is one `PoseStamped` on `goal_topic` (default
  `/<robot_name>/goal_pose`, frame `map_frame`), re-published on change or
  every `goal_republish_sec`. Arrival is judged by the planner itself from TF
  against `goal_xy_tolerance`/`goal_yaw_tolerance`, so any stack that accepts
  a goal pose works. With nav2 the planner also watches the
  `NavigateToPose` action (`nav_status_action`, default
  `/<robot_name>/navigate_to_pose`): its status, its feedback (recoveries,
  distance to go) and, on Jazzy, the error code of an ABORT. That is logged
  and counted per goal, and a goal nav2 aborts twice from the same spot is
  given up (`nav2_abort_limit`), or once where nav2 had already run its
  recoveries (`nav2_abort_final_after_recovery`). Every goal ends with one `Goal end
  [<outcome>] …` line; see real_robot_tuning.md §5. Keep both tolerances
  strictly looser than the navigator's own goal checker (the node warns below
  0.3 against Nav2's 0.25 defaults) or arrivals are missed and reached goals
  get blacklisted.
- **Stopping takes the action client, not silence.** Ceasing to publish is not
  enough: the last accepted `NavigateToPose` goal runs to completion. The
  planner therefore cancels the in-flight action through a cancel-only client
  on `proximity_nav_cancel_action` (default `/<robot_name>/navigate_to_pose`)
  and also publishes a zero-travel brake goal at its own pose. That client
  exists only while `proximity_stop_enabled` is true (the default); leave it
  on with Nav2.
- **`use_planning_map` defaults to false** in `shared_params.yaml`:
  straight-line costs, no 2D obstacle/reachability filtering, and the planner
  never subscribes to a 2D grid. The real-robot overlay turns it on with
  traversability_mapping, which makes `/occupancy_map_local` a hard startup
  precondition: without `traversability_map.launch.py` running the planner
  waits in WAIT_FOR_MAP indefinitely (logging the window count). The default
  topic, `/<robot_name>/dscovox_node/planning_map`, has no publisher. See
  "The planning map".
- **Coordination** (`coordination_enabled`): planners discover each other via
  `RobotIntent` messages on `coord_intent_topic` (default
  `/exploration/intents`), heartbeated at `coord_heartbeat_hz` (1 Hz). Each
  robot claims a disc around its current goal (`coord_claim_radius_m`, 0 =
  auto = `fov_max_range`; `coord_claim_ttl_sec` / `coord_claim_grace_sec`),
  and peers avoid claimed goals. The proximity stop yields below
  `proximity_hold_dist_m`, resuming beyond `proximity_resume_dist_m`; on
  hardware set `proximity_peer_pose_topics`
  (`["<peer_name>:<topic>"]`, a map-frame `PoseWithCovarianceStamped` from
  each peer's localizer) — on the 1 Hz heartbeat alone the guard sees a fast
  closing peer too late.
- **Rendezvous**: with coordination on and `rendezvous_expected_peers` > 0
  (team size − 1; `multi_robot_exploration.launch.py` derives it from the
  `robots` list, so set it by hand only when launching nodes directly),
  each robot records its pose as "home" the moment
  it starts planning. When exploration exhausts (coverage saturated or
  `max_steps`) with teammates out of comms, it drives back to home and waits
  there — start poses share one comms bubble, so returning re-establishes the
  link and the merged map. With the team present, robots still finish at
  home, so the mission ends with everyone parked together.
  `rendezvous_max_wait_sec` bounds the barrier wait (<= 0 = forever);
  `return_nav_max_timeout_sec` bounds the drive home. `shared_params.yaml`
  ships `done_action: "idle"`, which is what rendezvous needs — finished
  robots stay up and keep beaconing for later finishers. The bare C++
  fallback is `"shutdown"`, so a bring-up that does not load the shared file
  kills the first robot to finish; the node warns at startup if rendezvous is
  active and `done_action` is left there.
- **Per-deployment keys** (set in an overlay like
  `exploration_real_robot.yaml`, layered on `shared_params.yaml`):
  `robot_name`; `base_frame` — empty resolves to `<robot_name>/base_link`,
  which real TF trees rarely have, so set it explicitly; `map_frame`; the ROI
  `roi_yaw_deg`/`roi_origin_x`/`roi_origin_y` +
  `roi_min_x`/`roi_max_x`/`roi_min_y`/`roi_max_y` (candidates outside are
  discarded, and it gates the map precondition above — it must cover the
  survey area; see "Area of interest from the ground-truth map") and z band
  `roi_min_z`/`roi_max_z`; `map_resolution`; robot geometry —
  `candidate_robot_z` (sensor height), `candidate_occ_thresh`,
  `candidate_z_clearance` (terrain mode), and the proximity hold/resume
  distances; motion — `nav_speed_estimate_mps` and the no-progress watchdog
  `progress_window_sec`/`progress_min_distance_m` (must tolerate the
  platform's longest healthy standstill plus a full in-place turn); and the
  rendezvous timeouts above.
- **Termination**: DONE when the ROI unknown fraction stays below
  `done_unknown_fraction` for `done_min_consecutive_steps` planning cycles,
  or at `max_steps`. Metrics stream to `output_csv`.
