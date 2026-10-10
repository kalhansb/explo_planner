# explo_planner — two-robot field trial user manual

How to run a two-robot trial on real hardware: what each robot runs, the
bring-up order, what to watch, and what to do when something goes wrong. The
examples use the phase-1 forest pair, `bunker` and `curt`. The campaign
design is a separate document (§12).

**Contents**

1. [Scope](#1-scope)
2. [What each robot runs](#2-what-each-robot-runs)
3. [Two-robot behaviour](#3-two-robot-behaviour)
4. [Before field day](#4-before-field-day)
5. [Per-robot configuration](#5-per-robot-configuration)
6. [Bring-up order](#6-bring-up-order)
7. [Running the trial](#7-running-the-trial)
8. [What to watch](#8-what-to-watch)
9. [Panic stop and aborts](#9-panic-stop-and-aborts)
10. [Troubleshooting](#10-troubleshooting)
11. [Data offload](#11-data-offload)
12. [Where to go deeper](#12-where-to-go-deeper)

---

## 1. Scope

| Phase | What the planner does |
|-------|-----------------------|
| **EXPLORE** | Drives through Nav2 to the viewpoint expected to reveal the most of the fused map. Repeats until the area's unknown fraction drops below a threshold. |
| **EXPLOIT** | For each tree target, visits three vantages around the trunk and dwells at each so the bag captures overlapping views. Returns to EXPLORE when no targets are left. |

The captured data is fused offline. In the field the planner only positions
the robots and dwells.

There is no base station and no central map server. Each robot localises
against a shared ground-truth map, maps in its own frame, fuses its
teammate's map updates, and plans on its own fused copy.

---

## 2. What each robot runs

```
             bunker                                     curt
 ┌────────────────────────────────┐      ┌────────────────────────────────┐
 │ 1. NDT localiser               │      │ 1. NDT localiser               │
 │      map → odom → base_link    │      │      map → odom → base_link    │
 │ 2. SCovox mapper (bunker_map)  │      │ 2. SCovox mapper (curt_map)    │
 │      → /bunker/…/scovox_bin ───┼──────┼──→                             │
 │                            ←───┼──────┼─── /curt/…/scovox_bin          │
 │ 3. DScovox merger (fuses BOTH) │      │ 3. DScovox merger (fuses BOTH) │
 │      = bunker's global map     │      │      = curt's global map       │
 │ 4. Traversability map + Nav2   │      │ 4. Traversability map + Nav2   │
 │ 5. explo_planner               │      │ 5. explo_planner               │
 └───────────────┬────────────────┘      └───────────────┬────────────────┘
                 └──── /exploration/intents (shared) ────┘
                 └──── peer robot pose (shared) ─────────┘
```

| # | Program | Package | Role |
|---|---------|---------|------|
| 1 | `lidar_localization` + EKF | hmr_localisation | NDT against the shared `gt_map_us050.pcd`, so both robots share one `map` frame. |
| 2 | `scovox_mapping_node` (`mode: rolling`) | scovox | This robot's voxel map and its LZ4 delta stream. |
| 3 | `dscovox_mapping_node` | scovox | Fuses both delta streams into this robot's global map. |
| 4 | `traversability_map.launch.py` + Nav2 | explo_planner, traversability_mapping, nav2 | The traversability map scores the ground; Nav2 reads it and drives to `/<r>/goal_pose`. Both run in the robot's namespace. What the planner needs from Nav2: [nav2_config.md](nav2_config.md). |
| 5 | `explo_planner_node` | explo_planner | Goals, coordination, metrics. |

Only the delta streams, `/exploration/intents` and the localiser poses cross
the mesh. Raw LiDAR, IMU and RGB-D stay in each robot's bag.

---

## 3. Two-robot behaviour

Three coordination layers, all on by default. Each needs the teammate alive
and on the mesh. The mechanism in full:
[planner_method.md §10](planner_method.md#10-multi-robot-operation).

### 3.1 MinPos goal deconfliction

Each robot claims its goal on `/exploration/intents` at 1 Hz. When both want
one area, the closer robot keeps it. A claim lapses 5 s after it was last
heard (`coord_claim_ttl_sec`), so a robot that drops off the mesh frees its
area within about 5 s. Tree vantages use small claims, so the pair splits the
angles around a trunk.

### 3.2 Coordinated proximity stop

MinPos separates goals, not routes. When a teammate with right of way moves
nearby, the other robot cancels its Nav2 goal, holds, and resumes the same
goal once the teammate has moved off or parked.

| | |
|---|---|
| **Right of way** | The alphabetically smaller `robot_name`, so **curt yields** to bunker. Both robots decide from the names alone, so exactly one stops. |
| **Hold / resume** | Holds below 5 m, resumes beyond 6 m. |
| **Parked peer** | A teammate still for 10 s counts as an obstacle and is not held for, unless it is within 1.5 m. Then the hold lasts until it moves, 120 s at most. |
| **Peer pose source** | The 1 Hz intents plus `proximity_peer_pose_topics`, which must be set on hardware (§5). Intents alone lag by metres. |

> Best-effort coordination, not a certified safety stop. It needs live peer
> data, both planners running, and Nav2 honouring the cancel. The crewed
> 1.5 m panic stop (§9) is the hard backstop.

### 3.3 Rendezvous reconnection

A robot that runs out of goals drives to its start point (where it began
planning, inside the launch comms bubble). If its teammate is out of comms,
it waits there and re-plans on the merged map once they reconnect. Neither
robot finishes while the other explores, and both end parked at the launch
site.

- **Drive home** gives up after `return_nav_max_timeout_sec` (600 s). The
  robot waits where it is.
- **The wait** gives up after `rendezvous_max_wait_sec` (600 s). The robot
  parks in `DONE` and keeps broadcasting, so its teammate can still find it.

Rendezvous needs `rendezvous_expected_peers: 1`. `shared_params.yaml` ships
0 (off); the overlay sets 1 (§5).

---

## 4. Before field day

1. **Build the planner into the image, not `/tmp`.** explo_planner builds in
   an overlay workspace inside the `scovox` container. A build in
   `/tmp/explo_ws` is lost whenever `docker compose up` recreates the
   container. Bake it into the image, or be ready to re-run:
   ```bash
   docker exec scovox bash -c 'rm -rf /tmp/explo_ws/src/explo_planner && mkdir -p /tmp/explo_ws/src'
   docker cp . scovox:/tmp/explo_ws/src/explo_planner   # always onto a fresh dir
   docker exec scovox bash -c 'source /opt/ros/jazzy/setup.bash && source /scovox/install/setup.bash \
     && cd /tmp/explo_ws && colcon build --packages-up-to explo_planner --cmake-args -DCMAKE_BUILD_TYPE=Release'
   ```
   - The image lacks `nav2_msgs`, which the build needs:
     `apt-get install -y ros-jazzy-nav2-msgs`.
   - Use `--packages-up-to`, not `--packages-select`, so
     `explo_planner_msgs` is built too.
   - Build Release. Debug runs 3–4× slower.

2. **Run the same `explo_planner_msgs` build on both robots.** Its
   `RobotIntent` carries the exploit fields (`exploit`, `target_id`,
   `dwelled_mask`). Without them, team vantage sharing degrades and the
   exploitation-contribution metric cannot be computed offline.

3. **Namespace every sensor topic per robot.** Robots with the same LiDAR
   default to the same `/hesai/points` and `/imu/data` and collide. The same
   goes for the localiser's pose: launched alone it publishes `/pcl_pose`.
   Run it in the robot's namespace so it publishes `/<r>/pcl_pose`, the name
   the teammate's `proximity_peer_pose_topics` uses (§5).

4. **Clock sync.** chrony over the mesh, `bunker` as time master. The other
   robot's offset must be **< 50 ms**. This is a go/no-go gate: log the value
   on the run sheet.

5. **Network.** Both robots on one subnet with the same `ROS_DOMAIN_ID`. The
   compose files limit DDS discovery to loopback, so a `docker compose exec`
   that must reach the other robot needs
   `-e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.

6. **Disk and batteries.** At least 250 GB free per robot, and enough charged
   packs. One-hour batteries **cap every run at 55 min** (5 min reserve to RC
   home). A full day is up to 7 runs per robot.

7. **Tape the DBH** of every target trunk and update the radii in the target
   YAML. The shipped radii of unverified trees are upper bounds.

---

## 5. Per-robot configuration

Every launch loads [`shared_params.yaml`](../explo_planner/config/shared_params.yaml)
unchanged (field defaults: `use_sim_time: false`, `done_action: idle`,
`terrain_relative_z: true`, `coordination_enabled: true`), then two
overlays. A later `--params-file` wins.

1. [`exploration_real_robot.yaml`](../explo_planner/config/exploration_real_robot.yaml),
   shipped for this pair. Its `/bunker/explo_planner` and
   `/curt/explo_planner` blocks set `robot_name`, `base_frame`,
   `targets_topic`, `rendezvous_expected_peers: 1` and the planning-map
   topic. They match only a node in the robot's namespace (§6 step 7). Its
   `/**` block sets timeouts measured on these robots. It also turns the
   planning map on, so the planner waits in `WAIT_FOR_MAP` until
   `/<r>/occupancy_map_local` arrives.
2. A per-robot site file: ROI box, peer pose topic, CSV name.

How each overlay value was derived, and the keys to update before real runs:
[real_robot_tuning.md](real_robot_tuning.md).

```yaml
# field_curt.yaml   ('/**' so it matches the node in any namespace)
/**:
  ros__parameters:
    output_csv: "/tmp/RA-1_curt.csv"
    # Peer robot pose from its localiser. Intents alone are too coarse to
    # brake on. Confirm the name with `ros2 topic list | grep pcl_pose`.
    proximity_peer_pose_topics: ["bunker:/bunker/pcl_pose"]
    # ROI box for this site: roi_yaw_deg, roi_origin_x/y, roi_min/max_x/y
```

### Must differ between the two robots

| Setting | `bunker` | `curt` | If wrong |
|---------|----------|--------|----------|
| `robot_name` | `bunker` | `curt` | Sets default topic names **and** right of way (§3.2). |
| `output_csv` | `/tmp/<run>_bunker.csv` | `/tmp/<run>_curt.csv` | No per-run metrics. |
| `targets_topic` | `/exploration/targets/bunker` | `/exploration/targets/curt` | Both robots take both target lists and double-book the queue. |
| `proximity_peer_pose_topics` | `["curt:/curt/pcl_pose"]` | `["bunker:/bunker/pcl_pose"]` | Braking falls back to the 1 Hz intents; the planner warns at startup. |
| SCovox `integration_frame` | `bunker_map` | `curt_map` | The robots' maps overwrite each other. Each frame needs an identity static TF from `map`. |

### Must be identical on both robots

| Setting | Why |
|---------|-----|
| SCovox base config (`num_classes`, Dirichlet prior) | The merger **drops** a source whose prior differs (`prior mismatch … dropping frame`). |
| `n_vantages`, `vantage_start_angle_deg` | Dwells are shared by vantage index. A mismatch can close a trunk on an angle nobody captured ([limitations.md](../explo_planner/doc/limitations.md) §3). |
| `coord_intent_topic` (`/exploration/intents`) | Shared, root namespace. |
| DScovox `input_topics` | **Both** robots' bin topics, on **both** robots. |
| ROI box (`roi_yaw_deg`, `roi_origin_x/y`, `roi_min/max_x/y`) | Coverage-done is measured over it. Different boxes disagree on when exploration ends. |

### Check these against the platform

| Setting | Shipped | Check |
|---------|---------|-------|
| `base_frame` | `""` → `<robot_name>/base_link` | Neither platform has that frame. The overlay sets bunker `base_link`, curt `base_link_curt`. The single-robot launch needs `base_frame:=`. Wrong → no pose, and the planner never reaches PLAN. |
| `goal_xy_tolerance` / `goal_yaw_tolerance` | `0.4` / `0.4` | Must be looser than Nav2's goal checker (default 0.25 / 0.25). Otherwise arrival is never registered and the planner blacklists the goal the robot is standing on. The node warns at startup. |
| `goal_republish_sec` | `5.0` | Each re-send restarts the Nav2 goal and cuts its recoveries short, so lower values turn transient failures into aborts. `0` sends only on change, once Nav2 bring-up is reliable. |
| ROI box | 100 × 25 m, yawed 25.75° (the gt_map site's tree block) | Set it to the area being worked. A box that misses the area starves the planner of goals. One much larger than the reachable area stops coverage-done firing, so the run ends on battery. The startup log prints the ROI's map-frame extent. |
| Z-band | `roi_min/max_z` = `-5.5 … +4.0`, **robot-relative** | Moves with the robot. SCovox/DScovox `share_roi_z_min/max` are **absolute** and must cover everywhere the band can sit, or voxels near its edge read as unknown (starved candidates, phantom frontiers). |
| `done_unknown_fraction` | `0.05` × 3 steps | **Calibrate on the validation run**: set it above where the logged `source=scovox` fraction levels off. |
| `candidate_enable_polar` | `false` (frontier goals only) | Turn it on if, on the validation run, the robot runs out of goals near the ROI corners and oscillates. Adds 96 sample goals around the robot. |

---

## 6. Bring-up order

Order matters: localiser before mapper (no TF, no map), **merger before
mapper** (the delta stream only publishes once something subscribes), planner
last. Run each step on **both** robots, inside the containers, workspace
sourced.

**1. Localiser.** NDT against the shared GT map, plus the EKF. Wait for
`Activating end` in the log. Check the pose is within **0.5 m** of the
marked staging pose.

**2. Identity static TF** from `map` to the robot's own map frame:
```bash
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id bunker_map
```

**3. DScovox merger first:**
```bash
ros2 run scovox_mapping dscovox_mapping_node --ros-args -r __ns:=/bunker \
    --params-file /scovox/src/scovox_mapping/config/dscovox_params.yaml
```
`input_topics` must list **both** robots' bin topics.

**4. SCovox rolling mapper**: base config, share overlay, this robot's frame:
```bash
ros2 run scovox_mapping scovox_mapping_node --ros-args \
    -r __ns:=/bunker -r __node:=scovox_node \
    --params-file /scovox/config/scovox_fused_lidar_rgbd.yaml \
    --params-file /scovox/config/scovox_robot_share.yaml \
    -p integration_frame:=bunker_map
```

**5. Traversability map, then Nav2.** The traversability map scores the
ground from the LiDAR. Nav2 reads it, and the planner waits for it
([field_setup.md](field_setup.md), "Traversability map"):

```bash
# bunker
ros2 launch explo_planner traversability_map.launch.py namespace:=bunker \
  input_cloud_topic:=/hesai/points
# curt
ros2 launch explo_planner traversability_map.launch.py namespace:=curt \
  base_frame:=base_link_curt lidar_frame:=os_lidar \
  input_cloud_topic:=/ouster/points
```

Then Nav2, in the robot's namespace, from the robot's own nav2 params file
unchanged. Use this in place of `nav2_bringup navigation_launch.py`:

```bash
ros2 launch explo_planner nav2_namespaced.launch.py namespace:=<r> \
  params_file:=<the robot's nav2 params>
```

It keeps TF on `/tf` and points Nav2 at the namespaced traversability
topics. Its header lists what it changes. What the planner needs from Nav2:
[nav2_config.md](nav2_config.md).

**6. Segmentation** (bunker), **multispectral calibration panel capture**
(curt), E-stop test, then **start all recordings**.

**7. T0: the planners.** One per robot, on its own PC. `-r __ns:=/bunker`
makes the overlay's `/bunker/explo_planner` block apply:
```bash
CFG=/tmp/explo_ws/install/explo_planner/share/explo_planner/config
ros2 run explo_planner explo_planner_node --ros-args -r __ns:=/bunker \
    --params-file $CFG/shared_params.yaml \
    --params-file $CFG/exploration_real_robot.yaml \
    --params-file /field/field_bunker.yaml     # site keys: ROI box, peer pose topic, CSV
```
For a robot running **alone**, use the single-robot launch. Its node is the
un-namespaced `/explo_planner`, so the per-robot blocks do not apply, and
its arguments beat the yaml for the frames. Pass `base_frame:=`:
```bash
ros2 launch explo_planner exploration_experiment.launch.py \
    robot:=bunker base_frame:=base_link output_csv:=/tmp/RA-1_bunker.csv \
    params_file:=$CFG/exploration_real_robot.yaml
```

> `multi_robot_exploration.launch.py` starts **both** planners on one host.
> That is the simulation layout, not the field one.

Do **not** launch the target scheduler at T0 (§7).

### Startup lines that confirm a correct launch

```
EIG exploration planner ready: max_steps=200 frame=map base=base_link
    planning_map=/bunker/occupancy_map_local goal=/bunker/goal_pose
Subscribing to fused map (dscovox): /bunker/dscovox_node/scovox
Terrain-relative z ON: map z-band [-5.5, +4.0] m about the robot, …
Exploitation enabled: subscribing to tree targets on /exploration/targets/bunker
    (n_vantages=3, min_required=3, dwell=8.0s)
Proximity stop enabled: hold < 5.0 m, resume > 6.0 m, cancel via '/bunker/navigate_to_pose' …
Proximity stop: tracking peer 'curt' via /curt/pcl_pose
```

Any of these means a per-robot setting did not take:

| Line | Meaning |
|------|---------|
| `Rendezvous inactive (coordination_enabled=1, expected_peers=0)` | `rendezvous_expected_peers` is 0. Reconnection is off. |
| `Proximity stop: no peer pose topics configured — running on the …` | Braking on the 1 Hz intents only. |
| `Proximity stop: N peer pose topic(s) configured but nothing …` | Wrong topic name, or the peer is not on the mesh. |
| `Coverage termination … INACTIVE` | Nothing can measure the unknown fraction. The run can only end on `max_steps`. |
| `Fused map frame_id '…' != planner map_frame 'map'` | Voxels land in the wrong place. |

---

## 7. Running the trial

| Step | Cue | Action |
|------|-----|--------|
| **T0** | All checks green | Launch both planners. Both enter EXPLORE. |
| **Coverage-done** | `Step N: ROI unknown fraction 0.043 < 0.050 (source=scovox, streak 3/3)`, then `Exploration finished; idling (done_action=idle)` | Director launches the target scheduler(s). |
| **EXPLOIT** | `Target queued (1 pending) -> switching to EXPLOIT` | Robots ring each trunk: 3 vantages, 8 s dwell each. |
| **Revert** | `Target queue empty -> reverting to EXPLORE` | Back to exploration, or idle at DONE. |
| **End** | All targets closed and exploration DONE | Stop planners → robots home under RC → **stop recordings last** → `docker stop` the containers. |

Start the scheduler at the cue:

```bash
ros2 run explo_planner target_scheduler_node --ros-args \
    --params-file /field/targets_phase1_bunker.yaml
```

The scheduler starts its clock on its **first tick** and releases each target
`target_release_sec` after that. To release everything at the cue, set every
release time to `0.0`. Targets are published `transient_local`, so a planner
that subscribes later still gets them. `done_action: idle` keeps the
planners running at DONE, so the release cannot race a shutdown.

- **Type A** (explore all, then exploit): release every target at one cue.
  If coverage-done has not come by 40 min, the Director releases the targets
  anyway; note it on the sheet as a partial-coverage run.
- **Type B** (sub-areas interleaved): release each sub-area's list at its own
  coverage-done, then move the planners to the next sub-area.

---

## 8. What to watch

### Console, per robot

| Line | Reading |
|------|---------|
| `Step N: ROI unknown fraction …` | Exploration progress. Should fall, then level off. |
| `Step N: navigation failed [no-progress] … Blacklisted; K active …` | Occasional is normal. **Three on the same spot is a panic-stop trigger.** |
| `Goal end [reached] step N explore (x, y): 41.0s of 120s budget, path 8.3 m, …; nav2 SUCCEEDED, 0 abort(s), 1 recovery(ies)` | One per goal, whatever the outcome. Mostly `[reached]` is healthy. Recoveries on most goals mean Nav2 is struggling with the ground. |
| `navigation failed [nav2-aborted]` + `nav2 abort reason: error_code 204 GOAL_OUTSIDE_MAP` | Nav2 refused the goal twice; the planner picks another. Occasional is normal. On every other goal, Nav2's global costmap is too small for the goals (field_setup.md). |
| `nav2 ABORTED the current goal … giving the goal up: nav2 ran N recovery(ies) here first` | The robot was stuck and Nav2's recoveries failed, so the planner drops the goal. |
| `Proximity hold #N: yielding to 'bunker' at 4.20 m (< 5.00 m) …` | curt yielded. Expected where routes cross. |
| `Target 1: vantage 1/3 selected at (…) cost=4.31 (3 valid, 0 clear-LoS dwelled)` | Exploitation working the ring. `valid` falls as vantages are used. |
| `Target 1 exploitation PARTIAL (1/3 clear-LoS vantages dwelled)` | The trunk closed without full coverage. Note it; do not abort. |
| `Rendezvous: … team incomplete (0/1 peers) -> returning home (x, y).` | Comms lost at the end of exploration. Driving to the start point, not stuck. |
| `Rendezvous: waiting at home for team (0/1 present).` | Waiting for the teammate. Ends on reconnection or after `rendezvous_max_wait_sec`. |

### Topics

```bash
ros2 topic hz /bunker/scovox_node/scovox_bin        # ~2 Hz with the share overlay
ros2 topic hz /bunker/dscovox_node/pointcloud       # fused map flowing
ros2 topic echo /bunker/proximity_hold_state --once # latched: hold/clear + reason
ros2 topic echo /exploration/intents                # both robots' claims
```

**The most important check** is each merger's console line, about every 5 s:

```
dscovox_diag: sources=2 src_voxels=… fused_voxels=…
```

`sources=2` on **both** robots means the pair is sharing a map. `sources=1`
means one robot is mapping alone and the trial is not measuring what it
should. In a real merge `src_voxels > fused_voxels` (the maps overlap), and
the two robots' `fused_voxels` converge.

### Per-step CSV

Written to `output_csv` on each robot. Columns worth watching live:

| Column | Reading |
|--------|---------|
| `total_observed_voxels` | Should climb steadily. Flat means the map is not growing. |
| `distance_traveled` | Climbs on each robot. Two identical traces mean the robots are shadowing each other. |
| `coord_active_peers` | **1** in a healthy two-robot run. 0 means the teammate is not heard. |
| `rejected_by_minpos` | Non-zero confirms deconfliction is firing. |
| `phase`, `target_id`, `vantage_index` | The EXPLOIT windows for the offline per-trunk metric. |
| `prox_hold_count`, `prox_hold_total_sec` | Cumulative proximity holds. Compare with the trajectory afterwards. |
| `goal_outcome`, `nav_elapsed_sec`, `goal_path_length_m` | How each goal ended, how long it took, and its route length. A run of `no-progress` / `nav2-aborted` means the robot is fighting the terrain. |
| `nav2_aborts`, `nav2_recoveries`, `nav2_error_code` | Nav2's side of the same goal (error code on Jazzy only). |

---

## 9. Panic stop and aborts

Anyone calls **"PANIC STOP"** → Safety Officer e-stops both robots → Director
stops the planners, then the containers (`docker stop`, **never** `pkill`).
**Recording is stopped last.**

Triggers, any one:

1. **Geofence**: any robot more than 5 m outside the AO polygon.
2. **Proximity**: robots < 1.5 m apart and closing, a person within 1 m, or
   contact with a tree.
3. **Localisation divergence**: pose jump > 2 m, or the footprint visibly off
   the map. One robot diverging → stop that robot; both → full stop.
4. **Runaway navigation**: ≥ 3 blacklist cycles on the same spot, or > 3 min
   of oscillation.
5. **Platform health**: battery < 20 %, motor fault, tilt > 20°.
6. **Comms**: mesh loss to any robot > 15 s.
7. **Recording**: any bag stops growing.
8. **Environment**: person or animal in the plot, rain, or lighting outside
   the multispectral envelope.

Classify afterwards: **resumable** (brief single-robot hiccup, map intact) or
**void** (divergence, comms loss > 60 s, environment breach → rerun).

---

## 10. Troubleshooting

Almost every "no map" report is one of SCovox's **three gates** on the delta
stream: see the [SCovox user manual](../../scovox/docs/archive/user_manual.md)
§8. Two-robot and planner failures:

| Symptom | Cause → fix |
|---------|-------------|
| Merger stuck at `sources=1` | The peer's bin stream is not arriving. Check `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, same subnet, and that `input_topics` lists both robots. |
| `prior mismatch … dropping frame` | The robots run different SCovox base configs (§5). |
| Robots' maps overwrite each other | Both map in the same `integration_frame` (§5). |
| Planner sits in `WAIT_FOR_MAP` | The `Waiting to start: map=… pose=… planning_map=…` line (every 5 s) shows what is missing (0 = not yet). map: merger `fused_voxels > 0`? mapper `scovox_bin` flowing? pose: localiser publishing TF? planning_map: traversability map running in the robot's namespace? |
| Planner never ticks | `use_sim_time: true` with no `/clock` publisher. The 10 Hz timer never fires. After 10 s the node logs `use_sim_time=true but no /clock received`. Field runs need `false`. |
| Both robots pick the same goal | `coordination_enabled` is off on one, or `/exploration/intents` is not crossing the mesh. `coord_active_peers` in the CSV should be 1. |
| Robots never yield to each other | `proximity_stop_enabled` is off, or no peer pose topic is set (startup warning). |
| One robot finishes and stops while the other explores | `rendezvous_expected_peers` is 0: the planner was not launched with `-r __ns:=/<r>` (§6 step 7). |
| Goal blacklisted at the robot's own position | Goal tolerances tighter than Nav2's goal checker (§5). |
| Transient nav failures become hard aborts | `goal_republish_sec` too low (§5). |
| Coverage-done never fires | The ROI box takes in unreachable ground. Shrink it to the area being worked, or set `done_unknown_fraction` above the measured floor (§5). |
| Target activates, no vantage ever appears | All three failed the line-of-sight check. Check the target radius against the taped DBH, and that `vantage_start_angle_deg: 30` suits the row layout. A 0° start puts the first vantage straight down a plantation row and the other two in neighbouring canopy. |
| Trunk closes `PARTIAL` repeatedly | Vantages are reached and blacklisted without a dwell. Check the goal tolerances (§5) before the vantage geometry. |

---

## 11. Data offload

With no base station, the two per-robot bags are the **only** recordings.
The shared topics (`/exploration/intents`, each robot's `scovox_bin` as
received) are in both bags, as a fallback if one bag is lost.

Per robot, after every run:

1. `ros2 bag info` on the bag. Confirm duration and message counts.
2. Planner CSV (`output_csv`).
3. Multispectral SD card and the closing calibration panel capture (curt).
4. Run sheet: config git SHA, chrony offsets, lighting log, taped DBH,
   EXPLORE↔EXPLOIT switch counts, anomalies, battery state.

The exploitation-contribution metric needs **both** the planner CSV and
`/exploration/intents` with the exploit fields. Check both are in the
validation-run bag before spending a battery on a measured run.

---

## 12. Where to go deeper

| Doc | Use it for |
|-----|-----------|
| `doc/experiment_script_forest_inspection.md` (hmr_explo workspace) | Campaign design: AO, sub-areas, targets, run matrix, metrics. |
| [planner_method.md](planner_method.md) | How the planner works, including MinPos, rendezvous, the proximity stop and vantage selection. |
| [`shared_params.yaml`](../explo_planner/config/shared_params.yaml) | Every parameter, with the reasoning behind each default. |
| [real_robot_tuning.md](real_robot_tuning.md) | How the hardware overlay's values were derived, and the keys to update before real runs. |
| [SCovox user manual](../../scovox/docs/archive/user_manual.md) | Mapping and fusion: bring-up, the three delta-stream gates, bandwidth tuning. |
| [dscovox_exploration_run.md](../explo_planner/doc/dscovox_exploration_run.md) | Bag-replay dry run of exploration. |
| [dscovox_exploitation_run.md](../explo_planner/doc/dscovox_exploitation_run.md) | Bag-replay dry run of the vantage ring, the target queue and the live tree detector. |
| [limitations.md](../explo_planner/doc/limitations.md) | Known rough edges. Read §3 before changing a vantage parameter on one robot only. |
