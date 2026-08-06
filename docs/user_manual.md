# explo_planner — two-robot field trial user manual

How to run a **two-robot coupled exploration/exploitation trial** on real
hardware: what each robot runs, the order it comes up in, what a healthy run
looks like, and what to do when it isn't. Written for the phase-1 forest
inspection campaign — **`bunker` + `curt`** — but nothing here is specific to
those two names beyond the worked examples.

This is the *operating* manual. The campaign design — area of operations, run
matrix, targets, metrics — is `doc/experiment_script_forest_inspection.md` in
the hmr_explo workspace. Read that first if you are planning the trial; read
this if you are running it.

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

Two robots explore a shared area, build one co-registered map between them, and
then circle assigned trees to capture close-range imagery.

| Phase | What the planner does |
|-------|-----------------------|
| **EXPLORE** | Selects next-best viewpoints by expected information gain over the fused SCovox map, drives to them through Nav2, and repeats until the area's unknown fraction saturates. |
| **EXPLOIT** | On receiving a tree target, leaves exploration, visits three occlusion-free vantage points around the trunk and dwells at each so the rosbag captures overlapping views. Reverts to exploration when the queue drains. |

Sensor fusion of the captured data is **offline**. The planner only positions
and dwells; nothing is measured in the field.

**There is no base station and no central map server.** Each robot localises
against a shared ground-truth map, maps in its own frame, fuses its teammate's
map deltas locally, and plans against its own fused copy. The mesh carries
map deltas, coordination intents and peer poses — never raw sensor data.

---

## 2. What each robot runs

Five programs per robot, identical on both:

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
 │ 4. Nav2 (GT costmap)           │      │ 4. Nav2 (GT costmap)           │
 │ 5. explo_planner               │      │ 5. explo_planner               │
 └───────────────┬────────────────┘      └───────────────┬────────────────┘
                 └──── /exploration/intents (shared) ────┘
                 └──── peer robot pose (shared) ─────────┘
```

| # | Program | Package | Role |
|---|---------|---------|------|
| 1 | `lidar_localization` + EKF | hmr_localisation | NDT against the **shared** `gt_map_us050.pcd`. This is what puts both robots in one global `map` frame — no robot-to-robot pose estimation is needed. |
| 2 | `scovox_mapping_node` (`mode: rolling`) | scovox | Per-robot voxel map; emits the LZ4 delta stream. |
| 3 | `dscovox_mapping_node` | scovox | Fuses **both** robots' delta streams into this robot's global map. |
| 4 | Nav2 with the GT costmap | — | Consumes `/<r>/goal_pose`, owns obstacle avoidance. |
| 5 | `explo_planner_node` | explo_planner | Viewpoint/vantage selection, coordination, metrics. |

Only three things cross the mesh: each robot's `scovox_bin` delta stream,
`/exploration/intents`, and each peer's **robot pose** from its localiser.
Everything else — raw LiDAR, IMU, RGB-D — stays in the robot's own bag.

---

## 3. Two-robot behaviour

Three coordination layers run on top of single-robot exploration. All three
are **on by default** and all three need the teammate to be alive and reachable.

### 3.1 MinPos goal deconfliction

Each robot broadcasts its selected goal as a claim on `/exploration/intents`
at 1 Hz. A candidate inside a peer's claim disc is rejected unless this robot
is closer. Claims expire after `coord_claim_ttl_sec` (5 s), so a robot that
drops off the mesh releases its targets within ~5 s and the other continues.

Exploitation uses a **separate, much smaller** claim disc: three vantages on one
trunk sit only a couple of metres apart, and the exploration-scale disc would
swallow the whole ring and veto the tree instead of splitting angles across the
pair.

### 3.2 Coordinated proximity stop

MinPos deconflicts *goals*, not *paths* — two commanded routes can still cross.
While driving, each planner watches its teammate's live pose and yields when a
higher-priority teammate is moving nearby: it cancels the in-flight Nav2 goal,
publishes a brake goal at its own pose, holds, and resumes the same goal once
the peer clears or parks.

| | |
|---|---|
| **Right of way** | The **lexicographically smaller `robot_name`**. `bunker` < `curt`, so **curt yields**. Computed from ids alone, so both robots always agree: exactly one stops — never both, never neither. |
| **Hold / resume** | Enters below 5 m, releases beyond 6 m (hysteresis). |
| **Parked peer** | A peer that has not moved for 10 s is treated as a static obstacle and released — *unless* it sits inside 1.5 m, where the hold continues until it moves or the 120 s escape hatch fires. |
| **Peer pose source** | The 1 Hz intent heartbeat **plus** `proximity_peer_pose_topics`. On hardware you must configure the latter (§5) — at field closing speeds the heartbeat alone leaves metre-scale pose lag. |

> This is best-effort **coordination, not a certified safety stop**. It needs
> live peer data, both planners alive, and Nav2 honouring the cancel. The
> crewed 1.5 m panic stop (§9) remains the hard backstop.

### 3.3 Reconnection: rendezvous / pursuit / hybrid

A robot that loses comms keeps exploring alone. When it exhausts its goals it
does **not** stop while its teammate is still out of range: it runs the
`reconnect_mode` manoeuvre and holds at its endpoint until the team is back in
comms, then re-plans against the merged map. Neither robot can finish while
the other is still exploring — and a finished robot keeps beaconing while it
idles in `DONE`, so the slower robot can count it (this needs the field
default `done_action: idle`; `shutdown` would make the first finisher
invisible and the node warns about it at startup).

The modes, with the radios carried on the robots (no base station):
`rendezvous` returns to the **last-connected anchor** (own pose at last
contact); `pursuit` chases the missing teammate's last declared goal and then
its last heard pose, on a time budget (`pursuit_budget_max_sec`, skipped
entirely once the record is older than `pursuit_staleness_max_sec`); `hybrid`
— the shipped yaml default — chases on the budget, then waits at the midpoint
of the last-contact pose pair, which both robots compute independently.

The records need no configuration; they build automatically from incoming
peer intents. **`rendezvous_expected_peers` must be set to 1 by hand on a
hardware launch** (§5) — the shipped value is `0`, which leaves the whole
feature (all three modes) inert.

---

## 4. Before field day

1. **Build the overlay into an image, not `/tmp`.** `explo_planner` has no
   container of its own — it builds in an overlay workspace inside the running
   `scovox` container. The runbook approach (copy to `/tmp/explo_ws`, colcon
   build there) lives in the container's **writable layer**: any
   `docker compose up` that *recreates* the container loses it. Bake it into
   the image before field day, or be ready to re-run the copy + build.
   ```bash
   docker exec scovox bash -c 'rm -rf /tmp/explo_ws/src/explo_planner && mkdir -p /tmp/explo_ws/src'
   docker cp . scovox:/tmp/explo_ws/src/explo_planner   # always onto a fresh dir
   docker exec scovox bash -c 'source /opt/ros/jazzy/setup.bash && source /scovox/install/setup.bash \
     && cd /tmp/explo_ws && colcon build --packages-up-to explo_planner --cmake-args -DCMAKE_BUILD_TYPE=Release'
   ```
   Two build traps: the image ships neither `nav2_msgs` (needed for the
   proximity-stop cancel — `apt-get install -y ros-jazzy-nav2-msgs`) nor
   `explo_planner_msgs` (use `--packages-up-to`, **not** `--packages-select`).
   Release builds are mandatory; debug inflates timing 3–4×.

2. **Confirm `scovox_msgs` is rebuilt to the current `RobotIntent.msg`.** The
   exploit fields (`exploit`, `target_id`, `dwelled_mask`) are absent from a
   stale installed interface. Without them the team-quota vantage sharing
   degrades **and** the exploitation-contribution metric cannot be computed
   offline.

3. **Namespace every sensor topic per robot.** Platforms sharing a LiDAR model
   default to the same `/hesai/points` and `/imu/data` and will collide when
   run together.

4. **Clock sync.** chrony over the mesh with `bunker` as time master. Offset
   **< 50 ms** on the other robot — this is a go/no-go gate; log the measured
   value on the run sheet.

5. **Network.** Both robots on one subnet with the same `ROS_DOMAIN_ID`. The
   compose files pin DDS discovery to loopback, so any `docker compose exec`
   that must talk across machines needs
   `-e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.

6. **Disk** ≥ 250 GB per robot, and enough charged battery packs. One-hour
   batteries **hard-cap every run at 55 min** (5 min reserve to RC home); a
   full day is up to 7 runs per robot.

7. **Tape the DBH** of every target trunk and update the target YAML radii.
   The shipped values for unverified trees are cluster-fit *upper bounds*.

---

## 5. Per-robot configuration

All tuning lives in [`shared_params.yaml`](../explo_planner/config/shared_params.yaml)
— the parameter file shared by every launch and both robots — which ships
**field defaults** (`use_sim_time: false`, `done_action: idle`,
`terrain_relative_z: true`, `coordination_enabled: true`). Both robots load
that file unchanged, plus a small **per-robot overlay** for the handful of
values that must differ.

Ship the overlay as its own file and pass both — later `--params-file` wins,
the same base + overlay idiom SCovox uses:

```yaml
# field_curt.yaml   ('/**' so it matches the node in any namespace)
/**:
  ros__parameters:
    robot_name: "curt"
    output_csv: "/tmp/RA-1_curt.csv"
    targets_topic: "/exploration/targets/curt"
    # Reconnection is INERT at the shipped 0. Team size minus this robot.
    # (reconnect_mode / pursuit_* ride along from the shared yaml: hybrid,
    # 240 s chase ceiling, 180 s staleness gate — retune for the site scale.)
    rendezvous_expected_peers: 1
    # Peer robot pose from its localiser (~10 Hz), whatever that topic is
    # called on the platform. Intents alone are too coarse to brake on.
    proximity_peer_pose_topics: ["bunker:/bunker/robot_pose"]
```

### Must differ between the two robots

| Setting | `bunker` | `curt` | Consequence if wrong |
|---------|----------|--------|----------------------|
| `robot_name` | `bunker` | `curt` | Drives every default topic name **and** right of way (§3.2). |
| `output_csv` | `/tmp/<run>_bunker.csv` | `/tmp/<run>_curt.csv` | No per-run metrics. |
| `targets_topic` | `/exploration/targets/bunker` | `/exploration/targets/curt` | Both robots ingest both target lists and double-book the queue. |
| `proximity_peer_pose_topics` | `["curt:<curt robot pose>"]` | `["bunker:<bunker robot pose>"]` | Guard falls back to the 1 Hz heartbeat; metre-scale pose lag at closing speed. The planner **warns at startup** in this mode. Use whatever pose topic the localiser publishes on that platform. |
| SCovox `integration_frame` | `bunker_map` | `curt_map` | Two robots mapping in one frame collapse into a single source and overwrite each other. Each needs an identity static TF from `map`. |

### Must be identical on both robots

| Setting | Why |
|---------|-----|
| SCovox base config (`num_classes`, Dirichlet prior) | The merger pins the map prior from the first stream and **drops** any source whose prior differs (`prior mismatch … dropping frame`). |
| `n_vantages`, `vantage_start_angle_deg` | Team dwell credit is exchanged as ring **indices**. A mismatch silently credits the wrong bearings and can close a trunk on an angle nobody captured ([limitations.md](../explo_planner/doc/limitations.md) §3). |
| `coord_intent_topic` (`/exploration/intents`) | Shared, root namespace. |
| DScovox `input_topics` | Must list **both** robots' bin topics, on **both** robots. |
| `roi_min/max_x/y` | Coverage-done is measured over this box; different boxes mean the pair cannot agree on when exploration is finished. |

### Check these against the platform

| Setting | Shipped | Check |
|---------|---------|-------|
| `base_frame` | `""` → `<robot_name>/base_link` | If the localiser publishes a bare `base_link`, set it explicitly or every TF lookup fails. |
| `goal_xy_tolerance` / `goal_yaw_tolerance` | `0.4` / `0.4` | Must be strictly **looser** than Nav2's goal checker (shipped default 0.25/0.25). If tighter, Nav2 stops inside its own tolerance but outside the planner's, arrival is never registered, and the planner blacklists a goal the robot is standing on. The node warns at startup. |
| `goal_republish_sec` | `5.0` | Nav2 turns every `goal_pose` into a fresh `NavigateToPose` goal; an unthrottled re-send fires `GoalUpdated` continuously, which halts the recovery subtree while still burning its retries — transient failures become aborts. Set `0` for publish-on-change once bringup is known reliable. |
| `roi_min/max_x/y` | full-AO box | Tighten to the phase-1 box (commented next to these values in the yaml) when only part of the AO is worked. The full box wastes candidates and leaves the coverage floor high enough that `done_unknown_fraction: 0.05` may never be satisfiable — the run then ends on battery instead of on coverage. |
| Z-band | planner `roi_min/max_z` = `-5.5 … +4.0`, **robot-relative** | With `terrain_relative_z: true` this band rides with the robot. The SCovox/DScovox `share_roi_z_min/max` filters are **absolute** and must be a **superset** of everywhere that window can sit. Otherwise free voxels near the edge never reach the fused map and read as unknown — starved candidates and phantom frontiers at the boundary. |
| `done_unknown_fraction` | `0.05` × 3 steps | **Calibrate at the shakedown run.** Watch where the logged `source=scovox` fraction plateaus and set the threshold above that floor. |
| `candidate_enable_polar` | `true` | Check at shakedown whether selected goals sit far apart or bunch near the robot. The 96-sample polar ring can outscore frontier goals and burn battery on short hops; if so run frontier-only for the timed runs. |

---

## 6. Bring-up order

Order matters at three points: the localiser must be up before the mapper (no
TF, no map), the **merger must start before the mapper** (the delta stream is
subscriber-gated), and the planner comes last.

Run each step on **both** robots. All commands run inside the containers with
the workspace sourced.

**1. Localiser** — NDT against the shared GT map, plus the EKF.
Wait for the log to settle on `Activating end` before continuing. Verify the
published robot pose lands within **0.5 m** of the marked staging pose.

**2. Identity static TF** — bridges the robot's unique map frame to `map`:
```bash
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id bunker_map
```

**3. DScovox merger FIRST** — opens the subscriber gate:
```bash
ros2 run scovox_mapping dscovox_mapping_node --ros-args -r __ns:=/bunker \
    --params-file /scovox/src/scovox_mapping/config/dscovox_params.yaml
```
`input_topics` must list **both** robots' bin topics.

**4. SCovox rolling mapper** — base config + share overlay + this robot's frame:
```bash
ros2 run scovox_mapping scovox_mapping_node --ros-args \
    -r __ns:=/bunker -r __node:=scovox_node \
    --params-file /scovox/config/scovox_fused_lidar_rgbd.yaml \
    --params-file /scovox/config/scovox_robot_share.yaml \
    -p integration_frame:=bunker_map
```

**5. Nav2** with the GT costmap, listening on `/<r>/goal_pose`, with out-of-AO
cells painted lethal.

**6. Segmentation** (bunker), **multispectral calibration panel capture**
(curt), E-stop test, then **start all recordings**.

**7. T0 — the planners.** One per robot, on that robot's own PC:
```bash
ros2 run explo_planner explo_planner_node --ros-args \
    --params-file /tmp/explo_ws/install/explo_planner/share/explo_planner/config/shared_params.yaml \
    --params-file /field/field_bunker.yaml
```
The launch-file form is convenient when no overlay is needed, but it cannot
pass the per-robot settings in §5:
```bash
ros2 launch explo_planner exploration_experiment.launch.py \
    robot:=bunker output_csv:=/tmp/RA-1_bunker.csv
```

> `multi_robot_exploration.launch.py` starts **both** planners on one host. That
> is the single-host sim topology, not the field one — it is also the only
> launch file that sets `rendezvous_expected_peers` for you. On separate robot
> PCs, launch one planner each and set that value in the overlay (§5).

The target scheduler is **not** launched at T0 — see §7.

### Startup lines that confirm a correct launch

```
EIG exploration planner ready: max_steps=200 frame=map base=bunker/base_link
    planning_map=(disabled) goal=/bunker/goal_pose
Subscribing to fused map (dscovox): /bunker/dscovox_node/scovox
Terrain-relative z ON: map z-band [-5.5, +4.0] m about the robot, …
Exploitation enabled: subscribing to tree targets on /exploration/targets/bunker
    (n_vantages=3, min_required=3, dwell=8.0s)
Proximity stop enabled: hold < 5.0 m, resume > 6.0 m, cancel via …
Proximity stop: tracking peer 'curt' via <curt robot pose topic>
```

Any of these means a per-robot setting did not take:

| Line | Meaning |
|------|---------|
| `Rendezvous inactive (coordination_enabled=1, expected_peers=0)` | `rendezvous_expected_peers` still 0 — reconnection is off. |
| `Proximity stop: no peer pose topics configured — running on the …` | Braking on 1 Hz heartbeats only. |
| `Proximity stop: N peer pose topic(s) configured but nothing …` | Topic name wrong, or the peer is not on the mesh. |
| `Coverage termination … INACTIVE` | Nothing can measure the unknown fraction; the run can only end on `max_steps`. |
| `Fused map frame_id '…' != planner map_frame 'map'` | Voxels are used unreframed and would be misplaced. |

---

## 7. Running the trial

| Step | Cue | Action |
|------|-----|--------|
| **T0** | All checks green | Launch both planners. Both enter EXPLORE. |
| **Coverage-done** | Planner logs `Step N: ROI unknown fraction 0.043 < 0.050 (source=scovox, streak 3/3)`, then `Exploration finished; idling (done_action=idle)` | Director launches the target scheduler(s). |
| **EXPLOIT** | `Target queued (1 pending) -> switching to EXPLOIT` | Robots ring each trunk: 3 vantages, 8 s dwell each. |
| **Revert** | `Target queue empty -> reverting to EXPLORE` | Back to exploration, or DONE-idle. |
| **End** | All targets closed and exploration DONE | Stop planners → robots home under RC → **stop recordings last** → `docker stop` the containers. |

Release targets with the scheduler, started at the cue rather than at T0:

```bash
ros2 run explo_planner target_scheduler_node --ros-args \
    --params-file /field/targets_phase1_bunker.yaml
```

The scheduler latches its schedule origin on its **first tick** and releases
each target at its `target_release_sec` from that origin — so for a
release-at-the-cue run, set every release time to `0.0` and let the launch
moment be the schedule. It publishes `transient_local`, so a planner that
subscribes later still receives targets already released.

`done_action: idle` is what makes this safe: the planners stay up at DONE, so
the release cannot race a shutdown.

**Type A** (explore all, then exploit) releases every target at one cue.
**Type B** (per sub-area interleaved) releases the current sub-area's list at
its own coverage-done, then moves the planners to the next sub-area.

If Type A has not cued coverage-done by 40 min, the Director releases the
targets anyway — it becomes a partial-coverage run; note it on the sheet.

---

## 8. What to watch

### Console, per robot

| Line | Reading |
|------|---------|
| `Step N: ROI unknown fraction …` | Exploration progress. The number should fall and then plateau. |
| `Step N: navigation failed [no-progress] … Blacklisted; K active …` | Occasional is normal. **Three on the same spot is a panic-stop trigger.** |
| `Proximity hold #N: yielding to 'bunker' at 4.2 m` | The pair got close and curt yielded. Expected where routes cross. |
| `Target 1: vantage 1/3 selected at (…) cost=4.31 (3 valid, 0 clear-LoS dwelled)` | Exploitation working the ring. `valid=N` counts **down** as vantages are consumed. |
| `Target 1 exploitation PARTIAL (1/3 clear-LoS vantages dwelled)` | The trunk closed without full coverage. Note it; do not abort. |
| `Rendezvous: … team incomplete (0/1) -> returning to anchor` | Comms lost. The robot is driving back, not stuck. |

### Topics

```bash
ros2 topic hz /bunker/scovox_node/scovox_bin        # ~2 Hz with the share overlay
ros2 topic hz /bunker/dscovox_node/pointcloud       # fused map flowing
ros2 topic echo /bunker/proximity_hold_state --once # latched: hold/clear + reason
ros2 topic echo /exploration/intents                # both robots' claims
```

**The single most important check** is on each merger's console, about every
5 s:

```
dscovox_diag: sources=2 src_voxels=… fused_voxels=…
```

`sources=2` on **both** robots means the pair is genuinely sharing a map.
`sources=1` means one robot is mapping alone — the trial is not measuring what
it is supposed to measure. In a real merge `src_voxels > fused_voxels` (the
maps overlap, so they are co-registered), and the two robots' `fused_voxels`
converge toward each other.

### Per-step CSV

Written to `output_csv` on each robot. Columns worth watching live:

| Column | Reading |
|--------|---------|
| `total_observed_voxels` | Should climb steadily. A flat line means the map is not growing. |
| `distance_traveled` | Climbs independently per robot. Two identical traces mean the robots are shadowing each other. |
| `coord_active_peers` | **1** during a healthy two-robot run. 0 means the teammate is not being heard. |
| `rejected_by_minpos` | Non-zero confirms deconfliction is actually firing. |
| `phase`, `target_id`, `vantage_index` | The EXPLOIT windows the offline per-trunk metric is computed over. |
| `prox_hold_count`, `prox_hold_total_sec` | Cumulative proximity holds — correlate against the trajectory afterwards. |

---

## 9. Panic stop and aborts

Anyone calls **"PANIC STOP"** → Safety Officer e-stops both robots → Director
stops the planners, then the containers (`docker stop` — **never** `pkill`).
**Recording is stopped last.**

Triggers — any one:

1. **Geofence** — any robot more than 5 m outside the AO polygon.
2. **Proximity** — robot–robot < 1.5 m closing, person within 1 m, or contact
   with a tree.
3. **Localisation divergence** — pose jump > 2 m, or the footprint visibly off
   the map. One robot diverging → stop that robot; both → full stop.
4. **Runaway navigation** — ≥ 3 blacklist cycles on the same spot, or > 3 min
   of oscillation.
5. **Platform health** — battery < 20 %, motor fault, tilt > 20°.
6. **Comms** — mesh loss to any robot > 15 s.
7. **Recording** — any bag stops growing.
8. **Environment** — person or animal in the plot, rain, or lighting outside
   the multispectral envelope.

Classify afterwards: **resumable** (brief single-robot hiccup, map intact) or
**void** (divergence, comms > 60 s, environment breach → rerun).

---

## 10. Troubleshooting

Almost every "no map" report is one of SCovox's **three gates** on the delta
stream — see the [SCovox user manual](../../scovox/docs/user_manual.md) §8.
The two-robot and planner-specific failures:

| Symptom | Cause → fix |
|---------|-------------|
| Merger stuck at `sources=1` | The peer's bin stream is not arriving. Check `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, same subnet, and that `input_topics` lists both robots. |
| `prior mismatch … dropping frame` | The robots run different SCovox base configs. Use the identical base config on both. |
| Robots' maps overwrite each other | Both mapping in the same `integration_frame` → give each its own `<r>_map` plus an identity static TF. |
| Planner sits in `WAIT_FOR_MAP` | No fused map yet. Walk it back: merger `fused_voxels > 0`? mapper `scovox_bin` flowing? localiser publishing TF at the scan stamp? |
| Planner silent, no ticks at all | `use_sim_time: true` with no `/clock` publisher — the 10 Hz timer never fires and the node idles **with no warning**. Field runs need `false`. |
| Both robots pick the same goal | `coordination_enabled` off on one, or `/exploration/intents` not crossing the mesh. Check `coord_active_peers` in the CSV — it should read 1. |
| Robots never yield to each other | `proximity_stop_enabled` off, or no peer pose topic (check the startup warning). The guard is inert without live peer data. |
| One robot finishes and stops while the other explores | `rendezvous_expected_peers` still `0`. It must be 1 on a hand-launched two-robot run. |
| Goal blacklisted at the robot's own position | `goal_xy_tolerance`/`goal_yaw_tolerance` are tighter than Nav2's goal checker. Loosen them past Nav2's values. |
| Transient nav failures become hard aborts | `goal_republish_sec` too low — the keep-alive is preempting Nav2's recovery subtree. |
| Coverage-done never fires | Unreachable columns inside the ROI box put a permanent floor under the unknown fraction. Tighten `roi_min/max_x/y` to the phase-1 box (commented in the yaml), or calibrate `done_unknown_fraction` above the measured floor. |
| Target activates, no vantage ever appears | All three rejected on line-of-sight. Check the target's radius against the taped DBH, and that `vantage_start_angle_deg: 30` still suits the row geometry — a 0° start puts the first vantage straight down a plantation row and the other two land in neighbouring canopy. |
| Trunk closes `PARTIAL` repeatedly | Vantages are being reached and blacklisted without dwelling. Check the arrival gate (row above) before suspecting the vantage geometry. |

---

## 11. Data offload

With no base station, the two per-robot bags are the **only** recordings, so
their union must be complete. The shared topics (`/exploration/intents`, each
robot's `scovox_bin` as received) appear redundantly in both bags — that
redundancy is the intended fallback if one bag is lost.

Per robot, after every run:

1. `ros2 bag info` on the bag — confirm duration and message counts before
   moving on.
2. Planner CSV (`output_csv`).
3. Multispectral SD card and the closing calibration panel capture (curt).
4. Run sheet: config git SHA, chrony offsets, lighting log, taped DBH,
   EXPLORE↔EXPLOIT switch counts, anomalies, battery state.

The exploitation-contribution metric needs **both** the planner CSV and
`/exploration/intents` carrying the exploit fields. Verify both are present in
the shakedown bag before spending a battery on a measured run.

---

## 12. Where to go deeper

| Doc | Use it for |
|-----|-----------|
| `doc/experiment_script_forest_inspection.md` (hmr_explo workspace) | Campaign design: AO, sub-areas, targets, run matrix, metrics. |
| [explo_planner README](../explo_planner/README.md) | Planner internals: EIG scoring, MinPos, rendezvous, vantage selection. |
| [`shared_params.yaml`](../explo_planner/config/shared_params.yaml) | The shared parameter file: every parameter, heavily commented with the reasoning behind each field default. |
| [SCovox user manual](../../scovox/docs/user_manual.md) | Mapping and fusion: bring-up, the three delta-stream gates, bandwidth tuning. |
| [dscovox_exploration_run.md](../explo_planner/doc/dscovox_exploration_run.md) | Bag-replay dry run of the exploration half. |
| [dscovox_exploitation_run.md](../explo_planner/doc/dscovox_exploitation_run.md) | Bag-replay dry run of the vantage ring and target queue, plus the live tree detector. |
| [limitations.md](../explo_planner/doc/limitations.md) | Known rough edges — read §3 before changing any vantage parameter on one robot only. |
