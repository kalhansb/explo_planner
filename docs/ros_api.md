# explo_planner — ROS API

Reference for the package's nodes, topics, actions, tf, parameters, messages,
launch files and metrics CSV. How the algorithms work:
[planner_method.md](planner_method.md). Running a field trial: the
[user manual](user_manual.md).

**Contents**

1. [Package summary](#1-package-summary)
2. [Architecture at a glance](#2-architecture-at-a-glance)
3. [Quick start](#3-quick-start)
4. [Nodes](#4-nodes)
   - 4.1 [explo_planner_node](#41-explo_planner_node)
   - 4.2 [target_scheduler_node](#42-target_scheduler_node)
   - 4.3 [tree_detector_node](#43-tree_detector_node)
5. [Messages (explo_planner_msgs)](#5-messages-explo_planner_msgs)
6. [Launch files](#6-launch-files)
7. [The metrics CSV](#7-the-metrics-csv)
8. [Integration notes](#8-integration-notes)

---

## 1. Package summary

Next-best-view exploration and perceptive exploitation for multi-robot forest
inspection. The planner picks viewpoints by expected information gain (EIG)
over a fused [SCovox](https://github.com/kalhansb/scovox) occupancy map and
publishes them as `geometry_msgs/PoseStamped` goals for a navigator (nav2 in
every packaged setup). A `TreeTarget` switches it to exploitation: it circles
the trunk and dwells at occlusion-free vantage points. Overview:
[README](../README.md).

- **Repository:** <https://github.com/kalhansb/explo_planner>
- **Packages:** `explo_planner` (nodes), `explo_planner_msgs` (interfaces)
- **Build type:** `ament_cmake` · **License:** BSD-3-Clause
- **Depends on:** `scovox_core`, `scovox_msgs` (map input), `nav2_msgs`
  (`NavigateToPose` watch and cancel), `tf2_ros`, standard ROS 2 packages
- **Build:** `colcon build --packages-up-to explo_planner`, not
  `--packages-select`: `explo_planner_msgs` must build first
  ([README](../README.md#build)).

---

## 2. Architecture at a glance

One planner node per robot. Each reads its own robot's fused map and talks to
peers only through the shared intent topic, `/exploration/intents`. A state
machine ticks at 10 Hz on the node clock. The data-flow diagram is in the
[package README](../explo_planner/README.md) ("Architecture"); the states are
in [planner_method.md §3](planner_method.md#3-the-planning-cycle).

> **`use_sim_time`:** the tick runs on the node clock. With
> `use_sim_time: true` and no `/clock` publisher the planner never ticks. It
> logs one warning after 10 s and otherwise does nothing. Sim and bag runs
> must publish `/clock` and pass `use_sim_time:=true`; hardware runs leave it
> `false`.

---

## 3. Quick start

Already running (contracts in [§8](#8-integration-notes)):

1. A fused map: `scovox_msgs/ScovoxMap` on `/<robot>/dscovox_node/scovox`,
   from `dscovox_mapping_node` (or `scovox_mapping_node`).
2. tf `map` → `<robot>/base_link`, from the localiser or the sim.
3. A navigator taking `geometry_msgs/PoseStamped` on `/<robot>/goal_pose`
   (nav2 `bt_navigator` in every packaged setup).

One robot, pure exploration:

```bash
ros2 launch explo_planner exploration_experiment.launch.py \
    robot:=atlas use_sim_time:=true output_csv:=/tmp/run1.csv
```

Other launch forms: [§6](#6-launch-files) and the
[package README](../explo_planner/README.md) ("Run"). Hardware:
[field_setup.md](field_setup.md) and
[user manual §6](user_manual.md#6-bring-up-order). What to watch during a run:
[user manual §8](user_manual.md#8-what-to-watch).

---

## 4. Nodes

QoS below is given as reliability, durability, depth.

### 4.1 explo_planner_node

The planner. Node name `explo_planner`. `multi_robot_exploration.launch.py`
runs it as `/<robot>/explo_planner`; the single-robot launches run it
un-namespaced. Cross-node topic defaults are absolute, built from
`robot_name`, so the namespace only moves the node's relative topics
(`~/candidates`, `proximity_hold_state`) and decides which params-file blocks
match.

#### Subscribed topics

| Topic (parameter) | Type | QoS | Notes |
|---|---|---|---|
| `/<robot_name>/dscovox_node/scovox` (`dscovox_topic`) | `scovox_msgs/ScovoxMap` | reliable, transient_local, 1 | The fused map. Used without reframing (warns once if not in `map_frame`). Planning starts once this map (with voxels in the ROI), a tf pose and, if enabled, the planning map have arrived. |
| `/exploration/intents` (`coord_intent_topic`) | `explo_planner_msgs/RobotIntent` | reliable, volatile, 8 | Peer claims: MinPos, vantage contests, dwell credit, `staged`, rendezvous presence, and a coarse peer pose for the proximity guard. Always subscribed; claims used only with `coordination_enabled`. Own messages dropped by `robot_id`. |
| `/exploration/targets` (`targets_topic`) | `explo_planner_msgs/TreeTarget` | reliable, transient_local, 50 | Tree targets. Only with `exploitation_enabled`. A late-starting planner still receives released targets. |
| `/<robot_name>/dscovox_node/planning_map` (`planning_map_topic`) | `nav_msgs/OccupancyGrid` | reliable, transient_local, 1 | Optional 2D grid for goal filtering, snapping, reachability and path cost. Only with `use_planning_map: true`; without it, costs are straight-line and obstacle avoidance is the navigator's. `direct` mode: each message is the map. `accumulate`: each message is a robot-centred window (e.g. `/occupancy_map_local`), stitched into one grid over the ROI; windows that are misframed, rotated, malformed, too large or off the grid are dropped and counted in the WAIT_FOR_MAP log. |
| each `proximity_peer_pose_topics` entry | `geometry_msgs/PoseWithCovarianceStamped` | reliable, volatile, 5 | A peer's localiser pose for the proximity guard. Only with `proximity_stop_enabled`. Used without reframing (warns once if not in `map_frame`). |
| `<nav_status_action>/_action/status` | `action_msgs/GoalStatusArray` | reliable, transient_local, 1 | nav2's goal results, for the log, the `Goal end` line, the CSV and `nav2_abort_limit`. Default action `/<robot_name>/navigate_to_pose`. |
| `<nav_status_action>/_action/feedback` | `nav2_msgs/action/NavigateToPose_FeedbackMessage` | best_effort, volatile, 1 | nav2's recoveries and distance to go, for the same uses. |

The two nav2 topics are silent without nav2 (bag replay); nothing waits on them.

#### Published topics

| Topic (parameter) | Type | QoS | Notes |
|---|---|---|---|
| `/<robot_name>/goal_pose` (`goal_topic`) | `geometry_msgs/PoseStamped` | reliable, volatile, 10 | The selected viewpoint, vantage or home goal, in `map_frame`. Re-sent every `goal_republish_sec` while unchanged. |
| `/exploration/intents` (`coord_intent_topic`) | `explo_planner_msgs/RobotIntent` | reliable, volatile, 8 | This robot's claim. Sent on selection and at `coord_heartbeat_hz` while a claim is held; carries the exploit fields during exploitation. |
| `~/candidates` | `visualization_msgs/MarkerArray` | reliable, volatile, 10 | Candidate viewpoints and scores for RViz. Resolves to `/<ns>/explo_planner/candidates`. |
| `~/traversability_map` | `nav_msgs/OccupancyGrid` | reliable, transient_local, 1 | The accumulated planning map, on each snapshot. Only in `accumulate` mode. |
| `proximity_hold_state` | `std_msgs/String` | reliable, transient_local, 1 | `clear` or the hold reason. Latched: `ros2 topic echo` shows the current state at once. Relative name. Only with `proximity_stop_enabled`. |
| `/<robot_name>/scovox_node/refinement_region` (`refinement_region_topic`) | `scovox_msgs/RefinementRegion` | reliable, transient_local, 50 | Each tree target, relayed to the mapper as a fine-TSDF region. Only with `exploitation_enabled` and `publish_refinement_regions`. |

#### Actions and services called

- `proximity_nav_cancel_action` (`nav2_msgs/action/NavigateToPose`), default
  `/<robot_name>/navigate_to_pose`. **Cancel only**: the proximity stop
  cancels all goals on hold entry. Goals go only through `goal_pose`, so any
  navigator that takes `PoseStamped` works.
- `<nav_status_action>/_action/get_result` (service, Jazzy only). Called on
  each nav2 abort for `error_code` / `error_msg`, which go to the log
  (`nav2 abort reason: …`), the `Goal end` line and the CSV. Not created on
  Humble, whose result is empty.

#### Required tf transforms

- `map_frame` → `base_frame` (default `map` → `<robot_name>/base_link`),
  every tick: the robot pose. The node publishes no tf.

#### Parameters

Defaults are the values in
[`shared_params.yaml`](../explo_planner/config/shared_params.yaml), which
every packaged launch loads; its comments give the reasoning. Its top-level
key is `/**`, so it matches the node in any namespace. **Always load it:** a
node started without it runs on the C++ fallbacks, shown as "C++ x" where they
differ. *auto* = built from `robot_name` or another parameter. Real-robot
values: [real_robot_tuning.md](real_robot_tuning.md).

**Core**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_name` | string | `atlas` | Seeds every *auto* topic, the intent `robot_id`, and right of way (lexicographic). |
| `use_sim_time` | bool | `false` | See §2. |
| `max_steps` | int | `200` | Step budget (explore + exploit), then DONE. |
| `output_csv` | string | `/tmp/exploration_eig.csv` (C++ `/tmp/exploration.csv`) | Metrics CSV (§7). The node exits at startup if it cannot open it. |
| `map_frame` | string | `map` | Frame of the fused map, goals, targets and intents. |
| `base_frame` | string | `""` | *auto* → `<robot_name>/base_link`. |
| `map_resolution` | double | `0.10` | Map-cache voxel edge (m). Match the mapper. |
| `dscovox_topic` | string | `""` | *auto* → `/<robot_name>/dscovox_node/scovox`. |
| `goal_topic` | string | *auto* | `/<robot_name>/goal_pose`. |
| `flatten_goal_z` | bool | `false` | Publish goals with z = 0, for navigators that reject a non-zero z. |
| `use_planning_map` | bool | `false` | Use the 2D planning map (see subscribed topics). |
| `planning_map_topic` | string | *auto* | `/<robot_name>/dscovox_node/planning_map`. |
| `planning_map_mode` | string | `direct` | `direct` or `accumulate` (see subscribed topics). |
| `planning_map_accum_resolution` | double | `0.2` | Accumulated grid cell (m). Finer windows are max-pooled. |
| `planning_map_accum_margin_m` | double | `5.0` | Margin around the ROI bounding box (m). An unbounded ROI falls back to ±200 m about the map origin (warns). |
| `planning_map_accum_max_cells` | int | `4000000` | Cell cap. The resolution is coarsened to fit (warns; errors above 1 m). |
| `planning_map_accum_refresh_sec` | double | `1.0` | Minimum time between snapshots (s). Each rebuilds the cost grid. |
| `planning_map_obstacle_threshold` | int | `50` | Cells ≥ this are obstacles. Clamped to 1–101. |
| `planning_map_trav_weight` | double | `0` | Path cost of a passable cell holding v: `step × (1 + w × v / 100)`. |
| `planning_map_unknown_value` | double | `0` | The v an unseen (−1) cell counts as in the path cost, clamped to 0–99. |
| `planning_map_clearance_m` | double | `0` | Passable cells this close to an obstacle (m) are never goals and cost the penalty below. |
| `planning_map_clearance_penalty` | double | `1` | Step-cost multiplier in that zone (≥ 1). |
| `candidate_snap_radius_m` | double | `2.0` (C++ `0`) | Candidates on an obstacle, in the clearance zone or unreachable move to the nearest good cell within this radius (m), else are rejected; frontier candidates may land on unknown cells. Planning map only. |
| `planning_map_robot_clear_m` | double | `0` | Blocked cells this close to the robot (m) stay obstacles for goals but not for its own path search, so a LiDAR that sees the robot's body cannot wall it in. |

**Navigation gates and watchdogs**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `goal_xy_tolerance` / `goal_yaw_tolerance` | double | `0.4` / `0.4` (C++ `0.3` / `0.2`) | Arrival gate (m / rad). Must be looser than the navigator's goal checker (nav2 ships 0.25 / 0.25), or the planner never sees arrival and blacklists a goal the robot is on. Warns below 0.3. |
| `goal_rotate_timeout_sec` | double | `15.0` | Deadline for the in-place turn after XY arrival (s). |
| `goal_republish_sec` | double | `5.0` | Re-send period for an unchanged goal (s). Each re-send preempts nav2's goal (`GoalUpdated`) and cuts its recoveries short: never set it near the tick period. `0` = send on change only. |
| `integrate_wait` | double | `2.0` | Hold in INTEGRATE after arrival so the map absorbs the new view (s). |
| `nav_speed_estimate_mps` | double | `0.15` (C++ `0.5`) | Travel budget per goal = `clamp(dist / speed × safety, min, max)`, with `dist` the straight line or, if longer, the planned path (m). |
| `nav_safety_factor` | double | `3.0` (C++ `2.0`) | ⬑ |
| `nav_min_timeout_sec` / `nav_max_timeout_sec` | double | `30` / `180` (C++ `8` / `60`) | ⬑ clamp bounds (s). |
| `progress_window_sec` / `progress_min_distance_m` | double | `15` / `0.2` (C++ `6` / `0.3`) | No-progress watchdog: fail the goal if the robot's net displacement over a window (s) is below this (m). |
| `nav_status_action` | string | *auto* | nav2 action watched for status, feedback and (Jazzy) abort reason. *auto* → `/<robot_name>/navigate_to_pose`; `""` disables. |
| `nav2_abort_limit` | int | `2` | Fail the goal (`nav2-aborted`) after this many nav2 aborts in a row within `progress_min_distance_m` (0.25 m if 0) of the first. Without it the keep-alive re-send undoes every abort. `0` = log only. |
| `nav2_abort_final_after_recovery` | bool | `true` | Also fail the goal on one abort if nav2 ran ≥ 2 recoveries (costmap clears included) within 1 m of it. An instant refusal keeps the re-send. Needs `nav2_abort_limit` > 0. |
| `far_goal_warn_m` | double | `50` | Warn when a goal is farther than this (m), likely off nav2's costmap. `0` disables. |
| `tf_stale_warn_sec` | double | `1.0` | Warn when the newest `map_frame` → `base_frame` transform is older than this (s); the last pose is kept. `≤ 0` disables. |
| `max_pose_jump_m` | double | `1.0` | A one-tick pose change above this (m) is a relocalisation jump, left out of distance and the watchdog. `0` disables. |
| `failed_goal_radius_m` / `failed_goal_ttl_sec` | double | `2.0` / `60` | Blacklist disc (m) and lifetime (s) around a failed goal. |
| `visited_goal_radius_m` / `visited_goal_ttl_sec` | double | `6` / `120` (C++ `0` / `120`) | The same for a reached exploration goal: candidates within the radius (m, XY) are skipped for the lifetime (s), counted as `blk=`. Radius `0` = off. If it is what leaves no candidate, the planner picks the one whose latest reached goal within the radius is oldest (warns `going back to the area left longest ago`; step line ends `REVISIT`; CSV `pick_fallback` = `revisit`); that pick still obeys `candidate_min_goal_dist_m`, the failed-goal blacklist and peer claims. The lifetime also applies with the radius `0`: a pick with `candidate_min_goal_dist_m` relaxed skips 2 m around reached goals. |

**Coverage-based termination**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `done_unknown_fraction` | double | `0.05` | DONE once the ROI unknown fraction stays below this. `≤ 0` disables; then only `max_steps` ends the run. |
| `done_min_consecutive_steps` | int | `3` | PLAN cycles in a row below the threshold. |
| `done_coverage_source` | string | `scovox` (C++ `auto`) | `planning_map` (2D unknown cells), `scovox` (2.5D column coverage of the fused map) or `auto` (planning map if present). The two need different thresholds. |
| `done_action` | string | `idle` (C++ `shutdown`) | `shutdown` stops the node at DONE; `idle` keeps it up for late targets. |

**Candidate generation & ROI**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `candidate_enable_polar` | bool | `false` (C++ `true`) | Add `n_radial × n_rings × n_yaw` polar samples around the robot to the frontier candidates. |
| `candidate_n_radial` / `candidate_n_rings` / `candidate_n_yaw` | int | `8` / `3` / `4` | ⬑ 96 samples at these values. |
| `candidate_min_radius` / `candidate_max_radius` | double | `2.0` / `8.0` | Polar ring radii (m). |
| `candidate_robot_z` | double | `0.3` | Sensor height (m) in flat mode. Not used for placement in terrain mode. |
| `candidate_ground_z` | double | `0.15` | Flat mode: candidates at or below this z (m) skip the occupancy check (ground voxels). |
| `candidate_occ_thresh` | double | `0.7` | Occupancy at or above this blocks a candidate. |
| `frontier_cluster_radius_m` | double | `5.0` | Frontier clustering bin edge (m). |
| `frontier_z_lo_offset_m` / `frontier_z_hi_offset_m` | double | `0` / `0` | The frontier search starts this far (m) above the bottom of the ROI z-band and stops this far below its top. Negative values count as 0; if the two cross, the whole band is searched (warns). In terrain mode the result is a flat slab, not a height above the ground. |
| `candidate_min_goal_dist_m` | double | `4.0` (C++ `0`) | Exploration candidates closer to the robot than this (m, XY) are rejected, counted as `close=`. If that leaves none, it is dropped for that pick, which still skips candidates within `visited_goal_radius_m` (at least 2 m) of a goal reached in the last `visited_goal_ttl_sec` (counted as `blk=`). Such a pick warns, its step line ends `HOP RELAXED` with counts from the relaxed walk, and its CSV row has `pick_fallback` = `hop-relaxed`. `0` = off; negative or non-finite is treated as off (warns); warns if not above `goal_xy_tolerance`. |
| `roi_min_x` / `roi_max_x` / `roi_min_y` / `roi_max_y` | double | `0` / `100` / `0` / `25` (C++ `-15` / `15` / `-15` / `15`) | XY box in the ROI frame (m). Candidates must lie in it; coverage is measured over it. |
| `roi_yaw_deg` / `roi_origin_x` / `roi_origin_y` | double | `25.75` / `-40.30` / `-31.54` (C++ `0` / `0` / `0`) | The ROI frame: the map frame rotated by `roi_yaw_deg` about map point (`roi_origin_x`, `roi_origin_y`). All zero = the map frame. |
| `roi_min_z` / `roi_max_z` | double | `-5.5` / `4.0` (C++ `-0.5` / `2.0`) | Map ingest z-band (m), robot-relative with `terrain_relative_z`. Upstream z-filters must cover it. |
| `cost_grid_radius_cap_m` | double | `0` | Path-search radius (m). *auto* → `candidate_max_radius + 2`. With an ROI-sized planning map, set it to cover the ROI, or candidates beyond 10 m read unreachable. |
| `trajectory_scoring` | bool | `false` | Ablation: score poses along the whole path, not just the endpoint. Expensive. |
| `trajectory_sample_spacing_m` | double | `1.5` | ⬑ sample spacing (m). |

**Terrain-relative mode**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `terrain_relative_z` | bool | `true` (C++ `false`) | The z-band follows the robot; candidates and vantage sightlines sit at local ground + clearance. Set `false` on flat worlds (avoids `noground` vantage rejections behind trunks). |
| `candidate_z_clearance` | double | `0.3` (C++ `0.5`) | Height above local ground (m). |
| `ground_search_below_m` / `ground_search_above_m` | double | `4.0` / `1.0` | Ground-search window about the reference z (m). The z-band must contain it plus hysteresis; warns at startup if not. |
| `ground_stack_max_m` | double | `0.6` | Tallest occupied stack still treated as ground (m). |

**FOV / sensor model**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fov_hfov` / `fov_vfov` | double | `1.047` / `0.785` | Modelled camera FOV (rad). |
| `fov_min_range` / `fov_max_range` | double | `0.3` / `10.0` | Ray range (m). `fov_max_range` is also the *auto* exploration claim radius. |
| `fov_h_rays` / `fov_v_rays` | int | `16` / `12` | Rays per axis. |
| `fov_occ_stop` | double | `0.7` | Occupancy that stops a ray; also the vantage line-of-sight (LoS) threshold. |

**Multi-robot coordination**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `coordination_enabled` | bool | `true` (C++ `false`) | MinPos goal deconfliction. `false` = plain single-robot behaviour. |
| `coord_intent_topic` | string | `/exploration/intents` | Shared claim topic, root namespace. |
| `coord_heartbeat_hz` | double | `1.0` | Claim re-publish rate while a claim is held (Hz). |
| `coord_claim_radius_m` | double | `0` | Exploration claim disc (m). *auto* → `fov_max_range`. |
| `coord_claim_ttl_sec` | double | `5.0` | Claim lifetime from **local receipt** (s), not `header.stamp`: robot clocks may differ. A silent peer's claims lapse after this. |
| `coord_claim_grace_sec` | double | `10.0` | Extra time exploit claims count past the TTL in vantage contests (s). Barrier release and rendezvous presence use the plain TTL. `0` = none. |
| `coord_vantage_claim_radius_m` | double | `0` | Exploit claim disc (m). *auto* → `vantage_visited_tol_m`, so a claim reserves one ring angle, not the whole tree. |

**Rendezvous reconnection**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `rendezvous_enabled` | bool | `true` | When exploration runs out, return to the start point (where planning began) and wait there until the team is back in comms; with the team in comms, return and finish. |
| `rendezvous_expected_peers` | int | `0` | Teammates to wait for. **`0` leaves rendezvous off** (so does `coordination_enabled: false`). The multi-robot launch and the real-robot overlay's per-robot blocks set it; otherwise set it by hand. |
| `rendezvous_max_wait_sec` | double | `600.0` (C++ `0.0`) | Give up waiting after this (s). `≤ 0` = wait forever. |
| `return_nav_max_timeout_sec` | double | `600.0` | Cap on the drive home (s). On expiry or no progress the robot waits where it is. |

**Proximity stop (coordinated yield).** Best-effort, not a certified safety
stop. Of two close robots, the one whose `robot_name` sorts first drives; the
other yields (cancel, brake-hold, resume).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `proximity_stop_enabled` | bool | `true` | Inert without peers. |
| `proximity_hold_dist_m` / `proximity_resume_dist_m` | double | `5.0` / `6.0` | Yield below / resume above (m). |
| `proximity_pose_stale_sec` | double | `3.0` | Peer data older than this (s) cannot start a hold. |
| `proximity_hold_release_stale_sec` | double | `10.0` | A hold with no peer data for this long (s) releases. |
| `proximity_peer_static_sec` / `proximity_peer_static_move_m` | double | `10.0` / `0.3` | A peer moving less than this (m) for this long (s) is parked, and not held against. Keep above `exploit_dwell_sec`. |
| `proximity_parked_keep_dist_m` | double | `1.5` | …unless it is within this (m). |
| `proximity_max_hold_sec` | double | `120.0` | Longest hold (s). `0` = no limit. |
| `proximity_peer_pose_topics` | string[] | `[]` | Peer localiser poses, as `"<robot_name>:<topic>"`. Empty = intent heartbeat only (fine in sim; warns; set it on hardware). |
| `proximity_nav_cancel_action` | string | `""` | *auto* → `/<robot_name>/navigate_to_pose`. |

**Perceptive exploitation**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `exploitation_enabled` | bool | `true` | `false` = pure exploration; the targets topic is not subscribed. |
| `targets_topic` | string | `/exploration/targets` | `TreeTarget` input. |
| `n_vantages` | int | `3` | Vantages per trunk (3 → 120° apart). ≤ 32 for shared dwell credit. Same on every robot. |
| `min_vantages_required` | int | `3` (C++ `2`) | Clear-LoS dwells (team total) for a complete target. Clamped to 1–`n_vantages`. |
| `vantage_standoff_m` | double | `2.0` | Added to the trunk radius r (m), clamped to [`fov_min_range` + r, `fov_max_range`]. |
| `vantage_start_angle_deg` | double | `30.0` (C++ `0.0`) | Angle of the first vantage (deg). Same on every robot: credit is shared as ring indices. |
| `exploit_dwell_sec` | double | `8.0` | Hold time at each vantage (s). |
| `vantage_visited_tol_m` | double | `0.75` | Distance that counts a vantage as dwelled (m); also the *auto* exploit claim disc. |
| `exploit_dwell_sync_enabled` | bool | `true` | Dwell clocks on one trunk start only once every claimant is `staged`, so captures are simultaneous. |
| `exploit_dwell_sync_max_wait_sec` | double | `0.0` | Give up that wait this long after arrival (s). `0` = wait; a dead peer's claim lapses on the TTL, so the wait cannot hang. |
| `exploit_target_timeout_sec` | double | `300.0` | Time on a trunk since its last progress (approach reached or dwell done), in s; then it closes PARTIAL. Keep above `nav_max_timeout_sec`. `≤ 0` disables. |
| `target_dedup_radius_m` | double | `1.5` (C++ `0.0`) | Re-reported trees within this (m) merge into the existing target. Same-id dedup always applies. |
| `publish_refinement_regions` | bool | `true` | Relay targets to the mapper as fine-TSDF regions. |
| `fine_region_radius_m` | double | `0.5` | Region radius added around the trunk (m). |
| `refinement_region_topic` | string | `""` | *auto* → `/<robot_name>/scovox_node/refinement_region`. |

---

### 4.2 target_scheduler_node

Time-based `TreeTarget` publisher, the stand-in for a live detector. Node
name `target_scheduler`; ticks at 2 Hz. It releases each target of a
parallel-array list at its time, counted from the first tick with a valid
clock (under `use_sim_time`, from when playback starts). `0.0` releases at
node start, e.g. on an operator's cue.

#### Published topics

| Topic (parameter) | Type | QoS | Notes |
|---|---|---|---|
| `/exploration/targets` (`targets_topic`) | `explo_planner_msgs/TreeTarget` | reliable, transient_local, 50 | One message per target at its release time, `discovered_by: "schedule"`. Late planners receive everything already released. |

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `targets_topic` | string | `/exploration/targets` | Output topic. |
| `frame_id` | string | `map` | `TreeTarget.header.frame_id`. |
| `target_ids` | int[] | `[]` | Stable ids; sets the count. The other arrays must match its length (an empty `target_height` is allowed), or the scheduler disables itself with an error. |
| `target_x` / `target_y` / `target_z` | double[] | `[]` | Trunk centres (m, in `frame_id`). |
| `target_radius` | double[] | `[]` | Trunk radii (m). `≤ 0` = point trunk (default standoff only). |
| `target_height` | double[] | `[]` | Optional trunk heights (m); zeros if omitted. |
| `target_release_sec` | double[] | `[]` | Release times from the schedule start (s). |

Shipped schedules: [`targets.yaml`](../explo_planner/config/targets.yaml),
[`targets_map_test2.yaml`](../explo_planner/config/targets_map_test2.yaml),
and `targets_flatforest.yaml` (sim world) on the `new_experiments` branch.

---

### 4.3 tree_detector_node

Live tree detector, a drop-in replacement for the scheduler. Node name
`tree_detector`. It segments trunks out of the fused map, scores how well each
has been observed (mainly angular coverage), and publishes the under-observed
ones as `TreeTarget` on the same topic. Ids come from the trunk's quantised
map position, so a tree has the same id on every robot and across
re-detections, as the planner's dedup and shared dwell credit need. Each tree
is published **once** per node lifetime, after `confirm_ticks` under-observed
scans in a row and, with bearing coverage, a settled bearing history.

#### Subscribed topics

| Topic (parameter) | Type | QoS | Notes |
|---|---|---|---|
| `/<robot_name>/dscovox_node/scovox` (`dscovox_topic`) | `scovox_msgs/ScovoxMap` | reliable, transient_local, 1 | The fused map, scanned every `scan_period_sec`. |

#### Published topics

| Topic (parameter) | Type | QoS | Notes |
|---|---|---|---|
| `/exploration/targets` (`targets_topic`) | `explo_planner_msgs/TreeTarget` | reliable, transient_local, `targets_qos_depth` | `discovered_by: <robot_name>`. |

#### Required tf transforms

- `frame_id` → `base_frame` (default `map` → `base_link`): the robot pose
  for viewing bearings, with `use_bearing_coverage`. Optional: without it the
  score falls back to map geometry (throttled warning).

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_name` | string | `robot1` | Seeds the default map topic and `discovered_by`. |
| `dscovox_topic` | string | `""` | *auto* → `/<robot_name>/dscovox_node/scovox`. |
| `targets_topic` | string | `/exploration/targets` | Output topic. |
| `frame_id` | string | `map` | Output frame. |
| `scan_period_sec` | double | `2.0` | Time between scans (s). |
| `confirm_ticks` | int | `2` | Under-observed scans in a row before a tree is published. |
| `track_match_radius_m` | double | `1.5` | XY radius matching a detection to a track (m). |
| `track_timeout_sec` | double | `30.0` | Drop a track unseen this long (s). Published trees are remembered and never re-published. |
| `id_cell_m` | double | `1.0` | XY grid the position-derived id snaps to (m). |
| `use_bearing_coverage` | bool | `true` | Score angular coverage from the robot's viewing bearings, not the trunk's voxel spread (which reads full on a half-seen trunk). |
| `base_frame` | string | `base_link` | Robot frame for the bearing pose. |
| `bearing_settle_ticks` | int | `3` | Scans the bearing history must stay unchanged before publishing. `0` disables. |
| `targets_qos_depth` | int | `500` | Latched history depth. Must exceed the targets a run can publish. |
| `use_semantics` | bool | `true` | `false` = geometric mode. **Required** for LiDAR-only maps with empty semantic records: the semantic gate silently drops every unlabelled voxel. |
| *segmentation and scoring* | — | — | `voxel_size`, `veg_class`, `occ_thresh`, `min_class_conf`, `cluster_tol_m`, `trunk_band_lo/hi`, `min_trunk_voxels`, `min_height`, `max_radius`, `n_azimuth_bins`, `n_height_bins`, `w_coverage`, `w_entropy`, `w_vertical`, `deficit_thresh`, and in geometric mode `terrain_cell_m`, `ground_margin_m`, `stem_slice_lo/hi`, `attach_radius_m`, `min_linearity`, `max_tilt_deg`. Defaults are in [`tree_detector.hpp`](../explo_planner/include/explo_planner/tree_detector.hpp). |

---

## 5. Messages (explo_planner_msgs)

### TreeTarget

The interface between target producers and exploitation. The scheduler and
the detector publish the same message.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` = release or detection time; `frame_id` = frame of `center`. |
| `target_id` | `uint32` | Stable id. The planner keeps the first message per id and ignores repeats. |
| `center` | `geometry_msgs/Point` | Trunk axis (x, y) and base z. |
| `radius` | `float32` | Trunk radius (m). `≤ 0` = point trunk. |
| `height` | `float32` | Optional trunk height (m), informational. |
| `discovered_by` | `string` | `"schedule"` or a robot id. Diagnostic. |
| `status` | `uint8` | `STATUS_PENDING` = 0, `STATUS_DONE` = 1 (reserved for a completion broadcast). |

### RobotIntent

The multi-robot claim, published on `coord_intent_topic` by every planner.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` is diagnostic only: expiry counts from **local receipt** (`ttl_sec`), so robot clocks need not agree. |
| `robot_id` | `string` | Producer. Peers drop their own messages; tie-break key. |
| `goal_pos` / `yaw` | `Point` / `float32` | Claimed viewpoint. |
| `robot_pos` | `Point` | Producer's position when sent. Peers recompute the MinPos distance contest from it. |
| `claim_radius_m` | `float32` | Reserved disc around `goal_pos` (m). |
| `ttl_sec` | `float32` | Claim lifetime from receipt (s). |
| `planner_type` | `uint8` | Diagnostic (0 = eig). |
| `exploit` / `target_id` | `bool` / `uint32` | The claim is an exploit hop on that trunk. Test `exploit`, not `target_id`: a detector may emit id 0. |
| `dwelled_mask` | `uint32` | Ring indices the producer has dwelled at with clear LoS. Peers OR it into their state; cumulative, so a lost message loses no credit. |
| `staged` | `bool` | The producer is standing on `goal_pos` in its dwell state (set on dwell entry, after arrival and the turn settle). The dwell-sync barrier waits on it. |

**Wire compatibility.** Every robot must run the same interface build. An
older peer never sets `staged` (its teammates wait until the give-up path)
and lacks the exploit fields (team quota falls back to solo). A checksum
mismatch fails loudly in the planner and in `ros2 topic echo`, but a stale
installed `explo_planner_msgs` beside new planner code does not: rebuild with
`--packages-up-to`.

---

## 6. Launch files

The three planner launches load `config/shared_params.yaml` first and pass
their arguments on top; the argument defaults win for those keys. They do not
start the simulator, mapper, navigator or tf (see §3). The packaged two-robot
sim world is `sim/run_explo_sim_rviz.sh` on the `new_experiments` branch.

### exploration_experiment.launch.py

One planner, no target producer. Exploitation follows the yaml; set
`exploitation_enabled: false` there for strictly pure exploration.

| Argument | Default | Description |
|---|---|---|
| `robot` | `atlas` | Robot name. |
| `use_sim_time` | `false` | Sim and bag runs must pass `true`. |
| `max_steps` | `200` | Step budget. |
| `output_csv` | `/tmp/exploration.csv` | Metrics CSV. |
| `dscovox_topic` | `''` | Map topic override. |
| `map_frame` / `base_frame` | `map` / `''` | Frame overrides. |
| `trajectory_scoring` | `false` | SSMI ablation. Always passed, so it wins over the yaml. |
| `trajectory_sample_spacing_m` | `1.5` | ⬑ |
| `params_file` | `shared_params.yaml` | Extra parameter file layered over the shared file. The arguments above still win for their keys. |

### exploitation_experiment.launch.py

One planner plus a target producer: the time-based scheduler, or the live
detector with `use_detector:=true`.

| Argument | Default | Description |
|---|---|---|
| `robot` | `atlas` | Robot name. |
| `use_sim_time` | `false` | Also the scheduler's release clock. |
| `max_steps` | `200` | Step budget (explore + exploit). |
| `output_csv` | `/tmp/exploitation.csv` | Metrics CSV. |
| `targets_file` | `config/targets.yaml` | Scheduler parameter file. |
| `use_detector` | `false` | `true` = run `tree_detector_node` instead of the scheduler. |
| `use_semantics` | `true` | Detector gating; `false` on LiDAR-only maps. |
| `terrain_relative_z` | `true` | `false` on flat sim worlds (see the `noground` note in the launch file). |

### multi_robot_exploration.launch.py

One planner per robot on one host (the sim topology). On hardware, start one
planner per robot PC by hand. This is the only launch that sets
`rendezvous_expected_peers` for you (team size − 1).

| Argument | Default | Description |
|---|---|---|
| `robots` | `atlas,rama` | Comma-separated team. Each planner runs in namespace `/<robot>`. |
| `use_sim_time` | `false` | |
| `planner` | `eig` | CSV filename label only; the node is EIG-only. |
| `output_dir` / `config_id` / `world` | `/tmp` / `c1` / `flatforest` | Build the per-robot CSV name `exp7_<planner>_<world>_<config_id>_<robot>.csv`. |
| `max_steps` | `100` | Per-robot step budget. |
| `coordination_enabled` | `true` | MinPos deconfliction. |
| `rendezvous_enabled` | `true` | Return-home reconnection. |
| `rendezvous_max_wait_sec` | `600.0` | Rendezvous give-up (s). |
| `proximity_stop_enabled` | `true` | Coordinated yield. |
| `params_file` | `''` | Overlay layered over the shared file. On hardware use [`config/exploration_real_robot.yaml`](../explo_planner/config/exploration_real_robot.yaml), whose `/<robot>/explo_planner` blocks match these namespaced nodes; see [real_robot_tuning.md](real_robot_tuning.md). |

### traversability_map.launch.py

Starts traversability_mapping's `traversability_filter` and
`traversability_map` only (not its PRM/path planner): the planning-map source
of the real-robot overlay. One instance per robot. Setup:
[field_setup.md](field_setup.md), "Traversability map".

| Argument | Default | Description |
|---|---|---|
| `namespace` | `''` | Robot namespace. The nodes run in it, and every `*_topic` in the params file except `input_cloud_topic` gets a `/<namespace>` prefix, so the map lands on `/<namespace>/occupancy_map_local`. Empty = the params file's un-namespaced topics. |
| `input_cloud_topic` | `/hesai/points_decimated` | LiDAR `PointCloud2`, used as given. |
| `base_frame` / `lidar_frame` | `''` | Frame overrides; empty = the params file (`base_link` / `hesai_lidar`). Frames are never prefixed: the robots share one tf tree. |
| `use_sim_time` | `false` | `true` for bag replay. |
| `params_file` | `''` | traversability_mapping params; empty = its shipped `config/traversability_mapping.yaml`. |
| `sensor_range_limit` / `prediction_enable` | `''` | Overrides; empty = the params file. Floats keep their decimal point (`12.0`). |

### nav2_namespaced.launch.py

nav2_bringup's `navigation_launch.py` in the robot's namespace, with tf kept
on `/tf` and `/tf_static`. The params file is used as it is, except that
`/occupancy_map_local`, `/pointcloud_2_laserscan` and `/cmd_vel_smoothed`
are rewritten to their namespaced names (in a copy; the launch file header
lists the details). Needs Jazzy or later. What the planner needs from nav2:
[nav2_config.md](nav2_config.md).

| Argument | Default | Description |
|---|---|---|
| `namespace` | (required) | Robot namespace, e.g. `bunker`. |
| `params_file` | (required) | The robot's nav2 params file, left unchanged. |
| `use_sim_time` | `false` | `true` for bag replay. |
| `autostart` | `true` | Bring the nav2 lifecycle nodes up at once. |

---

## 7. The metrics CSV

One row per LOG_STEP (each completed explore step, exploit hop or dwell),
written to `output_csv`. Columns, in order:

| Column | Meaning |
|---|---|
| `step`, `sim_time_sec` | Step counter and node-clock time (s). |
| `total_observed_voxels`, `frontier_voxels` | Map growth. |
| `distance_traveled` | Integrated from tf (m), with jumps above `max_pose_jump_m` left out. Pose jitter still adds to it on a parked robot. |
| `selected_score`, `plan_time_ms`, `mean_eig`, `mean_entropy`, `mean_variance` | Selection diagnostics. |
| `mean_info_gain`, `mean_path_cost`, `selected_info_gain`, `selected_path_cost`, `selected_utility` | Utility terms, `U = gain / (0.1 + cost)`. With a planning map, path cost is weighted, not metres. |
| `coord_active_peers`, `rejected_by_minpos`, `rejected_by_unreachable` | Coordination diagnostics. `coord_active_peers` reads team size − 1 in a healthy run. |
| `phase`, `target_id`, `vantage_index` | `explore` or `exploit`; active target and dwelled ring index (`-1` in explore). |
| `n_vantages_valid`, `vantage_los_clear`, `dwell_sec` | Selectable vantages this tick; clear-LoS flag; capture time (s), not counting dwell-sync waiting. |
| `prox_hold_count`, `prox_hold_total_sec` | Cumulative proximity holds and their total time (s). Subtract consecutive rows for per-step values. |
| `goal_outcome` | How the goal that ended since the previous row ended: `reached`, `budget`, `budget-rotate`, `no-progress`, `nav2-aborted`, `goal-in-obstacle` (its cell became an obstacle), `target-switch`, `yielded` (a teammate picked the same vantage at the same moment), `team-reconnected` (drive home cut short). Empty if none ended. Ends that write no row of their own (the last four, a reached approach waypoint, and every drive home) are joined to the next row's with `+`, e.g. `goal-in-obstacle+budget`; the next five columns describe the last of them. Ends still pending when the run stops get one last row at DONE. |
| `nav_elapsed_sec`, `goal_path_length_m` | Time the goal took (s), and the distance its budget was built from (m): planned path length, or the straight line. |
| `nav2_aborts`, `nav2_recoveries`, `nav2_error_code` | nav2's aborts of this goal, the recoveries it ran on it, and the last abort's error code (Jazzy; `-1` = none, or Humble). `0` / `0` / `-1` when nav2 is not watched. |
| `pick_fallback` | How the goal was picked when the normal walk left no candidate: `hop-relaxed` (`candidate_min_goal_dist_m` dropped) or `revisit` (every candidate near a recently reached goal; the area left longest ago). Empty for a normal pick and in exploit. |

---

## 8. Integration notes

**Your own navigator.** The command interface is one topic: `PoseStamped` on
`goal_topic`, in `map_frame`, yaw used. The planner judges arrival itself from
tf against `goal_xy_tolerance` / `goal_yaw_tolerance`, so the navigator's own
tolerance must be tighter. It reads nav2's action status and feedback when
they exist (`nav_status_action`) but waits on neither, so a goal published
while the navigator is down is lost silently; the `goal_republish_sec`
re-send covers that. Only the proximity stop's cancel is nav2-specific
(`NavigateToPose` cancel-all). With another navigator, disable
`proximity_stop_enabled` or accept that a hold brakes only by publishing a
goal at the robot's own pose. What the planner needs from nav2:
[nav2_config.md](nav2_config.md).

**Your own target producer.** Publish `TreeTarget` on `targets_topic` with
reliable, transient_local QoS and stable ids. That is the whole contract: the
planner cannot tell the scheduler from the detector.

**Your own map.** The planner takes `scovox_msgs/ScovoxMap` only. Its frame
must be `map_frame`; another frame is used unreframed, with a warning. Publish
it transient_local so a restarted planner gets the map at once.

**Multi-robot invariants.**

- Same on every robot: `n_vantages`, `vantage_start_angle_deg` (credit travels
  as ring indices), `coord_intent_topic`, the interface build (§5), and the
  map prior upstream.
- Different per robot: `robot_name`, `output_csv`, the per-robot map topics.
  `targets_topic` may also differ, to give each robot its own target list.
- `rendezvous_expected_peers` = team size − 1. The multi-robot launch and
  the per-robot blocks of `exploration_real_robot.yaml` set it; otherwise set
  it by hand.

**Further reading:** [planner_method.md](planner_method.md) (algorithms and
coordination) · [user manual](user_manual.md) (field operations) ·
[`shared_params.yaml`](../explo_planner/config/shared_params.yaml) (every
parameter, commented) ·
[limitations.md](../explo_planner/doc/limitations.md) (known rough edges) ·
bag-replay dry runs: [exploration](../explo_planner/doc/dscovox_exploration_run.md),
[exploitation](../explo_planner/doc/dscovox_exploitation_run.md).
