# explo_planner — ROS API

Package reference for ROS 2 developers, in the classic ROS-wiki page layout:
what each node subscribes to, publishes, and expects from tf; every parameter
with its shipped default; the message contracts; and the launch files. Read
this to *integrate or drive* the planner. For how the algorithms work read
[planner_method.md](planner_method.md); for running a field trial read the
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

Next-Best-View **exploration** and perceptive **exploitation** for multi-robot
forest inspection. The planner selects viewpoints by expected information gain
(EIG) over a fused [SCovox](https://github.com/kalhansb/scovox) Beta-conjugate
occupancy map, publishes them as `geometry_msgs/PoseStamped` goals for a
downstream navigator (Nav2 in every packaged setup), and logs per-step metrics
to CSV. When a `TreeTarget` arrives it switches to exploitation: it circles the
trunk at occlusion-free vantage points and dwells at each so the rosbag
captures overlapping close-range views. Multiple robots deconflict goals
through a MinPos intent table on one shared topic, split a trunk's vantage
ring between them, dwell simultaneously, yield to each other when paths cross,
and reconnection manoeuvres (rendezvous / pursuit / hybrid) when comms drop.

- **Repository:** <https://github.com/kalhansb/explo_planner>
- **Packages:** `explo_planner` (nodes), `explo_planner_msgs` (interfaces)
- **Build type:** `ament_cmake` · **License:** BSD-3-Clause
- **Depends on:** `scovox_core`, `scovox_msgs` (map input), `nav2_msgs`
  (cancel action only), `tf2_ros`, standard ROS 2 (`rclcpp`, `nav_msgs`,
  `geometry_msgs`, `visualization_msgs`)

```bash
cd <ws>
colcon build --packages-up-to explo_planner   # NOT --packages-select:
source install/setup.bash                     # explo_planner_msgs must build first
```

---

## 2. Architecture at a glance

One planner node per robot. Each planner consumes its **own robot's** fused
map and talks to peers only through the shared intent topic.

```
                         (per robot)
  dscovox merger ──ScovoxMap──▶ ┌───────────────────┐ ──PoseStamped──▶ Nav2
  target producer ──TreeTarget▶ │ explo_planner_node │ ──MarkerArray──▶ RViz
  tf: map → base_link ────────▶ │  (10 Hz tick)      │ ──CSV──▶ output_csv
                                └─────────┬─────────┘
                                          │ RobotIntent (pub + sub)
                             /exploration/intents  ◀── every peer planner
```

Internal state machine, ticked at 10 Hz on the node clock:

```
WAIT_FOR_MAP → PLAN → NAVIGATE → INTEGRATE → LOG_STEP → PLAN … → DONE
                         │  ▲
                 PROXIMITY_HOLD           (yield while a peer drives past)
exploit sub-loop:  EXPLOIT_PLAN → NAVIGATE → EXPLOIT_DWELL → LOG_STEP → …
rendezvous:        RETURN_NAV → RETURN_SYNC  (drive to the anchor/meeting point, hold for the team)
pursuit:           PURSUE → RETURN_NAV / RETURN_SYNC  (chase the missing peer's trail, then fall back per reconnect_mode)
```

A `TreeTarget` on the targets topic pulls the planner from the exploration
loop into the exploitation sub-loop; it reverts when the target queue drains.
`done_action` decides whether `DONE` shuts the node down or idles it (idle
keeps it receptive to late targets — the field default).

> **The `use_sim_time` trap:** the 10 Hz tick runs off the node clock. With
> `use_sim_time: true` and no `/clock` publisher the timer never fires and the
> node idles **silently**. Sim/bag runs must publish `/clock` and pass
> `use_sim_time:=true`; hardware runs must leave it `false`.

---

## 3. Quick start

What must already be running (see [§8](#8-integration-notes) for the contracts):

1. A **fused map publisher** — `dscovox_mapping_node` (or `scovox_mapping_node`)
   publishing `scovox_msgs/ScovoxMap` on `/<robot>/dscovox_node/scovox`.
2. **tf**: `map` → `<robot>/base_link` (the localiser or sim glue node).
3. A **navigator** consuming `geometry_msgs/PoseStamped` on
   `/<robot>/goal_pose` (Nav2 `bt_navigator` in every packaged setup).

Pure exploration, one robot:

```bash
ros2 launch explo_planner exploration_experiment.launch.py \
    robot:=atlas use_sim_time:=true output_csv:=/tmp/run1.csv
```

Exploration + exploitation with the time-based target scheduler:

```bash
ros2 launch explo_planner exploitation_experiment.launch.py \
    robot:=atlas use_sim_time:=true \
    targets_file:=$(ros2 pkg prefix explo_planner)/share/explo_planner/config/targets.yaml
```

Two coordinated robots on one host (sim topology):

```bash
ros2 launch explo_planner multi_robot_exploration.launch.py \
    robots:=atlas,rama use_sim_time:=true output_dir:=/tmp/exp7
```

Hand-run with a per-robot overlay (the hardware form — the launch files cannot
pass per-robot settings like `proximity_peer_pose_topics`):

```bash
ros2 run explo_planner explo_planner_node --ros-args \
    --params-file $(ros2 pkg prefix explo_planner)/share/explo_planner/config/shared_params.yaml \
    --params-file /field/field_curt.yaml
```

Watching a run:

```bash
ros2 topic echo /exploration/intents                     # every robot's claims
ros2 topic echo /proximity_hold_state --qos-durability transient_local \
    --qos-reliability reliable --once                    # latched hold state
tail -f /tmp/run1.csv                                    # per-step metrics
```

> **Always load `shared_params.yaml`** (every packaged launch does). The bare
> C++ fallback defaults differ from the shipped file on several parameters
> (ROI box, nav speed estimate, `min_vantages_required`,
> `vantage_start_angle_deg`, `target_dedup_radius_m`); the tables below list
> the **shipped file's** values. The file's top-level key is `/**`, so it
> matches the node in any namespace.

---

## 4. Nodes

### 4.1 explo_planner_node

The planner. Node name `explo_planner`; namespaced `/<robot>/explo_planner` by
`multi_robot_exploration.launch.py`, un-namespaced by the single-robot
launches (all cross-node topic defaults are built **absolute** from
`robot_name`, so namespacing is cosmetic).

#### Subscribed topics

- `/<robot_name>/dscovox_node/scovox` (`scovox_msgs/ScovoxMap`) —
  the fused occupancy map the planner plans over. Override with
  `dscovox_topic`. QoS: reliable, `transient_local`, depth 1 (the current map
  is delivered immediately on connect; `WAIT_FOR_MAP` ends at the first
  message whose frame matches `map_frame`).
- `<coord_intent_topic>` (`explo_planner_msgs/RobotIntent`), default
  `/exploration/intents` — peer claims for MinPos deconfliction, exploit
  vantage contests, dwell-credit union, the staged flag, rendezvous presence
  and the proximity guard's coarse peer pose. Always subscribed; payload only
  consumed when `coordination_enabled`. Self-messages are dropped by
  `robot_id`. QoS: reliable, depth 8.
- `<targets_topic>` (`explo_planner_msgs/TreeTarget`), default
  `/exploration/targets` — tree targets to exploit. Only subscribed when
  `exploitation_enabled`. QoS: reliable, `transient_local`, depth 50, so a
  planner that starts after targets were released still receives them.
- `<planning_map_topic>` (`nav_msgs/OccupancyGrid`), default
  `/<robot_name>/dscovox_node/planning_map` — optional 2D inflated grid for
  candidate free-cell filtering, reachability and path costs. **Only
  subscribed when `use_planning_map: true`** (default off: straight-line
  costs, no 2D filtering, obstacle avoidance wholly delegated to the
  navigator). QoS: reliable, `transient_local`, depth 1.
- Each entry of `proximity_peer_pose_topics`
  (`geometry_msgs/PoseWithCovarianceStamped`) — a peer's localiser pose for
  the proximity guard, format `"<robot_name>:<topic>"`. Consumed in
  `map_frame` **without reframing** (warns once on mismatch). QoS: reliable,
  depth 5.

#### Published topics

- `/<robot_name>/goal_pose` (`geometry_msgs/PoseStamped`) — the selected
  viewpoint / vantage / rendezvous goal, in `map_frame`. Override with
  `goal_topic`. Republished every `goal_republish_sec` while unchanged (see
  that parameter's footgun note). QoS: volatile, depth 10.
- `<coord_intent_topic>` (`explo_planner_msgs/RobotIntent`) — this robot's
  claim, published on selection and re-published at `coord_heartbeat_hz`
  (1 Hz) while the goal is active, carrying `exploit`/`target_id`/
  `dwelled_mask`/`staged` during exploitation.
- `~/candidates` (`visualization_msgs/MarkerArray`) — candidate viewpoints and
  scores for RViz; resolves to `/<ns>/explo_planner/candidates`.
- `proximity_hold_state` (`std_msgs/String`) — `"clear"` or a hold reason;
  only when `proximity_stop_enabled`. Relative name: resolves under the node's
  namespace. QoS: reliable, `transient_local`, depth 1 (latched — `echo` shows
  the current state immediately).
- `/<robot_name>/scovox_node/refinement_region`
  (`scovox_msgs/RefinementRegion`) — relays each ingested tree target as a
  fine-TSDF refinement region for the mapper; only when
  `exploitation_enabled` and `publish_refinement_regions`. Override with
  `refinement_region_topic`. QoS: reliable, `transient_local`, depth 50.

#### Actions called

- `<proximity_nav_cancel_action>` (`nav2_msgs/action/NavigateToPose`), default
  `/<robot_name>/navigate_to_pose` — **cancel-only**. Used exclusively by the
  proximity stop to `async_cancel_all_goals()` on hold entry; goals are never
  *sent* through this client. The `goal_pose` topic remains the only command
  path, so any navigator that consumes `PoseStamped` works — only the
  proximity stop's cancel is Nav2-specific.

#### Required tf transforms

- `map_frame` → `base_frame` (default `map` → `<robot_name>/base_link`) —
  looked up every 10 Hz tick for the robot pose; drives arrival detection,
  distance integration and the no-progress watchdog. No tf is published.

#### Parameters

Defaults below are the shipped
[`shared_params.yaml`](../explo_planner/config/shared_params.yaml) values
(that file documents the *reasoning* behind each; this table is the index).
Topic-name parameters marked *auto* build their default from `robot_name`.

**Core**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_name` | string | `atlas` | Seeds every *auto* topic default, the intent `robot_id`, and right-of-way (lexicographic order). |
| `use_sim_time` | bool | `false` | See the trap in §2. |
| `max_steps` | int | `200` | Step budget (explore + exploit) before forced DONE. |
| `output_csv` | string | `/tmp/exploration_eig.csv` | Per-step metrics CSV (§7). Node aborts at startup if unwritable. |
| `map_frame` | string | `map` | Frame of the fused map, goals, targets and intents. |
| `base_frame` | string | `""` | *auto* → `<robot_name>/base_link`. |
| `map_resolution` | double | `0.10` | Voxel edge (m) of the local map cache; match the mapper. |
| `dscovox_topic` | string | `""` | *auto* → `/<robot_name>/dscovox_node/scovox`. |
| `goal_topic` | string | *auto* | `/<robot_name>/goal_pose`. |
| `use_planning_map` | bool | `false` | Master switch for the 2D grid (see subscribed topics). |
| `planning_map_topic` | string | *auto* | `/<robot_name>/dscovox_node/planning_map`. |

**Navigation gates and watchdogs**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `goal_xy_tolerance` / `goal_yaw_tolerance` | double | `0.4` / `0.4` | Arrival gate (m / rad). Must be strictly **looser** than the navigator's own goal checker (Nav2 ships 0.25/0.25), or the planner never registers arrival and blacklists a goal the robot is standing on. Warns at startup if not. |
| `goal_rotate_timeout_sec` | double | `15.0` | Deadline for the post-arrival in-place rotation. |
| `goal_republish_sec` | double | `5.0` | Keep-alive re-send of an unchanged goal. Each re-send preempts Nav2's running goal via `GoalUpdated`, cutting recoveries short — never set it near the tick rate; `0` = publish-on-change only (best once bringup is reliable). |
| `integrate_wait` | double | `2.0` | Dwell (s) in INTEGRATE after arrival so the map absorbs the new view. |
| `nav_speed_estimate_mps` | double | `0.15` | Travel budget per NAVIGATE = `clamp(dist / speed × safety, min, max)`. |
| `nav_safety_factor` | double | `3.0` | ⬑ |
| `nav_min_timeout_sec` / `nav_max_timeout_sec` | double | `30` / `180` | ⬑ clamp bounds (s). |
| `progress_window_sec` / `progress_min_distance_m` | double | `15` / `0.2` | No-progress watchdog: fail the goal if the robot travels less than this in this window. |
| `max_pose_jump_m` | double | `1.0` | Single-tick pose delta above this is a relocalisation jump, not travel (excluded from distance and the watchdog). `0` disables. |
| `pose_max_age_sec` | double | `5.0` | Frozen-TF guard: tf2 serves the *newest stored* transform forever, so a dead localiser keeps the lookup succeeding with a stale pose. A transform older than this reads as "pose lost" until TF resumes. `0` disables. |
| `failed_goal_radius_m` / `failed_goal_ttl_sec` | double | `2.0` / `60` | Blacklist disc and lifetime around a failed goal. |

**Coverage-based termination**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `done_unknown_fraction` | double | `0.05` | DONE once the ROI unknown fraction stays below this. `<= 0` disables (only `max_steps` stops the run). |
| `done_min_consecutive_steps` | int | `3` | Consecutive PLAN cycles below the threshold required. |
| `done_coverage_source` | string | `scovox` | Where the fraction is measured: `planning_map` (2D unknown cells), `scovox` (2.5D column coverage of the fused 3D map), `auto`. The two measures need different calibrations. |
| `done_action` | string | `idle` | `shutdown` stops the node at DONE; `idle` keeps it alive and receptive to late targets. |

**Candidate generation & ROI**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `candidate_enable_polar` | bool | `true` | Adds `n_radial × n_rings × n_yaw` polar samples around the robot on top of frontier candidates. |
| `candidate_n_radial` / `candidate_n_rings` / `candidate_n_yaw` | int | `8` / `3` / `4` | ⬑ 96 samples at the defaults. |
| `candidate_min_radius` / `candidate_max_radius` | double | `2.0` / `8.0` | Polar ring radii (m). |
| `candidate_robot_z` | double | `0.3` | Sensor height (m) — absolute in flat mode, ignored for placement in terrain mode. |
| `candidate_occ_thresh` | double | `0.7` | Occupancy above this blocks a candidate cell. |
| `frontier_cluster_radius_m` | double | `5.0` | Frontier-centroid clustering bin edge (m). |
| `roi_min_x/max_x/min_y/max_y` | double | AO box | Axis-aligned XY box (map frame) candidates must lie in; also the coverage-done measurement box. |
| `roi_min_z` / `roi_max_z` | double | `-5.5` / `4.0` | Map ingest z-band. **Robot-relative** when `terrain_relative_z`, absolute otherwise. Any upstream share/ingest z-filters must be a superset. |
| `cost_grid_radius_cap_m` | double | `0` | Bounded-Dijkstra flood radius. *auto* → `candidate_max_radius + 2`. |
| `utility_cost_exponent` | double | `1.0` | γ in `U = EIG / (ε + cost)^γ`: `1` = pure info-per-metre rate, `< 1` discounts distance (favours richer-but-farther candidates), `0` ignores cost. Clamped to [0, 2]; non-finite falls back to `1`. **`shared_params.yaml` ships `0.5`** — the C++ default and the shipped config diverge on purpose (γ is an experiment knob), so state which one a run used. |
| `trajectory_scoring` | bool | `false` | SSMI ablation: score poses sampled along the whole Dijkstra path, not just the endpoint. Expensive. |
| `trajectory_sample_spacing_m` | double | `1.5` | ⬑ sample spacing. |

**Terrain-relative mode**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `terrain_relative_z` | bool | `true` | z-band rides with the robot; candidates and vantage sightlines snap to local ground + clearance. Set `false` on flat worlds (avoids `noground` vantage rejection behind trunks). |
| `candidate_z_clearance` | double | `0.3` | Height above local ground (m). |
| `ground_search_below_m` / `ground_search_above_m` | double | `4.0` / `1.0` | Ground-search window about the reference z. The z-band must contain it plus hysteresis — see the invariant note in the yaml. |
| `ground_stack_max_m` | double | `0.6` | Max occupied stack still treated as ground. |

**FOV / sensor model**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fov_hfov` / `fov_vfov` | double | `1.047` / `0.785` | Modelled camera FOV (rad). |
| `fov_min_range` / `fov_max_range` | double | `0.3` / `10.0` | Ray range (m); `fov_max_range` is also the *auto* exploration claim radius. |
| `fov_h_rays` / `fov_v_rays` | int | `16` / `12` | Raycast resolution. |
| `fov_occ_stop` | double | `0.7` | Occupancy that stops a ray (also the vantage LoS threshold). |

**Multi-robot coordination**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `coordination_enabled` | bool | `true` | Master switch. `false` skips the MinPos branch entirely; single-robot behaviour is bit-for-bit preserved. |
| `coord_intent_topic` | string | `/exploration/intents` | Shared, root-namespace claim topic. |
| `coord_heartbeat_hz` | double | `1.0` | Claim re-publish rate while a goal is active. |
| `coord_claim_radius_m` | double | `0` | Exploration claim disc. *auto* → `fov_max_range`. |
| `coord_claim_ttl_sec` | double | `5.0` | Claim lifetime since **local receipt** (never `header.stamp` — fleet clocks may be unsynchronised). A silent peer releases its targets in ~5 s. |
| `coord_claim_grace_sec` | double | `10.0` | Extra retention of **exploit** claims past TTL, used only by vantage-contest lookups (survives receive-side executor starvation). Barrier release and rendezvous presence stay on the raw TTL. `0` = legacy single-TTL. |
| `coord_vantage_claim_radius_m` | double | `0` | Exploit claim disc. *auto* → `vantage_visited_tol_m`, so a claim reserves one ring angle, not the whole tree. |

**Rendezvous reconnection**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `rendezvous_enabled` | bool | `true` | On goal exhaustion with teammates out of comms, run the `reconnect_mode` manoeuvre. |
| `rendezvous_expected_peers` | int | `0` | Teammates to wait for. **`0` leaves the feature inert** — the multi-robot launch sets team size − 1; set it by hand on hardware. |
| `rendezvous_max_wait_sec` | double | `0.0` | **Terminal** barrier give-up (s); `0` = wait forever, which is the shipped field value. On expiry the robot escalates once to the meeting point (`hold_escalate`) and then finishes in `DONE` with the run logged `outcome=gave_up`. Mid-run barriers ignore this and use `reconnect_midrun_max_wait_sec` instead. The sim harness ships `600`. |
| `reconnect_mode` | string | `rendezvous` | Mesh (robot-carried radio) manoeuvre: `rendezvous` = drive to the deterministic meeting point and wait (it fell back to this robot's own last-contact anchor until 2026-08-17; with radios on the robots both anchors sit one comms range apart, so the two never converged — the own anchor is now used only when the missing peer was never heard this run); `pursuit` = budgeted chase of the missing peer's last declared goal, then hold in place; `hybrid` = chase, then the deterministic meeting point (midpoint of the last-contact pose pair). Case-insensitive; the yaml/sim ship `hybrid`. Designed for the 2-robot team (like the MinPos tiebreak): with 3+ robots the chase/midpoint pairs one missing peer at a time. Needs `done_action: idle` — a finished robot keeps beaconing so a later finisher can count it; `shutdown` makes the first finisher permanently invisible (startup WARN). |
| `pursuit_budget_max_sec` | double | `240.0` | Ceiling on one chase leg (s); always wins over the `nav_min_timeout_sec` floor. `<= 0` disables pursuit. NOT the total bound a waiting teammate sees: hybrid then drives the fallback leg (up to `nav_max_timeout_sec` more) and proximity-hold time is refunded to the chase. |
| `pursuit_staleness_max_sec` | double | `900.0` | Last-contact record age beyond which the chase is skipped entirely. Freshness scales the raw budget linearly across the window, but the clamp dominates at field scale: past a ~12 m trail head (flatforest params) the budget sits AT the ceiling until staleness eats below it, then floors, then gates to 0. `<= 0` = no gate. The node default, the sim harness and `shared_params.yaml` all read `900` since 2026-08-18 — one value on every path. At `180` the gate *was* an off switch rather than a mild setting: the mid-run trigger fired at `reconnect_midrun_silence_sec` = 240, so a mid-run dispatch carried a record age of ~240–251 s by construction and always declined, making pursuit and hybrid's chase unreachable at mid-run. That arithmetic does not survive generation 9 — the trigger now fires at 90, so dispatch record age is ~104–109 s and a 180 s gate would no longer decline it. (That is *not* threshold + the 5 s claim TTL, which an earlier revision of this row assumed. Two offsets stack: the TTL adds 4.98–4.99 s (measured 4.99, 4.99, 4.99, 4.99, 4.98 — an earlier revision said 4.98–5.01, and an upper end above the 5.0 s TTL is not a value this quantity can take), and the tick granularity of the mid-run path adds a further 9.02–13.82 s of overshoot. Measured on g8r1's five dispatches, nominal 240 fired at record ages 254.01–258.80 s, i.e. T+14.0 to T+18.8.) Keep `900` for the measured reason that follows, not for the old coincidence; and re-check this gate against the mid-run clock whenever either moves, since the two are coupled. Measured at `900` over 9 chases (p13, flatforest_dense): all reconnected in 36–106 s, none reached its destination — they released a median ~30% of the way in, because two robots driving to each other's last-contact poses converge en route — at a cost indistinguishable from the meeting-point fallback. `pursuit_goal_stale_sec` is what keeps the wider window honest. Sim only; re-check on the first hardware pursuit run. |
| `pursuit_goal_stale_sec` | double | `180.0` | Age past which the chase drops the peer's *declared goal* from the trail and drives only to its last heard pose. A stale declared goal is a dead hypothesis; the contact pose stays geometrically meaningful. In the stale-goal branch the budget must cover the whole remaining trail or the chase declines outright. |
| `pursuit_explore_fallback` / `pursuit_explore_max` | bool / int | `true` / `6` | Pure-`pursuit` only: when a chase is spent or declined, resume exploring (up to this many times) before parking at a barrier. A moving robot can still regain the link; a parked one can only be found. `hybrid` never reaches this — it falls back to the meeting point. |
| `reconnect_confirm_sec` | double | `3.0` | The team must read incomplete continuously this long before a manoeuvre arms. `0` = arm on a single read of the claim table, which produced firings that dissolved before the robot moved (8 of 24 recorded firings ended within 5 s having travelled under a metre) — the node WARNs and the timing from such a run is unusable. |
| `reconnect_midrun_silence_sec` | double | `90.0` | Continuous peer silence that triggers a reconnect manoeuvre **during** exploration, not only at exhaustion. `0` = terminal-only (pre-2026-08-17 behaviour). `240.0` through generation 8, when it had to clear the measured heartbeat-suppression tail (~180 s) by pure waiting, because record age alone cannot tell a silent teammate from an absent one. **Below ~200 s this is only safe with the link veto configured** (both `comms_link_states_topic` and `comms_link_robot_index_topic` set), which makes that distinction from the radio instead; the node WARNs on the low-threshold-without-veto pairing, and `run_campaign.sh` refuses a *sim campaign* that would run it (`campaign_guard_calib.sh` holds that guard's known-answer cases). The hardware launch path has no equivalent refusal — on a robot the WARN is the only thing standing between you and a bare 90 s clock. Lowered because 240 diluted the treatment to nothing — in g8r1 the clock expired in 3 of 23 hybrid cells, so 87% of the treated arm ran behaviourally identical to the control.<br><br>**Two drafts of this number were wrong before this one.** The first, `120`, came from a sweep that filtered episodes on the presence clock and then scored them on the same clock — circular, so its "zero waste" restated the filter. The second, `90`, was re-scored with the filter on the past (presence gap) and the score on the future (radio outage), but on the 20 hybrid cells that did *not* dispatch: a sample selected on the outcome being swept. The number that stands is scored on the `off` arm, which never ran the trigger and is untreated by construction, with the veto as it ships (`reconnect_link_down_confirm_sec` = 0):<br><br>`T`=240 → 6/23 cells arm, 12 fires, 50% wasted, 33% beat natural recovery · `150` → 12/23, 24, 42%, 58% · `120` → 18/23, 36, 33%, 72% · **`90` → 18/23, 40, 20%, 85%** · `75` → 18/23, 42, 29%, 86% · `60` → 21/23, 60, 43%, 73%.<br><br>On the **unwalked** grid `90` is the argmin of waste and `120`, `90`, `75` all arm the same 18 of 23 — but the model is optimistic by a measured 9.02–13.82 s of tick overshoot plus the 4.98–4.99 s TTL, and **neither fact survives walking it forward**: at each of the three measured offsets (+14.0, +16.4, +18.8 s) the waste argmin moves to `75` (20.0 % vs `90`'s 28.6 %; 19.2 % vs 26.3 %; 25.0 % vs 29.4 %) and the plateau breaks (`120` → 14/23, 15/23, 17/23 against `90`'s 18, 19, 17). `90` is therefore **not** chosen by the grid — a ranking that flips under a 4.8 s change in a nuisance offset is one 23 cells cannot resolve, and re-tuning to `75` on the same cells would repeat the error. What the grid supports is only the coarse verdict, stable across the walk: `240` too high (4–6 of 23 everywhere), `60` past the point where waste turns back up. Inside `120`–`75` the choice is off-grid: `90` clears the p90 radio outage (52.0–111.0 s, nearest-rank) and sits far below the `240` s point where the presence clock stops filtering. Out of sample: at generation 8's actual configuration (`T`=240, confirm=3) the model arms 4 of 23 `off` cells against the 3 of 23 hybrid cells that really dispatched. |
| `reconnect_midrun_max_wait_sec` | double | `240.0` | Barrier give-up for a **mid-run** attempt. On expiry the robot resumes exploring — a mid-run attempt must never end the run, because the map is not saturated when one fires. |
| `reconnect_min_share_voxels` | double | `0.0` | Info gate on the mid-run trigger: the estimated unshared-map backlog (dead-reckoned from the last RobotIntent beacon snapshot) must reach this many voxels before silence alone can dispatch. `0` = time-only trigger (shipped default; the gate is the experiment arm). **Only meaningful with `reconnect_midrun_silence_sec > 0`** — the node WARNs when the gate is set while mid-run triggering is off, because the gate is evaluated inside the mid-run path and can never fire. |
| `reconnect_midrun_min_silence_sec` / `reconnect_midrun_max_silence_sec` | double | `60` / *(tracks the clock, so `90`)* | Clamp bounds (s) on the info-gated trigger time, so a degenerate backlog estimate (zero or huge growth rate) cannot fire the manoeuvre instantly or defer it forever. With the gate off they are inert. The ceiling is defined as tracking `reconnect_midrun_silence_sec` — a ceiling above it would let a gated run wait *longer* than the ungated control. Since generation 9 that is **enforced rather than asserted**: the ceiling's default is the *resolved* value of `reconnect_midrun_silence_sec`, not a copied constant, because the copied constant is what let it sit at `240` after the clock had moved. An explicit parameter still overrides, which is the deliberate opt-out. The node WARNs if it is ever set above the clock — and that WARN only started working in generation 9: it used to be conjoined with `reconnect_min_share_voxels > 0`, i.e. it could fire only in the configuration where the clamps are *live*, and never in the shipped one where they are inert and drifting. A guard that cannot fire in the default configuration is not a guard. The degenerate `min > max` case is now pinned rather than warned about, since with the gate off there is no run to warn during. |
| `reconnect_midrun_max_attempts` | int | `6` | Mid-run attempt budget for the whole run. Once exhausted the robot reverts to terminal-only reconnection. Cooldown between attempts is stamped at manoeuvre *end* and equals `reconnect_midrun_silence_sec` — so generation 9's move from `240` to `90` shortened the cooldown by the same factor, and the budget of 6 can now be spent in well under half the wall-clock it used to need. That is a real behavioural change riding along with the threshold, not an independent knob; it is called out here because it is invisible at the call site. |
| `reconnect_release_confirm_sec` | double | `6.0` | The team (or the quarry alone) must read live continuously this long before a manoeuvre releases. **Must exceed `coord_claim_ttl_sec` (5.0)** — one packet holds a peer live for the whole TTL, so a shorter window is satisfied by that single packet and the flicker guard is inert. The node WARNs rather than clamping, so an A/B can still set `0` deliberately. |
| `hold_escalate` / `hold_escalate_wait_sec` | bool / double | `true` / `300.0` | On terminal barrier expiry, move once to the meeting point and wait again (a shortened second vigil: a confirmation of failure, not a second full one) before giving up. Sticky — an unreachable target cannot re-escalate forever. Pure `pursuit` keeps the own-anchor escalation by design; a mode-blind escalation would make pursuit perform hybrid's fallback ~300 s later and erase the contrast the arm exists to measure. Latent at the field default `rendezvous_max_wait_sec: 0`, which never expires. |
| `reconnect_arrive_tol_m` | double | `4.0` | Arrival tolerance for a manoeuvre destination. Deliberately not `goal_xy_tolerance` (0.4): the destination's value is *connectivity*, not position, and the meeting point is a synthetic coordinate never checked against the map — at 0.4 m the robot grinds against whatever occupies it until the budget expires. |
| `reconnect_nav_max_sec` | double | `600.0` | Ceiling on one manoeuvre drive leg. |
| `comms_link_states_topic` / `comms_link_robot_index_topic` | string | `""` / `""` | The **link veto**. Empty (the node default) disables it: `linkGateReady()` returns false on its first line and the mid-run trigger runs on record age alone. Set both to point the planner at the comms emulator's connected bit, and the trigger stands down when the radio says the peer is reachable — the record-age clock cannot tell a silent teammate from an absent one, and measured against the emulator's own trace it ran +49.1 s ahead of real link-down with 41–42% of mid-run fires buying nothing (16–21% firing while the radio was UP). **Required in practice below `reconnect_midrun_silence_sec` ≈ 200**, where the clock alone sits inside the ~180 s heartbeat-suppression tail; the node WARNs on that pairing. The sim harness sets both when `LINK_GATE=1` **and** `COMMS=1` (default since generation 9); `LINK_GATE` is validated to `0|1`, because every other value turns the veto off while reading like a request for it. The manifest records `link_gate=` (what was *asked for*) and `link_gate_effective=` (what the planner will actually get, derived from whether the topic was actually passed — they differ exactly when the gate was requested with no emulator to feed it); the node records `link_gate_configured` in the event log, true when **both** topics are set, since states-without-index is a gate that stands down on every tick while looking configured.<br><br>**`link_gate_configured` is not "the veto ran".** It is written at `startRun`, from two topic *names* being non-empty, before anything has been delivered — so a dead emulator satisfies it while the veto is absent for the whole run, which is exactly the configuration generation 9 exists to prevent. (It was called `link_gate_active` in the first draft, whose comment claimed it was "true only when both topics arrived"; it never checked arrival, and would have passed a dead-emulator run.) The runtime half is a one-shot `link_gate_live:` INFO line emitted from the **link-states subscription** on the first usable sample — deliberately not from `linkGateReady()`, which is reached only when something consults the gate, so a run whose team never went silent would emit nothing and be scored a failure. It lands in `<cell>/planner_<robot>.log` (the node's stdout), **not** in `<cell>.console.log`, which carries only the two harness scripts' own `log()` output. Gate check 3f asserts that line's **presence**, not the absence of the not-usable warning — a run whose logging broke would pass an absence test ([[nav-global-planner-never-planned]]). Check both when comparing runs: the manifest says what the launcher intended, `link_gate_configured` says what the node was handed, and `link_gate_live:` says whether a sample ever arrived. |
| `reconnect_link_down_confirm_sec` | double | `0.0` | How long the radio must have been **continuously down** before the link veto lets a mid-run chase through. Its own parameter since generation 9; both veto sites previously borrowed `reconnect_confirm_sec` (`3.0`), which is the team-*presence* release confirm and answers a different question. Decoupling them is right at any value.<br><br>**The value is `0`, and the 30 s debounce trialled in the first generation-9 draft is withdrawn.** At `0` the veto is exactly its disjunct — never chase a peer whose radio is up *right now* — because the check is written `link_connected_ || link_down_for < confirm`; the bare comparison is `0.0 < 0.0`, false, so the `link_connected_` term *is* the whole test. `30` was credited with moving wasted fires "from 50% to 33%", which was a misattribution: the table that came from held this conjunct fixed at 30 and varied the presence clock, so 50→33 is the 150→90 move, not this parameter's effect. Measured properly on the untreated `off` arm, holding `reconnect_midrun_silence_sec` at 90 — `confirm`=0 → 18/23 cells arm, 40 fires, 20% wasted, 85% beat natural recovery; `confirm`=30 → 12/23, 26, 15%, 92%. It buys 5 points of purity for **a third** of the arm's activation — 18 arming cells down to 12, i.e. 6 of the 18 that armed (an earlier revision said "a quarter", which is 6/23, the share of the *arm*) — and dilution is the defect this generation exists to fix. Negative values are clamped to 0 with a WARN.<br><br>Sizing note for anyone re-raising it, with the population stated because a range over an unstated one cannot be checked: over the 13 campaign tags carrying ≥10 cells (33 tags exist on disk), pooling cells within a tag and counting only outages that closed before the last sample, radio-outage medians are **6.20–12.20 s** (`td1` the floor, `gt1` the ceiling) and p90 is **52.0–111.0 s**. Percentiles are **nearest-rank** (`x[⌈q·n⌉]`); the median range is the same under linear interpolation but the p90 range would read 51.4–101.6 s, so the convention has to be stated with the number. An earlier revision said 6.9–12.1 s and p90 41–84 s over "the banked campaigns"; the p90 range was the wrong one and the population was never defined. |
| `comms_link_stale_sec` | double | `3.0` | Newest link sample older than this and the veto stands down to the record-age clock rather than acting on a stale belief. The emulator publishes at 5 Hz, so this is ~15 missed samples. |

**Proximity stop (coordinated yield)** — best-effort coordination, *not* a
certified safety stop. Right of way: the lexicographically smaller
`robot_name` drives; the larger yields (cancel + brake-hold + resume).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `proximity_stop_enabled` | bool | `true` | Inert without peers. |
| `proximity_hold_dist_m` / `proximity_resume_dist_m` | double | `5.0` / `6.0` | Yield below / resume beyond (hysteresis). Sized against the full reaction latency, not just the panic line. |
| `proximity_pose_stale_sec` | double | `3.0` | Data older than this cannot *start* a hold. |
| `proximity_hold_release_stale_sec` | double | `10.0` | An active hold with no data this long releases. |
| `proximity_peer_static_sec` / `proximity_peer_static_move_m` | double | `10.0` / `0.3` | Peer moving less than this for this long is "parked" → not held against (kept above `exploit_dwell_sec` so a mid-capture dwell is waited out). |
| `proximity_parked_keep_dist_m` | double | `1.5` | …unless parked inside this floor: the hold continues. |
| `proximity_max_hold_sec` | double | `120.0` | Escape hatch; `0` = unbounded. |
| `proximity_escape_grace_sec` | double | `30.0` | Post-escape immunity: after the hatch fires, the escaped-from peer cannot *start* a new hold for this long (else a peer still static inside `parked_keep_dist_m` re-holds on the next tick — 0.1 s of driving per `max_hold_sec`, forever). Cancels early if the peer moves. `0` disables. |
| `proximity_peer_pose_topics` | string[] | `[]` | `"<robot_name>:<topic>"` localiser poses (~10 Hz). Unset = heartbeat-only (fine in sim; the node WARNs — configure on hardware). |
| `proximity_nav_cancel_action` | string | `""` | *auto* → `/<robot_name>/navigate_to_pose`. |

**Perceptive exploitation**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `exploitation_enabled` | bool | `true` | `false` = pure exploration; the targets topic is never subscribed. |
| `targets_topic` | string | `/exploration/targets` | `TreeTarget` input. |
| `n_vantages` | int | `3` | Ring size (3 → 120° apart). ≤ 32 for team dwell-credit (bitmask). Must be identical fleet-wide. |
| `min_vantages_required` | int | `3` | Clear-LoS dwells (team union) for a COMPLETE target. Clamped into `[1, n_vantages]`. |
| `vantage_standoff_m` | double | `2.0` | Standoff added to trunk radius, clamped to `[fov_min_range + r, fov_max_range]`. |
| `vantage_start_angle_deg` | double | `30.0` | First ring angle. Must be identical fleet-wide (credit is exchanged as ring *indices*). |
| `exploit_dwell_sec` | double | `8.0` | Hold time per vantage. |
| `vantage_visited_tol_m` | double | `0.75` | Proximity that counts a vantage "dwelled"; also the *auto* exploit claim disc. |
| `exploit_dwell_sync_enabled` | bool | `true` | Vantage-ring barrier: dwell clocks start only once every same-trunk claimant is `staged`, so captures are simultaneous. Start-condition only. |
| `exploit_dwell_sync_max_wait_sec` | double | `0.0` | Barrier give-up from arrival; `0` = wait for the peer (a dead peer's claim ages out on the TTL and releases the barrier — an unbounded wait cannot hang the run). |
| `exploit_target_timeout_sec` | double | `300.0` | Wall-clock per trunk since the last progress event; expiry closes it PARTIAL. Keep above `nav_max_timeout_sec`. `<= 0` disables. |
| `target_dedup_radius_m` | double | `1.5` | Re-reported trees within this merge into the existing target (id dedup always applies). |
| `publish_refinement_regions` | bool | `true` | Relay targets as fine-TSDF regions to the mapper. |
| `fine_region_radius_m` | double | `0.5` | Region radius added around the trunk. |
| `refinement_region_topic` | string | `""` | *auto* → `/<robot_name>/scovox_node/refinement_region`. |

---

### 4.2 target_scheduler_node

Time-based `TreeTarget` publisher — the stand-in for a live detector. Reads a
preselected list as parallel parameter arrays and releases each target when
its time (relative to the schedule origin) elapses. The origin is latched on
the **first tick with a valid clock**, so under `use_sim_time` the schedule
starts when bag/sim playback actually starts, and a release time of `0.0`
means "the moment the node is started" — the pattern for releasing targets on
a director's cue. Node name `target_scheduler`; ticks at 2 Hz.

#### Published topics

- `<targets_topic>` (`explo_planner_msgs/TreeTarget`), default
  `/exploration/targets` — one message per target at its release time,
  `discovered_by: "schedule"`. QoS: reliable, `transient_local`, depth 50
  (late-joining planners receive everything already released).

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `targets_topic` | string | `/exploration/targets` | Output topic. |
| `frame_id` | string | `map` | `TreeTarget.header.frame_id`. |
| `target_ids` | int[] | `[]` | Stable ids; drives the count. All arrays below must match its length or the scheduler disables itself loudly. |
| `target_x` / `target_y` / `target_z` | double[] | `[]` | Trunk centres (m, `frame_id`). |
| `target_radius` | double[] | `[]` | Trunk radii (m); `<= 0` = point trunk (default standoff only). |
| `target_height` | double[] | zeros | Optional trunk extents (m). |
| `target_release_sec` | double[] | `[]` | Release times from the schedule origin (s). |

Shipped schedules: [`targets.yaml`](../explo_planner/config/targets.yaml),
[`targets_map_test2.yaml`](../explo_planner/config/targets_map_test2.yaml), and
on the `new_experiments` branch `targets_flatforest.yaml` (sim world).

---

### 4.3 tree_detector_node

Live tree detector — a drop-in replacement for the scheduler. Segments trunks
out of the fused map, scores how well each has been observed
(angular-coverage-primary), and publishes the **under-informed** ones as
`TreeTarget` on the same topic; the planner is unchanged. Target ids are
derived from the trunk's quantised map-frame position, so the same tree gets
the same id on every robot and across re-detections — which is what makes the
planner's id-keyed dedup and team dwell-credit refer to one tree fleet-wide.
Each tree is emitted **once** per node lifetime, after `confirm_ticks`
consecutive under-informed scans and (under bearing coverage) a settled
bearing history. Node name `tree_detector`.

#### Subscribed topics

- `<dscovox_topic>` (`scovox_msgs/ScovoxMap`), default
  `/<robot_name>/dscovox_node/scovox` — the fused map, scanned every
  `scan_period_sec`. QoS: reliable, `transient_local`, depth 1.

#### Published topics

- `<targets_topic>` (`explo_planner_msgs/TreeTarget`), default
  `/exploration/targets`, `discovered_by: <robot_name>`. QoS: reliable,
  `transient_local`, depth `targets_qos_depth`.

#### Required tf transforms

- `frame_id` → `base_frame` (default `map` → `base_link`) — the robot pose
  for per-track viewing-bearing accumulation. Optional: with no pose the
  score falls back to map-geometry coverage (with a throttled warning).

#### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_name` | string | `robot1` | Seeds the default map topic and `discovered_by`. |
| `dscovox_topic` | string | `""` | *auto* → `/<robot_name>/dscovox_node/scovox`. |
| `targets_topic` | string | `/exploration/targets` | Output topic. |
| `frame_id` | string | `map` | Output frame. |
| `scan_period_sec` | double | `2.0` | Detector cadence. |
| `confirm_ticks` | int | `2` | Consecutive under-informed scans before emit. |
| `track_match_radius_m` | double | `1.5` | XY radius matching a detection to a track. |
| `track_timeout_sec` | double | `30.0` | Drop a track unseen this long (emitted centres stay remembered — no re-nomination). |
| `id_cell_m` | double | `1.0` | XY grid the position-derived id snaps to. |
| `use_bearing_coverage` | bool | `true` | Score angular coverage from accumulated robot viewing bearings rather than the trunk's own voxel azimuth spread (which saturates half-seen). |
| `base_frame` | string | `base_link` | Robot frame for the bearing pose. |
| `bearing_settle_ticks` | int | `3` | Scans the bearing history must stay unchanged before emit; `0` disables. |
| `targets_qos_depth` | int | `500` | Latched history depth; must exceed the targets a run can emit. |
| `use_semantics` | bool | `true` | `false` = geometric mode for LiDAR-only maps with empty semantic records (**required** there — the semantic gate silently drops every unlabeled voxel). |
| *segmentation & scoring knobs* | — | — | `veg_class`, `occ_thresh`, `min_class_conf`, `cluster_tol_m`, `trunk_band_lo/hi`, `min_trunk_voxels`, `min_height`, `max_radius`, `n_azimuth_bins`, `n_height_bins`, `w_coverage`, `w_entropy`, `w_vertical`, `deficit_thresh`, and in geometric mode `terrain_cell_m`, `ground_margin_m`, `stem_slice_lo/hi`, `attach_radius_m`, `min_linearity`, `max_tilt_deg`. Defaults live in [`tree_detector.hpp`](../explo_planner/include/explo_planner/tree_detector.hpp). |

---

## 5. Messages (explo_planner_msgs)

### TreeTarget

The seam between target *production* and exploitation — the scheduler and the
detector publish the identical message.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` = release/detection time; `frame_id` = map frame of `center`. |
| `target_id` | `uint32` | Stable id; the planner keeps the first message per id and ignores later duplicates. |
| `center` | `geometry_msgs/Point` | Trunk axis (XY) and base z. |
| `radius` | `float32` | Trunk radius (m); `<= 0` = point trunk. |
| `height` | `float32` | Optional extent (m), informational. |
| `discovered_by` | `string` | `"schedule"` or a robot id. Diagnostic. |
| `status` | `uint8` | `STATUS_PENDING`=0 / `STATUS_DONE`=1 (reserved for a completion broadcast). |

### RobotIntent

The multi-robot claim, published on `coord_intent_topic` by every planner.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` is diagnostic only — expiry is timed from **local receipt** (`ttl_sec`), so unsynchronised fleet clocks cannot corrupt deconfliction. |
| `robot_id` | `string` | Producer; peers drop self-messages; lex tiebreak key. |
| `goal_pos` / `yaw` | `Point` / `float32` | Claimed viewpoint. |
| `robot_pos` | `Point` | Producer's pose at claim time — peers recompute the MinPos distance contest locally. |
| `claim_radius_m` | `float32` | Reserved disc around `goal_pos`. |
| `ttl_sec` | `float32` | Claim lifetime since receipt. |
| `planner_type` | `uint8` | Diagnostic (0 = eig). |
| `exploit` / `target_id` | `bool` / `uint32` | This claim is an exploit hop on that trunk (`target_id` alone is no discriminator — a detector may emit id 0). |
| `dwelled_mask` | `uint32` | Bitmask of ring indices the producer has dwelled with clear LoS. Peers OR it into local state — cumulative, so a lost message never loses credit. |
| `staged` | `bool` | Producer is physically standing on `goal_pos` in its dwell state (set at dwell entry, after arrival *and* rotation settle). Producer-declared because observers cannot infer it from geometry. The dwell-sync barrier's staging signal. |

**Wire compatibility:** every team member must run the same interface build.
An older peer's intents never set `staged` (its teammates hold their barriers
until the give-up path) and lack the exploit fields (team quota degrades to
solo). The planner and `ros2 topic echo` both fail loudly on a checksum
mismatch, but a *stale installed* `explo_planner_msgs` alongside new planner
code will not — rebuild with `--packages-up-to`.

---

## 6. Launch files

All three load `config/shared_params.yaml` first and pass their declared
arguments on top of it (the launch defaults win over the yaml for those keys).
None of them bring up the simulator, mapper, navigator or tf — see §3
prerequisites and, for the packaged 2-robot sim world, `sim/run_explo_sim_rviz.sh`
(on the `new_experiments` branch).

### exploration_experiment.launch.py

One planner, pure exploration configuration (exploitation still obeys the
yaml — set `exploitation_enabled: false` there for strictly-pure runs).

| Argument | Default | Description |
|---|---|---|
| `robot` | `atlas` | Robot name. |
| `use_sim_time` | `false` | Sim/bag runs must pass `true`. |
| `max_steps` | `200` | NBV step budget. |
| `output_csv` | `/tmp/exploration.csv` | Metrics CSV. |
| `dscovox_topic` | `''` | Map topic override. |
| `map_frame` / `base_frame` | `map` / `''` | Frame overrides. |
| `trajectory_scoring` | `false` | SSMI ablation (always passed — wins over the yaml). |
| `trajectory_sample_spacing_m` | `1.5` | ⬑ |

### exploitation_experiment.launch.py

One planner **plus a target producer**: the time-based scheduler by default,
or the live detector with `use_detector:=true`.

| Argument | Default | Description |
|---|---|---|
| `robot` | `atlas` | Robot name. |
| `use_sim_time` | `false` | Also governs the scheduler's release clock. |
| `max_steps` | `200` | Step budget (explore + exploit). |
| `output_csv` | `/tmp/exploitation.csv` | Metrics CSV. |
| `targets_file` | `config/targets.yaml` | Scheduler parameter file. |
| `use_detector` | `false` | `true` = run `tree_detector_node` instead of the scheduler. |
| `use_semantics` | `true` | Detector gating; set `false` on LiDAR-only maps. |
| `terrain_relative_z` | `true` | Set `false` on flat sim worlds (see the `noground` note in the launch file). |

### multi_robot_exploration.launch.py

One planner per robot on a single host (the sim topology; on hardware launch
one planner per robot PC by hand — this is also the only launch that sets
`rendezvous_expected_peers` for you, from the team size).

| Argument | Default | Description |
|---|---|---|
| `robots` | `atlas,rama` | Comma-separated team. Each planner runs namespaced `/<robot>`. |
| `use_sim_time` | `false` | |
| `planner` | `eig` | CSV filename label only (the node is EIG-only). |
| `output_dir` / `config_id` / `world` | `/tmp` / `c1` / `flatforest` | Compose the per-robot CSV name `exp7_<planner>_<world>_<config_id>_<robot>.csv`. |
| `max_steps` | `100` | Per-robot budget. |
| `coordination_enabled` | `true` | MinPos deconfliction. |
| `rendezvous_enabled` | `true` | Run the `reconnect_mode` manoeuvre on goal exhaustion with teammates out of comms. |
| `rendezvous_max_wait_sec` | `0.0` | Barrier give-up. |
| `reconnect_mode` | `hybrid` | Which manoeuvre: `rendezvous` / `pursuit` / `hybrid` (see the §4 parameter table). |
| `proximity_stop_enabled` | `true` | Coordinated yield. |

---

## 7. The metrics CSV

One row per LOG_STEP (each completed explore step, exploit hop or dwell), written
to `output_csv`. Columns, in order:

| Column | Meaning |
|---|---|
| `phase`, `target_id`, `vantage_index` | `explore`/`exploit`; active target and dwelled ring index (`-1` in explore). |
| `n_vantages_valid`, `vantage_los_clear`, `dwell_sec` | Exploit diagnostics: selectable vantages this tick, clear-LoS flag, honest capture length (dwell-sync wait time is *not* counted). |
| `step`, `sim_time_sec` | Step counter and node-clock time. |
| `total_observed_voxels`, `frontier_voxels` | Map growth. |
| `distance_traveled` | Integrated from tf, teleport-guarded (`max_pose_jump_m`). |
| `selected_score`, `plan_time_ms`, `mean_eig`, `mean_entropy`, `mean_variance` | Selection diagnostics. |
| `mean_info_gain`, `mean_path_cost`, `selected_info_gain`, `selected_path_cost` | Utility decomposition (`U = gain / (0.1 + cost)`) for post-hoc attribution. |
| `selected_utility` | **Redundant: an exact alias of `selected_score`** — both are assigned `current_goal_.score`, and all 3432 rows of the g6pilot campaign were identical. The planner has one scalar objective; there is no second "utility" quantity here. Kept only because the CSV is append-only and read positionally, so removing a mid-file column would shift every column after it. Never decompose score against utility or fit one on the other — that is a regression of a column on itself. |
| `info_gain_std` | Population std of `info_gain` across this step's candidates (last column, not beside `mean_info_gain` — the schema only grows at the end). Small relative to `mean_info_gain` means the candidates barely differ in information and the selection has degenerated to argmin-cost. |
| `coord_active_peers`, `rejected_by_minpos`, `rejected_by_unreachable` | Coordination diagnostics; `coord_active_peers` should read team size − 1 in a healthy run. |
| `prox_hold_count`, `prox_hold_total_sec` | Cumulative proximity holds (difference consecutive rows for per-step deltas). |

---

## 8. Integration notes

**Bringing your own navigator.** The command interface is one topic:
`PoseStamped` on `goal_topic`, goals in `map_frame`, yaw meaningful. The
planner judges arrival itself from tf against `goal_xy_tolerance` /
`goal_yaw_tolerance` — there is no action feedback, so a goal published while
the navigator is down is lost silently (the `goal_republish_sec` keep-alive
exists for exactly that). Your navigator's own arrival tolerance must be
tighter than the planner's. Only the proximity stop's cancel is Nav2-specific
(`NavigateToPose` cancel-all); with a different navigator either disable
`proximity_stop_enabled` or accept that holds brake by goal-republish alone.

**Bringing your own target producer.** Publish `TreeTarget` on
`targets_topic` with reliable + `transient_local` QoS and stable ids. That is
the entire contract — the planner cannot tell the scheduler from the detector.

**Bringing your own map.** The planner consumes `scovox_msgs/ScovoxMap` only.
The frame must equal `map_frame` (a mismatched frame is used unreframed, with
a warning). Publish latched (`transient_local`) so a restarting planner leaves
`WAIT_FOR_MAP` immediately.

**Multi-robot invariants.** Fleet-wide identical: `n_vantages`,
`vantage_start_angle_deg` (credit travels as ring indices),
`coord_intent_topic`, the interface build (§5), and the map prior upstream.
Per-robot distinct: `robot_name`, `output_csv`, per-robot map topics — and
`targets_topic` *may* be split per robot to assign different target lists.
`rendezvous_expected_peers` must be set to team size − 1 by hand outside the
multi-robot launch.

**Where to go deeper:** [planner_method.md](planner_method.md) (algorithms and
the reasoning behind the coordination machinery) ·
[user manual](user_manual.md) (field operations) ·
[`shared_params.yaml`](../explo_planner/config/shared_params.yaml) (every
parameter, commented) ·
[limitations.md](../explo_planner/doc/limitations.md) (known rough edges) ·
bag-replay dry runs: [exploration](../explo_planner/doc/dscovox_exploration_run.md),
[exploitation](../explo_planner/doc/dscovox_exploitation_run.md).
