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

## Notes

- **Navigation is pluggable; Nav2 is what you will use.** The only command
  path is one `PoseStamped` on `goal_topic` (default
  `/<robot_name>/goal_pose`, frame `map_frame`), re-published on change or
  every `goal_republish_sec`. Arrival is judged by the planner itself from TF
  against `goal_xy_tolerance`/`goal_yaw_tolerance` — no action feedback is
  consumed, so any stack that accepts a goal pose works. Keep both tolerances
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
- **`use_planning_map` defaults to false**: straight-line costs, no 2D
  obstacle/reachability filtering, and the planner never subscribes to a 2D
  grid. Setting it true makes the grid a hard startup precondition on
  `planning_map_topic` (default `/<robot_name>/dscovox_node/planning_map`) —
  **nothing publishes that topic by default**, so the planner waits in
  WAIT_FOR_MAP indefinitely. Only enable it with `planning_map_topic` pointed
  at a real latched `nav_msgs/OccupancyGrid` publisher.
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
  box `roi_min_x`/`roi_max_x`/`roi_min_y`/`roi_max_y` (candidates outside are
  discarded, and it gates the map precondition above — it must cover the
  survey area) and z band
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
