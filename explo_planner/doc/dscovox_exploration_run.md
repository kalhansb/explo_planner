# explo_planner exploration over a SINGLE-ROBOT dscovox map (bag replay)

Runs the EIG/NBV `explo_planner` against the **fused dscovox merger** output
(`/robot1/dscovox_node/scovox`) built live from the `map-test-2` bag — not the
direct `scovox_node` that `run_explo_experiment.sh` uses. This is the single-robot
dscovox bring-up ([../../scovox/docs/dscovox_single_robot_run.md](../../scovox/docs/dscovox_single_robot_run.md))
with the planner wired on top.

Pipeline: `bag → NDT/EKF localizer (map→odom→base_link) → scovox rolling mapper
→ dscovox merger → /robot1/dscovox_node/scovox → explo_planner`.

**Nothing navigates to the goal** — the "robot" replays the recorded trajectory,
so each NAVIGATE leg ends on the watchdog and the planner replans from wherever
the bag walk is. This exercises candidate generation + EIG scoring + goal
selection over a real dscovox-fused map; it is not a closed navigation loop.

All commands run from `ws/src/`, each block in its own terminal. **Start order
matters**: localizer + scovox + planner come up first (they idle on `use_sim_time`
until `/clock`), the **bag plays last**.

```bash
cd ~/Projects/hmr_explo_ws/hmr_explo/ws/src
```

## 0. Containers up, build scovox + the explo_planner overlay

`ros2 launch` resolves from `install/`, so rebuild after editing `src/`.
`explo_planner` is **not** mounted into the scovox container — tar-stream it into a
`/tmp/ovl` overlay and colcon-build it against `/scovox/install` (needs
`scovox_core` + `scovox_msgs`). The overlay binary persists across stop/start;
build once (~20 s).

Two dependencies the `scovox:jazzy` image does not supply:

- **`nav2_msgs`** — a hard dependency of `explo_planner` (the coordinated
  proximity stop cancels Nav2 goals through the action client). Without it CMake
  stops at `find_package(nav2_msgs REQUIRED)`. The `apt-get` below is one-off per
  container, but lives in the writable layer: a `docker compose up` that
  **recreates** the container loses it along with `/tmp/ovl`.
- **`explo_planner_msgs`** — a sibling package inside the same tar, so the build
  needs `--packages-up-to explo_planner`. With `--packages-select` colcon skips
  it and the build fails on `find_package(explo_planner_msgs REQUIRED)`.

```bash
docker compose -f hmr_localisation/compose.yaml up -d
docker compose -f scovox/compose.yaml up -d

docker compose -f scovox/compose.yaml exec -T scovox bash -lc \
  'apt-get update && apt-get install -y ros-jazzy-nav2-msgs'

# rebuild scovox_mapping if its src changed (safe to skip otherwise)
docker compose -f scovox/compose.yaml exec scovox bash -lc '
  source /opt/ros/jazzy/setup.bash
  cd /scovox && colcon build --packages-select scovox_mapping
'

# explo_planner overlay (re-run this block after any host edit to explo_planner)
tar --exclude=.git --exclude=build --exclude=install --exclude=log \
    -C . -cf - explo_planner | \
  docker compose -f scovox/compose.yaml exec -T scovox bash -c \
    'mkdir -p /tmp/ovl/src && rm -rf /tmp/ovl/src/explo_planner && tar -C /tmp/ovl/src -xf -'

docker compose -f scovox/compose.yaml exec -T scovox bash -lc '
  source /opt/ros/jazzy/setup.bash && source /scovox/install/setup.bash &&
  cd /tmp/ovl && colcon build --packages-up-to explo_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
'
```

## 1. Localizer (EKF + NDT map→odom + imu_link→imu alias) — leave running

```bash
docker compose -f hmr_localisation/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
  ros2 launch /ws/launch/ekf_odom.launch.py use_sim_time:=true &
  ros2 launch lidar_localization_ros2 lidar_localization.launch.py \
    localization_param_dir:=/ws/config/gt_ouster_ndt_tree_fused.yaml \
    cloud_topic:=/ouster/points imu_topic:=/imu/data use_sim_time:=true \
    global_frame_id:=map odom_frame_id:=odom base_frame_id:=base_link \
    use_imu_preintegration:=true imu_preintegration_use_base_frame_transform:=true \
    publish_lidar_tf:=false publish_imu_tf:=false &
  ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 --qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 \
    --frame-id imu_link --child-frame-id imu &
  wait
'
```

Wait until NDT loads its map (log settles on `Activating end`) before step 3.

## 2. Single-robot dscovox (rolling mapper + merger) — leave running

Publishes the planner's map on `/robot1/dscovox_node/scovox`. Leave the share
z-band OFF (the launch default): the planner re-bands `roi_min_z/roi_max_z`
relative to the robot, so an absolute clip here would starve it.

```bash
docker compose -f scovox/compose.yaml exec scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  ros2 launch scovox_mapping dscovox_single_robot.launch.py \
    robot:=robot1 cloud_topic:=/ouster/points base_frame:=os_lidar use_sim_time:=true
'
```

## 3. explo_planner against the dscovox map — leave running

Reuses the bag-tuned param set
([../config/exploration_fused_bag.yaml](../config/exploration_fused_bag.yaml),
bind-mounted live), overriding only the map topic to the merger. `base_frame:
base_link` and `use_planning_map: false` in that yaml already fit — the merger
publishes no `planning_map`, and terrain-3D mode does not need one.

```bash
docker compose -f scovox/compose.yaml exec scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  source /tmp/ovl/install/setup.bash
  ros2 run explo_planner explo_planner_node --ros-args \
    --params-file /tmp/ovl/install/explo_planner/share/explo_planner/config/exploration_fused_bag.yaml \
    -p dscovox_topic:=/robot1/dscovox_node/scovox
'
```

The planner idles in `WAIT_FOR_MAP` until the bag brings up `/clock`, TF and the map.

## 4. Play the bag LAST (starts /clock + streams data)

```bash
docker compose -f hmr_localisation/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
  ros2 bag play /ws/bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
    --clock --rate 0.5 --topics /ouster/points /imu/data /tf /tf_static
'
```

Quick trial: append `--playback-duration 120` for a bounded ~120 s run instead of
the full ~500 s bag.

## 5. RViz — visualize candidates + EIG + goal (scovox container, hardware GL)

**Required to see the planner viz**, not optional: `publishCandidateViz` is
subscriber-gated ([../src/explo_planner_node.cpp](../src/explo_planner_node.cpp)) —
with nobody subscribed to `/explo_planner/candidates` it publishes nothing
(headless = markers silent; the CSV still logs scores). Start RViz **before** the
bag (step 4) so it's subscribed when the first PLAN tick fires.

Use the purpose-built [../rviz/explo_experiment_dscovox.rviz](../rviz/explo_experiment_dscovox.rviz):
same layout as `explo_experiment.rviz`, but its **Semantic Map** cloud already
points at the merger (`/robot1/dscovox_node/pointcloud`), so the dscovox map draws
under the markers with **no manual retarget**. **Candidates**
(`/explo_planner/candidates`) and **Selected Goal** (`/goal_pose`) are keyed to the
planner namespace and work unchanged.

```bash
# on the HOST, once per login:
xhost +local:root
```

```bash
docker compose -f scovox/compose.yaml exec scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  export DISPLAY="${DISPLAY:-:1}"
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  rviz2 -d /tmp/ovl/install/explo_planner/share/explo_planner/rviz/explo_experiment_dscovox.rviz --ros-args -p use_sim_time:=true
'
# software-GL fallback if RViz crashes on GL:
#   export LIBGL_ALWAYS_SOFTWARE=1
#   unset __GLX_VENDOR_LIBRARY_NAME __NV_PRIME_RENDER_OFFLOAD
```

What you see per PLAN tick:

- **dscovox map** — the fused semantic-occupancy cloud from the merger
  (`/robot1/dscovox_node/pointcloud`), drawn as RGB8 class-colored boxes. Grows as
  the bag walk reveals more of the scene.
- **Candidate generation + EIG scoring** — one arrow per candidate viewpoint at
  its 3D pose+yaw (polar grid + frontier centroids), **colored red→green by EIG
  utility** (red = low / unreachable, green = best `info_gain/(ε+path_cost)`).
- **Goal selection** — a cyan sphere over the chosen candidate + the `/goal_pose`
  arrow.

The dscovox map draws automatically — this config already targets the merger's
cloud. (The committed `explo_experiment.rviz` is left pointing at
`/scovox_node/pointcloud` for `run_explo_experiment.sh`'s direct-node run.)

## Verify

- **Merger** (step 2 console): `dscovox_diag: sources=1 fused_voxels>0` every ~5 s.
- **Planner map flowing**:
  ```bash
  docker compose -f scovox/compose.yaml exec scovox bash -lc '
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    ros2 topic hz /robot1/dscovox_node/scovox
  '
  ```
- **Planner cycling** (step 3 console): `PLAN` / `selected goal` lines every
  ~15–40 s. Per-step metrics land in `/tmp/exploration_fused_bag.csv` **inside the
  scovox container**.

## Troubleshooting

- **Merger stuck at `sources=0` / `DROPPING scan`** — the
  `map→odom→base_link→os_lidar` TF isn't resolving: NDT hasn't reached
  `Activating end`, or the bag isn't playing `/tf /tf_static`.
- **NDT/scovox see no scans** — this is almost always **TF, not QoS** (see the
  merger bullet above). No `--qos-profile-overrides-path` is needed: the bag offers
  `/ouster/points` **best_effort**, and both consumers subscribe best_effort
  (NDT `cloud_sub_` = `SensorDataQoS`; scovox mapper `input_pc_sub_` = best_effort),
  so playback's native sensor QoS already matches. Forcing the topic to reliable on
  playback is unnecessary and can only add latency under fast replay.
- **Planner never leaves `WAIT_FOR_MAP`** — `/robot1/dscovox_node/scovox` isn't
  publishing (check the merger) or `dscovox_topic` wasn't overridden in step 3.
- **`explo_planner_node` not found** — the `/tmp/ovl` overlay wasn't built or
  sourced (step 0 / the `source /tmp/ovl/install/setup.bash` line in step 3).
