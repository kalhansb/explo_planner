# explo_planner exploration over a SINGLE-ROBOT dscovox map (bag replay)

Runs `explo_planner` on the fused dscovox merger output
(`/robot1/dscovox_node/scovox`), built live from the `map-test-2` bag.

Pipeline: `bag → NDT/EKF localizer (map→odom→base_link) → scovox rolling mapper
→ dscovox merger → /robot1/dscovox_node/scovox → explo_planner`.

Nothing drives to the goal. The bag replays a recorded walk, so each NAVIGATE
leg ends on its nav timeout and the planner replans from wherever the bag is.
The run tests candidate generation, EIG scoring and goal selection on a real
fused map, not closed-loop navigation.

Run every command from `ws/src/`, each block in its own terminal. Start order
matters: the localizer, scovox, planner and RViz come up first and idle on
`use_sim_time` until `/clock`. The bag plays last.

```bash
cd ~/Projects/hmr_explo_ws/hmr_explo/ws/src
```

## 0. Containers up, build scovox + the explo_planner overlay

`explo_planner` is not mounted into the scovox container. The block below
tar-streams it into a `/tmp/ovl` overlay and builds it against `/scovox/install`.
Build once (~20 s); the overlay survives container stop/start. Everything runs
from `install/`, so re-run the tar + build after any host edit to
`explo_planner`, configs included.

The `scovox:jazzy` image lacks two build dependencies:

- **`nav2_msgs`**. Without it CMake stops at `find_package(nav2_msgs REQUIRED)`.
  The `apt-get` below is needed once per container. A `docker compose up` that
  recreates the container loses it, along with `/tmp/ovl`.
- **`explo_planner_msgs`**, a sibling package in the same tar. Build with
  `--packages-up-to explo_planner`. `--packages-select` skips it and fails on
  `find_package(explo_planner_msgs REQUIRED)`.

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

Wait until NDT has loaded its map (the log settles on `Activating end`) before
step 3.

## 2. Single-robot dscovox (rolling mapper + merger) — leave running

Publishes the planner's map on `/robot1/dscovox_node/scovox`. Leave the share
z-band off (the launch default). The planner re-bands `roi_min_z`/`roi_max_z`
around the robot, so an absolute clip here would starve it.

```bash
docker compose -f scovox/compose.yaml exec scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  ros2 launch scovox_mapping dscovox_single_robot.launch.py \
    robot:=robot1 cloud_topic:=/ouster/points base_frame:=os_lidar use_sim_time:=true
'
```

## 3. explo_planner against the dscovox map — leave running

Uses the bag-tuned
[../config/exploration_fused_bag.yaml](../config/exploration_fused_bag.yaml)
from the overlay and overrides only the map topic. Its `base_frame: base_link`
and `use_planning_map: false` already fit this run.

```bash
docker compose -f scovox/compose.yaml exec scovox bash -lc '
  source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
  source /tmp/ovl/install/setup.bash
  ros2 run explo_planner explo_planner_node --ros-args \
    --params-file /tmp/ovl/install/explo_planner/share/explo_planner/config/exploration_fused_bag.yaml \
    -p dscovox_topic:=/robot1/dscovox_node/scovox
'
```

The planner waits in `WAIT_FOR_MAP` until the bag brings up `/clock`, TF and the
map.

## 4. Play the bag LAST (starts /clock + streams data)

```bash
docker compose -f hmr_localisation/compose.yaml exec ros bash -lc '
  source /opt/ros/jazzy/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/config/fastdds_shm.xml
  ros2 bag play /ws/bags/2026_06_19_18_19_06__kalhan-map-test-2_ \
    --clock --rate 0.5 --topics /ouster/points /imu/data /tf /tf_static
'
```

For a ~120 s trial instead of the full ~500 s bag, append
`--playback-duration 120`.

## 5. RViz — visualize candidates + EIG + goal (scovox container, hardware GL)

Start RViz before the bag (step 4). Without it you see nothing: the planner
publishes `/explo_planner/candidates` only while something subscribes
(`publishCandidateViz` in
[../src/explo_planner_node.cpp](../src/explo_planner_node.cpp)). The CSV still
logs scores.

[../rviz/explo_experiment_dscovox.rviz](../rviz/explo_experiment_dscovox.rviz)
already shows the merger's cloud (`/robot1/dscovox_node/pointcloud`),
**Candidates** (`/explo_planner/candidates`) and **Selected Goal**
(`/goal_pose`). Don't use `explo_experiment.rviz` here: it shows
`/scovox_node/pointcloud`, for `run_explo_experiment.sh`.

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

What you see on each PLAN tick:

- **dscovox map**: the merger's fused cloud as class-colored boxes. It grows as
  the bag walk reveals the scene.
- **Candidates**: one arrow per viewpoint (polar grid + frontier centroids) at
  its 3D pose and yaw, colored red→green by utility `info_gain/(ε+path_cost)`.
  Red is low or unreachable.
- **Goal**: a cyan sphere over the chosen candidate, plus the `/goal_pose` arrow.

## Verify

- **Merger** (step 2 console): `dscovox_diag: sources=1` with `fused_voxels` > 0,
  every ~5 s.
- **Planner map flowing**:
  ```bash
  docker compose -f scovox/compose.yaml exec scovox bash -lc '
    source /opt/ros/jazzy/setup.bash; source /scovox/install/setup.bash
    ros2 topic hz /robot1/dscovox_node/scovox
  '
  ```
- **Planner cycling** (step 3 console): a `Step N: selected goal` line every
  ~15–45 s of bag time. Per-step metrics go to `/tmp/exploration_fused_bag.csv`
  inside the scovox container.

## Troubleshooting

- **Merger stuck at `sources=0`, or the mapper logs `DROPPING scan`**: the
  `map→odom→base_link→os_lidar` TF isn't resolving. Either NDT hasn't reached
  `Activating end`, or the bag isn't playing `/tf /tf_static`.
- **NDT or scovox sees no scans**: almost always TF (above), not QoS. Don't add
  `--qos-profile-overrides-path`. The bag offers `/ouster/points` best_effort,
  and both NDT and the scovox mapper subscribe best_effort.
- **Planner never leaves `WAIT_FOR_MAP`**: `/robot1/dscovox_node/scovox` isn't
  publishing (check the merger), or step 3 is missing the `dscovox_topic`
  override.
- **`explo_planner_node` not found**: the `/tmp/ovl` overlay wasn't built
  (step 0) or sourced (`source /tmp/ovl/install/setup.bash` in step 3).
