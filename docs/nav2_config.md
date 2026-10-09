# nav2 on the robots: what explo_planner needs

The planner drives nav2 through names built from `robot_name`, and the
traversability map it plans on is also nav2's costmap source. For multi-robot
runs everything is namespaced per robot (`/bunker/…`, `/curt/…`), so each
robot's nav2 has to run in that namespace and name the namespaced topics.
bunker's nav2 params (as supplied on 2026-10-09) do neither, but they can be
used as they are: `nav2_namespaced.launch.py` runs them in the namespace and
points them at the namespaced topics. This page lists what nav2 must match,
what that launch changes and how it was tested. curt's params have not been
reviewed (last section).

## What the planner expects, per robot `<r>`

The goal and action names are the planner's defaults, which
`exploration_real_robot.yaml` keeps; the map is its per-robot
`planning_map_topic`.

| Planner side | Name | nav2 side |
| --- | --- | --- |
| `goal_topic` | `/<r>/goal_pose` | `bt_navigator` subscribes `goal_pose` and turns each pose into a `NavigateToPose` goal |
| `nav_status_action` | `/<r>/navigate_to_pose` | `bt_navigator`'s action server: status, feedback (recoveries), result |
| `proximity_nav_cancel_action` | `/<r>/navigate_to_pose` | the same server; a proximity hold cancels through it |
| planning map | `/<r>/occupancy_map_local` | the costmaps' static layers |
| — | `/<r>/pointcloud_2_laserscan` | the collision monitor's scan source |
| TF | `/tf`, `/tf_static` (one tree) | every nav2 node |

The map and the scan both come from `traversability_map.launch.py
namespace:=<r>` ([field_setup.md](field_setup.md), "Traversability map"): it
prefixes every topic of traversability_mapping except the input cloud, so
the map *and* `traversability_filter`'s laser scan move under `/<r>/`.
Frames are not namespaced: bunker uses `map`, `odom`, `base_link`; curt
`odom_curt`, `base_link_curt`.

## bunker's params as supplied

They run nav2 un-namespaced: the topics nav2 reads are absolute
(`/occupancy_map_local`, `/pointcloud_2_laserscan`, `/cmd_vel_smoothed`), and
the 2026-07-31 bunker bag carries `/goal_pose` and `/plan`. Against the
namespaced planner and traversability map nothing connects:

- the planner's goals on `/bunker/goal_pose` never reach `bt_navigator`, so
  the robot does not move;
- the abort watch and the proximity-hold cancel find no server on
  `/bunker/navigate_to_pose`, so a hold cancels nothing;
- the costmaps get no map;
- the collision monitor gets no scan. On Jazzy and later it then stops the
  robot (`invalid source`); on Humble it passes every command through
  unchecked.

## Launching bunker's nav2 with its params file unchanged

```bash
ros2 launch explo_planner nav2_namespaced.launch.py namespace:=bunker \
  params_file:=/path/to/bunker_nav2_params.yaml
```

This replaces `ros2 launch nav2_bringup navigation_launch.py
params_file:=…`; the params file stays exactly as it is. Add
`use_sim_time:=true` for a bag. The launch needs nav2_bringup and nothing
else from this package, so on a machine without this workspace it runs from
a copy of the file: `ros2 launch ./nav2_namespaced.launch.py …`. It includes
`navigation_launch.py`, so the same nodes start, with three changes:

**1. Namespace.** Every node runs in `/bunker`, and the launch passes
`namespace:=bunker` so that `navigation_launch.py` places the file's
un-namespaced keys (`bt_navigator:`, `controller_server:`, …) under it.

`navigation_launch.py` does not push its `namespace` argument itself (Humble
and Jazzy): it only rewrites the params file under that key
(`RewrittenYaml(root_key=namespace)`). Passed alone, the nodes stay
un-namespaced and no longer match their params. `bringup_launch.py` does
push it (`use_namespace:=true`), but it also starts SLAM or map_server +
AMCL, which must not run beside the NDT localiser.

**2. TF stays on `/tf`.** `navigation_launch.py` remaps `/tf` → `tf` and
`/tf_static` → `tf_static` (Humble and Jazzy). In namespace `bunker` every
node would then read `/bunker/tf`. Nothing publishes there in our setup, so
every transform lookup would fail and nav2 would never activate. The
launch puts `/tf` → `/tf` and `/tf_static` → `/tf_static` remaps ahead of
those, and the first matching rule wins.

**3. Namespaced topics.** nav2 reads the params from a rewritten copy in
`/tmp`; the launch logs its path and each rewrite at startup. Wherever a
value in the file is one of these, it is replaced:

| In bunker's file | Read as | Where it appears |
| --- | --- | --- |
| `/occupancy_map_local` | `/bunker/occupancy_map_local` | both costmaps' `static_layer.map_topic` |
| `/pointcloud_2_laserscan` | `/bunker/pointcloud_2_laserscan` | the collision monitor's `scan.topic` |
| `/cmd_vel_smoothed` | `cmd_vel_smoothed`, i.e. `/bunker/cmd_vel_smoothed` | the collision monitor's `cmd_vel_in_topic`: where the velocity smoother publishes inside the namespace |

The launch warns if the file has no `/occupancy_map_local`. Everything else
is used as it is:

- `odom_topic: /bunker_odom` is already per robot.
- `cmd_vel_out_topic: /cmd_vel/nav` is whatever bunker's base (or its mux)
  reads. It needs to be per robot only if both robots share one ROS graph.
- Frames are not namespaced.
- Relative names (`goal_pose`, `local_costmap/costmap_raw`,
  `local_costmap/published_footprint`, `speed_limit`,
  `collision_monitor_state`) follow the namespace by themselves.

**Tested on Jazzy** (nav2 from apt, in Docker). The setup was bunker's file
as supplied, a stand-in `/bunker/occupancy_map_local` and static
`map` → `odom` → `base_link` transforms. Results:

- Every node came up (`Managed nodes are active`).
- `bt_navigator` subscribed to `/bunker/goal_pose` and served
  `/bunker/navigate_to_pose`.
- Both costmaps subscribed to the map, and the collision monitor to the
  scan.
- The commands ran velocity smoother → `/bunker/cmd_vel_smoothed` →
  collision monitor → `/cmd_vel/nav`.
- TF was read on `/tf`.
- A pose sent on `/bunker/goal_pose` started navigation.

Without the two TF remaps, the same launch read `/bunker/tf`, and nav2
never came up.

**Hand-sent goals.** RViz's *2D Goal Pose* tool must publish
`/bunker/goal_pose`.

### Launching nav2 another way

If bunker's nav2 is started from a launch file of its own, make the same
three changes there:

- Include it in a `GroupAction` with `PushRosNamespace('bunker')`. If that
  launch does not rewrite the params under the namespace, the keys need full
  node names (`/bunker/bt_navigator:`) or wildcards (`/**/bt_navigator:`),
  or the params silently do not load.
- No `/tf` → `tf` remaps, or `SetRemap('/tf', '/tf')` ahead of them.
- In the params file:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      static_layer:
        map_topic: /bunker/occupancy_map_local     # was /occupancy_map_local

global_costmap:
  global_costmap:
    ros__parameters:
      static_layer:
        map_topic: /bunker/occupancy_map_local     # was /occupancy_map_local

collision_monitor:
  ros__parameters:
    cmd_vel_in_topic: "cmd_vel_smoothed"           # was /cmd_vel_smoothed
    scan:
      topic: /bunker/pointcloud_2_laserscan        # was /pointcloud_2_laserscan
```

Check where that launch puts the smoother's output: Humble's
`navigation_launch.py`, for one, remaps it to `cmd_vel` and starts no
collision monitor.

## Recommended, independent of namespacing

**Footprint.** `robot_radius: 0.2` makes the robot a 0.4 m disc; a Bunker
is about 1.0 × 0.78 m. The MPPI cost critic (`consider_footprint: false`)
and the collision monitor (its polygon is
`local_costmap/published_footprint`) both use that disc, so only the 0.8 m
inflation keeps the sides and corners off obstacles. Set the measured
`footprint` (about `base_link`) on both costmaps in place of `robot_radius`;
`consider_footprint: true` on the cost critic then checks the real shape,
at some CPU cost.

**Obstacle threshold.** Neither costmap sets `lethal_cost_threshold`, so it
stays 100. With the default `trinary_costmap: true`, only cells at 100 are
obstacles to nav2, while the planner blocks at 50
(`planning_map_obstacle_threshold`). nav2 can route through ground scored
50–99 that the planner keeps its goals 0.6 m away from. Setting
`lethal_cost_threshold: 50` (a costmap-level parameter, next to
`robot_radius`) would make the two agree, but the ring of 99-cells a parked
Bunker sees around itself 0.4–0.8 m out
([field_setup.md](field_setup.md), "Traversability map", step 6) would then
wall nav2 in: `footprint_clearing_enabled` clears only the footprint. Leave
it at 100 until the ring has been checked live. A middle way is
`trinary_costmap: false` (also costmap-level): 100 stays the only lethal
value, and a cell scored v costs v × 2.54 (50 → 127, 99 → 251, under the
inscribed 253), so navfn and MPPI prefer the ground the planner prefers,
and the ring costs a lot but walls nothing in. Try it on a bag before the
field: MPPI's cost critic then weighs rough ground too.

**Global costmap.** The 200 × 200 m rolling window at 0.1 m (4 M cells)
holds goals up to 100 m away in x and y, so it covers the whole shipped ROI
(its map-frame box is 101 × 66 m) from almost anywhere in it, and the
`GOAL_OUTSIDE_MAP` rejections in [field_setup.md](field_setup.md) do not
arise. Each map message covers only the 20 m window, but a rolling costmap
resets only the area the latest window covers: earlier windows' cells stay
until the costmap rolls past them. So nav2 remembers the ground it has seen,
as the planner's accumulator does, and plans through ground never seen
(`allow_unknown: true`). Nothing to change; at 0.2 m the window would have a
quarter of the cells, if the Jetson is short of CPU.

## Already compatible: keep

- **The default behaviour tree**
  (`navigate_to_pose_w_replanning_and_recovery.xml`). The planner's abort
  rules (`nav2_abort_limit`, `nav2_abort_final_after_recovery`) count its
  recoveries from the action feedback. A tree with other recoveries changes
  what they count.
- **Goal checker**: xy 0.25 m, yaw 0.25 rad. The planner's own
  `goal_xy_tolerance` / `goal_yaw_tolerance` (0.4) must stay looser than
  these, or reached goals are missed and blacklisted.
- **Progress checker**: 0.5 m in 10 s. nav2 notices a stall, and starts its
  recoveries, long before the field overlay's 120 s / 0.25 m watchdog.
- **Planner**: navfn, `tolerance: 0.5`, `allow_unknown: true`. Frontier
  goals and vantages are picked at least 0.6 m from any cell scored 50 or
  more, well clear of nav2's lethal cells.
- **Map QoS**: `map_subscribe_transient_local: true` matches
  traversability_map's reliable, transient-local publisher.

## Distro

bunker's file is for Jazzy or later, and bunker's 2026-07-31 bag was
recorded on Jazzy. Humble cannot run the file:

- Its plugin names use `::` (`nav2_navfn_planner::NavfnPlanner`), which
  Humble does not accept. `planner_server` fails with `Failed to create
  global planner`, and nav2 does not come up.
- Humble's `navigation_launch.py` starts no collision monitor.

Some keys come from nav2 releases newer than the tested Jazzy (1.3.13):
`introspection_mode`, `error_code_name_prefixes` and the MPPI
`TrajectoryValidator`. nav2 ignores keys it does not know, and with the file
as supplied every node came up on Jazzy.

On Humble a `NavigateToPose` result carries no error code. So with a Humble
nav2, the planner's `Watching nav2 …` line names no `abort error codes`, and
`nav2_error_code` in the metrics CSV stays −1.

## Check after launch

```bash
ros2 topic info -v /bunker/goal_pose               # subscriber: bt_navigator
ros2 action list | grep navigate_to_pose           # /bunker/navigate_to_pose
ros2 topic info -v /bunker/occupancy_map_local     # subscribers: both costmaps and explo_planner
ros2 topic info -v /bunker/pointcloud_2_laserscan  # subscriber: collision_monitor
ros2 topic info -v /bunker/cmd_vel_smoothed        # velocity_smoother -> collision_monitor
ros2 topic list | grep tf                          # /tf and /tf_static only, no /bunker/tf
```

The planner's startup lines then read `goal=/bunker/goal_pose` and
`Watching nav2 on /bunker/navigate_to_pose`, and a hand-sent goal on
`/bunker/goal_pose` drives the robot.

## curt

Not reviewed. The 2026-07-31 curt bag carries `/goal_pose` and `/plan`
un-namespaced, so expect its params to be un-namespaced like bunker's. Then
run them with `nav2_namespaced.launch.py namespace:=curt
params_file:=<curt's params>`. curt keeps its own frames (`odom_curt`,
`base_link_curt`), and its traversability_mapping runs with
`namespace:=curt base_frame:=base_link_curt`
([field_setup.md](field_setup.md)).
