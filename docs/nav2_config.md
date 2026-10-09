# nav2 on the robots: what explo_planner needs

The planner drives nav2 through names built from `robot_name`, and the
traversability map it plans on is also nav2's costmap source. For multi-robot
runs everything is namespaced per robot (`/bunker/…`, `/curt/…`), so each
robot's nav2 has to run in that namespace and name the namespaced topics.
This page lists what nav2 must match and what bunker's nav2 params (as
supplied on 2026-10-09) need changing. curt's params have not been reviewed;
the same list applies with `/curt/` (last section).

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

## Required changes

**1. Launch nav2 in the robot's namespace.** nav2_bringup's
`navigation_launch.py` does not push a namespace itself (Humble and Jazzy):
its `namespace` argument only rewrites the params file under that key
(`RewrittenYaml(root_key=namespace)`). Passed alone, the nodes stay
un-namespaced and no longer match their own params. `bringup_launch.py`
pushes it (`use_namespace:=true`), but it also starts SLAM or map_server +
AMCL, which must not run beside the NDT localiser. So include
`navigation_launch.py` (or bunker's own nav2 launch) inside a `GroupAction`
with `PushRosNamespace('bunker')`, and pass `namespace:=bunker` so the
rewritten params match: the file's un-namespaced keys (`bt_navigator:`,
`controller_server:`, …) can stay. A launch that does not rewrite the file
needs its keys as full node names (`/bunker/bt_navigator:`) or wildcards
(`/**/bt_navigator:`), or the params silently do not load.

**2. Keep TF un-namespaced.** nav2_bringup's `navigation_launch.py` and
`bringup_launch.py` (Humble and Jazzy) remap `/tf` → `tf` and `/tf_static`
→ `tf_static`, so in namespace `bunker` every nav2 node reads
`/bunker/tf`. Nothing publishes there in our setup: every transform lookup
fails and nav2 never starts navigating. Drop those two remappings in a copy
of the launch file (step 1 needs a wrapper launch anyway).

**3. Point the absolute topics at the namespaced ones.** In bunker's params
file:

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
    cmd_vel_in_topic: "cmd_vel_smoothed"           # was /cmd_vel_smoothed (see below)
    scan:
      topic: /bunker/pointcloud_2_laserscan        # was /pointcloud_2_laserscan
```

- `cmd_vel_in_topic` must be wherever the velocity smoother publishes inside
  the namespace. The smoother publishes the relative `cmd_vel_smoothed`,
  which becomes `/bunker/cmd_vel_smoothed`, so the relative name follows it.
  Humble's stock `navigation_launch.py` remaps the smoother's output to
  `cmd_vel` and starts no collision monitor, so check where bunker's launch
  puts it: `ros2 topic info -v /bunker/cmd_vel_smoothed` should list the
  smoother as publisher and the collision monitor as subscriber.
- `cmd_vel_out_topic: "/cmd_vel/nav"` stays if that is what bunker's base
  (or its mux) reads. It must be per robot only if both robots share one
  ROS graph.
- Leave the rest as is. `odom_topic: /bunker_odom` is already per robot, the
  frames are not namespaced, and the relative names
  (`local_costmap/costmap_raw`, `local_costmap/published_footprint`,
  `speed_limit`, `collision_monitor_state`) follow the namespace by
  themselves.

**4. Hand-sent goals.** RViz's *2D Goal Pose* tool must publish
`/bunker/goal_pose`.

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

## Distro note

The file is nav2_bringup's newer default params file: `navigators`,
`error_code_name_prefixes`, `introspection_mode`, `route_server`,
`docking_server` and the MPPI `TrajectoryValidator` all postdate Humble,
which [field_setup.md](field_setup.md) lists for the robots. Humble ignores
them. Humble's `NavigateToPose` result carries no error code (Jazzy's does),
so on Humble the planner's `Watching nav2 …` line has no `abort error
codes`, and `nav2_error_code` in the metrics CSV stays −1.

## Check after the change

```bash
ros2 topic info -v /bunker/goal_pose               # subscriber: bt_navigator
ros2 action list | grep navigate_to_pose           # /bunker/navigate_to_pose
ros2 topic info -v /bunker/occupancy_map_local     # subscribers: both costmaps and explo_planner
ros2 topic info -v /bunker/pointcloud_2_laserscan  # subscriber: collision_monitor
ros2 node info /bunker/local_costmap/local_costmap | grep tf  # /tf, /tf_static; not /bunker/tf
```

The planner's startup lines then read `goal=/bunker/goal_pose` and
`Watching nav2 on /bunker/navigate_to_pose`, and a hand-sent goal on
`/bunker/goal_pose` drives the robot.

## curt

Not reviewed. The same changes apply with `/curt/`, with curt's own frames
(`odom_curt`, `base_link_curt`) and its traversability_mapping launched with
`namespace:=curt base_frame:=base_link_curt`
([field_setup.md](field_setup.md)). The 2026-07-31 curt bag also carries
`/goal_pose` and `/plan` un-namespaced, so expect its nav2 to need the same
changes.
