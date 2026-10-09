# explo_planner and nav2

This page covers what explo_planner needs from a robot's nav2, and how to
change the names it uses to match your nav2. Below, `<r>` is the robot's
name: `bunker` or `curt`.

## What explo_planner needs

| What | Name now | explo_planner parameter |
| --- | --- | --- |
| A topic where nav2 takes goals (`geometry_msgs/PoseStamped`, `map` frame). explo_planner sends each goal there and repeats it every 5 s. | `/<r>/goal_pose` | `goal_topic` |
| nav2's `NavigateToPose` action. explo_planner follows each goal's status, feedback and result. | `/<r>/navigate_to_pose` | `nav_status_action` |
| The same action, to cancel a goal when the robot has to stop. | `/<r>/navigate_to_pose` | `proximity_nav_cancel_action` |
| The robot's position on `/tf`. | `map` → `base_link` (curt: `base_link_curt`) | `map_frame`, `base_frame` |

nav2 must stop within 0.4 m and 0.4 rad of a goal. explo_planner counts a
goal as reached only when the robot is that close.

## What explo_planner publishes for nav2

explo_planner's ground map (`traversability_map.launch.py namespace:=<r>`)
publishes these two topics. nav2's costmaps and collision monitor can use
them:

| Topic | Type |
| --- | --- |
| `/<r>/occupancy_map_local` | `nav_msgs/OccupancyGrid` |
| `/<r>/pointcloud_2_laserscan` | `sensor_msgs/LaserScan` |

## Changing the names

Set the parameters in the robot's block of the params file explo_planner
is launched with (`params_file:=`, normally
`explo_planner/config/exploration_real_robot.yaml`; rebuild the workspace
after editing it). For example, for a bunker nav2 without a namespace:

```yaml
/bunker/explo_planner:
  ros__parameters:
    goal_topic: /goal_pose
    nav_status_action: /navigate_to_pose
    proximity_nav_cancel_action: /navigate_to_pose
```

The ground map's topics follow its `namespace:=` argument. Without one, they
are `/occupancy_map_local` and `/pointcloud_2_laserscan`. explo_planner then
needs `planning_map_topic: /occupancy_map_local` in the same block.

If both robots share one ROS network, each robot needs its own names.

## Checking

At startup explo_planner logs the names it uses, for example
`goal=/bunker/goal_pose` and `Watching nav2 on /bunker/navigate_to_pose`.
If nothing answers on that action 10 s after the first goal, it logs
`No publisher on '/bunker/navigate_to_pose/_action/status'`.
