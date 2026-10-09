"""nav2 for one robot, in the robot's namespace, from the robot's own nav2
params file used as it is.

  ros2 launch explo_planner nav2_namespaced.launch.py namespace:=bunker \
       params_file:=/path/to/bunker_nav2_params.yaml

Use it in place of `ros2 launch nav2_bringup navigation_launch.py
params_file:=...`. It includes that file, so the same nodes start, with
three differences:

- They run in namespace /<namespace>. bt_navigator then takes
  /<namespace>/goal_pose and serves /<namespace>/navigate_to_pose, the
  names explo_planner uses.
- TF stays on /tf and /tf_static. navigation_launch.py remaps those to the
  relative tf / tf_static, which inside a namespace means /<namespace>/tf.
  Nothing publishes there in this setup.
- The params are read from a rewritten copy; params_file itself is not
  changed. Wherever a value names /occupancy_map_local or
  /pointcloud_2_laserscan, it gets the /<namespace> prefix, because
  traversability_map.launch.py namespace:=<namespace> publishes them there.
  /cmd_vel_smoothed becomes the relative cmd_vel_smoothed, which is where the
  velocity smoother publishes inside the namespace. Each rewrite is logged
  at startup.

Everything else in params_file is used unchanged:
- frames, which are not namespaced;
- the robot's own absolute topics (/bunker_odom, /cmd_vel/nav);
- the keys, which navigation_launch.py places under the namespace itself;
- the tuning.

Needs nav2_bringup, Jazzy or later. On Humble, navigation_launch.py starts
no collision monitor. See docs/nav2_config.md.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap

_ARGS = {
    "namespace": (None, "Robot namespace, e.g. bunker or curt"),
    "params_file": (None, "The robot's own nav2 params file (un-namespaced "
                          "keys and topics), used as it is"),
    "use_sim_time": ("false", "true for bag replay (play with --clock)"),
    "autostart": ("true", "Bring the nav2 lifecycle nodes up at once"),
}


def _topic_rewrites(ns):
    """Un-namespaced topic value in the robot's params -> its name when nav2
    runs in namespace ns."""
    return {
        "/occupancy_map_local": f"/{ns}/occupancy_map_local",
        "/pointcloud_2_laserscan": f"/{ns}/pointcloud_2_laserscan",
        "/cmd_vel_smoothed": "cmd_vel_smoothed",
    }


def _rewrite(node, rewrites, path, done):
    """Replace in place every string value under node that rewrites names,
    appending 'path: old -> new' to done for each."""
    items = node.items() if isinstance(node, dict) else enumerate(node)
    for key, value in items:
        here = f"{path}.{key}" if path else str(key)
        if isinstance(value, (dict, list)):
            _rewrite(value, rewrites, here, done)
        elif isinstance(value, str) and value in rewrites:
            node[key] = rewrites[value]
            done.append(f"{here}: {value} -> {node[key]}")


def _setup(context):
    ns = LaunchConfiguration("namespace").perform(context).strip("/")
    if not ns:
        raise RuntimeError("nav2_namespaced: namespace must name the robot "
                           "(e.g. bunker)")
    params_file = LaunchConfiguration("params_file").perform(context)
    with open(params_file) as f:
        params = yaml.safe_load(f) or {}
    done = []
    _rewrite(params, _topic_rewrites(ns), "", done)
    with tempfile.NamedTemporaryFile("w", prefix=f"nav2_{ns}_",
                                     suffix=".yaml", delete=False) as out:
        yaml.safe_dump(params, out, sort_keys=False)

    log = [LogInfo(msg=f"nav2_namespaced: nav2 in /{ns}, params "
                       f"{params_file} rewritten to {out.name}")]
    log += [LogInfo(msg=f"nav2_namespaced:   {d}") for d in done]
    if not any("occupancy_map_local" in d for d in done):
        log.append(LogInfo(msg="nav2_namespaced: WARNING no /occupancy_map_"
                               "local in params_file: check that the costmaps "
                               f"read /{ns}/occupancy_map_local"))

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("nav2_bringup"), "launch",
            "navigation_launch.py")),
        launch_arguments={
            "namespace": ns,
            "params_file": out.name,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("autostart"),
        }.items())
    # These remaps come before navigation_launch.py's own /tf -> tf on every
    # node's command line, and the first matching remap rule wins.
    return log + [GroupAction([
        PushRosNamespace(ns),
        SetRemap("/tf", "/tf"),
        SetRemap("/tf_static", "/tf_static"),
        navigation,
    ])]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=desc)
         for k, (d, desc) in _ARGS.items()]
        + [OpaqueFunction(function=_setup)])
