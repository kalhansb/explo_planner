"""Launch one explo_planner_node per robot for an Exp 7 trial.

Thin wrapper around exploration_experiment.launch.py: declares the robot
team, picks the planner, and stamps coordination on. Does NOT launch
Gazebo, robots, scovox_node, or dscovox_node -- those come from
single_robot_sim.launch.py + per-robot simple_nav_3d.launch.py invocations
(the per-robot dscovox_node lives inside simple_nav_3d's mapping=="dscovox"
block already; the only change there is wiring the peers list, see
simple_nav_3d.launch.py:peers).

Each planner reads its OWN robot's per-robot fused view via the existing
default /<robot>/dscovox_node/scovox -- there is no central merger.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    pkg_dir = get_package_share_directory("explo_planner")
    params_yaml = os.path.join(pkg_dir, "config", "exploration_params.yaml")

    robots_raw = LaunchConfiguration("robots").perform(context)
    robots = [r.strip() for r in robots_raw.split(",") if r.strip()]
    if not robots:
        raise RuntimeError("multi_robot_exploration: 'robots' is empty")

    planner = LaunchConfiguration("planner").perform(context)
    map_type = LaunchConfiguration("map_type").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)
    config_id = LaunchConfiguration("config_id").perform(context)
    world = LaunchConfiguration("world").perform(context)
    max_steps = LaunchConfiguration("max_steps").perform(context)
    coordination_enabled = LaunchConfiguration("coordination_enabled").perform(context)

    os.makedirs(output_dir, exist_ok=True)

    # Disambiguate CSV filename by map_type so trials with the same planner
    # but different maps (e.g. entropy:logodds vs entropy:dscovox) don't
    # overwrite each other's results. The dscovox map is the historical
    # default, so keep the legacy filename shape for those trials.
    map_suffix = "" if map_type == "dscovox" else f"_{map_type}"

    nodes = []
    for robot in robots:
        csv_name = f"exp7_{planner}{map_suffix}_{world}_{config_id}_{robot}.csv"
        output_csv = os.path.join(output_dir, csv_name)

        nodes.append(Node(
            package="explo_planner",
            # EIG-only NBV planner. The `planner` arg is kept only as a CSV
            # filename label (this node is hard-wired to EIG); multi-robot
            # MinPos deconfliction is toggled by coordination_enabled below.
            executable="explo_planner_node",
            namespace=robot,
            name="explo_planner",
            output="screen",
            parameters=[
                params_yaml,
                {
                    "use_sim_time": True,
                    "map_type": map_type,
                    "robot_name": robot,
                    "max_steps": int(max_steps),
                    "output_csv": output_csv,
                    # dscovox_topic / planning_map_topic intentionally left
                    # at their defaults so each planner reads its own
                    # robot's per-robot fused view.
                    # Multi-robot coordination on. The MinPos branch in the
                    # planner's doPlan walks claim_matching against peer
                    # intents on /exploration/intents.
                    "coordination_enabled":
                        coordination_enabled.lower() in ("true", "1", "yes", "on"),
                },
            ],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="atlas,rama",
                              description="Comma-separated robot team list"),
        DeclareLaunchArgument("planner", default_value="eig",
                              description="Planner type: eig, entropy, frontier, random"),
        DeclareLaunchArgument("map_type", default_value="dscovox",
                              description="Map backend: dscovox or logodds"),
        DeclareLaunchArgument("output_dir", default_value="/tmp",
                              description="Directory for per-robot CSVs"),
        DeclareLaunchArgument("config_id", default_value="c1",
                              description="Start configuration id (for CSV filename)"),
        DeclareLaunchArgument("world", default_value="flatforest",
                              description="World name (for CSV filename)"),
        DeclareLaunchArgument("max_steps", default_value="100",
                              description="Per-robot step budget"),
        DeclareLaunchArgument("coordination_enabled", default_value="true",
                              description="Enable MinPos peer-claim deconfliction"),
        OpaqueFunction(function=launch_setup),
    ])
