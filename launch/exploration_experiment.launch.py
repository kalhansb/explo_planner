"""Launch the EIG exploration planner for experiment 5."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('explo_planner')

    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='eig',
                              description='Planner type: eig, entropy, frontier, random'),
        DeclareLaunchArgument('map_type', default_value='dscovox',
                              description='Map type: dscovox or logodds'),
        DeclareLaunchArgument('robot', default_value='atlas',
                              description='Robot name (namespace)'),
        DeclareLaunchArgument('max_steps', default_value='200',
                              description='Maximum NBV steps'),
        DeclareLaunchArgument('output_csv', default_value='/tmp/exploration.csv',
                              description='Output CSV path'),
        DeclareLaunchArgument('dscovox_topic', default_value='',
                              description='Override DSCovox map topic (empty = /<robot>/dscovox_node/scovox)'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='Frame the dscovox map (and goals) live in'),
        DeclareLaunchArgument('base_frame', default_value='',
                              description='Robot base frame (empty = <robot>/base_link)'),
        DeclareLaunchArgument('trajectory_scoring', default_value='false',
                              description='Sum FOV scores along path (SSMI ablation)'),
        DeclareLaunchArgument('trajectory_sample_spacing_m', default_value='1.5',
                              description='Spacing between scored poses along the path (m)'),

        Node(
            package='explo_planner',
            # Comparison node: honours the planner:= arg (eig/entropy/frontier/
            # random/ssmi). The EIG-only explo_planner_node ignores planner_type.
            executable='exploration_planner_comp_node',
            name='explo_planner',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, 'config', 'exploration_params.yaml'),
                {
                    'planner_type': LaunchConfiguration('planner'),
                    'map_type': LaunchConfiguration('map_type'),
                    'robot_name': LaunchConfiguration('robot'),
                    'max_steps': LaunchConfiguration('max_steps'),
                    'output_csv': LaunchConfiguration('output_csv'),
                    'dscovox_topic': LaunchConfiguration('dscovox_topic'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'trajectory_scoring': LaunchConfiguration('trajectory_scoring'),
                    'trajectory_sample_spacing_m': LaunchConfiguration('trajectory_sample_spacing_m'),
                },
            ],
        ),
    ])
