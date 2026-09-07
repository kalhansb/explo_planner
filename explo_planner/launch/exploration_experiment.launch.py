"""Launch the EIG exploration planner for experiment 5.

Loads config/shared_params.yaml (the parameters shared by every launch) and
passes the arguments declared below on top of it — their defaults here are the
source of truth for those per-run values.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('explo_planner')
    shared_params = os.path.join(pkg_dir, 'config', 'shared_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='atlas',
                              description='Robot name (namespace)'),
        # Defaults false (hardware). The planner's 10 Hz tick runs off the node
        # clock, so use_sim_time:=true with no /clock publisher leaves the timer
        # dead and the planner silently idle. Sim/bag runs must pass true.
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock (sim/bag runs only)'),
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
        # Optional overlay layered ON TOP of shared_params.yaml — e.g. the
        # hardware set, config/exploration_real_robot.yaml. Defaults to
        # shared_params.yaml itself, so the unset case loads that file twice
        # (a no-op) rather than needing a conditional here.
        #
        # NOTE: the per-node dict below is applied AFTER this file and wins,
        # so map_frame/base_frame/output_csv/max_steps must still come from
        # the launch arguments — on hardware pass base_frame:=base_link (or
        # base_link_curt) explicitly. The overlay's `/**` block still applies
        # to every key the dict does not set (the no-progress watchdog above
        # all).
        DeclareLaunchArgument('params_file', default_value=shared_params,
                              description='Extra param file layered over shared_params.yaml'),
        DeclareLaunchArgument('trajectory_scoring', default_value='false',
                              description='Sum FOV scores along path (SSMI ablation)'),
        DeclareLaunchArgument('trajectory_sample_spacing_m', default_value='1.5',
                              description='Spacing between scored poses along the path (m)'),

        Node(
            package='explo_planner',
            # EIG-only NBV planner (SCovox Beta expected-information-gain).
            executable='explo_planner_node',
            name='explo_planner',
            output='screen',
            parameters=[
                shared_params,
                LaunchConfiguration('params_file'),
                {
                    'robot_name': LaunchConfiguration('robot'),
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    # A bare LaunchConfiguration evaluates to a *string*. The node
                    # declares these as int/bool/double, so passing them raw makes
                    # declare_parameter throw InvalidParameterTypeException at
                    # startup. ParameterValue(..., value_type=...) coerces first.
                    'max_steps': ParameterValue(
                        LaunchConfiguration('max_steps'), value_type=int),
                    'output_csv': LaunchConfiguration('output_csv'),
                    'dscovox_topic': LaunchConfiguration('dscovox_topic'),
                    'map_frame': LaunchConfiguration('map_frame'),
                    'base_frame': LaunchConfiguration('base_frame'),
                    'trajectory_scoring': ParameterValue(
                        LaunchConfiguration('trajectory_scoring'), value_type=bool),
                    'trajectory_sample_spacing_m': ParameterValue(
                        LaunchConfiguration('trajectory_sample_spacing_m'),
                        value_type=float),
                },
            ],
        ),
    ])
