# Moved comments: doc/explo_planner_code_notes.md
"""Launch one explo_planner_node per robot for an Exp 7 trial.

Thin wrapper around exploration_experiment.launch.py: declares the robot
team, picks the planner, and stamps coordination on. Every planner loads
config/shared_params.yaml (the parameters shared by every launch); the
arguments declared below are passed on top of it — their defaults here are
the source of truth for those per-run values. Does NOT launch Gazebo,
robots, scovox_node, or dscovox_node -- those come from
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
    shared_params = os.path.join(pkg_dir, "config", "shared_params.yaml")

    robots_raw = LaunchConfiguration("robots").perform(context)
    robots = [r.strip() for r in robots_raw.split(",") if r.strip()]
    if not robots:
        raise RuntimeError("multi_robot_exploration: 'robots' is empty")

    planner = LaunchConfiguration("planner").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context)
    config_id = LaunchConfiguration("config_id").perform(context)
    world = LaunchConfiguration("world").perform(context)
    max_steps = LaunchConfiguration("max_steps").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() \
        in ("true", "1", "yes", "on")
    coordination_enabled = LaunchConfiguration("coordination_enabled").perform(context)
    # Master switch for the reconnect subsystem. The deprecated alias
    # rendezvous_enabled defaults to empty so unset is distinguishable from set;
    # passing both unconditionally would let the alias shadow reconnect_enabled.
    # (notes: launch-reconnect-alias-empty)
    reconnect_enabled = LaunchConfiguration("reconnect_enabled").perform(context)
    rendezvous_enabled = LaunchConfiguration("rendezvous_enabled").perform(context)
    if rendezvous_enabled.strip():
        print("[multi_robot_exploration.launch.py] WARNING: launch argument "
              "'rendezvous_enabled' is deprecated (it is the master switch for "
              "the whole reconnect subsystem, not the rendezvous arm) — use "
              "'reconnect_enabled'.")
        # Use the stripped value: the truthiness test below is an exact
        # membership check, so padding would turn the subsystem off.
        # (notes: launch-alias-stripped)
        reconnect_enabled = rendezvous_enabled.strip()
    rendezvous_max_wait_sec = LaunchConfiguration("rendezvous_max_wait_sec").perform(context)
    reconnect_mode = LaunchConfiguration("reconnect_mode").perform(context)
    proximity_stop_enabled = LaunchConfiguration("proximity_stop_enabled").perform(context)

    os.makedirs(output_dir, exist_ok=True)

    # Rendezvous barrier: each robot waits for its (team size - 1) teammates.
    expected_peers = max(len(robots) - 1, 0)

    nodes = []
    for robot in robots:
        csv_name = f"exp7_{planner}_{world}_{config_id}_{robot}.csv"
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
                shared_params,
                {
                    "use_sim_time": use_sim_time,
                    "robot_name": robot,
                    "max_steps": int(max_steps),
                    "output_csv": output_csv,
                    # dscovox_topic and planning_map_topic stay at their
                    # defaults so each planner reads its own robot's fused view.
                    # Coordination on: MinPos in doPlan tests peer intents from
                    # /exploration/intents with claimMatching.
                    # (notes: launch-own-map-and-minpos)
                    "coordination_enabled":
                        coordination_enabled.lower() in ("true", "1", "yes", "on"),
                    # Reconnect subsystem master switch. False = no mid-run
                    # manoeuvre of any kind (the control arm); which manoeuvre
                    # a true runs is reconnect_mode's job. Resolved above, so
                    # only the canonical name reaches the node.
                    "reconnect_enabled":
                        reconnect_enabled.lower() in ("true", "1", "yes", "on"),
                    # expected_peers is the team size minus this robot;
                    # 0 = wait forever (STAY until all connected).
                    "rendezvous_expected_peers": expected_peers,
                    "rendezvous_max_wait_sec": float(rendezvous_max_wait_sec),
                    # List position is the robot id that in_range_mask, the
                    # knowledge bitmasks and the proposer rule (robot 0
                    # proposes) address. Required here: without fleet identity
                    # the rendezvous scheduler refuses to start.
                    # (notes: launch-ordered-fleet)
                    "team_robot_names": robots,
                    # rendezvous: drive to the team-agreed cell and time;
                    # pursuit: budgeted chase of the missing peer's trail;
                    # hybrid: chase until the agreed meeting is due, then keep
                    # it. pursuit_* budgets come from shared_params.yaml.
                    # (notes: launch-reconnect-modes)
                    "reconnect_mode": reconnect_mode,
                    # Required by rendezvous and hybrid, which are the agreed
                    # meeting; the node treats either with the scheduler off as
                    # fatal. It defaults to false in the node, so it must be
                    # passed here. (notes: launch-schedule-interlock)
                    "rendezvous_schedule_enable":
                        reconnect_mode.strip().lower() in ("rendezvous",
                                                           "hybrid"),
                    # Yield (cancel the nav goal, hold) when a lex-smaller
                    # teammate is moving nearby. Sim runs off the 1 Hz intent
                    # heartbeats; on hardware set proximity_peer_pose_topics in
                    # the yaml. (notes: launch-proximity-stop)
                    "proximity_stop_enabled":
                        proximity_stop_enabled.lower() in ("true", "1", "yes", "on"),
                },
            ],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="atlas,rama",
                              description="Comma-separated robot team list"),
        # Defaults false (hardware). The planner's 10 Hz tick runs off the node
        # clock, so use_sim_time:=true with no /clock publisher leaves the timer
        # dead and every planner silently idle. Gazebo/bag runs must pass true.
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Use /clock (sim/bag runs only)"),
        DeclareLaunchArgument("planner", default_value="eig",
                              description="Planner type: eig, entropy, frontier, random"),
        DeclareLaunchArgument("output_dir", default_value="/tmp",
                              description="Directory for per-robot CSVs"),
        DeclareLaunchArgument("config_id", default_value="c1",
                              description="Start configuration id (for CSV filename)"),
        DeclareLaunchArgument("world", default_value="flatforest",
                              description="World name (for CSV filename)"),
        # (notes: launch-max-steps-default)
        DeclareLaunchArgument("max_steps", default_value="500",
                              description="Per-robot step budget"),
        DeclareLaunchArgument("coordination_enabled", default_value="true",
                              description="Enable MinPos peer-claim deconfliction"),
        DeclareLaunchArgument("reconnect_enabled", default_value="true",
                              description="Master switch for the reconnect "
                                          "subsystem: run a manoeuvre when a "
                                          "teammate is out of comms (default on; "
                                          "set false for independent "
                                          "finish-and-stop, the control arm). "
                                          "reconnect_mode picks WHICH manoeuvre"),
        DeclareLaunchArgument("rendezvous_enabled", default_value="",
                              description="DEPRECATED alias for reconnect_enabled. "
                                          "Empty = unset; any value overrides "
                                          "reconnect_enabled and warns"),
        DeclareLaunchArgument("rendezvous_max_wait_sec", default_value="0.0",
                              description="Barrier give-up seconds (0 = wait forever)"),
        DeclareLaunchArgument("reconnect_mode", default_value="hybrid",
                              description="Mesh reconnection manoeuvre when a "
                                          "teammate goes out of comms: "
                                          "rendezvous (the whole team agreed a "
                                          "cell and a time while connected; "
                                          "drive there to arrive then), "
                                          "pursuit (budgeted chase of the "
                                          "missing peer's trail), or hybrid "
                                          "(chase while the agreed meeting is "
                                          "not due yet, keep it when it is). "
                                          "There is no 'off' mode — the off "
                                          "arm is reconnect_enabled:=false"),
        DeclareLaunchArgument("proximity_stop_enabled", default_value="true",
                              description="Coordinated proximity stop: the "
                                          "lex-larger robot of a close pair "
                                          "cancels its nav goal and holds "
                                          "until the peer clears or parks"),
        OpaqueFunction(function=launch_setup),
    ])
