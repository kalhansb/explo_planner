"""traversability_mapping, map side only: the planner's planning-map source.

Starts traversability_filter + traversability_map from the traversability_mapping
package (github.com/errorcodecritical/traversability_mapping) with its own
config, and NOT its traversability_prm / traversability_path nodes. Those two
are a planner of their own: they run on every map message whether or not a
goal is set, and grow without bound over a run (see docs/field_setup.md,
"Traversability map"). The explo_planner real-robot overlay consumes
/<robot>/occupancy_map_local with planning_map_mode: accumulate.

One instance per robot, each in the robot's namespace (topics and frames as
recorded on 2026-07-31):

  ros2 launch explo_planner traversability_map.launch.py namespace:=bunker \
       input_cloud_topic:=/hesai/points
  ros2 launch explo_planner traversability_map.launch.py namespace:=curt \
       base_frame:=base_link_curt lidar_frame:=os_lidar \
       input_cloud_topic:=/ouster/points

traversability_mapping's topics are absolute parameters, so a ROS namespace
alone does not move them: with namespace set, every *_topic in the params
file except input_cloud_topic is prefixed with /<namespace> here (the map
lands on /bunker/occupancy_map_local), and input_cloud_topic is taken as
given. Frames are not prefixed: both robots share one TF tree and map frame,
so give each its own base_frame / lidar_frame instead.

At 10 Hz input each node takes most of a desktop core; feed a 5 Hz cloud on
the Jetson (docs/field_setup.md, "Traversability map").

Float overrides must keep their decimal point (12.0, not 12): the nodes read
them with as_double() and die at startup on an integer.
"""
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_ARGS = {
    "use_sim_time": ("false", "true for bag replay (play with --clock)"),
    "input_cloud_topic": ("/hesai/points_decimated",
                          "LiDAR PointCloud2 (traversability_mapping default)"),
    "params_file": ("", "traversability_mapping params; empty = its shipped "
                        "config/traversability_mapping.yaml"),
    "namespace": ("", "Robot namespace (e.g. bunker): the nodes run in it and "
                      "every *_topic of params_file except input_cloud_topic "
                      "gets a /<namespace> prefix. Empty = the params file's "
                      "un-namespaced topics"),
    "base_frame": ("", "Override (e.g. base_link_curt); empty = params file"),
    "lidar_frame": ("", "Override (the LiDAR's TF frame); empty = params "
                        "file"),
    "sensor_range_limit": ("", "Override (m, e.g. 12.0); empty = params file. "
                               "Scales the filter's CPU and the map's memory "
                               "growth"),
    "prediction_enable": ("", "Override (true/false); empty = params file. "
                              "false halves the CPU but makes the map "
                              "noisier; a 5 Hz input is the better lever"),
}


def _namespaced_topics(params_file, ns):
    """Every *_topic string in params_file (any node block) under /<ns>,
    except input_cloud_topic, which the caller sets explicitly."""
    with open(params_file) as f:
        doc = yaml.safe_load(f) or {}
    out = {}
    for block in doc.values():
        params = (block or {}).get("ros__parameters") or {}
        for key, value in params.items():
            if (key.endswith("_topic") and key != "input_cloud_topic"
                    and isinstance(value, str)):
                out[key] = "/" + ns + "/" + value.lstrip("/")
    return out


def _setup(context):
    g = {k: LaunchConfiguration(k).perform(context) for k in _ARGS}
    params_file = g["params_file"] or (
        FindPackageShare("traversability_mapping").perform(context)
        + "/config/traversability_mapping.yaml")
    ns = g["namespace"].strip("/")
    overrides = {
        "use_sim_time": g["use_sim_time"].lower() in ("1", "true", "yes", "on"),
        "input_cloud_topic": g["input_cloud_topic"],
    }
    if ns:
        overrides.update(_namespaced_topics(params_file, ns))
    for frame in ("base_frame", "lidar_frame"):
        if g[frame]:
            overrides[frame] = g[frame]
    if g["sensor_range_limit"]:
        overrides["sensor_range_limit"] = float(g["sensor_range_limit"])
    if g["prediction_enable"]:
        overrides["prediction_enable"] = (
            g["prediction_enable"].lower() in ("1", "true", "yes", "on"))
    extra = {"namespace": ns} if ns else {}
    return [
        Node(package="traversability_mapping", executable=exe, name=exe,
             output="screen", parameters=[params_file, overrides], **extra)
        for exe in ("traversability_filter", "traversability_map")
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=desc)
         for k, (d, desc) in _ARGS.items()]
        + [OpaqueFunction(function=_setup)])
