#!/usr/bin/env python3
"""Draw released tree targets in RViz.

explo_planner's exploitation targets travel as explo_planner_msgs/TreeTarget,
which RViz cannot display, so on a live run the only visible sign that a target
was released is the vantage ring appearing in /<robot>/explo_planner/candidates
— i.e. you see the robot circling something, but not what. This node turns each
TreeTarget into markers:

  * a translucent trunk cylinder at (x, y), height = msg.height (fallback 6.0 m);
  * a ring at the vantage standoff, msg.radius + --standoff (the same sum the
    planner uses to place its vantages: standoff = radius + vantage_standoff_m,
    default 2.0 in shared_params.yaml) — the circle the robot should drive;
  * a text label "target <id>".

Publishes LATCHED (transient_local) so RViz can be started, restarted or
attached late and still see every target released so far, matching the
scheduler's own latched publisher. Read-only: it subscribes and draws, and
changes nothing about the run.

Usage:  python3 sim_target_markers.py [--standoff 2.0]
        [--targets-topic /exploration/targets]
        [--markers-topic /exploration/target_markers]
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from explo_planner_msgs.msg import TreeTarget


def latched_qos(depth=50):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class TargetMarkers(Node):
    def __init__(self, args):
        super().__init__("sim_target_markers")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.standoff = args.standoff
        # id -> (marker id base). Targets are deduped by id: the scheduler
        # publishes each once, but the topic is latched and a re-released or
        # re-reported target must overwrite, not stack.
        self.seen = {}
        self.pub = self.create_publisher(
            MarkerArray, args.markers_topic, latched_qos())
        self.sub = self.create_subscription(
            TreeTarget, args.targets_topic, self.on_target, latched_qos())
        self.get_logger().info(
            f"drawing {args.targets_topic} -> {args.markers_topic} "
            f"(vantage standoff = radius + {self.standoff:.1f} m)")

    def on_target(self, msg: TreeTarget):
        frame = msg.header.frame_id or "map"
        height = msg.height if msg.height > 0.0 else 6.0
        ring_r = max(msg.radius, 0.0) + self.standoff
        base_id = int(msg.target_id) * 10
        stamp = self.get_clock().now().to_msg()
        first = msg.target_id not in self.seen
        self.seen[msg.target_id] = True

        ma = MarkerArray()

        trunk = Marker()
        trunk.header.frame_id = frame
        trunk.header.stamp = stamp
        trunk.ns = "tree_targets"
        trunk.id = base_id
        trunk.type = Marker.CYLINDER
        trunk.action = Marker.ADD
        trunk.pose.position.x = msg.center.x
        trunk.pose.position.y = msg.center.y
        trunk.pose.position.z = msg.center.z + height / 2.0
        trunk.pose.orientation.w = 1.0
        # Drawn at the clean-trunk diameter, NOT msg.radius: radius is the
        # line-of-sight/standoff radius (1.8 m in targets_flatforest.yaml, sized
        # to cover the root flare), and a 3.6 m cylinder would hide the trunk it
        # is meant to mark. The ring below carries the radius.
        trunk.scale.x = trunk.scale.y = 0.8
        trunk.scale.z = height
        trunk.color.r, trunk.color.g, trunk.color.b, trunk.color.a = 1.0, 0.55, 0.0, 0.35
        ma.markers.append(trunk)

        ring = Marker()
        ring.header.frame_id = frame
        ring.header.stamp = stamp
        ring.ns = "tree_targets"
        ring.id = base_id + 1
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.pose.orientation.w = 1.0
        ring.scale.x = 0.08
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = 1.0, 0.55, 0.0, 0.9
        # Ring drawn at 0.3 m — the sightline height the planner tests LoS at,
        # so a ring segment buried in mapped ground is itself informative.
        for i in range(49):
            a = 2.0 * math.pi * i / 48.0
            ring.points.append(Point(x=msg.center.x + ring_r * math.cos(a),
                                     y=msg.center.y + ring_r * math.sin(a),
                                     z=msg.center.z + 0.3))
        ma.markers.append(ring)

        label = Marker()
        label.header.frame_id = frame
        label.header.stamp = stamp
        label.ns = "tree_targets"
        label.id = base_id + 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = msg.center.x
        label.pose.position.y = msg.center.y
        label.pose.position.z = msg.center.z + height + 0.8
        label.pose.orientation.w = 1.0
        label.scale.z = 1.0
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f"target {msg.target_id}"
        ma.markers.append(label)

        self.pub.publish(ma)
        if first:
            self.get_logger().info(
                f"target {msg.target_id} released at "
                f"({msg.center.x:.2f}, {msg.center.y:.2f}) "
                f"radius={msg.radius:.2f} ring={ring_r:.2f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--standoff", type=float, default=2.0,
                    help="vantage_standoff_m from shared_params.yaml")
    ap.add_argument("--targets-topic", default="/exploration/targets")
    ap.add_argument("--markers-topic", default="/exploration/target_markers")
    args = ap.parse_args()

    rclpy.init()
    node = TargetMarkers(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # ExternalShutdownException is what rclpy raises when the orchestrator
        # SIGINTs the process group at teardown — a normal exit, not a fault.
        pass


if __name__ == "__main__":
    main()
