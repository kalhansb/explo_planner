#!/usr/bin/env python3
"""TF glue for hmr_sim + scovox/simple_nav_3d native runs.

hmr_sim bridges sensor/odom topics but no TF (the old hmr_explore workspace
provided it). This node fills the gap for one robot:
  - dynamic:  <robot>/odom -> <robot>/base_link   from /<robot>/odom_ground_truth
  - static:   map -> <robot>/odom                 identity (odom origin is the
              gz world origin; ground plane at z=0)
  - static:   <robot>/base_link -> <robot>/realsense  camera mount from the
              COSTAR husky SDF (0.438, 0, 0.272, rpy 0 0 0 — body convention;
              SCovoxNode applies the optical->body rotation kR internally)
  - static:   <robot>/base_link -> <robot>/velodyne   lidar mount from the
              COSTAR husky SDF (0.0012, 0, 0.716, rpy 0 0 0). Lidar clouds are
              already body-convention — no optical rotation needed.

map -> odom identity is correct for every robot regardless of spawn point: the
gz OdometryPublisher system reports the model's absolute world pose, so
<robot>/odom IS the world frame (verified: bestla spawned at y=3 reports y≈3).
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SimTfPublisher(Node):
    def __init__(self, robot: str):
        super().__init__(f"{robot}_sim_tf_publisher")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.robot = robot
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, f"/{robot}/odom_ground_truth", self.on_odom, 50)
        self.statics_sent = False

    def send_statics(self, stamp):
        m2o = TransformStamped()
        m2o.header.stamp = stamp
        m2o.header.frame_id = "map"
        m2o.child_frame_id = f"{self.robot}/odom"
        m2o.transform.rotation.w = 1.0

        b2c = TransformStamped()
        b2c.header.stamp = stamp
        b2c.header.frame_id = f"{self.robot}/base_link"
        b2c.child_frame_id = f"{self.robot}/realsense"
        b2c.transform.translation.x = 0.438
        b2c.transform.translation.z = 0.272
        b2c.transform.rotation.w = 1.0

        b2l = TransformStamped()
        b2l.header.stamp = stamp
        b2l.header.frame_id = f"{self.robot}/base_link"
        b2l.child_frame_id = f"{self.robot}/velodyne"
        b2l.transform.translation.x = 0.0012
        b2l.transform.translation.z = 0.716
        b2l.transform.rotation.w = 1.0

        self.static_tf.sendTransform([m2o, b2c, b2l])
        self.statics_sent = True
        self.get_logger().info(
            "static TFs published (map->odom, base->realsense, base->velodyne)")

    def on_odom(self, msg: Odometry):
        if not self.statics_sent:
            self.send_statics(msg.header.stamp)
        t = TransformStamped()
        t.header = msg.header               # <robot>/odom, sim stamp
        t.child_frame_id = msg.child_frame_id   # <robot>/base_link
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(t)


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else "atlas"
    rclpy.init()
    node = SimTfPublisher(robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
