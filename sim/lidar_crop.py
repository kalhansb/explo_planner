#!/usr/bin/env python3
"""Republish a robot's lidar cloud with every return beyond a range cap set to +inf.

The Husky lidar now renders to 100 m (VLP-16 datasheet), but mapping and
exploration must keep behaving as they did under the generation-9 25 m far clip,
where a surface beyond 25 m gave no return (a non-finite point). This node
reproduces that in software: /<robot>/velodyne_points -> /<robot>/velodyne_points_25m,
same header, fields, layout and point order; x, y, z of any point whose range from
the sensor origin exceeds --max-range become +inf. Detection reads the original
topic, never this one.

Both consumers (scovox_node and the nav costmap) subscribe best-effort; a reliable
publisher serves either.

Usage: lidar_crop.py <robot> [--max-range 25.0] [--ros-args ...]
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class LidarCrop(Node):
    def __init__(self, robot, max_range):
        super().__init__(f"lidar_crop_{robot}")
        self.max_r2 = max_range * max_range
        self.offsets = None
        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        pub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, f"/{robot}/velodyne_points_25m", pub_qos)
        self.sub = self.create_subscription(PointCloud2, f"/{robot}/velodyne_points",
                                            self.on_cloud, sub_qos)
        self.get_logger().info(f"cropping /{robot}/velodyne_points at {max_range} m "
                               f"-> /{robot}/velodyne_points_25m")

    def on_cloud(self, msg):
        if self.offsets is None:
            f = {fl.name: fl for fl in msg.fields}
            if not all(k in f and f[k].datatype == 7 for k in "xyz"):   # FLOAT32
                self.get_logger().error("cloud has no float32 x/y/z; passing through")
                self.offsets = ()
            else:
                self.offsets = tuple(f[k].offset for k in "xyz")
        if self.offsets:
            n = msg.width * msg.height
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).copy()
            rows = buf.reshape(n, msg.point_step)
            xyz = np.stack([rows[:, o:o + 4].copy().view(np.float32)[:, 0] for o in self.offsets], 1)
            with np.errstate(invalid="ignore", over="ignore"):
                far = (xyz * xyz).sum(1) > self.max_r2
            if far.any():
                inf = np.array([np.inf], dtype=np.float32).view(np.uint8)
                for o in self.offsets:
                    rows[far, o:o + 4] = inf
                msg.data = buf.tobytes()
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot")
    ap.add_argument("--max-range", type=float, default=25.0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init(args=sys.argv)
    node = LidarCrop(args.robot, args.max_range)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
