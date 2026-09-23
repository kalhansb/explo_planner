#!/usr/bin/env python3
"""Register fixed fine-TSDF refinement regions on every robot's scovox_node.

Fine-band pilot: both arms get the SAME regions for the WHOLE run, so the only
difference between arms is where the robots drive. The planner's own
region publishing (publish_refinement_regions) is off in the run script; it
registers on target release and removes on close, which would give the on arm
fine integration the off arm never gets.

Each region is published once per robot on /<robot>/scovox_node/refinement_region
with reliable + transient_local QoS (depth 100), matching the node's
subscription, so a scovox_node that starts later still receives them. The node
keeps spinning because transient_local only holds while the writer is alive.

Usage: fine_region_registrar.py --robots atlas,bestla \
         --region 1,9.8917,8.2438,0.40 --region 2,... [--base-z 0.0]
"""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from scovox_msgs.msg import RefinementRegion


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", required=True)
    ap.add_argument("--region", action="append", required=True,
                    help="id,x,y,radius (world == <robot>/odom frame)")
    ap.add_argument("--base-z", type=float, default=0.0)
    a, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node("fine_region_registrar")
    qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    regions = []
    for spec in a.region:
        rid, x, y, r = spec.split(",")
        regions.append((int(rid), float(x), float(y), float(r)))
    pubs = []
    for robot in [s for s in a.robots.split(",") if s]:
        topic = f"/{robot}/scovox_node/refinement_region"
        pub = node.create_publisher(RefinementRegion, topic, qos)
        pubs.append(pub)
        for rid, x, y, r in regions:
            m = RefinementRegion()
            m.id, m.x, m.y, m.base_z, m.radius, m.remove = rid, x, y, a.base_z, r, False
            pub.publish(m)
            node.get_logger().info(
                f"registered region {rid} on {topic}: xy=({x:.3f},{y:.3f}) "
                f"base_z={a.base_z:.2f} radius={r:.3f}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
