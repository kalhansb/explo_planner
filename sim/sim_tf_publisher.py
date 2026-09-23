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
import math
import os
import sys

import numpy as np
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
        # POSE NOISE (fine-band pilot, POSE_NOISE=1). The published pose -- TF
        # odom->base_link AND /<robot>/odom_noisy, which the nav stack is pointed
        # at -- becomes a localisation ESTIMATE instead of ground truth:
        #   drift  : ground-truth body-frame increments are re-integrated with
        #            noise, so error accumulates along the path the way odometry
        #            / LIO drift does (yaw error bends the whole later track).
        #            sigma per increment: xy = a*sqrt(ds), yaw = b*sqrt(ds),
        #            a = POSE_DRIFT_XY [m/sqrt(m)], b = POSE_DRIFT_YAW [deg/sqrt(m)].
        #   jitter : first-order Gauss-Markov on (x, y, yaw), stationary sigma
        #            POSE_JITTER_XY [m] / POSE_JITTER_YAW [deg], correlation
        #            time POSE_JITTER_TAU [s]. Not integrated.
        # z / roll / pitch stay true (flat world). Seeded per robot for replay.
        self.noise = os.environ.get("POSE_NOISE", "0") == "1"
        if self.noise:
            f = lambda k, d: float(os.environ.get(k, d))
            self.a = f("POSE_DRIFT_XY", 0.01)
            self.b = math.radians(f("POSE_DRIFT_YAW", 0.01))
            self.jxy = f("POSE_JITTER_XY", 0.01)
            self.jyaw = math.radians(f("POSE_JITTER_YAW", 0.1))
            self.tau = f("POSE_JITTER_TAU", 0.5)
            seed = int(os.environ.get("POSE_NOISE_SEED", "1"))
            self.rng = np.random.default_rng([seed, sum(map(ord, robot))])
            self.prev_gt = None          # (x, y, yaw, t)
            self.est = None              # integrated drifted (x, y, yaw)
            self.jit = np.zeros(3)
            self.dist = 0.0
            self.last_log = -1e9
            self.pub_noisy = self.create_publisher(Odometry, f"/{robot}/odom_noisy", 50)
            self.get_logger().info(
                f"POSE NOISE on: drift xy {self.a} m/sqrt(m), yaw "
                f"{math.degrees(self.b)} deg/sqrt(m); jitter {self.jxy} m, "
                f"{math.degrees(self.jyaw)} deg, tau {self.tau} s; seed {seed}")

    @staticmethod
    def _yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _noisy(self, msg):
        """Return (x, y, yaw) estimate for this ground-truth message."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        g = (p.x, p.y, self._yaw(q), msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if self.prev_gt is None:
            self.prev_gt, self.est = g, np.array(g[:3])
            return tuple(self.est)
        x0, y0, th0, t0 = self.prev_gt
        dx, dy = g[0] - x0, g[1] - y0
        c, s = math.cos(th0), math.sin(th0)
        bx, by = c * dx + s * dy, -s * dx + c * dy          # body-frame increment
        dth = math.atan2(math.sin(g[2] - th0), math.cos(g[2] - th0))
        ds = math.hypot(bx, by)
        self.dist += ds
        if ds > 0.0:
            sd = math.sqrt(ds)
            bx += self.rng.normal(0.0, self.a * sd)
            by += self.rng.normal(0.0, self.a * sd)
            dth += self.rng.normal(0.0, self.b * sd)
        ex, ey, eth = self.est
        ce, se = math.cos(eth), math.sin(eth)
        self.est = np.array([ex + ce * bx - se * by, ey + se * bx + ce * by, eth + dth])
        dt = max(g[3] - t0, 0.0)
        if dt > 0.0 and self.tau > 0.0:
            k = math.exp(-dt / self.tau)
            sig = np.array([self.jxy, self.jxy, self.jyaw])
            self.jit = self.jit * k + sig * math.sqrt(1.0 - k * k) * self.rng.normal(size=3)
        self.prev_gt = g
        out = self.est + self.jit
        if g[3] - self.last_log >= 10.0:
            self.last_log = g[3]
            self.get_logger().info(
                "pose error t=%.1f dist=%.1f m: dx=%+.3f dy=%+.3f dyaw=%+.3f deg (drift |xy|=%.3f)" % (
                    g[3], self.dist, out[0] - g[0], out[1] - g[1],
                    math.degrees(math.atan2(math.sin(out[2] - g[2]), math.cos(out[2] - g[2]))),
                    math.hypot(self.est[0] - g[0], self.est[1] - g[1])))
        return tuple(out)

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
        if self.noise:
            x, y, yaw = self._noisy(msg)
            q = msg.pose.pose.orientation
            # Replace only the yaw: rotate the true attitude by the yaw error.
            dyaw = yaw - self._yaw(q)
            cz, sz = math.cos(dyaw / 2.0), math.sin(dyaw / 2.0)
            q2 = type(q)()
            q2.w = cz * q.w - sz * q.z
            q2.x = cz * q.x - sz * q.y
            q2.y = cz * q.y + sz * q.x
            q2.z = cz * q.z + sz * q.w
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = q2
            n = Odometry()
            n.header = msg.header
            n.child_frame_id = msg.child_frame_id
            n.pose.pose.position.x = x
            n.pose.pose.position.y = y
            n.pose.pose.position.z = msg.pose.pose.position.z
            n.pose.pose.orientation = q2
            n.pose.covariance = msg.pose.covariance
            n.twist = msg.twist             # body-frame velocity: unaffected
            self.pub_noisy.publish(n)
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
