#!/usr/bin/env python3
"""Log hmr_comms_sim's ~/link_states to CSV, one row per pair per tick.

Exists because the calibration sweep (plan §4) needs the raw connectivity trace
and nothing else in the stack writes one: the emulator publishes link_states at
link_rate_hz but only ever holds it in memory, the gate watcher reads the
aggregate `stats` topic instead, and `ros2 bag record` would give a bag that
still has to be decoded to get at the numbers.

Columns per pair, as documented on ~/robot_index:
    [i, j, distance_m, trees_on_link, path_loss_db, snr_db, ber,
     bandwidth_mbps, connected]

STARTUP MASK (plan §5.3): link_states has no validity column. Before a pair has
received both poses the emulator still publishes a row, and that row reads
connected=0 with every physical field zeroed. Counted naively those rows inflate
the disconnected duty cycle and, worse, the transition out of them looks like a
reconnection event at t=0. path_loss_db is the discriminator: the model's floor
is p0_db (49.17) at one metre and it only grows, so a row with path_loss_db <= 0
is a row the emulator never actually computed. Masked rows are counted and
reported rather than silently dropped -- if most of a trace is masked, the trace
is not evidence of anything.
"""

import argparse
import csv
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

FIELDS = 9


class LinkLogger(Node):
    def __init__(self, topic, out_path):
        super().__init__("link_logger")
        # use_sim_time is set from the command line by the caller; the stamps
        # written here are therefore SIM seconds, the same clock the planner
        # CSVs use, which is what makes the two joinable after the fact.
        self.out = open(out_path, "w", newline="")
        self.w = csv.writer(self.out)
        self.w.writerow(["t_sim", "i", "j", "distance_m", "trees_on_link",
                         "path_loss_db", "snr_db", "ber", "bandwidth_mbps",
                         "connected"])
        self.rows = 0
        self.masked = 0
        self.sub = self.create_subscription(
            Float64MultiArray, topic, self.cb, 50)
        self.get_logger().info(f"logging {topic} -> {out_path}")

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        d = list(msg.data)
        for k in range(0, len(d) - FIELDS + 1, FIELDS):
            row = d[k:k + FIELDS]
            if row[4] <= 0.0:          # path_loss_db -- see the startup mask note
                self.masked += 1
                continue
            self.w.writerow([f"{t:.3f}"] + [f"{v:g}" for v in row])
            self.rows += 1
        if self.rows and self.rows % 500 == 0:
            self.out.flush()

    def destroy_node(self):
        self.out.flush()
        self.out.close()
        # Goes to stderr so the caller can separate it from the CSV path on
        # stdout, and it is the only place the mask count is reported.
        print(f"[link_logger] rows={self.rows} masked_startup={self.masked}",
              file=sys.stderr, flush=True)
        return super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/hmr_comms_sim/link_states")
    ap.add_argument("--out", required=True)
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=sys.argv)
    node = LinkLogger(args.topic, args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
