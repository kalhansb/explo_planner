#!/usr/bin/env python3
"""Measure a platform's motion envelope from a rosbag's /tf stream.

Sizes explo_planner's motion-derived parameters from a recording of the real
robot driving. No planner has to be running in the bag: only /tf and
/tf_static are read. The script composes <base_frame> up the chain to its root
(map -> odom -> base_link, whatever the frames are called on that platform)
and prints, next to each figure, the parameter it sizes:

  LINEAR mean                 -> nav_speed_estimate_mps
  YAWRATE p90 (180 deg turn)  -> goal_rotate_timeout_sec
  longest standstill, min travel in any W-second window
                              -> progress_window_sec, progress_min_distance_m
  single-tick jumps per TF edge
                              -> max_pose_jump_m (and a doubled-publisher check)

Usage, inside the scovox container where rosbag2_py is installed:

  python3 measure_bag_motion.py <bag_dir> <base_frame> [--windows 15,30,45] [--only-3d]

Run it once per robot with that robot's base frame. --only-3d drops a planar
odometry publisher (z == 0, yaw-only) that shares the odom->base edge with the
real one, which is how the bunker recording had to be read. The EDGES section
reports that fault either way. docs/real_robot_tuning.md explains how each
number became a value in explo_planner/config/exploration_real_robot.yaml.
"""
import argparse
import math
import sys
from collections import defaultdict

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from tf2_msgs.msg import TFMessage


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def qrot(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pct(vals, p):
    v = sorted(vals)
    return v[min(len(v) - 1, int(p / 100 * len(v)))] if v else float('nan')


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='rosbag2 directory')
    ap.add_argument('base_frame', help="the robot's base frame in this bag, e.g. base_link_curt")
    ap.add_argument('--storage', default='mcap')
    ap.add_argument('--windows', default='10,15,20,30,45,60',
                    help='candidate progress_window_sec values to evaluate')
    ap.add_argument('--moving', type=float, default=0.05,
                    help='speed (m/s) below which a sample counts as stationary')
    ap.add_argument('--only-3d', action='store_true',
                    help='ignore base-frame transforms with z == 0 and a yaw-only '
                         'rotation: the signature of a planar odometry publisher '
                         'sharing the odom->base edge with the real one')
    args = ap.parse_args()
    target = args.base_frame.lstrip('/')
    windows = [float(w) for w in args.windows.split(',')]

    def open_reader(topics):
        rd = SequentialReader()
        rd.open(StorageOptions(uri=args.bag, storage_id=args.storage),
                ConverterOptions('cdr', 'cdr'))
        rd.set_filter(StorageFilter(topics=topics))
        return rd

    edges = {}                  # child -> (parent, trans, quat): latest transform
    series = defaultdict(list)  # (parent, child) -> [(t, x, y)] for dynamic edges
    poses = []                  # (t, x, y, yaw) of the base frame in the chain root

    def compose_to_root(child):
        t, q = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        seen, cur = set(), child
        while cur in edges and cur not in seen:
            seen.add(cur)
            parent, pt, pq = edges[cur]
            t = tuple(a + b for a, b in zip(qrot(pq, t), pt))
            q = qmul(pq, q)
            cur = parent
        return t, q, cur

    # Static transforms first. A latched /tf_static message can sit anywhere
    # in the recording; applying it mid-stream would shift every later pose
    # by its offset and show up as one huge fake jump.
    static_reader = open_reader(['/tf_static'])
    while static_reader.has_next():
        _topic, data, _ = static_reader.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            tl, rq = tr.transform.translation, tr.transform.rotation
            edges[tr.child_frame_id.lstrip('/')] = (
                tr.header.frame_id.lstrip('/'),
                (tl.x, tl.y, tl.z), (rq.x, rq.y, rq.z, rq.w))

    dropped = 0
    reader = open_reader(['/tf'])
    while reader.has_next():
        _topic, data, _ = reader.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            c = tr.child_frame_id.lstrip('/')
            p = tr.header.frame_id.lstrip('/')
            tl, rq = tr.transform.translation, tr.transform.rotation
            if (args.only_3d and c == target
                    and tl.z == 0.0 and rq.x == 0.0 and rq.y == 0.0):
                dropped += 1
                continue
            edges[c] = (p, (tl.x, tl.y, tl.z), (rq.x, rq.y, rq.z, rq.w))
            stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            series[(p, c)].append((stamp, tl.x, tl.y))
            if c == target:
                (x, y, _z), q, root = compose_to_root(target)
                poses.append((stamp, x, y, yaw_of(q), root))

    # Until every edge above the base frame has been seen once, the chain
    # ends early and those poses live in a different frame; drop them.
    _, _, root = compose_to_root(target)
    poses = [(t, x, y, Y) for t, x, y, Y, r in poses if r == root]

    # Bags can hold TF messages out of stamp order; differencing in recording
    # order would turn each one into a spurious jump.
    poses.sort(key=lambda r: r[0])
    for v in series.values():
        v.sort(key=lambda r: r[0])
    if not poses:
        print(f"no /tf poses for '{target}'. child frames seen: {sorted(edges)}",
              file=sys.stderr)
        return 1

    chain, cur = [], target
    while cur in edges and cur != root:
        parent = edges[cur][0]
        chain.append((parent, cur))
        cur = parent
    t0 = poses[0][0]
    dur = poses[-1][0] - t0

    lin, ang, path = [], [], 0.0
    for (ta, xa, ya, Ya), (tb, xb, yb, Yb) in zip(poses, poses[1:]):
        dt = tb - ta
        if dt < 0.005 or dt > 1.0:      # duplicate stamps and gaps: no speed
            continue
        d = math.hypot(xb - xa, yb - ya)
        dY = (Yb - Ya + math.pi) % (2 * math.pi) - math.pi
        path += d
        lin.append((tb - t0, d / dt))
        ang.append(abs(dY) / dt)
    lv = [v for _, v in lin]
    turn_p90 = math.pi / max(pct(ang, 90), 1e-9)

    print(f"chain: {' -> '.join([target] + [p for p, _ in chain])}")
    print(f"  samples={len(poses)}  rate={len(poses) / max(dur, 1e-9):.1f} Hz  "
          f"dur={dur:.0f} s  path={path:.1f} m"
          + (f"  (dropped {dropped} planar samples)" if args.only_3d else ""))
    print(f"LINEAR   mean={path / dur:.3f}  p50={pct(lv, 50):.3f}  p90={pct(lv, 90):.3f}  "
          f"p99={pct(lv, 99):.3f}  max={max(lv):.2f} m/s")
    print(f"           -> nav_speed_estimate_mps = mean (wall-clock, standstills included)")
    print(f"YAWRATE  p50={pct(ang, 50):.3f}  p90={pct(ang, 90):.3f}  p99={pct(ang, 99):.3f}  "
          f"max={max(ang):.2f} rad/s   180 deg turn at p90 = {turn_p90:.1f} s")
    print(f"           -> goal_rotate_timeout_sec >= one turn + settle margin")

    still = sum(1 for _, v in lin if v < args.moving)
    best = run = 0.0
    prev = None
    for t, v in lin:
        if prev is not None and v < args.moving:
            run += t - prev
            best = max(best, run)
        else:
            run = 0.0
        prev = t
    print(f"STILL    {100 * still / len(lin):.0f}% of samples below {args.moving} m/s;  "
          f"longest continuous standstill = {best:.1f} s")
    print(f"           -> progress_window_sec > longest standstill + one 180 deg turn")

    cum, run = [(0.0, 0.0)], 0.0
    for (ta, xa, ya, _), (tb, xb, yb, _) in zip(poses, poses[1:]):
        run += math.hypot(xb - xa, yb - ya)
        cum.append((tb - t0, run))
    print("WINDOW   min travel in any sliding window (the watchdog fires below this):")
    for W in windows:
        worst, j = float('inf'), 0
        for ti, ci in cum:
            while j < len(cum) and cum[j][0] < ti + W:
                j += 1
            if j >= len(cum):
                break
            worst = min(worst, cum[j][1] - ci)
        if worst < float('inf'):
            print(f"           {W:>5.0f} s -> {worst:.2f} m")
    print(f"           -> progress_min_distance_m <= about half the figure at your window")

    print("EDGES    single-tick translation jumps per dynamic TF edge in the chain:")
    for p, c in chain:
        s = sorted(series.get((p, c), []))
        if len(s) < 2:
            print(f"           {p}->{c}: static")
            continue
        jumps = [math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(s, s[1:])]
        rate = len(s) / max(s[-1][0] - s[0][0], 1e-9)
        big = sum(1 for j in jumps if j > 1.0)
        print(f"           {p}->{c}: n={len(s)} rate={rate:.1f} Hz  "
              f"p99={pct(jumps, 99):.3f}  max={max(jumps):.3f} m  (>1 m: {big})")
    print("           -> max_pose_jump_m above the p99 of the localiser's correction edge;")
    print("              metre-scale jumps on odom->base mean TWO publishers on that edge")
    return 0


if __name__ == '__main__':
    sys.exit(main())
