#!/usr/bin/env python3
# Moved comments: docs/sim_notes/link_logger_notes.md
"""Log hmr_comms_sim's ~/link_states to CSV, one row per pair per tick.

Exists because the calibration sweep (plan §4) needs the raw connectivity trace
and nothing else in the stack writes one: the emulator publishes link_states at
link_rate_hz but only ever holds it in memory, the gate watcher reads the
aggregate `stats` topic instead, and `ros2 bag record` would give a bag that
still has to be decoded to get at the numbers.

Columns per pair, as documented on ~/robot_index:
    [i, j, distance_m, trees_on_link, path_loss_db, snr_db, ber,
     bandwidth_mbps, connected, valid]

VALIDITY MASK (plan §5.3). A row the emulator did not actually compute must not
be counted: before a pair has both poses, and -- since generation 9 -- whenever
either endpoint's pose has gone stale, the emulator still publishes a row at
link_rate_hz, and that row reads connected=0 with every physical field zeroed.
Counted naively those rows inflate the disconnected duty cycle and, worse, the
transition out of them looks like a reconnection event.

Two discriminators, in order of preference:

  valid == 0     Generation 9 added this column, and it is the emulator's own
                 answer rather than an inference from one.
  path_loss_db   The pre-generation-9 fallback, used when the emulator on the
  <= 0           wire is older than this script. The model's floor is p0_db
                 (49.17) at one metre and it only grows, so a non-positive path
                 loss is a row nobody computed. It is a sound test, but it is
                 still a proxy: it reads the zeroing, not the invalidity.

The two agree by construction -- an invalid row is published zeroed precisely so
that the old mask keeps working -- so the fallback costs nothing but is reported
separately, because a trace logged against a stale emulator has no stale-pose
detection in it at all and should not be read as if it did.

Masked rows are counted and reported rather than silently dropped: if most of a
trace is masked, the trace is not evidence of anything.

AND "REPORTED" HAS TO MEAN REPORTED SOMEWHERE A READER LOOKS. Until 2026-09-18
the tally went to stderr and nowhere else. The harness redirects that to
<cell>/link_logger.log, no script in the tree opens that file, and the seven
scripts that DO read link_states.csv see only the survivors -- a trace that was
95 % masked is indistinguishable from a short one, and the sentence above was a
property nothing enforced. So the counts are also written to a sidecar,
<out-stem>.mask, in the harness's key=value shape. It is written at startup,
again every 500 published rows, and once more on shutdown, so a cell killed at
teardown still has one and an ABSENT sidecar means this node never started
rather than "we do not know". analyze_runs.py reads it. A high mask rate is
additionally logged at ERROR rather than INFO, because the one thing it must
not look like in a console log is normal progress.
"""

import argparse
import csv
import os
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Generation 9 publishes 10 columns. The stride is read off the message layout
# rather than assumed, because a workspace where hmr_sim is stale and this script
# is not would otherwise write a CSV whose columns are silently shifted by one
# every row -- the one failure mode worth code to avoid.
FIELDS = 10
FIELDS_LEGACY = 9
VALID_NOT_REPORTED = -1   # sentinel: a pre-generation-9 emulator, not a reading

# Mask-rate bands for the sidecar verdict, not tuning knobs. Some masking is
# expected (every pair is masked until both endpoints have a pose); the bands
# separate that startup burst from a pose pipeline that stayed broken.
# (notes: link-logger-mask-bands)
MASK_SUSPECT = 0.10
MASK_UNUSABLE = 0.50


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
                         "connected", "valid"])
        self.rows = 0
        self.masked = 0
        self.masked_by_valid = 0
        self.masked_by_path_loss = 0
        self.last_sidecar_at = 0
        self.stride = None
        # <out-stem>.mask, not <out>.mask: link_states.csv.mask reads like a
        # variant of the CSV and would be picked up by anything globbing
        # link_states* for traces.
        self.mask_path = os.path.splitext(out_path)[0] + ".mask"
        # Written at startup so the sidecar exists however the cell ends (a
        # SIGKILL reaches neither destroy_node() nor cb()); an absent sidecar
        # means this node never started. The initial rows=0 masked=0 reads
        # NO_ROWS. (notes: link-logger-sidecar-at-startup)
        self.write_sidecar()
        self.sub = self.create_subscription(
            Float64MultiArray, topic, self.cb, 50)
        self.get_logger().info(f"logging {topic} -> {out_path}")

    def mask_verdict(self):
        """(fraction, verdict) over every row the emulator published."""
        published = self.rows + self.masked
        if not published:
            # No denominator, so no rate -- and that is a THIRD state, not a
            # clean one. A cell whose emulator published nothing at all has no
            # connectivity evidence in it, which is the same practical answer
            # as a fully masked trace and must not be reported as 0.0 % masked.
            return None, "NO_ROWS"
        frac = self.masked / published
        if frac >= MASK_UNUSABLE:
            return frac, "UNUSABLE"
        if frac >= MASK_SUSPECT:
            return frac, "SUSPECT"
        return frac, "OK"

    def write_sidecar(self):
        """Rewrite the tally sidecar. Called as the run goes, not just at exit.

        destroy_node() is not guaranteed to run: the harness stops cells with a
        signal and a cell that is killed outright never reaches it. Writing this
        on every flush costs one small file rewrite per 500 rows and means the
        tally survives the teardown paths that lose stderr entirely.
        """
        frac, verdict = self.mask_verdict()
        try:
            with open(self.mask_path, "w") as fh:
                fh.write(f"rows={self.rows}\n")
                fh.write(f"masked={self.masked}\n")
                fh.write(f"masked_by_valid={self.masked_by_valid}\n")
                fh.write(f"masked_by_path_loss={self.masked_by_path_loss}\n")
                fh.write(f"published={self.rows + self.masked}\n")
                fh.write(f"masked_frac={'' if frac is None else f'{frac:.6f}'}\n")
                fh.write(f"mask_verdict={verdict}\n")
                fh.write(f"stride={self.stride}\n")
                fh.write(f"suspect_at={MASK_SUSPECT}\n")
                fh.write(f"unusable_at={MASK_UNUSABLE}\n")
        except OSError as e:
            # Never take the cell down over the sidecar. The CSV is the
            # deliverable; this file is the caveat attached to it.
            self.get_logger().warn(f"could not write {self.mask_path}: {e}")

    def resolve_stride(self, msg):
        """Column count per pair, from the publisher, decided once."""
        n = len(msg.data)
        if n == 0:
            # No pairs in this message. Divisibility says nothing about width
            # here (every candidate divides zero), so decide nothing and look
            # again next message rather than latching a stride off an empty
            # payload for the rest of the cell.
            return None

        # dim[1].size is the emulator's own kLinkStateCols. Trust it when it is
        # one of the two widths this script knows and it divides the payload.
        declared = None
        if len(msg.layout.dim) >= 2:
            declared = int(msg.layout.dim[1].size)

        if declared is not None:
            if declared in (FIELDS, FIELDS_LEGACY) and n % declared == 0:
                stride = declared
            else:
                # A declared width this script cannot use is not a reason to
                # guess: log nothing. declared is publisher data and may be 0,
                # so report the width rather than a remainder.
                # (notes: link-logger-unusable-declared-width)
                why = ("is not one of them"
                       if declared not in (FIELDS, FIELDS_LEGACY)
                       else f"leaves a remainder of {n % declared}")
                self.get_logger().error(
                    f"link_states declares {declared} columns for a payload of "
                    f"{n} values; this script knows {FIELDS} and "
                    f"{FIELDS_LEGACY} only, and {declared} {why}. Logging "
                    "nothing rather than reading the payload at a width its "
                    "publisher did not declare.")
                return None
        else:
            # With no declared layout, divisibility is the only evidence. A
            # payload both widths divide (any multiple of 90) is a tie, and a
            # tie logs nothing rather than guessing a width.
            # (notes: link-logger-undeclared-width-tie)
            fits = [c for c in (FIELDS, FIELDS_LEGACY) if n % c == 0]
            if len(fits) == 1:
                stride = fits[0]
            elif not fits:
                self.get_logger().error(
                    f"link_states payload of {n} values is not a whole number "
                    f"of {FIELDS}- or {FIELDS_LEGACY}-column rows; logging "
                    "nothing rather than a misaligned CSV.")
                return None
            else:
                self.get_logger().error(
                    f"link_states payload of {n} values divides evenly by BOTH "
                    f"{FIELDS} and {FIELDS_LEGACY} and the message declares no "
                    "layout, so the column count cannot be determined from the "
                    "payload. Logging nothing: picking either width here would "
                    "produce a full, well-formed, wrong CSV. Publish "
                    "layout.dim[1].size from hmr_comms_sim to resolve this.")
                return None
        if stride == FIELDS_LEGACY:
            self.get_logger().error(
                "link_states has 9 columns: hmr_comms_sim predates generation 9 "
                "and this workspace is half-built. The valid column will be "
                f"written as {VALID_NOT_REPORTED} and THIS TRACE CONTAINS NO "
                "STALE-POSE DETECTION. Rebuild hmr_sim.")
        return stride

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.stride is None:
            self.stride = self.resolve_stride(msg)
            if self.stride is None:
                return
        stride = self.stride
        d = list(msg.data)
        for k in range(0, len(d) - stride + 1, stride):
            row = d[k:k + stride]
            # Prefer the emulator's own verdict; fall back to the zeroing it
            # publishes alongside it. See the VALIDITY MASK note.
            if stride == FIELDS and row[9] <= 0.0:
                self.masked += 1
                self.masked_by_valid += 1
                continue
            if row[4] <= 0.0:          # path_loss_db
                self.masked += 1
                self.masked_by_path_loss += 1
                continue
            if stride == FIELDS_LEGACY:
                row = row + [VALID_NOT_REPORTED]
            self.w.writerow([f"{t:.3f}"] + [f"{v:g}" for v in row])
            self.rows += 1
        # Key the periodic write off published rows (rows + masked), not
        # self.rows, so a fully masked trace still writes the sidecar. Use a
        # threshold on the running total, not a modulus: one callback adds many
        # rows. (notes: link-logger-periodic-sidecar-trigger)
        published = self.rows + self.masked
        if published - self.last_sidecar_at >= 500:
            self.last_sidecar_at = published
            self.out.flush()
            self.write_sidecar()

    def destroy_node(self):
        self.out.flush()
        self.out.close()
        self.write_sidecar()
        frac, verdict = self.mask_verdict()
        # Summary goes to stderr and names the sidecar, the machine-readable
        # copy. Always print both mask counts with the total: which
        # discriminator fired shows whether the emulator reported validity.
        # (notes: link-logger-shutdown-summary)
        print(f"[link_logger] rows={self.rows} masked={self.masked} "
              f"(by_valid={self.masked_by_valid} "
              f"by_path_loss={self.masked_by_path_loss}) "
              f"published={self.rows + self.masked} stride={self.stride} "
              f"masked_frac={'n/a' if frac is None else f'{frac:.3f}'} "
              f"verdict={verdict} sidecar={self.mask_path}",
              file=sys.stderr, flush=True)
        # AT ERROR, not info. A trace that is mostly invalid rows produces a
        # CSV that parses cleanly, plots cleanly and is wrong, so the one thing
        # this must not do is scroll past looking like progress.
        if verdict == "UNUSABLE":
            self.get_logger().error(
                f"{frac * 100:.1f} % of published link_states rows were masked "
                f"as invalid. Most of this trace is rows the emulator never "
                f"computed; duty cycle, outage counts and reconnection events "
                f"derived from it are not evidence. See {self.mask_path}.")
        elif verdict == "NO_ROWS":
            self.get_logger().error(
                f"link_states published NOTHING for the whole cell: no valid "
                f"rows and no masked ones either. This is not a 0 % mask rate, "
                f"it is an absence of connectivity evidence — check that the "
                f"emulator was running and that the topic name matched. See "
                f"{self.mask_path}.")
        elif verdict == "SUSPECT":
            self.get_logger().warn(
                f"{frac * 100:.1f} % of published link_states rows were masked "
                f"as invalid. Some masking is normal — every pair is invalid "
                f"until both endpoints have posed — but above "
                f"{MASK_SUSPECT * 100:.0f} % it is worth checking that the pose "
                f"pipeline recovered rather than stayed down. See "
                f"{self.mask_path}.")
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
