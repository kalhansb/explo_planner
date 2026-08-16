#!/usr/bin/env python3
"""Teardown gate: did the two robots' copies of the merged map converge?

Prints one TAB-separated gate line (PASS/FAIL/UNRUN <name> <detail>) for
comms_gates.txt and exits 0 on PASS, 1 on FAIL, 2 if it could not be evaluated.

WHY THIS GATE EXISTS. Both robots run a dscovox_node merging BOTH robots'
scovox_bin streams, the delta stream is in the emulator's `reliable_topics`, and
the emulator's pre-relay queue counts every byte it drops. All of which is true
and none of which was sufficient: three consecutive dense-world runs finished
with the two merged maps disagreeing by 1.5-1.8 % of ROI voxels and every gate
green.

The leak is a KeepLast QoS queue, not the relay. On reconnect the emulator
releases a whole outage's backlog in one pass, far faster than the receiver
drains it, and a reliable KEEP_LAST reader that overflows discards the excess
with NO error and NO counter -- `drop_overflow` only ever sees the relay's own
pre-relay queue. Two such queues sit in the path (the emulator's `rx_qos_depth`
and dscovox_node's `scovox_bin_qos_depth`) and the binding limit is the
shallowest end of the chain. So the loss lands as permanently missing voxels
with nothing anywhere recording that it happened.

Nor does scovox self-repair it under the emulator. scovox_node DOES resend a
full snapshot to a new subscriber -- `snapshot = (cur_sub > prev_sub_count_)` --
which is exactly the mechanism that should heal a reconnect. It cannot fire
here: the radio outage happens INSIDE hmr_comms_sim_node, at the message-relay
level, while the emulator's DDS subscription to the sender stays up for the
whole run. `cur_sub` never falls and never rises, so the sender believes the
peer was connected throughout and never resends.

What keeps the damage bounded is that deltas carry ABSOLUTE voxel state and the
receiver snapshot-replaces, so any voxel touched again later self-heals. Only
voxels never revisited stay holed -- which is why the loss reads as a couple of
percent rather than a destroyed map, and why it is easy to miss.

THE THRESHOLD is empirical, and the two populations are far apart:

    healthy   sparse realistic 0.05 %, 0.01 %   dense ideal 0.04 %
    holed     dense realistic  1.76 %, 1.49 %, 1.78 %

0.5 % sits an order of magnitude above every healthy run and a third of the way
below every holed one. Raise it only with a reason; the point of the gate is
that a quiet 1.5 % confound is indistinguishable from a good run without it.

NOT a substitute for sizing the queues. A run that passes this gate had enough
depth for ITS outages; a longer outage in a different world can still overflow.
This makes the failure loud, it does not prevent it.
"""
import argparse
import csv
import glob
import os
import sys


def final_voxels(path):
    """Last non-blank total_observed_voxels in one planner CSV."""
    last = None
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                v = float(row["total_observed_voxels"])
            except (ValueError, KeyError, TypeError):
                continue
            if v > 0:
                last = v
    return last


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("outdir", help="run directory holding planner_*.csv")
    ap.add_argument("--max-pct", type=float, default=0.5,
                    help="max tolerated disagreement, %% of the larger map")
    ap.add_argument("--name", default="map_agree", help="gate name")
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.outdir, "planner_*.csv")))
    if len(paths) != 2:
        print(f"UNRUN\t{args.name}\texpected 2 planner_*.csv, found {len(paths)}")
        return 2

    vox = {}
    for p in paths:
        # planner_<robot>.csv
        robot = os.path.basename(p)[len("planner_"):-len(".csv")]
        vox[robot] = final_voxels(p)
    if any(v is None for v in vox.values()):
        missing = ",".join(r for r, v in vox.items() if v is None)
        print(f"UNRUN\t{args.name}\tno voxel counts logged for: {missing}")
        return 2

    (ra, va), (rb, vb) = sorted(vox.items())
    biggest = max(va, vb)
    if biggest <= 0:
        print(f"UNRUN\t{args.name}\tboth maps empty")
        return 2
    pct = abs(va - vb) / biggest * 100.0

    detail = (f"{ra}={va:.0f} {rb}={vb:.0f} differ {abs(va - vb):.0f} voxels "
              f"({pct:.2f}% of {biggest:.0f}), limit {args.max_pct:.2f}%")
    if pct > args.max_pct:
        print(f"FAIL\t{args.name}\t{detail} — merged maps did NOT converge; "
              f"suspect a KeepLast overflow (rx_qos_depth / scovox_bin_qos_depth), "
              f"which drops silently")
        return 1
    print(f"PASS\t{args.name}\t{detail}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
