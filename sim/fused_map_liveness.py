#!/usr/bin/env python3
"""Flags a planner whose map stopped growing (DESIGN_gen34 §12.9, pilot
finding 1: bestla's total_observed_voxels held at 1148734 from t~895 s to the
3025 s end of gen34n2_off_seed1 while its dscovox kept fusing).

Reads planner_<robot>.csv in each cell and reports, per robot, the longest
stretch of sim time over which total_observed_voxels did not change, and
whether that stretch ran to the end of the run. A robot is FROZEN when the
stretch is at least --min-sec (default 600) and still open at its last row,
unless every row in it has state DONE (a finished robot may stop mapping).
A flag for inspection before a cell is pooled, not a run gate: a robot
parked in a mapped pocket for ten minutes looks the same.

  fused_map_liveness.py CELL_DIR [CELL_DIR ...] [--min-sec S]

Exit 1 if any robot is FROZEN, 2 on unreadable input, else 0.
"""

import argparse
import csv
import glob
import os
import sys


def plateaus(rows):
    """Longest flat stretch: (length_s, t_start, t_end, open_at_end, all_done)."""
    best = (0.0, None, None, False, False)
    start = None
    done_all = True
    for i, r in enumerate(rows):
        if start is None or r['v'] != rows[start]['v']:
            start, done_all = i, True
        done_all = done_all and r['state'] == 'DONE'
        length = r['t'] - rows[start]['t']
        if length > best[0]:
            best = (length, rows[start]['t'], r['t'], i == len(rows) - 1, done_all)
    return best


def read_planner(path):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            try:
                rows.append({'t': float(r['sim_time_sec']),
                             'v': int(float(r['total_observed_voxels'])),
                             'state': (r.get('state') or '').strip()})
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('cells', nargs='+')
    ap.add_argument('--min-sec', type=float, default=600.0)
    a = ap.parse_args()
    frozen = bad = 0
    for cell in a.cells:
        paths = sorted(glob.glob(os.path.join(cell, 'planner_*.csv')))
        if not paths:
            print(f'{cell}: no planner_*.csv')
            bad += 1
            continue
        for p in paths:
            robot = os.path.basename(p)[len('planner_'):-len('.csv')]
            rows = read_planner(p)
            if len(rows) < 2:
                print(f'{cell} {robot}: fewer than 2 readable rows')
                bad += 1
                continue
            length, t0, t1, open_end, all_done = plateaus(rows)
            is_frozen = length >= a.min_sec and open_end and not all_done
            frozen += is_frozen
            print(f"{os.path.basename(os.path.normpath(cell))} {robot}: "
                  f"{'FROZEN' if is_frozen else 'ok'}  longest flat "
                  f"{length:.0f} s (t {t0:.0f}-{t1:.0f}"
                  f"{', open at end' if open_end else ''}"
                  f"{', all DONE' if all_done else ''}) at {rows[-1]['v']} voxels, "
                  f"last t {rows[-1]['t']:.0f} s")
    return 2 if bad else (1 if frozen else 0)


if __name__ == '__main__':
    sys.exit(main())
