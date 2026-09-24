#!/usr/bin/env python3
"""Checks that two cells meant as a matched pair differ only where they should
(DESIGN_gen34 §12.8: the full-map experiment's control and full cells live in
two campaign roots, and the resume guard never compares across roots).

Every key in either run_manifest.txt must have the same value in both, except
the independent variable of the pair (by default the map stream:
share_full_period_s, map_stream) and keys that describe the run rather than
configure it (timestamps, paths, host, outcomes). Commit and binary hashes
are compared: the pair must run the same build.

  manifest_pair_check.py CONTROL_CELL FULL_CELL [--allow KEY ...]

Exit 0 when the pair matches, 1 on a difference, 2 on unreadable input.
"""

import argparse
import os
import re
import sys

PAIR_VARIABLE = {'share_full_period_s', 'map_stream'}
DESCRIBES_THE_RUN = re.compile(
    r'^(started_utc|finished_utc|host|outdir|ros_domain_id|ign_partition|'
    r'mtime_.*|run_end_.*|run_gates_.*|done_drain_complete)$')


def read(cell):
    path = os.path.join(cell, 'run_manifest.txt')
    kv = {}
    with open(path) as f:
        for line in f:
            line = line.rstrip('\n')
            if not line or line.startswith('#') or '=' not in line:
                continue
            k, v = line.split('=', 1)
            kv.setdefault(k, v)   # first occurrence, as the resume guard reads
    return kv


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('control')
    ap.add_argument('full')
    ap.add_argument('--allow', nargs='*', default=[],
                    help='further keys allowed to differ')
    a = ap.parse_args()
    try:
        c, f = read(a.control), read(a.full)
    except OSError as e:
        print(f'unreadable: {e}')
        return 2
    allowed = PAIR_VARIABLE | set(a.allow)
    diffs = []
    for k in sorted(set(c) | set(f)):
        if k in allowed or DESCRIBES_THE_RUN.match(k):
            continue
        if c.get(k) != f.get(k):
            diffs.append((k, c.get(k, '<absent>'), f.get(k, '<absent>')))
    for k in sorted(PAIR_VARIABLE):
        print(f'  pair variable {k}: {c.get(k, "<absent>")} | {f.get(k, "<absent>")}')
    if c.get('map_stream') == f.get('map_stream'):
        diffs.append(('map_stream (must differ)', c.get('map_stream', '<absent>'),
                      f.get('map_stream', '<absent>')))
    for k, cv, fv in diffs:
        print(f'  DIFF {k}: {cv} | {fv}')
    print('PAIR OK' if not diffs else f'PAIR MISMATCH ({len(diffs)} key(s))')
    return 1 if diffs else 0


if __name__ == '__main__':
    sys.exit(main())
