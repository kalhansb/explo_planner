# gate_p1.py — design notes and history

The long comments of `sim/gate_p1.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 2
- [check_robot](#check_robot) — 1

## Module scope

### gate-p1-edge-slack

**Column-binning slack at the ROI edge** — attached to `EDGE_SLACK = 4.0e-3` (line 99)

```text
Column-binning slack, as a FRACTION of the ROI's columns.

censusFromMap bins columns on doubles; MapCache::unknownColumnFraction bins
them through Bonxai's float inv_resolution, which is not the exact inverse of
the float resolution (at 0.1 m, posToCoord(1.0) == 9, not 10). The two can
therefore disagree about which side of the ROI EDGE a boundary column falls,
by at most one column-row per edge. Interior disagreements cancel: a column
moving between two cells changes neither sum.

At the sim's 100 m ROI and 0.1 m voxels that is 4 * 1000 columns out of
1000^2, i.e. 4e-3. The value here is deliberately a fixed number rather than
something derived per run, so that if a future change makes the two binnings
diverge in the INTERIOR the slack does not silently absorb it.
```

### gate-p1-min-progress

**Minimum progress for saturated coverage** — attached to `MIN_PROGRESS = 0.05` (line 119)

```text
The run must have made this much progress in roi_unknown_fraction between its
first and last census for "coverage saturates" to describe what happened.
Not a coverage TARGET: this world's ROI is far larger than two robots clear in
a run (the shipped done_unknown_fraction is 0.64), so a threshold on the final
value would only encode how big the ROI happens to be.
```

## check_robot

### gate-p1-joint-reading-in-marginal

**Joint reading bounded by the marginal** — attached to `if ff[2] < ff[0] - 1e-12:` (line 278)

```text
The joint reading is one of the cells in the marginal, so it
cannot sit below the marginal's minimum. This is what catches the
field being wired to something that is not a cell in the set —
the failure mode that would leave the threshold advice below
confidently describing a candidate that does not exist.
```
