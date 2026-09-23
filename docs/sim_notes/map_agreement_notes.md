# map_agreement.py — design notes and history

The long comments of `sim/map_agreement.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [main](#main) — 3

## main

### map-agree-no-max-pct

**Why there is no max-pct flag** — attached to `args = ap.parse_args()` (line 194)

```text
NO --max-pct. It was accepted-and-ignored for one generation, which is
the worst of the three options: the caller passed 0.5, the manifest
banked `map_agree_max_pct=0.5`, and anyone reading either would conclude
a 50% spread threshold had decided something. Nothing was ever failed on
it. Removing it rather than keeping a SUPPRESSed no-op means a resurrected
banked command line dies with "unrecognized arguments: --max-pct" instead
of quietly agreeing that the threshold is live.
```

### map-agree-two-series-exit-zero

**Two-series floor and always exit 0** — attached to `if len(paths) < 2:` (line 219)

```text
Two is the FLOOR, not the shape. A spread needs at least two series; above
that the arithmetic is the same. Returning 0 on the INFO paths as well as
the PASS path is deliberate: this file is report-only, so a non-zero exit
would be one more way for it to influence a verdict it has no business
influencing (run_explo_sim_rviz.sh currently swallows it with `|| true`,
and that `|| true` should not be what keeps the contract).
```

### map-agree-empty-vs-absent-csv

**Empty CSVs versus absent CSVs** — attached to `dropped = f" (dropped, no voxel counts: {','.join(empty)})" if empty else ""` (line 238)

```text
A robot with no rows at all is dropped and NAMED. Silently comparing the
remaining two would print a clean spread for a team of four in which two
robots logged nothing.
Two different absences, kept apart because they mean different things: a
robot that logged a header and no rows produced a planner that ran, and a
robot with no file at all produced one that did not.
```
