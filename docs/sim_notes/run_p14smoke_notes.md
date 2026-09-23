# run_p14smoke.sh — design notes and history

The long comments of `sim/run_p14smoke.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 1

## Top level

### p14smoke-what-it-tests

**What the p14 smoke tests** — attached to `set -eo pipefail` (line 4)

```text
Everything except the criterion is byte-identical to p13: same scenario, same
binary (built 2026-08-17 19:08, older than every p12 and p13 cell -- the first
was written 2026-08-17 23:40 -- and nothing has been rebuilt since), tx 30.0,
duration 5400, record 2, FRONTIER_ONLY=1, PURSUIT_BUDGET_MAX=2400. So any
difference is the criterion and nothing else.

2026-08-06 15:15 appears on install/.../explo_planner_node but that is the
SYMLINK's own mtime, not the binary's: --symlink-install creates the link once
and later builds rewrite the target in place, leaving the link untouched.
Follow it to build/ before dating a build.

Same two cells as p13smoke -- hybrid:1 and pursuit:3 -- so the comparison is
paired rather than against an arm median. hybrid also exercises both halves of
the manoeuvre (chase, then the meeting-point fallback) in one cell.

Runs to exploration completion, not a truncated window: 5400 is the cap, and a
healthy cell terminates on the criterion long before reaching it.

WHAT THIS IS ACTUALLY TESTING. criterion_choice.py picked 0.60 by RESCORING
already-collected runs, which is valid for completion time but assumes the run
is otherwise unchanged. It is not: at 0.60 a cell stops ~30% earlier, and two
things could break that rescoring cannot see.

  1. COMMS GATES. A gate can fail because the link never dropped hard enough.
     A shorter run accumulates less outage, so the gates are the live risk
     here -- not the planner. This is the check that decides whether 0.60 is
     usable at all.
  2. TREATMENT ENGAGEMENT. A mid-run manoeuvre needs 240 s of continuous peer
     silence to arm. At 0.60, 4 of 17 treated cells finish before their first
     dispatch (3 of 17 at 0.55). Both smoke cells must still dispatch, or the
     arms are not being contrasted.
```
