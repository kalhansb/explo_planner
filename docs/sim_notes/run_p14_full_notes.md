# run_p14_full.sh — design notes and history

The long comments of `sim/run_p14_full.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 1

## Top level

### p14-criterion-and-campaign

**p14 campaign criterion and setup** — attached to `set -eo pipefail` (line 4)

```text
WHY 0.60 RATHER THAN THE 0.55 USED BY p12/p13. At 0.55 the run-to-run SD of
completion time is ~920 s, larger than any arm difference observed, so n=10
could only resolve a ~1150 s effect -- the campaign could not have answered
its own question. 0.55 sits ~0.045 above the lowest unknown fraction ever
reached here (0.5053), on a part of the curve whose marginal cost has already
risen ~4x, so completion time was partly measuring how long a run takes to
scrape past a threshold near the floor. At 0.60 the SD is ~198 s and the
detectable difference at n=10 falls to ~248 s. See criterion_choice.py; the
guard against over-raising it is in there too (at 0.70 the SD is better still
and 71% of treated runs finish before any manoeuvre fires, measuring nothing).

ORDER IS SEED-MAJOR (run_campaign.sh:78): every arm at seed 1 before any arm
at seed 2. Stopping this campaign at any point therefore leaves a COMPLETE
paired block for the seeds it reached, which is analysable on its own. That is
the whole reason it is safe to queue 40 cells overnight rather than 6.

Runs to exploration completion, not a truncated window. 5400 is a cap, not a
target: the deepest 0.60 crossing in 25 collected cells is ~1961 s, so the cap
is 2.7x the worst case and exists only to bound a pathological cell.

EVERYTHING ELSE IS IDENTICAL TO p13: same scenario, same binary, tx 30.0,
duration 5400, record 2, FRONTIER_ONLY=1, PURSUIT_BUDGET_MAX=2400. The
criterion is the only thing that moved.

NO colcon build while this runs. Every cell must share one binary, or an arm
difference is confounded with a code change. Nothing needs one: no .cpp or
.hpp is newer than the built node (2026-08-17 19:08), and the install tree is
--symlink-install so yaml/config edits are already live.
```
