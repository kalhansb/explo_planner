# analyze_runs.py — design notes and history

The long comments of `sim/analyze_runs.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [read_link_mask](#read_link_mask) — 1
- [duty_and_outages](#duty_and_outages) — 2
- [analyse_run](#analyse_run) — 1
- [main](#main) — 1

## read_link_mask

### linkmask-no-verdict-is-unknown

**A sidecar without a verdict is unknown** — attached to `return None` (line 128)

```text
A sidecar with no verdict in it grades nothing, so it is the same
answer as no sidecar at all: unknown. This is not a hypothetical
input -- write_sidecar() rewrites the file in place as the run goes,
so a cell killed at teardown can leave a truncated or zero-byte one,
and an empty file parses to an empty dict without raising anything.
Returning the partial dict instead would make it a GRADED run whose
verdict is None, which is not flagged and therefore gets counted in
the "all N graded run(s) OK" line: the unreadable case reported as
the clean one.
```

## duty_and_outages

### duty-outages-short-trace-return

**The short-trace branch returns four values** — attached to `return float("nan"), [], float("nan"), None` (line 267)

```text
FOUR values, like the normal return below. This branch returned three,
so any run with a missing or short link trace (read_link_trace returns
[] on OSError, and the path_loss_db <= 0 startup mask can empty a
short one) raised ValueError at the call site and took the whole batch
down with it rather than skipping the one bad run.
```

### duty-outages-open-outage-censored

**An open final outage is right-censored** — attached to `open_ep = (trace[-1][0] - start) if start is not None else None` (line 288)

```text
An outage still open when the trace stops is RIGHT-CENSORED, not absent.
Dropping it silently discarded the largest outage in the pursuit run
(1225 s, from t=2495 to the end) from a set whose median was 6 s, and
rendered a permanently-down link as "0 outages, median nan". Its duration
is a lower bound, so it is reported and counted but kept out of the
median — same discipline as calib_summary.episodes.
```

## analyse_run

### analyze-manoeuvre-count-sampled

**Manoeuvre count from the sampled state** — attached to `manoeuvres, reconnect_secs, open_manoeuvres = 0, [], 0` (line 412)

```text
Manoeuvre accounting straight off the state column.

NOTE (section 3.22): this UNDERCOUNTS. The CSV is sampled on a ~5-10 s
timer and manoeuvre episodes are routinely shorter, so 6 of the 22
firings on disk leave no row here at all — and they are the FAST ones,
which is the worst possible bias for a mode comparison. The count below
is retained because reconnect_secs is derived from it and needs the
in-episode rows, but the authoritative firing census is the log-derived
one in manoeuvre_events.py; `n_firings_log` is reported beside it.
reconnect_elapsed_sec must be read from INSIDE the episode, not from the
row that leaves it: the planner's transitionTo clears reconnect_active_
before that row is emitted, so the exit row always reads -1 and the old
">= 0" filter discarded every value. Verified: the exit rows carry -1
while the last in-manoeuvre rows carry 183.08 / 124.45 / 12.85. Take the
largest value seen within the episode — the field counts up, so that is
its duration, and it also survives an episode still running at the
horizon (which otherwise contributes nothing at all).
```

## main

### analyze-group-by-cell

**Summaries group by cell, not arm** — attached to `by_arm = {}` (line 551)

```text
Group by the CELL, not the arm alone. Keying on reconnect_mode_requested
by itself pooled three tx=160 controls, a tx=-60 blackout and a tx=-14
treatment run into one "arm off n=5" line with a realised-severity range
of 0.000-1.000 — three different experiments averaged into a number that
describes none of them. The criterion is in the key for the same reason:
it DEFINES the endpoint, so two thresholds are two endpoints.
```
