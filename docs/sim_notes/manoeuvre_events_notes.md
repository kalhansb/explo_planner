# manoeuvre_events.py — design notes and history

The long comments of `sim/manoeuvre_events.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 6
- [parse_log](#parse_log) — 2
- [analyse_run](#analyse_run) — 5

## Module scope

### manoeuvre-decision-lines

**Decision lines and the dispatch prefix** — attached to `RE_CHASE = re.compile(` (line 103)

```text
Decision lines. Each marks one firing; the kind is fixed by which line hit.
The 2026-08-17 planner renamed the dispatch prefix ("exploration ended" ->
"dispatched": with the mid-run trigger the manoeuvre no longer implies the
end of exploration); both spellings are accepted so old campaigns keep
parsing.
```

### manoeuvre-destination-spellings

**The three destination spellings** — attached to `DEST = r"appointment|meeting point|last-connected anchor"` (line 116)

```text
THREE DESTINATION SPELLINGS, TWO OF THEM HISTORICAL (2026-09-19).

The node names the destination with `return_dest_label_`, which is the `what`
argument of startReturnTo. Generation 19 renamed the agreed-cell destination
from "meeting point" (the midpoint construction, now deleted) to "appointment"
— and this alternation was not updated, so on a gen-19 or gen-20 log the line
"-> returning to appointment" matched NOTHING. Every appointment dispatch in
the two arms this tool exists to compare was invisible: not miscounted, absent.
That is the `checks-that-stopped-checking` failure again, one file over.

All three spellings stay so that banked runs remain re-analysable, and the
mapping to a `kind` is below at the one place that reads group 4.
```

### manoeuvre-midrun-dispatch-regex

**Anchoring the mid-run dispatch regex** — attached to `RE_MIDRUN_DISPATCH = re.compile(` (line 134)

```text
Mid-run trigger context (2026-08-17). The dispatch marker precedes the
firing line and tags it; the resume/exhaustion lines are their own events.

ANCHORED ON THE STABLE PREFIX ONLY, and the attempt index is pulled by a
separate search. The 2026-08-17 pattern spelled the whole line —
`peer silent (\d+)s >= \d+s \(attempt (\d+)/(\d+)\)` — and when the info gate
added `gate `, `radio down` and `est unshared` to the middle of that line the
regex stopped matching anything. It failed silently: `pending_midrun` never
went True, so the `midrun` column of the readout read 0 for every firing.
Measured on the banked logs: 6 dispatch lines present, 0 matched, i.e. every
mid-run fire was reported as terminal. See MIDRUN_LINE_MARKER below for the
guard that now makes that failure loud instead of silent.

Both spellings are accepted. "peer silent" is the pre-generation-8 wording for
the same quantity (seconds since the team last read complete); it was renamed
because it invited reading the number as the peer's record age, which stands
~coord_claim_ttl_sec clear of it. Keeping the alternation means logs from
either generation parse; it does not weaken the guard, which fires on any
wording this pattern has not been taught.
```

### manoeuvre-midrun-whitelist-guard

**Why the mid-run guard is a whitelist** — attached to `MIDRUN_LINE_MARKER = "Reconnect (mid-run):"` (line 157)

```text
The guard is a WHITELIST of the mid-run lines that are legitimately not
dispatches, not a pattern for what a dispatch looks like. Keying it on dispatch
syntax (an earlier attempt matched on " >= ") reproduces the original bug one
level up: the comparison operator is part of the wording, so a reworded line
escapes the detector exactly as it escapes the parser. Whitelisting inverts the
failure direction — a NEW kind of mid-run line raises a false alarm, which is
loud and cheap, instead of a silent undercount.
```

### manoeuvre-midrun-non-dispatch-lines

**Mid-run lines that start no manoeuvre** — attached to `RE_MIDRUN_ABORTED = re.compile(` (line 172)

```text
The whitelist working as designed is still noise if nobody adds to it. These
two were raising the `!! NOT PARSED` banner on EVERY banked run — 603 and 24
lines across ts3/ts4 — and a banner that fires on every run is one the reader
learns to scroll past, which is how the thing it was built to catch gets
through. Both are node lines that explicitly say no manoeuvre started.

The second one also has to CLEAR pending_midrun. The node logs the dispatch
line first and only then discovers dispatchReconnect() returned false, so the
mid-run flag is already set with no firing to consume it; left standing it
rides forward and stamps `midrun=1` on the next firing, which may be the
terminal one at mission end. That is a mid-run count that is too high in
exactly the arm that retries.
```

### manoeuvre-ended-line-outcome

**The ending line's authoritative outcome** — attached to `RE_ENDED = re.compile(` (line 206)

```text
The AUTHORITATIVE outcome, when the line carries one. Generation 8 puts the
planner's own classification into the ending line; the group is optional so
pre-gen-8 logs, which end `... s sim (-> STATE).`, still parse for duration.

This exists because the outcome regexes above resolved almost nothing in the
banked logs. Under mission return every manoeuvre ends "-> RETURN_HOME"
whether it reconnected or gave up, so no RESOLVING marker — rejoin, gave-up,
unreachable — was ever emitted across the 5 banked g6pilot firings: grep finds
0 of each. 4 of the 5 therefore fell through to the `open_at_horizon` default,
"the manoeuvre never ended", printed beside a duration parsed from this very
line. The 5th (hybrid_seed106, bestla) did emit one ARRIVED line — "reached
meeting point (dist=3.98, tol=4.0)" — and so became `arrived_waiting` at
line 710 instead. That is not a rescue: the jsonl records that manoeuvre as
gave_up, so the score against ground truth is 0/5 either way, and the arrived
case is strictly worse because it also carried the ARRIVAL instant as the
outcome time (see the t_ended note below). The destination state cannot
substitute; it is the same for both outcomes. Prefer this group over the walk
below whenever it is present.
```

## parse_log

### manoeuvre-midrun-attempt-sentinel

**attempt=0 is a sentinel** — attached to `events.append({"w": w, "type": "midrun_dispatch",` (line 353)

```text
attempt=0 is a SENTINEL, not a count: the node numbers
attempts from 1, so 0 can only mean the attempt clause was
missing from the line. Kept distinguishable rather than
defaulted to 1, because a silent 1 would read as "first
attempt" and understate a retry ladder in exactly the arm
that retries.

A 0 is therefore AMBIGUOUS on its own and must be read
alongside the `!! ... NOT PARSED` banner in main(). This
script only prints that banner; it still exits 0. The
campaign-level enforcement is check 20 of gate_g8.py, which
is deliberately not in-tree (it is scoring, not runtime), so
nothing in THIS repo will fail a run over a drifted parser.
Do not read a lone 0 as proof the clause was optional.
```

### manoeuvre-destination-to-kind

**Mapping destination spellings to kinds** — attached to `kind = {"appointment":   "appointment",` (line 423)

```text
THE THREE SPELLINGS, MAPPED ONCE. "appointment" is the
generation-19 name for the agreed cell and it is a DIFFERENT
kind from "anchor_return": the anchor is the robot's own
last-connected pose, which no teammate is committed to, while
the appointment is a place the whole fleet agreed on. Folding
them together (which is what the old `else` did, since the
gen-19 wording matched neither branch) reports the rendezvous
treatment as a solo retreat.
```

## analyse_run

### manoeuvre-outcome-walk

**Walking forward to a firing's outcome** — attached to `outcome, t_out, dur = "open_at_horizon", None, None` (line 837)

```text
Walk forward to the resolution of THIS firing. The LAST decisive
event before the next firing wins, not the first: an escalated
hold can arrive at the anchor and STILL give up later, and a
mid-run barrier can arrive and still resume — "arrived_waiting"
is only the outcome when nothing further resolved it. escalate /
escalate_leg / midrun context rows are continuations, never
resolutions.
```

### manoeuvre-stated-outcome-wins

**The planner's stated outcome wins** — attached to `if stated:` (line 879)

```text
The planner's own classification wins over the walk above. The
walk infers an outcome from which marker line appeared; this is
the node stating it from the two facts that decided it
(team-complete at the instant, and the state it landed in). Where
they disagree the walk is wrong by construction — it cannot see a
release that emitted no marker, which under mission return is
every release.
```

### manoeuvre-stated-outcome-time

**The stated outcome carries the ending's time** — attached to `if t_ended is not None:` (line 888)

```text
The timestamp has to move with the label. t_out was
backfilled only `if t_out is None`, so a manoeuvre that
reached the anchor and THEN ended kept the ARRIVAL instant
while taking the ending's outcome word — reporting
"gave_up at t=100" next to a duration measured to t=180, and
sampling dist/link at an instant the stated outcome is not
about. The stated outcome is a property of the ending, so it
carries the ending's time; the walk's arrival time survives
only where no ending line was parsed at all.

Scope, deliberately wider than the arrival case that
motivated it: this overrides t_out for EVERY stated outcome,
including reconnected/resumed_exploring where the walk had a
marker instant. Under mission return no resolving marker is
ever emitted, so today that branch is unreachable and the two
instants would coincide anyway. If a rejoin line is ever
added, revisit — dt_to_outcome_s would then measure the whole
manoeuvre rather than time-to-reconnect.
```

### manoeuvre-ended-unclassified

**Ended but unclassified manoeuvres** — attached to `outcome = "ended_unclassified"` (line 909)

```text
A manoeuvre that demonstrably ENDED (a duration was parsed
from its ending line) cannot also be "open at the horizon".
Pre-gen-8 logs carry no outcome word, so this is the honest
label for them; it is a distinct string from the default so
the two are never confused in a table, and so this exact
contradiction can never again be printed as "never ended".
```

### manoeuvre-team-column-names

**Team-level link columns and their names** — attached to `"team_connected_at_arm": ("" if lk is None else int(lk[1])),` (line 937)

```text
Renamed with the meaning, deliberately: these were
link_up_*/separation_m_at_arm when they were one pair's
numbers. They are now the team's (read_link), and a column
that quietly changes what it measures under an unchanged name
is how a banked table gets re-read as something it never said.
```
