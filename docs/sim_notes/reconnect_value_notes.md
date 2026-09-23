# reconnect_value.py — design notes and history

The long comments of `sim/reconnect_value.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [roster](#roster) — 1
- [analyse](#analyse) — 1
- [timing_table](#timing_table) — 1

## roster

### roster-manifest-authority

**The manifest is authoritative on who ran** — attached to `missing = [x for x in r` (line 107)

```text
The manifest is the authority on WHO RAN, so a name it lists with no log
is reported, not quietly swapped out: an event log missing for one robot
of three is a truncated cell, and falling back to the two that do have
logs would analyse it as a two-robot run. Only a roster with NO usable
names at all falls back, because at that point the manifest has told us
nothing we can use.
```

## analyse

### value-pairwise-only-refusal

**Why non-pairwise cells are refused** — attached to `print(f"SKIP\t{cell}\tthis script is pairwise and the cell has "` (line 259)

```text
Refuse, do not score. Everything below is pairwise: link_states.csv
carries one row PER PAIR per tick, and outages() folds the whole file
into a single connected series. On three robots that is three
interleaved link traces, so a genuine A-B outage is cancelled by the
healthy A-C rows at the same t_sim and sep_at() returns whichever
pair's row happened to be last. The result is not a degraded answer,
it is an arbitrary one -- a 3-robot cell with A-B down for 300 s
reports zero outages and an empty table, which reads as "no reconnect
opportunities arose".

An N-robot version needs a per-link outage series and a per-link
separation, i.e. the whole table re-keyed on the pair. Until that
exists this says so instead of guessing.
```

## timing_table

### value-unclassified-action-warning

**Announcing unclassified dispatch actions** — attached to `KNOWN = set(ACTED) | {"hold", "resume_exploring"}` (line 388)

```text
A NEW ACTION STRING MUST NOT BE ABSORBED SILENTLY. Membership in ACTED
fails open into `declined`, which is the direction that flatters the
untreated arm, so an unrecognised action is announced once per run rather
than binned. This is the check that would have caught `appointment`
arriving as an unclassified string in the first place.
```
