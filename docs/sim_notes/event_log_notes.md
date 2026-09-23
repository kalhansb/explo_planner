# event_log.py — design notes and history

The long comments of `sim/event_log.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 2
- [summarise_robot](#summarise_robot) — 8
- [summarise_run](#summarise_run) — 2
- [excluded](#excluded) — 1
- [summarise_run (part 2)](#summarise_run-part-2) — 4
- [arm_summary](#arm_summary) — 1
- [main](#main) — 5

## Module scope

### evlog-sys-import-top-level

**Why sys is imported at top level** — attached to `import sys` (line 70)

```text
Top-level, not in the __main__ block where the argv import lives: the schema
warning in summarise_robot() writes to sys.stderr, and that function runs when
this module is IMPORTED by another script — a path on which __main__ never
executes. A block-scoped import would have made the newer-file warning raise
NameError instead of warning.
```

### evlog-reader-vs-min-schema

**Reader schema versus minimum schema** — attached to `READER_SCHEMA = 9` (line 77)

```text
Two constants, not one, because they answer different questions.

READER_SCHEMA is the schema this file was WRITTEN AGAINST — the vintage of the
code. It is fixed and the CLI must not move it; it is what "newer than this
reader" is measured against.

MIN_SCHEMA is the lowest schema this INVOCATION will accept — a floor the
operator can lower with --min-schema. The default must track the current
generation rather than being permissive: scoring a generation-8 campaign
should not quietly read a generation-6 cell that happens to sit in the same
directory. Lowering it is legitimate for historical work; doing so silently is
not, which is why it is a flag and not a default.

They were the same variable until round 4, which made `--min-schema 2` warn
that every current file was "newer than this reader's 2" and made
`--min-schema 4` announce that raising the bar "may pool binary generations".

v4 (the M-TARE evolution) is the first bump that adds ONLY new event kinds and
changes no existing one, because every v4 mechanism ships default-off. A v4
cell run at defaults is therefore readable by this file's v3 logic and pools
legitimately with v3 cells — `--min-schema 3` is sound for such a mix, which
is not something that could be said of any earlier bump. It stops being sound
the moment a v4 arm is run with a mechanism ENABLED, and this file cannot tell
the two apart from the version alone: that is equiv_gate.py's job, and mixing
without it is the "don't pool across binary generations" mistake wearing a
newer number. The default stays at the current schema so the pooling decision
is always something an operator typed.

v5 (2026-09-16) is NOT such a bump and `--min-schema 4` is NOT sound for a
v4/v5 mix. The rendezvous appointment stopped being derived independently on
each robot and became a pair exchanged over TeamWorld and committed by
unanimity, so every rendezvous quantity in a v4 file was produced by a
different mechanism than the same-named quantity in a v5 file — the numbers
are comparable in type and in nothing else. `RendezvousAgreedEvent.excluded`
was also removed outright. Leaving this at 4 would have been the quiet
failure: `ver > READER_SCHEMA` only WARNS, so the floor is the only thing that
refuses, and a floor one generation stale accepts the old generation in
silence while the new one merely prints to stderr.

v6 (2026-09-17) is the same kind of bump and the same reasoning applies, so
this moved with the binary rather than one generation behind it. `t_meet_sec`
CHANGED MEANING: it was a single absolute instant, and it is now the FIRST
occurrence of a meeting that recurs every `interval_sec` for the rest of the
mission. A v5 reader handed a v6 file gets a number of the right type, in the
right units, that answers a different question — the failure this floor
exists to refuse. `agreed_provisional` was added in the same change; a v5 file
has no such column, so a mixed pool cannot tell a placeholder meeting cell
from a tour-informed one.

v7 (generation 19) is the THIRD consecutive bump driven by a meaning change
with no field movement, and the constant had moved to 7 before this block said
why. Two columns turned over:

  `t_meet_sec` stopped being the first occurrence of a recurring meeting and
  became this robot's own departure deadline — now plus a shared countdown,
  computed locally. The v6 equality-across-robots test therefore FAILS BY
  DESIGN on a healthy v7 cell, and `interval_sec` no longer generates it: the
  recurrence is gone and nothing reads the interval for timing.

  `rendezvous_outcome.outcome` now names what the barrier acted on
  (teamComplete over live peers) rather than whole-team direct contact. On v6
  N>=3 files that label is simply wrong — a relayed reunion was written down
  as a no-show — and it cannot be repaired by re-reading, because the
  barrier's own verdict was never in the file. `mutual` was added alongside to
  carry the strict predicate, so v7 rows hold both facts.

Do not pool v6 and v7 rendezvous rows. See ExperimentLog::kSchemaVersion for
the authoritative list.

v8 (generation 23, 2026-09-18) carves "unplaceable" out of what v7 wrote as
"no-show". A robot that could not place the agreed cell on any grid it held
was handed its OWN position as the meeting point, reached that goal on the
next tick, and logged `no-show arrived=true` — peers blamed for a departure
that never happened. On v8 those rows read `unplaceable arrived=false`. No
column moved and both vintages parse, so a no-show tally pooled across the
boundary is counting two different populations. Do not pool v7 and v8
rendezvous outcomes either.

v8 also adds run_end.midrun_attempts_used, and is behaviourally a different
binary in ways with no schema surface at all (the peer-liveness predicate
behind nearly every reconnection decision changed). Nothing here can detect
that; the stamp is the only handle.

v9 (generation 25, 2026-09-18) moves the arming floor behind `t_meet_sec`
from now + rendezvous_depart_delay_sec to bare now. On v8 a healthy cell
could write DIFFERENT t_meet_sec values for the same outage — the per-robot
notice forked one N=3 cell across two occurrences — while on v9 rows for the
same outage match whenever every robot armed before the agreed instant, and
`t_meet_sec - t_now_sec` lands in [0, interval_sec) instead of
[delay, delay + interval_sec). `rendezvous_depart_delay_sec` still appears in
params but decides nothing. v9 also splits `arrived=false` outcome rows into
two populations — a walker converted to the release barrier because the team
settled under it (new state_change reason `return-team-settled`) vs one
stranded while the team stayed apart — so do not pool v8 and v9 arrival
rates.
```

## summarise_robot

### evlog-schema-version-check

**The schema version check** — attached to `ver = start.get("schema_version")` (line 275)

```text
The version stamp, finally read by something.

It was written from the start and consumed by nothing, which made the
"an analysis script can refuse a file it predates" rationale in
experiment_log.hpp aspirational rather than true, and left the argument
for voiding the generation-7 cells resting on a check that did not exist.

Below MIN is a refusal. State the reason accurately, because the first
version of this message did not:

  - It said the refused files are "the generation-7 cells". They are not.
    v1 spans 25 campaign tags and v2 spans 6 (g5smoke, g6pilot, mr0pilot,
    mr0smoke, mr1, mr1smoke) — many generations, and NOT the void g7 cells,
    which were moved out of the campaign root entirely.
  - It said reading a v2 file anyway "yields plausible wrong numbers"
    because of the `metrics_rows` -> `metrics_timer_rows` rename. That is
    the dangerous shape in general, but it is not a hazard for THIS
    reader: grep says no function here touches either that field or the
    removed `last_contact_age_sec`. The rename is checked where it is
    actually read — gate_g8.py check 19.

So the honest justification is generation hygiene, not a parse hazard: a
generation-8 summary must not silently absorb pre-generation-8 cells,
because pooling across binary generations is the standing error this
project keeps making. That is a real reason to refuse BY DEFAULT, and a
bad reason to refuse ABSOLUTELY — hence --min-schema, which makes reading
the historical bank a deliberate, visible act rather than an impossible
one. Before this flag existed the guard refused 1004 of 1006 banked
robot-runs with no way to override, which is not a guard, it is an outage.

ABOVE max is deliberately NOT a refusal. A future generation is more
likely to add fields than to move them, and a check that hard-fails
forward gets deleted the first time it is wrong — which is how a guard
stops guarding. Warn, keep going, and let the field-level reads fail if
they actually break.
```

### evlog-refusal-message-wording

**Wording of the schema refusal** — attached to `if MIN_SCHEMA == READER_SCHEMA:` (line 317)

```text
Do NOT phrase this as "generation {MIN_SCHEMA}". The schema version
and the binary generation are different counters that happen to both
be small integers -- schema 3 belongs to generation 8 -- and naming
the wrong one in the error is how a reader ends up believing the void
generation-7 cells are the ones being refused. Say "schema".

And say WHICH refusal this is. "Refused by default" is a lie once the
operator has raised the floor with --min-schema: they are then being
told the tool made a conservative choice on their behalf, when in fact
it is obeying an instruction they gave.
```

### evlog-newer-than-reader-check

**Newer-file warning uses READER_SCHEMA** — attached to `if ver > READER_SCHEMA:` (line 336)

```text
Compare against READER_SCHEMA, not MIN_SCHEMA. MIN_SCHEMA is a floor the
operator can lower; the reader's own vintage does not move with it. When
they were the same variable, `--min-schema 2` -- the documented way to
read banked cells -- made every CURRENT schema-3 file print "newer than
this reader's 2", which is false: the reader is not older, the floor is.
```

### evlog-exploration-finish-latched

**Exploration finish is coverage-latched only** — attached to `latched = [e for e in completes if e.get("reason") == "coverage-latched"]` (line 358)

```text
Exploration FINISH (pre-registered secondary endpoint): only a
coverage-latched declaration is a finish. Under mission return a
step-budget robot still declares (and still goes home), so filtering on
the reason here -- not on what happened next -- is what keeps the
endpoint arm-invariant.
```

### evlog-mission-end-last-row

**Mission end uses the last declaration** — attached to `missions = [e for e in evs if e["event"] == "mission_complete"]` (line 366)

```text
Mission end (pre-registered primary endpoint, schema v2).

THE LAST one, not the first, and a duplicate is REPORTED rather than
excluded. This used to raise on len(missions) > 1, and that raise cost the
analysis more than the defect it was guarding against:

  * ts1b shipped 16 robot-runs with two mission_complete rows: a
    DONE -> RETURN_HOME re-entry, which generation 8 believed was already
    guarded because startReturnHome opened with `if (state_ ==
    State::RETURN_HOME)`. That test structurally could not see the
    re-entry that was measured, because the second request arrives from
    DONE. Generation 9 added the run-scoped `mission_return_done_` latch
    ahead of it; the state test is still there and still does its own,
    narrower job. The raise turned every one of those cells into an
    EXCLUSION, and they were not a random 16 — a re-entry needs a second
    homing attempt, so they were the slowest runs of the arm that homes
    most. Dropping them shortened that arm's mean by removing its tail.
  * The data was never actually missing. A robot that declares twice still
    declares; the endpoint question ("when did the mission end") has a
    clean answer, namely the LAST declaration, consistent with
    explore_done_sim_sec being the last exploration_complete.

So the duplicate is now surfaced as mission_occurrences / mission_duplicate
on the robot record and the cell still contributes its endpoint. The
planner also stamps `occurrence` on each row (see
MissionCompleteEvent::occurrence), so the two can be cross-checked: the
count here is what the FILE contains, the field is what the NODE believed
it was emitting, and a disagreement means events were lost in between.

On generation-9 and later logs this pair can no longer see a duplicate at
all, and that is deliberate rather than an oversight. The writer latches
after the first row (ExperimentLog::logMissionComplete) and keeps the
FIRST one, so a run that declared twice lands on disk as one row stamped
occurrence 1: count 1, stamp 1, mismatch False. Read that way and nothing
else, the detector below would report "checked and clean" on precisely the
fault it exists for. The replacement signal is on the run_end row --
mission_completes_suppressed and mission_return_reentries, both pulled out
below -- and it is strictly better than the two-row version because it
survives the refusal of the second homing leg, which no longer produces a
second declaration to count. The old pair is kept because it is the only
thing that can read a generation-8 log.
```

### evlog-occurrence-tri-state

**Occurrence stamp versus rows on disk** — attached to `mission_occurrence_stamped=mission_occ_stamped,` (line 441)

```text
`occurrence` as the NODE stamped it on the last row, against which
len(missions) is the file's own count of rows. Equal is healthy. The
node's number being HIGHER means rows were emitted and did not reach
the disk, which is a different and worse fault than a duplicate: a
duplicate is visible, a lost row is not.

None means the binary that wrote this log predates the field (it was
added with the generation-9 endpoint work), NOT that the check passed.
Those two have to stay distinguishable, which is why the mismatch flag
below is tri-state rather than a bool defaulting to False -- a False
there would read as "checked and clean" on a log that was never
checkable, which is exactly how a guard goes inert while still
printing passes.
```

### evlog-run-end-endpoint-counters

**The run_end mission counters** — attached to `mission_return_reentries=(end.get("mission_return_reentries")` (line 458)

```text
The generation-9 endpoint counters, read off run_end because that is
the only row guaranteed to exist after a suppression (a duplicate
mission_complete necessarily precedes the run ending, unlike a
duplicate run_end, which is why these two could be put there and
dup_run_ends could not).

  mission_return_reentries    -- startReturnHome calls the
      mission_return_done_ latch REFUSED. Non-zero is not a defect on
      its own: the late coverage latch legitimately asks for a second
      homing leg, and the refusal is the fix working. It is the
      manipulation check for that fix, so zero across a whole campaign
      is the thing to be suspicious of, not a large number.

  mission_completes_suppressed -- second and later mission_complete
      rows the writer threw away. On generation 9 this SHOULD be 0
      everywhere: logMissionComplete has exactly one call site,
      finishMissionReturn, and startReturnHome's latch is what lets
      that be reached once. Non-zero therefore means a
      finishMissionReturn re-entry that did not pass through
      startReturnHome -- a path the node-level guard cannot see -- and
      the affected run's homing metrics describe the first attempt
      while its behaviour contains two. Treat those runs as suspect.

Both tri-state for the same reason as the mismatch flag above: None
means the binary predates the field, which is not a pass.
```

### evlog-homing-duration-held

**Homing duration includes held time** — attached to `homing_duration_sec=(mission.get("homing_duration_sec")` (line 487)

```text
v8 CHANGED WHAT homing_duration_sec MEASURES: it is now wall time on
the leg, where v7 excluded time parked in PROXIMITY_HOLD. Both parse,
neither is wrong, and they are not the same number -- so a mixed-
schema pool of this column is comparing two intervals. Subtract
homing_held_sec to recover the v7 quantity, which is also the one the
mission_return_max_sec budget is charged against.

homing_held_sec is None on any v7 file, and that is not zero: it means
the binary did not measure it. Same tri-state discipline as the two
fields above.
```

## summarise_run

### evlog-exclusion-arm-source

**Arm label on exclusion records** — attached to `cell = os.path.basename(run_dir.rstrip("/"))` (line 621)

```text
`arm` on the exclusion paths: params FIRST, directory name only as a last
resort. An exclusion of unknown arm cannot answer the one question the
exclusion count exists to answer -- whether the arms lost the same number
of cells -- so every exclusion record carries an arm; but the label has to
be the same one the surviving cells are grouped by, or the exclusions file
under an arm that does not exist. `arm_source` says which it was, and a
disagreement between the two is reported rather than silently resolved.
```

### evlog-arm-note-token-test

**Arm note needs a token test** — attached to `arm_note = (f"directory says arm={dir_arm!r}, which does not contain the "` (line 635)

```text
Inequality alone is NOT the test, and this is the whole reason the
note is usable. Every real cell directory disagrees with its params
arm by construction — 'ts1b_n3_mtare_hybrid_r40_ttl0_seed1' yields a
dirname arm of 'n3_mtare_hybrid_r40_ttl0' against a params arm of
'mtare_hybrid' — so a note on `!=` would fire on 100% of cells and be
read as decoration within a day. What is worth reporting is the
directory naming a DIFFERENT arm, not the directory naming the same arm
plus team size, radio, and knob suffixes. See _arm_tokens_agree.
```

## excluded

### evlog-exclusion-n-robots

**Team size on exclusion records** — attached to `r = dict(excluded=reason, run=cell, arm=ex_arm, arm_source=arm_src,` (line 647)

```text
n_robots reads `expect_robots` at CALL time, so every exclusion raised
after the count is resolved carries it, and the one raised because it
could NOT be resolved carries None. That is the honest split: an
exclusion of unknown team size must not be filed into a size stratum,
because arm_summary keys on the stratum and a guess there would put a
lost cell under a team size it may not have had.
```

## summarise_run (part 2)

### evlog-lost-row-is-true-test

**Lost-row list tests is True** — attached to `mission_lost_row_robots=[r for r, s in per.items()` (line 707)

```text
Robots whose node-stamped occurrence disagrees with the number of rows
on disk, i.e. an endpoint declaration the node made and the file does
not hold.

`is True`, not truthiness. mission_occurrence_mismatch is TRI-STATE by
construction (summarise_robot says so at length): True = rows lost,
False = counted and equal, None = the binary that wrote the log never
stamped `occurrence` so nothing was compared. A bare truthiness test
maps None and False onto the same empty list, and the emptiness is
then read downstream as "0 cells lost rows" -- the exact failure the
tri-state was built to prevent, reintroduced one line below the
comment forbidding it. Measured on the bank: 792/792 robot-runs across
ts1b_n3, ts1b_n4 and cr5 are None, so the old test made this list
empty on every cell ever analysed and the NOTE below never printed
once.
```

### evlog-occurrence-uncheckable

**Robots the occurrence check cannot reach** — attached to `mission_occ_uncheckable_robots=[` (line 724)

```text
The other side of the same tri-state, carried as its own list so the
denominator survives to the report: robots where the comparison could
not be made at all. This is not a fault -- it is the reason the line
above is empty on a pre-generation-9 log -- and it exists so that
"0 of 80 cells lost rows" and "80 of 80 cells were never checked"
cannot print as the same silence.
```

### evlog-suppressed-robots

**Robots with a suppressed mission_complete** — attached to `mission_suppressed_robots=[` (line 733)

```text
Robots whose writer threw away a second mission_complete. This is the
generation-9 successor to mission_duplicate_robots: the same fault,
counted at the writer instead of by counting rows, because the rows
are no longer there to count. A `> 0` test and not a truthiness test
so that None (pre-generation-9, field absent) does not silently read
as clean.
```

### evlog-t-mean-per-robot

**Per-robot mean beside the makespan** — attached to `t_mean=(sum(finished) / len(finished)) if not censored else None,` (line 754)

```text
The per-robot MEAN of the same quantity, which is not an order
statistic and therefore does not drift with N. On ts1b the two
disagreed in MAGNITUDE (-20.2% per-robot mean vs -6.1% makespan), so a
table that quotes only the makespan understates the per-robot effect by
a factor of three. Emitted beside it, never instead of it: makespan is
the operationally meaningful one for "when can the team go home".
```

## arm_summary

### evlog-t-explore-denominator

**Censoring count for t_explore** — attached to `if r.get("t_explore") is not None:` (line 852)

```text
t_explore gets a denominator for the same reason t_mission does, and
it matters MORE, not less. Pre-registration 32.14 rule 4 promotes
t_explore to the primary inference precisely when t_mission censoring
is arm-unbalanced — so the case where the reader falls back to this
endpoint is the case where a silent, differential drop of unlatched
runs would bias it, with nothing on screen to show it happened.
```

## main

### evlog-column-widths-from-data

**Table column widths from the data** — attached to `_rw = max([len("run")] + [len(str(r.get("run", "?"))) for r in rows])` (line 918)

```text
Column widths from the DATA, never a literal. The 34 and 11 hardcoded
here were both short of what this project actually produces:
`ts1b_n4_mtare_hybrid_r40_ttl0_seed1` is 35 characters and
`mtare_rendezvous` is 16, so EVERY row of EVERY two-factor campaign
overflowed its own column and shoved the nine columns to its right
out of alignment. The numbers were all correct and the table was
unreadable, which is the same outcome as being wrong. Same fix, same
reasoning, as gate_g8.py's cell column.
```

### evlog-summary-table-widths

**Per-arm table censoring and width** — attached to `_sw = max([len("arm")] + [len(str(x["arm"])) for x in summ])` (line 979)

```text
Every mean carries its own censoring count in the adjacent column. Before
round 4 mean t_expl had none, which is the endpoint 32.14 rule 4 falls
back to when t_mission censoring is unbalanced -- so the number the
fallback rests on was the one number with no denominator on screen.
Same rule for the per-arm table, computed from ITS rows rather than
reusing the per-run width: an arm can appear in the summary that was
filtered out of the listing above, and borrowing a width from the
wrong population is how a "computed" width silently becomes a
literal again.
```

### evlog-lost-row-check-printed

**Lost-row check always prints** — attached to `lostrow = [s for s in summ if s["mission_lost_row_cells"]]` (line 1016)

```text
The lost-row check, printed WHETHER OR NOT it found anything, because its
healthy output is an empty list and an empty list is also what a check
that never ran produces. `occurrence` is stamped only by generation-9 and
later binaries, so on every cell in the bank the comparison is impossible
-- and for as long as this block was conditional, that impossibility
printed as silence, which a reader scanning the page takes for "checked,
nothing wrong". The denominator is therefore on the page in both
directions: how many cells failed, and how many could not be asked.
```

### evlog-reentry-manipulation-check

**Latch refusal count always printed** — attached to `ret_cells = sum(s["mission_reentry_cells"] for s in summ)` (line 1099)

```text
The manipulation check, and the only one of these four blocks that is
printed for a HEALTHY campaign. A guard that never reports firing and a
guard that was compiled out look identical from the outside, so the count
is put on the page whether it is zero or not -- the zero case says so in
words rather than by staying silent, because silence is what "the check
stopped checking" looks like.
```

### evlog-reentry-field-presence

**Whether the reentry field exists** — attached to `latch_seen = [rr.get("mission_return_reentries") is not None` (line 1106)

```text
Whether the FIELD exists, which is not the same question as whether it is
zero. Read off the robot records, because the arm record counts cells and
a count of cells is 0 both when no guard fired and when no guard was
compiled. An arm-level `is not None` test would have been vacuously true
and the branch below would have printed "clean" on a generation-8 log.

Counted rather than any()-ed, because a set holding both generations is
the worst case for this check: any() would call it present and the zero
branch would report a measured zero over robot-runs that were never
checkable. Reporting the denominator makes the mixture visible instead.
```
