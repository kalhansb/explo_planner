# equiv_gate_calib.py — design notes and history

The long comments of `sim/equiv_gate_calib.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 8
- [audit_fixture_against_real_cell](#audit_fixture_against_real_cell) — 3

## Module scope

### calib-pursuit-predictor-string

**pursuit_predictor pins the string parser** — attached to `case("the new pursuit_predictor at its compiled default", 0,` (line 336)

```text
P6's knob, the same pair. This one is a STRING default read out of
dp("pursuit_predictor", std::string("trail")), so it also pins that
_literal() still understands that initialiser form: teach the parser to
mis-read std::string(...) and the first of these two fails on a message about
the parser refusing to guess, not on a message about the arm.
```

### calib-cell-block-off

**The P1 cell block fixture** — attached to `CELL_BLOCK_OFF = ("cell_world=0\n"` (line 431)

```text
Copied from the P1 pair that first hit this: a run of the P1 harness with
CELL_WORLD unset writes all seven of these, and the P0 parent writes none of
them. Before GATED_MANIFEST_GROUPS the gate called that seven configuration
differences, which is a verdict of "not equivalent" against a subsystem that
was switched off.
```

### calib-team-block-off

**The P2 team block fixture** — attached to `TEAM_BLOCK_OFF = ("team_world=0\n"` (line 477)

```text
P2 adds a second block on top of P1's, so its equivalence pair has the cell
block on BOTH sides and the team block on the child only. Note the recorded
rate: TEAM_WORLD_HZ defaults to 1.0 in the launcher and the manifest records
the harness variable, so "off" here reads as team_world=0 with a non-zero
hz — the value that never reached the node, because the -p is passed only
inside the TEAM_WORLD=1 branch. If this case ever starts failing on the hz,
the fix is not to zero it in the manifest: block 2 of the gate is what proves
the node ran at its compiled 0.0, and this block is what proves the harness
knob was off. They are different facts and they are allowed to differ.
```

### calib-p3-p6-stack

**The P3-P6 gate keys fixture** — attached to `STACK_BLOCK_OFF = ("global_alloc=0\n"` (line 505)

```text
P3 through P6. Four gate keys with no dependents, so there is no
switch-missing case to write for them — the switch IS the whole group. What
there is instead, and what the P1/P2 blocks above cannot test, is that two of
the four are off at a WORD rather than at "0". A registry entry of
("0", set()) for either would fail an honest defaults child on a bookkeeping
line, and the fix somebody reaches for under time pressure is to delete the
check. Both directions are pinned below, per key.

These blocks exist at all because the registry has now been forgotten twice:
global_alloc and reconnect_gate were written to the manifest one commit
before they were declared, and pursuit_predictor one commit before that
again. Neither lapse was caught by a calibration case, because until now the
calibration stopped at P2 — it tested the mechanism on the two oldest groups
and said nothing about the four that came after.
```

### calib-reconnect-rename

**The renamed reconnect switch cases** — attached to `case("the renamed reconnect switch, restating a key both sides record", 0,` (line 574)

```text
The 2026-09-03 rename of the reconnect master switch. The harness echoes one
variable to both spellings, so the new key restates the legacy one — which
both sides still write, and which the direct comparison above already fails
on. The case that has to hold is the second: when the two sides ran DIFFERENT
arms, the new key must fail on its own line rather than being waved through
beside the legacy key's failure, or the relaxation becomes a way to smuggle an
arm change past a reader who saw one complaint and stopped reading.
```

### calib-alias-intra-manifest

**Catching two disagreeing switch spellings** — attached to `case("a child whose two spellings of the switch disagree", 1,` (line 593)

```text
The alias relaxation rests on the harness echoing both spellings from ONE
variable — an invariant enforced in run_explo_sim_rviz.sh, not here. This is
the case that notices if that ever stops being true. Without the intra-manifest
check the pair below reads EQUIVALENT: the source key agrees across the sides,
and nothing would look at what the child says under the new name.
```

### calib-directional-fov-child

**A directional child fails on the FOV** — attached to `rc, out = run(parent_side(),` (line 705)

```text
A directional child DOES fail -- D1 ships 360 deg, so a 60 deg comb is not a
defaults run -- but it must fail on the FOV it actually changed. If the
derived flag were pinned to True it would fail here a SECOND time, for a
reason that is not true, and the real finding would be one line of noise in
a pile. Asserted by reading the failure list, because what is being checked
is the ABSENCE of a line.
```

### calib-auto-sentinel-not-counted

**Declined params are not counted as checked** — attached to `rc, out = run(parent_side(),` (line 738)

```text
...and equally must not be quietly counted as one of the params that WERE
checked. A gate that says "1 new param at defaults" about a param it declined
to check is back to printing passes for things it never looked at.
NOT "the note is absent": this child also carries P0's three genuine new
params, so the note is printed and should be. What must not appear is the
declined param's NAME inside it.
```

## audit_fixture_against_real_cell

### calib-audit-scan-depth

**Audit scan depth and newest schema** — attached to `real_start, real_params, real_schema, real_where = None, None, -1, None` (line 817)

```text
Scan for the NEWEST schema present rather than stopping at the first cell
alphabetically. The fixture is written at the current schema, so auditing
it against the oldest banked cell in the directory would silently compare
it to a run_start that predates half the params it claims to copy.

AND IT WAS DOING EXACTLY THAT. The scan was one level deep, so it could
only reach cells sitting loose at the top of ~/hmr_campaign -- 3467 of
them, none newer than schema 4. Every campaign since then writes its cells
one level further down (ts4_smoke20_n2/<cell>/), so the schema-5, -6 and
-7 runs were invisible and this audit had been certifying the fixture
against a four-generation-old run_start while printing PASS. The comment
above was true about the loop and false about the outcome, which is the
failure mode this whole review pass is about.

Depth 2 is the campaign/cell layout and is where the scan stops; a deeper
walk would spend minutes crossing bag directories for nothing.
```

### calib-audit-both-directions

**Fixture run_start audited in both directions** — attached to `missing = real_start - fixture_start` (line 875)

```text
BOTH DIRECTIONS, and only one of them used to be checked. `invented` says
the fixture made something up; `missing` says the writer has moved on
without it -- and a fixture that is merely BEHIND passes every subset test
ever written while testing the wrong shape. The two are not symmetric in
how they are treated, because the two halves of run_start are not:

  top-level keys ARE a failure. There are twelve of them, the writer emits
  the same twelve on every event kind plus t0_sim_sec/coverage_milestones,
  and as of schema 7 the fixture has exactly that set. So equality is the
  real invariant here, and a new one appearing is the writer changing
  under the gate -- which is precisely what the gate exists to notice.

  params are NOT. The fixture carries 16 of the 138 a real run dumps, on
  purpose: the gate compares params generically, key by key, so a fixture
  that copied all 138 would test the same code path 138 times and go stale
  every time anyone declares a parameter. Missing params are reported as a
  count for reach, never as a failure.
```

### calib-audit-reference-age

**Tripwire on the audited reference's age** — attached to `hdr = os.path.join(HERE, os.pardir, "explo_planner", "include",` (line 920)

```text
AND A TRIPWIRE ON THE REACH, separately from the shape. Everything above
compares the fixture to whatever cell the scan found; none of it can tell
a current reference from a stale one, because the top-level run_start keys
have not changed since schema 3 and so an ancient cell agrees just as
loudly. That is how the one-level scan survived four generations here: it
kept finding a schema-4 cell, kept agreeing with it, and kept printing
PASS. The staleness has to be its own verdict or nothing checks it.

The header is the truth for the current schema — the binary stamps
kSchemaVersion into every run_start and nothing else votes. Reading it
here rather than pinning a number is deliberate: a second hand-kept pin is
what gate_g8_calib.py had to grow an assertion to police, three drifts in.

One behind is normal and stays green: the pin moves with the header, and
no cell of the new generation exists until that binary has been built and
run. Two behind means a whole generation was campaigned without this scan
ever reaching one of its cells.
```
