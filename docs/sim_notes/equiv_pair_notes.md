# equiv_pair.sh — design notes and history

The long comments of `sim/equiv_pair.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 1
- [run_side()](#run_side) — 2

## Top level

### pair-owns-whole-pair

**What equiv_pair.sh owns and refuses** — attached to `set -euo pipefail` (line 4)

```text
docs/mtare_evolution_plan.md §6 requires every phase to land default-off: at
shipped defaults the new binary must be the same planner as its parent
commit. equiv_gate.py scores that claim, but it can only score runs somebody
produced, and producing them by hand is where the claim quietly rots — one
side built from a tree that was not the rev it is labelled with, or a run
launched with a treatment env var still exported from the last shell, and the
gate compares two things that are not what the report says they are.

So this script owns the whole pair: checkout, build, run, restore, compare.
Both sides come out of ONE workspace built sequentially, because that is the
workspace the harness sources and a second install space would be a different
dependency set pretending to be a control.

What it refuses to do, and why each refusal matters:

  * run on a dirty explo_planner tree. The child side is labelled with a
    commit; if uncommitted work is in the tree, the binary is not that commit,
    and the resulting PASS certifies a rev that was never tested. Untracked
    files are equally disqualifying — they survive `git checkout` and would
    sit in the parent's tree too.
  * accept a parent that is not an ancestor of HEAD. "Equivalent to its
    parent" is a claim about a specific lineage; two unrelated revs can be
    equivalent and still say nothing about what this phase changed.
  * believe a build happened. After each run it reads the `git_rev` the
    binary BAKED IN at compile time back out of the run's own event log and
    requires it to match the rev that was checked out. A skipped or failed
    rebuild leaves the previous binary in place, and two runs of the same
    binary are the most convincing false PASS this gate can produce.
```

## run_side()

### pair-build-flags

**Build flags for each side** — attached to `( set +u` (line 111)

```text
A stale binary is the failure mode this whole exercise is blind to, so the
build is not allowed to be a no-op that succeeded quietly: --packages-up-to,
because explo_planner_msgs must build first, and the same Release type the
workspace already carries, because a build-type flip would change the
binary for a reason that has nothing to do with the phase.
```

### pair-unset-treatment-knobs

**Unsetting treatment knobs for both sides** — attached to `( unset CELL_WORLD CELL_SIZE_M CELL_CENSUS_S COMMS LINK_GATE EXPLOIT \` (line 125)

```text
None of the treatment knobs: the whole point is the SHIPPED defaults.
Unset rather than trust the caller's shell.

A LEAK HERE DOES NOT PRODUCE A SPURIOUS FAIL, IT PRODUCES A FALSE
EQUIVALENT, which is why this list has to be kept in step with every phase
rather than only mostly so. equiv_gate.py short-circuits on `if a == b:
continue` and its gated-key branch fires only for keys NEW in the child, so
a knob exported in the calling shell reaches BOTH sides identically and is
never examined at all. An operator with `export RECONNECT_MODE=mtare_hybrid`
live from scoring a campaign would run both sides with all four features on,
every manifest key would compare equal, and the pair would print EQUIVALENT
— certifying "no behaviour change at defaults" from a run in which no
default was in effect on either side. The gate cannot catch it; only this
line can.

The second row was added later, after a review found three of these knobs
(RENDEZVOUS_SCHEDULE, PURSUIT_PREDICTOR from P5/P6, COORD_CLAIM_R from cr2)
missing while the paragraph above described the hazard exactly.

What this list covers, stated so the next reader does not over-trust it: the
cell/comms geometry and the per-phase FEATURE switches. It is not every
environment knob run_explo_sim_rviz.sh reads. SCENARIO, SEED, MAX_STEPS,
DONE_CRITERION, DONE_SEEK and MISSION_RETURN are all still inherited from the
calling shell, and each one leaks by the identical mechanism — exported
equally to both sides, compared equal, never examined. They are left alone
here only because widening the list is a change to what "defaults" MEANS for
this gate, which deserves its own commit and its own re-run of a known-good
pair, not a drive-by. Until then: run equiv_pair.sh from a clean shell.
```
