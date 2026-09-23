# equiv_gate.py — design notes and history

The long comments of `sim/equiv_gate.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 12
- [compare](#compare) — 1
- [main](#main) — 1

## Module scope

### derived-defaults-registry

**Params with no dp() default** — attached to `DERIVED_DEFAULTS = {` (line 102)

```text
Run_start fields that are dumped as params but are DERIVED rather than
declared: addParamNum/addParamStr writes them, so no dp("name", default) call
exists to read a default from. The value here is what the derivation yields
when its inputs are at THEIR defaults — i.e. the unconfigured state a legacy
run shows.

This is the one hand-maintained list in this file, so it is arranged to fail
loudly rather than silently: it is consulted only for params dp() did not
supply, and a param in neither place is a hard failure. Forgetting to add an
entry cannot make the gate quietly accept something.

A value may be a plain constant, or a callable taking the child's own param
dump and returning the expected value. The callable form exists for fields
derived from OTHER logged params: pinning such a field to a constant would
make this gate fail an honest pair the day that input legitimately changes,
and a gate that fails honest pairs is a gate somebody widens until it passes
everything. Deriving it instead is strictly stronger — it checks the field
against the geometry the same run recorded, so a binary that logged a flag
inconsistent with its own FOV fails here even at defaults.
```

### derived-fov-omnidirectional

**Deriving fov_is_omnidirectional from the FOV comb** — attached to `"fov_is_omnidirectional": lambda cp: (` (line 124)

```text
Not a dp() knob at all: the node computes it from the ray comb, at
explo_planner_node.cpp "FOV model is omnidirectional" —
    h_step = h_rays > 0 ? hfov / h_rays : 0
    omnidirectional = hfov >= 2*pi - h_step
one h_step of slack being the honest tolerance for a circle the comb
cannot distinguish from a closed one. Mirrored rather than pinned to
True: D1 ships a 360 deg FOV, but a directional pair must still be
scoreable, and this way the copy is checkable against the same run's
fov_hfov/fov_h_rays instead of against a remembered verdict.
```

### auto-resolved-params

**Params whose dp() argument is a sentinel** — attached to `AUTO_RESOLVED_PARAMS = {` (line 140)

```text
Params whose dp() second argument is a SENTINEL rather than a default: the
node reads "<= 0" as AUTO and resolves the field from another knob in its
constructor, BEFORE the param dump is written. So an at-defaults run never
logs the compiled 0.0, and testing against it would report every honest
defaults run as "NOT at its default" — a hard failure nobody can clear, which
is how a gate gets deleted rather than fixed.

Registering one here does NOT make it pass. It makes the gate say it cannot
certify the field, which is the truthful answer and which exit 3 exists for.
Two of the three resolve from params the node does not log at all
(vantage_visited_tol_m, candidate_max_radius), so no stronger check is
available from the log; inventing one would be a guard that cannot check.
```

### gated-manifest-groups

**Manifest keys added with their switch off** — attached to `GATED_MANIFEST_GROUPS = {` (line 179)

```text
Manifest keys a phase ADDS to the record. A phase that ships a subsystem
default-off still writes that subsystem's whole knob block to the manifest,
deliberately: "this run had the knob and it was 0" and "this run predates the
knob" are different facts, and within one binary generation the node hash
cannot separate them. run_explo_sim_rviz.sh says so at the write site. So the
child's manifest legitimately carries keys the parent's does not, and a plain
set-difference reads EVERY phase boundary as a configuration difference —
which is the fastest way to get an equivalence gate switched off.

The permission granted here is not "new keys are fine". It is the rule block 2
already applies to params, transposed: a new key is inert only if the switch
that would make it matter is present and OFF. "At its compiled default" is the
wrong test for these, because they are HARNESS variables and the harness
overrides several on purpose — CELL_COVERED_U ships 0.55 against a library
default of 0.15 (a world-calibrated knob, cf. done_unknown_fraction) and is
still inert while cell_world=0.

Each entry is  gate key -> (off value, keys it gates). Hand-maintained, and
arranged to fail loudly like DERIVED_DEFAULTS above: a new key in neither the
gate-key position nor some group's dependent set is a hard failure, so
forgetting an entry cannot make the gate quietly accept a live knob.
```

### manifest-team-world-hz

**team_world_hz in manifest versus param dump** — attached to `"team_world_hz",` (line 212)

```text
team_world_hz is written to the manifest as 1.0 even when the exchange
is off, because the launcher's TEAM_WORLD_HZ default is 1.0 and the
manifest records the harness variable, not what reached the node. The
PARAM dump is the other way round — the -p is passed only inside the
TEAM_WORLD=1 branch, so with the switch off the node logs the compiled
0.0 and block 2 above compares it against dp("team_world_hz", 0.0).
Both facts are wanted: block 2 proves the binary ran at its default,
this block proves the harness knob that would have changed that was
off. Neither substitutes for the other, and the pair is why the two
values may legitimately disagree here.
```

### manifest-p3-p4-gate-keys

**global_alloc and reconnect_gate have no dependents** — attached to `"global_alloc": ("0", set()),` (line 224)

```text
P3 and P4. Both are gate keys with no dependents: the runner passes no
other knob for either subsystem, deliberately, so that the arm under test
is the mechanism rather than a tuning of it. If that ever changes, the
new knobs belong in these sets and this gate will say so.

These two were added to the manifest by the commit that added the
mtare_hybrid arm and NOT added here, which took the phase gate offline:
equiv_pair.sh checks out the parent, so the parent side writes neither
key, both land in the "new in the child" bucket, and the pair returns NOT
EQUIVALENT on two bookkeeping lines — while certifying a commit whose own
closing claim is "no behaviour change at defaults". That is the exact
failure this registry was created to remove, arriving one commit later
through the registry itself. The lesson is the one already written above:
a hand-maintained registry is only as good as the habit of editing it in
the same commit that adds the key.
```

### manifest-rendezvous-schedule

**rendezvous_schedule has no dependents** — attached to `"rendezvous_schedule": ("0", set()),` (line 241)

```text
P5. Same shape and same reasoning: one gate key, no dependents, because
the runner passes rendezvous_schedule_enable alone and leaves every
tuning knob of the scheduler (speed, safety factor, margin) at its
compiled default. Registered in the SAME commit that added the manifest
line, which is the habit the P3/P4 note above was written to enforce
after that habit was skipped once.
```

### manifest-pursuit-predictor-off

**pursuit_predictor is off at trail** — attached to `"pursuit_predictor": ("trail", set()),` (line 248)

```text
P6. Its off value is a WORD, not "0": the knob selects which aimer the
chase uses and the shipped one is the trail, so "off" for this group is
pursuit_predictor=trail. Registering it as ("0", ...) would make an
honest defaults child fail here and an mdp child fail for the right
reason by accident — equiv_gate_calib.py pins both directions.

And this one was, again, added to the manifest by one commit and to this
registry by a later one — the same lapse the P3/P4 note above records,
repeated by the author of that note. The habit does not stick by being
written down; what catches it is the hard failure at the bottom of this
ladder, which is why that failure is not softened.
```

### manifest-claim-radius-override

**The claim-radius override is off at none** — attached to `"coord_claim_radius_override": ("none", set()),` (line 260)

```text
cr2's claim-radius override, and the two params-file witnesses added
beside it. Registered in the same commit that adds the manifest lines —
the habit the P3/P4 and P6 notes above were written to enforce, having
now been skipped twice and caught the third time by this ladder's bottom
rung rather than by anyone remembering.

The override's off value is the WORD "none", not "0", and deliberately so:
the runner passes no -p at all when COORD_CLAIM_R is unset, and the node
reads a claim radius <= 0 as AUTO. Registering this as ("0.0", ...) would
certify a cell that ran the 10 m auto radius as if it had been pinned
there, which is the confound the "none" spelling exists to prevent.
```

### manifest-params-file-witnesses

**Params-file witnesses track the shipped yaml** — attached to `"coord_claim_radius_m_in_params": ("10.0", set()),` (line 272)

```text
These two restate what the params file said, so their off value is the
shipped yaml value: coord_claim_radius_m: 0.0 (auto) and
global_alloc_comms_mask: false. A child reading "missing" fails here
rather than relaxing — correct, because a missing params file is the very
failure these witnesses were added to expose.
Re-pinned 0.0 -> 10.0 by D1, which stops relying on the AUTO path: the
node reads <= 0 as "use fov_max_range", and D1 doubles fov_max_range to
20.0, so leaving the yaml at 0.0 would have silently doubled the claim
disc as a side effect of a sensor change. Pinning 10.0 holds the disc
where gen-8 actually ran it. The value here must track the yaml, per the
rule stated above; it was 0.0 and the yaml now says 10.0, so an honest
gen-9 child would have failed this rung on a stale pin.
```

### manifest-derived-keys

**Manifest keys restating the scenario** — attached to `DERIVED_MANIFEST = {` (line 288)

```text
Manifest keys that RESTATE a key both sides already record, rather than
describing a switch. P7 writes the roster (robots, n_robots) into the
manifest; it is derived from the scenario YAML, which the manifest has always
recorded and which the comparison above already fails on when it differs.

These cannot go in GATED_MANIFEST_GROUPS: there is no "off" value for a
roster, and inventing one would mean declaring some team size inert. The rule
applied instead is narrower than the gated-group rule, not looser — the key
is relaxed only when its SOURCE key is present on BOTH sides and equal, so a
pair that changed scenario, or that cannot show which scenario it ran, still
fails. Relaxing them is not a new permission: the parent had a roster too,
it simply did not write it down, so nothing that was previously compared
stops being compared here.
```

### manifest-alias-keys

**Renamed manifest keys and their check** — attached to `ALIAS_MANIFEST = {` (line 306)

```text
Keys that are a pure RENAME of another key: not merely derived from it, but
carrying the identical value under a second spelling, because a rename kept
the old name writing alongside the new one.

Separate from DERIVED_MANIFEST because a stronger check is available and a
strictly stronger check should not be optional. A roster cannot be compared
against the scenario it came from — different values by nature — so that
relaxation can only ask whether the SOURCE agrees across the two sides. An
alias can be compared against its own source within one manifest, and is:
both conditions below must hold. Without the second, a manifest claiming
rendezvous_enabled=false and reconnect_enabled=true reads as EQUIVALENT
against a parent that also says false — the gate would be trusting an
invariant that lives in a different file (run_explo_sim_rviz.sh echoes both
lines from one variable) and would go silent the day that file changed.
```

## compare

### check-3b-event-shape

**Check 3b compares event field shapes** — attached to `def merge_shape(side):` (line 670)

```text
3b. the SHAPE of each shared kind -------------------------------------
Check 3 compares the NAMES of the event kinds and stops there, so a child
that keeps every name and rewrites what is inside them passes it clean.
That is most of what a phase actually does: `rendezvous_commit` gaining a
`provisional` flag, a field renamed, a count changing from int to string.
None of it is visible above, and all of it changes what every downstream
reader gets — which is the same failure as a gate that stopped checking,
arriving through the one door this file left open.

Direction matters and the two are not symmetric. The CHILD is what is
being certified, so a field in the child and in no parent run anywhere is
a FAIL: across a whole population the parent's writer never once emitted
it. The reverse is a NOTE — a field the parent wrote and the child did not
reach can be a conditional field on a branch the child's runs missed, and
failing on that is how a gate starts failing honest pairs.

Unions are taken across the whole side, never per run, for the same
reason: a field written only on some branches is present in some runs and
absent in others within ONE binary, and a per-run comparison would call
that a difference between binaries.
```

## main

### exit-3-unresolved

**Unresolved is exit 3, not a pass** — attached to `print(f"UNRESOLVED  no differences found, but {len(unresolved)} "` (line 903)

```text
The distinction this file exists to keep: "I checked it and it
matched" and "I could not check it" are different answers, and a
wrapper branching on $? must not read the second as the first. Same
reasoning as the empty-side case above, which is why it is the same
exit code.
```
