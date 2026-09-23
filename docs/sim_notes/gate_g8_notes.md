# gate_g8.py — design notes and history

The long comments of `sim/gate_g8.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 1
- [robots_of](#robots_of) — 1
- [Module scope (part 2)](#module-scope-part-2) — 2
- [_declared_identity](#_declared_identity) — 1
- [Module scope (part 3)](#module-scope-part-3) — 1
- [check 3l: the configuration that must not vary silently](#check-3l-the-configuration-that-must-not-vary-silently) — 5
- [_regime_val](#_regime_val) — 1
- [check 3l: the configuration that must not vary silently (part 2)](#check-3l-the-configuration-that-must-not-vary-silently-part-2) — 5
- [D2 witness: the simple_nav_3d BINARY (check 3, extra key)](#d2-witness-the-simple_nav_3d-binary-check-3-extra-key) — 1
- [The M-TARE feature vector each arm token is DEFINED to carry (check 3g).](#the-m-tare-feature-vector-each-arm-token-is-defined-to-carry-check-3g) — 2
- [mtare_expect](#mtare_expect) — 1
- [The M-TARE feature vector each arm token is DEFINED to carry (check 3g). (part 2)](#the-m-tare-feature-vector-each-arm-token-is-defined-to-carry-check-3g-part-2) — 27
- [3h. ONE BINARY GENERATION PER CAMPAIGN.](#3h-one-binary-generation-per-campaign) — 1
- [3l. ONE SENSOR AND CANDIDATE CONFIGURATION PER CAMPAIGN.](#3l-one-sensor-and-candidate-configuration-per-campaign) — 2
- [_regime_str](#_regime_str) — 1
- [3j. ONE RADIO REGIME PER CAMPAIGN, AND IT MUST BE THE DECLARED ONE.](#3j-one-radio-regime-per-campaign-and-it-must-be-the-declared-one) — 6

## Module scope

### gate-report-only-gates

**Report-only comms gates** — attached to `REPORT_ONLY_GATES = frozenset({"map_agree"})` (line 67)

```text
Gates in comms_gates.txt that are READINGS, not thresholds: they describe the
run, they cannot condemn it. Used by check 2 only, and only to keep a line
such a gate was never entitled to write from failing the whole cell. Adding a
name here is a claim that the gate has no pass criterion — check the gate's
own header before doing it.
```

## robots_of

### gate-roster-from-logs-last-resort

**Roster from event logs, last resort** — attached to `found = sorted(os.path.basename(q)[:-len(".events.jsonl")]` (line 105)

```text
Last resort: whatever event logs are on disk. This is DELIBERATELY not
the primary source -- deriving the roster from the files present makes
"a robot's log is missing" unobservable, since the missing robot simply
drops out of the expectation. It is used only when the manifest names no
team at all, and the caller says so in its output.
```

## Module scope (part 2)

### gate-runtime-vs-design-suffixes

**Runtime versus design cell suffixes** — attached to `RUNTIME_SUFFIXES = ("_seek",)` (line 117)

```text
Suffixes a cell directory may carry that the BINARY never stamps into
run_start's `arm`. run_campaign.sh strips each before setting RECONNECT_MODE,
so a directory carrying them holds a node that stamped the bare policy name,
and anything compared against run_start has to compare the stripped form.

Two kinds, and the difference decides whether the suffix survives into the
cell's IDENTITY:

  RUNTIME  — a switch that is not a design dimension of any campaign using it.
             Pooled away entirely: `arm` loses it, so cells with and without
             it count into the same bucket.
  DESIGN   — an independent variable. Stripped for the comparison against the
             binary (policy_arm) but KEPT in `arm`, because folding r10 and
             r40 into one bucket would leave check 21 unable to notice that
             half a design never ran, which is the precise failure check 21
             exists for. Each also gets a manifest cross-check (3i, 3k2), on
             the same argument as 3e one level down: the analysis reads the
             level off the directory name, so a launcher bug that runs the
             other level silently swaps the design's two columns.
```

### gate-declared-identity-out-of-band

**Why the identity is declared out of band** — attached to `NAV_BIN_KEY = "sha256_simple_nav_planner_node"` (line 227)

```text
Declared identity of generation 8 (pre-registration section 32.14).

These are DECLARED CONSTANTS, deliberately not read from the working tree:
the gate's job is to prove the cells came from the binary the pre-registration
names, and a gate that reads the identity from whatever happens to be built
would pass for any binary at all.

Two of them CANNOT be literals here, for a reason that is structural and not
a matter of taste. The manifest's git_explo_planner is `git rev-parse HEAD`
at run time, and the JSONL's git_rev is baked into the binary at CMake
configure time — so both name the commit the campaign was built from. This
file is IN that commit. A file cannot contain its own commit hash, and the
binary's sha256 has the same problem, since the rev string is compiled into
it: filling either literal changes the tree, which changes the commit, which
changes both values again. There is no fixed point.

So they are declared OUT OF BAND, in a file written once at campaign launch
and living outside git:

    $GATE_ROOT/<TAG>.identity.txt      (key=value, one per line)

That preserves the property that matters — the gate is told what to expect by
something it does not compute — while being physically possible. The
pre-registration records the same four values, and the identity file being
outside git is not a weakness here: it is written BEFORE the first cell runs
and any later edit is visible in its mtime against the campaign's own cells.

Env overrides exist for the calibration harness, which must point the real
logic at synthetic cells of known identity.
```

## _declared_identity

### gate-identity-empty-value

**Empty identity values are skipped** — attached to `if v.strip():` (line 280)

```text
An empty right-hand side is not a declaration. Accepting it put
"" into EXPECT, which is not FILL_ME, so the refusal below did
not fire and every cell instead hard-failed with "expected "
and nothing after it — pointing the operator at the cells when
the fault is a truncated identity file.
```

## Module scope (part 3)

### gate-no-literal-revs

**No identity field is a literal** — attached to `EXPECT = {` (line 297)

```text
ALL FOUR are declared, none is a literal. git_simple_nav_3d and git_scovox
used to be hardcoded here ("c9f83a7" and "078d3f7"), which was invisible for as
long as the gate was only ever pointed at generation-8 cells, where those revs
were in fact correct. The moment simple_nav_3d changed -- which it does in
generation 9, for D2/D2b -- EVERY cell hard-failed check 3 on a non-defect, and
nothing in the identity file or the environment could clear it. An operator
facing a gate that cannot be satisfied does one of two things, and both are
worse than the bug: abort a good campaign, or learn to skim past check-3 lines.
Check 3 is the one that catches real cross-generation pooling, so teaching the
reader to ignore it is the expensive failure. A pin that only a source edit can
move is a pin that gets moved under time pressure, months later, by someone who
just wants the gate to go green -- which is exactly how the eight guards in
`checks-that-stopped-checking` went inert while still printing PASSes.
```

## check 3l: the configuration that must not vary silently

### gate-3l-config-hard-soft

**Check 3l hard and soft config keys** — attached to `CFG_HARD = ("fov_hfov", "fov_vfov", "fov_h_rays", "fov_v_rays",` (line 326)

```text
The sensor model and the candidate generator. None of these is an arm's
treatment in any campaign this gate scores, so variation in them is not a
design -- it is a stale installed shared_params.yaml, a -p override that
reached some cells and not others, or two invocations pooled.

It needs its own check because the existing provenance cannot see it.
sha256_shared_params moves if the yaml changes, but it is in no EXPECT set
and nothing compares it; and it says the file differed, never WHICH value
differed -- the same complaint the harness itself writes three times over the
*_in_params keys. git_explo_planner does not move for a params change at all.

CFG_HARD are campaign-wide constants. CFG_SOFT have been independent
variables before (cr2 varied the claim radius by arm), so between-arm
variation in them is a legitimate design and is reported, not failed --
but WITHIN one arm they are still constants, and that half stays hard.
```

### gate-schema-pin-override

**Schema pin override for banked campaigns** — attached to `SCHEMA_VERSION = int(os.environ.get("GATE_SCHEMA_VERSION", "11"))` (line 369)

```text
The event-log schema this gate scores against (check 3d). Overridable for the
same reason as everything else in this block: a banked generation-9 campaign
was written at schema 3 and is still perfectly valid on its own terms, so
re-gating it must be possible WITHOUT editing the pin. Editing the pin is how
a gate stops checking; passing GATE_SCHEMA_VERSION=3 says out loud, in the
invocation, which generation is being scored.
```

### gate-schema-version-history

**Schema version bump history** — attached to `SCHEMA_VERSION = int(os.environ.get("GATE_SCHEMA_VERSION", "11"))` (line 379)

```text
Bumped to 4 for the M-TARE evolution. The v4 additions are all default-off, so
a v4 binary run at shipped defaults is behaviourally the same planner as v3 —
but "the same behaviour" is a claim for equiv_gate.py to prove per phase, not
something this file may assume. Here the stamp stays what it has always been:
a one-field tripwire on mixing generations in one campaign directory.

Bumped to 5 on 2026-09-16 (generation 10), when the rendezvous appointment
stopped being derived independently on each robot and became a (cell,
interval) pair exchanged over TeamWorld and committed by unanimity. Unlike the
v4 bump this one is BEHAVIOURAL and not default-off: the rendezvous and hybrid
arms of a v5 campaign are running a different mechanism from the same-named
arms of a v4 one. The pin must move with the binary — check 3d is an exact
equality, so a stale pin here rejects every cell of the current generation
while the calibrator, which pins the gate's env to its own constant, goes on
reporting PASS against a schema nothing writes.

THE HISTORY ABOVE STOPPED AT v5 UNTIL 2026-09-18 while the pin below read 8,
which is the exact failure this block was written to prevent, committed in the
block itself. The three missing bumps:

  v6 (generation 17) — MEANING-ONLY. `RendezvousAgreedEvent::t_meet_sec` went
    from "the next meeting instant" to the phase offset of a repeating
    interval. Same field, same type, same name, different quantity. Nothing
    rejects a v6 file read as v5; it just answers a different question.
  v7 (generation 19) — MEANING-ONLY. A new `rendezvous_outcome` label, so the
    set of values a reader must switch on grew without the key changing.
  v8 (generation 23, today) — three meaning changes and three additive
    fields. See the block comment on the schema constant in experiment_log.hpp
    (`kSchemaVersion`), which is the authority; do not re-derive the list here
    and do not let the two drift again.

v9 (generation 25, 2026-09-18) moved the same day as the header — MEANING-
ONLY: the arming floor behind `t_meet_sec` went from now-plus-notice to bare
now, and arrived=false outcome rows split into barrier conversions vs
strandings. kSchemaVersion in experiment_log.hpp remains the authority.

A MEANING-ONLY BUMP IS STILL A BUMP, and those are the ones that get skipped:
nothing fails to parse, so the pressure to record them is entirely absent, and
a reader pooling v6 with v5 gets numbers rather than an error. The pin exists
to make that pooling impossible inside one campaign directory. It cannot do
that job if the reason for a version is only in the commit that raised it.

v10 (generation 29) and v11 (generation 31) moved with the header and are
recorded there, not here — the paragraph above already says the authority is
`kSchemaVersion` and that this list must not be re-derived. v11 is the first
VOCABULARY widening since v4: `appointment_leg`.
```

### gate-3f-treatment-overrides

**Check 3f overrides and calibration** — attached to `MIDRUN_SILENCE_EXPECT = float(os.environ.get("GATE_MIDRUN_SILENCE", "90"))` (line 426)

```text
Generation-9 treatment configuration (check 3f). Overridable so the check can
be calibrated against a known-answer case — a gate that has never been shown
to FAIL on a bad input is not evidence of anything, which is the lesson the
3b/3c git_rev bug taught this file.

EVERY sub-check is overridable, not just the clock. An earlier draft exposed
only GATE_MIDRUN_SILENCE, and the amendment claimed check 3f "goes silent when
told to expect 240". It does not: on g8r1 that override takes 138 hard
failures to 92, not to 0, because the three params generation 9 introduced
have no escape and no generation-8 binary can ever carry them. Re-gating a
banked campaign needs GATE_GEN9_PARAMS=0 as well:
```

### gate-3m-endpoint-once

**Check 3m endpoint declared once** — attached to `ENDPOINT_FIELDS_REQUIRED = os.environ.get("GATE_ENDPOINT_FIELDS", "1") != "0"` (line 457)

```text
Generation 8 shipped a DONE -> RETURN_HOME re-entry: a run that had already
homed could be sent home a second time by the late coverage latch, because the
only guard tested `state_ == State::RETURN_HOME` and the second request
arrives from DONE. On ts1b that produced 16 robot-runs carrying two
mission_complete rows — 1/3/12 at N=2/3/4, all in the treatment arm and none
in any control. Two rows is two homing budgets, a refilled escape ladder, and
homing_duration_sec/homing_distance_m restarting from zero on the second one.

Generation 9 closes it in two places and this check reads both:

  * the node latch (mission_return_done_) refuses the second homing LEG. What
    it refused rides out on run_end.mission_return_reentries, which this check
    does NOT require to be zero — a late coverage latch legitimately asks, and
    the refusal is the fix working.
  * the writer latch (ExperimentLog::logMissionComplete) refuses the second
    ROW, counting it into run_end.mission_completes_suppressed. That one MUST
    be zero: with one call site inside finishMissionReturn and the node latch
    in front of it, a suppressed row means finishMissionReturn re-entered
    without passing startReturnHome — a path the node guard cannot see.

Requiring the fields to EXIST is the other half, and the more important one.
Generations 8 and 9 both stamp schema_version 4, so check 3d cannot separate
THOSE two, and a zero read off a field that was never written is the exact
shape of a check that stopped checking. (Generation 10 stamps 5 and IS
separable by the stamp; this argument is about the 8/9 pair only, which is
what GATE_ENDPOINT_FIELDS=0 exists for.)
GATE_ENDPOINT_FIELDS=0 scores a banked pre-generation-9
campaign on its own terms; it relaxes only the existence requirement, and the
"at most one row" rule below still runs, because that one is readable on every
generation ever written.
```

## _regime_val

### gate-3j-radio-regime-pin

**Radio regime pin and override** — attached to `def _regime_val(raw):` (line 490)

```text
The radio regime (check 3j). Three manifest fields decide what "the link
dropped" MEANS on a run: how much a trunk costs, how far a trunk-free link
reaches, and how much power it started with. Every result this project
reports about connectivity is conditional on them.

They moved on 2026-09-03. tree_attenuation_db went from 11.98 dB — the IEEE
9260568 per-trunk fit, under which a single trunk almost never dropped a link
— to 70.0, where any trunk in the Fresnel zone is fatal; max_range_m appeared
at the same time and bounds a CLEAR lane at 30 m, which nothing did before.
cr3/cr4/cr5 and everything earlier ran the old radio. Pooling across that is
forbidden, and until this check existed nothing read the fields: the manifest
recorded the difference and every analysis averaged over it.

Overridable in the same spirit as GATE_SCHEMA_VERSION above, and for the same
reason: a banked campaign is perfectly valid on its own terms and must be
re-scorable WITHOUT editing the pin, by saying which regime it ran out loud
in the invocation —
```

## check 3l: the configuration that must not vary silently (part 2)

### gate-expect-arms-declared

**Arms are declared, not inferred** — attached to `EXPECT_ARMS = tuple(` (line 553)

```text
The arms the campaign was pre-registered with. Was the literal pair
("hybrid", "off") in check 21, which made every campaign that is not
hybrid-vs-off fail as "unexpected arm(s)" — including off vs mtare_hybrid,
which is the same experiment with a different treatment. Declared rather than
inferred from the directories on disk: inferring it would make check 21
incapable of noticing the truncation it exists to catch, since a campaign that
only ever ran one arm would "expect" exactly that arm.
`.strip()`: these arrive as a shell string, and `GATE_ARMS="off, mtare_hybrid"`
would otherwise declare an arm named " mtare_hybrid" that no cell can ever
match — check 21 then reports the real arm as unexpected AND the phantom as
missing, and the campaign fails for a space.
```

### gate-control-arms-default

**Control arms and their default** — attached to `CONTROL_ARMS = frozenset(` (line 569)

```text
The control arm, and the ONLY arm that runs with reconnect_enabled=false.
Everything else — hybrid, pursuit, rendezvous and the three treated
`mtare_*` tokens — reaches the manoeuvre and must have it enabled. Check 3e used to spell this as
`arm == "hybrid"`, which asserted reconnect_enabled=False for a pursuit or
rendezvous cell and would have hard-failed a correct run of either.

The default is `off` ALONE. It briefly shipped as "off,mtare_off" on the
theory that a future allocator-on/reconnect-off control would want the same
exemption, and that was a pre-authorised hole: `mtare_off` is a name the node
can already stamp (its arm string is "mtare_" + "off" whenever the allocator
is on and rendezvous is not), so the day that token is added to the runner's
case list, a cell carrying a decision-changing treatment would arrive with
check 3f skipped and 3e satisfied — every assertion about its treatment
exempted in advance, by a default nobody had to type. An arm earns the
exemption by being declared at scoring time, not by being guessed at here.

THAT DAY HAS ARRIVED and the default has deliberately not moved. `mtare_off`
is now a real runner token — the control cell of the §3.6.1 factorial, which
carries P1-P3 and neither reconnect mechanism. Scoring that campaign
therefore requires GATE_CONTROL_ARMS="mtare_off" to be typed out, and the
check below refuses a control arm that no cell is running, so the typing
cannot be wrong in the quiet direction.
```

### gate-orphan-control-arms

**Control arms must match an expected arm** — attached to `_expect_policies = {a: parse_cell_name(a, TAG)[1] for a in EXPECT_ARMS}` (line 596)

```text
A control arm nobody is running is a control arm that exempts nothing, and it
is the exact shape a typo takes: GATE_CONTROL_ARMS=of leaves check 3f demanding
reconnect config from the `off` cells, which fails loudly, but
GATE_CONTROL_ARMS=off with GATE_ARMS naming no `off` arm fails silently — the
exemption sits there unused while the campaign has no control at all.

Matched at BOTH levels, because control-ness is a property of the reconnect
policy and GATE_ARMS carries full cell identities: a two-factor campaign whose
arms are `mtare_off_r40_ttl0` and `mtare_hybrid_r40_ttl0` has a control, and
`GATE_CONTROL_ARMS=mtare_off` names it correctly. Refusing that spelling would
force the operator to repeat every design suffix in a second variable, where a
copy that drifts from the first is silent. The typo protection is unchanged:
a token matching neither an expected arm nor the policy of one still exempts
nothing and still aborts.
```

### gate-planner-log-not-console

**Planner log, not the console log** — attached to `PLANNER_LOG = "planner_{robot}.log"` (line 704)

```text
THE FIRST DRAFT OF THIS CHECK READ THE CONSOLE LOG. It would have hard-failed
every treated robot-run of every real campaign -- and not as an UNRESOLVED
skip, because that file exists (check 9 depends on it), so the probe returned
"log present, token absent" rather than "cannot answer". The calibration
fixture planted the token in the same wrong file, so the known-answer case
agreed with the bug and reported ALL PASS. A probe and its calibration written
from the same wrong assumption check nothing [[checks-that-stopped-checking]];
this one is now pinned to the path start() actually writes.
```

### gate-cell-enumeration-attempts

**Cell enumeration and .attempts siblings** — attached to `CELL_RE = re.compile(r"^" + re.escape(TAG) + r"_.+_seed\d+$")` (line 769)

```text
Anchored on a trailing _seed<N>, the way event_log.py and modes_compare.py
already spell it. `startswith(TAG + "_")` alone also swept up the
`<cell>.attempts/` directories run_campaign.sh creates beside a cell it is
redoing (it keeps the failed attempt's evidence rather than deleting it).
Those parse to a perfectly good arm name — "mtare_hybrid_seed7.attempts"
rsplits to "mtare_hybrid" — so every redone cell was counted TWICE in
check 21 and then hard-failed a second time for having no manifest and no
event log. A redo is not a defect and REDOs correlate with arm, so this
inflated one arm's count on exactly the campaigns that needed scoring most,
and the operator's only escapes were to raise GATE_CELLS_PER_ARM or delete
the evidence — each of which disables a check.

Excluded by NAME, not by shape: dropping everything that fails to match
`_seed<N>$` would also drop a genuinely malformed cell directory, and a
malformed cell is something this gate must shout about, not skip. So
`.attempts` — the one sibling the harness is known to create — is removed
explicitly, and anything else that does not parse is still enumerated and
still hard-fails below on its missing manifest.
```

## D2 witness: the simple_nav_3d BINARY (check 3, extra key)

### gate-nav-binary-witness

**Self-calibrating nav binary witness** — attached to `_nav_present = [c for c in cells` (line 802)

```text
The four keys in EXPECT pin one binary and three SOURCE trees. Generation 9
puts a behavioural change -- D2, the goal-snap append -- in a binary that none
of them covers. git_simple_nav_3d moves with the source tree, so it moves
whether or not colcon ran; a source rev cannot witness a rebuild. That left
D2's one failure mode (edited, not rebuilt: the node on the wire is still
generation 8 while every provenance field says generation 9) invisible to
every check in this file.

The requirement is self-calibrating rather than unconditional, and that is the
load-bearing part. Cells banked before the harness emitted the key cannot
answer, and a key that hard-fails 80 good cells with no way to clear it is a
key that gets deleted by the next person under time pressure -- which is the
bug this gate just had for git_simple_nav_3d, and the shape of every guard in
`checks-that-stopped-checking`. So ask the campaign which case it is:

  key in NO cell    -> the campaign predates it. One INFO line saying D2 is
                       unwitnessed here. Not a failure, and not silence.
  key in SOME cells -> the harness changed mid-campaign. Hard-fail the cells
                       that lack it; a split provenance is a real defect
                       ([[commit-mid-campaign-splits-provenance]]).
  key in EVERY cell, undeclared -> refuse. The campaign CAN answer and the
                       operator has not said what the answer should be, which
                       is exactly the FILL_ME case for the other four.
```

## The M-TARE feature vector each arm token is DEFINED to carry (check 3g).

### gate-mtare-arm-stack-table

**The M-TARE arm feature table** — attached to `MTARE_ARM_STACK = {` (line 852)

```text
Until P5 this was a single boolean — every feature ON for an `mtare_*` arm,
every feature OFF otherwise — because there was exactly one m-tare arm and it
carried the whole stack. The doc §3.6.1 factorial ends that: its four arms
share the P1-P3 stack (cell world, exchange, allocator) and differ precisely
in the two reconnect MECHANISMS, so "all features on" would reject three of
the four cells of the design this check exists to certify.

`mtare_off` carries reconnect_gate=silence on purpose and not as an
oversight: the gate can only suppress a dispatch, and that arm makes none, so
`info` there would pin an inert knob. What is emphatically NOT inert in it is
the allocator, which is why the factorial's control is `mtare_off` and not
plain `off` — a plain-`off` control would confound the two mechanisms under
test with P3.

`mtare_rendezvous` and `mtare_hybrid` have identical vectors. They are
separated by reconnect_mode / reconnect_enabled, which check 3e already
compares against the directory name; this table is about the stack, not the
mode.

P6 ADDED A SIXTH FEATURE AND TWO ARM NAMES, and the table has to carry both
or it rejects the campaign it exists to certify. `pursuit_predictor` selects
how a chase is AIMED — `trail` drives at the peer's last declared goal,
`mdp` at a modelled intercept — so `mtare_hybrid` and `mtare_hybrid_mdp` are
two treatments, exactly as `mtare_pursuit` and `mtare_hybrid` are. The node
stamps the suffix itself (explo_planner_node.cpp: `if (pursuit_predictor_mdp_)
arm += "_mdp"`), so the directory name and the stamp agree and check 3e is
silent — which is precisely why this table not knowing the token is
dangerous rather than noisy: without the two `_mdp` rows every cell of the
two chasing arms hard-failed 3g as "not an arm any binary can stamp", and
the four-arm ts4 design is half `_mdp`.

The suffix is a FEATURE here and not merely a name, for the same reason
every other row is a vector: `mtare_hybrid_mdp` carrying pursuit_predictor=
trail is the treatment arm silently running its own control, and nothing
else in this gate looks at that param.
```

### gate-mtare-arms-pre-p5

**Arm tokens a pre-P5 binary could stamp** — attached to `MTARE_ARMS_PRE_P5 = {"mtare_hybrid"}` (line 913)

```text
Which m-tare arm tokens a PRE-P5 binary could have stamped. Only one: the
three factorial names did not exist, and the runner refused every route to
`mtare_off` (the arm-name/arm-stamp check rejected a plain token with the
allocator on, and RECONNECT_GATE=info with RECONNECT_MODE=off was fatal). So
seeing one of them on a pre-P5 cell means the directory was renamed after the
fact, not that the binary ran that arm.
```

## mtare_expect

### gate-untreated-arm-vector

**Untreated arms carry no features** — attached to `want = {k: False for k in MTARE_ARM_STACK["mtare_hybrid_mdp"]}` (line 947)

```text
An untreated arm carries none of it, in either generation. Checked in
this direction too, and this is the direction that decides the
comparison: a control cell that somehow ran the allocator is a
treated cell sitting in the control column, and no amount of care in
the treated arm compensates for that.
```

## The M-TARE feature vector each arm token is DEFINED to carry (check 3g). (part 2)

### gate-3m-populations

**Check 3m populations** — attached to `n_3m_rowcount = n_3m_fields = 0` (line 977)

```text
check 3m's two populations, counted separately because they answer different
questions: how many robot-runs had their mission_complete ROW COUNT checked
(every generation can be), and how many carried the generation-9 run_end
counters at all (only generation 9 can be). A campaign where the second is 0
while the first is large is a pre-generation-9 binary wearing a generation-9
manifest, and it must read as UNRESOLVED rather than as a pass.
```

### gate-3g-populations

**Check 3g populations** — attached to `n_3g_treated = n_3g_control = 0` (line 997)

```text
Check 3g's two populations, counted separately and reported separately. An
off-vs-mtare_hybrid campaign must produce BOTH: a zero in the first means no
cell was ever certified as treated, a zero in the second means no cell was
ever certified as untreated, and either alone would let the check read as a
pass over the arm it never looked at.
```

### gate-3g-p6-populations

**P6 generation populations** — attached to `n_3g_gen_p6 = n_3g_gen_pre6 = 0` (line 1008)

```text
The same populations one phase further out. P6 is a SECOND generation
boundary and needs its own witness rather than riding on P5's: a binary can
have the scheduler and not the predictor (every cell banked between P5 and
P6 is one), so `rendezvous_schedule_enable` present says nothing about
whether `mtare_hybrid` on this cell means the trail chase or the modelled
one — which is the same pooling 3h refuses at P5, one name further down.
```

### gate-arm-vs-policy-arm

**arm versus policy_arm** — attached to `arm, policy_arm, dir_claims, _leftover = parse_cell_name(c, TAG)` (line 1024)

```text
Cell identity. ONE parser, because a name can carry several suffixes and
their ORDER is not fixed: `ts1b_n3_mtare_hybrid_r40_ttl0_seed1` and
`at2_mtare_hybrid_ttl180_r40_seed2` are the same two design dimensions
written the other way round. See parse_cell_name() for what the previous
single-pass regex did to both of them (76 hard failures on a clean
campaign).

Two names, and the distinction is load-bearing. `arm` is the CELL's
identity: what to count, what GATE_ARMS must have declared, which column
of the design this is. `policy_arm` is what the BINARY can stamp: the
reconnect policy alone, every runtime and design suffix removed, because
run_campaign.sh strips them before setting RECONNECT_MODE and the node
therefore stamps the bare policy name. Anything compared against
run_start or against the m-tare vocabulary uses policy_arm; everything
else uses arm. Comparing the full directory identity against the node's
stamp would hard-fail every correctly-run cr2, at2 and ts1b cell; what
the suffixes themselves claim is not dropped, it is held against the
manifest by check 3i below.
```

### gate-control-by-policy

**Control-ness is a policy property** — attached to `is_control = arm in CONTROL_ARMS or policy_arm in CONTROL_ARMS` (line 1048)

```text
Control-ness is a property of the reconnect POLICY, not of a design
level: `mtare_off_r40_ttl0` is the control column of a two-factor design,
and a claim radius does not make it a treated cell. Testing membership on
the full identity alone would have read every suffixed control cell as
treated — check 3e would then demand reconnect_enabled=True of a correct
`off` run and hard-fail it, and check 3g would run the treated-arm stack
assertions over the control. Either spelling may be declared in
GATE_CONTROL_ARMS; the full identity is tried first so a design that
deliberately controls only ONE level of a factor can still say so.
```

### gate-3k-roster-witnesses

**Check 3k roster witnesses agree** — attached to `_m_robots = [t.strip() for t in (m.get("robots") or "").split(",")` (line 1078)

```text
3k. the two roster witnesses must agree. run_campaign.sh writes `robots=`
from the value it launched; the node writes team_robot_names from the
list it was configured with. They are independent, so a disagreement
means one of the two is describing a team that did not run, and the
per-robot checks below are iterating the wrong set either way.
```

### gate-3i-design-level-witness

**Check 3i design level witness** — attached to `_levels = []` (line 1101)

```text
Same argument as 3e one level down. The analysis reads r10-vs-r40 and
ttl0-vs-ttl180 off the directory, so a launcher bug or a leaked
COORD_CLAIM_R that runs 10 m inside an _r40_ directory silently swaps the
design's two columns and every downstream number is attributed to the
wrong level. The manifest is written at launch from the value actually
passed, so it is the witness.

Iterated over DESIGN_SUFFIXES rather than written out once for the claim
radius, so adding a dimension to that table adds its check with it. The
hand-written version covered the radius only, which is why ts1b's _ttl0
— a level that is the DEFAULT since 2026-09-05 and is therefore written
on almost every new cell — went unchecked.

An absent key is only forgiven when the directory claims nothing either:
a campaign predating the override wrote no such line and its unsuffixed
cells really did run the yaml default. A suffixed directory with no line
to check it against is unverifiable, and unverifiable is not a pass.
```

### gate-2-suspect-report-only

**SUSPECT from report-only gates** — attached to `verdict = m.get("run_gates_verdict", "MISSING")` (line 1160)

```text
NOT relaxed: anything other than CLEAN is a hard failure, because the
verdict is the harness's own statement that it could not certify the run.
The one exception is narrow and provable, and it exists because a check
that documents itself as report-only was failing whole campaigns.

run_explo_sim_rviz.sh derives SUSPECT from `grep -c "^UNRUN"` over
comms_gates.txt, with no notion of which gate wrote the line. map_agree is
a REPORT-ONLY reading — it has no pass threshold and its own header says
so — but until 2026-09-15 it emitted UNRUN whenever it could not compute
(which, being hardcoded to two planner CSVs, was EVERY N>=3 cell). One
informational line it was never entitled to write therefore turned into
SUSPECT, and SUSPECT turned into a hard failure here: every cell of every
N>=3 campaign, rejected for a gate that does not gate.

So a SUSPECT is re-read against the file it came from. If it holds UNRUN
lines and EVERY one of them names a report-only gate, the cell is
UNRESOLVED — not a pass, because the harness still declined to certify it
and an operator must look; not a hard failure, because nothing that can
invalidate a run is among the reasons. Any other shape stays a hard
failure, including a SUSPECT with no UNRUN lines at all: that is the
"watcher never reported" path, where the missing line is the outage gate
itself and absence of failures is emphatically not a pass.
```

### gate-3a-nav-split-provenance

**Nav binary split provenance** — attached to `if _nav_present and not m.get(NAV_BIN_KEY):` (line 1210)

```text
3a'. the nav binary, two ways the EXPECT loop above cannot cover.

First: split provenance. EXPECT only carries NAV_BIN_KEY when EVERY cell
has it, so a campaign where only some do would otherwise drop the check
for the cells that lack it -- silently, and on exactly the cells whose
provenance is in question. A harness that changed mid-campaign splits the
run into two generations ([[commit-mid-campaign-splits-provenance]]).
```

### gate-3a-nav-partial-install

**Nav binary partial install** — attached to `_nav_missing = sorted(k for k in (NAV_BIN_KEY,) + NAV_BIN_SIBLINGS` (line 1224)

```text
Second: a partial install. The four nav executables are built and
installed as a unit; the campaign-wide value is pinned by declaration,
but internal disagreement is a defect on its own terms and needs no
declaration to detect. "missing" means the executable was not there when
the manifest was written, which for a run that produced nav logs means
the install tree and the running node had already diverged.
```

### gate-3b-git-rev-in-params

**git_rev lives in run_start params** — attached to `start = [e for e in ev if e.get("event") == "run_start"]` (line 1249)

```text
git_rev is NOT a top-level key of run_start. The node writes it with
addParamStr (explo_planner_node.cpp:3493, the EXPLO_PLANNER_GIT_REV
branch; :3495 is the "unknown" fallback), so it lands in
run_start.params.git_rev. Reading it at the top level yielded "" on
every real cell, and `anything.startswith("")` is True, so BOTH this
check and the -dirty witness below passed unconditionally — verified
by planting params.git_rev="deadbee-dirty" against a manifest saying
322b6fc and getting "HARD FAILURES: none". A stale rev is exactly the
generation mix this gate exists to catch, so this was the check
failing at its one job while reporting a pass.

A missing run_start is a hard failure rather than a skip. It used to
be `if start:` with no else, which meant a robot log truncated at the
head got none of 3b, 18b or 18c — and that is the robot-run whose
provenance you most want checked, not least.
```

### gate-3l-resolved-config

**Check 3l reads resolved config** — attached to `for _k in CFG_KEYS:` (line 1272)

```text
3l. Collect the configuration this robot-run actually resolved.
Read from run_start.params and not from the yaml, because the
yaml is a request: -p overrides beat it, and the "0 means auto"
knobs are rewritten at construction (coord_claim_radius_m=0.0
resolves to fov_max_range, which this generation doubles). The
node logs these AFTER resolution, so this is the outcome.
```

### gate-3d-schema-stamp

**Check 3d schema stamp** — attached to `sv = start[0].get("schema_version", pr.get("schema_version"))` (line 1293)

```text
3d. the schema stamp. The identity in section 32.14 pinned schema
3, the binary writes it (experiment_log.cpp:287) and every reader
downstream keys off it, but nothing here read it — so the one
field that catches a generation mix at a glance was unchecked.
The pin now lives in SCHEMA_VERSION above, where an operator
scoring a banked campaign can move it from the command line
instead of from a diff nobody reviews.
```

### gate-reconnect-enabled-spelling

**Two spellings of reconnect_enabled** — attached to `have_rdv = pr.get("reconnect_enabled")` (line 1306)

```text
Either spelling: `reconnect_enabled` since the 2026-09-03 rename,
`rendezvous_enabled` in every cell up to and including cr5. The
node stamps both now; reading both keeps one gate over a mixed
set of campaigns.
```

### gate-3e-arm-stamp

**Check 3e arm stamp against policy** — attached to `want_rdv = not is_control` (line 1314)

```text
3e. the treatment variable must not come from the directory name
alone. Everything downstream keys the arm off the cell directory,
so a launcher bug or a leaked environment variable that runs the
off configuration in a _hybrid_ directory corrupts the assignment
silently and the gate would have said CLEAN. mode_req/rdv were
parsed and PRINTED as an informational line; printing is not
checking.

Compared against policy_arm, not arm: the node stamps the
reconnect policy, and per-cell runtime suffixes like _r<N> are
stripped by run_campaign.sh before it ever sees one. Comparing the
full directory identity here would hard-fail every correctly-run
cr2 cell. What the suffix itself claims is not dropped — check 3i
above holds it against the manifest.
```

### gate-3g-whole-stack

**Check 3g whole feature stack** — attached to `_mt_want = arm.startswith("mtare_")` (line 1354)

```text
The node reconstitutes the arm itself and prefixes `mtare_` when
`global_alloc_enable_ || reconnect_gate_info_ ||
rendezvous_schedule_enable_` — an OR. So the name `mtare_hybrid`
proves at least ONE of P3, P4 and P5 was live and never all
three, and check 3e, which compares that name against the stamp
it came from, is satisfied by a cell running a third of the
treatment. The arm's definition is a feature VECTOR; nothing
above asks about the vector.

Two of them are worse than ambiguous, they are invisible.
cell_world_enable and team_world_hz rename NOTHING, so a cell
that ran no census or no exchange still carries an mtare_hybrid
directory name, an mtare_hybrid stamp, and passes 3e — and the
launcher guard that would have caught it cannot see an ambient
export either. These params are what the node was actually
handed, and they are the only record that says so.

Checked in BOTH directions, and the control direction is the one
that decides the comparison. An `off` cell that somehow ran the
allocator is a treated cell sitting in the control column, and no
amount of care in the treated arm compensates for that.
```

### gate-3g-p5-dating

**P5 param dates the cell** — attached to `_p5_raw = pr.get("rendezvous_schedule_enable")` (line 1380)

```text
P5 added a FIFTH feature, and its param's absence does not mean
what the other four's does. Those four have been emitted
unconditionally since P4, so a cell missing one predates the
whole stack and cannot be certified at all. rendezvous_schedule_
enable instead DATES the cell: absent means a pre-P5 binary,
which is a perfectly legitimate thing to score — campaign mh1 is
one — but a different generation, in which the token
`mtare_hybrid` does not name the same arm it names after P5
(there was no appointment to switch on, so hybrid's fallback
destination was the midpoint).

Handled by GENERATION rather than by exemption, because the two
loosenings on offer are both the failure this file exists to
prevent: dropping the fifth key from the vector would certify a
P7 mtare_hybrid cell that never armed an appointment, and
hard-failing its absence would make the gate unable to score any
campaign banked before today. So each cell is dated, a pre-P5
cell is held to the pre-P5 arm vocabulary, and a campaign that
mixes the two generations is rejected outright below (3h) — that
is pooling across binary generations, which is already forbidden.
```

### gate-3g-p6-dating

**P6 param dates the cell** — attached to `_p6_raw = pr.get("pursuit_predictor")` (line 1408)

```text
P6's discriminator, read the same way and for the same reason.
`pursuit_predictor` is emitted unconditionally by any binary that
has one, so its absence dates the cell rather than defaulting it
— and defaulting it to `trail` is exactly the wrong reflex here,
because that is the value a P6 `_mdp` cell must NOT have.
```

### gate-3g-lookup-arm

**Stack lookup by _lookup_arm** — attached to `_want = mtare_expect(_lookup_arm, _gen_p5, _gen_p6)` (line 1479)

```text
policy_arm again: MTARE_ARM_STACK is keyed by the reconnect
policy vocabulary, which has no room for a per-cell knob
suffix. A radius does not change which m-tare features the
arm promises, so mtare_hybrid_r10 must be looked up as
mtare_hybrid or the whole stack assertion is skipped in
favour of a spurious "not an arm this binary can stamp".

_lookup_arm, not policy_arm, so a cell carrying a suffix
this file has not learnt yet is still held to its stack:
3e has already reported the unknown token as UNRESOLVED,
and falling back to the binary's own stamp checks the
five features against the arm the binary says it ran
rather than skipping the assertion entirely.
```

### gate-3f-treatment-possible

**Check 3f treatment could fire** — attached to `if not is_control:` (line 1521)

```text
3f. THE TREATMENT MUST HAVE BEEN ABLE TO HAPPEN. This is the
check g8r1 needed and did not have. That campaign was launched,
ran to completion, and passed every gate here — while the mid-run
clock sat at 240 s against an outage distribution whose p90 is
52-111 s and only 0.00-3.34 % of whose outages reach 240 at all,
so the trigger expired in 3 of 23 hybrid cells and 87 %
of the treated arm was behaviourally the control. Nothing was
broken; the treatment was configured out of existence, and no
check asked whether it could fire at all.

Read from run_start params, not the manifest. The manifest
records what the launcher INTENDED; these are what the node was
actually handed, and generation 9's whole provenance argument is
that only the second one explains a run.

Applied to every arm EXCEPT `off`, not to "hybrid" by name: `off`
disables the manoeuvre on purpose and a check that fires on the
control every time is one nobody reads, but any other arm —
pursuit, rendezvous, a future one — reaches the same trigger and
has the same way of being configured out of existence. Naming the
one arm would have exempted the rest by accident.
```

### gate-3f-link-gate-live

**Link gate configured versus live** — attached to `if not GEN9_PARAMS_REQUIRED:` (line 1575)

```text
CONFIGURED vs LIVE — two questions, and only asking the first
is how g8r1 passed. link_gate_configured is computed at
startRun from two topic NAMES being non-empty, so it is true
whenever the launcher typed them; it cannot know whether a
sample ever arrived, because nothing has been delivered yet
when it is written. A dead emulator therefore satisfies it
while the veto is absent for the whole run — the exact
configuration this generation exists to prevent.

The runtime half is the one-shot "link_gate_live:" line the
node emits from its link-states subscription on the first
usable sample. (Deliberately not from linkGateReady(), which
only runs when something consults the gate: a run whose team
never went silent would emit nothing and be scored a failure.)
PRESENCE is asserted, not absence of the not-usable warning:
a run whose logging broke would pass an absence test
([[nav-global-planner-never-planned]]).
```

### gate-18c-done-action-idle

**Check 18c done_action idle** — attached to `if pr.get("done_action") != "idle":` (line 1644)

```text
18c. done_action must be "idle". The node's own default is
"shutdown", so a shared_params.yaml that failed to install leaves
every other provenance field in the manifest matching while the
planner exits at DONE — which means no homing leg and therefore
no mission_end, i.e. the primary endpoint is silently absent for
the whole campaign.

Read from the run_start params, NOT from the manifest's
done_action_in_params: the manifest reports what the YAML on disk
says, and this reports what the node actually loaded. They differ
in exactly the case worth catching (the node fell back to its own
default), which is the case where the manifest looks fine.
```

### gate-3m-reentries-not-failed

**Mission return re-entries are health** — attached to `if int(_ree) > 0:` (line 1741)

```text
mission_return_reentries is NOT failed on. The late coverage
latch legitimately asks to home a second time and the latch
refusing it is the fix working, so a non-zero value here is
evidence of health, not of a defect. It is collected for the
population report instead, where a campaign-wide zero can be seen
and questioned.
```

### gate-3n-planner-log-witness

**Check 3n reads the planner log** — attached to `_plog_path = os.path.join(d, f"planner_{r}.log")` (line 1753)

```text
Why this check lives in the planner log rather than beside 3m's two
run_end counters. `dup_run_ends_` can only become non-zero AFTER the
first run_end row is already on disk, so a `dup_run_ends` field
written beside mission_completes_suppressed would read 0 on every run
ever, including the runs it exists to catch — a field observed only
at zero because it is structurally unable to be anything else. And
appending a trailing `run_end_duplicate` event is worse than useless:
it breaks "run_end is the last line" and the
last-line-seq == events_written-1 invariant, and event_log.py's
`complete = (written == len(evs))` would then mark the run TRUNCATED
and drop the cell — destroying the evidence in the act of recording
it.

So the witness is the planner's own stderr and this is its reader.
TWO markers, written by different code at different times: the WARN
fires during the run on the first suppressed attempt
(experiment_log.cpp:692), the destructor ERROR fires at close with
the total (experiment_log.cpp:73). Either alone is the finding, and
both are matched because the destructor runs during rclcpp teardown
and its output is the less certain of the two to reach the file.

THE BASE RATE, so a clean reading is not over-read. The suppression
is new in generation 9; under generation 8 a second terminal state
would have written a second run_end ROW, and across the whole bank —
3342 robot-runs, every campaign — there are ZERO files with more than
one. So this failure mode has never once been observed, and a future
"0 duplicates" is consistent with a working guard AND with a guard
that could never have fired. What the check actually buys is that the
generation-9 suppression, which makes the event invisible in the
jsonl for the first time, cannot make it invisible everywhere.
```

### gate-19-field-writers

**Check 19 fields per writer** — attached to `peers = [e for e in ev if e.get("event") in ("peer_lost", "peer_seen")]` (line 1878)

```text
Assert each field against the writer that actually emits it. An
earlier version of this check demanded team_incomplete_sec on every
peer_lost/peer_seen row, which no binary has ever written: PeerEvent
has no such member and the only writer is logReconnectDispatch
(experiment_log.cpp:463, inside the reconnect_dispatch block at
:431-478). Run against a real cell it produced 34 hard
failures on clean data, and it survived calibration only because the
fixture manufactured the field the binary does not write — the
fixture asserting the gate's belief instead of the binary's output.
That is why the calibration is now seeded from a real cell.

What generation 8 actually did to the peer rows was REMOVE
last_contact_age_sec. That half is the real migration witness.
```

### gate-9-console-log-absent

**Missing console log is unresolved** — attached to `con = os.path.join(ROOT, c + ".console.log")` (line 1968)

```text
The absent-file case is UNRESOLVED, not a skip. This check used to sit
under a bare `if os.path.exists(con):` with no else, so deleting or
failing to write the console log turned the whole process-death check off
and printed nothing at all — and a cell whose console log never got
written is precisely the cell most likely to have had a process die.
```

### gate-table-cell-width

**Cell column width from the data** — attached to `_cw = max([len("cell")] + [len(r["cell"]) for r in rows])` (line 1994)

```text
Width from the data, not a literal. A fixed 34 was two characters short of
`ts1b_n3_mtare_hybrid_r40_ttl0_seed1` — every row of every two-factor campaign
overflowed its column and pushed the rest of the line out of alignment, which
is how a table stops being read.
```

## 3h. ONE BINARY GENERATION PER CAMPAIGN.

### gate-3h-one-generation

**Check 3h one binary generation** — attached to `if n_3g_gen_p5 and n_3g_gen_pre:` (line 2020)

```text
Check 3b already pins one git_rev across the campaign, which catches this
whenever the revision is recorded and readable. This is the same claim read
off the BEHAVIOUR instead of off the provenance string, and it is worth
having twice because the two fail differently: 3b compares a label, 3h
compares what the binary actually emitted.

The P5 param is the discriminator. A binary that has the scheduled
rendezvous emits `rendezvous_schedule_enable` in every run_start, on or off;
one that predates it emits nothing. So a campaign holding both kinds of cell
pooled two generations under one set of arm names — and the arm names are
exactly what does not survive that: `mtare_hybrid` means "chase, falling back
to the midpoint" on one side of P5 and "chase, falling back to the agreed
cell" on the other. Analysed together they are one arm run twice with two
different treatments in it.

P6 IS A SECOND SUCH BOUNDARY AND IS CHECKED SEPARATELY, not folded into the
first. The two are not the same question: every cell banked between the two
phases emits rendezvous_schedule_enable and no pursuit_predictor, so it is
uniform by the P5 test while `mtare_hybrid` on it means the trail chase and
`mtare_hybrid` on a P6 cell means the modelled one. Pooling those is the
identical failure one name further down, and the P5 test cannot see it.
```

## 3l. ONE SENSOR AND CANDIDATE CONFIGURATION PER CAMPAIGN.

### gate-3l-one-sensor-config

**Check 3l one sensor configuration** — attached to `_cfg_split_hard, _cfg_split_soft, _cfg_uniform = [], [], []` (line 2072)

```text
Same shape as 3h and 3j, one layer further down: 3h refuses to pool two
binaries, 3j two radios, this refuses to pool two sensor models. The
generation-9 lidar FOV change (full azimuth, 96 rays, 20 m) is a params-file
change, and a params-file change moves NO field this gate previously read --
git_explo_planner does not move for it, and sha256_shared_params moves but is
compared to nothing. A stale installed yaml therefore produced a cell that ran
the generation-8 sensor under a manifest that said generation 9 everywhere.

And it prints the configuration whether or not it varies, because the reason
this check exists is that "D1 was in effect" was an assumption with no witness
in the run record. An operator reading a green gate should be able to see the
FOV the campaign actually ran, not infer it from the yaml in the source tree.
```

### gate-3l-params-file-hash

**Params file hash per arm** — attached to `_ph = collections.defaultdict(set)` (line 2125)

```text
The params file is the other half of what the node ran, and until now the
gate never looked at it. Within an arm it is a constant; between arms it can
legitimately differ, because varying a yaml-only knob per arm inside one
invocation is done by giving the arm its own params file
([[per-cell-knobs-ride-the-arm-suffix]]).
```

## _regime_str

### gate-3j-one-radio-regime

**Check 3j one radio regime** — attached to `def _regime_str(t):` (line 2148)

```text
Same shape as 3h one layer down: 3h refuses to pool two binaries, this
refuses to pool two radios. Every connectivity result — deep_outage_sec, the
dropout count, the %-time-connected mediator, and through it the completion
time — is conditional on how much a trunk costs and how far a clear lane
reaches. Two regimes under one tag is not a noisier campaign, it is two
experiments whose arm means something different in each.

Read off the MANIFEST, which is written at launch from the values actually
passed to the emulator. The alternative — inferring the regime from observed
dropouts — cannot work: dropout rate is exactly the thing the regime is
supposed to explain, so inferring one from the other would make the check
agree with itself.
```

## 3j. ONE RADIO REGIME PER CAMPAIGN, AND IT MUST BE THE DECLARED ONE.

### gate-3g-population-exemption

**Check 3g population exemptions** — attached to `if not n and check == "3g":` (line 2240)

```text
3g's two populations are exempt only when the PRE-REGISTRATION says the
arm does not exist. A hybrid-vs-off campaign has no m-tare arm and must
not be marked unresolved for the absence of one; an all-m-tare campaign
has no untreated arm and likewise. What is never exempt is a campaign
that declared both and produced only one — that is the check failing to
look at half the design, and it must stay UNRESOLVED.
```

### gate-3m-mixture-half

**Check 3m mixture half** — attached to `if n_3m_fields and n_3m_fields < n_3m_rowcount:` (line 2260)

```text
check 3m, the mixture half. NOT relaxable by GATE_ENDPOINT_FIELDS, because
the flag says "this campaign predates the fields" and a campaign where SOME
robot-runs carry them and some do not is not that campaign — it is two binary
generations inside one directory, which is never a design. Found by the
negative control for the relaxation itself: with the flag set and the fields
stripped from one arm only, the gate reported a non-zero witness count and
read as though everything had been checked.
```

### gate-3m-health-half

**Check 3m health half** — attached to `if n_3m_fields:` (line 2275)

```text
check 3m, the health half. Printed unconditionally — including the zero case,
in words — because a guard that reports only when it fires is
indistinguishable from a guard that was compiled out. The fix is not FAILED
on a zero: a campaign of short runs may genuinely never latch coverage after
homing. It is reported so the zero has to be looked at rather than inferred.
```

### gate-21-campaign-shape

**Check 21 campaign shape** — attached to `if _attempts:` (line 2320)

```text
21. the campaign is the shape it was pre-registered as. Nothing above counts
cells, so a campaign that died after three of them scored CLEAN and invited
analysis of a truncated, arm-unbalanced dataset. Seed-major ordering makes an
early abort systematically unbalanced, so "small" here also means "biased".
Both lists are printed because both are overridable from the environment, and
an override is invisible in the output it changes: a reader seeing "campaign
shape: {...} CLEAN" cannot tell whether check 21 compared against the
pre-registered pair or against whatever GATE_ARMS happened to be exported in
that shell.
```

### gate-21-declaration-one-factor-short

**GATE_ARMS one factor short** — attached to `if (set(cells_by_arm) != set(EXPECT_ARMS)` (line 2350)

```text
One specific disagreement deserves its own sentence, because the generic
messages below describe it as a launcher bug when it is a declaration that is
one factor short. If the declared list matches the campaign's reconnect
POLICIES exactly, and the only difference is that every directory carries a
design suffix on top of them, then no cell is mislabelled: GATE_ARMS was
written for a one-factor design and the campaign ran a two-factor one. It
stays a hard failure — the count check is exactly what notices that only one
level of the new factor ran, and it cannot notice that at the policy level.
```

### gate-exit-codes

**Three states, three exit codes** — attached to `if hard_fail:` (line 2397)

```text
Three states, three exit codes. UNRESOLVED is the gate's designed answer for
an empty population — the property the module docstring pins — and it was
invisible to $?, so any wrapper branching on the exit code read "nothing was
tested" as "everything passed". That is the failure this gate was written to
prevent, reproduced in the gate's own interface.

3 rather than 1 because unresolved is not a failure: it is a claim that part
of the gate had no data to run on, which the operator must read and judge. A
campaign with the off arm in it will legitimately have empty reconnect
populations, so 3 is expected and is not an error — it means "read the
UNRESOLVED list before believing this".
```
