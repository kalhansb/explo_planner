# campaign_guard_calib.sh — design notes and history

The long comments of `sim/campaign_guard_calib.sh`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Top level](#top-level) — 2
- [t()](#t) — 1
- [Top level (part 2)](#top-level-part-2) — 2
- [census()](#census) — 1
- [Top level (part 3)](#top-level-part-3) — 2
- [lg()](#lg) — 1
- [and survived the mutation that was meant to kill it.](#and-survived-the-mutation-that-was-meant-to-kill-it) — 6
- [the roster axis (P7)](#the-roster-axis-p7) — 2
- [nplan_guard()](#nplan_guard) — 1
- [the roster axis (P7) (part 2)](#the-roster-axis-p7-part-2) — 1
- [and that is the problem: an operator who learns the launcher always prints](#and-that-is-the-problem-an-operator-who-learns-the-launcher-always-prints) — 1
- [that IS the set of things the guard models -- and assert the invariant](#that-is-the-set-of-things-the-guard-models----and-assert-the-invariant) — 5
- [rg_manifest()](#rg_manifest) — 1
- [rg()](#rg) — 2
- [that IS the set of things the guard models -- and assert the invariant (part 2)](#that-is-the-set-of-things-the-guard-models----and-assert-the-invariant-part-2) — 5
- [the verdict itself, which the resume guard used to read one bit of](#the-verdict-itself-which-the-resume-guard-used-to-read-one-bit-of) — 4

## Top level

### guardcal-link-veto-scope

**What this file's guards cover** — attached to `set -u` (line 4)

```text
The link veto below is the one this file was written for and still the
longest section. Two others have joined it: the launcher's own validation
blocks, and the RESUME guard, which decides whether a directory already
holding a finished cell is "already complete" or a different experiment
wearing the same name (last section).

The guard refuses a campaign that would arm the mid-run reconnect trigger at
the generation-9 clock (90 s, below the ~180 s heartbeat-suppression tail)
with no link veto to tell a quiet teammate from an absent one. Its first draft
was wrong in four ways at once, none of which a reading would have caught:
it disarmed on the mere PRESENCE of the string "MIDRUN_SILENCE=" (so
GATE_MIDRUN_SILENCE=240 turned it off, and MIDRUN_SILENCE=5 passed), it armed
on "LINK_GATE=0" as a substring (so MY_LINK_GATE=0 tripped it), it matched
only the literal value 0 while the launcher treats every non-1 value as off,
it never saw COMMS= arriving through --env where it beats the flag, and it
blocked control-only campaigns that have no treatment to protect.
```

### guardcal-nothing-launches

**The harness must launch nothing** — attached to `set -u` (line 21)

```text
Every case runs --dry-run, so nothing launches. Asserting the ALLOW cases by
letting them start and killing the driver orphaned six gazebo/scovox/
robot_state_publisher processes the first time it was tried; the last section
checks the process census to make sure that cannot come back.
```

## t()

### guardcal-allow-needs-exit-zero

**ALLOW means exit 0** — attached to `t() {` (line 45)

```text
ALLOW checks the EXIT CODE and not merely the absence of the guard's FATAL
text. An earlier version asserted absence alone and discarded the status, so
any unrelated refusal scored as ALLOW -- `--bogus` exits 2 on an unknown
argument, never reaches the guard, and was counted a pass. That is a check
that agrees with itself: the campaign it certifies as permitted is one that
cannot run at all. The planted case at the end of the arm section pins it.
```

## Top level (part 2)

### guardcal-ambient-link-gate-strip

**Strip ambient LINK_GATE from each cell** — attached to `cases=$((cases+1))` (line 126)

```text
The hole every case above was blind to, because every case passes its setting
through --env, which is the one channel the guard reads. `env` without -i
inherits the caller's environment and the launch line did not assign LINK_GATE
or MIDRUN_SILENCE, so `export LINK_GATE=0` before a normal launch ran the whole
treated matrix on the 90 s clock with no veto and no complaint.

Asserted on the LAUNCH LINE rather than on the guard's verdict, because the
guard would (correctly) stay silent either way -- the bug was never in what it
concluded, it was in what reached the cells afterwards.
```

### guardcal-link-gate-default-readback

**Reading back the launcher LINK_GATE default** — attached to `LAUNCHER="$(dirname "$CS")/run_explo_sim_rviz.sh"` (line 169)

```text
run_campaign.sh decides whether the veto will be live BEFORE anything is
launched, so it has to know what LINK_GATE defaults to inside
run_explo_sim_rviz.sh, and it hard-codes 1. Nothing else links the two files:
flip the launcher default to 0 and the guard would happily pass a campaign
that runs the 90 s clock bare, which is the entire hazard it exists to stop.
Read the literal back out and compare.
```

## census()

### guardcal-process-census-delta

**The process census is a delta** — attached to `census() {` (line 212)

```text
The first version of this file had no --dry-run and asserted the ALLOW cases
by letting them start and killing the driver after 20 s. That orphaned six
gazebo/scovox/robot_state_publisher processes, which is a worse bug than any
it was testing for. Assert the absence directly rather than assuming it.
A DELTA, not an absolute. The first version took one census after the cases and
demanded zero, which is wrong in both directions: a stray sim already on the box
is reported as "this harness started a simulation", and on a busy box the check
fails for someone else's work. What this harness is responsible for is the
CHANGE it caused.

It also had no known-answer case -- every case runs --dry-run, so nothing could
ever start a process and the check could not fail however broken it was. That is
the shape §32.14 is about, in the harness whose whole purpose is to prevent it.
The planted case below starts a process the census pattern matches and requires
the census to see it, so a census that has stopped counting says so.
```

## Top level (part 3)

### guardcal-prelude-cut

**Running the launcher prelude on its own** — attached to `PRELUDE_ANCHOR='^unset _rzv_needed$'` (line 272)

```text
run_explo_sim_rviz.sh has no --dry-run: invoking it launches gazebo for real.
Probing it that way once burned two minutes and left processes on the box --
the same failure the census section above exists to stop. Every guard tested
here lives in the launcher's PRELUDE: the defaults, the validation and the
refusals, ending at the arm-stamp check. That region runs no ROS, spawns
nothing and writes nothing, so it is cut out of the SHIPPED file by line range
and run on its own. It is the real text, not a transcription of it: edit a
guard and these cases move with it.

THE CUT MUST BE WRITTEN INTO sim/, not a tmpdir. The launcher computes HERE
from BASH_SOURCE[0] and the workspace root three levels above it, so a copy
anywhere else dies at the scenario-installed check for a reason that has
nothing to do with the guard under test. Every BLOCK case would still see a
non-zero exit, and this whole section would pass while testing nothing.
Moved past the arm-stamp guard 2026-09-16, to the end of the _rzv_needed
block that now follows it. Cutting at the arm-stamp unset would have left the
`a mode that is nothing without the schedule` refusal outside the probe
entirely -- so the bare-default ALLOW case below would have certified a
default that the real launcher refuses one line later, which is the exact
shape of a check that has stopped checking.
```

### guardcal-lg-env-i-and-scen

**Probe environment and scenario choice** — attached to `SCEN=flatforest_dense_2robot_lidar.yaml` (line 332)

```text
env -i: the point of several of these guards is that an AMBIENT export must
not reach a cell, so the probe must not inherit one either.
$SCEN, not a literal: every case here used to pin the 2-robot scenario, so
the whole section only ever saw N == 2 and nothing that varies with the
roster was under test in either direction. Callers that care set SCEN.
```

## lg()

### guardcal-sentinel-whole-line

**Sentinels match whole lines** — attached to `local ok=0 _mx=-qF` (line 347)

```text
A sentinel is matched as a WHOLE LINE, a FATAL as a substring. The
distinction is not cosmetic: the sentinel's last field is the arm name, so
a substring match for `rm=mtare_hybrid` is also satisfied by an output
reading `rm=mtare_hybrid_mdp`. The palette case in the derivations section
below was written with a substring match and could not fail for exactly
this reason -- COSTAR_..._REDUCED is a prefix of COSTAR_..._REDUCED_YELLOW
-- and survived the mutation that was meant to kill it.
```

## and survived the mutation that was meant to kill it.

### guardcal-default-is-mtare-hybrid

**Default and token assert one vector** — attached to `lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=1 pp=trail rm=mtare_hybrid' \` (line 375)

```text
The first used to assert `every M-TARE knob off`, because RECONNECT_MODE
defaulted to plain `hybrid`. That default was UNRUNNABLE from the day the
schedule-needed refusal landed -- plain hybrid with rs=0 is refused there,
and with rs=1 it is refused by the arm-stamp guard -- so the default moved
to mtare_hybrid and this case moved with it. The two now assert the same
vector by two routes, which is the point: the default and the token that
names it must not drift apart.
```

### guardcal-factorial-tokens

**The other three factorial tokens** — attached to `lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=silence rs=0 pp=trail rm=mtare_off' \` (line 407)

```text
The other three cells of the doc §3.6.1 factorial. Each expands to a
DIFFERENT vector, and the two that differ from mtare_hybrid are the point
of the design: mtare_pursuit is chase without an appointment,
mtare_rendezvous is an appointment without a chase, mtare_off is neither.
Asserted one knob at a time because a token that expanded to the hybrid
stack under a different name would run the campaign as one arm four times
and every gate downstream would agree with it.
```

### guardcal-01-knob-typos

**Typos in 0/1 knobs are refused** — attached to `lg BLOCK "FATAL: CELL_WORLD='true'" \` (line 424)

```text
A typo in a 0/1 knob reads as OFF, and an off treatment knob is invisible:
the run completes, the manifest records the value it was handed, and the
cell is analysed as treated. Only the launcher can catch this.

RECONNECT_MODE=pursuit on both, so the arm stack is EMPTY and the 0/1
validity check is what the case actually reaches. Under an mtare_* token
(which the default now is) the stack's own contradiction check fires first
and reports the same input as a different mistake -- still a refusal, but
not the one this case is calibrating.
```

### guardcal-arm-stamp-per-knob

**One arm-stamp case per knob** — attached to `lg BLOCK "FATAL: the arm name and the arm" \` (line 459)

```text
The node reconstitutes the arm itself, prefixing `mtare_` when
global_alloc_enable_ || reconnect_gate_info_ || rendezvous_schedule_enable_.
Every directory, index row and analysis keys off the NAME; only run_start
carries the stamp. Setting a knob by hand under the plain name puts a
treated cell in the control column.

One case per disjunct, and that is the whole reason this block is a list
rather than one case: a guard that tracked only two of the three would keep
printing two PASSes while the third knob walked straight through it.
```

### guardcal-pursuit-predictor-cases

**PURSUIT_PREDICTOR typo and stamp cases** — attached to `lg BLOCK "FATAL: PURSUIT_PREDICTOR='MDP'" \` (line 497)

```text
The P6 knob. It is the SIXTH thing that flips the node's `mtare_` prefix, so
it needs the same three-way coverage the fifth got: the typo, the arm-stamp
direction, and the preconditions.
RECONNECT_MODE=pursuit for the same reason as the two 0/1 typo cases above:
every mtare_* token pins PURSUIT_PREDICTOR, so under the default the arm
stack refuses 'MDP' as a contradiction before the spelling check sees it.
```

### guardcal-mdp-tokens

**The two _mdp arm tokens** — attached to `lg ALLOW '__PRELUDE_OK__ cw=1 tw=1 hz=1.0 ga=1 rg=info rs=0 pp=mdp rm=mtare_pursuit_mdp' \` (line 532)

```text
The two _mdp tokens are the only way to REQUEST the predictor, and they are
ordinary arm tokens: the whole stack comes from the name, and the name is
what the node will stamp back (its arm rule gained the matching `_mdp`
suffix in the same commit). These four assert both halves -- the stack the
token expands to, and the refusal of an override that would make the
directory name and the run_start stamp disagree.
```

## the roster axis (P7)

### guardcal-roster-axis

**The roster axis and its control** — attached to `lg ALLOW '__ROSTER_OK__ n=2 robots=[atlas bestla]' \` (line 555)

```text
Everything above this line ran at N == 2 and could not have told a harness
that generalised from one that merely still works for a pair. That is not a
hypothetical gap: the launch-time planner-count guard kept comparing against
a literal 2 through a whole "78/78 ALL PASS" run, because no case ever asked
for a third robot.

The roster is read from the SCENARIO and is deliberately not overridable, so
the only way to vary it is to point at a different scenario file. The N == 2
case below is the control: without it, an assertion that matched a constant
string would pass at both sizes and prove nothing.
```

### guardcal-derivation-cut

**Probing the per-robot derivations** — attached to `DERIV_START_ANCHOR='^VIZ_PALETTE=(COSTAR_'` (line 584)

```text
The prelude section above stops at the arm-stamp guard, which is where the
launcher stops being inert. Everything the P7 change actually rewired --
peers_of / peers_csv / peers_ros_array, and the roster-position viz palette --
lives below that line and had no coverage at all, in either direction.

They are still inert: pure shell over $ROBOTS, no ROS, no processes. So they
get their own cut, assembled from the prelude (which is what resolves $ROBOTS
from the scenario) plus this block. The two are not adjacent, and the region
skipped between them sets exactly one variable this block reads -- $OUTDIR --
which the shim supplies. If that ever stops being true the probe fails loudly
under `set -u` rather than testing a stub.
```

## nplan_guard()

### guardcal-planner-count-textual

**Textual check of the planner count** — attached to `nplan_guard() {` (line 685)

```text
This one guard cannot be executed without launching gazebo -- it counts real
processes 8 s after a real bring-up -- so it is asserted textually. That is a
weaker check than running it, and it is here because the ALTERNATIVE was no
check: it compared against a literal 2 while everything around it was
generalised, so a 3-robot scenario started the entire stack and then died on
"expected exactly 2 explo_planner_node, found 3".

A textual assertion is worth nothing unless it can fail, so it is run three
times: on the shipped launcher, on a copy mutated back to the literal, and on
a copy with the guard deleted. The last one matters most -- a check that
reports PASS when its subject is absent has stopped checking.
```

## the roster axis (P7) (part 2)

### guardcal-silent-stderr

**A clean launch is silent on stderr** — attached to `cases=$((cases+1))` (line 722)

```text
A clean launch must be SILENT on stderr. `env_val` took its default from a
bare "$2" while two callers legitimately omit it, so under `set -u` every
campaign launch opened with two `line 190: $2: unbound variable` lines. They
were harmless -- the value returned was the empty string the callers wanted
-- and that is the problem: an operator who learns the launcher always prints
two errors has learned to skim past the third one that means something. This
asserts the absence, not a message, because the next such regression will
have a different line number and a different variable.
```

## and that is the problem: an operator who learns the launcher always prints

### guardcal-launch-line-strips-knobs

**Every modelled knob is stripped or assigned** — attached to `_launch_line=$(sed -n '/^  env /,/run_explo_sim_rviz.sh"/p' "$CS")` (line 743)

```text
The ambient-export strip on the launch line. This is the one leak the
arm-stamp guard above CANNOT see: CELL_WORLD and TEAM_WORLD rename nothing, so
`export CELL_WORLD=1 TEAM_WORLD=1` in the launching shell would run the census
and the exchange in every cell of a plain hybrid-vs-off campaign, stamp the
same arm on both sides, and score CLEAN. Read the -u list back out rather than
trusting that it was kept in step with the knobs the launcher grew.

DERIVED, not re-listed. This loop used to enumerate nine names by hand, and a
hand-maintained copy of a list is a check that goes stale silently: by the
time anything looked, six knobs the guard reasons about (TREE_ATTEN,
MAX_RANGE, CELL_SIZE_M and the three SEPARATION_*) had been added to
run_campaign.sh and to none of them, and this section printed nine PASSes
without noticing. So take the knob list from the script's own env_val() calls
-- that IS the set of things the guard models -- and assert the invariant
stated at the -u list: each one is either stripped there, or assigned
explicitly on the same launch line. A knob that is neither is one the guard
scores at its default while the cell runs on the ambient shell's value.
Anchored on `env ` and not on `env -u LINK_GATE`, because the launch line
grew a derived sweep in front of the literal strips on 2026-09-18 and the
tighter anchor then matched nothing: sed returned an empty range, every knob
below was "neither stripped nor assigned", and fifteen guards that were in
fact intact reported FAIL. An extraction that can silently select nothing is
the same defect as a gate that can silently examine nothing, so the emptiness
of BOTH halves is now an explicit case rather than a fifteen-way symptom.
```

## that IS the set of things the guard models -- and assert the invariant

### guardcal-runner-knob-sweep

**Sweeping all runner knobs** — attached to `_keep=$(sed -n 's/^RUNNER_ENV_KEEP="\(.*\)"$/\1/p' "$CS" | head -1)` (line 796)

```text
The section above asserts an invariant over the knobs run_campaign.sh MODELS.
That is roughly twenty names. run_explo_sim_rviz.sh reads about a hundred and
ten, all as `${NAME:-default}`, and until 2026-09-18 the other ninety reached
every cell straight from the launching shell: `env` without -i inherits, the
strip list had never heard of them, the resume guard does not compare them,
and for most of them run_manifest.txt does not record them either. So an
`export RDV_DEPART_DELAY=30` left in a terminal retuned every cell of a
multi-day matrix with nothing anywhere able to say it had.

RDV_DEPART_DELAY is not a hypothetical: through generation 24 it was the
100 s in "after 100s after one robot disconnect, all robots go to the
rendezvous point" — the treatment itself in two of ts4's four arms. Since
generation 25 it is inert in the binary but still logged, so a leak now
forges the param rows rather than the behaviour.

The sweep is derived from the runner rather than listed, so these cases check
the DERIVATION -- reproduced here from the same source, which is the only way
to notice the scan silently matching nothing.
```

### guardcal-scanner-extracted

**The scanner is extracted, not copied** — attached to `_awkprog=$(sed -n '/^RUNNER_STRIP=\$(awk -v keep=/,/^'"'"' "\$HERE\/run_explo_sim_rviz.sh")$/p' "$CS" \` (line 815)

```text
THE SCANNER IS EXTRACTED, NOT REPRODUCED, and that distinction was found the
hard way. The first draft of this section pasted a copy of the awk program
here; two mutations of the launcher's real scanner -- dropping the bare
`${X-default}` form, and letting commented-out knobs through -- then passed
every case below, because the thing under test was a second copy that had not
been mutated. A calibrator holding its own copy of the code it certifies is
the same defect as the hand-maintained knob list this whole section replaced,
and it fails the same way: silently, while printing PASS.
```

### guardcal-tripwire-threshold

**Reading back the tripwire threshold** — attached to `cases=$((cases+1))` (line 926)

```text
And the tripwire's own threshold, read back rather than assumed. The case
above establishes that a broken scan reaches zero; this one establishes that
zero is refused. `-lt 0` can never fire and `-lt 200` would refuse every real
campaign, and both edits look equally innocuous in a diff -- which is how a
threshold is the quietest place for a guard to die.
```

### guardcal-sweep-output-and-wiring

**The sweep's output form and wiring** — attached to `cases=$((cases+1))` (line 944)

```text
The sweep must be WIRED, not merely computed. A derivation that is never
referenced on the launch line is the same nothing as no derivation, and it
reads as a fix in every diff.
The OUTPUT FORM, which every other case here is deliberately blind to: _scan
splits the `-u` flags off before comparing names, so a scanner that emitted
bare names would satisfy all of them. It would also turn the launch line into
`env RDV_APPT_WAIT OUTDIR=... runner`, where env runs the first name as the
command. That fails loudly at the first cell rather than silently -- but it is
the launcher's own `<10` tripwire that makes it loud, by counting exactly the
`-u` tokens asserted here, so this is the case that keeps that tripwire honest.
```

### guardcal-resume-guard-cases

**Known-answer cases for the resume guard** — attached to `RG_ARM=mtare_hybrid` (line 993)

```text
The resume guard decides whether a directory that already holds a finished
cell counts as "already complete" or as a different experiment wearing the
same name. It had no known-answer case of any kind until this section, while
growing from one compared key to seventeen -- and the failure it exists to
stop is silent by construction: the campaign prints SKIP, the matrix fills
up, and two configurations end up pooled under one tag with nothing in the
analysis able to tell.

Both directions are asserted, because each has its own way of going wrong. A
guard that never aborts is a rubber stamp; a guard that always aborts is
worse than none, since the operator learns to reach for a fresh --tag every
time and the guard stops being read. The float keys make the second failure
easy to write by accident: the two sides of the comparison reach it by
different routes -- the manifest carries the value as the run recorded it,
the guard the value the operator typed -- so a cell banked at "20.0" can be
checked against a request for "20" and a string compare would abort a correct
resume on a formatting difference alone. Cases feeding both spellings pin
that, and the garbage-value cases below pin the other end of it: awk reads an
unparseable string as 0, so the numeric branch has to check that what it was
handed is a number before believing they agree.

Nothing launches. The cases run without --dry-run -- the guard sits below the
point where --dry-run exits -- so MIN_FREE_MB is set absurdly high, which
trips the disk guard immediately AFTER the resume guard and before the cell
is started. That also gives the "guard wrongly let it through" outcome its
own distinguishable name (LAUNCHED) instead of a 3000 s gazebo run.
```

## rg_manifest()

### guardcal-fixture-verdict-clean

**The fixture verdict must be CLEAN** — attached to `rg_manifest() {` (line 1023)

```text
run_gates_verdict IS `CLEAN`, WHICH IS A TOKEN THE HARNESS CAN ACTUALLY WRITE.
It said `VALID` until 2026-09-18, and nothing in run_explo_sim_rviz.sh has ever
emitted that word: the teardown writes exactly one of CLEAN, SUSPECT or
INVALID. The reference manifest was therefore a manifest no cell could have,
which cost nothing while the resume guard asked only "is this literally
=INVALID", and became thirteen simultaneous failures the moment it started
distinguishing CLEAN from everything else. A fixture that cannot be produced
by the thing it stands in for is a fixture that will one day disagree with it
for a reason that has nothing to do with the case being tested.
```

## rg()

### guardcal-fixture-keys

**What the reference manifest must carry** — attached to `rg() {` (line 1063)

```text
The five keys above the EOF joined the guard in the C2/C3 pass and were not
added here at the time, which put SIXTEEN of the cases below into permanent
ABORT: every SKIP case failed on `link_gate=<absent>` long before reaching the
key it was written to exercise. The suite exited non-zero either way, so the
16 reds read as one known breakage rather than as sixteen assertions that had
stopped asserting anything -- the cases pinning the FLOAT-formatting branch
(20 vs 20.0, padded values, the nan/awk trap) were the expensive ones to lose,
because that branch is the one that aborts a CORRECT resume.

Values are the reference campaign's, i.e. what a no---env `--arms mtare_hybrid
--seeds 1` invocation predicts: LINK_GATE defaults to 1 in the launcher and
run_campaign mirrors that when --env carries no LINK_GATE, --comms defaults to
1 so the effective veto is live too, MIDRUN_SILENCE and TEAM_WORLD_HZ mirror
the launcher's 90 and 1.0, and done_seek is off (the campaign spells it 0, the
runner writes the ROS bool).

n_robots is deliberately NOT here. Its check is a consistency test across the
banked cells of a tag, and it skips a manifest that does not carry the key --
so an absent one is a case in its own right rather than a hole, and every rg()
case banks exactly one cell anyway.
```

### guardcal-skip-dirty

**The SKIP-DIRTY outcome** — attached to `_got=SKIP-DIRTY` (line 1109)

```text
BANKED BUT NOT CERTIFIED. Its own outcome name, because it is neither of
the two the guard used to have: the cell IS skipped (so it is not ABORT
and not LAUNCHED) but it is skipped with a verdict gate_g8 will hard-fail,
and folding it into SKIP would make the loud path and the silent path
indistinguishable here — which is the exact defect being calibrated.
```

## that IS the set of things the guard models -- and assert the invariant (part 2)

### guardcal-radio-regime-keys

**Radio regime keys** — attached to `rg ABORT "tree_attenuation_db=11.98 is the old radio"         "$RG_ARM" tree_attenuation_db 11.98` (line 1136)

```text
The radio regime. The first of these is the change of 2026-09-03 itself: a
cell banked under the 11.98 dB trunks, resumed by a campaign running the
70 dB ones. Before this key was compared it scored SKIP.
```

### guardcal-ttl-suffix

**The allocator TTL suffix and sentinel** — attached to `rg SKIP  "_ttl120 with a matching 120.0 is skipped"           mtare_hybrid_ttl120 alloc_peer_pos_max_age_sec 1` (line 1164)

```text
The allocator peer-position TTL, set from the "_ttl<N>" suffix. Same sentinel
shape as the radius above, with one difference that matters: 0 is a REAL level
here (unbounded, the control arm) rather than a spelling of "unset", so "0.0"
and "none" are the same behaviour and must still be different cells.

The two SKIP cases below are also the only test of the SUFFIX PARSER itself,
and they test both halves of it at once. A parser that failed to strip "_ttl120"
would leave cell_pos_ttl empty (want "none" against a manifest saying 120.0 ->
ABORT, not SKIP) AND leave cell_mode as "mtare_hybrid_ttl120", which matches no
branch of the arm-stack case, so cell_world would fall back to 0 against a
manifest saying 1 and abort there instead. Either way the SKIP does not happen.
```

### guardcal-numeric-compare

**Float keys compare numerically** — attached to `rg SKIP  "separation_radius_m=20.0 matches a requested 20"    "$RG_ARM" separation_radius_m 20.0` (line 1199)

```text
Numeric, not string: a cell can bank "20.0" against a campaign requesting
"20" -- the two spellings travel by different routes, the manifest taking the
value the node resolved and the guard the value the operator typed -- so a
string compare would abort a correct resume. This is the case that would catch
that regression, and the 25.0 one below is what stops the fix from
degenerating into "any radius agrees".
```

### guardcal-non-number-aborts

**Non-numeric manifest values abort** — attached to `rg ABORT "separation_weight=off is not a number"              "$RG_ARM" separation_weight off` (line 1212)

```text
A manifest value that is not a NUMBER must abort, and separation_weight is the
key where getting this wrong is invisible: awk reads any unparseable string as
0, the requested weight is 0 in every campaign run so far, so before the
numeric branch was taught to check its inputs each of these compared EQUAL and
banked a cell whose weight could no longer be determined. "0.0" is the control
alongside them -- a real number that really does agree -- so a fix that
degenerated into "abort on everything" would not pass this block either.
```

### guardcal-nan-compare

**Why nan gets its own case** — attached to `rg ABORT "separation_max_age_sec=nan is not a number"         "$RG_ARM" separation_max_age_sec nan` (line 1225)

```text
"nan" gets its own case at a key whose requested value is NOT 0, because it
fails differently from the strings above. This awk parses it as a real NaN
and then reports nan == <anything> as TRUE, so under a bare `a + 0 == b + 0`
a single corrupted value agrees with every key at every level -- 25.0 as
readily as 0. The "off"/"unset" cases cannot catch that: they only compare
equal where the request happens to be 0.
```

## the verdict itself, which the resume guard used to read one bit of

### guardcal-verdict-directions

**The resume guard reads the whole verdict** — attached to `rg SKIP-DIRTY "a banked SUSPECT cell still skips, but says so"  "$RG_ARM" run_gates_verdict SUSPECT` (line 1241)

```text
The guard's test was `grep -q '^run_gates_verdict=INVALID'`, so of the four
states a banked cell can be in it distinguished exactly one. SUSPECT and a
missing key both landed in the `else` and printed `SKIP (already complete)` --
the same line a certified cell gets -- and gate_g8, which is the only thing
that reads the verdict, is run by hand after the campaign. A bank full of
uncertifiable cells therefore announced itself for the first time at analysis
time, with every hour of sim already spent.

These cases pin the three directions separately, because "it still skips" and
"it says why" are different properties and only one of them was broken.
```

### guardcal-invalid-still-redoes

**INVALID still re-runs the cell** — attached to `rg LAUNCHED   "INVALID still redoes the cell, not just reports it" "$RG_ARM" run_gates_verdict INVALID` (line 1254)

```text
LAUNCHED, not SKIP-DIRTY: INVALID keeps its old behaviour of falling through
to a re-run (the disk guard then stops it, which is what LAUNCHED names here).
The new branch must not have swallowed the one verdict that was already acted
on — an INVALID cell that started merely being reported instead of redone
would be a silent loss of the only automatic remedy this driver has.
```

### guardcal-redo-suspect

**REDO_SUSPECT tested both ways** — attached to `for _rs in 1 0; do` (line 1261)

```text
REDO_SUSPECT=1 IS THE OPT-IN, AND IT HAS TO BE TESTED IN BOTH DIRECTIONS.
The env var is the operator's way of saying "re-roll the uncertified cells",
and a knob that silently does nothing is worse than no knob: it converts a
deliberate decision into a no-op that looks like it was honoured. Run inline
rather than through rg(), which has no env hook.

LAUNCHED is the wanted outcome — the cell is passed through to be re-run and
stopped immediately afterwards by the absurd MIN_FREE_MB, which is the same
trick every case above uses to avoid starting a 3000 s gazebo run.
```

### guardcal-resume-default-readback

**Resume guard copies of launcher defaults** — attached to `for _spec in TREE_ATTEN:TREE_ATTEN_REQ MAX_RANGE:MAX_RANGE_REQ \` (line 1312)

```text
Same hazard as the LINK_GATE readback above, and the reason that one exists is
on display here: TREE_ATTEN's shipped default MOVED, from 11.98 to 70.0. The
resume guard predicts what a cell's manifest will say, so a stale copy of a
default does not fail loudly -- it aborts every resume of a campaign that is
running exactly as intended, or, in the other direction, skips a cell from
the wrong regime. Read all three literals back out of the launcher.
```
