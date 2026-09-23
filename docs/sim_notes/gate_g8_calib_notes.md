# gate_g8_calib.py — design notes and history

The long comments of `sim/gate_g8_calib.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 10
- [base_events](#base_events) — 5
- [Module scope (part 2)](#module-scope-part-2) — 2
- [run_gate](#run_gate) — 1
- [two_mission_completes](#two_mission_completes) — 1
- [Module scope (part 3)](#module-scope-part-3) — 1
- [strip_endpoint_fields](#strip_endpoint_fields) — 1
- [Module scope (part 4)](#module-scope-part-4) — 3
- [_each_runstart](#_each_runstart) — 1
- [Module scope (part 5)](#module-scope-part-5) — 5
- [_mtare_campaign](#_mtare_campaign) — 1
- [_mtare_campaign_patched](#_mtare_campaign_patched) — 1
- [Module scope (part 6)](#module-scope-part-6) — 8
- [audit_fixture_against_real_cell](#audit_fixture_against_real_cell) — 4
- [Module scope (part 7)](#module-scope-part-7) — 6
- [check 3j — one radio regime per campaign, and it must be the declared one.](#check-3j--one-radio-regime-per-campaign-and-it-must-be-the-declared-one) — 1
- [The nav-binary witness (check 3, NAV_BIN_KEY)](#the-nav-binary-witness-check-3-nav_bin_key) — 1
- [cfg_case](#cfg_case) — 1
- [check 3l -- the sensor and candidate configuration](#check-3l----the-sensor-and-candidate-configuration) — 2

## Module scope

### g8cal-nav-scovox-rev

**Nav and scovox identity constants** — attached to `NAV_REV = "c9f83a7"` (line 30)

```text
The other two identity fields the gate checks. They are named here rather than
spelled inline because the gate used to hardcode them and the fixture happened
to bake the same strings, so the two files agreed by coincidence rather than by
construction -- and the coincidence held right up until simple_nav_3d changed.
```

### g8cal-control-arms-from-gate

**Control arms read from the gate** — attached to `_m = re.search(r'os\.environ\.get\(\s*"GATE_CONTROL_ARMS"\s*,\s*"([^"]*)"\s*\)',` (line 38)

```text
WHICH ARMS ARE CONTROLS, read out of the gate rather than restated here.

The fixture writes reconnect_enabled=False and omits the reconnect_dispatch
row for a control arm, and gate check 3e compares exactly that against the arm
name. So if the two files ever disagree about what "control" means, every case
in this file exercises a cell the gate refuses for a reason no case mentions:
the expect_clean cases go red and the planted-defect cases still "pass",
having caught the disagreement instead of their defect.

That drift had already happened. This file said ("off", "mtare_off") while the
gate's default had become "off" alone, so an mtare_off fixture would have been
built manoeuvre-less and then scored as a treated arm with its dispatch
missing. Reading the literal back out is the same move campaign_guard_calib.sh
makes for the launcher's LINK_GATE default, and for the same reason: nothing
else links the two files.
```

### g8cal-manifest-radio-roster

**Manifest radio and roster fields** — attached to `MANIFEST = f"""run_end_reason=all_done` (line 61)

```text
The three radio-regime fields carry the SHIPPED values, because that is what
a cell run today records and check 3j compares every campaign against them.
Leaving them out would make every fixture cell look like a pre-2026-09-03 run
and fire 3j on all of them — which is the correct behaviour of the check and
the wrong fixture.

The two ROSTER keys are written from ROBOTS rather than left out, because a
real run_manifest.txt carries both and the gate reads its per-cell roster from
them. Omitting them sent every fixture cell down robots_of()'s last-resort
path -- the roster inferred from the *.events.jsonl files present -- which is
reported as UNRESOLVED on purpose, because a robot whose log is missing
entirely drops out of an expectation derived from the logs that exist. Every
expect_clean case then scored rc=3 for a property of the FIXTURE. They are
written from ROBOTS so a fixture that grows a third robot cannot forget to
declare it, which is precisely the mistake that made the gate blind at N>=3.
```

### g8cal-schema-pin

**The calibrator's schema pin** — attached to `SCHEMA_VERSION = 11` (line 91)

```text
MUST TRACK gate_g8.py's pin. This file exports GATE_SCHEMA_VERSION from this
constant when it invokes the gate, so a stale value here does not fail — it
calibrates the gate against a schema no binary writes and reports PASS, while
production rejects every real cell. Moved 4 -> 5 with generation 10, and
5 -> 6 with generation 17 (the rendezvous appointment fields).

THAT FAILURE MODE IS NOT HYPOTHETICAL — it happened to this pin's sibling on
2026-09-17. experiment_log.hpp went to kSchemaVersion=6 and gate_g8.py's pin
stayed at 5; check 3d is an exact equality, so every cell of the new
generation would have scored INVALID and run_campaign.sh, under GATES_STRICT=1,
would have REDOne each one forever. The gate was caught before a campaign
started. THIS copy was not caught by the same pass, and it is the more
dangerous of the two precisely because it cannot fail loudly: it exports
itself to the gate, so a stale value here silently moves the whole calibration
onto a dead schema and still prints ALL PASS. Whenever kSchemaVersion moves,
BOTH of these move with it, and the one that matters more is this one.

AND IT HAPPENED AGAIN, on 2026-09-18, exactly as written above. The header went
to kSchemaVersion=8 and BOTH pins stayed at 7 — the paragraph predicting the
failure was sitting directly over the stale constant while it drifted. So the
instruction "whenever kSchemaVersion moves, both of these move with it" is now
enforced by assert_schema_pins_agree() below instead of being asked for: a
prose reminder has now failed twice at the only job it had, and the third
occurrence would have taken ts4 with it (check 3d is an exact equality, so a
gate pinned at 7 hard-fails every cell of a schema-8 campaign).
```

### g8cal-audit-schemas

**Schemas usable as audit references** — attached to `AUDIT_SCHEMAS = (10, 9, 8, 7, 6, 5, 4, 3)` (line 122)

```text
Schemas whose EVENT SHAPES the fixture may be audited against. A SET of
permissible references, not a preference order — the scan below picks the
newest member actually present, which is a different thing from the order the
names are written in here.

Not the same question as the pin: v4 adds five default-off event kinds and no
field to any existing one, so a banked schema-3 cell is still an exact shape
reference for all sixteen legacy kinds. Keeping 3 here is what stops
audit_fixture_against_real_cell() from going UNRESOLVED — permanently silent —
on the very schema bump it most needs to be watching.

Drop a version from this tuple the moment a bump CHANGES an existing event's
fields, because then its cells stop being a valid reference.

AND THAT RULE HAD NOT BEEN APPLIED SINCE v4, which is four bumps and a year of
campaigns. Read against the schema history in experiment_log.hpp, every one of
them qualifies to be here:
  v5 (gen 10)  kEventKinds unchanged, nothing removed; two v4 fields go inert
               but keep being written. Shapes identical.
  v6 (gen 17)  "kEventKinds is unchanged and nothing was removed" — the bump is
               RendezvousAgreedEvent::t_meet_sec changing MEANING.
  v7 (gen 19)  "No field moved and no name changed" — a rendezvous_outcome
               label changes meaning.
  v8 (gen 23)  one meaning change (`outcome` gains "unplaceable") and three
               ADDITIVE fields: run_end.midrun_attempts_used,
               mission_complete.homing_held_sec, goal_amnesty.source.
  v9 (gen 25)  "No column moves; two meanings do" — rendezvous_agreed.
               t_meet_sec's floor and a new state_change reason. Shapes
               identical. It belonged here when it landed and was missed; the
               tripwire below is what caught the omission, two bumps late.
  v10 (gen 29) one ADDITIVE field (rendezvous_agreed.agreed_base_sec) and two
               meaning changes on that same event. Additive, so every older
               entry stays valid as a weaker reference.
Meaning changes are invisible to a key-set comparison, and additive fields make
an OLDER cell a weaker reference, never a wrong one — the writer scan below is
what covers the gap. So the entries stay and the newest present wins.

Leaving it at (4, 3) was not neutral. Nothing under ~/hmr_campaign at schema 5
or later sits where the scan could see it, so the tuple and the one-level scan
fixed each other in place: the audit certified the fixture against a
schema-3/4 event shape, printed the schema it used, and looked exactly like a
check that was working.
```

### g8cal-fields-from-writers

**Fixture fields follow the C++ writers** — attached to `PARAMS = {` (line 166)

```text
The field names below are taken from the C++ WRITERS, not from what the gate
expects to find. That distinction is the whole reason this file was rewritten
in round 4: the fixture used to put git_rev at the top level of run_start and
team_incomplete_sec on peer_lost/peer_seen, matching two mistaken beliefs in
gate_g8.py. The two errors cancelled inside the harness, so it printed ALL
PASS while the gate was simultaneously unable to fail on a stale binary and
unable to pass on any real cell (34 spurious hard failures on the first real
one). A fixture that encodes the reader's assumptions tests nothing; see
audit_fixture_against_real_cell() at the bottom, which now checks this
mechanically against a banked cell whenever one is present.

  run_start.params.git_rev        explo_planner_node.cpp:3493 (addParamStr)
  run_start.schema_version        experiment_log.cpp:287, top level
  peer_lost / peer_seen           experiment_log.cpp:407-425 -- peer,
                                  silent_sec, peers_live, expected_peers,
                                  and first_contact on peer_seen only
  reconnect_dispatch              experiment_log.cpp:431-478 -- the ONLY
                                  writer of team_incomplete_sec
```

### g8cal-done-action-present

**done_action in the clean fixture** — attached to `"done_action": "idle",` (line 196)

```text
The fixture had no done_action, so check 18c fired on the CLEAN cell the
moment it was added -- the fixture, not the gate, was wrong. Worth
recording: a check whose happy path was never represented in the
calibration would have looked like a real hard failure on the first live
campaign, and the temptation then is to delete the check.
```

### g8cal-3l-sensor-params

**Check 3l sensor and candidate params** — attached to `"fov_hfov": 6.28318,` (line 202)

```text
Check 3l, generation 9. The sensor and candidate configuration. Present
at the D1 values, because 3l's happy path has to be represented here or
the first live campaign would be the first time anyone saw it run --
the same mistake done_action above records, and the reason a check that
first appears as a red line on real data gets deleted rather than read.
```

### g8cal-3f-reachability-params

**Check 3f reachability params** — attached to `"reconnect_midrun_silence_sec": 90.0,` (line 222)

```text
Generation 9, check 3f. These three are the whole treatment-reachability
argument: the clock the mid-run trigger arms on, whether the link veto was
wired up at all, and the debounce in front of it. g8r1 satisfied every
other field in this dict and still ran a treatment that could not fire, so
a fixture without them cannot exercise the check that exists to catch it.
```

### g8cal-3g-stack-keys

**Stack keys always emitted** — attached to `"cell_world_enable": False,` (line 230)

```text
Check 3g. The node emits all five UNCONDITIONALLY, which is what lets the
gate treat their absence as "this cell predates the stack" rather than as
a default — so the fixture has to emit them unconditionally too, at the
off values, and let build_events flip them for an m-tare arm. A fixture
that carried them only on the treated cells would have made 3g's control
direction untestable, and the control direction is the one that decides
whether the comparison holds.

rendezvous_schedule_enable is the fifth, and it does double duty: check
3h reads its PRESENCE as the binary generation, so a fixture that omitted
it would silently date every synthetic cell to before P5 and take the
per-arm half of 3g out of service entirely.

pursuit_predictor is the SIXTH, added by P6, and it does the same double
duty one phase further out: 3h reads its presence as the P6 generation.
Omitting it here would date every synthetic cell to before P6 and quietly
retire the two `_mdp` arms from this harness entirely — which is the state
the gate was in when it hard-failed 60 of ts4's 120 cells for running the
arm they were assigned. `trail` is the node's own default and is the value
a non-chasing arm carries; base_events flips it for the `_mdp` tokens.
```

## base_events

### g8cal-reconnect-by-control-arms

**Which arms run the manoeuvre** — attached to `_rdv = arm not in (CONTROL_ARMS if control_arms is None else control_arms)` (line 281)

```text
Every arm but the control reaches the manoeuvre. Was `arm == "hybrid"`,
which made the fixture unable to express any OTHER treated arm: an
mtare_hybrid cell would have been written with reconnect_enabled=False
and check 3e would have "caught" a defect the fixture invented.
`control_arms` overrides the default parsed out of gate_g8.py, for the
cases that declare their own GATE_CONTROL_ARMS. Without it a fixture for
a newly-declared control arm — mtare_off, the §3.6.1 factorial's own
control — would be written with reconnect_enabled=True while the gate
was told to expect False, so check 3e would hard-fail alongside the
planted defect and the case would "pass" on a failure it did not plant.
Both spellings, exactly as the node stamps them since the 2026-09-03
rename. A fixture carrying only one would leave the gate's fallback
untested on the shape it will actually meet; the two cases below then
take each spelling away in turn, so neither read can rot silently.
```

### g8cal-mtare-per-token-stack

**Per-token m-tare feature stacks** — attached to `if arm.startswith("mtare_"):` (line 298)

```text
Each m-tare arm ran ITS OWN stack; anything else ran none. This mirrors
the launcher's own per-token expansion rather than the node's OR,
deliberately: the OR is what check 3g exists to close, so a fixture built
from it could never express the half-treated cell.

Per token and not a single "all on" branch, because the doc §3.6.1
factorial's whole point is that its four arms differ in the two reconnect
mechanisms while sharing P1-P3. A fixture that switched everything on for
any mtare_* name would build mtare_pursuit cells carrying an appointment
and mtare_off cells carrying the value gate — i.e. it would be unable to
express three of the four arms it is meant to calibrate the gate against,
and the cases below would all be testing mtare_hybrid under four names.

SIX ARM TOKENS, not four, since P6 split the two chasing arms by how the
chase is aimed. The `_mdp` suffix is the node's own (`if
(pursuit_predictor_mdp_) arm += "_mdp"`), so it has to be understood here
too or ts4 — whose treated arms are `mtare_pursuit_mdp` and
`mtare_hybrid_mdp` — is a design this harness cannot express.
```

### g8cal-endpoint-counters-zero

**Endpoint counters present at zero** — attached to `_row("run_end", 10, metrics_timer_rows=163,` (line 365)

```text
The two generation-9 endpoint counters ride the run_end row. A healthy
cell carries them AT ZERO rather than omitting them, which is the
whole point of check 3m's existence half: both generations stamp
schema_version 4, so an absent field is the only witness that the
binary predates the fix.
```

### g8cal-dispatch-follows-reconnect

**Dispatch rows follow reconnect_enabled** — attached to `if params["reconnect_enabled"]:` (line 373)

```text
Whether the manoeuvre ran is `reconnect_enabled`, not the module-level
control list: mtare_off is the factorial's own control and dispatches
nothing, so keying off CONTROL_ARMS here would have written it a dispatch
pair while `build()` gave it the reconnect-free planner log, and check 17
would hard-fail on a defect the fixture invented.
```

### g8cal-reconnect-dispatch-row

**The reconnect_dispatch fixture row** — attached to `ev.insert(9, _row(` (line 379)

```text
logReconnectDispatch, experiment_log.cpp:431-478 -- the only writer of
team_incomplete_sec, and therefore the only event on which check 19
can legitimately demand it. A real generation-6 dispatch row carries
link_down_sec but not team_incomplete_sec, which is what makes this a
valid migration witness rather than a field that was always there.
```

## Module scope (part 2)

### g8cal-nav-log-recovery-pair

**Paired recovery episode in nav log** — attached to `NAV_LOG = ("[INFO] global plan ok: 41 poses\n"` (line 411)

```text
A PAIRED recovery episode. The fixture used to omit these entirely, so gate E
reported UNRESOLVED on the clean cell and its pairing arithmetic -- the part
that decides whether a run is scoreable -- had no known-answer case at all.
```

### g8cal-link-gate-live-line

**The link_gate_live witness line** — attached to `LIVE_LINE = (` (line 419)

```text
Check 3f's runtime half asserts the PRESENCE of this token, so the clean
fixture has to carry it. Copied from the RCLCPP_INFO in the link-states
subscription (explo_planner_node.cpp); if that wording is edited without
editing this, the clean case fails loudly, which is the right direction.

It goes in the PER-ROBOT PLANNER LOG, which is where the node's stdout
actually lands (run_explo_sim_rviz.sh's start() redirects each node to
"$OUTDIR/planner_$r.log"). The first draft of this fixture planted it in
$ROOT/<cell>.console.log -- and so did the first draft of the check, so the
harness printed ALL PASS while agreeing with a bug that would have hard-failed
every treated robot-run of every real campaign. A calibration that shares the
code's mistake is [[checks-that-stopped-checking]] with extra steps, so the
planted-defect case below pins the file down and not just the wording.
```

## run_gate

### g8cal-run-gate-pinned-env

**Gate env pinned, not inherited** — attached to `env = dict(os.environ, GATE_ROOT=root,` (line 497)

```text
GATE_SCHEMA_VERSION is pinned, not inherited. An operator who exported it
to re-gate a banked campaign would otherwise run this calibration against
a gate expecting a different schema from the one the fixture writes, and
every case would fail on check 3d — a red harness that says nothing about
the check it claims to be calibrating.

GATE_ARMS and GATE_CONTROL_ARMS are pinned for exactly the same reason,
and the reason is not hypothetical for them: scoring the live campaign
requires `export GATE_ARMS=mtare_hybrid,off`, and running this
calibration afterwards in that same shell would hand every one of the
hybrid/off fixtures below an expectation naming an arm they do not
build. Check 21 would then hard-fail all of them — every expect_clean
case reporting FAIL, and every planted-defect case still "passing" while
the gate failed for a reason having nothing to do with its defect. The
harness would go red across the board and be unable to say why.
env_extra still overrides, which is how the mtare cases declare theirs.
```

## two_mission_completes

### g8cal-3m-endpoint-cases

**Check 3m endpoint cases** — attached to `def two_mission_completes(ev):` (line 658)

```text
Generation 8 let a run that had already homed be sent home again by the late
coverage latch -- 16 robot-runs on ts1b with two mission_complete rows, all in
the treatment arm. Generation 9 closes it in the node (mission_return_done_)
and in the writer (logMissionComplete). These cases calibrate the gate's read
of BOTH halves, and of the third thing that matters more than either: whether
the fields were there to read at all.
```

## Module scope (part 3)

### g8cal-3m-refused-homing-health

**A refused homing request is health** — attached to `case("the latch having REFUSED a later homing request (health, not fault)",` (line 685)

```text
NOT a failure, and this is the case most likely to be got wrong by a later
edit: the late coverage latch legitimately asks to home a second time, and the
node refusing it is the fix working. Failing on it would make the gate reject
exactly the runs the fix protects. The health line must be PRINTED, which is
what expect_text asserts -- rc=0 alone would also be satisfied by a gate that
had stopped reading the field.
```

## strip_endpoint_fields

### g8cal-strip-both-arms

**Strip endpoint fields from both arms** — attached to `_strip(ev)` (line 720)

```text
BOTH arms. build() writes the off cell from a fresh base_events, so a
mutation of `ev` alone reaches half the campaign -- which is what the
first draft of this case did, and the gate then reported 2 of 4 witnesses
and read as though everything had been checked. That draft failure is why
check 3m has a mixture clause at all; the mixture case below is its
known-answer.
```

## Module scope (part 4)

### g8cal-3n-planner-log-witness

**Check 3n reads the planner log** — attached to `_DUP_WARN = ("[WARN] run_end was already written at t_sim=603.900; this "` (line 758)

```text
3n reads the PLANNER LOG, not the jsonl, and that is the whole point: the
duplicate-run_end counter cannot ride out on run_end itself (a counter that
only becomes non-zero after the row is already on disk would read 0 forever),
and a trailing event would break the "run_end is the last line" invariant
every tail-reading consumer depends on. So the witness is stderr, and these
cases are what make the stderr reader a check instead of a hope.

Both markers are exercised separately, because they are written by DIFFERENT
code at different times — the in-run WARN and the destructor ERROR — and the
destructor's output is the less certain of the two to survive rclcpp
teardown. A reader that matched only one would go quiet exactly when the
other was all that reached the file.
```

### g8cal-3n-missing-log

**Missing planner log for 3n** — attached to `root = tempfile.mkdtemp(prefix="gatecal_")` (line 792)

```text
A MISSING planner log is unanswerable, not clean — the same distinction check
3f draws, and for a stronger reason: 3n has no fallback reading whatsoever,
because the counter has no path into the jsonl. Asserted directly rather than
through case(), since deleting the log also blinds 3f, 17 and 20 and rc is 1
for reasons that are not what is under test.
```

### g8cal-3n-denominator

**3n denominator shrinks to two** — attached to `said_partial = re.search(r"no duplicate run_end in 2 robot-run", out)` (line 806)

```text
The off cell's two robot-runs survive; the hybrid cell's two must be
UNRESOLVED and must NOT be silently folded into the clean denominator.
Pinned to 2-of-4 by VALUE: a denominator that stayed at 4 would be the
gate counting robot-runs it never read, which is the exact failure this
whole check exists to make impossible.
```

## _each_runstart

### g8cal-3b-3c-inert-history

**Checks 3b and 3c were inert** — attached to `def _each_runstart(ev, fn):` (line 845)

```text
Every case below fired zero times before round 4. 3b and 3c were inert
because they read git_rev from the top level of run_start, where the node has
never written it, so `rev` was always "" and startswith("") is always True.
The fixture agreed with the bug, which is why the harness printed ALL PASS.
```

## Module scope (part 5)

### g8cal-3e-two-spellings

**Two spellings of the master switch** — attached to `case("reconnect disabled, LEGACY key only (a pre-rename cell)",` (line 881)

```text
The master switch has two spellings after the 2026-09-03 rename, and check 3e
reads whichever is present. Each of the next two cases DELETES one spelling
and disables the other, so a gate that stopped reading either one fails here
instead of passing a control run in a treated directory. The legacy case is
not hypothetical: every cell up to and including cr5 carries only that key.
```

### g8cal-3f-reachability-cases

**Check 3f reachability cases** — attached to `case("mid-run clock set where the trigger cannot reach it",` (line 899)

```text
g8r1 is the known-answer case this whole section is calibrated against: it
passed every other check in this file while 87 % of its treated arm was
behaviourally the control, because the mid-run clock sat at 240 s against an
outage distribution whose p90 is 52-111 s. Every case below is a way that can
recur, and each was confirmed to FAIL the gate before being written down --
the guard added without a plant-a-failure case is the guard that quietly stops
checking [[checks-that-stopped-checking]].
```

### g8cal-3f-token-wrong-file

**Token in the wrong log file** — attached to `case("token present but in the harness console log, not the node's",` (line 936)

```text
The regression pin for the defect this fixture itself once carried: the token
present, but in $ROOT/<cell>.console.log, which holds only the two harness
scripts' own log() output and never a line the node emitted. The check and the
fixture agreed on the wrong file, so the harness printed ALL PASS for a check
that would have hard-failed every treated robot-run ever recorded.
```

### g8cal-3f-missing-log

**Missing planner log for 3f** — attached to `root = tempfile.mkdtemp(prefix="gatecal_")` (line 950)

```text
The planner log being ABSENT is a different verdict from the line being
absent: one is unanswerable, the other is a failure. Conflating them is how a
missing artefact reads as a pass.

Deleting the log also blinds checks 17 and 20, which read the same file, so
this cell hard-fails for those reasons too and rc is 1. That is not what is
under test here, so the assertion is made directly on 3f's two verdicts: the
UNRESOLVED line must be present AND the "no link_gate_live:" hard failure must
NOT be, because a missing artefact must never be scored as a missing veto.
```

### g8cal-declared-nav-rev-cases

**Declared nav revision is compared** — attached to `identity_file_case(` (line 1143)

```text
The regression that motivated declaring these two. git_simple_nav_3d was a
literal "c9f83a7" in the gate, so a campaign built on ANY other nav revision
hard-failed check 3 on every cell with no way to clear it. The case that proves
the fix is not "a correct file passes" -- that passed before too, by
coincidence, because the fixture baked the same literal. It is these two:
```

## _mtare_campaign

### g8cal-mtare-arm-cases

**Arm-agnostic known-answer cases** — attached to `def _mtare_campaign(root, tag, treated_stamp="mtare_hybrid"):` (line 1187)

```text
Every arm assertion in this gate used to be spelled `arm == "hybrid"` or the
literal pair ("hybrid", "off"). That is not a check of the campaign, it is a
check of one campaign's spelling, and the three cases below are the
known-answer set for the generalisation.

The first is the one that matters: an off-vs-mtare_hybrid campaign is the
same experiment with a different treatment, and before this it hard-failed on
a correct run — check 3e demanded reconnect_enabled=False of every arm that
was not literally "hybrid", and check 21 called mtare_hybrid unexpected.
```

## _mtare_campaign_patched

### g8cal-3g-or-stamp

**Why 3g checks the whole stack** — attached to `def _mtare_campaign_patched(root, tag, treated=None, control=None,` (line 1258)

```text
The node stamps `mtare_` when `global_alloc_enable_ || reconnect_gate_info_
|| rendezvous_schedule_enable_`. That is an OR, so the arm NAME proves at
most one of P3, P4 and P5 was live, and check 3e — which compares the name
against the stamp it was derived from — is satisfied by a cell running a
third of the treatment. Worse, cell_world_enable and team_world_hz rename
NOTHING, so a cell that ran no census or no exchange keeps the mtare_hybrid
name, the mtare_hybrid stamp, and every other check.

Since P5 the expectation is a per-arm VECTOR rather than one boolean, because
the §3.6.1 factorial's four arms deliberately differ in two of the five
features. That makes two new ways to be wrong, and both are planted below: a
treated arm missing a feature it is defined to carry (the old failure), and a
treated arm CARRYING one it is defined not to — an appointment in the
chase-only cell, which would confound the very contrast the campaign exists
to measure and which the old all-on rule could not even express.

Below: every way a treated cell can be less (or more) than its arm, the one
way a control cell can be treated, the pre-stack binary whose arm cannot be
certified in either direction, and the two generation cases 3h owns. Each
planted defect leaves every OTHER check passing, which is what makes 3g the
only thing standing between the campaign and a contrast that is not the
contrast it reports.
```

## Module scope (part 6)

### g8cal-3g-no-appointment

**Hybrid with no appointment armed** — attached to `_g3("an mtare_hybrid cell with no appointment armed: caught by 3g alone",` (line 1343)

```text
The P5 analogue, and the same blind spot: with the allocator on, the arm is
still stamped mtare_hybrid, so nothing but 3g can tell that hybrid fell back
to the last-contact midpoint instead of to an agreed cell. Post-P5 that is a
different arm wearing the same name — precisely what check 3h refuses to pool
ACROSS binaries, refused here WITHIN one.
```

### g8cal-3g-pursuit-with-appointment

**Pursuit arm carrying an appointment** — attached to `_g3("an mtare_pursuit cell that armed an appointment: caught",` (line 1352)

```text
THE FAILURE THE OLD ALL-ON RULE COULD NOT EXPRESS: a treated arm carrying a
feature its own definition excludes. mtare_pursuit IS the appointment-off
cell of the 2x2; an appointment in it makes it a second mtare_hybrid arm, and
the factorial then estimates the chase effect from two identical columns.
Only reachable now that the expectation is a per-arm vector.
```

### g8cal-pre-stack-refused

**Pre-stack binary is refused** — attached to `PRE_P5_KEY = "rendezvous_schedule_enable"` (line 1381)

```text
Absence is not a default. The node emits all four unconditionally, so a cell
missing them came from a binary predating the stack and its arm cannot be
certified either way -- the one honest verdict is a refusal, not a pass.

All FIVE keys are stripped, from both arms, so the campaign is uniformly
pre-stack. Stripping only the four would additionally trip check 3h (one
generation per campaign) and the case would then be passing on a defect it
did not plant.
```

### g8cal-gen-keys-together

**The two dating keys** — attached to `GEN_KEYS = (PRE_P5_KEY, PRE_P6_KEY)` (line 1391)

```text
The two DATING keys, stripped together wherever a case means "a binary older
than the m-tare stack". Taking only PRE_P5_KEY away would leave the cell
claiming a predictor it cannot have had, which the gate refuses on its own
("no binary was ever built that way") — a second, unplanted defect, and a case
that passes on one of those is a case that has stopped testing its own.
```

### g8cal-3h-p5-generation

**Check 3h P5 generation cases** — attached to `root = tempfile.mkdtemp(prefix="gatecal_")` (line 1428)

```text
P5 changed what `mtare_hybrid` MEANS: before it, hybrid's fallback
destination was the last-contact midpoint; after it, the agreed cell. The arm
token is identical on both sides of that line, so a campaign holding cells
from both is one arm run twice with two different treatments in it -- and
every downstream script keys off the token.

The discriminator is the PRESENCE of rendezvous_schedule_enable, which a P5+
binary emits unconditionally and an earlier one cannot emit at all. Three
known-answer cases: the mix is refused, and neither pure generation is.
```

### g8cal-p6-predictor-cases

**P6 predictor cases** — attached to `_g3("an mtare_hybrid_mdp cell that ran the trail: caught",` (line 1529)

```text
P6 split the two chasing arms by HOW the chase is aimed: `trail` drives at the
peer's last declared goal, `mdp` at a modelled intercept. The node stamps the
`_mdp` suffix itself, so the directory name and the stamp agree and check 3e
is silent — which is exactly why the gate not knowing the token was dangerous
rather than noisy. Every cell of ts4's two treated arms hard-failed 3g as "not
an arm any binary can stamp", and ts4 is half `_mdp`.

Seven known-answer cases: the treated arm that ran its own control, the
untreated arm that ran the treatment, both directions of the generation
boundary, the two malformed dumps, and — the one that makes the rest mean
something — the real four-arm ts4 design scoring CLEAN.
```

### g8cal-3h-p6-mix

**P6 mix needs its own witness** — attached to `_p6("a campaign pooling a pre-P6 arm with a P6 one is refused",` (line 1601)

```text
(h) the P5 test cannot see the P6 mix. Every cell here emits
rendezvous_schedule_enable, so 3h's first half reports "P5+, uniform" and is
satisfied — while `mtare_hybrid` means the trail chase on the control cells
and is ambiguous on the treated ones. That is the whole reason the boundary
gets its own witness instead of riding on P5's.
```

### g8cal-ts4-four-arms-clean

**ts4's four arms score clean** — attached to `root = tempfile.mkdtemp(prefix="gatecal_")` (line 1635)

```text
(l) THE CASE THE OTHERS EXIST FOR: ts4's actual four arms, correctly
configured, scoring CLEAN. Before the `_mdp` rows were added to
MTARE_ARM_STACK this campaign hard-failed 3g on 60 of its 120 cells — every
cell of both treated arms — for running exactly the arm it was assigned.
```

## audit_fixture_against_real_cell

### g8cal-audit-scan-newest

**Audit scan picks newest schema** — attached to `best_path, best_schema, best_dir = None, None, None` (line 1715)

```text
NEWEST PERMISSIBLE SCHEMA, NOT THE FIRST CELL ALPHABETICALLY, and the two
were not the same answer. The old loop broke on the first cell whose
schema was in AUDIT_SCHEMAS, which under `sorted(os.listdir(root))` means
whichever campaign name sorts earliest -- a reference chosen by filename.

AND IT SCANNED ONE LEVEL, which is where the real damage was. Campaigns up
to schema 4 dropped their cells loose at the top of ~/hmr_campaign; every
campaign since writes them one level down (ts4_smoke20_n2/<cell>/), so the
schema 5, 6 and 7 runs were unreachable and this audit could only ever
find a schema-3 or -4 cell. Widening AUDIT_SCHEMAS alone would have
changed nothing, and deepening the scan alone would have changed nothing;
each defect was load-bearing for the other, and together they read as a
working check that named the schema it used.

Depth 2 is the campaign/cell layout and is where it stops.

TWO PASSES, and the split is not stylistic. "Newest" cannot be decided
without reading every candidate's run_start, but the OLD single pass
accumulated the full key set as it went and only stopped early on a
schema it could not use -- which, now that AUDIT_SCHEMAS spans 3..8,
is almost nothing. That version parses all 3649 banked event logs end to
end and takes minutes. So pass 1 reads each file only as far as its
run_start (the first line) to find the winner, and pass 2 parses that one
file. Under a second, and the parse cost no longer grows with the bank.
```

### g8cal-audit-new-vs-invented

**New field versus invented field** — attached to `writers, writer_err = _event_field_writers()` (line 1787)

```text
A key the banked cell does not have is NOT automatically invented: a
banked cell predates whatever the current generation added, so every new
field looks invented to it. The authority for "does this field exist" is
the writer that emits it, the same argument audit_params_against_writers
makes one level down for run_start params. So the real-cell comparison
narrows to "is this a field the CURRENT tree writes", and a key that is
in neither the banked cell nor the writers is the only failure.

("the banked cells are all pre-generation-9" is what this said until
2026-09-18, fourteen generations after that stopped being true. It was
describing the bank as of the round-4 rewrite, and the reference cell the
scan could actually reach never moved off schema 3/4, so the sentence
stayed accidentally true of the audit while going badly false about the
bank. The reach is now the newest cell present, whatever generation that
is, and the paragraph no longer names one.)
```

### g8cal-audit-unseen-kinds

**Report unaudited event kinds** — attached to `if unseen:` (line 1831)

```text
The loop above skips fixture kinds the reference cell never emitted, which
is correct and also a silent narrowing of what "subset of a real cell"
covers. Say how much was skipped: a reference that exercises two of the
fixture's kinds is a much weaker statement than one that exercises seven,
and the PASS line reads the same either way.
```

### g8cal-audit-staleness-tripwire

**Tripwire on a stale reference** — attached to `gap = SCHEMA_VERSION - real_schema` (line 1852)

```text
AND A TRIPWIRE ON THE REFERENCE ITSELF, because everything above is a
SUBSET test and a subset test cannot fail for being stale. A reference
four generations behind agreed with the fixture on every key it had, said
so, and printed PASS -- the audit degraded silently into a weaker audit
rather than into a failure, which is the whole `checks-that-stopped-
checking` shape. The staleness has to be its own verdict or it is not
checked at all.

One behind is the NORMAL state and must stay green: the pin moves when the
header moves, and no cell of the new generation exists until that binary
has been built and run. Today that is exactly the case -- pin 8, newest
bank 7, because generation 23 has not run yet. Two behind cannot happen
that way; it means a whole generation was campaigned without this audit
ever seeing one of its cells, or that the scan has lost its reach again.
```

## Module scope (part 7)

### g8cal-radius-suffix

**The per-cell claim-radius suffix** — attached to `R10_MANIFEST = MANIFEST + "coord_claim_radius_override=10.0\n"` (line 1880)

```text
cr2 varies the MinPos claim radius PER CELL, inside one campaign invocation,
by appending _r10/_r40 to the arm name -- run_campaign.sh strips the suffix
before setting RECONNECT_MODE, so the node stamps `mtare_hybrid` in an
`_mtare_hybrid_r10_` directory. Before this was taught to the gate, every
correctly-run cr2 cell hard-failed twice (3e "directory says arm=X but params
say Y", 3g "not an arm this binary can stamp") and cr3/cr4/cr5 could not be
scored at all.

The fix strips the suffix for those two comparisons only. That is exactly the
shape of change that can turn a check into a rubber stamp, so the first two
cases below are the ones that matter: (a) a suffixed campaign must now score
CLEAN, and (b) 3e must STILL catch a genuinely mis-assigned cell. Without (b),
stripping harder and harder until nothing fails would look like progress.

The suffix is not merely tolerated either. It names a treatment level the
analysis reads straight off the directory, so 3i holds it against the
manifest, and the three ways that can be wrong each get a case.
```

### g8cal-two-suffix-orders

**Two suffixes, both orders** — attached to `TTL0_MANIFEST = R40_MANIFEST + "alloc_peer_pos_max_age_sec=0\n"` (line 1970)

```text
The suffix table grew a second dimension and the hand-written check did not.
`_ttl0` has been the DEFAULT on every campaign since 2026-09-05, so almost
every new cell carries both suffixes -- and the single-pass regex that handled
`_r<N>` alone matched NEITHER on `ts1b_n3_mtare_hybrid_r40_ttl0_seed1`. That
one miss produced five hard failures per cell (3e, 3g, 3i, and check 21 twice)
on a campaign with nothing wrong with it: 76 in total over six cells, which is
how an operator learns to read this gate's output as noise.

So the two-suffix shape is pinned in BOTH orders. Order-independence is a
property of the loop, not of the table, and a case in one order only would
pass against a parser that hardcoded the sequence it happens to see today.
```

### g8cal-3e-unknown-suffix

**Unknown suffix is UNRESOLVED** — attached to `radius_case("an unrecognised trailing token is UNRESOLVED, not a hard failure",` (line 2005)

```text
The two failure modes of check 3e need opposite answers, and conflating them
is what makes a provenance gate untrustworthy in the expensive direction.

A directory reading `mtare_hybrid_zz9` against a stamp of `mtare_hybrid` is
not evidence about the run: it is a knob added to run_campaign.sh after this
file was last edited, and the cells are fine. Condemning them teaches the
operator to score campaigns with the gate disabled. Reporting UNRESOLVED
names the missing table entry instead, and still refuses to call the cell
certified -- the level of that knob genuinely is unchecked.
```

### g8cal-roster-from-cell

**Roster comes from the cell** — attached to `TRIO = list(ROBOTS) + ["husky"]` (line 2034)

```text
The N>=3 analysis blackout was three independent two-robot hardcodes, and this
was the one inside the gate: `for r in ROBOTS` with ROBOTS = ["atlas",
"bestla"] at module scope. On a three-robot campaign it was wrong in both
directions at once. husky's log was never opened, so checks 3b/3c/3d/3e/3f/3g
and 18b/18c ran over two thirds of each team and the gate reported a pass over
the third; and a campaign that RENAMED its robots would have hard-failed "no
events" on cells that were perfectly good.

Every case below needs a fixture that can build a team of any size, which is
why _cell() and build() take a roster. A fixture that could only build pairs
could not have told you whether any of this was fixed.
```

### g8cal-check2-report-only

**Report-only gates and check 2** — attached to `SUSPECT_MANIFEST = re.sub(r"^run_gates_verdict=.*$",` (line 2149)

```text
run_explo_sim_rviz.sh derives SUSPECT from `grep -c "^UNRUN"` over
comms_gates.txt, with no notion of which gate wrote the line, and check 2 then
turns any non-CLEAN verdict into a hard failure. map_agree is a report-only
READING -- no pass threshold, its own header says so -- but until 2026-09-15 it
emitted UNRUN whenever it could not compute, which (being hardcoded to two
planner CSVs) was every N>=3 cell. One informational line it was never
entitled to write therefore rejected every cell of every three- and four-robot
campaign. Both halves of the fix are pinned here, because the forgiving branch
is the kind that quietly becomes "check 2 no longer checks anything".
```

### g8cal-report-only-names-pinned

**Report-only gate names read back** — attached to `_rog = re.search(r"REPORT_ONLY_GATES\s*=\s*frozenset\(\{([^}]*)\}\)",` (line 2164)

```text
The exception is keyed on a GATE NAME, so the two files must agree on the
spelling or it protects nothing. Both literals are read back out rather than
restated here -- the same move this file already makes for the launcher's
GATE_CONTROL_ARMS default, and for the same reason: a rename on either side is
silent, and the failure is a report-only gate rejecting a whole campaign again.
```

## check 3j — one radio regime per campaign, and it must be the declared one.

### g8cal-3j-radio-regime

**Check 3j radio regime cases** — attached to `OLD_RADIO = MANIFEST.replace("tree_attenuation_db=70.0\n",` (line 2253)

```text
The shipped regime moved on 2026-09-03: trunks 11.98 -> 70.0 dB, plus a 30 m
horizon that did not exist before. Nothing read those manifest fields until
3j, so a campaign resumed across the change pooled two radios and every
analysis averaged over them. These cases pin both halves and, more
importantly, pin that the ESCAPE HATCH for the declaration half does not
reach the uniformity half — an override that quietly relaxes more than it
names is how a check stops checking.
```

## The nav-binary witness (check 3, NAV_BIN_KEY)

### g8cal-nav-binary-witness

**Nav-binary witness cases** — attached to `NAV_SHA = "cccccccccccccccc"` (line 2342)

```text
D2 -- the goal-snap append -- lives in simple_nav_planner_node, and until this
key existed NOTHING in the manifest or the gate could say which simple_nav_3d
BINARY ran. git_simple_nav_3d moves with the source tree whether or not colcon
ran, so "edited but not rebuilt" produced a manifest that reported generation 9
on every field while the node on the wire was generation 8.

The requirement is conditional on the campaign carrying the key, which is a
branch, which means it can be wrong in both directions: too lax (a campaign
that CAN answer is not made to) and too strict (banked cells that cannot
answer are hard-failed, so the key gets deleted). Both directions are below.
```

## cfg_case

### g8cal-3l-stale-yaml

**Check 3l and a stale yaml** — attached to `def cfg_case(label, expect_pat, treated_over=None, off_over=None,` (line 2428)

```text
D1 is a params-file change, and a params-file change moves no field the gate
read before this check existed: git_explo_planner does not move for it, and
sha256_shared_params moves but was compared to nothing. So a stale installed
yaml produced a cell running the generation-8 sensor under a manifest that
said generation 9 in every field. These cases are what make that visible.
```

## check 3l -- the sensor and candidate configuration

### g8cal-3l-format-not-finding

**Number format is not a finding** — attached to `cfg_case("int vs float spelling of the same value is not a finding",` (line 2478)

```text
Format is not a finding. The node writes 96.0; a hand-edited params file or a
differently-typed yaml loader may yield 96, and they are the same sensor.
Comparing json.dumps output directly would have called that a campaign built
against two sensor models -- an unsatisfiable hard failure on a CORRECT
campaign, which is exactly the trap check 3 had for git_simple_nav_3d.
```

### g8cal-3l-within-arm

**Within-arm split is a hard failure** — attached to `cfg_case("...but WITHIN one arm it is still a hard failure",` (line 2513)

```text
...but WITHIN one arm the same parameter is still a constant, and a split
there is never a design. This is the case that stops the soft branch from
swallowing a real defect.
treated_only_first is the whole point: overriding BOTH robots of the hybrid
cell would make this a between-arm split and quietly exercise the soft branch
instead -- a case that passes while testing the opposite of its label.
```
