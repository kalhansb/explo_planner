# rendezvous_agreement.py — design notes and history

The long comments of `sim/rendezvous_agreement.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 3
- [main](#main) — 20

## Module scope

### gate-commit-line-wordings

**Why the newest commit wording differs** — attached to `LINE = re.compile(` (line 130)

```text
The gen-20 wording dropped "from t+Ns" because armAppointment had been changed
to overwrite the agreed meeting time with a private per-robot countdown, so
there was no agreed meeting time left to print; it added the echo count because
a FINAL triple committed on first-hand evidence and the old line asserted the
fleet size as though it had been observed. Gen 23 restored the agreed instant
— armAppointment attends the committed schedule again — and so restored the
meeting time to the line.

GEN 23 DOES NOT REUSE THE GEN-19 PHRASING, deliberately. The two forms carry
the same pair of integers, so a gen-23 line written as "every 30s from t+130s"
would parse through the FIRST alternative below and be indistinguishable from a
pre-gen-20 binary in a re-gated bank. The meeting time is appended to the
interval term instead, which lands in the third alternative and nowhere else.

`tmeet` is therefore unknown-not-zero on a gen-20..22 log, and the code below
must keep treating it that way: 0 is a value the other two formats can really
emit.
```

### gate-release-phrase

**The barrier-release phrase** — attached to `RELEASE = "-> re-planning against merged map"` (line 167)

```text
A BARRIER THAT ACTUALLY RELEASED. Generation 29 lets a run hold a SEQUENCE of
triples — a meeting that is kept re-sites the next one — so "how many times
may this robot commit" stopped being a constant and became "once, plus one for
each meeting it kept". This is the phrase that marks a keeping. Matched as a
substring and deliberately on the half of the line that both variants share:
the settled form appends "(held Ns; the exchange applied ...)" and the
no-settle form "(no settle configured, ...)", and both authorise exactly one
re-agreement. sim/manoeuvre_events.py greps the same line's prefix.
```

### gate-split-fail-bound

**The G2b split bound** — attached to `SPLIT_FAIL_SEC = float(os.environ.get("RZV_SPLIT_FAIL_SEC", "240"))` (line 177)

```text
G2b's bound, in seconds of wall clock between the first and last robot to
commit the cell the fleet ends on. Set to the node's mid-run maximum wait,
because that is the point at which the harm stops being theoretical: a robot
holding the other cell for longer than one full wait can arrive, wait out an
entire appointment and leave, all at a cell nobody else is coming to.

IT IS A CEILING, NOT A TARGET. The design's own accepted residual is one
TeamWorld period, so a healthy cell should read single-digit seconds and
anything approaching this bound is already pathological. It is set here rather
than tight because no campaign has yet produced a distribution of this
quantity -- writing a tight bound from a guess is how a gate starts failing
healthy cells, and writing one from the first campaign's own numbers is how it
stops failing anything. Override to calibrate; do not edit the default until
there are numbers behind it.
```

## main

### gate-schedule-key-precondition

**Scheduled arm by parameter, not name** — attached to `if "rendezvous_schedule" not in man:` (line 225)

```text
Inert on the unscheduled arms BY THE EFFECTIVE PARAMETER, never by the arm
name. `reconnect_mode_requested` is the request and the off arm's manifest
has been observed carrying a treatment name in it; `rendezvous_schedule` is
what the launcher actually passed to the node.
A MISSING KEY IS NOT AN INERT ARM. Splitting these two cases is the whole
point: `rendezvous_schedule=0` is an arm that legitimately has nothing to
check, and silence is right. An ABSENT key means this gate no longer knows
how to find its own precondition — a launcher rename, a manifest format
change — and the old `!= "1"` lumped them together and returned 0, so a
rename would have made this gate pass-by-absence on every cell of a
campaign while still printing nothing at all. That is the shape in
~/hmr_campaign: a check that stops checking keeps reporting success.
```

### gate-every-commit-kept

**Why every commit line is kept** — attached to `commits = {}        # robot -> [(cell, interval, t_meet, prov, wall), ...]` (line 277)

```text
EVERY commit line is kept, not just the first. A robot commits more than
once for two unrelated reasons, and both are legitimate:

  THE UPGRADE. The proposer can derive before the allocator has produced a
  single tour, in which case the argmin has one candidate (the centroid of
  the frozen snapshot) and the triple it yields is a placeholder rather
  than a choice. It is published with rendezvous_provisional=1 and may be
  replaced exactly once, by a triple that had something to choose between.

  THE RE-AGREEMENT (generation 29). A meeting the team KEEPS ends with the
  fleet standing on the cell agreeing where and when to meet next, so a
  run holds a sequence of triples rather than one. One per meeting kept,
  which is what the shape check below counts.

Keying G2 on the FIRST commit — which is what this gate did when neither
path existed — fails a healthy upgraded run as a SPLIT FLEET, because
robots upgrade in whatever order the echoes land. G2 therefore compares
where the fleet CONVERGED, and the shape itself is checked separately
below so that the relaxation cannot hide a real split.
```

### gate-echo-terms

**Parsing the echo terms** — attached to `echo_rows = []      # (robot, n_asserted, echoes, peers, wall) — gen-23 only` (line 299)

```text
THE ECHO TERMS, WHICH LINE HAS ALWAYS CAPTURED AND THIS FILE USED TO THROW
AWAY. `echoes E/P` is the node's own statement of how many peers had been
heard holding this exact triple (E) out of how many there are (P), and on
generation 23 the commit rule requires E == P == fleet-1 for EVERY commit
(the `if (on_pair != fleet_.size() - 1) return;` early return above the log
call). The node's comment beside that line says, in as many words, that if
the two ever differ in a banked log the commit rule regressed and a reader
holding only the text log should be able to see it. Capturing three regex
groups and never comparing them is how that reader never does.

Kept SEPARATE from the commit tuple rather than appended to it, because
four call sites index that tuple positionally and none of them needs these.

G23 ONLY. On generations 20-22 a final triple could legitimately commit on
first-hand evidence with E < P, so the same numbers mean different things
either side of that boundary and the check has to know which side it is on.
The discriminator is `tmeet_new`: only the third regex alternative can set
it, and that alternative is the gen-23 wording.
```

### gate-tmeet-none-not-zero

**tmeet is None, not zero** — attached to `tmeet = m.group("tmeet_new")` (line 343)

```text
tmeet is absent on a generation-20..22 log by design — the
line stopped printing a meeting time when there stopped
being one, and gen 23 put it back. None, not 0: 0 is a
value both printing formats could really emit, and the two
must not merge.
```

### gate-echo-term-missing

**A meeting time without an echo term** — attached to `if m.group("echoes") is None:` (line 352)

```text
Gen-23 wording. The format string prints the meeting
time and the echo term TOGETHER, in one literal, so a
line carrying one without the other is a shape this
parser does not actually know — the echo group is
optional in LINE only so that gen-19 and gen-23 can
share one alternative's tail. Recorded, not skipped:
skipping is how a reworded line turns the check below
into a permanent silent pass.
```

### gate-unparsed-line-unrun

**Any unparsed commit line is UNRUN** — attached to `if unparsed:` (line 375)

```text
ANY UNPARSED LINE IS A BROKEN GATE, not a quiet robot (2026-09-19).

This used to be `if unparsed and not commits`, so a PARTIAL parse failure
— some robots on a wording this file knows, some on one it does not — fell
straight through to G1 below, where the unparsed robots were reported as
never having committed and the cell FAILed as "SCHEDULED ARM WITH NO
AGREEMENT". That is the gate's own blind spot misattributed to the node,
in the INVALID direction, on a cell that may be perfectly healthy; three of
them abort the campaign. The parser is either current or it is not, and
when it is not the only honest verdict is that nothing was checked.
```

### gate-silent-robot-assert

**Silent robots versus parser drift** — attached to `mis = [r for r in never if r in saw_phrase]` (line 399)

```text
Cannot be a parse failure — that returned UNRUN above — so every robot
here printed nothing at all. Asserted rather than assumed, because the
difference between the two is the difference between blaming the node
and blaming this file, and the whole point of saw_phrase is to keep
that distinction observable instead of inferred.
```

### gate-g1b-echo-invariant

**G1b: the commit rule's invariant** — attached to `if echo_missing:` (line 435)

```text
Generation 23 commits only when every peer has been heard holding the
triple, so on a gen-23 line "all N robots" is an OBSERVATION, not an
inference from the protocol — and the two numbers that make it one are
printed right there. Three things must hold on every such line:

  E == P     every peer echoed. E < P means a robot printed "AGREED by
             all" while holding fewer echoes than there are peers, i.e.
             the early return that guards the commit stopped guarding it.
  P == N-1   the peer count is the fleet minus self. A mismatch means the
             node's own two statements of team size disagree with each
             other, within one format string.
  N == roster  the node's fleet is the campaign's roster. G1 above proves
             every robot in the manifest committed; this proves the node
             was not simultaneously counting somebody else. Without it a
             cell where one robot never joined the fleet passes G1 (the
             robots that exist all committed) while the line itself says
             the team is smaller than the campaign thinks it is.

FAIL and not UNRUN: nothing here is a limit of this gate's knowledge. The
line parsed, the integers are present, and they contradict the binary that
printed them.
```

### gate-g2-cell-not-triple

**G2 compares the cell, not the triple** — attached to `cells = set(v[0] for v in final.values())` (line 485)

```text
THE CELL, NOT THE TRIPLE. This compared all three integers until
generation 20 made two of them inert, and it stays on the cell alone now
that generation 23 has made them live again — for a DIFFERENT reason at
each end, so both halves of the argument are written down here.

On gen 20-22 the other two integers steered nothing: `t_meet` was not
printed at all (a robot departed on its own countdown, so there was no
fleet-wide meeting instant to agree on) and `interval` was carried for
provenance. Failing on them meant two robots both driving to cell 44 could
FAIL as a SPLIT FLEET over integers neither of them would ever act on.

On gen 23 both are live: armAppointment attends the agreed occurrence of
the agreed schedule. But a same-cell disagreement about WHEN is
self-healing on this generation and a disagreement about WHERE is not.
rendezvous_appointment_wait_sec is 0.0 on every campaign arm, which is an
UNBOUNDED barrier: the robot that picked the earlier occurrence stands at
the agreed cell until the others arrive, so the fleet still meets and the
cost is waiting, bounded by one interval. Two robots at two cells never
meet at all. So the cell is still the whole test — not because the other
two integers are inert, but because only one of the three can cost the
experiment its meeting. Disagreement on the other two is reported below.

A gate that can invalidate a healthy cell is worse than no gate, because it
spends the run AND reports a defect that is not there; three of them abort
the campaign.
```

### gate-round-skew-live-commits

**Disagreements nobody can still act on** — attached to `resolved = {}` (line 513)

```text
A DISAGREEMENT NOBODY CAN STILL ACT ON IS NOT A SPLIT FLEET. The
comment above calls the last commit "the only one it can still drive
to"; this block checks that claim instead of assuming it. `final` is
positional, so it silently assumes every robot is on the same round —
and a robot that fell behind (arrived late, waited out its
appointment, never committed again) gets scored against a peer that
advanced two more rounds. That reports a disagreement about a meeting
neither robot is waiting for, on a run where every appointment the two
actually shared named the same cell.

An appointment stops being actionable in exactly two ways: the node
published a rendezvous_outcome for it (it came and went, whatever the
outcome), or its meeting instant lands after the run's last sample so
nobody could attend it. Both are read off the event log on t_rel,
which is the clock t_meet is on — t_sim would manufacture 20-30 s of
fake lateness and with it a fake live appointment.

EVERYTHING UNPROVEN COUNTS AS LIVE. No event log, or a gen-20..22 log
that prints no meeting instant, leaves every commit live and fails
exactly as this gate does today. The block sits inside the FAIL branch
for the same reason: it can only turn a FAIL into a pass, never create
one, so it cannot invalidate a cell that used to be healthy.
```

### gate-round-skew-cell

**Naming the cell under round skew** — attached to `shared = set.intersection(*[set(c[2] for c in rows` (line 569)

```text
THE FLEET IS NOT SPLIT, BUT SOMETHING STILL HAS TO NAME THE CELL.
G2b and the PASS line both need one, and the positional finals
disagree, so use the last appointment EVERY robot committed to:
the last round the fleet was demonstrably together. Without one —
no shared meeting instant at all — there is no fleet state to
describe and the FAIL below stands.
```

### gate-split-no-bug-causes

**Split fleets that no bug produced** — attached to `by_prov = {}` (line 594)

```text
NAME THE TWO NO-BUG PRODUCERS BEFORE BLAMING THE CODE. This detail
used to assert the split "cannot happen without a bug in the
propose/echo path", which is not what the design says: TeamWorld.msg
documents the derive-round cut as the residual the design accepts
rather than pretends away, and the one permitted P->R upgrade reopens
a second window of the same kind. Both produce a genuine split that no
code change would remove, and at N=4 -- where the mutual window is
seconds -- they are the likely explanation, not the unlikely one. The
cell is still INVALID either way; what changes is where the next hour
goes.
```

### gate-schedule-integer-note

**Same cell, different schedule integers** — attached to `inert_txt = ""` (line 632)

```text
THE OTHER TWO INTEGERS, REPORTED AND NOT FAILED ON. Whether they steer a
robot depends on the generation (see the block above): on gen 20-22 they
steer nothing, on gen 23 they are the meeting schedule. Either way a
same-cell disagreement is not a split fleet — the unbounded appointment
barrier turns a wrong occurrence into waiting — but it IS the fingerprint
of a propose/echo path that has started dropping fields, or of an upgrade
round that reached some robots and not others, so it must be visible
rather than dropped on the floor. Carried into the PASS/FAIL detail line
below; never a verdict of its own.

ON A GEN-23 LOG THIS NOTE HAS A COST ATTACHED and the reader should price
it: the robots that disagree pay up to the difference in meeting times
standing at the agreed cell, which is exploration time the arm does not
get back. It is reported in seconds for exactly that reason.
```

### gate-commit-shape-budget

**The commit shape check and budget** — attached to `shape = []` (line 668)

```text
THE SHAPE CHECK. G2 above was relaxed from "every commit line agrees" to
"every robot ended in the same place"; this is what keeps that relaxation
from covering for a real defect. A commit nothing paid for is a protocol
violation even when the fleet happens to agree at the end, and a
provisional commit AFTER a final one is a downgrade off a real choice back
onto a placeholder. Sits out entirely on a pre-generation-17 log, which
cannot report the flag.

THE BUDGET IS NOT A CONSTANT ANY MORE (generation 29). It was "max 2: P
then R" — one pair per run, optionally upgraded once off the centroid
fallback — and gen 29 makes a kept meeting re-site the next one, so a
healthy two-meeting run commits three times and the old rule failed it by
construction. What replaces it is the same rule with the meetings counted:
one pair to start, one more if the first was the placeholder, and ONE PER
BARRIER THIS ROBOT ACTUALLY RELEASED FROM. Anything above that is a derive
no meeting authorised, which is not cosmetic — deriveRendezvousProposal
mints t_meet as now+interval, so every unpaid-for commit slides the
instant the fleet had agreed to. The ts4 gen-29 N=2 rendezvous smoke is
the case: one release, four commits, cell 23 at t+1055s and then t+1061s.

Per robot rather than fleet-wide because the two are not interchangeable:
a follower adopts the proposer's next triple as soon as it hears it, which
can be a tick before its own release, so a running comparison would race.
Counted over the whole run, it cannot.
```

### gate-g2b-split-duration

**G2b: how long the fleet was split** — attached to `first_on_winner = {}` (line 718)

```text
G2 compares only each robot's LAST commit, which is the right question for
"can they still drive to the same place" and the wrong one for "did this
cell contain a split appointment". Those come apart in exactly one way:
the fleet cuts an upgrade round, some robots drive the placeholder while
others drive the final cell, and then the radio comes back and everyone
re-latches before the run ends. Every robot's final commit agrees, G2
PASSes, and the cell silently contains up to a full wait-cap of one robot
standing at the wrong cell. That is a completion-time bias landing
squarely in the two arms the campaign is comparing, with no witness.

The measurement is the spread of FIRST commits of the winning cell: from
the earliest robot to adopt it to the last, at least one robot was holding
something else. Reported on every cell rather than only on the bad ones,
because a threshold nobody has calibrated is worse than a number everyone
can see -- and this number has never been observed, so the FAIL bound
below is deliberately set where the harm is unarguable (a robot could have
waited out an entire appointment at the wrong cell) rather than where the
protocol is merely untidy. Tighten it once a campaign has produced a
distribution; do not tighten it from this comment.
Keyed on the cell for the same reason G2 is: a robot that re-committed the
same cell with a different interval never moved, so counting that as
"adopting the winner late" would manufacture a split window out of a field
nothing reads.
```

### gate-g3-t-wall-required

**G3 requires t_wall_sec** — attached to `t_wall_raw = d.get("t_wall_sec")` (line 798)

```text
t_wall_sec IS REQUIRED, NOT OPTIONAL. This read
`float(d.get("t_wall_sec", 0.0))` until 2026-09-18, so a
row without the field compared 0.0 > <epoch seconds> —
False, always — and was silently skipped. Drop the field
from the envelope and G3 stops finding contradictions and
PASSes forever, with nothing in the output distinguishing
"no post-commit no-agreement armings happened" from "the
field this test reads is gone". That is precisely what
this module's docstring forbids (UNRUN IS NOT A PASS) and
what the PHRASE/LINE split was built to prevent on the
log-line half of the same file.

"The commits parsed, so the field must be there" is NOT an
argument: commit_wall comes from the cell>=0 rows, these
are the refused cell<0 rows, and the two schemas can drift
independently.
```

### gate-pass-line-mechanism

**Which mechanism the cell exercised** — attached to `final_prov = [v[3] for v in final.values()]` (line 843)

```text
WHICH MECHANISM THIS CELL ACTUALLY EXERCISED, on the PASS line, because a
cell that converged on a placeholder is a legitimate rendezvous cell that
never ran the scheduler's argmin over the allocator's tours. It passes, it
banks, and it measures meet-at-the-centroid — a different treatment from
the one the arm name implies. That distinction is invisible in every other
artifact the run produces, so it has to be said here or not at all.
```

### gate-split-vs-span

**Split is not span** — attached to `split_txt = ("; split %.1fs" % split_sec if split_sec is not None` (line 865)

```text
`split` is G2b's measurement and is NOT the same number as `span`. `span`
runs from the earliest commit of anything to the latest commit of
anything, so a healthy [P, R] fleet shows a large span purely because the
upgrade happened late. `split` runs between the first and last adoption of
the WINNING CELL, which is the interval during which the fleet was
actually holding two different appointments. Reading span as if it were
split overstates the harm on every upgraded cell; reading split as if it
were span hides the upgrade entirely. Both are here so neither has to be
inferred from the other.
```

### gate-g1b-ran-on-pass-line

**Whether G1b ran, on the PASS line** — attached to `echo_txt = ("; %d commit line(s) carried a full echo round" % len(echo_rows)` (line 876)

```text
WHETHER G1b ACTUALLY RAN, on the PASS line. `bad_echo` being empty has two
very different causes — every gen-23 line checked out, or there were no
gen-23 lines to check because the binary is older — and a PASS that does
not distinguish them is the shape `checks-that-stopped-checking` warns
about: the invariant would go quiet the moment the wording changed and the
output would look identical.
```

### gate-zero-armings-wording

**Zero armings spelled out** — attached to `if armed_total == 0:` (line 886)

```text
0/0 IS SPELLED OUT RATHER THAN PRINTED AS A RATIO. G3 looks for armings
that refused for want of an agreement their own robot already held, so
with no armings at all it has nothing to contradict and passes vacuously
— which is correct, because armings only happen when the link drops, and a
cell whose team never separated is a legitimate rendezvous cell that never
reached the mechanism. But "0/0 arming(s) carried the agreed cell" reads
like a measurement, and the one thing a reader must not conclude from it
is that the appointment machinery was exercised and found sound.
```
