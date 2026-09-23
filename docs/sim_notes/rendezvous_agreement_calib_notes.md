# rendezvous_agreement_calib.py — design notes and history

The long comments of `sim/rendezvous_agreement_calib.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [cell](#cell) — 1
- [1. the gate's parser still matches the node's own format string](#1-the-gates-parser-still-matches-the-nodes-own-format-string) — 5
- [6b. G3's ordering field is missing -> UNRUN, not a silent pass](#6b-g3s-ordering-field-is-missing---unrun-not-a-silent-pass) — 1
- [10. THE UPGRADE: provisional first, final second, in ECHO ORDER](#10-the-upgrade-provisional-first-final-second-in-echo-order) — 1
- [11. converged on the PLACEHOLDER: passes, but says which mechanism](#11-converged-on-the-placeholder-passes-but-says-which-mechanism) — 1
- [13b. THE SAME THREE COMMITS, WITH A MEETING TO PAY FOR THE THIRD](#13b-the-same-three-commits-with-a-meeting-to-pay-for-the-third) — 1
- [14. THE RELAXATION MUST STILL CATCH A REAL SPLIT](#14-the-relaxation-must-still-catch-a-real-split) — 1
- [15. a binary that cannot report the flag makes the shape check sit out](#15-a-binary-that-cannot-report-the-flag-makes-the-shape-check-sit-out) — 1
- [16. A HEALED SPLIT IS STILL A SPLIT WHILE IT LASTS](#16-a-healed-split-is-still-a-split-while-it-lasts) — 1
- [17. ...AND A NARROW ONE IS NOT, BUT IS STILL REPORTED](#17-and-a-narrow-one-is-not-but-is-still-reported) — 1
- [18. A RENAMED MANIFEST KEY MUST NOT READ AS "NOTHING TO CHECK"](#18-a-renamed-manifest-key-must-not-read-as-nothing-to-check) — 1
- [19. A PARTIAL PARSE FAILURE IS UNRUN, NOT A FAIL ON THE NODE](#19-a-partial-parse-failure-is-unrun-not-a-fail-on-the-node) — 1
- [20. SAME CELL, DIFFERENT SCHEDULE INTEGERS: PASS WITH A NOTE](#20-same-cell-different-schedule-integers-pass-with-a-note) — 1
- [21. A BANKED GENERATION-20..22 CELL STILL PARSES](#21-a-banked-generation-2022-cell-still-parses) — 1
- [22. GEN 23: SAME CELL, DIFFERENT MEETING TIME, COST REPORTED](#22-gen-23-same-cell-different-meeting-time-cost-reported) — 1
- [23-28. G1b: THE ECHO TERMS, WHICH USED TO BE PARSED AND DROPPED](#23-28-g1b-the-echo-terms-which-used-to-be-parsed-and-dropped) — 2
- [29. 0/0 ARMINGS IS SPELLED OUT, NOT PRINTED AS A RATIO](#29-00-armings-is-spelled-out-not-printed-as-a-ratio) — 1

## cell

### calib-fixture-wording-choice

**How a fixture picks its wording** — attached to `if prov is None:` (line 141)

```text
WHICH OF THE THREE PRINTING FORMATS THIS FIXTURE IS ON, chosen
by what the caller left out rather than by a flag, so a case
cannot claim one generation and write another:

  prov  is None -> gen <= 19  "every Xs from t+Ys."
  tmeet is None -> gen 20-22  "interval Xs (provisional=P, echoes a/b)."
  otherwise     -> gen >= 23  "interval Xs from t+Ys (provisional=P, echoes a/b)."

The default is the CURRENT format on purpose. A calibrator
whose fixtures are all on a retired wording proves the gate can
read the bank and says nothing about the binary about to run.
```

## 1. the gate's parser still matches the node's own format string

### calib-literal-recovery

**Recovering the node's format literal** — attached to `i = -1` (line 184)

```text
Recover the concatenated string literal the node passes to RCLCPP_INFO,
starting at the phrase the gate greps for. Adjacent C string literals are
joined the way the compiler joins them.

COMMENT OCCURRENCES ARE SKIPPED. The node quotes this phrase in a comment
right above the call, to say that the text is load-bearing — and a comment
is a one-chunk "literal" that renders to the bare phrase and would fail
this case for the wrong reason.
```

### calib-provisional-asserted

**The provisional group is asserted** — attached to `if not m:` (line 234)

```text
THE PROVISIONAL GROUP IS ASSERTED, NOT MERELY TOLERATED. LINE has to
keep that group optional so pre-generation-17 logs still parse, which
means a node that stopped printing the flag would go on matching — the
gate would quietly lose its ability to tell a tour-informed cell from
a meet-at-the-centroid one and say nothing. Requiring the group HERE,
against the node's live format string, is what makes that loud.
```

### calib-conversions-substituted

**Every conversion must be substituted** — attached to `bad("the node's format string has a conversion this calibrator "` (line 245)

```text
EVERY CONVERSION MUST HAVE BEEN SUBSTITUTED. The replaces above
are literal, so a node that changes `%.0f` to `%d` would leave a
raw conversion in the rendered string, the regex could still match
the parts it cares about, and this case would PASS while testing a
message no node ever emits. Checked here rather than assumed,
because that is precisely how a calibrator stops calibrating.
```

### calib-newest-tail-not-oldest

**The newest tail must not match the oldest** — attached to `bad("the node's commit message parsed on the PRE-GENERATION-20 "` (line 260)

```text
THE GEN-23 TAIL MUST NOT MATCH THE GEN-19 ONE. Generation 23 put
the meeting time back on this line, and the gen-19 form carried the
same two integers — so if the node had restored the OLD phrasing
("every Xs from t+Ys") the line would parse on the FIRST
alternative, `interval`/`prov`/`echoes` would all come back None,
and the parse loop coalesces those, so nothing would look broken
while a gen-23 bank silently read as pre-gen-20 binaries.

This is why the node appends `from t+Ys` to the interval term
instead of reusing the old wording. Pin it: the tail the node is on
is the third alternative and no other.
```

### calib-meeting-time-asserted

**The meeting time is asserted** — attached to `bad("the node's commit message does not carry the agreed meeting "` (line 279)

```text
THE MEETING TIME IS ASSERTED, NOT MERELY TOLERATED, for the same
reason the provisional flag is below: the group is optional so that
banked gen-20..22 logs still parse, which means a node that stopped
printing the agreed instant would go on matching and the gate would
quietly lose the only evidence in the text logs that two robots
armed the same MEETING rather than merely the same cell. On gen 23
that instant is what armAppointment attends, so losing it is losing
the arm's defining property.
```

## 6b. G3's ordering field is missing -> UNRUN, not a silent pass

### calib-case6b-missing-t-wall

**Case 6b: missing t_wall_sec** — attached to `cases += 1` (line 382)

```text
THE REGRESSION THIS PINS. G3 read `float(d.get("t_wall_sec", 0.0))` until
2026-09-18, so a row without the field compared 0.0 > <epoch seconds>,
which is False for every row that will ever exist. Drop the field from the
envelope — one schema edit, one binary generation — and G3 stops finding
contradictions and PASSes forever, emitting output IDENTICAL to a
genuinely clean cell. Nothing downstream could tell the two apart.

The arming below is case 6's contradiction with t_wall_sec deleted and
nothing else changed, so the two cases differ by exactly the field under
test: case 6 proves the check fires when it can run, this proves it
refuses when it cannot. Under the old code THIS CASE PASSED — which is the
only reason it is worth its lines.
```

## 10. THE UPGRADE: provisional first, final second, in ECHO ORDER

### calib-case10-upgrade

**Case 10: the provisional-to-final upgrade** — attached to `cases += 1` (line 444)

```text
This is the case that made the gate wrong before the flag existed. The
proposer can derive its triple before the allocator has produced a tour;
the argmin then has exactly one candidate (the centroid of the frozen
snapshot) and the result is a placeholder, published provisional=1 and
replaceable exactly once. Robots adopt the replacement whenever the echo
reaches them, so their FIRST commits legitimately disagree — and keying
G2 on first commits, which is what this gate used to do, scores a
perfectly healthy run as a SPLIT FLEET and sends the cell back for a REDO
that will do the same thing again.
```

## 11. converged on the PLACEHOLDER: passes, but says which mechanism

### calib-case11-placeholder

**Case 11: converged on the placeholder** — attached to `cases += 1` (line 470)

```text
A cell where nobody ever upgraded is a valid rendezvous cell — the team
has one place and one time and will drive to it. It is ALSO a cell whose
meeting point is the centroid of the spawn snapshot rather than an argmin
over the allocator's tours, i.e. a different treatment from the one the
arm name implies. It must not FAIL, and it must not pass silently either,
because nothing else in the run's artifacts records the difference.
```

## 13b. THE SAME THREE COMMITS, WITH A MEETING TO PAY FOR THE THIRD

### calib-case13b-paid-third-commit

**Case 13b: a kept meeting pays** — attached to `cases += 1` (line 520)

```text
Generation 29's re-agreement: the fleet keeps its appointment, stands on
the cell while the maps merge, and agrees the next place and time before
it goes back out. That is a third commit on a healthy run, and the budget
above is the ONLY thing separating it from case 13 — same commit lines,
one extra release line in the log. Paired with 13 on purpose: a budget
that counted nothing would pass both and a budget of two would fail both,
and neither mistake is visible from one case alone.
```

## 14. THE RELAXATION MUST STILL CATCH A REAL SPLIT

### calib-case14-real-split

**Case 14: relaxation still catches splits** — attached to `cases += 1` (line 547)

```text
Case 10 relaxed G2 from "every commit agrees" to "every robot ends in the
same place". The thing that relaxation could plausibly hide is a fleet
that started together and ended apart — one robot upgraded, another never
heard the echo. It is the worst outcome the protocol has (two robots, two
different cells, each waiting for the other) and it must still FAIL.
```

## 15. a binary that cannot report the flag makes the shape check sit out

### calib-case15-flag-unreported

**Case 15: the flag is unreported** — attached to `cases += 1` (line 566)

```text
Written with prov=None, i.e. the pre-generation-17 line. Two commits with
no flag could be [P, R] or a downgrade and there is no way to tell; the
gate must not guess. It reports the convergence it CAN see and says the
flag was unreported, rather than treating a missing group as a zero.
```

## 16. A HEALED SPLIT IS STILL A SPLIT WHILE IT LASTS

### calib-case16-wide-healed-split

**Case 16: a wide healed split** — attached to `cases += 1` (line 585)

```text
The case G2 structurally cannot see. Both robots end on the SAME triple,
both shapes are the legal [P, R], so G1, G2 and the shape check are all
satisfied — and between the two adoptions of the winning triple one robot
was standing at cell 9 while the other drove to cell 64. 300 s of that is
more than a whole appointment interval, i.e. a robot could have waited out
an entire rendezvous at the wrong place, and the cell would have scored as
a clean treated cell. If this ever goes quiet, the completion-time bias it
guards lands in the two arms the campaign exists to compare.
```

## 17. ...AND A NARROW ONE IS NOT, BUT IS STILL REPORTED

### calib-case17-narrow-healed-split

**Case 17: a narrow healed split** — attached to `cases += 1` (line 608)

```text
The matched half of case 16, and the one that stops G2b being a gate that
fails every healthy upgrade. Same shape, same convergence, 120 s apart:
under the bound, so PASS. What it also pins is that `span` and `split` are
DIFFERENT numbers on the same cell — span 140.0 s (first commit of
anything to last commit of anything) against split 120.0 s (first to last
adoption of the winner). If a future edit collapses them into one, this
case is what notices; reading span as split overstates the harm on every
upgraded cell.
```

## 18. A RENAMED MANIFEST KEY MUST NOT READ AS "NOTHING TO CHECK"

### calib-case18-missing-schedule-key

**Case 18: a missing schedule key** — attached to `cases += 1` (line 631)

```text
`rendezvous_schedule` absent used to take the same early `return 0` as
`rendezvous_schedule=0`, i.e. an unscheduled arm — so renaming the key in
the launcher would have turned this gate into a silent pass on every cell
of a campaign, printing nothing, failing nothing. Absence is now UNRUN,
which the teardown scores SUSPECT. This case is the only thing linking the
launcher's spelling of the key to the gate that depends on it.
```

## 19. A PARTIAL PARSE FAILURE IS UNRUN, NOT A FAIL ON THE NODE

### calib-case19-partial-parse

**Case 19: a partial parse failure** — attached to `cases += 1` (line 653)

```text
The gate used to reach UNRUN only when NOTHING parsed. A partial break —
this file current for some robots and stale for others, which is exactly
what a wording change looks like while a campaign straddles two binaries —
fell through to G1, where the unparsed robots were reported as never having
committed and the cell FAILed as "SCHEDULED ARM WITH NO AGREEMENT". That is
the gate's own blind spot charged to the node, in the INVALID direction, on
a cell that may be perfectly healthy; three of them abort the campaign.

THE POINT OF THE CASE IS THE WORD "FAIL" NOT APPEARING. A gate that
misattributes its own breakage is worse than one that is merely broken,
because the FAIL text names a defect in the node and sends the next hour
to the wrong file.
```

## 20. SAME CELL, DIFFERENT SCHEDULE INTEGERS: PASS WITH A NOTE

### calib-case20-schedule-integers

**Case 20: same cell, different integers** — attached to `cases += 1` (line 682)

```text
The other half of the G2 relaxation, and the reason it is a relaxation
rather than a deletion. Two robots holding cell 56 drive to the same place
whatever `interval` and `t_meet` say — on gen 20-22 because nothing read
those fields, and on gen 23 because the appointment barrier is unbounded,
so the robot that picked the earlier occurrence waits at the cell for the
others. Failing the cell would spend a healthy run. But a propose/echo path
that had started dropping fields would look EXACTLY like this, so the
disagreement has to stay visible. PASS, with the numbers on the line.
```

## 21. A BANKED GENERATION-20..22 CELL STILL PARSES

### calib-case21-middle-wording-bank

**Case 21: banked middle-wording cells** — attached to `cases += 1` (line 704)

```text
The gate is re-run over banked campaigns, and the middle of the three
printing formats — the one with no meeting time at all — is the one every
cell between 2026-09-17 and generation 23 was written in. If the gen-23
tail had been added by REPLACING the middle alternative rather than sitting
beside it, every one of those cells would re-gate as UNRUN "the commit line
changed shape" and a whole bank would stop being checkable. tmeet=None
selects that tail in the fixture writer.
```

## 22. GEN 23: SAME CELL, DIFFERENT MEETING TIME, COST REPORTED

### calib-case22-meeting-time-spread

**Case 22: same cell, different meeting times** — attached to `cases += 1` (line 723)

```text
The failure the rendezvous arm is defined against is "N robots agreed on a
place and went at N different times". On gen 23 that does not cost the
meeting — the barrier is unbounded, so the early robot waits — but it does
cost exploration time, and a reader who sees only PASS learns nothing. The
spread has to be on the line, in seconds.
```

## 23-28. G1b: THE ECHO TERMS, WHICH USED TO BE PARSED AND DROPPED

### calib-g1b-echo-cases

**Cases 23-28: the echo terms** — attached to `cases += 1` (line 743)

```text
LINE has captured `echoes E/P` since generation 20 and nothing compared
them until 2026-09-18. On generation 23 the commit rule requires
E == P == fleet-1 at EVERY commit, the node says so in a comment beside the
log call, and the integers proving it are printed on the line. These six
cases are the difference between that being an invariant and it being a
decoration.
```

### calib-case28-echo-unexercised

**Case 28: invariant not exercised** — attached to `cases += 1` (line 828)

```text
28. A gen-20..22 bank has no gen-23 lines, so the invariant is not
exercised — and that is NOT the same PASS as case 27. E < P was legal on
those binaries (a final triple could commit on first-hand evidence), so
the check must sit out rather than fail them, and the PASS must say it sat
out or a whole re-gated bank reads as verified when nothing was verified.
```

## 29. 0/0 ARMINGS IS SPELLED OUT, NOT PRINTED AS A RATIO

### calib-case29-zero-armings

**Case 29: zero armings** — attached to `cases += 1` (line 846)

```text
G3 passes vacuously when nothing armed, which is correct: armings only
happen when the link drops. But "0/0 arming(s) carried the agreed cell"
reads like a measurement, and the one thing a reader must not take from it
is that the appointment machinery ran and was found sound.
```
