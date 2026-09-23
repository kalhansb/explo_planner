# comms_gates.py — design notes and history

The long comments of `sim/comms_gates.py`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [Module scope](#module-scope) — 3
- [gate_leakage](#gate_leakage) — 2
- [gate_qos](#gate_qos) — 4
- [gate_relay_set](#gate_relay_set) — 1
- [Gate 3 — reliable-queue overflow](#gate-3--reliable-queue-overflow) — 1
- [gate_overflow](#gate_overflow) — 1
- [gate_outage_occurred](#gate_outage_occurred) — 1
- [main](#main) — 5
- [watch_summary](#watch_summary) — 3

## Module scope

### gates-leakage-allowlist

**Why the recorder and rviz are allowed** — attached to `DEFAULT_ALLOW = ["rosbag", "rviz", "transform_listener", "_ros2cli"]` (line 45)

```text
Nodes permitted to subscribe to a cross-robot topic. The §5.1 merge-attribution
oracle REQUIRES bagging cross-robot topics, so the recorder is not a leak;
neither is a visualiser, which consumes but never feeds a planner. Anything
else holding a cross-robot subscription is routing real data around the
emulator.
```

### gates-gated-suffixes

**Which streams belong in GATED_SUFFIXES** — attached to `GATED_SUFFIXES = ["scovox_node/scovox_bin", "exploration/intents"]` (line 52)

```text
Topic suffixes that must cross the emulator when COMMS=1. A subscriber on the
pre-relay copy of one of these is the leak that silently disables the
experiment.

These two are unconditional: every COMMS=1 run has both streams. Streams that
only some runs carry go in via --gated-extra instead of being added here —
gate_relay_set FAILS on a relay that does not exist, so listing a topic
nothing publishes would make every run without that feature fail a gate for
behaving exactly as configured, which is the same "baseline arm always FAILs"
trap --expect-outage exists to avoid.
```

### gates-stats-wall-timeout

**Sizing the stats wall timeout** — attached to `STATS_WALL_TIMEOUT_S = 90` (line 142)

```text
Sim-time safety factor for CLI reads that wait on a sim-clock publisher.

/hmr_comms_sim/stats is published on a ROS timer at stats_period_s (10.0) with
use_sim_time, so its WALL period is 10 / RTF seconds. A fixed wall timeout
therefore turns real-time factor into a gate verdict: below RTF ~0.5 a 20 s
timeout expires before the next sample and a healthy run is reported invalid
every poll. The gate's job is to detect overflow, not slowness, so the wait is
sized for a slow box and the "could not read" case is reported as UNRUNNABLE
(exit 2) rather than as a failed gate.
```

## gate_leakage

### gates-leakage-shared-bus

**Leakage check on the shared buses** — attached to `for suffix in GATED_SUFFIXES:` (line 226)

```text
The shared buses, explicitly. These bypass the emulator by construction
rather than by accident: each is a single global topic, so nothing can sit
between two robots on it. Under COMMS=1 the planners must have moved off
them, and any remaining subscriber means at least one planner did not.

Driven off GATED_SUFFIXES rather than hard-coded, so a stream added by
--gated-extra gets this check too. Only the /exploration/* suffixes have a
shared-bus form to fall back to; the scovox stream is per-robot already.
```

### gates-shared-bus-unread

**Bus absent versus bus unreadable** — attached to `if err and "timed out" in err:` (line 240)

```text
"The bus is gone" and "the bus could not be read" are OPPOSITE
findings and this was `if not out: continue` — a silent skip that
produced no note, no unrunnable and no failure, so the only trace
of it was the ABSENCE of the "no live subscribers" note, and
nothing downstream reads an absence. A non-zero exit is the normal,
expected case under COMMS=1 (the topic genuinely does not exist
because every planner moved off the shared bus), so that stays
quiet-but-recorded as a note; a timeout is not, and it means this
half of the leakage gate did not run.
```

## gate_qos

### gates-qos-shared-listing

**gate_qos uses the caller's listing** — attached to `rx = [t for t in present if "/rx/" in t]` (line 336)

```text
The caller's listing, not a fresh one. It was `run(["ros2","topic","list"])`
here, which meant a timeout emptied `rx` and tripped the fail() below with
"the relay never formed" — a statement about the radio, made from a CLI
that did not run. main() now reads the graph once through topic_list() and
returns 2 (UNRUN) if that read fails, so by the time control is here the
listing is known to have succeeded and an empty `rx` is a real observation.
```

### gates-qos-retried-read

**Retried per-topic QoS reads** — attached to `out, err = "", None` (line 349)

```text
NOT a silent skip any more, and read through run_checked (2026-09-18).

This used to call `run()` and `continue` on a falsy return. `run()`
collapses a non-zero exit, a timeout and a stale ros2 daemon into the
same "", which is exactly the ambiguity its own docstring warns
callers not to swallow: the topic was dropped on the floor while the
summary below still counted it in `len(rx)` and called it compatible.
That is how this gate came to emit zero FAILs across 896 cells — not
because the QoS always matched, but because nothing forced it to have
looked.

Retried because a single miss is not evidence of anything. The CLI
competes with a live sim for the box, and a one-off timeout on a
loaded machine says nothing about QoS; only a topic that stays
unreadable is worth a verdict. Reasons are kept, not just the count,
so "timed out after 15s" (slow box) stays distinguishable from
"exit 1: ..." (genuinely absent) when this fires.
```

### gates-qos-no-publisher

**A relay topic with no publisher** — attached to `bad += 1` (line 378)

```text
This is the case the module docstring names first — "a QoS
mismatch makes a relayed stream vanish entirely, which reads as
'the link was always down'" — and it was being skipped. A topic
that exists with a subscriber and NO publisher is exactly a relay
the emulator never formed: the consumer is waiting on a name
nothing writes to, and every downstream metric records a
permanent outage that the radio model never produced.
```

### gates-qos-denominator

**Reporting the QoS denominator** — attached to `checked = len(rx) - len(unreadable)` (line 417)

```text
REPORT THE DENOMINATOR. An unread topic is UNRUNNABLE, not a failure.

The old summary said "{len(rx)} relay topic(s) QoS-compatible" regardless
of how many were actually inspected, so a run in which every single
`ros2 topic info -v` came back empty produced the same reassuring line as
a run in which every pair was genuinely checked. Those are opposite
findings. `checked` is the only number that makes the note mean anything.

UNRUNNABLE and not fail(), for the reason STATS_WALL_TIMEOUT_S is written
up top: "the gate's job is to detect overflow, not slowness, so ... the
'could not read' case is reported as UNRUNNABLE (exit 2) rather than as a
failed gate". The same holds here. A topic whose QoS would not load is not
evidence of a QoS defect — calling it FAIL would state something about the
cell that was never observed, and FAIL scores the run INVALID. UNRUN
scores it SUSPECT, which is the honest classification and still surfaces:
`qos` is not in gate_g8's REPORT_ONLY_GATES, so this cannot be ignored
downstream the way a report-only gate can.

Note the wholesale-stale-daemon case never reaches here — `ros2 topic
list` returning nothing empties `rx` and trips the fail() above — so this
path is specifically "the listing worked but an individual topic would not
load", which is narrower and genuinely anomalous. (2026-09-18)
```

## gate_relay_set

### gates-relay-set-shared-listing

**gate_relay_set uses the caller's listing** — attached to `missing = [t for t in expected_relays(robots) if t not in present]` (line 480)

```text
The caller's listing. This ran its own `run(["ros2","topic","list"])`, and
the failure mode was the loudest of the three: an unreadable listing gave
an empty set, so EVERY expected relay was reported absent and the cell was
scored INVALID with a message naming topics that were in fact present. It
also meant this gate and gate_qos could disagree about the graph, having
sampled it seconds apart.
```

## Gate 3 — reliable-queue overflow

### gates-required-link-keys

**Asserting the stats link schema** — attached to `REQUIRED_LINK_KEYS = ("drop_overflow", "drop_disconnected")` (line 518)

```text
The two counters everything downstream of the emulator's stats message is
decided from: drop_overflow is gate 3's whole subject, and drop_disconnected
is the ONLY evidence that the independent variable varied. Both are read with
`.get(name, 0)` — which is correct for arithmetic and catastrophic as a
schema assumption, because a renamed or dropped field reads as a clean zero on
every link of every run. Gate 3 would then pass forever and the outage gate
would answer "no outage" without ever having looked at an outage counter.

So the schema is asserted ONCE, here, against the same dict both gates read.
`checks-that-stopped-checking`, third occurrence in this file: t_wall_sec in
rendezvous_agreement.py and the leakage gate's `checked == 0` are the other two.
```

## gate_overflow

### gates-overflow-schema-first

**Schema check before the overflow loop** — attached to `problem = stats_schema_problem(s)` (line 590)

```text
THE MESSAGE ARRIVED; THAT IS NOT THE SAME AS IT BEING READABLE. Checked
before the loop rather than inside it, because a loop over an empty list
and a loop over links whose counter is absent both finish with tripped
still False and fall into the `no overflow` note below. Unrunnable is the
honest verdict for both: the gate did not decline to trip, it never had a
number to trip on. Returning None also withdraws the dict from the caller,
so the outage gate is not handed a message this one just rejected.
```

## gate_outage_occurred

### gates-outage-zero-polls

**Zero polls is not zero outages** — attached to `if polls <= 0:` (line 692)

```text
ZERO POLLS IS NOT ZERO OUTAGES. This gate's claim is about the LINK, and
the only support for it is a run of polls that each looked and saw
nothing. With polls == 0 nobody looked: the watcher was killed before its
first period elapsed, or every read timed out, and the FAIL text below
would read "NO link outage in 0 poll(s)" — a sentence that blames the
radio for the watcher's death and sends the operator to check separation
against tx_power_dbm, which is exactly the wrong place. FAIL and UNRUN
are both non-CLEAN, so nothing banks either way; what changes is that the
report names the real defect.
```

## main

### gates-gated-extra

**Streams only some runs carry** — attached to `ap.add_argument("--gated-extra", default="",` (line 732)

```text
Extra gated suffixes for streams only some runs carry. The M-TARE
TeamWorld exchange is the first: team_world_hz defaults to 0.0, so a
COMMS=1 run without it publishes nothing on that topic and its relay
correctly never forms. The caller that turned the stream ON is the only
one that knows it should be there, so it is the caller that says so.
```

### gates-expect-outage-control

**Why control runs pass expect-outage no** — attached to `ap.add_argument("--expect-outage", choices=["yes", "no"], default="yes",` (line 741)

```text
The outage gate is the one gate that is wrong for a deliberate control
run. Phase 1 of the plan runs the control THROUGH the emulator at high
tx_power_dbm — so the relay path (extra hop, delay_ms, rx QoS) is present
but the link never drops, which is the intended condition, not a defect.
Left on, every control run would end flagged as "a comms arm that never
differed from the control", which is exactly backwards, and a campaign
whose baseline arm always reports FAIL trains the reader to ignore the
gate on the arms where it matters. Off, the gate still runs and still
reports what it saw — it just does not make the run a failure, and an
outage under a control label is worth knowing about either way.
```

### gates-topic-list-unrun

**Unreadable topic list is UNRUN** — attached to `rep.unrunnable("setup",` (line 770)

```text
UNRUN, not FAIL. Nothing was observed about this cell's radio, so
nothing may be asserted about it; see topic_list's docstring for why
this used to be a FAIL and what that cost. The return stays 2 — it
always was 2 here, which is itself the tell that the author meant
"unrunnable" while writing fail().

WHAT FAIL->UNRUN ACTUALLY CHANGES DOWNSTREAM, traced rather than
assumed. No caller reads this number: run_explo_sim_rviz.sh tests
`GATE_RC = 0` and nothing else, so rc 1 and rc 2 are the same thing to
every consumer, and both die() under GATES_STRICT — which defaults to
$COMMS, i.e. 1 for every campaign cell. The die is not the end of the
cell though: `trap teardown EXIT` is already installed by then, so the
verdict block still runs and still banks a run_gates_verdict. That is
where the change lands. This report line used to make NFAIL non-zero
and bank INVALID; it now makes NUNRUN non-zero and banks SUSPECT.

The consequence, which is real and is the price: run_campaign.sh
auto-redoes an INVALID cell on resume and does NOT auto-redo a
SUSPECT one (REDO_SUSPECT is opt-in, deliberately — re-rolling
conditions the retained sample on whatever made the cell fail). So a
stale daemon that used to be retried for the wrong reason is now
skipped for the right one. It is skipped LOUDLY — named, counted into
n_susp and repeated in the campaign summary — and the abort happens at
bring-up, before any sim time is spent, so the cost of noticing late
is seconds rather than an hour. Taking the retry back would mean
asserting a finding about a radio nobody observed, which is the defect
this whole path was rewritten to stop.
```

### gates-usable-polls

**Polls versus usable polls** — attached to `usable_polls = 0` (line 827)

```text
POLLS AND USABLE POLLS ARE DIFFERENT NUMBERS, and the outage gate is
entitled to the second one. `polls` counts times round the loop, which is
a measure of how long the watcher lived; it says nothing about whether any
of those iterations obtained a readable stats message. A run where the
emulator never publishes increments `polls` on every period and then ends
with "no link outage in 240 poll(s), as expected" — 240 pieces of evidence
claimed from 240 timeouts. gate_overflow returns the dict only when the
message arrived AND passed stats_schema_problem, so a non-None return is
exactly "this poll could have seen an outage and did not".
```

### gates-watch-sigint

**Why SIGINT is registered explicitly** — attached to `signal.signal(signal.SIGINT, _raise_stop)` (line 838)

```text
SIGINT must be registered EXPLICITLY, and the reason is not obvious.

The harness starts this watcher as a background job of a non-interactive
shell, and POSIX says such a job inherits SIGINT (and SIGQUIT) as SIG_IGN.
CPython honours an inherited SIG_IGN: it installs its default
KeyboardInterrupt handler only when the disposition it inherits is not
already "ignore". So the `except KeyboardInterrupt` below was unreachable —
the process ignored SIGINT outright. Teardown then waited 60 s and
escalated to SIGKILL, which cannot be caught, so the summary was never
written. signal.signal() here overrides the inherited SIG_IGN, which is
the whole fix; verified by sending SIGINT to the process group before and
after (before: still alive, empty report / after: "stopped; polls=2" and
both summary lines present).

Measured cost of the bug: all 10 runs on disk logged "gateswatch ignored
SIGINT" and not one wrote a `watch` or `outage` line. The only run that
ever produced a run-time verdict was one killed with SIGTERM by hand.

What that cost: gate_outage_occurred is the check that the INDEPENDENT
VARIABLE ACTUALLY VARIED — that a COMMS arm's link really dropped. It has
therefore never adjudicated a real run, while runs carried
run_gates_verdict=CLEAN earned entirely by bring-up gates. Same lesson as
plan §3.14 for the third time: a validity gate that cannot fail the run is
a log message.
```

## watch_summary

### gates-trip-on-failure

**Trip on a failure, not a False** — attached to `before = len(summary.failures)` (line 903)

```text
TRIP ON A GATE THAT FAILED, NOT ON ONE THAT RETURNED False.
gate_outage_occurred returns "was an outage seen", which is False on
BOTH of its negative paths — the link genuinely never dropped, and
nobody ever looked. Reading that bool as "trip" collapses the
distinction the unrunnable path was added to draw: a watcher killed at
t=0 set tripped, and the `watch` line below then read `gates tripped
during the run (0 polls)`, blaming tripped gates for a watcher that
never ran and making the cell INVALID where it should be SUSPECT.
The failure list is the gate's own statement of which it meant.
```

### gates-control-zero-polls

**Control arm with no usable polls** — attached to `summary.unrunnable("outage",` (line 925)

```text
The same "nobody looked" condition as the yes-branch, and it has to be
said on this branch too. A control arm is the one place where "no
outage" is the expected reading, which makes it the one place where an
unreadable run looks most like a healthy one — and the note this
replaces asserted the expectation had been met.
```

### gates-watch-line-teardown

**The watch line the teardown reads** — attached to `summary.unrunnable("watch",` (line 942)

```text
THE `watch` LINE IS LOAD-BEARING IN THE TEARDOWN, which treats a
comms_gates.txt with no `\twatch\t` line in it as SUSPECT precisely so
that a watcher which died silently cannot bank a clean cell. Writing
the note below on polls == 0 satisfied that check with a sentence
meaning the opposite of what it appeared to mean: "0 polls, no gate
tripped" reads as an all-clear and is produced by a watcher that never
completed one period. UNRUN still carries the `watch` token, so the
teardown finds its line — and then counts an UNRUN and lands SUSPECT,
which is the verdict a run with no run-time supervision has earned.
```
