# planner_util.hpp — design notes and history

The long comments of `include/explo_planner/planner_util.hpp`, moved out of the code on 2026-09-23 so the source carries short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

Sections follow the order of the source file and are grouped by the function (or section) they sit in. Each gives the line of code the comment was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [navBudgetSec](#navbudgetsec) — 1
- [shouldRendezvous](#shouldrendezvous) — 1
- [Mesh reconnection (robot-carried radios; see explo_planner_node.cpp)](#mesh-reconnection-robot-carried-radios-see-explo_planner_nodecpp) — 1
- [nextAgreedOccurrence](#nextagreedoccurrence) — 1
- [arrivalShortfallSec](#arrivalshortfallsec) — 1
- [dwellConfirmed](#dwellconfirmed) — 1
- [dwellHeld](#dwellheld) — 1
- [reconnectModeFromString](#reconnectmodefromstring) — 1
- [reconnectModeName](#reconnectmodename) — 1
- [pursuitBudgetSec](#pursuitbudgetsec) — 1
- [allocPeerPositionFresh](#allocpeerpositionfresh) — 1
- [meetingPoint](#meetingpoint) — 1

## navBudgetSec

### util-nav-budget-guarantees

**Nav budget clamp guarantees** — attached to `double navBudgetSec(double dist_m, double speed_est_mps, double safety_factor,` (line 22)

```text
Distance-budgeted NAVIGATE timeout (seconds):
  budget = clamp(dist / max(speed_est, 1e-3) * safety, min_sec, max_sec)
so a short hop gets a small budget and a long hop a larger one, instead of
every goal sharing one fixed timeout.

Two properties of that clamp are guarantees, not incidental, because this is
a watchdog and a watchdog that does not fire is worse than no watchdog:
  - max_sec < min_sec: the CEILING wins, exactly as in pursuitBudgetSec
    below. The two parameters come from unrelated families and nothing
    orders them, and std::clamp with lo > hi is undefined behaviour.
  - a non-finite distance (or speed, or safety factor) yields max_sec, not
    NaN. A NaN budget makes every `elapsed > budget` test false, so the
    timeout never fires and the cell runs to max_steps looking merely slow.
The return value is therefore always finite and always <= max_sec.
```

## shouldRendezvous

### util-should-rendezvous

**Reconnect decision at exhaustion** — attached to `bool shouldRendezvous(bool reconnect_enabled, bool have_anchor,` (line 49)

```text
Decision at exploration exhaustion: run a reconnect manoeuvre (true), or
finish now (false). Returns true only when the feature is on, a comms
anchor has been recorded, and the team is NOT already complete (if it is,
we are synced and can finish straight away).

`reconnect_enabled` is the subsystem master switch, not the rendezvous arm:
which manoeuvre a true answer dispatches is reconnect_mode's business, and
the caller is dispatchReconnect. The name says "Rendezvous" for history
only, from before pursuit and hybrid existed.
```

## Mesh reconnection (robot-carried radios; see explo_planner_node.cpp)

### reconnect-mode-arms

**The reconnection arms defined** — attached to `enum class ReconnectMode { RENDEZVOUS, PURSUIT, HYBRID };` (line 70)

```text
Reconnection policy when a teammate goes out of comms. Rewritten
2026-09-17 (generation 19): the previous text described a RECURRING
SCHEDULE — "cell C, every P seconds" — which no longer exists. What each
mode is now, in one sentence each. These sentences are the experiment's
definition of its four arms, and any behaviour outside them is a defect, not
a refinement.

  (the OFF arm is reconnect_enabled=false, not a member of this enum)

  RENDEZVOUS: the team agrees ONE meeting PLACE AND TIME while still
    connected — a (cell, interval, t_meet) triple proposed by robot 0,
    echoed verbatim, committed only once EVERY robot is holding the
    identical triple. The triple is a recurring appointment: meet at `cell`
    at mission-elapsed `t_meet`, and every `interval` thereafter. A robot
    whose reconnect trigger fires drives to that cell to keep the FIRST
    agreed occurrence not already past when it armed — not a private
    countdown from the moment it noticed, and since generation 25 not
    now-plus-notice either (the per-robot notice forked the ts4 N=3 cell) —
    waits there until every robot is connected, holds a further
    rendezvous_settle_sec so the merged map propagates, agrees the NEXT
    place and time with the reassembled team, and resumes exploring. An
    agreed schedule, or nothing: with no committed triple this arm performs
    no manoeuvre at all. Requires rendezvous_schedule_enable (interlocked).
  PURSUIT: chase the peer's last declared goal (the trail head) on a
    budget; when the budget is spent, explore on the fallback allowance and
    only then hold in place and beacon. No agreed destination ever — that
    absence is the A/B against hybrid.
  HYBRID: exactly the union of the two above and NOTHING ELSE — chase while
    the next agreed occurrence is not yet due, keep the appointment once it
    is. It has no third behaviour of its own; if it acquires one, the
    factorial reading of the four-arm design stops holding.

    THE CHASE WINDOW IS NARROW AND THIS IS DELIBERATE, not an oversight to
    be widened. The mid-run trigger needs 90 s of continuous team-incomplete
    time (reconnect_midrun_silence_sec, info-gated down to 60 s), and the
    chase runs only until the next agreed occurrence is due — since
    generation 25 that is the bare lattice, no 100 s notice on top, so the
    window is [trigger, occurrence] and can be EMPTY when the break lands
    just before an occurrence. That is the arm's definition working ("do
    pursuit if rendezvous is not due yet"), not a defect. If a campaign
    measures zero reconnect_gate events fleet-wide, hybrid and rendezvous
    are the same arm in that run and the contrast must be REPORTED as null
    by construction rather than read as a null result.

    GENERATION 23 WIDENED THE UPPER EDGE OF THAT WINDOW AND THE ANALYSIS
    MUST NOT KEEP QUOTING "60-100 s". The chase closes at the first agreed
    occurrence not already past when the trigger fired: under generation
    25's bare floor that is somewhere in [trigger, trigger + interval),
    and on generations 23-24 the 100 s notice pushed it out to
    [100 s, 100 s + interval) after the trigger. The spread is not slack
    to be tuned out: it is the price of every robot departing for the SAME
    instant instead of each for its own, and the extra time is spent
    exploring, not waiting. It does mean chase opportunity moves with the
    generation — gen 23-24 strictly more than gen 20-22, gen 25 up to
    100 s less than gen 23-24 and possibly none — which is one of several
    reasons generations cannot be pooled.

WHAT WAS REMOVED, WHY IT CAME BACK, AND WHAT ACTUALLY FIXED IT. The interval
is a recurrence period and a separating robot keeps the next occurrence of a
standing meeting. That is generation 18's design, it was removed in
generation 19-20, and generation 23 restored it. Do not remove it a second
time without reading this paragraph, because the measurement that killed it
is real and the reason it no longer applies is NOT that the measurement was
wrong.

The removal's case: three inequalities had to hold at once — worst-case
drive <= period <= barrier wait cap, and arming spread <= period — and none
was enforceable. The worst-case in-ROI drive is ~283 s (100x100 m at
0.5 m/s, 340 s with the safety markup) against a 240 s cap. The N=3 smoke
armed at 16.1 / 52.5 / 67.4 s against a 30 s period and the three robots
selected three DIFFERENT occurrences off byte-identical committed integers.

What replaced it was a private per-robot countdown, and THAT IS NOT A
RENDEZVOUS. N robots each departing 100 s after its own notice, for a place
they agree on and an instant they never discussed, is N robots visiting the
same cell. The experiment's rendezvous arm is defined as an agreed place AND
time, so the countdown made the arm measure something the design does not
name.

What makes the schedule safe now is not tighter inequalities — the drive
bound is still ~283 s and the arming spread is still unbounded — but that
MISSING AN OCCURRENCE NO LONGER MISSES THE MEETING.
rendezvous_appointment_wait_sec is 0.0 on every campaign arm, which is an
unbounded barrier: a robot that selects an earlier occurrence stands at the
agreed cell until the rest arrive. The 16.1 / 52.5 / 67.4 s spread costs
waiting instead of costing the reunion. The
generation-18 failure was a NO-SHOW on a bounded wait, and the no-show was
the defect in that design, not the schedule.
```

## nextAgreedOccurrence

### util-next-agreed-occurrence

**Choosing the agreed occurrence** — attached to `long long nextAgreedOccurrence(long long t_meet_ms, long long interval_ms,` (line 160)

```text
The occurrence of an agreed recurring appointment that a robot arming now
should keep: the first `t_meet_ms + k*interval_ms` (k >= 0 integer) that is
at or after `not_before_sec`.

THIS IS THE ONE PIECE OF ARITHMETIC THAT MAKES THE RENDEZVOUS ARM A
RENDEZVOUS, which is why it lives here, as a pure function with executable
tests, rather than as a file-static helper in a 17000-line node that no test
can link against. Every robot passes the SAME agreed (t_meet_ms,
interval_ms) — authored once by the proposer, adopted verbatim, compared
field-by-field before the team commits — and its OWN `not_before_sec`. The
shared origin is what the generation-18 attempt lacked: it indexed the
recurrence from each robot's own arming instant, so identical integers
produced different meetings.

WHAT THE CALLER MAY PUT IN THE FLOOR, because this is where the arm has been
broken twice. Generations 19-24 passed `now + rendezvous_depart_delay_sec`:
a lead time added UNCONDITIONALLY, so robots that could all comfortably
attend the same occurrence still split across two of them, and the ts4 N=3
cell forked with two robots on one instant and the third a whole interval
past it. Generation 25 made it bare `now`. Generation 29 passes
`now + max(0, own_marked_up_drive - rendezvous_max_lateness_sec)`.

The distinction that makes the third safe and the first not is the CLAMP AT
ZERO, not the size of the term: a robot that can reach the nearest
occurrence contributes nothing and gets the bare floor, so a team that can
all attend cannot fork. Only a robot that genuinely cannot arrive in time
rolls, and then by exactly its shortfall. A future caller adding anything
here must preserve that property — an unconditional term, however small,
re-creates the generation-19 fork.

Contract, exactly:
  - `t_meet_ms` at or after the floor is returned unchanged, so the first
    agreed meeting is kept as agreed rather than rolled forward.
  - `interval_ms <= 0` means "one meeting, not a recurrence": the agreed
    instant is returned even when it has already passed. The caller sees a
    due appointment, departs immediately, and waits at the cell — which is
    the same behaviour as a missed occurrence and is safe for the same
    reason (the barrier is unbounded). Returning something in the future
    would be inventing an instant the team never agreed.
  - No clamp on the result. A far-future floor rolls forward as far as it
    takes; the caller's own wait caps bound the consequences.
All arithmetic is exact integer; nothing here rounds through a double except
the caller-supplied seconds floor.
```

## arrivalShortfallSec

### util-arrival-shortfall

**Arrival shortfall floor clamp** — attached to `double arrivalShortfallSec(long long lead_ms, double max_lateness_sec);` (line 206)

```text
How far above bare `now` a robot must floor its occurrence search to reach
an appointment no more than `max_lateness_sec` late, given `lead_ms` — its
own estimated drive, already marked up — in milliseconds.

THIS IS THE CLAMP nextAgreedOccurrence's floor contract is about, and it is
a separate function for one reason: nothing else in the package can be
tested for it. The node computes the lead from a live grid lookup that no
gtest target can link against, so if the clamp lived inline there, deleting
it would be a source-scan question rather than an executable one — and the
generation-19 fork it prevents took a smoke run and a censored campaign to
find the first time.

Contract, exactly:
  - `lead_ms < 0` means "no estimate" and returns 0.0. A robot that cannot
    price its own drive does not get to decide the team is unreachable; it
    keeps the nearest agreed occurrence.
  - A lead within budget returns 0.0 — EXACTLY zero, not a small positive
    number. This is what stops a team that can all attend from forking.
  - Otherwise it returns the shortfall, `lead - budget`, in seconds, so the
    floor rises by exactly what this robot is short and no more.
  - A non-finite or negative `max_lateness_sec` is treated as 0.0 rather
    than propagated: the caller passes it through llround() inside
    nextAgreedOccurrence, where a NaN is undefined behaviour, and a negative
    budget would demand every robot arrive EARLY by that much and fork a
    team that had no reason to. The node validates the parameter too; this
    holds for direct callers and for a future one that does not.
```

## dwellConfirmed

### util-dwell-confirmed

**The flicker dwell window** — attached to `bool dwellConfirmed(bool eligible, double now_sec, double confirm_sec,` (line 234)

```text
FLICKER DWELL: "has `eligible` held CONTINUOUSLY for `confirm_sec`?"

The node reads "the team is whole again" from a claim table with a 5 s TTL,
so ONE claim arriving inside one TTL is enough to make the team look complete
for a single 10 Hz tick. Acting on that tick credits a reunion to a range-edge
flicker that drained no map deltas. Every site that acts on the team coming
BACK therefore has to see it hold, and this is the one implementation of
"hold" in the package — three sites used to carry two copies and a third site
carried none.

The caller owns the two state words. ONE WINDOW PER QUESTION, not one per
call site, and the node's flicker-guarded sites divide on exactly that
line: the manoeuvre barrier asks about manoeuvre release eligibility and
owns its own pair; the appointment supersede, the rendezvous_spent_ release
and the appointment walker's barrier conversion ask the identical
teamSettled question and share a second pair. Two windows over one
predicate is how "the outage is over" acquires two answers, and one of
them then fires on evidence the other is still refusing.

WHERE SITES SHARE A WINDOW, EXACTLY ONE OF THEM MAY CALL THIS FUNCTION.
`dwellHeld` below exists for the others. A second caller advancing or
disarming the pair would make asking the question change its answer.

AND THE WRITER MUST BE THE SITE THAT RUNS UNCONDITIONALLY. This measures a
continuous run, so it is only as continuous as its own sampling — and an
un-ticked window does not decay, it FREEZES, holding `armed` true with an
arbitrarily old `since_sec`. Put the call behind a branch that is untaken for
minutes and the first sample after the gap sees `now - since` far past
`confirm_sec` and fires on ONE reading, which is the failure the dwell was
added to prevent, now wearing the guard's name. Tick it from the timer, read
it from the branch.

Contract:
  - `eligible` false disarms and returns false. The window restarts from
    scratch on the next eligible tick; there is no partial credit, because a
    predicate that keeps dropping is exactly the flicker being filtered.
  - `confirm_sec <= 0` means "no dwell" and returns true immediately, so a
    campaign can switch the guard off with a parameter and the arithmetic
    below never runs.
  - The FIRST eligible tick arms the window and returns FALSE. `confirm_sec`
    is a dwell, not a deadline: the site fires on the first tick at or after
    `since + confirm_sec`, never on the tick that started it.
  - Comparison is `>=`, so a confirm window shorter than the tick period
    still fires on the second eligible tick rather than never.

TIME IS PLAIN SECONDS, deliberately. Taking rclcpp::Time here would put this
out of reach of an executable test for exactly the reason the node itself is
(see nextAgreedOccurrence above), and it would reintroduce a real hazard:
subtracting a default-constructed rclcpp::Time from a node clock reading
THROWS on mismatched clock types. Doubles cannot do that.
```

## dwellHeld

### util-dwell-held

**The non-mutating dwell twin** — attached to `bool dwellHeld(bool eligible, double now_sec, double confirm_sec, bool armed,` (line 287)

```text
The non-mutating twin of dwellConfirmed, for a caller that needs another
site's answer without advancing that site's window.

It must stay a pure function of its arguments and it must stay in step with
dwellConfirmed's return expression — the two answering differently is
precisely the defect the split exists to remove, so a change to one is a
change to both. They are adjacent in the .cpp for that reason.
```

## reconnectModeFromString

### util-reconnect-mode-parse

**Parsing the reconnect_mode parameter** — attached to `ReconnectMode reconnectModeFromString(const std::string& s,` (line 297)

```text
Parse the `reconnect_mode` parameter, case-insensitively. Unknown strings
map to RENDEZVOUS; `known`, when non-null, receives whether the string
matched a mode. The fallback is kept here because this is a pure function
with tests pinning it, but NOTE that it is not a usable policy: the node
treats `known == false` as fatal, because silently retargeting a run at a
different arm of the same experiment is undetectable downstream.
```

## reconnectModeName

### util-reconnect-mode-name

**Mode names are an interface** — attached to `const char* reconnectModeName(ReconnectMode m);` (line 306)

```text
Stable, machine-readable mode name — the inverse of
reconnectModeFromString, and the arm label the experiment event log stamps
into every run. Treat it as an interface exactly like stateName(): these
strings are what an analysis groups runs by, so renaming one silently
re-buckets every past run. Round-trips through reconnectModeFromString.
```

## pursuitBudgetSec

### util-pursuit-budget

**Pursuit budget and staleness decay** — attached to `double pursuitBudgetSec(double trail_head_dist_m, double staleness_sec,` (line 313)

```text
Pursuit spend limit (seconds). 0 means "do not pursue" — the caller falls
straight through to its fallback. Non-zero budgets follow the navBudgetSec
shape (distance to the trail head at the conservative speed estimate,
clamped to [min_sec, max_sec]) scaled by the freshness of the last-contact
record: trust in the trail head decays linearly with `staleness_sec` and
hits zero at `staleness_max_sec` — by then the peer could be anywhere in
the plot and chasing the record is worse than the guaranteed fallback.
Note the clamp: at field-scale distances the raw distance term saturates
the ceiling (flatforest params put that at a ~12 m trail head), so in
practice the returned budget IS max_sec until staleness eats into it, and
it never lands in (0, min(min_sec, max_sec)).
  - max_sec <= 0 disables pursuit outright (always 0).
  - staleness_max_sec <= 0 disables the staleness gate (freshness = 1).
  - max_sec < min_sec: the ceiling wins (the result never exceeds
    max_sec — it is the bound a waiting teammate relies on; min_sec comes
    from the unrelated nav-timeout family and must not override it).
```

## allocPeerPositionFresh

### util-alloc-peer-position-ttl

**Peer position freshness for allocation** — attached to `bool allocPeerPositionFresh(double age_sec, double max_age_sec);` (line 334)

```text
May a peer's latched POSITION still hold cells in the allocation problem?

`age_sec` is what TeamModel::positionAgeSec() returned — seconds since the
pose we hold was measured, or NEGATIVE for "we hold no position". Both
arguments are on the mission clock.

Three rules, and the first two are the ones that make the default safe:
  - `max_age_sec <= 0` is UNBOUNDED and returns true for everything. This
    is the file-wide convention for "no limit" (see pursuitBudgetSec) and
    it is what makes `alloc_peer_pos_max_age_sec = 0` reproduce the
    pre-TTL planner bit-for-bit, so one binary can run both arms.
  - A negative `age_sec` is never fresh under a live bound. It means no
    position exists, and a robot with no position is dropped from the
    problem anyway — but returning true here would make "unknown" read as
    "recent" the moment a caller forgot the have-position test.
  - Otherwise fresh means `age_sec <= max_age_sec`, inclusive, so a TTL of
    exactly N seconds admits a pose measured N seconds ago.

NaN in either argument returns false (every comparison against NaN is
false, and the `<= 0` unbounded test fails too), which drops every peer
rather than admitting every peer. That is the safe direction for a
misconfiguration, and the node refuses a non-finite parameter at load
anyway so it should be unreachable from the campaign harness.
```

## meetingPoint

### util-meeting-point-retired

**Why meetingPoint is no longer used** — attached to `Eigen::Vector3f meetingPoint(const Eigen::Vector3f& self_at_contact,` (line 359)

```text
The midpoint of the last-contact pose pair (all three axes — on flat worlds
the z average is the shared ground height; the arrival test is xy-only
either way).

NO LONGER CALLED BY THE PLANNER as of 2026-09-16, and kept deliberately
rather than deleted: it is a pure function with a unit test, and it is the
reference implementation of a construction that three past campaign
generations drove to, so an analysis reconstructing what a banked run did
needs it to still exist and still mean the same thing. Do not reintroduce a
call to it without reading why it went.

Why it went. It was the HYBRID (and briefly the RENDEZVOUS) fallback
destination, justified on the grounds that each side computes it from its
OWN record so determinism substitutes for negotiation, exactly as in the
MinPos tiebreak. Two things were wrong with that. The construction is a
place with NO TIME — nothing tells the peer when to be there or how long to
wait — and an agreed time is precisely what the rendezvous arm exists to
test. And the determinism argument does not survive contact: the two
records agree only as closely as the two directions' last successful
receptions were simultaneous, intent traffic is state-gated, and a one-way
fly-by refreshes one side's record and not the other's. The `floor_won`
telemetry measured the result — both ends chose the midpoint in only 2 of 6
separated pairs. See planner_method.md and the removal notes in
dispatchReconnect().
```
