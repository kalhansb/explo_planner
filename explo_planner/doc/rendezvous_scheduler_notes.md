# rendezvous_scheduler.hpp, rendezvous_scheduler.cpp — design notes and history

The long comments of rendezvous_scheduler.hpp, rendezvous_scheduler.cpp, moved out of the code on 2026-09-23 so the sources carry short comments only. Where a comment was moved, the code keeps a short gist ending in `(notes: <id>)`; the section headed `<id>` below holds the original comment, word for word.

One section per source file. Each entry names the function (or section) the comment sat in, the line of code it was attached to and its original line number. Line numbers, dates, generation numbers and cross-references inside the moved text are as they were when written; they record history and are not maintained.

## Contents

- [include/explo_planner/rendezvous_scheduler.hpp](#includeexplo_plannerrendezvous_schedulerhpp) — 18
- [src/rendezvous_scheduler.cpp](#srcrendezvous_schedulercpp) — 13

## include/explo_planner/rendezvous_scheduler.hpp

### rdv-plan-auditable

**Appointment carries its derivation** — in `RendezvousPlan — declarations`, attached to `struct RendezvousPlan {` (line 108)

```text
An appointment: a cell to meet in and a mission-elapsed time to be there.

Everything that produced it is carried alongside, for the same reason the
reconnection gate carries its arithmetic: an appointment nobody can
re-derive offline is one nobody can audit, and the whole design rests on
two robots computing the same numbers from different processes.
```

### rdv-interval-ms

**interval_ms is the recurrence period** — in `RendezvousPlan — declarations`, attached to `long long interval_ms = -1;` (line 127)

```text
The interval the solve landed on. ON A PLAN `RendezvousScheduler::solve`
RETURNED it is `t_meet_ms - mission_elapsed_ms` by construction — there is
one assignment site and it writes the two together. This is the integer
that goes on the wire and the one every consumer means by "the interval".

THAT IDENTITY DOES NOT HOLD ON THE NODE'S `appointment_`, which is this
struct reused as a carrier rather than a solve result (see `capped` below
for the same caveat on the derived fields). Arming copies `interval_ms`
off the wire and overwrites `t_meet_ms` with the first agreed occurrence
not already past at arming (nextAgreedOccurrence — a local countdown
until generation 23, now-plus-notice until generation 25) — so the
difference there depends on when this robot armed and has nothing to do
with this field. Anything that recovers one of the pair from the other
is reading a solve-time invariant on a struct that is no longer a
solve's output.

IT IS THE RECURRENCE PERIOD: `t_meet_ms` is the first occurrence of a
meeting repeating every `interval_ms`, and a separating robot keeps the
next occurrence at or after its own departure floor. That was true, was
briefly not (generation 19 replaced the timetable with a local countdown),
and is true again since generation 23 — see nextAgreedOccurrence in
planner_util.hpp, which is the only reader.

The integer is derived, goes on the wire, has to match byte for byte
across the fleet, and is logged — it is the agreement evidence the
campaign gate scores, and the reachability floor that sets it is the
honest statement of "how long the furthest robot needs". That floor is
what keeps the arrival stagger inside one occurrence, so a change to it
is a change to the meeting, not only to the evidence.
```

### rdv-tour-interval

**The raw tour interval** — in `RendezvousPlan — declarations`, attached to `long long tour_interval_ms = -1;` (line 158)

```text
The raw tour term — max over robots of "time to reach `cell` along its
own tour" — before the lead floor raised it or the divergence cap pulled
it in. Kept separate because it is the only one of the three that answers
"was this meeting nearly free?", and because the slowest robot's lateness
under a cap is `tour_interval_ms - interval_ms`.
```

### rdv-floored-flag

**When the interval is floored** — in `RendezvousPlan — declarations`, attached to `bool floored = false;` (line 165)

```text
The floor raised the interval above what the objective AND the cap between
them asked for: the tours put the occurrences closer together than the
direct drive or `min_interval_ms` allow, so the period is set by
reachability rather than by the objective.

AGAINST `min(tour_interval_ms, max_interval_ms)`, NOT AGAINST THE TOUR
TERM ALONE (2026-09-18). Measuring it against the tour term alone reads
false whenever the cap had already pulled the ask below the floor, which is
the one configuration where the floor is doing something the caller needs
to know about.

EXPECT THIS TO BE TRUE ON MOST ROWS, and do not read it as a defect. The
objective deliberately finds the cheapest space-time near-miss, so the
tour term is small by design — it was 28.284 s at N=2 in the ts4 smoke,
one diagonal grid step. A campaign where this is always FALSE is the
surprising one.
```

### rdv-capped-flag

**When the interval is capped** — in `RendezvousPlan — declarations`, attached to `bool capped = false;` (line 183)

```text
True when the cap pulled the period in below what the tours imply. The far
robot is then structurally late for each occurrence by `tour_interval_ms -
interval_ms`; the barrier's wait absorbs it, which is the whole reason the
cap is set from that wait. Reported rather than hidden because a run where
this is always true is one where the cap, not the objective, is choosing
the meeting time.

A LIVE COLUMN AGAIN (2026-09-17). It was false on every row the node
produced while `max_interval_ms` carried the old divergence cap, which was
fed from a last-contact quantity that does not exist at proposal time.
The field now carries the findability bound instead and the node passes
`reconnect_midrun_max_wait_sec`, so the cap really can bind.

IT CANNOT PUSH THE PERIOD BELOW THE REACHABILITY FLOOR — a period nobody
can drive in is not made findable by being short — so when the cap asks
for less than the floor, the period stops at the floor. This still reads
true in that case, because the cap did pull the period in from the tours;
it just did not get all it asked for.

THE ONE SHAPE WHERE THE FINDABILITY INEQUALITY IS BROKEN is BOTH FLAGS
TRUE, which is the same statement as `interval_ms > max_interval_ms`: the
cap cut, the floor refused the cut, and the team is further apart than the
barrier is willing to wait — so a robot that misses an occurrence is not
guaranteed to find anyone at the next one. The scheduler reports it rather
than pretending otherwise, and the node warns on it (the warn keys on the
inequality directly, so it holds whatever these two flags say).

The two are NOT mutually exclusive and never were. A revision on
2026-09-18 briefly made them so by deriving both from the returned value,
which collapsed this shape to "neither bound bound"; see the derivation in
rendezvous_scheduler.cpp.
```

### rdv-floor-won

**floor_won is an identity** — in `RendezvousPlan — declarations`, attached to `bool floor_won = false;` (line 216)

```text
The committed cell IS the caller's `floor_cell`. NOT the midpoint: that
floor is gone, and the one the node passes today is the centroid of the
team's own vehicle cells (see `floor_cell` on `solve`). Historical
`floor_won` rates quoted in planner_util.hpp and in the node's removal
notes were measured against the MIDPOINT and do not carry over.

READ IT AS AN IDENTITY, NOT AS A VERDICT (2026-09-18). This used to say
"no tour cell was worth its detour, so the team agreed to meet at the
floor instead of anywhere a tour goes", and that reading is not sound. The
candidate scan SKIPS the floor cell and re-adds it unconditionally, so a
cell that is both on a tour and the floor appears exactly once, as the
floor — and if it then wins the argmin at penalty 0 it wins as the cell it
always was, with the floor mechanism contributing nothing. The same solve
reports floor_won=0 with floor_cell=-1 and floor_won=1 with floor_cell set
to the cell it was going to pick anyway, which is enough to inflate any
rate computed from this column. `penalty_mm` is the quantity that says
whether the choice cost anything; use the two together.
```

### rdv-speed-mm-s

**Integer drive speed** — in `Config — declarations`, attached to `long long speed_mm_s = 500;` (line 255)

```text
Conservative drive speed, in mm/s. Integer so that travel times
quantise exactly and two robots converting the same distance land on
the same millisecond. A float speed would reintroduce, at the last
division, precisely the cross-process disagreement the allocator spent
its whole design removing.

Non-positive disables the scheduler outright (every solve refuses),
because a zero speed makes every arrival time infinite and there is no
safe value to substitute.
```

### rdv-depart-safety-milli

**Shared travel-time safety multiplier** — in `Config — declarations`, attached to `int depart_safety_milli = 1200;` (line 266)

```text
Safety multiplier on travel time, in thousandths. 1200 = allow 1.2x the
estimated drive. It exists because the estimate is a straight-line-ish
graph distance at a nominal speed and the real drive is neither.

TWO READERS, AND THEY HAVE TO BE THE SAME CONSTANT: the reachability
floor in solve(), which sizes the lattice spacing to cover the furthest
robot's marked-up drive, and ExploPlannerNode::appointmentLeadMs, which
is how much lead a robot actually takes. Pricing the lead higher than
the floor would mean a robot needing more notice than the spacing the
team sized for it — the one shortfall a rung roll cannot fix, because
the next rung is only one spacing away.

The name is the generation-19 departure test's and that test is gone
(see the class comment); it is kept because the quantity is the same one
and renaming it would break every banked params row for no gain.
```

### rdv-min-interval

**Minimum interval floor** — in `Config — declarations`, attached to `long long min_interval_ms = 0;` (line 283)

```text
Lower bound on the interval, in ms. <= 0 means no floor.

IT STOPS THE INTERVAL DEGENERATING. The objective below minimises the
detour, so its winner is the nearest space-time near-miss and its raw
interval can be a single grid step — 28.284 s at N=2 in the ts4 smoke,
and nothing prevents a candidate the whole team is already standing on
from producing nearly zero.

WHY THAT STILL MATTERS WITH THE SCHEDULE GONE (generation 19). It used
to matter because the interval was the recurrence period and a period of
a few seconds is not a schedule. Nothing recurs now, so the argument is
narrower and worth stating rather than inheriting: `interval_ms` is
exchanged, compared for exact equality and scored, and it is read as
"how long the furthest robot needs". A near-zero value is a false
statement of that, and the floor is what keeps the logged integer
meaning what every consumer takes it to mean.

The node feeds this from `rendezvous_interval_sec` (300 s), which exists
for exactly this and nothing else. It used to come from
`rendezvous_proposal_period_sec` "for no deeper reason than that it is
the coarsest cadence the protocol already has", and that accident cost
something real: the proposal period also bounds how stale the snapshot
the punctuality estimate is costed against may be, so the two could not
be raised together. They are now two numbers because they bound two
things.
```

### rdv-max-interval-findability

**Maximum interval as findability bound** — in `Config — declarations`, attached to `long long max_interval_ms = 0;` (line 310)

```text
Upper bound on the interval, in ms. <= 0 means uncapped.

REPURPOSED (2026-09-17), and the new reason is much stronger than the
old one. It used to carry a map-divergence model — meet sooner because
the maps are separating fast — which the node never fed (it passed 0)
because the model's inputs do not exist at proposal time and do not
survive N > 2.

It is now the FINDABILITY BOUND. The arithmetic has not changed since,
but WHAT IT BOUNDS HAS (generation 19), and the two readings are easy to
confuse because the inequality looks identical.

    v6, the schedule:  period <= wait cap. A robot arming too close to
    occurrence k arrives late by less than one period; its peers, having
    arrived at k, are still standing there, so a missed occurrence is
    harmless.

    v8, the schedule again: furthest drive <= wait cap. After the
    reachability floor in solve() the interval IS the furthest robot's
    direct drive, so the same comparison now asks whether the last robot
    to set off can still arrive before the ones already waiting give up
    — which, the floor being what it is, is also the v6 reading.

THE NODE PASSES `rendezvous_appointment_wait_sec` (2026-09-19), the wait
a robot standing at the AGREED CELL actually spends. It passed
`reconnect_midrun_max_wait_sec` until generation 29 on the argument that
the two barriers should not drift apart in a yaml — but they are not the
same barrier, and the robots this bound is about are the ones already
standing at the cell, whom doReturnSync holds on the appointment wait.
The mid-run wait belongs to a PURSUIT reunion, and no pursuit consults
this schedule at all.

SO IT IS 0 = UNCAPPED ON THE CAMPAIGN DEFAULT, `capped` false on every
row, and that is the honest answer rather than a dead column: a barrier
nobody walks away from cannot be outrun by any interval, so there is no
findability bound left to state. The node says so once at startup so the
constant is legible as configuration rather than as a bug.

THE FLOOR OUTRANKS THIS CAP in solve(), so the bound is not guaranteed;
when it fails, the node warns once with the numbers (see
deriveRendezvousProposal). Cutting below the drive would shrink the
integer without shortening the journey.
```

### rdv-exclude-retired

**The retired exclusion list** — in `Config — declarations`, attached to `std::vector<int> exclude;` (line 354)

```text
Cells to drop from consideration: a no-show write-off list.

THE NODE NO LONGER USES THIS. It hard-clears the vector before every
solve (explo_planner_node.cpp, deriveRendezvousProposal) and the
mechanism is retired, so `RendezvousPlan::rejected_excluded` is 0 on
every row a campaign will produce. Kept only because the unit tests
exercise it and because deleting a Config field is a wire-compatibility
event for no gain.

It was retired for a reason that outlives it: the list was PER ROBOT,
and a per-robot input to a value the whole team must share is the exact
mistake gen 9 made everywhere else. Nor can it return as a shared input
— the team would first have to agree on the write-offs, which is the
same agreement problem one level down. Anti-deadlock is now the wait cap
and the one-appointment-per-outage rule instead, both of which are
local, terminating, and need nobody's consent.

When it is non-empty it NEVER applies to the floor (the loop skips
`floor_cell` and re-adds it unconditionally afterwards). That is not a
property of what the floor happens to be — it is the point of having
one: if exclusion could empty the candidate set, the no-show handler
would have reinvented the deadlock it existed to prevent. The floor the
node supplies is the team's own vehicle centroid, which is reachable by
construction for whoever is nearest it, but this guarantee holds for any
floor the caller passes.
```

### rdv-solve-inputs

**Scheduler solve inputs** — in `RendezvousScheduler — declarations`, attached to `static RendezvousPlan solve(const CellWorld& world,` (line 382)

```text
Derive the appointment.

`robots` is the vehicle set exactly as it was handed to
GlobalAllocator::solve, and `alloc` is that solve's result — index-aligned
with `robots`, which is what lets this function attribute a tour to a
starting cell without searching by id. Handing in an allocation solved
from a DIFFERENT vehicle set is the one misuse that cannot be detected
here and will silently produce a plan neither robot can keep.

`floor_cell` is the caller's GUARANTEED CANDIDATE: a grid cell admitted
unconditionally, exempt from `Config::exclude` and from the reachability
rule, so that a solve cannot refuse merely because the tours were empty.
Pass -1 to supply none — and understand that -1 means the candidate set
CAN be empty and `solve` can then refuse with nothing to fall back on.
That is not hypothetical: it cost the first ts4 smoke its whole N=4 rung.

It is NOT the last-contact midpoint. That quantity was removed on
2026-09-16 and does not even exist at proposal time — the proposal is
derived while the team is still connected, so there is no last contact to
take a midpoint of. What the node passes today is the centroid of the
snapshot's own vehicle cells: defined for any N, drawn from the frozen
shared snapshot rather than anything private, and in-ROI by convexity.
There is no "raw midpoint" for a caller to keep using.

`mission_elapsed_ms` is the shared clock. It is the ONLY time input, and
it is a parameter rather than a call to now() so this stays a pure
function of values both robots hold.
```

### rdv-departure-rule-history

**Departure rule and schedule history** — in `RendezvousScheduler — declarations`, attached to `};` (line 421)

```text
THE DEPARTURE RULE IS GONE (generation 19, 2026-09-17); THE SCHEDULE CAME
BACK (generation 23).

`occurrenceAtOrAfter` turned one frozen (phase, period) pair into a
standing meeting, and `shouldDepart` made each robot leave early enough
that a staggered departure produced a synchronised arrival. Both were
deleted here. The recurrence came back as nextAgreedOccurrence in
planner_util.hpp, a pure function with executable tests, because N robots
departing on private countdowns for an instant they never discussed is not
a rendezvous.

THE LEAVE-EARLY RULE CAME BACK TOO (generation 29, 2026-09-19), in the node
rather than here: ExploPlannerNode::appointmentDue departs when
`now + appointmentLeadMs >= t_meet_ms`, degrading to the bare comparison
when the robot is off the snapshot grid and cannot price its own drive. It
is NOT a restoration of shouldDepart's arithmetic — it reads a live lead
each tick against an instant the team agreed, where shouldDepart aimed at a
deadline each robot had computed privately.

WHAT THE SCHEDULE LOOKS LIKE NOW. A robot whose reconnect trigger fires
signs up to the first agreed occurrence it can still ARRIVE at within
rendezvous_max_lateness_sec, leaves in time to be there, and waits until
the team is whole. The residual spread is still absorbed by an unbounded
barrier wait rather than by an inequality between a period and a drive —
but it is now the spread of robots that could not make the rung at all,
not the spread of everyone's drive distance.

WHY IT WAS REMOVED, IN ONE MEASUREMENT — and see nextAgreedOccurrence in
planner_util.hpp for why the measurement is still real and no longer
decides the question. The timetable had to satisfy three inequalities at
once — worst-case drive <= period <= barrier wait cap, AND arming spread <=
period — and no period satisfies all three: the worst-case in-ROI drive is
~283 s (100x100 m at 0.5 m/s), 340 s with the safety markup, against a
240 s cap. The N=3 smoke showed the failure directly: three robots armed
16.1 / 52.5 / 67.4 s apart against a 30 s period, so they selected three
DIFFERENT occurrences (t+34 / t+64 / t+94) off byte-identical integers.
Two of them met; the third arrived to an empty cell.

`depart_safety_milli` survives this deletion because it is also the
multiplier on the reachability floor in solve() — and, since generation 29,
on the node's departure lead, deliberately the same constant so a robot
never needs more lead than the spacing the floor sized for it.
`depart_margin_ms` did not survive, and is gone with shouldDepart, its only
reader.
```

### rdv-handshake-decisions

**The handshake as pure decisions** — in `RendezvousHandshake — declarations`, attached to `struct RendezvousHandshake {` (line 471)

```text
Deciding WHERE to meet is the scheduler's job and it is a search. Deciding
that the whole team is holding the same answer is a different job, it is a
state machine, and until 2026-09-17 it lived entirely inside a 14k-line ROS
node where nothing could reach it. That is not an aesthetic complaint: the
two defects that killed the N>=3 rendezvous arm were both in these few lines,
both survived three code reviews, and neither was reproducible in under ten
minutes of wall clock because the only way to run the code was to run a cell.

What is here is ONLY the decisions — no ROS, no clock, no I/O, no proposal
type of its own. The node keeps its control flow, its logging and its own
nested RendezvousProposal; it delegates the two questions that turned out to
be hard. A test can then enumerate the transitions exhaustively, which is
what `RendezvousHandshake` in test_rendezvous_scheduler.cpp does.
```

### rdv-adopt-sequence

**Permitted proposal adoption sequence** — in `RendezvousHandshake — declarations`, attached to `enum class Adopt {` (line 485)

```text
What a non-proposer should do with the triple the proposer is currently
publishing.

THE PERMITTED SEQUENCE IS empty -> P -> R -> R' -> R'' ..., where P is the
centroid placeholder (`provisional`) and each R is a tour-informed choice.
The first two steps are free-standing; every step after them must be
EARNED, and what earns one is the team having actually kept the meeting R
names (generation 29 — the user's rule is that the next place and time are
agreed before anyone resumes exploring). A robot signals that it is owed a
replacement with `held_reagree_due`, and outside that window a second
generation is still the protocol violation it has always been.

So the count of adoptions in a run is no longer two: it is two plus one
per meeting kept. What remains invariant — and is the property the commit
gate rests on — is that a robot only ever moves off R when it is expecting
to, so the fleet cannot be walked onto a new generation mid-outage.
```

### rdv-adopt-flags

**Adopt takes flags, not triples** — in `RendezvousHandshake — declarations`, attached to `static Adopt adopt(bool peer_valid, bool peer_provisional,` (line 517)

```text
The flags are passed rather than the triples so that this cannot acquire
an opinion about what a triple IS. Note in particular that
`peer_equals_held` is deliberately independent of the provisional flags:
the protocol's commit comparison is over the three integers alone, and a
robot that cleared its own flag without the numbers changing has not
changed what it will drive to.
```

### rdv-peer-latch

**Clearing a contradicted peer record** — in `RendezvousHandshake — declarations`, attached to `template <class Triple>` (line 531)

```text
Update what peer `id` has been HEARD to hold, given its newest message.

The latch exists because at N>=3 the echoes do not coincide — requiring
them to was a deadlock (see maintainRendezvousProposal) — so each is
recorded when it arrives and the commit counts records. The cost of that
is that a record can be contradicted later, and THIS FUNCTION IS WHERE THE
CONTRADICTION IS ACTED ON: a message that disagrees with the record clears
it, on the tick it arrives, from the peer's own words.

Dropping only on heard-nothing, which is what the node did until
2026-09-17, is a FALSE-COMMIT bug and not a missed-commit one: a peer that
upgrades P -> R leaves its old P record standing, the holder of P counts it
toward fleet-1, announces an agreement the peer does not share, and drives
to a cell nobody else is coming to. Committing something false is strictly
worse than committing nothing.

@param heard   the peer's newest published triple (may be invalid)
@param held    what THIS robot holds; a record only counts if it matches
@param latched in/out, the record for that peer
```

### rdv-peer-latch-order

**Latch clear-then-record order** — in `Triple — declarations`, attached to `if (!(heard == latched)) latched = Triple{};` (line 553)

```text
Order matters. Clear on contradiction FIRST, then re-record, so that a
peer moving straight from one triple to another in a single message both
loses the old record and gains the new one without a tick in between —
and so that a peer moving to something that is NOT ours ends with no
record at all rather than keeping the convenient one.
```

## src/rendezvous_scheduler.cpp

### rzv-vehicle-filter-duplicated

**The duplicated vehicle filter** — in `vehicles`, attached to `std::vector<Veh> vehicles(const CellWorld& world,` (line 18)

```text
The same vehicle filter GlobalAllocator::solve applies, deliberately
duplicated rather than shared: the filter is part of the ALLOCATION
problem's definition, and this file's contract is "the same set the
allocation was solved over". If the allocator's filter ever changes, this
one has to change with it, and a compile-time coupling would hide that by
silently agreeing. The tests pin the agreement instead.
```

### rzv-detour-best-insertion

**Best insertion of a meeting cell** — in `detour`, attached to `Detour detour(const CellWorld& w, int start, const std::vector<int>& tour,` (line 46)

```text
Best insertion of `c` into `tour` driven from `start`.

"Best" = smallest resulting ROUTE COST, matching §3.5's definition of the
penalty; ties take the lowest position under a strict `<`, so the scan order
is the tie-break and there is nothing else to agree on.

A cell already on the tour is not re-inserted: the robot is going there
anyway, the detour is zero, and its arrival is simply the prefix to the
first occurrence. That asymmetry is the objective's whole point — the
cheapest meeting is one somebody was already driving to.
```

### rzv-base-makespan-recomputed

**Recomputing the base makespan** — in `RendezvousScheduler::solve`, attached to `std::vector<long long> base(veh.size(), 0);` (line 127)

```text
Recomputed from the tours rather than read out of alloc.costs_mm. The two
agree (a test pins it), but recomputing keeps this function well-defined
for a hand-built Allocation and, more importantly, makes the penalty a
difference of two values produced HERE — so it cannot be quietly turned
into a difference between a polished cost and an unpolished one.
```

### rzv-admissibility

**What makes a meeting cell admissible** — in `RendezvousScheduler::solve`, attached to `std::vector<int> admissible;` (line 154)

```text
Admissibility. A candidate every robot can reach on the roadmap, and that
the caller has not written off. Note what is NOT checked: presence in each
peer's cell world. Cell ids are geometry and the fleet's grid config hash
is verified on the wire, so a cell that exists here exists everywhere —
there is no analogue of mTARE's missing-cell case, which it handles by
calling exit(1) (§3.5.1, edge 4).
```

### rzv-unreachable-sentinel

**Testing reachability without the fallback** — in `RendezvousScheduler::solve`, attached to `if (!(world.distance(v.cell, c) >= 0.0)) { reachable = false; break; }` (line 171)

```text
Negative is the graph's unreachable sentinel. Tested directly rather
than through costMm, which deliberately FALLS BACK to centroid
distance: that fallback is right for ranking a cell somebody will
eventually clear, and wrong for promising to stand in it at a
particular minute.
```

### rzv-floor-not-midpoint

**The floor cell is not a midpoint** — in `RendezvousScheduler::solve`, attached to `admissible.push_back(floor_cell);` (line 186)

```text
NOT THE MIDPOINT (stale through generation 19; corrected 2026-09-18).
This comment used to justify the unconditional admission by saying the
floor "is the midpoint of two poses that were in radio contact, so it is
at most half a comms range from each robot". That construction was
deleted along with every other midpoint drive; what the node passes today
is the centroid of the TEAM'S OWN VEHICLE CELLS (see `floor_cell` on
solve), which carries no comms-range guarantee at all. The admission is
still unconditional, but now for the deadlock reason alone — which is the
reason that was always doing the work.
```

### rzv-name-empty-candidates-cause

**Naming why no cell is admissible** — in `RendezvousScheduler::solve`, attached to `if (cand.empty()) {` (line 201)

```text
NAME THE CAUSE. This message used to assert "the tours are empty and the
last-contact midpoint is outside the ROI" unconditionally, which is two
claims it is not entitled to make. An empty `cand` and a `cand` filtered
down to nothing are different failures with opposite remedies, and the
ts4 smoke spent a full N=4 rung on the message insisting on the first
while the allocator had been publishing tours since t+1.0 s.

The distinction matters most at N>=3, because reachability below is an
AND over the whole team: every extra robot can only shrink the admissible
set. At N=3 the proposer already rejected 12 of 16 candidates. A fleet
that scales past the point where some robot can route to no shared tour
cell at all does not degrade — it loses the arm outright, silently.
```

### rzv-reachability-floor

**The reachability floor on the interval** — in `RendezvousScheduler::solve`, attached to `long long direct_mm = 0;` (line 256)

```text
THE REACHABILITY FLOOR. The interval is also the gap between occurrences,
so it has to be at least as long as it takes the furthest robot to get
there — otherwise a robot that keeps occurrence k is still driving when
k+1 happens, and the schedule is one it can never be on time for.

DIRECT distance, not along-tour. The tour prefix was the right quantity for
the objective (it measures how nearly free the meeting is) and is the wrong
one here: a robot keeping an appointment ABANDONS its tour and drives
straight to the cell. The two differ by however much of the tour sat
between the robot and the near-miss, which is exactly the slack the
objective was maximising.

It carries `depart_safety_milli` because a floor built on the bare travel
time would be one nothing downstream considers achievable — the estimate is
a graph distance at a nominal speed and the real drive is neither. A note
here called this the multiplier's only reader because the departure test
was gone (generation 19); the test came back on 2026-09-19 and reads the
same constant, deliberately — see appointmentLeadMs, which prices a robot's
notice at exactly the rate this floor sized the spacing for.
```

### rzv-findability-cap

**The findability cap and the floor** — in `RendezvousScheduler::solve`, attached to `if (cfg.max_interval_ms > 0 && interval > cfg.max_interval_ms) {` (line 289)

```text
The findability cap: hold the interval at or under the barrier wait, so the
last robot to set off still finds its peers standing there. See
Config::max_interval_ms for the inequality this exists to keep, and for the
generation-19 change of subject that left the arithmetic untouched.

IT CANNOT CUT BELOW THE REACHABILITY FLOOR, and when the two conflict the
floor wins. An interval shorter than the drive does not shorten the drive;
it just stops describing it, and the robot is late anyway. Either way the
result is `interval_ms > max_interval_ms`, which is the broken-inequality
shape the node warns on — and the WARN is keyed on that comparison rather
than on the flags, because `capped` does NOT have to be true here: it asks
whether the cap cut the TOUR term, so a cap that sits above the tours and
below the floor overrules nothing and reports nothing. See the derivation
below. The clamp is symmetric, like the cap: every robot computes the same
floor from the same committed cell and the same shared config, so agreement
survives it.
```

### rzv-bound-flags

**What capped and floored mean** — in `RendezvousScheduler::solve`, attached to `const long long ask =` (line 309)

```text
THE FLAGS DESCRIBE WHICH BOUND THE ANSWER SITS ON, NOT WHICH BRANCH RAN
(2026-09-18). They used to be set inside the two branches above, and that
made them exactly inverted in the one configuration where they matter most.
When the floor exceeds BOTH the tour interval and the cap — production
constants and a 20x20 ROI reach this, e.g. floor 456 s against a 240 s cap
— the first branch does not run (the floor only beat the tour interval, so
`floored` stayed false only if the tour interval was already above it;
where it wasn't, it ran) and the cap branch then assigns
max(cap, floor) == floor and stamps `capped`. The solve returns
interval_ms == floor_ms while reporting floored=0 capped=1: a tally of "how
often did the reachability floor bind" reads ZERO precisely for the cases
where it bound, and the cap is credited with a value it was overruled on.

DERIVED FROM THE THREE BOUNDS, NOT FROM THE RETURNED VALUE (2026-09-18,
second revision). The first revision derived them from `interval` alone and
that lost the one configuration both flags exist for. The returned value is

    interval = max(floor, min(tour, cap))

and asking "which bound is the answer equal to" cannot separate "the cap
never had anything to cut" from "the cap cut and the floor overruled it",
because the answer sits on the floor either way. In the cap-below-floor
configuration — the exact case RendezvousCap.CannotCutBelowTheReachability-
Floor covers, cap 5 s against a 20 s floor and an 84.7 s tour term — that
derivation returned floored=0 AND capped=0: the findability inequality was
broken and the plan said nothing bound at all.

Each flag is asked of its own bound instead, against what that bound was
competing with:
  capped  — the cap CUT the objective's answer. True whenever the cap is
            below the tour term, whether or not the floor then overruled
            it: the cap did pull the period in from the tours, it just did
            not get everything it asked for. This is what
            RendezvousPlan::capped has always documented.
  floored — the floor RAISED the result above what cap-and-objective
            between them asked for. min(tour, cap) is that ask.

Both false means nothing bound. BOTH TRUE IS THE BROKEN FINDABILITY
INEQUALITY and is not a contradiction: it says the cap cut and the floor
refused the cut, which is precisely `interval_ms > max_interval_ms`, the
condition the node warns on. The previous comment here claimed the two
"cannot both be true"; that claim is what made the case invisible.
```

### rzv-handshake-upgrade

**The one provisional upgrade** — in `RendezvousHandshake::adopt`, attached to `if (held_provisional && !peer_provisional) return Adopt::kUpgrade;` (line 375)

```text
THE ONE UPGRADE. Tested BEFORE the equality below, deliberately: the
proposer can re-derive and land on the same three integers with a real
choice behind them this time, and that is still an upgrade — what changes
is not where the team meets but whether the run may be reported as having
exercised the scheduler at all. Taking it clears this robot's flag, which
is what makes the transition terminal on both sides.
```

### rzv-handshake-reagree

**The replacement this robot is owed** — in `RendezvousHandshake::adopt`, attached to `if (held_reagree_due && !peer_provisional) return Adopt::kReagree;` (line 387)

```text
THE REPLACEMENT THIS ROBOT IS OWED. Tested AFTER the equality above, also
deliberately: the proposer republishes on every heartbeat, and while it is
still republishing the OLD triple the numbers match and nothing has been
answered yet. Only a triple that actually differs can settle the request,
so the flag must survive the re-publications and be spent on the change.

`!peer_provisional` mirrors the proposer's own guard on the same decision:
a replacement that is the centroid placeholder would trade a tour-informed
meeting for a position, and the request is better left outstanding than
answered with that. A proposer walking backwards to provisional is a fault,
and falls through to the refusal below where faults belong.
```

### rzv-handshake-conflict

**An unrequested differing triple** — in `RendezvousHandshake::adopt`, attached to `return Adopt::kConflict;` (line 400)

```text
Neither the first triple, nor the one upgrade, nor a replacement anyone
asked for. A restarted proposer, a fleet that disagrees about which robot
is robot 0, or two campaigns sharing a bus. There is no correct silent
resolution: taking it splits the fleet across two generations, and so does
refusing it, so the caller keeps what it has AND says so loudly. Keeping is
the lesser evil only because the triple already held is the one its peers
have echoed.
```
